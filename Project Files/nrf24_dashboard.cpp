/*
 * nrf24_dashboard.cpp  –  SCARAB nRF24 Live Monitor  (SDL2 / v10)
 *
 * SDL2 renders into one single Wayland surface buffer.  No subsurfaces,
 * no Cairo, no GTK drawing areas → Wayland Error 71 cannot happen.
 *
 * ── Prerequisites ─────────────────────────────────────────────────────
 *
 *  On the BOARD (via SSH):
 *    ssh root@192.168.1.232
 *    apt install -y libsdl2-ttf-2.0-0 libsdl2-ttf-dev   # if not present
 *    exit
 *
 *  Copy SDL2_ttf into the ST SDK sysroot (run on Ubuntu):
 *    source ~/st-sdk/environment-setup-cortexa7t2hf-neon-vfpv4-ostl-linux-gnueabi
 *    scp root@192.168.1.232:/usr/include/SDL2/SDL_ttf.h   $SDKTARGETSYSROOT/usr/include/SDL2/
 *    scp root@192.168.1.232:/usr/lib/libSDL2_ttf.so*       $SDKTARGETSYSROOT/usr/lib/
 *    (skip the above two lines if SDL_ttf.h already exists in the sysroot)
 *
 * ── Build ──────────────────────────────────────────────────────────────
 *
 *    source ~/st-sdk/environment-setup-cortexa7t2hf-neon-vfpv4-ostl-linux-gnueabi
 *    arm-ostl-linux-gnueabi-g++ -mfloat-abi=hard -mfpu=neon-vfpv4      \
 *      --sysroot=$SDKTARGETSYSROOT                                       \
 *      -I$HOME/projects/RF24                                             \
 *      -I$HOME/projects/RF24/utility/SPIDEV                             \
 *      -I$SDKTARGETSYSROOT/usr/include/SDL2                             \
 *      -DRF24_SPIDEV                                                     \
 *      -DRF24_LINUX_GPIO_CHIP='"/dev/gpiochip5"'                        \
 *      nrf24_dashboard.cpp                                               \
 *      $HOME/projects/RF24/RF24.cpp                                      \
 *      $HOME/projects/RF24/utility/SPIDEV/spi.cpp                       \
 *      $HOME/projects/RF24/utility/SPIDEV/gpio.cpp                      \
 *      $HOME/projects/RF24/utility/SPIDEV/compatibility.cpp             \
 *      $HOME/projects/RF24/utility/SPIDEV/interrupt.cpp                 \
 *      -L$SDKTARGETSYSROOT/usr/lib                                       \
 *      -lSDL2 -lSDL2_ttf                                                 \
 *      -lcurl -lcrypto                                                   \
 *      -lpthread -lm                                                     \
 *      -Wl,-rpath-link,$SDKTARGETSYSROOT/usr/lib                        \
 *      -o nrf24_dashboard
 *
 * ── Update launch.sh on the board ─────────────────────────────────────
 *    Add before the exec line:
 *      export SDL_VIDEODRIVER=wayland
 *      export SDL_DYNAMIC_API=""
 *
 * ── Network for the cloud-upload feature ──────────────────────────────
 *    The "CLOUD" footer button POSTs telemetry to Azure IoT Hub over
 *    HTTPS, so the board needs a working internet route at the moment it
 *    sends (either wlan0, or tun0 via sim7600_data.py --use-mobile as
 *    described in README.md). No special code is needed on top of that –
 *    libcurl just uses whatever default route is active. If both are
 *    down, the status dot on the CLOUD button turns red and the error is
 *    logged to stderr ("[cloud] upload failed: ...").
 */

#include <RF24.h>
#include <SDL.h>
#include <SDL_ttf.h>

#include <cstdio>
#include <cstring>
#include <cmath>
#include <unistd.h>
#include <pthread.h>
#include <stdexcept>

/* ── Azure IoT cloud upload ─────────────────────────────────────────────
 *  Needs libcurl (HTTPS POST) + OpenSSL libcrypto (HMAC-SHA256 for the
 *  SAS token). Both are normally already present in the ST OpenSTLinux
 *  sysroot; if not:
 *    On the BOARD:
 *      apt install -y libcurl4-openssl-dev libssl-dev
 *    Copy headers/libs into the SDK sysroot the same way as SDL2_ttf
 *    (scp curl/*.h, openssl/*.h, libcurl.so*, libssl.so*, libcrypto.so*).
 *  Add to the link line: -lcurl -lcrypto
 * ─────────────────────────────────────────────────────────────────────── */
#include <curl/curl.h>
#include <openssl/hmac.h>

#include <string>
#include <vector>
#include <cstdint>
#include <cctype>
#include <ctime>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Radio ───────────────────────────────────────────────────────────── */
RF24 radio(3, 0);

#pragma pack(1)
struct RFPacket {
    uint16_t tag;
    uint8_t  DevNamFlag;
    uint8_t  currentState;
    int16_t  devID;
    uint8_t  deviceRate;
    uint8_t  currentUsage;
    uint32_t volumePulses;
    uint32_t VinFBConv;
    uint32_t runTicks;
    uint16_t totVolumeDefault;
    uint8_t  deviceSensors;
    uint8_t  playBtnFlag;
    uint8_t  restart;
    uint8_t  resetCnt;
    uint8_t  currentRate;
    uint8_t  removePrimeCmd;
    uint8_t  deviceError;
    uint16_t chksum;
};
#pragma pack()

const char* rate_tbl[]  = {"2.5","5","10","25","50","120","160","240"};
const char* state_tbl[] = {
    "START","PRIME STOP","PRIME RUN","PAUSE","RUN",
    "COMPLETE","END","OCCUL. DnL","OCCUL. DnH","OCCUL. UP"
};
#define RATE_COUNT   8
#define STATE_COUNT  10
#define VOL_CONST    133.0f
#define normalUpstream 1

/* ── Device name table (index = DevNamFlag from packet) ──────────────────
 *  Matches the Arduino: PumpNam[DevNamFlag]                               */
static const char* dev_name_tbl[] = {
    "Pump #1", "Pump #2", "Pump #3", "Pump #4", "Pump #5",
    "Pump #6", "Pump #7", "Pump #8", "Pump #9", "Pump #10"
};
#define DEV_NAME_COUNT 10
static inline const char* dev_name(uint8_t flag) {
    return (flag < DEV_NAME_COUNT) ? dev_name_tbl[flag] : "Pump ?";
}

const uint8_t addr1[] = "00001";

static RFPacket        latest_pkt = {};
static bool            new_data   = false;
static pthread_mutex_t mtx        = PTHREAD_MUTEX_INITIALIZER;

/* ── Device discovery ─────────────────────────────────────────────────── */
#define MAX_DEVICES 16
struct DevEntry { int16_t devID; uint8_t namFlag; };
static DevEntry dev_list[MAX_DEVICES] = {};
static int      dev_count = 0;
static int16_t  sel_devID = -1;   /* -1 = accept any (first seen auto-selects) */

static void sec_to_time(uint32_t s, char *buf, int len) {
    snprintf(buf, len, "%02d:%02d:%02d",
             (int)(s/3600), (int)((s%3600)/60), (int)(s%60));
}

void* rf_thread(void*) {
    while (true) {
        if (radio.available()) {
            RFPacket pkt;
            radio.read(&pkt, sizeof(pkt));
            if (pkt.tag != 0) {
                pthread_mutex_lock(&mtx);

                /* ── Keep device discovery list up-to-date ── */
                bool found = false;
                for (int i = 0; i < dev_count; i++) {
                    if (dev_list[i].devID == pkt.devID) {
                        dev_list[i].namFlag = pkt.DevNamFlag; /* refresh name */
                        found = true; break;
                    }
                }
                if (!found && dev_count < MAX_DEVICES)
                    dev_list[dev_count++] = {pkt.devID, pkt.DevNamFlag};

                /* ── Forward packet only for the selected device ──
                 *  sel_devID == -1 means "take first device seen".   */
                if (sel_devID < 0) sel_devID = pkt.devID;
                if (pkt.devID == sel_devID) {
                    latest_pkt = pkt;
                    new_data   = true;
                }

                pthread_mutex_unlock(&mtx);
            }
        }
        usleep(10000);
    }
    return NULL;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Azure IoT Hub – device-to-cloud telemetry upload
 * ═══════════════════════════════════════════════════════════════════════ */

// Azure IoT
#define IOT_CONFIG_IOTHUB_FQDN   "asappIOT.azure-devices.net"
#define IOT_CONFIG_DEVICE_ID     "ESP8266_asap"
#define IOT_CONFIG_DEVICE_KEY    "xfys1gJbZeO9tAxTzROs/JmVtsGka3Jx/AIoTOKO6Ng="

#define CLOUD_SAS_TTL_SEC        3600     /* SAS token lifetime           */
#define CLOUD_AUTO_INTERVAL_MS   30000    /* auto-upload period (30 s)    */
#define CLOUD_HTTP_TIMEOUT_SEC   15

/* ── Base64 encode / decode (small self-contained implementation) ──────── */
static std::string base64_encode(const unsigned char *data, size_t len) {
    static const char *tbl =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve(((len + 2) / 3) * 4);

    size_t i = 0;
    for (; i + 3 <= len; i += 3) {
        uint32_t n = ((uint32_t)data[i] << 16) | ((uint32_t)data[i+1] << 8) | data[i+2];
        out += tbl[(n >> 18) & 0x3F];
        out += tbl[(n >> 12) & 0x3F];
        out += tbl[(n >> 6)  & 0x3F];
        out += tbl[n & 0x3F];
    }
    size_t rem = len - i;
    if (rem == 1) {
        uint32_t n = (uint32_t)data[i] << 16;
        out += tbl[(n >> 18) & 0x3F];
        out += tbl[(n >> 12) & 0x3F];
        out += "==";
    } else if (rem == 2) {
        uint32_t n = ((uint32_t)data[i] << 16) | ((uint32_t)data[i+1] << 8);
        out += tbl[(n >> 18) & 0x3F];
        out += tbl[(n >> 12) & 0x3F];
        out += tbl[(n >> 6)  & 0x3F];
        out += '=';
    }
    return out;
}

static std::vector<unsigned char> base64_decode(const std::string &in) {
    static int8_t T[256];
    static bool   init = false;
    if (!init) {
        for (int i = 0; i < 256; i++) T[i] = -1;
        const char *tbl =
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        for (int i = 0; i < 64; i++) T[(unsigned char)tbl[i]] = (int8_t)i;
        init = true;
    }

    std::vector<unsigned char> out;
    int val = 0, bits = -8;
    for (unsigned char c : in) {
        if (c == '=') break;
        if (T[c] == -1) continue;
        val  = (val << 6) + T[c];
        bits += 6;
        if (bits >= 0) {
            out.push_back((unsigned char)((val >> bits) & 0xFF));
            bits -= 8;
        }
    }
    return out;
}

/* ── RFC 3986 percent-encoding (unreserved: A-Z a-z 0-9 - _ . ~) ────────── */
static std::string url_encode(const std::string &s) {
    static const char *hex = "0123456789ABCDEF";
    std::string out;
    out.reserve(s.size() * 3);
    for (unsigned char c : s) {
        if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
            out += (char)c;
        } else {
            out += '%';
            out += hex[(c >> 4) & 0xF];
            out += hex[c & 0xF];
        }
    }
    return out;
}

/* ── HMAC-SHA256 (OpenSSL libcrypto) ────────────────────────────────────── */
static std::vector<unsigned char> hmac_sha256(const std::vector<unsigned char> &key,
                                               const std::string &msg) {
    unsigned char result[EVP_MAX_MD_SIZE];
    unsigned int  len = 0;
    HMAC(EVP_sha256(), key.data(), (int)key.size(),
         (const unsigned char *)msg.data(), msg.size(), result, &len);
    return std::vector<unsigned char>(result, result + len);
}

/* ── Build a SharedAccessSignature SAS token for the device identity ───── */
static std::string generate_sas_token(const char *fqdn, const char *deviceId,
                                       const char *deviceKeyB64, unsigned int ttlSeconds) {
    std::string resourceUri = std::string(fqdn) + "/devices/" + deviceId;
    std::string encodedUri  = url_encode(resourceUri);

    unsigned long expiry = (unsigned long)time(nullptr) + ttlSeconds;
    std::string   toSign = encodedUri + "\n" + std::to_string(expiry);

    std::vector<unsigned char> key = base64_decode(deviceKeyB64);
    std::vector<unsigned char> sig = hmac_sha256(key, toSign);
    std::string sigB64 = base64_encode(sig.data(), sig.size());

    return "SharedAccessSignature sr=" + encodedUri +
           "&sig=" + url_encode(sigB64) +
           "&se=" + std::to_string(expiry);
}

/* ── libcurl: discard response body, we only care about the HTTP code ──── */
static size_t curl_discard_cb(void *, size_t size, size_t nmemb, void *) {
    return size * nmemb;
}

/* ── POST one JSON telemetry message to the IoT Hub D2C endpoint ───────── */
static bool azure_send_telemetry(const std::string &jsonPayload,
                                  long *httpCode, std::string *errMsg) {
    std::string sas = generate_sas_token(IOT_CONFIG_IOTHUB_FQDN, IOT_CONFIG_DEVICE_ID,
                                          IOT_CONFIG_DEVICE_KEY, CLOUD_SAS_TTL_SEC);

    std::string url = std::string("https://") + IOT_CONFIG_IOTHUB_FQDN +
                       "/devices/" + IOT_CONFIG_DEVICE_ID +
                       "/messages/events?api-version=2018-06-30";

    CURL *curl = curl_easy_init();
    if (!curl) { if (errMsg) *errMsg = "curl_easy_init failed"; return false; }

    std::string authHeader = "Authorization: " + sas;
    struct curl_slist *headers = nullptr;
    headers = curl_slist_append(headers, authHeader.c_str());
    headers = curl_slist_append(headers, "Content-Type: application/json");

    curl_easy_setopt(curl, CURLOPT_URL,            url.c_str());
    curl_easy_setopt(curl, CURLOPT_POST,           1L);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS,     jsonPayload.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE,  (long)jsonPayload.size());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER,     headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,  curl_discard_cb);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT,        (long)CLOUD_HTTP_TIMEOUT_SEC);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);

    CURLcode res = curl_easy_perform(curl);

    bool ok = false;
    if (res == CURLE_OK) {
        long code = 0;
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &code);
        if (httpCode) *httpCode = code;
        ok = (code >= 200 && code < 300);
        if (!ok && errMsg) *errMsg = "HTTP " + std::to_string(code);
    } else {
        if (httpCode) *httpCode = 0;
        if (errMsg)   *errMsg   = curl_easy_strerror(res);
    }

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    return ok;
}

/* ── Latest-data snapshot shared with the UI thread ─────────────────────── */
enum CloudStatus { CLOUD_IDLE, CLOUD_SENDING, CLOUD_OK, CLOUD_ERROR };

struct CloudSnapshot {
    bool    valid;
    int16_t devID;
    char    state[32];
    char    vtbi[32];
    char    vi[32];
    char    rate[32];
    char    time[32];
    int     usage;
    uint8_t sensors;
};

static pthread_mutex_t cloud_mtx          = PTHREAD_MUTEX_INITIALIZER;
static CloudSnapshot   cloud_snapshot     = {};
static volatile bool   cloud_auto_upload  = false;  /* user toggle (footer button) */
static volatile bool   cloud_upload_now   = false;  /* one-shot request flag       */
static volatile int    cloud_status       = CLOUD_IDLE;
static volatile Uint32 cloud_last_send_ms = 0;

/* ── Background thread: sends telemetry on demand / on a timer ─────────── */
void* cloud_thread(void*) {
    for (;;) {
        bool          do_send = false;
        CloudSnapshot snap;

        pthread_mutex_lock(&cloud_mtx);
        Uint32 now = SDL_GetTicks();
        if (cloud_upload_now) {
            do_send          = true;
            cloud_upload_now = false;
        } else if (cloud_auto_upload &&
                   (now - cloud_last_send_ms >= CLOUD_AUTO_INTERVAL_MS)) {
            do_send = true;
        }
        snap = cloud_snapshot;
        pthread_mutex_unlock(&cloud_mtx);

        if (do_send && snap.valid) {
            cloud_status = CLOUD_SENDING;

            char json[512];
            snprintf(json, sizeof(json),
                "{\"devID\":%d,\"state\":\"%s\",\"vtbi\":\"%s\",\"vi\":\"%s\","
                "\"rate\":\"%s\",\"runtime\":\"%s\",\"usage\":%d,\"sensors\":%u}",
                snap.devID, snap.state, snap.vtbi, snap.vi,
                snap.rate, snap.time, snap.usage, (unsigned)snap.sensors);

            long httpCode = 0; std::string err;
            bool ok = azure_send_telemetry(json, &httpCode, &err);

            cloud_status       = ok ? CLOUD_OK : CLOUD_ERROR;
            cloud_last_send_ms = SDL_GetTicks();

            if (!ok) fprintf(stderr, "[cloud] upload failed: %s\n", err.c_str());
            else     fprintf(stderr, "[cloud] upload ok (HTTP %ld)\n", httpCode);
        }

        usleep(200000); /* poll 5x/sec */
    }
    return NULL;
}

/* ── Colour palette ───────────────────────────────────────────────────── */
struct Col { uint8_t r, g, b, a; };

static const Col C_BG     = {0x1e, 0x1e, 0x2e, 0xff};
static const Col C_CARD   = {0x31, 0x32, 0x44, 0xff};
static const Col C_TEXT   = {0xcd, 0xd6, 0xf4, 0xff};
static const Col C_SUBTLE = {0xa6, 0xad, 0xc8, 0xff};
static const Col C_DIM    = {0x58, 0x5b, 0x70, 0xff};
static const Col C_BLUE   = {0x89, 0xb4, 0xfa, 0xff};
static const Col C_GREEN  = {0xa6, 0xe3, 0xa1, 0xff};
static const Col C_YELLOW = {0xf9, 0xe2, 0xaf, 0xff};
static const Col C_RED    = {0xe0, 0x40, 0x40, 0xff};   /* vivid red (was pinkish Catppuccin) */
static const Col C_OFF    = {0x4c, 0x4e, 0x66, 0xff};   /* dim navy – visible on card */

static void set_col(SDL_Renderer *r, Col c) {
    SDL_SetRenderDrawColor(r, c.r, c.g, c.b, c.a);
}

/* ── Draw filled circle via scanlines ────────────────────────────────── */
static void fill_circle(SDL_Renderer *r, int cx, int cy, int radius) {
    for (int dy = -radius; dy <= radius; dy++) {
        int dx = (int)sqrtf((float)(radius*radius - dy*dy));
        SDL_RenderDrawLine(r, cx - dx, cy + dy, cx + dx, cy + dy);
    }
}

/* ── Font loading ─────────────────────────────────────────────────────
 *  1. Try known paths first (fast).
 *  2. Recursively scan /usr/share/fonts for any .ttf (fallback).        */
#include <dirent.h>

/* Recursively find the first .ttf/.otf under `dir`, write path to `out`.
 * Returns true on success.                                               */
#include <sys/stat.h>
static bool find_any_ttf(const char *dir, char *out, int outsz) {
    DIR *d = opendir(dir);
    if (!d) return false;
    struct dirent *e;
    bool found = false;
    while (!found && (e = readdir(d)) != NULL) {
        if (e->d_name[0] == '.') continue;
        char p[512];
        snprintf(p, sizeof(p), "%s/%s", dir, e->d_name);

        /* Determine if entry is a directory (DT_UNKNOWN fallback via stat) */
        bool is_dir = (e->d_type == DT_DIR);
        if (e->d_type == DT_UNKNOWN) {
            struct stat st;
            if (stat(p, &st) == 0) is_dir = S_ISDIR(st.st_mode);
        }

        if (is_dir) {
            found = find_any_ttf(p, out, outsz);
        } else {
            /* Match *.ttf or *.otf (case-insensitive last 4 chars) */
            int n = (int)strlen(e->d_name);
            if (n > 4 && e->d_name[n-4] == '.') {
                char e1 = tolower((unsigned char)e->d_name[n-3]);
                char e2 = tolower((unsigned char)e->d_name[n-2]);
                char e3 = tolower((unsigned char)e->d_name[n-1]);
                if ((e1=='t' && e2=='t' && e3=='f') ||
                    (e1=='o' && e2=='t' && e3=='f')) {
                    snprintf(out, outsz, "%s", p);
                    found = true;
                }
            }
        }
    }
    closedir(d);
    return found;
}

static char g_font_path[512] = "";   /* cached on first successful find  */

static TTF_Font* open_font(int size) {
    /* ── known paths (tried in order) ── */
    const char *paths[] = {
        /* ── confirmed present on this board ── */
        "/usr/share/fonts/ttf/LiberationSans-Regular.ttf",
        "/usr/share/fonts/ttf/LiberationMono-Regular.ttf",
        "/usr/share/fonts/ttf/LiberationSerif-Regular.ttf",
        /* ── other common locations ── */
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/TTF/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/ttf-dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/TTF/Vera.ttf",
        "/usr/share/fonts/truetype/ttf-bitstream-vera/Vera.ttf",
        "/usr/share/fonts/ttf-bitstream-vera/Vera.ttf",
        "/usr/share/fonts/noto/NotoSans-Regular.ttf",
        "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf",
        "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
        "/usr/share/fonts/freefont/FreeSans.ttf",
        "/usr/share/fonts/truetype/droid/DroidSans.ttf",
        "/usr/share/fonts/droid/DroidSans.ttf",
        NULL
    };
    for (int i = 0; paths[i]; i++) {
        TTF_Font *f = TTF_OpenFont(paths[i], size);
        if (f) {
            if (!g_font_path[0]) snprintf(g_font_path, sizeof(g_font_path), "%s", paths[i]);
            fprintf(stderr, "Font: %s @ %d\n", paths[i], size);
            return f;
        }
    }
    /* ── recursive scan fallback ── */
    if (!g_font_path[0])
        find_any_ttf("/usr/share/fonts", g_font_path, sizeof(g_font_path));
    if (g_font_path[0]) {
        TTF_Font *f = TTF_OpenFont(g_font_path, size);
        if (f) { fprintf(stderr, "Font (scan): %s @ %d\n", g_font_path, size); return f; }
    }
    fprintf(stderr, "WARNING: no font found for size %d\n", size);
    return NULL;
}

/* ── Render text (y = vertical centre of text) ───────────────────────── */
static void draw_text(SDL_Renderer *r, TTF_Font *f, const char *txt,
                      int x, int y, Col c, bool center_x = false)
{
    if (!f || !txt || !txt[0]) return;
    SDL_Color sc = {c.r, c.g, c.b, c.a};
    SDL_Surface *s = TTF_RenderUTF8_Blended(f, txt, sc);
    if (!s) return;
    SDL_Texture *t = SDL_CreateTextureFromSurface(r, s);
    if (t) {
        int tx = center_x ? x - s->w / 2 : x;
        SDL_Rect dst = {tx, y - s->h / 2, s->w, s->h};
        SDL_RenderCopy(r, t, NULL, &dst);
        SDL_DestroyTexture(t);
    }
    SDL_FreeSurface(s);
}

/* ── Draw a thick arc sector (radial lines from r_in to r_out) ────────── */
static void draw_arc_sector(SDL_Renderer *r, int cx, int cy,
                             int r_out, int r_in, float a0, float a1, Col c)
{
    set_col(r, c);
    float step = 1.2f / r_out;          /* ~1 px arc step at outer rim */
    for (float a = a0; a <= a1 + step * 0.5f; a += step) {
        float ca = cosf(a), sa = sinf(a);
        SDL_RenderDrawLine(r,
            cx + (int)(r_in  * ca), cy + (int)(r_in  * sa),
            cx + (int)(r_out * ca), cy + (int)(r_out * sa));
    }
}

/* ── Dot descriptor ──────────────────────────────────────────────────── */
struct Dot { int cx, cy, r; Col on; bool lit; };

static void draw_dot(SDL_Renderer *ren, const Dot &d) {
    set_col(ren, d.lit ? d.on : C_OFF);
    fill_circle(ren, d.cx, d.cy, d.r);
}

/* ═══════════════════════════════════════════════════════════════════════ */
int main(int argc, char *argv[]) {

    setbuf(stderr, NULL);
    fprintf(stderr, "[dashboard] starting up\n");

    /* Wayland environment – must match what Weston was started with */
    setenv("XDG_RUNTIME_DIR", "/home/weston", 0);
    setenv("SDL_VIDEODRIVER", "wayland",       0);

    /* ── Radio (non-fatal: UI still runs if GPIO is busy) ── */
    bool radio_ok = false;
    try {
        radio.begin();
        radio.setChannel(115);
        radio.setPALevel(RF24_PA_LOW);
        radio.enableDynamicPayloads();
        radio.openReadingPipe(1, addr1);
        radio.startListening();
        radio_ok = true;
        fprintf(stderr, "[dashboard] radio OK\n");
    } catch (std::exception &e) {
        fprintf(stderr, "[dashboard] radio FAILED: %s\n", e.what());
    } catch (...) {
        fprintf(stderr, "[dashboard] radio FAILED (unknown exception)\n");
    }

    pthread_t tid;
    if (radio_ok)
        pthread_create(&tid, NULL, rf_thread, NULL);

    /* ── Azure IoT cloud-upload background thread ── */
    curl_global_init(CURL_GLOBAL_DEFAULT);
    pthread_t cloud_tid;
    pthread_create(&cloud_tid, NULL, cloud_thread, NULL);

    /* SDL2 */
    SDL_Init(SDL_INIT_VIDEO);
    TTF_Init();

    SDL_Window *win = SDL_CreateWindow(
        "Infusion Monitor",
        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
        480, 800,
        SDL_WINDOW_FULLSCREEN_DESKTOP | SDL_WINDOW_SHOWN);

    SDL_Renderer *ren = SDL_CreateRenderer(
        win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

    /* Print actual screen size so we can confirm the layout fits */
    int ACT_W = 480, ACT_H = 800;
    SDL_GetRendererOutputSize(ren, &ACT_W, &ACT_H);
    fprintf(stderr, "[dashboard] renderer output: %d x %d\n", ACT_W, ACT_H);

    /* ── Adaptive layout – all geometry derived from actual screen size ── */
    int SW = ACT_W, SH = ACT_H;
    fprintf(stderr, "[dashboard] screen: %d x %d\n", SW, SH);

    /* Padding scales with screen (min 4 px) */
    int P = SH / 100;
    if (P < 4) P = 4;
    int W = SW - 2 * P;

    /* ── Full-width top/bottom bands + vertical split in the middle ──────
     *   LEFT  panel (~55 % of W) : 7 data rows  (label + value, 2 lines)
     *   RIGHT panel (~45 % of W) : animated arc-ring logo (larger ring)
     * ─────────────────────────────────────────────────────────────────── */
    int HDR_H = SH * 7 / 100;
    int LED_H = SH * 9 / 100;
    int BTN_H = SH * 8 / 100;

    int HDR_Y = P;
    int LED_Y = HDR_Y + HDR_H + P;
    int SPL_Y = LED_Y + LED_H + P;     /* top of the split area  */
    int BTN_Y = SH - P - BTN_H;
    int SPL_H = BTN_Y - P - SPL_Y;     /* height of split area   */

    /* ── Footer: Close (left) + Cloud-upload (right) buttons ── */
    int CLOSE_BTN_W = W * 45 / 100;
    int CLOUD_BTN_X = P + CLOSE_BTN_W + P;
    int CLOUD_BTN_W = W - CLOSE_BTN_W - P;

    /* Vertical panel widths */
    int DAT_W = W * 55 / 100;          /* left  – data           */
    int LOG_W = W - DAT_W - P;         /* right – logo           */
    int DAT_X = P;
    int LOG_X = P + DAT_W + P;

    /* Font sizes (user-tuned) */
    int fsLG = SH * 40 / 800;  if (fsLG < 11) fsLG = 11;
    int fsMD = SH * 36 / 800;  if (fsMD < 10) fsMD = 10;
    int fsRN = SH * 34 / 800;  if (fsRN < 9)  fsRN = 9;
    int fsSM = SH * 28 / 800;  if (fsSM < 8)  fsSM = 8;
    int fsXS = SH * 24 / 800;  if (fsXS < 7)  fsXS = 7;

    TTF_Font *f16 = open_font(fsLG);
    TTF_Font *f14 = open_font(fsMD);
    TTF_Font *f13 = open_font(fsRN);
    TTF_Font *f11 = open_font(fsSM);
    TTF_Font *f9  = open_font(fsXS);

    /* Fallback chain so we always have at least one font */
    if (!f13) f13 = f14 ? f14 : f11;
    if (!f16) f16 = f13;
    if (!f14) f14 = f13;
    if (!f11) f11 = f13;
    if (!f9)  f9  = f13;

    /* ── Small LED row (4 circles) ── */
    int LED_R  = LED_H * 13 / 68;      /* proportional radius */
    int LED_CY = LED_Y + LED_H / 2;
    int LCOL   = W / 4;

    Dot led[4] = {
        {P + LCOL/2,          LED_CY, LED_R, C_BLUE,   false},
        {P + LCOL + LCOL/2,   LED_CY, LED_R, C_GREEN,  false},
        {P + 2*LCOL + LCOL/2, LED_CY, LED_R, C_YELLOW, false},
        {P + 3*LCOL + LCOL/2, LED_CY, LED_R, C_RED,    false},
    };
    const char *led_lbl[] = {"PRIME", "RUN", "ALARM", "OCCL."};

    /* ── Logo arc ring (right panel) ── */
    int LOG_CX = LOG_X + LOG_W / 2;
    int LOG_CY = SPL_Y + SPL_H * 40 / 100;
    /* Outer radius = smaller of (half panel width - margin) or (30% of panel height) */
    int LOG_RO = (LOG_W / 2 - P * 2) < (SPL_H * 30 / 100)
                 ? (LOG_W / 2 - P * 2)
                 : (SPL_H * 30 / 100);
    int LOG_RI = LOG_RO * 50 / 100;          /* inner radius (thick ring) */
    if (LOG_RI < 4) LOG_RI = 4;

    /* lit state: 0=Blue/PRIME, 1=Red/UPL, 2=Green/RUN, 3=Yellow/DNL */
    bool logo_lit[4] = {false, false, false, false};
    const Col  logo_col[4] = {C_BLUE, C_RED, C_GREEN, C_YELLOW};
    const char *logo_lbl[4] = {"PRIME", "UPL", "RUN", "DNL"};

    /* Arc definitions: each quadrant with a 6° gap between sectors.
     * SDL y-down: 0°=right, 90°=down, 180°=left, 270°=up
     *   TL (180-270) = upper-left  → Blue  PRIME
     *   TR (270-360) = upper-right → Red   UPL
     *   BL  (90-180) = lower-left  → Green RUN
     *   BR   (0-90)  = lower-right → Yellow DNL              */
    const float GAP = 6.0f * (float)M_PI / 180.0f;
    struct ArcSeg { float a0, a1; int idx; };
    ArcSeg arcs[4] = {
        {(float)M_PI         + GAP/2, (float)(3*M_PI/2) - GAP/2, 2}, /* Green RUN  – upper-left  */
        {(float)(3*M_PI/2)   + GAP/2, (float)(2*M_PI)   - GAP/2, 1}, /* Red   UPL  – upper-right */
        {(float)(M_PI/2)     + GAP/2, (float)M_PI       - GAP/2, 0}, /* Blue  PRIME– lower-left  */
        {                     GAP/2,  (float)(M_PI/2)   - GAP/2, 3}, /* Yellow DNL – lower-right */
    };

    /* Label positions: just outside ring at mid-angle of each arc */
    float lbl_r = LOG_RO * 1.40f;
    float lbl_mid[4] = {
        135.0f*(float)M_PI/180.0f,   /* PRIME (Blue) lower-left  (swapped) */
        315.0f*(float)M_PI/180.0f,   /* UPL   (Red)  upper-right           */
        225.0f*(float)M_PI/180.0f,   /* RUN  (Green) upper-left  (swapped) */
         45.0f*(float)M_PI/180.0f,   /* DNL  (Yell)  lower-right           */
    };
    int lbl_x[4], lbl_y[4];
    for (int i = 0; i < 4; i++) {
        lbl_x[i] = LOG_CX + (int)(lbl_r * cosf(lbl_mid[i]));
        lbl_y[i] = LOG_CY + (int)(lbl_r * sinf(lbl_mid[i]));
    }

    int brand_y   = LOG_CY + LOG_RO + P * 3;
    int tagline_y = brand_y + fsRN + P;

    /* ── Device-selector button (logo panel, below tagline) ── */
    int SEL_BTN_W = LOG_W - P * 6;
    int SEL_BTN_H = fsSM + P * 3;
    int SEL_BTN_X = LOG_X + (LOG_W - SEL_BTN_W) / 2;   /* centred in panel */
    int SEL_BTN_Y = tagline_y + fsXS / 2 + P * 4;

    /* ── Overlay geometry (reused in events and render) ── */
    int OV_TITLE_H = fsMD + P * 4;
    int OV_ROW_H   = fsSM + P * 5;

    /* ── Data rows ── */
    const char *row_lbl[] = {
        "VTBI", "VI", "D_RATE", "RATE", "TIME", "USAGE", "STATUS"
    };
    char row_val[7][64] = {
        "--", "--", "--", "--", "--", "--", "--"
    };
    Col  status_col = C_TEXT;

    /* ── Header strings ── */
    char devid_str[40] = "Device ID: --";
    char conn_str[24]  = "Waiting...";
    Col  conn_col      = C_RED;

    /* ── Close-button press tracking ── */
    bool btn_pressed       = false;   /* finger/cursor is currently held inside the Close button  */
    bool cloud_btn_pressed = false;   /* finger/cursor is currently held inside the Cloud button  */
    bool touch_active      = false;   /* a real touch is in progress – suppress synth mouse */

    /* ── Blink state ── */
    bool blink_state = false;
    Uint32 last_blink = SDL_GetTicks();

    int  blink_blue   = 0;
    int  blink_green  = 0;
    int  blink_yellow = 0;
    int  blink_red    = 0;
    int  blink_ry     = 0;
    bool solid_blue   = false;   /* state==1 PRIME STOP → solid on, not blinking */

    /* ── Device selector state ── */
    bool selector_open        = false;
    bool selector_just_opened = false; /* suppress the UP that immediately follows open */

    /* ── Data update interval ── */
    Uint32 last_data        = SDL_GetTicks();
    Uint32 last_packet_time = 0;          /* timestamp of last received packet   */
    const Uint32 SIGNAL_TIMEOUT = 3000;   /* ms without a packet → "No Signal"   */

    /* ── Row height inside data panel (7 rows, 2-line each) ── */
    int ROW_H = SPL_H / 7;

    /* ── Main loop ─────────────────────────────────────────────────── */
    bool running = true;

    while (running) {

        /* Events */
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {

            /* ── Always handle hard quit ── */
            if (ev.type == SDL_QUIT) { running = false; continue; }

            /* ── Escape / Q: close selector first, then quit ── */
            if (ev.type == SDL_KEYDOWN) {
                if (ev.key.keysym.sym == SDLK_ESCAPE ||
                    ev.key.keysym.sym == SDLK_q) {
                    if (selector_open) selector_open = false;
                    else running = false;
                }
                continue;
            }

            /* ═══════════════════════════════════════════════════════════
             *  SELECTOR OVERLAY MODE
             *  While open, all touch/mouse is intercepted here.
             *  Action fires on UP so the user can cancel by sliding away.
             * ══════════════════════════════════════════════════════════ */
            if (selector_open) {
                int tx = -1, ty = -1;
                bool is_up = false;

                if (ev.type == SDL_FINGERUP) {
                    touch_active = false;
                    tx = (int)(ev.tfinger.x * SW);
                    ty = (int)(ev.tfinger.y * SH);
                    is_up = true;
                } else if (ev.type == SDL_FINGERDOWN) {
                    touch_active = true;
                } else if (ev.type == SDL_MOUSEBUTTONUP && !touch_active) {
                    tx = ev.button.x; ty = ev.button.y; is_up = true;
                }

                if (is_up) {
                    if (selector_just_opened) {
                        /* This UP came from the tap that opened the overlay –
                         * ignore it so we don't accidentally select a row.    */
                        selector_just_opened = false;
                    } else {
                        /* Determine which row was tapped */
                        int row_top = SPL_Y + OV_TITLE_H;
                        int row_idx = (ty >= row_top) ? (ty - row_top) / OV_ROW_H : -1;

                        pthread_mutex_lock(&mtx);
                        int dc = dev_count;
                        pthread_mutex_unlock(&mtx);

                        if (row_idx >= 0 && row_idx < dc) {
                            pthread_mutex_lock(&mtx);
                            int16_t new_id = dev_list[row_idx].devID;
                            bool changed   = (new_id != sel_devID);
                            sel_devID      = new_id;
                            if (changed) new_data = false;
                            pthread_mutex_unlock(&mtx);

                            if (changed) {
                                /* Clear stale display for the previous device */
                                for (int i = 0; i < 7; i++) strcpy(row_val[i], "--");
                                blink_blue=blink_green=blink_yellow=blink_red=blink_ry=0;
                                solid_blue = false;
                                for (int i=0;i<4;i++){led[i].lit=false;logo_lit[i]=false;}
                                strcpy(devid_str, "Device ID: --");
                                strcpy(conn_str,  "Waiting...");
                                conn_col = C_RED;
                            }
                        }
                        selector_open = false;
                    }
                }
                continue; /* swallow all other events while overlay is open */
            }

            /* ═══════════════════════════════════════════════════════════
             *  NORMAL MODE
             * ══════════════════════════════════════════════════════════ */
            switch (ev.type) {

                /* ── Touch ──────────────────────────────────────────── */
                case SDL_FINGERDOWN: {
                    touch_active = true;
                    int fy = (int)(ev.tfinger.y * SH);
                    int fx = (int)(ev.tfinger.x * SW);
                    /* Device selector button */
                    if (fx >= SEL_BTN_X && fx <= SEL_BTN_X + SEL_BTN_W &&
                        fy >= SEL_BTN_Y && fy <= SEL_BTN_Y + SEL_BTN_H) {
                        selector_open        = true;
                        selector_just_opened = true;
                    } else if (fy >= BTN_Y && fy <= BTN_Y + BTN_H &&
                               fx >= P && fx <= P + CLOSE_BTN_W) {
                        btn_pressed = true;
                    } else if (fy >= BTN_Y && fy <= BTN_Y + BTN_H &&
                               fx >= CLOUD_BTN_X && fx <= CLOUD_BTN_X + CLOUD_BTN_W) {
                        cloud_btn_pressed = true;
                    }
                    break;
                }
                case SDL_FINGERUP: {
                    int fy = (int)(ev.tfinger.y * SH);
                    int fx = (int)(ev.tfinger.x * SW);
                    if (btn_pressed && fy >= BTN_Y && fy <= BTN_Y + BTN_H &&
                        fx >= P && fx <= P + CLOSE_BTN_W)
                        running = false;
                    if (cloud_btn_pressed && fy >= BTN_Y && fy <= BTN_Y + BTN_H &&
                        fx >= CLOUD_BTN_X && fx <= CLOUD_BTN_X + CLOUD_BTN_W) {
                        cloud_auto_upload = !cloud_auto_upload;
                        if (cloud_auto_upload) cloud_upload_now = true; /* immediate send too */
                    }
                    btn_pressed       = false;
                    cloud_btn_pressed = false;
                    touch_active      = false;
                    break;
                }
                case SDL_FINGERMOTION: {
                    int fy = (int)(ev.tfinger.y * SH);
                    int fx = (int)(ev.tfinger.x * SW);
                    if (fy < BTN_Y || fy > BTN_Y + BTN_H ||
                        fx < P || fx > P + CLOSE_BTN_W)
                        btn_pressed = false;
                    if (fy < BTN_Y || fy > BTN_Y + BTN_H ||
                        fx < CLOUD_BTN_X || fx > CLOUD_BTN_X + CLOUD_BTN_W)
                        cloud_btn_pressed = false;
                    break;
                }

                /* ── Mouse (desktop / stylus) ───────────────────────── */
                case SDL_MOUSEBUTTONDOWN: {
                    if (touch_active) break;
                    int mx = ev.button.x, my = ev.button.y;
                    if (mx >= SEL_BTN_X && mx <= SEL_BTN_X + SEL_BTN_W &&
                        my >= SEL_BTN_Y && my <= SEL_BTN_Y + SEL_BTN_H) {
                        selector_open        = true;
                        selector_just_opened = true;
                    } else if (my >= BTN_Y && my <= BTN_Y + BTN_H &&
                               mx >= P && mx <= P + CLOSE_BTN_W) {
                        btn_pressed = true;
                    } else if (my >= BTN_Y && my <= BTN_Y + BTN_H &&
                               mx >= CLOUD_BTN_X && mx <= CLOUD_BTN_X + CLOUD_BTN_W) {
                        cloud_btn_pressed = true;
                    }
                    break;
                }
                case SDL_MOUSEBUTTONUP: {
                    if (touch_active) break;
                    int mx = ev.button.x, my = ev.button.y;
                    if (btn_pressed && my >= BTN_Y && my <= BTN_Y + BTN_H &&
                        mx >= P && mx <= P + CLOSE_BTN_W)
                        running = false;
                    if (cloud_btn_pressed && my >= BTN_Y && my <= BTN_Y + BTN_H &&
                        mx >= CLOUD_BTN_X && mx <= CLOUD_BTN_X + CLOUD_BTN_W) {
                        cloud_auto_upload = !cloud_auto_upload;
                        if (cloud_auto_upload) cloud_upload_now = true; /* immediate send too */
                    }
                    btn_pressed       = false;
                    cloud_btn_pressed = false;
                    break;
                }
                case SDL_MOUSEMOTION: {
                    if (touch_active) break;
                    int mx = ev.motion.x, my = ev.motion.y;
                    if (btn_pressed &&
                        (my < BTN_Y || my > BTN_Y + BTN_H ||
                         mx < P || mx > P + CLOSE_BTN_W))
                        btn_pressed = false;
                    if (cloud_btn_pressed &&
                        (my < BTN_Y || my > BTN_Y + BTN_H ||
                         mx < CLOUD_BTN_X || mx > CLOUD_BTN_X + CLOUD_BTN_W))
                        cloud_btn_pressed = false;
                    break;
                }

                default: break;
            }
        }

        Uint32 now = SDL_GetTicks();

        /* ── 500 ms blink ── */
        if (now - last_blink >= 500) {
            blink_state = !blink_state;
            last_blink  = now;

            if (blink_blue) {
                led[0].lit   = blink_state;
                logo_lit[0]  = blink_state;
            } else if (solid_blue) {
                led[0].lit   = true;         /* state==1: solid on, never toggled */
                logo_lit[0]  = true;
            }
            if (blink_green) {
                led[1].lit   = blink_state;
                logo_lit[2]  = blink_state;
            }
            if (blink_yellow && !blink_ry) {
                led[2].lit   = blink_state;
                logo_lit[3]  = blink_state;
            }
            if (blink_red && !blink_ry) {
                led[3].lit   = blink_state;
                logo_lit[1]  = blink_state;
            }
            if (blink_ry) {
                led[3].lit   = blink_state;  logo_lit[1] = blink_state;
                led[2].lit   = true;          logo_lit[3] = true;
            }
        }

        /* ── 200 ms data update ── */
        if (now - last_data >= 200) {
            last_data = now;

            pthread_mutex_lock(&mtx);
            if (!new_data) { pthread_mutex_unlock(&mtx); goto render; }
            RFPacket p = latest_pkt;
            new_data   = false;
            pthread_mutex_unlock(&mtx);

            {
                float vol  = VOL_CONST * p.volumePulses / 1000000.0f;
                float vtbi = p.totVolumeDefault - vol;

                snprintf(row_val[0], 64, "%.1f mL", vtbi < 0 ? 0.0f : vtbi);
                snprintf(row_val[1], 64, "%.1f mL", vol);

                if (p.deviceRate  < RATE_COUNT)
                    snprintf(row_val[2], 64, "%s mL/h", rate_tbl[p.deviceRate]);
                else snprintf(row_val[2], 64, "-- mL/h");

                if (p.currentRate < RATE_COUNT)
                    snprintf(row_val[3], 64, "%s mL/h", rate_tbl[p.currentRate]);
                else snprintf(row_val[3], 64, "-- mL/h");

                sec_to_time(p.runTicks, row_val[4], 64);
                snprintf(row_val[5], 64, "%d", p.currentUsage + 1);

                /* Sensor / state logic */
                int  indxsens    = 0;
                bool uplFlag     = false;
                int  uplsense    = normalUpstream ? 0 : 4;
                int  normalsense = normalUpstream ? 4 : 0;
                int  idxupl      = 5;

                if (p.deviceSensors == normalsense)
                    { indxsens = 0; uplFlag = false; }
                else if (p.deviceSensors == uplsense  && p.currentState == 4)
                    { indxsens = idxupl;     uplFlag = true; }
                else if (p.deviceSensors == 1         && p.currentState == 4)
                    { indxsens = 3; }
                else if (p.deviceSensors == 3         && p.currentState == 4)
                    { indxsens = 4; }
                else if (p.deviceSensors == uplsense  && p.currentState == 3)
                    { indxsens = idxupl + 1; uplFlag = true; }
                else
                    { indxsens = 0; uplFlag = false; }

                int si = p.currentState + indxsens;
                if (si >= STATE_COUNT) si = STATE_COUNT - 1;
                snprintf(row_val[6], 64, "%s", state_tbl[si]);

                if      (p.currentState == 4 && p.deviceSensors == 0)
                    status_col = C_GREEN;
                else if (uplFlag || p.deviceSensors == 7 || p.deviceSensors == 1)
                    status_col = C_RED;
                else if (p.currentState == 3)
                    status_col = C_YELLOW;
                else
                    status_col = C_TEXT;

                snprintf(devid_str, 40, "Device ID: %d", p.devID);
                snprintf(conn_str,  24, "LIVE");
                conn_col        = C_GREEN;
                last_packet_time = now;   /* reset signal-loss watchdog */

                /* ── Refresh the snapshot shared with the cloud-upload thread ── */
                pthread_mutex_lock(&cloud_mtx);
                cloud_snapshot.valid   = true;
                cloud_snapshot.devID   = p.devID;
                strncpy(cloud_snapshot.state, row_val[6], sizeof(cloud_snapshot.state) - 1);
                cloud_snapshot.state[sizeof(cloud_snapshot.state) - 1] = '\0';
                strncpy(cloud_snapshot.vtbi,  row_val[0], sizeof(cloud_snapshot.vtbi) - 1);
                cloud_snapshot.vtbi[sizeof(cloud_snapshot.vtbi) - 1] = '\0';
                strncpy(cloud_snapshot.vi,    row_val[1], sizeof(cloud_snapshot.vi) - 1);
                cloud_snapshot.vi[sizeof(cloud_snapshot.vi) - 1] = '\0';
                strncpy(cloud_snapshot.rate,  row_val[3], sizeof(cloud_snapshot.rate) - 1);
                cloud_snapshot.rate[sizeof(cloud_snapshot.rate) - 1] = '\0';
                strncpy(cloud_snapshot.time,  row_val[4], sizeof(cloud_snapshot.time) - 1);
                cloud_snapshot.time[sizeof(cloud_snapshot.time) - 1] = '\0';
                cloud_snapshot.usage   = p.currentUsage + 1;
                cloud_snapshot.sensors = p.deviceSensors;
                pthread_mutex_unlock(&cloud_mtx);

                /* ── Blink flags – logic mirrors the Arduino BlinkXxx() functions ──
                 *
                 *  BlinkBlue   : currentState == 2  (PRIME RUN)           → blink
                 *  solid_blue  : currentState == 1  (PRIME STOP)          → solid on
                 *  BlinkGreen  : state==4 && sensors==normalsense && !ry   → blink (RUN, all clear)
                 *  BlinkYell   : (sensors==1 || state==3) && !upl && !ry  → blink (DNL / PAUSE)
                 *  BlinkRed    : uplFlag                                   → blink (UPL)
                 *  BlinkRedYell: sensors==7                                → RED blinks, YELLOW solid
                 */
                blink_ry     = (p.deviceSensors == 7)                                       ? 1 : 0;
                solid_blue   = (p.currentState == 1);
                blink_blue   = (p.currentState == 2)                                        ? 1 : 0;
                blink_green  = (p.currentState == 4 && p.deviceSensors == (uint8_t)normalsense
                                && !blink_ry)                                                ? 1 : 0;
                blink_yellow = ((p.deviceSensors == 1 || p.currentState == 3)
                                && !uplFlag && !blink_ry)                                    ? 1 : 0;
                blink_red    = uplFlag                                                       ? 1 : 0;

                /* Clear indicators that are no longer active */
                if (!blink_blue && !solid_blue) { led[0].lit=false; logo_lit[0]=false; }
                if  (solid_blue)                { led[0].lit=true;  logo_lit[0]=true;  }
                if (!blink_green)               { led[1].lit=false; logo_lit[2]=false; }
                if (!blink_yellow && !blink_ry) { led[2].lit=false; logo_lit[3]=false; }
                if (!blink_red    && !blink_ry) { led[3].lit=false; logo_lit[1]=false; }
            }
        }

        /* ── Signal-loss watchdog ──────────────────────────────────── */
        if (last_packet_time > 0 &&
            now - last_packet_time > SIGNAL_TIMEOUT &&
            strcmp(conn_str, "LIVE") == 0) {
            /* Packet stream has gone silent – show "No Signal" */
            snprintf(conn_str, 24, "No Signal");
            conn_col = C_YELLOW;
        }

        /* ── Render ─────────────────────────────────────────────────── */
        render:

        /* Background */
        set_col(ren, C_BG);
        SDL_RenderClear(ren);

        /* ── Header ── */
        draw_text(ren, f11, devid_str, P + 8, HDR_Y + HDR_H/2, C_SUBTLE);
        /* centre: version tag so we can confirm correct binary is running */
        draw_text(ren, f9, "v10", SW/2, HDR_Y + HDR_H/2, C_DIM, true);
        /* right-align connection status */
        if (f11) {
            int tw = 0, th = 0;
            TTF_SizeUTF8(f11, conn_str, &tw, &th);
            draw_text(ren, f11, conn_str, SW - P - 8 - tw, HDR_Y + HDR_H/2, conn_col);
        }

        /* ── LED row card ── */
        SDL_Rect led_rc = {P, LED_Y, W, LED_H};
        set_col(ren, C_CARD); SDL_RenderFillRect(ren, &led_rc);
        for (int i = 0; i < 4; i++) {
            draw_dot(ren, led[i]);
            draw_text(ren, f9, led_lbl[i],
                      led[i].cx, led[i].cy + LED_R + 10, C_DIM, true);
        }

        /* ── Data panel (left) – 2-line rows: label on top, value below ── */
        SDL_Rect dat_rc = {DAT_X, SPL_Y, DAT_W, SPL_H};
        set_col(ren, C_CARD); SDL_RenderFillRect(ren, &dat_rc);
        for (int i = 0; i < 7; i++) {
            int row_top = SPL_Y + i * ROW_H;
            /* thin divider between rows */
            if (i > 0) {
                set_col(ren, C_DIM);
                SDL_RenderDrawLine(ren, DAT_X + 6, row_top, DAT_X + DAT_W - 6, row_top);
            }
            int lbl_cy = row_top + ROW_H * 30 / 100;
            int val_cy = row_top + ROW_H * 72 / 100;
            draw_text(ren, f11, row_lbl[i], DAT_X + 10, lbl_cy, C_SUBTLE);
            Col vc = (i == 6) ? status_col : C_TEXT;
            draw_text(ren, f13, row_val[i], DAT_X + 10, val_cy, vc);
        }

        /* ── Logo panel (right) ── */
        SDL_Rect log_rc = {LOG_X, SPL_Y, LOG_W, SPL_H};
        set_col(ren, C_CARD); SDL_RenderFillRect(ren, &log_rc);

        /* 4-segment arc ring */
        for (int i = 0; i < 4; i++) {
            int idx = arcs[i].idx;
            Col c   = logo_lit[idx] ? logo_col[idx] : C_OFF;
            draw_arc_sector(ren, LOG_CX, LOG_CY, LOG_RO, LOG_RI,
                            arcs[i].a0, arcs[i].a1, c);
        }

        /* Labels at mid-angle just outside ring */
        for (int i = 0; i < 4; i++) {
            Col lc = logo_lit[i] ? logo_col[i] : C_SUBTLE;
            draw_text(ren, f9, logo_lbl[i], lbl_x[i], lbl_y[i], lc, true);
        }

        /* Brand text – centred in the right panel */
        draw_text(ren, f13, "infusion",    LOG_CX, brand_y,   C_BLUE, true);
        draw_text(ren, f9,  "INNOVATIONS", LOG_CX, tagline_y, C_DIM,  true);

        /* ── Device selector button ── */
        {
            pthread_mutex_lock(&mtx);
            int  dc   = dev_count;
            int16_t sid = sel_devID;
            pthread_mutex_unlock(&mtx);

            /* Button background (highlight if selector is open) */
            Col sel_btn_c = selector_open
                            ? C_BLUE
                            : Col{0x45, 0x47, 0x5a, 0xff};
            SDL_Rect sbr = {SEL_BTN_X, SEL_BTN_Y, SEL_BTN_W, SEL_BTN_H};
            set_col(ren, sel_btn_c); SDL_RenderFillRect(ren, &sbr);

            /* Button label: "DEVICES (n)" */
            char sel_lbl[32];
            snprintf(sel_lbl, sizeof(sel_lbl), "DEVICES (%d)", dc);
            Col sel_txt_c = selector_open ? Col{0x1e,0x1e,0x2e,0xff} : C_SUBTLE;
            draw_text(ren, f9, sel_lbl,
                      SEL_BTN_X + SEL_BTN_W / 2,
                      SEL_BTN_Y + SEL_BTN_H / 2, sel_txt_c, true);
        }

        /* ── Close button (darker when pressed for tactile feedback) ── */
        SDL_Rect btn_rc = {P, BTN_Y, CLOSE_BTN_W, BTN_H};
        Col btn_col = btn_pressed ? Col{0xa0, 0x20, 0x20, 0xff} : C_RED;
        set_col(ren, btn_col);
        SDL_RenderFillRect(ren, &btn_rc);
        draw_text(ren, f14, "X  Close", P + CLOSE_BTN_W / 2, BTN_Y + BTN_H / 2,
                  {0x1e, 0x1e, 0x2e, 0xff}, true);

        /* ── Cloud-upload button (Azure IoT) ──────────────────────────
         *  Off  : dark grey, idle.
         *  On   : blue – auto-upload running every CLOUD_AUTO_INTERVAL_MS.
         *  A small status dot on the right shows the result of the last
         *  send: grey = idle, yellow = sending, green = ok, red = error. */
        {
            bool auto_on = cloud_auto_upload;
            Col cloud_base = auto_on ? C_BLUE : Col{0x45, 0x47, 0x5a, 0xff};
            Col cloud_col  = cloud_btn_pressed
                             ? Col{(uint8_t)(cloud_base.r * 7 / 10),
                                   (uint8_t)(cloud_base.g * 7 / 10),
                                   (uint8_t)(cloud_base.b * 7 / 10), 0xff}
                             : cloud_base;

            SDL_Rect cloud_rc = {CLOUD_BTN_X, BTN_Y, CLOUD_BTN_W, BTN_H};
            set_col(ren, cloud_col);
            SDL_RenderFillRect(ren, &cloud_rc);

            Col cloud_txt_c = auto_on ? Col{0x1e, 0x1e, 0x2e, 0xff} : C_SUBTLE;
            const char *cloud_lbl = auto_on ? "CLOUD: AUTO" : "CLOUD: OFF";
            draw_text(ren, f14, cloud_lbl,
                      CLOUD_BTN_X + CLOUD_BTN_W / 2 - fsXS, BTN_Y + BTN_H / 2,
                      cloud_txt_c, true);

            /* Status dot */
            Col dot_col;
            switch (cloud_status) {
                case CLOUD_SENDING: dot_col = C_YELLOW; break;
                case CLOUD_OK:      dot_col = C_GREEN;  break;
                case CLOUD_ERROR:   dot_col = C_RED;    break;
                default:            dot_col = C_OFF;    break;
            }
            int dot_r = BTN_H / 8;
            int dot_cx = CLOUD_BTN_X + CLOUD_BTN_W - P * 2 - dot_r;
            int dot_cy = BTN_Y + BTN_H / 2;
            set_col(ren, dot_col);
            fill_circle(ren, dot_cx, dot_cy, dot_r);
        }

        /* ── Device selector overlay ──────────────────────────────────── */
        if (selector_open) {
            /* Dark panel covering the split area */
            SDL_Rect ov_rc = {P, SPL_Y, W, SPL_H};
            set_col(ren, Col{0x18, 0x18, 0x28, 0xff});
            SDL_RenderFillRect(ren, &ov_rc);

            /* Title bar */
            SDL_Rect title_rc = {P, SPL_Y, W, OV_TITLE_H};
            set_col(ren, C_BLUE);
            SDL_RenderFillRect(ren, &title_rc);
            draw_text(ren, f14, "SELECT DEVICE",
                      SW / 2, SPL_Y + OV_TITLE_H / 2,
                      Col{0x1e, 0x1e, 0x2e, 0xff}, true);

            /* Device rows */
            pthread_mutex_lock(&mtx);
            int   dc   = dev_count;
            int16_t sid = sel_devID;
            DevEntry snap[MAX_DEVICES];
            for (int i = 0; i < dc; i++) snap[i] = dev_list[i];
            pthread_mutex_unlock(&mtx);

            int max_rows = (SPL_H - OV_TITLE_H) / OV_ROW_H;
            int rows_shown = dc < max_rows ? dc : max_rows;

            for (int i = 0; i < rows_shown; i++) {
                int ry = SPL_Y + OV_TITLE_H + i * OV_ROW_H;

                /* Row background – highlight selected device */
                bool is_sel = (snap[i].devID == sid);
                if (is_sel) {
                    SDL_Rect sel_rc = {P + 2, ry, W - 4, OV_ROW_H - 2};
                    set_col(ren, Col{0x28, 0x38, 0x58, 0xff});
                    SDL_RenderFillRect(ren, &sel_rc);
                }

                /* Separator */
                set_col(ren, C_DIM);
                SDL_RenderDrawLine(ren, P + 10, ry, P + W - 10, ry);

                /* Device name (left) and ID (right) */
                int row_cy = ry + OV_ROW_H / 2;
                char id_str[24];
                snprintf(id_str, sizeof(id_str), "ID: %d", snap[i].devID);

                Col name_col = is_sel ? C_BLUE  : C_TEXT;
                Col id_col   = is_sel ? C_BLUE  : C_SUBTLE;
                draw_text(ren, f11, dev_name(snap[i].namFlag),
                          P + 18, row_cy, name_col);
                draw_text(ren, f9,  id_str,
                          P + W - 18, row_cy, id_col);
                if (is_sel)
                    draw_text(ren, f9, ">", P + 6, row_cy, C_BLUE);
            }

            if (dc == 0) {
                draw_text(ren, f11, "No devices detected yet...",
                          SW / 2, SPL_Y + OV_TITLE_H + SPL_H / 4,
                          C_DIM, true);
            }
        }

        SDL_RenderPresent(ren);
        SDL_Delay(40);   /* ~25 fps – plenty for this dashboard */
    }

    /* ── Cleanup ── */
    TTF_CloseFont(f9);
    if (f11 != f13) TTF_CloseFont(f11);
    TTF_CloseFont(f13);
    if (f14 != f13) TTF_CloseFont(f14);
    if (f16 != f13) TTF_CloseFont(f16);
    TTF_Quit();
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    curl_global_cleanup();
    return 0;
}
