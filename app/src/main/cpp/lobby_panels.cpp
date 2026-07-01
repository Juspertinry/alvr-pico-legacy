#include "lobby_panels.h"
#include "app_state.h"   // gVid*/g*MsX10 diag publish, gEyeDebugOn/gDiagHudMode
#include "alvr_ext.h"    // alvr_get_client_stats
#include "input.h"       // gCtrl/gCtrlMutex (controller battery for the system page)
#include "log.h"         // nowNs
#include <cstdio>
#include <cstring>

// Build the diagnostics text into `v` in PANEL-LOCAL metres (centred at origin);
// the caller projects it onto a tilted 3D panel. Categorised, clearly named, and
// the fast ALVR pipeline numbers are throttled so they don't flicker.
// ---------------------------------------------------------------------------
// System telemetry (page 2): CPU/GPU utilisation + on-die temperatures, read
// straight from sysfs/procfs. All nodes are world-readable on the Neo 2 (SD845);
// any that fail just read as "--". Sampled at the HUD's ~4Hz rebuild cadence.
// ---------------------------------------------------------------------------
namespace {

// read a single whole-number value from a sysfs/proc file; returns false on miss.
bool readLong(const char *path, long *out) {
    FILE *f = fopen(path, "r");
    if (!f) return false;
    long v = 0; int n = fscanf(f, "%ld", &v); fclose(f);
    if (n != 1) return false;
    *out = v; return true;
}

// max thermal_zone temp (milli-C) whose `type` contains `needle`. -1 if none.
// Zone->type mapping is fixed after boot: cache the matching zone indices per needle
// on first use so repeat calls only read temp nodes (not all 40 type files). Small
// fixed cache -- the diag HUD queries a handful of distinct needles.
float zoneTempC(const char *needle) {
    struct Cache { char needle[16]; int zones[8]; int n; };
    static Cache sCache[6];
    static int   sCacheN = 0;
    Cache *c = nullptr;
    for (int i = 0; i < sCacheN; i++)
        if (strncmp(sCache[i].needle, needle, sizeof(sCache[i].needle) - 1) == 0) { c = &sCache[i]; break; }
    if (!c && sCacheN < (int)(sizeof(sCache) / sizeof(sCache[0]))) {
        c = &sCache[sCacheN++];
        snprintf(c->needle, sizeof(c->needle), "%s", needle);
        c->n = 0;
        for (int z = 0; z < 40 && c->n < 8; z++) {
            char p[128], type[64] = {0};
            snprintf(p, sizeof(p), "/sys/class/thermal/thermal_zone%d/type", z);
            FILE *f = fopen(p, "r");
            if (!f) continue;             // gap in numbering -> keep scanning
            if (!fgets(type, sizeof(type), f)) { fclose(f); continue; }
            fclose(f);
            if (strstr(type, needle)) c->zones[c->n++] = z;
        }
    }
    if (!c) return -1.0f;
    long best = -1;
    for (int i = 0; i < c->n; i++) {
        char p[128];
        snprintf(p, sizeof(p), "/sys/class/thermal/thermal_zone%d/temp", c->zones[i]);
        long t = 0; if (readLong(p, &t) && t > best) best = t;
    }
    return best < 0 ? -1.0f : (float)best / 1000.0f;
}

// average scaling_cur_freq/cpuinfo_max_freq (%) over a cpu index range.
float cpuClockPct(int lo, int hi) {
    static long sMaxFreq[8] = {0};   // cpuinfo_max_freq is fixed per core -> read once
    float sum = 0; int n = 0;
    for (int c = lo; c <= hi && c < 8; c++) {
        char p[128]; long cur = 0;
        if (sMaxFreq[c] == 0) {
            snprintf(p, sizeof(p), "/sys/devices/system/cpu/cpu%d/cpufreq/cpuinfo_max_freq", c);
            if (!readLong(p, &sMaxFreq[c]) || sMaxFreq[c] <= 0) { sMaxFreq[c] = -1; }
        }
        if (sMaxFreq[c] <= 0) continue;
        snprintf(p, sizeof(p), "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_cur_freq", c);
        if (!readLong(p, &cur)) continue;
        sum += 100.0f * (float)cur / (float)sMaxFreq[c]; n++;
    }
    return n ? sum / n : -1.0f;
}

// aggregate CPU busy% from /proc/stat deltas between calls (stateful).
float cpuBusyPct() {
    FILE *f = fopen("/proc/stat", "r");
    if (!f) return -1.0f;
    char cpu[8]; unsigned long long u=0,ni=0,s=0,id=0,io=0,irq=0,sirq=0,st=0;
    int n = fscanf(f, "%7s %llu %llu %llu %llu %llu %llu %llu %llu",
                   cpu,&u,&ni,&s,&id,&io,&irq,&sirq,&st);
    fclose(f);
    if (n < 5) return -1.0f;
    unsigned long long idle = id + io;
    unsigned long long total = u+ni+s+id+io+irq+sirq+st;
    static unsigned long long pIdle = 0, pTotal = 0;
    unsigned long long dT = total - pTotal, dI = idle - pIdle;
    pIdle = idle; pTotal = total;
    if (dT == 0) return -1.0f;
    float pct = 100.0f * (float)(dT - dI) / (float)dT;
    return pct < 0 ? 0 : (pct > 100 ? 100 : pct);
}

} // namespace

// Page 2: processor / GPU / heat. Same 3-column layout + bg quad as page 1.
static void buildSysOverlay(std::vector<float> &v) {
    // throttle the actual sysfs sampling to ~3/sec so values are readable.
    static float held[12]; static bool have = false; static uint64_t last = 0;
    uint64_t now = nowNs();
    if (!have || now - last > 333000000ULL) {
        held[0] = cpuBusyPct();          // overall CPU busy %
        held[1] = cpuClockPct(0, 3);     // little-cluster clock %
        held[2] = cpuClockPct(4, 7);     // big-cluster clock %
        long gl = 0;
        held[3] = readLong("/sys/class/kgsl/kgsl-3d0/devfreq/gpu_load", &gl) ? (float)gl : -1.0f; // GPU busy %
        long gc = 0, gm = 0;
        bool okc = readLong("/sys/class/kgsl/kgsl-3d0/devfreq/cur_freq", &gc);
        bool okm = readLong("/sys/class/kgsl/kgsl-3d0/devfreq/max_freq", &gm);
        held[4] = (okc && okm && gm > 0) ? 100.0f * (float)gc / (float)gm : -1.0f; // GPU clock %
        held[5] = okc ? (float)gc / 1e6f : -1.0f;                                  // GPU MHz
        held[6] = zoneTempC("cpu");      // hottest CPU zone
        held[7] = zoneTempC("gpu");      // hottest GPU zone
        held[8] = zoneTempC("ddr");      // memory
        held[9] = zoneTempC("aoss");     // SoC / always-on
        // Controller battery % (ext[40], plumbed via keys[10]); -1 = not connected.
        held[10] = held[11] = -1.0f;
        { std::lock_guard<std::mutex> lk(gCtrlMutex);
          if (gCtrl[0].conn == 1 && gCtrl[0].keyCount > 10) held[10] = (float)gCtrl[0].keys[10];
          if (gCtrl[1].conn == 1 && gCtrl[1].keyCount > 10) held[11] = (float)gCtrl[1].keys[10]; }
        last = now; have = true;
    }

    const float px  = 0.0019f;
    const float dy  = 0.026f;
    const float hdrCol[3] = {kUiFill[0], kUiFill[1], kUiFill[2]};   // themed accent
    const float valCol[3] = {0.95f, 0.96f, 1.00f};
    const float hotCol[3] = {1.00f, 0.55f, 0.35f};   // warm tint for high temps
    char line[96];

    const float colL[3] = { -0.24f, -0.07f, 0.10f };
    const float yTop    = 0.090f;

    appendQuad(v, -0.28f, yTop + 0.030f, 0.28f, yTop - 6.0f*dy - 0.050f,
               kUiBg[0], kUiBg[1], kUiBg[2]);

    auto hdr = [&](int c, const char *s){ uiTextL(v, s, colL[c], yTop, px, hdrCol[0], hdrCol[1], hdrCol[2]); };
    auto row = [&](int c, int r, const char *s){ uiTextL(v, s, colL[c], yTop-(r+1)*dy, px, valCol[0], valCol[1], valCol[2]); };
    // temp row: tints orange past 80C so throttling is obvious at a glance.
    auto trow = [&](int c, int r, const char *lbl, float t){
        char b[96];
        if (t < 0) snprintf(b, sizeof(b), "%s --", lbl);
        else       snprintf(b, sizeof(b), "%s %.0f", lbl, t);
        const float *col = (t >= 80.0f) ? hotCol : valCol;
        uiTextL(v, b, colL[c], yTop-(r+1)*dy, px, col[0], col[1], col[2]);
    };
    auto pctStr = [](char *b, size_t n, const char *lbl, float p){
        if (p < 0) snprintf(b, n, "%s --", lbl); else snprintf(b, n, "%s %.0f%%", lbl, p);
    };

    // --- Column 0: CPU ---
    hdr(0, "CPU");
    pctStr(line, sizeof(line), "USAGE", held[0]); row(0,0,line);
    pctStr(line, sizeof(line), "LITTLE", held[1]); row(0,1,line);
    pctStr(line, sizeof(line), "BIG", held[2]);    row(0,2,line);
    trow(0,3,"TEMP", held[6]);

    // --- Column 1: GPU ---
    hdr(1, "GPU");
    pctStr(line, sizeof(line), "USAGE", held[3]); row(1,0,line);
    pctStr(line, sizeof(line), "CLOCK", held[4]); row(1,1,line);
    if (held[5] < 0) snprintf(line, sizeof(line), "MHZ  --");
    else             snprintf(line, sizeof(line), "MHZ  %.0f", held[5]);
    row(1,2,line);
    trow(1,3,"TEMP", held[7]);

    // --- Column 2: heat (C) ---
    hdr(2, "HEAT (C)");
    trow(2,0,"CPU", held[6]);
    trow(2,1,"GPU", held[7]);
    trow(2,2,"DDR", held[8]);
    trow(2,3,"SOC", held[9]);

    // --- Row 4: controller battery (L/R), warm tint under 20% ---
    auto batCell = [&](int c, const char *lbl, float p){
        char b[96];
        if (p < 0) snprintf(b, sizeof(b), "%s --", lbl);
        else       snprintf(b, sizeof(b), "%s %.0f%%", lbl, p);
        const float *col = (p >= 0.0f && p < 20.0f) ? hotCol : valCol;
        uiTextL(v, b, colL[c], yTop - 5.0f*dy, px, col[0], col[1], col[2]);
    };
    batCell(0, "CTRL L", held[10]);
    batCell(1, "CTRL R", held[11]);

    uiTextC(v, "SYSTEM TELEMETRY", 0.0f, yTop - 6.0f*dy - 0.010f, px*0.9f, 0.55f, 0.65f, 0.78f);
}

void buildDiagOverlay(std::vector<float> &v, int page) {
    if (page == 2) { buildSysOverlay(v); return; }
    // Throttle the per-frame ALVR latency stats to ~3/sec (the decode/queue/render
    // numbers jitter every frame otherwise).
    static float held[6] = {0,0,0,0,0,0};
    static bool  haveHeld = false;
    static uint64_t lastUpd = 0;
    uint64_t now = nowNs();
    if (now - lastUpd > 333000000ULL) {
        float st[6];
        if (alvr_get_client_stats(st)) { for (int i=0;i<6;i++) held[i]=st[i]; haveHeld=true; }
        else haveHeld = false;
        lastUpd = now;
    }
    // Virtual-Desktop-style compact bar: a flat grey background plane with three
    // columns (label + value), blue headers, white-ish values. Small & 2D.
    const float px  = 0.0019f;          // glyph size (metres) - 2x
    const float dy  = 0.026f;           // line spacing
    const float hdrCol[3] = {kUiFill[0], kUiFill[1], kUiFill[2]};   // themed accent   // blue/cyan headers
    const float valCol[3] = {0.95f, 0.96f, 1.00f};   // near-white values
    char line[96];

    // Column geometry (panel-local metres). Three label-left columns.
    const float colL[3] = { -0.24f, -0.07f, 0.10f };   // tight columns, block centred on x=0
    const float colW    = 0.16f;
    const float yTop    = 0.090f;       // first header baseline
    (void) colW;

    // Background plane (drawn first so text lands on top - painter's order).
    appendQuad(v, -0.28f, yTop + 0.030f, 0.28f, yTop - 5.0f*dy - 0.050f,
               kUiBg[0], kUiBg[1], kUiBg[2]);

    auto colHdr = [&](int c, const char *s){
        uiTextL(v, s, colL[c], yTop, px, hdrCol[0], hdrCol[1], hdrCol[2]);
    };
    auto colRow = [&](int c, int r, const char *s){
        uiTextL(v, s, colL[c], yTop - (r+1)*dy, px, valCol[0], valCol[1], valCol[2]);
    };

    // --- Column 0: pipeline latency (ms) ---
    colHdr(0, "LATENCY (MS)");
    if (haveHeld) {
        snprintf(line, sizeof(line), "TOTAL   %.1f", held[0]); colRow(0,0,line);
        snprintf(line, sizeof(line), "DECODE  %.1f", held[1]); colRow(0,1,line);
        snprintf(line, sizeof(line), "QUEUE   %.1f", held[2]); colRow(0,2,line);
        snprintf(line, sizeof(line), "RENDER  %.1f", held[3]); colRow(0,3,line);
    } else {
        colRow(0,0,"WAITING...");
    }

    // --- Column 1: video rate (per second) ---
    colHdr(1, "VIDEO RATE");
    float fps = (haveHeld && held[5] > 0.01f) ? (1000.0f / held[5]) : (float) gVidSubmit.load();
    snprintf(line, sizeof(line), "FPS      %.0f", fps);              colRow(1,0,line);
    snprintf(line, sizeof(line), "DECODED  %d",  gVidDecoded.load()); colRow(1,1,line);
    snprintf(line, sizeof(line), "SUBMIT   %d",  gVidSubmit.load());  colRow(1,2,line);
    snprintf(line, sizeof(line), "DROPPED  %d",  gVidDropped.load()); colRow(1,3,line);

    // --- Column 2: per-stage frame timing (ms). render/encode are our GPU-issue
    // cost; enqueue is the submit-to-SDK-warp cost (incl vsync backpressure), not
    // the warp's own compute. ---
    colHdr(2, "TIME(MS)");
    snprintf(line, sizeof(line), "GAP     %.1f", gGapMsX10.load()/10.0f); colRow(2,0,line);
    snprintf(line, sizeof(line), "ENCODE  %.1f", gEncMsX10.load()/10.0f); colRow(2,1,line);
    snprintf(line, sizeof(line), "ENQUEUE %.1f", gEnqMsX10.load()/10.0f); colRow(2,2,line);
    // warp-submit fence timeouts/sec (slot not GPU-complete in budget -> tear). 0 = healthy.
    snprintf(line, sizeof(line), "FENCETMO %d",  gFenceTimeouts.load()); colRow(2,3,line);

    // Footer, centred under the columns.
    uiTextC(v, "ALVR DIAGNOSTICS", 0.0f, yTop - 5.0f*dy - 0.010f, px*0.9f, 0.55f, 0.65f, 0.78f);
}
