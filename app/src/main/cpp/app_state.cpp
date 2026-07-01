#include "app_state.h"
#include "eq_panel.h"   // EQ state folded into the unified config file
#include "log.h"
#include <cstdio>
#include <cstdlib>   // getenv

std::atomic<float>    gSoftIpdMm{65.0f};
std::atomic<bool>     gIpdDirty{false};
std::atomic<uint64_t> gIpdChangeNs{0};
std::atomic<float>    gSentIpdMm{-1.0f};

std::atomic<bool> gOkClick{false};
std::atomic<bool> gOkHeld{false};
std::atomic<bool> gSideHeld{false};
std::atomic<bool> gManualLobby{false};

std::atomic<bool> gEyeDebugOn{false};
std::atomic<int>  gDiagHudMode{0};
std::atomic<bool>  gThemeAmber{false};
std::atomic<float> gBrightnessFrac{1.0f};
std::atomic<bool>  gBrightnessSaved{false};

std::atomic<bool> gGridThemeDirty{false};
std::atomic<bool> gEyeTrackReapply{false};
std::atomic<bool> gBrightnessApply{false};

std::atomic<int> gVidDecoded{0}, gVidSubmit{0}, gVidDropped{0};
std::atomic<int> gGapMsX10{0}, gRenderMsX10{0}, gEncMsX10{0}, gEnqMsX10{0};
std::atomic<int> gFenceTimeouts{0};   // warp-submit fence-wait timeouts/sec

// ---------------------------------------------------------------------------
// Unified persistence: one $HOME/config.txt, each setting on its own line as
// raw values (no keys/labels -- positional, not meant to be hand-edited).
// saveAllConfig() rewrites the whole file from the live atomics; every public
// saveX() is a thin wrapper around it. loadAllConfig() reads the lines back in
// the same fixed order at boot, clamping like the old per-file loaders did.
// Line order:
//   1 softIpd  2 eyeDebug  3 diagHud  4 theme  5 brightness(-1 = unset)
//   6 eqActive  7 eqCustom1[16]  8 eqCustom2[16]
// ---------------------------------------------------------------------------
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static bool configPath(char *out, size_t n) {
    const char *home = getenv("HOME");
    if (!home) return false;
    snprintf(out, n, "%s/config.txt", home);
    return true;
}

void saveAllConfig() {
    char path[512];
    if (!configPath(path, sizeof(path))) return;
    FILE *f = fopen(path, "w");
    if (!f) { LOGE("config: save failed (%s)", path); return; }
    fprintf(f, "%.2f\n", gSoftIpdMm.load());
    fprintf(f, "%d\n",   gEyeDebugOn.load() ? 1 : 0);
    fprintf(f, "%d\n",   gDiagHudMode.load());
    fprintf(f, "%d\n",   gThemeAmber.load() ? 1 : 0);
    fprintf(f, "%.3f\n", gBrightnessSaved.load() ? gBrightnessFrac.load() : -1.0f);
    fprintf(f, "%d\n",   gEqPresetIdx);
    for (int i = 0; i < kEqBands; i++) fprintf(f, "%.2f%c", gEqCustoms[0][i], i == kEqBands - 1 ? '\n' : ' ');
    for (int i = 0; i < kEqBands; i++) fprintf(f, "%.2f%c", gEqCustoms[1][i], i == kEqBands - 1 ? '\n' : ' ');
    fclose(f);
}

// One-time migration: read the OLD per-setting files into the live atomics, then
// delete every legacy file -- both the ones we migrate and the dead leftovers from
// removed features (power toggles, the decoder buffer A/B toggle, eye_calib). The
// caller writes the unified config.txt afterward. Returns true if any migratable
// setting file existed. Runs only when config.txt is absent (i.e. once, after the
// upgrade), so the device transparently converts old format -> new on next launch.
static bool migrateLegacyConfig() {
    const char *home = getenv("HOME");
    if (!home) return false;
    char path[512];
    FILE *f;
    bool any = false;
    auto openOld = [&](const char *name) -> FILE * {
        snprintf(path, sizeof(path), "%s/%s", home, name);
        FILE *ff = fopen(path, "r");
        if (ff) any = true;
        return ff;
    };
    if ((f = openOld("software_ipd.txt"))) { float v; if (fscanf(f, "%f", &v) == 1) gSoftIpdMm.store(clampf(v, kIpdMin, kIpdMax)); fclose(f); }
    if ((f = openOld("eye_debug.txt")))    { int v;   if (fscanf(f, "%d", &v) == 1) gEyeDebugOn.store(v != 0); fclose(f); }
    if ((f = openOld("diag_hud.txt")))     { int v;   if (fscanf(f, "%d", &v) == 1) { if (v < 0) v = 0; if (v > 2) v = 2; gDiagHudMode.store(v); } fclose(f); }
    if ((f = openOld("theme.txt")))        { int v;   if (fscanf(f, "%d", &v) == 1) gThemeAmber.store(v != 0); fclose(f); }
    if ((f = openOld("brightness.txt")))   { float v; if (fscanf(f, "%f", &v) == 1) { gBrightnessFrac.store(clampf(v, 0.0f, 1.0f)); gBrightnessSaved.store(true); } fclose(f); }
    if ((f = openOld("eq_profile.txt"))) {   // old format: idx line, then 2x16 gains (one per line)
        int idx = 0; if (fscanf(f, "%d", &idx) != 1) idx = 0; if (idx < 0 || idx >= kEqNumPresets) idx = 0; gEqPresetIdx = idx;
        for (int s = 0; s < kEqNumPresets; s++)
            for (int i = 0; i < kEqBands; i++) { float g; if (fscanf(f, "%f", &g) != 1) break; gEqCustoms[s][i] = clampf(g, -kEqGainMax, kEqGainMax); }
        fclose(f);
    }
    // Sweep away every legacy file: the migrated ones above plus dead leftovers from
    // removed features (maxpower/adaptive_power toggles, the decoder buffer A/B
    // toggle's low_latency/buffer_frames, and the unused eye_calib).
    static const char *kLegacy[] = {
        "software_ipd.txt", "eye_debug.txt", "diag_hud.txt", "theme.txt", "brightness.txt",
        "eq_profile.txt",
        "maxpower.txt", "adaptive_power.txt", "low_latency.txt", "buffer_frames.txt", "eye_calib.txt",
    };
    for (const char *name : kLegacy) { snprintf(path, sizeof(path), "%s/%s", home, name); remove(path); }
    return any;
}

void loadAllConfig() {
    char path[512];
    if (!configPath(path, sizeof(path))) return;
    FILE *f = fopen(path, "r");
    if (!f) {
        // No unified file yet: migrate the old per-setting files (and sweep dead
        // leftovers), then bake the result into config.txt for all future launches.
        if (migrateLegacyConfig()) { saveAllConfig(); LOGI("config: migrated legacy files -> %s", path); }
        else                       { LOGI("config: no saved file, using defaults"); }
    } else {
    char ln[1024];
    float fv; int iv;
    if (fgets(ln, sizeof(ln), f) && sscanf(ln, "%f", &fv) == 1) gSoftIpdMm.store(clampf(fv, kIpdMin, kIpdMax));
    if (fgets(ln, sizeof(ln), f) && sscanf(ln, "%d", &iv) == 1) gEyeDebugOn.store(iv != 0);
    if (fgets(ln, sizeof(ln), f) && sscanf(ln, "%d", &iv) == 1) { if (iv < 0) iv = 0; if (iv > 2) iv = 2; gDiagHudMode.store(iv); }
    if (fgets(ln, sizeof(ln), f) && sscanf(ln, "%d", &iv) == 1) gThemeAmber.store(iv != 0);
    if (fgets(ln, sizeof(ln), f) && sscanf(ln, "%f", &fv) == 1 && fv >= 0.0f) {
        gBrightnessFrac.store(clampf(fv, 0.0f, 1.0f)); gBrightnessSaved.store(true);
    }
    if (fgets(ln, sizeof(ln), f) && sscanf(ln, "%d", &iv) == 1) { if (iv < 0 || iv >= kEqNumPresets) iv = 0; gEqPresetIdx = iv; }
    for (int slot = 0; slot < 2; slot++) {
        if (!fgets(ln, sizeof(ln), f)) break;
        const char *p = ln;
        for (int i = 0; i < kEqBands; i++) {
            float g; int adv = 0;
            if (sscanf(p, " %f%n", &g, &adv) != 1) break;
            gEqCustoms[slot][i] = clampf(g, -kEqGainMax, kEqGainMax);
            p += adv;
        }
    }
    fclose(f);
    LOGI("config: loaded %s", path);
    }
    // mirror the active EQ slot into the live gain set (the caller pushes it to the DSP)
    if (gEqPresetIdx < 0 || gEqPresetIdx >= kEqNumPresets) gEqPresetIdx = 0;
    for (int i = 0; i < kEqBands; i++) gEqGains[i] = gEqCustoms[gEqPresetIdx][i];
}

// Thin wrappers: every settings change rewrites the whole config.txt.
void saveSoftIpd()    { saveAllConfig(); }
void saveEyeDebug()   { saveAllConfig(); }
void saveDiagHud()    { saveAllConfig(); }
void saveTheme()      { saveAllConfig(); }
void saveBrightness() { gBrightnessSaved.store(true); saveAllConfig(); }
