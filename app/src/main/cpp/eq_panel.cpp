#include "eq_panel.h"
#include "app_state.h"  // saveAllConfig (EQ now lives in the unified config.txt)
#include "ui_kit.h"     // appendTextLine / appendQuad / uiTextL
#include "alvr_ext.h"   // alvr_set_eq_gains
#include "log.h"
#include <cstdio>
#include <cstdlib>      // getenv

float    gEqGains[kEqBands] = {0};
float    gEqCustoms[2][kEqBands] = {{0},{0}};
int      gEqActiveBand = -1;
bool     gEqGrabbing   = false;
bool     gEqDirty      = false;
uint64_t gEqChangeNs   = 0;
bool     gEqPresetOpen = false;
int      gEqPresetIdx  = 0;
int      gDominantHand = 1;

const char *kEqPresetNames[kEqNumPresets] = { "CUSTOM 1", "CUSTOM 2" };
// Band centre frequencies for the readout (Hz) -- MUST match eq.rs CENTER_FREQS.
static const float kEqFreqs[kEqBands] = {
    31, 47, 71, 107, 161, 242, 364, 548, 825, 1242, 1869, 2813, 4234, 6373, 9593, 14438
};

void pushEqGains() {
    alvr_set_eq_gains(gEqGains, kEqBands);
}
// Switch active slot to CUSTOM idx: load that slot's saved curve into the live gains.
void applyEqPreset(int idx) {
    if (idx < 0 || idx >= kEqNumPresets) return;
    for (int i = 0; i < kEqBands; i++) gEqGains[i] = gEqCustoms[idx][i];
    gEqPresetIdx = idx;
    pushEqGains();
    saveEqProfile();
    LOGI("EQ: switched to %s", kEqPresetNames[idx]);
}
// EQ state (active slot + the two custom curves) is persisted in the unified
// $HOME/config.txt; saving just rewrites that whole file via saveAllConfig().
void saveEqProfile() { saveAllConfig(); }

// Build the EQ panel: title + RESET button + 16 faders + readout + preset
// dropdown, in panel-local metres. Hover flags brighten the targeted control.
void buildEqVerts(std::vector<float> &v, int hoverBand, bool resetHover,
                  bool headerHover, int presetHoverItem) {
    const float px = kEqText;   // 1x text
    appendTextLine(v, "AUDIO EQ", kEqTitleY, px, kUiTitle[0], kUiTitle[1], kUiTitle[2]);   // title

    // RESET button (centred, between the faders/readout and the preset dropdown).
    {
        float rr = resetHover ? 0.55f : 0.30f, rg = resetHover ? 0.30f : 0.18f, rb = resetHover ? 0.30f : 0.18f;
        appendQuad(v, kEqResetX0, kEqResetYTop, kEqResetX1, kEqResetYBot, rr, rg, rb);
        float cx = (kEqResetX0 + kEqResetX1) * 0.5f;
        // appendTextLine centres on x=0, so build the label there and shift it to the
        // button centre (cx) below.
        float rpx = px * 0.78f;   // smaller so RESET fits the narrower right-side button
        std::vector<float> tmp;
        appendTextLine(tmp, "RESET", (kEqResetYTop+kEqResetYBot)*0.5f + 3.5f*rpx, rpx, 1,1,1);
        for (size_t i = 0; i < tmp.size(); i += 6) tmp[i] += cx;   // shift x to button centre
        v.insert(v.end(), tmp.begin(), tmp.end());
    }

    // Faders.
    const float colW = (kEqX1 - kEqX0) / kEqBands;
    const float trackHalf = colW * 0.16f;
    const float y0dB = eqGainToY(0.0f);
    for (int b = 0; b < kEqBands; b++) {
        float cx = eqColCenterX(b);
        bool hot = (b == hoverBand);
        float tr = hot ? 0.30f : 0.16f, tg = hot ? 0.30f : 0.16f, tb = hot ? 0.36f : 0.20f;
        appendQuad(v, cx-trackHalf, kEqYTrackTop, cx+trackHalf, kEqYTrackBot, tr, tg, tb);
        appendQuad(v, cx-trackHalf, y0dB+0.003f, cx+trackHalf, y0dB-0.003f, 0.45f, 0.45f, 0.5f);  // 0 dB tick
        float ky = eqGainToY(gEqGains[b]);
        float fr_, fg_, fb_;
        if (gEqGains[b] >= 0) { fr_=kUiFill[0]; fg_=kUiFill[1]; fb_=kUiFill[2]; }  // boost = accent
        else                  { fr_=1.0f;  fg_=0.55f; fb_=0.2f; }                  // cut = orange
        float yhi = ky > y0dB ? ky : y0dB, ylo = ky > y0dB ? y0dB : ky;
        appendQuad(v, cx-trackHalf, yhi, cx+trackHalf, ylo, fr_, fg_, fb_);
        float kr = hot ? 1.0f : 0.9f, kg = hot ? 1.0f : 0.9f, kb = hot ? 0.4f : 0.9f;
        appendQuad(v, cx-colW*0.40f, ky+0.02f, cx+colW*0.40f, ky-0.02f, kr, kg, kb);
    }

    // Readout of the hovered/last band.
    int rb = (hoverBand >= 0) ? hoverBand : gEqActiveBand;
    if (rb >= 0) {
        char buf[32];
        float fr = kEqFreqs[rb];
        if (fr >= 1000.0f) snprintf(buf, sizeof(buf), "%.1fKHZ %+.1fDB", fr/1000.0f, gEqGains[rb]);
        else               snprintf(buf, sizeof(buf), "%.0fHZ %+.1fDB", fr, gEqGains[rb]);
        uiTextL(v, buf, kEqX0, kEqReadoutY, px, 1, 1, 1);   // condensed to the left edge
    }

    // Preset dropdown header.
    {
        float hr = headerHover ? 0.40f : 0.22f, hg = headerHover ? 0.40f : 0.22f, hb = headerHover ? 0.48f : 0.28f;
        appendQuad(v, kEqPresetX0, kEqPresetYTop, kEqPresetX1, kEqPresetYBot, hr, hg, hb);
        char hbuf[40];
        const char *pn = kEqPresetNames[gEqPresetIdx];   // 0..5 always valid (0 = CUSTOM)
        snprintf(hbuf, sizeof(hbuf), "PRESET: %s %s", pn, gEqPresetOpen ? "^" : "~");
        appendTextLine(v, hbuf, (kEqPresetYTop+kEqPresetYBot)*0.5f + 3.5f*px, px, 1, 1, 1);
    }
    // Dropdown items (when open), stacked below the header.
    if (gEqPresetOpen) {
        for (int i = 0; i < kEqNumPresets; i++) {
            float yT = kEqPresetYBot - i * kEqItemH;
            float yB = yT - kEqItemH;
            bool ih = (i == presetHoverItem);
            float ir = ih ? 0.35f : 0.14f, ig = ih ? 0.35f : 0.14f, ib = ih ? 0.42f : 0.18f;
            appendQuad(v, kEqPresetX0, yT, kEqPresetX1, yB, ir, ig, ib);
            appendTextLine(v, kEqPresetNames[i], (yT+yB)*0.5f + 3.5f*px, px, 1, 1, 1);
        }
    }
}
