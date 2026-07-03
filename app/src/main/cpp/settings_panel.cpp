#include "settings_panel.h"
#include "app_state.h"
#include "eq_panel.h"
#include "menu_model.h"
#include "log.h"        // nowNs()
#include <cstdio>
#include <cstring>
#include <cmath>

// session state
bool gSettingsOpen = false;
int  gSettingsCat  = 0;
std::vector<float> gSettingsScroll;

static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ===========================================================================
// AUDIO category: the 16-band EQ as a MK_CUSTOM item. It owns its own native
// panel-local layout (eq_panel.h coords); the MenuHover.part encodes which sub-
// region the pointer is over:
//   0..15    : over band b's track (highlight only -> empty space, scrolls)
//   100..115 : over band b's KNOB  (arms drag-draw)
//   200      : RESET button     201 : preset header     300+i : dropdown item i
// ===========================================================================
static void eqBuild(std::vector<float> &v, const MenuHover &h) {
    int band = (h.part >= 0 && h.part < 16)     ? h.part
             : (h.part >= 100 && h.part < 116)  ? h.part - 100 : -1;
    bool reset  = (h.part == 200);
    bool header = (h.part == 201);
    int  item   = (h.part >= 300) ? h.part - 300 : -1;
    buildEqVerts(v, band, reset, header, item);
}
static void eqHit(float cx, float cy, MenuHover &h) {
    if (cx >= kEqResetX0 && cx <= kEqResetX1 && cy <= kEqResetYTop && cy >= kEqResetYBot) {
        h.item = 0; h.part = 200; h.grab = true; return;
    }
    if (cx >= kEqPresetX0 && cx <= kEqPresetX1 && cy <= kEqPresetYTop && cy >= kEqPresetYBot) {
        h.item = 0; h.part = 201; h.grab = true; return;
    }
    if (gEqPresetOpen && cx >= kEqPresetX0 && cx <= kEqPresetX1 &&
        cy < kEqPresetYBot && cy >= kEqPresetYBot - kEqNumPresets*kEqItemH) {
        int it = (int)((kEqPresetYBot - cy) / kEqItemH);
        if (it >= 0 && it < kEqNumPresets) { h.item = 0; h.part = 300 + it; h.grab = true; }
        return;
    }
    if (cx >= kEqX0 && cx <= kEqX1 && cy <= kEqYTrackTop+0.03f && cy >= kEqYTrackBot-0.03f) {
        float colW = (kEqX1 - kEqX0) / kEqBands;
        int b = (int)((cx - kEqX0) / colW);
        if (b < 0) b = 0; else if (b >= kEqBands) b = kEqBands-1;
        // Drag-draw only ARMS on the knob (lever); the rest of the column scrolls.
        float lky = eqGainToY(gEqGains[b]);
        bool onKnob = (fabsf(cx - eqColCenterX(b)) <= colW*0.40f && fabsf(cy - lky) <= 0.045f);
        h.item = 0; h.part = onKnob ? (100 + b) : b; h.grab = onKnob;
    }
}
static void eqAct(const MenuHover &h, bool click, bool grab, float cx, float cy) {
    static bool sArmed = false;
    // current column under the pointer (drag can sweep across columns)
    int band = -1;
    if (cx >= kEqX0 && cx <= kEqX1) {
        float colW = (kEqX1 - kEqX0) / kEqBands;
        band = (int)((cx - kEqX0) / colW);
        if (band < 0) band = 0; else if (band >= kEqBands) band = kEqBands-1;
    }
    if (click && h.part >= 100 && h.part < 116) sArmed = true;
    if (sArmed && grab && band >= 0) {
        if (!gEqGrabbing) { gEqGrabbing = true; gEqActiveBand = band; }
        float ng = eqYToGain(cy);
        int to = band, from = (gEqActiveBand >= 0) ? gEqActiveBand : to;
        float fromG = gEqGains[from];
        if (from == to) gEqGains[to] = ng;
        else {
            int step = (to > from) ? 1 : -1;
            int n = (to > from) ? (to - from) : (from - to);
            for (int bb = from; ; bb += step) {
                int d = (bb > from) ? (bb - from) : (from - bb);
                gEqGains[bb] = fromG + (ng - fromG) * ((float)d / (float)n);
                if (bb == to) break;
            }
        }
        gEqActiveBand = to;
        for (int i = 0; i < kEqBands; i++) gEqCustoms[gEqPresetIdx][i] = gEqGains[i];
        gEqDirty = true; gEqChangeNs = nowNs();
        pushEqGains();
    }
    if (!grab) { sArmed = false; if (gEqGrabbing) { gEqGrabbing = false; gEqActiveBand = -1; } }
    if (click && h.part >= 200) {
        if (h.part == 200) {           // RESET
            for (int i = 0; i < kEqBands; i++) { gEqGains[i] = 0.0f; gEqCustoms[gEqPresetIdx][i] = 0.0f; }
            pushEqGains(); saveEqProfile();
            LOGI("EQ: reset %s to flat", kEqPresetNames[gEqPresetIdx]);
        } else if (h.part == 201) {    // preset header
            gEqPresetOpen = !gEqPresetOpen;
        } else if (h.part >= 300) {    // dropdown item
            applyEqPreset(h.part - 300); gEqPresetOpen = false;
        }
    }
}

// ---- model assembly --------------------------------------------------------
static void buildCoreModel(MenuModel &m) {
    // VIDEO -----------------------------------------------------------------
    MenuCategory video; video.name = "VIDEO";
    {
        MenuItem ipd; ipd.kind = MK_STEPPER; ipd.label = "SOFTWARE IPD";
        ipd.vmin = kIpdMin; ipd.vmax = kIpdMax; ipd.vstep = kIpdStep;
        ipd.get = []{ return gSoftIpdMm.load(); };
        ipd.set = [](float v){ gSoftIpdMm.store(v); gIpdChangeNs.store(nowNs()); gIpdDirty.store(true); };
        ipd.valueText = [](char *b,int n){ snprintf(b,n,"%.1f MM", gSoftIpdMm.load()); };
        video.items.push_back(ipd);

        MenuItem br; br.kind = MK_FADER; br.label = "BRIGHTNESS";
        br.vmin = 0.0f; br.vmax = 1.0f;
        br.get = []{ return gBrightnessFrac.load(); };
        br.set = [](float f){ gBrightnessFrac.store(f); gBrightnessSaved.store(true); gBrightnessApply.store(true); };
        br.valueText = [](char *b,int n){ snprintf(b,n,"%d PERCENT",(int)(gBrightnessFrac.load()*100.0f+0.5f)); };   // no '%' glyph in the font
        br.onCommit = []{ saveBrightness(); };
        video.items.push_back(br);

        // FIELD OF VIEW (higher-DPI lever): lower the per-eye FOV so the fixed server
        // buffer packs more pixels into the visible lens cone. A release-commit slider:
        // set() only updates the displayed value live during the drag (the "NN DEG"
        // readout above the bar follows the knob); the expensive apply -- warp mesh
        // rebuild (Pvr_SetProjectionFov + fEyeTextureFov globals) + resend view_params
        // -- fires ONCE on release via onCommit -> gFovDirty, so dragging doesn't churn
        // a warp re-point every frame. Range 50 deg .. headset native (kFovMax).
        MenuItem fov; fov.kind = MK_FADER; fov.label = "FIELD OF VIEW";
        fov.vmin = kFovMin; fov.vmax = kFovMax;
        fov.get = []{ return gStreamFovDeg.load(); };
        fov.set = [](float v){ gStreamFovDeg.store(roundf(v)); };   // live readout only (integer deg); no apply
        fov.valueText = [](char *b,int n){ snprintf(b,n,"%.0f DEG", gStreamFovDeg.load()); };
        fov.onCommit = []{ gFovDirty.store(true); };                // apply + save on release
        video.items.push_back(fov);
    }
    m.push_back(video);

    // AUDIO (EQ custom) -----------------------------------------------------
    MenuCategory audio; audio.name = "AUDIO"; audio.custom = true;
    {
        MenuItem eq; eq.kind = MK_CUSTOM; eq.customH = 0.66f;
        eq.cBuild = eqBuild; eq.cHit = eqHit; eq.cAct = eqAct;
        audio.items.push_back(eq);
    }
    m.push_back(audio);

    // DEBUG -----------------------------------------------------------------
    MenuCategory dbg; dbg.name = "DEBUG";
    {
        MenuItem eye; eye.kind = MK_TOGGLE; eye.label = "EYE DEBUG";
        eye.get = []{ return gEyeDebugOn.load()?1.0f:0.0f; };
        eye.set = [](float v){ gEyeDebugOn.store(v>0.5f); };
        eye.onChange = []{ saveEyeDebug(); gEyeTrackReapply.store(true); };
        dbg.items.push_back(eye);

        MenuItem diag; diag.kind = MK_DROPDOWN; diag.label = "DIAGNOSTICS HUD";
        diag.options = { "OFF", "PIPELINE", "SYSTEM" };
        diag.get = []{ return (float)gDiagHudMode.load(); };
        diag.set = [](float v){ gDiagHudMode.store((int)(v+0.5f)); };
        diag.onChange = []{ saveDiagHud(); };
        dbg.items.push_back(diag);
    }
    m.push_back(dbg);

    // LOBBY -----------------------------------------------------------------
    MenuCategory lobby; lobby.name = "LOBBY";
    {
        MenuItem theme; theme.kind = MK_TOGGLE; theme.label = "AMBER THEME";
        theme.get = []{ return gThemeAmber.load()?1.0f:0.0f; };
        theme.set = [](float v){ bool a=v>0.5f; gThemeAmber.store(a); applyUiTheme(a); };
        theme.onChange = []{ saveTheme(); gGridThemeDirty.store(true); };
        lobby.items.push_back(theme);
    }
    m.push_back(lobby);
}

MenuModel &settingsModel() {
    static MenuModel m;
    if (m.empty()) {
        buildCoreModel(m);
        gSettingsScroll.assign(m.size(), 0.0f);
    }
    return m;
}
int  settingsNumCats() { return (int)settingsModel().size(); }
void settingsClampCat() { int n = settingsNumCats(); if (gSettingsCat < 0) gSettingsCat = 0; else if (gSettingsCat >= n) gSettingsCat = n-1; }
float &settingsScroll() { settingsModel(); settingsClampCat(); return gSettingsScroll[gSettingsCat]; }

UiRect settingsTabRect(int i) { return { -0.55f, 0.28f - 0.11f*i, 0.18f, 0.085f }; }

// Structure signature: everything that can move the active category's geometry
// (item set + each item's kind/dropdown-open state + the EQ preset dropdown).
// Hover and scroll do NOT change vertex positions, so they're excluded -- which is
// why the extent below can be cached across frames and re-measured only when this
// signature changes.
static unsigned menuStructSig(const MenuCategory &c) {
    unsigned h = 2166136261u;
    auto mix = [&](unsigned x){ h = (h ^ x) * 16777619u; };
    mix((unsigned)c.items.size());
    for (const auto &it : c.items) { mix((unsigned)it.kind); mix(it.dropOpen ? 1u : 0u);
                                     mix((unsigned)it.options.size()); }
    mix(gEqPresetOpen ? 1u : 0u);
    return h;
}

// ---- measure (top-align the active content; scan its real vertex extent so it
// works for both auto-stacked simple items and self-positioning custom panels) --
//
// PERF: the extent only depends on the menu STRUCTURE (menuStructSig), not on
// hover/scroll, so it's cached per (category,structure). We build-to-measure (a
// throwaway geometry copy, scanned for its Y-extent) only on the rare frame the
// structure changes (open a category / toggle a dropdown); steady state is a cache
// hit, so there's no per-frame double-build against buildSettingsPanel's own draw.
void settingsMeasure(float &offX, float &offY, float &contentH) {
    settingsModel(); settingsClampCat();
    MenuCategory &cat = settingsModel()[gSettingsCat];

    static int      sExtCat = -1;
    static unsigned sExtSig = 0;
    static float    sExtMaxY = 0.0f, sExtContentH = 0.0f;

    unsigned sig = menuStructSig(cat);
    float maxY;
    if (sExtCat == gSettingsCat && sExtSig == sig) {
        maxY = sExtMaxY; contentH = sExtContentH;
    } else {
        static std::vector<float> tmp; tmp.clear();
        MenuHover none;
        menuBuild(tmp, cat, none);
        float minY = 1e9f, mxY = -1e9f;
        for (size_t i = 1; i < tmp.size(); i += 6) { float y = tmp[i]; if (y<minY) minY=y; if (y>mxY) mxY=y; }
        if (mxY < minY) { minY = 0; mxY = 0; }
        maxY = mxY; contentH = mxY - minY;
        sExtCat = gSettingsCat; sExtSig = sig; sExtMaxY = maxY; sExtContentH = contentH;
    }
    float maxScroll = contentH - kSetViewportH; if (maxScroll < 0) maxScroll = 0;
    float &s = gSettingsScroll[gSettingsCat];
    s = clampf(s, 0.0f, maxScroll);
    offX = kSetContentOffX;
    offY = (kCtTop - maxY) + s;
}

void buildSettingsPanel(std::vector<float> &v, float offX, float offY, float contentH,
                        const MenuHover &content, int tabHover, bool closeHover) {
    MenuModel &m = settingsModel();
    int cat = gSettingsCat;
    bool amber = gThemeAmber.load();
    if (amber) {
        appendQuad(v, kSetPanelL, kSetPanelTop, kSetPanelR, kSetPanelBot, 0.09f, 0.06f, 0.03f);
        appendQuad(v, kSetPanelL, kSetPanelTop, -0.45f, kSetPanelBot, 0.15f, 0.10f, 0.04f);
    } else {
        appendQuad(v, kSetPanelL, kSetPanelTop, kSetPanelR, kSetPanelBot, 0.06f, 0.07f, 0.10f);
        appendQuad(v, kSetPanelL, kSetPanelTop, -0.45f, kSetPanelBot, 0.10f, 0.12f, 0.16f);
    }
    uiTextC(v, m[cat].name, (kCtX0 + kCtX1) * 0.5f, kSetHdrY, 0.0055f, kUiTitle[0], kUiTitle[1], kUiTitle[2]);
    uiButton(v, kSetClose, "X", closeHover);

    // sidebar tabs (one per category, auto-stacked)
    for (int i = 0; i < (int)m.size(); i++) {
        bool active = (i == cat);
        UiRect r = settingsTabRect(i);
        float xL = r.cx - r.w*0.5f, xR = r.cx + r.w*0.5f;
        float yT = r.cy + r.h*0.5f, yB = r.cy - r.h*0.5f;
        float c0,c1,c2;
        if (active)            { c0=kUiFill[0]*0.40f; c1=kUiFill[1]*0.40f; c2=kUiFill[2]*0.40f; }
        else if (tabHover==i)  { c0=0.18f; c1=0.20f; c2=0.26f; }
        else                   { c0=0.10f; c1=0.11f; c2=0.15f; }
        appendQuad(v, xL, yT, xR, yB, c0, c1, c2);
        const float *tc = active ? kUiWhite : kUiTitle;
        uiTextC(v, m[i].name, r.cx, r.cy + 3.0f*0.005f, 0.005f, tc[0], tc[1], tc[2]);
    }

    // content: build builder-local, then translate + triangle-clip to the viewport
    static std::vector<float> c; c.clear();
    menuBuild(c, m[cat], content);
    for (size_t t = 0; t + 18 <= c.size(); t += 18) {
        float y0 = c[t+1]+offY, y1 = c[t+7]+offY, y2 = c[t+13]+offY;
        if ((y0>kCtTop && y1>kCtTop && y2>kCtTop) || (y0<kCtBot && y1<kCtBot && y2<kCtBot)) continue;
        for (int k = 0; k < 3; k++) {
            size_t b = t + k*6;
            float yy = c[b+1]+offY;
            if (yy>kCtTop) yy=kCtTop; else if (yy<kCtBot) yy=kCtBot;
            v.push_back(c[b+0]+offX); v.push_back(yy); v.push_back(c[b+2]);
            v.push_back(c[b+3]); v.push_back(c[b+4]); v.push_back(c[b+5]);
        }
    }

    // reference scrollbar
    if (contentH > kSetViewportH + 1e-4f) {
        appendQuad(v, kSbX, kCtTop, kSbX + kSbW, kCtBot, 0.14f, 0.15f, 0.20f);
        float maxScroll = contentH - kSetViewportH;
        float thumbH = kSetViewportH * (kSetViewportH / contentH); if (thumbH < 0.04f) thumbH = 0.04f;
        float frac = (maxScroll > 1e-5f) ? (gSettingsScroll[cat] / maxScroll) : 0.0f;
        float thumbTop = kCtTop - frac * (kSetViewportH - thumbH);
        appendQuad(v, kSbX, thumbTop, kSbX + kSbW, thumbTop - thumbH, kUiFill[0], kUiFill[1], kUiFill[2]);
    }
}
