#include "menu_model.h"
#include "log.h"      // nowNs() (CLOCK_MONOTONIC, shared with the render loop)
#include <cstdio>
#include <cmath>
#include <cstdint>

// ---- per-kind row metrics (builder-local metres) --------------------------
// Each row occupies [yTop, yTop-height]; widgets sit at fixed offsets below yTop.
static constexpr float kGap        = 0.02f;
static constexpr float kHToggle    = 0.075f, kRowToggle  = kHToggle + 0.055f;
static constexpr float kHButton    = 0.070f, kRowButton  = kHButton + 0.060f;
static constexpr float kFaderH     = 0.050f, kRowFader   = 0.160f;
static constexpr float kStepBtnH   = 0.100f, kRowStepper = 0.210f;
static constexpr float kDropHdrH   = 0.075f, kDropItemH  = 0.060f, kDropLabelH = 0.055f;
static constexpr float kRowW       = 0.50f;          // toggle band width
static constexpr float kFaderW     = 0.40f;
static constexpr float kStepBtnW   = 0.13f, kStepBtnX = 0.13f;

static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Interactive rects, derived purely from the row's top Y (single source of truth
// shared by build + hit + apply).
static inline UiRect rToggle(float yTop) { return { 0.0f, yTop - kHToggle*0.5f, kRowW, kHToggle }; }
static inline UiRect rButton(float yTop) { return { 0.0f, yTop - kHButton*0.5f, 0.30f, kHButton }; }
static inline UiRect rFader (float yTop) { return { 0.0f, yTop - 0.105f, kFaderW, kFaderH }; }
static inline UiRect rStepMinus(float yTop) { return { -kStepBtnX, yTop - 0.150f, kStepBtnW, kStepBtnH }; }
static inline UiRect rStepPlus (float yTop) { return {  kStepBtnX, yTop - 0.150f, kStepBtnW, kStepBtnH }; }
// The dropdown reserves a title line at the top (kDropLabelH), then the header,
// then (when open) the option list -- so the header/items sit below the label.
static inline UiRect rDropHdr (float yTop) { return { 0.0f, yTop - kDropLabelH - kDropHdrH*0.5f, kRowW, kDropHdrH }; }
static inline UiRect rDropItem(float yTop, int i) { return { 0.0f, yTop - kDropLabelH - kDropHdrH - kDropItemH*(i + 0.5f), kRowW, kDropItemH }; }

float menuRowHeight(const MenuItem &it) {
    switch (it.kind) {
    case MK_TOGGLE:  return kRowToggle;
    case MK_BUTTON:  return kRowButton;
    case MK_FADER:   return kRowFader;
    case MK_STEPPER: return kRowStepper;
    case MK_DROPDOWN: return kDropLabelH + kDropHdrH + (it.dropOpen ? (float)it.options.size()*kDropItemH : 0.0f) + 0.055f;
    case MK_CUSTOM:  return it.customH + kGap;
    }
    return kRowToggle;
}

float menuRowTop(const MenuCategory &c, int i) {
    float y = kMenuTopY;
    for (int k = 0; k < i && k < (int)c.items.size(); k++) y -= menuRowHeight(c.items[k]);
    return y;
}

float menuContentH(const MenuCategory &c) {
    float h = 0.0f;
    for (const auto &it : c.items) h += menuRowHeight(it);
    return h;
}

// ---- build -----------------------------------------------------------------
void menuBuild(std::vector<float> &v, const MenuCategory &c, const MenuHover &h) {
    if (c.custom && !c.items.empty() && c.items[0].cBuild) { c.items[0].cBuild(v, h); return; }

    for (int i = 0; i < (int)c.items.size(); i++) {
        const MenuItem &it = c.items[i];
        float yTop = menuRowTop(c, i);
        bool hov = (h.item == i);
        switch (it.kind) {
        case MK_TOGGLE: {
            bool on = it.get && it.get() > 0.5f;
            uiToggle(v, rToggle(yTop), it.label, on, hov);
            break;
        }
        case MK_BUTTON:
            uiButton(v, rButton(yTop), it.label, hov);
            break;
        case MK_FADER: {
            float frac = it.get ? clampf((it.get() - it.vmin) / (it.vmax - it.vmin), 0.0f, 1.0f) : 0.0f;
            char lbl[40];
            if (it.valueText) { char vb[24]; it.valueText(vb, sizeof(vb));
                                snprintf(lbl, sizeof(lbl), "%s  %s", it.label, vb); }
            else snprintf(lbl, sizeof(lbl), "%s", it.label);
            uiTextC(v, lbl, 0.0f, yTop, 0.004f, kUiTitle[0], kUiTitle[1], kUiTitle[2]);
            uiHFader(v, rFader(yTop), frac, hov);
            break;
        }
        case MK_STEPPER: {
            uiTextC(v, it.label, 0.0f, yTop, 0.0045f, kUiTitle[0], kUiTitle[1], kUiTitle[2]);
            char vb[24];
            if (it.valueText) it.valueText(vb, sizeof(vb));
            else snprintf(vb, sizeof(vb), "%.2f", it.get ? it.get() : 0.0f);
            uiTextC(v, vb, 0.0f, yTop - 0.05f, 0.0052f, 1, 1, 1);
            uiButton(v, rStepMinus(yTop), "-", hov && h.part == 0);
            uiButton(v, rStepPlus(yTop),  "+", hov && h.part == 1);
            break;
        }
        case MK_DROPDOWN: {
            uiTextC(v, it.label, 0.0f, yTop, 0.0045f, kUiTitle[0], kUiTitle[1], kUiTitle[2]);
            int sel = it.get ? (int)(it.get() + 0.5f) : 0;
            const char *cur = (sel >= 0 && sel < (int)it.options.size()) ? it.options[sel] : "";
            uiDropdownHeader(v, rDropHdr(yTop), cur, it.dropOpen, hov && h.part == 0);
            if (it.dropOpen)
                for (int k = 0; k < (int)it.options.size(); k++)
                    uiDropdownItem(v, rDropItem(yTop, k), it.options[k], hov && h.part == 100 + k);
            break;
        }
        case MK_CUSTOM:
            if (it.cBuild) it.cBuild(v, h);
            break;
        }
    }
}

// ---- hit -------------------------------------------------------------------
MenuHover menuHit(const MenuCategory &c, float cx, float cy) {
    MenuHover h;
    if (c.custom && !c.items.empty() && c.items[0].cHit) { c.items[0].cHit(cx, cy, h); return h; }

    for (int i = 0; i < (int)c.items.size(); i++) {
        const MenuItem &it = c.items[i];
        float yTop = menuRowTop(c, i);
        switch (it.kind) {
        case MK_TOGGLE: if (uiHit(rToggle(yTop), cx, cy)) { h.item=i; h.part=0; h.grab=true; } break;
        case MK_BUTTON: if (uiHit(rButton(yTop), cx, cy)) { h.item=i; h.part=0; h.grab=true; } break;
        case MK_FADER:  if (uiHit(rFader(yTop),  cx, cy)) { h.item=i; h.part=0; h.grab=true; } break;
        case MK_STEPPER:
            if      (uiHit(rStepMinus(yTop), cx, cy)) { h.item=i; h.part=0; h.grab=true; }
            else if (uiHit(rStepPlus(yTop),  cx, cy)) { h.item=i; h.part=1; h.grab=true; }
            break;
        case MK_DROPDOWN:
            if (uiHit(rDropHdr(yTop), cx, cy)) { h.item=i; h.part=0; h.grab=true; }
            else if (it.dropOpen)
                for (int k = 0; k < (int)it.options.size(); k++)
                    if (uiHit(rDropItem(yTop, k), cx, cy)) { h.item=i; h.part=100+k; h.grab=true; break; }
            break;
        case MK_CUSTOM: if (it.cHit) it.cHit(cx, cy, h); break;
        }
        if (h.item >= 0) break;
    }
    return h;
}

// ---- apply -----------------------------------------------------------------
// Keyboard-style hold-to-repeat for steppers + fader drag/commit. State is keyed
// by (catId,item,part); only one control is ever active at a time.
void menuApply(int catId, MenuCategory &c, const MenuHover &h,
               bool click, bool grab, float cx, float cy) {
    static int sHoldKey = -1; static uint64_t sHoldNext = 0;   // stepper repeat
    static int sFadeKey = -1;                                  // active fader

    auto key = [&](int part){ return (catId << 16) | (h.item << 2) | part; };

    // Fader release -> commit/persist. Checked FIRST and unconditionally so a fader
    // that was being dragged still saves even if the pointer slid off it on release.
    if (!grab && sFadeKey >= 0) {
        int ci = (sFadeKey >> 2) & 0x3fff;
        if (ci >= 0 && ci < (int)c.items.size() && c.items[ci].onCommit) c.items[ci].onCommit();
        sFadeKey = -1;
    }

    if (c.custom && !c.items.empty() && c.items[0].cAct) { c.items[0].cAct(h, click, grab, cx, cy); return; }
    if (h.item < 0 || h.item >= (int)c.items.size()) { if (!grab) { sHoldKey = -1; } return; }
    MenuItem &it = c.items[h.item];

    switch (it.kind) {
    case MK_TOGGLE:
        if (click && it.get && it.set) {
            it.set(it.get() > 0.5f ? 0.0f : 1.0f);
            if (it.onChange) it.onChange();
        }
        break;
    case MK_BUTTON:
        if (click && it.onClick) it.onClick();
        break;
    case MK_STEPPER: {
        int k = key(h.part);
        bool onBtn = grab && h.grab;
        bool doStep = false;
        if (click) { doStep = true; sHoldKey = k; sHoldNext = nowNs() + 400000000ULL; }
        else if (onBtn && k == sHoldKey) {
            uint64_t now = nowNs();
            if (now >= sHoldNext) { doStep = true; sHoldNext = now + 80000000ULL; }
        }
        if (!onBtn) sHoldKey = -1;
        if (doStep && it.get && it.set) {
            float val = it.get() + (h.part == 1 ? it.vstep : -it.vstep);
            it.set(clampf(val, it.vmin, it.vmax));
            if (it.onChange) it.onChange();
        }
        break;
    }
    case MK_FADER: {
        int k = key(0);
        if (grab && it.set) {
            float frac = clampf((cx + kFaderW*0.5f) / kFaderW, 0.0f, 1.0f);
            it.set(it.vmin + frac * (it.vmax - it.vmin));
            if (it.onChange) it.onChange();
            sFadeKey = k;
        }
        break;
    }
    case MK_DROPDOWN:
        if (click) {
            if (h.part == 0) it.dropOpen = !it.dropOpen;
            else if (h.part >= 100) {
                if (it.set) it.set((float)(h.part - 100));
                it.dropOpen = false;
                if (it.onChange) it.onChange();
            }
        }
        break;
    case MK_CUSTOM: break;
    }
}

unsigned menuValueSig(const MenuCategory &c) {
    unsigned h = 2166136261u;
    auto mix = [&](long q){ for (int b=0;b<4;b++){ h=(h^(unsigned char)(q&0xff))*16777619u; q>>=8; } };
    for (const auto &it : c.items) {
        if (it.kind == MK_CUSTOM) { mix((long)(intptr_t)&it); continue; }
        if (it.get) mix((long)lroundf(it.get() * 2000.0f));
    }
    return h & 0x7fffffff;
}
