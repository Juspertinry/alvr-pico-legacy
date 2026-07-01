#include "ui_kit.h"
#include <cstdint>
#include <cstring>
#include <cstdio>

// ---------------- 3D HUD text (lobby IP + status) ------------------------
// Minimal 5x7 bitmap font, row-major: 7 bytes/glyph, low 5 bits, bit4 = leftmost
// column, byte[0] = top row. Covers space, 0-9, '.', ':', '-', A-Z (enough for an
// IPv4 address + status words). Rendered as filled quads (one per lit pixel).
static const uint8_t kFont[][7] = {
    {0,0,0,0,0,0,0},                                   // ' '
    {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E},              // 0
    {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E},              // 1
    {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F},              // 2
    {0x1F,0x02,0x04,0x02,0x01,0x11,0x0E},              // 3
    {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02},              // 4
    {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E},              // 5
    {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E},              // 6
    {0x1F,0x01,0x02,0x04,0x08,0x08,0x08},              // 7
    {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E},              // 8
    {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C},              // 9
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C},              // '.'
    {0x00,0x0C,0x0C,0x00,0x0C,0x0C,0x00},              // ':'
    {0x00,0x00,0x00,0x1F,0x00,0x00,0x00},              // '-'
    {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11},              // A
    {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E},              // B
    {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E},              // C
    {0x1C,0x12,0x11,0x11,0x11,0x12,0x1C},              // D
    {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F},              // E
    {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10},              // F
    {0x0E,0x11,0x10,0x17,0x11,0x11,0x0F},              // G
    {0x11,0x11,0x11,0x1F,0x11,0x11,0x11},              // H
    {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E},              // I
    {0x07,0x02,0x02,0x02,0x02,0x12,0x0C},              // J
    {0x11,0x12,0x14,0x18,0x14,0x12,0x11},              // K
    {0x10,0x10,0x10,0x10,0x10,0x10,0x1F},              // L
    {0x11,0x1B,0x15,0x15,0x11,0x11,0x11},              // M
    {0x11,0x11,0x19,0x15,0x13,0x11,0x11},              // N
    {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E},              // O
    {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10},              // P
    {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D},              // Q
    {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11},              // R
    {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E},              // S
    {0x1F,0x04,0x04,0x04,0x04,0x04,0x04},              // T
    {0x11,0x11,0x11,0x11,0x11,0x11,0x0E},              // U
    {0x11,0x11,0x11,0x11,0x11,0x0A,0x04},              // V
    {0x11,0x11,0x11,0x15,0x15,0x1B,0x11},              // W
    {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11},              // X
    {0x11,0x11,0x0A,0x04,0x04,0x04,0x04},              // Y
    {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F},              // Z
    {0x00,0x04,0x04,0x0E,0x0E,0x1F,0x00},              // '^' = UP triangle   (index 40)
    {0x00,0x1F,0x0E,0x0E,0x04,0x04,0x00},              // '~' = DOWN triangle (index 41)
    {0x00,0x04,0x04,0x1F,0x04,0x04,0x00},              // '+' = plus          (index 42)
};
static int glyphIndex(char c) {
    if (c == ' ') return 0;
    if (c >= '0' && c <= '9') return 1 + (c - '0');
    if (c == '.') return 11;
    if (c == ':') return 12;
    if (c == '-') return 13;
    if (c == '^') return 40;   // up triangle
    if (c == '~') return 41;   // down triangle
    if (c == '+') return 42;   // plus
    if (c >= 'a' && c <= 'z') c = (char)(c - 32);
    if (c >= 'A' && c <= 'Z') return 14 + (c - 'A');
    return 0;   // unknown -> space
}

void appendTextLine(std::vector<float> &v, const char *s, float yTop,
                    float px, float r, float g, float b) {
    int n = (int) strlen(s);
    float lineW = (n * 6 - 1) * px;     // 5 px glyph + 1 px gap
    float x0 = -lineW * 0.5f;
    for (int i = 0; i < n; i++) {
        const uint8_t *gl = kFont[glyphIndex(s[i])];
        float cx = x0 + i * 6 * px;
        for (int row = 0; row < 7; row++) {
            for (int col = 0; col < 5; col++) {
                if (!(gl[row] & (1 << (4 - col)))) continue;
                float x = cx + col * px, y = yTop - row * px;
                float x1 = x + px, y1 = y - px;
                float q[6][6] = {
                    {x,y,0, r,g,b}, {x,y1,0, r,g,b}, {x1,y1,0, r,g,b},
                    {x,y,0, r,g,b}, {x1,y1,0, r,g,b}, {x1,y,0, r,g,b},
                };
                v.insert(v.end(), &q[0][0], &q[0][0] + 36);
            }
        }
    }
}

// Append a filled rectangle (two triangles) in the XY plane (z=0), top-left
// (xL,yTop) to bottom-right (xR,yBot). Same pos.xyz+rgb format as appendTextLine.
void appendQuad(std::vector<float> &v, float xL, float yTop, float xR, float yBot,
                float r, float g, float b) {
    float q[6][6] = {
        {xL,yTop,0, r,g,b}, {xL,yBot,0, r,g,b}, {xR,yBot,0, r,g,b},
        {xL,yTop,0, r,g,b}, {xR,yBot,0, r,g,b}, {xR,yTop,0, r,g,b},
    };
    v.insert(v.end(), &q[0][0], &q[0][0] + 36);
}

// ============================================================================
//  STANDARDIZED LOBBY UI KIT  (immediate-mode, panel-local metres, +x/+y)
//  Every new panel should build from these so the look stays uniform. Each widget
//  emits geometry into a vertex vector (pos.xyz+rgb); the caller hit-tests with
//  uiHit() against the SAME UiRect it passed to draw, then acts on click/drag.
//  Visual language = the EQ fader: dark slate boxes, cyan accent, green = on.
// ============================================================================
const float kUiText      = 0.0045f;            // standard 1x text size (menu widgets)
const float kUiBg[3]     = {0.16f, 0.16f, 0.20f};
const float kUiBgHot[3]  = {0.32f, 0.32f, 0.40f};
const float kUiTrack[3]  = {0.16f, 0.16f, 0.20f};
float       kUiFill[3]   = {0.30f, 0.80f, 1.00f};   // accent (themeable)
const float kUiOn[3]     = {0.25f, 0.95f, 0.40f};   // green = on
const float kUiOff[3]    = {0.30f, 0.30f, 0.36f};
const float kUiWhite[3]  = {1.0f, 1.0f, 1.0f};
float       kUiTitle[3]  = {0.70f, 0.90f, 1.00f};   // title/label (themeable)

void applyUiTheme(bool amber) {
    if (amber) {
        // 1970s amber terminal: blue accent -> orange, light-blue title -> cream.
        kUiFill[0]  = 0.95f; kUiFill[1]  = 0.72f; kUiFill[2]  = 0.10f;   // yellow-amber (less red)
        kUiTitle[0] = 0.98f; kUiTitle[1] = 0.92f; kUiTitle[2] = 0.70f;   // warm cream
    } else {
        kUiFill[0]  = 0.30f; kUiFill[1]  = 0.80f; kUiFill[2]  = 1.00f;   // cyan
        kUiTitle[0] = 0.70f; kUiTitle[1] = 0.90f; kUiTitle[2] = 1.00f;   // light blue
    }
}

void uiBox(std::vector<float> &v, const UiRect &r, const float c[3]) {
    appendQuad(v, r.cx-r.w*0.5f, r.cy+r.h*0.5f, r.cx+r.w*0.5f, r.cy-r.h*0.5f, c[0],c[1],c[2]);
}
// Text centred horizontally at cx (appendTextLine centres on x=0, so we shift).
void uiTextC(std::vector<float> &v, const char *s, float cx, float yTop, float px,
             float r, float g, float b) {
    std::vector<float> t; appendTextLine(t, s, yTop, px, r, g, b);
    for (size_t i = 0; i < t.size(); i += 6) t[i] += cx;
    v.insert(v.end(), t.begin(), t.end());
}
// Left-aligned text starting at xLeft.
void uiTextL(std::vector<float> &v, const char *s, float xLeft, float yTop, float px,
             float r, float g, float b) {
    int n = (int) strlen(s);
    float lineW = (n * 6 - 1) * px;
    uiTextC(v, s, xLeft + lineW * 0.5f, yTop, px, r, g, b);
}
// Plain label (centred), baseline auto from rect centre.
void uiLabel(std::vector<float> &v, const char *s, float cx, float cy, float px,
             const float col[3]) {
    uiTextC(v, s, cx, cy + 3.5f*px, px, col[0], col[1], col[2]);
}
// BUTTON: filled box + centred label.
void uiButton(std::vector<float> &v, const UiRect &r, const char *label, bool hot) {
    uiBox(v, r, hot ? kUiBgHot : kUiBg);
    uiTextC(v, label, r.cx, r.cy + 3.5f*kUiText, kUiText, 1,1,1);
}
// TOGGLE SWITCH: row box + left label + sliding pill (green on / grey off) at right.
void uiToggle(std::vector<float> &v, const UiRect &r, const char *label, bool on, bool hot, float textScale) {
    uiBox(v, r, hot ? kUiBgHot : kUiBg);
    float sw = 0.11f, sh = r.h * 0.60f;
    float tpx = kUiText * textScale;
    // Auto-shrink the label so it never clips under the pill: cap the font size to
    // the room between the label's left edge and the pill's left edge. Glyph advance
    // is 6 px (5 + 1 gap) and the last glyph drops the trailing gap -> width =
    // (n*6 - 1)*px. Only ever shrinks; short labels keep their requested size.
    int n = (int) strlen(label);
    if (n > 0) {
        float labelLeft = r.cx - r.w*0.5f + 0.012f;
        float pillLeft  = r.cx + r.w*0.5f - sw - 0.012f;
        float avail = pillLeft - labelLeft - 0.010f;     // small gap before the pill
        if (avail > 0.0f) { float fit = avail / (n*6 - 1); if (fit < tpx) tpx = fit; }
    }
    uiTextL(v, label, r.cx - r.w*0.5f + 0.012f, r.cy + 3.5f*tpx, tpx, 0.85f,0.88f,0.92f);
    float scx = r.cx + r.w*0.5f - sw*0.5f - 0.012f;
    UiRect track = { scx, r.cy, sw, sh };
    uiBox(v, track, on ? kUiOn : kUiOff);
    float kx = on ? (scx + sw*0.22f) : (scx - sw*0.22f);
    UiRect knob = { kx, r.cy, sw*0.42f, sh*0.82f };
    uiBox(v, knob, kUiWhite);
}
// VERTICAL FADER: track + cyan fill from bottom + white knob bar. frac 0..1.
void uiVFader(std::vector<float> &v, const UiRect &r, float frac, bool hot) {
    if (frac < 0) frac = 0; else if (frac > 1) frac = 1;
    float th = r.w * 0.30f;
    appendQuad(v, r.cx-th, r.cy+r.h*0.5f, r.cx+th, r.cy-r.h*0.5f, hot?0.30f:kUiTrack[0], hot?0.30f:kUiTrack[1], hot?0.36f:kUiTrack[2]);
    float yb = r.cy - r.h*0.5f, ky = yb + frac*r.h;
    appendQuad(v, r.cx-th, ky, r.cx+th, yb, kUiFill[0],kUiFill[1],kUiFill[2]);
    appendQuad(v, r.cx-r.w*0.5f, ky+0.012f, r.cx+r.w*0.5f, ky-0.012f, 1,1,1);
}
// HORIZONTAL SLIDER: track + cyan fill from left + white knob bar. frac 0..1.
void uiHFader(std::vector<float> &v, const UiRect &r, float frac, bool hot) {
    if (frac < 0) frac = 0; else if (frac > 1) frac = 1;
    float th = r.h * 0.30f;
    appendQuad(v, r.cx-r.w*0.5f, r.cy+th, r.cx+r.w*0.5f, r.cy-th, hot?0.30f:kUiTrack[0], hot?0.30f:kUiTrack[1], hot?0.36f:kUiTrack[2]);
    float xl = r.cx - r.w*0.5f, kx = xl + frac*r.w;
    appendQuad(v, xl, r.cy+th, kx, r.cy-th, kUiFill[0],kUiFill[1],kUiFill[2]);
    appendQuad(v, kx-0.012f, r.cy+r.h*0.5f, kx+0.012f, r.cy-r.h*0.5f, 1,1,1);
}
// DROPDOWN header: box + label + up/down arrow glyph (open/closed).
void uiDropdownHeader(std::vector<float> &v, const UiRect &r, const char *label, bool open, bool hot) {
    uiBox(v, r, hot ? kUiBgHot : kUiBg);
    char buf[48]; snprintf(buf, sizeof(buf), "%s %s", label, open ? "^" : "~");
    uiTextC(v, buf, r.cx, r.cy + 3.5f*kUiText, kUiText, 1,1,1);
}
void uiDropdownItem(std::vector<float> &v, const UiRect &r, const char *label, bool hot) {
    uiBox(v, r, hot ? kUiBgHot : kUiOff);
    uiTextC(v, label, r.cx, r.cy + 3.5f*kUiText, kUiText, 1,1,1);
}
