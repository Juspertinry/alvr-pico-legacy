#pragma once
// Standardized immediate-mode lobby UI kit + 5x7 bitmap font. Pure geometry
// emitters: each widget appends pos.xyz+rgb triangles into a vertex vector
// (panel-local metres, +x right / +y up). No GL, no app state. The caller draws
// the vector and hit-tests with uiHit() against the SAME UiRect it passed.
#include <vector>

// ---- text ----------------------------------------------------------------
// Append one horizontally-centred line of text as filled quads. px = metres per
// font pixel; yTop = baseline of the top row; colour r,g,b.
void appendTextLine(std::vector<float> &v, const char *s, float yTop,
                    float px, float r, float g, float b);
// Append a filled rectangle (two triangles, z=0), top-left (xL,yTop) to
// bottom-right (xR,yBot).
void appendQuad(std::vector<float> &v, float xL, float yTop, float xR, float yBot,
                float r, float g, float b);

// ---- widget kit ----------------------------------------------------------
struct UiRect { float cx, cy, w, h; };   // centre + full size (panel-local metres)
static inline bool uiHit(const UiRect &r, float lx, float ly) {
    return lx >= r.cx - r.w*0.5f && lx <= r.cx + r.w*0.5f &&
           ly >= r.cy - r.h*0.5f && ly <= r.cy + r.h*0.5f;
}

// Shared visual language (defined in ui_kit.cpp).
extern const float kUiText;        // standard 1x text size
extern const float kUiBg[3];
extern const float kUiBgHot[3];
extern const float kUiTrack[3];
extern float       kUiFill[3];     // accent (themeable: cyan <-> amber-orange)
extern const float kUiOn[3];       // green = on
extern const float kUiOff[3];
extern const float kUiWhite[3];
extern float       kUiTitle[3];    // title/label text (themeable: light blue <-> cream)

// Apply the lobby UI theme: false = cold blue (default), true = amber terminal.
// Recolours kUiFill (accent) + kUiTitle. White stays white. Call on startup and
// whenever the theme toggles.
void applyUiTheme(bool amber);

void uiBox(std::vector<float> &v, const UiRect &r, const float c[3]);
void uiTextC(std::vector<float> &v, const char *s, float cx, float yTop, float px,
             float r, float g, float b);
void uiTextL(std::vector<float> &v, const char *s, float xLeft, float yTop, float px,
             float r, float g, float b);
void uiLabel(std::vector<float> &v, const char *s, float cx, float cy, float px,
             const float col[3]);
void uiButton(std::vector<float> &v, const UiRect &r, const char *label, bool hot);
void uiToggle(std::vector<float> &v, const UiRect &r, const char *label, bool on, bool hot, float textScale = 1.0f);
void uiVFader(std::vector<float> &v, const UiRect &r, float frac, bool hot);
void uiHFader(std::vector<float> &v, const UiRect &r, float frac, bool hot);
void uiDropdownHeader(std::vector<float> &v, const UiRect &r, const char *label, bool open, bool hot);
void uiDropdownItem(std::vector<float> &v, const UiRect &r, const char *label, bool hot);
