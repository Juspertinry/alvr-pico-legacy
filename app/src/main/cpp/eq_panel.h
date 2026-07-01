#pragma once
// 16-band audio EQ (lobby-adjustable, persisted). Tunes the streamed ALVR audio
// via the DSP in our client_core fork (eq.rs). Owns all EQ state + persistence +
// the panel geometry; the render-loop interaction code reads these by the same
// names (extern handles). Panel-local metres, +x right / +y up.
#include <vector>
#include <cstdint>

// ---- geometry / layout constants (compile-time; usable as array sizes) ----
constexpr int   kEqBands   = 16;
constexpr float kEqGainMax = 12.0f;   // +/- dB range of each fader
constexpr float kEqGeo  = 2.0f;       // fader geometry scale (text stays 1x)
constexpr float kEqText = 0.0045f;    // 1x text size on the EQ panel (menu, 50% smaller)
constexpr float kEqX0 = -0.235f*kEqGeo, kEqX1 = 0.235f*kEqGeo;   // panel left/right edge
constexpr float kEqYTrackTop = 0.06f, kEqYTrackBot = -0.34f;     // fader travel
constexpr float kEqTitleY    = 0.16f;
constexpr float kEqReadoutY  = -0.37f;
constexpr float kEqResetX0 = 0.28f, kEqResetX1 = 0.50f;          // RESET button
constexpr float kEqResetYTop = -0.345f, kEqResetYBot = -0.415f;
constexpr float kEqPresetX0 = -0.46f, kEqPresetX1 = 0.46f;       // preset dropdown
constexpr float kEqPresetYTop = -0.43f, kEqPresetYBot = -0.475f; // tucked right under RESET
constexpr float kEqItemH = 0.045f;    // dropdown item height (small: opening fits w/o scroll)
constexpr int   kEqNumPresets = 2;    // two user slots only: CUSTOM 1 / CUSTOM 2
extern const char *kEqPresetNames[kEqNumPresets];

// ---- state (render-thread owned; read by the interaction code) ----
extern float    gEqGains[kEqBands];        // current band gains (dB)
extern float    gEqCustoms[2][kEqBands];   // two PERSISTENT user curves
extern int      gEqActiveBand;             // band last painted while dragging (-1 = none)
extern bool     gEqGrabbing;               // a drag is in progress
extern bool     gEqDirty;                  // gains changed, needs persisting
extern uint64_t gEqChangeNs;               // last-change time (debounce save)
extern bool     gEqPresetOpen;             // preset dropdown expanded
extern int      gEqPresetIdx;              // active preset (0 = CUSTOM 1)
extern int      gDominantHand;             // laser hand (1=right default)

// ---- coordinate helpers ----
static inline float eqColCenterX(int b) {
    float colW = (kEqX1 - kEqX0) / kEqBands;
    return kEqX0 + colW * (b + 0.5f);
}
// Map a fader Y (panel-local) to gain dB and back.
static inline float eqYToGain(float ly) {
    float f = (ly - kEqYTrackBot) / (kEqYTrackTop - kEqYTrackBot);   // 0..1 bottom..top
    if (f < 0) f = 0; else if (f > 1) f = 1;
    return (f * 2.0f - 1.0f) * kEqGainMax;
}
static inline float eqGainToY(float g) {
    float f = (g / kEqGainMax + 1.0f) * 0.5f;
    if (f < 0) f = 0; else if (f > 1) f = 1;
    return kEqYTrackBot + f * (kEqYTrackTop - kEqYTrackBot);
}

void pushEqGains();                  // push the live gain set to the audio DSP
void applyEqPreset(int idx);         // switch active slot, load its saved curve
void saveEqProfile();                // persist EQ to the unified config.txt (via saveAllConfig)
// Build the EQ panel geometry (title + RESET + 16 faders + readout + dropdown).
void buildEqVerts(std::vector<float> &v, int hoverBand, bool resetHover,
                  bool headerHover, int presetHoverItem);
