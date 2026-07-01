#pragma once
// Cross-cutting shared application state: the lobby/render knobs that the render
// loop, the lobby panels, and the JNI input handlers all touch. Atomics where
// the producer (Android UI thread / JNI) and consumer (render thread) differ.
// Owned here (defined in app_state.cpp); everyone else references via these
// extern handles.
#include <atomic>
#include <cstdint>

// ---- Software IPD (lobby-adjustable, persisted) ----------------------------
// World-scale knob: drives the per-eye optical-centre offset for the lobby
// render, the warp submit pose, AND the ALVR view_params. Stored in millimetres.
constexpr float kIpdMin = 58.0f, kIpdMax = 72.0f, kIpdStep = 0.5f;
extern std::atomic<float>    gSoftIpdMm;
extern std::atomic<bool>     gIpdDirty;     // value changed, needs persisting
extern std::atomic<uint64_t> gIpdChangeNs;  // last-change time (debounce save)
extern std::atomic<float>    gSentIpdMm;    // IPD last pushed to the server via view_params
inline float softIpdM() { return gSoftIpdMm.load() * 0.001f; }

// ---- lobby input edges (set by the JNI key handler, consumed by render loop) -
extern std::atomic<bool> gOkClick;   // OK/select pressed (edge)
extern std::atomic<bool> gOkHeld;    // OK/select held (gaze drag of the EQ)
extern std::atomic<bool> gSideHeld;  // headset SIDE button (keycode 1001) held

// Show the lobby mid-stream WITHOUT disconnecting.
extern std::atomic<bool> gManualLobby;

// ---- persisted toggles -----------------------------------------------------
extern std::atomic<bool> gEyeDebugOn;   // eye-gaze debug marker on/off
extern std::atomic<int>  gDiagHudMode;  // streaming diagnostics HUD: 0=off, 1=pipeline, 2=system (CPU/GPU/heat)
extern std::atomic<bool>  gThemeAmber;   // lobby UI theme: false=cold blue, true=amber terminal
extern std::atomic<float> gBrightnessFrac; // HMD panel brightness 0..1 (VIDEO tab slider)
extern std::atomic<bool>  gBrightnessSaved; // a brightness value was persisted (vs. panel default)

// ---- render-thread side-effect requests ------------------------------------
// The data-driven menu (menu_model) runs without access to the render thread's GL
// context, JNIEnv, or locomotion locals, so menu callbacks just raise these flags;
// the render loop polls + clears them and performs the actual effect.
extern std::atomic<bool> gGridThemeDirty;    // rebuild floor VBO (theme changed)
extern std::atomic<bool> gEyeTrackReapply;   // re-evaluate the Pico eye-tracking mode
extern std::atomic<bool> gBrightnessApply;   // push gBrightnessFrac to the HMD panel

// ---- streaming controller grip offset --------------------------------------
// The applied offset is a baked baseline (the constants below). It positions a
// FIXED point on the physical controller, expressed in its LOCAL
// frame (mm): +X = controller's right (mirrored on the right hand), +Y = up toward
// the buttons, +Z = back toward the wrist.
constexpr float kBaseGripSideMm = 0.0f, kBaseGripUpMm = 12.5f, kBaseGripBackMm = 40.0f;
constexpr float kBasePredict    = 0.4f, kBaseRotSwing = 1.0f;
constexpr float kBaseYawDeg     = 35.0f, kBaseRotXDeg = 10.0f, kBaseRotYDeg = -34.0f;

// ---- per-second diagnostics published for the HUD --------------------------
extern std::atomic<int> gVidDecoded, gVidSubmit, gVidDropped;
extern std::atomic<int> gGapMsX10, gRenderMsX10, gEncMsX10, gEnqMsX10;
// Per-second count of warp-submit fence waits that TIMED OUT (the slot wasn't
// GPU-complete within budget -> the warp may sample a partially-written texture =
// tear). 0 in healthy streaming; >0 flags a render overrun (e.g. thermal stall).
extern std::atomic<int> gFenceTimeouts;

// ---- persistence (single $HOME/config.txt) ---------------------------------
// loadAllConfig() restores every setting at boot; saveAllConfig() rewrites the
// whole file. The per-setting saveX() wrappers below just call saveAllConfig()
// so existing onChange call sites stay unchanged.
void loadAllConfig();
void saveAllConfig();
void saveSoftIpd();
void saveEyeDebug();
void saveDiagHud();
void saveTheme();
void saveBrightness();
