#pragma once
// Neo 2 EYE eye-tracking: read the Pico eye service and build ALVR eye-gaze
// poses. Owns the gaze/openness state; the render loop + lobby read it via
// these extern handles.
#include "alvr_client_core.h"   // AlvrPose
#include "math3d.h"             // Quat
#include <atomic>

// Atomic: written on the tracking thread (readEyeGazes) AND the eye-mode worker
// thread (applyEyeModeNow clears them when the IR turns off), but read on the
// RENDER thread (the lobby gaze marker + its dirty signature). Plain globals here
// were a cross-thread data race; the marker is cosmetic so per-field atomics
// (lock-free, possible inter-field tear is irrelevant) are enough.
// gEyeOnline doubles as the "is this a Neo 2 EYE" latch: only an Eye unit makes
// Pvr_GetEyeTrackingData service a valid combined gaze, so it flips true exactly
// when real eye data first arrives. All eye-gated behaviour keys off it.
extern std::atomic<bool>  gEyeOnline;
// Latest COMBINED gaze direction in head-local space + validity (lobby marker).
extern std::atomic<float> gGazeLocal[3];
extern std::atomic<bool>  gGazeValid;
// Per-eye openness (0=closed..1=open) for blink forwarding, captured even mid-
// blink (when gaze is invalid). Tracking-thread-only (write + read), so plain.
extern float gEyeOpen[2];
extern bool  gEyeHaveOpen;
// Smoothed openness for forwarding (the raw Tobii openness is ~quantized 0/1).
extern float gEyeOpenSmooth[2];

// Read Neo 2 EYE tracking and build ALVR eye-gaze poses. out[0]=left, out[1]=
// right; fills *vL/*vR per-eye validity. Returns true if serviced (false on
// non-Eye units / no valid gaze). headQ = current head orientation (for the
// global-space gaze compose).
bool readEyeGazes(AlvrPose out[2], bool *vL, bool *vR, int frame, Quat headQ);

// ---- server-driven eye-tracking power gating ----
// The Neo 2 EYE's IR illuminators only burn power while the EYE tracking-mode bit
// is set. There's no point running them (and heating the SoC) if the ALVR server
// isn't consuming gaze, so we keep EYE OFF unless the server's Face Tracking eye
// source is actually enabled.
extern std::atomic<bool> gEyeSupported;  // EYE bit present in Pvr_GetTrackingMode() (set once at render init, read on the eye-mode worker)
extern bool gServerEyeEnabled;   // server Face Tracking eye source on (eye-worker-only; plain)
// Boot: detect EYE support but leave the illuminators OFF (POSITION-only mode).
void initEyeTrackingMode();
// (Re)evaluate the Pico tracking mode: enable the EYE bit only if the unit
// supports it AND `streaming` AND the server wants gaze; otherwise POSITION-only
// so the IR stops. Call on STREAMING_STARTED (after settings JSON is live) and on
// STREAMING_STOPPED. Reads the server flag itself when streaming.
void applyServerEyeTracking(bool streaming);
