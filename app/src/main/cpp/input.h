#pragma once
// Controller + head-pose state shared between the JNI input sinks (which the
// Java ControllerClient poller pushes into) and the render thread (which reads
// the latest snapshot to build the ALVR uplink). Owned here; referenced via
// these extern handles.
#include <mutex>

// ---------- Pico controllers (data pushed from Java ControllerClient) -------
// The Java side binds the CV controller service and polls per-hand state, then
// pushes it here via nativeControllerState(). The render thread reads the latest
// snapshot under gCtrlMutex to build the ALVR uplink. hand: 0=left, 1=right.
struct CtrlState {
    int   conn = 0;                 // 1 = connected
    float q[4] = {0,0,0,1};         // orientation (Pico frame, raw)
    float pos[3] = {0,0,0};         // position (Pico frame, raw)
    float angVel[3] = {0,0,0};
    int   keys[16] = {0};
    int   keyCount = 0;
    bool  fresh = false;            // got at least one update
};
extern std::mutex gCtrlMutex;
extern CtrlState  gCtrl[2];

// Latest head pose (raw Pvr_GetMainSensorState frame: qx,qy,qz,qw,px,py,pz),
// published by the render thread for the Java controller poller to feed into the
// CV service's head-aligned getControllerSensorState(hand, headData) overload --
// this is how the legacy Pico SDK keeps controllers locked to the head frame.
extern std::mutex gHeadMutex;
extern float      gHeadData[7];

// ---------- haptics (server -> controller rumble) ---------------------------
// ALVR delivers HAPTICS events on the render thread (device_id + duration/freq/
// amplitude). The Pico CV2 wand is driven only from Java (ControllerClient), so
// the render thread parks the latest request here and the Java controller poller
// drains it each tick and calls vibrateCV2ControllerStrength(). hand 0=L/1=R.
// We coalesce (keep the strongest pending pulse) rather than queue: between two
// ~11ms Java polls at most one rumble matters, and an unbounded queue would lag
// behind the action. amplitude 0..1, durationMs already clamped.
struct PendingHaptic {
    float amplitude  = 0.0f;
    int   durationMs = 0;
    bool  pending    = false;
};
extern std::mutex     gHapticMutex;
extern PendingHaptic  gHaptic[2];

// Called from the render thread's ALVR event loop. Maps the raw event fields to
// the Pico vibrate params (clamps duration, drops empty pulses) and coalesces.
void queueHaptic(int hand, float amplitude, float frequency, float durationS);
