#include "input.h"

std::mutex gCtrlMutex;
CtrlState  gCtrl[2];

std::mutex gHeadMutex;
float      gHeadData[7] = {0,0,0,1,0,0,0};

std::mutex    gHapticMutex;
PendingHaptic gHaptic[2];

void queueHaptic(int hand, float amplitude, float frequency, float durationS) {
    (void) frequency;   // Pico CV2 vibrate has no frequency arg; strength only
    if (hand < 0 || hand > 1) return;
    if (amplitude <= 0.0f || durationS <= 0.0f) return;   // empty pulse -> ignore
    if (amplitude > 1.0f) amplitude = 1.0f;
    // Clamp to a sane on-device window: floor very short pulses so the motor
    // actually fires, cap long ones so a stuck/looping event can't buzz forever.
    int ms = (int) (durationS * 1000.0f + 0.5f);
    if (ms < 12)   ms = 12;
    if (ms > 1000) ms = 1000;
    std::lock_guard<std::mutex> lk(gHapticMutex);
    PendingHaptic &p = gHaptic[hand];
    // Coalesce: keep the strongest pulse + its longer duration until drained.
    if (!p.pending || amplitude > p.amplitude) p.amplitude = amplitude;
    if (!p.pending || ms > p.durationMs)       p.durationMs = ms;
    p.pending = true;
}
