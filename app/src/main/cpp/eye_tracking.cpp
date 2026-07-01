#include "eye_tracking.h"
#include "pico_sdk.h"   // Pvr_GetEyeTrackingData / Pvr_Get/SetTrackingMode
#include "log.h"
#include "alvr_client_core.h"   // alvr_get_settings_json
#include "alvr_ext.h"           // alvr_get_settings_json_bounded (fork-only)
#include "app_state.h"          // gEyeDebugOn (lobby EYE DEBUG toggle)
#include <cmath>
#include <cstring>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>

std::atomic<bool>  gEyeOnline{false};
// Per-element nested braces (NOT `= {...}`): array aggregate init of atomics must
// direct-construct each element via atomic(float); the `=` copy-init form selects
// atomic's deleted copy ctor on libc++ and fails to compile.
std::atomic<float> gGazeLocal[3] = { {0.0f}, {0.0f}, {-1.0f} };
std::atomic<bool>  gGazeValid{false};
float gEyeOpen[2] = {1,1};
bool  gEyeHaveOpen = false;
float gEyeOpenSmooth[2] = {1,1};

std::atomic<bool> gEyeSupported{false};
bool gServerEyeEnabled = false;
static std::atomic<bool> gEyeIrOn{false};   // EYE bit currently set (server stream OR debug toggle)

// Pico tracking-mode bits (ROTATION=0x1, POSITION=0x2, EYE=0x4). The EYE bit gates
// the IR illuminators + the eye service feeding Pvr_GetEyeTrackingData.
static const int MODE_POSITION = 0x2, MODE_EYE = 0x4;

// Does the ALVR server actually consume gaze? Parse the (StreamingStarted-updated)
// settings JSON: headset.face_tracking is a Switch -> "Disabled" or {"Enabled":{
// "sources":{"eye_tracking_fb":true,...}}}. We only want the IR running when the
// eye source is on (the per-eye gaze ALVR forwards to VRChat eye-OSC / VRCFT).
// Lightweight string scan (matches foveation.cpp's approach; no JSON lib).
static bool parseServerEyeEnabled() {
    static char sj[65536];
    sj[0] = 0;
    alvr_get_settings_json_bounded(sj, sizeof(sj));   // bounded; NUL-terminated within cap
    const char *p = strstr(sj, "\"face_tracking\":");
    if (!p) return false;                       // key absent (e.g. pre-stream empty JSON)
    p += strlen("\"face_tracking\":");
    while (*p == ' ') p++;
    if (strncmp(p, "\"Disabled\"", 10) == 0) return false;   // face tracking off entirely
    // Enabled object -> the eye source flag lives just inside it.
    const char *e = strstr(p, "\"eye_tracking_fb\":");
    if (!e) return false;
    e += strlen("\"eye_tracking_fb\":");
    while (*e == ' ') e++;
    return strncmp(e, "true", 4) == 0;
}

void initEyeTrackingMode() {
    int supported = Pvr_GetTrackingMode();
    gEyeSupported.store((supported & MODE_EYE) != 0);
    // Boot with the illuminators OFF: POSITION-only. We light them up later, only
    // if a stream connects with the server's eye source enabled. This runs once at
    // render-thread init (before the first frame), so the synchronous SetTrackingMode
    // cost is harmless here -- unlike the interactive toggle, which we offload below.
    bool setRc = Pvr_SetTrackingMode(MODE_POSITION);
    // The eye-mode worker (eyeModeWorker) is a process-lifetime singleton --
    // started once via gEyeWorkerStarted and NEVER re-spawned, so on a nativeStop/
    // nativeStart it (and gEyeIrOn / gEyeOnline / gGazeValid) survive into the next
    // render-thread session. Force the eye state back to the POSITION-only boot here
    // so the FRESH tracking thread can't read eye data off a stale gEyeIrOn=true (or
    // draw a stale gaze marker) before applyServerEyeTracking re-evaluates on the
    // next STREAMING_STARTED. The persistent worker itself is fine to reuse: it just
    // blocks on its condvar between sessions and re-applies the mode on the next request.
    gEyeIrOn.store(false);
    gEyeOnline.store(false);
    gGazeValid.store(false);
    LOGI("eye: Pvr_GetTrackingMode supported=0x%x (EYE=%d); boot POSITION-only (IR off) set=%d",
         supported, gEyeSupported.load() ? 1 : 0, setRc ? 1 : 0);
}

// The actual mode switch. Pvr_SetTrackingMode (re-)arms the eye service / IR and
// BLOCKS for tens of ms, so this must never run on the render thread -- it's driven
// only from the worker below.
static void applyEyeModeNow(bool streaming) {
    gServerEyeEnabled = (streaming && gEyeSupported.load()) ? parseServerEyeEnabled() : false;
    bool debugWants = gEyeDebugOn.load();
    bool wantEye = gEyeSupported.load() && (gServerEyeEnabled || debugWants);
    int mode = MODE_POSITION | (wantEye ? MODE_EYE : 0);
    bool setRc = Pvr_SetTrackingMode(mode);
    gEyeIrOn.store(wantEye);
    if (!wantEye) { gEyeOnline.store(false); gGazeValid.store(false); }   // forget stale gaze when IR is off
    LOGI("eye: tracking mode -> %s (streaming=%d supported=%d serverEye=%d debug=%d) set=%d",
         wantEye ? "POSITION|EYE (IR on)" : "POSITION-only (IR off)",
         streaming ? 1 : 0, gEyeSupported.load() ? 1 : 0, gServerEyeEnabled ? 1 : 0,
         debugWants ? 1 : 0, setRc ? 1 : 0);
}

// Dedicated worker so the blocking Pvr_SetTrackingMode never stalls the render
// thread (toggling EYE DEBUG mid-frame caused a visible hitch). Requests coalesce:
// we only keep the LATEST desired `streaming` state, so rapid on/off toggles don't
// queue up a backlog of blocking calls.
static std::mutex              gEyeReqMtx;
static std::condition_variable gEyeReqCv;
static bool                    gEyeReqPending = false;
static bool                    gEyeReqStreaming = false;
static bool                    gEyeWorkerStarted = false;

static void eyeModeWorker() {
    for (;;) {
        bool streaming;
        {
            std::unique_lock<std::mutex> lk(gEyeReqMtx);
            gEyeReqCv.wait(lk, [] { return gEyeReqPending; });
            gEyeReqPending = false;
            streaming = gEyeReqStreaming;
        }
        applyEyeModeNow(streaming);   // blocking SetTrackingMode, off the render thread
    }
}

void applyServerEyeTracking(bool streaming) {
    std::lock_guard<std::mutex> lk(gEyeReqMtx);
    if (!gEyeWorkerStarted) {
        gEyeWorkerStarted = true;
        std::thread(eyeModeWorker).detach();   // lives for the process; never joined
    }
    gEyeReqStreaming = streaming;   // coalesce to the latest desired state
    gEyeReqPending = true;
    gEyeReqCv.notify_one();
}

// Read Neo 2 EYE eye tracking and build ALVR eye-gaze poses.
// out[0]=left, out[1]=right (matching ALVR's [left,right] eye_gazes ordering).
// Fills *vL/*vR per-eye validity. Returns true if the call was serviced at all.
// On non-Eye units Pvr_GetEyeTrackingData returns false -> we send no gaze.
// Position is left at the head origin for now (VRChat eye-OSC uses the gaze
// DIRECTION; gaze point/openness/pupil are read + logged but not yet forwarded,
// because the vanilla ALVR C API only consumes the gaze pose).
// gEyeOnline doubles as our "is this a Neo 2 EYE" check: only an Eye unit makes
// Pvr_GetEyeTrackingData service successfully, so it latches true exactly when
// real eye data first arrives. Everything eye-related (forwarding gaze to the
// server, the lobby debug circle) is gated on it -> non-Eye units never advertise
// eye tracking and never draw the marker.
bool readEyeGazes(AlvrPose out[2], bool *vL, bool *vR, int frame, Quat headQ) {
    *vL = *vR = false;
    // IR is off (server isn't consuming gaze AND debug viz is off) -> skip the SDK call.
    if (!gEyeIrOn.load()) return false;
    int ls=0, rs=0, cs=0;
    float lp[3]={0}, rp[3]={0}, cp[3]={0};
    float lv[3]={0}, rv[3]={0}, cv[3]={0};
    float lo=0, ro=0, lpd=0, rpd=0;
    float lg[3]={0}, rg[3]={0}, fg[3]={0}; int fgs=0;
    bool ok = Pvr_GetEyeTrackingData(&ls,&rs,&cs,
        &lp[0],&lp[1],&lp[2], &rp[0],&rp[1],&rp[2], &cp[0],&cp[1],&cp[2],
        &lv[0],&lv[1],&lv[2], &rv[0],&rv[1],&rv[2], &cv[0],&cv[1],&cv[2],
        &lo,&ro, &lpd,&rpd,
        &lg[0],&lg[1],&lg[2], &rg[0],&rg[1],&rg[2],
        &fg[0],&fg[1],&fg[2], &fgs);
    if (!ok) return false;
    // NOTE: do NOT negate Z. The SDK negates gaze-vector Z to convert to Unity's
    // (+Z forward) frame; ALVR/OpenXR eye_gazes want -Z forward, which is exactly
    // the NATIVE RAW convention: the raw combined/foveated vectors read ~(0,0,-1)
    // looking forward = OpenXR forward.
    // On the Neo 2 EYE the PER-EYE gaze vectors (lv/rv) are NOT
    // populated -- they read 0 or garbage. Only the COMBINED gaze vector (cv, status
    // cs) carries the real direction (the foveated dir fg matches it). So we drive
    // BOTH ALVR eyes from the combined gaze. Blinks: status L/R drop to 4 and cv->0;
    // gate on a sane unit-length combined vector so we don't snap the gaze on blink.
    // Capture openness for blink BEFORE the gaze-validity gate (a blink makes the
    // gaze invalid but is exactly when openness->0 matters). Any positive status
    // means a real Eye unit is responding (status=4 even mid-blink).
    if (ls > 0 || rs > 0 || cs > 0) {
        gEyeOpen[0] = lo; gEyeOpen[1] = ro; gEyeHaveOpen = true;
    }
    float cmag = sqrtf(cv[0]*cv[0] + cv[1]*cv[1] + cv[2]*cv[2]);
    bool valid = (cs > 0 && cmag > 0.5f && cmag < 2.0f);
    if (!valid) {
        gGazeValid.store(false);
        static bool warned = false;
        if (!warned) {
            warned = true;
            LOGI("eye service responds but no valid combined gaze (cs=%d |cv|=%.2f) -- "
                 "not forwarding gaze (non-EYE unit or no eyes detected)", cs, cmag);
        }
        return false;
    }
    if (!gEyeOnline.load()) {
        gEyeOnline.store(true);
        LOGI("EYE TRACKING ONLINE: status L=%d R=%d C=%d", ls, rs, cs);
    }
    // Drive both eyes from the combined gaze (per-eye unavailable on this HW).
    // Decompose the gaze direction into DECOUPLED spherical pitch/yaw that exactly
    // invert the server's eye-OSC conversion (face.rs: orientation.to_euler(XYZ) ->
    // pitch=X, yaw=Y). A shortest-arc forward->gaze quaternion cross-couples
    // pitch & yaw when the server decomposes it into XYZ Euler (accurate on the lobby
    // green-dot plane, which uses cv directly, but skewed in game, with no flat gain
    // able to fix the coupling). Build q = Rx(pitch)*Ry(yaw) instead: that round-trips
    // through to_euler(XYZ) as exactly (pitch, yaw, 0).
    // Forward is -Z; for q=Rx(P)*Ry(Y): dir = (-sinY, cosY*sinP, -cosY*cosP).
    float gx = cv[0], gy = cv[1], gz = cv[2];
    float gn = sqrtf(gx*gx + gy*gy + gz*gz);
    if (gn > 1e-6f) { gx /= gn; gy /= gn; gz /= gn; }
    float yaw   = atan2f(-gx, sqrtf(gy*gy + gz*gz));
    float pitch = atan2f(gy, -gz);
    // Debug visualizer uses the RAW Tobii gaze vector directly.
    gGazeLocal[0].store(cv[0]); gGazeLocal[1].store(cv[1]); gGazeLocal[2].store(cv[2]);
    Quat qP = { sinf(pitch * 0.5f), 0.0f, 0.0f, cosf(pitch * 0.5f) };
    Quat qY = { 0.0f, sinf(yaw * 0.5f), 0.0f, cosf(yaw * 0.5f) };
    Quat q = quatNorm(quatMul(qP, qY));   // head-LOCAL gaze (for the debug marker via cv)
    // ALVR eye_gazes are poses in the GLOBAL tracking space (same frame as the head
    // pose), NOT head-relative. Sending head-local gaze makes it world-locked -> turn
    // 180 and the eyes aim behind you. Compose with the head orientation so the gaze
    // is global: q_global = headQ * q_local.
    q = quatNorm(quatMul(headQ, q));
    out[0] = {}; out[1] = {};
    out[0].orientation = { q.x, q.y, q.z, q.w };
    out[1].orientation = { q.x, q.y, q.z, q.w };
    *vL = *vR = true;
    gGazeValid.store(true);
    if ((frame % 60) == 0) {
        LOGI("eye: yaw=%.1f pitch=%.1f deg Cv=(%.3f,%.3f,%.3f) open=(%.2f,%.2f) status=(%d,%d,%d)",
             yaw*57.2957795f, pitch*57.2957795f, cv[0],cv[1],cv[2], lo,ro, ls,rs,cs);
    }
    return true;
}
