#pragma once
// Prototypes for the prebuilt Pico (legacy Neo 2) native SDK we link against
// (libPvr_UnitySDK.so). Most are reverse-engineered ABI contracts -- see the
// per-function notes. This header is the single place that documents them.

extern "C" {
    int   Pvr_Init(int index);
    int   Pvr_StartSensor(int index);
    int   Pvr_Enable6DofModule(bool enable);
    int   InitSensor();   // sets the sensor-TYPE selector (svr/falcon/usb...) from config;
                          // without it Pvr_GetMainSensorState returns identity.
    void  Pvr_SetInitActivity(void *activity, void *vrActivityClass);
    int   Pvr_GetMainSensorState(float *x, float *y, float *z, float *w,
                                 float *px, float *py, float *pz,
                                 float *vfov, float *hfov, int *viewNumber);
    // Controller tracking (Neo 2: 3DoF controllers + arm model). RE'd from
    // libPvr_UnitySDK: fills outQuat[4] (xyzw) + outPos[3]. No hand index in this
    // export -- returns the SDK's "current" controller. Data is fed in via the
    // Java controller service (setControllerData); if that module is broken the
    // values stay idle/zero.
    void  Pvr_GetControllerTrackingData(float *outQuat, float *outPos);
    void  Pvr_GetPointerPose(float *outQuat, float *outPos);  // arm-model pointer
    // Eye tracking (Neo 2 EYE only; returns false / status<=0 on non-Eye units).
    // Exposes far more than gaze: per-eye + combined gaze POINT and gaze VECTOR,
    // eye OPENNESS, PUPIL DILATION, eye POSITION GUIDE, and foveated gaze. All out
    // params by pointer; bool return = call serviced. Signature mirrors the C#
    // Pvr_GetEyeTrackingData exactly (35 out pointers). We currently forward only
    // the per-eye gaze vector (the one thing the vanilla ALVR C API consumes via
    // alvr_send_tracking's eye_gazes); the rest is read for logging only.
    bool  Pvr_GetEyeTrackingData(
        int *lStatus, int *rStatus, int *cStatus,
        float *lPx, float *lPy, float *lPz,   // left gaze point
        float *rPx, float *rPy, float *rPz,   // right gaze point
        float *cPx, float *cPy, float *cPz,   // combined gaze point
        float *lVx, float *lVy, float *lVz,   // left gaze vector (direction)
        float *rVx, float *rVy, float *rVz,   // right gaze vector
        float *cVx, float *cVy, float *cVz,   // combined gaze vector
        float *lOpen, float *rOpen,           // eyelid openness
        float *lPupil, float *rPupil,         // pupil dilation
        float *lGuideX, float *lGuideY, float *lGuideZ,   // eye position guide
        float *rGuideX, float *rGuideY, float *rGuideZ,
        float *fovGazeX, float *fovGazeY, float *fovGazeZ, // foveated gaze dir
        int *fovGazeState);
    bool  Pvr_GetEyeTrackingAutoIPD(float *autoIPD);   // measured IPD (meters)
    // Tracking-mode bitmask: ROTATION=0x1, POSITION=0x2, EYE=0x4. Pvr_GetTrackingMode
    // returns the SUPPORTED modes (EYE bit set only on a Neo 2 EYE); Pvr_SetTrackingMode
    // ENABLES the requested modes. Eye tracking is OFF until we set POSITION|EYE -- this
    // is what makes Pvr_GetEyeTrackingData actually return data (mirrors the SDK's
    // Pvr_UnitySDKEyeManager.SetEyeTrackingMode on enter-VR).
    bool  Pvr_SetTrackingMode(int trackingMode);
    int   Pvr_GetTrackingMode();
    void  Pvr_DisableBoundary();
    void  Pvr_ShutdownSDKBoundary();
    // Tracking origin: 0=EyeLevel (py~0 at head), 1=FloorLevel (py = height above
    // floor, includes the device's floor calibration), 2=StageLevel. SteamVR/ALVR
    // expect a FLOOR origin (standing universe); without this the head sits at y=0
    // and you spawn inside the floor. Mirrors C# Pvr_SetTrackingOriginType.
    bool  Pvr_SetTrackingOriginType(int trackingOriginType);
    float Pvr_GetFloorHeight();   // device floor calibration height (diagnostic)
    // Play-area extents in meters: x=width, z=depth (y unused). isPlayArea=true
    // returns the inner play area, false the outer boundary. Must be read BEFORE
    // Pvr_ShutdownSDKBoundary(). Returns count of geometry points (0 if unset).
    int   Pvr_BoundaryGetDimensions(float *x, float *y, float *z, bool isPlayArea);
    bool  Pvr_BoundaryGetConfigured();
    void *GetRenderEventFunc();
    bool  Pvr_SetSinglePassDepthBufferWidthHeight(int width, int height);
    void  Pvr_GetFOV(float *outA, float *outB);
    float Pvr_GetIPD();   // device IPD in METERS (Neo 2: fixed ~0.065)
    // --- HW compositor (DIATW) eye-buffer submit path ----------------------
    // Feed our rendered eye textures to the SDK warp thread, which does HW lens
    // distortion + async reprojection + direct low-latency present (home-shell
    // path), instead of self-presenting through SurfaceFlinger.
    void  Pvr_SetCurrentRenderTexture(unsigned int texId);   // stores current eye tex
    void  PvrBeginEyeEvent();                                  // register current tex into warp ring
    void  PvrEndEyeEvent();                                    // finalize eye
    void  PVR_CameraEndFrame(unsigned int eye, unsigned int texId);  // store eye tex slot
    // pose block: 88 bytes (22 floats) passed BY VALUE starting in r2 (eye=r0,
    // pad=r1). Copied into ctx+eye*0x128+0x8ce0, then into the warp source by
    // PVR_TimeWarpEvent. SelectRT validates a unit quaternion in it.
    struct PvrPoseBlk { float v[22]; };
    void  PVR_ChangeRenderPose(unsigned int eye, unsigned int pad, struct PvrPoseBlk blk);
    void  Pvr_SetAsyncTimeWarp(unsigned char enable);
    // The per-frame submit: reads eye textures (CameraEndFrame slots) + pose
    // (ChangeRenderPose) -> builds warp source -> pvr_WarpSwap into the ring.
    // The async warp thread then composites (HW distortion + reproject) + presents.
    void  PVR_TimeWarpEvent(unsigned int eye);
    // SDK data globals: the per-eye texture FOV the distortion mesh expects, and
    // the eye-buffer FOV in X/Y. Populated at Pvr_Init. We render the ALVR video
    // at this FOV so the content matches the lens/distortion.
    extern float fEyeTextureFov0;
    extern float fEyeTextureFov1;
    extern float gEyeBufferFovX;
    extern float gEyeBufferFovY;
}

// ---- CPU/GPU performance-level pinning (Qualcomm VR perf service) ----------
// The same clock-floor lever the VR-home Power Profile drives.
// SetCpuLevel/SetGpuLevel push a perf LEVEL (higher = higher
// floor) + a sustained/static flag into the QVR perf service; they return >=0 when
// applied, -1 if the service client isn't bound yet (so retry). Get* read it back.
// These are C++-mangled (NOT extern "C"): declaring them here as plain free
// functions reproduces _Z11SetCpuLevelib / _Z11SetGpuLevelib / _Z11GetCpuLevelRiRb
// / _Z11GetGpuLevelRiRb, which are exported T symbols in libPvr_UnitySDK.so.
int  SetCpuLevel(int level, bool sustained);
int  SetGpuLevel(int level, bool sustained);
void GetCpuLevel(int &level, bool &sustained);
void GetGpuLevel(int &level, bool &sustained);

// ---- HMD panel backlight (the lobby brightness slider + off-head auto-dim) ---
// Raw panel backlight level (0..255 on the Neo 2 LCD). These are C++-mangled
// exported T symbols in libPvr_UnitySDK.so -- declaring them as plain free
// functions reproduces _Z22SetHmdScreenBrightnessi and _Z22GetHmdScreenBrightnessRi
// (Set takes the level by value; Get writes the current level into the int&). If
// the SDK path doesn't take we fall back to the Android window-brightness API.
void SetHmdScreenBrightness(int brightness);
void GetHmdScreenBrightness(int &brightness);
constexpr int kBrightMin = 8, kBrightMax = 255;   // slider floor (avoid full black) .. panel max

// Vsync phase oracle (C++ symbol _ZN3PVR18GetFractionalVsyncEv). Returns the
// fractional vsync number: integer part = vsync count, fractional part =
// progress through the CURRENT refresh interval (computed live from
// GetTicksNanos vs the last vsync timestamp / interval). Used to phase-lock our
// drain+submit to a fixed late phase of each interval so the warp always gets
// the freshest decoded frame at a consistent content-age (kills the steady beat
// from random-phase draining against the panel scanout).
namespace PVR { double GetFractionalVsync(); }

// Internal Pico config accessors (the C++-mangled symbols, NOT the remapped
// public Pvr_Get*Config which only expose ~7 presets). These read arbitrary
// DECRYPTED config indices the SDK loaded at Pvr_Init -- including the
// per-device lens distortion polynomial + chromatic aberration. Declaring with
// enums named exactly GlobalFloatConfigs/GlobalIntConfigs reproduces the mangled
// names _Z20psmvr_GetFloatConfig18GlobalFloatConfigs / _Z18psmvr_GetIntConfig...
enum GlobalFloatConfigs : int {};
enum GlobalIntConfigs   : int {};
float  psmvr_GetFloatConfig(GlobalFloatConfigs idx);   // returns float (r0), NOT double
int    psmvr_GetIntConfig(GlobalIntConfigs idx);
static inline float cfgF(int i) { return psmvr_GetFloatConfig((GlobalFloatConfigs) i); }
static inline int   cfgI(int i) { return psmvr_GetIntConfig((GlobalIntConfigs) i); }

// SDK render-event ids (issued via the GetRenderEventFunc pointer). We only use
// InitRenderThread -- it wires up the tracking data pipe so Pvr_GetMainSensorState
// returns live pose. We deliberately never issue TimeWarp (that path needs Unity
// and aborts); we present the eyes ourselves.
enum { EV_InitRenderThread = 1024, EV_Pause = 1025, EV_Resume = 1026 };
typedef void (*RenderEventFunc)(int);
