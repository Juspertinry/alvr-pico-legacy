#include <jni.h>
#include <android/log.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl3.h>
#include <android/native_window.h>
#include <android/native_window_jni.h>
#include <pthread.h>
#include <unistd.h>
#include <sched.h>          // sched_setaffinity / SCHED_FIFO
#include <sys/resource.h>   // setpriority fallback
#include <dirent.h>         // scan /proc/self/task to find the SDK warp thread
#include <errno.h>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <atomic>
#include <mutex>
#include <vector>
#include <cstdio>
#include <time.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <sys/system_properties.h>

#include "alvr_client_core.h"
#include "alvr_ext.h"    // fork-only ALVR C API additions
#include "log.h"         // TAG / LOGI / LOGE / nowNs()
#include "pico_sdk.h"    // Pico native SDK prototypes + config accessors + render events
#include "math3d.h"      // Mat4 / Quat helpers
#include "gl_util.h"     // compile()
#include "eye_tracking.h"// readEyeGazes() + gaze/openness state
#include "device_info.h" // IP / status / model strings + readers
#include "ui_kit.h"      // font + appendTextLine/Quad + immediate-mode widget kit
#include "eq_panel.h"    // 16-band audio EQ: state + persistence + buildEqVerts
#include "settings_panel.h"  // unified lobby SETTINGS window (sidebar + scroll)
#include "app_state.h"   // shared lobby/render knobs: IPD, input edges, toggles, diag
#include "lobby_panels.h"// diagnostics overlay builders
#include "input.h"       // controller + head-pose shared state
#include "foveation.h"   // readFoveationParams() from the settings JSON
#include "render_thread.h"// shared render-thread lifetime/window/sleep state

static const char *alvrEventName(AlvrEvent_Tag t) {
    switch (t) {
        case ALVR_EVENT_HUD_MESSAGE_UPDATED: return "HUD_MESSAGE_UPDATED";
        case ALVR_EVENT_STREAMING_STARTED:   return "STREAMING_STARTED";
        case ALVR_EVENT_STREAMING_STOPPED:   return "STREAMING_STOPPED";
        case ALVR_EVENT_HAPTICS:             return "HAPTICS";
        case ALVR_EVENT_DECODER_CONFIG:      return "DECODER_CONFIG";
        case ALVR_EVENT_REAL_TIME_CONFIG:    return "REAL_TIME_CONFIG";
        default: return "?";
    }
}

JavaVM   *gVM       = nullptr;
jobject   gActivity = nullptr;
jclass    gVrClass  = nullptr;
pthread_t gThread;
std::atomic<bool> gRunning{false};   // render-thread lifetime

// Dedicated fixed-rate tracking thread. Running the pose read + ALVR uplink at a
// fixed cadence here (rather than inline in the render thread's frame-paced spin)
// keeps the rate-dependent velocity filters stable and bounds the tracking packet
// rate over the video Wi-Fi. The render thread only renders + submits and reads
// the published head pose.
static pthread_t        gTrackThread;
static std::atomic<bool> gTrackRunning{false};   // tracking-thread lifetime

// ALVR path ids -- file scope so BOTH the render thread (button-id log) and the
// tracking thread (uplink) can see them. Populated once in renderThread after
// alvr_initialize(), before the tracking thread is started.
struct BtnIds { uint64_t trigVal, gripVal, thumbX, thumbY, thumbClick, menu, face1, face2; };
static uint64_t alvrHeadId = 0;
static uint64_t alvrHandId[2] = { 0, 0 };
static BtnIds   alvrBtn[2] = {};

// Window handed in by the SurfaceView callbacks. The render thread owns one
// long-lived EGL display+context; it (re)creates only the *window surface* as
// the SurfaceView surface is destroyed (sleep / off-head) and recreated (wake).
std::mutex     gWinMutex;
ANativeWindow *gPendingWindow = nullptr;   // latest window (or null)
std::atomic<bool> gWindowDirty{false};     // a change is pending (see render_thread.h)

// Shared "position + per-vertex colour" shader: the grid, HUD text, and eye-gaze
// marker all reuse this same program (gProg) + uMVP.
static const char *kVtxSrc =
    "#version 300 es\n"
    "layout(location=0) in vec3 aPos;\n"
    "layout(location=1) in vec3 aColor;\n"
    "uniform mat4 uMVP;\n"
    "out vec3 vColor;\n"
    "void main(){ vColor=aColor; gl_Position=uMVP*vec4(aPos,1.0); }\n";

static const char *kFrgSrc =
    "#version 300 es\n"
    "precision mediump float;\n"
    "in vec3 vColor; out vec4 oColor;\n"
    "void main(){ oColor=vec4(vColor,1.0); }\n";

static GLuint gProg = 0;
static GLint  gMvpLoc = -1;

// Build the shared pos+colour program (gProg). The grid / HUD text / gaze marker
// each create their own VAO+VBO against it; there is no geometry to upload here.
static void buildGraphics() {
    gProg = glCreateProgram();
    GLuint vs = compile(GL_VERTEX_SHADER, kVtxSrc);
    GLuint fs = compile(GL_FRAGMENT_SHADER, kFrgSrc);
    glAttachShader(gProg, vs); glAttachShader(gProg, fs);
    glLinkProgram(gProg);
    GLint ok=0; glGetProgramiv(gProg, GL_LINK_STATUS, &ok);
    if (!ok) { char log[512]; glGetProgramInfoLog(gProg, 512, nullptr, log); LOGE("link: %s", log); }
    gMvpLoc = glGetUniformLocation(gProg, "uMVP");
    LOGI("graphics built prog=%u", gProg);
}

// ALVR's storage uses app_dirs2, which needs $HOME (unset on Android) -> it
// panics. Point HOME at the app's private files dir before initializing ALVR.
static void setHomeFromFilesDir(JNIEnv *env, jobject activity) {
    jclass cls = env->GetObjectClass(activity);
    jmethodID m = env->GetMethodID(cls, "getFilesDir", "()Ljava/io/File;");
    jobject file = env->CallObjectMethod(activity, m);
    jclass fcls = env->GetObjectClass(file);
    jmethodID gp = env->GetMethodID(fcls, "getAbsolutePath", "()Ljava/lang/String;");
    jstring jpath = (jstring) env->CallObjectMethod(file, gp);
    const char *path = env->GetStringUTFChars(jpath, nullptr);
    setenv("HOME", path, 1);
    LOGI("set HOME=%s", path);
    env->ReleaseStringUTFChars(jpath, path);
}

// ---------- ALVR stream presentation (swapchain -> SDK warp) ----------------
// PicoNeo2 fork: ALVR's stream shader (forked stream.wgsl) now writes the FINAL
// present-domain bytes -- warm white balance + linear->sRGB encode + triangular
// dither baked in -- straight into these plain RGBA8 swapchain textures, which we
// hand DIRECTLY to the SDK DIATW warp. There is no separate sRGB-encode blit pass:
// that saves one full-res read+write per eye per frame (~the device's main thermal/
// bandwidth lever on this Adreno). The legacy warp presents raw bytes (no sRGB
// encode, no dither), which is why the encode+dither must live INSIDE the stream
// shader rather than rely on the warp.
//
// Ring depth: the same ring is both ALVR's render target AND the texture submitted
// to the warp, so the render write-head must never alias a frame still live in the
// warp's ring. The warp keeps up to 4 submitted entries (SelectRT may fall back to
// the oldest) + 1 slot we're currently rendering into = 5.
static const int kSwapLen = 5;
static GLuint gSwap[2][kSwapLen] = {{0},{0}};
static GLuint gStreamFbo = 0;   // reusable FBO for the diag HUD overlay into gSwap
static int    gSwapIdx = 0;     // render write-head into the ring
// PIPELINE: hand the warp the PREVIOUS frame's slot (already GPU-complete -> the
// fence wait returns instantly) instead of CPU-blocking ~5ms on the current frame.
// Costs +1 frame of WORLD-content latency (head tracking stays low-latency via warp
// reprojection). Per-slot fence + the render pose that slot was rendered with.
static GLsync gSwapFence[kSwapLen] = {0};
static AlvrViewParams gSwapVP[kSwapLen][2] = {};
static int    gPrevSwapIdx = -1;
static bool   gPrevSwapValid = false;
// Async present state: the SDK DIATW compositor owns the window and re-projects
// the last decoded eye textures to a fresh pose every vsync; we just keep feeding
// it the newest frame (so head motion stays smooth even below video framerate).
// The +1-frame pipeline state is gPrevSwapIdx/gPrevSwapValid (above).

// The SDK warp thread owns the window: we hand it eye textures (HW lens
// distortion + async reprojection + direct present) and never self-present.
static bool   gWarpToWindow = false;  // warp thread was given the real window surface
static bool   gAtwEnabled = false;
static uint32_t gStreamW = 0, gStreamH = 0;
// Foveation params currently APPLIED to the de-foveation pipeline (baked into the
// last alvr_start_stream_opengl). Cached so we can detect a server-side foveation
// change mid-session and re-sync -- otherwise stale de-foveation params vs the
// freshly-foveated stream misalign the eyes until an app restart.
static bool  gFoveOn = false;
static float gFovParams[6] = {0,0,0,0,0,0};   // csx,csy,shx,shy,erx,ery (currently APPLIED)
// Foveation re-sync debounce. A re-sync rebuilds the de-foveation swapchain
// (multi-MB GPU realloc) + restarts the stream renderer, so a burst of
// REAL_TIME_CONFIG events (e.g. a server-side FFR slider drag) must NOT trigger one
// realloc per event. The handler stashes the latest pending params + the time they
// last changed; the loop applies a single rebuild once they've been stable ~250ms.
static float    gFovPending[6] = {0,0,0,0,0,0};
static uint64_t gFovPendingNs = 0;
static bool     gFovResyncPending = false;
static const uint64_t kFovDebounceNs = 250000000ULL;   // 250ms settle window
// Proximity sleep: the Java proximity listener sets gSleepReq true after the headset
// has been OFF the head for the timeout (default 60s) and false on don. The render
// thread acts on the EDGE: pause ALVR (server stops sending; decoder torn down) to
// save power, then resume on wear. alvr_pause/resume must run on THIS (render) thread
// where the connection lives, so we only flip a flag from Java.
std::atomic<bool> gSleepReq{false};
static bool gSlept = false;
// Play-area extents (meters) read from the Pico boundary at init; forwarded to
// SteamVR as the chaperone on each STREAMING_STARTED. 0 = none configured.
static float gPlayspaceW = 0.0f, gPlayspaceD = 0.0f;
// Negotiated stream refresh rate (from STREAMING_STARTED); fed to the decoder as
// the frame-rate so the Venus driver stops logging "Unable to convey fps info".
static float gRefreshHint = 72.0f;
// These stream-lifecycle flags are touched ONLY on the render thread (the event
// drain, the video path, createVideoDecoder, and applyServerEyeTracking which
// merely passes gStreaming by value to the eye worker). No other thread reads or
// writes them, so they stay plain bool -- making them atomic would be pure noise.
// gSleepReq / gManualLobby / gWindowDirty ARE cross-thread and are atomic.
static bool   gStreaming = false;
static bool   gDecoderReady = false;
static bool   gAlvrGlReady = false;


// HW-compositor lobby: a per-eye ring of textures we render the lobby into and
// hand to the SDK warp (it does the lens distortion + reprojection + present),
// exactly like the video eye textures. RGBA8 holding authored (display) colours
// -- the warp presents them as-is (no sRGB encode).
// Ring DEPTH: the async warp free-runs and re-samples the textures we submit
// across several vsyncs; its SelectRT ring keeps up to 4 submitted entries. While
// the lobby ANIMATES (EQ fader drag, locomotion, theme flip) we render+submit a
// fresh slot every frame, so the warp can be holding the last 4 distinct slots at
// the instant we render the next one. A ring of 2 would mean the slot we
// re-rendered is still live in the warp's ring -> it could sample a half-written
// lobby texture = tear/flicker. 5 = the warp's 4 held entries + the
// 1 we're rendering into never alias (matches the video ring's kSwapLen reasoning).
static const int kLobbySz = 1536;   // per-eye lobby render target (~1.5x linear res)
static const int kLobbyRing = 5;
static GLuint gLobbyEye[2][kLobbyRing] = {{0},{0}};
static bool   gLobbyEyeReady = false;
static GLuint gLobbyFbo = 0, gLobbyDepth = 0;
// Per-slot GPU fence + the pose each slot was rendered at, so the lobby submit can
// PIPELINE like the video path: this frame renders into slot X (fence + flush, no
// block) and hands the warp slot X-1 -- rendered a full frame ago and therefore
// already GPU-complete -- instead of blocking on the just-issued render every frame.
static GLsync gLobbyFence[kLobbyRing] = {0};
static Quat   gLobbyPoseQ[kLobbyRing];
static float  gLobbyPoseP[kLobbyRing][3];
static void buildLobbyTarget() {
    for (int e = 0; e < 2; e++) {
        glGenTextures(kLobbyRing, gLobbyEye[e]);
        for (int i = 0; i < kLobbyRing; i++) {
            glBindTexture(GL_TEXTURE_2D, gLobbyEye[e][i]);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, kLobbySz, kLobbySz, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        }
    }
    gLobbyEyeReady = true;
    glGenRenderbuffers(1, &gLobbyDepth);
    glBindRenderbuffer(GL_RENDERBUFFER, gLobbyDepth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, kLobbySz, kLobbySz);
    glGenFramebuffers(1, &gLobbyFbo);
    glBindFramebuffer(GL_FRAMEBUFFER, gLobbyFbo);
    // Attach ring slot [0][0] purely for the completeness check; the render loop
    // re-attaches the live per-eye ring texture each frame.
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, gLobbyEye[0][0], 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, gLobbyDepth);
    bool ok = (glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
    LOGI("lobby target built %dx%d ok=%d", kLobbySz, kLobbySz, ok);
}

// Release the lobby eye-texture ring + FBO/depth (the only sizeable lobby GPU
// allocation: 2 eyes x kLobbyRing x kLobbySz^2 RGBA8 ~ 94 MB). Called when a
// stream starts so that memory isn't parked while we're in-game; buildLobbyTarget
// is re-run lazily the next time the lobby is shown. Requires a current context.
static void freeLobbyTarget() {
    if (!gLobbyEyeReady) return;
    for (int e = 0; e < 2; e++) {
        glDeleteTextures(kLobbyRing, gLobbyEye[e]);
        for (int i = 0; i < kLobbyRing; i++) gLobbyEye[e][i] = 0;
    }
    if (gLobbyFbo)   { glDeleteFramebuffers(1, &gLobbyFbo);   gLobbyFbo = 0; }
    if (gLobbyDepth) { glDeleteRenderbuffers(1, &gLobbyDepth); gLobbyDepth = 0; }
    for (int i = 0; i < kLobbyRing; i++) if (gLobbyFence[i]) { glDeleteSync(gLobbyFence[i]); gLobbyFence[i] = 0; }
    gLobbyEyeReady = false;
    LOGI("lobby target freed (~%d MB reclaimed)", (2*kLobbyRing*kLobbySz*kLobbySz*4)/(1024*1024));
}

// Eye-gaze debug marker: a small bright-green filled disc (triangle fan emitted
// as GL_TRIANGLES) in the XY plane (normal +Z, radius pre-baked). Reuses gProg
// (attrib0=pos.xyz, attrib1=rgb). Drawn in the lobby at the gaze point, billboard
// to the head. Only shown on a Neo 2 EYE (gated by gEyeOnline).
static GLuint gGazeVao = 0, gGazeVbo = 0;
static int    gGazeVertCount = 0;
static void buildGazeMarker() {
    const int   kSeg = 28;
    const float kR   = 0.05f;     // ~5cm disc at the gaze point
    std::vector<float> v;
    for (int i = 0; i < kSeg; i++) {
        float a0 = (float)i       / kSeg * 2.0f * (float)M_PI;
        float a1 = (float)(i + 1) / kSeg * 2.0f * (float)M_PI;
        // center, then two rim points -> one triangle per segment
        const float g[3] = { 0.1f, 1.0f, 0.2f };   // bright green
        v.insert(v.end(), { 0,0,0, g[0],g[1],g[2] });
        v.insert(v.end(), { cosf(a0)*kR, sinf(a0)*kR, 0, g[0],g[1],g[2] });
        v.insert(v.end(), { cosf(a1)*kR, sinf(a1)*kR, 0, g[0],g[1],g[2] });
    }
    gGazeVertCount = (int)(v.size() / 6);
    glGenVertexArrays(1, &gGazeVao);
    glBindVertexArray(gGazeVao);
    glGenBuffers(1, &gGazeVbo);
    glBindBuffer(GL_ARRAY_BUFFER, gGazeVbo);
    glBufferData(GL_ARRAY_BUFFER, v.size()*sizeof(float), v.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glBindVertexArray(0);
    LOGI("gaze marker built verts=%d", gGazeVertCount);
}

// Floor grid for the lobby so it reads as a space, not a black void. Lines on the
// y=0 plane (the calibrated floor, since we use a Floor/Stage tracking origin), in
// a dim blue-grey. WORLD-locked (drawn with the head-tracked view) for spatial
// reference. Reuses gProg (pos.xyz + rgb), drawn as GL_LINES.
static GLuint gGridVao = 0, gGridVbo = 0;
static int    gGridVertCount = 0;
static void buildGridFloor() {
    std::vector<float> v;
    const float ext = 5.0f, step = 0.5f;
    // Themed floor: cold blue-grey or warm amber-grey. Brighter centre axes.
    bool amber = gThemeAmber.load();
    const float c0 = amber ? 0.34f : 0.16f, c1 = amber ? 0.22f : 0.26f, c2 = amber ? 0.08f : 0.40f;
    const float a0 = amber ? 0.60f : 0.30f, a1 = amber ? 0.44f : 0.36f, a2 = amber ? 0.18f : 0.42f;
    for (float i = -ext; i <= ext + 1e-3f; i += step) {
        bool axis = (i > -1e-3f && i < 1e-3f);
        float r = axis ? a0 : c0, g = axis ? a1 : c1, b = axis ? a2 : c2;
        v.insert(v.end(), { -ext,0,i, r,g,b }); v.insert(v.end(), {  ext,0,i, r,g,b });
        v.insert(v.end(), {  i,0,-ext, r,g,b }); v.insert(v.end(), {  i,0, ext, r,g,b });
    }
    gGridVertCount = (int)(v.size() / 6);
    if (!gGridVao) {   // first build: create GL objects. Later calls just recolour.
        glGenVertexArrays(1, &gGridVao);
        glBindVertexArray(gGridVao);
        glGenBuffers(1, &gGridVbo);
        glBindBuffer(GL_ARRAY_BUFFER, gGridVbo);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
        glBindVertexArray(0);
    }
    glBindBuffer(GL_ARRAY_BUFFER, gGridVbo);
    glBufferData(GL_ARRAY_BUFFER, v.size()*sizeof(float), v.data(), GL_DYNAMIC_DRAW);
    LOGI("grid floor built verts=%d amber=%d", gGridVertCount, (int)amber);
}

// ---------------------------------------------------------------------------
// Controller wireframe models, pulled live from the Neo 2 system assets. The
// meshes live world-readable on /system as plain Wavefront OBJ (centimetres,
// centred at origin, long axis = local Z, same frame as the tracked pose). We
// read v + f, emit each face's edges as line segments, scale to metres, and draw
// them as themed wireframe attached to the live controller pose each frame.
// 0 = left (controller2s.obj / "ppController_NEO2_Left"), 1 = right (r.obj).
// ---------------------------------------------------------------------------
static GLuint gCtrlVao[2] = {0,0}, gCtrlVbo[2] = {0,0};
static int    gCtrlVertCount[2] = {0,0};
static std::vector<float> gCtrlPos[2];   // line-endpoint positions (xyz), in metres
// The system OBJ filenames don't match the hand they look correct on in our
// frame: r.obj is the mesh that reads right on the LEFT hand's pose, and
// controller2s.obj reads right on the RIGHT hand. Mapped by what's correct
// on-device, not by the filename.
static const char *kCtrlObjPath[2] = {
    "/system/pre_resource/data/misc/user/controller/r.obj",            // 0 = left hand
    "/system/pre_resource/data/misc/user/controller/controller2s.obj", // 1 = right hand
};
static void loadCtrlObjLines(const char *path, std::vector<float> &out) {
    out.clear();
    FILE *f = fopen(path, "r");
    if (!f) { LOGE("ctrl obj missing: %s", path); return; }
    std::vector<float> vx, vy, vz;
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        if (line[0]=='v' && line[1]==' ') {
            float x,y,z;
            if (sscanf(line+2, "%f %f %f", &x,&y,&z)==3) { vx.push_back(x); vy.push_back(y); vz.push_back(z); }
        } else if (line[0]=='f' && line[1]==' ') {
            int idx[8]; int n=0;
            char *tok = strtok(line+2, " \t\r\n");
            while (tok && n<8) {
                int vi = atoi(tok);          // leading number = vertex index (1-based; neg = relative)
                if (vi != 0) idx[n++] = (vi > 0) ? vi-1 : (int)vx.size()+vi;
                tok = strtok(nullptr, " \t\r\n");
            }
            auto edge = [&](int a, int b){
                if (a<0||b<0||a>=(int)vx.size()||b>=(int)vx.size()) return;
                out.insert(out.end(), { vx[a],vy[a],vz[a] });
                out.insert(out.end(), { vx[b],vy[b],vz[b] });
            };
            for (int i=0;i<n;i++) edge(idx[i], idx[(i+1)%n]);
        }
    }
    fclose(f);
    for (auto &c : out) c *= 0.01f;          // centimetres -> metres
    LOGI("ctrl obj %s -> %d line verts", path, (int)(out.size()/3));
}
static void buildControllerMeshes() {
    bool amber = gThemeAmber.load();
    const float cr = amber?0.98f:0.80f, cg = amber?0.80f:0.90f, cb = amber?0.45f:1.00f;  // themed, bright
    for (int h=0; h<2; h++) {
        if (gCtrlPos[h].empty()) loadCtrlObjLines(kCtrlObjPath[h], gCtrlPos[h]);
        std::vector<float> v; v.reserve(gCtrlPos[h].size()*2);
        for (size_t i=0; i+3<=gCtrlPos[h].size(); i+=3)
            v.insert(v.end(), { gCtrlPos[h][i], gCtrlPos[h][i+1], gCtrlPos[h][i+2], cr,cg,cb });
        gCtrlVertCount[h] = (int)(v.size()/6);
        if (!gCtrlVao[h]) {
            glGenVertexArrays(1,&gCtrlVao[h]);
            glBindVertexArray(gCtrlVao[h]);
            glGenBuffers(1,&gCtrlVbo[h]);
            glBindBuffer(GL_ARRAY_BUFFER,gCtrlVbo[h]);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,6*sizeof(float),(void*)0);
            glEnableVertexAttribArray(1);
            glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,6*sizeof(float),(void*)(3*sizeof(float)));
            glBindVertexArray(0);
        }
        glBindBuffer(GL_ARRAY_BUFFER, gCtrlVbo[h]);
        glBufferData(GL_ARRAY_BUFFER, v.size()*sizeof(float), v.data(), GL_STATIC_DRAW);
    }
}

// 3D HUD text + lobby UI kit (font, appendTextLine/Quad, ui* widgets) now live
// in ui_kit.h/.cpp. The dynamic VBOs they fill are still owned here.
static GLuint gTextVao = 0, gTextVbo = 0;
static GLuint gSliderVao = 0, gSliderVbo = 0;   // lobby SETTINGS panel (dynamic)
static GLuint gReticleVao = 0, gReticleVbo = 0; // head-gaze crosshair (static "+")
static int    gReticleVertCount = 0;
static GLuint gEqVao = 0, gEqVbo = 0;           // lobby 16-band audio EQ panel (dynamic)
static GLuint gLaserVao = 0, gLaserVbo = 0;     // controller laser beam (dynamic, world-space)
static GLuint gDiagVao = 0, gDiagVbo = 0;       // streaming diagnostics overlay (dynamic, NDC)
static void buildTextBuffers() {
    glGenVertexArrays(1, &gTextVao);
    glBindVertexArray(gTextVao);
    glGenBuffers(1, &gTextVbo);
    glBindBuffer(GL_ARRAY_BUFFER, gTextVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    // same pos.xyz+rgb layout for the slider panel
    glGenVertexArrays(1, &gSliderVao);
    glBindVertexArray(gSliderVao);
    glGenBuffers(1, &gSliderVbo);
    glBindBuffer(GL_ARRAY_BUFFER, gSliderVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    // same pos.xyz+rgb layout for the 16-band EQ panel
    glGenVertexArrays(1, &gEqVao);
    glBindVertexArray(gEqVao);
    glGenBuffers(1, &gEqVbo);
    glBindBuffer(GL_ARRAY_BUFFER, gEqVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    // controller laser beam (world-space line)
    glGenVertexArrays(1, &gLaserVao);
    glBindVertexArray(gLaserVao);
    glGenBuffers(1, &gLaserVbo);
    glBindBuffer(GL_ARRAY_BUFFER, gLaserVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    // diagnostics overlay (same pos.xyz+rgb layout)
    glGenVertexArrays(1, &gDiagVao);
    glBindVertexArray(gDiagVao);
    glGenBuffers(1, &gDiagVbo);
    glBindBuffer(GL_ARRAY_BUFFER, gDiagVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glBindVertexArray(0);
}

// Cached video decoder config so we can RECREATE the decoder after a proximity-sleep
// teardown without waiting for a reconnect. The DecoderConfig event only fires once
// per stream, so we keep a copy.
static int      gDecCodec   = 0;
static uint8_t  gDecCfg[16384];
static uint64_t gDecCfgLen  = 0;
static bool     gHaveDecCfg = false;
// The decoder output-pump thread is created + destroyed with each decoder, so
// the big-core pin must be re-armed every time a new one is made. createVideoDecoder
// clears these; the render loop retries the /proc scan until it lands or gives up.
static bool     gDecoderPinned = false;
static int      gDecoderPinTries = 0;

// Push the current Software IPD (as per-eye optical-centre offset) + our FOV to
// the server. Called at STREAMING_STARTED and again whenever the IPD changes mid
// -stream so the server's render tracks the lobby setting without a reconnect.
static void sendViewParams() {
    const float hh = 50.5f * (float) M_PI / 180.0f;   // per-eye half-FOV (~101deg total)
    AlvrViewParams vp[2];
    for (int e = 0; e < 2; e++) {
        vp[e].pose.orientation = { 0, 0, 0, 1 };
        vp[e].pose.position[0] = (e == 0 ? -softIpdM()*0.5f : softIpdM()*0.5f);
        vp[e].pose.position[1] = 0; vp[e].pose.position[2] = 0;
        vp[e].fov = { -hh, hh, hh, -hh };
    }
    alvr_send_view_params(vp);
    gSentIpdMm.store(gSoftIpdMm.load());
}

// Report HMD + controller battery levels to the server so the SteamVR dashboard
// shows real charge (it read 0% before -- alvr_send_battery was never called).
// Separate sink from the lobby HUD battery readout; this only talks to the server.
// Call ~1/sec while connected. HMD level from Android's power_supply sysfs; the
// controllers report a 0..100 level via the CV service (plumbed as keys[10]).
static void sendBatteryReports() {
    long cap = -1; bool plugged = false;
    FILE *f = fopen("/sys/class/power_supply/battery/capacity", "r");
    if (f) { if (fscanf(f, "%ld", &cap) != 1) cap = -1; fclose(f); }
    f = fopen("/sys/class/power_supply/battery/status", "r");
    if (f) { char s[32] = {0}; if (fgets(s, sizeof(s), f)) plugged = (strstr(s, "Charging") || strstr(s, "Full")); fclose(f); }
    if (cap >= 0) alvr_send_battery(alvrHeadId, (float) cap / 100.0f, plugged);

    int  cbat[2] = { -1, -1 };
    { std::lock_guard<std::mutex> lk(gCtrlMutex);
      for (int h = 0; h < 2; h++)
          if (gCtrl[h].conn == 1 && gCtrl[h].keyCount > 10) cbat[h] = gCtrl[h].keys[10]; }
    for (int h = 0; h < 2; h++)
        if (cbat[h] >= 0) alvr_send_battery(alvrHandId[h], (float) cbat[h] / 100.0f, false);
}


// (Re)create the MediaCodec video decoder from the cached stream config. Used both
// for the initial DecoderConfig event and to resume after a proximity-sleep teardown.
static void createVideoDecoder() {
    if (!gHaveDecCfg) return;
    // Freshness invariant: the frame-rate / operating-rate opts below are baked
    // from gRefreshHint, which is set by STREAMING_STARTED. Every legitimate caller
    // runs strictly AFTER that event -- DECODER_CONFIG is a later FIFO event in the
    // same drain, and the manual-lobby resume is mid-session -- so gStreaming is true
    // and gRefreshHint holds the negotiated rate (a reconnect destroys the stale
    // decoder in STREAMING_STARTED, so the next DECODER_CONFIG rebuilds fresh). If we
    // ever reach here with !gStreaming the ordering broke and gRefreshHint may be the
    // 72.0 default -> log it loudly rather than silently bake a stale rate.
    if (!gStreaming)
        LOGE("createVideoDecoder: !gStreaming -- gRefreshHint(%.1f) may be stale/default", gRefreshHint);
    AlvrDecoderConfig dc = {};
    dc.codec = (AlvrCodec) gDecCodec;
    dc.force_software_decoder   = false;
    // 1.5-frame jitter buffer: lower QUEUE latency than 2.0 with no starvation. The
    // bigger latency lever from here is server-side encode/bitrate, not this.
    dc.max_buffering_frames     = 1.5f;
    dc.buffering_history_weight = 0.90f;
    int fps = (int)(gRefreshHint > 1.0f ? gRefreshHint + 0.5f : 72.0f);
    static AlvrMediacodecOption decOpts[5] = {};
    decOpts[0].key = "vendor.qti-ext-dec-low-latency.enable";
    decOpts[0].ty = ALVR_MEDIACODEC_PROP_TYPE_INT32; decOpts[0].value.int32 = 1;
    decOpts[1].key = "low-latency";
    decOpts[1].ty = ALVR_MEDIACODEC_PROP_TYPE_INT32; decOpts[1].value.int32 = 1;
    decOpts[2].key = "frame-rate";
    decOpts[2].ty = ALVR_MEDIACODEC_PROP_TYPE_INT32; decOpts[2].value.int32 = fps;
    // Give the Venus driver an explicit OPERATING RATE (= refresh) so it picks
    // a stable clock for that throughput instead of DVFS-ramping per frame (lower
    // decode-time jitter), and PRIORITY 0 (realtime) so decode is scheduled ahead
    // of best-effort codec work. operating-rate = target fps, NOT a max-clock
    // request (SHORT_MAX) -- that would only add heat on this thermally limited SoC.
    decOpts[3].key = "operating-rate";
    decOpts[3].ty = ALVR_MEDIACODEC_PROP_TYPE_INT32; decOpts[3].value.int32 = fps;
    decOpts[4].key = "priority";
    decOpts[4].ty = ALVR_MEDIACODEC_PROP_TYPE_INT32; decOpts[4].value.int32 = 0;
    dc.options = decOpts; dc.options_count = 5;
    dc.config_buffer = gDecCfg;
    dc.config_buffer_size = gDecCfgLen;
    alvr_create_decoder(dc);
    gDecoderReady = true;
    gDecoderPinned = false; gDecoderPinTries = 0;   // re-arm the big-core pin for the new pump thread
    LOGI("video decoder created codec=%d config=%llu bytes buffering=%.2f fps/operating-rate=%d (refreshHint=%.1f) opts=5",
         gDecCodec, (unsigned long long)gDecCfgLen, dc.max_buffering_frames, fps, gRefreshHint);
}

// Build the head-gaze crosshair: a small "+" centred at the origin (XY plane, in
// metres at the reticle's display distance). Drawn head-locked at view centre.
static void buildReticle() {
    std::vector<float> v;
    appendQuad(v, -0.020f,  0.0025f, 0.020f, -0.0025f, 0.9f, 1.0f, 0.9f);  // horizontal arm
    appendQuad(v, -0.0025f, 0.020f,  0.0025f, -0.020f, 0.9f, 1.0f, 0.9f);  // vertical arm
    gReticleVertCount = (int)(v.size() / 6);
    glGenVertexArrays(1, &gReticleVao);
    glBindVertexArray(gReticleVao);
    glGenBuffers(1, &gReticleVbo);
    glBindBuffer(GL_ARRAY_BUFFER, gReticleVbo);
    glBufferData(GL_ARRAY_BUFFER, v.size()*sizeof(float), v.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glBindVertexArray(0);
}

static void destroyStreamSwapchain() {
    for (int e = 0; e < 2; e++) {
        if (gSwap[e][0] != 0) glDeleteTextures(kSwapLen, gSwap[e]);
        for (int i = 0; i < kSwapLen; i++) gSwap[e][i] = 0;
    }
    // Invalidate the pipeline state: the ring textures are gone, so the stale
    // "previous" index must not be handed to the warp after recreation.
    for (int i = 0; i < kSwapLen; i++) {
        if (gSwapFence[i]) { glDeleteSync(gSwapFence[i]); gSwapFence[i] = 0; }
    }
    // Also clear the stashed per-slot render poses. gPrevSwapValid=false
    // already forces the first frame onto the freshly-stashed current slot, but a
    // zeroed gSwapVP makes the invariant robust (a non-unit quat read from stale
    // memory would otherwise be rejected by the warp's SelectRT).
    memset(gSwapVP, 0, sizeof(gSwapVP));
    gSwapIdx = 0;
    gPrevSwapIdx = -1; gPrevSwapValid = false;
    if (gStreamFbo != 0) { glDeleteFramebuffers(1, &gStreamFbo); gStreamFbo = 0; }
}

static void createStreamSwapchain(uint32_t w, uint32_t h) {
    destroyStreamSwapchain();   // free any prior textures (reconnect / res change)
    for (int e = 0; e < 2; e++) {
        glGenTextures(kSwapLen, gSwap[e]);
        for (int i = 0; i < kSwapLen; i++) {
            glBindTexture(GL_TEXTURE_2D, gSwap[e][i]);
            // Plain RGBA8 (NOT sRGB-storage): the forked stream shader writes already
            // sRGB-ENCODED + dithered bytes here, so the texture must store them
            // verbatim. An sRGB-format texture would re-encode on sample and the warp
            // (which presents raw bytes) would show a double-encoded, wrong image.
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        }
    }
    glGenFramebuffers(1, &gStreamFbo);   // diag HUD overlay draws into gSwap via this
    glBindTexture(GL_TEXTURE_2D, 0);
    LOGI("created stream swapchain %ux%u x%d/eye", w, h, kSwapLen);
}

// Apply a foveation re-sync -- rebuild the de-foveation swapchain + restart the
// stream renderer with new FFR params, so the client de-foveation matches the server
// without an app restart. This is a multi-MB GPU realloc on the render thread, hence
// it's DEBOUNCED by the caller (a burst of REAL_TIME_CONFIG events coalesces into ONE
// call here once the params settle). `np` = the new params; they become the applied
// gFovParams. Caller guarantees gStreaming && gAlvrGlReady && gFoveOn && gStreamW>0
// and that our GL context is current.
static void applyFoveationResync(const float np[6]) {
    memcpy(gFovParams, np, sizeof(gFovParams));
    createStreamSwapchain(gStreamW, gStreamH);
    const uint32_t *swapArr[2] = { gSwap[0], gSwap[1] };
    AlvrStreamConfig sc = {};
    sc.view_resolution_width  = gStreamW;
    sc.view_resolution_height = gStreamH;
    sc.swapchain_textures = (const uint32_t **) swapArr;
    sc.swapchain_length   = kSwapLen;
    sc.enable_foveation        = gFoveOn;
    sc.foveation_center_size_x = np[0];
    sc.foveation_center_size_y = np[1];
    sc.foveation_center_shift_x= np[2];
    sc.foveation_center_shift_y= np[3];
    sc.foveation_edge_ratio_x  = np[4];
    sc.foveation_edge_ratio_y  = np[5];
    sc.enable_upscaling   = false;
    alvr_start_stream_opengl(sc);
    gSwapIdx = 0;   // createStreamSwapchain already reset the pipeline
    LOGI("REAL_TIME_CONFIG: de-foveation re-synced center=(%.3f,%.3f) shift=(%.3f,%.3f) edge=(%.2f,%.2f)",
         np[0],np[1],np[2],np[3],np[4],np[5]);
}

static void callVrStatic(JNIEnv *env, const char *name) {
    if (gVrClass == nullptr) return;   // firmware without VrActivity (see nativeStart)
    if (env->ExceptionCheck()) { env->ExceptionDescribe(); env->ExceptionClear(); }
    jmethodID m = env->GetStaticMethodID(gVrClass, name, "()V");
    if (m == nullptr) { env->ExceptionClear(); LOGE("no static %s", name); return; }
    env->CallStaticVoidMethod(gVrClass, m);
    if (env->ExceptionCheck()) { env->ExceptionDescribe(); env->ExceptionClear(); }
    LOGI("called VrActivity.%s", name);
}

// Upload `v` into a DYNAMIC vbo while REUSING its storage. Re-specs (orphans)
// the buffer only when the data outgrows the current allocation; otherwise it
// glBufferSubData's into the existing buffer. Paired with a STATIC source vector
// at each call site (clear() keeps the heap capacity), this removes the per-frame
// "fresh std::vector + full glBufferData re-spec" churn the lobby builders did
// every frame (~90Hz). `cap` is the caller-owned tracker of the vbo's byte size.
static void uploadDynamicVbo(GLuint vbo, const std::vector<float> &v, GLsizeiptr &cap) {
    GLsizeiptr bytes = (GLsizeiptr)(v.size() * sizeof(float));
    if (bytes == 0) return;
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    if (bytes > cap) {
        cap = bytes + bytes / 2 + 256;   // grow with slack so re-spec is rare
        glBufferData(GL_ARRAY_BUFFER, cap, nullptr, GL_DYNAMIC_DRAW);
    }
    glBufferSubData(GL_ARRAY_BUFFER, 0, bytes, v.data());
}

// Keep the render/submit thread off the little cores and at an elevated
// scheduling priority, so a background Android task can't preempt it between a
// frame becoming ready and the warp submit -> pushing the present into the next
// vsync ("frame from the past"). SD845 = cpu0-3 little (silver), cpu4-7 big
// (gold); we pin to the top half. CPU affinity needs no special permission;
// SCHED_FIFO almost always EPERMs for a normal app, so we try it and fall back to
// the most favourable nice value we're allowed. The thread sleeps whenever idle
// (blocks on the decoder and on the warp submit), so an RT class here
// can't starve a core. This reduces latency JITTER; it does NOT address the
// thermal-throttle FPS drop.
static int pinSubmitThreadForLowLatency() {
    int reservedCpu = -1;   // a big core kept OFF our mask, reserved for the warp
    long n = sysconf(_SC_NPROCESSORS_CONF);
    if (n >= 2) {
        cpu_set_t set; CPU_ZERO(&set);
        long lo = n / 2, hi = n - 1;
        // Leave the TOP big core free for the time-critical SDK warp thread
        // (pinned separately below) so our render/submit work never has to share a
        // core with the warp's per-vsync present -> the EGL context-priority split
        // (GPU side) is now matched by a CPU-side split. Only reserve when there are
        // >=2 big cores; otherwise fall back to using the whole big half.
        if (hi - lo >= 1) { reservedCpu = (int)hi; hi -= 1; }
        for (long c = lo; c <= hi; c++) CPU_SET((int)c, &set);
        if (sched_setaffinity(0, sizeof(set), &set) == 0)
            LOGI("submit thread pinned to big cores [%ld..%ld] (cpu%d reserved for warp)", lo, hi, reservedCpu);
        else
            LOGI("sched_setaffinity failed (errno=%d)", errno);
    }
    struct sched_param sp; sp.sched_priority = 2;   // low RT prio is plenty
    if (sched_setscheduler(0, SCHED_FIFO, &sp) == 0) {
        LOGI("submit thread SCHED_FIFO prio=2");
    } else {
        // Normal apps can't get SCHED_FIFO -> best-effort nice (urgent-display ~ -8).
        if (setpriority(PRIO_PROCESS, 0, -8) == 0)
            LOGI("SCHED_FIFO denied (errno=%d); set nice=-8", errno);
        else
            LOGI("SCHED_FIFO denied + setpriority denied (errno=%d)", errno);
    }
    return reservedCpu;
}

// Find the first thread in THIS process whose comm == name (or 0). Used to locate
// the SDK/fork-created threads we want to pin (the warp + the decoder pump). The
// /proc/self/task walk is why callers throttle their retries.
static pid_t findTidByComm(const char *name) {
    DIR *d = opendir("/proc/self/task");
    if (!d) return 0;
    pid_t tid = 0;
    struct dirent *e;
    while ((e = readdir(d)) != nullptr) {
        if (e->d_name[0] < '0' || e->d_name[0] > '9') continue;
        char path[80]; snprintf(path, sizeof(path), "/proc/self/task/%s/comm", e->d_name);
        FILE *f = fopen(path, "r");
        if (!f) continue;
        char comm[64] = {0};
        if (fgets(comm, sizeof(comm), f)) {
            char *nl = strchr(comm, '\n'); if (nl) *nl = 0;
            if (strcmp(comm, name) == 0) tid = (pid_t) atoi(e->d_name);
        }
        fclose(f);
        if (tid) break;
    }
    closedir(d);
    return tid;
}

// CPU set of the big-core half [n/2 .. n-1] MINUS exceptCpu (the core the SDK warp
// actually runs on, kept clear for it). Falls back to the full big half if
// excluding would leave it empty. Writes the [lo,hi] big range for logging.
static cpu_set_t bigCoreSetExcept(int exceptCpu, long &loOut, long &hiOut) {
    long n = sysconf(_SC_NPROCESSORS_CONF);
    long lo = (n >= 2) ? n / 2 : 0, hi = n - 1;
    cpu_set_t set; CPU_ZERO(&set);
    for (long c = lo; c <= hi; c++) if ((int)c != exceptCpu) CPU_SET((int)c, &set);
    if (CPU_COUNT(&set) == 0) for (long c = lo; c <= hi; c++) CPU_SET((int)c, &set);
    loOut = lo; hiOut = hi;
    return set;
}

// Raise the SDK's DIATW warp thread priority and DISCOVER which big core it runs
// on, so the caller can keep our submit + decoder threads off it. The warp must
// finish its reproject+present within every ~13.3ms refresh; sharing its core with
// our render/submit work delays it -> it re-presents the last frame ("frame from
// the past"). The EGL context-priority split makes the GPU preempt in the warp's
// favour; this does the same on the CPU. The warp thread is created by the SDK
// inside EV_InitRenderThread and named "WarpThread" (PVR::TimeWarpLocal::
// WarpThread). The SDK re-affinitizes its OWN warp thread (observed pinned to a
// single big core, cpu6) AFTER any affinity we set, so rather than fight it we READ
// where it landed and reserve THAT core. Only if the warp is left floating across
// multiple big cores (SDK didn't pin it) do we pin it to the top big core ourselves
// so it still gets an exclusive one. Returns the warp's big core to reserve, or -1
// if the thread isn't up/named yet (caller retries).
// Best-effort: SCHED_FIFO EPERMs for a normal app -> favourable nice fallback.
static int pinWarpThreadForLowLatency(int guessReservedCpu) {
    pid_t warpTid = findTidByComm("WarpThread");
    if (!warpTid) return -1;   // warp not up / not named yet -> caller retries

    // Priority is separate from affinity and (unlike affinity) is NOT observed to be
    // overridden by the SDK, so still raise it above our submit thread's prio 2.
    struct sched_param sp; sp.sched_priority = 3;
    if (sched_setscheduler(warpTid, SCHED_FIFO, &sp) == 0)
        LOGI("WarpThread tid=%d SCHED_FIFO prio=3", (int)warpTid);
    else if (setpriority(PRIO_PROCESS, warpTid, -10) == 0)
        LOGI("WarpThread SCHED_FIFO denied (errno=%d); set nice=-10", errno);
    else
        LOGI("WarpThread prio elevation denied (errno=%d)", errno);

    long n = sysconf(_SC_NPROCESSORS_CONF);
    long bigLo = (n >= 2) ? n / 2 : 0, bigHi = n - 1;
    int warpCore = guessReservedCpu;
    cpu_set_t cur; CPU_ZERO(&cur);
    if (sched_getaffinity(warpTid, sizeof(cur), &cur) == 0) {
        int bigSet = 0, highBig = -1;
        for (long c = bigLo; c <= bigHi; c++) if (CPU_ISSET((int)c, &cur)) { bigSet++; highBig = (int)c; }
        if (bigSet == 1) {
            warpCore = highBig;   // SDK pinned it here -> respect it, reserve this core
            LOGI("WarpThread tid=%d is SDK-pinned to big core %d -> reserving it", (int)warpTid, warpCore);
        } else {
            // Floating across >1 big core (or none): the SDK didn't pin it, so WE pin
            // it to the top big core and reserve that.
            warpCore = (bigHi >= bigLo) ? (int)bigHi : guessReservedCpu;
            cpu_set_t one; CPU_ZERO(&one); CPU_SET(warpCore, &one);
            if (sched_setaffinity(warpTid, sizeof(one), &one) == 0)
                LOGI("WarpThread tid=%d was floating (%d big cores) -> pinned to dedicated core %d",
                     (int)warpTid, bigSet, warpCore);
            else
                LOGI("WarpThread tid=%d float-pin to cpu%d failed (errno=%d); reserving it anyway",
                     (int)warpTid, warpCore, errno);
        }
    }
    return warpCore;
}

// Pin the fork's decoder OUTPUT-PUMP thread (named "AlvrDecOutput" in the
// client_core fork's video_decoder/android.rs) to the big cores + raise its
// priority. It's the in-process thread that loops dequeue_output_buffer +
// release_output_buffer, which paces how fast decoded frames land in the
// ImageReader -> how soon alvr_get_frame(_timeout) returns. On the SD845 a
// background task parking it on a little core stretches decode->present latency
// and adds frame-arrival jitter -- the same hazard the render + warp pins address.
// (The actual HW codec runs out-of-process in the Venus HAL and can't be pinned
// from here; this targets the part of the decode path we own.) reservedCpu = the
// warp's dedicated big core, kept OUT of the decoder mask so the pump never shares
// a core with the per-vsync present. Best-effort: SCHED_FIFO EPERMs for a normal
// app -> favourable nice fallback. Returns true once found + pinned so the caller
// stops scanning. Re-armed per decoder (see createVideoDecoder / the loop).
static bool pinDecoderThreadForLowLatency(int reservedCpu) {
    pid_t decTid = findTidByComm("AlvrDecOutput");
    if (!decTid) return false;   // pump not up / not named yet -> caller retries

    long lo = 0, hi = 0;
    cpu_set_t set = bigCoreSetExcept(reservedCpu, lo, hi);   // big half minus the warp's core
    if (sched_setaffinity(decTid, sizeof(set), &set) == 0)
        LOGI("decoder pump tid=%d pinned to big cores [%ld..%ld] minus warp core %d",
             (int)decTid, lo, hi, reservedCpu);
    else
        LOGI("decoder pump affinity failed (errno=%d)", errno);
    // prio just below the warp (3) -- match the submit thread's 2 so the pump and
    // submit cooperate (they alternate: submit blocks waiting on the pump's frame).
    struct sched_param sp; sp.sched_priority = 2;
    if (sched_setscheduler(decTid, SCHED_FIFO, &sp) == 0)
        LOGI("decoder pump SCHED_FIFO prio=2");
    else if (setpriority(PRIO_PROCESS, decTid, -8) == 0)
        LOGI("decoder pump SCHED_FIFO denied (errno=%d); set nice=-8", errno);
    else
        LOGI("decoder pump prio elevation denied (errno=%d)", errno);
    return true;
}

// MAX PERFORMANCE: pin the SoC CPU/GPU perf level via the Pico/QVR perf service --
// the same clock-floor knob the VR-home Power Profile drives. Always on, so we don't
// depend on the headset's Power Profile for a stable floor during streaming (a
// Battery-Saving headset would otherwise starve the decode/submit path). The thermal
// governor still clamps the GPU when hot, so a high floor can't overheat us on its
// own -- it just removes the governor's ramp-up latency when the SoC is cool,
// tightening frame timing.
// Level range is 0..5 (0..5 accepted, >=6 rejected; 5 = max, 0 = the system default
// baseline). SetCpuLevel/SetGpuLevel(5,true) takes effect on this device.
static const int kPerfLevelMax = 5;   // max valid perf level (hardware ceiling)
// Level we actually PIN CPU+GPU to. The top level (5 -> GPU 710MHz) heats the SD845
// fast, then the thermal driver clamps hard -> a boost/throttle sawtooth. Pinning one
// level lower (4) targets a steadier clock (~675MHz GPU) on both CPU and GPU. Root-
// free + universal (Pico perf service, not sysfs). Thermal step-down still applies
// (kPerfLevelPin-1 when hot). Set == kPerfLevelMax to restore the old top pin.
static const int kPerfLevelPin = 4;
// Baseline perf levels captured BEFORE we ever change them (the system default we
// restore to when releasing the floor).
static bool gPerfBaseCaptured = false;
static int  gPerfBaseCpu = 0, gPerfBaseGpu = 0;
static void capturePerfBaseline() {
    if (gPerfBaseCaptured) return;
    bool cs = false, gs = false; int cl = 0, gl = 0;
    GetCpuLevel(cl, cs); GetGpuLevel(gl, gs);
    gPerfBaseCpu = cl; gPerfBaseGpu = gl; gPerfBaseCaptured = true;
    LOGI("perf: baseline levels cpu=%d gpu=%d (restored when floor released)", cl, gl);
}

// Highest on-die GPU temperature in Celsius, or -1 if no GPU thermal zone is found.
// Scans /sys/class/thermal for a zone whose `type` contains "gpu". Throttled by the
// caller -- this does a handful of small sysfs reads. (lobby_panels.cpp has a richer
// zoneTempC() but it's file-local; this is a self-contained copy for the perf path.)
static float gpuZoneTempC() {
    // Zone->type mapping is fixed after boot, so scan for the "gpu" zones ONCE and
    // cache their indices; subsequent calls only read the (few) temp nodes instead
    // of re-opening all 40 type files every second.
    static int  sGpuZones[8];
    static int  sGpuN = -1;
    if (sGpuN < 0) {
        sGpuN = 0;
        for (int z = 0; z < 40 && sGpuN < 8; z++) {
            char p[128], type[64] = {0};
            snprintf(p, sizeof(p), "/sys/class/thermal/thermal_zone%d/type", z);
            FILE *f = fopen(p, "r");
            if (!f) continue;                   // gap in numbering -> keep scanning
            bool got = fgets(type, sizeof(type), f) != nullptr;
            fclose(f);
            if (got && strstr(type, "gpu")) sGpuZones[sGpuN++] = z;
        }
    }
    long best = -1;
    for (int i = 0; i < sGpuN; i++) {
        char p[128];
        snprintf(p, sizeof(p), "/sys/class/thermal/thermal_zone%d/temp", sGpuZones[i]);
        FILE *f = fopen(p, "r");
        if (!f) continue;
        long t = 0; if (fscanf(f, "%ld", &t) == 1 && t > best) best = t;
        fclose(f);
    }
    return best < 0 ? -1.0f : (float)best / 1000.0f;
}

// THERMAL-ADAPTIVE perf level. We normally hard-pin CPU/GPU to kPerfLevelMax (removes
// the governor's ramp-up latency when cool -- tighter frame timing). But on this
// thermally limited SD845, sustained heavy worlds drive the GPU into the high 80s/90s
// where the governor HARD-clamps the clock anyway.
// Holding the request at MAX through that does nothing but add self-heat. So when the
// GPU crosses kHotC we drop the requested floor ONE notch (max-1); we restore to max
// once it falls back below kCoolC. The wide hysteresis band (kCoolC..kHotC) plus the
// ~1Hz sample cadence prevents level flapping. Backing off the REQUEST proactively
// eases the SoC before the governor's harder clamp -> smoother throttle, not a cliff.
static const float kHotC  = 88.0f;   // step down above this GPU temp
static const float kCoolC = 82.0f;   // restore to max below this (hysteresis)
static int desiredPerfLevel() {
    static bool   sBackedOff = false;
    static uint64_t sLastSample = 0;
    uint64_t now = nowNs();
    if (now - sLastSample > 1000000000ULL) {     // sample at ~1Hz; sysfs is cheap but not free
        sLastSample = now;
        float t = gpuZoneTempC();
        if (t > 0.0f) {
            if (!sBackedOff && t >= kHotC) {
                sBackedOff = true;
                LOGI("perf: GPU %.1fC >= %.1fC -- backing off perf floor to max-1 (thermal)", t, kHotC);
            } else if (sBackedOff && t <= kCoolC) {
                sBackedOff = false;
                LOGI("perf: GPU %.1fC <= %.1fC -- restoring max perf floor", t, kCoolC);
            }
        }
    }
    return sBackedOff ? (kPerfLevelPin - 1) : kPerfLevelPin;
}

// Optional GPU-only sub-cap below the CPU level. Currently DISABLED (== kPerfLevelMax)
// because CPU+GPU already pin to kPerfLevelPin (4) uniformly. Kept as a lever in case
// we later want the GPU a notch below the CPU.
static const int kGpuLevelCap = kPerfLevelMax;

// Apply a perf level (>=0 pins CPU to it + GPU to min(it,kGpuLevelCap); -1 releases
// to the captured baseline). Returns true once the perf service accepted it;
// SetCpuLevel returns -1 until the QVR service client is bound, so the caller retries.
static bool applyPerfLevels(int level) {
    capturePerfBaseline();
    bool on = (level >= 0);
    if (level > kPerfLevelMax) level = kPerfLevelMax;     // defensive clamp to valid range
    int wantCpu = on ? level : gPerfBaseCpu;
    int wantGpu = on ? (level < kGpuLevelCap ? level : kGpuLevelCap) : gPerfBaseGpu;
    int rc = SetCpuLevel(wantCpu, on);   // 2nd arg = sustained/static floor while pinned
    int rg = SetGpuLevel(wantGpu, on);
    if (rc < 0 || rg < 0) {              // perf service client not bound yet -> retry
        static bool warned = false;
        if (!warned) { warned = true; LOGI("perf: Set Cpu/Gpu Level not applied yet (rc=%d rg=%d) -- service client null; retrying", rc, rg); }
        return false;
    }
    int cl = -1, gl = -1; bool cs = false, gs = false;
    GetCpuLevel(cl, cs); GetGpuLevel(gl, gs);   // read back what actually took
    LOGI("perf: target=%d (%s) -> CPU=%d GPU=%d (rc=%d rg=%d; readback cpu=%d gpu=%d)",
         level, on ? "pin" : "release", wantCpu, wantGpu, rc, rg, cl, gl);
    return true;
}

// Fixed-rate tracking/uplink thread. Reads the head pose, derives filtered
// velocity, reads eye gaze + controllers, and pushes the whole tracking frame to
// ALVR -- decoupled from the render thread's frame-paced spin so the rate-stable
// filters hold their time constants and the uplink fires at a constant ~300Hz.
// Runs from just before the render loop until gTrackRunning clears. Calls only
// native SDK + ALVR-channel APIs (no JNI), so it does NOT attach to the JVM.
static void *trackingThread(void *) {
    // Keep this thread on the LITTLE cores -- off the gold cores the render + warp
    // threads contend for. It's only ~300Hz of light pose work, which
    // fits the little cluster comfortably, and this removes a scheduling rival from
    // the warp's cores plus cuts cross-core cache traffic on the head/ctrl mutexes
    // it shares with the render thread.
    {
        long n = sysconf(_SC_NPROCESSORS_CONF);
        if (n >= 2) {
            cpu_set_t set; CPU_ZERO(&set);
            for (long c = 0; c < n / 2; c++) CPU_SET((int)c, &set);   // little-core half
            if (sched_setaffinity(0, sizeof(set), &set) == 0)
                LOGI("tracking thread pinned to little cores [0..%ld]", n/2 - 1);
            else
                LOGI("tracking affinity failed (errno=%d)", errno);
        }
    }
    const long  kPeriodNs = 1000000000L / 300;   // ~300Hz fixed cadence
    // Decouple the UPLINK packet rate from the FILTER step rate. The EMA
    // velocity/blink filters must step at the full fixed 300Hz to hold their tuned
    // tau's, but the server only needs pose packets dense enough to extrapolate to
    // a 72Hz display -- ~150Hz is plenty (>2x refresh). So we step
    // every loop but only SEND every kUplinkDiv-th iteration, halving tracking
    // packets over the Wi-Fi link that also carries video (the same congestion
    // hazard the button-spam edge-gating already guards against). Button edges stay
    // event-driven (sent on change, not paced), so they're unaffected.
    const int   kUplinkDiv = 2;                  // 300Hz / 2 = ~150Hz uplink
    int  tframe = 0;
    // head-velocity filter state (was inline in the render loop)
    Quat     prevQ   = { 0, 0, 0, 1 };
    float    prevP[3] = { 0, 0, 0 };
    uint64_t prevTs  = 0;
    bool     havePrev = false;
    float    fLin[3] = { 0, 0, 0 };   // EMA-filtered linear velocity (m/s)
    float    fAng[3] = { 0, 0, 0 };   // EMA-filtered angular velocity (rad/s, world)
    float    ctrlOrigin[3] = { 0.0f, 1.6f, 0.0f };
    bool     ctrlOriginSet = false;
    // controller button edge-detect state (only emit deltas; see render loop note)
    bool  lastBin[2][4]  = {};
    bool  binInit[2]     = { false, false };
    float lastScal[2][4] = {};
    bool  scalInit[2]    = { false, false };
    // Controller linear-velocity filter state (per hand). The CV service has no
    // hardware linear velocity, so we differentiate the world position we send and
    // EMA-smooth it -- mirroring the head -- so the server can positionally
    // extrapolate the hands to display time. Stepped ONLY on a fresh CV sample: the
    // Java poller runs ~90Hz while we read at 300Hz, so the in-between reads are
    // bit-identical duplicates that would inject dt-noise / zero-velocity spikes.
    float    ctrlPrevP[2][3]  = {};     // last world pos used for differencing
    uint64_t ctrlPrevTs[2]    = { 0, 0 };
    bool     ctrlHavePrev[2]  = { false, false };
    float    ctrlRawP[2][3]   = {};     // last raw sample seen (new-sample detect)
    float    ctrlFLin[2][3]   = {};     // EMA-filtered CENTER linear velocity (m/s, pose frame)
    // Eye-gaze read cadence. Pvr_GetEyeTrackingData (35 out-params) is heavy and
    // readEyeGazes is stateless, so call it at ~100Hz (every 3rd 300Hz tick) and reuse
    // the cached gaze on the in-between ticks. The server consumes gaze at ~display
    // rate so 100Hz is ample; only matters on an EYE unit with IR on (otherwise the
    // call early-returns instantly anyway).
    AlvrPose eyeGazeCache[2] = {};
    bool     eyeVLCache = false, eyeVRCache = false;

    // Pace the loop against an ABSOLUTE deadline (clock_nanosleep TIMER_ABSTIME)
    // rather than a relative usleep of "period minus elapsed": a relative sleep lets
    // scheduling jitter + usleep oversleep accumulate, so the real rate drifts under
    // the nominal 300Hz. An absolute deadline advanced by exactly kPeriodNs each tick
    // holds a true 300Hz average -> the EMA filter tau's stay exact and the uplink
    // decimation (every kUplinkDiv-th tick) stays rate-stable.
    auto tsAddNs = [](struct timespec &t, long ns){
        t.tv_nsec += ns;
        while (t.tv_nsec >= 1000000000L) { t.tv_nsec -= 1000000000L; t.tv_sec += 1; }
    };
    struct timespec nextTick;
    clock_gettime(CLOCK_MONOTONIC, &nextTick);

    while (gTrackRunning.load()) {
        float qx=0,qy=0,qz=0,qw=1, px=0,py=0,pz=0, vfov=90, hfov=90; int viewNumber=0;
        Pvr_GetMainSensorState(&qx,&qy,&qz,&qw,&px,&py,&pz,&vfov,&hfov,&viewNumber);
        // Publish head pose for the Java controller poller (head-aligned CV call)
        // and for the render thread's lobby head transform.
        {
            std::lock_guard<std::mutex> lk(gHeadMutex);
            gHeadData[0]=qx; gHeadData[1]=qy; gHeadData[2]=qz; gHeadData[3]=qw;
            gHeadData[4]=px; gHeadData[5]=py; gHeadData[6]=pz;
        }
        if (!ctrlOriginSet && py > 0.5f) {
            ctrlOrigin[0] = px; ctrlOrigin[1] = py; ctrlOrigin[2] = pz;
            ctrlOriginSet = true;
            LOGI("controller world origin captured (%.2f,%.2f,%.2f)", px, py, pz);
        }

        AlvrDeviceMotion hmd = {};
        hmd.device_id = alvrHeadId;
        hmd.pose.orientation = { qx, qy, qz, qw };
        hmd.pose.position[0] = px; hmd.pose.position[1] = py; hmd.pose.position[2] = pz;

        const uint64_t ts = nowNs();

        // ---- Head-velocity prediction (server-side extrapolation) -----------
        {
            Quat curQ = quatNorm({ qx, qy, qz, qw });
            if (havePrev) {
                double dt = (double)(ts - prevTs) * 1e-9;
                if (dt > 0.002 && dt < 0.05) {        // 2ms..50ms; else skip this sample
                    // dt-normalized EMA (rate-stable): tau=0.66s, correct at the
                    // ~300Hz this thread runs at (independent of step rate).
                    const float a = 1.0f - expf(-(float)dt / 0.66f);
                    float lvx = (float)((px - prevP[0]) / dt);
                    float lvy = (float)((py - prevP[1]) / dt);
                    float lvz = (float)((pz - prevP[2]) / dt);
                    Quat dq = quatNorm(quatMul(curQ, quatConj(prevQ)));
                    if (dq.w < 0) { dq.x=-dq.x; dq.y=-dq.y; dq.z=-dq.z; dq.w=-dq.w; }
                    float w   = dq.w > 1.0f ? 1.0f : dq.w;
                    float ang = 2.0f * acosf(w);
                    float s   = sqrtf(1.0f - w*w);
                    float avx = 0, avy = 0, avz = 0;
                    if (s > 1e-6f) {
                        float k = (ang / s) / (float)dt;
                        avx = dq.x * k; avy = dq.y * k; avz = dq.z * k;
                    }
                    fLin[0] += (lvx - fLin[0]) * a; fLin[1] += (lvy - fLin[1]) * a; fLin[2] += (lvz - fLin[2]) * a;
                    fAng[0] += (avx - fAng[0]) * a; fAng[1] += (avy - fAng[1]) * a; fAng[2] += (avz - fAng[2]) * a;
                }
            }
            prevQ = curQ; prevP[0]=px; prevP[1]=py; prevP[2]=pz; prevTs = ts; havePrev = true;
            // Position: scale the reported linear velocity by the same predict factor
            // the controllers use (kBasePredict), so the head's positional extrapolation
            // reach matches the hands' instead of running at full 1.0. Rotation is left
            // unscaled -- the DIATW warp already reprojects orientation to the live pose
            // every vsync locally, so the server-side angular reach isn't the lever here.
            float hp = kBasePredict;
            if (hp < 0.0f) hp = 0.0f; else if (hp > 1.0f) hp = 1.0f;
            hmd.linear_velocity[0]  = fLin[0] * hp; hmd.linear_velocity[1]  = fLin[1] * hp; hmd.linear_velocity[2]  = fLin[2] * hp;
            hmd.angular_velocity[0] = fAng[0]; hmd.angular_velocity[1] = fAng[1]; hmd.angular_velocity[2] = fAng[2];
        }

        // Eye gaze (Neo 2 EYE); no-op on non-Eye units (returns false). Read at
        // ~100Hz and reuse the cache between reads (the in-between uplinks still carry
        // the latest gaze). The cached pose is global-space (composed with the headQ
        // at read time); ~10ms of head motion between reads is negligible for gaze OSC.
        AlvrPose eyeGaze[2];
        const AlvrPose *eyeGazePtr[2] = { nullptr, nullptr };
        const AlvrPose *const *eyeGazes = nullptr;
        const int kEyeDiv = 3;   // 300Hz / 3 = ~100Hz
        if ((tframe % kEyeDiv) == 0) {
            if (!readEyeGazes(eyeGazeCache, &eyeVLCache, &eyeVRCache, tframe, quatNorm({ qx, qy, qz, qw })))
                eyeVLCache = eyeVRCache = false;   // blink/invalid -> don't forward stale gaze
        }
        eyeGaze[0] = eyeGazeCache[0]; eyeGaze[1] = eyeGazeCache[1];
        bool vL = eyeVLCache, vR = eyeVRCache;
        if (vL) eyeGazePtr[0] = &eyeGaze[0];
        if (vR) eyeGazePtr[1] = &eyeGaze[1];
        if (vL || vR) eyeGazes = eyeGazePtr;

        // ---- Controllers: forward 6DoF (+touchpad) to ALVR (Neo 2 EYE) -------
        AlvrDeviceMotion motions[3];
        motions[0] = hmd;
        int motionCount = 1;
        CtrlState cs[2];
        { std::lock_guard<std::mutex> lk(gCtrlMutex); cs[0] = gCtrl[0]; cs[1] = gCtrl[1]; }
        // In the lobby (manual lobby over a live stream) suppress ALL controller
        // input to the server -- motions AND buttons -- so interacting with our
        // menu doesn't fire ghost actions in the SteamVR scene. HMD pose + eye data
        // still flow (not "actions"). Only the HMD motion is forwarded.
        bool inLobby = gManualLobby.load();
        for (int h = 0; h < 2 && !inLobby; h++) {
            if (!cs[h].fresh || cs[h].conn != 1) continue;
            AlvrDeviceMotion &m = motions[motionCount++];
            m = {};
            m.device_id = alvrHandId[h];
            Quat oq = { -cs[h].q[0], -cs[h].q[1], cs[h].q[2], cs[h].q[3] };
            float kYawOutDeg = kBaseYawDeg;   // baked baseline
            float yaw = (h == 0 ? +kYawOutDeg : -kYawOutDeg) * 0.01745329f;
            Quat qYaw = { 0.0f, sinf(yaw*0.5f), 0.0f, cosf(yaw*0.5f) };
            oq = quatNorm(quatMul(oq, qYaw));
            // Controller rotation offset, baked from the ALVR dashboard "Left
            // controller rotation offset" (XYZ euler degrees). The server applies it
            // as a final LOCAL post-multiply (Quat::from_euler(XYZ); see server_core
            // tracking/mod.rs) with the RIGHT hand mirrored on Y and Z (X unchanged).
            // We replicate it here so it lives in client code -- keep the dashboard
            // values at 0 to avoid double-applying. On TOP of the kYawOutDeg term.
            float kRotOffXDeg = kBaseRotXDeg;   // baseline; left X (right uses same)
            float kRotOffYDeg = kBaseRotYDeg;   // baseline; left Y (right negated)
            static const float kRotOffZDeg =  0.0f;    // left Z (right negated)
            float rx = kRotOffXDeg * 0.01745329f;
            float ry = (h == 0 ? kRotOffYDeg : -kRotOffYDeg) * 0.01745329f;
            float rz = (h == 0 ? kRotOffZDeg : -kRotOffZDeg) * 0.01745329f;
            Quat qrx = { sinf(rx*0.5f), 0.0f, 0.0f, cosf(rx*0.5f) };
            Quat qry = { 0.0f, sinf(ry*0.5f), 0.0f, cosf(ry*0.5f) };
            Quat qrz = { 0.0f, 0.0f, sinf(rz*0.5f), cosf(rz*0.5f) };
            Quat qRotOff = quatMul(quatMul(qrx, qry), qrz);   // from_euler(XYZ)
            oq = quatNorm(quatMul(oq, qRotOff));
            m.pose.orientation = { oq.x, oq.y, oq.z, oq.w };
            // Grip offset = a FIXED point on the physical controller, so it lives in
            // the controller's LOCAL frame and must rotate with it. (Was applied in
            // world space, which kept the offset world-aligned while the controller
            // rotated -> the reported origin drifted off the real one along whatever
            // axis you pitched/rolled.) Rotate the local offset by the SAME reported
            // orientation, then add to the raw tracked position. Local axes (in the
            // controller's own frame): X = sideways, Y = up/down, Z = front/back.
            //   +X = toward the controller's RIGHT side   / -X = toward its LEFT
            //   +Y = up toward the buttons (top face)      / -Y = down toward the trigger
            //   +Z = back toward the wrist                 / -Z = forward toward the tip
            // kGripSideLocal is mirrored for the right hand (the two controllers are
            // mirror images), like kYawOutDeg above; the other two apply to both hands.
            float kGripSideMm = kBaseGripSideMm, kGripUpMmV = kBaseGripUpMm, kGripBackMmV = kBaseGripBackMm;
            float kGripSideLocal = kGripSideMm * 0.001f;  // baseline; X
            float kGripUpLocal   = kGripUpMmV  * 0.001f;  // Y
            float kGripBackLocal = kGripBackMmV * 0.001f; // Z
            float sideSign = (h == 0) ? +1.0f : -1.0f;    // mirror X on the right hand
            float gripLocal[3] = { kGripSideLocal * sideSign, kGripUpLocal, kGripBackLocal };
            // ROT SWING: split the offset into an ORBITING part (rotated by the full
            // controller orientation -> swings with pitch/roll, the felt pivot sits
            // at the SDK tracked point) and a LEVEL part (rotated by heading ONLY ->
            // stays gravity-level, so the controller spins more in place). swing=1 is
            // fully rigid (old behavior); swing=0 removes pitch/roll orbit. At a level,
            // forward pose the two frames coincide, so the rest position is unchanged.
            float swing = kBaseRotSwing;   // baseline
            if (swing < 0.0f) swing = 0.0f; else if (swing > 1.0f) swing = 1.0f;
            // Heading-only quaternion = twist of oq about the up (Y) axis (swing-twist).
            float yn = sqrtf(oq.y*oq.y + oq.w*oq.w);
            Quat oqYaw = (yn > 1e-6f) ? Quat{ 0.0f, oq.y/yn, 0.0f, oq.w/yn }
                                      : Quat{ 0.0f, 0.0f, 0.0f, 1.0f };
            float gripOrbit[3] = { gripLocal[0]*swing,        gripLocal[1]*swing,        gripLocal[2]*swing };
            float gripLevel[3] = { gripLocal[0]*(1.0f-swing), gripLocal[1]*(1.0f-swing), gripLocal[2]*(1.0f-swing) };
            float wOrbit[3], wLevel[3];
            quatRotateVec(oq,    gripOrbit, wOrbit);
            quatRotateVec(oqYaw, gripLevel, wLevel);
            float gripWorld[3] = { wOrbit[0]+wLevel[0], wOrbit[1]+wLevel[1], wOrbit[2]+wLevel[2] };
            float wpx = cs[h].pos[0]*0.001f + gripWorld[0];
            float wpy = cs[h].pos[1]*0.001f + gripWorld[1];
            float wpz = cs[h].pos[2]*0.001f + gripWorld[2];
            m.pose.position[0] = wpx;
            m.pose.position[1] = wpy;
            m.pose.position[2] = wpz;
            // Angular velocity: hardware gyro, re-expressed into the converted pose
            // frame. The pose orientation is conjugated by a 180deg Z flip (the
            // (-x,-y,z,w) mapping), so a world-frame angular-velocity vector maps the
            // same way: (wx,wy,wz) -> (-wx,-wy,wz). Forwarding it raw would fight the
            // converted pose during fast turns.
            // Prediction strength: the server extrapolates pose + velocity*dt, so
            // scaling the reported velocities scales the prediction reach. 0.75 backs
            // off ~25% -- the raw rates overshoot on quick stops/flicks.
            float kCtrlPredict = kBasePredict;   // baseline
            if (kCtrlPredict < 0.0f) kCtrlPredict = 0.0f; else if (kCtrlPredict > 1.0f) kCtrlPredict = 1.0f;
            // Angular velocity: hardware gyro re-expressed into the converted pose
            // frame (-wx,-wy,wz to match the (-x,-y,z,w) orientation flip). This SAME
            // omega also drives the grip lever's linear velocity below, so position
            // and rotation extrapolate by exactly the same amount (no shear).
            float omega[3] = { -cs[h].angVel[0], -cs[h].angVel[1], cs[h].angVel[2] };
            m.angular_velocity[0] = omega[0] * kCtrlPredict;
            m.angular_velocity[1] = omega[1] * kCtrlPredict;
            m.angular_velocity[2] = omega[2] * kCtrlPredict;
            // Linear velocity = velocity of the Pico CENTER + omega x lever. The
            // center term is differentiated (smoothed); the lever-swing term is
            // computed ANALYTICALLY from the same omega we send, so the rotation
            // prediction and the lever's positional prediction stay locked together
            // (numerically differentiating the whole position incl. the lever would
            // lag the crisp gyro rotation and shear the rigid body during fast turns).
            float leverVel[3] = {
                omega[1]*wOrbit[2] - omega[2]*wOrbit[1],
                omega[2]*wOrbit[0] - omega[0]*wOrbit[2],
                omega[0]*wOrbit[1] - omega[1]*wOrbit[0],
            };
            float cx = cs[h].pos[0]*0.001f, cy = cs[h].pos[1]*0.001f, cz = cs[h].pos[2]*0.001f;
            bool ctrlNew = !ctrlHavePrev[h] ||
                cs[h].pos[0]!=ctrlRawP[h][0] || cs[h].pos[1]!=ctrlRawP[h][1] ||
                cs[h].pos[2]!=ctrlRawP[h][2];
            if (ctrlNew) {
                if (ctrlHavePrev[h]) {
                    double dt = (double)(ts - ctrlPrevTs[h]) * 1e-9;
                    if (dt > 0.002 && dt < 0.05) {
                        // tau=0.05s: light smoothing (~one CV sample period) on the
                        // center translation -- the lever term is unfiltered (analytic).
                        const float a = 1.0f - expf(-(float)dt / 0.05f);
                        float lvx = (float)((cx - ctrlPrevP[h][0]) / dt);
                        float lvy = (float)((cy - ctrlPrevP[h][1]) / dt);
                        float lvz = (float)((cz - ctrlPrevP[h][2]) / dt);
                        ctrlFLin[h][0] += (lvx - ctrlFLin[h][0]) * a;
                        ctrlFLin[h][1] += (lvy - ctrlFLin[h][1]) * a;
                        ctrlFLin[h][2] += (lvz - ctrlFLin[h][2]) * a;
                    }
                }
                ctrlPrevP[h][0]=cx; ctrlPrevP[h][1]=cy; ctrlPrevP[h][2]=cz;
                ctrlPrevTs[h]=ts; ctrlHavePrev[h]=true;
                ctrlRawP[h][0]=cs[h].pos[0]; ctrlRawP[h][1]=cs[h].pos[1]; ctrlRawP[h][2]=cs[h].pos[2];
            }
            m.linear_velocity[0] = (ctrlFLin[h][0] + leverVel[0]) * kCtrlPredict;
            m.linear_velocity[1] = (ctrlFLin[h][1] + leverVel[1]) * kCtrlPredict;
            m.linear_velocity[2] = (ctrlFLin[h][2] + leverVel[2]) * kCtrlPredict;
        }
        if ((tframe % 38) == 0) {
            for (int h = 0; h < 2; h++) {
                if (!cs[h].fresh || cs[h].conn != 1) continue;
                LOGI("CTRLALIGN head p=(%.3f,%.3f,%.3f) q=(%.3f,%.3f,%.3f,%.3f) | hand%d RAW pmm=(%.0f,%.0f,%.0f) q=(%.3f,%.3f,%.3f,%.3f)",
                     px, py, pz, qx, qy, qz, qw,
                     h, cs[h].pos[0], cs[h].pos[1], cs[h].pos[2],
                     cs[h].q[0], cs[h].q[1], cs[h].q[2], cs[h].q[3]);
            }
        }

        // Send pose+gaze every kUplinkDiv-th step (~150Hz); the filters above
        // already stepped this iteration regardless, so decimating only the SEND
        // keeps them rate-stable while halving the uplink packet rate.
        bool doUplink = (tframe % kUplinkDiv) == 0;

        // Forward eye blink (per-eye openness) once eye tracking has data. The
        // openness is latched to be sent with the NEXT alvr_send_tracking, so only
        // push it on uplink steps (the smoothing itself still runs every iteration).
        if (gEyeHaveOpen) {
            // dt-normalized blink smoothing (tau=0.011s ~= alpha 0.70 at 72Hz).
            static uint64_t sBlinkPrevTs = 0;
            float kBlinkAlpha = 1.0f;
            if (sBlinkPrevTs != 0) {
                double bdt = (double)(ts - sBlinkPrevTs) * 1e-9;
                if (bdt > 0.0 && bdt < 0.05) kBlinkAlpha = 1.0f - expf(-(float)bdt / 0.011f);
            }
            sBlinkPrevTs = ts;
            gEyeOpenSmooth[0] += (gEyeOpen[0] - gEyeOpenSmooth[0]) * kBlinkAlpha;
            gEyeOpenSmooth[1] += (gEyeOpen[1] - gEyeOpenSmooth[1]) * kBlinkAlpha;
            if (doUplink) alvr_send_eye_openness(gEyeOpenSmooth[0], gEyeOpenSmooth[1]);
        }
        if (doUplink) alvr_send_tracking(ts, motions, motionCount, nullptr, eyeGazes);

        // ---- Controller buttons: send only on CHANGE (edge) ------------------
        auto sendBinEdge = [&](int h, int i, uint64_t id, bool v) {
            if (binInit[h] && lastBin[h][i] == v) return;
            lastBin[h][i] = v;
            AlvrButtonValue b; b.tag = ALVR_BUTTON_VALUE_BINARY; b.binary = v;
            alvr_send_button(id, b);
        };
        auto sendScalEdge = [&](int h, int i, uint64_t id, float v) {
            if (scalInit[h] && lastScal[h][i] == v) return;
            lastScal[h][i] = v;
            AlvrButtonValue b; b.tag = ALVR_BUTTON_VALUE_SCALAR; b.scalar = v;
            alvr_send_button(id, b);
        };
        // Thumbstick response curve. The Neo 2 stick reports LINEARLY, which feels
        // touchy: a small deflection already commands a big value, so locomotion
        // ramps up fast. Reshape with an expo (blend of cubic + linear) so small
        // inputs are gentle while full throw still reaches 1.0. Plus a center
        // deadzone (resting drift) AND an outer saturation band: the stick can't
        // physically reach its reported max, so the last kStickOuter of travel is
        // dead -- clamp anything past (1 - kStickOuter) to full 1.0 and rescale the
        // ramp into the usable [kStickDead, 1 - kStickOuter] band. Tuning:
        //   kStickExpo  0=linear .. 1=pure cubic (more = more relaxed near center)
        //   kStickDead  fraction of travel ignored at center
        //   kStickOuter fraction of travel at the edge that all reads as 100%
        static const float kStickExpo  = 0.6f;
        static const float kStickDead  = 0.06f;
        static const float kStickOuter = 0.20f;
        auto stickCurve = [](float x) -> float {
            float s = (x < 0.0f) ? -1.0f : 1.0f;
            float a = fabsf(x); if (a > 1.0f) a = 1.0f;
            if (a <= kStickDead) return 0.0f;
            float top = 1.0f - kStickOuter;
            if (a >= top) return s * 1.0f;                 // outer band -> full deflection
            a = (a - kStickDead) / (top - kStickDead);     // rescale usable travel to [0,1]
            float curved = kStickExpo * (a*a*a) + (1.0f - kStickExpo) * a;
            return s * curved;
        };
        for (int h = 0; h < 2 && !inLobby; h++) {
            if (!cs[h].fresh || cs[h].conn != 1 || cs[h].keyCount < 10) continue;
            const int *k = cs[h].keys;
            if (k[2]||k[3]||k[4]||k[5]||k[6]||k[7])
                LOGI("BTN[%d] trig=%d grip=%d joyClk=%d menu=%d A/X=%d B/Y=%d (joy %d,%d trigA=%d gripA=%d)",
                     h, k[2],k[3],k[4],k[5],k[6],k[7], k[0],k[1],k[8],k[9]);
            // The stick reports a SQUARE range: a full diagonal gives x~1 AND y~1,
            // a vector of radius sqrt(2). SteamVR expects a CIRCULAR stick (radius
            // <= 1), so trim the square corners to the unit circle -- clamp the
            // post-curve vector's magnitude to 1 (cardinals untouched; only the
            // diagonal corner is pulled in to the rim). The per-axis response curve
            // above (deadzone/expo/outer band) is preserved.
            float stx = stickCurve((k[0] - 128) / 128.0f);
            float sty = stickCurve((k[1] - 128) / 128.0f);
            float mag2 = stx*stx + sty*sty;
            if (mag2 > 1.0f) { float inv = 1.0f / sqrtf(mag2); stx *= inv; sty *= inv; }
            sendScalEdge(h, 0, alvrBtn[h].thumbX, stx);
            sendScalEdge(h, 1, alvrBtn[h].thumbY, sty);
            float trig = k[8] / 255.0f; if (trig < 0) trig = 0; if (trig > 1) trig = 1;
            sendScalEdge(h, 2, alvrBtn[h].trigVal, trig);
            float grip = k[9] / 255.0f; if (grip < 0) grip = 0; if (grip > 1) grip = 1;
            if (grip == 0.0f && k[3]) grip = 1.0f;
            sendScalEdge(h, 3, alvrBtn[h].gripVal, grip);
            sendBinEdge(h, 0, alvrBtn[h].thumbClick, k[4] != 0);
            sendBinEdge(h, 1, alvrBtn[h].menu,       k[5] != 0);
            sendBinEdge(h, 2, alvrBtn[h].face1,      k[6] != 0);
            sendBinEdge(h, 3, alvrBtn[h].face2,      k[7] != 0);
            binInit[h] = true;
            scalInit[h] = true;
        }

        tframe++;
        // Advance the absolute deadline by exactly one period and sleep to it. If
        // we overran (deadline already past), re-anchor to now instead of firing a
        // catch-up burst of zero-length iterations (which would spike the uplink rate
        // and inject tiny-dt samples into the EMA filters).
        tsAddNs(nextTick, kPeriodNs);
        struct timespec nowT;
        clock_gettime(CLOCK_MONOTONIC, &nowT);
        if (nowT.tv_sec > nextTick.tv_sec ||
            (nowT.tv_sec == nextTick.tv_sec && nowT.tv_nsec > nextTick.tv_nsec)) {
            nextTick = nowT;   // overran -> re-anchor; no sleep this iteration
        } else {
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &nextTick, nullptr);
        }
    }
    LOGI("tracking thread exit at tframe %d", tframe);
    return nullptr;
}

// Apply HMD panel brightness from a 0..1 slider fraction. Primary path is the Pico
// SDK backlight control (SetHmdScreenBrightness) -- it has a readback and drives the
// panel directly; we read it back and, if it didn't take, fall back to the Android
// window-brightness API via a Java upcall. `env` is the render thread's JNIEnv.
static void applyHmdBrightness(float frac, JNIEnv *env) {
    if (frac < 0.0f) frac = 0.0f; else if (frac > 1.0f) frac = 1.0f;
    int level = kBrightMin + (int)(frac * (float)(kBrightMax - kBrightMin) + 0.5f);
    SetHmdScreenBrightness(level);
    int rb = -1; GetHmdScreenBrightness(rb);
    int diff = rb - level; if (diff < 0) diff = -diff;
    bool pvrOk = (rb >= 0 && diff <= 32);
    if (!pvrOk && env && gActivity) {   // Pico path inert -> Android window brightness
        jclass cls = env->GetObjectClass(gActivity);
        jmethodID m = cls ? env->GetMethodID(cls, "setWindowBrightness", "(F)V") : nullptr;
        if (m) env->CallVoidMethod(gActivity, m, (jfloat) frac);
        if (cls) env->DeleteLocalRef(cls);
        if (env->ExceptionCheck()) env->ExceptionClear();
        LOGI("brightness: Pvr readback=%d (wanted %d) -> Android fallback %.2f", rb, level, frac);
    } else {
        LOGI("brightness: Pvr set level=%d (readback=%d)", level, rb);
    }
}

void *renderThread(void *) {
    // Reset file-scope statics that survive a nativeStop->nativeStart in the same
    // process: a relaunch builds a fresh EGL context, so any "already done" latch
    // left true would skip the re-setup (warp not re-given the window, ATW not
    // re-enabled, perf baseline never recaptured).
    gWarpToWindow = false;
    gAtwEnabled = false;
    gPerfBaseCaptured = false;
    JNIEnv *env = nullptr;
    gVM->AttachCurrentThread(&env, nullptr);
    int reservedCpu = pinSubmitThreadForLowLatency();   // big-core affinity + prio; returns the core reserved for the warp
    bool warpPinned = false;          // warp thread pinned once it exists (see loop)

    EGLDisplay dpy = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    eglInitialize(dpy, nullptr, nullptr);
    const EGLint cfgAttribs[] = {
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT,
        EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
        EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8,
        EGL_DEPTH_SIZE, 24, EGL_NONE
    };
    EGLConfig cfg; EGLint n = 0;
    eglChooseConfig(dpy, cfgAttribs, &cfg, 1, &n);
    const EGLint ctxAttribs[] = { EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE };
    // EGL_IMG_context_priority: the GPU scheduler preempts in favour of higher-
    // priority contexts. The async warp present is time-critical (must finish
    // within each ~13.9ms refresh), so it gets HIGH; our render/encode work gets
    // LOW so the GPU yields to the warp instead of starving it (= "frame from the
    // past" stutter when our decode+reconstruct overruns a vsync). Best-effort:
    // the driver may clamp the level, but Adreno honours these.
    #ifndef EGL_CONTEXT_PRIORITY_LEVEL_IMG
    #define EGL_CONTEXT_PRIORITY_LEVEL_IMG 0x3100
    #define EGL_CONTEXT_PRIORITY_HIGH_IMG  0x3101
    #define EGL_CONTEXT_PRIORITY_MEDIUM_IMG 0x3102
    #define EGL_CONTEXT_PRIORITY_LOW_IMG   0x3103
    #endif
    const EGLint ctxAttribsLow[] = { EGL_CONTEXT_CLIENT_VERSION, 3,
        EGL_CONTEXT_PRIORITY_LEVEL_IMG, EGL_CONTEXT_PRIORITY_LOW_IMG, EGL_NONE };
    const EGLint ctxAttribsHigh[] = { EGL_CONTEXT_CLIENT_VERSION, 3,
        EGL_CONTEXT_PRIORITY_LEVEL_IMG, EGL_CONTEXT_PRIORITY_HIGH_IMG, EGL_NONE };
    EGLContext ctx = eglCreateContext(dpy, cfg, EGL_NO_CONTEXT, ctxAttribs);

    // SDK init + GL resources are created once, on a tiny pbuffer so the GL
    // context is valid before the first window surface exists. They survive
    // surface destroy/recreate because the context itself is never destroyed.
    const EGLint pbufAttribs[] = { EGL_WIDTH, 16, EGL_HEIGHT, 16, EGL_NONE };
    EGLSurface pbuf = eglCreatePbufferSurface(dpy, cfg, pbufAttribs);
    eglMakeCurrent(dpy, pbuf, pbuf, ctx);

    Pvr_SetInitActivity((void *) gActivity, (void *) gVrClass);
    Pvr_Enable6DofModule(true);                 // enable 6DoF before init/start
    int initRc = Pvr_Init(0);
    int sensRc = InitSensor();                  // select sensor type (else identity pose)
    int startRc = Pvr_StartSensor(0);
    LOGI("Pvr_Enable6DofModule(true); Pvr_Init=%d InitSensor=%d Pvr_StartSensor=%d",
         initRc, sensRc, startRc);
    // Detect Neo 2 EYE support but leave the IR illuminators OFF for now (POSITION-
    // only). We only light them up once a stream connects with the server's Face
    // Tracking eye source enabled -- no point burning power/heat on gaze the server
    // won't consume. See applyServerEyeTracking() on STREAMING_STARTED/STOPPED.
    initEyeTrackingMode();
    refreshDeviceIp();   // prime the lobby HUD IP (also re-read periodically in lobby)
    readHeadsetModel(env);
    LOGI("device IP = %s, model = %s", gIpText, gModelText);

    // Pick the tracking origin to mirror the server's tracking space, BEFORE we
    // tear the boundary system down (StageLevel needs the live guardian):
    //  - Guardian configured -> StageLevel (2): origin anchored to the room/stage,
    //    the Meta-STAGE analog. Pair with the server's recentering = Disabled
    //    (room-scale) for true 1:1 room tracking.
    //  - No guardian -> FloorLevel (1): floor Y under the recenter point (seated).
    //    Pair with the server's default LocalFloor recentering.
    // Either way py includes the floor height, so you never spawn in the floor.
    bool guardian = Pvr_BoundaryGetConfigured();
    int  originType = guardian ? 2 /* StageLevel */ : 1 /* FloorLevel */;
    bool originRc = Pvr_SetTrackingOriginType(originType);
    LOGI("Pvr_SetTrackingOriginType(%s)=%d floorHeight=%.3f guardian=%d",
         guardian ? "StageLevel" : "FloorLevel", originRc, Pvr_GetFloorHeight(),
         guardian);

    // Capture the play-area extents (meters) BEFORE we tear the boundary system
    // down -- we forward them to SteamVR as the chaperone via alvr_send_playspace
    // once a stream starts. Best-effort: if the user never set a guardian, this
    // stays unconfigured and we send nothing (server keeps its default).
    if (guardian) {
        float bx = 0, by = 0, bz = 0;
        int n = Pvr_BoundaryGetDimensions(&bx, &by, &bz, true /* play area */);
        if (n > 0 && bx > 0.05f && bz > 0.05f) {
            gPlayspaceW = bx; gPlayspaceD = bz;
        }
        LOGI("boundary configured: dims=%.2fx%.2f (n=%d) -> playspace %.2fx%.2f",
             bx, bz, n, gPlayspaceW, gPlayspaceD);
    } else {
        LOGI("boundary not configured; no playspace to forward");
    }

    Pvr_DisableBoundary();
    Pvr_ShutdownSDKBoundary();

    // Render IPD = world-scale knob, now USER-ADJUSTABLE in the lobby (gSoftIpdMm,
    // persisted) instead of hardcoded. Default 65mm; loaded from disk below.
    // Device reports ~63mm via Pvr_GetIPD(); softIpdM() is the live value used by
    // the lobby render, the warp submit pose, and the ALVR view_params.
    LOGI("render IPD = %.4f m (software, adjustable); device reports %.4f",
         softIpdM(), Pvr_GetIPD());

    // ---- ALVR client_core init -------------------------------------------
    // Context must be the Activity (a Context); JavaVM from JNI. After this the
    // client searches for an ALVR PC server on the LAN (mDNS) and connects.
    setHomeFromFilesDir(env, gActivity);   // before any ALVR storage access
    loadAllConfig();                        // restore ALL persisted settings (single $HOME/config.txt)
    applyUiTheme(gThemeAmber.load());        // apply the restored lobby UI theme
    if (gBrightnessSaved.load()) {
        applyHmdBrightness(gBrightnessFrac.load(), env);   // re-apply the saved level
    } else {
        // No saved value: seed the slider from the panel's current backlight so it
        // reflects reality, and leave the brightness untouched.
        int cur = -1; GetHmdScreenBrightness(cur);
        if (cur >= 0) {
            float fr = (float)(cur - kBrightMin) / (float)(kBrightMax - kBrightMin);
            if (fr < 0.0f) fr = 0.0f; else if (fr > 1.0f) fr = 1.0f;
            gBrightnessFrac.store(fr);
            LOGI("brightness: panel default level=%d -> slider %.2f", cur, fr);
        }
    }
    pushEqGains();                          // apply the restored EQ (loaded by loadAllConfig) to the DSP
    LOGI("alvr: calling initialize_logging"); alvr_initialize_logging();
    LOGI("alvr: calling initialize_android_context");
    alvr_initialize_android_context((void *) gVM, (void *) gActivity);
    LOGI("alvr: android_context done");
    // RECORD_AUDIO is a runtime (dangerous) permission on Android 10; without it
    // the mic-to-PC stream stays silent on a fresh install. Surface the system
    // grant dialog via the activity so the user doesn't have to adb-grant it.
    alvr_try_get_permission("android.permission.RECORD_AUDIO");
    // Pico Neo 2 (Eye) panel scans out at exactly 72Hz -- the HWComposer vsync
    // period is 13888888 ns = 72.0000Hz, with a single DSI-1
    // mode; there is NO 75Hz mode anywhere in the display stack (the commonly cited
    // "75Hz" spec does not match this unit). Advertise only the real native rate so
    // the stream paces to the true vsync -- a wrong rate just desyncs the warp.
    const float kRefreshRates[] = { 72.0f };
    AlvrClientCapabilities caps = {};
    // SQUARE per-eye render -> matches the Pico's square eye buffer (1664x1664)
    // and our SQUARE per-eye FOV (101deg H == V). A non-square buffer with a
    // square FOV squishes content horizontally (wrong depth/proportions). Square
    // also cuts pixel rate vs 2144x2400, easing the SD845 decoder.
    // NB: the distortion pipeline assumes a ~square per-eye buffer. Changing the
    // server resolution to a non-square aspect (e.g. 2560x1280 = 2:1) breaks the
    // geometry. Reduce decode load via foveated encoding, NOT raw resolution.
    caps.default_view_width  = 1664;
    caps.default_view_height = 1664;
    caps.refresh_rates       = kRefreshRates;
    caps.refresh_rates_count = 1;
    caps.foveated_encoding   = true;   // we de-foveate in alvr_render_stream_opengl
    caps.encoder_high_profile= true;
    caps.encoder_10_bits     = false;
    caps.encoder_av1         = false;   // SD845 Venus: H.264/HEVC only, no AV1
    caps.prefer_10bit        = false;
    caps.prefer_full_range   = true;
    caps.preferred_encoding_gamma = 1.0f;
    caps.prefer_hdr          = false;
    LOGI("alvr: calling initialize"); alvr_initialize(caps);
    LOGI("alvr: calling resume"); alvr_resume();
    alvrHeadId = alvr_path_string_to_id("/user/head");
    alvrHandId[0] = alvr_path_string_to_id("/user/hand/left");
    alvrHandId[1] = alvr_path_string_to_id("/user/hand/right");
    LOGI("ALVR initialized + resumed; head id=%llu", (unsigned long long) alvrHeadId);
    // Controller input path ids (per hand). Neo 2 CV2 = thumbstick wand: trigger
    // (click+analog value), grip, thumbstick (x/y/click), menu, and two face buttons
    // (A/B on the right hand, X/Y on the left). Path strings MUST match what the ALVR
    // v20 server registers in Paths.cpp or the button is dropped.
    const char *kHand[2] = { "/user/hand/left", "/user/hand/right" };
    for (int h = 0; h < 2; h++) {
        char p[128];
        #define PID(f, sfx) snprintf(p,sizeof(p),"%s%s",kHand[h],sfx); alvrBtn[h].f = alvr_path_string_to_id(p)
        // Paths must match the ALVR server's Quest source profile (input_mapping.rs
        // automatic_bindings) or the button is "not mapped" and dropped:
        //  - grip: squeeze/VALUE only (squeeze/click not a Quest source; value auto-
        //    derives the click via hysteresis).
        //  - trigger: value only (auto-derives click); trigger/click not a source.
        //  - menu: menu/click (system/click is never read as a source).
        PID(trigVal,    "/input/trigger/value");
        PID(gripVal,    "/input/squeeze/value");
        PID(thumbX,     "/input/thumbstick/x");
        PID(thumbY,     "/input/thumbstick/y");
        PID(thumbClick, "/input/thumbstick/click");
        PID(menu,       "/input/menu/click");
        // Face buttons: right = a/b, left = x/y (the server registers both sets on the
        // left hand but only a/b on the right, so pick per hand).
        PID(face1, h == 1 ? "/input/a/click" : "/input/x/click");
        PID(face2, h == 1 ? "/input/b/click" : "/input/y/click");
        #undef PID
    }

    // ALVR's GL renderer (wgpu) creates its OWN, separate EGL context inside
    // alvr_initialize_opengl(). It is NOT our context, so GL objects (our
    // swapchain textures) are invisible across the boundary -> ALVR's FBO would
    // reference a "texture is NULL" and abort. Fix: after ALVR makes its context,
    // recreate OUR rendering context as a SHARE context of ALVR's, so texture
    // names are common to both. ALL our GL resources (swapchain, lobby targets) are
    // then built in this shared context.
    alvr_initialize_opengl();           // creates + makes current ALVR's GL context
    EGLContext alvrCtx = eglGetCurrentContext();
    EGLDisplay alvrDpy = eglGetCurrentDisplay();
    if (alvrCtx == EGL_NO_CONTEXT) {
        LOGE("alvr_initialize_opengl left no current context!");
    } else {
        if (alvrDpy != EGL_NO_DISPLAY && alvrDpy != dpy) {
            LOGI("ALVR uses a different EGLDisplay; adopting it for sharing");
            dpy = alvrDpy;
            pbuf = eglCreatePbufferSurface(dpy, cfg, pbufAttribs);
        }
        EGLContext shared = eglCreateContext(dpy, cfg, alvrCtx, ctxAttribsLow);
        if (shared == EGL_NO_CONTEXT) {
            LOGE("eglCreateContext(shared with ALVR) failed 0x%x", eglGetError());
        } else {
            eglMakeCurrent(dpy, pbuf, pbuf, shared);
            eglDestroyContext(dpy, ctx);   // drop the bootstrap context
            ctx = shared;
            LOGI("created shared GL context %p (shares ALVR ctx %p)", shared, alvrCtx);
        }
    }
    gAlvrGlReady = true;
    LOGI("alvr_initialize_opengl done");

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    buildGraphics();
    buildLobbyTarget();      // per-eye ring the lobby HUD renders into for the SDK warp
    buildGazeMarker();       // eye-gaze debug disc (shown only on Neo 2 EYE)
    buildTextBuffers();      // dynamic VBO for the lobby IP/status HUD text + slider
    buildReticle();          // head-gaze crosshair (shown in lobby, no controllers)
    buildGridFloor();        // lobby floor grid (spatial reference, not a void)
    buildControllerMeshes(); // Neo 2 controller wireframes (from /system OBJ)

    RenderEventFunc re = (RenderEventFunc) GetRenderEventFunc();
    LOGI("GetRenderEventFunc=%p", (void *) re);

    ANativeWindow *curWin = nullptr;            // window this surface was made from
    EGLSurface     sfc    = EGL_NO_SURFACE;
    int  winW = 0, winH = 0;
    bool vrStarted = false;
    bool rtInited  = false;                      // SDK render thread init (tracking pipe)
    EGLContext warpCtx = EGL_NO_CONTEXT;         // persistent: the warp thread's ctx (HW mode) so we can RE-POINT it to a new surface on resume
    int  frame = 0, framesWithSurface = 0;
    // Lobby HUD anchor: the IP/status panel is WORLD-locked (not head-tracked).
    // We capture a world transform once on lobby (re)entry -- planted in front of
    // wherever the user is looking at that moment -- so it stays put as they look
    // around. Re-anchored whenever a stream ends (see reset in the video block).
    bool hudAnchored = false;
    Mat4 hudWorld = mat4Identity();
    Mat4 settingsWorld = mat4Identity(); // unified SETTINGS window, anchored in front (overlays the info HUD)
    int  lobbyEyeIdx = 0;   // HW-compositor lobby eye-texture ring index
    // Render-on-demand: re-render the lobby eyes only when content changes; the
    // warp reprojects the last frame to live head pose in between. We submit the
    // POSE THE TEXTURES WERE RENDERED AT so reprojection stays correct on reused
    // frames. lastRenderedIdx = -1 forces a first render.
    int  lastRenderedIdx = -1;   // slot rendered LAST frame -> the warp's submit target this frame (pipelined; -1 = none yet)
    int  lobbySubmitIdx  = 0;    // slot currently handed to the warp
    // Countdown to DEFER freeing the lobby eye-texture ring after a stream resumes.
    // The free must NOT happen the instant we leave the lobby: the SDK warp thread
    // free-runs and is STILL re-sampling the last lobby submit's textures every vsync.
    // Deleting them out from under it (glDeleteTextures while the GPU samples them,
    // the name then reused by the next stream allocation) corrupts the warp ring and
    // crashes the client after a few manual-lobby<->stream toggles. So wait until the
    // warp ring (4 entries) has fully cycled to fresh stream textures before freeing.
    // -1 = no free pending.
    int  lobbyFreeDelay = -1;
    // gGridThemeDirty is now a global atomic (app_state.h): the menu raises it, the
    // render loop consumes it below to rebuild the floor VBO.

    // Spin up the fixed-rate tracking/uplink thread now that the SDK + ALVR are
    // initialized and the path ids are populated. It runs independently of this
    // render loop and is joined after the loop exits.
    gTrackRunning.store(true);
    pthread_create(&gTrackThread, nullptr, trackingThread, nullptr);
    LOGI("tracking thread started (~300Hz fixed-rate uplink)");

    while (gRunning.load()) {
        // Wall-clock at the top of EVERY iteration, used by the video submit path's
        // trailing floor sleep (below) to bound submit-to-submit to >= one vsync. (The
        // lobby path has its own tIterStart further down; this covers the stream path.)
        uint64_t tLoopStart = nowNs();
        // --- react to surface changes (create / destroy) coming from Java ---
        if (gWindowDirty.load()) {
            ANativeWindow *newWin;
            {
                std::lock_guard<std::mutex> lk(gWinMutex);
                newWin = gPendingWindow;
                gWindowDirty.store(false);
            }
            if (newWin != curWin) {
                // RESUME FIX (HW mode): the warp thread captured this exact window
                // surface at EV_InitRenderThread and presents to it forever. Before
                // we destroy it, tell the warp to PAUSE so it stops presenting to a
                // surface that's about to die (otherwise on resume it stays bound to
                // the dead surface -> black). EV_Pause reads no args; just fire it.
                if (gWarpToWindow && re && warpCtx != EGL_NO_CONTEXT) {
                    re(EV_Pause);
                    LOGI("HW compositor: warp paused for surface teardown");
                }
                // tear down old surface
                eglMakeCurrent(dpy, pbuf, pbuf, ctx);
                if (sfc != EGL_NO_SURFACE) { eglDestroySurface(dpy, sfc); sfc = EGL_NO_SURFACE; }
                if (curWin) { ANativeWindow_release(curWin); curWin = nullptr; }

                if (newWin) {
                    curWin = newWin;            // ownership transferred to us
                    sfc = eglCreateWindowSurface(dpy, cfg, curWin, nullptr);
                    if (sfc == EGL_NO_SURFACE) {
                        LOGE("eglCreateWindowSurface failed 0x%x", eglGetError());
                    } else {
                        eglMakeCurrent(dpy, sfc, sfc, ctx);
                        eglQuerySurface(dpy, sfc, EGL_WIDTH, &winW);
                        eglQuerySurface(dpy, sfc, EGL_HEIGHT, &winH);
                        // Reset the surfaced-frame counter on every (re)creation,
                        // symmetric with the recovery path above, so a fresh surface
                        // re-settles its ~3 frames before the one-shot logo removal
                        // (pre-vrStarted) instead of carrying a stale count.
                        framesWithSurface = 0;
                        LOGI("window surface ready %dx%d", winW, winH);
                        // First real surface: wire up the SDK render thread so the
                        // tracking data pipe is established (live head pose).
                        // NB(pico): EV_InitRenderThread spins up the SDK render
                        // thread -- which IS the DIATW TimeWarp compositor -- AND
                        // establishes the tracking pipe. The two branches below
                        // decide what the warp presents to: HW path = the real
                        // window (it composites the eye textures we feed it);
                        // self-present path = a private throwaway pbuffer (parked,
                        // so it can't steal our window) while WE present.
                        if (!rtInited && re) {
                            // The SDK warp thread (TimeWarpLocal::ReadSurface,
                            // run inside InitRenderThread) captures whatever EGL
                            // surface is CURRENT at this instant and presents to
                            // it forever. If our window is current it steals it
                            // and blacks us out. So make the tiny 16x16 pbuffer
                            // current first -> the warp thread captures THAT
                            // (invisible, harmless) -> then restore our window.
                            // Tracking still flows (the warp thread polls pose
                            // regardless of where it presents).
                            // Give the warp thread its OWN context + pbuffer to
                            // capture, so it never touches our context/window.
                            // (ReadSurface stores eglGetCurrent{Context,Surface};
                            // the warp thread later makes them current on ITS
                            // thread -- a context can only be current on one
                            // thread, so it must not be ours.)
                            // Note: these are the PANEL per-eye scanout dims (3840/2 x
                            // 2160 = 1920x2160), NOT our square render targets (1664^2
                            // video / 1536^2 lobby). Do NOT change to a square value --
                            // this sizes the SDK's own
                            // single-pass depth buffer, which our path never uses anyway
                            // (we submit finished colour textures via PVR_CameraEndFrame
                            // with no depth -> the warp does rotational-only reprojection),
                            // so the value is inert here. Left as the panel size to match
                            // what the SDK expects.
                            Pvr_SetSinglePassDepthBufferWidthHeight(winW / 2, winH);
                            {
                                // HW COMPOSITOR PATH: let the warp thread own the
                                // WINDOW (direct low-latency present) but on its OWN
                                // context that SHARES our (and ALVR's) textures, so
                                // it can sample the eye textures we feed it. Make
                                // warpCtx + the real window current, issue
                                // InitRenderThread (warp captures warpCtx+window),
                                // then move OUR ctx onto a pbuffer for offscreen
                                // rendering (we no longer self-present).
                                warpCtx = eglCreateContext(dpy, cfg, ctx, ctxAttribsHigh);  // SHARE ours, HIGH prio (persistent: re-pointed on resume)
                                { EGLint pr = -1; eglQueryContext(dpy, warpCtx, EGL_CONTEXT_PRIORITY_LEVEL_IMG, &pr);
                                  LOGI("warpCtx priority level = 0x%x (HIGH=0x%x)", pr, EGL_CONTEXT_PRIORITY_HIGH_IMG); }
                                eglMakeCurrent(dpy, sfc, sfc, warpCtx);
                                re(EV_InitRenderThread);             // warp captures warpCtx + WINDOW
                                eglMakeCurrent(dpy, pbuf, pbuf, ctx);// our ctx -> offscreen pbuffer
                                gWarpToWindow = true;
                                LOGI("HW compositor: warp owns window via shared ctx %p; we render offscreen", warpCtx);
                            }
                            rtInited = true;
                        } else if (rtInited && gWarpToWindow && re && warpCtx != EGL_NO_CONTEXT) {
                            // RESUME RE-POINT (HW mode): the warp thread is alive but
                            // bound to the OLD (destroyed) window surface. Make warpCtx
                            // current on the NEW surface and re-issue EV_InitRenderThread
                            // -- that re-runs TimeWarpLocal::ReadSurface, which re-reads
                            // eglGetCurrentSurface and re-captures THIS new surface. Then
                            // EV_Resume to unpause, and restore our ctx on the pbuffer.
                            eglMakeCurrent(dpy, sfc, sfc, warpCtx);
                            re(EV_InitRenderThread);   // re-capture the new window surface
                            re(EV_Resume);             // unpause the warp
                            eglMakeCurrent(dpy, pbuf, pbuf, ctx);  // our ctx -> offscreen
                            LOGI("HW compositor: warp RE-POINTED to new surface (resume)");
                        }
                    }
                } else {
                    LOGI("window surface destroyed -> pausing render");
                }
            }
        }

        // --- proximity power-sleep (don/doff) --------------------------------
        // Off-head for the timeout -> pause the connection so the server stops
        // streaming and we tear the decoder down (big power saver). On don, resume
        // and let it reconnect. Edge-triggered so pause/resume fire once each.
        {
            bool wantSleep = gSleepReq.load();
            if (wantSleep && !gSlept) {
                LOGI("proximity: off-head timeout -> pausing stream (power save)");
                if (gDecoderReady) { alvr_destroy_decoder(); gDecoderReady = false; }
                gStreaming = false;
                alvr_pause();
                gSlept = true;
            } else if (!wantSleep && gSlept) {
                LOGI("proximity: headset donned -> resuming stream");
                alvr_resume();
                gSlept = false;
            }
        }

        // --- recover a lost EGL surface (display sleep/wake or doff/don) ------
        // The Pico blanks the panel off-head, invalidating our EGL surface
        // (eglSwapBuffers -> EGL_BAD_SURFACE) often WITHOUT a Java surface
        // callback. As long as we still hold the ANativeWindow, just recreate
        // the surface from it instead of freezing on the last frame. Throttled.
        if (sfc == EGL_NO_SURFACE && curWin != nullptr && (frame % 8) == 0) {
            EGLSurface ns = eglCreateWindowSurface(dpy, cfg, curWin, nullptr);
            if (ns != EGL_NO_SURFACE) {
                sfc = ns;
                eglMakeCurrent(dpy, sfc, sfc, ctx);
                eglQuerySurface(dpy, sfc, EGL_WIDTH, &winW);
                eglQuerySurface(dpy, sfc, EGL_HEIGHT, &winH);
                framesWithSurface = 0;
                LOGI("window surface RECOVERED %dx%d", winW, winH);
            }
        }

        // Once the SDK warp thread is up (created by EV_InitRenderThread), pin it to
        // the reserved big core + raise its priority so neither our render thread nor
        // a background task can delay its per-vsync present. Retried because the thread
        // may not be named the instant init returns; stop after ~150 frames so we
        // don't scan /proc forever if the name ever changes. The scan is
        // opendir+readdir+fopen per task, so only retry every 8th frame -- the thread
        // is up within ~1s either way and this keeps the /proc walk off the per-frame
        // path during the latency-sensitive connect moment.
        if (rtInited && !warpPinned && reservedCpu >= 0 && (frame % 8) == 0) {
            int warpCore = pinWarpThreadForLowLatency(reservedCpu);
            if (warpCore >= 0) {
                // The SDK may place the warp on a DIFFERENT big core than our initial
                // guess (observed: cpu6, not the reserved top core). Adopt the warp's
                // real core and move THIS (submit) thread off it -- otherwise the
                // submit + decoder threads share the warp's core and the reservation
                // is defeated. reservedCpu now drives the decoder pin's mask too.
                if (warpCore != reservedCpu) {
                    reservedCpu = warpCore;
                    long lo = 0, hi = 0;
                    cpu_set_t s = bigCoreSetExcept(reservedCpu, lo, hi);
                    if (sched_setaffinity(0, sizeof(s), &s) == 0)
                        LOGI("submit thread re-pinned to big cores [%ld..%ld] minus warp core %d",
                             lo, hi, reservedCpu);
                    else
                        LOGI("submit thread re-pin (minus warp core %d) failed (errno=%d)", reservedCpu, errno);
                }
                warpPinned = true;
            } else if (frame > 150) {
                warpPinned = true;   // give up scanning if the warp never shows / is renamed
            }
        }

        // MAX PERFORMANCE (always on): hold the CPU/GPU pinned at the level
        // desiredPerfLevel() returns (kPerfLevelMax). The binder call only fires once
        // the target is reached; applyPerfLevels returns false until the QVR perf
        // service client binds, so the first apply retries each frame until it takes.
        if (rtInited) {
            static int perfApplied = -2;  // sentinel: != any real level (0..5) or -1 release
            // Only hold the CPU/GPU floor pinned while actively streaming. In the lobby
            // (!gStreaming) or when the headset is doffed/asleep (gSleepReq), RELEASE to
            // the captured baseline (-1) so the SoC can downclock -- no reason to burn
            // watts + self-heat pinned at max when there's no video to keep at 72Hz.
            int want = (!gStreaming || gSleepReq.load()) ? -1 : desiredPerfLevel();
            if (want != perfApplied && applyPerfLevels(want))
                perfApplied = want;

            // Report battery to the server (~1/sec) so the SteamVR dashboard is populated.
            if (gStreaming) {
                static uint64_t sLastBatt = 0;
                uint64_t nb = nowNs();
                if (nb - sLastBatt > 1000000000ULL) { sLastBatt = nb; sendBatteryReports(); }
            }
        }

        // Once a decoder exists, find + pin its output-pump thread (created by
        // alvr_create_decoder, named "AlvrDecOutput" in the fork). The pump spawns
        // synchronously inside alvr_create_decoder, but pthread_setname lands a beat
        // later on the pump's own thread, so retry the /proc scan before giving up.
        // Re-armed per decoder (gDecoderPinned cleared in createVideoDecoder), so it
        // re-pins after a reconnect / manual-lobby resume. Throttled to every 8th
        // frame like the warp scan; ~25 attempts (~200 frames) before giving up.
        if (gStreaming && gDecoderReady && !gDecoderPinned && (frame % 8) == 0) {
            if (pinDecoderThreadForLowLatency(reservedCpu) || ++gDecoderPinTries > 25)
                gDecoderPinned = true;
        }

        // ---- Head pose read-back (produced by the tracking thread) -----------
        // Tracking + the ALVR uplink now live on trackingThread (fixed ~300Hz).
        // The render thread only needs the latest head pose for the lobby head
        // transform + heartbeat log, so it reads the pose the tracking thread
        // publishes into gHeadData rather than calling the SDK a second time.
        float qx=0,qy=0,qz=0,qw=1, px=0,py=0,pz=0, vfov=90;
        {
            std::lock_guard<std::mutex> lk(gHeadMutex);
            qx=gHeadData[0]; qy=gHeadData[1]; qz=gHeadData[2]; qw=gHeadData[3];
            px=gHeadData[4]; py=gHeadData[5]; pz=gHeadData[6];
        }
        if ((frame % 120) == 0)
            LOGI("frame %d q=(%.3f,%.3f,%.3f,%.3f) p=(%.3f,%.3f,%.3f) fov=%.1f vr=%d surf=%d",
                 frame, qx,qy,qz,qw, px,py,pz, vfov, vrStarted, (sfc != EGL_NO_SURFACE));

        const uint64_t ts = nowNs();

        // Persist a changed Software IPD once it settles (~0.5s after the last
        // adjustment) so we don't hammer the file during a slider sweep.
        if (gIpdDirty.load() && ts - gIpdChangeNs.load() > 500000000ULL) {
            saveSoftIpd();
            gIpdDirty.store(false);
        }
        // If the IPD changed while streaming, push fresh view_params so the server
        // re-renders at the new eye separation without needing a reconnect.
        if (gStreaming && gSentIpdMm.load() != gSoftIpdMm.load()) {
            sendViewParams();
            LOGI("Software IPD changed mid-stream -> resent view_params (%.1f mm)", gSoftIpdMm.load());
        }

        // ---- ALVR: drain events --------------------------------------------
        AlvrEvent ev;
        while (alvr_poll_event(&ev)) {
            if (ev.tag == ALVR_EVENT_HUD_MESSAGE_UPDATED) {
                char msg[1024] = {0};
                alvr_hud_message_bounded(msg, sizeof(msg));   // never overflows; NUL-terminated within cap
                LOGI("ALVR event HUD_MESSAGE_UPDATED: %s", msg);
                // Map ALVR's HUD message to a short lobby status. ALVR emits e.g.
                // "Searching for streamer..." / "Streamer found. Connecting...".
                if (strstr(msg, "onnect"))      strcpy(gStatusText, "CONNECTING");
                else if (strstr(msg, "earch"))  strcpy(gStatusText, "SEARCHING");
                else if (msg[0] == 0)           strcpy(gStatusText, "DISCONNECTED");
                // ALVR prints "hostname: XXXX.client" -- the name to trust on the
                // PC. Parse it into gHostnameText for the lobby HUD.
                const char *hn = strstr(msg, "hostname:");
                if (hn) {
                    hn += 9;
                    while (*hn == ' ' || *hn == '\t') hn++;
                    int i = 0;
                    while (*hn && *hn != '\n' && *hn != '\r' && i < (int)sizeof(gHostnameText)-1)
                        gHostnameText[i++] = *hn++;
                    gHostnameText[i] = 0;
                    toUpperAscii(gHostnameText);
                }
            } else if (ev.tag == ALVR_EVENT_STREAMING_STARTED) {
                gStreamW = ev.STREAMING_STARTED.view_width;
                gStreamH = ev.STREAMING_STARTED.view_height;
                gRefreshHint = ev.STREAMING_STARTED.refresh_rate_hint;
                LOGI("ALVR STREAMING_STARTED %ux%u @%.0fHz fov-enc=%d hdr=%d",
                     gStreamW, gStreamH, ev.STREAMING_STARTED.refresh_rate_hint,
                     ev.STREAMING_STARTED.enable_foveated_encoding,
                     ev.STREAMING_STARTED.enable_hdr);
                // Read the server's foveation params live from settings JSON so
                // our de-foveation (done inside alvr_render_stream_opengl) matches
                // the server exactly, whatever FFR preset is selected. Gated on
                // the event's enable_foveated_encoding flag.
                bool foveOn = ev.STREAMING_STARTED.enable_foveated_encoding;
                float foCsx=0, foCsy=0, foShx=0, foShy=0, foErx=0, foEry=0;
                if (foveOn) {
                    float fp[6]; readFoveationParams(fp);
                    foCsx=fp[0]; foCsy=fp[1]; foShx=fp[2]; foShy=fp[3]; foErx=fp[4]; foEry=fp[5];
                    LOGI("foveation ON: center=(%.3f,%.3f) shift=(%.3f,%.3f) edge=(%.2f,%.2f)",
                         foCsx, foCsy, foShx, foShy, foErx, foEry);
                }
                // Cache the params we're about to bake so a later mid-session
                // foveation change can be detected + re-synced (see REAL_TIME_CONFIG).
                gFoveOn = foveOn;
                gFovParams[0]=foCsx; gFovParams[1]=foCsy; gFovParams[2]=foShx;
                gFovParams[3]=foShy; gFovParams[4]=foErx; gFovParams[5]=foEry;
                gFovResyncPending = false;   // fresh stream -> drop any stale pending re-sync
                // Defensive: if a reconnect arrives without a STREAMING_STOPPED,
                // drop the stale decoder so the DECODER_CONFIG that follows this
                // event rebuilds it fresh (ALVR sends config after each started).
                if (gDecoderReady) { alvr_destroy_decoder(); gDecoderReady = false; }
                // createStreamSwapchain below allocates NEW textures + resets the
                // pipeline (gPrevSwapValid=false), so no stale slot is handed to the
                // warp on reconnect.
                if (gAlvrGlReady && gStreamW > 0) {
                    createStreamSwapchain(gStreamW, gStreamH);
                    const uint32_t *swapArr[2] = { gSwap[0], gSwap[1] };
                    AlvrStreamConfig sc = {};
                    sc.view_resolution_width  = gStreamW;
                    sc.view_resolution_height = gStreamH;
                    sc.swapchain_textures = (const uint32_t **) swapArr;
                    sc.swapchain_length   = kSwapLen;
                    sc.enable_foveation        = foveOn;
                    sc.foveation_center_size_x = foCsx;
                    sc.foveation_center_size_y = foCsy;
                    sc.foveation_center_shift_x= foShx;
                    sc.foveation_center_shift_y= foShy;
                    sc.foveation_edge_ratio_x  = foErx;
                    sc.foveation_edge_ratio_y  = foEry;
                    sc.enable_upscaling   = false;
                    alvr_start_stream_opengl(sc);
                    // Tell the server our per-eye FOV (~101deg) + current Software IPD.
                    sendViewParams();
                    gSwapIdx = 0;
                    gStreaming = true;
                    gManualLobby.store(false);   // start in the stream, not the manual lobby
                    // Announce a CUSTOM interaction profile per hand: the exact source
                    // button set we actually drive. The server's automatic binder maps
                    // these to the emulated controller. Critically this includes the
                    // RIGHT hand's menu/click, which the stock Quest profile lacks -- the
                    // binder then maps right menu -> right SYSTEM (SteamVR dashboard),
                    // giving the Pico's right App button a dashboard button it otherwise
                    // wouldn't have. (Left menu still maps to the app menu.)
                    // NOTE: a custom interaction profile rebuilds the GLOBAL button
                    // mapping manager from exactly the ids it carries, so it must be
                    // sent ONCE with BOTH hands' buttons -- sending it per-hand makes
                    // the second (right) call wipe the left hand's mappings.
                    {
                        uint64_t ids[16] = {
                            alvrBtn[0].trigVal, alvrBtn[0].gripVal,
                            alvrBtn[0].thumbX,  alvrBtn[0].thumbY, alvrBtn[0].thumbClick,
                            alvrBtn[0].menu,    alvrBtn[0].face1,  alvrBtn[0].face2,
                            alvrBtn[1].trigVal, alvrBtn[1].gripVal,
                            alvrBtn[1].thumbX,  alvrBtn[1].thumbY, alvrBtn[1].thumbClick,
                            alvrBtn[1].menu,    alvrBtn[1].face1,  alvrBtn[1].face2,
                        };
                        alvr_send_custom_interaction_profile(alvrHandId[0], ids, 16);
                    }
                    LOGI("sent custom interaction profile (both hands, incl. right menu->system)");
                    strcpy(gStatusText, "CONNECTED");
                    LOGI("stream renderer ready (%ux%u)", gStreamW, gStreamH);
                    // Settings JSON is now live: light up the EYE illuminators only if
                    // the server's Face Tracking eye source is on (else IR stays off).
                    applyServerEyeTracking(true);
                    // Forward the Pico play area to SteamVR as the chaperone (over
                    // the now-open control channel). Only if a guardian was set up.
                    if (gPlayspaceW > 0.0f && gPlayspaceD > 0.0f) {
                        alvr_send_playspace(gPlayspaceW, gPlayspaceD);
                        LOGI("sent playspace %.2fx%.2f m", gPlayspaceW, gPlayspaceD);
                    }
                }
            } else if (ev.tag == ALVR_EVENT_DECODER_CONFIG) {
                if (!gDecoderReady) {
                    // Cache the config (codec + NAL) so we can rebuild the decoder
                    // after the manual-lobby pause without a reconnect.
                    // Bounded copy: alvr_get_decoder_config_bounded never writes
                    // past sizeof(gDecCfg) and returns the FULL length. The NAL is
                    // server-controlled, so an oversized payload is hostile/corrupt
                    // -- REJECT it (don't feed a truncated SPS/PPS to MediaCodec).
                    uint64_t n = alvr_get_decoder_config_bounded(
                            (char *) gDecCfg, sizeof(gDecCfg));
                    if (n > sizeof(gDecCfg)) {
                        LOGE("decoder config %llu B exceeds %zu B buffer -- rejecting",
                             (unsigned long long) n, sizeof(gDecCfg));
                    } else {
                        gDecCodec = ev.DECODER_CONFIG.codec;
                        gDecCfgLen = n;
                        gHaveDecCfg = true;
                        createVideoDecoder();
                    }
                }
            } else if (ev.tag == ALVR_EVENT_STREAMING_STOPPED) {
                // Tear the stream down fully so a reconnect (e.g. SteamVR
                // restart) rebuilds the decoder + swapchain from the NEW config.
                // Without this, the stale decoder (old SPS/PPS, maybe old
                // resolution) decodes the new stream -> solid colors + garbled
                // blocks. gDecoderReady=false forces re-create on next config.
                gStreaming = false;
                gManualLobby.store(false);   // back to the normal disconnected lobby
                gHaveDecCfg = false;         // stale config; next stream sends a fresh one
                strcpy(gStatusText, "DISCONNECTED");
                if (gDecoderReady) { alvr_destroy_decoder(); gDecoderReady = false; }
                destroyStreamSwapchain();   // also resets the pipeline (no stale slot)
                gFovResyncPending = false;   // drop any pending re-sync for the dead stream
                applyServerEyeTracking(false);       // stream gone -> turn the IR off
                LOGI("ALVR STREAMING_STOPPED -> decoder+swapchain torn down");
            } else if (ev.tag == ALVR_EVENT_REAL_TIME_CONFIG) {
                // The server changed a live setting mid-session. CHECK for a
                // foveation change: our de-foveation params are baked into the
                // stream renderer at StreamingStarted, so if the server re-foveated
                // without a full restart the eyes misalign (stale FFR map) until an
                // app restart. Re-read the params and, if they moved, rebuild the
                // de-foveation pipeline in place. (No-op if unchanged. Settings only
                // refresh on StreamingStarted, so this self-heals whenever ALVR does
                // surface the new values; otherwise a reconnect remains the cure.)
                if (gStreaming && gAlvrGlReady && gFoveOn && gStreamW > 0) {
                    float np[6]; readFoveationParams(np);
                    bool changedVsApplied = false;
                    for (int i = 0; i < 6; i++)
                        if (fabsf(np[i] - gFovParams[i]) > 1e-4f) changedVsApplied = true;
                    if (changedVsApplied) {
                        // DEFER the rebuild -- stash the latest params + reset the
                        // settle timer. The loop applies one rebuild once these stop
                        // changing for kFovDebounceNs, so an FFR slider drag's burst of
                        // events = a single swapchain realloc, not one per event.
                        memcpy(gFovPending, np, sizeof(gFovPending));
                        gFovPendingNs = nowNs();
                        gFovResyncPending = true;
                        LOGI("REAL_TIME_CONFIG: foveation change pending center=(%.3f,%.3f) "
                             "shift=(%.3f,%.3f) edge=(%.2f,%.2f) -> debounced %llums",
                             np[0],np[1],np[2],np[3],np[4],np[5],
                             (unsigned long long)(kFovDebounceNs/1000000ULL));
                    } else {
                        // Matches what's applied -> cancel any pending rebuild (e.g. a
                        // drag that returned to the original value) and no-op.
                        gFovResyncPending = false;
                        LOGI("REAL_TIME_CONFIG: no foveation change");
                    }
                } else {
                    LOGI("ALVR event REAL_TIME_CONFIG (not streaming / no foveation)");
                }
            } else if (ev.tag == ALVR_EVENT_HAPTICS) {
                // Server-driven controller rumble. Map device_id -> hand and park
                // the pulse; the Java ControllerClient poller drains it and calls
                // the Pico CV2 vibrate. (Pico's API takes strength+time, no freq.)
                uint64_t did = ev.HAPTICS.device_id;
                int hand = (did == alvrHandId[1]) ? 1 : (did == alvrHandId[0]) ? 0 : -1;
                if (hand >= 0)
                    queueHaptic(hand, ev.HAPTICS.amplitude, ev.HAPTICS.frequency,
                                ev.HAPTICS.duration_s);
            } else {
                LOGI("ALVR event %s", alvrEventName(ev.tag));
            }
        }

        // Apply a debounced foveation re-sync once the pending params have settled
        // (no further REAL_TIME_CONFIG change for kFovDebounceNs). Coalesces an FFR
        // slider drag's burst of events into a single swapchain rebuild. Runs here --
        // right after the event drain, before the per-frame GL work -- so our context
        // is current. Guards mirror the handler's; STREAMING_STARTED/STOPPED clear
        // gFovResyncPending so a pending change can't leak across streams.
        if (gFovResyncPending && gStreaming && gAlvrGlReady && gFoveOn && gStreamW > 0 &&
            nowNs() - gFovPendingNs > kFovDebounceNs) {
            applyFoveationResync(gFovPending);
            gFovResyncPending = false;
        }

        // Without a display surface (headset asleep/off-head) we keep the ALVR
        // uplink alive above, but skip GL rendering.
        // No surface (panel asleep / doffed): nothing to draw. Idle at ~20Hz instead
        // of busy-spinning at ~90Hz -- pure power saving. Surface recovery above is
        // gated on (frame % 8), so worst-case wake latency stays well under a second.
        if (sfc == EGL_NO_SURFACE) { frame++; usleep(50000); continue; }

        // Enter VR a few frames after we first have a surface.
        //
        // NB(pico): this gates the Java startVRModel() entry point, which is a
        // DIFFERENT, cruder way to bring up the SDK compositor than the path we
        // actually use. startVRModel() spins up DIATW and lets it grab the panel
        // WITHOUT our shared-context handoff -- so it either blacks the screen
        // ("No valid Eye Buffers") or steals our window surface (eglSwapBuffers ->
        // EGL_BAD_SURFACE -> surf=0 forever). The HW compositor we DO use is
        // started up in the surface handler above via EV_InitRenderThread with a
        // shared warp context + the eye textures fed to it; that path does not go
        // through startVRModel. So leave this Java call gated off in both modes
        // (PICO_USE_SDK_COMPOSITOR is off): HW mode already has its warp, and
        // self-present owns the window itself. Tracking is service-based either way.
        // Use >= (not ==) so the one-shot logo removal still fires if framesWithSurface
        // ever skips the exact value 3 (e.g. a reset to a higher count, or surface
        // churn). The !vrStarted guard makes it fire exactly once.
        if (!vrStarted && framesWithSurface >= 3) {
            callVrStatic(env, "removePlatformLogo");
#ifdef PICO_USE_SDK_COMPOSITOR
            callVrStatic(env, "startVRModel");
            LOGI("VR mode started (SDK compositor)");
#else
            LOGI("VR mode: self-present (SDK compositor disabled)");
#endif
            vrStarted = true;
        }

        const int halfW = winW / 2;

        // ---- Toggle the lobby overlay mid-stream (stream stays alive) --------
        // Two gestures flip gManualLobby (render the lobby WITHOUT tearing down the
        // stream -- connection/decoder stay alive, decoder kept drained):
        //   (a) hold the headset SIDE button (keycode 1001) for 3s, or
        //   (b) DOUBLE-TAP the RIGHT controller app/menu button.
        // Both toggle in/out. The side hold is suppressed while dragging an EQ fader.
        auto toggleManualLobby = [&](const char *why) {
            bool nowLobby = !gManualLobby.load();
            gManualLobby.store(nowLobby);
            if (nowLobby) hudAnchored = false;   // re-anchor panels at current head
            gOkClick.store(false);               // swallow any pending click
            LOGI("%s -> manual lobby = %d (stream stays alive)", why, (int)nowLobby);
        };
        {
            static uint64_t sideHoldStart = 0;
            if (gSideHeld.load() && !gEqGrabbing) {
                if (sideHoldStart == 0) sideHoldStart = nowNs();
                else if (nowNs() - sideHoldStart > 2000000000ULL) {   // 2s hold
                    toggleManualLobby("side button 2s hold");
                    sideHoldStart = 0;                                 // consume this hold
                }
            } else {
                sideHoldStart = 0;
            }
        }
        // Right controller app/menu double-tap (two rising edges within 400ms).
        {
            static bool menuPrev = false; static uint64_t lastTapNs = 0;
            bool menuNow = false;
            { std::lock_guard<std::mutex> lk(gCtrlMutex);
              menuNow = (gCtrl[1].conn==1 && gCtrl[1].keyCount>5 && gCtrl[1].keys[5]!=0); }
            if (menuNow && !menuPrev) {           // rising edge
                uint64_t now = nowNs();
                if (lastTapNs != 0 && now - lastTapNs < 400000000ULL) {
                    toggleManualLobby("right menu double-tap");
                    lastTapNs = 0;
                } else {
                    lastTapNs = now;
                }
            }
            menuPrev = menuNow;
        }

        // ---- Manual-lobby: KEEP the video decoder ALIVE (no teardown) --------
        // Do NOT destroy the decoder on lobby entry to save power: the fork leaks an
        // ImageReader on every decoder teardown ("FIXME: Leaking Imagereader!"), so
        // rapid lobby<->stream toggling wedges the Venus decoder/surface until a device
        // reboot (every decoded frame then fails with sf error -38 -> "too many decoder
        // errors" -> restart connection, forever). Instead the decoder stays alive
        // across the toggle and is kept DRAINED below so it doesn't saturate while we
        // aren't presenting its frames. Trade-off: the SoC keeps decoding video we don't
        // show in the lobby (more power), but the connection stays rock-solid. Audio is
        // on its own path, untouched.
        {
            static bool sManualPrev = false;
            bool ml = gManualLobby.load();
            if (ml && !sManualPrev) LOGI("manual lobby: decoder kept alive + drained");
            else if (!ml && sManualPrev) LOGI("manual lobby exit: resuming presentation");
            sManualPrev = ml;
        }

        // While in the manual lobby the present block below is short-circuited, so
        // nothing consumes the decoder's output. Keep it DRAINED here: alvr_get_frame
        // pop_front()s (and thus releases) the previous image and hands the next, so
        // discarding everything available keeps the bounded ImageReader cycling and
        // the decoder from saturating. Cheap no-op when no frame is ready.
        if (gStreaming && gDecoderReady && gManualLobby.load()) {
            uint64_t dts = 0; void *dbuf = nullptr;
            while (alvr_get_frame(&dts, &dbuf)) { /* discard: not presenting in lobby */ }
        }

        // ---- ALVR video path: async TimeWarp (present every refresh) --------
        // gManualLobby short-circuits this so the lobby renders over a live stream.
        if (gStreaming && gDecoderReady && !gManualLobby.load()) {
            hudAnchored = false;   // re-anchor the lobby HUD next time we return to it
            // Free the lobby eye-texture ring (~94 MB) while streaming; it's rebuilt
            // lazily when we next return to the lobby. DEFER the free -- arm a short
            // countdown on the first stream frame after a lobby, then free only once the
            // warp ring (4 entries) has been refilled with stream textures, so we never
            // glDeleteTextures slots the free-running warp is still sampling (that was
            // the manual-lobby<->stream toggle crash). Re-entering the lobby cancels a
            // pending free (handled in the lobby block: it disarms lobbyFreeDelay).
            if (gLobbyEyeReady && lobbyFreeDelay < 0) {
                lobbyFreeDelay = kSwapLen + 2;   // wait out the 4-entry warp ring + margin
            }
            if (lobbyFreeDelay == 0) {
                eglMakeCurrent(dpy, pbuf, pbuf, ctx);
                freeLobbyTarget();
                lastRenderedIdx = -1; lobbyEyeIdx = 0;   // force a fresh render on rebuild
                lobbyFreeDelay = -1;
            } else if (lobbyFreeDelay > 0) {
                lobbyFreeDelay--;
            }
            // (0/1) ADAPTIVE PHASE-LOCK + UPDATE SOURCE (HW path).
            // The warp free-runs reprojection every vsync; the beat comes from
            // the AGE of the video frame we submit jittering as the PC's frame
            // arrival drifts (non-genlocked) past a fixed capture gate -- when the
            // drift brings arrival onto the gate, intervals alternate 0/2 new
            // frames = drop/double = stutter. So we TRACK where frames actually
            // arrive (EMA of the vsync phase at arrival) and place the submit gate
            // a fixed margin AFTER it. The gate chases the drift, so it never sits
            // on the arrival boundary -> no drop/double beat, no buffering latency.
            uint64_t ts = 0; void *hwbuf = nullptr;
            uint64_t fts = 0; void *fbuf = nullptr; int drained = 0;
            // BLOCK on the decoder for the first frame instead of busy-polling.
            // The fork's alvr_get_frame_timeout sleeps until the ImageReader pushes a
            // freshly decoded frame (or the cap elapses), so this loop wakes ~once per
            // arrived frame (~72-120Hz) instead of spinning. Cap = 2 frame
            // intervals: under healthy streaming a frame always lands within 1, so we
            // wake on the frame; the cap only bounds how often top-of-loop housekeeping
            // (surface/proximity/manual-lobby) runs when the stream stalls. Once the
            // first frame is in hand, drain the rest non-blocking to coalesce to newest.
            //
            // BUFFER RELEASE (verified against the client_core fork, video_decoder/
            // android.rs dequeue_frame): coalescing here does NOT leak or starve the
            // decoder. Each alvr_get_frame pop_front()s the PREVIOUS call's image first
            // -- Rust's Image::Drop returns that AHardwareBuffer to the ImageReader for
            // the decoder to reuse -- then hands back the next. So every intermediate
            // frame in this drain loop is released by the very next call; pop_front() IS
            // the release. The final coalesced `hwbuf` stays marked in_use (valid)
            // until the NEXT iteration's first dequeue, by which point
            // alvr_render_stream_opengl below has already consumed it. The ImageReader
            // is bounded (maxImages=10; its listener drops the front on overflow), so
            // nothing accumulates even if we briefly fall behind. (One benign quirk:
            // dequeue_frame samples its buffering EMA every call, so a coalesced burst
            // samples it 2-3x/iteration -- but that biases the average DOWN, the safe
            // keep-more-buffer direction, so it's left as-is rather than forked.)
            uint64_t blockNs = (uint64_t)(2e9f / (gRefreshHint > 1.0f ? gRefreshHint : 72.0f));
            if (alvr_get_frame_timeout(&fts, &fbuf, blockNs)) {
                ts = fts; hwbuf = fbuf; drained++;
                while (alvr_get_frame(&fts, &fbuf)) { ts = fts; hwbuf = fbuf; drained++; }
            }

            // SUBMIT EVERY DECODED FRAME. The warp thread free-runs reprojection
            // to the live predicted pose every vsync on its own, so we just need to
            // keep the latest decoded frame in front of it. Do NOT add an adaptive
            // vsync submit gate: it submits at most once per vsync, so a frame landing
            // after the gate in its vcount is overwritten before submit (72 arrive but
            // only ~60 shown) for no benefit now the per-eye texture race is fixed.
            bool doSubmit = (drained > 0);

            // ADAPTIVE FRAME PACING (anti-judder). Submitting every decoded frame is
            // fine while we hold 72, but once the device can't keep up it presents at
            // an irregular 55-67 fps that beats the 72Hz scanout: each warp-pose update
            // then lands a different number of vsyncs apart and snaps the view by a
            // different amount, which reads as violent shake. Dropping CLEANLY to an
            // evenly-spaced lower rate (every 2nd/3rd vsync) is lower fps but smooth,
            // because every present is held the same span. Opening a HUD only ever
            // "fixed" this by loading the GPU enough to force a steady 36Hz by accident.
            //
            // ZERO COST WHILE WE KEEP UP: at sPaceDiv==1 this ONLY observes cadence to
            // decide when to back off -- it never sleeps and never re-drains, so a
            // healthy 72Hz stream submits exactly as before with no added latency. The
            // phase-lock sleep engages only after we've already dropped to 36/24Hz,
            // where holding an even beat is worth a little queueing. sPaceDiv moves with
            // asymmetric hysteresis (back off fast on missed deadlines, speed up only
            // after a long clean spell) so it settles instead of oscillating.
            static int      sPaceDiv = 1;          // 1=72Hz, 2=36Hz, 3=24Hz
            static int64_t  sLastSubmitVs = 0;
            static int      sWinMiss = 0;
            static uint64_t sWinT0 = 0, sLastMissNs = 0;
            if (doSubmit) {
                const double interval = 1e9 / 72.0;         // panel vsync period (ns)
                int64_t cur = (int64_t) floor(PVR::GetFractionalVsync());
                if (sPaceDiv > 1) {
                    // Reduced rate: phase-lock the present to an even sPaceDiv-vsync beat.
                    int64_t target = sLastSubmitVs + sPaceDiv;
                    if (cur < target) {
                        double waitVs = (double) target - PVR::GetFractionalVsync();
                        if (waitVs > 0.0 && waitVs < 4.0)   // guard a bogus oracle read
                            usleep((useconds_t)(waitVs * interval / 1000.0));
                        cur = (int64_t) floor(PVR::GetFractionalVsync());
                    }
                    // Grab any fresher frame that arrived during the wait, so pacing only
                    // regularises WHEN we present -- no extra latency on the frame shown.
                    while (alvr_get_frame(&fts, &fbuf)) { ts = fts; hwbuf = fbuf; drained++; }
                }
                // A miss = this submit ran past the current rate's vsync budget AND the
                // decoder had a backlog (drained>1 => our fault, not a slow/bursty source:
                // a late submit with one frame in hand is sub-72 server fps / a network
                // gap / a reconnect transient, which must not drop the rate).
                int64_t spacing = (sLastSubmitVs != 0) ? (cur - sLastSubmitVs) : sPaceDiv;
                sLastSubmitVs = cur;
                bool ourMiss = (spacing - sPaceDiv > 0 && drained > 1);

                // Act on the FREQUENCY of misses, never on a single one. A lone stall --
                // a periodic keyframe-decode or GC spike, seen as one frame overrunning
                // 10+ vsyncs -- must NOT step the rate down: the frame already hitched, so
                // dropping the rate can't un-hitch it and only pumps 72<->24, which looks
                // worse than the isolated jerk. Only a SUSTAINED beat of misses in a short
                // window -- where an even lower rate is genuinely smoother -- backs off.
                uint64_t pnow = nowNs();
                if (ourMiss) { sWinMiss++; sLastMissNs = pnow; }
                if (sWinT0 == 0) sWinT0 = pnow;
                if (pnow - sWinT0 >= 250000000ULL) {        // short window -> quick onset
                    if (sWinMiss >= 2 && sPaceDiv < 3) {
                        sPaceDiv++;
                        LOGI("pace: shake guard -> %dHz (%d misses/0.25s)", 72 / sPaceDiv, sWinMiss);
                    } else if (sPaceDiv > 1 && pnow - sLastMissNs >= 3000000000ULL) {
                        // Step back up only after a sustained clean spell (no miss for ~3s),
                        // one level at a time, so it settles instead of oscillating.
                        sPaceDiv--;
                        LOGI("pace: recovered -> %dHz", 72 / sPaceDiv);
                    }
                    sWinMiss = 0; sWinT0 = pnow;
                }
            }
            // STUTTER DIAGNOSTIC: per-stage timing, per-second max (ms). gap =
            // submit-to-submit period (captures EVERYTHING between presents incl. the
            // top-of-loop tracking + sleep + drain); render/enc = our GPU-issue cost.
            // `enq` = the cost of handing the frame to the SDK warp
            // (PVR_CameraEndFrame/ChangeRenderPose/TimeWarpEvent) -- i.e. submit/
            // enqueue time INCL. the warp thread's vsync backpressure, NOT the warp's
            // own reproject/present compute (the SDK exposes no hook for that).
            // These per-stage nowNs() probes feed ONLY the diag HUD's pipeline
            // page (+ its once/sec VIDEO-TIMING log). With no HUD shown there's no
            // consumer, so skip the ~7 clock reads/frame on the shipping in-game path.
            // The per-second VIDEO / VIDEO-LATENCY logs (decoded counts + server-acked
            // latency) need no per-frame timing and stay ALWAYS, so the primary
            // diagnostics survive with the HUD off. (First frame after the HUD is
            // turned on may show one inflated `gap` from the stale _lastStart; it
            // self-corrects within the second.)
            bool diagTiming = gDiagHudMode.load() != 0;
            static uint64_t _lastStart=0, _mGap=0, _mRender=0, _mEnc=0, _mEnq=0;
            uint64_t _tStart = diagTiming ? nowNs() : 0, _tRender = _tStart, _tEnc = _tStart;
            if (doSubmit) {
                if (diagTiming) {
                    if (_lastStart) { uint64_t g = _tStart - _lastStart; if (g > _mGap) _mGap = g; }
                    _lastStart = _tStart;
                }
                AlvrViewParams outVP[2] = {};
                alvr_report_compositor_start(ts, outVP);

                if (!gAtwEnabled) { Pvr_SetAsyncTimeWarp(1); gAtwEnabled = true;
                    LOGI("HW compositor: async TimeWarp enabled; cfg8(asyncMode)=%d cfg0x19=%d",
                         cfgI(8), cfgI(0x19)); }

                // Hand one ring slot to the SDK warp thread: wait its (frame-old)
                // fence so it's GPU-complete, set both eyes' textures + the render
                // pose they were drawn at, then enqueue. The warp reprojects that
                // baseline to the live predicted pose every vsync. A lambda so it can
                // be invoked at two points (see the previous-frame submit below).
                static int sFenceTimeouts = 0;   // per-second tally (reset below), render-thread-only
                auto submitSlot = [&](int p, uint64_t fenceWaitNs) {
                    if (gSwapFence[p]) {
                        // Don't ignore the wait result. GL_TIMEOUT_EXPIRED means the
                        // slot wasn't GPU-complete within budget, so we're about to hand
                        // the warp a possibly-torn texture -- count it so it surfaces in
                        // the diag HUD / log instead of being a silent visual tear.
                        GLenum w = glClientWaitSync(gSwapFence[p], GL_SYNC_FLUSH_COMMANDS_BIT, fenceWaitNs);
                        if (w == GL_TIMEOUT_EXPIRED) sFenceTimeouts++;
                    }
                    PVR_CameraEndFrame(0, gSwap[0][p]);
                    PVR_CameraEndFrame(1, gSwap[1][p]);
                    // Render-pose baseline for BOTH eyes. Normalize the quat; reuse
                    // last good if degenerate (the warp's SelectRT rejects non-unit
                    // quats -> "frame from the past").
                    static Quat sLastGoodQ[2] = { {0,0,0,1}, {0,0,0,1} };
                    for (int e = 0; e < 2; e++) {
                        Quat q = { gSwapVP[p][e].pose.orientation.x, gSwapVP[p][e].pose.orientation.y,
                                   gSwapVP[p][e].pose.orientation.z, gSwapVP[p][e].pose.orientation.w };
                        float n2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
                        if (n2 > 1e-6f) { q = quatNorm(q); sLastGoodQ[e] = q; }
                        else            { q = sLastGoodQ[e]; }
                        PvrPoseBlk blk; memset(&blk, 0, sizeof(blk));
                        blk.v[0] = q.x; blk.v[1] = q.y; blk.v[2] = q.z; blk.v[3] = q.w;
                        blk.v[4] = gSwapVP[p][e].pose.position[0];
                        blk.v[5] = gSwapVP[p][e].pose.position[1];
                        blk.v[6] = gSwapVP[p][e].pose.position[2];
                        PVR_ChangeRenderPose(e, 0, blk);
                    }
                    PVR_TimeWarpEvent(0);
                };

                // SUBMIT THE PREVIOUS (already GPU-complete) FRAME *FIRST*, before
                // we spend this iteration de-foveating the current one. The previous
                // slot's textures + pose were finalized last frame and don't depend on
                // anything we do below, so handing them to the warp now -- instead of
                // after the render -- gets the frame in front of the warp ~one render's
                // worth of CPU/GPU-issue time sooner each frame (lower motion-to-photon).
                // Its fence is a full frame old, so the wait returns ~instantly.
                // The inaugural frame (no previous slot yet) is submitted AFTER its own
                // render below; ALVR's compositor_start/report_submit accounting
                // stays paired on the current `ts` either way (the +1-frame texture
                // hand-off is orthogonal to the latency bookkeeping).
                bool firstFrame = !gPrevSwapValid;
                uint64_t _tEnqStart = diagTiming ? nowNs() : 0;
                if (!firstFrame) submitSlot(gPrevSwapIdx, 5000000ULL /* ~0ms; frame old */);
                if (diagTiming) { uint64_t e = nowNs(); if (e - _tEnqStart > _mEnq) _mEnq = e - _tEnqStart; }

                AlvrStreamViewParams svp[2];
                for (int e = 0; e < 2; e++) {
                    svp[e].swapchain_index = (uint32_t) gSwapIdx;
                    svp[e].reprojection_rotation = { 0, 0, 0, 1 };
                    svp[e].fov = outVP[e].fov;
                }
                uint64_t _tRenderStart = diagTiming ? nowNs() : 0;
                alvr_render_stream_opengl(hwbuf, svp);   // -> gSwap[e][gSwapIdx] (leaves wgpu ctx current)
                if (diagTiming) { _tRender = nowNs(); if (_tRender - _tRenderStart > _mRender) _mRender = _tRender - _tRenderStart; }

                {
                    // CRITICAL (HW path): the video is rendered in ALVR's wgpu context.
                    // Fence it (sync objects are shared across the share group) so the
                    // warp -- which samples this slot a frame later -- never reads a
                    // half-written texture. NOT glFinish (that stalls the warp thread).
                    GLsync alvrFence = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0); glFlush();
                    // wgpu made its ctx current; restore ours (offscreen pbuffer; the
                    // warp owns the window). Textures are shared across the group.
                    // This restore is load-bearing and must stay unconditional: the SDK
                    // warp hand-off (PVR_CameraEndFrame / glClientWaitSync at the top of
                    // the NEXT iteration) requires OUR context current, not wgpu's, even
                    // though they share an EGL group -- leaving wgpu's context current
                    // (e.g. to skip this call when the HUD is off) aborts in submitSlot.
                    // The ~1 eglMakeCurrent/frame is microseconds, not a bottleneck
                    // (render~2.4ms, enq~12ms).
                    eglMakeCurrent(dpy, pbuf, pbuf, ctx);
                    // PicoNeo2 fork: NO encode blit. The forked stream shader already
                    // wrote the FINAL present-domain bytes (warm WB + linear->sRGB
                    // encode + triangular dither) directly into gSwap[e][gSwapIdx], so
                    // we feed gSwap straight to the warp. The only thing left to draw
                    // here is the optional diag HUD, overlaid into the just-rendered slot.
                    int diagPage = gDiagHudMode.load();
                    if (diagPage != 0) {
                        // The HUD draws into gSwap in OUR ctx, so it must be
                        // ordered after ALVR's de-foveation render -> make our queue
                        // wait on the ALVR fence (GPU-side, no CPU/warp stall), then a
                        // fresh fence below covers render+HUD as this slot's fence.
                        if (alvrFence) { glWaitSync(alvrFence, 0, GL_TIMEOUT_IGNORED); glDeleteSync(alvrFence); alvrFence = 0; }
                        glBindFramebuffer(GL_FRAMEBUFFER, gStreamFbo);
                        glDisable(GL_DEPTH_TEST); glDisable(GL_CULL_FACE); glDisable(GL_SCISSOR_TEST);
                        glViewport(0, 0, (GLsizei) gStreamW, (GLsizei) gStreamH);
                      if (diagPage != 0) {
                        // Rebuild + re-upload the HUD geometry at most ~4Hz, not every
                        // video frame. Its content only changes at the 1-3Hz stat
                        // cadence (see buildDiagOverlay's own held[] throttle), so
                        // regenerating ~6.5k floats + glBufferData ~26KB every frame on
                        // the submit critical path would stutter the HUD. Keep a
                        // persistent vector (clear() retains capacity, no realloc) and
                        // only re-spec the VBO when we rebuild; the draw (cheap, 2 calls
                        // of cached geometry) still runs every frame.
                        static std::vector<float> sDiagV;
                        static int sDiagCount = 0;
                        static uint64_t sDiagBuilt = 0;
                        static int sDiagPage = -1;
                        uint64_t dnow = nowNs();
                        // Rebuild on the throttle OR immediately when the page changes,
                        // so cycling to the other HUD doesn't show 250ms of stale geometry.
                        if (sDiagBuilt == 0 || dnow - sDiagBuilt > 250000000ULL || sDiagPage != diagPage) {
                            sDiagV.clear();
                            buildDiagOverlay(sDiagV, diagPage);
                            sDiagPage = diagPage;
                            sDiagCount = (int)(sDiagV.size()/6);
                            glBindBuffer(GL_ARRAY_BUFFER, gDiagVbo);
                            glBufferData(GL_ARRAY_BUFFER, sDiagV.size()*sizeof(float),
                                         sDiagV.data(), GL_DYNAMIC_DRAW);
                            sDiagBuilt = dnow;
                        }
                        int dc = sDiagCount;
                        if (dc > 0) {
                            // Flat panel floating below centre, ~1m away, tilted up ~30deg
                            // to face the eyes. Per-eye offset gives it real (close) depth.
                            Mat4 proj = mat4Perspective(1.5708f, 1.0f, 0.05f, 50.0f);  // ~90deg
                            const float a = -30.0f * 0.01745329f;   // -ve = face tilts UP
                            float ca = cosf(a), sa = sinf(a);
                            Mat4 rx = mat4Identity();
                            rx.m[5]=ca; rx.m[6]=sa; rx.m[9]=-sa; rx.m[10]=ca;
                            // 1.125 uniform (no glyph squish; columns condensed in layout).
                            Mat4 sc = mat4Identity(); sc.m[0]=1.125f; sc.m[5]=1.125f; sc.m[10]=1.125f;
                            Mat4 model = mat4Mul(mat4Mul(mat4Translate(0.0f, -0.34f, -1.0f), rx), sc);
                            glUseProgram(gProg);
                            glBindVertexArray(gDiagVao);
                            // PERF: this draw goes into gSwap, which ALVR's de-foveation
                            // pass already rendered+stored. On the Adreno (tiler) a fresh
                            // draw into an existing target makes the GPU LOAD every tile of
                            // the full ~2K^2 eye texture back into tilemem and STORE it
                            // again -- a full-res read+write per eye/frame, i.e. ~the same
                            // bandwidth the stream-shader fork removed. That's why opening
                            // the HUD costs real GPU time/heat. Fix: SCISSOR the draw to the
                            // panel's projected screen box so only the handful of tiles the
                            // panel actually covers (~15% of the frame) pay the load/store;
                            // the rest of the eye texture is never touched.
                            glEnable(GL_SCISSOR_TEST);
                            for (int e = 0; e < 2; e++) {
                                float exh = (e == 0 ? -softIpdM()*0.5f : softIpdM()*0.5f);
                                Mat4 mvp = mat4Mul(proj, mat4Mul(mat4Translate(-exh,0,0), model));
                                glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                                       GL_TEXTURE_2D, gSwap[e][gSwapIdx], 0);
                                glUniformMatrix4fv(gMvpLoc, 1, GL_FALSE, mvp.m);
                                // Project the panel's local AABB (x +/-0.30, y [-0.11,0.14],
                                // z=0 -- a touch larger than buildDiagOverlay's bg quad) to
                                // pixels and scissor to its bounding box (+8px margin).
                                const float cx[4] = { -0.30f, 0.30f, 0.30f, -0.30f };
                                const float cy[4] = { -0.11f, -0.11f, 0.14f, 0.14f };
                                float minX = 1e9f, minY = 1e9f, maxX = -1e9f, maxY = -1e9f;
                                for (int k = 0; k < 4; k++) {
                                    float X = cx[k], Y = cy[k];
                                    float cw = mvp.m[3]*X + mvp.m[7]*Y + mvp.m[15];
                                    float cxx = mvp.m[0]*X + mvp.m[4]*Y + mvp.m[12];
                                    float cyy = mvp.m[1]*X + mvp.m[5]*Y + mvp.m[13];
                                    if (cw < 1e-4f) cw = 1e-4f;
                                    float sx = ( cxx/cw*0.5f + 0.5f) * (float)gStreamW;
                                    float sy = ( cyy/cw*0.5f + 0.5f) * (float)gStreamH;
                                    if (sx < minX) minX = sx; if (sx > maxX) maxX = sx;
                                    if (sy < minY) minY = sy; if (sy > maxY) maxY = sy;
                                }
                                int rx = (int)floorf(minX) - 8, ry = (int)floorf(minY) - 8;
                                int rw = (int)ceilf(maxX) + 8 - rx, rh = (int)ceilf(maxY) + 8 - ry;
                                if (rx < 0) { rw += rx; rx = 0; }
                                if (ry < 0) { rh += ry; ry = 0; }
                                if (rx + rw > (int)gStreamW) rw = (int)gStreamW - rx;
                                if (ry + rh > (int)gStreamH) rh = (int)gStreamH - ry;
                                if (rw < 0) rw = 0; if (rh < 0) rh = 0;
                                glScissor(rx, ry, rw, rh);
                                glDrawArrays(GL_TRIANGLES, 0, dc);
                            }
                            glDisable(GL_SCISSOR_TEST);
                            glBindVertexArray(0);
                        }
                      } // end if (diagPage != 0)

                      glBindFramebuffer(GL_FRAMEBUFFER, 0);
                    }
                    // PIPELINE: fence THIS slot and FLUSH so the GPU starts it, but DO
                    // NOT block. The warp samples this slot a frame from now (via the
                    // previous-frame submit at the top of the next iteration), by which point the fence
                    // is long-signalled -> no torn texture, no render stall.
                    uint64_t _tEncStart = diagTiming ? nowNs() : 0;
                    if (gSwapFence[gSwapIdx]) glDeleteSync(gSwapFence[gSwapIdx]);
                    if (diagPage != 0) {
                        // HUD path: render + HUD were issued in our ctx, ordered after
                        // ALVR via glWaitSync; a fresh fence covers them both.
                        gSwapFence[gSwapIdx] = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
                        glFlush();
                    } else {
                        // No HUD -> our ctx issued NO work this frame, so the ALVR
                        // render fence (created on wgpu's ctx, already flushed) IS this
                        // slot's fence directly. Skips the per-frame glWaitSync + a
                        // redundant second fence the diag path needs. (Sync objects are
                        // shared, so next frame's glClientWaitSync on our ctx still works.)
                        gSwapFence[gSwapIdx] = alvrFence; alvrFence = 0;
                    }
                    if (alvrFence) glDeleteSync(alvrFence);   // safety (unreached paths)
                    // Stash this frame's render pose with its ring slot (the warp gets
                    // it next frame, paired with the texture it was rendered for).
                    gSwapVP[gSwapIdx][0] = outVP[0];
                    gSwapVP[gSwapIdx][1] = outVP[1];
                    if (diagTiming) { _tEnc = nowNs(); if (_tEnc - _tEncStart > _mEnc) _mEnc = _tEnc - _tEncStart; }

                    // The inaugural frame has no previous slot, so it wasn't submitted
                    // by the previous-frame block above -- submit it now (after its own
                    // render). The just-issued render fence isn't signalled yet, so the
                    // client-wait gets a wide ~50ms budget; happens only once per stream
                    // start / foveation re-sync, so the stall is cosmetic.
                    if (firstFrame) submitSlot(gSwapIdx, 50000000ULL /* ~50ms: render in flight */);
                    gPrevSwapIdx = gSwapIdx; gPrevSwapValid = true;
                }
                // Report the submit->present queue time. GetFractionalVsync()'s
                // fractional part = progress through the current refresh interval, so
                // time-to-next-vsync = (1-frac)*interval (sub-frame, exact phase). Fall
                // back to one interval if the oracle reads out of range (warp not up yet).
                float interval = 1e9f / (gRefreshHint > 1.0f ? gRefreshHint : 72.0f);
                double fv = PVR::GetFractionalVsync();
                double frac = fv - floor(fv);
                float timeToVsync = (frac >= 0.0 && frac <= 1.0) ? (float)((1.0 - frac) * interval)
                                                                 : interval;
                // Account for the +1-frame submit pipeline. In steady state the texture
                // we just rendered for `ts` is NOT the one handed to the warp this
                // iteration -- that's the PREVIOUS slot. `ts`'s texture is submitted NEXT
                // iteration and presents ~one interval LATER than the naive next-vsync,
                // so its true submit->photon queue is interval + time-to-vsync. Reporting
                // only time-to-vsync would understate frame `ts`'s latency by a whole
                // frame and bias the server's pose-extrapolation horizon short (the warp
                // hides head latency by reproject, but a truer horizon = less residual
                // correction). The inaugural frame has no pipeline (submitted the same
                // iteration), so it gets no +1.
                float vsyncQ = timeToVsync + (firstFrame ? 0.0f : interval);
                alvr_report_submit(ts, (uint64_t) vsyncQ);
                gSwapIdx = (gSwapIdx + 1) % kSwapLen;
                // Measure actual decoder output. submits = loop iterations that
                // presented a fresh frame; decoded = TRUE count of frames pulled
                // from the decoder (sum of `drained`, so coalesced frames count);
                // dropped = frames we discarded because >1 arrived in one iteration
                // (newest kept). decoded is the real decoder FPS; if decoded~72 but
                // submits<72 we're coalescing, not starving.
                static int sSubmits = 0, sDecoded = 0, sDropped = 0; static uint64_t vT0 = 0;
                sSubmits++;
                sDecoded += drained;
                if (drained > 1) sDropped += (drained - 1);
                uint64_t vNow = nowNs();
                if (vT0 == 0) vT0 = vNow;
                if (vNow - vT0 >= 1000000000ULL) {
                    LOGI("VIDEO: decoded=%d submits=%d dropped(coalesced)=%d  (panel 72Hz, negotiated %.0fHz)",
                         sDecoded, sSubmits, sDropped, gRefreshHint);
                    // Per-stage timing is only collected when the diag HUD is on
                    // (the probes above are gated on diagTiming); skip the log otherwise.
                    if (diagTiming)
                        LOGI("VIDEO-TIMING(ms,max): gap=%.1f render=%.1f enc=%.1f enq=%.1f  "
                             "(gap=submit-to-submit incl tracking/sleep; enq=submit to SDK warp, "
                             "incl vsync backpressure -- not warp compute; >13.3 = a coalesce)",
                             _mGap/1e6, _mRender/1e6, _mEnc/1e6, _mEnq/1e6);
                    // End-to-end latency from ALVR (server-acknowledged). QUEUE is the
                    // decoder jitter-buffer wait that max_buffering_frames sets -- the
                    // component that moves with the buffering-frames setting.
                    float st[6];
                    if (alvr_get_client_stats(st))
                        LOGI("VIDEO-LATENCY(ms): TOTAL=%.1f decode=%.1f QUEUE=%.1f render=%.1f vsyncQ=%.1f frameItvl=%.1f",
                             st[0], st[1], st[2], st[3], st[4], st[5]);
                    // Publish to the diagnostics HUD (per-second snapshot). Decoded/
                    // submit/dropped are always cheap; the per-stage maxima are only
                    // meaningful (and only collected) when the HUD is on.
                    gVidDecoded.store(sDecoded); gVidSubmit.store(sSubmits); gVidDropped.store(sDropped);
                    // Publish the fence-timeout tally (always -- it's a correctness
                    // signal, not per-stage timing) and log loudly only when nonzero.
                    gFenceTimeouts.store(sFenceTimeouts);
                    if (sFenceTimeouts > 0)
                        LOGI("%d warp-submit fence timeout(s) this second -- a slot wasn't "
                             "GPU-complete within budget (render overrun / thermal stall) -> possible tear",
                             sFenceTimeouts);
                    if (diagTiming) {
                        gGapMsX10.store((int)(_mGap/1e5)); gRenderMsX10.store((int)(_mRender/1e5));
                        gEncMsX10.store((int)(_mEnc/1e5)); gEnqMsX10.store((int)(_mEnq/1e5));
                    }
                    sSubmits = 0; sDecoded = 0; sDropped = 0; vT0 = vNow;
                    sFenceTimeouts = 0;
                    _mGap = _mRender = _mEnc = _mEnq = 0;
                }
            }
            // alvr_get_frame_timeout above blocks on the decoder for up to ~2 frame
            // intervals, so under healthy streaming the loop is paced by frame arrival
            // (~one vsync/iteration) and the floor below sleeps ~0. But when the device
            // falls behind -- a render/thermal overrun backs frames up in the decoder --
            // the NEXT alvr_get_frame_timeout returns a coalesced burst INSTANTLY (no
            // block) and the loop would spin faster than the 72Hz panel. Two warp submits
            // then land inside one refresh, and the legacy DIATW latches each eye
            // SEPARATELY (CameraEndFrame/ChangeRenderPose per eye) -> a clustered submit
            // pairs eye0 from one frame with eye1 from the next = per-eye pose desync
            // (the video detaches and swims on head turn). So pace each iteration to >=
            // one vsync (the same bounded trailing floor the lobby submit path uses) so
            // two submits can never share a refresh. Only bites in the burst case; costs
            // nothing under healthy streaming where the decoder block already paced us.
            // Do NOT target above 72Hz or drop the floor to 0 -- that reintroduces the
            // per-eye submit desync.
            {
                const uint64_t kVsyncNs = (uint64_t)(1e9 / 72.0);   // 72Hz panel (do NOT exceed)
                uint64_t work = nowNs() - tLoopStart;
                if (work < kVsyncNs) usleep((useconds_t)((kVsyncNs - work) / 1000));
            }
            frame++; framesWithSurface++;
            continue;
        }

        // (In the manual lobby the video decoder is fully stopped -- see the decode
        // pause block above -- so there are no frames to drain here. Audio keeps
        // running on its own client_core path, and the connection stays alive.)

        // ---- lobby (pre-stream / between streams): world-locked IP/status/model
        // HUD + floor grid + eye-gaze marker (Neo 2 EYE). Rendered in BOTH render
        // modes: HW compositor renders each eye into a ring texture fed to the SDK
        // warp (it does distortion + reprojection + present); self-present does its
        // own distortion + window present. ----
        // View from head pose: inverse head rotation, then -head position (the grid,
        // gaze marker, AND the HUD are all rendered through this head-tracked view).
        uint64_t tIterStart = nowNs();   // pace this iteration's submit cadence (see trailing sleep)
        Mat4 headRot  = quatToMat4(qx, qy, qz, qw);
        Mat4 invRot   = mat4Transpose3x3(headRot);
        Mat4 viewBase = mat4Mul(invRot, mat4Translate(-px, -py, -pz));

        // Render at the SDK's render FOV (square target) so the distortion maps
        // correctly, matching the video path.
        float lobbyFovDeg = (fEyeTextureFov0 > 1.0f) ? fEyeTextureFov0 : 101.0f;
        float fovy = lobbyFovDeg * (float)M_PI / 180.0f;
        Mat4 proj  = mat4Perspective(fovy, 1.0f, 0.05f, 250.0f);   // square target -> aspect 1

        // ---- HUD content: device IP (line 1) + client status (line 2). A
        // WORLD-LOCKED panel: anchored once in front of where the user is
        // looking at lobby entry, then rendered through the head-tracked
        // view so it stays planted in space (not following the head). Built once per
        // frame in panel-local meters (same for both eyes).
        // refresh occasionally, and keep retrying while we still have no numeric IP
        if ((frame % 120) == 0 || !(gIpText[0] >= '0' && gIpText[0] <= '9')) refreshDeviceIp();
        const float kHudDist = 2.0f;         // panel distance in front of the user at anchor time
        // Three lines, sized by "points" relative to the IP line: model = IP+2pt,
        // status = IP-2pt. We model 1 "point" as 10% of the IP pixel size.
        const float pxI = 0.012f;            // IP (base) metres per font pixel
        const float pxM = pxI * 1.2f;        // model: +2pt (larger)
        const float pxH = pxI * 0.8f;        // hostname: -2pt (smaller)
        const float pxS = pxI * 0.8f;        // status: -2pt (smaller)
        // status colour: green=connected, yellow=connecting, cyan=searching, red=down
        float sr=1, sg=0.3f, sb=0.3f;        // default DISCONNECTED (red)
        if      (!strcmp(gStatusText, "CONNECTED"))  { sr=0.2f; sg=1.0f; sb=0.3f; }
        else if (!strcmp(gStatusText, "CONNECTING")) { sr=1.0f; sg=0.9f; sb=0.2f; }
        else if (!strcmp(gStatusText, "SEARCHING"))  { sr=0.3f; sg=0.8f; sb=1.0f; }
        // Stack model / IP / hostname / status (top->bottom), vertically centred.
        // The hostname line (the name to "Trust" on the PC) shows only once known.
        bool haveHost = gHostnameText[0] != 0;
        const float gap = 3.0f * pxI;
        float hM = 7*pxM, hI = 7*pxI, hH = 7*pxH, hS = 7*pxS;
        float total = hM + gap + hI + gap + hS + (haveHost ? (gap + hH) : 0);
        float yTopM = total * 0.5f;
        float yTopI = yTopM - hM - gap;
        float yTopH = yTopI - hI - gap;                       // hostname (if shown)
        float yTopS = (haveHost ? yTopH - hH - gap : yTopI - hI - gap);
        // SETTINGS button below the info text -> opens the unified settings window.
        // Rect is recomputed each frame (its Y depends on whether the hostname
        // line is shown) and reused for both the geometry and the hit-test.
        UiRect settingsBtn = { 0.0f, yTopS - hS - 0.07f, 0.34f, 0.085f };
        static bool sSettingsBtnHot = false;   // hover carried from the prev frame's hit-test
        bool setBtnHot = sSettingsBtnHot && !gSettingsOpen;
        // THROTTLE the HUD text rebuild+upload. The 4 lines + SETTINGS button only
        // change when the strings, the button hover, or the theme change -- not every
        // frame -- so regenerate the glyph quads and re-upload the VBO only then. The
        // rect above is still computed every frame for the hit-test below.
        static GLsizeiptr sTextCap = 0;
        static int sTextVertCount = 0;
        static uint32_t sHudSig = 0; static bool sHudHave = false;
        uint32_t hudSig = 2166136261u;
        { const char *ss[4] = { gModelText, gIpText, gHostnameText, gStatusText };
          for (int k = 0; k < 4; k++) for (const char *p = ss[k]; *p; ++p) hudSig = (hudSig ^ (unsigned char)*p) * 16777619u; }
        hudSig = (hudSig ^ (setBtnHot ? 0x9E3779B9u : 0u)) * 16777619u;
        hudSig = (hudSig ^ (gThemeAmber.load() ? 0x85EBCA77u : 0u)) * 16777619u;
        if (!sHudHave || hudSig != sHudSig) {
            static std::vector<float> tverts; tverts.clear();   // reuse capacity
            appendTextLine(tverts, gModelText,  yTopM, pxM, kUiTitle[0], kUiTitle[1], kUiTitle[2]);  // model: themed title
            appendTextLine(tverts, gIpText,     yTopI, pxI, 1.0f, 1.0f, 1.0f);  // IP: white
            if (haveHost)
                appendTextLine(tverts, gHostnameText, yTopH, pxH, 1.0f, 0.8f, 0.4f);  // hostname: amber
            appendTextLine(tverts, gStatusText, yTopS, pxS, sr, sg, sb);        // status: state colour
            uiButton(tverts, settingsBtn, "SETTINGS", setBtnHot);
            sTextVertCount = (int)(tverts.size() / 6);
            if (sTextVertCount > 0) uploadDynamicVbo(gTextVbo, tverts, sTextCap);
            sHudSig = hudSig; sHudHave = true;
        }
        int textVertCount = sTextVertCount;
        // World-anchor the HUD once per lobby entry: plant it kHudDist in front of
        // the current head, at head height, facing back toward the user, using
        // YAW ONLY (no pitch/roll) so it stands upright. Captured into hudWorld and
        // reused every frame thereafter -> the panel is fixed in space, not tracked.
        if (!hudAnchored) {
            // head forward = local -Z in world (3rd column of the rotation, negated)
            float fx = -headRot.m[8], fz = -headRot.m[10];
            float fn = sqrtf(fx*fx + fz*fz);
            if (fn > 1e-5f) { fx /= fn; fz /= fn; } else { fx = 0; fz = -1; }  // flat forward
            float ax = px + fx * kHudDist, ay = py, az = pz + fz * kHudDist;   // anchor position
            // yaw so the panel's local +Z (its readable face) points back at the
            // user (-forward): RotY(phi) maps +Z -> (sin phi, 0, cos phi) = (-fx,0,-fz)
            float cphi = -fz, sphi = -fx;        // already unit-length
            Mat4 m = mat4Identity();
            m.m[0] = cphi; m.m[2] = -sphi;       // column-major yaw about +Y
            m.m[8] = sphi; m.m[10] = cphi;
            m.m[12] = ax;  m.m[13] = ay; m.m[14] = az;   // translation
            hudWorld = m;

            // Unified SETTINGS window: same facing as the info HUD, planted a touch
            // CLOSER to the user so it overlays the info area when open. Same yaw.
            const float kSetDist = kHudDist - 0.25f;
            float setx = px + fx * kSetDist, setz = pz + fz * kSetDist;
            Mat4 mset = mat4Identity();
            mset.m[0] = cphi; mset.m[2] = -sphi;
            mset.m[8] = sphi; mset.m[10] = cphi;
            mset.m[12] = setx; mset.m[13] = py; mset.m[14] = setz;
            settingsWorld = mset;

            hudAnchored = true;
        }

        // ===== UNIFIED LOBBY POINTER =========================================
        // One ray drives EVERY interactable element (the settings panel controls and
        // the EQ): a controller LASER when any controller is connected,
        // otherwise the head-gaze ray. The gaze reticle is shown ONLY when there's
        // no controller. "grab" = controller trigger held / OK button held; a
        // single "click" = trigger press-edge (controller) or OK click (gaze).
        float ptrOx=px, ptrOy=py, ptrOz=pz;
        float ptrDx=-headRot.m[8], ptrDy=-headRot.m[9], ptrDz=-headRot.m[10];
        bool  eqUsingController = false;   // (name kept for the laser draw below)
        bool  ptrGrab = gOkHeld.load();
        float ptrStickY = 0.0f;            // dominant-hand thumbstick Y (-1..1, up = +)
        bool  recenterDown = false;        // app/menu button (either hand) -> re-anchor panels
        // Per-hand world pose for drawing the controller models (0=L,1=R).
        bool  ctrlConn[2] = {false,false};
        float ctrlPos[2][3] = {{0,0,0},{0,0,0}};
        float ctrlQuat[2][4] = {{0,0,0,1},{0,0,0,1}};
        {
            CtrlState cc[2];
            { std::lock_guard<std::mutex> lk(gCtrlMutex); cc[0]=gCtrl[0]; cc[1]=gCtrl[1]; }
            for (int hh=0; hh<2; hh++)
                if (cc[hh].conn==1 && cc[hh].keyCount>5 && cc[hh].keys[5]!=0) recenterDown = true;
            // Capture both hands' world pose for the controller-model draw. Same
            // conversion as the laser path: pos*0.001, quat = (-x,-y,z,w).
            for (int hh=0; hh<2; hh++) if (cc[hh].conn==1) {
                ctrlConn[hh] = true;
                ctrlPos[hh][0]=cc[hh].pos[0]*0.001f; ctrlPos[hh][1]=cc[hh].pos[1]*0.001f; ctrlPos[hh][2]=cc[hh].pos[2]*0.001f;
                Quat cq = quatNorm({ -cc[hh].q[0], -cc[hh].q[1], cc[hh].q[2], cc[hh].q[3] });
                ctrlQuat[hh][0]=cq.x; ctrlQuat[hh][1]=cq.y; ctrlQuat[hh][2]=cq.z; ctrlQuat[hh][3]=cq.w;
            }
            auto trig = [](const CtrlState &s){
                return (s.keyCount>2 && s.keys[2]!=0) || (s.keyCount>8 && s.keys[8]>40);
            };
            // Off-hand dominance: pulling the trigger on a connected NON-dominant
            // controller claims the laser (so either hand can become dominant).
            for (int hh=0; hh<2; hh++)
                if (hh != gDominantHand && cc[hh].conn==1 && trig(cc[hh])) gDominantHand = hh;
            int h = (cc[gDominantHand].conn==1) ? gDominantHand
                  : (cc[0].conn==1 ? 0 : (cc[1].conn==1 ? 1 : -1));
            if (h >= 0) {
                // Same conversion as the streaming controller path (CV service returns
                // a world-frame pose because we feed it the head pose): take pos*0.001
                // and (-x,-y,z,w) DIRECTLY -- no headRot/headPos, no yaw trim (0 angle).
                Quat oq = quatNorm({ -cc[h].q[0], -cc[h].q[1], cc[h].q[2], cc[h].q[3] });
                float qx2=oq.x, qy2=oq.y, qz2=oq.z, qw2=oq.w;
                ptrDx = -2.0f*(qx2*qz2 + qw2*qy2);
                ptrDy = -2.0f*(qy2*qz2 - qw2*qx2);
                ptrDz = -(1.0f - 2.0f*(qx2*qx2 + qy2*qy2));
                float fwd0x = ptrDx, fwd0y = ptrDy, fwd0z = ptrDz;   // untilted local forward
                // controller local up axis (+Y)
                float ux = 2.0f*(qx2*qy2 - qw2*qz2);
                float uy = 1.0f - 2.0f*(qx2*qx2 + qz2*qz2);
                float uz = 2.0f*(qy2*qz2 + qw2*qx2);
                // Tilt the laser DOWN 30deg in the controller's local frame (toward
                // its underside) -- more natural pointing ergonomics. Pitch the
                // forward vector toward -up (up = controller local +Y axis).
                {
                    const float th = 30.0f * 0.01745329f;
                    float ca = cosf(th), sa = sinf(th);
                    ptrDx = ptrDx*ca - ux*sa;
                    ptrDy = ptrDy*ca - uy*sa;
                    ptrDz = ptrDz*ca - uz*sa;
                }
                // Emit from a FIXED point on the model -- the front-top tip near
                // the ring/trigger -- expressed in the controller's OWN axes
                // (untilted forward + local up), so it stays anchored there in
                // every orientation (offsetting along the tilted beam looked
                // centred head-on but drifted off from the side).
                const float kFrontOff = 0.075f;    // toward the front tip (pointing axis)
                const float kUpOff    = -0.005f;   // down the front face toward the tip (was +0.025)
                ptrOx = cc[h].pos[0]*0.001f + fwd0x*kFrontOff + ux*kUpOff;
                ptrOy = cc[h].pos[1]*0.001f + fwd0y*kFrontOff + uy*kUpOff;
                ptrOz = cc[h].pos[2]*0.001f + fwd0z*kFrontOff + uz*kUpOff;
                ptrGrab = trig(cc[h]);
                eqUsingController = true;
                // thumbstick Y for menu page-scrolling (keys[1]: center 128, 0..255)
                if (cc[h].keyCount > 1) ptrStickY = (cc[h].keys[1] - 128) / 127.0f;
            }
        }

        // Recenter: the app/menu button re-anchors the lobby panels in front of the
        // head on its rising edge (like the streaming recenter).
        {
            static bool recenterPrev = false;
            if (recenterDown && !recenterPrev) hudAnchored = false;
            recenterPrev = recenterDown;
        }

        bool okClicked = gOkClick.exchange(false);
        static bool sPtrGrabPrev = false;
        bool grabEdge = ptrGrab && !sPtrGrabPrev;
        bool clickEdge = eqUsingController ? grabEdge : okClicked;   // unified single-click
        float eqLaserOx=ptrOx, eqLaserOy=ptrOy, eqLaserOz=ptrOz;     // laser draw inputs
        float eqLaserDx=ptrDx, eqLaserDy=ptrDy, eqLaserDz=ptrDz;
        float eqLaserLen = 100.0f;   // 100 m; clipped to a panel hit (below)

        // Ray vs a panel plane (column-major model matrix W). Returns hit + local x/y
        // (panel metres) + distance t. Also clips the laser length to the hit.
        auto rayPanel = [&](const Mat4 &W, float &lx, float &ly, float &t)->bool {
            float Rx=W.m[0],Ry=W.m[1],Rz=W.m[2];
            float Ux=W.m[4],Uy=W.m[5],Uz=W.m[6];
            float Nx=W.m[8],Ny=W.m[9],Nz=W.m[10];
            float Qx=W.m[12],Qy=W.m[13],Qz=W.m[14];
            float denom = ptrDx*Nx + ptrDy*Ny + ptrDz*Nz;
            if (fabsf(denom) <= 1e-5f) return false;
            t = ((Qx-ptrOx)*Nx + (Qy-ptrOy)*Ny + (Qz-ptrOz)*Nz) / denom;
            if (t <= 0.0f) return false;
            float hx=ptrOx+ptrDx*t, hy=ptrOy+ptrDy*t, hz=ptrOz+ptrDz*t;
            float rx=hx-Qx, ry=hy-Qy, rz=hz-Qz;
            lx = rx*Rx + ry*Ry + rz*Rz;
            ly = rx*Ux + ry*Uy + rz*Uz;
            if (eqUsingController && t < eqLaserLen) eqLaserLen = t - 0.01f;
            return true;
        };

        // ===== UNIFIED SETTINGS WINDOW =======================================
        // The info-HUD SETTINGS button opens ONE world-locked panel: a category
        // sidebar (VIDEO/AUDIO/DEBUG) + a grab-anywhere-to-scroll content area +
        // a reference scrollbar. Every lobby control lives inside it. Content
        // widgets are addressed in builder-local coords (cx,cy = panel-local minus
        // the scroll/centre offset from settingsMeasure()).
        MenuHover mh;                    // hovered content item (data-driven menu)
        int  setTabHover   = -1;         // hovered sidebar tab
        bool setCloseHover = false;
        bool lobbyHover = false;
        static int   sDragMode = 0;     // 0=none, 1=scroll, 2=widget (decided on grab edge)
        static float sScrollLastLy = 0;

        // The panels are anchored in world space; the pointer ray is in the same
        // frame, so hit-test directly against each panel's world matrix.
        Mat4 hudHit = hudWorld;
        Mat4 setHit = settingsWorld;

        // ---- SETTINGS open button (lives on the info HUD) ----
        sSettingsBtnHot = false;
        if (!gSettingsOpen) {
            float lx, ly, t;
            if (rayPanel(hudHit, lx, ly, t) && uiHit(settingsBtn, lx, ly)) {
                sSettingsBtnHot = true; lobbyHover = true;
                if (clickEdge) { gSettingsOpen = true; sDragMode = 0; }
            }
        }

        static std::vector<float> sverts; sverts.clear();   // reuse capacity (settings panel)
        int sliderVertCount = 0;
        float setContentH = 0;
        if (gSettingsOpen) {
            float setOffX = 0, setOffY = 0;
            settingsMeasure(setOffX, setOffY, setContentH);
            bool scrollable = setContentH > kSetViewportH + 1e-4f;

            MenuModel &model = settingsModel();
            MenuCategory &cat = model[gSettingsCat];

            float lx, ly, t;
            bool onPanel = rayPanel(setHit, lx, ly, t);
            bool inContent = onPanel && lx >= kCtX0 && lx <= kCtX1 && ly <= kCtTop && ly >= kCtBot;
            float cx = lx - setOffX, cy = ly - setOffY;   // builder-local content coords

            // ---- chrome: close + sidebar tabs (always clickable) ----
            if (onPanel) {
                if (uiHit(kSetClose, lx, ly)) { setCloseHover = true; lobbyHover = true; }
                for (int i = 0; i < (int)model.size(); i++)
                    if (uiHit(settingsTabRect(i), lx, ly)) { setTabHover = i; lobbyHover = true; }
            }

            // ---- content hover (generic, over the active category's items) ----
            if (inContent) mh = menuHit(cat, cx, cy);
            if (mh.item >= 0 || setTabHover >= 0 || setCloseHover) lobbyHover = true;

            // ---- grab-mode latch (decided once, on the grab edge) ----
            // mh.grab = pointer is on a press/drag widget (toggle/stepper/fader knob/
            // button); empty content (or EQ track away from a knob) -> scroll instead.
            if (grabEdge) {
                if (inContent && setTabHover < 0 && !setCloseHover)
                    sDragMode = mh.grab ? 2 : (scrollable ? 1 : 2);
                else
                    sDragMode = 2;   // chrome / off-panel: treat as discrete
                sScrollLastLy = ly;
            }
            if (!ptrGrab) sDragMode = 0;

            // ---- scroll (grab on empty content) ----
            if (sDragMode == 1 && ptrGrab && scrollable) {
                gSettingsScroll[gSettingsCat] += (ly - sScrollLastLy);   // content follows the hand
                sScrollLastLy = ly;     // clamp applied in settingsMeasure() next frame
            }

            // ---- thumbstick page-scroll (whole page only, never sliders) ----
            // Independent of the grab/drag path, so it can't grab a fader. Push up
            // = move toward the top of the page; down = toward the bottom.
            // When locomotion is on the LEFT stick walks, so menu scroll uses the
            // RIGHT stick Y; otherwise the dominant-hand stick Y.
            float scrollStickY = ptrStickY;
            if (scrollable && fabsf(scrollStickY) > 0.18f) {
                gSettingsScroll[gSettingsCat] -= scrollStickY * 0.01875f;  // clamp next frame
            }

            // ---- discrete clicks on chrome ----
            if (clickEdge && setCloseHover) { gSettingsOpen = false; sDragMode = 0; }
            if (clickEdge && setTabHover >= 0 && setTabHover != gSettingsCat) { gSettingsCat = setTabHover; sDragMode = 0; }

            // ---- content actions (suppressed while scrolling) ----
            // ONE generic dispatch over the active category's items: hold-to-repeat
            // steppers, fader drag + release-commit, toggle/button clicks, and the
            // custom EQ panel is ALL handled inside menuApply(). Menu
            // side effects that need the render thread (GL rebuild, JNIEnv, loco)
            // are raised as dirty flags and applied just below.
            if (sDragMode != 1)
                menuApply(gSettingsCat, cat, mh, clickEdge, ptrGrab, cx, cy);
            if (!ptrGrab && gEqGrabbing) { gEqGrabbing = false; gEqActiveBand = -1; }
            if (gEqDirty && nowNs() - gEqChangeNs > 500000000ULL) { saveEqProfile(); gEqDirty = false; }

            // THROTTLE the settings-panel rebuild+upload. settingsMeasure / hit-test
            // / menuApply above still run every frame so dragging stays responsive, but
            // the (relatively heavy, esp. the 16-band EQ) geometry build + VBO upload
            // only happen when something that affects the drawn pixels changed. The
            // signature covers category/theme/hover/scroll-offset + the menu's get()
            // values + dropdown-open + EQ state (menuValueSig deliberately skips the
            // CUSTOM EQ item, so its gains/active-band/preset are mixed in explicitly).
            uint32_t panelSig = 2166136261u;
            auto pmix = [&](long x){ for (int b = 0; b < 8; b++) { panelSig = (panelSig ^ (unsigned char)(x & 0xff)) * 16777619u; x >>= 8; } };
            pmix(gSettingsCat); pmix(gThemeAmber.load() ? 1 : 0);
            pmix(setTabHover); pmix(setCloseHover ? 1 : 0);
            pmix(mh.item); pmix(mh.part); pmix(mh.grab ? 1 : 0);
            pmix(lroundf(setOffX * 2000.0f)); pmix(lroundf(setOffY * 2000.0f)); pmix(lroundf(setContentH * 2000.0f));
            pmix((long)menuValueSig(cat));
            for (const auto &it : cat.items) pmix(it.dropOpen ? 1 : 0);
            pmix(gEqActiveBand); pmix(gEqPresetOpen ? 1 : 0); pmix(gEqPresetIdx);
            for (int i = 0; i < kEqBands; i++) pmix(lroundf(gEqGains[i] * 2000.0f));
            static uint32_t sPanelSig = 0; static bool sPanelHave = false;
            static int sSliderVertCount = 0; static GLsizeiptr sSetCap = 0;
            if (!sPanelHave || panelSig != sPanelSig) {
                buildSettingsPanel(sverts, setOffX, setOffY, setContentH, mh, setTabHover, setCloseHover);
                sSliderVertCount = (int)(sverts.size() / 6);
                if (sSliderVertCount > 0) uploadDynamicVbo(gSliderVbo, sverts, sSetCap);
                sPanelSig = panelSig; sPanelHave = true;
            }
            sliderVertCount = sSliderVertCount;
        } else {
            if (!ptrGrab && gEqGrabbing) { gEqGrabbing = false; gEqActiveBand = -1; }
        }

        sPtrGrabPrev = ptrGrab;
        bool eqShowReticle = (!eqUsingController && lobbyHover);

        // ---- apply menu-requested side effects ----
        // The data-driven menu has no GL context / JNIEnv / locomotion access, so its
        // callbacks raise these flags; perform the real effect here where we do.
        if (gEyeTrackReapply.exchange(false)) applyServerEyeTracking(gStreaming);
        if (gBrightnessApply.exchange(false)) applyHmdBrightness(gBrightnessFrac.load(), env);

        // Render the lobby scene (world floor grid + world-locked HUD + gaze disc)
        // into the currently-bound FBO for the given per-eye projection + view.
        // Shared by the HW-compositor and self-present paths.
        auto drawLobbyScene = [&](const Mat4 &sproj, const Mat4 &sview, const Mat4 &sEyeShift) {
            // World-fixed geometry (floor grid, anchored panels) draws through the
            // head-tracked view; player-attached geometry (laser, gaze) uses it too.
            const Mat4 &worldView = sview;
            if (gGridVertCount > 0) {
                glEnable(GL_DEPTH_TEST); glDisable(GL_CULL_FACE);
                glUseProgram(gProg);
                Mat4 gridMvp = mat4Mul(sproj, worldView);   // model = identity (floor at y=0)
                glUniformMatrix4fv(gMvpLoc, 1, GL_FALSE, gridMvp.m);
                glBindVertexArray(gGridVao);        // wireframe floor grid
                glDrawArrays(GL_LINES, 0, gGridVertCount);
                glBindVertexArray(0);
            }
            glDisable(GL_DEPTH_TEST); glDisable(GL_CULL_FACE);
            if (textVertCount > 0) {
                Mat4 hudMvp = mat4Mul(sproj, mat4Mul(worldView, hudWorld));   // world-locked HUD
                glUseProgram(gProg);
                glBindVertexArray(gTextVao);
                glUniformMatrix4fv(gMvpLoc, 1, GL_FALSE, hudMvp.m);
                glDrawArrays(GL_TRIANGLES, 0, textVertCount);
                glBindVertexArray(0);
            }
            // Unified SETTINGS window (world-locked, in front, overlays the HUD).
            if (sliderVertCount > 0) {
                Mat4 setMvp = mat4Mul(sproj, mat4Mul(worldView, settingsWorld));
                glUseProgram(gProg);
                glBindVertexArray(gSliderVao);
                glUniformMatrix4fv(gMvpLoc, 1, GL_FALSE, setMvp.m);
                glDrawArrays(GL_TRIANGLES, 0, sliderVertCount);
                glBindVertexArray(0);
            }
            // Controller laser beam (world-space) when a controller drives the pointer.
            if (eqUsingController) {
                const float L = eqLaserLen;
                float exr = eqLaserOx + eqLaserDx*L, eyr = eqLaserOy + eqLaserDy*L, ezr = eqLaserOz + eqLaserDz*L;
                // Solid red beam, full length (no fade).
                float lv[12] = { eqLaserOx,eqLaserOy,eqLaserOz, 1.0f,0.25f,0.25f,
                                 exr,eyr,ezr,                   1.0f,0.25f,0.25f };
                glBindBuffer(GL_ARRAY_BUFFER, gLaserVbo);
                glBufferData(GL_ARRAY_BUFFER, sizeof(lv), lv, GL_DYNAMIC_DRAW);
                Mat4 lMvp = mat4Mul(sproj, sview);   // verts already in world space
                glUseProgram(gProg);
                glBindVertexArray(gLaserVao);
                glUniformMatrix4fv(gMvpLoc, 1, GL_FALSE, lMvp.m);
                glDrawArrays(GL_LINES, 0, 2);
                glBindVertexArray(0);
            }
            // Controller models: wireframe meshes attached to each live controller
            // pose (room frame, like the laser). Mesh is pre-scaled to metres and
            // centred at origin; model = T(pos) * R(quat).
            for (int h = 0; h < 2; h++) {
                if (!ctrlConn[h] || gCtrlVertCount[h] <= 0) continue;
                Mat4 M = quatToMat4(ctrlQuat[h][0], ctrlQuat[h][1], ctrlQuat[h][2], ctrlQuat[h][3]);
                M.m[12] = ctrlPos[h][0]; M.m[13] = ctrlPos[h][1]; M.m[14] = ctrlPos[h][2];
                Mat4 cMvp = mat4Mul(sproj, mat4Mul(sview, M));
                glUseProgram(gProg);
                glBindVertexArray(gCtrlVao[h]);
                glUniformMatrix4fv(gMvpLoc, 1, GL_FALSE, cMvp.m);
                glDrawArrays(GL_LINES, 0, gCtrlVertCount[h]);
                glBindVertexArray(0);
            }
            // Eye-gaze debug marker: the green disc at the live RAW Tobii gaze point.
            // Gated by the persisted EYE DEBUG toggle (in the settings panel).
            if (gEyeDebugOn.load() && gEyeOnline.load() && gGazeValid.load() && gGazeVertCount > 0) {
                const float gd = 1.6f;
                float g0=gGazeLocal[0].load(), g1=gGazeLocal[1].load(), g2=gGazeLocal[2].load();
                float dx = headRot.m[0]*g0 + headRot.m[4]*g1 + headRot.m[8]*g2;
                float dy = headRot.m[1]*g0 + headRot.m[5]*g1 + headRot.m[9]*g2;
                float dz = headRot.m[2]*g0 + headRot.m[6]*g1 + headRot.m[10]*g2;
                float dn = sqrtf(dx*dx+dy*dy+dz*dz);
                if (dn > 1e-5f) { dx/=dn; dy/=dn; dz/=dn; }
                // Orient the disc TANGENT to the gaze hemisphere: its +Z normal points
                // from the marker back toward the head, so the dot sits flush on a
                // half-sphere of radius gd and always faces the user. (Using headRot
                // instead would give every dot a head-forward normal, so they'd lie on a
                // flat plane in front of you and go edge-on at steep gaze angles.)
                float nx=-dx, ny=-dy, nz=-dz;                 // marker -> head
                float rx = 1.0f*nz - 0.0f*ny;                 // right = worldUp(0,1,0) x n
                float ry = 0.0f*nx - 0.0f*nz;
                float rz = 0.0f*ny - 1.0f*nx;
                float rn = sqrtf(rx*rx+ry*ry+rz*rz);
                if (rn < 1e-4f) { rx=1; ry=0; rz=0; rn=1; }   // gaze ~vertical: stable fallback
                rx/=rn; ry/=rn; rz/=rn;
                float ux = ny*rz - nz*ry;                     // up = n x right
                float uy = nz*rx - nx*rz;
                float uz = nx*ry - ny*rx;
                Mat4 face = mat4Identity();
                face.m[0]=rx; face.m[1]=ry; face.m[2]=rz;
                face.m[4]=ux; face.m[5]=uy; face.m[6]=uz;
                face.m[8]=nx; face.m[9]=ny; face.m[10]=nz;
                Mat4 mk = mat4Mul(mat4Translate(px+dx*gd, py+dy*gd, pz+dz*gd), face);
                Mat4 mkMvp = mat4Mul(sproj, mat4Mul(sview, mk));
                glUseProgram(gProg);
                glBindVertexArray(gGazeVao);
                glUniformMatrix4fv(gMvpLoc, 1, GL_FALSE, mkMvp.m);
                glDrawArrays(GL_TRIANGLES, 0, gGazeVertCount);
                glBindVertexArray(0);
            }
            // Head-gaze crosshair: HEAD-LOCKED at view centre (eye shift only, no
            // head rotation/translation) so it marks where the gaze ray points.
            // Drawn last, on top. Shown (in gaze mode) when the gaze is over an
            // interactable lobby control.
            if (gReticleVertCount > 0 && eqShowReticle) {
                const float kReticleDist = 1.5f;
                Mat4 rMvp = mat4Mul(sproj, mat4Mul(sEyeShift, mat4Translate(0, 0, -kReticleDist)));
                glUseProgram(gProg);
                glBindVertexArray(gReticleVao);
                glUniformMatrix4fv(gMvpLoc, 1, GL_FALSE, rMvp.m);
                glDrawArrays(GL_TRIANGLES, 0, gReticleVertCount);
                glBindVertexArray(0);
            }
            glEnable(GL_DEPTH_TEST); glEnable(GL_CULL_FACE);
        };

        // ===== HW-COMPOSITOR LOBBY: render each eye into its ring texture and hand
        // them to the SDK warp (same submit path as the video). The warp owns the
        // window, applies lens distortion + async reprojection, and presents.
        // RENDER EVERY FRAME: do NOT gate this on a content-dirty signature. This
        // lobby submit path does NOT async-reproject a re-submitted stale frame to the
        // live head pose, so between redraws the world-locked view would freeze and
        // head motion judder. The lobby is only shown pre-stream/between streams, so
        // drawing it every frame costs no in-game thermal headroom. =====
        {
            // We're back in the lobby -- cancel any pending deferred free of the
            // eye-texture ring (a manual-lobby toggle landed inside the countdown), so
            // we keep reusing the live ring instead of freeing then rebuilding it.
            lobbyFreeDelay = -1;
            if (!gLobbyEyeReady && rtInited) {   // rebuild after a stream freed it
                eglMakeCurrent(dpy, pbuf, pbuf, ctx);
                buildLobbyTarget();
                lastRenderedIdx = -1; lobbyEyeIdx = 0;
            }
            if (gLobbyEyeReady && rtInited) {
                {
                    eglMakeCurrent(dpy, pbuf, pbuf, ctx);   // stay offscreen; warp owns the window
                    if (gGridThemeDirty.exchange(false)) { buildGridFloor(); buildControllerMeshes(); }   // recolour floor + controllers
                    for (int eye = 0; eye < 2; eye++) {
                        float ex = (eye == 0 ? -softIpdM()*0.5f : softIpdM()*0.5f);
                        Mat4 eyeShift = mat4Translate(-ex, 0, 0);
                        Mat4 view = mat4Mul(eyeShift, viewBase);
                        glBindFramebuffer(GL_FRAMEBUFFER, gLobbyFbo);
                        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                               GL_TEXTURE_2D, gLobbyEye[eye][lobbyEyeIdx], 0);
                        glDisable(GL_SCISSOR_TEST);
                        glViewport(0, 0, kLobbySz, kLobbySz);
                        glClearColor(0, 0, 0, 1);
                        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                        drawLobbyScene(proj, view, eyeShift);
                    }
                    glBindFramebuffer(GL_FRAMEBUFFER, 0);
                    // PIPELINE: fence THIS slot + glFlush so the GPU starts it, but
                    // DON'T block on it. We hand the warp the slot we rendered LAST frame
                    // (below) -- GPU-complete a full frame ago -- which avoids a per-frame
                    // glClientWaitSync stall (up to 5ms) on the lobby's critical path.
                    // Same +1-frame submit pipeline the video path uses; the lobby is
                    // shown only pre/between streams, so the one extra frame of latency is
                    // invisible and both eyes still submit together (no per-eye desync).
                    if (gLobbyFence[lobbyEyeIdx]) glDeleteSync(gLobbyFence[lobbyEyeIdx]);
                    gLobbyFence[lobbyEyeIdx] = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
                    glFlush();
                    gLobbyPoseQ[lobbyEyeIdx] = quatNorm({ qx, qy, qz, qw });   // pose this slot was rendered at
                    gLobbyPoseP[lobbyEyeIdx][0] = px; gLobbyPoseP[lobbyEyeIdx][1] = py; gLobbyPoseP[lobbyEyeIdx][2] = pz;
                    int thisIdx = lobbyEyeIdx;
                    lobbyEyeIdx = (lobbyEyeIdx + 1) % kLobbyRing;
                    // Submit the PREVIOUS frame's slot; the very first frame has none, so
                    // fall back to the slot we just rendered (and wait on its fresh fence).
                    lobbySubmitIdx = (lastRenderedIdx >= 0) ? lastRenderedIdx : thisIdx;
                    lastRenderedIdx = thisIdx;
                    // Make sure the slot we're about to hand the warp is GPU-complete. The
                    // pipelined slot's fence was created last frame -> already signalled, so
                    // this returns immediately; the first-frame fallback waits on a fresh
                    // render under a wide one-off budget.
                    if (gLobbyFence[lobbySubmitIdx]) {
                        uint64_t budget = (lobbySubmitIdx == thisIdx) ? 50000000ULL : 5000000ULL;
                        glClientWaitSync(gLobbyFence[lobbySubmitIdx], GL_SYNC_FLUSH_COMMANDS_BIT, budget);
                    }
                }
                if (!gAtwEnabled) { Pvr_SetAsyncTimeWarp(1); gAtwEnabled = true; }
                PVR_CameraEndFrame(0, gLobbyEye[0][lobbySubmitIdx]);
                PVR_CameraEndFrame(1, gLobbyEye[1][lobbySubmitIdx]);
                // Submit the RENDER pose (not the live pose) so the warp's
                // reprojection delta is correct for the pipelined/reused slot too.
                Quat sQ = gLobbyPoseQ[lobbySubmitIdx];
                Mat4 rRot = quatToMat4(sQ.x, sQ.y, sQ.z, sQ.w);
                for (int e = 0; e < 2; e++) {
                    float ex = (e == 0 ? -softIpdM()*0.5f : softIpdM()*0.5f);
                    PvrPoseBlk blk; memset(&blk, 0, sizeof(blk));
                    blk.v[0] = sQ.x; blk.v[1] = sQ.y; blk.v[2] = sQ.z; blk.v[3] = sQ.w;
                    blk.v[4] = gLobbyPoseP[lobbySubmitIdx][0] + rRot.m[0]*ex;
                    blk.v[5] = gLobbyPoseP[lobbySubmitIdx][1] + rRot.m[1]*ex;
                    blk.v[6] = gLobbyPoseP[lobbySubmitIdx][2] + rRot.m[2]*ex;
                    PVR_ChangeRenderPose(e, 0, blk);
                }
                PVR_TimeWarpEvent(0);
            }
            // PACING: trailing sleep AFTER the submit, sized to hold a steady ONE-VSYNC
            // cadence. period = work + sleep; we sleep the remainder of one 72Hz interval
            // when there's slack, so a light frame and a heavier frame both submit
            // ~13.9ms apart rather than overshooting to ~77fps when idle and
            // undershooting under load. CRITICAL invariant: period is ALWAYS >= one
            // vsync -- never target above the 72Hz panel and always leave a floor gap --
            // so two submits can never land in one refresh and desync the per-eye DIATW
            // latch (which is what targeting above the panel rate would cause).
            const uint64_t kVsyncNs  = (uint64_t)(1e9 / 72.0);   // 72Hz panel native (do NOT exceed)
            const uint64_t kMinGapNs = 3000000ULL;               // always yield >=3ms so the warp thread runs
            uint64_t work = nowNs() - tIterStart;
            uint64_t sleepNs = (work < kVsyncNs) ? (kVsyncNs - work) : 0;
            if (sleepNs < kMinGapNs) sleepNs = kMinGapNs;        // floor -> period stays >= one vsync
            usleep((useconds_t)(sleepNs / 1000));
            frame++; framesWithSurface++;
            continue;
        }

    }

    LOGI("render loop exit at frame %d", frame);
    // Stop + join the tracking thread before tearing down GL/JVM.
    gTrackRunning.store(false);
    pthread_join(gTrackThread, nullptr);
    // Tear down the ALVR core in stock order so a later nativeStart re-inits a
    // clean client (singleTask relaunch can reuse this process): pause the
    // stream, free the GL-side resources while the context is still current,
    // then destroy the core.
    alvr_pause();
    alvr_destroy_opengl();
    alvr_destroy();
    eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    if (sfc != EGL_NO_SURFACE) eglDestroySurface(dpy, sfc);
    if (pbuf != EGL_NO_SURFACE) eglDestroySurface(dpy, pbuf);
    if (curWin) ANativeWindow_release(curWin);
    if (ctx != EGL_NO_CONTEXT) eglDestroyContext(dpy, ctx);
    eglTerminate(dpy);
    gVM->DetachCurrentThread();
    return nullptr;
}

// Called from Java surface callbacks: hand the render thread a new window (or
// null when the surface is destroyed).
void setWindow(ANativeWindow *win) {
    std::lock_guard<std::mutex> lk(gWinMutex);
    // If a previous window is STILL pending (gWindowDirty -> the render thread hasn't
    // consumed it yet), overwriting gPendingWindow would drop that ANativeWindow ref
    // without releasing it -> one leak per unconsumed surface callback. Release it
    // first. (Only safe while dirty: once consumed, curWin owns that ref, not us.)
    if (gWindowDirty.load() && gPendingWindow && gPendingWindow != win)
        ANativeWindow_release(gPendingWindow);
    gPendingWindow = win;
    gWindowDirty.store(true);
}
