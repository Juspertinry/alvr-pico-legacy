#pragma once
// Public interface of the render thread: lifetime/JNI globals, the window
// hand-off, and the proximity-sleep flag. Shared between render_thread.cpp
// (owner / definitions) and jni.cpp (the JNI entry points that drive it).
#include <jni.h>
#include <pthread.h>
#include <mutex>
#include <atomic>
#include <android/native_window.h>

// JNI + thread lifetime (set in nativeStart, torn down in nativeStop).
extern JavaVM   *gVM;
extern jobject   gActivity;
extern jclass    gVrClass;
extern pthread_t gThread;
extern std::atomic<bool> gRunning;   // render-thread lifetime

// Window handed in by the SurfaceView callbacks; the render thread owns one
// long-lived EGL display+context and (re)creates only the window surface.
extern std::mutex     gWinMutex;
extern ANativeWindow *gPendingWindow;   // latest window (or null), guarded by gWinMutex
// Atomic: the render loop reads this on its fast path WITHOUT taking gWinMutex
// (it only locks once a change is actually pending), while the Java JNI thread
// sets it in setWindow under the lock. The unlocked read was a data race on a
// plain bool; atomic makes it defined. gPendingWindow itself stays mutex-guarded.
extern std::atomic<bool> gWindowDirty;  // a change is pending

// Proximity power-sleep: set by the Java proximity listener (off-head timeout =
// true, donned = false); the render loop acts on the edge (pause/resume ALVR).
extern std::atomic<bool> gSleepReq;

// Hand the render thread a new window (or null when the surface is destroyed).
void  setWindow(ANativeWindow *win);
// The long-lived render-thread entry point (started via pthread_create).
void *renderThread(void *);
