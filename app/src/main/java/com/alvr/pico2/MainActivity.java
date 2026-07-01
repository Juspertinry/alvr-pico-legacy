package com.alvr.pico2;

import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.KeyEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.WindowManager;
import android.widget.FrameLayout;

import com.unity3d.player.UnityPlayer;

import com.picovr.picovrlib.cvcontrollerclient.ControllerClient;
import com.picovr.picovrlib.cvcontrollerclient.BindControllerCallback;

/**
 * Host activity (Unity-emulation approach).
 *
 * The Pico SDK obtains the render Surface from the activity's `mUnityPlayer`
 * field (a ViewGroup whose first child is the SurfaceView). We provide exactly
 * that. Our native render thread renders the eyes into offscreen FBOs; the SDK's
 * own TimeWarp thread owns the window surface (pulled from mUnityPlayer) and
 * presents to the HMD.
 */
public class MainActivity extends Activity implements SurfaceHolder.Callback {

    private static final String TAG = "P2ALVR";

    static {
        System.loadLibrary("tracking_module");
        System.loadLibrary("native");
        System.loadLibrary("Pvr_UnitySDK");
        System.loadLibrary("alvr_client_core");   // ALVR client (Rust); load before p2alvr (depends on it)
        System.loadLibrary("p2alvr");
    }

    // IMPORTANT: the SDK looks up this exact field name/type on the activity.
    // Its surfaceView is what the SDK's TimeWarp thread presents to (we let it
    // black that one out harmlessly, BEHIND our own render surface).
    public UnityPlayer mUnityPlayer;

    // Our own render surface, composited ON TOP of the SDK's. We self-present
    // video here; the SDK warp thread never touches it, so no surface conflict.
    private SurfaceView mRenderView;

    private native void nativeStart(Activity activity);
    private native void nativeSurfaceChanged(android.view.Surface surface);
    private native void nativeSurfaceDestroyed();
    private native void nativeStop();
    private native void nativeSetSleep(boolean sleep);
    private native void nativeKeyEvent(int keyCode, boolean down);
    private native void nativeControllerState(int hand, int conn,
            float[] sensor, float[] angVel, int[] keys);
    // Latest head pose (raw sensor frame: qx,qy,qz,qw,px,py,pz) from native, fed
    // into the CV service so controller poses come back in the head's frame.
    private native void nativeGetHeadData(float[] out);
    private final float[] mHeadData = new float[7];
    // Server-driven haptics: native parks a pending rumble per hand; this poller
    // drains it and drives the Pico CV2 wand. out[0]=amplitude(0..1), out[1]=ms.
    private native boolean nativeDrainHaptic(int hand, float[] out);
    private final float[] mHaptic = new float[2];

    // Pico CV controller service: bound once, then polled on a background thread
    // and pushed to native. headDof/handDof = 1,1 requests 6DoF.
    private volatile boolean mCtrlRunning = false;
    private Thread mCtrlThread;
    // The Pico CV service assigns controller handedness at the moment
    // startControllerThread() runs. On a cold start, calling it in bindSuccess fires
    // before BOTH controllers have connected, so the service binds handedness with
    // only one controller present -> left maps to the right controller, the other
    // hand is dead, and the stick reads uninitialized (diagonal) garbage. Re-opening
    // the app "fixes" it only because both are connected by the next start.
    //
    // Rather than restart the thread to re-bind (fragile: a 60ms stop/start often
    // doesn't take, and driving it off lifecycle hooks loops forever), we DEFER the
    // first and ONLY startControllerThread until both controllers have been reported
    // connected CONTINUOUSLY for a settle period. The service reports "connected"
    // instantly on bind (stale/cached state) -- starting the 6DoF thread that early
    // binds handedness before the controller link has truly re-established (worst on
    // the first launch after an update, when the service was still attached to the
    // just-killed process), which is the swap. Waiting for the link to settle first
    // lets it bind correctly. Tracked in the poll loop; started exactly once.
    private volatile boolean mConn0 = false;   // controller 0 connected
    private volatile boolean mConn1 = false;   // controller 1 connected
    private volatile boolean mCtrlThreadStarted = false;
    private long mBothConnSinceMs = 0;         // when both first became connected (0 = not yet)
    private static final long CTRL_SETTLE_MS = 1500;  // both-connected must hold this long
    // Set when our surface is torn down; consumed when it comes back. A full-screen
    // takeover (e.g. the double-click-Home shortcut panel com.pvr.shortcut, a Unity VR
    // Activity) destroys our surface AND reconfigures the SHARED CV controller service
    // for its own session (it re-sets unity version to 2.9.0.0 + re-anchors the
    // controller link). When it exits our surface is recreated and render recovers,
    // but the CV service is left on the shortcut's head-fusion/recenter reference, so
    // our hands stay locked to a stale head frame (they swing opposite to head
    // rotation, decoupled from world). Home/back fixes it only because it re-runs our
    // bind. We replicate that recovery on the surface destroy->recreate edge -- a clean
    // signal (unlike onResume, which fires on every focus change and loops). Re-claim
    // settle is short: both controllers are already healthy here (the shortcut kept
    // them connected), so this is NOT the confused cold-start state.
    private volatile boolean mSurfaceWasDestroyed = false;
    private static final long CTRL_RECLAIM_SETTLE_MS = 300;

    // True only while the activity is in the foreground (between onResume and
    // onPause). When backgrounded (Home pressed / another app on top) we stop
    // forwarding controller tracking + input to the server and stop driving
    // haptics, so the emulated controllers don't keep moving/buzzing in SteamVR
    // while the user is away. The poll loop keeps running so the CV link stays
    // warm and resumes instantly on return.
    private volatile boolean mForeground = false;
    private volatile long mSettleMs = CTRL_SETTLE_MS;  // active settle window (longer cold, short on re-claim)
    // Reported to the CV service so it flips callunityversion=true and assigns
    // handedness from real per-controller state (see bindSuccess). Matches the
    // version the system Unity/CVControllerManager reports on this firmware.
    private static final String CTRL_UNITY_VERSION = "2.8.6.9";

    // Held for the app's lifetime so the Wi-Fi link stays reliable for streaming:
    //  - MulticastLock: ALVR discovers the PC server via mDNS/broadcast; without
    //    this, Android filters multicast/broadcast packets and discovery can stall.
    //  - WifiLock(HIGH_PERF): keeps the Wi-Fi radio out of power-save so latency
    //    doesn't spike / the link doesn't drop when the screen state changes.
    private WifiManager.MulticastLock mMulticastLock;
    private WifiManager.WifiLock mWifiLock;

    // Proximity power-sleep: when the headset is taken OFF the head (proximity
    // reads "far") for PROX_SLEEP_MS, pause the stream + release the screen-on lock
    // so the panel can sleep -- big power saving while it sits idle. Donning it
    // (proximity "near") cancels the timer / wakes it back up. We MUST drop
    // FLAG_KEEP_SCREEN_ON here: that flag (held for streaming) is exactly what stops
    // the system from auto-sleeping the panel when the headset is doffed.
    private SensorManager mSensorManager;
    private Sensor mProximity;
    private final Handler mMainHandler = new Handler(Looper.getMainLooper());
    private static final long PROX_SLEEP_MS = 60_000L;   // 1 minute off-head -> sleep
    private boolean mAsleep = false;
    private final Runnable mSleepRunnable = new Runnable() {
        @Override public void run() {
            if (mAsleep) return;
            mAsleep = true;
            Log.i(TAG, "proximity: off-head " + (PROX_SLEEP_MS / 1000) + "s -> sleeping stream");
            getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
            try { nativeSetSleep(true); } catch (Throwable t) { /* native not up */ }
        }
    };

    /**
     * Android fallback for the in-app brightness slider, called from native only
     * when the Pico SDK backlight control didn't take. Sets this activity window's
     * brightness (0..1); needs no special permission. Must run on the UI thread.
     */
    public void setWindowBrightness(final float frac) {
        final float f = frac < 0f ? 0f : (frac > 1f ? 1f : frac);
        mMainHandler.post(new Runnable() {
            @Override public void run() {
                try {
                    WindowManager.LayoutParams lp = getWindow().getAttributes();
                    lp.screenBrightness = f;
                    getWindow().setAttributes(lp);
                    Log.i(TAG, "brightness: Android window fallback -> " + f);
                } catch (Throwable t) {
                    Log.w(TAG, "brightness: window fallback failed", t);
                }
            }
        });
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        acquireWifiLocks();

        mUnityPlayer = new UnityPlayer(this);
        setContentView(mUnityPlayer);
        mUnityPlayer.surfaceView.getHolder().addCallback(this);

        // Long-lived render thread; surface comes/goes via the callbacks below
        // (so sleep/wake = surface destroy/recreate is handled gracefully).
        nativeStart(this);

        setupControllers();
        setupProximitySleep();
    }

    @Override
    protected void onResume() {
        super.onResume();
        mForeground = true;   // resume forwarding controller input + haptics
    }

    @Override
    protected void onPause() {
        super.onPause();
        mForeground = false;  // backgrounded -> stop controller tracking + haptics
    }

    // Listen to the headset's proximity sensor for don/doff. "far" starts the
    // off-head sleep timer; "near" cancels it and wakes the stream back up.
    private void setupProximitySleep() {
        try {
            mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
            if (mSensorManager == null) { Log.e(TAG, "no SensorManager"); return; }
            mProximity = mSensorManager.getDefaultSensor(Sensor.TYPE_PROXIMITY);
            if (mProximity == null) { Log.e(TAG, "no proximity sensor"); return; }
            mSensorManager.registerListener(mProximityListener, mProximity,
                    SensorManager.SENSOR_DELAY_NORMAL);
            Log.i(TAG, "proximity sensor registered (maxRange=" + mProximity.getMaximumRange() + ")");
        } catch (Throwable t) {
            Log.e(TAG, "setupProximitySleep failed", t);
        }
    }

    private final SensorEventListener mProximityListener = new SensorEventListener() {
        @Override public void onSensorChanged(SensorEvent event) {
            if (event.values.length == 0) return;
            // Convention: distance < maxRange => something is near (headset on head).
            boolean near = event.values[0] < mProximity.getMaximumRange();
            if (near) {
                // Donned: cancel any pending sleep and wake if we slept.
                mMainHandler.removeCallbacks(mSleepRunnable);
                if (mAsleep) {
                    mAsleep = false;
                    Log.i(TAG, "proximity: donned -> waking stream");
                    getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
                    try { nativeSetSleep(false); } catch (Throwable t) { /* native not up */ }
                }
            } else {
                // Doffed: arm the sleep timer (no-op if already armed/asleep).
                if (!mAsleep) {
                    mMainHandler.removeCallbacks(mSleepRunnable);
                    mMainHandler.postDelayed(mSleepRunnable, PROX_SLEEP_MS);
                }
            }
        }
        @Override public void onAccuracyChanged(Sensor sensor, int accuracy) {}
    };

    // Bind the Pico CV controller service. On success, start the SDK controller
    // thread (6DoF) and our poll loop. Poll state is pushed to native each tick.
    private void setupControllers() {
        try {
            ControllerClient.registerBindCallback(new BindControllerCallback() {
                @Override public void bindSuccess() {
                    // CRITICAL handedness fix: tell the CV service our "unity version".
                    // The service's getPvrMainController() branches on callunityversion:
                    // when it's false (we never set it) AND not-all-controllers-connected,
                    // it FORCES the main controller to index 1 and collapses both virtual
                    // hands onto one controller -- this is the cold-start handedness swap.
                    // Setting a version flips callunityversion=true so the service uses the
                    // real per-controller state to assign handedness. (This is exactly what
                    // the system's CVControllerManager does on a home/back, which is why
                    // that "fixes" it.) Must happen before startControllerThread.
                    try { ControllerClient.setUnityVersion(CTRL_UNITY_VERSION); } catch (Throwable t) {
                        Log.e(TAG, "setUnityVersion failed", t);
                    }
                    // Do NOT start the 6DoF thread here -- wait until both controllers
                    // are connected so handedness binds correctly (see field comment).
                    Log.i(TAG, "controller service bound (unity version set); deferring 6DoF start until both connected");
                    startControllerPoll();
                }
                @Override public void unbindSuccess() {
                    Log.i(TAG, "controller service unbound");
                    stopControllerPoll();
                }
                @Override public void controllerConnectStateChanged(int hand, int state) {
                    Log.i(TAG, "controller " + hand + " connect state -> " + state);
                    // Just record state; the poll loop enforces the settle window
                    // before the one-shot start (see maybeStartControllerThread).
                    if (hand == 0) mConn0 = (state == 1);
                    else if (hand == 1) mConn1 = (state == 1);
                }
            });
            ControllerClient.bindControllerService(this);
            Log.i(TAG, "controller service bind requested");
        } catch (Throwable t) {
            Log.e(TAG, "setupControllers failed", t);
        }
    }

    // Start the 6DoF controller thread exactly once, only after BOTH controllers have
    // been continuously connected for CTRL_SETTLE_MS. The service reports "connected"
    // instantly on bind (cached), but the link isn't truly re-established yet; binding
    // handedness that early causes the swap. Waiting for it to hold steady lets the
    // service settle so the bind is correct. Idempotent; safe to call repeatedly.
    private synchronized void maybeStartControllerThread() {
        if (mCtrlThreadStarted) return;
        if (!(mConn0 && mConn1)) {
            mBothConnSinceMs = 0;   // link dropped -> restart the settle window
            return;
        }
        long now = android.os.SystemClock.uptimeMillis();
        if (mBothConnSinceMs == 0) {
            mBothConnSinceMs = now;
            Log.i(TAG, "both controllers connected -> settling " + mSettleMs + "ms before 6DoF start");
            return;
        }
        if (now - mBothConnSinceMs < mSettleMs) return;
        try {
            // Re-assert the unity version right before the handedness-binding start,
            // so callunityversion is definitely true at the moment the service
            // assigns the main controller (see bindSuccess for why).
            try { ControllerClient.setUnityVersion(CTRL_UNITY_VERSION); } catch (Throwable t) { /* logged at bind */ }
            ControllerClient.startControllerThread(1, 1);
            mCtrlThreadStarted = true;
            Log.i(TAG, "controller link settled -> startControllerThread (handedness bound)");
        } catch (Throwable t) {
            Log.e(TAG, "startControllerThread failed", t);
        }
    }

    // Reclaim the SHARED CV service after a full-screen takeover stole it (see
    // mSurfaceWasDestroyed). Replicates the proven Home/back recovery without a full
    // rebind: re-assert OUR unity version (overriding the shortcut's 2.9.0.0) and let
    // the (short) settle path re-issue startControllerThread, which re-anchors the
    // controller link to our session's head frame. No-op if controllers never started.
    private synchronized void reclaimControllerLink() {
        if (!mCtrlRunning) return;   // service not bound/polling yet -> nothing to reclaim
        try { ControllerClient.setUnityVersion(CTRL_UNITY_VERSION); } catch (Throwable t) {
            Log.e(TAG, "reclaim setUnityVersion failed", t);
        }
        mSettleMs = CTRL_RECLAIM_SETTLE_MS;   // both hands already healthy -> short window
        mBothConnSinceMs = 0;                 // restart the settle clock
        mCtrlThreadStarted = false;           // allow the poll loop to re-issue the start
        Log.i(TAG, "surface returned after takeover -> reclaiming CV controller link");
    }

    private void startControllerPoll() {
        if (mCtrlRunning) return;
        mCtrlRunning = true;
        mCtrlThread = new Thread(() -> {
            while (mCtrlRunning) {
                // Fallback for the deferred first start: if the connect-state callback
                // didn't fire (or fired before the poll thread existed), pick up the
                // live connection state here and start once both are connected.
                if (!mCtrlThreadStarted) {
                    try {
                        mConn0 = (ControllerClient.getControllerConnectionState(0) == 1);
                        mConn1 = (ControllerClient.getControllerConnectionState(1) == 1);
                        maybeStartControllerThread();
                    } catch (Throwable t) { /* service not ready; retry next tick */ }
                }
                for (int h = 0; h < 2; h++) {
                    try {
                        int conn = ControllerClient.getControllerConnectionState(h);
                        // Head-aligned pose: feed the live head pose so the CV service
                        // returns the controller in the HEAD's tracking frame (this is
                        // what the legacy SDK does when enablehand6dofbyhead==1). Keeps
                        // controllers locked to the head instead of a separate universe.
                        nativeGetHeadData(mHeadData);
                        float[] sensor = ControllerClient.getControllerSensorState(h, mHeadData);
                        float[] angVel = ControllerClient.getControllerAngularVelocity(h);
                        // Button/joystick state lives in getControllerKeyEventUnityExt
                        // (a ~67-int array). Reverse-engineered held-state indices on
                        // this CV2 (FalconCV2) controller (see EXT_* below). We pack the
                        // values we need into a compact, fixed-layout array for native:
                        //   keys[0]=joyX  keys[1]=joyY (analog, center 128)
                        //   keys[2]=trigger  keys[3]=grip  keys[4]=joyClick  keys[5]=menu
                        int[] ext = ControllerClient.getControllerKeyEventUnityExt(h);
                        int[] legacy = ControllerClient.getControllerKeyEvent(h);
                        int gripAnalog = 0;
                        try { gripAnalog = ControllerClient.getControllerGripValue(h); } catch (Throwable t) {}
                        int[] keys = new int[11];
                        keys[0] = pick(ext, EXT_JOY_X);      // joystick X, center 128
                        keys[1] = pick(ext, EXT_JOY_Y);      // joystick Y, center 128
                        keys[2] = pick(ext, EXT_TRIGGER);    // trigger digital (0/1)
                        keys[3] = pick(ext, h == 1 ? EXT_GRIP_RIGHT : EXT_GRIP_LEFT); // grip (0/1), per-hand index
                        keys[4] = pick(ext, EXT_JOY_CLICK);  // joystick click (0/1)
                        keys[5] = pick(ext, EXT_MENU);       // app/menu (0/1)
                        keys[6] = pick(ext, EXT_AX);         // A (right) / X (left) (0/1)
                        keys[7] = pick(ext, EXT_BY);         // B (right) / Y (left) (0/1)
                        keys[8] = pick(legacy, LEGACY_TRIG); // trigger analog 0..255
                        keys[9] = gripAnalog;                // grip analog 0..255
                        keys[10] = pick(ext, EXT_BATTERY);   // controller battery (0..100, diag HUD)
                        // Backgrounded (Home / app not in foreground): report the
                        // controller as disconnected so the native uplink drops it
                        // (the render thread skips conn != 1) -- no controller tracking
                        // or button input reaches the server while the user is away.
                        int sendConn = mForeground ? conn : 0;
                        nativeControllerState(h, sendConn, sensor, angVel, keys);
                        // Always drain the parked haptic so a stale pulse can't buzz on
                        // resume, but only actually rumble the CV2 wand while foreground
                        // and connected. strength 0..1, time ms, hand.
                        boolean haveHaptic = nativeDrainHaptic(h, mHaptic);
                        if (mForeground && conn == 1 && haveHaptic) {
                            try {
                                ControllerClient.vibrateCV2ControllerStrength(
                                        mHaptic[0], (int) mHaptic[1], h);
                            } catch (Throwable t) { /* old service: no CV2 vibrate */ }
                        }
                    } catch (Throwable t) {
                        // service not ready yet / transient; keep polling
                    }
                }
                try { Thread.sleep(11); } catch (InterruptedException e) { break; }
            }
        }, "ctrl-poll");
        mCtrlThread.start();
        Log.i(TAG, "controller poll thread started");
    }

    // getControllerKeyEventUnityExt index layout, from the Pico legacy SDK source
    // (Pvr_ControllerManager.TransformData, per-hand offset 0):
    //   0=joyX 5=joyY 10=Home 15=App(menu) 20=touch 35=Trigger 40=Battery
    //   45=A/X 50=B/Y 55=Right-grip(R hand) 60=Left-grip(L hand) 66=touchpad Click
    private static final int EXT_JOY_X      = 0;
    private static final int EXT_JOY_Y      = 5;
    private static final int EXT_MENU       = 15;  // App button
    private static final int EXT_TRIGGER    = 35;
    private static final int EXT_BATTERY    = 40;  // controller battery level (0..100)
    private static final int EXT_AX         = 45;  // A (right) / X (left)
    private static final int EXT_BY         = 50;  // B (right) / Y (left)
    private static final int EXT_GRIP_RIGHT = 55;  // grip on the right controller
    private static final int EXT_GRIP_LEFT  = 60;  // grip on the left controller
    // Thumbstick CLICK. The SDK exposes TWO clicks depending on mode: in the lobby's
    // touchpad/pointer mode it's TouchPadClick=ext[66]; in 6DoF/joystick mode (i.e.
    // while STREAMING -- the case that matters) ext[66] is dead and the stick press
    // is the "Touch" key State at ext[20] (held while pressed). Use 20 so click works
    // in-stream. (Verified on-device: streaming clicks toggle ext[20], never ext[66].)
    private static final int EXT_JOY_CLICK  = 20;
    private static final int LEGACY_TRIG    = 7;   // analog trigger 0..255 in getControllerKeyEvent
    private static int pick(int[] a, int i) { return (a != null && i < a.length) ? a[i] : 0; }

    private void stopControllerPoll() {
        mCtrlRunning = false;
        if (mCtrlThread != null) { mCtrlThread.interrupt(); mCtrlThread = null; }
    }

    private void acquireWifiLocks() {
        try {
            WifiManager wifi = (WifiManager)
                    getApplicationContext().getSystemService(Context.WIFI_SERVICE);
            if (wifi == null) { Log.e(TAG, "no WifiManager"); return; }
            mMulticastLock = wifi.createMulticastLock("alvr-mdns");
            mMulticastLock.setReferenceCounted(false);
            mMulticastLock.acquire();
            mWifiLock = wifi.createWifiLock(
                    WifiManager.WIFI_MODE_FULL_HIGH_PERF, "alvr-stream");
            mWifiLock.setReferenceCounted(false);
            mWifiLock.acquire();
            Log.i(TAG, "wifi locks acquired: multicast=" + mMulticastLock.isHeld()
                    + " wifi=" + mWifiLock.isHeld());
        } catch (Throwable t) {
            Log.e(TAG, "acquireWifiLocks failed", t);
        }
    }

    private void releaseWifiLocks() {
        try {
            if (mMulticastLock != null && mMulticastLock.isHeld()) mMulticastLock.release();
            if (mWifiLock != null && mWifiLock.isHeld()) mWifiLock.release();
        } catch (Throwable t) { Log.e(TAG, "releaseWifiLocks failed", t); }
    }

    // Forward headset/controller key presses to native. The side OK button drives
    // the lobby Software-IPD slider. We forward all keycodes (native logs them so
    // we can identify the OK button), and consume the OK candidates so the system
    // doesn't also act on them; everything else passes through normally.
    // The Pico side OK button is custom keycode 1001 (no KeyEvent constant).
    private static final int KEYCODE_PICO_OK = 1001;
    private static boolean isOkKey(int keyCode) {
        return keyCode == KEYCODE_PICO_OK
                || keyCode == KeyEvent.KEYCODE_ENTER
                || keyCode == KeyEvent.KEYCODE_DPAD_CENTER
                || keyCode == KeyEvent.KEYCODE_BUTTON_A;
    }

    @Override
    public boolean onKeyDown(int keyCode, KeyEvent event) {
        nativeKeyEvent(keyCode, true);
        if (isOkKey(keyCode)) return true;
        return super.onKeyDown(keyCode, event);
    }

    @Override
    public boolean onKeyUp(int keyCode, KeyEvent event) {
        nativeKeyEvent(keyCode, false);
        if (isOkKey(keyCode)) return true;
        return super.onKeyUp(keyCode, event);
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {
        Log.i(TAG, "surfaceCreated");
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
        Log.i(TAG, "surfaceChanged " + width + "x" + height);
        if (width > 0 && height > 0) {
            nativeSurfaceChanged(holder.getSurface());
            // If we're coming back from a full-screen takeover (surface was destroyed),
            // the shared CV service was reconfigured by that app -> reclaim it.
            if (mSurfaceWasDestroyed) {
                mSurfaceWasDestroyed = false;
                reclaimControllerLink();
            }
        }
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        Log.i(TAG, "surfaceDestroyed");
        mSurfaceWasDestroyed = true;   // arm CV-service reclaim for when the surface returns
        nativeSurfaceDestroyed();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        mMainHandler.removeCallbacks(mSleepRunnable);
        if (mSensorManager != null) {
            try { mSensorManager.unregisterListener(mProximityListener); } catch (Throwable t) { /* ignore */ }
        }
        stopControllerPoll();
        try { ControllerClient.unbindControllerService(this); } catch (Throwable t) { /* ignore */ }
        nativeStop();
        releaseWifiLocks();
    }
}
