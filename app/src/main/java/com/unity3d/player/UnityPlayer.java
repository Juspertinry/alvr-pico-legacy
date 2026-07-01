package com.unity3d.player;

import android.content.Context;
import android.view.SurfaceView;
import android.view.View;
import android.widget.FrameLayout;

/**
 * Minimal stand-in for Unity's UnityPlayer. The Pico SDK reaches into the
 * activity's `mUnityPlayer` field and pulls the render Surface out of it via
 * getChildAt(0).getHolder().getSurface() -- i.e. it expects a ViewGroup whose
 * first child is the SurfaceView. We reproduce exactly that shape so the SDK's
 * TimeWarp/compositor and Guardian can obtain our Surface.
 */
public class UnityPlayer extends FrameLayout {

    public final SurfaceView surfaceView;

    public UnityPlayer(Context context) {
        super(context);
        surfaceView = new SurfaceView(context);
        addView(surfaceView, new FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.MATCH_PARENT,
                FrameLayout.LayoutParams.MATCH_PARENT));
    }

    // Some SDK paths ask for the player view directly.
    public View getView() {
        return surfaceView;
    }
}
