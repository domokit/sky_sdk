// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.android;

import android.content.Context;
import android.graphics.PixelFormat;
import android.graphics.Region;
import android.util.AttributeSet;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.VisibleForTesting;
import io.flutter.Log;
import io.flutter.embedding.engine.renderer.FlutterRenderer;
import io.flutter.embedding.engine.renderer.FlutterUiDisplayListener;
import io.flutter.embedding.engine.renderer.RenderSurface;

/**
 * Paints a Flutter UI on a {@link android.view.Surface}.
 *
 * <p>To begin rendering a Flutter UI, the owner of this {@code FlutterSurfaceView} must invoke
 * {@link #attachToRenderer(FlutterRenderer)} with the desired {@link FlutterRenderer}.
 *
 * <p>To stop rendering a Flutter UI, the owner of this {@code FlutterSurfaceView} must invoke
 * {@link #detachFromRenderer()}.
 *
 * <p>A {@code FlutterSurfaceView} is intended for situations where a developer needs to render a
 * Flutter UI, but does not require any keyboard input, gesture input, accessibility integrations or
 * any other interactivity beyond rendering. If standard interactivity is desired, consider using a
 * {@link FlutterView} which provides all of these behaviors and utilizes a {@code
 * FlutterSurfaceView} internally.
 */
public class FlutterSurfaceView extends SurfaceView implements RenderSurface {
  private static final String TAG = "FlutterSurfaceView";

  private final boolean renderTransparently;
  private boolean isSurfaceAvailableForRendering = false;
  private boolean isPaused = false;
  @Nullable private FlutterRenderer flutterRenderer;

  private boolean shouldNotify() {
    return flutterRenderer != null && !isPaused;
  }

  // Connects the {@code Surface} beneath this {@code SurfaceView} with Flutter's native code.
  // Callbacks are received by this Object and then those messages are forwarded to our
  // FlutterRenderer, and then on to the JNI bridge over to native Flutter code.
  private final SurfaceHolder.Callback surfaceCallback =
      new SurfaceHolder.Callback() {
        @Override
        public void surfaceCreated(@NonNull SurfaceHolder holder) {
          Log.v(TAG, "SurfaceHolder.Callback.startRenderingToSurface()");
          isSurfaceAvailableForRendering = true;

          if (shouldNotify()) {
            connectSurfaceToRenderer();
          }
        }

        @Override
        public void surfaceChanged(
            @NonNull SurfaceHolder holder, int format, int width, int height) {
          Log.v(TAG, "SurfaceHolder.Callback.surfaceChanged()");
          if (shouldNotify()) {
            changeSurfaceSize(width, height);
          }
        }

        @Override
        public void surfaceDestroyed(@NonNull SurfaceHolder holder) {
          Log.v(TAG, "SurfaceHolder.Callback.stopRenderingToSurface()");
          isSurfaceAvailableForRendering = false;

          if (shouldNotify()) {
            disconnectSurfaceFromRenderer();
          }
        }
      };

  private final FlutterUiDisplayListener flutterUiDisplayListener =
      new FlutterUiDisplayListener() {
        @Override
        public void onFlutterUiDisplayed() {
          Log.v(TAG, "onFlutterUiDisplayed()");
          // Now that a frame is ready to display, take this SurfaceView from transparent to opaque.
          setAlpha(1.0f);

          if (flutterRenderer != null) {
            flutterRenderer.removeIsDisplayingFlutterUiListener(this);
          }
        }

        @Override
        public void onFlutterUiNoLongerDisplayed() {
          // no-op
        }
      };

  /** Constructs a {@code FlutterSurfaceView} programmatically, without any XML attributes. */
  public FlutterSurfaceView(@NonNull Context context) {
    this(context, null, false);
  }

  /**
   * Constructs a {@code FlutterSurfaceView} programmatically, without any XML attributes, and with
   * control over whether or not this {@code FlutterSurfaceView} renders with transparency.
   */
  public FlutterSurfaceView(@NonNull Context context, boolean renderTransparently) {
    this(context, null, renderTransparently);
  }

  /** Constructs a {@code FlutterSurfaceView} in an XML-inflation-compliant manner. */
  public FlutterSurfaceView(@NonNull Context context, @NonNull AttributeSet attrs) {
    this(context, attrs, false);
  }

  private FlutterSurfaceView(
      @NonNull Context context, @Nullable AttributeSet attrs, boolean renderTransparently) {
    super(context, attrs);
    this.renderTransparently = renderTransparently;
    init();
  }

  private void init() {
    // If transparency is desired then we'll enable a transparent pixel format and place
    // our Window above everything else to get transparent background rendering.
    if (renderTransparently) {
      getHolder().setFormat(PixelFormat.TRANSPARENT);
      setZOrderOnTop(true);
    }

    // Grab a reference to our underlying Surface and register callbacks with that Surface so we
    // can monitor changes and forward those changes on to native Flutter code.
    getHolder().addCallback(surfaceCallback);

    // Keep this SurfaceView transparent until Flutter has a frame ready to render. This avoids
    // displaying a black rectangle in our place.
    setAlpha(0.0f);
  }

  // This is a work around for TalkBack.
  // If Android decides that our layer is transparent because, e.g. the status-
  // bar is transparent, TalkBack highlighting stops working.
  // Explicitly telling Android this part of the region is not actually
  // transparent makes TalkBack work again.
  // See https://github.com/flutter/flutter/issues/73413 for context.
  @Override
  public boolean gatherTransparentRegion(Region region) {
    if (getAlpha() < 1.0f) {
      return false;
    }
    final int[] location = new int[2];
    getLocationInWindow(location);
    region.op(
        location[0],
        location[1],
        location[0] + getRight() - getLeft(),
        location[1] + getBottom() - getTop(),
        Region.Op.DIFFERENCE);
    return true;
  }

  @Nullable
  @Override
  public FlutterRenderer getAttachedRenderer() {
    return flutterRenderer;
  }

  @VisibleForTesting
  /* package */ boolean isSurfaceAvailableForRendering() {
    return isSurfaceAvailableForRendering;
  }
  /**
   * Invoked by the owner of this {@code FlutterSurfaceView} when it wants to begin rendering a
   * Flutter UI to this {@code FlutterSurfaceView}.
   *
   * <p>If an Android {@link android.view.Surface} is available, this method will give that {@link
   * android.view.Surface} to the given {@link FlutterRenderer} to begin rendering Flutter's UI to
   * this {@code FlutterSurfaceView}.
   *
   * <p>If no Android {@link android.view.Surface} is available yet, this {@code FlutterSurfaceView}
   * will wait until a {@link android.view.Surface} becomes available and then give that {@link
   * android.view.Surface} to the given {@link FlutterRenderer} to begin rendering Flutter's UI to
   * this {@code FlutterSurfaceView}.
   */
  public void attachToRenderer(@NonNull FlutterRenderer flutterRenderer) {
    Log.v(TAG, "Attaching to FlutterRenderer.");
    if (this.flutterRenderer != null) {
      Log.v(
          TAG,
          "Already connected to a FlutterRenderer. Detaching from old one and attaching to new"
              + " one.");
      this.flutterRenderer.stopRenderingToSurface();
      this.flutterRenderer.removeIsDisplayingFlutterUiListener(flutterUiDisplayListener);
    }

    this.flutterRenderer = flutterRenderer;

    resume();
  }

  /**
   * Invoked by the owner of this {@code FlutterSurfaceView} when it no longer wants to render a
   * Flutter UI to this {@code FlutterSurfaceView}.
   *
   * <p>This method will cease any on-going rendering from Flutter to this {@code
   * FlutterSurfaceView}.
   */
  public void detachFromRenderer() {
    if (flutterRenderer != null) {
      // If we're attached to an Android window then we were rendering a Flutter UI. Now that
      // this FlutterSurfaceView is detached from the FlutterRenderer, we need to stop rendering.
      // TODO(mattcarroll): introduce a isRendererConnectedToSurface() to wrap "getWindowToken() !=
      // null"
      if (getWindowToken() != null) {
        Log.v(TAG, "Disconnecting FlutterRenderer from Android surface.");
        disconnectSurfaceFromRenderer();
      }

      // Make the SurfaceView invisible to avoid showing a black rectangle.
      setAlpha(0.0f);
      flutterRenderer.removeIsDisplayingFlutterUiListener(flutterUiDisplayListener);
      flutterRenderer = null;

    } else {
      Log.w(TAG, "detachFromRenderer() invoked when no FlutterRenderer was attached.");
    }
  }

  /**
   * Invoked by the owner of this {@code FlutterSurfaceView} when it should pause rendering Flutter
   * UI to this {@code FlutterSurfaceView}.
   */
  public void pause() {
    if (flutterRenderer == null) {
      Log.w(TAG, "pause() invoked when no FlutterRenderer was attached.");
      return;
    }
    isPaused = true;
  }

  public void resume() {
    if (flutterRenderer == null) {
      Log.w(TAG, "resume() invoked when no FlutterRenderer was attached.");
      return;
    }
    this.flutterRenderer.addIsDisplayingFlutterUiListener(flutterUiDisplayListener);

    // If we're already attached to an Android window then we're now attached to both a renderer
    // and the Android window. We can begin rendering now.
    if (isSurfaceAvailableForRendering()) {
      Log.v(
          TAG,
          "Surface is available for rendering. Connecting FlutterRenderer to Android surface.");
      connectSurfaceToRenderer();
    }
    isPaused = false;
  }

  // FlutterRenderer and getSurfaceTexture() must both be non-null.
  private void connectSurfaceToRenderer() {
    if (flutterRenderer == null || getHolder() == null) {
      throw new IllegalStateException(
          "connectSurfaceToRenderer() should only be called when flutterRenderer and getHolder()"
              + " are non-null.");
    }
    // When connecting the surface to the renderer, it's possible that the surface is currently
    // paused. For instance, when a platform view is displayed, the current FlutterSurfaceView
    // is paused, and rendering continues in a FlutterImageView buffer while the platform view
    // is displayed.
    //
    // startRenderingToSurface stops rendering to an active surface if it isn't paused.
    flutterRenderer.startRenderingToSurface(getHolder().getSurface(), isPaused);
  }

  // FlutterRenderer must be non-null.
  private void changeSurfaceSize(int width, int height) {
    if (flutterRenderer == null) {
      throw new IllegalStateException(
          "changeSurfaceSize() should only be called when flutterRenderer is non-null.");
    }

    Log.v(
        TAG,
        "Notifying FlutterRenderer that Android surface size has changed to "
            + width
            + " x "
            + height);
    flutterRenderer.surfaceChanged(width, height);
  }

  // FlutterRenderer must be non-null.
  private void disconnectSurfaceFromRenderer() {
    if (flutterRenderer == null) {
      throw new IllegalStateException(
          "disconnectSurfaceFromRenderer() should only be called when flutterRenderer is"
              + " non-null.");
    }

    flutterRenderer.stopRenderingToSurface();
  }
}