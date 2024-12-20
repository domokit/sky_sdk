// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.platform;

import static io.flutter.Build.API_LEVELS;

import android.annotation.SuppressLint;
import android.annotation.TargetApi;
import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.PorterDuff;
import android.graphics.Rect;
import android.view.MotionEvent;
import android.view.Surface;
import android.view.View;
import android.view.ViewParent;
import android.view.ViewTreeObserver;
import android.view.accessibility.AccessibilityEvent;
import android.widget.FrameLayout;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.VisibleForTesting;
import io.flutter.Log;
import io.flutter.embedding.android.AndroidTouchProcessor;
import io.flutter.embedding.engine.renderer.FlutterRenderer;
import io.flutter.util.ViewUtils;

/**
 * Wraps a platform view to intercept gestures and project this view onto a {@link
 * PlatformViewRenderTarget}.
 *
 * <p>An Android platform view is composed by the engine using a {@code TextureLayer}. The view is
 * embeded to the Android view hierarchy like a normal view, but it's projected onto a {@link
 * PlatformViewRenderTarget}, so it can be efficiently composed by the engine.
 *
 * <p>Since the view is in the Android view hierarchy, keyboard and accessibility interactions
 * behave normally.
 */
@TargetApi(API_LEVELS.API_23)
public class PlatformViewWrapper extends FrameLayout {
  private static final String TAG = "PlatformViewWrapper";

  private int prevLeft;
  private int prevTop;
  private int left;
  private int top;
  private AndroidTouchProcessor touchProcessor;
  private PlatformViewRenderTarget renderTarget;

  private ViewTreeObserver.OnGlobalFocusChangeListener activeFocusListener;

  public PlatformViewWrapper(@NonNull Context context) {
    super(context);
    setWillNotDraw(false);
  }

  public PlatformViewWrapper(
      @NonNull Context context, @NonNull PlatformViewRenderTarget renderTarget) {
    this(context);
    this.renderTarget = renderTarget;

    Surface surface = renderTarget.getSurface();
    if (surface != null && !FlutterRenderer.debugDisableSurfaceClear) {
      final Canvas canvas = surface.lockHardwareCanvas();
      try {
        canvas.drawColor(Color.TRANSPARENT, PorterDuff.Mode.CLEAR);
      } finally {
        surface.unlockCanvasAndPost(canvas);
      }
    }
  }

  /**
   * Sets the touch processor that allows to intercept gestures.
   *
   * @param newTouchProcessor The touch processor.
   */
  public void setTouchProcessor(@Nullable AndroidTouchProcessor newTouchProcessor) {
    touchProcessor = newTouchProcessor;
  }

  /**
   * Sets the layout parameters for this view.
   *
   * @param params The new parameters.
   */
  public void setLayoutParams(@NonNull FrameLayout.LayoutParams params) {
    super.setLayoutParams(params);

    left = params.leftMargin;
    top = params.topMargin;
  }

  public void resizeRenderTarget(int width, int height) {
    if (renderTarget != null) {
      renderTarget.resize(width, height);
    }
  }

  public int getRenderTargetWidth() {
    if (renderTarget != null) {
      return renderTarget.getWidth();
    }
    return 0;
  }

  public int getRenderTargetHeight() {
    if (renderTarget != null) {
      return renderTarget.getHeight();
    }
    return 0;
  }

  /** Releases resources. */
  public void release() {
    if (renderTarget != null) {
      renderTarget.release();
      renderTarget = null;
    }
  }

  @Override
  public boolean onInterceptTouchEvent(@NonNull MotionEvent event) {
    return true;
  }

  @Override
  public boolean requestSendAccessibilityEvent(View child, AccessibilityEvent event) {
    final View embeddedView = getChildAt(0);
    if (embeddedView != null
        && embeddedView.getImportantForAccessibility()
            == View.IMPORTANT_FOR_ACCESSIBILITY_NO_HIDE_DESCENDANTS) {
      return false;
    }
    // Forward the request only if the embedded view is in the Flutter accessibility tree.
    // The embedded view may be ignored when the framework doesn't populate a SemanticNode
    // for the current platform view.
    // See AccessibilityBridge for more.
    return super.requestSendAccessibilityEvent(child, event);
  }

  /** Used on Android O+, {@link invalidateChildInParent} used for previous versions. */
  @SuppressLint("NewApi")
  @Override
  public void onDescendantInvalidated(@NonNull View child, @NonNull View target) {
    super.onDescendantInvalidated(child, target);
    invalidate();
  }

  @Override
  public ViewParent invalidateChildInParent(int[] location, Rect dirty) {
    invalidate();
    return super.invalidateChildInParent(location, dirty);
  }

  @Override
  @SuppressLint("NewApi")
  public void draw(Canvas canvas) {
    if (renderTarget == null) {
      super.draw(canvas);
      Log.e(TAG, "Platform view cannot be composed without a RenderTarget.");
      return;
    }

    final Surface targetSurface = renderTarget.getSurface();
    if (!targetSurface.isValid()) {
      Log.e(TAG, "Platform view cannot be composed without a valid RenderTarget surface.");
      return;
    }

    final Canvas targetCanvas = targetSurface.lockHardwareCanvas();
    if (targetCanvas == null) {
      // Cannot render right now.
      invalidate();
      return;
    }

    try {
      // Fill the render target with transparent pixels. This is needed for platform views that
      // expect a transparent background.
      targetCanvas.drawColor(Color.TRANSPARENT, PorterDuff.Mode.CLEAR);
      // Override the canvas that this subtree of views will use to draw.
      super.draw(targetCanvas);
    } finally {
      renderTarget.scheduleFrame();
      targetSurface.unlockCanvasAndPost(targetCanvas);
    }
  }

  @Override
  @SuppressLint("ClickableViewAccessibility")
  public boolean onTouchEvent(@NonNull MotionEvent event) {
    if (touchProcessor == null) {
      return super.onTouchEvent(event);
    }
    final Matrix screenMatrix = new Matrix();
    switch (event.getAction()) {
      case MotionEvent.ACTION_DOWN:
        prevLeft = left;
        prevTop = top;
        screenMatrix.postTranslate(left, top);
        break;
      case MotionEvent.ACTION_MOVE:
        // While the view is dragged, use the left and top positions as
        // they were at the moment the touch event fired.
        screenMatrix.postTranslate(prevLeft, prevTop);
        prevLeft = left;
        prevTop = top;
        break;
      case MotionEvent.ACTION_UP:
      default:
        screenMatrix.postTranslate(left, top);
        break;
    }
    return touchProcessor.onTouchEvent(event, screenMatrix);
  }

  @VisibleForTesting
  public ViewTreeObserver.OnGlobalFocusChangeListener getActiveFocusListener() {
    return this.activeFocusListener;
  }

  public void setOnDescendantFocusChangeListener(@NonNull OnFocusChangeListener userFocusListener) {
    unsetOnDescendantFocusChangeListener();
    final ViewTreeObserver observer = getViewTreeObserver();
    if (observer.isAlive() && activeFocusListener == null) {
      activeFocusListener =
          new ViewTreeObserver.OnGlobalFocusChangeListener() {
            @Override
            public void onGlobalFocusChanged(View oldFocus, View newFocus) {
              userFocusListener.onFocusChange(
                  PlatformViewWrapper.this, ViewUtils.childHasFocus(PlatformViewWrapper.this));
            }
          };
      observer.addOnGlobalFocusChangeListener(activeFocusListener);
    }
  }

  public void unsetOnDescendantFocusChangeListener() {
    final ViewTreeObserver observer = getViewTreeObserver();
    if (observer.isAlive() && activeFocusListener != null) {
      final ViewTreeObserver.OnGlobalFocusChangeListener currFocusListener = activeFocusListener;
      activeFocusListener = null;
      observer.removeOnGlobalFocusChangeListener(currFocusListener);
    }
  }
}
