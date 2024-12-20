// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.platform;

import static android.view.View.OnFocusChangeListener;
import static io.flutter.Build.API_LEVELS;

import android.annotation.TargetApi;
import android.content.Context;
import android.hardware.display.DisplayManager;
import android.hardware.display.VirtualDisplay;
import android.os.Build;
import android.util.DisplayMetrics;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnFocusChangeListener;
import android.view.ViewTreeObserver;
import androidx.annotation.NonNull;
import androidx.annotation.VisibleForTesting;

class VirtualDisplayController {
  private static String TAG = "VirtualDisplayController";

  private static VirtualDisplay.Callback callback =
      new VirtualDisplay.Callback() {
        @Override
        public void onPaused() {}

        @Override
        public void onResumed() {}
      };

  public static VirtualDisplayController create(
      Context context,
      AccessibilityEventsDelegate accessibilityEventsDelegate,
      PlatformView view,
      PlatformViewRenderTarget renderTarget,
      int width,
      int height,
      int viewId,
      Object createParams,
      OnFocusChangeListener focusChangeListener) {
    if (width == 0 || height == 0) {
      return null;
    }

    DisplayManager displayManager =
        (DisplayManager) context.getSystemService(Context.DISPLAY_SERVICE);
    final DisplayMetrics metrics = context.getResources().getDisplayMetrics();

    // Virtual Display crashes for some PlatformViews if the width or height is bigger
    // than the physical screen size. We have tried to clamp or scale down the size to prevent
    // the crash, but both solutions lead to unwanted behavior because the
    // AndroidPlatformView(https://github.com/flutter/flutter/blob/master/packages/flutter/lib/src/widgets/platform_view.dart#L677) widget doesn't
    // scale or clamp, which leads to a mismatch between the size of the widget and the size of
    // virtual display.
    // This mismatch leads to some test failures: https://github.com/flutter/flutter/issues/106750
    // TODO(cyanglaz): find a way to prevent the crash without introducing size mistach betewen
    // virtual display and AndroidPlatformView widget.
    // https://github.com/flutter/flutter/issues/93115
    renderTarget.resize(width, height);
    int flags = 0;
    VirtualDisplay virtualDisplay =
        displayManager.createVirtualDisplay(
            "flutter-vd#" + viewId,
            width,
            height,
            metrics.densityDpi,
            renderTarget.getSurface(),
            flags,
            callback,
            null /* handler */);

    if (virtualDisplay == null) {
      return null;
    }
    VirtualDisplayController controller =
        new VirtualDisplayController(
            context,
            accessibilityEventsDelegate,
            virtualDisplay,
            view,
            renderTarget,
            focusChangeListener,
            viewId,
            createParams);
    return controller;
  }

  @VisibleForTesting SingleViewPresentation presentation;

  private final Context context;
  private final AccessibilityEventsDelegate accessibilityEventsDelegate;
  private final int densityDpi;
  private final int viewId;
  private final PlatformViewRenderTarget renderTarget;
  private final OnFocusChangeListener focusChangeListener;

  private VirtualDisplay virtualDisplay;

  private VirtualDisplayController(
      Context context,
      AccessibilityEventsDelegate accessibilityEventsDelegate,
      VirtualDisplay virtualDisplay,
      PlatformView view,
      PlatformViewRenderTarget renderTarget,
      OnFocusChangeListener focusChangeListener,
      int viewId,
      Object createParams) {
    this.context = context;
    this.accessibilityEventsDelegate = accessibilityEventsDelegate;
    this.renderTarget = renderTarget;
    this.focusChangeListener = focusChangeListener;
    this.viewId = viewId;
    this.virtualDisplay = virtualDisplay;
    this.densityDpi = context.getResources().getDisplayMetrics().densityDpi;
    presentation =
        new SingleViewPresentation(
            context,
            this.virtualDisplay.getDisplay(),
            view,
            accessibilityEventsDelegate,
            viewId,
            focusChangeListener);
    presentation.show();
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

  public void resize(final int width, final int height, final Runnable onNewSizeFrameAvailable) {
    // When 'hot reload', although the resize method is triggered, the size of the native View has
    // not changed.
    if (width == getRenderTargetWidth() && height == getRenderTargetHeight()) {
      getView().postDelayed(onNewSizeFrameAvailable, 0);
      return;
    }
    if (Build.VERSION.SDK_INT >= API_LEVELS.API_31) {
      resize31(getView(), width, height, onNewSizeFrameAvailable);
      return;
    }
    boolean isFocused = getView().isFocused();
    final SingleViewPresentation.PresentationState presentationState = presentation.detachState();
    // We detach the surface to prevent it being destroyed when releasing the vd.
    virtualDisplay.setSurface(null);
    virtualDisplay.release();

    final DisplayManager displayManager =
        (DisplayManager) context.getSystemService(Context.DISPLAY_SERVICE);
    renderTarget.resize(width, height);
    int flags = 0;
    virtualDisplay =
        displayManager.createVirtualDisplay(
            "flutter-vd#" + viewId,
            width,
            height,
            densityDpi,
            renderTarget.getSurface(),
            flags,
            callback,
            null /* handler */);

    final View embeddedView = getView();
    // There's a bug in Android version older than O where view tree observer onDrawListeners don't
    // get properly
    // merged when attaching to window, as a workaround we register the on draw listener after the
    // view is attached.
    embeddedView.addOnAttachStateChangeListener(
        new View.OnAttachStateChangeListener() {
          @Override
          public void onViewAttachedToWindow(View v) {
            OneTimeOnDrawListener.schedule(
                embeddedView,
                new Runnable() {
                  @Override
                  public void run() {
                    // We need some delay here until the frame propagates through the vd surface to
                    // the texture,
                    // 128ms was picked pretty arbitrarily based on trial and error.
                    // As long as we invoke the runnable after a new frame is available we avoid the
                    // scaling jank
                    // described in: https://github.com/flutter/flutter/issues/19572
                    // We should ideally run onNewSizeFrameAvailable ASAP to make the embedded view
                    // more responsive
                    // following a resize.
                    embeddedView.postDelayed(onNewSizeFrameAvailable, 128);
                  }
                });
            embeddedView.removeOnAttachStateChangeListener(this);
          }

          @Override
          public void onViewDetachedFromWindow(View v) {}
        });

    // Create a new SingleViewPresentation and show() it before we cancel() the existing
    // presentation. Calling show() and cancel() in this order fixes
    // https://github.com/flutter/flutter/issues/26345 and maintains seamless transition
    // of the contents of the presentation.
    SingleViewPresentation newPresentation =
        new SingleViewPresentation(
            context,
            virtualDisplay.getDisplay(),
            accessibilityEventsDelegate,
            presentationState,
            focusChangeListener,
            isFocused);
    newPresentation.show();
    presentation.cancel();
    presentation = newPresentation;
  }

  public void dispose() {
    // Fix rare crash on HuaWei device described in: https://github.com/flutter/engine/pull/9192
    presentation.cancel();
    presentation.detachState();
    virtualDisplay.release();
    renderTarget.release();
  }

  // On Android versions 31+ resizing of a Virtual Display's Presentation is natively supported.
  @TargetApi(API_LEVELS.API_31)
  private void resize31(
      View embeddedView, int width, int height, final Runnable onNewSizeFrameAvailable) {
    renderTarget.resize(width, height);
    virtualDisplay.resize(width, height, densityDpi);
    // Must update the surface to match the renderTarget's current surface.
    virtualDisplay.setSurface(renderTarget.getSurface());
    embeddedView.postDelayed(onNewSizeFrameAvailable, 0);
  }

  /** See {@link PlatformView#onFlutterViewAttached(View)} */
  /*package*/ void onFlutterViewAttached(@NonNull View flutterView) {
    if (presentation == null || presentation.getView() == null) {
      return;
    }
    presentation.getView().onFlutterViewAttached(flutterView);
  }

  /** See {@link PlatformView#onFlutterViewDetached()} */
  /*package*/ void onFlutterViewDetached() {
    if (presentation == null || presentation.getView() == null) {
      return;
    }
    presentation.getView().onFlutterViewDetached();
  }

  /*package*/ void onInputConnectionLocked() {
    if (presentation == null || presentation.getView() == null) {
      return;
    }
    presentation.getView().onInputConnectionLocked();
  }

  /*package*/ void onInputConnectionUnlocked() {
    if (presentation == null || presentation.getView() == null) {
      return;
    }
    presentation.getView().onInputConnectionUnlocked();
  }

  public View getView() {
    if (presentation == null) return null;
    PlatformView platformView = presentation.getView();
    return platformView.getView();
  }

  /** Dispatches a motion event to the presentation for this controller. */
  public void dispatchTouchEvent(MotionEvent event) {
    if (presentation == null) return;
    presentation.dispatchTouchEvent(event);
  }

  public void clearSurface() {
    virtualDisplay.setSurface(null);
  }

  public void resetSurface() {
    final int width = getRenderTargetWidth();
    final int height = getRenderTargetHeight();
    final boolean isFocused = getView().isFocused();
    final SingleViewPresentation.PresentationState presentationState = presentation.detachState();

    // We detach the surface to prevent it being destroyed when releasing the vd.
    virtualDisplay.setSurface(null);
    virtualDisplay.release();
    final DisplayManager displayManager =
        (DisplayManager) context.getSystemService(Context.DISPLAY_SERVICE);
    int flags = 0;
    virtualDisplay =
        displayManager.createVirtualDisplay(
            "flutter-vd#" + viewId,
            width,
            height,
            densityDpi,
            renderTarget.getSurface(),
            flags,
            callback,
            null /* handler */);
    // Create a new SingleViewPresentation and show() it before we cancel() the existing
    // presentation. Calling show() and cancel() in this order fixes
    // https://github.com/flutter/flutter/issues/26345 and maintains seamless transition
    // of the contents of the presentation.
    SingleViewPresentation newPresentation =
        new SingleViewPresentation(
            context,
            virtualDisplay.getDisplay(),
            accessibilityEventsDelegate,
            presentationState,
            focusChangeListener,
            isFocused);
    newPresentation.show();
    presentation.cancel();
    presentation = newPresentation;
  }

  static class OneTimeOnDrawListener implements ViewTreeObserver.OnDrawListener {
    static void schedule(View view, Runnable runnable) {
      OneTimeOnDrawListener listener = new OneTimeOnDrawListener(view, runnable);
      view.getViewTreeObserver().addOnDrawListener(listener);
    }

    final View mView;
    Runnable mOnDrawRunnable;

    OneTimeOnDrawListener(View view, Runnable onDrawRunnable) {
      this.mView = view;
      this.mOnDrawRunnable = onDrawRunnable;
    }

    @Override
    public void onDraw() {
      if (mOnDrawRunnable == null) {
        return;
      }
      mOnDrawRunnable.run();
      mOnDrawRunnable = null;
      mView.post(
          new Runnable() {
            @Override
            public void run() {
              mView.getViewTreeObserver().removeOnDrawListener(OneTimeOnDrawListener.this);
            }
          });
    }
  }
}
