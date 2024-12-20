// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.platform;

import static android.content.Context.WINDOW_SERVICE;
import static android.view.View.OnFocusChangeListener;

import android.app.AlertDialog;
import android.app.Presentation;
import android.content.Context;
import android.content.ContextWrapper;
import android.content.MutableContextWrapper;
import android.graphics.drawable.ColorDrawable;
import android.os.Bundle;
import android.view.Display;
import android.view.View;
import android.view.WindowManager;
import android.view.accessibility.AccessibilityEvent;
import android.view.inputmethod.InputMethodManager;
import android.widget.FrameLayout;
import androidx.annotation.Keep;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import io.flutter.Log;

/*
 * A presentation used for hosting a single Android view in a virtual display.
 *
 * This presentation overrides the WindowManager's addView/removeView/updateViewLayout methods, such that views added
 * directly to the WindowManager are added as part of the presentation's view hierarchy (to fakeWindowViewGroup).
 *
 * The view hierarchy for the presentation is as following:
 *
 *          rootView
 *         /         \
 *        /           \
 *       /             \
 *   container       state.fakeWindowViewGroup
 *      |
 *   EmbeddedView
 */
@Keep
class SingleViewPresentation extends Presentation {
  private static final String TAG = "PlatformViewsController";

  /*
   * When an embedded view is resized in Flutterverse we move the Android view to a new virtual display
   * that has the new size. This class keeps the presentation state that moves with the view to the presentation of
   * the new virtual display.
   */
  static class PresentationState {
    // The Android view we are embedding in the Flutter app.
    private PlatformView platformView;

    // The InvocationHandler for a WindowManager proxy. This is essentially the custom window
    // manager for the
    // presentation.
    private WindowManagerHandler windowManagerHandler;

    // Contains views that were added directly to the window manager (e.g
    // android.widget.PopupWindow).
    private SingleViewFakeWindowViewGroup fakeWindowViewGroup;
  }

  // A reference to the current accessibility bridge to which accessibility events will be
  // delegated.
  private final AccessibilityEventsDelegate accessibilityEventsDelegate;

  private final OnFocusChangeListener focusChangeListener;

  // This is the view id assigned by the Flutter framework to the embedded view, we keep it here
  // so when we create the platform view we can tell it its view id.
  private int viewId;

  // The root view for the presentation, it has 2 childs: container which contains the embedded
  // view, and
  // fakeWindowViewGroup which contains views that were added directly to the presentation's window
  // manager.
  private AccessibilityDelegatingFrameLayout rootView;

  // Contains the embedded platform view (platformView.getView()) when it is attached to the
  // presentation.
  private FrameLayout container;

  private final PresentationState state;

  private boolean startFocused = false;

  // The context for the application window that hosts FlutterView.
  private final Context outerContext;

  /**
   * Creates a presentation that will use the view factory to create a new platform view in the
   * presentation's onCreate, and attach it.
   */
  public SingleViewPresentation(
      Context outerContext,
      Display display,
      PlatformView view,
      AccessibilityEventsDelegate accessibilityEventsDelegate,
      int viewId,
      OnFocusChangeListener focusChangeListener) {
    super(new ImmContext(outerContext), display);
    this.accessibilityEventsDelegate = accessibilityEventsDelegate;
    this.viewId = viewId;
    this.focusChangeListener = focusChangeListener;
    this.outerContext = outerContext;
    state = new PresentationState();
    state.platformView = view;
    getWindow()
        .setFlags(
            WindowManager.LayoutParams.FLAG_NOT_FOCUSABLE,
            WindowManager.LayoutParams.FLAG_NOT_FOCUSABLE);
    getWindow().setType(WindowManager.LayoutParams.TYPE_PRIVATE_PRESENTATION);
  }

  /**
   * Creates a presentation that will attach an already existing view as its root view.
   *
   * <p>The display's density must match the density of the context used when the view was created.
   */
  public SingleViewPresentation(
      Context outerContext,
      Display display,
      AccessibilityEventsDelegate accessibilityEventsDelegate,
      PresentationState state,
      OnFocusChangeListener focusChangeListener,
      boolean startFocused) {
    super(new ImmContext(outerContext), display);
    this.accessibilityEventsDelegate = accessibilityEventsDelegate;
    this.state = state;
    this.focusChangeListener = focusChangeListener;
    this.outerContext = outerContext;
    getWindow()
        .setFlags(
            WindowManager.LayoutParams.FLAG_NOT_FOCUSABLE,
            WindowManager.LayoutParams.FLAG_NOT_FOCUSABLE);
    this.startFocused = startFocused;
  }

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    // This makes sure we preserve alpha for the VD's content.
    getWindow().setBackgroundDrawable(new ColorDrawable(android.graphics.Color.TRANSPARENT));
    if (state.fakeWindowViewGroup == null) {
      state.fakeWindowViewGroup = new SingleViewFakeWindowViewGroup(getContext());
    }
    if (state.windowManagerHandler == null) {
      WindowManager windowManagerDelegate =
          (WindowManager) getContext().getSystemService(WINDOW_SERVICE);
      state.windowManagerHandler =
          new WindowManagerHandler(windowManagerDelegate, state.fakeWindowViewGroup);
    }

    container = new FrameLayout(getContext());

    // Our base mContext has already been wrapped with an IMM cache at instantiation time, but
    // we want to wrap it again here to also return state.windowManagerHandler.
    Context baseContext =
        new PresentationContext(getContext(), state.windowManagerHandler, outerContext);

    View embeddedView = state.platformView.getView();
    if (embeddedView.getContext() instanceof MutableContextWrapper) {
      MutableContextWrapper currentContext = (MutableContextWrapper) embeddedView.getContext();
      currentContext.setBaseContext(baseContext);
    } else {
      // In some cases, such as when using LayoutInflator, the original context
      // may not be preserved. For backward compatibility with previous
      // implementations of Virtual Display, which didn't validate the context,
      // continue, but log a warning indicating that some functionality may not
      // work as expected.
      // See https://github.com/flutter/flutter/issues/110146 for context.
      Log.w(
          TAG,
          "Unexpected platform view context for view ID "
              + viewId
              + "; some functionality may not work correctly. When constructing a platform view "
              + "in the factory, ensure that the view returned from PlatformViewFactory#create "
              + "returns the provided context from getContext(). If you are unable to associate "
              + "the view with that context, consider using Hybrid Composition instead.");
    }

    container.addView(embeddedView);
    rootView =
        new AccessibilityDelegatingFrameLayout(
            getContext(), accessibilityEventsDelegate, embeddedView);
    rootView.addView(container);
    rootView.addView(state.fakeWindowViewGroup);

    embeddedView.setOnFocusChangeListener(focusChangeListener);
    rootView.setFocusableInTouchMode(true);
    if (startFocused) {
      embeddedView.requestFocus();
    } else {
      rootView.requestFocus();
    }
    setContentView(rootView);
  }

  public PresentationState detachState() {
    // These views can be null before onCreate() is called
    if (container != null) {
      container.removeAllViews();
    }
    if (rootView != null) {
      rootView.removeAllViews();
    }
    return state;
  }

  @Nullable
  public PlatformView getView() {
    return state.platformView;
  }

  /** Answers calls for {@link InputMethodManager} with an instance cached at creation time. */
  // TODO(mklim): This caches the IMM at construction time and won't pick up any changes. In rare
  // cases where the FlutterView changes windows this will return an outdated instance. This
  // should be fixed to instead defer returning the IMM to something that know's FlutterView's
  // true Context.
  private static class ImmContext extends ContextWrapper {
    private @NonNull final InputMethodManager inputMethodManager;

    ImmContext(Context base) {
      this(base, /*inputMethodManager=*/ null);
    }

    private ImmContext(Context base, @Nullable InputMethodManager inputMethodManager) {
      super(base);
      this.inputMethodManager =
          inputMethodManager != null
              ? inputMethodManager
              : (InputMethodManager) base.getSystemService(INPUT_METHOD_SERVICE);
    }

    @Override
    public Object getSystemService(String name) {
      if (INPUT_METHOD_SERVICE.equals(name)) {
        return inputMethodManager;
      }
      return super.getSystemService(name);
    }

    @Override
    public Context createDisplayContext(Display display) {
      Context displayContext = super.createDisplayContext(display);
      return new ImmContext(displayContext, inputMethodManager);
    }
  }

  /** Proxies a Context replacing the WindowManager with our custom instance. */
  // TODO(mklim): This caches the IMM at construction time and won't pick up any changes. In rare
  // cases where the FlutterView changes windows this will return an outdated instance. This
  // should be fixed to instead defer returning the IMM to something that know's FlutterView's
  // true Context.
  private static class PresentationContext extends ContextWrapper {
    private @NonNull final WindowManagerHandler windowManagerHandler;
    private @Nullable WindowManager windowManager;
    private final Context flutterAppWindowContext;

    PresentationContext(
        Context base,
        @NonNull WindowManagerHandler windowManagerHandler,
        Context flutterAppWindowContext) {
      super(base);
      this.windowManagerHandler = windowManagerHandler;
      this.flutterAppWindowContext = flutterAppWindowContext;
    }

    @Override
    public Object getSystemService(String name) {
      if (WINDOW_SERVICE.equals(name)) {
        if (isCalledFromAlertDialog()) {
          // Alert dialogs are showing on top of the entire application and should not be limited to
          // the virtual
          // display. If we detect that an android.app.AlertDialog constructor is what's fetching
          // the window manager
          // we return the one for the application's window.
          //
          // Note that if we don't do this AlertDialog will throw a ClassCastException as down the
          // line it tries
          // to case this instance to a WindowManagerImpl which the object returned by
          // getWindowManager is not
          // a subclass of.
          return flutterAppWindowContext.getSystemService(name);
        }
        return getWindowManager();
      }
      return super.getSystemService(name);
    }

    private WindowManager getWindowManager() {
      if (windowManager == null) {
        windowManager = windowManagerHandler;
      }
      return windowManager;
    }

    private boolean isCalledFromAlertDialog() {
      StackTraceElement[] stackTraceElements = Thread.currentThread().getStackTrace();
      for (int i = 0; i < stackTraceElements.length && i < 11; i++) {
        if (stackTraceElements[i].getClassName().equals(AlertDialog.class.getCanonicalName())
            && stackTraceElements[i].getMethodName().equals("<init>")) {
          return true;
        }
      }
      return false;
    }
  }

  private static class AccessibilityDelegatingFrameLayout extends FrameLayout {
    private final AccessibilityEventsDelegate accessibilityEventsDelegate;
    private final View embeddedView;

    public AccessibilityDelegatingFrameLayout(
        Context context,
        AccessibilityEventsDelegate accessibilityEventsDelegate,
        View embeddedView) {
      super(context);
      this.accessibilityEventsDelegate = accessibilityEventsDelegate;
      this.embeddedView = embeddedView;
    }

    @Override
    public boolean requestSendAccessibilityEvent(View child, AccessibilityEvent event) {
      return accessibilityEventsDelegate.requestSendAccessibilityEvent(embeddedView, child, event);
    }
  }
}
