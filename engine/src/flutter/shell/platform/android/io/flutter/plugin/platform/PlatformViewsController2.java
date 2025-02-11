// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.platform;

import static io.flutter.Build.API_LEVELS;

import android.annotation.TargetApi;
import android.content.Context;
import android.graphics.PixelFormat;
import android.util.SparseArray;
import android.view.MotionEvent;
import android.view.MotionEvent.PointerCoords;
import android.view.MotionEvent.PointerProperties;
import android.view.Surface;
import android.view.SurfaceControl;
import android.view.View;
import android.view.ViewGroup;
import android.widget.FrameLayout;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.RequiresApi;
import androidx.annotation.UiThread;
import androidx.annotation.VisibleForTesting;
import io.flutter.Log;
import io.flutter.embedding.android.AndroidTouchProcessor;
import io.flutter.embedding.android.FlutterView;
import io.flutter.embedding.android.MotionEventTracker;
import io.flutter.embedding.engine.FlutterJNI;
import io.flutter.embedding.engine.FlutterOverlaySurface;
import io.flutter.embedding.engine.dart.DartExecutor;
import io.flutter.embedding.engine.mutatorsstack.*;
import io.flutter.embedding.engine.renderer.FlutterRenderer;
import io.flutter.embedding.engine.systemchannels.PlatformViewsChannel2;
import io.flutter.plugin.editing.TextInputPlugin;
import io.flutter.view.AccessibilityBridge;
import java.util.ArrayList;
import java.util.List;

/**
 * Manages platform views.
 *
 * <p>Each {@link io.flutter.embedding.engine.FlutterEngine} has a single platform views controller.
 * A platform views controller can be attached to at most one Flutter view.
 */
public class PlatformViewsController2 implements PlatformViewsAccessibilityDelegate {
  private static final String TAG = "PlatformViewsController2";

  private PlatformViewRegistryImpl registry;
  private AndroidTouchProcessor androidTouchProcessor;
  private Context context;
  private FlutterView flutterView;
  private FlutterJNI flutterJNI = null;

  @Nullable private TextInputPlugin textInputPlugin;

  private PlatformViewsChannel2 platformViewsChannel;
  private final AccessibilityEventsDelegate accessibilityEventsDelegate;

  private final SparseArray<PlatformView> platformViews;
  private final SparseArray<FlutterMutatorView> platformViewParent;
  private final MotionEventTracker motionEventTracker;

  private final ArrayList<SurfaceControl.Transaction> pendingTransactions;
  private final ArrayList<SurfaceControl.Transaction> activeTransactions;
  private Surface overlayerSurface = null;
  private SurfaceControl overlaySurfaceControl = null;

  public PlatformViewsController2() {
    accessibilityEventsDelegate = new AccessibilityEventsDelegate();
    platformViews = new SparseArray<>();
    platformViewParent = new SparseArray<>();
    pendingTransactions = new ArrayList<>();
    activeTransactions = new ArrayList<>();
    motionEventTracker = MotionEventTracker.getInstance();
  }

  public void setRegistry(@NonNull PlatformViewRegistry registry) {
    this.registry = (PlatformViewRegistryImpl) registry;
  }

  /** Whether the SurfaceControl swapchain mode is enabled. */
  public void setFlutterJNI(FlutterJNI flutterJNI) {
    this.flutterJNI = flutterJNI;
  }

  @Override
  public boolean usesVirtualDisplay(int id) {
    return false;
  }

  public PlatformView createFlutterPlatformView(
      @NonNull PlatformViewsChannel2.PlatformViewCreationRequest request) {
    final PlatformViewFactory viewFactory = registry.getFactory(request.viewType);
    if (viewFactory == null) {
      throw new IllegalStateException(
          "Trying to create a platform view of unregistered type: " + request.viewType);
    }

    Object createParams = null;
    if (request.params != null) {
      createParams = viewFactory.getCreateArgsCodec().decodeMessage(request.params);
    }
    final PlatformView platformView = viewFactory.create(context, request.viewId, createParams);

    // Configure the view to match the requested layout direction.
    final View embeddedView = platformView.getView();
    if (embeddedView == null) {
      throw new IllegalStateException(
          "PlatformView#getView() returned null, but an Android view reference was expected.");
    }
    embeddedView.setLayoutDirection(request.direction);
    platformViews.put(request.viewId, platformView);
    maybeInvokeOnFlutterViewAttached(platformView);
    return platformView;
  }

  /**
   * Translates an original touch event to have the same locations as the ones that Flutter
   * calculates (because original + flutter's - original = flutter's).
   *
   * @param originalEvent The saved original input event.
   * @param pointerCoords The coordinates that Flutter thinks the touch is happening at.
   */
  private static void translateMotionEvent(
      MotionEvent originalEvent, PointerCoords[] pointerCoords) {
    if (pointerCoords.length < 1) {
      return;
    }

    float xOffset = pointerCoords[0].x - originalEvent.getX();
    float yOffset = pointerCoords[0].y - originalEvent.getY();

    originalEvent.offsetLocation(xOffset, yOffset);
  }

  @VisibleForTesting
  public MotionEvent toMotionEvent(float density, PlatformViewsChannel2.PlatformViewTouch touch) {
    MotionEventTracker.MotionEventId motionEventId =
        MotionEventTracker.MotionEventId.from(touch.motionEventId);
    MotionEvent trackedEvent = motionEventTracker.pop(motionEventId);

    // Pointer coordinates in the tracked events are global to FlutterView
    // The framework converts them to be local to a widget, given that
    // motion events operate on local coords, we need to replace these in the tracked
    // event with their local counterparts.
    // Compute this early so it can be used as input to translateNonVirtualDisplayMotionEvent.
    PointerCoords[] pointerCoords =
        parsePointerCoordsList(touch.rawPointerCoords, density)
            .toArray(new PointerCoords[touch.pointerCount]);

    if (trackedEvent != null) {
      // We have the original event, deliver it after offsetting as it will pass the verifiable
      // input check.
      translateMotionEvent(trackedEvent, pointerCoords);
      return trackedEvent;
    }
    // We don't have a reference to the original MotionEvent.
    // In this case we manually recreate a MotionEvent to be delivered. This MotionEvent
    // will fail the verifiable input check.
    PointerProperties[] pointerProperties =
        parsePointerPropertiesList(touch.rawPointerPropertiesList)
            .toArray(new PointerProperties[touch.pointerCount]);

    return MotionEvent.obtain(
        touch.downTime.longValue(),
        touch.eventTime.longValue(),
        touch.action,
        touch.pointerCount,
        pointerProperties,
        pointerCoords,
        touch.metaState,
        touch.buttonState,
        touch.xPrecision,
        touch.yPrecision,
        touch.deviceId,
        touch.edgeFlags,
        touch.source,
        touch.flags);
  }

  /**
   * Attaches this platform views controller to its input and output channels.
   *
   * @param context The base context that will be passed to embedded views created by this
   *     controller. This should be the context of the Activity hosting the Flutter application.
   * @param dartExecutor The dart execution context, which is used to set up a system channel.
   */
  public void attach(@Nullable Context context, @NonNull DartExecutor dartExecutor) {
    if (this.context != null) {
      throw new AssertionError(
          "A PlatformViewsController can only be attached to a single output target.\n"
              + "attach was called while the PlatformViewsController was already attached.");
    }
    this.context = context;
    platformViewsChannel = new PlatformViewsChannel2(dartExecutor);
    platformViewsChannel.setPlatformViewsHandler(channelHandler);
  }

  /**
   * Detaches this platform views controller.
   *
   * <p>This is typically called when a Flutter applications moves to run in the background, or is
   * destroyed. After calling this the platform views controller will no longer listen to it's
   * previous messenger, and will not maintain references to the texture registry, context, and
   * messenger passed to the previous attach call.
   */
  @UiThread
  public void detach() {
    if (platformViewsChannel != null) {
      platformViewsChannel.setPlatformViewsHandler(null);
    }
    destroyOverlaySurface();
    platformViewsChannel = null;
    context = null;
  }

  /**
   * Attaches the controller to a {@link FlutterView}.
   *
   * <p>When {@link io.flutter.embedding.android.FlutterFragment} is used, this method is called
   * after the device rotates since the FlutterView is recreated after a rotation.
   */
  public void attachToView(@NonNull FlutterView newFlutterView) {
    flutterView = newFlutterView;
    // Add wrapper for platform views that are composed at the view hierarchy level.
    for (int index = 0; index < platformViewParent.size(); index++) {
      final FlutterMutatorView view = platformViewParent.valueAt(index);
      flutterView.addView(view);
    }
    // Notify platform views that they are now attached to a FlutterView.
    for (int index = 0; index < platformViews.size(); index++) {
      final PlatformView view = platformViews.valueAt(index);
      view.onFlutterViewAttached(flutterView);
    }
  }

  /**
   * Detaches the controller from {@link FlutterView}.
   *
   * <p>When {@link io.flutter.embedding.android.FlutterFragment} is used, this method is called
   * when the device rotates since the FlutterView is detached from the fragment. The next time the
   * fragment needs to be displayed, a new Flutter view is created, so attachToView is called again.
   */
  public void detachFromView() {
    // Remove wrapper for platform views that are composed at the view hierarchy level.
    for (int index = 0; index < platformViewParent.size(); index++) {
      final FlutterMutatorView view = platformViewParent.valueAt(index);
      flutterView.removeView(view);
    }

    destroyOverlaySurface();
    flutterView = null;

    // Notify that the platform view have been detached from FlutterView.
    for (int index = 0; index < platformViews.size(); index++) {
      final PlatformView view = platformViews.valueAt(index);
      view.onFlutterViewDetached();
    }
  }

  private void maybeInvokeOnFlutterViewAttached(PlatformView view) {
    if (flutterView == null) {
      Log.i(TAG, "null flutterView");
      // There is currently no FlutterView that we are attached to.
      return;
    }
    view.onFlutterViewAttached(flutterView);
  }

  @Override
  public void attachAccessibilityBridge(@NonNull AccessibilityBridge accessibilityBridge) {
    accessibilityEventsDelegate.setAccessibilityBridge(accessibilityBridge);
  }

  @Override
  public void detachAccessibilityBridge() {
    accessibilityEventsDelegate.setAccessibilityBridge(null);
  }

  /**
   * Attaches this controller to a text input plugin.
   *
   * <p>While a text input plugin is available, the platform views controller interacts with it to
   * facilitate delegation of text input connections to platform views.
   *
   * <p>A platform views controller should be attached to a text input plugin whenever it is
   * possible for the Flutter framework to receive text input.
   */
  public void attachTextInputPlugin(@NonNull TextInputPlugin textInputPlugin) {
    this.textInputPlugin = textInputPlugin;
  }

  /** Detaches this controller from the currently attached text input plugin. */
  public void detachTextInputPlugin() {
    textInputPlugin = null;
  }

  public PlatformViewRegistry getRegistry() {
    return registry;
  }

  /**
   * Invoked when the {@link io.flutter.embedding.engine.FlutterEngine} that owns this {@link
   * PlatformViewsController} detaches from JNI.
   */
  public void onDetachedFromJNI() {
    diposeAllViews();
  }

  public void onPreEngineRestart() {
    diposeAllViews();
  }

  @Override
  @Nullable
  public View getPlatformViewById(int viewId) {
    final PlatformView platformView = platformViews.get(viewId);
    if (platformView == null) {
      return null;
    }
    return platformView.getView();
  }

  private void lockInputConnection(@NonNull VirtualDisplayController controller) {
    if (textInputPlugin == null) {
      return;
    }
    textInputPlugin.lockPlatformViewInputConnection();
    controller.onInputConnectionLocked();
  }

  private void unlockInputConnection(@NonNull VirtualDisplayController controller) {
    if (textInputPlugin == null) {
      return;
    }
    textInputPlugin.unlockPlatformViewInputConnection();
    controller.onInputConnectionUnlocked();
  }

  private static boolean validateDirection(int direction) {
    return direction == View.LAYOUT_DIRECTION_LTR || direction == View.LAYOUT_DIRECTION_RTL;
  }

  @SuppressWarnings("unchecked")
  private static List<PointerProperties> parsePointerPropertiesList(Object rawPropertiesList) {
    List<Object> rawProperties = (List<Object>) rawPropertiesList;
    List<PointerProperties> pointerProperties = new ArrayList<>();
    for (Object o : rawProperties) {
      pointerProperties.add(parsePointerProperties(o));
    }
    return pointerProperties;
  }

  @SuppressWarnings("unchecked")
  private static PointerProperties parsePointerProperties(Object rawProperties) {
    List<Object> propertiesList = (List<Object>) rawProperties;
    PointerProperties properties = new MotionEvent.PointerProperties();
    properties.id = (int) propertiesList.get(0);
    properties.toolType = (int) propertiesList.get(1);
    return properties;
  }

  @SuppressWarnings("unchecked")
  private static List<PointerCoords> parsePointerCoordsList(Object rawCoordsList, float density) {
    List<Object> rawCoords = (List<Object>) rawCoordsList;
    List<PointerCoords> pointerCoords = new ArrayList<>();
    for (Object o : rawCoords) {
      pointerCoords.add(parsePointerCoords(o, density));
    }
    return pointerCoords;
  }

  @SuppressWarnings("unchecked")
  private static PointerCoords parsePointerCoords(Object rawCoords, float density) {
    List<Object> coordsList = (List<Object>) rawCoords;
    PointerCoords coords = new MotionEvent.PointerCoords();
    coords.orientation = (float) (double) coordsList.get(0);
    coords.pressure = (float) (double) coordsList.get(1);
    coords.size = (float) (double) coordsList.get(2);
    coords.toolMajor = (float) ((double) coordsList.get(3) * density);
    coords.toolMinor = (float) ((double) coordsList.get(4) * density);
    coords.touchMajor = (float) ((double) coordsList.get(5) * density);
    coords.touchMinor = (float) ((double) coordsList.get(6) * density);
    coords.x = (float) ((double) coordsList.get(7) * density);
    coords.y = (float) ((double) coordsList.get(8) * density);
    return coords;
  }

  private float getDisplayDensity() {
    return context.getResources().getDisplayMetrics().density;
  }

  private int toPhysicalPixels(double logicalPixels) {
    return (int) Math.round(logicalPixels * getDisplayDensity());
  }

  private int toLogicalPixels(double physicalPixels, float displayDensity) {
    return (int) Math.round(physicalPixels / displayDensity);
  }

  private int toLogicalPixels(double physicalPixels) {
    return toLogicalPixels(physicalPixels, getDisplayDensity());
  }

  private void diposeAllViews() {
    while (platformViews.size() > 0) {
      final int viewId = platformViews.keyAt(0);
      // Dispose deletes the entry from platformViews and clears associated resources.
      channelHandler.dispose(viewId);
    }
  }

  /**
   * Disposes a single
   *
   * @param viewId the PlatformView ID.
   */
  @VisibleForTesting
  public void disposePlatformView(int viewId) {
    channelHandler.dispose(viewId);
  }

  /**
   * Initializes a platform view and adds it to the view hierarchy.
   *
   * @param viewId The view ID. This member is not intended for public use, and is only visible for
   *     testing.
   */
  @VisibleForTesting
  boolean initializePlatformViewIfNeeded(int viewId) {
    final PlatformView platformView = platformViews.get(viewId);
    if (platformView == null) {
      return false;
    }
    if (platformViewParent.get(viewId) != null) {
      return true;
    }
    final View embeddedView = platformView.getView();
    if (embeddedView == null) {
      throw new IllegalStateException(
          "PlatformView#getView() returned null, but an Android view reference was expected.");
    }
    if (embeddedView.getParent() != null) {
      throw new IllegalStateException(
          "The Android view returned from PlatformView#getView() was already added to a parent"
              + " view.");
    }
    final FlutterMutatorView parentView =
        new FlutterMutatorView(
            context, context.getResources().getDisplayMetrics().density, androidTouchProcessor);

    parentView.setOnDescendantFocusChangeListener(
        (view, hasFocus) -> {
          if (hasFocus) {
            platformViewsChannel.invokeViewFocused(viewId);
          } else if (textInputPlugin != null) {
            textInputPlugin.clearPlatformViewClient(viewId);
          }
        });

    platformViewParent.put(viewId, parentView);

    // Accessibility in the embedded view is initially disabled because if a Flutter app disabled
    // accessibility in the first frame, the embedding won't receive an update to disable
    // accessibility since the embedding never received an update to enable it.
    // The AccessibilityBridge keeps track of the accessibility nodes, and handles the deltas when
    // the framework sends a new a11y tree to the embedding.
    // To prevent races, the framework populate the SemanticsNode after the platform view has been
    // created.
    embeddedView.setImportantForAccessibility(View.IMPORTANT_FOR_ACCESSIBILITY_NO_HIDE_DESCENDANTS);

    parentView.addView(embeddedView);
    flutterView.addView(parentView);
    return true;
  }

  public void attachToFlutterRenderer(@NonNull FlutterRenderer flutterRenderer) {
    androidTouchProcessor = new AndroidTouchProcessor(flutterRenderer, /*trackMotionEvents=*/ true);
  }

  /**
   * Called when a platform view id displayed in the current frame.
   *
   * @param viewId The ID of the platform view.
   * @param x The left position relative to {@code FlutterView}.
   * @param y The top position relative to {@code FlutterView}.
   * @param width The width of the platform view.
   * @param height The height of the platform view.
   * @param viewWidth The original width of the platform view before applying the mutator stack.
   * @param viewHeight The original height of the platform view before applying the mutator stack.
   * @param mutatorsStack The mutator stack. This member is not intended for public use, and is only
   *     visible for testing.
   */
  public void onDisplayPlatformView(
      int viewId,
      int x,
      int y,
      int width,
      int height,
      int viewWidth,
      int viewHeight,
      @NonNull FlutterMutatorsStack mutatorsStack) {
    if (!initializePlatformViewIfNeeded(viewId)) {
      return;
    }

    final FlutterMutatorView parentView = platformViewParent.get(viewId);
    parentView.readyToDisplay(mutatorsStack, x, y, width, height);
    parentView.setVisibility(View.VISIBLE);
    parentView.bringToFront();

    final FrameLayout.LayoutParams layoutParams =
        new FrameLayout.LayoutParams(viewWidth, viewHeight);
    final View view = platformViews.get(viewId).getView();
    if (view != null) {
      view.setLayoutParams(layoutParams);
      view.bringToFront();
    }
  }

  @TargetApi(API_LEVELS.API_34)
  @RequiresApi(API_LEVELS.API_34)
  public void onEndFrame() {
    SurfaceControl.Transaction tx = new SurfaceControl.Transaction();
    for (int i = 0; i < activeTransactions.size(); i++) {
      tx = tx.merge(activeTransactions.get(i));
    }
    activeTransactions.clear();
    flutterView.invalidate();
    flutterView.getRootSurfaceControl().applyTransactionOnDraw(tx);
  }

  // NOT called from UI thread.
  public synchronized void swapTransactions() {
    activeTransactions.clear();
    for (int i = 0; i < pendingTransactions.size(); i++) {
      activeTransactions.add(pendingTransactions.get(i));
    }
    pendingTransactions.clear();
  }

  // NOT called from UI thread.
  @TargetApi(API_LEVELS.API_34)
  @RequiresApi(API_LEVELS.API_34)
  public SurfaceControl.Transaction createTransaction() {
    SurfaceControl.Transaction tx = new SurfaceControl.Transaction();
    pendingTransactions.add(tx);
    return tx;
  }

  // NOT called from UI thread.
  @TargetApi(API_LEVELS.API_34)
  @RequiresApi(API_LEVELS.API_34)
  public void applyTransactions() {
    SurfaceControl.Transaction tx = new SurfaceControl.Transaction();
    for (int i = 0; i < pendingTransactions.size(); i++) {
      tx = tx.merge(pendingTransactions.get(i));
    }
    tx.apply();
    pendingTransactions.clear();
  }

  @TargetApi(API_LEVELS.API_34)
  @RequiresApi(API_LEVELS.API_34)
  public FlutterOverlaySurface createOverlaySurface() {
    if (overlayerSurface == null) {
      final SurfaceControl.Builder surfaceControlBuilder = new SurfaceControl.Builder();
      surfaceControlBuilder.setBufferSize(flutterView.getWidth(), flutterView.getHeight());
      surfaceControlBuilder.setFormat(PixelFormat.RGBA_8888);
      surfaceControlBuilder.setName("Flutter Overlay Surface");
      surfaceControlBuilder.setOpaque(false);
      surfaceControlBuilder.setHidden(false);
      final SurfaceControl surfaceControl = surfaceControlBuilder.build();
      final SurfaceControl.Transaction tx =
          flutterView.getRootSurfaceControl().buildReparentTransaction(surfaceControl);
      tx.setLayer(surfaceControl, 1000);
      tx.apply();
      overlayerSurface = new Surface(surfaceControl);
      overlaySurfaceControl = surfaceControl;
    }

    return new FlutterOverlaySurface(0, overlayerSurface);
  }

  public void destroyOverlaySurface() {
    if (overlayerSurface != null) {
      overlayerSurface.release();
      overlayerSurface = null;
      overlaySurfaceControl = null;
    }
  }

  @TargetApi(API_LEVELS.API_34)
  @RequiresApi(API_LEVELS.API_34)
  public void showOverlaySurface() {
    if (overlaySurfaceControl == null) {
      return;
    }
    SurfaceControl.Transaction tx = new SurfaceControl.Transaction();
    tx.setVisibility(overlaySurfaceControl, /*visible=*/ true);
    tx.apply();
  }

  @TargetApi(API_LEVELS.API_34)
  @RequiresApi(API_LEVELS.API_34)
  public void hideOverlaySurface() {
    if (overlaySurfaceControl == null) {
      return;
    }
    SurfaceControl.Transaction tx = new SurfaceControl.Transaction();
    tx.setVisibility(overlaySurfaceControl, /*visible=*/ false);
    tx.apply();
  }

  //// Message Handler ///////

  private final PlatformViewsChannel2.PlatformViewsHandler channelHandler =
      new PlatformViewsChannel2.PlatformViewsHandler() {

        @Override
        public void createPlatformView(
            @NonNull PlatformViewsChannel2.PlatformViewCreationRequest request) {
          createFlutterPlatformView(request);
        }

        @Override
        public void dispose(int viewId) {
          final PlatformView platformView = platformViews.get(viewId);
          if (platformView == null) {
            Log.e(TAG, "Disposing unknown platform view with id: " + viewId);
            return;
          }
          if (platformView.getView() != null) {
            final View embeddedView = platformView.getView();
            final ViewGroup pvParent = (ViewGroup) embeddedView.getParent();
            if (pvParent != null) {
              // Eagerly remove the embedded view from the PlatformViewWrapper.
              // Without this call, we see some crashes because removing the view
              // is used as a signal to stop processing.
              pvParent.removeView(embeddedView);
            }
          }
          platformViews.remove(viewId);
          try {
            platformView.dispose();
          } catch (RuntimeException exception) {
            Log.e(TAG, "Disposing platform view threw an exception", exception);
          }

          // The platform view is displayed using a PlatformViewLayer.
          final FlutterMutatorView parentView = platformViewParent.get(viewId);
          if (parentView != null) {
            parentView.removeAllViews();
            parentView.unsetOnDescendantFocusChangeListener();

            final ViewGroup mutatorViewParent = (ViewGroup) parentView.getParent();
            if (mutatorViewParent != null) {
              mutatorViewParent.removeView(parentView);
            }
            platformViewParent.remove(viewId);
          }
        }

        @Override
        public void onTouch(@NonNull PlatformViewsChannel2.PlatformViewTouch touch) {
          final int viewId = touch.viewId;
          final float density = context.getResources().getDisplayMetrics().density;

          final PlatformView platformView = platformViews.get(viewId);
          if (platformView == null) {
            Log.e(TAG, "Sending touch to an unknown view with id: " + viewId);
            return;
          }
          final View view = platformView.getView();
          if (view == null) {
            Log.e(TAG, "Sending touch to a null view with id: " + viewId);
            return;
          }
          final MotionEvent event = toMotionEvent(density, touch);
          view.dispatchTouchEvent(event);
        }

        @Override
        public void setDirection(int viewId, int direction) {
          final PlatformView platformView = platformViews.get(viewId);
          if (platformView == null) {
            Log.e(TAG, "Setting direction to an unknown view with id: " + viewId);
            return;
          }
          View embeddedView = platformView.getView();
          if (embeddedView == null) {
            Log.e(TAG, "Setting direction to a null view with id: " + viewId);
            return;
          }
          embeddedView.setLayoutDirection(direction);
        }

        @Override
        public void clearFocus(int viewId) {
          final PlatformView platformView = platformViews.get(viewId);
          if (platformView == null) {
            Log.e(TAG, "Clearing focus on an unknown view with id: " + viewId);
            return;
          }
          View embeddedView = platformView.getView();
          if (embeddedView == null) {
            Log.e(TAG, "Clearing focus on a null view with id: " + viewId);
            return;
          }
          embeddedView.clearFocus();
        }

        @Override
        public boolean isSurfaceControlEnabled() {
          if (flutterJNI == null) {
            return false;
          }
          return flutterJNI.IsSurfaceControlEnabled();
        }
      };
}
