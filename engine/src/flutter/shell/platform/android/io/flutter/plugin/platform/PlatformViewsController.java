// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.platform;

import static io.flutter.Build.API_LEVELS;

import android.annotation.TargetApi;
import android.content.ComponentCallbacks2;
import android.content.Context;
import android.content.MutableContextWrapper;
import android.os.Build;
import android.util.SparseArray;
import android.view.MotionEvent;
import android.view.MotionEvent.PointerCoords;
import android.view.MotionEvent.PointerProperties;
import android.view.SurfaceView;
import android.view.View;
import android.view.ViewGroup;
import android.widget.FrameLayout;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.UiThread;
import androidx.annotation.VisibleForTesting;
import io.flutter.Log;
import io.flutter.embedding.android.AndroidTouchProcessor;
import io.flutter.embedding.android.FlutterView;
import io.flutter.embedding.android.MotionEventTracker;
import io.flutter.embedding.engine.FlutterOverlaySurface;
import io.flutter.embedding.engine.dart.DartExecutor;
import io.flutter.embedding.engine.mutatorsstack.*;
import io.flutter.embedding.engine.renderer.FlutterRenderer;
import io.flutter.embedding.engine.systemchannels.PlatformViewsChannel;
import io.flutter.plugin.editing.TextInputPlugin;
import io.flutter.util.ViewUtils;
import io.flutter.view.AccessibilityBridge;
import io.flutter.view.TextureRegistry;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;

/**
 * Manages platform views.
 *
 * <p>Each {@link io.flutter.embedding.engine.FlutterEngine} has a single platform views controller.
 * A platform views controller can be attached to at most one Flutter view.
 */
public class PlatformViewsController implements PlatformViewsAccessibilityDelegate {
  private static final String TAG = "PlatformViewsController";

  // These view types allow out-of-band drawing commands that don't notify the Android view
  // hierarchy.
  // To support these cases, Flutter hosts the embedded view in a VirtualDisplay,
  // and binds the VirtualDisplay to a GL texture that is then composed by the engine.
  // However, there are a few issues with Virtual Displays. For example, they don't fully support
  // accessibility due to https://github.com/flutter/flutter/issues/29717,
  // and keyboard interactions may have non-deterministic behavior.
  // Views that issue out-of-band drawing commands that aren't included in this array are
  // required to call `View#invalidate()` to notify Flutter about the update.
  // This isn't ideal, but given all the other limitations it's a reasonable tradeoff.
  // Related issue: https://github.com/flutter/flutter/issues/103630
  private static Class[] VIEW_TYPES_REQUIRE_VIRTUAL_DISPLAY = {SurfaceView.class};

  private final PlatformViewRegistryImpl registry;

  private AndroidTouchProcessor androidTouchProcessor;

  // The context of the Activity or Fragment hosting the render target for the Flutter engine.
  private Context context;

  // The View currently rendering the Flutter UI associated with these platform views.
  private FlutterView flutterView;

  // The texture registry maintaining the textures into which the embedded views will be rendered.
  @Nullable private TextureRegistry textureRegistry;

  @Nullable private TextInputPlugin textInputPlugin;

  // The system channel used to communicate with the framework about platform views.
  private PlatformViewsChannel platformViewsChannel;

  // The accessibility bridge to which accessibility events form the platform views will be
  // dispatched.
  private final AccessibilityEventsDelegate accessibilityEventsDelegate;

  // TODO(mattcarroll): Refactor overall platform views to facilitate testing and then make
  // this private. This is visible as a hack to facilitate testing. This was deemed the least
  // bad option at the time of writing.
  @VisibleForTesting /* package */ final HashMap<Integer, VirtualDisplayController> vdControllers;

  // Maps a virtual display's context to the embedded view hosted in this virtual display.
  // Since each virtual display has it's unique context this allows associating any view with the
  // platform view that
  // it is associated with(e.g if a platform view creates other views in the same virtual display.
  @VisibleForTesting /* package */ final HashMap<Context, View> contextToEmbeddedView;

  // The platform views.
  private final SparseArray<PlatformView> platformViews;

  // The platform view wrappers that are appended to FlutterView.
  //
  // These platform views use a PlatformViewLayer in the framework. This is different than
  // the platform views that use a TextureLayer.
  //
  // This distinction is necessary because a PlatformViewLayer allows to embed Android's
  // SurfaceViews in a Flutter app whereas the texture layer is unable to support such native views.
  //
  // If an entry in `platformViews` doesn't have an entry in this array, the platform view isn't
  // in the view hierarchy.
  //
  // This view provides a wrapper that applies scene builder operations to the platform view.
  // For example, a transform matrix, or setting opacity on the platform view layer.
  private final SparseArray<FlutterMutatorView> platformViewParent;

  // Map of unique IDs to views that render overlay layers.
  private final SparseArray<PlatformOverlayView> overlayLayerViews;

  // The platform view wrappers that are appended to FlutterView.
  //
  // These platform views use a TextureLayer in the framework. This is different than
  // the platform views that use a PlatformViewLayer.
  //
  // This is the default mode, and recommended for better performance.
  private final SparseArray<PlatformViewWrapper> viewWrappers;

  // Next available unique ID for use in overlayLayerViews.
  private int nextOverlayLayerId = 0;

  // Tracks whether the flutterView has been converted to use a FlutterImageView.
  private boolean flutterViewConvertedToImageView = false;

  // When adding platform views using Hybrid Composition, the engine converts the render surface
  // to a FlutterImageView to help improve animation synchronization on Android. This flag allows
  // disabling this conversion through the PlatformView platform channel.
  private boolean synchronizeToNativeViewHierarchy = true;

  // Overlay layer IDs that were displayed since the start of the current frame.
  private final HashSet<Integer> currentFrameUsedOverlayLayerIds;

  // Platform view IDs that were displayed since the start of the current frame.
  private final HashSet<Integer> currentFrameUsedPlatformViewIds;

  // Used to acquire the original motion events using the motionEventIds.
  private final MotionEventTracker motionEventTracker;

  // Whether software rendering is used.
  private boolean usesSoftwareRendering = false;

  private static boolean enableImageRenderTarget = true;

  private static boolean enableSurfaceProducerRenderTarget = true;

  private final PlatformViewsChannel.PlatformViewsHandler channelHandler =
      new PlatformViewsChannel.PlatformViewsHandler() {

        @Override
        // TODO(egarciad): Remove the need for this.
        // https://github.com/flutter/flutter/issues/96679
        public void createForPlatformViewLayer(
            @NonNull PlatformViewsChannel.PlatformViewCreationRequest request) {
          // API level 19 is required for `android.graphics.ImageReader`.
          enforceMinimumAndroidApiVersion(19);
          ensureValidRequest(request);

          final PlatformView platformView = createPlatformView(request, false);

          configureForHybridComposition(platformView, request);
          // New code should be added to configureForHybridComposition, not here, unless it is
          // not applicable to fallback from TLHC to HC.
        }

        @Override
        public long createForTextureLayer(
            @NonNull PlatformViewsChannel.PlatformViewCreationRequest request) {
          ensureValidRequest(request);
          final int viewId = request.viewId;
          if (viewWrappers.get(viewId) != null) {
            throw new IllegalStateException(
                "Trying to create an already created platform view, view id: " + viewId);
          }
          if (textureRegistry == null) {
            throw new IllegalStateException(
                "Texture registry is null. This means that platform views controller was detached,"
                    + " view id: "
                    + viewId);
          }
          if (flutterView == null) {
            throw new IllegalStateException(
                "Flutter view is null. This means the platform views controller doesn't have an"
                    + " attached view, view id: "
                    + viewId);
          }

          final PlatformView platformView = createPlatformView(request, true);

          final View embeddedView = platformView.getView();
          if (embeddedView.getParent() != null) {
            throw new IllegalStateException(
                "The Android view returned from PlatformView#getView() was already added to a"
                    + " parent view.");
          }

          // The newer Texture Layer Hybrid Composition mode isn't suppported if any of the
          // following are true:
          // - The embedded view contains any of the VIEW_TYPES_REQUIRE_VIRTUAL_DISPLAY view types.
          //   These views allow out-of-band graphics operations that aren't notified to the Android
          //   view hierarchy via callbacks such as ViewParent#onDescendantInvalidated().
          // - The API level is <23, due to TLHC implementation API requirements.
          final boolean supportsTextureLayerMode =
              Build.VERSION.SDK_INT >= API_LEVELS.API_23
                  && !ViewUtils.hasChildViewOfType(
                      embeddedView, VIEW_TYPES_REQUIRE_VIRTUAL_DISPLAY);

          // Fall back to Hybrid Composition or Virtual Display when necessary, depending on which
          // fallback mode is requested.
          if (!supportsTextureLayerMode) {
            if (request.displayMode
                == PlatformViewsChannel.PlatformViewCreationRequest.RequestedDisplayMode
                    .TEXTURE_WITH_HYBRID_FALLBACK) {
              configureForHybridComposition(platformView, request);
              return PlatformViewsChannel.PlatformViewsHandler.NON_TEXTURE_FALLBACK;
            } else if (!usesSoftwareRendering) { // Virtual Display doesn't support software mode.
              return configureForVirtualDisplay(platformView, request);
            }
            // TODO(stuartmorgan): Consider throwing a specific exception here as a breaking change.
            // For now, preserve the 3.0 behavior of falling through to Texture Layer mode even
            // though it won't work correctly.
          }
          return configureForTextureLayerComposition(platformView, request);
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
          if (usesVirtualDisplay(viewId)) {
            final VirtualDisplayController vdController = vdControllers.get(viewId);
            final View embeddedView = vdController.getView();
            if (embeddedView != null) {
              contextToEmbeddedView.remove(embeddedView.getContext());
            }
            vdController.dispose();
            vdControllers.remove(viewId);
            return;
          }
          // The platform view is displayed using a TextureLayer and is inserted in the view
          // hierarchy.
          final PlatformViewWrapper viewWrapper = viewWrappers.get(viewId);
          if (viewWrapper != null) {
            viewWrapper.removeAllViews();
            viewWrapper.release();
            viewWrapper.unsetOnDescendantFocusChangeListener();

            final ViewGroup wrapperParent = (ViewGroup) viewWrapper.getParent();
            if (wrapperParent != null) {
              wrapperParent.removeView(viewWrapper);
            }
            viewWrappers.remove(viewId);
            return;
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
        public void offset(int viewId, double top, double left) {
          if (usesVirtualDisplay(viewId)) {
            // Virtual displays don't need an accessibility offset.
            return;
          }
          // For platform views that use TextureView and are in the view hierarchy, set
          // an offset to the wrapper view.
          // This ensures that the accessibility highlights are drawn in the expected position on
          // screen.
          // This offset doesn't affect the position of the embeded view by itself since the GL
          // texture is positioned by the Flutter engine, which knows where to position different
          // types of layers.
          final PlatformViewWrapper viewWrapper = viewWrappers.get(viewId);
          if (viewWrapper == null) {
            Log.e(TAG, "Setting offset for unknown platform view with id: " + viewId);
            return;
          }
          final int physicalTop = toPhysicalPixels(top);
          final int physicalLeft = toPhysicalPixels(left);
          final FrameLayout.LayoutParams layoutParams =
              (FrameLayout.LayoutParams) viewWrapper.getLayoutParams();
          layoutParams.topMargin = physicalTop;
          layoutParams.leftMargin = physicalLeft;
          viewWrapper.setLayoutParams(layoutParams);
        }

        @Override
        public void resize(
            @NonNull PlatformViewsChannel.PlatformViewResizeRequest request,
            @NonNull PlatformViewsChannel.PlatformViewBufferResized onComplete) {
          final int physicalWidth = toPhysicalPixels(request.newLogicalWidth);
          final int physicalHeight = toPhysicalPixels(request.newLogicalHeight);
          final int viewId = request.viewId;

          if (usesVirtualDisplay(viewId)) {
            final float originalDisplayDensity = getDisplayDensity();
            final VirtualDisplayController vdController = vdControllers.get(viewId);
            // Resizing involved moving the platform view to a new virtual display. Doing so
            // potentially results in losing an active input connection. To make sure we preserve
            // the input connection when resizing we lock it here and unlock after the resize is
            // complete.
            lockInputConnection(vdController);
            vdController.resize(
                physicalWidth,
                physicalHeight,
                () -> {
                  unlockInputConnection(vdController);
                  // Converting back to logic pixels requires a context, which may no longer be
                  // available. If that happens, assume the same logic/physical relationship as
                  // was present when the request arrived.
                  final float displayDensity =
                      context == null ? originalDisplayDensity : getDisplayDensity();
                  onComplete.run(
                      new PlatformViewsChannel.PlatformViewBufferSize(
                          toLogicalPixels(vdController.getRenderTargetWidth(), displayDensity),
                          toLogicalPixels(vdController.getRenderTargetHeight(), displayDensity)));
                });
            return;
          }

          final PlatformView platformView = platformViews.get(viewId);
          final PlatformViewWrapper viewWrapper = viewWrappers.get(viewId);
          if (platformView == null || viewWrapper == null) {
            Log.e(TAG, "Resizing unknown platform view with id: " + viewId);
            return;
          }
          // Resize the buffer only when the current buffer size is smaller than the new size.
          // This is required to prevent a situation when smooth keyboard animation
          // resizes the texture too often, such that the GPU and the platform thread don't agree on
          // the timing of the new size.
          // Resizing the texture causes pixel stretching since the size of the GL texture used in
          // the engine is set by the framework, but the texture buffer size is set by the
          // platform down below.
          if (physicalWidth > viewWrapper.getRenderTargetWidth()
              || physicalHeight > viewWrapper.getRenderTargetHeight()) {
            viewWrapper.resizeRenderTarget(physicalWidth, physicalHeight);
          }

          final ViewGroup.LayoutParams viewWrapperLayoutParams = viewWrapper.getLayoutParams();
          viewWrapperLayoutParams.width = physicalWidth;
          viewWrapperLayoutParams.height = physicalHeight;
          viewWrapper.setLayoutParams(viewWrapperLayoutParams);

          final View embeddedView = platformView.getView();
          if (embeddedView != null) {
            final ViewGroup.LayoutParams embeddedViewLayoutParams = embeddedView.getLayoutParams();
            embeddedViewLayoutParams.width = physicalWidth;
            embeddedViewLayoutParams.height = physicalHeight;
            embeddedView.setLayoutParams(embeddedViewLayoutParams);
          }
          onComplete.run(
              new PlatformViewsChannel.PlatformViewBufferSize(
                  toLogicalPixels(viewWrapper.getRenderTargetWidth()),
                  toLogicalPixels(viewWrapper.getRenderTargetHeight())));
        }

        @Override
        public void onTouch(@NonNull PlatformViewsChannel.PlatformViewTouch touch) {
          final int viewId = touch.viewId;
          final float density = context.getResources().getDisplayMetrics().density;

          if (usesVirtualDisplay(viewId)) {
            final VirtualDisplayController vdController = vdControllers.get(viewId);
            final MotionEvent event = toMotionEvent(density, touch, true);
            vdController.dispatchTouchEvent(event);
            return;
          }

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
          final MotionEvent event = toMotionEvent(density, touch, false);
          view.dispatchTouchEvent(event);
        }

        @Override
        public void setDirection(int viewId, int direction) {
          if (!validateDirection(direction)) {
            throw new IllegalStateException(
                "Trying to set unknown direction value: "
                    + direction
                    + "(view id: "
                    + viewId
                    + ")");
          }

          View embeddedView;

          if (usesVirtualDisplay(viewId)) {
            final VirtualDisplayController controller = vdControllers.get(viewId);
            embeddedView = controller.getView();
          } else {
            final PlatformView platformView = platformViews.get(viewId);
            if (platformView == null) {
              Log.e(TAG, "Setting direction to an unknown view with id: " + viewId);
              return;
            }
            embeddedView = platformView.getView();
          }
          if (embeddedView == null) {
            Log.e(TAG, "Setting direction to a null view with id: " + viewId);
            return;
          }
          embeddedView.setLayoutDirection(direction);
        }

        @Override
        public void clearFocus(int viewId) {
          View embeddedView;

          if (usesVirtualDisplay(viewId)) {
            final VirtualDisplayController controller = vdControllers.get(viewId);
            embeddedView = controller.getView();
          } else {
            final PlatformView platformView = platformViews.get(viewId);
            if (platformView == null) {
              Log.e(TAG, "Clearing focus on an unknown view with id: " + viewId);
              return;
            }
            embeddedView = platformView.getView();
          }
          if (embeddedView == null) {
            Log.e(TAG, "Clearing focus on a null view with id: " + viewId);
            return;
          }
          embeddedView.clearFocus();
        }

        @Override
        public void synchronizeToNativeViewHierarchy(boolean yes) {
          synchronizeToNativeViewHierarchy = yes;
        }
      };

  /// Throws an exception if the SDK version is below minSdkVersion.
  private void enforceMinimumAndroidApiVersion(int minSdkVersion) {
    if (Build.VERSION.SDK_INT < minSdkVersion) {
      throw new IllegalStateException(
          "Trying to use platform views with API "
              + Build.VERSION.SDK_INT
              + ", required API level is: "
              + minSdkVersion);
    }
  }

  private void ensureValidRequest(
      @NonNull PlatformViewsChannel.PlatformViewCreationRequest request) {
    if (!validateDirection(request.direction)) {
      throw new IllegalStateException(
          "Trying to create a view with unknown direction value: "
              + request.direction
              + "(view id: "
              + request.viewId
              + ")");
    }
  }

  // Creates a platform view based on `request`, performs configuration that's common to
  // all display modes, and adds it to `platformViews`.
  @VisibleForTesting(otherwise = VisibleForTesting.PACKAGE_PRIVATE)
  public PlatformView createPlatformView(
      @NonNull PlatformViewsChannel.PlatformViewCreationRequest request, boolean wrapContext) {
    final PlatformViewFactory viewFactory = registry.getFactory(request.viewType);
    if (viewFactory == null) {
      throw new IllegalStateException(
          "Trying to create a platform view of unregistered type: " + request.viewType);
    }

    Object createParams = null;
    if (request.params != null) {
      createParams = viewFactory.getCreateArgsCodec().decodeMessage(request.params);
    }

    // In some display modes, the context needs to be modified during display.
    // TODO(stuartmorgan): Make this wrapping unconditional if possible; for context see
    // https://github.com/flutter/flutter/issues/113449
    final Context mutableContext = wrapContext ? new MutableContextWrapper(context) : context;
    final PlatformView platformView =
        viewFactory.create(mutableContext, request.viewId, createParams);

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

  // Configures the view for Hybrid Composition mode.
  private void configureForHybridComposition(
      @NonNull PlatformView platformView,
      @NonNull PlatformViewsChannel.PlatformViewCreationRequest request) {
    enforceMinimumAndroidApiVersion(19);
    Log.i(TAG, "Using hybrid composition for platform view: " + request.viewId);
  }

  // Configures the view for Virtual Display mode, returning the associated texture ID.
  private long configureForVirtualDisplay(
      @NonNull PlatformView platformView,
      @NonNull PlatformViewsChannel.PlatformViewCreationRequest request) {
    // This mode adds the view to a virtual display, which is wired up to a GL texture that
    // is composed by the Flutter engine.

    // API level 20 is required to use VirtualDisplay#setSurface.
    enforceMinimumAndroidApiVersion(20);

    Log.i(TAG, "Hosting view in a virtual display for platform view: " + request.viewId);

    final PlatformViewRenderTarget renderTarget = makePlatformViewRenderTarget(textureRegistry);
    final int physicalWidth = toPhysicalPixels(request.logicalWidth);
    final int physicalHeight = toPhysicalPixels(request.logicalHeight);
    final VirtualDisplayController vdController =
        VirtualDisplayController.create(
            context,
            accessibilityEventsDelegate,
            platformView,
            renderTarget,
            physicalWidth,
            physicalHeight,
            request.viewId,
            null,
            (view, hasFocus) -> {
              if (hasFocus) {
                platformViewsChannel.invokeViewFocused(request.viewId);
              }
            });

    if (vdController == null) {
      throw new IllegalStateException(
          "Failed creating virtual display for a "
              + request.viewType
              + " with id: "
              + request.viewId);
    }

    // The embedded view doesn't need to be sized in Virtual Display mode because the
    // virtual display itself is sized.

    vdControllers.put(request.viewId, vdController);
    final View embeddedView = platformView.getView();
    contextToEmbeddedView.put(embeddedView.getContext(), embeddedView);

    return renderTarget.getId();
  }

  // Configures the view for Texture Layer Hybrid Composition mode, returning the associated
  // texture ID.
  @TargetApi(API_LEVELS.API_23)
  @VisibleForTesting(otherwise = VisibleForTesting.PACKAGE_PRIVATE)
  public long configureForTextureLayerComposition(
      @NonNull PlatformView platformView,
      @NonNull PlatformViewsChannel.PlatformViewCreationRequest request) {
    // This mode attaches the view to the Android view hierarchy and record its drawing
    // operations, so they can be forwarded to a GL texture that is composed by the
    // Flutter engine.

    // API level 23 is required to use Surface#lockHardwareCanvas().
    enforceMinimumAndroidApiVersion(23);
    Log.i(TAG, "Hosting view in view hierarchy for platform view: " + request.viewId);

    final int physicalWidth = toPhysicalPixels(request.logicalWidth);
    final int physicalHeight = toPhysicalPixels(request.logicalHeight);
    PlatformViewWrapper viewWrapper;
    long textureId;
    if (usesSoftwareRendering) {
      viewWrapper = new PlatformViewWrapper(context);
      textureId = -1;
    } else {
      final PlatformViewRenderTarget renderTarget = makePlatformViewRenderTarget(textureRegistry);
      viewWrapper = new PlatformViewWrapper(context, renderTarget);
      textureId = renderTarget.getId();
    }
    viewWrapper.setTouchProcessor(androidTouchProcessor);
    viewWrapper.resizeRenderTarget(physicalWidth, physicalHeight);

    final FrameLayout.LayoutParams viewWrapperLayoutParams =
        new FrameLayout.LayoutParams(physicalWidth, physicalHeight);

    // Size and position the view wrapper.
    final int physicalTop = toPhysicalPixels(request.logicalTop);
    final int physicalLeft = toPhysicalPixels(request.logicalLeft);
    viewWrapperLayoutParams.topMargin = physicalTop;
    viewWrapperLayoutParams.leftMargin = physicalLeft;
    viewWrapper.setLayoutParams(viewWrapperLayoutParams);

    // Size the embedded view.
    final View embeddedView = platformView.getView();
    embeddedView.setLayoutParams(new FrameLayout.LayoutParams(physicalWidth, physicalHeight));

    // Accessibility in the embedded view is initially disabled because if a Flutter app
    // disabled accessibility in the first frame, the embedding won't receive an update to
    // disable accessibility since the embedding never received an update to enable it.
    // The AccessibilityBridge keeps track of the accessibility nodes, and handles the deltas
    // when the framework sends a new a11y tree to the embedding.
    // To prevent races, the framework populate the SemanticsNode after the platform view has
    // been created.
    embeddedView.setImportantForAccessibility(View.IMPORTANT_FOR_ACCESSIBILITY_NO_HIDE_DESCENDANTS);

    // Add the embedded view to the wrapper.
    viewWrapper.addView(embeddedView);

    // Listen for focus changed in any subview, so the framework is notified when the platform
    // view is focused.
    viewWrapper.setOnDescendantFocusChangeListener(
        (v, hasFocus) -> {
          if (hasFocus) {
            platformViewsChannel.invokeViewFocused(request.viewId);
          } else if (textInputPlugin != null) {
            textInputPlugin.clearPlatformViewClient(request.viewId);
          }
        });

    flutterView.addView(viewWrapper);
    viewWrappers.append(request.viewId, viewWrapper);

    maybeInvokeOnFlutterViewAttached(platformView);

    return textureId;
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
  public MotionEvent toMotionEvent(
      float density, PlatformViewsChannel.PlatformViewTouch touch, boolean usingVirtualDiplay) {
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

    if (!usingVirtualDiplay && trackedEvent != null) {
      // We have the original event, deliver it after offsetting as it will pass the verifiable
      // input check.
      translateMotionEvent(trackedEvent, pointerCoords);
      return trackedEvent;
    }
    // We are in virtual display mode or don't have a reference to the original MotionEvent.
    // In this case we manually recreate a MotionEvent to be delivered. This MotionEvent
    // will fail the verifiable input check.
    PointerProperties[] pointerProperties =
        parsePointerPropertiesList(touch.rawPointerPropertiesList)
            .toArray(new PointerProperties[touch.pointerCount]);

    // TODO (kaushikiska) : warn that we are potentially using an untracked
    // event in the platform views.
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

  public PlatformViewsController() {
    registry = new PlatformViewRegistryImpl();
    vdControllers = new HashMap<>();
    accessibilityEventsDelegate = new AccessibilityEventsDelegate();
    contextToEmbeddedView = new HashMap<>();
    overlayLayerViews = new SparseArray<>();
    currentFrameUsedOverlayLayerIds = new HashSet<>();
    currentFrameUsedPlatformViewIds = new HashSet<>();
    viewWrappers = new SparseArray<>();
    platformViews = new SparseArray<>();
    platformViewParent = new SparseArray<>();

    motionEventTracker = MotionEventTracker.getInstance();
  }

  /**
   * Attaches this platform views controller to its input and output channels.
   *
   * @param context The base context that will be passed to embedded views created by this
   *     controller. This should be the context of the Activity hosting the Flutter application.
   * @param textureRegistry The texture registry which provides the output textures into which the
   *     embedded views will be rendered.
   * @param dartExecutor The dart execution context, which is used to set up a system channel.
   */
  public void attach(
      @Nullable Context context,
      @NonNull TextureRegistry textureRegistry,
      @NonNull DartExecutor dartExecutor) {
    if (this.context != null) {
      throw new AssertionError(
          "A PlatformViewsController can only be attached to a single output target.\n"
              + "attach was called while the PlatformViewsController was already attached.");
    }
    this.context = context;
    this.textureRegistry = textureRegistry;
    platformViewsChannel = new PlatformViewsChannel(dartExecutor);
    platformViewsChannel.setPlatformViewsHandler(channelHandler);
  }

  /**
   * Sets whether Flutter uses software rendering.
   *
   * <p>When software rendering is used, no GL context is available on the raster thread. When this
   * is set to true, there's no Flutter composition of Android views and Flutter widgets since GL
   * textures cannot be used.
   *
   * <p>Software rendering is only used for testing in emulators, and it should never be set to true
   * in a production environment.
   *
   * @param useSoftwareRendering Whether software rendering is used.
   */
  public void setSoftwareRendering(boolean useSoftwareRendering) {
    usesSoftwareRendering = useSoftwareRendering;
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
    destroyOverlaySurfaces();
    platformViewsChannel = null;
    context = null;
    textureRegistry = null;
  }

  /**
   * Attaches the controller to a {@link FlutterView}.
   *
   * <p>When {@link io.flutter.embedding.android.FlutterFragment} is used, this method is called
   * after the device rotates since the FlutterView is recreated after a rotation.
   */
  public void attachToView(@NonNull FlutterView newFlutterView) {
    flutterView = newFlutterView;
    // Add wrapper for platform views that use GL texture.
    for (int index = 0; index < viewWrappers.size(); index++) {
      final PlatformViewWrapper view = viewWrappers.valueAt(index);
      flutterView.addView(view);
    }
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
    // Remove wrapper for platform views that use GL texture.
    for (int index = 0; index < viewWrappers.size(); index++) {
      final PlatformViewWrapper view = viewWrappers.valueAt(index);
      flutterView.removeView(view);
    }
    // Remove wrapper for platform views that are composed at the view hierarchy level.
    for (int index = 0; index < platformViewParent.size(); index++) {
      final FlutterMutatorView view = platformViewParent.valueAt(index);
      flutterView.removeView(view);
    }

    destroyOverlaySurfaces();
    removeOverlaySurfaces();
    flutterView = null;
    flutterViewConvertedToImageView = false;

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

  /**
   * Returns true if Flutter should perform input connection proxying for the view.
   *
   * <p>If the view is a platform view managed by this platform views controller returns true. Else
   * if the view was created in a platform view's VD, delegates the decision to the platform view's
   * {@link View#checkInputConnectionProxy(View)} method. Else returns false.
   */
  public boolean checkInputConnectionProxy(@Nullable View view) {
    // View can be null on some devices
    // See: https://github.com/flutter/flutter/issues/36517
    if (view == null) {
      return false;
    }
    if (!contextToEmbeddedView.containsKey(view.getContext())) {
      return false;
    }
    View platformView = contextToEmbeddedView.get(view.getContext());
    if (platformView == view) {
      return true;
    }
    return platformView.checkInputConnectionProxy(view);
  }

  public PlatformViewRegistry getRegistry() {
    return registry;
  }

  /**
   * Invoked when the {@link io.flutter.embedding.engine.FlutterEngine} that owns this {@link
   * PlatformViewsController} attaches to JNI.
   */
  public void onAttachedToJNI() {
    // Currently no action needs to be taken after JNI attachment.
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
    if (usesVirtualDisplay(viewId)) {
      final VirtualDisplayController controller = vdControllers.get(viewId);
      return controller.getView();
    }

    final PlatformView platformView = platformViews.get(viewId);
    if (platformView == null) {
      return null;
    }
    return platformView.getView();
  }

  @Override
  public boolean usesVirtualDisplay(int id) {
    return vdControllers.containsKey(id);
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

  private static PlatformViewRenderTarget makePlatformViewRenderTarget(
      TextureRegistry textureRegistry) {
    if (enableSurfaceProducerRenderTarget && Build.VERSION.SDK_INT >= API_LEVELS.API_29) {
      final TextureRegistry.SurfaceProducer textureEntry = textureRegistry.createSurfaceProducer();
      Log.i(TAG, "PlatformView is using SurfaceProducer backend");
      return new SurfaceProducerPlatformViewRenderTarget(textureEntry);
    }
    if (enableImageRenderTarget && Build.VERSION.SDK_INT >= API_LEVELS.API_29) {
      final TextureRegistry.ImageTextureEntry textureEntry = textureRegistry.createImageTexture();
      Log.i(TAG, "PlatformView is using ImageReader backend");
      return new ImageReaderPlatformViewRenderTarget(textureEntry);
    }
    final TextureRegistry.SurfaceTextureEntry textureEntry = textureRegistry.createSurfaceTexture();
    Log.i(TAG, "PlatformView is using SurfaceTexture backend");
    return new SurfaceTexturePlatformViewRenderTarget(textureEntry);
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

  // Invoked when the Android system is requesting we reduce memory usage.
  public void onTrimMemory(int level) {
    if (level < ComponentCallbacks2.TRIM_MEMORY_BACKGROUND) {
      return;
    }
    for (VirtualDisplayController vdc : vdControllers.values()) {
      vdc.clearSurface();
    }
  }

  // Called after the application has been resumed.
  // This is where we undo whatever may have been done in onTrimMemory.
  public void onResume() {
    for (VirtualDisplayController vdc : vdControllers.values()) {
      vdc.resetSurface();
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

  private void initializeRootImageViewIfNeeded() {
    if (synchronizeToNativeViewHierarchy && !flutterViewConvertedToImageView) {
      flutterView.convertToImageView();
      flutterViewConvertedToImageView = true;
    }
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
    initializeRootImageViewIfNeeded();
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
    currentFrameUsedPlatformViewIds.add(viewId);
  }

  /**
   * Called when an overlay surface is displayed in the current frame.
   *
   * @param id The ID of the surface.
   * @param x The left position relative to {@code FlutterView}.
   * @param y The top position relative to {@code FlutterView}.
   * @param width The width of the surface.
   * @param height The height of the surface. This member is not intended for public use, and is
   *     only visible for testing.
   */
  public void onDisplayOverlaySurface(int id, int x, int y, int width, int height) {
    if (overlayLayerViews.get(id) == null) {
      throw new IllegalStateException("The overlay surface (id:" + id + ") doesn't exist");
    }
    initializeRootImageViewIfNeeded();

    final PlatformOverlayView overlayView = overlayLayerViews.get(id);
    if (overlayView.getParent() == null) {
      flutterView.addView(overlayView);
    }

    FrameLayout.LayoutParams layoutParams = new FrameLayout.LayoutParams((int) width, (int) height);
    layoutParams.leftMargin = (int) x;
    layoutParams.topMargin = (int) y;
    overlayView.setLayoutParams(layoutParams);
    overlayView.setVisibility(View.VISIBLE);
    overlayView.bringToFront();
    currentFrameUsedOverlayLayerIds.add(id);
  }

  public void onBeginFrame() {
    currentFrameUsedOverlayLayerIds.clear();
    currentFrameUsedPlatformViewIds.clear();
  }

  /**
   * Called by {@code FlutterJNI} when the Flutter frame was submitted.
   *
   * <p>This member is not intended for public use, and is only visible for testing.
   */
  public void onEndFrame() {
    // If there are no platform views in the current frame,
    // then revert the image view surface and use the previous surface.
    //
    // Otherwise, acquire the latest image.
    if (flutterViewConvertedToImageView && currentFrameUsedPlatformViewIds.isEmpty()) {
      flutterViewConvertedToImageView = false;
      flutterView.revertImageView(
          () -> {
            // Destroy overlay surfaces once the surface reversion is completed.
            finishFrame(false);
          });
      return;
    }
    // Whether the current frame was rendered using ImageReaders.
    //
    // Since the image readers may not have images available at this point,
    // this becomes true if all the required surfaces have images available.
    //
    // This is used to decide if the platform views can be rendered in the current frame.
    // If one of the surfaces doesn't have an image, the frame may be incomplete and must be
    // dropped.
    // For example, a toolbar widget painted by Flutter may not be rendered.
    final boolean isFrameRenderedUsingImageReaders =
        flutterViewConvertedToImageView && flutterView.acquireLatestImageViewFrame();
    finishFrame(isFrameRenderedUsingImageReaders);
  }

  private void finishFrame(boolean isFrameRenderedUsingImageReaders) {
    for (int i = 0; i < overlayLayerViews.size(); i++) {
      final int overlayId = overlayLayerViews.keyAt(i);
      final PlatformOverlayView overlayView = overlayLayerViews.valueAt(i);

      if (currentFrameUsedOverlayLayerIds.contains(overlayId)) {
        flutterView.attachOverlaySurfaceToRender(overlayView);
        final boolean didAcquireOverlaySurfaceImage = overlayView.acquireLatestImage();
        isFrameRenderedUsingImageReaders &= didAcquireOverlaySurfaceImage;
      } else {
        // If the background surface isn't rendered by the image view, then the
        // overlay surfaces can be detached from the rendered.
        // This releases resources used by the ImageReader.
        if (!flutterViewConvertedToImageView) {
          overlayView.detachFromRenderer();
        }
        // Hide overlay surfaces that aren't rendered in the current frame.
        overlayView.setVisibility(View.GONE);
        flutterView.removeView(overlayView);
      }
    }

    for (int i = 0; i < platformViewParent.size(); i++) {
      final int viewId = platformViewParent.keyAt(i);
      final View parentView = platformViewParent.get(viewId);

      // This should only show platform views that are rendered in this frame and either:
      //  1. Surface has images available in this frame or,
      //  2. Surface does not have images available in this frame because the render surface should
      // not be an ImageView.
      //
      // The platform view is appended to a mutator view.
      //
      // Otherwise, hide the platform view, but don't remove it from the view hierarchy yet as
      // they are removed when the framework disposes the platform view widget.
      if (currentFrameUsedPlatformViewIds.contains(viewId)
          && (isFrameRenderedUsingImageReaders || !synchronizeToNativeViewHierarchy)) {
        parentView.setVisibility(View.VISIBLE);
      } else {
        parentView.setVisibility(View.GONE);
      }
    }
  }

  /**
   * Creates and tracks the overlay surface.
   *
   * @param imageView The surface that displays the overlay.
   * @return Wrapper object that provides the layer id and the surface. This member is not intended
   *     for public use, and is only visible for testing.
   */
  @VisibleForTesting
  @NonNull
  public FlutterOverlaySurface createOverlaySurface(@NonNull PlatformOverlayView imageView) {
    final int id = nextOverlayLayerId++;
    overlayLayerViews.put(id, imageView);
    return new FlutterOverlaySurface(id, imageView.getSurface());
  }

  /**
   * Creates an overlay surface while the Flutter view is rendered by {@code PlatformOverlayView}.
   *
   * <p>This method is invoked by {@code FlutterJNI} only.
   *
   * <p>This member is not intended for public use, and is only visible for testing.
   */
  @NonNull
  public FlutterOverlaySurface createOverlaySurface() {
    // Overlay surfaces have the same size as the background surface.
    //
    // This allows to reuse these surfaces in consecutive frames even
    // if the drawings they contain have a different tight bound.
    //
    // The final view size is determined when its frame is set.
    return createOverlaySurface(
        new PlatformOverlayView(
            flutterView.getContext(),
            flutterView.getWidth(),
            flutterView.getHeight(),
            accessibilityEventsDelegate));
  }

  /**
   * Destroys the overlay surfaces and removes them from the view hierarchy.
   *
   * <p>This method is used only internally by {@code FlutterJNI}.
   */
  public void destroyOverlaySurfaces() {
    for (int viewId = 0; viewId < overlayLayerViews.size(); viewId++) {
      final PlatformOverlayView overlayView = overlayLayerViews.valueAt(viewId);
      overlayView.detachFromRenderer();
      overlayView.closeImageReader();
      // Don't remove overlayView from the view hierarchy since this method can
      // be called while the Android framework is iterating over the array of views.
      // See ViewGroup#dispatchDetachedFromWindow(), and
      // https://github.com/flutter/flutter/issues/97679.
    }
  }

  private void removeOverlaySurfaces() {
    if (flutterView == null) {
      Log.e(TAG, "removeOverlaySurfaces called while flutter view is null");
      return;
    }
    for (int viewId = 0; viewId < overlayLayerViews.size(); viewId++) {
      flutterView.removeView(overlayLayerViews.valueAt(viewId));
    }
    overlayLayerViews.clear();
  }

  @VisibleForTesting
  public SparseArray<PlatformOverlayView> getOverlayLayerViews() {
    return overlayLayerViews;
  }
}
