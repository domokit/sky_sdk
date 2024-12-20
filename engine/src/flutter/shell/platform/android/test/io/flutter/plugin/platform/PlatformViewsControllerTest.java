// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.platform;

import static android.os.Looper.getMainLooper;
import static io.flutter.embedding.engine.systemchannels.PlatformViewsChannel.PlatformViewTouch;
import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;
import static org.robolectric.Shadows.shadowOf;

import android.app.Presentation;
import android.content.Context;
import android.content.MutableContextWrapper;
import android.content.res.AssetManager;
import android.graphics.SurfaceTexture;
import android.media.Image;
import android.util.SparseArray;
import android.view.MotionEvent;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.ViewParent;
import android.widget.FrameLayout;
import android.widget.FrameLayout.LayoutParams;
import androidx.annotation.NonNull;
import androidx.test.core.app.ApplicationProvider;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.embedding.android.FlutterImageView;
import io.flutter.embedding.android.FlutterSurfaceView;
import io.flutter.embedding.android.FlutterView;
import io.flutter.embedding.android.MotionEventTracker;
import io.flutter.embedding.engine.FlutterEngine;
import io.flutter.embedding.engine.FlutterJNI;
import io.flutter.embedding.engine.FlutterOverlaySurface;
import io.flutter.embedding.engine.dart.DartExecutor;
import io.flutter.embedding.engine.mutatorsstack.FlutterMutatorView;
import io.flutter.embedding.engine.mutatorsstack.FlutterMutatorsStack;
import io.flutter.embedding.engine.renderer.FlutterRenderer;
import io.flutter.embedding.engine.systemchannels.AccessibilityChannel;
import io.flutter.embedding.engine.systemchannels.MouseCursorChannel;
import io.flutter.embedding.engine.systemchannels.PlatformViewsChannel;
import io.flutter.embedding.engine.systemchannels.PlatformViewsChannel.PlatformViewTouch;
import io.flutter.embedding.engine.systemchannels.ScribeChannel;
import io.flutter.embedding.engine.systemchannels.SettingsChannel;
import io.flutter.embedding.engine.systemchannels.TextInputChannel;
import io.flutter.plugin.common.MethodCall;
import io.flutter.plugin.common.StandardMessageCodec;
import io.flutter.plugin.common.StandardMethodCodec;
import io.flutter.plugin.localization.LocalizationPlugin;
import io.flutter.view.TextureRegistry;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.junit.Ignore;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.ArgumentCaptor;
import org.robolectric.annotation.Config;
import org.robolectric.annotation.Implementation;
import org.robolectric.annotation.Implements;
import org.robolectric.shadows.ShadowDialog;
import org.robolectric.shadows.ShadowSurfaceView;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
public class PlatformViewsControllerTest {
  // An implementation of PlatformView that counts invocations of its lifecycle callbacks.
  class CountingPlatformView implements PlatformView {
    static final String VIEW_TYPE_ID = "CountingPlatformView";
    private View view;

    public CountingPlatformView(Context context) {
      view = new SurfaceView(context);
    }

    public int disposeCalls = 0;
    public int attachCalls = 0;
    public int detachCalls = 0;

    @Override
    public void dispose() {
      // We have been removed from the view hierarhy before the call to dispose.
      assertNull(view.getParent());
      disposeCalls++;
    }

    @Override
    public View getView() {
      return view;
    }

    @Override
    public void onFlutterViewAttached(View flutterView) {
      attachCalls++;
    }

    @Override
    public void onFlutterViewDetached() {
      detachCalls++;
    }
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void itRemovesPlatformViewBeforeDiposeIsCalled() {
    PlatformViewsController platformViewsController = new PlatformViewsController();
    FlutterJNI jni = new FlutterJNI();
    attach(jni, platformViewsController);
    // Get the platform view registry.
    PlatformViewRegistry registry = platformViewsController.getRegistry();

    // Register a factory for our platform view.
    registry.registerViewFactory(
        CountingPlatformView.VIEW_TYPE_ID,
        new PlatformViewFactory(StandardMessageCodec.INSTANCE) {
          @Override
          public PlatformView create(Context context, int viewId, Object args) {
            return new CountingPlatformView(context);
          }
        });

    // Create the platform view.
    int viewId = 0;
    final PlatformViewsChannel.PlatformViewCreationRequest request =
        new PlatformViewsChannel.PlatformViewCreationRequest(
            viewId,
            CountingPlatformView.VIEW_TYPE_ID,
            0,
            0,
            128,
            128,
            View.LAYOUT_DIRECTION_LTR,
            null);
    PlatformView pView = platformViewsController.createPlatformView(request, true);
    assertTrue(pView instanceof CountingPlatformView);
    CountingPlatformView cpv = (CountingPlatformView) pView;
    platformViewsController.configureForTextureLayerComposition(pView, request);
    assertEquals(0, cpv.disposeCalls);
    platformViewsController.disposePlatformView(viewId);
    assertEquals(1, cpv.disposeCalls);
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void itNotifiesPlatformViewsOfEngineAttachmentAndDetachment() {
    PlatformViewsController platformViewsController = new PlatformViewsController();
    FlutterJNI jni = new FlutterJNI();
    attach(jni, platformViewsController);
    // Get the platform view registry.
    PlatformViewRegistry registry = platformViewsController.getRegistry();

    // Register a factory for our platform view.
    registry.registerViewFactory(
        CountingPlatformView.VIEW_TYPE_ID,
        new PlatformViewFactory(StandardMessageCodec.INSTANCE) {
          @Override
          public PlatformView create(Context context, int viewId, Object args) {
            return new CountingPlatformView(context);
          }
        });

    // Create the platform view.
    int viewId = 0;
    final PlatformViewsChannel.PlatformViewCreationRequest request =
        new PlatformViewsChannel.PlatformViewCreationRequest(
            viewId,
            CountingPlatformView.VIEW_TYPE_ID,
            0,
            0,
            128,
            128,
            View.LAYOUT_DIRECTION_LTR,
            null);
    PlatformView pView = platformViewsController.createPlatformView(request, true);
    assertTrue(pView instanceof CountingPlatformView);
    CountingPlatformView cpv = (CountingPlatformView) pView;
    assertEquals(1, cpv.attachCalls);
    assertEquals(0, cpv.detachCalls);
    assertEquals(0, cpv.disposeCalls);
    platformViewsController.detachFromView();
    assertEquals(1, cpv.attachCalls);
    assertEquals(1, cpv.detachCalls);
    assertEquals(0, cpv.disposeCalls);
    platformViewsController.disposePlatformView(viewId);
  }

  @Ignore
  @Test
  public void itCancelsOldPresentationOnResize() {
    // Setup test structure.
    // Create a fake View that represents the View that renders a Flutter UI.
    View fakeFlutterView = new View(ApplicationProvider.getApplicationContext());

    // Create fake VirtualDisplayControllers. This requires internal knowledge of
    // PlatformViewsController. We know that all PlatformViewsController does is
    // forward view attachment/detachment calls to it's VirtualDisplayControllers.
    //
    // TODO(mattcarroll): once PlatformViewsController is refactored into testable
    // pieces, remove this test and avoid verifying private behavior.
    VirtualDisplayController fakeVdController1 = mock(VirtualDisplayController.class);

    SingleViewPresentation presentation = fakeVdController1.presentation;

    fakeVdController1.resize(10, 10, null);

    assertEquals(fakeVdController1.presentation != presentation, true);
    assertEquals(presentation.isShowing(), false);
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void virtualDisplay_handlesResizeResponseWithoutContext() {
    final int platformViewId = 0;
    FlutterView fakeFlutterView = new FlutterView(ApplicationProvider.getApplicationContext());
    VirtualDisplayController fakeVdController = mock(VirtualDisplayController.class);
    PlatformViewsController platformViewsController = new PlatformViewsController();
    platformViewsController.vdControllers.put(platformViewId, fakeVdController);

    platformViewsController.attachToView(fakeFlutterView);

    FlutterJNI jni = new FlutterJNI();
    attach(jni, platformViewsController);

    resize(jni, platformViewsController, platformViewId, 10.0, 20.0);

    ArgumentCaptor<Runnable> resizeCallbackCaptor = ArgumentCaptor.forClass(Runnable.class);
    verify(fakeVdController, times(1)).resize(anyInt(), anyInt(), resizeCallbackCaptor.capture());

    // Simulate a detach call before the resize completes.
    platformViewsController.detach();

    // Trigger the callback to ensure that it doesn't crash.
    resizeCallbackCaptor.getValue().run();
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void virtualDisplay_ensureDisposeVirtualDisplayController() {
    final int platformViewId = 0;
    FlutterView fakeFlutterView = new FlutterView(ApplicationProvider.getApplicationContext());
    VirtualDisplayController fakeVdController = mock(VirtualDisplayController.class);
    PlatformViewsController platformViewsController = new PlatformViewsController();
    FlutterJNI jni = new FlutterJNI();
    attach(jni, platformViewsController);
    platformViewsController.attachToView(fakeFlutterView);

    PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    PlatformView platformView = mock(PlatformView.class);
    Context context = ApplicationProvider.getApplicationContext();
    View androidView = new View(context);
    when(platformView.getView()).thenReturn(androidView);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);
    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);
    createPlatformView(
        jni, platformViewsController, platformViewId, "testType", /* hybrid=*/ false);

    platformViewsController.vdControllers.put(platformViewId, fakeVdController);

    // Simulate dispose call from the framework.
    disposePlatformView(jni, platformViewsController, platformViewId);

    // Ensure VirtualDisplayController.dispose() is called
    verify(fakeVdController, times(1)).dispose();
  }

  @Test
  public void itUsesActionEventTypeFromFrameworkEventForVirtualDisplays() {
    MotionEventTracker motionEventTracker = MotionEventTracker.getInstance();
    PlatformViewsController platformViewsController = new PlatformViewsController();

    MotionEvent original =
        MotionEvent.obtain(
            100, // downTime
            100, // eventTime
            1, // action
            0, // x
            0, // y
            0 // metaState
            );

    // track an event that will later get passed to us from framework
    MotionEventTracker.MotionEventId motionEventId = motionEventTracker.track(original);

    PlatformViewTouch frameWorkTouch =
        new PlatformViewTouch(
            0, // viewId
            original.getDownTime(),
            original.getEventTime(),
            2, // action
            1, // pointerCount
            Arrays.asList(Arrays.asList(0, 0)), // pointer properties
            Arrays.asList(Arrays.asList(0., 1., 2., 3., 4., 5., 6., 7., 8.)), // pointer coords
            original.getMetaState(),
            original.getButtonState(),
            original.getXPrecision(),
            original.getYPrecision(),
            original.getDeviceId(),
            original.getEdgeFlags(),
            original.getSource(),
            original.getFlags(),
            motionEventId.getId());

    MotionEvent resolvedEvent =
        platformViewsController.toMotionEvent(
            1, // density
            frameWorkTouch,
            true // usingVirtualDisplays
            );

    assertEquals(resolvedEvent.getAction(), frameWorkTouch.action);
    assertNotEquals(resolvedEvent.getAction(), original.getAction());
  }

  @Test
  public void itUsesActionEventTypeFromFrameworkEventAsActionChanged() {
    MotionEventTracker motionEventTracker = MotionEventTracker.getInstance();
    PlatformViewsController platformViewsController = new PlatformViewsController();

    MotionEvent original =
        MotionEvent.obtain(
            10, // downTime
            10, // eventTime
            261, // action
            0, // x
            0, // y
            0 // metaState
            );

    MotionEventTracker.MotionEventId motionEventId = motionEventTracker.track(original);

    PlatformViewTouch frameWorkTouch =
        new PlatformViewTouch(
            0, // viewId
            original.getDownTime(),
            original.getEventTime(),
            0, // action
            1, // pointerCount
            Arrays.asList(Arrays.asList(0, 0)), // pointer properties
            Arrays.asList(Arrays.asList(0., 1., 2., 3., 4., 5., 6., 7., 8.)), // pointer coords
            original.getMetaState(),
            original.getButtonState(),
            original.getXPrecision(),
            original.getYPrecision(),
            original.getDeviceId(),
            original.getEdgeFlags(),
            original.getSource(),
            original.getFlags(),
            motionEventId.getId());
    MotionEvent resolvedEvent =
        platformViewsController.toMotionEvent(
            1, // density
            frameWorkTouch,
            false // usingVirtualDisplays
            );
    assertEquals(resolvedEvent.getAction(), original.getAction());
    assertNotEquals(resolvedEvent.getAction(), frameWorkTouch.action);
  }

  private MotionEvent makePlatformViewTouchAndInvokeToMotionEvent(
      PlatformViewsController platformViewsController,
      MotionEventTracker motionEventTracker,
      MotionEvent original,
      boolean usingVirtualDisplays) {
    MotionEventTracker.MotionEventId motionEventId = motionEventTracker.track(original);

    // Construct a PlatformViewTouch.rawPointerPropertiesList by doing the inverse of
    // PlatformViewsController.parsePointerPropertiesList.
    List<List<Integer>> pointerProperties =
        Arrays.asList(Arrays.asList(original.getPointerId(0), original.getToolType(0)));
    // Construct a PlatformViewTouch.rawPointerCoords by doing the inverse of
    // PlatformViewsController.parsePointerCoordsList.
    List<List<Double>> pointerCoordinates =
        Arrays.asList(
            Arrays.asList(
                (double) original.getOrientation(),
                (double) original.getPressure(),
                (double) original.getSize(),
                (double) original.getToolMajor(),
                (double) original.getToolMinor(),
                (double) original.getTouchMajor(),
                (double) original.getTouchMinor(),
                (double) original.getX(),
                (double) original.getY()));
    // Make a platform view touch from the motion event.
    PlatformViewTouch frameWorkTouchNonVd =
        new PlatformViewTouch(
            0, // viewId
            original.getDownTime(),
            original.getEventTime(),
            original.getAction(),
            1, // pointerCount
            pointerProperties, // pointer properties
            pointerCoordinates, // pointer coords
            original.getMetaState(),
            original.getButtonState(),
            original.getXPrecision(),
            original.getYPrecision(),
            original.getDeviceId(),
            original.getEdgeFlags(),
            original.getSource(),
            original.getFlags(),
            motionEventId.getId());

    return platformViewsController.toMotionEvent(
        1, // density
        frameWorkTouchNonVd,
        usingVirtualDisplays);
  }

  @Test
  public void toMotionEvent_returnsSameCoordsForVdAndNonVd() {
    MotionEventTracker motionEventTracker = MotionEventTracker.getInstance();
    PlatformViewsController platformViewsController = new PlatformViewsController();

    MotionEvent original =
        MotionEvent.obtain(
            10, // downTime
            10, // eventTime
            261, // action
            1, // x
            1, // y
            0 // metaState
            );

    MotionEvent resolvedNonVdEvent =
        makePlatformViewTouchAndInvokeToMotionEvent(
            platformViewsController, motionEventTracker, original, false);

    MotionEvent resolvedVdEvent =
        makePlatformViewTouchAndInvokeToMotionEvent(
            platformViewsController, motionEventTracker, original, true);

    assertEquals(resolvedVdEvent.getEventTime(), resolvedNonVdEvent.getEventTime());
    assertEquals(resolvedVdEvent.getX(), resolvedNonVdEvent.getX(), 0.001f);
    assertEquals(resolvedVdEvent.getY(), resolvedNonVdEvent.getY(), 0.001f);
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void getPlatformViewById_hybridComposition() {
    PlatformViewsController platformViewsController = new PlatformViewsController();

    int platformViewId = 0;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    PlatformView platformView = mock(PlatformView.class);
    View androidView = mock(View.class);
    when(platformView.getView()).thenReturn(androidView);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);
    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    FlutterJNI jni = new FlutterJNI();
    attach(jni, platformViewsController);

    // Simulate create call from the framework.
    createPlatformView(jni, platformViewsController, platformViewId, "testType", /* hybrid=*/ true);

    assertTrue(platformViewsController.initializePlatformViewIfNeeded(platformViewId));

    View resultAndroidView = platformViewsController.getPlatformViewById(platformViewId);
    assertNotNull(resultAndroidView);
    assertEquals(resultAndroidView, androidView);
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void createPlatformViewMessage_initializesAndroidView() {
    PlatformViewsController platformViewsController = new PlatformViewsController();

    int platformViewId = 0;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    PlatformView platformView = mock(PlatformView.class);
    when(platformView.getView()).thenReturn(mock(View.class));
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);
    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    FlutterJNI jni = new FlutterJNI();
    attach(jni, platformViewsController);

    // Simulate create call from the framework.
    createPlatformView(jni, platformViewsController, platformViewId, "testType", /* hybrid=*/ true);
    verify(viewFactory, times(1)).create(any(), eq(platformViewId), any());
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void createPlatformViewMessage_wrapsContextForVirtualDisplay() {
    PlatformViewsController platformViewsController = new PlatformViewsController();

    int platformViewId = 0;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    PlatformView platformView = mock(PlatformView.class);
    when(platformView.getView()).thenReturn(mock(View.class));
    ArgumentCaptor<Context> passedContext = ArgumentCaptor.forClass(Context.class);
    when(viewFactory.create(passedContext.capture(), eq(platformViewId), any()))
        .thenReturn(platformView);
    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    FlutterJNI jni = new FlutterJNI();
    attach(jni, platformViewsController);

    // Simulate create call from the framework.
    createPlatformView(
        jni, platformViewsController, platformViewId, "testType", /* hybrid=*/ false);
    assertTrue(passedContext.getValue() instanceof MutableContextWrapper);
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void createPlatformViewMessage_doesNotWrapContextForHybrid() {
    PlatformViewsController platformViewsController = new PlatformViewsController();

    int platformViewId = 0;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    PlatformView platformView = mock(PlatformView.class);
    when(platformView.getView()).thenReturn(mock(View.class));
    ArgumentCaptor<Context> passedContext = ArgumentCaptor.forClass(Context.class);
    when(viewFactory.create(passedContext.capture(), eq(platformViewId), any()))
        .thenReturn(platformView);
    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    FlutterJNI jni = new FlutterJNI();
    attach(jni, platformViewsController);

    // Simulate create call from the framework.
    createPlatformView(jni, platformViewsController, platformViewId, "testType", /* hybrid=*/ true);
    assertFalse(passedContext.getValue() instanceof MutableContextWrapper);
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void createPlatformViewMessage_setsAndroidViewLayoutDirection() {
    PlatformViewsController platformViewsController = new PlatformViewsController();
    platformViewsController.setSoftwareRendering(true);

    int platformViewId = 0;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    PlatformView platformView = mock(PlatformView.class);

    View androidView = mock(View.class);
    when(platformView.getView()).thenReturn(androidView);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);
    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    FlutterJNI jni = new FlutterJNI();
    attach(jni, platformViewsController);

    // Simulate create call from the framework.
    createPlatformView(
        jni, platformViewsController, platformViewId, "testType", /* hybrid=*/ false);
    verify(androidView, times(1)).setLayoutDirection(0);
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void createPlatformViewMessage_setsAndroidViewSize() {
    PlatformViewsController platformViewsController = new PlatformViewsController();
    platformViewsController.setSoftwareRendering(true);

    int platformViewId = 0;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    PlatformView platformView = mock(PlatformView.class);

    View androidView = mock(View.class);
    when(platformView.getView()).thenReturn(androidView);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);
    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    FlutterJNI jni = new FlutterJNI();
    attach(jni, platformViewsController);

    // Simulate create call from the framework.
    createPlatformView(
        jni, platformViewsController, platformViewId, "testType", /* hybrid=*/ false);

    ArgumentCaptor<FrameLayout.LayoutParams> layoutParamsCaptor =
        ArgumentCaptor.forClass(FrameLayout.LayoutParams.class);
    verify(androidView, times(2)).setLayoutParams(layoutParamsCaptor.capture());

    List<FrameLayout.LayoutParams> capturedLayoutParams = layoutParamsCaptor.getAllValues();
    assertEquals(capturedLayoutParams.get(0).width, 1);
    assertEquals(capturedLayoutParams.get(0).height, 1);
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void createPlatformViewMessage_disablesAccessibility() {
    PlatformViewsController platformViewsController = new PlatformViewsController();
    platformViewsController.setSoftwareRendering(true);

    int platformViewId = 0;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    PlatformView platformView = mock(PlatformView.class);

    View androidView = mock(View.class);
    when(platformView.getView()).thenReturn(androidView);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);
    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    FlutterJNI jni = new FlutterJNI();
    attach(jni, platformViewsController);

    // Simulate create call from the framework.
    createPlatformView(
        jni, platformViewsController, platformViewId, "testType", /* hybrid=*/ false);
    verify(androidView, times(1))
        .setImportantForAccessibility(View.IMPORTANT_FOR_ACCESSIBILITY_NO_HIDE_DESCENDANTS);
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void createPlatformViewMessage_throwsIfViewIsNull() {
    PlatformViewsController platformViewsController = new PlatformViewsController();

    int platformViewId = 0;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    PlatformView platformView = mock(PlatformView.class);
    when(platformView.getView()).thenReturn(null);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);
    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    FlutterJNI jni = new FlutterJNI();
    attach(jni, platformViewsController);

    // Simulate create call from the framework.
    createPlatformView(
        jni, platformViewsController, platformViewId, "testType", /* hybrid=*/ false);
    assertEquals(ShadowFlutterJNI.getResponses().size(), 1);

    assertFalse(platformViewsController.initializePlatformViewIfNeeded(platformViewId));
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void createHybridPlatformViewMessage_throwsIfViewIsNull() {
    PlatformViewsController platformViewsController = new PlatformViewsController();

    int platformViewId = 0;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    PlatformView platformView = mock(PlatformView.class);
    when(platformView.getView()).thenReturn(null);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);
    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    FlutterJNI jni = new FlutterJNI();
    attach(jni, platformViewsController);

    // Simulate create call from the framework.
    createPlatformView(jni, platformViewsController, platformViewId, "testType", /* hybrid=*/ true);
    assertEquals(ShadowFlutterJNI.getResponses().size(), 1);

    assertFalse(platformViewsController.initializePlatformViewIfNeeded(platformViewId));
  }

  @Test
  @Config(
      shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class, ShadowPresentation.class})
  public void onDetachedFromJNI_clearsPlatformViewContext() {
    PlatformViewsController platformViewsController = new PlatformViewsController();

    int platformViewId = 0;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    PlatformView platformView = mock(PlatformView.class);

    SurfaceView pv = mock(SurfaceView.class);
    when(pv.getContext()).thenReturn(mock(MutableContextWrapper.class));
    when(pv.getLayoutParams()).thenReturn(new LayoutParams(1, 1));

    when(platformView.getView()).thenReturn(pv);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);
    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    FlutterJNI jni = new FlutterJNI();
    attach(jni, platformViewsController);

    // Simulate create call from the framework.
    createPlatformView(
        jni, platformViewsController, platformViewId, "testType", /* hybrid=*/ false);

    assertFalse(platformViewsController.contextToEmbeddedView.isEmpty());
    platformViewsController.onDetachedFromJNI();
    assertTrue(platformViewsController.contextToEmbeddedView.isEmpty());
  }

  @Test
  @Config(
      shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class, ShadowPresentation.class})
  public void onPreEngineRestart_clearsPlatformViewContext() {
    PlatformViewsController platformViewsController = new PlatformViewsController();

    int platformViewId = 0;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    PlatformView platformView = mock(PlatformView.class);

    SurfaceView pv = mock(SurfaceView.class);
    when(pv.getContext()).thenReturn(mock(MutableContextWrapper.class));
    when(pv.getLayoutParams()).thenReturn(new LayoutParams(1, 1));

    when(platformView.getView()).thenReturn(pv);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);
    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    FlutterJNI jni = new FlutterJNI();
    attach(jni, platformViewsController);

    // Simulate create call from the framework.
    createPlatformView(
        jni, platformViewsController, platformViewId, "testType", /* hybrid=*/ false);

    assertFalse(platformViewsController.contextToEmbeddedView.isEmpty());
    platformViewsController.onDetachedFromJNI();
    assertTrue(platformViewsController.contextToEmbeddedView.isEmpty());
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void createPlatformViewMessage_throwsIfViewHasParent() {
    PlatformViewsController platformViewsController = new PlatformViewsController();

    int platformViewId = 0;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    PlatformView platformView = mock(PlatformView.class);
    View androidView = mock(View.class);
    when(androidView.getParent()).thenReturn(mock(ViewParent.class));
    when(platformView.getView()).thenReturn(androidView);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);
    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    FlutterJNI jni = new FlutterJNI();
    attach(jni, platformViewsController);

    // Simulate create call from the framework.
    createPlatformView(
        jni, platformViewsController, platformViewId, "testType", /* hybrid=*/ false);
    assertEquals(ShadowFlutterJNI.getResponses().size(), 1);

    assertThrows(
        IllegalStateException.class,
        () -> {
          platformViewsController.initializePlatformViewIfNeeded(platformViewId);
        });
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void createHybridPlatformViewMessage_throwsIfViewHasParent() {
    PlatformViewsController platformViewsController = new PlatformViewsController();

    int platformViewId = 0;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    PlatformView platformView = mock(PlatformView.class);
    View androidView = mock(View.class);
    when(androidView.getParent()).thenReturn(mock(ViewParent.class));
    when(platformView.getView()).thenReturn(androidView);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);
    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    FlutterJNI jni = new FlutterJNI();
    attach(jni, platformViewsController);

    // Simulate create call from the framework.
    createPlatformView(jni, platformViewsController, platformViewId, "testType", /* hybrid=*/ true);
    assertEquals(ShadowFlutterJNI.getResponses().size(), 1);

    assertThrows(
        IllegalStateException.class,
        () -> {
          platformViewsController.initializePlatformViewIfNeeded(platformViewId);
        });
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void setPlatformViewDirection_throwIfPlatformViewNotFound() {
    PlatformViewsController platformViewsController = new PlatformViewsController();

    int platformViewId = 0;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    PlatformView platformView = mock(PlatformView.class);
    final View androidView = mock(View.class);
    when(platformView.getView()).thenReturn(androidView);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);
    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    FlutterJNI jni = new FlutterJNI();
    attach(jni, platformViewsController);

    verify(androidView, never()).setLayoutDirection(anyInt());

    // Simulate create call from the framework.
    createPlatformView(jni, platformViewsController, platformViewId, "testType", /* hybrid=*/ true);
    assertEquals(ShadowFlutterJNI.getResponses().size(), 1);

    // Simulate set direction call from the framework.
    setLayoutDirection(jni, platformViewsController, platformViewId, 1);
    verify(androidView, times(1)).setLayoutDirection(1);

    // The limit value of reply message will be equal to 2 if the layout direction is set
    // successfully, otherwise it will be much more than 2 due to the reply message contains
    // an error message wrapped with exception detail information.
    assertEquals(ShadowFlutterJNI.getResponses().get(0).limit(), 2);
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void resizeAndroidView() {
    PlatformViewsController platformViewsController = new PlatformViewsController();
    platformViewsController.setSoftwareRendering(true);

    int platformViewId = 0;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    PlatformView platformView = mock(PlatformView.class);
    final View androidView = mock(View.class);
    when(platformView.getView()).thenReturn(androidView);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);
    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    FlutterJNI jni = new FlutterJNI();
    attach(jni, platformViewsController);

    // Simulate create call from the framework.
    createPlatformView(
        jni, platformViewsController, platformViewId, "testType", /* hybrid=*/ false);

    reset(androidView);
    when(androidView.getLayoutParams()).thenReturn(new FrameLayout.LayoutParams(0, 0));

    // Simulate a resize call from the framework.
    resize(jni, platformViewsController, platformViewId, 10.0, 20.0);

    ArgumentCaptor<FrameLayout.LayoutParams> layoutParamsCaptor =
        ArgumentCaptor.forClass(FrameLayout.LayoutParams.class);
    verify(androidView, times(1)).setLayoutParams(layoutParamsCaptor.capture());

    assertEquals(layoutParamsCaptor.getValue().width, 10);
    assertEquals(layoutParamsCaptor.getValue().height, 20);
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void disposeAndroidView_hybridComposition() {
    PlatformViewsController platformViewsController = new PlatformViewsController();

    int platformViewId = 0;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    PlatformView platformView = mock(PlatformView.class);

    Context context = ApplicationProvider.getApplicationContext();
    View androidView = new View(context);

    when(platformView.getView()).thenReturn(androidView);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);
    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    FlutterJNI jni = new FlutterJNI();
    attach(jni, platformViewsController);

    // Simulate create call from the framework.
    createPlatformView(jni, platformViewsController, platformViewId, "testType", /* hybrid=*/ true);
    assertTrue(platformViewsController.initializePlatformViewIfNeeded(platformViewId));

    assertNotNull(androidView.getParent());
    assertTrue(androidView.getParent() instanceof FlutterMutatorView);

    // Simulate dispose call from the framework.
    disposePlatformView(jni, platformViewsController, platformViewId);
    assertNull(androidView.getParent());

    // Simulate create call from the framework.
    createPlatformView(jni, platformViewsController, platformViewId, "testType", /* hybrid=*/ true);
    assertTrue(platformViewsController.initializePlatformViewIfNeeded(platformViewId));

    assertNotNull(androidView.getParent());
    assertTrue(androidView.getParent() instanceof FlutterMutatorView);
    verify(platformView, times(1)).dispose();
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void disposeNullAndroidView() {
    PlatformViewsController platformViewsController = new PlatformViewsController();

    int platformViewId = 0;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    PlatformView platformView = mock(PlatformView.class);

    View androidView = mock(View.class);
    when(platformView.getView()).thenReturn(androidView);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);
    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    FlutterJNI jni = new FlutterJNI();
    attach(jni, platformViewsController);

    // Simulate create call from the framework.
    createPlatformView(
        jni, platformViewsController, platformViewId, "testType", /* hybrid=*/ false);
    assertTrue(platformViewsController.initializePlatformViewIfNeeded(platformViewId));

    when(platformView.getView()).thenReturn(null);

    // Simulate dispose call from the framework.
    disposePlatformView(jni, platformViewsController, platformViewId);
    verify(platformView, times(1)).dispose();
  }

  @Test
  @Config(
      shadows = {
        ShadowFlutterSurfaceView.class,
        ShadowFlutterJNI.class,
        ShadowPlatformTaskQueue.class
      })
  public void onEndFrame_destroysOverlaySurfaceAfterFrameOnFlutterSurfaceView() {
    final PlatformViewsController platformViewsController = new PlatformViewsController();

    final int platformViewId = 0;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    final PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    final PlatformView platformView = mock(PlatformView.class);
    when(platformView.getView()).thenReturn(mock(View.class));
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);

    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    final FlutterJNI jni = new FlutterJNI();
    jni.attachToNative();
    attach(jni, platformViewsController);

    jni.onFirstFrame();

    // Simulate create call from the framework.
    createPlatformView(jni, platformViewsController, platformViewId, "testType", /* hybrid=*/ true);

    // Produce a frame that displays a platform view and an overlay surface.
    platformViewsController.onBeginFrame();
    platformViewsController.onDisplayPlatformView(
        platformViewId,
        /* x=*/ 0,
        /* y=*/ 0,
        /* width=*/ 10,
        /* height=*/ 10,
        /* viewWidth=*/ 10,
        /* viewHeight=*/ 10,
        /* mutatorsStack=*/ new FlutterMutatorsStack());

    final PlatformOverlayView overlayImageView = mock(PlatformOverlayView.class);
    when(overlayImageView.acquireLatestImage()).thenReturn(true);

    final FlutterOverlaySurface overlaySurface =
        platformViewsController.createOverlaySurface(overlayImageView);
    platformViewsController.onDisplayOverlaySurface(
        overlaySurface.getId(), /* x=*/ 0, /* y=*/ 0, /* width=*/ 10, /* height=*/ 10);

    platformViewsController.onEndFrame();

    // Simulate first frame from the framework.
    jni.onFirstFrame();

    verify(overlayImageView, never()).detachFromRenderer();

    // Produce a frame that doesn't display platform views.
    platformViewsController.onBeginFrame();
    platformViewsController.onEndFrame();

    shadowOf(getMainLooper()).idle();
    verify(overlayImageView, times(1)).detachFromRenderer();
  }

  @Test
  @Config(shadows = {ShadowFlutterSurfaceView.class, ShadowFlutterJNI.class})
  public void onEndFrame_removesPlatformView() {
    final PlatformViewsController platformViewsController = new PlatformViewsController();

    final int platformViewId = 0;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    final PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    final PlatformView platformView = mock(PlatformView.class);
    final View androidView = mock(View.class);
    when(platformView.getView()).thenReturn(androidView);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);

    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    final FlutterJNI jni = new FlutterJNI();
    jni.attachToNative();
    attach(jni, platformViewsController);

    jni.onFirstFrame();

    // Simulate create call from the framework.
    createPlatformView(jni, platformViewsController, platformViewId, "testType", /* hybrid=*/ true);

    // Simulate first frame from the framework.
    jni.onFirstFrame();
    platformViewsController.onBeginFrame();

    platformViewsController.onEndFrame();
    verify(androidView, never()).setVisibility(View.GONE);

    final ViewParent parentView = mock(ViewParent.class);
    when(androidView.getParent()).thenReturn(parentView);
  }

  @Test
  @Config(
      shadows = {
        ShadowFlutterSurfaceView.class,
        ShadowFlutterJNI.class,
        ShadowPlatformTaskQueue.class
      })
  public void onEndFrame_removesPlatformViewParent() {
    final PlatformViewsController platformViewsController = new PlatformViewsController();

    final int platformViewId = 0;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    final PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    final PlatformView platformView = mock(PlatformView.class);
    final View androidView = mock(View.class);
    when(platformView.getView()).thenReturn(androidView);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);

    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    final FlutterJNI jni = new FlutterJNI();
    jni.attachToNative();

    final FlutterView flutterView = attach(jni, platformViewsController);

    jni.onFirstFrame();

    // Simulate create call from the framework.
    createPlatformView(jni, platformViewsController, platformViewId, "testType", /* hybrid=*/ true);
    assertTrue(platformViewsController.initializePlatformViewIfNeeded(platformViewId));
    assertEquals(flutterView.getChildCount(), 2);

    // Simulate first frame from the framework.
    jni.onFirstFrame();
    platformViewsController.onBeginFrame();
    platformViewsController.onEndFrame();

    // Simulate dispose call from the framework.
    disposePlatformView(jni, platformViewsController, platformViewId);
    assertEquals(flutterView.getChildCount(), 1);
  }

  @Test
  @Config(
      shadows = {
        ShadowFlutterSurfaceView.class,
        ShadowFlutterJNI.class,
        ShadowPlatformTaskQueue.class
      })
  public void detach_destroysOverlaySurfaces() {
    final PlatformViewsController platformViewsController = new PlatformViewsController();

    final int platformViewId = 0;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    final PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    final PlatformView platformView = mock(PlatformView.class);
    when(platformView.getView()).thenReturn(mock(View.class));
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);

    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    final FlutterJNI jni = new FlutterJNI();
    jni.attachToNative();
    attach(jni, platformViewsController);

    jni.onFirstFrame();

    // Simulate create call from the framework.
    createPlatformView(jni, platformViewsController, platformViewId, "testType", /* hybrid=*/ true);

    // Produce a frame that displays a platform view and an overlay surface.
    platformViewsController.onBeginFrame();
    platformViewsController.onDisplayPlatformView(
        platformViewId,
        /* x=*/ 0,
        /* y=*/ 0,
        /* width=*/ 10,
        /* height=*/ 10,
        /* viewWidth=*/ 10,
        /* viewHeight=*/ 10,
        /* mutatorsStack=*/ new FlutterMutatorsStack());

    final PlatformOverlayView overlayImageView = mock(PlatformOverlayView.class);
    when(overlayImageView.acquireLatestImage()).thenReturn(true);

    final FlutterOverlaySurface overlaySurface =
        platformViewsController.createOverlaySurface(overlayImageView);
    // This is OK.
    platformViewsController.onDisplayOverlaySurface(
        overlaySurface.getId(), /* x=*/ 0, /* y=*/ 0, /* width=*/ 10, /* height=*/ 10);

    platformViewsController.detach();

    verify(overlayImageView, times(1)).closeImageReader();
    verify(overlayImageView, times(1)).detachFromRenderer();
  }

  @Test
  @Config(shadows = {ShadowFlutterSurfaceView.class, ShadowFlutterJNI.class})
  public void detachFromView_removesAndDestroysOverlayViews() {
    final PlatformViewsController platformViewsController = new PlatformViewsController();

    final int platformViewId = 0;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    final PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    final PlatformView platformView = mock(PlatformView.class);
    when(platformView.getView()).thenReturn(mock(View.class));
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);

    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    final FlutterJNI jni = new FlutterJNI();
    jni.attachToNative();
    attach(jni, platformViewsController);

    final FlutterView flutterView = mock(FlutterView.class);
    platformViewsController.attachToView(flutterView);

    final PlatformOverlayView overlayImageView = mock(PlatformOverlayView.class);
    when(overlayImageView.acquireLatestImage()).thenReturn(true);

    final FlutterOverlaySurface overlaySurface =
        platformViewsController.createOverlaySurface(overlayImageView);

    platformViewsController.onDisplayOverlaySurface(
        overlaySurface.getId(), /* x=*/ 0, /* y=*/ 0, /* width=*/ 10, /* height=*/ 10);

    platformViewsController.detachFromView();

    verify(overlayImageView, times(1)).closeImageReader();
    verify(overlayImageView, times(1)).detachFromRenderer();
    verify(flutterView, times(1)).removeView(overlayImageView);
  }

  @Test
  @Config(shadows = {ShadowFlutterSurfaceView.class, ShadowFlutterJNI.class})
  public void destroyOverlaySurfaces_doesNotThrowIfFlutterViewIsDetached() {
    final PlatformViewsController platformViewsController = new PlatformViewsController();

    final int platformViewId = 0;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    final PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    final PlatformView platformView = mock(PlatformView.class);
    when(platformView.getView()).thenReturn(mock(View.class));
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);

    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    final FlutterJNI jni = new FlutterJNI();
    jni.attachToNative();
    attach(jni, platformViewsController);

    final FlutterView flutterView = mock(FlutterView.class);
    platformViewsController.attachToView(flutterView);

    final PlatformOverlayView overlayImageView = mock(PlatformOverlayView.class);
    when(overlayImageView.acquireLatestImage()).thenReturn(true);

    final FlutterOverlaySurface overlaySurface =
        platformViewsController.createOverlaySurface(overlayImageView);

    platformViewsController.onDisplayOverlaySurface(
        overlaySurface.getId(), /* x=*/ 0, /* y=*/ 0, /* width=*/ 10, /* height=*/ 10);

    platformViewsController.detachFromView();

    platformViewsController.destroyOverlaySurfaces();
    verify(overlayImageView, times(1)).closeImageReader();
    verify(overlayImageView, times(1)).detachFromRenderer();
  }

  @Test
  @Config(shadows = {ShadowFlutterSurfaceView.class, ShadowFlutterJNI.class})
  public void destroyOverlaySurfaces_doesNotRemoveOverlayView() {
    final PlatformViewsController platformViewsController = new PlatformViewsController();

    final int platformViewId = 0;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    final PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    final PlatformView platformView = mock(PlatformView.class);
    when(platformView.getView()).thenReturn(mock(View.class));
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);

    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    final FlutterJNI jni = new FlutterJNI();
    jni.attachToNative();
    attach(jni, platformViewsController);

    final FlutterView flutterView = mock(FlutterView.class);
    platformViewsController.attachToView(flutterView);

    final PlatformOverlayView overlayImageView = mock(PlatformOverlayView.class);
    when(overlayImageView.acquireLatestImage()).thenReturn(true);

    final FlutterOverlaySurface overlaySurface =
        platformViewsController.createOverlaySurface(overlayImageView);

    platformViewsController.onDisplayOverlaySurface(
        overlaySurface.getId(), /* x=*/ 0, /* y=*/ 0, /* width=*/ 10, /* height=*/ 10);

    platformViewsController.destroyOverlaySurfaces();
    verify(flutterView, never()).removeView(overlayImageView);
  }

  @Test
  public void checkInputConnectionProxy_falseIfViewIsNull() {
    final PlatformViewsController platformViewsController = new PlatformViewsController();
    boolean shouldProxying = platformViewsController.checkInputConnectionProxy(null);
    assertFalse(shouldProxying);
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void convertPlatformViewRenderSurfaceAsDefault() {
    final PlatformViewsController platformViewsController = new PlatformViewsController();

    final int platformViewId = 0;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    final PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    final PlatformView platformView = mock(PlatformView.class);
    final View androidView = mock(View.class);
    when(platformView.getView()).thenReturn(androidView);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);

    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    final FlutterJNI jni = new FlutterJNI();
    jni.attachToNative();
    final FlutterView flutterView = attach(jni, platformViewsController);

    jni.onFirstFrame();

    // Simulate create call from the framework.
    createPlatformView(jni, platformViewsController, platformViewId, "testType", /* hybrid=*/ true);

    // Produce a frame that displays a platform view and an overlay surface.
    platformViewsController.onBeginFrame();
    platformViewsController.onDisplayPlatformView(
        platformViewId,
        /* x=*/ 0,
        /* y=*/ 0,
        /* width=*/ 10,
        /* height=*/ 10,
        /* viewWidth=*/ 10,
        /* viewHeight=*/ 10,
        /* mutatorsStack=*/ new FlutterMutatorsStack());

    assertEquals(flutterView.getChildCount(), 3);

    final View view = flutterView.getChildAt(1);
    assertTrue(view instanceof FlutterImageView);

    // Simulate dispose call from the framework.
    disposePlatformView(jni, platformViewsController, platformViewId);
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void dontConverRenderSurfaceWhenFlagIsTrue() {
    final PlatformViewsController platformViewsController = new PlatformViewsController();

    final int platformViewId = 0;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    final PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    final PlatformView platformView = mock(PlatformView.class);
    final View androidView = mock(View.class);
    when(platformView.getView()).thenReturn(androidView);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);

    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    final FlutterJNI jni = new FlutterJNI();
    jni.attachToNative();
    final FlutterView flutterView = attach(jni, platformViewsController);

    jni.onFirstFrame();

    // Simulate setting render surface conversion flag.
    synchronizeToNativeViewHierarchy(jni, platformViewsController, false);

    // Simulate create call from the framework.
    createPlatformView(jni, platformViewsController, platformViewId, "testType", /* hybrid=*/ true);

    // Produce a frame that displays a platform view and an overlay surface.
    platformViewsController.onBeginFrame();
    platformViewsController.onDisplayPlatformView(
        platformViewId,
        /* x=*/ 0,
        /* y=*/ 0,
        /* width=*/ 10,
        /* height=*/ 10,
        /* viewWidth=*/ 10,
        /* viewHeight=*/ 10,
        /* mutatorsStack=*/ new FlutterMutatorsStack());

    assertEquals(flutterView.getChildCount(), 2);
    assertTrue(!(flutterView.getChildAt(0) instanceof PlatformOverlayView));
    assertTrue(flutterView.getChildAt(1) instanceof FlutterMutatorView);

    // Simulate dispose call from the framework.
    disposePlatformView(jni, platformViewsController, platformViewId);
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void reattachToFlutterView() {
    PlatformViewsController platformViewsController = new PlatformViewsController();
    platformViewsController.setSoftwareRendering(true);

    int platformViewId = 100;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    PlatformView platformView = mock(PlatformView.class);
    View androidView = mock(View.class);
    when(platformView.getView()).thenReturn(androidView);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);
    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    FlutterJNI jni = new FlutterJNI();
    FlutterView initFlutterView = mock(FlutterView.class);
    attachToFlutterView(jni, platformViewsController, initFlutterView);

    createPlatformView(
        jni, platformViewsController, platformViewId, "testType", /* hybrid=*/ false);
    verify(initFlutterView, times(1)).addView(any(PlatformViewWrapper.class));

    platformViewsController.detachFromView();
    verify(initFlutterView, times(1)).removeView(any(PlatformViewWrapper.class));

    FlutterView newFlutterView = mock(FlutterView.class);
    platformViewsController.attachToView(newFlutterView);
    verify(newFlutterView, times(1)).addView(any(PlatformViewWrapper.class));
  }

  @Config(
      shadows = {
        ShadowFlutterSurfaceView.class,
        ShadowFlutterJNI.class,
        ShadowPlatformTaskQueue.class
      })
  public void revertImageViewAndRemoveImageView() {
    final PlatformViewsController platformViewsController = new PlatformViewsController();

    final int platformViewId = 0;
    assertNull(platformViewsController.getPlatformViewById(platformViewId));

    final PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    final PlatformView platformView = mock(PlatformView.class);
    final View androidView = mock(View.class);
    when(platformView.getView()).thenReturn(androidView);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);

    platformViewsController.getRegistry().registerViewFactory("testType", viewFactory);

    final FlutterJNI jni = new FlutterJNI();
    jni.attachToNative();

    final FlutterView flutterView = attach(jni, platformViewsController);

    jni.onFirstFrame();

    // Simulate create call from the framework.
    createPlatformView(jni, platformViewsController, platformViewId, "testType", /* hybrid=*/ true);

    // The simulation creates an Overlay on top of the PlatformView
    // This is going to be called `flutterView.convertToImageView`
    platformViewsController.createOverlaySurface();
    platformViewsController.onDisplayOverlaySurface(platformViewId, 0, 0, 10, 10);

    // This will contain three views: Background ImageView、PlatformView、Overlay ImageView
    assertEquals(flutterView.getChildCount(), 3);

    FlutterImageView imageView = flutterView.getCurrentImageSurface();

    // Make sure the ImageView is inside the current FlutterView.
    assertTrue(imageView != null);
    assertTrue(flutterView.indexOfChild(imageView) != -1);

    // Make sure the overlayView is inside the current FlutterView
    assertTrue(platformViewsController.getOverlayLayerViews().size() != 0);
    PlatformOverlayView overlayView = platformViewsController.getOverlayLayerViews().get(0);
    assertTrue(overlayView != null);
    assertTrue(flutterView.indexOfChild(overlayView) != -1);

    // Simulate in a new frame, there's no PlatformView, which is called
    // `flutterView.revertImageView`. And register a `FlutterUiDisplayListener` callback.
    // During callback execution it will invoke `flutterImageView.detachFromRenderer()`.
    platformViewsController.onBeginFrame();
    platformViewsController.onEndFrame();

    // Invoke all registered `FlutterUiDisplayListener` callback
    jni.onFirstFrame();

    assertEquals(null, flutterView.getCurrentImageSurface());

    // Make sure the background ImageVIew is not in the FlutterView
    assertTrue(flutterView.indexOfChild(imageView) == -1);

    // Make sure the overlay ImageVIew is not in the FlutterView
    assertTrue(flutterView.indexOfChild(overlayView) == -1);
  }

  private static ByteBuffer encodeMethodCall(MethodCall call) {
    final ByteBuffer buffer = StandardMethodCodec.INSTANCE.encodeMethodCall(call);
    buffer.rewind();
    return buffer;
  }

  private static void createPlatformView(
      FlutterJNI jni,
      PlatformViewsController platformViewsController,
      int platformViewId,
      String viewType,
      boolean hybrid) {
    final Map<String, Object> args = new HashMap<>();
    args.put("hybrid", hybrid);
    args.put("id", platformViewId);
    args.put("viewType", viewType);
    args.put("direction", 0);
    args.put("width", 1.0);
    args.put("height", 1.0);

    final MethodCall platformCreateMethodCall = new MethodCall("create", args);

    jni.handlePlatformMessage(
        "flutter/platform_views",
        encodeMethodCall(platformCreateMethodCall),
        /*replyId=*/ 0,
        /*messageData=*/ 0);
  }

  private static void setLayoutDirection(
      FlutterJNI jni,
      PlatformViewsController platformViewsController,
      int platformViewId,
      int direction) {
    final Map<String, Object> args = new HashMap<>();
    args.put("id", platformViewId);
    args.put("direction", direction);

    final MethodCall platformSetDirectionMethodCall = new MethodCall("setDirection", args);

    jni.handlePlatformMessage(
        "flutter/platform_views",
        encodeMethodCall(platformSetDirectionMethodCall),
        /*replyId=*/ 0,
        /*messageData=*/ 0);
  }

  private static void resize(
      FlutterJNI jni,
      PlatformViewsController platformViewsController,
      int platformViewId,
      double width,
      double height) {
    final Map<String, Object> args = new HashMap<>();
    args.put("id", platformViewId);
    args.put("width", width);
    args.put("height", height);

    final MethodCall platformResizeMethodCall = new MethodCall("resize", args);

    jni.handlePlatformMessage(
        "flutter/platform_views",
        encodeMethodCall(platformResizeMethodCall),
        /*replyId=*/ 0,
        /*messageData=*/ 0);
  }

  private static void disposePlatformView(
      FlutterJNI jni, PlatformViewsController platformViewsController, int platformViewId) {

    final Map<String, Object> args = new HashMap<>();
    args.put("hybrid", true);
    args.put("id", platformViewId);

    final MethodCall platformDisposeMethodCall = new MethodCall("dispose", args);

    jni.handlePlatformMessage(
        "flutter/platform_views",
        encodeMethodCall(platformDisposeMethodCall),
        /*replyId=*/ 0,
        /*messageData=*/ 0);
  }

  private static void synchronizeToNativeViewHierarchy(
      FlutterJNI jni, PlatformViewsController platformViewsController, boolean yes) {

    final MethodCall convertMethodCall = new MethodCall("synchronizeToNativeViewHierarchy", yes);

    jni.handlePlatformMessage(
        "flutter/platform_views",
        encodeMethodCall(convertMethodCall),
        /*replyId=*/ 0,
        /*messageData=*/ 0);
  }

  private static FlutterView attach(
      FlutterJNI jni, PlatformViewsController platformViewsController) {
    final Context context = ApplicationProvider.getApplicationContext();
    final FlutterView flutterView =
        new FlutterView(context, new FlutterSurfaceView(context)) {
          @Override
          public FlutterImageView createImageView() {
            final FlutterImageView view = mock(FlutterImageView.class);
            when(view.acquireLatestImage()).thenReturn(true);
            return mock(FlutterImageView.class);
          }
        };
    attachToFlutterView(jni, platformViewsController, flutterView);
    return flutterView;
  }

  private static void attachToFlutterView(
      FlutterJNI jni, PlatformViewsController platformViewsController, FlutterView flutterView) {
    final DartExecutor executor = new DartExecutor(jni, mock(AssetManager.class));
    executor.onAttachedToJNI();

    final Context context = ApplicationProvider.getApplicationContext();
    final TextureRegistry registry =
        new TextureRegistry() {
          public void TextureRegistry() {}

          @NonNull
          @Override
          public SurfaceTextureEntry createSurfaceTexture() {
            return registerSurfaceTexture(mock(SurfaceTexture.class));
          }

          @NonNull
          @Override
          public SurfaceTextureEntry registerSurfaceTexture(
              @NonNull SurfaceTexture surfaceTexture) {
            return new SurfaceTextureEntry() {
              @NonNull
              @Override
              public SurfaceTexture surfaceTexture() {
                return mock(SurfaceTexture.class);
              }

              @Override
              public long id() {
                return 0;
              }

              @Override
              public void release() {}
            };
          }

          @NonNull
          @Override
          public ImageTextureEntry createImageTexture() {
            return new ImageTextureEntry() {
              @Override
              public long id() {
                return 0;
              }

              @Override
              public void release() {}

              @Override
              public void pushImage(Image image) {}
            };
          }

          @NonNull
          @Override
          public SurfaceProducer createSurfaceProducer() {
            return new SurfaceProducer() {
              @Override
              public void setCallback(SurfaceProducer.Callback cb) {}

              @Override
              public long id() {
                return 0;
              }

              @Override
              public void release() {}

              @Override
              public int getWidth() {
                return 0;
              }

              @Override
              public int getHeight() {
                return 0;
              }

              @Override
              public void setSize(int width, int height) {}

              @Override
              public Surface getSurface() {
                return null;
              }

              @Override
              public boolean handlesCropAndRotation() {
                return false;
              }

              public void scheduleFrame() {}
            };
          }
        };

    platformViewsController.attach(context, registry, executor);

    final FlutterEngine engine = mock(FlutterEngine.class);
    when(engine.getRenderer()).thenReturn(new FlutterRenderer(jni));
    when(engine.getMouseCursorChannel()).thenReturn(mock(MouseCursorChannel.class));
    when(engine.getTextInputChannel()).thenReturn(mock(TextInputChannel.class));
    when(engine.getSettingsChannel()).thenReturn(new SettingsChannel(executor));
    when(engine.getScribeChannel()).thenReturn(mock(ScribeChannel.class));
    when(engine.getPlatformViewsController()).thenReturn(platformViewsController);
    when(engine.getLocalizationPlugin()).thenReturn(mock(LocalizationPlugin.class));
    when(engine.getAccessibilityChannel()).thenReturn(mock(AccessibilityChannel.class));
    when(engine.getDartExecutor()).thenReturn(executor);

    flutterView.attachToFlutterEngine(engine);
    platformViewsController.attachToView(flutterView);
  }

  /**
   * For convenience when writing tests, this allows us to make fake messages from Flutter via
   * Platform Channels. Typically those calls happen on the ui thread which dispatches to the
   * platform thread. Since tests run on the platform thread it makes it difficult to test without
   * this, but isn't technically required.
   */
  @Implements(io.flutter.embedding.engine.dart.PlatformTaskQueue.class)
  public static class ShadowPlatformTaskQueue {
    @Implementation
    public void dispatch(Runnable runnable) {
      runnable.run();
    }
  }

  /**
   * The shadow class of {@link Presentation} to simulate Presentation showing logic.
   *
   * <p>Robolectric doesn't support VirtualDisplay creating correctly now, so this shadow class is
   * used to simulate custom logic for Presentation.
   */
  @Implements(Presentation.class)
  public static class ShadowPresentation extends ShadowDialog {
    private boolean isShowing = false;

    public ShadowPresentation() {}

    @Implementation
    protected void show() {
      isShowing = true;
    }

    @Implementation
    protected void dismiss() {
      isShowing = false;
    }

    @Implementation
    protected boolean isShowing() {
      return isShowing;
    }
  }

  @Implements(FlutterJNI.class)
  public static class ShadowFlutterJNI {
    private static SparseArray<ByteBuffer> replies = new SparseArray<>();

    public ShadowFlutterJNI() {}

    @Implementation
    public boolean getIsSoftwareRenderingEnabled() {
      return false;
    }

    @Implementation
    public long performNativeAttach(FlutterJNI flutterJNI) {
      return 1;
    }

    @Implementation
    public void dispatchPlatformMessage(
        String channel, ByteBuffer message, int position, int responseId) {}

    @Implementation
    public void onSurfaceCreated(Surface surface) {}

    @Implementation
    public void onSurfaceDestroyed() {}

    @Implementation
    public void onSurfaceWindowChanged(Surface surface) {}

    @Implementation
    public void setViewportMetrics(
        float devicePixelRatio,
        int physicalWidth,
        int physicalHeight,
        int physicalPaddingTop,
        int physicalPaddingRight,
        int physicalPaddingBottom,
        int physicalPaddingLeft,
        int physicalViewInsetTop,
        int physicalViewInsetRight,
        int physicalViewInsetBottom,
        int physicalViewInsetLeft,
        int systemGestureInsetTop,
        int systemGestureInsetRight,
        int systemGestureInsetBottom,
        int systemGestureInsetLeft,
        int physicalTouchSlop,
        int[] displayFeaturesBounds,
        int[] displayFeaturesType,
        int[] displayFeaturesState) {}

    @Implementation
    public void invokePlatformMessageResponseCallback(
        int responseId, ByteBuffer message, int position) {
      replies.put(responseId, message);
    }

    public static SparseArray<ByteBuffer> getResponses() {
      return replies;
    }
  }

  @Implements(SurfaceView.class)
  public static class ShadowFlutterSurfaceView extends ShadowSurfaceView {
    private final FakeSurfaceHolder holder = new FakeSurfaceHolder();

    public static class FakeSurfaceHolder extends ShadowSurfaceView.FakeSurfaceHolder {
      private final Surface surface = mock(Surface.class);

      public Surface getSurface() {
        return surface;
      }

      @Implementation
      public void addCallback(SurfaceHolder.Callback callback) {
        callback.surfaceCreated(this);
      }
    }

    public ShadowFlutterSurfaceView() {}

    @Implementation
    public SurfaceHolder getHolder() {
      return holder;
    }
  }
}
