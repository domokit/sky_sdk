// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.platform;

import static io.flutter.embedding.engine.systemchannels.PlatformViewsChannel2.PlatformViewTouch;
import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

import android.app.Presentation;
import android.content.Context;
import android.content.res.AssetManager;
import android.util.SparseArray;
import android.view.MotionEvent;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import androidx.test.core.app.ApplicationProvider;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.embedding.android.FlutterImageView;
import io.flutter.embedding.android.FlutterSurfaceView;
import io.flutter.embedding.android.FlutterView;
import io.flutter.embedding.android.MotionEventTracker;
import io.flutter.embedding.engine.FlutterEngine;
import io.flutter.embedding.engine.FlutterJNI;
import io.flutter.embedding.engine.dart.DartExecutor;
import io.flutter.embedding.engine.mutatorsstack.FlutterMutatorView;
import io.flutter.embedding.engine.renderer.FlutterRenderer;
import io.flutter.embedding.engine.systemchannels.AccessibilityChannel;
import io.flutter.embedding.engine.systemchannels.MouseCursorChannel;
import io.flutter.embedding.engine.systemchannels.PlatformViewsChannel2;
import io.flutter.embedding.engine.systemchannels.PlatformViewsChannel2.PlatformViewTouch;
import io.flutter.embedding.engine.systemchannels.ScribeChannel;
import io.flutter.embedding.engine.systemchannels.SettingsChannel;
import io.flutter.embedding.engine.systemchannels.TextInputChannel;
import io.flutter.plugin.common.MethodCall;
import io.flutter.plugin.common.StandardMessageCodec;
import io.flutter.plugin.common.StandardMethodCodec;
import io.flutter.plugin.localization.LocalizationPlugin;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.annotation.Config;
import org.robolectric.annotation.Implementation;
import org.robolectric.annotation.Implements;
import org.robolectric.shadows.ShadowDialog;
import org.robolectric.shadows.ShadowSurfaceView;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
public class PlatformViewsController2Test {
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
    PlatformViewRegistryImpl registryImpl = new PlatformViewRegistryImpl();
    PlatformViewsController2 PlatformViewsController2 = new PlatformViewsController2();
    PlatformViewsController2.setRegistry(registryImpl);
    FlutterJNI jni = new FlutterJNI();
    attach(jni, PlatformViewsController2);
    // Get the platform view registry.
    PlatformViewRegistry registry = PlatformViewsController2.getRegistry();

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
    final PlatformViewsChannel2.PlatformViewCreationRequest request =
        new PlatformViewsChannel2.PlatformViewCreationRequest(
            viewId, CountingPlatformView.VIEW_TYPE_ID, 128, 128, View.LAYOUT_DIRECTION_LTR, null);
    PlatformView pView = PlatformViewsController2.createFlutterPlatformView(request);
    assertTrue(pView instanceof CountingPlatformView);
    CountingPlatformView cpv = (CountingPlatformView) pView;

    PlatformViewsController2.disposePlatformView(viewId);
    assertEquals(1, cpv.disposeCalls);
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void itNotifiesPlatformViewsOfEngineAttachmentAndDetachment() {
    PlatformViewRegistryImpl registryImpl = new PlatformViewRegistryImpl();
    PlatformViewsController2 PlatformViewsController2 = new PlatformViewsController2();
    PlatformViewsController2.setRegistry(registryImpl);
    FlutterJNI jni = new FlutterJNI();
    attach(jni, PlatformViewsController2);
    // Get the platform view registry.
    PlatformViewRegistry registry = PlatformViewsController2.getRegistry();

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
    final PlatformViewsChannel2.PlatformViewCreationRequest request =
        new PlatformViewsChannel2.PlatformViewCreationRequest(
            viewId, CountingPlatformView.VIEW_TYPE_ID, 128, 128, View.LAYOUT_DIRECTION_LTR, null);

    PlatformView pView = PlatformViewsController2.createFlutterPlatformView(request);
    assertTrue(pView instanceof CountingPlatformView);
    CountingPlatformView cpv = (CountingPlatformView) pView;
    assertEquals(1, cpv.attachCalls);
    assertEquals(0, cpv.detachCalls);
    assertEquals(0, cpv.disposeCalls);
    PlatformViewsController2.detachFromView();
    assertEquals(1, cpv.attachCalls);
    assertEquals(1, cpv.detachCalls);
    assertEquals(0, cpv.disposeCalls);
    PlatformViewsController2.disposePlatformView(viewId);
  }

  @Test
  public void itUsesActionEventTypeFromFrameworkEventAsActionChanged() {
    MotionEventTracker motionEventTracker = MotionEventTracker.getInstance();
    PlatformViewRegistryImpl registryImpl = new PlatformViewRegistryImpl();
    PlatformViewsController2 PlatformViewsController2 = new PlatformViewsController2();
    PlatformViewsController2.setRegistry(registryImpl);

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
        PlatformViewsController2.toMotionEvent(
            1, // density
            frameWorkTouch);
    assertEquals(resolvedEvent.getAction(), original.getAction());
    assertNotEquals(resolvedEvent.getAction(), frameWorkTouch.action);
  }

  private MotionEvent makePlatformViewTouchAndInvokeToMotionEvent(
      PlatformViewsController2 PlatformViewsController2,
      MotionEventTracker motionEventTracker,
      MotionEvent original,
      boolean usingVirtualDisplays) {
    MotionEventTracker.MotionEventId motionEventId = motionEventTracker.track(original);

    // Construct a PlatformViewTouch.rawPointerPropertiesList by doing the inverse of
    // PlatformViewsController2.parsePointerPropertiesList.
    List<List<Integer>> pointerProperties =
        Arrays.asList(Arrays.asList(original.getPointerId(0), original.getToolType(0)));
    // Construct a PlatformViewTouch.rawPointerCoords by doing the inverse of
    // PlatformViewsController2.parsePointerCoordsList.
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

    return PlatformViewsController2.toMotionEvent(
        1, // density
        frameWorkTouchNonVd);
  }

  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void getPlatformViewById() {
    PlatformViewRegistryImpl registryImpl = new PlatformViewRegistryImpl();
    PlatformViewsController2 PlatformViewsController2 = new PlatformViewsController2();
    PlatformViewsController2.setRegistry(registryImpl);

    int platformViewId = 0;
    assertNull(PlatformViewsController2.getPlatformViewById(platformViewId));

    PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    PlatformView platformView = mock(PlatformView.class);
    View androidView = mock(View.class);
    when(platformView.getView()).thenReturn(androidView);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);
    PlatformViewsController2.getRegistry().registerViewFactory("testType", viewFactory);

    FlutterJNI jni = new FlutterJNI();
    attach(jni, PlatformViewsController2);

    // Simulate create call from the framework.
    createPlatformView(jni, PlatformViewsController2, platformViewId, "testType");

    assertTrue(PlatformViewsController2.initializePlatformViewIfNeeded(platformViewId));

    View resultAndroidView = PlatformViewsController2.getPlatformViewById(platformViewId);
    assertNotNull(resultAndroidView);
    assertEquals(resultAndroidView, androidView);
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void createPlatformViewMessage_initializesAndroidView() {
    PlatformViewRegistryImpl registryImpl = new PlatformViewRegistryImpl();
    PlatformViewsController2 PlatformViewsController2 = new PlatformViewsController2();
    PlatformViewsController2.setRegistry(registryImpl);

    int platformViewId = 0;
    assertNull(PlatformViewsController2.getPlatformViewById(platformViewId));

    PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    PlatformView platformView = mock(PlatformView.class);
    when(platformView.getView()).thenReturn(mock(View.class));
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);
    PlatformViewsController2.getRegistry().registerViewFactory("testType", viewFactory);

    FlutterJNI jni = new FlutterJNI();
    attach(jni, PlatformViewsController2);

    // Simulate create call from the framework.
    createPlatformView(jni, PlatformViewsController2, platformViewId, "testType");
    verify(viewFactory, times(1)).create(any(), eq(platformViewId), any());
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void createPlatformViewMessage_setsAndroidViewLayoutDirection() {
    PlatformViewRegistryImpl registryImpl = new PlatformViewRegistryImpl();
    PlatformViewsController2 PlatformViewsController2 = new PlatformViewsController2();
    PlatformViewsController2.setRegistry(registryImpl);

    int platformViewId = 0;
    assertNull(PlatformViewsController2.getPlatformViewById(platformViewId));

    PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    PlatformView platformView = mock(PlatformView.class);

    View androidView = mock(View.class);
    when(platformView.getView()).thenReturn(androidView);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);
    PlatformViewsController2.getRegistry().registerViewFactory("testType", viewFactory);

    FlutterJNI jni = new FlutterJNI();
    attach(jni, PlatformViewsController2);

    // Simulate create call from the framework.
    createPlatformView(jni, PlatformViewsController2, platformViewId, "testType");
    verify(androidView, times(1)).setLayoutDirection(0);
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void createPlatformViewMessage_throwsIfViewIsNull() {
    PlatformViewRegistryImpl registryImpl = new PlatformViewRegistryImpl();
    PlatformViewsController2 PlatformViewsController2 = new PlatformViewsController2();
    PlatformViewsController2.setRegistry(registryImpl);

    int platformViewId = 0;
    assertNull(PlatformViewsController2.getPlatformViewById(platformViewId));

    PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    PlatformView platformView = mock(PlatformView.class);
    when(platformView.getView()).thenReturn(null);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);
    PlatformViewsController2.getRegistry().registerViewFactory("testType", viewFactory);

    FlutterJNI jni = new FlutterJNI();
    attach(jni, PlatformViewsController2);

    // Simulate create call from the framework.
    createPlatformView(jni, PlatformViewsController2, platformViewId, "testType");
    assertEquals(ShadowFlutterJNI.getResponses().size(), 1);

    assertFalse(PlatformViewsController2.initializePlatformViewIfNeeded(platformViewId));
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void createHybridPlatformViewMessage_throwsIfViewIsNull() {
    PlatformViewRegistryImpl registryImpl = new PlatformViewRegistryImpl();
    PlatformViewsController2 PlatformViewsController2 = new PlatformViewsController2();
    PlatformViewsController2.setRegistry(registryImpl);

    int platformViewId = 0;
    assertNull(PlatformViewsController2.getPlatformViewById(platformViewId));

    PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    PlatformView platformView = mock(PlatformView.class);
    when(platformView.getView()).thenReturn(null);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);
    PlatformViewsController2.getRegistry().registerViewFactory("testType", viewFactory);

    FlutterJNI jni = new FlutterJNI();
    attach(jni, PlatformViewsController2);

    // Simulate create call from the framework.
    createPlatformView(jni, PlatformViewsController2, platformViewId, "testType");
    assertEquals(ShadowFlutterJNI.getResponses().size(), 1);

    assertFalse(PlatformViewsController2.initializePlatformViewIfNeeded(platformViewId));
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void setPlatformViewDirection_throwIfPlatformViewNotFound() {
    PlatformViewRegistryImpl registryImpl = new PlatformViewRegistryImpl();
    PlatformViewsController2 PlatformViewsController2 = new PlatformViewsController2();
    PlatformViewsController2.setRegistry(registryImpl);

    int platformViewId = 0;
    assertNull(PlatformViewsController2.getPlatformViewById(platformViewId));

    PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    PlatformView platformView = mock(PlatformView.class);
    final View androidView = mock(View.class);
    when(platformView.getView()).thenReturn(androidView);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);
    PlatformViewsController2.getRegistry().registerViewFactory("testType", viewFactory);

    FlutterJNI jni = new FlutterJNI();
    attach(jni, PlatformViewsController2);

    verify(androidView, never()).setLayoutDirection(anyInt());

    // Simulate create call from the framework.
    createPlatformView(jni, PlatformViewsController2, platformViewId, "testType");
    assertEquals(ShadowFlutterJNI.getResponses().size(), 1);

    // Simulate set direction call from the framework.
    setLayoutDirection(jni, PlatformViewsController2, platformViewId, 1);
    verify(androidView, times(1)).setLayoutDirection(1);

    // The limit value of reply message will be equal to 2 if the layout direction is set
    // successfully, otherwise it will be much more than 2 due to the reply message contains
    // an error message wrapped with exception detail information.
    assertEquals(ShadowFlutterJNI.getResponses().get(0).limit(), 2);
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void disposeAndroidView() {
    PlatformViewRegistryImpl registryImpl = new PlatformViewRegistryImpl();
    PlatformViewsController2 PlatformViewsController2 = new PlatformViewsController2();
    PlatformViewsController2.setRegistry(registryImpl);

    int platformViewId = 0;
    assertNull(PlatformViewsController2.getPlatformViewById(platformViewId));

    PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    PlatformView platformView = mock(PlatformView.class);

    Context context = ApplicationProvider.getApplicationContext();
    View androidView = new View(context);

    when(platformView.getView()).thenReturn(androidView);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);
    PlatformViewsController2.getRegistry().registerViewFactory("testType", viewFactory);

    FlutterJNI jni = new FlutterJNI();
    attach(jni, PlatformViewsController2);

    // Simulate create call from the framework.
    createPlatformView(jni, PlatformViewsController2, platformViewId, "testType");
    assertTrue(PlatformViewsController2.initializePlatformViewIfNeeded(platformViewId));

    assertNotNull(androidView.getParent());
    assertTrue(androidView.getParent() instanceof FlutterMutatorView);

    // Simulate dispose call from the framework.
    disposePlatformView(jni, PlatformViewsController2, platformViewId);
    assertNull(androidView.getParent());

    // Simulate create call from the framework.
    createPlatformView(jni, PlatformViewsController2, platformViewId, "testType");
    assertTrue(PlatformViewsController2.initializePlatformViewIfNeeded(platformViewId));

    assertNotNull(androidView.getParent());
    assertTrue(androidView.getParent() instanceof FlutterMutatorView);
    verify(platformView, times(1)).dispose();
  }

  @Test
  @Config(shadows = {ShadowFlutterJNI.class, ShadowPlatformTaskQueue.class})
  public void disposeNullAndroidView() {
    PlatformViewRegistryImpl registryImpl = new PlatformViewRegistryImpl();
    PlatformViewsController2 PlatformViewsController2 = new PlatformViewsController2();
    PlatformViewsController2.setRegistry(registryImpl);

    int platformViewId = 0;
    assertNull(PlatformViewsController2.getPlatformViewById(platformViewId));

    PlatformViewFactory viewFactory = mock(PlatformViewFactory.class);
    PlatformView platformView = mock(PlatformView.class);

    View androidView = mock(View.class);
    when(platformView.getView()).thenReturn(androidView);
    when(viewFactory.create(any(), eq(platformViewId), any())).thenReturn(platformView);
    PlatformViewsController2.getRegistry().registerViewFactory("testType", viewFactory);

    FlutterJNI jni = new FlutterJNI();
    attach(jni, PlatformViewsController2);

    // Simulate create call from the framework.
    createPlatformView(jni, PlatformViewsController2, platformViewId, "testType");
    assertTrue(PlatformViewsController2.initializePlatformViewIfNeeded(platformViewId));

    when(platformView.getView()).thenReturn(null);

    // Simulate dispose call from the framework.
    disposePlatformView(jni, PlatformViewsController2, platformViewId);
    verify(platformView, times(1)).dispose();
  }

  private static ByteBuffer encodeMethodCall(MethodCall call) {
    final ByteBuffer buffer = StandardMethodCodec.INSTANCE.encodeMethodCall(call);
    buffer.rewind();
    return buffer;
  }

  private static void createPlatformView(
      FlutterJNI jni,
      PlatformViewsController2 PlatformViewsController2,
      int platformViewId,
      String viewType) {
    final Map<String, Object> args = new HashMap<>();
    args.put("id", platformViewId);
    args.put("viewType", viewType);
    args.put("direction", 0);
    args.put("width", 1.0);
    args.put("height", 1.0);

    final MethodCall platformCreateMethodCall = new MethodCall("create", args);

    jni.handlePlatformMessage(
        "flutter/platform_views_2",
        encodeMethodCall(platformCreateMethodCall),
        /*replyId=*/ 0,
        /*messageData=*/ 0);
  }

  private static void setLayoutDirection(
      FlutterJNI jni,
      PlatformViewsController2 PlatformViewsController2,
      int platformViewId,
      int direction) {
    final Map<String, Object> args = new HashMap<>();
    args.put("id", platformViewId);
    args.put("direction", direction);

    final MethodCall platformSetDirectionMethodCall = new MethodCall("setDirection", args);

    jni.handlePlatformMessage(
        "flutter/platform_views_2",
        encodeMethodCall(platformSetDirectionMethodCall),
        /*replyId=*/ 0,
        /*messageData=*/ 0);
  }

  private static void disposePlatformView(
      FlutterJNI jni, PlatformViewsController2 PlatformViewsController2, int platformViewId) {

    final Map<String, Object> args = new HashMap<>();
    args.put("id", platformViewId);

    final MethodCall platformDisposeMethodCall = new MethodCall("dispose", args);

    jni.handlePlatformMessage(
        "flutter/platform_views_2",
        encodeMethodCall(platformDisposeMethodCall),
        /*replyId=*/ 0,
        /*messageData=*/ 0);
  }

  private static void synchronizeToNativeViewHierarchy(
      FlutterJNI jni, PlatformViewsController2 PlatformViewsController2, boolean yes) {

    final MethodCall convertMethodCall = new MethodCall("synchronizeToNativeViewHierarchy", yes);

    jni.handlePlatformMessage(
        "flutter/platform_views_2",
        encodeMethodCall(convertMethodCall),
        /*replyId=*/ 0,
        /*messageData=*/ 0);
  }

  private static FlutterView attach(
      FlutterJNI jni, PlatformViewsController2 PlatformViewsController2) {
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
    attachToFlutterView(jni, PlatformViewsController2, flutterView);
    return flutterView;
  }

  private static void attachToFlutterView(
      FlutterJNI jni, PlatformViewsController2 PlatformViewsController2, FlutterView flutterView) {
    final DartExecutor executor = new DartExecutor(jni, mock(AssetManager.class));
    executor.onAttachedToJNI();

    final Context context = ApplicationProvider.getApplicationContext();
    PlatformViewsController2.attach(context, executor);

    PlatformViewsController oldController = new PlatformViewsController();

    final FlutterEngine engine = mock(FlutterEngine.class);
    when(engine.getRenderer()).thenReturn(new FlutterRenderer(jni));
    when(engine.getMouseCursorChannel()).thenReturn(mock(MouseCursorChannel.class));
    when(engine.getTextInputChannel()).thenReturn(mock(TextInputChannel.class));
    when(engine.getSettingsChannel()).thenReturn(new SettingsChannel(executor));
    when(engine.getScribeChannel()).thenReturn(mock(ScribeChannel.class));
    when(engine.getPlatformViewsController2()).thenReturn(PlatformViewsController2);
    when(engine.getPlatformViewsController()).thenReturn(oldController);
    when(engine.getLocalizationPlugin()).thenReturn(mock(LocalizationPlugin.class));
    when(engine.getAccessibilityChannel()).thenReturn(mock(AccessibilityChannel.class));
    when(engine.getDartExecutor()).thenReturn(executor);

    flutterView.attachToFlutterEngine(engine);
    PlatformViewsController2.attachToView(flutterView);
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
