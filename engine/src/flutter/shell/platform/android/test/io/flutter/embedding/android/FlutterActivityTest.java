// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.android;

import static io.flutter.Build.API_LEVELS;
import static io.flutter.embedding.android.FlutterActivityLaunchConfigs.EXTRA_CACHED_ENGINE_ID;
import static io.flutter.embedding.android.FlutterActivityLaunchConfigs.HANDLE_DEEPLINKING_META_DATA_KEY;
import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.doAnswer;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import android.annotation.SuppressLint;
import android.annotation.TargetApi;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.window.BackEvent;
import android.window.OnBackAnimationCallback;
import android.window.OnBackInvokedCallback;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.RequiresApi;
import androidx.lifecycle.DefaultLifecycleObserver;
import androidx.lifecycle.LifecycleOwner;
import androidx.test.core.app.ActivityScenario;
import androidx.test.core.app.ApplicationProvider;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.FlutterInjector;
import io.flutter.embedding.android.FlutterActivityLaunchConfigs.BackgroundMode;
import io.flutter.embedding.engine.FlutterEngine;
import io.flutter.embedding.engine.FlutterEngineCache;
import io.flutter.embedding.engine.FlutterJNI;
import io.flutter.embedding.engine.loader.FlutterLoader;
import io.flutter.embedding.engine.plugins.FlutterPlugin;
import io.flutter.embedding.engine.plugins.activity.ActivityAware;
import io.flutter.embedding.engine.plugins.activity.ActivityPluginBinding;
import io.flutter.embedding.engine.plugins.activity.ActivityPluginBinding.OnSaveInstanceStateListener;
import io.flutter.plugins.GeneratedPluginRegistrant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.Robolectric;
import org.robolectric.android.controller.ActivityController;
import org.robolectric.annotation.Config;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
public class FlutterActivityTest {
  private final Context ctx = ApplicationProvider.getApplicationContext();
  boolean isDelegateAttached;

  @Before
  public void setUp() {
    FlutterInjector.reset();
    GeneratedPluginRegistrant.clearRegisteredEngines();
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    when(mockFlutterJNI.isAttached()).thenReturn(true);
    FlutterJNI.Factory mockFlutterJNIFactory = mock(FlutterJNI.Factory.class);
    when(mockFlutterJNIFactory.provideFlutterJNI()).thenReturn(mockFlutterJNI);
    FlutterInjector.setInstance(
        new FlutterInjector.Builder().setFlutterJNIFactory(mockFlutterJNIFactory).build());
  }

  @After
  public void tearDown() {
    GeneratedPluginRegistrant.clearRegisteredEngines();
    FlutterInjector.reset();
  }

  @Test
  public void flutterViewHasId() {
    Intent intent = FlutterActivity.createDefaultIntent(ctx);
    ActivityController<FlutterActivity> activityController =
        Robolectric.buildActivity(FlutterActivity.class, intent);
    FlutterActivity activity = activityController.get();

    activity.onCreate(null);
    assertNotNull(activity.findViewById(FlutterActivity.FLUTTER_VIEW_ID));
    assertTrue(activity.findViewById(FlutterActivity.FLUTTER_VIEW_ID) instanceof FlutterView);
  }

  @Test
  @Config(minSdk = API_LEVELS.API_34)
  @TargetApi(API_LEVELS.API_34)
  public void whenUsingCachedEngine_predictiveBackStateIsSaved() {
    FlutterLoader mockFlutterLoader = mock(FlutterLoader.class);
    FlutterJNI mockFlutterJni = mock(FlutterJNI.class);
    when(mockFlutterJni.isAttached()).thenReturn(true);
    FlutterEngine cachedEngine = new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni);
    FlutterEngineCache.getInstance().put("my_cached_engine", cachedEngine);

    ActivityScenario<FlutterActivity> flutterActivityScenario =
        ActivityScenario.launch(FlutterActivity.class);

    // Set to framework handling and then recreate the activity and check the state is preserved.
    flutterActivityScenario.onActivity(activity -> activity.setFrameworkHandlesBack(true));
    flutterActivityScenario.onActivity(
        activity -> activity.getIntent().putExtra(EXTRA_CACHED_ENGINE_ID, "my_cached_engine"));

    flutterActivityScenario.recreate();
    flutterActivityScenario.onActivity(activity -> assertTrue(activity.hasRegisteredBackCallback));

    // Clean up.
    flutterActivityScenario.close();
  }

  @Test
  @Config(minSdk = API_LEVELS.API_34)
  @TargetApi(API_LEVELS.API_34)
  public void whenNotUsingCachedEngine_predictiveBackStateIsNotSaved() {
    ActivityScenario<FlutterActivity> flutterActivityScenario =
        ActivityScenario.launch(FlutterActivity.class);

    // Set to framework handling and then recreate the activity and check the state is preserved.
    flutterActivityScenario.onActivity(activity -> activity.setFrameworkHandlesBack(true));

    flutterActivityScenario.recreate();
    flutterActivityScenario.onActivity(activity -> assertFalse(activity.hasRegisteredBackCallback));

    // Clean up.
    flutterActivityScenario.close();
  }

  // TODO(garyq): Robolectric does not yet support android api 33 yet. Switch to a robolectric
  // test that directly exercises the OnBackInvoked APIs when API 33 is supported.
  @Test
  @TargetApi(API_LEVELS.API_33)
  public void itRegistersOnBackInvokedCallbackOnChangingFrameworkHandlesBack() {
    Intent intent = FlutterActivityWithReportFullyDrawn.createDefaultIntent(ctx);
    ActivityController<FlutterActivityWithReportFullyDrawn> activityController =
        Robolectric.buildActivity(FlutterActivityWithReportFullyDrawn.class, intent);
    FlutterActivityWithReportFullyDrawn activity = spy(activityController.get());

    activity.onCreate(null);

    verify(activity, times(0)).registerOnBackInvokedCallback();

    activity.setFrameworkHandlesBack(false);
    verify(activity, times(0)).registerOnBackInvokedCallback();

    activity.setFrameworkHandlesBack(true);
    verify(activity, times(1)).registerOnBackInvokedCallback();
  }

  // TODO(garyq): Robolectric does not yet support android api 33 yet. Switch to a robolectric
  // test that directly exercises the OnBackInvoked APIs when API 33 is supported.
  @Test
  @TargetApi(API_LEVELS.API_33)
  public void itUnregistersOnBackInvokedCallbackOnRelease() {
    Intent intent = FlutterActivityWithReportFullyDrawn.createDefaultIntent(ctx);
    ActivityController<FlutterActivityWithReportFullyDrawn> activityController =
        Robolectric.buildActivity(FlutterActivityWithReportFullyDrawn.class, intent);
    FlutterActivityWithReportFullyDrawn activity = spy(activityController.get());

    activity.release();

    verify(activity, times(1)).unregisterOnBackInvokedCallback();
  }

  @Test
  @Config(sdk = API_LEVELS.API_32)
  public void onBackInvokedCallbackIsNullForSdk32OrLower() {
    Intent intent = FlutterActivity.createDefaultIntent(ctx);
    ActivityController<FlutterActivity> activityController =
        Robolectric.buildActivity(FlutterActivity.class, intent);
    FlutterActivity flutterActivity = activityController.get();

    assertNull(
        "onBackInvokedCallback should be null for SDK 32 or lower",
        flutterActivity.getOnBackInvokedCallback());
  }

  @Test
  @Config(sdk = API_LEVELS.API_33)
  @TargetApi(API_LEVELS.API_33)
  public void onBackInvokedCallbackCallsOnBackPressedForSdk33() {
    Intent intent = FlutterActivityWithMockBackInvokedHandling.createDefaultIntent(ctx);
    ActivityController<FlutterActivityWithMockBackInvokedHandling> activityController =
        Robolectric.buildActivity(FlutterActivityWithMockBackInvokedHandling.class, intent);
    FlutterActivityWithMockBackInvokedHandling activity = activityController.get();

    OnBackInvokedCallback callback = activity.getOnBackInvokedCallback();
    assertNotNull("onBackInvokedCallback should not be null for SDK 33", callback);

    callback.onBackInvoked();
    assertEquals("Expected onBackPressed to be called 1 times", 1, activity.onBackPressedCounter);
  }

  @Test
  @Config(sdk = API_LEVELS.API_34)
  @TargetApi(API_LEVELS.API_34)
  public void itHandlesOnBackAnimationCallbackAsExpectedForSdk34OrHigher() {
    Intent intent = FlutterActivityWithMockBackInvokedHandling.createDefaultIntent(ctx);
    ActivityController<FlutterActivityWithMockBackInvokedHandling> activityController =
        Robolectric.buildActivity(FlutterActivityWithMockBackInvokedHandling.class, intent);
    FlutterActivityWithMockBackInvokedHandling activity = activityController.get();

    assertTrue(
        "onBackInvokedCallback should be an instance of OnBackAnimationCallback for SDK 34 or higher",
        activity.getOnBackInvokedCallback() instanceof OnBackAnimationCallback);

    OnBackAnimationCallback callback =
        (OnBackAnimationCallback) activity.getOnBackInvokedCallback();

    BackEvent mockBackEvent = mock(BackEvent.class);
    callback.onBackStarted(mockBackEvent);
    assertEquals(
        "Expected startBackGesture to be called 1 times", 1, activity.startBackGestureCounter);

    callback.onBackProgressed(mockBackEvent);
    assertEquals(
        "Expected updateBackGestureProgress to be called 1 times",
        1,
        activity.updateBackGestureProgressCounter);

    callback.onBackInvoked();
    assertEquals(
        "Expected commitBackGesture to be called 1 times", 1, activity.commitBackGestureCounter);

    callback.onBackCancelled();
    assertEquals(
        "Expected cancelBackGesture to be called 1 times", 1, activity.cancelBackGestureCounter);
  }

  @Test
  public void itCreatesDefaultIntentWithExpectedDefaults() {
    Intent intent = FlutterActivity.createDefaultIntent(ctx);
    ActivityController<FlutterActivity> activityController =
        Robolectric.buildActivity(FlutterActivity.class, intent);
    FlutterActivity flutterActivity = activityController.get();
    flutterActivity.setDelegate(new FlutterActivityAndFragmentDelegate(flutterActivity));

    assertEquals("main", flutterActivity.getDartEntrypointFunctionName());
    assertNull(flutterActivity.getDartEntrypointLibraryUri());
    assertNull(flutterActivity.getDartEntrypointArgs());
    assertEquals("/", flutterActivity.getInitialRoute());
    assertArrayEquals(new String[] {}, flutterActivity.getFlutterShellArgs().toArray());
    assertTrue(flutterActivity.shouldAttachEngineToActivity());
    assertNull(flutterActivity.getCachedEngineId());
    assertTrue(flutterActivity.shouldDestroyEngineWithHost());
    assertEquals(BackgroundMode.opaque, flutterActivity.getBackgroundMode());
    assertEquals(RenderMode.surface, flutterActivity.getRenderMode());
    assertEquals(TransparencyMode.opaque, flutterActivity.getTransparencyMode());
  }

  @Test
  public void itDestroysNewEngineWhenIntentIsMissingParameter() {
    // All clients should use the static members of FlutterActivity to construct an
    // Intent. Missing extras is an error. However, Flutter has number of tests that
    // don't seem to use the static members of FlutterActivity to construct the
    // launching Intent, so this test explicitly verifies that even illegal Intents
    // result in the automatic destruction of a non-cached FlutterEngine, which prevents
    // the breakage of memory usage benchmark tests.
    Intent intent = new Intent(ctx, FlutterActivity.class);
    ActivityController<FlutterActivity> activityController =
        Robolectric.buildActivity(FlutterActivity.class, intent);
    FlutterActivity flutterActivity = activityController.get();
    flutterActivity.setDelegate(new FlutterActivityAndFragmentDelegate(flutterActivity));

    assertTrue(flutterActivity.shouldDestroyEngineWithHost());
  }

  @Test
  public void itDoesNotDestroyFlutterEngineWhenProvidedByHost() {
    Intent intent = new Intent(ctx, FlutterActivityWithProvidedEngine.class);
    ActivityController<FlutterActivityWithProvidedEngine> activityController =
        Robolectric.buildActivity(FlutterActivityWithProvidedEngine.class, intent);
    activityController.create();
    FlutterActivityWithProvidedEngine flutterActivity = activityController.get();

    assertFalse(flutterActivity.shouldDestroyEngineWithHost());
  }

  @Test
  public void itCreatesNewEngineIntentWithRequestedSettings() {
    Intent intent =
        FlutterActivity.withNewEngine()
            .initialRoute("/custom/route")
            .dartEntrypointArgs(new ArrayList<String>(Arrays.asList("foo", "bar")))
            .backgroundMode(BackgroundMode.transparent)
            .build(ctx);
    ActivityController<FlutterActivity> activityController =
        Robolectric.buildActivity(FlutterActivity.class, intent);
    FlutterActivity flutterActivity = activityController.get();
    flutterActivity.setDelegate(new FlutterActivityAndFragmentDelegate(flutterActivity));

    assertEquals("/custom/route", flutterActivity.getInitialRoute());
    assertArrayEquals(
        new String[] {"foo", "bar"}, flutterActivity.getDartEntrypointArgs().toArray());
    assertArrayEquals(new String[] {}, flutterActivity.getFlutterShellArgs().toArray());
    assertTrue(flutterActivity.shouldAttachEngineToActivity());
    assertNull(flutterActivity.getCachedEngineId());
    assertTrue(flutterActivity.shouldDestroyEngineWithHost());
    assertEquals(BackgroundMode.transparent, flutterActivity.getBackgroundMode());
    assertEquals(RenderMode.texture, flutterActivity.getRenderMode());
    assertEquals(TransparencyMode.transparent, flutterActivity.getTransparencyMode());
  }

  @Test
  public void itCreatesNewEngineInGroupIntentWithRequestedSettings() {
    Intent intent =
        FlutterActivity.withNewEngineInGroup("my_cached_engine_group")
            .dartEntrypoint("custom_entrypoint")
            .initialRoute("/custom/route")
            .backgroundMode(BackgroundMode.transparent)
            .build(ctx);
    ActivityController<FlutterActivity> activityController =
        Robolectric.buildActivity(FlutterActivity.class, intent);
    FlutterActivity flutterActivity = activityController.get();
    flutterActivity.setDelegate(new FlutterActivityAndFragmentDelegate(flutterActivity));

    assertEquals("my_cached_engine_group", flutterActivity.getCachedEngineGroupId());
    assertEquals("custom_entrypoint", flutterActivity.getDartEntrypointFunctionName());
    assertEquals("/custom/route", flutterActivity.getInitialRoute());
    assertArrayEquals(new String[] {}, flutterActivity.getFlutterShellArgs().toArray());
    assertTrue(flutterActivity.shouldAttachEngineToActivity());
    assertTrue(flutterActivity.shouldDestroyEngineWithHost());
    assertNull(flutterActivity.getCachedEngineId());
    assertEquals(BackgroundMode.transparent, flutterActivity.getBackgroundMode());
    assertEquals(RenderMode.texture, flutterActivity.getRenderMode());
    assertEquals(TransparencyMode.transparent, flutterActivity.getTransparencyMode());
  }

  @Test
  public void itReturnsValueFromMetaDataWhenCallsShouldHandleDeepLinkingCase1()
      throws PackageManager.NameNotFoundException {
    Intent intent =
        FlutterActivity.withNewEngine().backgroundMode(BackgroundMode.transparent).build(ctx);
    ActivityController<FlutterActivity> activityController =
        Robolectric.buildActivity(FlutterActivity.class, intent);
    FlutterActivity flutterActivity = activityController.get();
    Bundle bundle = new Bundle();
    bundle.putBoolean(HANDLE_DEEPLINKING_META_DATA_KEY, true);
    FlutterActivity spyFlutterActivity = spy(flutterActivity);
    when(spyFlutterActivity.getMetaData()).thenReturn(bundle);
    assertTrue(spyFlutterActivity.shouldHandleDeeplinking());
  }

  @Test
  public void itReturnsValueFromMetaDataWhenCallsShouldHandleDeepLinkingCase2()
      throws PackageManager.NameNotFoundException {
    Intent intent =
        FlutterActivity.withNewEngine().backgroundMode(BackgroundMode.transparent).build(ctx);
    ActivityController<FlutterActivity> activityController =
        Robolectric.buildActivity(FlutterActivity.class, intent);
    FlutterActivity flutterActivity = activityController.get();
    Bundle bundle = new Bundle();
    bundle.putBoolean(HANDLE_DEEPLINKING_META_DATA_KEY, false);
    FlutterActivity spyFlutterActivity = spy(flutterActivity);
    when(spyFlutterActivity.getMetaData()).thenReturn(bundle);
    assertFalse(spyFlutterActivity.shouldHandleDeeplinking());
  }

  @Test
  public void itReturnsValueFromMetaDataWhenCallsShouldHandleDeepLinkingCase3()
      throws PackageManager.NameNotFoundException {
    Intent intent =
        FlutterActivity.withNewEngine().backgroundMode(BackgroundMode.transparent).build(ctx);
    ActivityController<FlutterActivity> activityController =
        Robolectric.buildActivity(FlutterActivity.class, intent);
    FlutterActivity flutterActivity = activityController.get();
    // Creates an empty bundle.
    Bundle bundle = new Bundle();
    FlutterActivity spyFlutterActivity = spy(flutterActivity);
    when(spyFlutterActivity.getMetaData()).thenReturn(bundle);
    // Empty bundle should return true.
    assertTrue(spyFlutterActivity.shouldHandleDeeplinking());
  }

  @Test
  public void itCreatesCachedEngineIntentThatDoesNotDestroyTheEngine() {
    Intent intent =
        FlutterActivity.withCachedEngine("my_cached_engine")
            .destroyEngineWithActivity(false)
            .build(ctx);
    ActivityController<FlutterActivity> activityController =
        Robolectric.buildActivity(FlutterActivity.class, intent);
    FlutterActivity flutterActivity = activityController.get();

    assertArrayEquals(new String[] {}, flutterActivity.getFlutterShellArgs().toArray());
    assertTrue(flutterActivity.shouldAttachEngineToActivity());
    assertEquals("my_cached_engine", flutterActivity.getCachedEngineId());
    assertFalse(flutterActivity.shouldDestroyEngineWithHost());
  }

  @Test
  public void itCreatesCachedEngineIntentThatDestroysTheEngine() {
    Intent intent =
        FlutterActivity.withCachedEngine("my_cached_engine")
            .destroyEngineWithActivity(true)
            .build(ctx);
    ActivityController<FlutterActivity> activityController =
        Robolectric.buildActivity(FlutterActivity.class, intent);
    FlutterActivity flutterActivity = activityController.get();

    assertArrayEquals(new String[] {}, flutterActivity.getFlutterShellArgs().toArray());
    assertTrue(flutterActivity.shouldAttachEngineToActivity());
    assertEquals("my_cached_engine", flutterActivity.getCachedEngineId());
    assertTrue(flutterActivity.shouldDestroyEngineWithHost());
  }

  @Test
  public void itRegistersPluginsAtConfigurationTime() {
    Intent intent = FlutterActivity.createDefaultIntent(ctx);
    ActivityController<FlutterActivity> activityController =
        Robolectric.buildActivity(FlutterActivity.class, intent);
    FlutterActivity activity = activityController.get();

    // This calls onAttach on FlutterActivityAndFragmentDelegate and subsequently
    // configureFlutterEngine which registers the plugins.
    activity.onCreate(null);

    List<FlutterEngine> registeredEngines = GeneratedPluginRegistrant.getRegisteredEngines();
    assertEquals(1, registeredEngines.size());
    assertEquals(activity.getFlutterEngine(), registeredEngines.get(0));
  }

  @Test
  public void itCanBeDetachedFromTheEngineAndStopSendingFurtherEvents() {
    FlutterActivityAndFragmentDelegate mockDelegate =
        mock(FlutterActivityAndFragmentDelegate.class);
    FlutterEngine mockEngine = mock(FlutterEngine.class);
    FlutterEngineCache.getInstance().put("my_cached_engine", mockEngine);

    Intent intent = FlutterActivity.withCachedEngine("my_cached_engine").build(ctx);
    ActivityController<FlutterActivity> activityController =
        Robolectric.buildActivity(FlutterActivity.class, intent);
    FlutterActivity flutterActivity = activityController.get();

    isDelegateAttached = true;
    when(mockDelegate.isAttached()).thenAnswer(invocation -> isDelegateAttached);
    doAnswer(invocation -> isDelegateAttached = false).when(mockDelegate).onDetach();

    flutterActivity.setDelegate(mockDelegate);
    flutterActivity.onStart();
    flutterActivity.onResume();

    verify(mockDelegate, times(1)).onStart();
    verify(mockDelegate, times(1)).onResume();

    flutterActivity.onPause();
    flutterActivity.detachFromFlutterEngine();
    verify(mockDelegate, times(1)).onPause();
    verify(mockDelegate, times(1)).onDestroyView();
    verify(mockDelegate, times(1)).onDetach();

    flutterActivity.onStop();
    verify(mockDelegate, never()).onStop();

    // Simulate the disconnected activity resuming again.
    flutterActivity.onStart();
    flutterActivity.onResume();
    // Shouldn't send more events to the delegates as before and shouldn't crash.
    verify(mockDelegate, times(1)).onStart();
    verify(mockDelegate, times(1)).onResume();

    flutterActivity.onDestroy();
    // 1 time same as before.
    verify(mockDelegate, times(1)).onDestroyView();
    verify(mockDelegate, times(1)).onDetach();
    verify(mockDelegate, times(1)).release();
  }

  @Test
  public void itReturnsExclusiveAppComponent() {
    Intent intent = FlutterActivity.createDefaultIntent(ctx);
    ActivityController<FlutterActivity> activityController =
        Robolectric.buildActivity(FlutterActivity.class, intent);
    FlutterActivity flutterActivity = activityController.get();
    FlutterActivityAndFragmentDelegate delegate =
        new FlutterActivityAndFragmentDelegate(flutterActivity);
    flutterActivity.setDelegate(delegate);

    assertEquals(flutterActivity.getExclusiveAppComponent(), delegate);
  }

  @Test
  public void itDelaysDrawing() {
    Intent intent = FlutterActivity.createDefaultIntent(ctx);
    ActivityController<FlutterActivity> activityController =
        Robolectric.buildActivity(FlutterActivity.class, intent);
    FlutterActivity flutterActivity = activityController.get();

    flutterActivity.onCreate(null);

    assertNotNull(flutterActivity.delegate.activePreDrawListener);
  }

  @Test
  public void itDoesNotDelayDrawingwhenUsingTextureRendering() {
    Intent intent = FlutterActivityWithTextureRendering.createDefaultIntent(ctx);
    ActivityController<FlutterActivityWithTextureRendering> activityController =
        Robolectric.buildActivity(FlutterActivityWithTextureRendering.class, intent);
    FlutterActivityWithTextureRendering flutterActivity = activityController.get();

    flutterActivity.onCreate(null);

    assertNull(flutterActivity.delegate.activePreDrawListener);
  }

  @Test
  public void itRestoresPluginStateBeforePluginOnCreate() {
    FlutterLoader mockFlutterLoader = mock(FlutterLoader.class);
    FlutterJNI mockFlutterJni = mock(FlutterJNI.class);
    when(mockFlutterJni.isAttached()).thenReturn(true);
    FlutterEngine cachedEngine = new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni);
    FakeFlutterPlugin fakeFlutterPlugin = new FakeFlutterPlugin();
    cachedEngine.getPlugins().add(fakeFlutterPlugin);
    FlutterEngineCache.getInstance().put("my_cached_engine", cachedEngine);

    Intent intent = FlutterActivity.withCachedEngine("my_cached_engine").build(ctx);
    Robolectric.buildActivity(FlutterActivity.class, intent).setup();
    assertTrue(
        "Expected FakeFlutterPlugin onCreateCalled to be true", fakeFlutterPlugin.onCreateCalled);
  }

  @Test
  public void itDoesNotRegisterPluginsTwiceWhenUsingACachedEngine() {
    Intent intent = new Intent(ctx, FlutterActivityWithProvidedEngine.class);
    ActivityController<FlutterActivityWithProvidedEngine> activityController =
        Robolectric.buildActivity(FlutterActivityWithProvidedEngine.class, intent);
    activityController.create();
    FlutterActivityWithProvidedEngine flutterActivity = activityController.get();
    flutterActivity.configureFlutterEngine(flutterActivity.getFlutterEngine());

    List<FlutterEngine> registeredEngines = GeneratedPluginRegistrant.getRegisteredEngines();
    // This might cause the plugins to be registered twice, once by the FlutterEngine constructor,
    // and once by the default FlutterActivity.configureFlutterEngine implementation.
    // Test that it doesn't happen.
    assertEquals(1, registeredEngines.size());
  }

  @Test
  public void itDoesNotReleaseEnginewhenDetachFromFlutterEngine() {
    FlutterActivityAndFragmentDelegate mockDelegate =
        mock(FlutterActivityAndFragmentDelegate.class);
    isDelegateAttached = true;
    when(mockDelegate.isAttached()).thenAnswer(invocation -> isDelegateAttached);
    doAnswer(invocation -> isDelegateAttached = false).when(mockDelegate).onDetach();
    FlutterEngine mockEngine = mock(FlutterEngine.class);
    FlutterEngineCache.getInstance().put("my_cached_engine", mockEngine);

    Intent intent = FlutterActivity.withCachedEngine("my_cached_engine").build(ctx);
    ActivityController<FlutterActivity> activityController =
        Robolectric.buildActivity(FlutterActivity.class, intent);
    FlutterActivity flutterActivity = activityController.get();

    flutterActivity.setDelegate(mockDelegate);
    flutterActivity.onStart();
    flutterActivity.onResume();
    flutterActivity.onPause();

    assertTrue(mockDelegate.isAttached());
    flutterActivity.detachFromFlutterEngine();
    verify(mockDelegate, times(1)).onDetach();
    verify(mockDelegate, never()).release();
    assertFalse(mockDelegate.isAttached());
  }

  @Test
  public void itReleaseEngineWhenOnDestroy() {
    FlutterActivityAndFragmentDelegate mockDelegate =
        mock(FlutterActivityAndFragmentDelegate.class);
    isDelegateAttached = true;
    when(mockDelegate.isAttached()).thenAnswer(invocation -> isDelegateAttached);
    doAnswer(invocation -> isDelegateAttached = false).when(mockDelegate).onDetach();
    FlutterEngine mockEngine = mock(FlutterEngine.class);
    FlutterEngineCache.getInstance().put("my_cached_engine", mockEngine);

    Intent intent = FlutterActivity.withCachedEngine("my_cached_engine").build(ctx);
    ActivityController<FlutterActivity> activityController =
        Robolectric.buildActivity(FlutterActivity.class, intent);
    FlutterActivity flutterActivity = activityController.get();

    flutterActivity.setDelegate(mockDelegate);
    flutterActivity.onStart();
    flutterActivity.onResume();
    flutterActivity.onPause();

    assertTrue(mockDelegate.isAttached());
    flutterActivity.onDestroy();
    verify(mockDelegate, times(1)).onDetach();
    verify(mockDelegate, times(1)).release();
    assertFalse(mockDelegate.isAttached());
  }

  @Test
  @Config(minSdk = API_LEVELS.API_21, maxSdk = API_LEVELS.API_28)
  public void fullyDrawn_beforeAndroidQ() {
    Intent intent = FlutterActivityWithReportFullyDrawn.createDefaultIntent(ctx);
    ActivityController<FlutterActivityWithReportFullyDrawn> activityController =
        Robolectric.buildActivity(FlutterActivityWithReportFullyDrawn.class, intent);
    FlutterActivityWithReportFullyDrawn flutterActivity = activityController.get();

    // See https://github.com/flutter/flutter/issues/46172, and
    // https://github.com/flutter/flutter/issues/88767.
    flutterActivity.onFlutterUiDisplayed();
    assertFalse("reportFullyDrawn isn't used", flutterActivity.isFullyDrawn());
  }

  @Test
  @Config(minSdk = API_LEVELS.API_29)
  public void fullyDrawn_fromAndroidQ() {
    Intent intent = FlutterActivityWithReportFullyDrawn.createDefaultIntent(ctx);
    ActivityController<FlutterActivityWithReportFullyDrawn> activityController =
        Robolectric.buildActivity(FlutterActivityWithReportFullyDrawn.class, intent);
    FlutterActivityWithReportFullyDrawn flutterActivity = activityController.get();

    flutterActivity.onFlutterUiDisplayed();
    assertTrue("reportFullyDrawn is used", flutterActivity.isFullyDrawn());
    flutterActivity.resetFullyDrawn();
  }

  static class FlutterActivityWithProvidedEngine extends FlutterActivity {
    @Override
    @SuppressLint("MissingSuperCall")
    protected void onCreate(@Nullable Bundle savedInstanceState) {
      super.delegate = new FlutterActivityAndFragmentDelegate(this);
      super.delegate.setUpFlutterEngine();
    }

    @Nullable
    @Override
    public FlutterEngine provideFlutterEngine(@NonNull Context context) {
      FlutterJNI flutterJNI = mock(FlutterJNI.class);
      FlutterLoader flutterLoader = mock(FlutterLoader.class);
      when(flutterJNI.isAttached()).thenReturn(true);
      when(flutterLoader.automaticallyRegisterPlugins()).thenReturn(true);

      return new FlutterEngine(context, flutterLoader, flutterJNI, new String[] {}, true);
    }
  }

  // This is just a compile time check to ensure that it's possible for FlutterActivity subclasses
  // to provide their own intent builders which builds their own runtime types.
  static class FlutterActivityWithIntentBuilders extends FlutterActivity {

    public static NewEngineIntentBuilder withNewEngine() {
      return new NewEngineIntentBuilder(FlutterActivityWithIntentBuilders.class);
    }

    public static CachedEngineIntentBuilder withCachedEngine(@NonNull String cachedEngineId) {
      return new CachedEngineIntentBuilder(FlutterActivityWithIntentBuilders.class, cachedEngineId);
    }
  }

  private static class FlutterActivityWithTextureRendering extends FlutterActivity {
    @Override
    public RenderMode getRenderMode() {
      return RenderMode.texture;
    }
  }

  private static class FlutterActivityWithReportFullyDrawn extends FlutterActivity {
    private boolean fullyDrawn = false;

    @Override
    public void reportFullyDrawn() {
      fullyDrawn = true;
    }

    public boolean isFullyDrawn() {
      return fullyDrawn;
    }

    public void resetFullyDrawn() {
      fullyDrawn = false;
    }
  }

  private static class FlutterActivityWithMockBackInvokedHandling extends FlutterActivity {

    int onBackPressedCounter = 0;
    int startBackGestureCounter = 0;
    int updateBackGestureProgressCounter = 0;
    int commitBackGestureCounter = 0;
    int cancelBackGestureCounter = 0;

    @Override
    public void onBackPressed() {
      onBackPressedCounter++;
    }

    @TargetApi(API_LEVELS.API_34)
    @RequiresApi(API_LEVELS.API_34)
    @Override
    public void startBackGesture(@NonNull BackEvent backEvent) {
      startBackGestureCounter++;
    }

    @TargetApi(API_LEVELS.API_34)
    @RequiresApi(API_LEVELS.API_34)
    @Override
    public void updateBackGestureProgress(@NonNull BackEvent backEvent) {
      updateBackGestureProgressCounter++;
    }

    @TargetApi(API_LEVELS.API_34)
    @RequiresApi(API_LEVELS.API_34)
    @Override
    public void commitBackGesture() {
      commitBackGestureCounter++;
    }

    @TargetApi(API_LEVELS.API_34)
    @RequiresApi(API_LEVELS.API_34)
    @Override
    public void cancelBackGesture() {
      cancelBackGestureCounter++;
    }
  }

  private static final class FakeFlutterPlugin
      implements FlutterPlugin,
          ActivityAware,
          OnSaveInstanceStateListener,
          DefaultLifecycleObserver {

    private ActivityPluginBinding activityPluginBinding;
    private boolean stateRestored = false;
    private boolean onCreateCalled = false;

    @Override
    public void onAttachedToEngine(@NonNull FlutterPluginBinding binding) {}

    @Override
    public void onDetachedFromEngine(@NonNull FlutterPluginBinding binding) {}

    @Override
    public void onAttachedToActivity(@NonNull ActivityPluginBinding binding) {
      activityPluginBinding = binding;
      binding.addOnSaveStateListener(this);
      ((FlutterActivity) binding.getActivity()).getLifecycle().addObserver(this);
    }

    @Override
    public void onDetachedFromActivityForConfigChanges() {
      onDetachedFromActivity();
    }

    @Override
    public void onReattachedToActivityForConfigChanges(@NonNull ActivityPluginBinding binding) {
      onAttachedToActivity(binding);
    }

    @Override
    public void onDetachedFromActivity() {
      ((FlutterActivity) activityPluginBinding.getActivity()).getLifecycle().removeObserver(this);
      activityPluginBinding.removeOnSaveStateListener(this);
      activityPluginBinding = null;
    }

    @Override
    public void onSaveInstanceState(@NonNull Bundle bundle) {}

    @Override
    public void onRestoreInstanceState(@Nullable Bundle bundle) {
      stateRestored = true;
    }

    @Override
    public void onCreate(@NonNull LifecycleOwner lifecycleOwner) {
      assertTrue("State was restored before onCreate", stateRestored);
      onCreateCalled = true;
    }
  }
}
