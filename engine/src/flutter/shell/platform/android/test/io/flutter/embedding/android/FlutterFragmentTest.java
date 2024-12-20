// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.android;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.doAnswer;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import android.content.Context;
import androidx.activity.OnBackPressedCallback;
import androidx.fragment.app.FragmentActivity;
import androidx.test.core.app.ApplicationProvider;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.embedding.engine.FlutterEngine;
import io.flutter.embedding.engine.FlutterEngineCache;
import io.flutter.embedding.engine.FlutterJNI;
import io.flutter.embedding.engine.loader.FlutterLoader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.Robolectric;
import org.robolectric.annotation.Config;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
public class FlutterFragmentTest {
  private final Context ctx = ApplicationProvider.getApplicationContext();
  boolean isDelegateAttached;

  class TestDelegateFactory implements FlutterActivityAndFragmentDelegate.DelegateFactory {
    FlutterActivityAndFragmentDelegate delegate;

    TestDelegateFactory(FlutterActivityAndFragmentDelegate delegate) {
      this.delegate = delegate;
    }

    public FlutterActivityAndFragmentDelegate createDelegate(
        FlutterActivityAndFragmentDelegate.Host host) {
      return delegate;
    }
  }

  @Test
  public void itCreatesDefaultFragmentWithExpectedDefaults() {
    FlutterFragment fragment = FlutterFragment.createDefault();
    TestDelegateFactory delegateFactory =
        new TestDelegateFactory(new FlutterActivityAndFragmentDelegate(fragment));
    fragment.setDelegateFactory(delegateFactory);

    assertEquals("main", fragment.getDartEntrypointFunctionName());
    assertNull(fragment.getDartEntrypointLibraryUri());
    assertNull(fragment.getDartEntrypointArgs());
    assertEquals("/", fragment.getInitialRoute());
    assertArrayEquals(new String[] {}, fragment.getFlutterShellArgs().toArray());
    assertTrue(fragment.shouldAttachEngineToActivity());
    assertFalse(fragment.shouldHandleDeeplinking());
    assertNull(fragment.getCachedEngineId());
    assertTrue(fragment.shouldDestroyEngineWithHost());
    assertEquals(RenderMode.surface, fragment.getRenderMode());
    assertEquals(TransparencyMode.transparent, fragment.getTransparencyMode());
    assertFalse(fragment.shouldDelayFirstAndroidViewDraw());
  }

  @Test
  public void itCreatesNewEngineFragmentWithRequestedSettings() {
    FlutterFragment fragment =
        FlutterFragment.withNewEngine()
            .dartEntrypoint("custom_entrypoint")
            .dartLibraryUri("package:foo/bar.dart")
            .dartEntrypointArgs(new ArrayList<String>(Arrays.asList("foo", "bar")))
            .initialRoute("/custom/route")
            .shouldAttachEngineToActivity(false)
            .handleDeeplinking(true)
            .renderMode(RenderMode.texture)
            .transparencyMode(TransparencyMode.opaque)
            .build();
    TestDelegateFactory delegateFactory =
        new TestDelegateFactory(new FlutterActivityAndFragmentDelegate(fragment));
    fragment.setDelegateFactory(delegateFactory);

    assertEquals("custom_entrypoint", fragment.getDartEntrypointFunctionName());
    assertEquals("package:foo/bar.dart", fragment.getDartEntrypointLibraryUri());
    assertEquals("/custom/route", fragment.getInitialRoute());
    assertArrayEquals(new String[] {"foo", "bar"}, fragment.getDartEntrypointArgs().toArray());
    assertArrayEquals(new String[] {}, fragment.getFlutterShellArgs().toArray());
    assertFalse(fragment.shouldAttachEngineToActivity());
    assertTrue(fragment.shouldHandleDeeplinking());
    assertNull(fragment.getCachedEngineId());
    assertTrue(fragment.shouldDestroyEngineWithHost());
    assertEquals(RenderMode.texture, fragment.getRenderMode());
    assertEquals(TransparencyMode.opaque, fragment.getTransparencyMode());
  }

  @Test
  public void itCreatesNewEngineInGroupFragmentWithRequestedSettings() {
    FlutterFragment fragment =
        FlutterFragment.withNewEngineInGroup("my_cached_engine_group")
            .dartEntrypoint("custom_entrypoint")
            .initialRoute("/custom/route")
            .shouldAttachEngineToActivity(false)
            .handleDeeplinking(true)
            .renderMode(RenderMode.texture)
            .transparencyMode(TransparencyMode.opaque)
            .build();

    TestDelegateFactory delegateFactory =
        new TestDelegateFactory(new FlutterActivityAndFragmentDelegate(fragment));

    fragment.setDelegateFactory(delegateFactory);

    assertEquals("my_cached_engine_group", fragment.getCachedEngineGroupId());
    assertEquals("custom_entrypoint", fragment.getDartEntrypointFunctionName());
    assertEquals("/custom/route", fragment.getInitialRoute());
    assertArrayEquals(new String[] {}, fragment.getFlutterShellArgs().toArray());
    assertFalse(fragment.shouldAttachEngineToActivity());
    assertTrue(fragment.shouldHandleDeeplinking());
    assertNull(fragment.getCachedEngineId());
    assertTrue(fragment.shouldDestroyEngineWithHost());
    assertEquals(RenderMode.texture, fragment.getRenderMode());
    assertEquals(TransparencyMode.opaque, fragment.getTransparencyMode());
  }

  @Test
  public void itCreatesNewEngineFragmentThatDelaysFirstDrawWhenRequested() {
    FlutterFragment fragment =
        FlutterFragment.withNewEngine().shouldDelayFirstAndroidViewDraw(true).build();

    assertNotNull(fragment.shouldDelayFirstAndroidViewDraw());
  }

  @Test
  public void itCreatesCachedEngineFragmentWithExpectedDefaults() {
    FlutterFragment fragment = FlutterFragment.withCachedEngine("my_cached_engine").build();

    assertTrue(fragment.shouldAttachEngineToActivity());
    assertEquals("my_cached_engine", fragment.getCachedEngineId());
    assertFalse(fragment.shouldDestroyEngineWithHost());
    assertFalse(fragment.shouldDelayFirstAndroidViewDraw());
  }

  @Test
  public void itCreatesCachedEngineFragmentThatDestroysTheEngine() {
    FlutterFragment fragment =
        FlutterFragment.withCachedEngine("my_cached_engine")
            .destroyEngineWithFragment(true)
            .build();

    assertTrue(fragment.shouldAttachEngineToActivity());
    assertEquals("my_cached_engine", fragment.getCachedEngineId());
    assertTrue(fragment.shouldDestroyEngineWithHost());
  }

  @Test
  public void itCreatesCachedEngineFragmentThatDelaysFirstDrawWhenRequested() {
    FlutterFragment fragment =
        FlutterFragment.withCachedEngine("my_cached_engine")
            .shouldDelayFirstAndroidViewDraw(true)
            .build();

    assertNotNull(fragment.shouldDelayFirstAndroidViewDraw());
  }

  @Test
  public void itCanBeDetachedFromTheEngineAndStopSendingFurtherEvents() {
    FlutterActivityAndFragmentDelegate mockDelegate =
        mock(FlutterActivityAndFragmentDelegate.class);
    TestDelegateFactory delegateFactory = new TestDelegateFactory(mockDelegate);
    FlutterFragment fragment =
        FlutterFragment.withCachedEngine("my_cached_engine")
            .destroyEngineWithFragment(true)
            .build();

    isDelegateAttached = true;
    when(mockDelegate.isAttached()).thenAnswer(invocation -> isDelegateAttached);
    doAnswer(invocation -> isDelegateAttached = false).when(mockDelegate).onDetach();

    fragment.setDelegateFactory(delegateFactory);
    fragment.onStart();
    fragment.onResume();
    fragment.onPostResume();

    verify(mockDelegate, times(1)).onStart();
    verify(mockDelegate, times(1)).onResume();
    verify(mockDelegate, times(1)).onPostResume();

    fragment.onPause();
    fragment.detachFromFlutterEngine();
    verify(mockDelegate, times(1)).onPause();
    verify(mockDelegate, times(1)).onDestroyView();
    verify(mockDelegate, times(1)).onDetach();

    fragment.onStop();
    verify(mockDelegate, never()).onStop();

    fragment.onStart();
    fragment.onResume();
    fragment.onPostResume();
    // No more events through to the delegate.
    verify(mockDelegate, times(1)).onStart();
    verify(mockDelegate, times(1)).onResume();
    verify(mockDelegate, times(1)).onPostResume();

    fragment.onDestroy();
    // 1 time same as before.
    verify(mockDelegate, times(1)).onDestroyView();
    verify(mockDelegate, times(1)).onDetach();
  }

  @Test
  public void itDoesNotReleaseEnginewhenDetachFromFlutterEngine() {
    FlutterActivityAndFragmentDelegate mockDelegate =
        mock(FlutterActivityAndFragmentDelegate.class);
    isDelegateAttached = true;
    when(mockDelegate.isAttached()).thenAnswer(invocation -> isDelegateAttached);
    doAnswer(invocation -> isDelegateAttached = false).when(mockDelegate).onDetach();
    TestDelegateFactory delegateFactory = new TestDelegateFactory(mockDelegate);

    FlutterFragment fragment =
        FlutterFragment.withCachedEngine("my_cached_engine")
            .destroyEngineWithFragment(true)
            .build();

    fragment.setDelegateFactory(delegateFactory);
    fragment.onStart();
    fragment.onResume();
    fragment.onPostResume();
    fragment.onPause();

    assertTrue(mockDelegate.isAttached());
    fragment.detachFromFlutterEngine();
    verify(mockDelegate, times(1)).onDetach();
    verify(mockDelegate, never()).release();
    assertFalse(mockDelegate.isAttached());
  }

  @Test
  public void itReleaseEngineWhenOnDetach() {
    FlutterActivityAndFragmentDelegate mockDelegate =
        mock(FlutterActivityAndFragmentDelegate.class);
    isDelegateAttached = true;
    when(mockDelegate.isAttached()).thenAnswer(invocation -> isDelegateAttached);
    doAnswer(invocation -> isDelegateAttached = false).when(mockDelegate).onDetach();
    TestDelegateFactory delegateFactory = new TestDelegateFactory(mockDelegate);

    FlutterFragment fragment =
        spy(
            FlutterFragment.withCachedEngine("my_cached_engine")
                .destroyEngineWithFragment(true)
                .build());
    when(fragment.getContext()).thenReturn(mock(Context.class));

    fragment.setDelegateFactory(delegateFactory);
    fragment.onStart();
    fragment.onResume();
    fragment.onPostResume();
    fragment.onPause();

    assertTrue(mockDelegate.isAttached());
    fragment.onDetach();
    verify(mockDelegate, times(1)).onDetach();
    verify(mockDelegate, times(1)).release();
    assertFalse(mockDelegate.isAttached());
  }

  @Test
  public void itReturnsExclusiveAppComponent() {
    FlutterFragment fragment = FlutterFragment.createDefault();
    FlutterActivityAndFragmentDelegate delegate = new FlutterActivityAndFragmentDelegate(fragment);
    TestDelegateFactory delegateFactory = new TestDelegateFactory(delegate);
    fragment.setDelegateFactory(delegateFactory);

    assertEquals(fragment.getExclusiveAppComponent(), delegate);
  }

  @SuppressWarnings("deprecation")
  private FragmentActivity getMockFragmentActivity() {
    // TODO(reidbaker): https://github.com/flutter/flutter/issues/133151
    return Robolectric.setupActivity(FragmentActivity.class);
  }

  @Test
  public void itDelegatesOnBackPressedWithSetFrameworkHandlesBack() {
    // We need to mock FlutterJNI to avoid triggering native code.
    FlutterJNI flutterJNI = mock(FlutterJNI.class);
    when(flutterJNI.isAttached()).thenReturn(true);

    FlutterEngine flutterEngine =
        new FlutterEngine(ctx, new FlutterLoader(), flutterJNI, null, false);
    FlutterEngineCache.getInstance().put("my_cached_engine", flutterEngine);

    FlutterFragment fragment =
        FlutterFragment.withCachedEngine("my_cached_engine")
            // This enables the use of onBackPressedCallback, which is what
            // sends backs to the framework if setFrameworkHandlesBack is true.
            .shouldAutomaticallyHandleOnBackPressed(true)
            .build();
    FragmentActivity activity = getMockFragmentActivity();
    activity
        .getSupportFragmentManager()
        .beginTransaction()
        .add(android.R.id.content, fragment)
        .commitNow();

    FlutterActivityAndFragmentDelegate mockDelegate =
        mock(FlutterActivityAndFragmentDelegate.class);
    isDelegateAttached = true;
    when(mockDelegate.isAttached()).thenAnswer(invocation -> isDelegateAttached);
    doAnswer(invocation -> isDelegateAttached = false).when(mockDelegate).onDetach();
    TestDelegateFactory delegateFactory = new TestDelegateFactory(mockDelegate);
    fragment.setDelegateFactory(delegateFactory);

    // Calling onBackPressed now will still be handled by Android (the default),
    // until setFrameworkHandlesBack is set to true.
    activity.getOnBackPressedDispatcher().onBackPressed();
    verify(mockDelegate, times(0)).onBackPressed();

    // Setting setFrameworkHandlesBack to true means the delegate will receive
    // the back and Android won't handle it.
    fragment.setFrameworkHandlesBack(true);
    activity.getOnBackPressedDispatcher().onBackPressed();
    verify(mockDelegate, times(1)).onBackPressed();
  }

  @SuppressWarnings("deprecation")
  // Robolectric.setupActivity
  // TODO(reidbaker): https://github.com/flutter/flutter/issues/133151
  @Test
  public void itHandlesPopSystemNavigationAutomaticallyWhenEnabled() {
    // We need to mock FlutterJNI to avoid triggering native code.
    FlutterJNI flutterJNI = mock(FlutterJNI.class);
    when(flutterJNI.isAttached()).thenReturn(true);

    FlutterEngine flutterEngine =
        new FlutterEngine(ctx, new FlutterLoader(), flutterJNI, null, false);
    FlutterEngineCache.getInstance().put("my_cached_engine", flutterEngine);

    FlutterFragment fragment =
        FlutterFragment.withCachedEngine("my_cached_engine")
            .shouldAutomaticallyHandleOnBackPressed(true)
            .build();
    FragmentActivity activity = getMockFragmentActivity();
    activity
        .getSupportFragmentManager()
        .beginTransaction()
        .add(android.R.id.content, fragment)
        .commitNow();
    final AtomicBoolean onBackPressedCalled = new AtomicBoolean(false);
    OnBackPressedCallback callback =
        new OnBackPressedCallback(true) {
          @Override
          public void handleOnBackPressed() {
            onBackPressedCalled.set(true);
          }
        };
    activity.getOnBackPressedDispatcher().addCallback(callback);

    FlutterActivityAndFragmentDelegate mockDelegate =
        mock(FlutterActivityAndFragmentDelegate.class);
    TestDelegateFactory delegateFactory = new TestDelegateFactory(mockDelegate);
    fragment.setDelegateFactory(delegateFactory);

    assertTrue(callback.isEnabled());

    assertTrue(fragment.popSystemNavigator());

    verify(mockDelegate, never()).onBackPressed();
    assertTrue(onBackPressedCalled.get());
    assertTrue(callback.isEnabled());

    callback.setEnabled(false);
    assertFalse(callback.isEnabled());
    assertTrue(fragment.popSystemNavigator());

    verify(mockDelegate, never()).onBackPressed();
    assertFalse(callback.isEnabled());
  }

  @Test
  public void itRegistersComponentCallbacks() {
    FlutterActivityAndFragmentDelegate mockDelegate =
        mock(FlutterActivityAndFragmentDelegate.class);
    isDelegateAttached = true;
    when(mockDelegate.isAttached()).thenAnswer(invocation -> isDelegateAttached);
    doAnswer(invocation -> isDelegateAttached = false).when(mockDelegate).onDetach();
    TestDelegateFactory delegateFactory = new TestDelegateFactory(mockDelegate);

    Context spyCtx = spy(ctx);
    // We need to mock FlutterJNI to avoid triggering native code.
    FlutterJNI flutterJNI = mock(FlutterJNI.class);
    when(flutterJNI.isAttached()).thenReturn(true);

    FlutterEngine flutterEngine =
        new FlutterEngine(spyCtx, new FlutterLoader(), flutterJNI, null, false);
    FlutterEngineCache.getInstance().put("my_cached_engine", flutterEngine);

    FlutterFragment fragment = spy(FlutterFragment.withCachedEngine("my_cached_engine").build());
    when(fragment.getContext()).thenReturn(spyCtx);
    fragment.setDelegateFactory(delegateFactory);

    fragment.onAttach(spyCtx);
    verify(spyCtx, times(1)).registerComponentCallbacks(any());
    verify(spyCtx, never()).unregisterComponentCallbacks(any());

    fragment.onDetach();
    verify(spyCtx, times(1)).registerComponentCallbacks(any());
    verify(spyCtx, times(1)).unregisterComponentCallbacks(any());
  }
}
