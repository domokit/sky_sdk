// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine;

import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import androidx.annotation.NonNull;
import androidx.lifecycle.Lifecycle;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.embedding.android.ExclusiveAppComponent;
import io.flutter.embedding.engine.loader.FlutterLoader;
import io.flutter.embedding.engine.plugins.FlutterPlugin;
import io.flutter.embedding.engine.plugins.activity.ActivityAware;
import io.flutter.embedding.engine.plugins.activity.ActivityPluginBinding;
import io.flutter.plugin.common.PluginRegistry;
import io.flutter.plugin.platform.PlatformViewsController;
import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.annotation.Config;

// Run with Robolectric so that Log calls don't crash.
@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
public class FlutterEngineConnectionRegistryTest {
  @Test
  public void itDoesNotRegisterTheSamePluginTwice() {
    Context context = mock(Context.class);

    FlutterEngine flutterEngine = mock(FlutterEngine.class);
    PlatformViewsController platformViewsController = mock(PlatformViewsController.class);
    when(flutterEngine.getPlatformViewsController()).thenReturn(platformViewsController);

    FlutterLoader flutterLoader = mock(FlutterLoader.class);

    FakeFlutterPlugin fakePlugin1 = new FakeFlutterPlugin();
    FakeFlutterPlugin fakePlugin2 = new FakeFlutterPlugin();

    FlutterEngineConnectionRegistry registry =
        new FlutterEngineConnectionRegistry(context, flutterEngine, flutterLoader, null);

    // Verify that the registry doesn't think it contains our plugin yet.
    assertFalse(registry.has(fakePlugin1.getClass()));

    // Add our plugin to the registry.
    registry.add(fakePlugin1);

    // Verify that the registry now thinks it contains our plugin.
    assertTrue(registry.has(fakePlugin1.getClass()));
    assertEquals(1, fakePlugin1.attachmentCallCount);

    // Add a different instance of the same plugin class.
    registry.add(fakePlugin2);

    // Verify that the registry did not detach the 1st plugin, and
    // it did not attach the 2nd plugin.
    assertEquals(1, fakePlugin1.attachmentCallCount);
    assertEquals(0, fakePlugin1.detachmentCallCount);

    assertEquals(0, fakePlugin2.attachmentCallCount);
    assertEquals(0, fakePlugin2.detachmentCallCount);
  }

  @Test
  public void activityResultListenerCanBeRemovedFromListener() {
    Context context = mock(Context.class);

    FlutterEngine flutterEngine = mock(FlutterEngine.class);
    PlatformViewsController platformViewsController = mock(PlatformViewsController.class);
    when(flutterEngine.getPlatformViewsController()).thenReturn(platformViewsController);

    FlutterLoader flutterLoader = mock(FlutterLoader.class);

    ExclusiveAppComponent appComponent = mock(ExclusiveAppComponent.class);
    Activity activity = mock(Activity.class);
    when(appComponent.getAppComponent()).thenReturn(activity);

    Intent intent = mock(Intent.class);
    when(activity.getIntent()).thenReturn(intent);

    Lifecycle lifecycle = mock(Lifecycle.class);
    AtomicBoolean isFirstCall = new AtomicBoolean(true);

    // Set up the environment to get the required internal data
    FlutterEngineConnectionRegistry registry =
        new FlutterEngineConnectionRegistry(context, flutterEngine, flutterLoader, null);
    FakeActivityAwareFlutterPlugin fakePlugin = new FakeActivityAwareFlutterPlugin();
    registry.add(fakePlugin);
    registry.attachToActivity(appComponent, lifecycle);

    // The binding is now available via `fakePlugin.binding`: Create and add the listeners
    FakeActivityResultListener listener1 =
        new FakeActivityResultListener(isFirstCall, fakePlugin.binding);
    FakeActivityResultListener listener2 =
        new FakeActivityResultListener(isFirstCall, fakePlugin.binding);

    fakePlugin.binding.addActivityResultListener(listener1);
    fakePlugin.binding.addActivityResultListener(listener2);

    // fire the onActivityResult which should invoke both listeners
    registry.onActivityResult(0, 0, intent);

    assertEquals(1, listener1.callCount);
    assertEquals(1, listener2.callCount);

    // fire it again to check if the first called listener was removed
    registry.onActivityResult(0, 0, intent);

    // The order of the listeners in the HashSet is random: So just check the sum of calls
    assertEquals(3, listener1.callCount + listener2.callCount);
  }

  @Test
  public void softwareRendering() {
    Context context = mock(Context.class);

    FlutterEngine flutterEngine = mock(FlutterEngine.class);
    PlatformViewsController platformViewsController = mock(PlatformViewsController.class);
    when(flutterEngine.getPlatformViewsController()).thenReturn(platformViewsController);

    FlutterLoader flutterLoader = mock(FlutterLoader.class);

    ExclusiveAppComponent appComponent = mock(ExclusiveAppComponent.class);
    Activity activity = mock(Activity.class);
    when(appComponent.getAppComponent()).thenReturn(activity);

    // Test attachToActivity with an Activity that has no Intent.
    FlutterEngineConnectionRegistry registry =
        new FlutterEngineConnectionRegistry(context, flutterEngine, flutterLoader, null);
    registry.attachToActivity(appComponent, mock(Lifecycle.class));
    verify(platformViewsController).setSoftwareRendering(false);

    Intent intent = mock(Intent.class);
    when(intent.getBooleanExtra("enable-software-rendering", false)).thenReturn(false);
    when(activity.getIntent()).thenReturn(intent);

    registry.attachToActivity(appComponent, mock(Lifecycle.class));
    verify(platformViewsController, times(2)).setSoftwareRendering(false);

    when(intent.getBooleanExtra("enable-software-rendering", false)).thenReturn(true);

    registry.attachToActivity(appComponent, mock(Lifecycle.class));
    verify(platformViewsController).setSoftwareRendering(true);
  }

  private static class FakeFlutterPlugin implements FlutterPlugin {
    public int attachmentCallCount = 0;
    public int detachmentCallCount = 0;

    @Override
    public void onAttachedToEngine(@NonNull FlutterPluginBinding binding) {
      attachmentCallCount += 1;
    }

    @Override
    public void onDetachedFromEngine(@NonNull FlutterPluginBinding binding) {
      detachmentCallCount += 1;
    }
  }

  private static class FakeActivityAwareFlutterPlugin implements FlutterPlugin, ActivityAware {
    public ActivityPluginBinding binding;

    @Override
    public void onAttachedToEngine(@NonNull FlutterPluginBinding binding) {}

    @Override
    public void onDetachedFromEngine(@NonNull FlutterPluginBinding binding) {}

    @Override
    public void onAttachedToActivity(final ActivityPluginBinding binding) {
      this.binding = binding;
    }

    @Override
    public void onDetachedFromActivityForConfigChanges() {}

    @Override
    public void onReattachedToActivityForConfigChanges(final ActivityPluginBinding binding) {}

    @Override
    public void onDetachedFromActivity() {}
  }

  private static class FakeActivityResultListener implements PluginRegistry.ActivityResultListener {
    public int callCount = 0;
    private final AtomicBoolean isFirstCall;
    private final ActivityPluginBinding binding;

    public FakeActivityResultListener(AtomicBoolean isFirstCall, ActivityPluginBinding binding) {
      this.isFirstCall = isFirstCall;
      this.binding = binding;
    }

    @Override
    public boolean onActivityResult(
        final int requestCode, final int resultCode, final Intent data) {
      callCount++;
      if (isFirstCall.get()) {
        isFirstCall.set(false);
        binding.removeActivityResultListener(this);
      }
      return false;
    }
  }
}
