// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.android;

import static org.junit.Assert.assertNotNull;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.isNull;
import static org.mockito.Mockito.doAnswer;
import static org.mockito.Mockito.inOrder;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.verifyNoMoreInteractions;
import static org.mockito.Mockito.when;
import static org.mockito.Mockito.withSettings;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.lifecycle.Lifecycle;
import androidx.test.core.app.ApplicationProvider;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.embedding.engine.FlutterEngine;
import io.flutter.embedding.engine.FlutterEngineCache;
import io.flutter.embedding.engine.FlutterJNI;
import io.flutter.embedding.engine.FlutterShellArgs;
import io.flutter.embedding.engine.loader.FlutterLoader;
import io.flutter.embedding.engine.plugins.FlutterPlugin;
import io.flutter.embedding.engine.plugins.activity.ActivityAware;
import io.flutter.embedding.engine.plugins.activity.ActivityPluginBinding;
import io.flutter.embedding.engine.systemchannels.LifecycleChannel;
import io.flutter.plugin.platform.PlatformPlugin;
import java.util.List;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.ArgumentCaptor;
import org.mockito.InOrder;
import org.mockito.invocation.InvocationOnMock;
import org.mockito.stubbing.Answer;
import org.robolectric.Robolectric;
import org.robolectric.android.controller.ActivityController;
import org.robolectric.annotation.Config;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
public class FlutterAndroidComponentTest {
  private final Context ctx = ApplicationProvider.getApplicationContext();

  @Test
  public void pluginsReceiveFlutterPluginBinding() {
    // ---- Test setup ----
    // Place a FlutterEngine in the static cache.
    FlutterLoader mockFlutterLoader = mock(FlutterLoader.class);
    FlutterJNI mockFlutterJni = mock(FlutterJNI.class);
    when(mockFlutterJni.isAttached()).thenReturn(true);
    FlutterEngine cachedEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));
    FlutterEngineCache.getInstance().put("my_flutter_engine", cachedEngine);

    // Add mock plugin.
    FlutterPlugin mockPlugin = mock(FlutterPlugin.class);
    cachedEngine.getPlugins().add(mockPlugin);

    // Create a fake Host, which is required by the delegate.
    FakeHost fakeHost = new FakeHost(cachedEngine);
    fakeHost.shouldDestroyEngineWithHost = true;

    // Create the real object that we're testing.
    FlutterActivityAndFragmentDelegate delegate = new FlutterActivityAndFragmentDelegate(fakeHost);

    // --- Execute the behavior under test ---
    // Push the delegate through all lifecycle methods all the way to destruction.
    delegate.onAttach(ctx);

    // Verify that the plugin is attached to the FlutterEngine.
    ArgumentCaptor<FlutterPlugin.FlutterPluginBinding> pluginBindingCaptor =
        ArgumentCaptor.forClass(FlutterPlugin.FlutterPluginBinding.class);
    verify(mockPlugin, times(1)).onAttachedToEngine(pluginBindingCaptor.capture());
    FlutterPlugin.FlutterPluginBinding binding = pluginBindingCaptor.getValue();
    assertNotNull(binding.getApplicationContext());
    assertNotNull(binding.getBinaryMessenger());
    assertNotNull(binding.getTextureRegistry());
    assertNotNull(binding.getPlatformViewRegistry());

    delegate.onRestoreInstanceState(null);
    delegate.onCreateView(null, null, null, 0, true);
    delegate.onStart();
    delegate.onResume();
    delegate.onPause();
    delegate.onStop();
    delegate.onDestroyView();
    delegate.onDetach();

    // Verify the plugin was detached from the FlutterEngine.
    pluginBindingCaptor = ArgumentCaptor.forClass(FlutterPlugin.FlutterPluginBinding.class);
    verify(mockPlugin, times(1)).onDetachedFromEngine(pluginBindingCaptor.capture());
    binding = pluginBindingCaptor.getValue();
    assertNotNull(binding.getApplicationContext());
    assertNotNull(binding.getBinaryMessenger());
    assertNotNull(binding.getTextureRegistry());
    assertNotNull(binding.getPlatformViewRegistry());
  }

  @Test
  public void activityAwarePluginsReceiveActivityBinding() {
    // ---- Test setup ----
    // Place a FlutterEngine in the static cache.
    FlutterLoader mockFlutterLoader = mock(FlutterLoader.class);
    FlutterJNI mockFlutterJni = mock(FlutterJNI.class);
    when(mockFlutterJni.isAttached()).thenReturn(true);
    FlutterEngine cachedEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));
    FlutterEngineCache.getInstance().put("my_flutter_engine", cachedEngine);

    // Add mock plugin.
    FlutterPlugin mockPlugin =
        mock(FlutterPlugin.class, withSettings().extraInterfaces(ActivityAware.class));
    ActivityAware activityAwarePlugin = (ActivityAware) mockPlugin;
    ActivityPluginBinding.OnSaveInstanceStateListener mockSaveStateListener =
        mock(ActivityPluginBinding.OnSaveInstanceStateListener.class);

    // Add a OnSaveStateListener when the Activity plugin binding is made available.
    doAnswer(
            new Answer() {
              @Override
              public Object answer(InvocationOnMock invocation) throws Throwable {
                ActivityPluginBinding binding =
                    (ActivityPluginBinding) invocation.getArguments()[0];
                binding.addOnSaveStateListener(mockSaveStateListener);
                return null;
              }
            })
        .when(activityAwarePlugin)
        .onAttachedToActivity(any(ActivityPluginBinding.class));

    cachedEngine.getPlugins().add(mockPlugin);

    // Create a fake Host, which is required by the delegate.
    FlutterActivityAndFragmentDelegate.Host fakeHost = new FakeHost(cachedEngine);

    FlutterActivityAndFragmentDelegate delegate = new FlutterActivityAndFragmentDelegate(fakeHost);

    // --- Execute the behavior under test ---
    // Push the delegate through all lifecycle methods all the way to destruction.
    delegate.onAttach(ctx);

    // Verify plugin was given an ActivityPluginBinding.
    ArgumentCaptor<ActivityPluginBinding> pluginBindingCaptor =
        ArgumentCaptor.forClass(ActivityPluginBinding.class);
    verify(activityAwarePlugin, times(1)).onAttachedToActivity(pluginBindingCaptor.capture());
    ActivityPluginBinding binding = pluginBindingCaptor.getValue();
    assertNotNull(binding.getActivity());
    assertNotNull(binding.getLifecycle());

    delegate.onRestoreInstanceState(null);

    // Verify that after Activity creation, the plugin was allowed to restore state.
    verify(mockSaveStateListener, times(1)).onRestoreInstanceState(isNull());

    delegate.onCreateView(null, null, null, 0, true);
    delegate.onStart();
    delegate.onResume();
    delegate.onPause();
    delegate.onStop();
    delegate.onSaveInstanceState(mock(Bundle.class));

    // Verify that the plugin was allowed to save state.
    verify(mockSaveStateListener, times(1)).onSaveInstanceState(any(Bundle.class));

    delegate.onDestroyView();
    delegate.onDetach();

    // Verify that the plugin was detached from the Activity.
    verify(activityAwarePlugin, times(1)).onDetachedFromActivity();
  }

  @Test
  public void normalLifecycleStepsDoNotTriggerADetachFromFlutterEngine() {
    // ---- Test setup ----
    // Place a FlutterEngine in the static cache.
    FlutterLoader mockFlutterLoader = mock(FlutterLoader.class);
    FlutterJNI mockFlutterJni = mock(FlutterJNI.class);
    when(mockFlutterJni.isAttached()).thenReturn(true);
    FlutterEngine cachedEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));
    FlutterEngineCache.getInstance().put("my_flutter_engine", cachedEngine);

    // Create a fake Host, which is required by the delegate.
    FakeHost fakeHost = new FakeHost(cachedEngine);

    // Create the real object that we're testing.
    FlutterActivityAndFragmentDelegate delegate =
        spy(new FlutterActivityAndFragmentDelegate(fakeHost));

    // --- Execute the behavior under test ---
    // Push the delegate through all lifecycle methods all the way to destruction.
    delegate.onAttach(ctx);
    delegate.onRestoreInstanceState(null);
    delegate.onCreateView(null, null, null, 0, true);
    delegate.onStart();
    delegate.onResume();
    delegate.onPause();
    delegate.onStop();
    delegate.onDestroyView();
    delegate.onDetach();

    verify(delegate, never()).detachFromFlutterEngine();
  }

  @Test
  public void twoOverlappingFlutterActivitiesDoNotCrosstalk() {
    // ---- Test setup ----
    // Place a FlutterEngine in the static cache.
    FlutterLoader mockFlutterLoader = mock(FlutterLoader.class);
    FlutterJNI mockFlutterJni = mock(FlutterJNI.class);
    when(mockFlutterJni.isAttached()).thenReturn(true);
    FlutterEngine cachedEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));
    FlutterEngineCache.getInstance().put("my_flutter_engine", cachedEngine);
    LifecycleChannel mockLifecycleChannel = mock(LifecycleChannel.class);
    when(cachedEngine.getLifecycleChannel()).thenReturn(mockLifecycleChannel);

    Intent intent = FlutterActivity.withCachedEngine("my_flutter_engine").build(ctx);
    ActivityController<FlutterActivity> activityController1 =
        Robolectric.buildActivity(FlutterActivity.class, intent);
    activityController1.create().start().resume();

    InOrder inOrder = inOrder(mockLifecycleChannel);
    inOrder.verify(mockLifecycleChannel, times(1)).appIsResumed();
    verifyNoMoreInteractions(mockLifecycleChannel);

    activityController1.pause();
    // Create a second instance on the same engine and start running it as well.
    ActivityController<FlutterActivity> activityController2 =
        Robolectric.buildActivity(FlutterActivity.class, intent);
    activityController2.create().start().resume();

    // From the onPause of the first activity.
    inOrder.verify(mockLifecycleChannel, times(1)).appIsInactive();
    // By creating the second activity, we should automatically detach the first activity.
    inOrder.verify(mockLifecycleChannel, times(1)).appIsDetached();
    // In order, the second activity then is resumed.
    inOrder.verify(mockLifecycleChannel, times(1)).appIsResumed();
    verifyNoMoreInteractions(mockLifecycleChannel);

    // The first activity goes through the normal lifecycles of destruction, but since we
    // detached the first activity during the second activity's creation, we should ignore the
    // first activity's destruction events to avoid crosstalk.
    activityController1.stop().destroy();
    verifyNoMoreInteractions(mockLifecycleChannel);
  }

  private static class FakeHost implements FlutterActivityAndFragmentDelegate.Host {
    final FlutterEngine cachedEngine;
    Activity activity;
    boolean shouldDestroyEngineWithHost = false;
    Lifecycle lifecycle = mock(Lifecycle.class);

    private FakeHost(@NonNull FlutterEngine flutterEngine) {
      cachedEngine = flutterEngine;
    }

    @NonNull
    @Override
    public Context getContext() {
      return ApplicationProvider.getApplicationContext();
    }

    @SuppressWarnings("deprecation")
    // Robolectric.setupActivity
    // TODO(reidbaker): https://github.com/flutter/flutter/issues/133151
    @Nullable
    @Override
    public Activity getActivity() {
      if (activity == null) {
        activity = Robolectric.setupActivity(Activity.class);
      }
      return activity;
    }

    @NonNull
    @Override
    public Lifecycle getLifecycle() {
      return lifecycle;
    }

    @NonNull
    @Override
    public FlutterShellArgs getFlutterShellArgs() {
      return new FlutterShellArgs(new String[] {});
    }

    @Nullable
    @Override
    public String getCachedEngineId() {
      return "my_flutter_engine";
    }

    @Nullable
    @Override
    public String getCachedEngineGroupId() {
      return "my_flutter_engine_group";
    }

    @Override
    public boolean shouldDestroyEngineWithHost() {
      return shouldDestroyEngineWithHost;
    }

    @NonNull
    @Override
    public String getDartEntrypointFunctionName() {
      return "main";
    }

    @Nullable
    @Override
    public String getDartEntrypointLibraryUri() {
      return null;
    }

    @Nullable
    @Override
    public List<String> getDartEntrypointArgs() {
      return null;
    }

    @NonNull
    @Override
    public String getAppBundlePath() {
      return "/fake/path";
    }

    @Nullable
    @Override
    public String getInitialRoute() {
      return "/";
    }

    @NonNull
    @Override
    public RenderMode getRenderMode() {
      return RenderMode.surface;
    }

    @NonNull
    @Override
    public TransparencyMode getTransparencyMode() {
      return TransparencyMode.transparent;
    }

    @Override
    public ExclusiveAppComponent<Activity> getExclusiveAppComponent() {
      return null;
    }

    @Nullable
    @Override
    public FlutterEngine provideFlutterEngine(@NonNull Context context) {
      return cachedEngine;
    }

    @Nullable
    @Override
    public PlatformPlugin providePlatformPlugin(
        @Nullable Activity activity, @NonNull FlutterEngine flutterEngine) {
      return null;
    }

    @Override
    public void configureFlutterEngine(@NonNull FlutterEngine flutterEngine) {}

    @Override
    public void cleanUpFlutterEngine(@NonNull FlutterEngine flutterEngine) {}

    @Override
    public boolean shouldAttachEngineToActivity() {
      return true;
    }

    @Override
    public boolean shouldHandleDeeplinking() {
      return false;
    }

    @Override
    public boolean shouldRestoreAndSaveState() {
      return true;
    }

    @Override
    public boolean shouldDispatchAppLifecycleState() {
      return true;
    }

    @Override
    public boolean attachToEngineAutomatically() {
      return true;
    }

    @Override
    public boolean getBackCallbackState() {
      return false;
    }

    @Override
    public void onFlutterSurfaceViewCreated(@NonNull FlutterSurfaceView flutterSurfaceView) {}

    @Override
    public void onFlutterTextureViewCreated(@NonNull FlutterTextureView flutterTextureView) {}

    @Override
    public void onFlutterUiDisplayed() {}

    @Override
    public void onFlutterUiNoLongerDisplayed() {}

    @Override
    public void detachFromFlutterEngine() {}

    @Override
    public void updateSystemUiOverlays() {}

    @Override
    public boolean popSystemNavigator() {
      return false;
    }

    @Override
    public void setFrameworkHandlesBack(boolean frameworkHandlesBack) {}
  }
}
