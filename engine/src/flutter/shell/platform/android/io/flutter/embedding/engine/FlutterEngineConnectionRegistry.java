// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine;

import android.app.Activity;
import android.app.Service;
import android.content.BroadcastReceiver;
import android.content.ContentProvider;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.lifecycle.Lifecycle;
import io.flutter.Log;
import io.flutter.embedding.android.ExclusiveAppComponent;
import io.flutter.embedding.engine.loader.FlutterLoader;
import io.flutter.embedding.engine.plugins.FlutterPlugin;
import io.flutter.embedding.engine.plugins.PluginRegistry;
import io.flutter.embedding.engine.plugins.activity.ActivityAware;
import io.flutter.embedding.engine.plugins.activity.ActivityControlSurface;
import io.flutter.embedding.engine.plugins.activity.ActivityPluginBinding;
import io.flutter.embedding.engine.plugins.broadcastreceiver.BroadcastReceiverAware;
import io.flutter.embedding.engine.plugins.broadcastreceiver.BroadcastReceiverControlSurface;
import io.flutter.embedding.engine.plugins.broadcastreceiver.BroadcastReceiverPluginBinding;
import io.flutter.embedding.engine.plugins.contentprovider.ContentProviderAware;
import io.flutter.embedding.engine.plugins.contentprovider.ContentProviderControlSurface;
import io.flutter.embedding.engine.plugins.contentprovider.ContentProviderPluginBinding;
import io.flutter.embedding.engine.plugins.lifecycle.HiddenLifecycleReference;
import io.flutter.embedding.engine.plugins.service.ServiceAware;
import io.flutter.embedding.engine.plugins.service.ServiceControlSurface;
import io.flutter.embedding.engine.plugins.service.ServicePluginBinding;
import io.flutter.util.TraceSection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/**
 * This class is owned by the {@link io.flutter.embedding.engine.FlutterEngine} and its role is to
 * managed its connections with Android App Components and Flutter plugins.
 *
 * <p>It enforces the {0|1}:1 relationship between activity and engine, and propagates the app
 * component connection to the plugins.
 */
/* package */ class FlutterEngineConnectionRegistry
    implements PluginRegistry,
        ActivityControlSurface,
        ServiceControlSurface,
        BroadcastReceiverControlSurface,
        ContentProviderControlSurface {
  private static final String TAG = "FlutterEngineCxnRegstry";

  // PluginRegistry
  @NonNull
  private final Map<Class<? extends FlutterPlugin>, FlutterPlugin> plugins = new HashMap<>();

  // Standard FlutterPlugin
  @NonNull private final FlutterEngine flutterEngine;
  @NonNull private final FlutterPlugin.FlutterPluginBinding pluginBinding;

  // ActivityAware
  @NonNull
  private final Map<Class<? extends FlutterPlugin>, ActivityAware> activityAwarePlugins =
      new HashMap<>();

  @Nullable private ExclusiveAppComponent<Activity> exclusiveActivity;
  @Nullable private FlutterEngineActivityPluginBinding activityPluginBinding;
  private boolean isWaitingForActivityReattachment = false;

  // ServiceAware
  @NonNull
  private final Map<Class<? extends FlutterPlugin>, ServiceAware> serviceAwarePlugins =
      new HashMap<>();

  @Nullable private Service service;
  @Nullable private FlutterEngineServicePluginBinding servicePluginBinding;

  // BroadcastReceiver
  @NonNull
  private final Map<Class<? extends FlutterPlugin>, BroadcastReceiverAware>
      broadcastReceiverAwarePlugins = new HashMap<>();

  @Nullable private BroadcastReceiver broadcastReceiver;
  @Nullable private FlutterEngineBroadcastReceiverPluginBinding broadcastReceiverPluginBinding;

  // ContentProvider
  @NonNull
  private final Map<Class<? extends FlutterPlugin>, ContentProviderAware>
      contentProviderAwarePlugins = new HashMap<>();

  @Nullable private ContentProvider contentProvider;
  @Nullable private FlutterEngineContentProviderPluginBinding contentProviderPluginBinding;

  FlutterEngineConnectionRegistry(
      @NonNull Context appContext,
      @NonNull FlutterEngine flutterEngine,
      @NonNull FlutterLoader flutterLoader,
      @Nullable FlutterEngineGroup group) {
    this.flutterEngine = flutterEngine;
    pluginBinding =
        new FlutterPlugin.FlutterPluginBinding(
            appContext,
            flutterEngine,
            flutterEngine.getDartExecutor(),
            flutterEngine.getRenderer(),
            flutterEngine.getPlatformViewsController().getRegistry(),
            new DefaultFlutterAssets(flutterLoader),
            group);
  }

  public void destroy() {
    Log.v(TAG, "Destroying.");
    // Detach from any Android component that we may currently be attached to, e.g., Activity,
    // Service, BroadcastReceiver, ContentProvider. This must happen before removing all plugins so
    // that the plugins have an opportunity to clean up references as a result of component
    // detachment.
    detachFromAppComponent();

    // Remove all registered plugins.
    removeAll();
  }

  @Override
  public void add(@NonNull FlutterPlugin plugin) {
    try (TraceSection e =
        TraceSection.scoped(
            "FlutterEngineConnectionRegistry#add " + plugin.getClass().getSimpleName())) {
      if (has(plugin.getClass())) {
        Log.w(
            TAG,
            "Attempted to register plugin ("
                + plugin
                + ") but it was "
                + "already registered with this FlutterEngine ("
                + flutterEngine
                + ").");
        return;
      }

      Log.v(TAG, "Adding plugin: " + plugin);
      // Add the plugin to our generic set of plugins and notify the plugin
      // that is has been attached to an engine.
      plugins.put(plugin.getClass(), plugin);
      plugin.onAttachedToEngine(pluginBinding);

      // For ActivityAware plugins, add the plugin to our set of ActivityAware
      // plugins, and if this engine is currently attached to an Activity,
      // notify the ActivityAware plugin that it is now attached to an Activity.
      if (plugin instanceof ActivityAware) {
        ActivityAware activityAware = (ActivityAware) plugin;
        activityAwarePlugins.put(plugin.getClass(), activityAware);

        if (isAttachedToActivity()) {
          activityAware.onAttachedToActivity(activityPluginBinding);
        }
      }

      // For ServiceAware plugins, add the plugin to our set of ServiceAware
      // plugins, and if this engine is currently attached to a Service,
      // notify the ServiceAware plugin that it is now attached to a Service.
      if (plugin instanceof ServiceAware) {
        ServiceAware serviceAware = (ServiceAware) plugin;
        serviceAwarePlugins.put(plugin.getClass(), serviceAware);

        if (isAttachedToService()) {
          serviceAware.onAttachedToService(servicePluginBinding);
        }
      }

      // For BroadcastReceiverAware plugins, add the plugin to our set of BroadcastReceiverAware
      // plugins, and if this engine is currently attached to a BroadcastReceiver,
      // notify the BroadcastReceiverAware plugin that it is now attached to a BroadcastReceiver.
      if (plugin instanceof BroadcastReceiverAware) {
        BroadcastReceiverAware broadcastReceiverAware = (BroadcastReceiverAware) plugin;
        broadcastReceiverAwarePlugins.put(plugin.getClass(), broadcastReceiverAware);

        if (isAttachedToBroadcastReceiver()) {
          broadcastReceiverAware.onAttachedToBroadcastReceiver(broadcastReceiverPluginBinding);
        }
      }

      // For ContentProviderAware plugins, add the plugin to our set of ContentProviderAware
      // plugins, and if this engine is currently attached to a ContentProvider,
      // notify the ContentProviderAware plugin that it is now attached to a ContentProvider.
      if (plugin instanceof ContentProviderAware) {
        ContentProviderAware contentProviderAware = (ContentProviderAware) plugin;
        contentProviderAwarePlugins.put(plugin.getClass(), contentProviderAware);

        if (isAttachedToContentProvider()) {
          contentProviderAware.onAttachedToContentProvider(contentProviderPluginBinding);
        }
      }
    }
  }

  @Override
  public void add(@NonNull Set<FlutterPlugin> plugins) {
    for (FlutterPlugin plugin : plugins) {
      add(plugin);
    }
  }

  @Override
  public boolean has(@NonNull Class<? extends FlutterPlugin> pluginClass) {
    return plugins.containsKey(pluginClass);
  }

  @Override
  public FlutterPlugin get(@NonNull Class<? extends FlutterPlugin> pluginClass) {
    return plugins.get(pluginClass);
  }

  @Override
  public void remove(@NonNull Class<? extends FlutterPlugin> pluginClass) {
    FlutterPlugin plugin = plugins.get(pluginClass);
    if (plugin == null) {
      return;
    }

    try (TraceSection e =
        TraceSection.scoped(
            "FlutterEngineConnectionRegistry#remove " + pluginClass.getSimpleName())) {
      // For ActivityAware plugins, notify the plugin that it is detached from
      // an Activity if an Activity is currently attached to this engine. Then
      // remove the plugin from our set of ActivityAware plugins.
      if (plugin instanceof ActivityAware) {
        if (isAttachedToActivity()) {
          ActivityAware activityAware = (ActivityAware) plugin;
          activityAware.onDetachedFromActivity();
        }
        activityAwarePlugins.remove(pluginClass);
      }

      // For ServiceAware plugins, notify the plugin that it is detached from
      // a Service if a Service is currently attached to this engine. Then
      // remove the plugin from our set of ServiceAware plugins.
      if (plugin instanceof ServiceAware) {
        if (isAttachedToService()) {
          ServiceAware serviceAware = (ServiceAware) plugin;
          serviceAware.onDetachedFromService();
        }
        serviceAwarePlugins.remove(pluginClass);
      }

      // For BroadcastReceiverAware plugins, notify the plugin that it is detached from
      // a BroadcastReceiver if a BroadcastReceiver is currently attached to this engine. Then
      // remove the plugin from our set of BroadcastReceiverAware plugins.
      if (plugin instanceof BroadcastReceiverAware) {
        if (isAttachedToBroadcastReceiver()) {
          BroadcastReceiverAware broadcastReceiverAware = (BroadcastReceiverAware) plugin;
          broadcastReceiverAware.onDetachedFromBroadcastReceiver();
        }
        broadcastReceiverAwarePlugins.remove(pluginClass);
      }

      // For ContentProviderAware plugins, notify the plugin that it is detached from
      // a ContentProvider if a ContentProvider is currently attached to this engine. Then
      // remove the plugin from our set of ContentProviderAware plugins.
      if (plugin instanceof ContentProviderAware) {
        if (isAttachedToContentProvider()) {
          ContentProviderAware contentProviderAware = (ContentProviderAware) plugin;
          contentProviderAware.onDetachedFromContentProvider();
        }
        contentProviderAwarePlugins.remove(pluginClass);
      }

      // Notify the plugin that is now detached from this engine. Then remove
      // it from our set of generic plugins.
      plugin.onDetachedFromEngine(pluginBinding);
      plugins.remove(pluginClass);
    }
  }

  @Override
  public void remove(@NonNull Set<Class<? extends FlutterPlugin>> pluginClasses) {
    for (Class<? extends FlutterPlugin> pluginClass : pluginClasses) {
      remove(pluginClass);
    }
  }

  @Override
  public void removeAll() {
    // We copy the keys to a new set so that we can mutate the set while using
    // the keys.
    remove(new HashSet<>(plugins.keySet()));
    plugins.clear();
  }

  private void detachFromAppComponent() {
    if (isAttachedToActivity()) {
      detachFromActivity();
    } else if (isAttachedToService()) {
      detachFromService();
    } else if (isAttachedToBroadcastReceiver()) {
      detachFromBroadcastReceiver();
    } else if (isAttachedToContentProvider()) {
      detachFromContentProvider();
    }
  }

  // -------- Start ActivityControlSurface -------
  private boolean isAttachedToActivity() {
    return exclusiveActivity != null;
  }

  private Activity attachedActivity() {
    return exclusiveActivity != null ? exclusiveActivity.getAppComponent() : null;
  }

  @Override
  public void attachToActivity(
      @NonNull ExclusiveAppComponent<Activity> exclusiveActivity, @NonNull Lifecycle lifecycle) {
    try (TraceSection e = TraceSection.scoped("FlutterEngineConnectionRegistry#attachToActivity")) {
      if (this.exclusiveActivity != null) {
        this.exclusiveActivity.detachFromFlutterEngine();
      }
      // If we were already attached to an app component, detach from it.
      detachFromAppComponent();
      this.exclusiveActivity = exclusiveActivity;
      attachToActivityInternal(exclusiveActivity.getAppComponent(), lifecycle);
    }
  }

  private void attachToActivityInternal(@NonNull Activity activity, @NonNull Lifecycle lifecycle) {
    this.activityPluginBinding = new FlutterEngineActivityPluginBinding(activity, lifecycle);

    final boolean useSoftwareRendering =
        activity.getIntent() != null
            ? activity
                .getIntent()
                .getBooleanExtra(FlutterShellArgs.ARG_KEY_ENABLE_SOFTWARE_RENDERING, false)
            : false;
    flutterEngine.getPlatformViewsController().setSoftwareRendering(useSoftwareRendering);

    // Activate the PlatformViewsController. This must happen before any plugins attempt
    // to use it, otherwise an error stack trace will appear that says there is no
    // flutter/platform_views channel.
    flutterEngine
        .getPlatformViewsController()
        .attach(activity, flutterEngine.getRenderer(), flutterEngine.getDartExecutor());

    // Notify all ActivityAware plugins that they are now attached to a new Activity.
    for (ActivityAware activityAware : activityAwarePlugins.values()) {
      if (isWaitingForActivityReattachment) {
        activityAware.onReattachedToActivityForConfigChanges(activityPluginBinding);
      } else {
        activityAware.onAttachedToActivity(activityPluginBinding);
      }
    }
    isWaitingForActivityReattachment = false;
  }

  @Override
  public void detachFromActivityForConfigChanges() {
    if (isAttachedToActivity()) {
      try (TraceSection e =
          TraceSection.scoped(
              "FlutterEngineConnectionRegistry#detachFromActivityForConfigChanges")) {
        isWaitingForActivityReattachment = true;

        for (ActivityAware activityAware : activityAwarePlugins.values()) {
          activityAware.onDetachedFromActivityForConfigChanges();
        }

        detachFromActivityInternal();
      }
    } else {
      Log.e(TAG, "Attempted to detach plugins from an Activity when no Activity was attached.");
    }
  }

  @Override
  public void detachFromActivity() {
    if (isAttachedToActivity()) {
      try (TraceSection e =
          TraceSection.scoped("FlutterEngineConnectionRegistry#detachFromActivity")) {
        for (ActivityAware activityAware : activityAwarePlugins.values()) {
          activityAware.onDetachedFromActivity();
        }

        detachFromActivityInternal();
      }
    } else {
      Log.e(TAG, "Attempted to detach plugins from an Activity when no Activity was attached.");
    }
  }

  private void detachFromActivityInternal() {
    // Deactivate PlatformViewsController.
    flutterEngine.getPlatformViewsController().detach();

    exclusiveActivity = null;
    activityPluginBinding = null;
  }

  @Override
  public boolean onRequestPermissionsResult(
      int requestCode, @NonNull String[] permissions, @NonNull int[] grantResult) {
    if (isAttachedToActivity()) {
      try (TraceSection e =
          TraceSection.scoped("FlutterEngineConnectionRegistry#onRequestPermissionsResult")) {
        return activityPluginBinding.onRequestPermissionsResult(
            requestCode, permissions, grantResult);
      }
    } else {
      Log.e(
          TAG,
          "Attempted to notify ActivityAware plugins of onRequestPermissionsResult, but no Activity"
              + " was attached.");
      return false;
    }
  }

  @Override
  public boolean onActivityResult(int requestCode, int resultCode, @Nullable Intent data) {
    if (isAttachedToActivity()) {
      try (TraceSection e =
          TraceSection.scoped("FlutterEngineConnectionRegistry#onActivityResult")) {
        return activityPluginBinding.onActivityResult(requestCode, resultCode, data);
      }
    } else {
      Log.e(
          TAG,
          "Attempted to notify ActivityAware plugins of onActivityResult, but no Activity was"
              + " attached.");
      return false;
    }
  }

  @Override
  public void onNewIntent(@NonNull Intent intent) {
    if (isAttachedToActivity()) {
      try (TraceSection e = TraceSection.scoped("FlutterEngineConnectionRegistry#onNewIntent")) {
        activityPluginBinding.onNewIntent(intent);
      }
    } else {
      Log.e(
          TAG,
          "Attempted to notify ActivityAware plugins of onNewIntent, but no Activity was"
              + " attached.");
    }
  }

  @Override
  public void onUserLeaveHint() {
    if (isAttachedToActivity()) {
      try (TraceSection e =
          TraceSection.scoped("FlutterEngineConnectionRegistry#onUserLeaveHint")) {
        activityPluginBinding.onUserLeaveHint();
      }
    } else {
      Log.e(
          TAG,
          "Attempted to notify ActivityAware plugins of onUserLeaveHint, but no Activity was"
              + " attached.");
    }
  }

  @Override
  public void onSaveInstanceState(@NonNull Bundle bundle) {
    if (isAttachedToActivity()) {
      try (TraceSection e =
          TraceSection.scoped("FlutterEngineConnectionRegistry#onSaveInstanceState")) {
        activityPluginBinding.onSaveInstanceState(bundle);
      }
    } else {
      Log.e(
          TAG,
          "Attempted to notify ActivityAware plugins of onSaveInstanceState, but no Activity was"
              + " attached.");
    }
  }

  @Override
  public void onRestoreInstanceState(@Nullable Bundle bundle) {
    if (isAttachedToActivity()) {
      try (TraceSection e =
          TraceSection.scoped("FlutterEngineConnectionRegistry#onRestoreInstanceState")) {
        activityPluginBinding.onRestoreInstanceState(bundle);
      }
    } else {
      Log.e(
          TAG,
          "Attempted to notify ActivityAware plugins of onRestoreInstanceState, but no Activity was"
              + " attached.");
    }
  }
  // ------- End ActivityControlSurface -----

  // ----- Start ServiceControlSurface ----
  private boolean isAttachedToService() {
    return service != null;
  }

  @Override
  public void attachToService(
      @NonNull Service service, @Nullable Lifecycle lifecycle, boolean isForeground) {
    try (TraceSection e = TraceSection.scoped("FlutterEngineConnectionRegistry#attachToService")) {
      // If we were already attached to an Android component, detach from it.
      detachFromAppComponent();

      this.service = service;
      this.servicePluginBinding = new FlutterEngineServicePluginBinding(service, lifecycle);

      // Notify all ServiceAware plugins that they are now attached to a new Service.
      for (ServiceAware serviceAware : serviceAwarePlugins.values()) {
        serviceAware.onAttachedToService(servicePluginBinding);
      }
    }
  }

  @Override
  public void detachFromService() {
    if (isAttachedToService()) {
      try (TraceSection e =
          TraceSection.scoped("FlutterEngineConnectionRegistry#detachFromService")) {
        // Notify all ServiceAware plugins that they are no longer attached to a Service.
        for (ServiceAware serviceAware : serviceAwarePlugins.values()) {
          serviceAware.onDetachedFromService();
        }

        service = null;
        servicePluginBinding = null;
      }
    } else {
      Log.e(TAG, "Attempted to detach plugins from a Service when no Service was attached.");
    }
  }

  @Override
  public void onMoveToForeground() {
    if (isAttachedToService()) {
      try (TraceSection e =
          TraceSection.scoped("FlutterEngineConnectionRegistry#onMoveToForeground")) {
        servicePluginBinding.onMoveToForeground();
      }
    }
  }

  @Override
  public void onMoveToBackground() {
    if (isAttachedToService()) {
      try (TraceSection e =
          TraceSection.scoped("FlutterEngineConnectionRegistry#onMoveToBackground")) {
        servicePluginBinding.onMoveToBackground();
      }
    }
  }
  // ----- End ServiceControlSurface ---

  // ----- Start BroadcastReceiverControlSurface ---
  private boolean isAttachedToBroadcastReceiver() {
    return broadcastReceiver != null;
  }

  @Override
  public void attachToBroadcastReceiver(
      @NonNull BroadcastReceiver broadcastReceiver, @NonNull Lifecycle lifecycle) {
    try (TraceSection e =
        TraceSection.scoped("FlutterEngineConnectionRegistry#attachToBroadcastReceiver")) {
      // If we were already attached to an Android component, detach from it.
      detachFromAppComponent();

      this.broadcastReceiver = broadcastReceiver;
      this.broadcastReceiverPluginBinding =
          new FlutterEngineBroadcastReceiverPluginBinding(broadcastReceiver);
      // TODO(mattcarroll): resolve possibility of different lifecycles between this and engine
      // attachment

      // Notify all BroadcastReceiverAware plugins that they are now attached to a new
      // BroadcastReceiver.
      for (BroadcastReceiverAware broadcastReceiverAware : broadcastReceiverAwarePlugins.values()) {
        broadcastReceiverAware.onAttachedToBroadcastReceiver(broadcastReceiverPluginBinding);
      }
    }
  }

  @Override
  public void detachFromBroadcastReceiver() {
    if (isAttachedToBroadcastReceiver()) {
      try (TraceSection e =
          TraceSection.scoped("FlutterEngineConnectionRegistry#detachFromBroadcastReceiver")) {
        // Notify all BroadcastReceiverAware plugins that they are no longer attached to a
        // BroadcastReceiver.
        for (BroadcastReceiverAware broadcastReceiverAware :
            broadcastReceiverAwarePlugins.values()) {
          broadcastReceiverAware.onDetachedFromBroadcastReceiver();
        }
      }
    } else {
      Log.e(
          TAG,
          "Attempted to detach plugins from a BroadcastReceiver when no BroadcastReceiver was"
              + " attached.");
    }
  }
  // ----- End BroadcastReceiverControlSurface ----

  // ----- Start ContentProviderControlSurface ----
  private boolean isAttachedToContentProvider() {
    return contentProvider != null;
  }

  @Override
  public void attachToContentProvider(
      @NonNull ContentProvider contentProvider, @NonNull Lifecycle lifecycle) {

    try (TraceSection e =
        TraceSection.scoped("FlutterEngineConnectionRegistry#attachToContentProvider")) {
      // If we were already attached to an Android component, detach from it.
      detachFromAppComponent();

      this.contentProvider = contentProvider;
      this.contentProviderPluginBinding =
          new FlutterEngineContentProviderPluginBinding(contentProvider);
      // TODO(mattcarroll): resolve possibility of different lifecycles between this and engine
      // attachment

      // Notify all ContentProviderAware plugins that they are now attached to a new
      // ContentProvider.
      for (ContentProviderAware contentProviderAware : contentProviderAwarePlugins.values()) {
        contentProviderAware.onAttachedToContentProvider(contentProviderPluginBinding);
      }
    }
  }

  @Override
  public void detachFromContentProvider() {
    if (isAttachedToContentProvider()) {
      try (TraceSection e =
          TraceSection.scoped("FlutterEngineConnectionRegistry#detachFromContentProvider")) {
        // Notify all ContentProviderAware plugins that they are no longer attached to a
        // ContentProvider.
        for (ContentProviderAware contentProviderAware : contentProviderAwarePlugins.values()) {
          contentProviderAware.onDetachedFromContentProvider();
        }
      }
    } else {
      Log.e(
          TAG,
          "Attempted to detach plugins from a ContentProvider when no ContentProvider was"
              + " attached.");
    }
  }
  // ----- End ContentProviderControlSurface -----

  private static class DefaultFlutterAssets implements FlutterPlugin.FlutterAssets {
    final FlutterLoader flutterLoader;

    private DefaultFlutterAssets(@NonNull FlutterLoader flutterLoader) {
      this.flutterLoader = flutterLoader;
    }

    public String getAssetFilePathByName(@NonNull String assetFileName) {
      return flutterLoader.getLookupKeyForAsset(assetFileName);
    }

    public String getAssetFilePathByName(
        @NonNull String assetFileName, @NonNull String packageName) {
      return flutterLoader.getLookupKeyForAsset(assetFileName, packageName);
    }

    public String getAssetFilePathBySubpath(@NonNull String assetSubpath) {
      return flutterLoader.getLookupKeyForAsset(assetSubpath);
    }

    public String getAssetFilePathBySubpath(
        @NonNull String assetSubpath, @NonNull String packageName) {
      return flutterLoader.getLookupKeyForAsset(assetSubpath, packageName);
    }
  }

  private static class FlutterEngineActivityPluginBinding implements ActivityPluginBinding {
    @NonNull private final Activity activity;
    @NonNull private final HiddenLifecycleReference hiddenLifecycleReference;

    @NonNull
    private final Set<io.flutter.plugin.common.PluginRegistry.RequestPermissionsResultListener>
        onRequestPermissionsResultListeners = new HashSet<>();

    @NonNull
    private final Set<io.flutter.plugin.common.PluginRegistry.ActivityResultListener>
        onActivityResultListeners = new HashSet<>();

    @NonNull
    private final Set<io.flutter.plugin.common.PluginRegistry.NewIntentListener>
        onNewIntentListeners = new HashSet<>();

    @NonNull
    private final Set<io.flutter.plugin.common.PluginRegistry.UserLeaveHintListener>
        onUserLeaveHintListeners = new HashSet<>();

    @NonNull
    private final Set<io.flutter.plugin.common.PluginRegistry.WindowFocusChangedListener>
        onWindowFocusChangedListeners = new HashSet<>();

    @NonNull
    private final Set<OnSaveInstanceStateListener> onSaveInstanceStateListeners = new HashSet<>();

    public FlutterEngineActivityPluginBinding(
        @NonNull Activity activity, @NonNull Lifecycle lifecycle) {
      this.activity = activity;
      this.hiddenLifecycleReference = new HiddenLifecycleReference(lifecycle);
    }

    @Override
    @NonNull
    public Activity getActivity() {
      return activity;
    }

    @NonNull
    @Override
    public Object getLifecycle() {
      return hiddenLifecycleReference;
    }

    @Override
    public void addRequestPermissionsResultListener(
        @NonNull
            io.flutter.plugin.common.PluginRegistry.RequestPermissionsResultListener listener) {
      onRequestPermissionsResultListeners.add(listener);
    }

    @Override
    public void removeRequestPermissionsResultListener(
        @NonNull
            io.flutter.plugin.common.PluginRegistry.RequestPermissionsResultListener listener) {
      onRequestPermissionsResultListeners.remove(listener);
    }

    /**
     * Invoked by the {@link io.flutter.embedding.engine.FlutterEngine} that owns this {@code
     * ActivityPluginBinding} when its associated {@link android.app.Activity} has its {@code
     * onRequestPermissionsResult(...)} method invoked.
     */
    boolean onRequestPermissionsResult(
        int requestCode, @NonNull String[] permissions, @NonNull int[] grantResult) {
      boolean didConsumeResult = false;
      for (io.flutter.plugin.common.PluginRegistry.RequestPermissionsResultListener listener :
          onRequestPermissionsResultListeners) {
        didConsumeResult =
            listener.onRequestPermissionsResult(requestCode, permissions, grantResult)
                || didConsumeResult;
      }
      return didConsumeResult;
    }

    @Override
    public void addActivityResultListener(
        @NonNull io.flutter.plugin.common.PluginRegistry.ActivityResultListener listener) {
      onActivityResultListeners.add(listener);
    }

    @Override
    public void removeActivityResultListener(
        @NonNull io.flutter.plugin.common.PluginRegistry.ActivityResultListener listener) {
      onActivityResultListeners.remove(listener);
    }

    /**
     * Invoked by the {@link io.flutter.embedding.engine.FlutterEngine} that owns this {@code
     * ActivityPluginBinding} when its associated {@link android.app.Activity} has its {@code
     * onActivityResult(...)} method invoked.
     */
    boolean onActivityResult(int requestCode, int resultCode, @Nullable Intent data) {
      boolean didConsumeResult = false;
      for (io.flutter.plugin.common.PluginRegistry.ActivityResultListener listener :
          new HashSet<>(onActivityResultListeners)) {
        didConsumeResult =
            listener.onActivityResult(requestCode, resultCode, data) || didConsumeResult;
      }
      return didConsumeResult;
    }

    @Override
    public void addOnNewIntentListener(
        @NonNull io.flutter.plugin.common.PluginRegistry.NewIntentListener listener) {
      onNewIntentListeners.add(listener);
    }

    @Override
    public void removeOnNewIntentListener(
        @NonNull io.flutter.plugin.common.PluginRegistry.NewIntentListener listener) {
      onNewIntentListeners.remove(listener);
    }

    /**
     * Invoked by the {@link io.flutter.embedding.engine.FlutterEngine} that owns this {@code
     * ActivityPluginBinding} when its associated {@link android.app.Activity} has its {@code
     * onNewIntent(...)} method invoked.
     */
    void onNewIntent(@Nullable Intent intent) {
      for (io.flutter.plugin.common.PluginRegistry.NewIntentListener listener :
          onNewIntentListeners) {
        listener.onNewIntent(intent);
      }
    }

    @Override
    public void addOnUserLeaveHintListener(
        @NonNull io.flutter.plugin.common.PluginRegistry.UserLeaveHintListener listener) {
      onUserLeaveHintListeners.add(listener);
    }

    @Override
    public void removeOnUserLeaveHintListener(
        @NonNull io.flutter.plugin.common.PluginRegistry.UserLeaveHintListener listener) {
      onUserLeaveHintListeners.remove(listener);
    }

    @Override
    public void addOnWindowFocusChangedListener(
        @NonNull io.flutter.plugin.common.PluginRegistry.WindowFocusChangedListener listener) {
      onWindowFocusChangedListeners.add(listener);
    }

    @Override
    public void removeOnWindowFocusChangedListener(
        @NonNull io.flutter.plugin.common.PluginRegistry.WindowFocusChangedListener listener) {
      onWindowFocusChangedListeners.remove(listener);
    }

    void onWindowFocusChanged(boolean hasFocus) {
      for (io.flutter.plugin.common.PluginRegistry.WindowFocusChangedListener listener :
          onWindowFocusChangedListeners) {
        listener.onWindowFocusChanged(hasFocus);
      }
    }

    @Override
    public void addOnSaveStateListener(@NonNull OnSaveInstanceStateListener listener) {
      onSaveInstanceStateListeners.add(listener);
    }

    @Override
    public void removeOnSaveStateListener(@NonNull OnSaveInstanceStateListener listener) {
      onSaveInstanceStateListeners.remove(listener);
    }

    /**
     * Invoked by the {@link io.flutter.embedding.engine.FlutterEngine} that owns this {@code
     * ActivityPluginBinding} when its associated {@link android.app.Activity} has its {@code
     * onUserLeaveHint()} method invoked.
     */
    void onUserLeaveHint() {
      for (io.flutter.plugin.common.PluginRegistry.UserLeaveHintListener listener :
          onUserLeaveHintListeners) {
        listener.onUserLeaveHint();
      }
    }

    /**
     * Invoked by the {@link io.flutter.embedding.engine.FlutterEngine} that owns this {@code
     * ActivityPluginBinding} when its associated {@link android.app.Activity} or {@code Fragment}
     * has its {@code onSaveInstanceState(Bundle)} method invoked.
     */
    void onSaveInstanceState(@NonNull Bundle bundle) {
      for (OnSaveInstanceStateListener listener : onSaveInstanceStateListeners) {
        listener.onSaveInstanceState(bundle);
      }
    }

    /**
     * Invoked by the {@link io.flutter.embedding.engine.FlutterEngine} that owns this {@code
     * ActivityPluginBinding} when its associated {@link android.app.Activity} or {@code Fragment}
     * has its {@code onCreate(Bundle)} method invoked.
     */
    void onRestoreInstanceState(@Nullable Bundle bundle) {
      for (OnSaveInstanceStateListener listener : onSaveInstanceStateListeners) {
        listener.onRestoreInstanceState(bundle);
      }
    }
  }

  private static class FlutterEngineServicePluginBinding implements ServicePluginBinding {
    @NonNull private final Service service;
    @Nullable private final HiddenLifecycleReference hiddenLifecycleReference;

    @NonNull
    private final Set<ServiceAware.OnModeChangeListener> onModeChangeListeners = new HashSet<>();

    FlutterEngineServicePluginBinding(@NonNull Service service, @Nullable Lifecycle lifecycle) {
      this.service = service;
      hiddenLifecycleReference = lifecycle != null ? new HiddenLifecycleReference(lifecycle) : null;
    }

    @Override
    @NonNull
    public Service getService() {
      return service;
    }

    @Nullable
    @Override
    public Object getLifecycle() {
      return hiddenLifecycleReference;
    }

    @Override
    public void addOnModeChangeListener(@NonNull ServiceAware.OnModeChangeListener listener) {
      onModeChangeListeners.add(listener);
    }

    @Override
    public void removeOnModeChangeListener(@NonNull ServiceAware.OnModeChangeListener listener) {
      onModeChangeListeners.remove(listener);
    }

    void onMoveToForeground() {
      for (ServiceAware.OnModeChangeListener listener : onModeChangeListeners) {
        listener.onMoveToForeground();
      }
    }

    void onMoveToBackground() {
      for (ServiceAware.OnModeChangeListener listener : onModeChangeListeners) {
        listener.onMoveToBackground();
      }
    }
  }

  private static class FlutterEngineBroadcastReceiverPluginBinding
      implements BroadcastReceiverPluginBinding {
    @NonNull private final BroadcastReceiver broadcastReceiver;

    FlutterEngineBroadcastReceiverPluginBinding(@NonNull BroadcastReceiver broadcastReceiver) {
      this.broadcastReceiver = broadcastReceiver;
    }

    @NonNull
    @Override
    public BroadcastReceiver getBroadcastReceiver() {
      return broadcastReceiver;
    }
  }

  private static class FlutterEngineContentProviderPluginBinding
      implements ContentProviderPluginBinding {
    @NonNull private final ContentProvider contentProvider;

    FlutterEngineContentProviderPluginBinding(@NonNull ContentProvider contentProvider) {
      this.contentProvider = contentProvider;
    }

    @NonNull
    @Override
    public ContentProvider getContentProvider() {
      return contentProvider;
    }
  }
}
