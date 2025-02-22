// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine;

import android.content.Context;
import android.content.pm.PackageManager.NameNotFoundException;
import android.content.res.AssetManager;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.VisibleForTesting;
import io.flutter.FlutterInjector;
import io.flutter.Log;
import io.flutter.embedding.engine.dart.DartExecutor;
import io.flutter.embedding.engine.dart.DartExecutor.DartEntrypoint;
import io.flutter.embedding.engine.deferredcomponents.DeferredComponentManager;
import io.flutter.embedding.engine.loader.FlutterLoader;
import io.flutter.embedding.engine.plugins.PluginRegistry;
import io.flutter.embedding.engine.plugins.activity.ActivityControlSurface;
import io.flutter.embedding.engine.plugins.broadcastreceiver.BroadcastReceiverControlSurface;
import io.flutter.embedding.engine.plugins.contentprovider.ContentProviderControlSurface;
import io.flutter.embedding.engine.plugins.service.ServiceControlSurface;
import io.flutter.embedding.engine.plugins.util.GeneratedPluginRegister;
import io.flutter.embedding.engine.renderer.FlutterRenderer;
import io.flutter.embedding.engine.renderer.RenderSurface;
import io.flutter.embedding.engine.systemchannels.AccessibilityChannel;
import io.flutter.embedding.engine.systemchannels.BackGestureChannel;
import io.flutter.embedding.engine.systemchannels.DeferredComponentChannel;
import io.flutter.embedding.engine.systemchannels.LifecycleChannel;
import io.flutter.embedding.engine.systemchannels.LocalizationChannel;
import io.flutter.embedding.engine.systemchannels.MouseCursorChannel;
import io.flutter.embedding.engine.systemchannels.NavigationChannel;
import io.flutter.embedding.engine.systemchannels.PlatformChannel;
import io.flutter.embedding.engine.systemchannels.ProcessTextChannel;
import io.flutter.embedding.engine.systemchannels.RestorationChannel;
import io.flutter.embedding.engine.systemchannels.ScribeChannel;
import io.flutter.embedding.engine.systemchannels.SettingsChannel;
import io.flutter.embedding.engine.systemchannels.SpellCheckChannel;
import io.flutter.embedding.engine.systemchannels.SystemChannel;
import io.flutter.embedding.engine.systemchannels.TextInputChannel;
import io.flutter.plugin.localization.LocalizationPlugin;
import io.flutter.plugin.platform.PlatformViewsController;
import io.flutter.plugin.platform.PlatformViewsController2;
import io.flutter.plugin.text.ProcessTextPlugin;
import io.flutter.util.ViewUtils;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * A single Flutter execution environment.
 *
 * <p>The {@code FlutterEngine} is the container through which Dart code can be run in an Android
 * application.
 *
 * <p>Dart code in a {@code FlutterEngine} can execute in the background, or it can be render to the
 * screen by using the accompanying {@link FlutterRenderer} and Dart code using the Flutter
 * framework on the Dart side. Rendering can be started and stopped, thus allowing a {@code
 * FlutterEngine} to move from UI interaction to data-only processing and then back to UI
 * interaction.
 *
 * <p>Multiple {@code FlutterEngine}s may exist, execute Dart code, and render UIs within a single
 * Android app. For better memory performance characteristics, construct multiple {@code
 * FlutterEngine}s via {@link io.flutter.embedding.engine.FlutterEngineGroup} rather than via {@code
 * FlutterEngine}'s constructor directly.
 *
 * <p>To start running Dart and/or Flutter within this {@code FlutterEngine}, get a reference to
 * this engine's {@link DartExecutor} and then use {@link
 * DartExecutor#executeDartEntrypoint(DartExecutor.DartEntrypoint)}. The {@link
 * DartExecutor#executeDartEntrypoint(DartExecutor.DartEntrypoint)} method must not be invoked twice
 * on the same {@code FlutterEngine}.
 *
 * <p>To start rendering Flutter content to the screen, use {@link #getRenderer()} to obtain a
 * {@link FlutterRenderer} and then attach a {@link RenderSurface}. Consider using a {@link
 * io.flutter.embedding.android.FlutterView} as a {@link RenderSurface}.
 *
 * <p>Instatiating the first {@code FlutterEngine} per process will also load the Flutter engine's
 * native library and start the Dart VM. Subsequent {@code FlutterEngine}s will run on the same VM
 * instance but will have their own Dart <a
 * href="https://api.dartlang.org/stable/dart-isolate/Isolate-class.html">Isolate</a> when the
 * {@link DartExecutor} is run. Each Isolate is a self-contained Dart environment and cannot
 * communicate with each other except via Isolate ports.
 */
public class FlutterEngine implements ViewUtils.DisplayUpdater {
  private static final String TAG = "FlutterEngine";

  @NonNull private final FlutterJNI flutterJNI;
  @NonNull private final FlutterRenderer renderer;
  @NonNull private final DartExecutor dartExecutor;
  @NonNull private final FlutterEngineConnectionRegistry pluginRegistry;
  @NonNull private final LocalizationPlugin localizationPlugin;

  // System channels.
  @NonNull private final AccessibilityChannel accessibilityChannel;
  @NonNull private final DeferredComponentChannel deferredComponentChannel;
  @NonNull private final LifecycleChannel lifecycleChannel;
  @NonNull private final LocalizationChannel localizationChannel;
  @NonNull private final MouseCursorChannel mouseCursorChannel;
  @NonNull private final NavigationChannel navigationChannel;
  @NonNull private final BackGestureChannel backGestureChannel;
  @NonNull private final RestorationChannel restorationChannel;
  @NonNull private final PlatformChannel platformChannel;
  @NonNull private final ProcessTextChannel processTextChannel;
  @NonNull private final ScribeChannel scribeChannel;
  @NonNull private final SettingsChannel settingsChannel;
  @NonNull private final SpellCheckChannel spellCheckChannel;
  @NonNull private final SystemChannel systemChannel;
  @NonNull private final TextInputChannel textInputChannel;

  // Platform Views.
  @NonNull private final PlatformViewsController platformViewsController;
  @NonNull private final PlatformViewsController2 platformViewsController2;

  // Engine Lifecycle.
  @NonNull private final Set<EngineLifecycleListener> engineLifecycleListeners = new HashSet<>();

  // Unique handle for this engine.
  @NonNull private final long engineId;

  @VisibleForTesting(otherwise = VisibleForTesting.PACKAGE_PRIVATE)
  public static void resetNextEngineId() {
    nextEngineId = 1;
  }

  // Handle to assign to the next engine created.
  private static long nextEngineId = 1;

  // Map of engine identifiers to engines.
  private static final Map<Long, FlutterEngine> idToEngine = new HashMap<>();

  @NonNull
  private final EngineLifecycleListener engineLifecycleListener =
      new EngineLifecycleListener() {
        @SuppressWarnings("unused")
        public void onPreEngineRestart() {
          Log.v(TAG, "onPreEngineRestart()");
          for (EngineLifecycleListener lifecycleListener : engineLifecycleListeners) {
            lifecycleListener.onPreEngineRestart();
          }

          platformViewsController.onPreEngineRestart();
          platformViewsController2.onPreEngineRestart();
          restorationChannel.clearData();
        }

        @Override
        public void onEngineWillDestroy() {
          // This inner implementation doesn't do anything since FlutterEngine sent this
          // notification in the first place. It's meant for external listeners.
        }
      };

  /**
   * Constructs a new {@code FlutterEngine}.
   *
   * <p>A new {@code FlutterEngine} does not execute any Dart code automatically. See {@link
   * #getDartExecutor()} and {@link DartExecutor#executeDartEntrypoint(DartExecutor.DartEntrypoint)}
   * to begin executing Dart code within this {@code FlutterEngine}.
   *
   * <p>A new {@code FlutterEngine} will not display any UI until a {@link RenderSurface} is
   * registered. See {@link #getRenderer()} and {@link
   * FlutterRenderer#startRenderingToSurface(Surface, boolean)}.
   *
   * <p>A new {@code FlutterEngine} automatically attaches all plugins. See {@link #getPlugins()}.
   *
   * <p>A new {@code FlutterEngine} does come with all default system channels attached.
   *
   * <p>The first {@code FlutterEngine} instance constructed per process will also load the Flutter
   * native library and start a Dart VM.
   *
   * <p>In order to pass Dart VM initialization arguments (see {@link
   * io.flutter.embedding.engine.FlutterShellArgs}) when creating the VM, manually set the
   * initialization arguments by calling {@link
   * io.flutter.embedding.engine.loader.FlutterLoader#startInitialization(Context)} and {@link
   * io.flutter.embedding.engine.loader.FlutterLoader#ensureInitializationComplete(Context,
   * String[])} before constructing the engine.
   */
  public FlutterEngine(@NonNull Context context) {
    this(context, null);
  }

  /**
   * Same as {@link #FlutterEngine(Context)} with added support for passing Dart VM arguments.
   *
   * <p>If the Dart VM has already started, the given arguments will have no effect.
   */
  public FlutterEngine(@NonNull Context context, @Nullable String[] dartVmArgs) {
    this(context, /* flutterLoader */ null, /* flutterJNI */ null, dartVmArgs, true);
  }

  /**
   * Same as {@link #FlutterEngine(Context)} with added support for passing Dart VM arguments and
   * avoiding automatic plugin registration.
   *
   * <p>If the Dart VM has already started, the given arguments will have no effect.
   */
  public FlutterEngine(
      @NonNull Context context,
      @Nullable String[] dartVmArgs,
      boolean automaticallyRegisterPlugins) {
    this(
        context,
        /* flutterLoader */ null,
        /* flutterJNI */ null,
        dartVmArgs,
        automaticallyRegisterPlugins);
  }

  /**
   * Same as {@link #FlutterEngine(Context, String[], boolean)} with added support for configuring
   * whether the engine will receive restoration data.
   *
   * <p>The {@code waitForRestorationData} flag controls whether the engine delays responding to
   * requests from the framework for restoration data until that data has been provided to the
   * engine via {@code RestorationChannel.setRestorationData(byte[] data)}. If the flag is false,
   * the framework may temporarily initialize itself to default values before the restoration data
   * has been made available to the engine. Setting {@code waitForRestorationData} to true avoids
   * this extra work by delaying initialization until the data is available.
   *
   * <p>When {@code waitForRestorationData} is set, {@code
   * RestorationChannel.setRestorationData(byte[] data)} must be called at a later point in time. If
   * it later turns out that no restoration data is available to restore the framework from, that
   * method must still be called with null as an argument to indicate "no data".
   *
   * <p>If the framework never requests the restoration data, this flag has no effect.
   */
  public FlutterEngine(
      @NonNull Context context,
      @Nullable String[] dartVmArgs,
      boolean automaticallyRegisterPlugins,
      boolean waitForRestorationData) {
    this(
        context,
        /* flutterLoader */ null,
        /* flutterJNI */ null,
        new PlatformViewsController(),
        dartVmArgs,
        automaticallyRegisterPlugins,
        waitForRestorationData);
  }

  /**
   * Same as {@link #FlutterEngine(Context, FlutterLoader, FlutterJNI, String[], boolean)} but with
   * no Dart VM flags and automatically registers plugins.
   *
   * <p>{@code flutterJNI} should be a new instance that has never been attached to an engine
   * before.
   */
  public FlutterEngine(
      @NonNull Context context,
      @Nullable FlutterLoader flutterLoader,
      @NonNull FlutterJNI flutterJNI) {
    this(context, flutterLoader, flutterJNI, null, true);
  }

  /**
   * Same as {@link #FlutterEngine(Context, FlutterLoader, FlutterJNI)}, plus Dart VM flags in
   * {@code dartVmArgs}, and control over whether plugins are automatically registered with this
   * {@code FlutterEngine} in {@code automaticallyRegisterPlugins}. If plugins are automatically
   * registered, then they are registered during the execution of this constructor.
   */
  public FlutterEngine(
      @NonNull Context context,
      @Nullable FlutterLoader flutterLoader,
      @NonNull FlutterJNI flutterJNI,
      @Nullable String[] dartVmArgs,
      boolean automaticallyRegisterPlugins) {
    this(
        context,
        flutterLoader,
        flutterJNI,
        new PlatformViewsController(),
        dartVmArgs,
        automaticallyRegisterPlugins);
  }

  /**
   * Same as {@link #FlutterEngine(Context, FlutterLoader, FlutterJNI, String[], boolean)}, plus the
   * ability to provide a custom {@code PlatformViewsController}.
   */
  public FlutterEngine(
      @NonNull Context context,
      @Nullable FlutterLoader flutterLoader,
      @NonNull FlutterJNI flutterJNI,
      @NonNull PlatformViewsController platformViewsController,
      @Nullable String[] dartVmArgs,
      boolean automaticallyRegisterPlugins) {
    this(
        context,
        flutterLoader,
        flutterJNI,
        platformViewsController,
        dartVmArgs,
        automaticallyRegisterPlugins,
        false);
  }

  /** Fully configurable {@code FlutterEngine} constructor. */
  public FlutterEngine(
      @NonNull Context context,
      @Nullable FlutterLoader flutterLoader,
      @NonNull FlutterJNI flutterJNI,
      @NonNull PlatformViewsController platformViewsController,
      @Nullable String[] dartVmArgs,
      boolean automaticallyRegisterPlugins,
      boolean waitForRestorationData) {
    this(
        context,
        flutterLoader,
        flutterJNI,
        platformViewsController,
        dartVmArgs,
        automaticallyRegisterPlugins,
        waitForRestorationData,
        null);
  }

  @VisibleForTesting(otherwise = VisibleForTesting.PACKAGE_PRIVATE)
  public FlutterEngine(
      @NonNull Context context,
      @Nullable FlutterLoader flutterLoader,
      @NonNull FlutterJNI flutterJNI,
      @NonNull PlatformViewsController platformViewsController,
      @Nullable String[] dartVmArgs,
      boolean automaticallyRegisterPlugins,
      boolean waitForRestorationData,
      @Nullable FlutterEngineGroup group) {

    this.engineId = nextEngineId++;
    idToEngine.put(engineId, this);

    AssetManager assetManager;
    try {
      assetManager = context.createPackageContext(context.getPackageName(), 0).getAssets();
    } catch (NameNotFoundException e) {
      assetManager = context.getAssets();
    }

    FlutterInjector injector = FlutterInjector.instance();

    if (flutterJNI == null) {
      flutterJNI = injector.getFlutterJNIFactory().provideFlutterJNI();
    }
    this.flutterJNI = flutterJNI;

    this.dartExecutor = new DartExecutor(flutterJNI, assetManager, engineId);
    this.dartExecutor.onAttachedToJNI();

    DeferredComponentManager deferredComponentManager =
        FlutterInjector.instance().deferredComponentManager();

    accessibilityChannel = new AccessibilityChannel(dartExecutor, flutterJNI);
    deferredComponentChannel = new DeferredComponentChannel(dartExecutor);
    lifecycleChannel = new LifecycleChannel(dartExecutor);
    localizationChannel = new LocalizationChannel(dartExecutor);
    mouseCursorChannel = new MouseCursorChannel(dartExecutor);
    navigationChannel = new NavigationChannel(dartExecutor);
    backGestureChannel = new BackGestureChannel(dartExecutor);
    platformChannel = new PlatformChannel(dartExecutor);
    processTextChannel = new ProcessTextChannel(dartExecutor, context.getPackageManager());
    restorationChannel = new RestorationChannel(dartExecutor, waitForRestorationData);
    scribeChannel = new ScribeChannel(dartExecutor);
    settingsChannel = new SettingsChannel(dartExecutor);
    spellCheckChannel = new SpellCheckChannel(dartExecutor);
    systemChannel = new SystemChannel(dartExecutor);
    textInputChannel = new TextInputChannel(dartExecutor);

    if (deferredComponentManager != null) {
      deferredComponentManager.setDeferredComponentChannel(deferredComponentChannel);
    }

    this.localizationPlugin = new LocalizationPlugin(context, localizationChannel);

    if (flutterLoader == null) {
      flutterLoader = injector.flutterLoader();
    }

    if (!flutterJNI.isAttached()) {
      flutterLoader.startInitialization(context.getApplicationContext());
      flutterLoader.ensureInitializationComplete(context, dartVmArgs);
    }

    PlatformViewsController2 platformViewsController2 = new PlatformViewsController2();
    platformViewsController2.setRegistry(platformViewsController.getRegistry());
    platformViewsController2.setFlutterJNI(flutterJNI);

    flutterJNI.addEngineLifecycleListener(engineLifecycleListener);
    flutterJNI.setPlatformViewsController(platformViewsController);
    flutterJNI.setPlatformViewsController2(platformViewsController2);
    flutterJNI.setLocalizationPlugin(localizationPlugin);
    flutterJNI.setDeferredComponentManager(injector.deferredComponentManager());

    // It should typically be a fresh, unattached JNI. But on a spawned engine, the JNI instance
    // is already attached to a native shell. In that case, the Java FlutterEngine is created around
    // an existing shell.
    if (!flutterJNI.isAttached()) {
      attachToJni();
    }

    this.renderer = new FlutterRenderer(flutterJNI);
    this.platformViewsController = platformViewsController;
    this.platformViewsController2 = platformViewsController2;

    this.pluginRegistry =
        new FlutterEngineConnectionRegistry(
            context.getApplicationContext(), this, flutterLoader, group);

    localizationPlugin.sendLocalesToFlutter(context.getResources().getConfiguration());

    // Only automatically register plugins if both constructor parameter and
    // loaded AndroidManifest config turn this feature on.
    if (automaticallyRegisterPlugins && flutterLoader.automaticallyRegisterPlugins()) {
      GeneratedPluginRegister.registerGeneratedPlugins(this);
    }

    ViewUtils.calculateMaximumDisplayMetrics(context, this);

    ProcessTextPlugin processTextPlugin = new ProcessTextPlugin(this.getProcessTextChannel());
    this.pluginRegistry.add(processTextPlugin);
  }

  private void attachToJni() {
    Log.v(TAG, "Attaching to JNI.");
    flutterJNI.attachToNative();

    if (!isAttachedToJni()) {
      throw new RuntimeException("FlutterEngine failed to attach to its native Object reference.");
    }
  }

  @SuppressWarnings("BooleanMethodIsAlwaysInverted")
  private boolean isAttachedToJni() {
    return flutterJNI.isAttached();
  }

  /**
   * Create a second {@link io.flutter.embedding.engine.FlutterEngine} based on this current one by
   * sharing as much resources together as possible to minimize startup latency and memory cost.
   *
   * @param context is a Context used to create the {@link
   *     io.flutter.embedding.engine.FlutterEngine}. Could be the same Context as the current engine
   *     or a different one. Generally, only an application Context is needed for the {@link
   *     io.flutter.embedding.engine.FlutterEngine} and its dependencies.
   * @param dartEntrypoint specifies the {@link DartEntrypoint} the new engine should run. It
   *     doesn't need to be the same entrypoint as the current engine but must be built in the same
   *     AOT or snapshot.
   * @param initialRoute The name of the initial Flutter `Navigator` `Route` to load. If this is
   *     null, it will default to the "/" route.
   * @param dartEntrypointArgs Arguments passed as a list of string to Dart's entrypoint function.
   * @return a new {@link io.flutter.embedding.engine.FlutterEngine}.
   */
  @NonNull
  /*package*/ FlutterEngine spawn(
      @NonNull Context context,
      @NonNull DartEntrypoint dartEntrypoint,
      @Nullable String initialRoute,
      @Nullable List<String> dartEntrypointArgs,
      @Nullable PlatformViewsController platformViewsController,
      boolean automaticallyRegisterPlugins,
      boolean waitForRestorationData) {
    if (!isAttachedToJni()) {
      throw new IllegalStateException(
          "Spawn can only be called on a fully constructed FlutterEngine");
    }

    FlutterJNI newFlutterJNI =
        flutterJNI.spawn(
            dartEntrypoint.dartEntrypointFunctionName,
            dartEntrypoint.dartEntrypointLibrary,
            initialRoute,
            dartEntrypointArgs,
            nextEngineId);
    return new FlutterEngine(
        context, // Context.
        null, // FlutterLoader. A null value passed here causes the constructor to get it from the
        // FlutterInjector.
        newFlutterJNI, // FlutterJNI.
        platformViewsController, // PlatformViewsController.
        null, // String[]. The Dart VM has already started, this arguments will have no effect.
        automaticallyRegisterPlugins, // boolean.
        waitForRestorationData); // boolean
  }

  /**
   * Cleans up all components within this {@code FlutterEngine} and destroys the associated Dart
   * Isolate. All state held by the Dart Isolate, such as the Flutter Elements tree, is lost.
   *
   * <p>This {@code FlutterEngine} instance should be discarded after invoking this method.
   */
  public void destroy() {
    Log.v(TAG, "Destroying.");
    for (EngineLifecycleListener listener : engineLifecycleListeners) {
      listener.onEngineWillDestroy();
    }
    // The order that these things are destroyed is important.
    pluginRegistry.destroy();
    platformViewsController.onDetachedFromJNI();
    platformViewsController2.onDetachedFromJNI();
    dartExecutor.onDetachedFromJNI();
    flutterJNI.removeEngineLifecycleListener(engineLifecycleListener);
    flutterJNI.setDeferredComponentManager(null);
    flutterJNI.detachFromNativeAndReleaseResources();
    if (FlutterInjector.instance().deferredComponentManager() != null) {
      FlutterInjector.instance().deferredComponentManager().destroy();
      deferredComponentChannel.setDeferredComponentManager(null);
    }
    idToEngine.remove(engineId);
  }

  /**
   * Adds a {@code listener} to be notified of Flutter engine lifecycle events, e.g., {@code
   * onPreEngineStart()}.
   */
  public void addEngineLifecycleListener(@NonNull EngineLifecycleListener listener) {
    engineLifecycleListeners.add(listener);
  }

  /**
   * Removes a {@code listener} that was previously added with {@link
   * #addEngineLifecycleListener(EngineLifecycleListener)}.
   */
  public void removeEngineLifecycleListener(@NonNull EngineLifecycleListener listener) {
    engineLifecycleListeners.remove(listener);
  }

  /**
   * The Dart execution context associated with this {@code FlutterEngine}.
   *
   * <p>The {@link DartExecutor} can be used to start executing Dart code from a given entrypoint.
   * See {@link DartExecutor#executeDartEntrypoint(DartExecutor.DartEntrypoint)}.
   *
   * <p>Use the {@link DartExecutor} to connect any desired message channels and method channels to
   * facilitate communication between Android and Dart/Flutter.
   */
  @NonNull
  public DartExecutor getDartExecutor() {
    return dartExecutor;
  }

  /**
   * The rendering system associated with this {@code FlutterEngine}.
   *
   * <p>To render a Flutter UI that is produced by this {@code FlutterEngine}'s Dart code, attach a
   * {@link RenderSurface} to this {@link FlutterRenderer}.
   */
  @NonNull
  public FlutterRenderer getRenderer() {
    return renderer;
  }

  /** System channel that sends accessibility requests and events from Flutter to Android. */
  @NonNull
  public AccessibilityChannel getAccessibilityChannel() {
    return accessibilityChannel;
  }

  /** System channel that sends Android lifecycle events to Flutter. */
  @NonNull
  public LifecycleChannel getLifecycleChannel() {
    return lifecycleChannel;
  }

  /** System channel that sends locale data from Android to Flutter. */
  @NonNull
  public LocalizationChannel getLocalizationChannel() {
    return localizationChannel;
  }

  /** System channel that sends Flutter navigation commands from Android to Flutter. */
  @NonNull
  public NavigationChannel getNavigationChannel() {
    return navigationChannel;
  }

  /** System channel that sends back gesture commands from Android to Flutter. */
  @NonNull
  public BackGestureChannel getBackGestureChannel() {
    return backGestureChannel;
  }

  /**
   * System channel that sends platform-oriented requests and information to Flutter, e.g., requests
   * to play sounds, requests for haptics, system chrome settings, etc.
   */
  @NonNull
  public PlatformChannel getPlatformChannel() {
    return platformChannel;
  }

  /** System channel that sends text processing requests from Flutter to Android. */
  @NonNull
  public ProcessTextChannel getProcessTextChannel() {
    return processTextChannel;
  }

  /**
   * System channel to exchange restoration data between framework and engine.
   *
   * <p>The engine can obtain the current restoration data from the framework via this channel to
   * store it on disk and - when the app is relaunched - provide the stored data back to the
   * framework to recreate the original state of the app.
   */
  @NonNull
  public RestorationChannel getRestorationChannel() {
    return restorationChannel;
  }

  /**
   * System channel that sends platform/user settings from Android to Flutter, e.g., time format,
   * scale factor, etc.
   */
  @NonNull
  public SettingsChannel getSettingsChannel() {
    return settingsChannel;
  }

  /** System channel that allows manual installation and state querying of deferred components. */
  @NonNull
  public DeferredComponentChannel getDeferredComponentChannel() {
    return deferredComponentChannel;
  }

  /** System channel that sends memory pressure warnings from Android to Flutter. */
  @NonNull
  public SystemChannel getSystemChannel() {
    return systemChannel;
  }

  /** System channel that sends and receives text input requests and state. */
  @NonNull
  public MouseCursorChannel getMouseCursorChannel() {
    return mouseCursorChannel;
  }

  /** System channel that sends and receives text input requests and state. */
  @NonNull
  public TextInputChannel getTextInputChannel() {
    return textInputChannel;
  }

  /** System channel that sends and receives Scribe requests and results. */
  @NonNull
  public ScribeChannel getScribeChannel() {
    return scribeChannel;
  }

  /** System channel that sends and receives spell check requests and results. */
  @NonNull
  public SpellCheckChannel getSpellCheckChannel() {
    return spellCheckChannel;
  }

  /**
   * Plugin registry, which registers plugins that want to be applied to this {@code FlutterEngine}.
   */
  @NonNull
  public PluginRegistry getPlugins() {
    return pluginRegistry;
  }

  /** The LocalizationPlugin this FlutterEngine created. */
  @NonNull
  public LocalizationPlugin getLocalizationPlugin() {
    return localizationPlugin;
  }

  /**
   * {@code PlatformViewsController}, which controls all platform views running within this {@code
   * FlutterEngine}.
   */
  @NonNull
  public PlatformViewsController getPlatformViewsController() {
    return platformViewsController;
  }

  @NonNull
  public PlatformViewsController2 getPlatformViewsController2() {
    return platformViewsController2;
  }

  @NonNull
  public ActivityControlSurface getActivityControlSurface() {
    return pluginRegistry;
  }

  @NonNull
  public ServiceControlSurface getServiceControlSurface() {
    return pluginRegistry;
  }

  @NonNull
  public BroadcastReceiverControlSurface getBroadcastReceiverControlSurface() {
    return pluginRegistry;
  }

  @NonNull
  public ContentProviderControlSurface getContentProviderControlSurface() {
    return pluginRegistry;
  }

  /** Returns unique identifier for this engine. */
  public long getEngineId() {
    return engineId;
  }

  /**
   * Returns engine for the given identifier or null if identifier is not valid. The handle can be
   * obtained through
   *
   * <pre>PlatformDispatcher.instance.engineId</pre>
   *
   * <p>Must be called on the UI thread.
   */
  @Nullable
  @VisibleForTesting(otherwise = VisibleForTesting.PACKAGE_PRIVATE)
  public static FlutterEngine engineForId(long handle) {
    return idToEngine.get(handle);
  }

  /** Lifecycle callbacks for Flutter engine lifecycle events. */
  public interface EngineLifecycleListener {
    /** Lifecycle callback invoked before a hot restart of the Flutter engine. */
    void onPreEngineRestart();
    /**
     * Lifecycle callback invoked before the Flutter engine is destroyed.
     *
     * <p>For the duration of the call, the Flutter engine is still valid.
     */
    void onEngineWillDestroy();
  }

  @Override
  public void updateDisplayMetrics(float width, float height, float density) {
    flutterJNI.updateDisplayMetrics(0 /* display ID */, width, height, density);
  }
}
