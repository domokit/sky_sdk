// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine;

import android.content.Context;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.VisibleForTesting;
import io.flutter.FlutterInjector;
import io.flutter.embedding.engine.dart.DartExecutor.DartEntrypoint;
import io.flutter.embedding.engine.loader.FlutterLoader;
import io.flutter.plugin.platform.PlatformViewsController;
import java.util.ArrayList;
import java.util.List;

/**
 * Represents a collection of {@link io.flutter.embedding.engine.FlutterEngine}s who share resources
 * to allow them to be created faster and with less memory than calling the {@link
 * io.flutter.embedding.engine.FlutterEngine}'s constructor multiple times.
 *
 * <p>When creating or recreating the first {@link io.flutter.embedding.engine.FlutterEngine} in the
 * FlutterEngineGroup, the behavior is the same as creating a {@link
 * io.flutter.embedding.engine.FlutterEngine} via its constructor. When subsequent {@link
 * io.flutter.embedding.engine.FlutterEngine}s are created, resources from an existing living {@link
 * io.flutter.embedding.engine.FlutterEngine} is re-used.
 *
 * <p>The shared resources are kept until the last surviving {@link
 * io.flutter.embedding.engine.FlutterEngine} is destroyed.
 *
 * <p>Deleting a FlutterEngineGroup doesn't invalidate its existing {@link
 * io.flutter.embedding.engine.FlutterEngine}s, but it eliminates the possibility to create more
 * {@link io.flutter.embedding.engine.FlutterEngine}s in that group.
 */
public class FlutterEngineGroup {

  /* package */ @VisibleForTesting final List<FlutterEngine> activeEngines = new ArrayList<>();

  /**
   * Create a FlutterEngineGroup whose child engines will share resources.
   *
   * <p>Since the FlutterEngineGroup is likely to have a longer lifecycle than any individual
   * Android component, it's more semantically correct to pass in an application context rather than
   * the individual Android component's context to minimize the chances of leaks.
   */
  public FlutterEngineGroup(@NonNull Context context) {
    this(context, null);
  }

  /**
   * Create a FlutterEngineGroup whose child engines will share resources. Use {@code dartVmArgs} to
   * pass flags to the Dart VM during initialization.
   *
   * <p>Since the FlutterEngineGroup is likely to have a longer lifecycle than any individual
   * Android component, it's more semantically correct to pass in an application context rather than
   * the individual Android component's context to minimize the chances of leaks.
   */
  public FlutterEngineGroup(@NonNull Context context, @Nullable String[] dartVmArgs) {
    FlutterLoader loader = FlutterInjector.instance().flutterLoader();
    if (!loader.initialized()) {
      loader.startInitialization(context.getApplicationContext());
      loader.ensureInitializationComplete(context.getApplicationContext(), dartVmArgs);
    }
  }

  /**
   * Creates a {@link io.flutter.embedding.engine.FlutterEngine} in this group and run its {@link
   * io.flutter.embedding.engine.dart.DartExecutor} with a default entrypoint of the "main" function
   * in the "lib/main.dart" file.
   *
   * <p>If no prior {@link io.flutter.embedding.engine.FlutterEngine} were created in this group,
   * the initialization cost will be slightly higher than subsequent engines. The very first {@link
   * io.flutter.embedding.engine.FlutterEngine} created per program, regardless of
   * FlutterEngineGroup, also incurs the Dart VM creation time.
   *
   * <p>Subsequent engine creations will share resources with existing engines. However, if all
   * existing engines were {@link io.flutter.embedding.engine.FlutterEngine#destroy()}ed, the next
   * engine created will recreate its dependencies.
   */
  public FlutterEngine createAndRunDefaultEngine(@NonNull Context context) {
    return createAndRunEngine(context, null);
  }

  /**
   * Creates a {@link io.flutter.embedding.engine.FlutterEngine} in this group and run its {@link
   * io.flutter.embedding.engine.dart.DartExecutor} with the specified {@link DartEntrypoint}.
   *
   * <p>If no prior {@link io.flutter.embedding.engine.FlutterEngine} were created in this group,
   * the initialization cost will be slightly higher than subsequent engines. The very first {@link
   * io.flutter.embedding.engine.FlutterEngine} created per program, regardless of
   * FlutterEngineGroup, also incurs the Dart VM creation time.
   *
   * <p>Subsequent engine creations will share resources with existing engines. However, if all
   * existing engines were {@link io.flutter.embedding.engine.FlutterEngine#destroy()}ed, the next
   * engine created will recreate its dependencies.
   */
  public FlutterEngine createAndRunEngine(
      @NonNull Context context, @Nullable DartEntrypoint dartEntrypoint) {
    return createAndRunEngine(context, dartEntrypoint, null);
  }

  /**
   * Creates a {@link io.flutter.embedding.engine.FlutterEngine} in this group and run its {@link
   * io.flutter.embedding.engine.dart.DartExecutor} with the specified {@link DartEntrypoint} and
   * the specified {@code initialRoute}.
   *
   * <p>If no prior {@link io.flutter.embedding.engine.FlutterEngine} were created in this group,
   * the initialization cost will be slightly higher than subsequent engines. The very first {@link
   * io.flutter.embedding.engine.FlutterEngine} created per program, regardless of
   * FlutterEngineGroup, also incurs the Dart VM creation time.
   *
   * <p>Subsequent engine creations will share resources with existing engines. However, if all
   * existing engines were {@link io.flutter.embedding.engine.FlutterEngine#destroy()}ed, the next
   * engine created will recreate its dependencies.
   */
  public FlutterEngine createAndRunEngine(
      @NonNull Context context,
      @Nullable DartEntrypoint dartEntrypoint,
      @Nullable String initialRoute) {
    return createAndRunEngine(
        new Options(context).setDartEntrypoint(dartEntrypoint).setInitialRoute(initialRoute));
  }

  /**
   * Creates a {@link io.flutter.embedding.engine.FlutterEngine} in this group and run its {@link
   * io.flutter.embedding.engine.dart.DartExecutor} with the specified {@link DartEntrypoint}, the
   * specified {@code initialRoute} and the {@code dartEntrypointArgs}.
   *
   * <p>If no prior {@link io.flutter.embedding.engine.FlutterEngine} were created in this group,
   * the initialization cost will be slightly higher than subsequent engines. The very first {@link
   * io.flutter.embedding.engine.FlutterEngine} created per program, regardless of
   * FlutterEngineGroup, also incurs the Dart VM creation time.
   *
   * <p>Subsequent engine creations will share resources with existing engines. However, if all
   * existing engines were {@link io.flutter.embedding.engine.FlutterEngine#destroy()}ed, the next
   * engine created will recreate its dependencies.
   */
  public FlutterEngine createAndRunEngine(@NonNull Options options) {
    FlutterEngine engine = null;

    Context context = options.getContext();
    DartEntrypoint dartEntrypoint = options.getDartEntrypoint();
    String initialRoute = options.getInitialRoute();
    List<String> dartEntrypointArgs = options.getDartEntrypointArgs();
    PlatformViewsController platformViewsController = options.getPlatformViewsController();
    platformViewsController =
        platformViewsController != null ? platformViewsController : new PlatformViewsController();
    boolean automaticallyRegisterPlugins = options.getAutomaticallyRegisterPlugins();
    boolean waitForRestorationData = options.getWaitForRestorationData();

    if (dartEntrypoint == null) {
      dartEntrypoint = DartEntrypoint.createDefault();
    }

    if (activeEngines.size() == 0) {
      engine =
          createEngine(
              context,
              platformViewsController,
              automaticallyRegisterPlugins,
              waitForRestorationData);
      if (initialRoute != null) {
        engine.getNavigationChannel().setInitialRoute(initialRoute);
      }
      engine.getDartExecutor().executeDartEntrypoint(dartEntrypoint, dartEntrypointArgs);
    } else {
      engine =
          activeEngines
              .get(0)
              .spawn(
                  context,
                  dartEntrypoint,
                  initialRoute,
                  dartEntrypointArgs,
                  platformViewsController,
                  automaticallyRegisterPlugins,
                  waitForRestorationData);
    }

    activeEngines.add(engine);

    final FlutterEngine engineToCleanUpOnDestroy = engine;
    engine.addEngineLifecycleListener(
        new FlutterEngine.EngineLifecycleListener() {

          @Override
          public void onPreEngineRestart() {
            // No-op. Not interested.
          }

          @Override
          public void onEngineWillDestroy() {
            activeEngines.remove(engineToCleanUpOnDestroy);
          }
        });
    return engine;
  }

  @VisibleForTesting
  /* package */ FlutterEngine createEngine(
      Context context,
      @NonNull PlatformViewsController platformViewsController,
      boolean automaticallyRegisterPlugins,
      boolean waitForRestorationData) {
    return new FlutterEngine(
        context, // Context.
        null, // FlutterLoader.
        null, // FlutterJNI.
        platformViewsController, // PlatformViewsController.
        null, // String[]. The Dart VM has already started, this arguments will have no effect.
        automaticallyRegisterPlugins, // boolean.
        waitForRestorationData, // boolean.
        this);
  }

  /** Options that control how a FlutterEngine should be created. */
  public static class Options {
    @NonNull private Context context;
    @Nullable private DartEntrypoint dartEntrypoint;
    @Nullable private String initialRoute;
    @Nullable private List<String> dartEntrypointArgs;
    @NonNull private PlatformViewsController platformViewsController;
    private boolean automaticallyRegisterPlugins = true;
    private boolean waitForRestorationData = false;

    public Options(@NonNull Context context) {
      this.context = context;
    }

    public Context getContext() {
      return context;
    }

    /**
     * dartEntrypoint specifies the {@link DartEntrypoint} the new engine should run. It doesn't
     * need to be the same entrypoint as the current engine but must be built in the same AOT or
     * snapshot.
     */
    public DartEntrypoint getDartEntrypoint() {
      return dartEntrypoint;
    }

    /**
     * The name of the initial Flutter `Navigator` `Route` to load. If this is null, it will default
     * to the "/" route.
     */
    public String getInitialRoute() {
      return initialRoute;
    }

    /** Arguments passed as a list of string to Dart's entrypoint function. */
    public List<String> getDartEntrypointArgs() {
      return dartEntrypointArgs;
    }

    /** Manages platform views. */
    public PlatformViewsController getPlatformViewsController() {
      return platformViewsController;
    }

    /**
     * If plugins are automatically registered, then they are registered during the {@link
     * io.flutter.embedding.engine.FlutterEngine}'s constructor.
     */
    public boolean getAutomaticallyRegisterPlugins() {
      return automaticallyRegisterPlugins;
    }

    /**
     * The waitForRestorationData flag controls whether the engine delays responding to requests
     * from the framework for restoration data until that data has been provided to the engine via
     * {@code RestorationChannel.setRestorationData(byte[] data)}.
     */
    public boolean getWaitForRestorationData() {
      return waitForRestorationData;
    }

    /**
     * Setter for `dartEntrypoint` property.
     *
     * @param dartEntrypoint specifies the {@link DartEntrypoint} the new engine should run. It
     *     doesn't need to be the same entrypoint as the current engine but must be built in the
     *     same AOT or snapshot.
     */
    public Options setDartEntrypoint(DartEntrypoint dartEntrypoint) {
      this.dartEntrypoint = dartEntrypoint;
      return this;
    }

    /**
     * Setter for `initialRoute` property.
     *
     * @param initialRoute The name of the initial Flutter `Navigator` `Route` to load. If this is
     *     null, it will default to the "/" route.
     */
    public Options setInitialRoute(String initialRoute) {
      this.initialRoute = initialRoute;
      return this;
    }

    /**
     * Setter for `dartEntrypointArgs` property.
     *
     * @param dartEntrypointArgs Arguments passed as a list of string to Dart's entrypoint function.
     */
    public Options setDartEntrypointArgs(List<String> dartEntrypointArgs) {
      this.dartEntrypointArgs = dartEntrypointArgs;
      return this;
    }

    /**
     * Setter for `platformViewsController` property.
     *
     * @param platformViewsController Manages platform views.
     */
    public Options setPlatformViewsController(
        @NonNull PlatformViewsController platformViewsController) {
      this.platformViewsController = platformViewsController;
      return this;
    }

    /**
     * Setter for `automaticallyRegisterPlugins` property.
     *
     * @param automaticallyRegisterPlugins If plugins are automatically registered, then they are
     *     registered during the execution of {@link io.flutter.embedding.engine.FlutterEngine}'s
     *     constructor.
     */
    public Options setAutomaticallyRegisterPlugins(boolean automaticallyRegisterPlugins) {
      this.automaticallyRegisterPlugins = automaticallyRegisterPlugins;
      return this;
    }

    /**
     * Setter for `waitForRestorationData` property.
     *
     * @param waitForRestorationData The waitForRestorationData flag controls whether the engine
     *     delays responding to requests from the framework for restoration data until that data has
     *     been provided to the engine via {@code RestorationChannel.setRestorationData(byte[]
     *     data)}.
     */
    public Options setWaitForRestorationData(boolean waitForRestorationData) {
      this.waitForRestorationData = waitForRestorationData;
      return this;
    }
  }
}
