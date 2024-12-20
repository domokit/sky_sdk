// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.VisibleForTesting;
import io.flutter.embedding.engine.FlutterJNI;
import io.flutter.embedding.engine.deferredcomponents.DeferredComponentManager;
import io.flutter.embedding.engine.loader.FlutterLoader;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadFactory;

/**
 * This class is a simple dependency injector for the relatively thin Android part of the Flutter
 * engine.
 *
 * <p>This simple solution is used facilitate testability without bringing in heavier
 * app-development centric dependency injection frameworks such as Guice or Dagger2 or spreading
 * construction injection everywhere.
 */
public final class FlutterInjector {

  private static FlutterInjector instance;
  private static boolean accessed;

  /**
   * Use {@link FlutterInjector.Builder} to specify members to be injected via the static {@code
   * FlutterInjector}.
   *
   * <p>This can only be called at the beginning of the program before the {@link #instance()} is
   * accessed.
   */
  public static void setInstance(@NonNull FlutterInjector injector) {
    if (accessed) {
      throw new IllegalStateException(
          "Cannot change the FlutterInjector instance once it's been "
              + "read. If you're trying to dependency inject, be sure to do so at the beginning of "
              + "the program");
    }
    instance = injector;
  }

  /**
   * Retrieve the static instance of the {@code FlutterInjector} to use in your program.
   *
   * <p>Once you access it, you can no longer change the values injected.
   *
   * <p>If no override is provided for the injector, reasonable defaults are provided.
   */
  public static FlutterInjector instance() {
    accessed = true;
    if (instance == null) {
      instance = new Builder().build();
    }
    return instance;
  }

  // This whole class is here to enable testing so to test the thing that lets you test, some degree
  // of hack is needed.
  @VisibleForTesting
  public static void reset() {
    accessed = false;
    instance = null;
  }

  private FlutterInjector(
      @NonNull FlutterLoader flutterLoader,
      @Nullable DeferredComponentManager deferredComponentManager,
      @NonNull FlutterJNI.Factory flutterJniFactory,
      @NonNull ExecutorService executorService) {
    this.flutterLoader = flutterLoader;
    this.deferredComponentManager = deferredComponentManager;
    this.flutterJniFactory = flutterJniFactory;
    this.executorService = executorService;
  }

  private FlutterLoader flutterLoader;
  private DeferredComponentManager deferredComponentManager;
  private FlutterJNI.Factory flutterJniFactory;
  private ExecutorService executorService;
  /**
   * Returns the {@link io.flutter.embedding.engine.loader.FlutterLoader} instance to use for the
   * Flutter Android engine embedding.
   */
  @NonNull
  public FlutterLoader flutterLoader() {
    return flutterLoader;
  }

  /**
   * Returns the {@link DeferredComponentManager} instance to use for the Flutter Android engine
   * embedding.
   */
  @Nullable
  public DeferredComponentManager deferredComponentManager() {
    return deferredComponentManager;
  }

  public ExecutorService executorService() {
    return executorService;
  }

  @NonNull
  public FlutterJNI.Factory getFlutterJNIFactory() {
    return flutterJniFactory;
  }

  /**
   * Builder used to supply a custom FlutterInjector instance to {@link
   * FlutterInjector#setInstance(FlutterInjector)}.
   *
   * <p>Non-overridden values have reasonable defaults.
   */
  public static final class Builder {
    private class NamedThreadFactory implements ThreadFactory {
      private int threadId = 0;

      public Thread newThread(Runnable command) {
        Thread thread = new Thread(command);
        thread.setName("flutter-worker-" + threadId++);
        return thread;
      }
    }

    private FlutterLoader flutterLoader;
    private DeferredComponentManager deferredComponentManager;
    private FlutterJNI.Factory flutterJniFactory;
    private ExecutorService executorService;
    /**
     * Sets a {@link io.flutter.embedding.engine.loader.FlutterLoader} override.
     *
     * <p>A reasonable default will be used if unspecified.
     */
    public Builder setFlutterLoader(@NonNull FlutterLoader flutterLoader) {
      this.flutterLoader = flutterLoader;
      return this;
    }

    public Builder setDeferredComponentManager(
        @Nullable DeferredComponentManager deferredComponentManager) {
      this.deferredComponentManager = deferredComponentManager;
      return this;
    }

    public Builder setFlutterJNIFactory(@NonNull FlutterJNI.Factory factory) {
      this.flutterJniFactory = factory;
      return this;
    }

    public Builder setExecutorService(@NonNull ExecutorService executorService) {
      this.executorService = executorService;
      return this;
    }

    private void fillDefaults() {
      if (flutterJniFactory == null) {
        flutterJniFactory = new FlutterJNI.Factory();
      }

      if (executorService == null) {
        executorService = Executors.newCachedThreadPool(new NamedThreadFactory());
      }

      if (flutterLoader == null) {
        flutterLoader = new FlutterLoader(flutterJniFactory.provideFlutterJNI(), executorService);
      }
      // DeferredComponentManager's intended default is null.
    }

    /**
     * Builds a {@link FlutterInjector} from the builder. Unspecified properties will have
     * reasonable defaults.
     */
    public FlutterInjector build() {
      fillDefaults();

      return new FlutterInjector(
          flutterLoader, deferredComponentManager, flutterJniFactory, executorService);
    }
  }
}
