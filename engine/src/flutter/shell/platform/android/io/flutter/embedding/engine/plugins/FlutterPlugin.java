// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.plugins;

import android.content.Context;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.lifecycle.Lifecycle;
import io.flutter.embedding.engine.FlutterEngine;
import io.flutter.embedding.engine.FlutterEngineGroup;
import io.flutter.plugin.common.BinaryMessenger;
import io.flutter.plugin.platform.PlatformViewRegistry;
import io.flutter.view.TextureRegistry;

/**
 * Interface to be implemented by all Flutter plugins.
 *
 * <p>A Flutter plugin allows Flutter developers to interact with a host platform, e.g., Android and
 * iOS, via Dart code. It includes platform code, as well as Dart code. A plugin author is
 * responsible for setting up an appropriate {@link io.flutter.plugin.common.MethodChannel} to
 * communicate between platform code and Dart code.
 *
 * <p>A Flutter plugin has a lifecycle. First, a developer must add a {@code FlutterPlugin} to an
 * instance of {@link io.flutter.embedding.engine.FlutterEngine}. To do this, obtain a {@link
 * PluginRegistry} with {@link FlutterEngine#getPlugins()}, then call {@link
 * PluginRegistry#add(FlutterPlugin)}, passing the instance of the Flutter plugin. During the call
 * to {@link PluginRegistry#add(FlutterPlugin)}, the {@link
 * io.flutter.embedding.engine.FlutterEngine} will invoke {@link
 * #onAttachedToEngine(FlutterPluginBinding)} on the given {@code FlutterPlugin}. If the {@code
 * FlutterPlugin} is removed from the {@link io.flutter.embedding.engine.FlutterEngine} via {@link
 * PluginRegistry#remove(Class)}, or if the {@link io.flutter.embedding.engine.FlutterEngine} is
 * destroyed, the {@link FlutterEngine} will invoke {@link
 * FlutterPlugin#onDetachedFromEngine(FlutterPluginBinding)} on the given {@code FlutterPlugin}.
 *
 * <p>Once a {@code FlutterPlugin} is attached to a {@link
 * io.flutter.embedding.engine.FlutterEngine}, the plugin's code is permitted to access and invoke
 * methods on resources within the {@link FlutterPluginBinding} that the {@link
 * io.flutter.embedding.engine.FlutterEngine} gave to the {@code FlutterPlugin} in {@link
 * #onAttachedToEngine(FlutterPluginBinding)}. This includes, for example, the application {@link
 * Context} for the running app.
 *
 * <p>The {@link FlutterPluginBinding} provided in {@link #onAttachedToEngine(FlutterPluginBinding)}
 * is no longer valid after the execution of {@link #onDetachedFromEngine(FlutterPluginBinding)}. Do
 * not access any properties of the {@link FlutterPluginBinding} after the completion of {@link
 * #onDetachedFromEngine(FlutterPluginBinding)}.
 *
 * <p>To register a {@link io.flutter.plugin.common.MethodChannel}, obtain a {@link BinaryMessenger}
 * via the {@link FlutterPluginBinding}.
 *
 * <p>An Android Flutter plugin may require access to app resources or other artifacts that can only
 * be retrieved through a {@link Context}. Developers can access the application context via {@link
 * FlutterPluginBinding#getApplicationContext()}.
 *
 * <p>Some plugins may require access to the {@code Activity} that is displaying a Flutter
 * experience, or may need to react to {@code Activity} lifecycle events, e.g., {@code onCreate()},
 * {@code onStart()}, {@code onResume()}, {@code onPause()}, {@code onStop()}, {@code onDestroy()}.
 * Any such plugin should implement {@link
 * io.flutter.embedding.engine.plugins.activity.ActivityAware} in addition to implementing {@code
 * FlutterPlugin}. {@code ActivityAware} provides callback hooks that expose access to an associated
 * {@code Activity} and its {@code Lifecycle}. All plugins must respect the possibility that a
 * Flutter experience may never be associated with an {@code Activity}, e.g., when Flutter is used
 * for background behavior. Additionally, all plugins must respect that a {@code Activity}s may come
 * and go over time, thus requiring plugins to cleanup resources and recreate those resources as the
 * {@code Activity} comes and goes.
 */
public interface FlutterPlugin {

  /**
   * This {@code FlutterPlugin} has been associated with a {@link
   * io.flutter.embedding.engine.FlutterEngine} instance.
   *
   * <p>Relevant resources that this {@code FlutterPlugin} may need are provided via the {@code
   * binding}. The {@code binding} may be cached and referenced until {@link
   * #onDetachedFromEngine(FlutterPluginBinding)} is invoked and returns.
   */
  void onAttachedToEngine(@NonNull FlutterPluginBinding binding);

  /**
   * This {@code FlutterPlugin} has been removed from a {@link
   * io.flutter.embedding.engine.FlutterEngine} instance.
   *
   * <p>The {@code binding} passed to this method is the same instance that was passed in {@link
   * #onAttachedToEngine(FlutterPluginBinding)}. It is provided again in this method as a
   * convenience. The {@code binding} may be referenced during the execution of this method, but it
   * must not be cached or referenced after this method returns.
   *
   * <p>{@code FlutterPlugin}s should release all resources in this method.
   */
  void onDetachedFromEngine(@NonNull FlutterPluginBinding binding);

  /**
   * Resources made available to all plugins registered with a given {@link
   * io.flutter.embedding.engine.FlutterEngine}.
   *
   * <p>The provided {@link BinaryMessenger} can be used to communicate with Dart code running in
   * the Flutter context associated with this plugin binding.
   *
   * <p>Plugins that need to respond to {@code Lifecycle} events should implement the additional
   * {@link io.flutter.embedding.engine.plugins.activity.ActivityAware} and/or {@link
   * io.flutter.embedding.engine.plugins.service.ServiceAware} interfaces, where a {@link Lifecycle}
   * reference can be obtained.
   */
  class FlutterPluginBinding {
    private final Context applicationContext;
    private final FlutterEngine flutterEngine;
    private final BinaryMessenger binaryMessenger;
    private final TextureRegistry textureRegistry;
    private final PlatformViewRegistry platformViewRegistry;
    private final FlutterAssets flutterAssets;
    private final FlutterEngineGroup group;

    public FlutterPluginBinding(
        @NonNull Context applicationContext,
        @NonNull FlutterEngine flutterEngine,
        @NonNull BinaryMessenger binaryMessenger,
        @NonNull TextureRegistry textureRegistry,
        @NonNull PlatformViewRegistry platformViewRegistry,
        @NonNull FlutterAssets flutterAssets,
        @Nullable FlutterEngineGroup group) {
      this.applicationContext = applicationContext;
      this.flutterEngine = flutterEngine;
      this.binaryMessenger = binaryMessenger;
      this.textureRegistry = textureRegistry;
      this.platformViewRegistry = platformViewRegistry;
      this.flutterAssets = flutterAssets;
      this.group = group;
    }

    @NonNull
    public Context getApplicationContext() {
      return applicationContext;
    }

    /**
     * @deprecated Use {@code getBinaryMessenger()}, {@code getTextureRegistry()}, or {@code
     *     getPlatformViewRegistry()} instead.
     */
    @Deprecated
    @NonNull
    public FlutterEngine getFlutterEngine() {
      return flutterEngine;
    }

    @NonNull
    public BinaryMessenger getBinaryMessenger() {
      return binaryMessenger;
    }

    @NonNull
    public TextureRegistry getTextureRegistry() {
      return textureRegistry;
    }

    @NonNull
    public PlatformViewRegistry getPlatformViewRegistry() {
      return platformViewRegistry;
    }

    @NonNull
    public FlutterAssets getFlutterAssets() {
      return flutterAssets;
    }

    /**
     * Accessor for the {@link FlutterEngineGroup} used to create the {@link FlutterEngine} for the
     * app.
     *
     * <p>This is useful in the rare case that a plugin has to spawn its own engine (for example,
     * running an engine the background). The result is nullable since old versions of Flutter and
     * custom setups may not have used a {@link FlutterEngineGroup}. Failing to use this when it is
     * available will result in suboptimal performance and odd behaviors related to Dart isolate
     * groups.
     */
    @Nullable
    public FlutterEngineGroup getEngineGroup() {
      return group;
    }
  }

  /** Provides Flutter plugins with access to Flutter asset information. */
  interface FlutterAssets {
    /**
     * Returns the relative file path to the Flutter asset with the given name, including the file's
     * extension, e.g., {@code "myImage.jpg"}.
     *
     * <p>The returned file path is relative to the Android app's standard assets directory.
     * Therefore, the returned path is appropriate to pass to Android's {@code AssetManager}, but
     * the path is not appropriate to load as an absolute path.
     */
    String getAssetFilePathByName(@NonNull String assetFileName);

    /**
     * Same as {@link #getAssetFilePathByName(String)} but with added support for an explicit
     * Android {@code packageName}.
     */
    String getAssetFilePathByName(@NonNull String assetFileName, @NonNull String packageName);

    /**
     * Returns the relative file path to the Flutter asset with the given subpath, including the
     * file's extension, e.g., {@code "/dir1/dir2/myImage.jpg"}.
     *
     * <p>The returned file path is relative to the Android app's standard assets directory.
     * Therefore, the returned path is appropriate to pass to Android's {@code AssetManager}, but
     * the path is not appropriate to load as an absolute path.
     */
    String getAssetFilePathBySubpath(@NonNull String assetSubpath);

    /**
     * Same as {@link #getAssetFilePathBySubpath(String)} but with added support for an explicit
     * Android {@code packageName}.
     */
    String getAssetFilePathBySubpath(@NonNull String assetSubpath, @NonNull String packageName);
  }
}
