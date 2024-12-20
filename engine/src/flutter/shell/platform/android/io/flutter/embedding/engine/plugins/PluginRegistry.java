// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.plugins;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import java.util.Set;

public interface PluginRegistry {

  /**
   * Attaches the given {@code plugin} to the {@link io.flutter.embedding.engine.FlutterEngine}
   * associated with this {@code PluginRegistry}.
   */
  void add(@NonNull FlutterPlugin plugin);

  /**
   * Attaches the given {@code plugins} to the {@link io.flutter.embedding.engine.FlutterEngine}
   * associated with this {@code PluginRegistry}.
   */
  void add(@NonNull Set<FlutterPlugin> plugins);

  /**
   * Returns true if a plugin of the given type is currently attached to the {@link
   * io.flutter.embedding.engine.FlutterEngine} associated with this {@code PluginRegistry}.
   */
  boolean has(@NonNull Class<? extends FlutterPlugin> pluginClass);

  /**
   * Returns the instance of a plugin that is currently attached to the {@link
   * io.flutter.embedding.engine.FlutterEngine} associated with this {@code PluginRegistry}, which
   * matches the given {@code pluginClass}.
   *
   * <p>If no matching plugin is found, {@code null} is returned.
   */
  @Nullable
  FlutterPlugin get(@NonNull Class<? extends FlutterPlugin> pluginClass);

  /**
   * Detaches the plugin of the given type from the {@link
   * io.flutter.embedding.engine.FlutterEngine} associated with this {@code PluginRegistry}.
   *
   * <p>If no such plugin exists, this method does nothing.
   */
  void remove(@NonNull Class<? extends FlutterPlugin> pluginClass);

  /**
   * Detaches the plugins of the given types from the {@link
   * io.flutter.embedding.engine.FlutterEngine} associated with this {@code PluginRegistry}.
   *
   * <p>If no such plugins exist, this method does nothing.
   */
  void remove(@NonNull Set<Class<? extends FlutterPlugin>> plugins);

  /**
   * Detaches all plugins that are currently attached to the {@link
   * io.flutter.embedding.engine.FlutterEngine} associated with this {@code PluginRegistry}.
   *
   * <p>If no plugins are currently attached, this method does nothing.
   */
  void removeAll();
}
