// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.android;

import androidx.annotation.NonNull;
import androidx.lifecycle.Lifecycle;
import io.flutter.embedding.engine.FlutterEngine;

/**
 * Configures a {@link io.flutter.embedding.engine.FlutterEngine} after it is created, e.g., adds
 * plugins.
 *
 * <p>This interface may be applied to a {@link androidx.fragment.app.FragmentActivity} that owns a
 * {@code FlutterFragment}.
 */
public interface FlutterEngineConfigurator {
  /**
   * Configures the given {@link io.flutter.embedding.engine.FlutterEngine}.
   *
   * <p>This method is called after the given {@link io.flutter.embedding.engine.FlutterEngine} has
   * been attached to the owning {@code FragmentActivity}. See {@link
   * io.flutter.embedding.engine.plugins.activity.ActivityControlSurface#attachToActivity(
   * ExclusiveAppComponent, Lifecycle)}.
   *
   * <p>It is possible that the owning {@code FragmentActivity} opted not to connect itself as an
   * {@link io.flutter.embedding.engine.plugins.activity.ActivityControlSurface}. In that case, any
   * configuration, e.g., plugins, must not expect or depend upon an available {@code Activity} at
   * the time that this method is invoked.
   *
   * @param flutterEngine The Flutter engine.
   */
  void configureFlutterEngine(@NonNull FlutterEngine flutterEngine);

  /**
   * Cleans up references that were established in {@link #configureFlutterEngine(FlutterEngine)}
   * before the host is destroyed or detached.
   *
   * @param flutterEngine The Flutter engine.
   */
  void cleanUpFlutterEngine(@NonNull FlutterEngine flutterEngine);
}
