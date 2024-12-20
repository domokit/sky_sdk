// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.plugins.service;

import android.app.Service;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

/** Binding that gives {@link ServiceAware} plugins access to an associated {@link Service}. */
public interface ServicePluginBinding {

  /**
   * Returns the {@link Service} that is currently attached to the {@link
   * io.flutter.embedding.engine.FlutterEngine} that owns this {@code ServicePluginBinding}.
   */
  @NonNull
  Service getService();

  /**
   * Returns the {@code Lifecycle} associated with the attached {@code Service}.
   *
   * <p>Use the flutter_plugin_android_lifecycle plugin to turn the returned {@code Object} into a
   * {@code Lifecycle} object. See
   * (https://github.com/flutter/plugins/tree/master/packages/flutter_plugin_android_lifecycle).
   * Flutter plugins that rely on {@code Lifecycle} are forced to use the
   * flutter_plugin_android_lifecycle plugin so that the version of the Android Lifecycle library is
   * exposed to pub, which allows Flutter to manage different versions library over time.
   */
  @Nullable
  Object getLifecycle();

  /**
   * Adds the given {@code listener} to be notified when the associated {@link Service} goes from
   * background to foreground, or foreground to background.
   */
  void addOnModeChangeListener(@NonNull ServiceAware.OnModeChangeListener listener);

  /**
   * Removes the given {@code listener}, which was previously added with {@link
   * #addOnModeChangeListener(ServiceAware.OnModeChangeListener)}.
   */
  void removeOnModeChangeListener(@NonNull ServiceAware.OnModeChangeListener listener);
}
