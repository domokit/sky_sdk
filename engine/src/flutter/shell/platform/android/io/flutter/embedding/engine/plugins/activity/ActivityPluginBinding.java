// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.plugins.activity;

import android.app.Activity;
import android.os.Bundle;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import io.flutter.plugin.common.PluginRegistry;

/**
 * Binding that gives {@link ActivityAware} plugins access to an associated {@link
 * android.app.Activity} and the {@link android.app.Activity}'s lifecycle methods.
 *
 * <p>To obtain an instance of an {@code ActivityPluginBinding} in a Flutter plugin, implement the
 * {@link ActivityAware} interface. A binding is provided in {@link
 * ActivityAware#onAttachedToActivity(ActivityPluginBinding)} and {@link
 * ActivityAware#onReattachedToActivityForConfigChanges(ActivityPluginBinding)}.
 */
public interface ActivityPluginBinding {

  /**
   * Returns the {@link android.app.Activity} that is currently attached to the {@link
   * io.flutter.embedding.engine.FlutterEngine} that owns this {@code ActivityPluginBinding}.
   */
  @NonNull
  Activity getActivity();

  /**
   * Returns the {@code Lifecycle} associated with the attached {@code Activity}.
   *
   * <p>Use the flutter_plugin_android_lifecycle plugin to turn the returned {@code Object} into a
   * {@code Lifecycle} object. See
   * (https://github.com/flutter/plugins/tree/master/packages/flutter_plugin_android_lifecycle).
   * Flutter plugins that rely on {@code Lifecycle} are forced to use the
   * flutter_plugin_android_lifecycle plugin so that the version of the Android Lifecycle library is
   * exposed to pub, which allows Flutter to manage different versions library over time.
   */
  @NonNull
  Object getLifecycle();

  /**
   * Adds a listener that is invoked whenever the associated {@link android.app.Activity}'s {@code
   * onRequestPermissionsResult(...)} method is invoked.
   */
  void addRequestPermissionsResultListener(
      @NonNull PluginRegistry.RequestPermissionsResultListener listener);

  /**
   * Removes a listener that was added in {@link
   * #addRequestPermissionsResultListener(PluginRegistry.RequestPermissionsResultListener)}.
   */
  void removeRequestPermissionsResultListener(
      @NonNull PluginRegistry.RequestPermissionsResultListener listener);

  /**
   * Adds a listener that is invoked whenever the associated {@link android.app.Activity}'s {@code
   * onActivityResult(...)} method is invoked.
   */
  void addActivityResultListener(@NonNull PluginRegistry.ActivityResultListener listener);

  /**
   * Removes a listener that was added in {@link
   * #addActivityResultListener(PluginRegistry.ActivityResultListener)}.
   */
  void removeActivityResultListener(@NonNull PluginRegistry.ActivityResultListener listener);

  /**
   * Adds a listener that is invoked whenever the associated {@link android.app.Activity}'s {@code
   * onNewIntent(...)} method is invoked.
   */
  void addOnNewIntentListener(@NonNull PluginRegistry.NewIntentListener listener);

  /**
   * Removes a listener that was added in {@link
   * #addOnNewIntentListener(PluginRegistry.NewIntentListener)}.
   */
  void removeOnNewIntentListener(@NonNull PluginRegistry.NewIntentListener listener);

  /**
   * Adds a listener that is invoked whenever the associated {@link android.app.Activity}'s {@code
   * onUserLeaveHint()} method is invoked.
   */
  void addOnUserLeaveHintListener(@NonNull PluginRegistry.UserLeaveHintListener listener);

  /**
   * Removes a listener that was added in {@link
   * #addOnUserLeaveHintListener(PluginRegistry.UserLeaveHintListener)}.
   */
  void removeOnUserLeaveHintListener(@NonNull PluginRegistry.UserLeaveHintListener listener);

  /**
   * Adds a listener that is invoked whenever the associated {@link android.app.Activity}'s {@code
   * onWindowFocusChanged()} method is invoked.
   */
  void addOnWindowFocusChangedListener(@NonNull PluginRegistry.WindowFocusChangedListener listener);

  /**
   * Removes a listener that was added in {@link
   * #addOnWindowFocusChangedListener(PluginRegistry.WindowFocusChangedListener)}.
   */
  void removeOnWindowFocusChangedListener(
      @NonNull PluginRegistry.WindowFocusChangedListener listener);

  /**
   * Adds a listener that is invoked when the associated {@code Activity} or {@code Fragment} saves
   * and restores instance state.
   */
  void addOnSaveStateListener(@NonNull OnSaveInstanceStateListener listener);

  /**
   * Removes a listener that was added in {@link
   * #addOnSaveStateListener(OnSaveInstanceStateListener)}.
   */
  void removeOnSaveStateListener(@NonNull OnSaveInstanceStateListener listener);

  interface OnSaveInstanceStateListener {
    /**
     * Invoked when the associated {@code Activity} or {@code Fragment} executes {@link
     * Activity#onSaveInstanceState(Bundle)}.
     */
    void onSaveInstanceState(@NonNull Bundle bundle);

    /**
     * Invoked when the associated {@code Activity} executes {@link
     * android.app.Activity#onCreate(Bundle)} or associated {@code Fragment} executes {@code
     * Fragment#onCreate(Bundle)}.
     */
    void onRestoreInstanceState(@Nullable Bundle bundle);
  }
}
