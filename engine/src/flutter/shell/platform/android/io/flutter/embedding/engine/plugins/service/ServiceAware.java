// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.plugins.service;

import androidx.annotation.NonNull;

/**
 * A {@link io.flutter.embedding.engine.plugins.FlutterPlugin} that wants to know when it is running
 * within a {@link android.app.Service}.
 */
public interface ServiceAware {
  /**
   * Callback triggered when a {@code ServiceAware} {@link
   * io.flutter.embedding.engine.plugins.FlutterPlugin} is associated with a {@link
   * android.app.Service}.
   */
  void onAttachedToService(@NonNull ServicePluginBinding binding);

  /**
   * Callback triggered when a {@code ServiceAware} {@link
   * io.flutter.embedding.engine.plugins.FlutterPlugin} is detached from a {@link
   * android.app.Service}.
   *
   * <p>Any {@code Lifecycle} listeners that were registered in {@link
   * #onAttachedToService(ServicePluginBinding)} should be deregistered here to avoid a possible
   * memory leak and other side effects.
   */
  void onDetachedFromService();

  interface OnModeChangeListener {
    /**
     * Callback triggered when the associated {@link android.app.Service} goes from background
     * execution to foreground execution.
     */
    void onMoveToForeground();

    /**
     * Callback triggered when the associated {@link android.app.Service} goes from foreground
     * execution to background execution.
     */
    void onMoveToBackground();
  }
}
