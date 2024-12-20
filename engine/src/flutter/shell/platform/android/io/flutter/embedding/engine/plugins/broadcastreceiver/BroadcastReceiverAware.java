// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.plugins.broadcastreceiver;

import androidx.annotation.NonNull;

/**
 * A {@link io.flutter.embedding.engine.plugins.FlutterPlugin} that wants to know when it is running
 * within a {@link android.content.BroadcastReceiver}.
 */
public interface BroadcastReceiverAware {
  /**
   * Callback triggered when a {@code BroadcastReceiverAware} {@link
   * io.flutter.embedding.engine.plugins.FlutterPlugin} is associated with a {@link
   * android.content.BroadcastReceiver}.
   */
  void onAttachedToBroadcastReceiver(@NonNull BroadcastReceiverPluginBinding binding);

  /**
   * Callback triggered when a {@code BroadcastReceiverAware} {@link
   * io.flutter.embedding.engine.plugins.FlutterPlugin} is detached from a {@link
   * android.content.BroadcastReceiver}.
   */
  void onDetachedFromBroadcastReceiver();
}
