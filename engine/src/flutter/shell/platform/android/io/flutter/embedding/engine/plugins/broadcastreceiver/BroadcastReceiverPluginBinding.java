// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.plugins.broadcastreceiver;

import android.content.BroadcastReceiver;
import androidx.annotation.NonNull;

/**
 * Binding that gives {@link BroadcastReceiverAware} plugins access to an associated {@link
 * BroadcastReceiver}.
 */
public interface BroadcastReceiverPluginBinding {

  /**
   * Returns the {@link BroadcastReceiver} that is currently attached to the {@link
   * io.flutter.embedding.engine.FlutterEngine} that owns this {@code
   * BroadcastReceiverAwarePluginBinding}.
   */
  @NonNull
  BroadcastReceiver getBroadcastReceiver();
}
