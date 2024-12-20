// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.systemchannels;

import androidx.annotation.NonNull;
import io.flutter.Log;
import io.flutter.embedding.engine.dart.DartExecutor;
import io.flutter.plugin.common.BasicMessageChannel;
import io.flutter.plugin.common.JSONMessageCodec;
import java.util.HashMap;
import java.util.Map;

/** TODO(mattcarroll): fill in javadoc for SystemChannel. */
public class SystemChannel {
  private static final String TAG = "SystemChannel";

  @NonNull public final BasicMessageChannel<Object> channel;

  public SystemChannel(@NonNull DartExecutor dartExecutor) {
    this.channel =
        new BasicMessageChannel<>(dartExecutor, "flutter/system", JSONMessageCodec.INSTANCE);
  }

  public void sendMemoryPressureWarning() {
    Log.v(TAG, "Sending memory pressure warning to Flutter.");
    Map<String, Object> message = new HashMap<>(1);
    message.put("type", "memoryPressure");
    channel.send(message);
  }
}
