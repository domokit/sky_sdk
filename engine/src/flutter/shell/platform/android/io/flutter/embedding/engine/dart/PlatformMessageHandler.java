// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.dart;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import java.nio.ByteBuffer;

/** Handler that receives messages from Dart code. */
public interface PlatformMessageHandler {
  /** Called from any thread. */
  void handleMessageFromDart(
      @NonNull final String channel,
      @Nullable ByteBuffer message,
      final int replyId,
      long messageData);

  void handlePlatformMessageResponse(int replyId, @Nullable ByteBuffer reply);
}
