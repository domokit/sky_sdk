// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.common;

import androidx.annotation.Nullable;
import java.nio.ByteBuffer;
import java.nio.charset.Charset;

/**
 * A {@link MessageCodec} using UTF-8 encoded String messages.
 *
 * <p>This codec is guaranteed to be compatible with the corresponding <a
 * href="https://api.flutter.dev/flutter/services/StringCodec-class.html">StringCodec</a> on the
 * Dart side. These parts of the Flutter SDK are evolved synchronously.
 */
public final class StringCodec implements MessageCodec<String> {
  private static final Charset UTF8 = Charset.forName("UTF8");
  public static final StringCodec INSTANCE = new StringCodec();

  private StringCodec() {}

  @Override
  @Nullable
  public ByteBuffer encodeMessage(@Nullable String message) {
    if (message == null) {
      return null;
    }
    // TODO(mravn): Avoid the extra copy below.
    final byte[] bytes = message.getBytes(UTF8);
    final ByteBuffer buffer = ByteBuffer.allocateDirect(bytes.length);
    buffer.put(bytes);
    return buffer;
  }

  @Override
  @Nullable
  public String decodeMessage(@Nullable ByteBuffer message) {
    if (message == null) {
      return null;
    }
    final byte[] bytes;
    final int offset;
    final int length = message.remaining();
    if (message.hasArray()) {
      bytes = message.array();
      offset = message.arrayOffset();
    } else {
      // TODO(mravn): Avoid the extra copy below.
      bytes = new byte[length];
      message.get(bytes);
      offset = 0;
    }
    return new String(bytes, offset, length, UTF8);
  }
}
