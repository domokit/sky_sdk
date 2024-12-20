// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.common;

import androidx.annotation.Nullable;
import java.nio.ByteBuffer;

/**
 * A message encoding/decoding mechanism.
 *
 * <p>Both operations throw {@link IllegalArgumentException}, if conversion fails.
 */
public interface MessageCodec<T> {
  /**
   * Encodes the specified message into binary.
   *
   * @param message the T message, possibly null.
   * @return a ByteBuffer containing the encoding between position 0 and the current position, or
   *     null, if message is null.
   */
  @Nullable
  ByteBuffer encodeMessage(@Nullable T message);

  /**
   * Decodes the specified message from binary.
   *
   * <p><b>Warning:</b> The ByteBuffer is "direct" and it won't be valid beyond this call. Storing
   * the ByteBuffer and using it later and will lead to a {@code java.nio.BufferUnderflowException}.
   * If you want to retain the data you'll need to copy it.
   *
   * @param message the {@link ByteBuffer} message, possibly null.
   * @return a T value representation of the bytes between the given buffer's current position and
   *     its limit, or null, if message is null.
   */
  @Nullable
  T decodeMessage(@Nullable ByteBuffer message);
}
