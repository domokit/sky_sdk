// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.common;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import java.nio.ByteBuffer;

/**
 * A codec for method calls and enveloped results.
 *
 * <p>Method calls are encoded as binary messages with enough structure that the codec can extract a
 * method name String and an arguments Object. These data items are used to populate a {@link
 * MethodCall}.
 *
 * <p>All operations throw {@link IllegalArgumentException}, if conversion fails.
 */
public interface MethodCodec {
  /**
   * Encodes a message call into binary.
   *
   * @param methodCall a {@link MethodCall}.
   * @return a {@link ByteBuffer} containing the encoding between position 0 and the current
   *     position.
   */
  @NonNull
  ByteBuffer encodeMethodCall(@NonNull MethodCall methodCall);

  /**
   * Decodes a message call from binary.
   *
   * @param methodCall the binary encoding of the method call as a {@link ByteBuffer}.
   * @return a {@link MethodCall} representation of the bytes between the given buffer's current
   *     position and its limit.
   */
  @NonNull
  MethodCall decodeMethodCall(@NonNull ByteBuffer methodCall);

  /**
   * Encodes a successful result into a binary envelope message.
   *
   * @param result The result value, possibly null.
   * @return a {@link ByteBuffer} containing the encoding between position 0 and the current
   *     position.
   */
  @NonNull
  ByteBuffer encodeSuccessEnvelope(@Nullable Object result);

  /**
   * Encodes an error result into a binary envelope message.
   *
   * @param errorCode An error code String.
   * @param errorMessage An error message String, possibly null.
   * @param errorDetails Error details, possibly null. Consider supporting {@link Throwable} in your
   *     codec. This is the most common value passed to this field.
   * @return a {@link ByteBuffer} containing the encoding between position 0 and the current
   *     position.
   */
  @NonNull
  ByteBuffer encodeErrorEnvelope(
      @NonNull String errorCode, @Nullable String errorMessage, @Nullable Object errorDetails);

  /**
   * Encodes an error result into a binary envelope message with the native stacktrace.
   *
   * @param errorCode An error code String.
   * @param errorMessage An error message String, possibly null.
   * @param errorDetails Error details, possibly null. Consider supporting {@link Throwable} in your
   *     codec. This is the most common value passed to this field.
   * @param errorStacktrace Platform stacktrace for the error. possibly null.
   * @return a {@link ByteBuffer} containing the encoding between position 0 and the current
   *     position.
   */
  @NonNull
  ByteBuffer encodeErrorEnvelopeWithStacktrace(
      @NonNull String errorCode,
      @Nullable String errorMessage,
      @Nullable Object errorDetails,
      @Nullable String errorStacktrace);

  /**
   * Decodes a result envelope from binary.
   *
   * @param envelope the binary encoding of a result envelope as a {@link ByteBuffer}.
   * @return the enveloped result Object.
   * @throws FlutterException if the envelope was an error envelope.
   */
  @NonNull
  Object decodeEnvelope(@NonNull ByteBuffer envelope);
}
