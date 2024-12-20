// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.common;

import androidx.annotation.NonNull;
import io.flutter.Log;
import io.flutter.plugin.common.StandardMessageCodec.ExposedByteArrayOutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * A {@link MethodCodec} using the Flutter standard binary encoding.
 *
 * <p>This codec is guaranteed to be compatible with the corresponding <a
 * href="https://api.flutter.dev/flutter/services/StandardMethodCodec-class.html">StandardMethodCodec</a>
 * on the Dart side. These parts of the Flutter SDK are evolved synchronously.
 *
 * <p>Values supported as method arguments and result payloads are those supported by {@link
 * StandardMessageCodec}.
 */
public final class StandardMethodCodec implements MethodCodec {
  public static final StandardMethodCodec INSTANCE =
      new StandardMethodCodec(StandardMessageCodec.INSTANCE);
  private final StandardMessageCodec messageCodec;

  /** Creates a new method codec based on the specified message codec. */
  public StandardMethodCodec(@NonNull StandardMessageCodec messageCodec) {
    this.messageCodec = messageCodec;
  }

  @Override
  @NonNull
  public ByteBuffer encodeMethodCall(@NonNull MethodCall methodCall) {
    final ExposedByteArrayOutputStream stream = new ExposedByteArrayOutputStream();
    messageCodec.writeValue(stream, methodCall.method);
    messageCodec.writeValue(stream, methodCall.arguments);
    final ByteBuffer buffer = ByteBuffer.allocateDirect(stream.size());
    buffer.put(stream.buffer(), 0, stream.size());
    return buffer;
  }

  @Override
  @NonNull
  public MethodCall decodeMethodCall(@NonNull ByteBuffer methodCall) {
    methodCall.order(ByteOrder.nativeOrder());
    final Object method = messageCodec.readValue(methodCall);
    final Object arguments = messageCodec.readValue(methodCall);
    if (method instanceof String && !methodCall.hasRemaining()) {
      return new MethodCall((String) method, arguments);
    }
    throw new IllegalArgumentException("Method call corrupted");
  }

  @Override
  @NonNull
  public ByteBuffer encodeSuccessEnvelope(@NonNull Object result) {
    final ExposedByteArrayOutputStream stream = new ExposedByteArrayOutputStream();
    stream.write(0);
    messageCodec.writeValue(stream, result);
    final ByteBuffer buffer = ByteBuffer.allocateDirect(stream.size());
    buffer.put(stream.buffer(), 0, stream.size());
    return buffer;
  }

  @Override
  @NonNull
  public ByteBuffer encodeErrorEnvelope(
      @NonNull String errorCode, @NonNull String errorMessage, @NonNull Object errorDetails) {
    final ExposedByteArrayOutputStream stream = new ExposedByteArrayOutputStream();
    stream.write(1);
    messageCodec.writeValue(stream, errorCode);
    messageCodec.writeValue(stream, errorMessage);
    if (errorDetails instanceof Throwable) {
      messageCodec.writeValue(stream, Log.getStackTraceString((Throwable) errorDetails));
    } else {
      messageCodec.writeValue(stream, errorDetails);
    }
    final ByteBuffer buffer = ByteBuffer.allocateDirect(stream.size());
    buffer.put(stream.buffer(), 0, stream.size());
    return buffer;
  }

  @Override
  @NonNull
  public ByteBuffer encodeErrorEnvelopeWithStacktrace(
      @NonNull String errorCode,
      @NonNull String errorMessage,
      @NonNull Object errorDetails,
      @NonNull String errorStacktrace) {
    final ExposedByteArrayOutputStream stream = new ExposedByteArrayOutputStream();
    stream.write(1);
    messageCodec.writeValue(stream, errorCode);
    messageCodec.writeValue(stream, errorMessage);
    if (errorDetails instanceof Throwable) {
      messageCodec.writeValue(stream, Log.getStackTraceString((Throwable) errorDetails));
    } else {
      messageCodec.writeValue(stream, errorDetails);
    }
    messageCodec.writeValue(stream, errorStacktrace);
    final ByteBuffer buffer = ByteBuffer.allocateDirect(stream.size());
    buffer.put(stream.buffer(), 0, stream.size());
    return buffer;
  }

  @Override
  @NonNull
  public Object decodeEnvelope(@NonNull ByteBuffer envelope) {
    envelope.order(ByteOrder.nativeOrder());
    final byte flag = envelope.get();
    switch (flag) {
      case 0:
        {
          final Object result = messageCodec.readValue(envelope);
          if (!envelope.hasRemaining()) {
            return result;
          }
        }
        // Falls through intentionally.
      case 1:
        {
          final Object code = messageCodec.readValue(envelope);
          final Object message = messageCodec.readValue(envelope);
          final Object details = messageCodec.readValue(envelope);
          if (code instanceof String
              && (message == null || message instanceof String)
              && !envelope.hasRemaining()) {
            throw new FlutterException((String) code, (String) message, details);
          }
        }
    }
    throw new IllegalArgumentException("Envelope corrupted");
  }
}
