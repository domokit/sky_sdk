// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.common;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import java.nio.ByteBuffer;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

/**
 * A {@link MethodCodec} using UTF-8 encoded JSON method calls and result envelopes.
 *
 * <p>This codec is guaranteed to be compatible with the corresponding <a
 * href="https://api.flutter.dev/flutter/services/JSONMethodCodec-class.html">JSONMethodCodec</a> on
 * the Dart side. These parts of the Flutter SDK are evolved synchronously.
 *
 * <p>Values supported as methods arguments and result payloads are those supported by {@link
 * JSONMessageCodec}.
 */
public final class JSONMethodCodec implements MethodCodec {
  // This codec must match the Dart codec of the same name in package flutter/services.
  public static final JSONMethodCodec INSTANCE = new JSONMethodCodec();

  private JSONMethodCodec() {}

  @Override
  @NonNull
  public ByteBuffer encodeMethodCall(@NonNull MethodCall methodCall) {
    try {
      final JSONObject map = new JSONObject();
      map.put("method", methodCall.method);
      map.put("args", JSONUtil.wrap(methodCall.arguments));
      return JSONMessageCodec.INSTANCE.encodeMessage(map);
    } catch (JSONException e) {
      throw new IllegalArgumentException("Invalid JSON", e);
    }
  }

  @Override
  @NonNull
  public MethodCall decodeMethodCall(@NonNull ByteBuffer message) {
    try {
      final Object json = JSONMessageCodec.INSTANCE.decodeMessage(message);
      if (json instanceof JSONObject) {
        final JSONObject map = (JSONObject) json;
        final Object method = map.get("method");
        final Object arguments = unwrapNull(map.opt("args"));
        if (method instanceof String) {
          return new MethodCall((String) method, arguments);
        }
      }
      throw new IllegalArgumentException("Invalid method call: " + json);
    } catch (JSONException e) {
      throw new IllegalArgumentException("Invalid JSON", e);
    }
  }

  @Override
  @NonNull
  public ByteBuffer encodeSuccessEnvelope(@Nullable Object result) {
    return JSONMessageCodec.INSTANCE.encodeMessage(new JSONArray().put(JSONUtil.wrap(result)));
  }

  @Override
  @NonNull
  public ByteBuffer encodeErrorEnvelope(
      @NonNull String errorCode, @Nullable String errorMessage, @Nullable Object errorDetails) {
    return JSONMessageCodec.INSTANCE.encodeMessage(
        new JSONArray()
            .put(errorCode)
            .put(JSONUtil.wrap(errorMessage))
            .put(JSONUtil.wrap(errorDetails)));
  }

  @Override
  @NonNull
  public ByteBuffer encodeErrorEnvelopeWithStacktrace(
      @NonNull String errorCode,
      @Nullable String errorMessage,
      @Nullable Object errorDetails,
      @Nullable String errorStacktrace) {
    return JSONMessageCodec.INSTANCE.encodeMessage(
        new JSONArray()
            .put(errorCode)
            .put(JSONUtil.wrap(errorMessage))
            .put(JSONUtil.wrap(errorDetails))
            .put(JSONUtil.wrap(errorStacktrace)));
  }

  @Override
  @NonNull
  public Object decodeEnvelope(@NonNull ByteBuffer envelope) {
    try {
      final Object json = JSONMessageCodec.INSTANCE.decodeMessage(envelope);
      if (json instanceof JSONArray) {
        final JSONArray array = (JSONArray) json;
        if (array.length() == 1) {
          return unwrapNull(array.opt(0));
        }
        if (array.length() == 3) {
          final Object code = array.get(0);
          final Object message = unwrapNull(array.opt(1));
          final Object details = unwrapNull(array.opt(2));
          if (code instanceof String && (message == null || message instanceof String)) {
            throw new FlutterException((String) code, (String) message, details);
          }
        }
      }
      throw new IllegalArgumentException("Invalid envelope: " + json);
    } catch (JSONException e) {
      throw new IllegalArgumentException("Invalid JSON", e);
    }
  }

  Object unwrapNull(Object value) {
    return (value == JSONObject.NULL) ? null : value;
  }
}
