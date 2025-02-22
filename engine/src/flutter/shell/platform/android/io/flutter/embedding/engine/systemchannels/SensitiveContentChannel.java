// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.systemchannels;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import io.flutter.Log;
import io.flutter.embedding.engine.dart.DartExecutor;
import io.flutter.plugin.common.MethodCall;
import io.flutter.plugin.common.MethodChannel;
import io.flutter.plugin.common.StandardMethodCodec;

/**
 * {@link SensitiveContentChannel} is a platform channel that is used by the framework to get and
 * set the content sensitivity of a native Flutter Android {@code View}.
 */
public class SensitiveContentChannel {
  private static final String TAG = "SensitiveContentChannel";

  public final MethodChannel channel;
  private SensitiveContentMethodHandler sensitiveContentMethodHandler;

  @NonNull
  public final MethodChannel.MethodCallHandler parsingMethodHandler =
      new MethodChannel.MethodCallHandler() {
        @Override
        public void onMethodCall(@NonNull MethodCall call, @NonNull MethodChannel.Result result) {
          if (sensitiveContentMethodHandler == null) {
            // No SensitiveContentChannel registered, call not forwarded to sensitive content
            // API.");
            return;
          }
          String method = call.method;
          Log.v(TAG, "Received '" + method + "' message.");
          switch (method) {
            case "SensitiveContent.setContentSensitivity":
              final int contentSensitivityLevel = call.arguments();
              try {
                sensitiveContentMethodHandler.setContentSensitivity(
                    contentSensitivityLevel, result);
              } catch (IllegalStateException exception) {
                result.error("error", exception.getMessage(), null);
              }
              break;
            case "SensitiveContent.getContentSensitivity":
              try {
                sensitiveContentMethodHandler.getContentSensitivity(result);
              } catch (IllegalStateException exception) {
                result.error("error", exception.getMessage(), null);
              }
              break;
            case "SensitiveContent.isSupported":
              try {
                sensitiveContentMethodHandler.isSupported(result);
              } catch (IllegalStateException exception) {
                result.error("error", exception.getMessage(), null);
              }
              break;
            default:
              Log.v(
                  TAG, "Method " + method + " is not implemented for the SensitiveContentChannel.");
              result.notImplemented();
              break;
          }
        }
      };

  public SensitiveContentChannel(@NonNull DartExecutor dartExecutor) {
    channel =
        new MethodChannel(dartExecutor, "flutter/sensitivecontent", StandardMethodCodec.INSTANCE);
    channel.setMethodCallHandler(parsingMethodHandler);
  }

  /**
   * Sets the {@link SensitiveContentMethodHandler} which receives all requests to get and set a
   * particular content sensitivty level sent through this channel.
   */
  public void setSensitiveContentMethodHandler(
      @Nullable SensitiveContentMethodHandler sensitiveContentMethodHandler) {
    this.sensitiveContentMethodHandler = sensitiveContentMethodHandler;
  }

  public interface SensitiveContentMethodHandler {
    /**
     * Requests that a native Flutter Android {@code View} sets its content sensitivity level to
     * {@code requestedContentSensitivity}.
     */
    void setContentSensitivity(
        @NonNull int requestedContentSensitivity, @NonNull MethodChannel.Result result);

    /** Returns the current content sensitivity level of a Flutter Android {@code View}. */
    void getContentSensitivity(@NonNull MethodChannel.Result result);

    /** Returns whether or not marking content sensitivity is supported on the device. */
    void isSupported(@NonNull MethodChannel.Result result);
  }
}
