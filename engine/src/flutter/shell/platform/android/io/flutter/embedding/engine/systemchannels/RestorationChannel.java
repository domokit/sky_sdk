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
import java.util.HashMap;
import java.util.Map;

/**
 * System channel to exchange restoration data between framework and engine.
 *
 * <p>The engine can obtain the current restoration data from the framework via this channel to
 * store it on disk and - when the app is relaunched - provide the stored data back to the framework
 * to recreate the original state of the app.
 *
 * <p>The channel can be configured to delay responding to the framework's request for restoration
 * data via {@code waitForRestorationData} until the engine-side has provided the data. This is
 * useful when the engine is pre-warmed at a point in the application's life cycle where the
 * restoration data is not available yet. For example, if the engine is pre-warmed as part of the
 * Application before an Activity is created, this flag should be set to true because Android will
 * only provide the restoration data to the Activity during the onCreate callback.
 *
 * <p>The current restoration data provided by the framework can be read via {@code
 * getRestorationData()}.
 */
public class RestorationChannel {
  private static final String TAG = "RestorationChannel";

  public RestorationChannel(
      @NonNull DartExecutor dartExecutor, @NonNull boolean waitForRestorationData) {
    this(
        new MethodChannel(dartExecutor, "flutter/restoration", StandardMethodCodec.INSTANCE),
        waitForRestorationData);
  }

  RestorationChannel(MethodChannel channel, @NonNull boolean waitForRestorationData) {
    this.channel = channel;
    this.waitForRestorationData = waitForRestorationData;

    channel.setMethodCallHandler(handler);
  }

  /**
   * Whether the channel delays responding to the framework's initial request for restoration data
   * until {@code setRestorationData} has been called.
   *
   * <p>If the engine never calls {@code setRestorationData} this flag must be set to false. If set
   * to true, the engine must call {@code setRestorationData} either with the actual restoration
   * data as argument or null if it turns out that there is no restoration data.
   *
   * <p>If the response to the framework's request for restoration data is not delayed until the
   * data has been set via {@code setRestorationData}, the framework may intermittently initialize
   * itself to default values until the restoration data has been made available. Setting this flag
   * to true avoids that extra work.
   */
  public final boolean waitForRestorationData;

  // Holds the most current restoration data which may have been provided by the engine
  // via "setRestorationData" or by the framework via the method channel. This is the data the
  // framework should be restored to in case the app is terminated.
  private byte[] restorationData;
  private MethodChannel channel;
  private MethodChannel.Result pendingFrameworkRestorationChannelRequest;
  private boolean engineHasProvidedData = false;
  private boolean frameworkHasRequestedData = false;

  /** Obtain the most current restoration data that the framework has provided. */
  @Nullable
  public byte[] getRestorationData() {
    return restorationData;
  }

  /** Set the restoration data from which the framework will restore its state. */
  public void setRestorationData(@NonNull byte[] data) {
    engineHasProvidedData = true;
    if (pendingFrameworkRestorationChannelRequest != null) {
      // If their is a pending request from the framework, answer it.
      pendingFrameworkRestorationChannelRequest.success(packageData(data));
      pendingFrameworkRestorationChannelRequest = null;
      restorationData = data;
    } else if (frameworkHasRequestedData) {
      // If the framework has previously received the engine's restoration data, push the new data
      // directly to it. This case can happen when "waitForRestorationData" is false and the
      // framework retrieved the restoration state before it was set via this method.
      // Experimentally, this can also be used to restore a previously used engine to another state,
      // e.g. when the engine is attached to a new activity.
      channel.invokeMethod(
          "push",
          packageData(data),
          new MethodChannel.Result() {
            @Override
            public void success(Object result) {
              restorationData = data;
            }

            @Override
            public void error(String errorCode, String errorMessage, Object errorDetails) {
              Log.e(
                  TAG,
                  "Error "
                      + errorCode
                      + " while sending restoration data to framework: "
                      + errorMessage);
            }

            @Override
            public void notImplemented() {
              // Nothing to do.
            }
          });
    } else {
      // Otherwise, just cache the data until the framework asks for it.
      restorationData = data;
    }
  }

  /**
   * Clears the current restoration data.
   *
   * <p>This should be called just prior to a hot restart. Otherwise, after the hot restart the
   * state prior to the hot restart will get restored.
   */
  public void clearData() {
    restorationData = null;
  }

  private final MethodChannel.MethodCallHandler handler =
      new MethodChannel.MethodCallHandler() {
        @Override
        public void onMethodCall(@NonNull MethodCall call, @NonNull MethodChannel.Result result) {
          final String method = call.method;
          final Object args = call.arguments;
          switch (method) {
            case "put":
              restorationData = (byte[]) args;
              result.success(null);
              break;
            case "get":
              frameworkHasRequestedData = true;
              if (engineHasProvidedData || !waitForRestorationData) {
                result.success(packageData(restorationData));
                // Do not delete the restoration data on the engine side after sending it to the
                // framework. We may need to hand this data back to the operating system if the
                // framework never modifies the data (and thus doesn't send us any
                // data back).
              } else {
                pendingFrameworkRestorationChannelRequest = result;
              }
              break;
            default:
              result.notImplemented();
              break;
          }
        }
      };

  private Map<String, Object> packageData(byte[] data) {
    final Map<String, Object> packaged = new HashMap<String, Object>();
    packaged.put("enabled", true); // Android supports state restoration.
    packaged.put("data", data);
    return packaged;
  }
}
