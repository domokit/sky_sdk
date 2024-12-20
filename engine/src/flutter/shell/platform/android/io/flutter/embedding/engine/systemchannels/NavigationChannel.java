// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.systemchannels;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import io.flutter.Log;
import io.flutter.embedding.engine.dart.DartExecutor;
import io.flutter.plugin.common.JSONMethodCodec;
import io.flutter.plugin.common.MethodCall;
import io.flutter.plugin.common.MethodChannel;
import java.util.HashMap;
import java.util.Map;

/** TODO(mattcarroll): fill in javadoc for NavigationChannel. */
public class NavigationChannel {
  private static final String TAG = "NavigationChannel";

  @NonNull public final MethodChannel channel;

  public NavigationChannel(@NonNull DartExecutor dartExecutor) {
    this.channel = new MethodChannel(dartExecutor, "flutter/navigation", JSONMethodCodec.INSTANCE);
    channel.setMethodCallHandler(defaultHandler);
  }

  // Provide a default handler that returns an empty response to any messages
  // on this channel.
  private final MethodChannel.MethodCallHandler defaultHandler =
      new MethodChannel.MethodCallHandler() {
        @Override
        public void onMethodCall(@NonNull MethodCall call, @NonNull MethodChannel.Result result) {
          result.success(null);
        }
      };

  public void setInitialRoute(@NonNull String initialRoute) {
    Log.v(TAG, "Sending message to set initial route to '" + initialRoute + "'");
    channel.invokeMethod("setInitialRoute", initialRoute);
  }

  public void pushRoute(@NonNull String route) {
    Log.v(TAG, "Sending message to push route '" + route + "'");
    channel.invokeMethod("pushRoute", route);
  }

  public void pushRouteInformation(@NonNull String route) {
    Log.v(TAG, "Sending message to push route information '" + route + "'");
    Map<String, String> message = new HashMap<>();
    message.put("location", route);
    channel.invokeMethod("pushRouteInformation", message);
  }

  public void popRoute() {
    Log.v(TAG, "Sending message to pop route.");
    channel.invokeMethod("popRoute", null);
  }

  public void setMethodCallHandler(@Nullable MethodChannel.MethodCallHandler handler) {
    channel.setMethodCallHandler(handler);
  }
}
