// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.systemchannels;

import android.annotation.TargetApi;
import android.window.BackEvent;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.RequiresApi;
import io.flutter.Build.API_LEVELS;
import io.flutter.Log;
import io.flutter.embedding.engine.dart.DartExecutor;
import io.flutter.plugin.common.MethodCall;
import io.flutter.plugin.common.MethodChannel;
import io.flutter.plugin.common.StandardMethodCodec;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

/**
 * A {@link MethodChannel} for communicating back gesture events to the Flutter framework.
 *
 * <p>The BackGestureChannel facilitates communication between the platform-specific Android back
 * gesture handling code and the Flutter framework. It enables the dispatch of back gesture events
 * such as start, progress, commit, and cancellation from the platform to the Flutter application.
 */
public class BackGestureChannel {
  private static final String TAG = "BackGestureChannel";

  @NonNull public final MethodChannel channel;

  /**
   * Constructs a BackGestureChannel.
   *
   * @param dartExecutor The DartExecutor used to establish communication with the Flutter
   *     framework.
   */
  public BackGestureChannel(@NonNull DartExecutor dartExecutor) {
    this.channel =
        new MethodChannel(dartExecutor, "flutter/backgesture", StandardMethodCodec.INSTANCE);
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

  /**
   * Initiates a back gesture event.
   *
   * <p>This method should be called when the back gesture is initiated by the user.
   *
   * @param backEvent The BackEvent object containing information about the touch.
   */
  @TargetApi(API_LEVELS.API_34)
  @RequiresApi(API_LEVELS.API_34)
  public void startBackGesture(@NonNull BackEvent backEvent) {
    Log.v(TAG, "Sending message to start back gesture");
    channel.invokeMethod("startBackGesture", backEventToJsonMap(backEvent));
  }

  /**
   * Updates the progress of a back gesture event.
   *
   * <p>This method should be called to update the progress of an ongoing back gesture event.
   *
   * @param backEvent An BackEvent object describing the progress event.
   */
  @TargetApi(API_LEVELS.API_34)
  @RequiresApi(API_LEVELS.API_34)
  public void updateBackGestureProgress(@NonNull BackEvent backEvent) {
    Log.v(TAG, "Sending message to update back gesture progress");
    channel.invokeMethod("updateBackGestureProgress", backEventToJsonMap(backEvent));
  }

  /**
   * Commits the back gesture event.
   *
   * <p>This method should be called to signify the completion of a back gesture event and commit
   * the navigation action initiated by the gesture.
   */
  @TargetApi(API_LEVELS.API_34)
  @RequiresApi(API_LEVELS.API_34)
  public void commitBackGesture() {
    Log.v(TAG, "Sending message to commit back gesture");
    channel.invokeMethod("commitBackGesture", null);
  }

  /**
   * Cancels the back gesture event.
   *
   * <p>This method should be called when a back gesture is cancelled or the back button is pressed.
   */
  @TargetApi(API_LEVELS.API_34)
  @RequiresApi(API_LEVELS.API_34)
  public void cancelBackGesture() {
    Log.v(TAG, "Sending message to cancel back gesture");
    channel.invokeMethod("cancelBackGesture", null);
  }

  /**
   * Sets a method call handler for the channel.
   *
   * @param handler The handler to set for the channel.
   */
  public void setMethodCallHandler(@Nullable MethodChannel.MethodCallHandler handler) {
    channel.setMethodCallHandler(handler);
  }

  @TargetApi(API_LEVELS.API_34)
  @RequiresApi(API_LEVELS.API_34)
  private Map<String, Object> backEventToJsonMap(@NonNull BackEvent backEvent) {
    Map<String, Object> message = new HashMap<>(3);
    final float x = backEvent.getTouchX();
    final float y = backEvent.getTouchY();
    final Object touchOffset = (Float.isNaN(x) || Float.isNaN(y)) ? null : Arrays.asList(x, y);
    message.put("touchOffset", touchOffset);
    message.put("progress", backEvent.getProgress());
    message.put("swipeEdge", backEvent.getSwipeEdge());

    return message;
  }
}
