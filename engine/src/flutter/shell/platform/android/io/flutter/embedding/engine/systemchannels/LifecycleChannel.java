// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.systemchannels;

import androidx.annotation.NonNull;
import androidx.annotation.VisibleForTesting;
import io.flutter.Log;
import io.flutter.embedding.engine.dart.DartExecutor;
import io.flutter.plugin.common.BasicMessageChannel;
import io.flutter.plugin.common.StringCodec;
import java.util.Locale;

/**
 * A {@link BasicMessageChannel} that communicates lifecycle events to the framework.
 *
 * <p>The activity listens to the Android lifecycle events, in addition to the focus events for
 * windows, and this channel combines that information to decide if the application is the inactive,
 * resumed, paused, or detached state.
 */
public class LifecycleChannel {
  private static final String TAG = "LifecycleChannel";
  private static final String CHANNEL_NAME = "flutter/lifecycle";

  // This enum should match the Dart enum of the same name.
  //
  // HIDDEN isn't used on Android (it's synthesized in the Framework code). It's
  // only listed here so that apicheck_test.dart can make sure that the states here
  // match the Dart code.
  private enum AppLifecycleState {
    DETACHED,
    RESUMED,
    INACTIVE,
    HIDDEN,
    PAUSED,
  };

  private AppLifecycleState lastAndroidState = null;
  private AppLifecycleState lastFlutterState = null;
  private boolean lastFocus = true;

  @NonNull private final BasicMessageChannel<String> channel;

  public LifecycleChannel(@NonNull DartExecutor dartExecutor) {
    this(new BasicMessageChannel<String>(dartExecutor, CHANNEL_NAME, StringCodec.INSTANCE));
  }

  @VisibleForTesting
  public LifecycleChannel(@NonNull BasicMessageChannel<String> channel) {
    this.channel = channel;
  }

  // Here's the state table this implements:
  //
  // | Android State | Window focused | Flutter state |
  // |---------------|----------------|---------------|
  // | Resumed       |     true       |    resumed    |
  // | Resumed       |     false      |    inactive   |
  // | Paused        |     true       |    inactive   |
  // | Paused        |     false      |    inactive   |
  // | Stopped       |     true       |    paused     |
  // | Stopped       |     false      |    paused     |
  // | Detached      |     true       |    detached   |
  // | Detached      |     false      |    detached   |
  //
  // The hidden state isn't used on Android, it's synthesized in the Framework
  // code when transitioning between paused and inactive in either direction.
  private void sendState(AppLifecycleState state, boolean hasFocus) {
    if (lastAndroidState == state && hasFocus == lastFocus) {
      // No inputs changed, so Flutter state could not have changed.
      return;
    }
    if (state == null && lastAndroidState == null) {
      // If we're responding to a focus change before the state is set, just
      // keep the last reported focus state and don't send anything to the
      // framework. This could happen if focus events and lifecycle events are
      // delivered out of the expected order.
      lastFocus = hasFocus;
      return;
    }
    AppLifecycleState newState = null;
    switch (state) {
      case RESUMED:
        // Focus is only taken into account when the Android state is "Resumed".
        // In all other states, focus is ignored, because we can't know what order
        // Android lifecycle notifications and window focus notifications events
        // will arrive in, and those states don't send input events anyhow.
        newState = hasFocus ? AppLifecycleState.RESUMED : AppLifecycleState.INACTIVE;
        break;
      case INACTIVE:
      case HIDDEN:
      case PAUSED:
      case DETACHED:
        newState = state;
        break;
    }

    // Keep the last reported values for future updates.
    lastAndroidState = state;
    lastFocus = hasFocus;
    if (newState == lastFlutterState) {
      // No change in the resulting Flutter state, so don't report anything.
      return;
    }
    String message = "AppLifecycleState." + newState.name().toLowerCase(Locale.ROOT);
    Log.v(TAG, "Sending " + message + " message.");
    channel.send(message);
    lastFlutterState = newState;
  }

  // Called if at least one window in the app has focus, even if the focused
  // window doesn't contain a Flutter view.
  public void aWindowIsFocused() {
    sendState(lastAndroidState, true);
  }

  // Called if no windows in the app have focus.
  public void noWindowsAreFocused() {
    sendState(lastAndroidState, false);
  }

  public void appIsResumed() {
    sendState(AppLifecycleState.RESUMED, lastFocus);
  }

  public void appIsInactive() {
    sendState(AppLifecycleState.INACTIVE, lastFocus);
  }

  public void appIsPaused() {
    sendState(AppLifecycleState.PAUSED, lastFocus);
  }

  public void appIsDetached() {
    sendState(AppLifecycleState.DETACHED, lastFocus);
  }
}
