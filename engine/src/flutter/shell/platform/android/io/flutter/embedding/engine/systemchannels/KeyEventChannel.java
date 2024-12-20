// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.systemchannels;

import android.view.KeyEvent;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import io.flutter.Log;
import io.flutter.plugin.common.BasicMessageChannel;
import io.flutter.plugin.common.BinaryMessenger;
import io.flutter.plugin.common.JSONMessageCodec;
import java.util.HashMap;
import java.util.Map;
import org.json.JSONException;
import org.json.JSONObject;

/**
 * Event message channel for key events to/from the Flutter framework.
 *
 * <p>Sends key up/down events to the framework, and receives asynchronous messages from the
 * framework about whether or not the key was handled.
 */
public class KeyEventChannel {
  private static final String TAG = "KeyEventChannel";

  /** A handler of incoming key handling messages. */
  public interface EventResponseHandler {

    /**
     * Called whenever the framework responds that a given key event was handled or not handled by
     * the framework.
     *
     * @param isEventHandled whether the framework decides to handle the event.
     */
    public void onFrameworkResponse(boolean isEventHandled);
  }

  /**
   * A constructor that creates a KeyEventChannel with the default message handler.
   *
   * @param binaryMessenger the binary messenger used to send messages on this channel.
   */
  public KeyEventChannel(@NonNull BinaryMessenger binaryMessenger) {
    this.channel =
        new BasicMessageChannel<>(binaryMessenger, "flutter/keyevent", JSONMessageCodec.INSTANCE);
  }

  @NonNull public final BasicMessageChannel<Object> channel;

  public void sendFlutterKeyEvent(
      @NonNull FlutterKeyEvent keyEvent,
      boolean isKeyUp,
      @NonNull EventResponseHandler responseHandler) {
    channel.send(encodeKeyEvent(keyEvent, isKeyUp), createReplyHandler(responseHandler));
  }

  private Map<String, Object> encodeKeyEvent(@NonNull FlutterKeyEvent keyEvent, boolean isKeyUp) {
    Map<String, Object> message = new HashMap<>();
    message.put("type", isKeyUp ? "keyup" : "keydown");
    message.put("keymap", "android");
    message.put("flags", keyEvent.event.getFlags());
    message.put("plainCodePoint", keyEvent.event.getUnicodeChar(0x0));
    message.put("codePoint", keyEvent.event.getUnicodeChar());
    message.put("keyCode", keyEvent.event.getKeyCode());
    message.put("scanCode", keyEvent.event.getScanCode());
    message.put("metaState", keyEvent.event.getMetaState());
    if (keyEvent.complexCharacter != null) {
      message.put("character", keyEvent.complexCharacter.toString());
    }
    message.put("source", keyEvent.event.getSource());
    message.put("deviceId", keyEvent.event.getDeviceId());
    message.put("repeatCount", keyEvent.event.getRepeatCount());
    return message;
  }

  /**
   * Creates a reply handler for the given key event.
   *
   * @param responseHandler the completion handler to call when the framework responds.
   */
  private static BasicMessageChannel.Reply<Object> createReplyHandler(
      @NonNull EventResponseHandler responseHandler) {
    return message -> {
      boolean isEventHandled = false;
      try {
        if (message != null) {
          final JSONObject annotatedEvent = (JSONObject) message;
          isEventHandled = annotatedEvent.getBoolean("handled");
        }
      } catch (JSONException e) {
        Log.e(TAG, "Unable to unpack JSON message: " + e);
      }
      responseHandler.onFrameworkResponse(isEventHandled);
    };
  }

  /** A key event as defined by Flutter. */
  public static class FlutterKeyEvent {
    /**
     * The Android key event that this Flutter key event was created from.
     *
     * <p>This event is used to identify pending events when results are received from the
     * framework.
     */
    public final KeyEvent event;
    /**
     * The character produced by this event, including any combining characters pressed before it.
     */
    @Nullable public final Character complexCharacter;

    public FlutterKeyEvent(@NonNull KeyEvent androidKeyEvent) {
      this(androidKeyEvent, null);
    }

    public FlutterKeyEvent(
        @NonNull KeyEvent androidKeyEvent, @Nullable Character complexCharacter) {
      this.event = androidKeyEvent;
      this.complexCharacter = complexCharacter;
    }
  }
}
