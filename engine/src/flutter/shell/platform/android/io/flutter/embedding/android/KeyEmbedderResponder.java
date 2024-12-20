// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.android;

import android.view.InputDevice;
import android.view.KeyEvent;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import io.flutter.Log;
import io.flutter.embedding.android.KeyboardMap.PressingGoal;
import io.flutter.embedding.android.KeyboardMap.TogglingGoal;
import io.flutter.plugin.common.BinaryMessenger;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * A {@link KeyboardManager.Responder} of {@link KeyboardManager} that handles events by sending
 * processed information in {@link KeyData}.
 *
 * <p>This class corresponds to the HardwareKeyboard API in the framework.
 */
public class KeyEmbedderResponder implements KeyboardManager.Responder {
  private static final String TAG = "KeyEmbedderResponder";

  // Maps KeyEvent's action and repeatCount to a KeyData type.
  private static KeyData.Type getEventType(KeyEvent event) {
    final boolean isRepeatEvent = event.getRepeatCount() > 0;
    switch (event.getAction()) {
      case KeyEvent.ACTION_DOWN:
        return isRepeatEvent ? KeyData.Type.kRepeat : KeyData.Type.kDown;
      case KeyEvent.ACTION_UP:
        return KeyData.Type.kUp;
      default:
        throw new AssertionError("Unexpected event type");
    }
  }

  // The messenger that is used to send Flutter key events to the framework.
  //
  // On `handleEvent`, Flutter events are marshalled into byte buffers in the format specified by
  // `KeyData.toBytes`.
  @NonNull private final BinaryMessenger messenger;
  // The keys being pressed currently, mapped from physical keys to logical keys.
  @NonNull private final HashMap<Long, Long> pressingRecords = new HashMap<>();
  // Map from logical key to toggling goals.
  //
  // Besides immutable configuration, the toggling goals are also used to store the current enabling
  // states in their `enabled` field.
  @NonNull private final HashMap<Long, TogglingGoal> togglingGoals = new HashMap<>();

  @NonNull
  private final KeyboardManager.CharacterCombiner characterCombiner =
      new KeyboardManager.CharacterCombiner();

  public KeyEmbedderResponder(BinaryMessenger messenger) {
    this.messenger = messenger;
    for (final TogglingGoal goal : KeyboardMap.getTogglingGoals()) {
      togglingGoals.put(goal.logicalKey, goal);
    }
  }

  private static long keyOfPlane(long key, long plane) {
    // Apply '& kValueMask' in case the key is a negative number before being converted to long.
    return plane | (key & KeyboardMap.kValueMask);
  }

  // Get the physical key for this event.
  //
  // The returned value is never null.
  private Long getPhysicalKey(@NonNull KeyEvent event) {
    final long scancode = event.getScanCode();
    // Scancode 0 can occur during emulation using `adb shell input keyevent`. Synthesize a physical
    // key from the key code so that keys can be told apart.
    if (scancode == 0) {
      // The key code can't also be 0, since those events have been filtered.
      return keyOfPlane(event.getKeyCode(), KeyboardMap.kAndroidPlane);
    }
    final Long byMapping = KeyboardMap.scanCodeToPhysical.get(scancode);
    if (byMapping != null) {
      return byMapping;
    }
    return keyOfPlane(event.getScanCode(), KeyboardMap.kAndroidPlane);
  }

  // Get the logical key for this event.
  //
  // The returned value is never null.
  private Long getLogicalKey(@NonNull KeyEvent event) {
    final Long byMapping = KeyboardMap.keyCodeToLogical.get((long) event.getKeyCode());
    if (byMapping != null) {
      return byMapping;
    }
    return keyOfPlane(event.getKeyCode(), KeyboardMap.kAndroidPlane);
  }

  // Update `pressingRecords`.
  //
  // If the key indicated by `physicalKey` is currently not pressed, then `logicalKey` must not be
  // null and this key will be marked pressed.
  //
  // If the key indicated by `physicalKey` is currently pressed, then `logicalKey` must be null
  // and this key will be marked released.
  void updatePressingState(@NonNull Long physicalKey, @Nullable Long logicalKey) {
    if (logicalKey != null) {
      final Long previousValue = pressingRecords.put(physicalKey, logicalKey);
      if (previousValue != null) {
        throw new AssertionError("The key was not empty");
      }
    } else {
      final Long previousValue = pressingRecords.remove(physicalKey);
      if (previousValue == null) {
        throw new AssertionError("The key was empty");
      }
    }
  }

  // Synchronize for a pressing modifier (such as Shift or Ctrl).
  //
  // A pressing modifier is defined by a `PressingGoal`, which consists of a mask to get the true
  // state out of `KeyEvent.getMetaState`, and a list of keys. The synchronization process
  // dispatches synthesized events so that the state of these keys matches the true state taking
  // the current event in consideration.
  //
  // Events that should be synthesized before the main event are synthesized
  // immediately, while events that should be synthesized after the main event are appended to
  // `postSynchronize`.
  //
  // Although Android KeyEvent defined bitmasks for sided modifiers (SHIFT_LEFT_ON and
  // SHIFT_RIGHT_ON),
  // this function only uses the unsided modifiers (SHIFT_ON), due to the weird behaviors observed
  // on ChromeOS, where right modifiers produce events with UNSIDED | LEFT_SIDE meta state bits.
  void synchronizePressingKey(
      PressingGoal goal,
      boolean truePressed,
      long eventLogicalKey,
      long eventPhysicalKey,
      KeyEvent event,
      ArrayList<Runnable> postSynchronize) {
    // During an incoming event, there might be a synthesized Flutter event for each key of each
    // pressing goal, followed by an eventual main Flutter event.
    //
    //    NowState ---------------->  PreEventState --------------> -------------->TrueState
    //              PreSynchronize                       Event      PostSynchronize
    //
    // The goal of the synchronization algorithm is to derive a pre-event state that can satisfy the
    // true state (`truePressed`) after the event, and that requires as few synthesized events based
    // on the current state (`nowStates`) as possible.
    final boolean[] nowStates = new boolean[goal.keys.length];
    final Boolean[] preEventStates = new Boolean[goal.keys.length];
    boolean postEventAnyPressed = false;
    // 1. Find the current states of all keys.
    // 2. Derive the pre-event state of the event key (if applicable.)
    for (int keyIdx = 0; keyIdx < goal.keys.length; keyIdx += 1) {
      final KeyboardMap.KeyPair key = goal.keys[keyIdx];
      nowStates[keyIdx] = pressingRecords.containsKey(key.physicalKey);
      if (key.logicalKey == eventLogicalKey) {
        switch (getEventType(event)) {
          case kDown:
            preEventStates[keyIdx] = false;
            postEventAnyPressed = true;
            if (!truePressed) {
              postSynchronize.add(
                  () ->
                      synthesizeEvent(
                          false, key.logicalKey, eventPhysicalKey, event.getEventTime()));
            }
            break;
          case kUp:
            // Incoming event is an up. Although the previous state should be pressed, don't
            // synthesize a down event even if it's not. The later code will handle such cases by
            // skipping abrupt up events. Obviously don't synthesize up events either.
            preEventStates[keyIdx] = nowStates[keyIdx];
            break;
          case kRepeat:
            // Incoming event is repeat. The previous state can be either pressed or released. Don't
            // synthesize a down event here, or there will be a down event *and* a repeat event,
            // both of which have printable characters. Obviously don't synthesize up events either.
            if (!truePressed) {
              postSynchronize.add(
                  () ->
                      synthesizeEvent(
                          false, key.logicalKey, key.physicalKey, event.getEventTime()));
            }
            preEventStates[keyIdx] = nowStates[keyIdx];
            postEventAnyPressed = true;
            break;
        }
      } else {
        postEventAnyPressed = postEventAnyPressed || nowStates[keyIdx];
      }
    }

    // Fill the rest of the pre-event states to match the true state.
    if (truePressed) {
      // It is required that at least one key is pressed.
      for (int keyIdx = 0; keyIdx < goal.keys.length; keyIdx += 1) {
        if (preEventStates[keyIdx] != null) {
          continue;
        }
        if (postEventAnyPressed) {
          preEventStates[keyIdx] = nowStates[keyIdx];
        } else {
          preEventStates[keyIdx] = true;
          postEventAnyPressed = true;
        }
      }
      if (!postEventAnyPressed) {
        preEventStates[0] = true;
      }
    } else {
      for (int keyIdx = 0; keyIdx < goal.keys.length; keyIdx += 1) {
        if (preEventStates[keyIdx] != null) {
          continue;
        }
        preEventStates[keyIdx] = false;
      }
    }

    // Dispatch synthesized events for state differences.
    for (int keyIdx = 0; keyIdx < goal.keys.length; keyIdx += 1) {
      if (nowStates[keyIdx] != preEventStates[keyIdx]) {
        final KeyboardMap.KeyPair key = goal.keys[keyIdx];
        synthesizeEvent(
            preEventStates[keyIdx], key.logicalKey, key.physicalKey, event.getEventTime());
      }
    }
  }

  // Synchronize for a toggling modifier (such as CapsLock).
  //
  // A toggling modifier is defined by a `TogglingGoal`, which consists of a mask to get the true
  // state out of `KeyEvent.getMetaState`, and a key. The synchronization process dispatches
  // synthesized events so that the state of these keys matches the true state taking the current
  // event in consideration.
  //
  // Although Android KeyEvent defined bitmasks for all "lock" modifiers and define them as the
  // "lock" state, weird behaviors are observed on ChromeOS. First, ScrollLock and NumLock presses
  // do not set metaState bits. Second, CapsLock key events set the CapsLock bit as if it is a
  // pressing modifier (key down having state 1, key up having state 0), while other key events set
  // the CapsLock bit correctly (locked having state 1, unlocked having state 0). Therefore this
  // function only synchronizes the CapsLock state, and only does so during non-CapsLock key events.
  void synchronizeTogglingKey(
      TogglingGoal goal, boolean trueEnabled, long eventLogicalKey, KeyEvent event) {
    if (goal.logicalKey == eventLogicalKey) {
      // Don't synthesize for self events, because the self events have weird metaStates on
      // ChromeOS.
      return;
    }
    if (goal.enabled != trueEnabled) {
      final boolean firstIsDown = !pressingRecords.containsKey(goal.physicalKey);
      if (firstIsDown) {
        goal.enabled = !goal.enabled;
      }
      synthesizeEvent(firstIsDown, goal.logicalKey, goal.physicalKey, event.getEventTime());
      if (!firstIsDown) {
        goal.enabled = !goal.enabled;
      }
      synthesizeEvent(!firstIsDown, goal.logicalKey, goal.physicalKey, event.getEventTime());
    }
  }

  // Implements the core algorithm of `handleEvent`.
  //
  // Returns whether any events are dispatched.
  private boolean handleEventImpl(
      @NonNull KeyEvent event, @NonNull OnKeyEventHandledCallback onKeyEventHandledCallback) {
    // Events with no codes at all can not be recognized.
    if (event.getScanCode() == 0 && event.getKeyCode() == 0) {
      return false;
    }
    final Long physicalKey = getPhysicalKey(event);
    final Long logicalKey = getLogicalKey(event);

    final ArrayList<Runnable> postSynchronizeEvents = new ArrayList<>();
    for (final PressingGoal goal : KeyboardMap.pressingGoals) {
      synchronizePressingKey(
          goal,
          (event.getMetaState() & goal.mask) != 0,
          logicalKey,
          physicalKey,
          event,
          postSynchronizeEvents);
    }

    for (final TogglingGoal goal : togglingGoals.values()) {
      synchronizeTogglingKey(goal, (event.getMetaState() & goal.mask) != 0, logicalKey, event);
    }

    boolean isDownEvent;
    switch (event.getAction()) {
      case KeyEvent.ACTION_DOWN:
        isDownEvent = true;
        break;
      case KeyEvent.ACTION_UP:
        isDownEvent = false;
        break;
      default:
        return false;
    }

    KeyData.Type type;
    String character = null;
    final Long lastLogicalRecord = pressingRecords.get(physicalKey);
    if (isDownEvent) {
      if (lastLogicalRecord == null) {
        type = KeyData.Type.kDown;
      } else {
        // A key has been pressed that has the exact physical key as a currently
        // pressed one.
        if (event.getRepeatCount() > 0) {
          type = KeyData.Type.kRepeat;
        } else {
          synthesizeEvent(false, lastLogicalRecord, physicalKey, event.getEventTime());
          type = KeyData.Type.kDown;
        }
      }
      final char complexChar =
          characterCombiner.applyCombiningCharacterToBaseCharacter(event.getUnicodeChar());
      if (complexChar != 0) {
        character = "" + complexChar;
      }
    } else { // isDownEvent is false
      if (lastLogicalRecord == null) {
        // Ignore abrupt up events.
        return false;
      } else {
        type = KeyData.Type.kUp;
      }
    }

    if (type != KeyData.Type.kRepeat) {
      updatePressingState(physicalKey, isDownEvent ? logicalKey : null);
    }
    if (type == KeyData.Type.kDown) {
      final TogglingGoal maybeTogglingGoal = togglingGoals.get(logicalKey);
      if (maybeTogglingGoal != null) {
        maybeTogglingGoal.enabled = !maybeTogglingGoal.enabled;
      }
    }

    final KeyData output = new KeyData();

    switch (event.getSource()) {
      default:
      case InputDevice.SOURCE_KEYBOARD:
        output.deviceType = KeyData.DeviceType.kKeyboard;
        break;
      case InputDevice.SOURCE_DPAD:
        output.deviceType = KeyData.DeviceType.kDirectionalPad;
        break;
      case InputDevice.SOURCE_GAMEPAD:
        output.deviceType = KeyData.DeviceType.kGamepad;
        break;
      case InputDevice.SOURCE_JOYSTICK:
        output.deviceType = KeyData.DeviceType.kJoystick;
        break;
      case InputDevice.SOURCE_HDMI:
        output.deviceType = KeyData.DeviceType.kHdmi;
        break;
    }

    output.timestamp = event.getEventTime();
    output.type = type;
    output.logicalKey = logicalKey;
    output.physicalKey = physicalKey;
    output.character = character;
    output.synthesized = false;

    sendKeyEvent(output, onKeyEventHandledCallback);
    for (final Runnable postSyncEvent : postSynchronizeEvents) {
      postSyncEvent.run();
    }
    return true;
  }

  private void synthesizeEvent(boolean isDown, Long logicalKey, Long physicalKey, long timestamp) {
    final KeyData output = new KeyData();
    output.timestamp = timestamp;
    output.type = isDown ? KeyData.Type.kDown : KeyData.Type.kUp;
    output.logicalKey = logicalKey;
    output.physicalKey = physicalKey;
    output.character = null;
    output.synthesized = true;
    output.deviceType = KeyData.DeviceType.kKeyboard;
    if (physicalKey != 0 && logicalKey != 0) {
      updatePressingState(physicalKey, isDown ? logicalKey : null);
    }
    sendKeyEvent(output, null);
  }

  private void sendKeyEvent(KeyData data, OnKeyEventHandledCallback onKeyEventHandledCallback) {
    final BinaryMessenger.BinaryReply handleMessageReply =
        onKeyEventHandledCallback == null
            ? null
            : message -> {
              Boolean handled = false;
              if (message != null) {
                message.rewind();
                if (message.capacity() != 0) {
                  handled = message.get() != 0;
                }
              } else {
                Log.w(TAG, "A null reply was received when sending a key event to the framework.");
              }
              onKeyEventHandledCallback.onKeyEventHandled(handled);
            };

    messenger.send(KeyData.CHANNEL, data.toBytes(), handleMessageReply);
  }

  /**
   * Parses an Android key event, performs synchronization, and dispatches Flutter events through
   * the messenger to the framework with the given callback.
   *
   * <p>At least one event will be dispatched. If there are no others, an empty event with 0
   * physical key and 0 logical key will be synthesized.
   *
   * @param event The Android key event to be handled.
   * @param onKeyEventHandledCallback the method to call when the framework has decided whether to
   *     handle this event. This callback will always be called once and only once. If there are no
   *     non-synthesized out of this event, this callback will be called during this method with
   *     true.
   */
  @Override
  public void handleEvent(
      @NonNull KeyEvent event, @NonNull OnKeyEventHandledCallback onKeyEventHandledCallback) {
    final boolean sentAny = handleEventImpl(event, onKeyEventHandledCallback);
    if (!sentAny) {
      synthesizeEvent(true, 0L, 0L, 0L);
      onKeyEventHandledCallback.onKeyEventHandled(true);
    }
  }

  /**
   * Returns an unmodifiable view of the pressed state.
   *
   * @return A map whose keys are physical keyboard key IDs and values are the corresponding logical
   *     keyboard key IDs.
   */
  public Map<Long, Long> getPressedState() {
    return Collections.unmodifiableMap(pressingRecords);
  }
}
