// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.android;

import android.view.KeyEvent;
import androidx.annotation.NonNull;
import io.flutter.embedding.engine.systemchannels.KeyEventChannel;

/**
 * A {@link KeyboardManager.Responder} of {@link KeyboardManager} that handles events by sending the
 * raw information through the method channel.
 *
 * <p>This class corresponds to the RawKeyboard API in the framework.
 */
public class KeyChannelResponder implements KeyboardManager.Responder {
  private static final String TAG = "KeyChannelResponder";

  @NonNull private final KeyEventChannel keyEventChannel;

  @NonNull
  private final KeyboardManager.CharacterCombiner characterCombiner =
      new KeyboardManager.CharacterCombiner();

  public KeyChannelResponder(@NonNull KeyEventChannel keyEventChannel) {
    this.keyEventChannel = keyEventChannel;
  }

  @Override
  public void handleEvent(
      @NonNull KeyEvent keyEvent, @NonNull OnKeyEventHandledCallback onKeyEventHandledCallback) {
    final int action = keyEvent.getAction();
    if (action != KeyEvent.ACTION_DOWN && action != KeyEvent.ACTION_UP) {
      // There is theoretically a KeyEvent.ACTION_MULTIPLE, but theoretically
      // that isn't sent by Android anymore, so this is just for protection in
      // case the theory is wrong.
      onKeyEventHandledCallback.onKeyEventHandled(false);
      return;
    }

    final Character complexCharacter =
        characterCombiner.applyCombiningCharacterToBaseCharacter(keyEvent.getUnicodeChar());
    KeyEventChannel.FlutterKeyEvent flutterEvent =
        new KeyEventChannel.FlutterKeyEvent(keyEvent, complexCharacter);

    final boolean isKeyUp = action != KeyEvent.ACTION_DOWN;
    keyEventChannel.sendFlutterKeyEvent(
        flutterEvent,
        isKeyUp,
        (isEventHandled) -> onKeyEventHandledCallback.onKeyEventHandled(isEventHandled));
  }
}
