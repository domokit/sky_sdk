package io.flutter.util;

import android.view.KeyEvent;

// In the test environment, keyEvent.getUnicodeChar throws an exception. This
// class works around the exception by hardcoding the returned value.
public class FakeKeyEvent extends KeyEvent {
  public FakeKeyEvent(int action, int keyCode) {
    super(action, keyCode);
  }

  public FakeKeyEvent(int action, int keyCode, char character) {
    super(action, keyCode);
    this.character = character;
  }

  public FakeKeyEvent(
      int action, int scancode, int code, int repeat, char character, int metaState) {
    super(0, 0, action, code, repeat, metaState, 0, scancode);
    this.character = character;
  }

  public FakeKeyEvent(
      int action, int scancode, int code, int repeat, char character, int metaState, int source) {
    super(0, 0, action, code, repeat, metaState, 0, scancode, 0, source);
    this.character = character;
  }

  private char character = 0;

  public final int getUnicodeChar() {
    if (getKeyCode() == KeyEvent.KEYCODE_BACK) {
      return 0;
    }
    return character;
  }
}
