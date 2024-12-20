// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.editing;

import static io.flutter.Build.API_LEVELS;

import android.annotation.TargetApi;
import android.content.ClipData;
import android.content.ClipboardManager;
import android.content.Context;
import android.net.Uri;
import android.os.Build;
import android.os.Bundle;
import android.text.DynamicLayout;
import android.text.Editable;
import android.text.InputType;
import android.text.Layout;
import android.text.Selection;
import android.text.TextPaint;
import android.view.KeyEvent;
import android.view.View;
import android.view.inputmethod.BaseInputConnection;
import android.view.inputmethod.CursorAnchorInfo;
import android.view.inputmethod.EditorInfo;
import android.view.inputmethod.ExtractedText;
import android.view.inputmethod.ExtractedTextRequest;
import android.view.inputmethod.InputContentInfo;
import android.view.inputmethod.InputMethodManager;
import androidx.annotation.NonNull;
import androidx.annotation.RequiresApi;
import androidx.core.view.inputmethod.InputConnectionCompat;
import io.flutter.Log;
import io.flutter.embedding.engine.FlutterJNI;
import io.flutter.embedding.engine.systemchannels.ScribeChannel;
import io.flutter.embedding.engine.systemchannels.TextInputChannel;
import java.io.ByteArrayOutputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.HashMap;
import java.util.Map;

public class InputConnectionAdaptor extends BaseInputConnection
    implements ListenableEditingState.EditingStateWatcher {
  private static final String TAG = "InputConnectionAdaptor";

  public interface KeyboardDelegate {
    public boolean handleEvent(@NonNull KeyEvent keyEvent);
  }

  private final View mFlutterView;
  private final int mClient;
  private final ScribeChannel scribeChannel;
  private final TextInputChannel textInputChannel;
  private final ListenableEditingState mEditable;
  private final EditorInfo mEditorInfo;
  private ExtractedTextRequest mExtractRequest;
  private boolean mMonitorCursorUpdate = false;
  private CursorAnchorInfo.Builder mCursorAnchorInfoBuilder;
  private ExtractedText mExtractedText = new ExtractedText();
  private InputMethodManager mImm;
  private final Layout mLayout;
  private FlutterTextUtils flutterTextUtils;
  private final KeyboardDelegate keyboardDelegate;
  private int batchEditNestDepth = 0;

  @SuppressWarnings("deprecation")
  public InputConnectionAdaptor(
      View view,
      int client,
      TextInputChannel textInputChannel,
      ScribeChannel scribeChannel,
      KeyboardDelegate keyboardDelegate,
      ListenableEditingState editable,
      EditorInfo editorInfo,
      FlutterJNI flutterJNI) {
    super(view, true);
    mFlutterView = view;
    mClient = client;
    this.textInputChannel = textInputChannel;
    this.scribeChannel = scribeChannel;
    mEditable = editable;
    mEditable.addEditingStateListener(this);
    mEditorInfo = editorInfo;
    this.keyboardDelegate = keyboardDelegate;
    this.flutterTextUtils = new FlutterTextUtils(flutterJNI);
    // We create a dummy Layout with max width so that the selection
    // shifting acts as if all text were in one line.
    mLayout =
        new DynamicLayout(
            mEditable,
            new TextPaint(),
            Integer.MAX_VALUE,
            Layout.Alignment.ALIGN_NORMAL,
            1.0f,
            0.0f,
            false);
    mImm = (InputMethodManager) view.getContext().getSystemService(Context.INPUT_METHOD_SERVICE);
  }

  public InputConnectionAdaptor(
      View view,
      int client,
      TextInputChannel textInputChannel,
      ScribeChannel scribeChannel,
      KeyboardDelegate keyboardDelegate,
      ListenableEditingState editable,
      EditorInfo editorInfo) {
    this(
        view,
        client,
        textInputChannel,
        scribeChannel,
        keyboardDelegate,
        editable,
        editorInfo,
        new FlutterJNI());
  }

  private ExtractedText getExtractedText(ExtractedTextRequest request) {
    mExtractedText.startOffset = 0;
    mExtractedText.partialStartOffset = -1;
    mExtractedText.partialEndOffset = -1;
    mExtractedText.selectionStart = mEditable.getSelectionStart();
    mExtractedText.selectionEnd = mEditable.getSelectionEnd();
    mExtractedText.text =
        request == null || (request.flags & GET_TEXT_WITH_STYLES) == 0
            ? mEditable.toString()
            : mEditable;
    return mExtractedText;
  }

  private CursorAnchorInfo getCursorAnchorInfo() {
    if (mCursorAnchorInfoBuilder == null) {
      mCursorAnchorInfoBuilder = new CursorAnchorInfo.Builder();
    } else {
      mCursorAnchorInfoBuilder.reset();
    }

    mCursorAnchorInfoBuilder.setSelectionRange(
        mEditable.getSelectionStart(), mEditable.getSelectionEnd());
    final int composingStart = mEditable.getComposingStart();
    final int composingEnd = mEditable.getComposingEnd();
    if (composingStart >= 0 && composingEnd > composingStart) {
      mCursorAnchorInfoBuilder.setComposingText(
          composingStart, mEditable.toString().subSequence(composingStart, composingEnd));
    } else {
      mCursorAnchorInfoBuilder.setComposingText(-1, "");
    }
    return mCursorAnchorInfoBuilder.build();
  }

  @Override
  public Editable getEditable() {
    return mEditable;
  }

  @Override
  public boolean beginBatchEdit() {
    mEditable.beginBatchEdit();
    batchEditNestDepth += 1;
    return super.beginBatchEdit();
  }

  @Override
  public boolean endBatchEdit() {
    boolean result = super.endBatchEdit();
    batchEditNestDepth -= 1;
    mEditable.endBatchEdit();
    return result;
  }

  @Override
  public boolean commitText(CharSequence text, int newCursorPosition) {
    final boolean result = super.commitText(text, newCursorPosition);
    return result;
  }

  @Override
  public boolean deleteSurroundingText(int beforeLength, int afterLength) {
    if (mEditable.getSelectionStart() == -1) {
      return true;
    }

    final boolean result = super.deleteSurroundingText(beforeLength, afterLength);
    return result;
  }

  @Override
  public boolean deleteSurroundingTextInCodePoints(int beforeLength, int afterLength) {
    boolean result = super.deleteSurroundingTextInCodePoints(beforeLength, afterLength);
    return result;
  }

  @Override
  public boolean setComposingRegion(int start, int end) {
    final boolean result = super.setComposingRegion(start, end);
    return result;
  }

  @Override
  public boolean setComposingText(CharSequence text, int newCursorPosition) {
    boolean result;
    beginBatchEdit();
    if (text.length() == 0) {
      result = super.commitText(text, newCursorPosition);
    } else {
      result = super.setComposingText(text, newCursorPosition);
    }
    endBatchEdit();
    return result;
  }

  @Override
  public boolean finishComposingText() {
    final boolean result = super.finishComposingText();
    return result;
  }

  // When there's not enough vertical screen space, the IME may enter fullscreen mode and this
  // method will be used to get (a portion of) the currently edited text. Samsung keyboard seems
  // to use this method instead of InputConnection#getText{Before,After}Cursor.
  // See https://github.com/flutter/engine/pull/17426.
  // TODO(garyq): Implement a more feature complete version of getExtractedText
  @Override
  public ExtractedText getExtractedText(ExtractedTextRequest request, int flags) {
    final boolean textMonitor = (flags & GET_EXTRACTED_TEXT_MONITOR) != 0;
    if (textMonitor == (mExtractRequest == null)) {
      Log.d(TAG, "The input method toggled text monitoring " + (textMonitor ? "on" : "off"));
    }
    // Enables text monitoring if the relevant flag is set. See
    // InputConnectionAdaptor#didChangeEditingState.
    mExtractRequest = textMonitor ? request : null;
    return getExtractedText(request);
  }

  @Override
  public boolean requestCursorUpdates(int cursorUpdateMode) {
    if ((cursorUpdateMode & CURSOR_UPDATE_IMMEDIATE) != 0) {
      mImm.updateCursorAnchorInfo(mFlutterView, getCursorAnchorInfo());
    }

    final boolean updated = (cursorUpdateMode & CURSOR_UPDATE_MONITOR) != 0;
    if (updated != mMonitorCursorUpdate) {
      Log.d(TAG, "The input method toggled cursor monitoring " + (updated ? "on" : "off"));
    }

    // Enables cursor monitoring. See InputConnectionAdaptor#didChangeEditingState.
    mMonitorCursorUpdate = updated;
    return true;
  }

  @Override
  public boolean clearMetaKeyStates(int states) {
    boolean result = super.clearMetaKeyStates(states);
    return result;
  }

  @Override
  public void closeConnection() {
    super.closeConnection();
    mEditable.removeEditingStateListener(this);
    for (; batchEditNestDepth > 0; batchEditNestDepth--) {
      endBatchEdit();
    }
  }

  @Override
  public boolean setSelection(int start, int end) {
    beginBatchEdit();
    boolean result = super.setSelection(start, end);
    endBatchEdit();
    return result;
  }

  // TODO(justinmc): Scribe stylus gestures should be supported here via
  // performHandwritingGesture.
  // https://github.com/flutter/flutter/issues/156018

  // Sanitizes the index to ensure the index is within the range of the
  // contents of editable.
  private static int clampIndexToEditable(int index, Editable editable) {
    int clamped = Math.max(0, Math.min(editable.length(), index));
    if (clamped != index) {
      Log.d(
          "flutter",
          "Text selection index was clamped ("
              + index
              + "->"
              + clamped
              + ") to remain in bounds. This may not be your fault, as some keyboards may select outside of bounds.");
    }
    return clamped;
  }

  // This function is called both when hardware key events occur and aren't
  // handled by the framework, as well as when soft keyboard editing events
  // occur, and need a chance to be handled by the framework.
  @Override
  public boolean sendKeyEvent(KeyEvent event) {
    return keyboardDelegate.handleEvent(event);
  }

  public boolean handleKeyEvent(KeyEvent event) {
    if (event.getAction() == KeyEvent.ACTION_DOWN) {
      if (event.getKeyCode() == KeyEvent.KEYCODE_DPAD_LEFT) {
        return handleHorizontalMovement(true, event.isShiftPressed());
      } else if (event.getKeyCode() == KeyEvent.KEYCODE_DPAD_RIGHT) {
        return handleHorizontalMovement(false, event.isShiftPressed());
      } else if (event.getKeyCode() == KeyEvent.KEYCODE_DPAD_UP) {
        return handleVerticalMovement(true, event.isShiftPressed());
      } else if (event.getKeyCode() == KeyEvent.KEYCODE_DPAD_DOWN) {
        return handleVerticalMovement(false, event.isShiftPressed());
        // When the enter key is pressed on a non-multiline field, consider it a
        // submit instead of a newline.
      } else if ((event.getKeyCode() == KeyEvent.KEYCODE_ENTER
              || event.getKeyCode() == KeyEvent.KEYCODE_NUMPAD_ENTER)
          && (InputType.TYPE_TEXT_FLAG_MULTI_LINE & mEditorInfo.inputType) == 0) {
        performEditorAction(mEditorInfo.imeOptions & EditorInfo.IME_MASK_ACTION);
        return true;
      } else {
        // Enter a character.
        final int selStart = Selection.getSelectionStart(mEditable);
        final int selEnd = Selection.getSelectionEnd(mEditable);
        final int character = event.getUnicodeChar();
        if (selStart < 0 || selEnd < 0 || character == 0) {
          return false;
        }

        final int selMin = Math.min(selStart, selEnd);
        final int selMax = Math.max(selStart, selEnd);
        beginBatchEdit();
        if (selMin != selMax) mEditable.delete(selMin, selMax);
        mEditable.insert(selMin, String.valueOf((char) character));
        setSelection(selMin + 1, selMin + 1);
        endBatchEdit();
        return true;
      }
    }
    return false;
  }

  private boolean handleHorizontalMovement(boolean isLeft, boolean isShiftPressed) {
    final int selStart = Selection.getSelectionStart(mEditable);
    final int selEnd = Selection.getSelectionEnd(mEditable);

    if (selStart < 0 || selEnd < 0) {
      return false;
    }

    final int newSelectionEnd =
        isLeft
            ? Math.max(flutterTextUtils.getOffsetBefore(mEditable, selEnd), 0)
            : Math.min(flutterTextUtils.getOffsetAfter(mEditable, selEnd), mEditable.length());

    final boolean shouldCollapse = selStart == selEnd && !isShiftPressed;

    if (shouldCollapse) {
      setSelection(newSelectionEnd, newSelectionEnd);
    } else {
      setSelection(selStart, newSelectionEnd);
    }
    return true;
  };

  private boolean handleVerticalMovement(boolean isUp, boolean isShiftPressed) {
    final int selStart = Selection.getSelectionStart(mEditable);
    final int selEnd = Selection.getSelectionEnd(mEditable);

    if (selStart < 0 || selEnd < 0) {
      return false;
    }

    final boolean shouldCollapse = selStart == selEnd && !isShiftPressed;

    beginBatchEdit();
    if (shouldCollapse) {
      if (isUp) {
        Selection.moveUp(mEditable, mLayout);
      } else {
        Selection.moveDown(mEditable, mLayout);
      }
      final int newSelection = Selection.getSelectionStart(mEditable);
      setSelection(newSelection, newSelection);
    } else {
      if (isUp) {
        Selection.extendUp(mEditable, mLayout);
      } else {
        Selection.extendDown(mEditable, mLayout);
      }
      setSelection(Selection.getSelectionStart(mEditable), Selection.getSelectionEnd(mEditable));
    }
    endBatchEdit();
    return true;
  }

  @Override
  public boolean performContextMenuAction(int id) {
    beginBatchEdit();
    final boolean result = doPerformContextMenuAction(id);
    endBatchEdit();
    return result;
  }

  private boolean doPerformContextMenuAction(int id) {
    if (id == android.R.id.selectAll) {
      setSelection(0, mEditable.length());
      return true;
    } else if (id == android.R.id.cut) {
      int selStart = Selection.getSelectionStart(mEditable);
      int selEnd = Selection.getSelectionEnd(mEditable);
      if (selStart != selEnd) {
        int selMin = Math.min(selStart, selEnd);
        int selMax = Math.max(selStart, selEnd);
        CharSequence textToCut = mEditable.subSequence(selMin, selMax);
        ClipboardManager clipboard =
            (ClipboardManager)
                mFlutterView.getContext().getSystemService(Context.CLIPBOARD_SERVICE);
        ClipData clip = ClipData.newPlainText("text label?", textToCut);
        clipboard.setPrimaryClip(clip);
        mEditable.delete(selMin, selMax);
        setSelection(selMin, selMin);
      }
      return true;
    } else if (id == android.R.id.copy) {
      int selStart = Selection.getSelectionStart(mEditable);
      int selEnd = Selection.getSelectionEnd(mEditable);
      if (selStart != selEnd) {
        CharSequence textToCopy =
            mEditable.subSequence(Math.min(selStart, selEnd), Math.max(selStart, selEnd));
        ClipboardManager clipboard =
            (ClipboardManager)
                mFlutterView.getContext().getSystemService(Context.CLIPBOARD_SERVICE);
        clipboard.setPrimaryClip(ClipData.newPlainText("text label?", textToCopy));
      }
      return true;
    } else if (id == android.R.id.paste) {
      ClipboardManager clipboard =
          (ClipboardManager) mFlutterView.getContext().getSystemService(Context.CLIPBOARD_SERVICE);
      ClipData clip = clipboard.getPrimaryClip();
      if (clip != null) {
        CharSequence textToPaste = clip.getItemAt(0).coerceToText(mFlutterView.getContext());
        int selStart = Math.max(0, Selection.getSelectionStart(mEditable));
        int selEnd = Math.max(0, Selection.getSelectionEnd(mEditable));
        int selMin = Math.min(selStart, selEnd);
        int selMax = Math.max(selStart, selEnd);
        if (selMin != selMax) mEditable.delete(selMin, selMax);
        mEditable.insert(selMin, textToPaste);
        int newSelStart = selMin + textToPaste.length();
        setSelection(newSelStart, newSelStart);
      }
      return true;
    }
    return false;
  }

  @Override
  public boolean performPrivateCommand(String action, Bundle data) {
    textInputChannel.performPrivateCommand(mClient, action, data);
    return true;
  }

  @Override
  public boolean performEditorAction(int actionCode) {
    switch (actionCode) {
      case EditorInfo.IME_ACTION_NONE:
        textInputChannel.newline(mClient);
        break;
      case EditorInfo.IME_ACTION_UNSPECIFIED:
        textInputChannel.unspecifiedAction(mClient);
        break;
      case EditorInfo.IME_ACTION_GO:
        textInputChannel.go(mClient);
        break;
      case EditorInfo.IME_ACTION_SEARCH:
        textInputChannel.search(mClient);
        break;
      case EditorInfo.IME_ACTION_SEND:
        textInputChannel.send(mClient);
        break;
      case EditorInfo.IME_ACTION_NEXT:
        textInputChannel.next(mClient);
        break;
      case EditorInfo.IME_ACTION_PREVIOUS:
        textInputChannel.previous(mClient);
        break;
      default:
      case EditorInfo.IME_ACTION_DONE:
        textInputChannel.done(mClient);
        break;
    }
    return true;
  }

  @Override
  @TargetApi(API_LEVELS.API_25)
  @RequiresApi(API_LEVELS.API_25)
  public boolean commitContent(InputContentInfo inputContentInfo, int flags, Bundle opts) {
    // Ensure permission is granted.
    if (Build.VERSION.SDK_INT >= API_LEVELS.API_25
        && (flags & InputConnectionCompat.INPUT_CONTENT_GRANT_READ_URI_PERMISSION) != 0) {
      try {
        inputContentInfo.requestPermission();
      } catch (Exception e) {
        return false;
      }
    } else {
      return false;
    }

    if (inputContentInfo.getDescription().getMimeTypeCount() > 0) {
      inputContentInfo.requestPermission();

      final Uri uri = inputContentInfo.getContentUri();
      final String mimeType = inputContentInfo.getDescription().getMimeType(0);
      Context context = mFlutterView.getContext();

      if (uri != null) {
        InputStream is;
        try {
          // Extract byte data from the given URI.
          is = context.getContentResolver().openInputStream(uri);
        } catch (FileNotFoundException ex) {
          inputContentInfo.releasePermission();
          return false;
        }

        if (is != null) {
          final byte[] data = this.readStreamFully(is, 64 * 1024);

          final Map<String, Object> obj = new HashMap<>();
          obj.put("mimeType", mimeType);
          obj.put("data", data);
          obj.put("uri", uri.toString());

          // Commit the content to the text input channel and release the permission.
          textInputChannel.commitContent(mClient, obj);
          inputContentInfo.releasePermission();
          return true;
        }
      }

      inputContentInfo.releasePermission();
    }
    return false;
  }

  private byte[] readStreamFully(InputStream is, int blocksize) {
    ByteArrayOutputStream baos = new ByteArrayOutputStream();

    byte[] buffer = new byte[blocksize];
    while (true) {
      int len = -1;
      try {
        len = is.read(buffer);
      } catch (IOException ex) {
      }
      if (len == -1) break;
      baos.write(buffer, 0, len);
    }
    return baos.toByteArray();
  }

  // -------- Start: ListenableEditingState watcher implementation -------
  @Override
  public void didChangeEditingState(
      boolean textChanged, boolean selectionChanged, boolean composingRegionChanged) {
    // This method notifies the input method that the editing state has changed.
    // updateSelection is mandatory. updateExtractedText and updateCursorAnchorInfo
    // are on demand (if the input method set the correspoinding monitoring
    // flags). See getExtractedText and requestCursorUpdates.

    // Always send selection update. InputMethodManager#updateSelection skips
    // sending the message if none of the parameters have changed since the last
    // time we called it.
    mImm.updateSelection(
        mFlutterView,
        mEditable.getSelectionStart(),
        mEditable.getSelectionEnd(),
        mEditable.getComposingStart(),
        mEditable.getComposingEnd());

    if (mExtractRequest != null) {
      mImm.updateExtractedText(
          mFlutterView, mExtractRequest.token, getExtractedText(mExtractRequest));
    }
    if (mMonitorCursorUpdate) {
      final CursorAnchorInfo info = getCursorAnchorInfo();
      mImm.updateCursorAnchorInfo(mFlutterView, info);
    }
  }
  // -------- End: ListenableEditingState watcher implementation -------
}
