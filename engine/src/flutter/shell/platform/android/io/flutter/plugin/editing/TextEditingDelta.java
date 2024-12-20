// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.editing;

import androidx.annotation.NonNull;
import androidx.annotation.VisibleForTesting;
import io.flutter.Log;
import org.json.JSONException;
import org.json.JSONObject;

/// A representation of the change that occured to an editing state, along with the resulting
/// composing and selection regions.
public final class TextEditingDelta {
  private @NonNull CharSequence oldText;
  private @NonNull CharSequence deltaText;
  private int deltaStart;
  private int deltaEnd;
  private int newSelectionStart;
  private int newSelectionEnd;
  private int newComposingStart;
  private int newComposingEnd;

  private static final String TAG = "TextEditingDelta";

  public TextEditingDelta(
      @NonNull CharSequence oldEditable,
      int replacementDestinationStart,
      int replacementDestinationEnd,
      @NonNull CharSequence replacementSource,
      int selectionStart,
      int selectionEnd,
      int composingStart,
      int composingEnd) {
    newSelectionStart = selectionStart;
    newSelectionEnd = selectionEnd;
    newComposingStart = composingStart;
    newComposingEnd = composingEnd;

    setDeltas(
        oldEditable,
        replacementSource.toString(),
        replacementDestinationStart,
        replacementDestinationEnd);
  }

  // Non text update delta constructor.
  public TextEditingDelta(
      @NonNull CharSequence oldText,
      int selectionStart,
      int selectionEnd,
      int composingStart,
      int composingEnd) {
    newSelectionStart = selectionStart;
    newSelectionEnd = selectionEnd;
    newComposingStart = composingStart;
    newComposingEnd = composingEnd;

    setDeltas(oldText, "", -1, -1);
  }

  @VisibleForTesting
  @NonNull
  public CharSequence getOldText() {
    return oldText;
  }

  @VisibleForTesting
  @NonNull
  public CharSequence getDeltaText() {
    return deltaText;
  }

  @VisibleForTesting
  public int getDeltaStart() {
    return deltaStart;
  }

  @VisibleForTesting
  public int getDeltaEnd() {
    return deltaEnd;
  }

  @VisibleForTesting
  public int getNewSelectionStart() {
    return newSelectionStart;
  }

  @VisibleForTesting
  public int getNewSelectionEnd() {
    return newSelectionEnd;
  }

  @VisibleForTesting
  public int getNewComposingStart() {
    return newComposingStart;
  }

  @VisibleForTesting
  public int getNewComposingEnd() {
    return newComposingEnd;
  }

  private void setDeltas(
      @NonNull CharSequence oldText, @NonNull CharSequence newText, int newStart, int newExtent) {
    this.oldText = oldText;
    deltaText = newText;
    deltaStart = newStart;
    deltaEnd = newExtent;
  }

  @NonNull
  public JSONObject toJSON() {
    JSONObject delta = new JSONObject();

    try {
      delta.put("oldText", oldText.toString());
      delta.put("deltaText", deltaText.toString());
      delta.put("deltaStart", deltaStart);
      delta.put("deltaEnd", deltaEnd);
      delta.put("selectionBase", newSelectionStart);
      delta.put("selectionExtent", newSelectionEnd);
      delta.put("composingBase", newComposingStart);
      delta.put("composingExtent", newComposingEnd);
    } catch (JSONException e) {
      Log.e(TAG, "unable to create JSONObject: " + e);
    }

    return delta;
  }
}
