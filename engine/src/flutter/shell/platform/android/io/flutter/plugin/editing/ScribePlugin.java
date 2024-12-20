// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.editing;

import static io.flutter.Build.API_LEVELS;

import android.annotation.TargetApi;
import android.os.Build;
import android.view.View;
import android.view.inputmethod.InputMethodManager;
import androidx.annotation.NonNull;
import androidx.annotation.RequiresApi;
import io.flutter.embedding.engine.systemchannels.ScribeChannel;

/**
 * {@link ScribePlugin} is the implementation of all functionality needed for handwriting stylus
 * text input.
 *
 * <p>The plugin handles requests for scribe sent by the {@link
 * io.flutter.embedding.engine.systemchannels.ScribeChannel}.
 *
 * <p>On API versions below 33, the plugin does nothing.
 */
public class ScribePlugin implements ScribeChannel.ScribeMethodHandler {

  @NonNull private final ScribeChannel mScribeChannel;
  @NonNull private final InputMethodManager mInputMethodManager;
  @NonNull private View mView;

  public ScribePlugin(
      @NonNull View view, @NonNull InputMethodManager imm, @NonNull ScribeChannel scribeChannel) {
    if (Build.VERSION.SDK_INT >= API_LEVELS.API_33) {
      view.setAutoHandwritingEnabled(false);
    }

    mView = view;
    mInputMethodManager = imm;
    mScribeChannel = scribeChannel;

    mScribeChannel.setScribeMethodHandler(this);
  }

  /**
   * Sets the View in which Scribe input is handled.
   *
   * <p>Only one View can be set at any given time.
   */
  public void setView(@NonNull View view) {
    if (view == mView) {
      return;
    }
    mView = view;
  }

  /**
   * Unregisters this {@code ScribePlugin} as the {@code ScribeChannel.ScribeMethodHandler}, for the
   * {@link io.flutter.embedding.engine.systemchannels.ScribeChannel}.
   *
   * <p>Do not invoke any methods on a {@code ScribePlugin} after invoking this method.
   */
  public void destroy() {
    mScribeChannel.setScribeMethodHandler(null);
  }

  /**
   * Returns true if the InputMethodManager supports Scribe stylus handwriting input.
   *
   * <p>Call this or isFeatureAvailable before calling startStylusHandwriting to make sure it's
   * available.
   */
  @TargetApi(API_LEVELS.API_34)
  @RequiresApi(API_LEVELS.API_34)
  @Override
  public boolean isStylusHandwritingAvailable() {
    return mInputMethodManager.isStylusHandwritingAvailable();
  }

  /**
   * Starts stylus handwriting input.
   *
   * <p>Typically isStylusHandwritingAvailable should be called first to determine whether this is
   * supported by the IME.
   */
  @TargetApi(API_LEVELS.API_33)
  @RequiresApi(API_LEVELS.API_33)
  @Override
  public void startStylusHandwriting() {
    mInputMethodManager.startStylusHandwriting(mView);
  }

  /**
   * A convenience method to check if Scribe is available.
   *
   * <p>Differs from isStylusHandwritingAvailable in that it can be called from any API level
   * without throwing an error.
   *
   * <p>Call this or isStylusHandwritingAvailable before calling startStylusHandwriting to make sure
   * it's available.
   */
  @Override
  public boolean isFeatureAvailable() {
    return Build.VERSION.SDK_INT >= API_LEVELS.API_34 && isStylusHandwritingAvailable();
  }
}
