// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.view;

import android.app.Activity;
import android.view.View;
import androidx.annotation.NonNull;
import io.flutter.embedding.engine.systemchannels.SensitiveContentChannel;
import io.flutter.plugin.common.MethodChannel;

/**
 * {@link SensitiveContentPlugin} is the implementation of all functionality needed to set
 * the content sensitivity level of a native Flutter Android {@code View}.
 *
 * <p>This plugin handles requests for setting content sensitivity sent by the {@link
 * io.flutter.embedding.engine.systemchannels.SensitiveContentChannel}.
 */
public class SensitiveContentPlugin
    implements SensitiveContentChannel.SensitiveContentMethodHandler {

  private final Activity mflutterActivity;
  private final SensitiveContentChannel mSensitiveContentChannel;

  public SensitiveContentPlugin(
      @NonNull Activity flutterActivity, @NonNull SensitiveContentChannel sensitiveContentChannel) {
    mflutterActivity = flutterActivity;
    mSensitiveContentChannel = sensitiveContentChannel;

    mSensitiveContentChannel.setSensitiveContentMethodHandler(this);
  }

  /**
   * Sets content sensitivity level of the Android {@code View} with the specified {@code
   * flutterViewId} to the level specified by {@contentSensitivity}.
   */
  @Override
  public void setContentSensitivity(
      @NonNull int flutterViewId,
      @NonNull int contentSensitivity,
      @NonNull MethodChannel.Result result) {
    final View flutterView = mflutterActivity.findViewById(flutterViewId);
    if (flutterView == null) {
      result.error("error", "Requested Flutter View to set content sensitivty of not found.", null);
    }

    // Set contentSensitivity on the requested View.
    flutterView.setContentSensitivity(contentSensitivity);

    // Invalidate the View to force a redraw if we require that the screen
    // become unobscured, which is the case where the View was previously
    // marked sensitive but now is no longer.
    final int currentContentSensitivity = flutterView.getContentSensitivity();
    final boolean shouldInvalidateView =
        currentContentSensitivity == View.CONTENT_SENSITIVITY_SENSITIVE
            && contentSensitivity != View.CONTENT_SENSITIVITY_SENSITIVE;
    if (shouldInvalidateView) {
      flutterView.invalidate();
    }

    result.success(null);
  }

  /**
   * Releases all resources held by this {@code SensitiveContentPlugin}.
   *
   * <p>Do not invoke any methods on a {@code SensitiveContentPlugin} after invoking this method.
   */
  public void destroy() {
    this.mSensitiveContentChannel.setSensitiveContentMethodHandler(null);
  }
}
