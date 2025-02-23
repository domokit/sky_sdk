// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.view;

import static io.flutter.Build.API_LEVELS;

import android.app.Activity;
import android.os.Build;
import android.view.View;
import androidx.annotation.NonNull;
import io.flutter.embedding.engine.systemchannels.SensitiveContentChannel;
import io.flutter.plugin.common.MethodChannel;

/**
 * {@link SensitiveContentPlugin} is the implementation of all functionality needed to set and
 * retrieve the content sensitivity level of a native Flutter Android {@code View}.
 *
 * <p>This plugin handles requests for getting and setting content sensitivity sent by the {@link
 * io.flutter.embedding.engine.systemchannels.SensitiveContentChannel}.
 */
public class SensitiveContentPlugin
    implements SensitiveContentChannel.SensitiveContentMethodHandler {

  private final int mFlutterViewId;
  private final Activity mFlutterActivity;
  private final SensitiveContentChannel mSensitiveContentChannel;

  public SensitiveContentPlugin(
      @NonNull int flutterViewId,
      @NonNull Activity activity,
      @NonNull SensitiveContentChannel sensitiveContentChannel) {
    mFlutterActivity = activity;
    mFlutterViewId = flutterViewId;
    mSensitiveContentChannel = sensitiveContentChannel;

    mSensitiveContentChannel.setSensitiveContentMethodHandler(this);
  }

  /**
   * Sets content sensitivity level of the Android {@code View} associated with this plugin's {@code
   * mFlutterViewId} to the level specified by {@requestedContentSensitivity}.
   */
  @Override
  public void setContentSensitivity(
      @NonNull int requestedContentSensitivity, @NonNull MethodChannel.Result result) {
    if (!isSupported()) {
      // This feature is only available on > API 35.
      result.success(null);
      return;
    }

    final View flutterView = mFlutterActivity.findViewById(mFlutterViewId);
    final int initialContentSensitivity = flutterView.getContentSensitivity();

    if (initialContentSensitivity == requestedContentSensitivity) {
      // Content sensitivity for the requested View already set to requestedContentSensitivity.
      result.success(null);
      return;
    }

    // Set requestedContentSensitivity on the View.
    flutterView.setContentSensitivity(requestedContentSensitivity);

    // Invalidate the View to force a redraw in order to ensure that it updates to be
    // obscured/unobscured as expected.
    flutterView.invalidate();

    result.success(null);
  }

  /**
   * Gets content sensitivity level of the Android {@code View} associated with this plugin's {@code
   * mFlutterViewId}.
   */
  @Override
  public void getContentSensitivity(@NonNull MethodChannel.Result result) {
    if (!isSupported()) {
      // This feature is only available on > API 35.
      result.success(null);
      return;
    }

    final View flutterView = mFlutterActivity.findViewById(mFlutterViewId);
    final int currentContentSensitivity = flutterView.getContentSensitivity();
    result.success(currentContentSensitivity);
  }

  /**
   * Returns whether or not marking content sensitivity is supported on the device.
   *
   * <p>It is supported on devices running Android API >= 35.
   */
  @Override
  public void isSupported(@NonNull MethodChannel.Result result) {
    result.success(isSupported());
  }

  private boolean isSupported() {
    return Build.VERSION.SDK_INT >= API_LEVELS.API_35;
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
