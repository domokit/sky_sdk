// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.common;

import androidx.annotation.Nullable;
import io.flutter.BuildConfig;
import io.flutter.Log;

/**
 * An implementation of {@link MethodChannel.Result} that writes error results to the Android log.
 */
public class ErrorLogResult implements MethodChannel.Result {
  private String tag;
  private int level;

  public ErrorLogResult(String tag) {
    this(tag, Log.WARN);
  }

  public ErrorLogResult(String tag, int level) {
    this.tag = tag;
    this.level = level;
  }

  @Override
  public void success(@Nullable Object result) {}

  @Override
  public void error(
      String errorCode, @Nullable String errorMessage, @Nullable Object errorDetails) {
    String details = (errorDetails != null) ? " details: " + errorDetails : "";
    if (level >= Log.WARN || BuildConfig.DEBUG) {
      Log.println(level, tag, errorMessage + details);
    }
  }

  @Override
  public void notImplemented() {
    if (level >= Log.WARN || BuildConfig.DEBUG) {
      Log.println(level, tag, "method not implemented");
    }
  }
}
