// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

/**
 * Port of {@link android.util.Log} that only logs in {@value io.flutter.BuildConfig#DEBUG} mode and
 * internally filters logs based on a {@link #logLevel}.
 */
public class Log {
  private static int logLevel = android.util.Log.DEBUG;

  public static int ASSERT = android.util.Log.ASSERT;
  public static int DEBUG = android.util.Log.DEBUG;
  public static int ERROR = android.util.Log.ERROR;
  public static int INFO = android.util.Log.INFO;
  public static int VERBOSE = android.util.Log.VERBOSE;
  public static int WARN = android.util.Log.WARN;

  /**
   * Sets a log cutoff such that a log level of lower priority than {@code logLevel} is filtered
   * out.
   *
   * <p>See {@link android.util.Log} for log level constants.
   */
  public static void setLogLevel(int logLevel) {
    Log.logLevel = logLevel;
  }

  public static void println(@NonNull int level, @NonNull String tag, @NonNull String message) {
    if (BuildConfig.DEBUG && logLevel <= level) {
      android.util.Log.println(level, tag, message);
    }
  }

  public static void v(@NonNull String tag, @NonNull String message) {
    if (BuildConfig.DEBUG && logLevel <= android.util.Log.VERBOSE) {
      android.util.Log.v(tag, message);
    }
  }

  public static void v(@NonNull String tag, @NonNull String message, @NonNull Throwable tr) {
    if (BuildConfig.DEBUG && logLevel <= android.util.Log.VERBOSE) {
      android.util.Log.v(tag, message, tr);
    }
  }

  public static void i(@NonNull String tag, @NonNull String message) {
    if (BuildConfig.DEBUG && logLevel <= android.util.Log.INFO) {
      android.util.Log.i(tag, message);
    }
  }

  public static void i(@NonNull String tag, @NonNull String message, @NonNull Throwable tr) {
    if (BuildConfig.DEBUG && logLevel <= android.util.Log.INFO) {
      android.util.Log.i(tag, message, tr);
    }
  }

  public static void d(@NonNull String tag, @NonNull String message) {
    if (BuildConfig.DEBUG && logLevel <= android.util.Log.DEBUG) {
      android.util.Log.d(tag, message);
    }
  }

  public static void d(@NonNull String tag, @NonNull String message, @NonNull Throwable tr) {
    if (BuildConfig.DEBUG && logLevel <= android.util.Log.DEBUG) {
      android.util.Log.d(tag, message, tr);
    }
  }

  public static void w(@NonNull String tag, @NonNull String message) {
    android.util.Log.w(tag, message);
  }

  public static void w(@NonNull String tag, @NonNull String message, @NonNull Throwable tr) {
    android.util.Log.w(tag, message, tr);
  }

  public static void e(@NonNull String tag, @NonNull String message) {
    android.util.Log.e(tag, message);
  }

  public static void e(@NonNull String tag, @NonNull String message, @NonNull Throwable tr) {
    android.util.Log.e(tag, message, tr);
  }

  public static void wtf(@NonNull String tag, @NonNull String message) {
    android.util.Log.wtf(tag, message);
  }

  public static void wtf(@NonNull String tag, @NonNull String message, @NonNull Throwable tr) {
    android.util.Log.wtf(tag, message, tr);
  }

  @NonNull
  public static String getStackTraceString(@Nullable Throwable tr) {
    return android.util.Log.getStackTraceString(tr);
  }
}
