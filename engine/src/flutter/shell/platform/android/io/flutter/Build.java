// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter;

import androidx.annotation.VisibleForTesting;

/** A replacement of utilities from android.os.Build. */
public class Build {
  /** For use in place of the Android Build.VERSION_CODES class. */
  public static class API_LEVELS {
    @VisibleForTesting public static final int FLUTTER_MIN = 21;
    public static final int API_21 = 21;
    public static final int API_22 = 22;
    public static final int API_23 = 23;
    public static final int API_24 = 24;
    public static final int API_25 = 25;
    public static final int API_26 = 26;
    public static final int API_27 = 27;
    public static final int API_28 = 28;
    public static final int API_29 = 29;
    public static final int API_30 = 30;
    public static final int API_31 = 31;
    public static final int API_32 = 32;
    public static final int API_33 = 33;
    public static final int API_34 = 34;
    public static final int API_35 = 35;
  }
}
