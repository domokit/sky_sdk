// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.platform;

import android.view.WindowManager;

/** Default implementation when using the regular Android SDK. */
final class WindowManagerHandler extends SingleViewWindowManager {

  WindowManagerHandler(WindowManager delegate, SingleViewFakeWindowViewGroup fakeWindowViewGroup) {
    super(delegate, fakeWindowViewGroup);
  }
}
