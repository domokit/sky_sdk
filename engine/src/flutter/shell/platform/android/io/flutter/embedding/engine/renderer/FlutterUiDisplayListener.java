// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.renderer;

/**
 * Listener invoked when Flutter starts and stops rendering pixels to an Android {@code View}
 * hierarchy.
 */
public interface FlutterUiDisplayListener {
  /**
   * Flutter started painting pixels to an Android {@code View} hierarchy.
   *
   * <p>This method will not be invoked if this listener is added after the {@link FlutterRenderer}
   * has started painting pixels.
   */
  void onFlutterUiDisplayed();

  /** Flutter stopped painting pixels to an Android {@code View} hierarchy. */
  void onFlutterUiNoLongerDisplayed();
}
