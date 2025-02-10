// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

/// A scenario to run for testing.
abstract class Scenario {
  /// Creates a new scenario using a specific FlutterView instance.
  Scenario(this.view);

  /// The FlutterView used by this scenario. May be mocked.
  final FlutterView view;

  /// [true] if a screenshot is taken in the next frame.
  bool _didScheduleScreenshot = false;

  /// Called by the program when a frame is ready to be drawn.
  ///
  /// See [PlatformDispatcher.onBeginFrame] for more details.
  void onBeginFrame(Duration duration) {}

  /// Called by the program when the microtasks from [onBeginFrame] have been
  /// flushed.
  ///
  /// See [PlatformDispatcher.onDrawFrame] for more details.
  void onDrawFrame() {
    if (_didScheduleScreenshot) {
      view.platformDispatcher.sendPlatformMessage('take_screenshot', null, null);
      return;
    }
    Future<void>.delayed(const Duration(seconds: 2), () {
      _didScheduleScreenshot = true;
      view.platformDispatcher.scheduleFrame();
    });
  }

  /// Called when the current scenario has been unmount due to a
  /// new scenario being mount.
  void unmount() {
    _didScheduleScreenshot = false;
  }

  /// Called by the program when the window metrics have changed.
  ///
  /// See [PlatformDispatcher.onMetricsChanged].
  void onMetricsChanged() {}

  /// Called by the program when a pointer event is received.
  ///
  /// See [PlatformDispatcher.onPointerDataPacket].
  void onPointerDataPacket(PointerDataPacket packet) {}
}
