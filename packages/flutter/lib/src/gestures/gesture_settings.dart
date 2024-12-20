// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
library;

import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';

export 'dart:ui' show FlutterView;

/// The device specific gesture settings scaled into logical pixels.
///
/// This configuration can be retrieved from the window, or more commonly from a
/// [MediaQuery] widget.
///
/// See also:
///
///  * [ui.GestureSettings], the configuration that this is derived from.
@immutable
class DeviceGestureSettings {
  /// Create a new [DeviceGestureSettings] with configured settings in logical
  /// pixels.
  const DeviceGestureSettings({this.touchSlop});

  /// Create a new [DeviceGestureSettings] from the provided [view].
  factory DeviceGestureSettings.fromView(ui.FlutterView view) {
    final double? physicalTouchSlop = view.gestureSettings.physicalTouchSlop;
    return DeviceGestureSettings(
      touchSlop: physicalTouchSlop == null ? null : physicalTouchSlop / view.devicePixelRatio,
    );
  }

  /// The touch slop value in logical pixels, or `null` if it was not set.
  final double? touchSlop;

  /// The touch slop value for pan gestures, in logical pixels, or `null` if it
  /// was not set.
  double? get panSlop => touchSlop != null ? (touchSlop! * 2) : null;

  @override
  int get hashCode => Object.hash(touchSlop, 23);

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is DeviceGestureSettings && other.touchSlop == touchSlop;
  }

  @override
  String toString() => 'DeviceGestureSettings(touchSlop: $touchSlop)';
}
