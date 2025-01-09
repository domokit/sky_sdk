// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
///
/// @docImport 'multidrag.dart';
library;

import 'drag_details.dart';

export 'drag_details.dart' show DragEndDetails, DragUpdateDetails;

/// Interface for objects that receive updates about drags.
///
/// This interface is used in various ways. For example,
/// [MultiDragGestureRecognizer] uses it to update its clients when it
/// recognizes a gesture. Similarly, the scrolling infrastructure in the widgets
/// library uses it to notify the [DragScrollActivity] when the user drags the
/// scrollable.
abstract class Drag {
  /// The pointer has moved.
  void update(DragUpdateDetails details) {}

  /// The pointer is no longer in contact with the screen.
  ///
  /// The velocity at which the pointer was moving when it stopped contacting
  /// the screen is available in the `details`.
  void end(DragEndDetails details) {}

  /// The input from the pointer is no longer directed towards this receiver.
  ///
  /// For example, the user might have been interrupted by a system-modal dialog
  /// in the middle of the drag.
  void cancel() {}
}
