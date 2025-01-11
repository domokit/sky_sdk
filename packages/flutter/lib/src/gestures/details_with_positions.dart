// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' show Offset;

import 'package:flutter/foundation.dart'
    show DiagnosticPropertiesBuilder, Diagnosticable, DiagnosticsProperty;

/// An abstract class representing gesture details that include
/// positional information.
///
/// This class serve as a common interface for gesture details that involve
/// positional data, such as dragging and tapping.
/// It simplifies gesture handling by enabling the use of shared logic across
/// multiple gesture types, users can create a method to handle gesture details
/// with this position information. For example:
///
/// ```dart
/// void handleGesture(GestureDetailsWithPositions details) {
///   final Offset globalPosition = details.globalPosition;
///
///   if (details is TapDownDetails) {
///     // Handles specific detail cases.
///   }
///
///   final Offset localPosition = details.localPosition;
///
///   // Rest of business logic.
/// }
/// ```
abstract class GestureDetailsWithPositions with Diagnosticable {
  /// Creates details with positions.
  const GestureDetailsWithPositions({this.globalPosition = Offset.zero, Offset? localPosition})
    : localPosition = localPosition ?? globalPosition;

  /// The global position at which the pointer interacts with the screen.
  ///
  /// Defaults to the origin if not specified in the constructor.
  ///
  /// See also:
  ///
  ///  * [localPosition], which is the [globalPosition] transformed to the
  ///    coordinate space of the event receiver.
  final Offset globalPosition;

  /// The local position in the coordinate system of the event receiver at
  /// which the pointer interacts with the screen.
  ///  * For *start details, interact means contacting the screen.
  ///  * For *update details, interact means moving on the screen.
  ///  * For *end details, interact means lifted from the screen.
  ///
  /// Defaults to [globalPosition] if not specified in the constructor.
  final Offset localPosition;

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      DiagnosticsProperty<Offset>('globalPosition', globalPosition, defaultValue: Offset.zero),
    );
    properties.add(DiagnosticsProperty<Offset>('localPosition', localPosition));
  }
}
