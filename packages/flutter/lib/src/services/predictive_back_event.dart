// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/foundation.dart';

/// Enum representing the edge from which a swipe starts in a back gesture.
///
/// This is used in [PredictiveBackEvent] to indicate the starting edge of the
/// swipe gesture.
enum SwipeEdge {
  /// Indicates that the swipe gesture starts from the left edge of the screen.
  left,

  /// Indicates that the swipe gesture starts from the right edge of the screen.
  right,
}

/// Object used to report back gesture progress in Android.
///
/// Holds information about the touch event, swipe direction, and the animation
/// progress that predictive back animations should follow.
@immutable
final class PredictiveBackEvent {
  /// Creates a new [PredictiveBackEvent] instance.
  const PredictiveBackEvent._({
    required this.touchOffset,
    required this.progress,
    required this.swipeEdge,
  }) : assert(progress >= 0.0 && progress <= 1.0);

  /// Creates an [PredictiveBackEvent] from a Map, typically used when converting
  /// data received from a platform channel.
  factory PredictiveBackEvent.fromMap(Map<String?, Object?> map) {
    final List<Object?>? touchOffset = map['touchOffset'] as List<Object?>?;
    return PredictiveBackEvent._(
      touchOffset:
          touchOffset == null
              ? null
              : Offset((touchOffset[0]! as num).toDouble(), (touchOffset[1]! as num).toDouble()),
      progress: (map['progress']! as num).toDouble(),
      swipeEdge: SwipeEdge.values[map['swipeEdge']! as int],
    );
  }

  /// The global position of the touch point as an `Offset`, or `null` if the
  /// event is triggered by a button press.
  ///
  /// This represents the touch location that initiates or interacts with the
  /// back gesture. When `null`, it indicates the gesture was not started by a
  /// touch event, such as a back button press in devices with hardware buttons.
  final Offset? touchOffset;

  /// Returns a value between 0.0 and 1.0 representing how far along the back
  /// gesture is.
  ///
  /// This value is driven by the horizontal location of the touch point, and
  /// should be used as the fraction to seek the predictive back animation with.
  /// Specifically,
  ///
  /// - The progress is 0.0 when the touch is at the starting edge of the screen
  ///   (left or right), and the animation should seek to its start state.
  /// - The progress is approximately 1.0 when the touch is at the opposite side
  ///   of the screen, and the animation should seek to its end state. Exact end
  ///   value may vary depending on screen size.
  ///
  /// When the gesture is canceled, the progress value continues to update,
  /// animating back to 0.0 until the cancellation animation completes.
  ///
  /// In-between locations are linearly interpolated based on horizontal
  /// distance from the starting edge and smooth clamped to 1.0 when the
  /// distance exceeds a system-wide threshold.
  final double progress;

  /// The screen edge from which the swipe gesture starts.
  final SwipeEdge swipeEdge;

  /// Indicates if the event was triggered by a system back button press.
  ///
  /// Returns false for a predictive back gesture.
  bool get isButtonEvent =>
      // The Android documentation for BackEvent
      // (https://developer.android.com/reference/android/window/BackEvent#getTouchX())
      // says that getTouchX and getTouchY should return NaN when the system
      // back button is pressed, but in practice it seems to return 0.0, hence
      // the check for Offset.zero here. This was tested directly in the engine
      // on Android emulator running API 34.
      touchOffset == null || (progress == 0.0 && touchOffset == Offset.zero);

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is PredictiveBackEvent &&
        touchOffset == other.touchOffset &&
        progress == other.progress &&
        swipeEdge == other.swipeEdge;
  }

  @override
  int get hashCode => Object.hash(touchOffset, progress, swipeEdge);

  @override
  String toString() {
    return 'PredictiveBackEvent{touchOffset: $touchOffset, progress: $progress, swipeEdge: $swipeEdge}';
  }
}
