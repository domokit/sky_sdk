// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' show max, min;

import 'package:flutter/foundation.dart';

/// A class that describes how textual contents should be scaled for better
/// readability.
///
/// The [scale] function computes the scaled font size given the original
/// unscaled font size specified by app developers.
///
/// The [==] operator defines the equality of 2 [TextScaler]s, which the
/// framework uses to determine whether text widgets should rebuild when their
/// [TextScaler] changes. Consider overriding the [==] operator if applicable
/// to avoid unnecessary rebuilds.
@immutable
abstract class TextScaler {
  /// Creates a TextScaler.
  const TextScaler();

  /// Creates a proportional [TextScaler] that scales the incoming font size by
  /// multiplying it with the given `textScaleFactor`.
  const factory TextScaler.linear(double textScaleFactor) = _LinearTextScaler;

  /// A [TextScaler] that doesn't scale the input font size.
  ///
  /// This is equivalent to `TextScaler.linear(1.0)`, the [TextScaler.scale]
  /// implementation always returns the input font size as-is.
  static const TextScaler noScaling = _LinearTextScaler(1.0);

  /// Computes the scaled font size (in logical pixels) with the given unscaled
  /// `fontSize` (in logical pixels).
  ///
  /// The input `fontSize` must be finite and non-negative.
  ///
  /// When given the same `fontSize` input, this method returns the same value.
  /// The output of a larger input `fontSize` is typically larger than that of a
  /// smaller input, but on unusual occasions they may produce the same output.
  /// For example, some platforms use single-precision floats to represent font
  /// sizes, as a result of truncation two different unscaled font sizes can be
  /// scaled to the same value.
  double scale(double fontSize);

  /// The estimated number of font pixels for each logical pixel. This property
  /// exists only for backward compatibility purposes, and will be removed in
  /// a future version of Flutter.
  ///
  /// The value of this property is only an estimate, so it may not reflect the
  /// exact text scaling strategy this [TextScaler] represents, especially when
  /// this [TextScaler] is not linear. Consider using [TextScaler.scale] instead.
  @Deprecated(
    'Use of textScaleFactor was deprecated in preparation for the upcoming nonlinear text scaling support. '
    'This feature was deprecated after v3.12.0-2.0.pre.',
  )
  double get textScaleFactor;

  /// Returns a new [TextScaler] that restricts the scaled font size to within
  /// the range `[minScaleFactor * fontSize, maxScaleFactor * fontSize]`.
  TextScaler clamp({double minScaleFactor = 0, double maxScaleFactor = double.infinity}) {
    assert(maxScaleFactor >= minScaleFactor);
    assert(!maxScaleFactor.isNaN);
    assert(minScaleFactor.isFinite);
    assert(minScaleFactor >= 0);

    return minScaleFactor == maxScaleFactor
        ? TextScaler.linear(minScaleFactor)
        : _ClampedTextScaler(this, minScaleFactor, maxScaleFactor);
  }
}

final class _LinearTextScaler implements TextScaler {
  const _LinearTextScaler(this.textScaleFactor) : assert(textScaleFactor >= 0);

  @override
  final double textScaleFactor;

  @override
  double scale(double fontSize) {
    assert(fontSize >= 0);
    assert(fontSize.isFinite);
    return fontSize * textScaleFactor;
  }

  @override
  TextScaler clamp({double minScaleFactor = 0, double maxScaleFactor = double.infinity}) {
    assert(maxScaleFactor >= minScaleFactor);
    assert(!maxScaleFactor.isNaN);
    assert(minScaleFactor.isFinite);
    assert(minScaleFactor >= 0);

    final double newScaleFactor = clampDouble(textScaleFactor, minScaleFactor, maxScaleFactor);
    return newScaleFactor == textScaleFactor ? this : _LinearTextScaler(newScaleFactor);
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    return other is _LinearTextScaler && other.textScaleFactor == textScaleFactor;
  }

  @override
  int get hashCode => textScaleFactor.hashCode;

  @override
  String toString() => textScaleFactor == 1.0 ? 'no scaling' : 'linear (${textScaleFactor}x)';
}

final class _ClampedTextScaler implements TextScaler {
  const _ClampedTextScaler(this.scaler, this.minScale, this.maxScale) : assert(maxScale > minScale);
  final TextScaler scaler;
  final double minScale;
  final double maxScale;

  @override
  double get textScaleFactor => clampDouble(scaler.textScaleFactor, minScale, maxScale);

  @override
  double scale(double fontSize) {
    assert(fontSize >= 0);
    assert(fontSize.isFinite);
    return minScale == maxScale
        ? minScale * fontSize
        : clampDouble(scaler.scale(fontSize), minScale * fontSize, maxScale * fontSize);
  }

  @override
  TextScaler clamp({double minScaleFactor = 0, double maxScaleFactor = double.infinity}) {
    return minScaleFactor == maxScaleFactor
        ? _LinearTextScaler(minScaleFactor)
        : _ClampedTextScaler(scaler, max(minScaleFactor, minScale), min(maxScaleFactor, maxScale));
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    return other is _ClampedTextScaler &&
        minScale == other.minScale &&
        maxScale == other.maxScale &&
        (minScale == maxScale || scaler == other.scaler);
  }

  @override
  int get hashCode =>
      minScale == maxScale ? minScale.hashCode : Object.hash(scaler, minScale, maxScale);
}
