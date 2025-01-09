// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:ui';
/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'scroll_position.dart';
/// @docImport 'scrollable.dart';
/// @docImport 'viewport.dart';
library;

import 'dart:math' as math;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';

/// A description of a [Scrollable]'s contents, useful for modeling the state
/// of its viewport.
///
/// This class defines a current position, [pixels], and a range of values
/// considered "in bounds" for that position. The range has a minimum value at
/// [minScrollExtent] and a maximum value at [maxScrollExtent] (inclusive). The
/// viewport scrolls in the direction and axis described by [axisDirection]
/// and [axis].
///
/// The [outOfRange] getter will return true if [pixels] is outside this defined
/// range. The [atEdge] getter will return true if the [pixels] position equals
/// either the [minScrollExtent] or the [maxScrollExtent].
///
/// The dimensions of the viewport in the given [axis] are described by
/// [viewportDimension].
///
/// The above values are also exposed in terms of [extentBefore],
/// [extentInside], and [extentAfter], which may be more useful for use cases
/// such as scroll bars; for example, see [Scrollbar].
///
/// {@tool dartpad}
/// This sample shows how a [ScrollMetricsNotification] is dispatched when
/// the [ScrollMetrics] changed as a result of resizing the [Viewport].
/// Press the floating action button to increase the scrollable window's size.
///
/// ** See code in examples/api/lib/widgets/scroll_position/scroll_metrics_notification.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [FixedScrollMetrics], which is an immutable object that implements this
///    interface.
mixin ScrollMetrics {
  /// Creates a [ScrollMetrics] that has the same properties as this object.
  ///
  /// This is useful if this object is mutable, but you want to get a snapshot
  /// of the current state.
  ///
  /// The named arguments allow the values to be adjusted in the process. This
  /// is useful to examine hypothetical situations, for example "would applying
  /// this delta unmodified take the position [outOfRange]?".
  ScrollMetrics copyWith({
    double? minScrollExtent,
    double? maxScrollExtent,
    double? pixels,
    double? viewportDimension,
    AxisDirection? axisDirection,
    double? devicePixelRatio,
  }) {
    return FixedScrollMetrics(
      minScrollExtent: minScrollExtent ?? (hasContentDimensions ? this.minScrollExtent : null),
      maxScrollExtent: maxScrollExtent ?? (hasContentDimensions ? this.maxScrollExtent : null),
      pixels: pixels ?? (hasPixels ? this.pixels : null),
      viewportDimension:
          viewportDimension ?? (hasViewportDimension ? this.viewportDimension : null),
      axisDirection: axisDirection ?? this.axisDirection,
      devicePixelRatio: devicePixelRatio ?? this.devicePixelRatio,
    );
  }

  /// The minimum in-range value for [pixels].
  ///
  /// The actual [pixels] value might be [outOfRange].
  ///
  /// This value is typically less than or equal to
  /// [maxScrollExtent]. It can be negative infinity, if the scroll is unbounded.
  double get minScrollExtent;

  /// The maximum in-range value for [pixels].
  ///
  /// The actual [pixels] value might be [outOfRange].
  ///
  /// This value is typically greater than or equal to
  /// [minScrollExtent]. It can be infinity, if the scroll is unbounded.
  double get maxScrollExtent;

  /// Whether the [minScrollExtent] and the [maxScrollExtent] properties are available.
  bool get hasContentDimensions;

  /// The current scroll position, in logical pixels along the [axisDirection].
  double get pixels;

  /// Whether the [pixels] property is available.
  bool get hasPixels;

  /// The extent of the viewport along the [axisDirection].
  double get viewportDimension;

  /// Whether the [viewportDimension] property is available.
  bool get hasViewportDimension;

  /// The direction in which the scroll view scrolls.
  AxisDirection get axisDirection;

  /// The axis in which the scroll view scrolls.
  Axis get axis => axisDirectionToAxis(axisDirection);

  /// Whether the [pixels] value is outside the [minScrollExtent] and
  /// [maxScrollExtent].
  bool get outOfRange => pixels < minScrollExtent || pixels > maxScrollExtent;

  /// Whether the [pixels] value is exactly at the [minScrollExtent] or the
  /// [maxScrollExtent].
  bool get atEdge => pixels == minScrollExtent || pixels == maxScrollExtent;

  /// The quantity of content conceptually "above" the viewport in the scrollable.
  /// This is the content above the content described by [extentInside].
  double get extentBefore => math.max(pixels - minScrollExtent, 0.0);

  /// The quantity of content conceptually "inside" the viewport in the
  /// scrollable (including empty space if the total amount of content is less
  /// than the [viewportDimension]).
  ///
  /// The value is typically the extent of the viewport ([viewportDimension])
  /// when [outOfRange] is false. It can be less when overscrolling.
  ///
  /// The value is always non-negative, and less than or equal to [viewportDimension].
  double get extentInside {
    assert(minScrollExtent <= maxScrollExtent);
    return viewportDimension
        // "above" overscroll value
        -
        clampDouble(minScrollExtent - pixels, 0, viewportDimension)
        // "below" overscroll value
        -
        clampDouble(pixels - maxScrollExtent, 0, viewportDimension);
  }

  /// The quantity of content conceptually "below" the viewport in the scrollable.
  /// This is the content below the content described by [extentInside].
  double get extentAfter => math.max(maxScrollExtent - pixels, 0.0);

  /// The total quantity of content available.
  ///
  /// This is the sum of [extentBefore], [extentInside], and [extentAfter], modulo
  /// any rounding errors.
  double get extentTotal => maxScrollExtent - minScrollExtent + viewportDimension;

  /// The [FlutterView.devicePixelRatio] of the view that the [Scrollable]
  /// associated with this metrics object is drawn into.
  double get devicePixelRatio;
}

/// An immutable snapshot of values associated with a [Scrollable] viewport.
///
/// For details, see [ScrollMetrics], which defines this object's interfaces.
///
/// {@tool dartpad}
/// This sample shows how a [ScrollMetricsNotification] is dispatched when
/// the [ScrollMetrics] changed as a result of resizing the [Viewport].
/// Press the floating action button to increase the scrollable window's size.
///
/// ** See code in examples/api/lib/widgets/scroll_position/scroll_metrics_notification.0.dart **
/// {@end-tool}
class FixedScrollMetrics with ScrollMetrics {
  /// Creates an immutable snapshot of values associated with a [Scrollable] viewport.
  FixedScrollMetrics({
    required double? minScrollExtent,
    required double? maxScrollExtent,
    required double? pixels,
    required double? viewportDimension,
    required this.axisDirection,
    required this.devicePixelRatio,
  }) : _minScrollExtent = minScrollExtent,
       _maxScrollExtent = maxScrollExtent,
       _pixels = pixels,
       _viewportDimension = viewportDimension;

  @override
  double get minScrollExtent => _minScrollExtent!;
  final double? _minScrollExtent;

  @override
  double get maxScrollExtent => _maxScrollExtent!;
  final double? _maxScrollExtent;

  @override
  bool get hasContentDimensions => _minScrollExtent != null && _maxScrollExtent != null;

  @override
  double get pixels => _pixels!;
  final double? _pixels;

  @override
  bool get hasPixels => _pixels != null;

  @override
  double get viewportDimension => _viewportDimension!;
  final double? _viewportDimension;

  @override
  bool get hasViewportDimension => _viewportDimension != null;

  @override
  final AxisDirection axisDirection;

  @override
  final double devicePixelRatio;

  @override
  String toString() {
    return '${objectRuntimeType(this, 'FixedScrollMetrics')}(${extentBefore.toStringAsFixed(1)}..[${extentInside.toStringAsFixed(1)}]..${extentAfter.toStringAsFixed(1)})';
  }
}
