// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'notification_listener.dart';
/// @docImport 'scroll_view.dart';
/// @docImport 'sliver_floating_header.dart';
/// @docImport 'sliver_persistent_header.dart';
/// @docImport 'sliver_resizing_header.dart';
library;

import 'dart:math' as math;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';

import 'framework.dart';

/// A sliver that keeps its Widget child at the top of the a [CustomScrollView].
///
/// This sliver is preferable to the general purpose [SliverPersistentHeader]
/// for its relatively narrow use case because there's no need to create a
/// [SliverPersistentHeaderDelegate] or to predict the header's size.
///
/// {@tool dartpad}
/// This example demonstrates that the sliver's size can change. Pressing the
/// floating action button replaces the one line of header text with two lines.
///
/// ** See code in examples/api/lib/widgets/sliver/pinned_header_sliver.0.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// A more elaborate example which creates an app bar that's similar to the one
/// that appears in the iOS Settings app. In this example the pinned header
/// starts out transparent and the first item in the list serves as the app's
/// "Settings" title. When the title item has been scrolled completely behind
/// the pinned header, the header animates its opacity from 0 to 1 and its
/// (centered) "Settings" title appears. The fact that the header's opacity
/// depends on the height of the title item - which is unknown until the list
/// has been laid out - necessitates monitoring the title item's
/// [SliverGeometry.scrollExtent] and the header's [SliverConstraints.scrollOffset]
/// from a scroll [NotificationListener]. See the source code for more details.
///
/// ** See code in examples/api/lib/widgets/sliver/pinned_header_sliver.1.dart **
/// {@end-tool}
///
/// See also:
///
///  * [SliverResizingHeader] - which similarly pins the header at the top
///    of the [CustomScrollView] but reacts to scrolling by resizing the header
///    between its minimum and maximum extent limits.
///  * [SliverFloatingHeader] - which animates the header in and out of view
///    in response to downward and upwards scrolls.
///  * [SliverPersistentHeader] - a general purpose header that can be
///    configured as a pinned, resizing, or floating header.
class PinnedHeaderSliver extends SingleChildRenderObjectWidget {
  /// Creates a sliver whose [Widget] child appears at the top of a
  /// [CustomScrollView].
  const PinnedHeaderSliver({super.key, super.child});

  @override
  RenderObject createRenderObject(BuildContext context) {
    return _RenderPinnedHeaderSliver();
  }
}

class _RenderPinnedHeaderSliver extends RenderSliverSingleBoxAdapter {
  _RenderPinnedHeaderSliver();

  double get childExtent {
    if (child == null) {
      return 0.0;
    }
    assert(child!.hasSize);
    return switch (constraints.axis) {
      Axis.vertical => child!.size.height,
      Axis.horizontal => child!.size.width,
    };
  }

  @override
  double childMainAxisPosition(covariant RenderObject child) => 0;

  @override
  void performLayout() {
    final SliverConstraints constraints = this.constraints;
    child?.layout(constraints.asBoxConstraints(), parentUsesSize: true);

    final double layoutExtent = clampDouble(
      childExtent - constraints.scrollOffset,
      0,
      constraints.remainingPaintExtent,
    );
    final double paintExtent = math.min(
      childExtent,
      constraints.remainingPaintExtent - constraints.overlap,
    );
    geometry = SliverGeometry(
      scrollExtent: childExtent,
      paintOrigin: constraints.overlap,
      paintExtent: paintExtent,
      layoutExtent: layoutExtent,
      maxPaintExtent: childExtent,
      maxScrollObstructionExtent: childExtent,
      cacheExtent: calculateCacheOffset(constraints, from: 0.0, to: childExtent),
      hasVisualOverflow: true, // Conservatively say we do have overflow to avoid complexity.
    );
  }
}
