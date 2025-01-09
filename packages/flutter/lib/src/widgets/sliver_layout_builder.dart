// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';

import 'framework.dart';
import 'layout_builder.dart';

/// The signature of the [SliverLayoutBuilder] builder function.
typedef SliverLayoutWidgetBuilder =
    Widget Function(BuildContext context, SliverConstraints constraints);

/// Builds a sliver widget tree that can depend on its own [SliverConstraints].
///
/// Similar to the [LayoutBuilder] widget except its builder should return a sliver
/// widget, and [SliverLayoutBuilder] is itself a sliver. The framework calls the
/// [builder] function at layout time and provides the current [SliverConstraints].
/// The [SliverLayoutBuilder]'s final [SliverGeometry] will match the [SliverGeometry]
/// of its child.
///
/// {@macro flutter.widgets.ConstrainedLayoutBuilder}
///
/// See also:
///
///  * [LayoutBuilder], the non-sliver version of this widget.
class SliverLayoutBuilder extends ConstrainedLayoutBuilder<SliverConstraints> {
  /// Creates a sliver widget that defers its building until layout.
  const SliverLayoutBuilder({super.key, required super.builder});

  @override
  RenderObject createRenderObject(BuildContext context) => _RenderSliverLayoutBuilder();
}

class _RenderSliverLayoutBuilder extends RenderSliver
    with
        RenderObjectWithChildMixin<RenderSliver>,
        RenderConstrainedLayoutBuilder<SliverConstraints, RenderSliver> {
  @override
  double childMainAxisPosition(RenderObject child) {
    assert(child == this.child);
    return 0;
  }

  @override
  void performLayout() {
    rebuildIfNecessary();
    child?.layout(constraints, parentUsesSize: true);
    geometry = child?.geometry ?? SliverGeometry.zero;
  }

  @override
  void applyPaintTransform(RenderObject child, Matrix4 transform) {
    assert(child == this.child);
    // child's offset is always (0, 0), transform.translate(0, 0) does not mutate the transform.
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    // This renderObject does not introduce additional offset to child's position.
    if (child?.geometry?.visible ?? false) {
      context.paintChild(child!, offset);
    }
  }

  @override
  bool hitTestChildren(
    SliverHitTestResult result, {
    required double mainAxisPosition,
    required double crossAxisPosition,
  }) {
    return child != null &&
        child!.geometry!.hitTestExtent > 0 &&
        child!.hitTest(
          result,
          mainAxisPosition: mainAxisPosition,
          crossAxisPosition: crossAxisPosition,
        );
  }
}
