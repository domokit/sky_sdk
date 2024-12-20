// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
library;

import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

import 'object.dart';
import 'sliver.dart';

/// A sliver that places multiple sliver children in a linear array along the cross
/// axis.
///
/// Since the extent of the viewport in the cross axis direction is finite,
/// this extent will be divided up and allocated to the children slivers.
///
/// The algorithm for dividing up the cross axis extent is as follows.
/// Every widget has a [SliverPhysicalParentData.crossAxisFlex] value associated with them.
/// First, lay out all of the slivers with flex of 0 or null, in which case the slivers themselves will
/// figure out how much cross axis extent to take up. For example, [SliverConstrainedCrossAxis]
/// is an example of a widget which sets its own flex to 0. Then [RenderSliverCrossAxisGroup] will
/// divide up the remaining space to all the remaining children proportionally
/// to each child's flex factor. By default, children of [SliverCrossAxisGroup]
/// are setup to have a flex factor of 1, but a different flex factor can be
/// specified via the [SliverCrossAxisExpanded] widgets.
class RenderSliverCrossAxisGroup extends RenderSliver
    with ContainerRenderObjectMixin<RenderSliver, SliverPhysicalContainerParentData> {
  @override
  void setupParentData(RenderObject child) {
    if (child.parentData is! SliverPhysicalContainerParentData) {
      child.parentData = SliverPhysicalContainerParentData();
      (child.parentData! as SliverPhysicalParentData).crossAxisFlex = 1;
    }
  }

  @override
  double childMainAxisPosition(RenderSliver child) => 0.0;

  @override
  double childCrossAxisPosition(RenderSliver child) {
    final Offset paintOffset = (child.parentData! as SliverPhysicalParentData).paintOffset;
    return switch (constraints.axis) {
      Axis.vertical => paintOffset.dx,
      Axis.horizontal => paintOffset.dy,
    };
  }

  @override
  void performLayout() {
    // Iterate through each sliver.
    // Get the parent's dimensions.
    final double crossAxisExtent = constraints.crossAxisExtent;
    assert(crossAxisExtent.isFinite);

    // First, layout each child with flex == 0 or null.
    int totalFlex = 0;
    double remainingExtent = crossAxisExtent;
    RenderSliver? child = firstChild;
    while (child != null) {
      final SliverPhysicalParentData childParentData =
          child.parentData! as SliverPhysicalParentData;
      final int flex = childParentData.crossAxisFlex ?? 0;
      if (flex == 0) {
        // If flex is 0 or null, then the child sliver must provide their own crossAxisExtent.
        assert(_assertOutOfExtent(remainingExtent));
        child.layout(constraints.copyWith(crossAxisExtent: remainingExtent), parentUsesSize: true);
        final double? childCrossAxisExtent = child.geometry!.crossAxisExtent;
        assert(childCrossAxisExtent != null);
        remainingExtent = math.max(0.0, remainingExtent - childCrossAxisExtent!);
      } else {
        totalFlex += flex;
      }
      child = childAfter(child);
    }
    final double extentPerFlexValue = remainingExtent / totalFlex;

    child = firstChild;

    // At this point, all slivers with constrained cross axis should already be laid out.
    // Layout the rest and keep track of the child geometry with greatest scrollExtent.
    geometry = SliverGeometry.zero;
    while (child != null) {
      final SliverPhysicalParentData childParentData =
          child.parentData! as SliverPhysicalParentData;
      final int flex = childParentData.crossAxisFlex ?? 0;
      double childExtent;
      if (flex != 0) {
        childExtent = extentPerFlexValue * flex;
        assert(_assertOutOfExtent(childExtent));
        child.layout(
          constraints.copyWith(crossAxisExtent: extentPerFlexValue * flex),
          parentUsesSize: true,
        );
      } else {
        childExtent = child.geometry!.crossAxisExtent!;
      }
      final SliverGeometry childLayoutGeometry = child.geometry!;
      if (geometry!.scrollExtent < childLayoutGeometry.scrollExtent) {
        geometry = childLayoutGeometry;
      }
      child = childAfter(child);
    }

    // Go back and correct any slivers using a negative paint offset if it tries
    // to paint outside the bounds of the sliver group.
    child = firstChild;
    double offset = 0.0;
    while (child != null) {
      final SliverPhysicalParentData childParentData =
          child.parentData! as SliverPhysicalParentData;
      final SliverGeometry childLayoutGeometry = child.geometry!;
      final double remainingExtent = geometry!.scrollExtent - constraints.scrollOffset;
      final double paintCorrection =
          childLayoutGeometry.paintExtent > remainingExtent
              ? childLayoutGeometry.paintExtent - remainingExtent
              : 0.0;
      final double childExtent =
          child.geometry!.crossAxisExtent ??
          extentPerFlexValue * (childParentData.crossAxisFlex ?? 0);
      // Set child parent data.
      childParentData.paintOffset = switch (constraints.axis) {
        Axis.vertical => Offset(offset, -paintCorrection),
        Axis.horizontal => Offset(-paintCorrection, offset),
      };
      offset += childExtent;
      child = childAfter(child);
    }
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    RenderSliver? child = firstChild;

    while (child != null) {
      if (child.geometry!.visible) {
        final SliverPhysicalParentData childParentData =
            child.parentData! as SliverPhysicalParentData;
        context.paintChild(child, offset + childParentData.paintOffset);
      }
      child = childAfter(child);
    }
  }

  @override
  void applyPaintTransform(RenderSliver child, Matrix4 transform) {
    final SliverPhysicalParentData childParentData = child.parentData! as SliverPhysicalParentData;
    childParentData.applyPaintTransform(transform);
  }

  @override
  bool hitTestChildren(
    SliverHitTestResult result, {
    required double mainAxisPosition,
    required double crossAxisPosition,
  }) {
    RenderSliver? child = lastChild;
    while (child != null) {
      final bool isHit = result.addWithAxisOffset(
        mainAxisPosition: mainAxisPosition,
        crossAxisPosition: crossAxisPosition,
        paintOffset: null,
        mainAxisOffset: childMainAxisPosition(child),
        crossAxisOffset: childCrossAxisPosition(child),
        hitTest: child.hitTest,
      );
      if (isHit) {
        return true;
      }
      child = childBefore(child);
    }
    return false;
  }
}

bool _assertOutOfExtent(double extent) {
  if (extent <= 0.0) {
    throw FlutterError.fromParts(<DiagnosticsNode>[
      ErrorSummary('SliverCrossAxisGroup ran out of extent before child could be laid out.'),
      ErrorDescription(
        'SliverCrossAxisGroup lays out any slivers with a constrained cross '
        'axis before laying out those which expand. In this case, cross axis '
        'extent was used up before the next sliver could be laid out.',
      ),
      ErrorHint(
        'Make sure that the total amount of extent allocated by constrained '
        'child slivers does not exceed the cross axis extent that is available '
        'for the SliverCrossAxisGroup.',
      ),
    ]);
  }
  return true;
}

/// A sliver that places multiple sliver children in a linear array along the
/// main axis.
///
/// The layout algorithm lays out slivers one by one. If the sliver is at the top
/// of the viewport or above the top, then we pass in a nonzero [SliverConstraints.scrollOffset]
/// to inform the sliver at what point along the main axis we should start layout.
/// For the slivers that come after it, we compute the amount of space taken up so
/// far to be used as the [SliverPhysicalParentData.paintOffset] and the
/// [SliverConstraints.remainingPaintExtent] to be passed in as a constraint.
///
/// Finally, this sliver will also ensure that all child slivers are painted within
/// the total scroll extent of the group by adjusting the child's
/// [SliverPhysicalParentData.paintOffset] as necessary. This can happen for
/// slivers such as [SliverPersistentHeader] which, when pinned, positions itself
/// at the top of the [Viewport] regardless of the scroll offset.
class RenderSliverMainAxisGroup extends RenderSliver
    with ContainerRenderObjectMixin<RenderSliver, SliverPhysicalContainerParentData> {
  @override
  void setupParentData(RenderObject child) {
    if (child.parentData is! SliverPhysicalContainerParentData) {
      child.parentData = SliverPhysicalContainerParentData();
    }
  }

  @override
  double? childScrollOffset(RenderObject child) {
    assert(child.parent == this);
    final GrowthDirection growthDirection = constraints.growthDirection;
    switch (growthDirection) {
      case GrowthDirection.forward:
        double childScrollOffset = 0.0;
        RenderSliver? current = childBefore(child as RenderSliver);
        while (current != null) {
          // If the current child is not the first child, then we need to
          // add the scroll extent of the previous child to the current child's
          // scroll offset.
          if (childBefore(current) != null) {
            childScrollOffset +=
                childAfter(current)!.geometry!.scrollExtent + child.geometry!.scrollExtent;
          } else if (!(childAfter(child) != null && current.geometry!.hasVisualOverflow)) {
            childScrollOffset += current.geometry!.scrollExtent;
          }
          current = childBefore(current);
        }
        return childScrollOffset;
      case GrowthDirection.reverse:
        double childScrollOffset = 0.0;
        RenderSliver? current = childAfter(child as RenderSliver);
        while (current != null) {
          childScrollOffset -= current.geometry!.scrollExtent;
          current = childAfter(current);
        }
        return childScrollOffset;
    }
  }

  @override
  double childMainAxisPosition(RenderSliver child) {
    final Offset paintOffset = (child.parentData! as SliverPhysicalParentData).paintOffset;
    return switch (constraints.axis) {
      Axis.horizontal => paintOffset.dx,
      Axis.vertical => paintOffset.dy,
    };
  }

  @override
  double childCrossAxisPosition(RenderSliver child) => 0.0;

  @override
  void performLayout() {
    double offset = 0;
    double maxPaintExtent = 0;

    RenderSliver? child = firstChild;

    while (child != null) {
      final double beforeOffsetPaintExtent = calculatePaintOffset(
        constraints,
        from: 0.0,
        to: offset,
      );
      child.layout(
        constraints.copyWith(
          scrollOffset: math.max(0.0, constraints.scrollOffset - offset),
          cacheOrigin: math.min(0.0, constraints.cacheOrigin + offset),
          overlap: math.max(0.0, constraints.overlap - beforeOffsetPaintExtent),
          remainingPaintExtent: constraints.remainingPaintExtent - beforeOffsetPaintExtent,
          remainingCacheExtent:
              constraints.remainingCacheExtent -
              calculateCacheOffset(constraints, from: 0.0, to: offset),
          precedingScrollExtent: offset + constraints.precedingScrollExtent,
        ),
        parentUsesSize: true,
      );
      final SliverGeometry childLayoutGeometry = child.geometry!;
      final SliverPhysicalParentData childParentData =
          child.parentData! as SliverPhysicalParentData;
      childParentData.paintOffset = switch (constraints.axis) {
        Axis.vertical => Offset(0.0, beforeOffsetPaintExtent),
        Axis.horizontal => Offset(beforeOffsetPaintExtent, 0.0),
      };
      offset += childLayoutGeometry.scrollExtent;
      maxPaintExtent += child.geometry!.maxPaintExtent;
      child = childAfter(child);
      assert(() {
        if (child != null && maxPaintExtent.isInfinite) {
          throw FlutterError(
            'Unreachable sliver found, you may have a sliver following '
            'a sliver with an infinite extent. ',
          );
        }
        return true;
      }());
    }

    final double totalScrollExtent = offset;
    offset = 0.0;
    child = firstChild;
    // Second pass to correct out of bound paintOffsets.
    while (child != null) {
      final double beforeOffsetPaintExtent = calculatePaintOffset(
        constraints,
        from: 0.0,
        to: offset,
      );
      final SliverGeometry childLayoutGeometry = child.geometry!;
      final SliverPhysicalParentData childParentData =
          child.parentData! as SliverPhysicalParentData;
      final double remainingExtent = totalScrollExtent - constraints.scrollOffset;
      if (childLayoutGeometry.paintExtent > remainingExtent) {
        final double paintCorrection = childLayoutGeometry.paintExtent - remainingExtent;
        childParentData.paintOffset = switch (constraints.axis) {
          Axis.vertical => Offset(0.0, beforeOffsetPaintExtent - paintCorrection),
          Axis.horizontal => Offset(beforeOffsetPaintExtent - paintCorrection, 0.0),
        };
      }
      offset += child.geometry!.scrollExtent;
      child = childAfter(child);
    }

    final double paintExtent = calculatePaintOffset(
      constraints,
      from: math.min(constraints.scrollOffset, 0),
      to: totalScrollExtent,
    );
    final double cacheExtent = calculateCacheOffset(
      constraints,
      from: math.min(constraints.scrollOffset, 0),
      to: totalScrollExtent,
    );
    geometry = SliverGeometry(
      scrollExtent: totalScrollExtent,
      paintExtent: paintExtent,
      cacheExtent: cacheExtent,
      maxPaintExtent: maxPaintExtent,
      hasVisualOverflow:
          totalScrollExtent > constraints.remainingPaintExtent || constraints.scrollOffset > 0.0,
    );
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (firstChild == null) {
      return;
    }
    // offset is to the top-left corner, regardless of our axis direction.
    // originOffset gives us the delta from the real origin to the origin in the axis direction.
    final Offset mainAxisUnit, crossAxisUnit, originOffset;
    final bool addExtent;
    switch (applyGrowthDirectionToAxisDirection(
      constraints.axisDirection,
      constraints.growthDirection,
    )) {
      case AxisDirection.up:
        mainAxisUnit = const Offset(0.0, -1.0);
        crossAxisUnit = const Offset(1.0, 0.0);
        originOffset = offset + Offset(0.0, geometry!.paintExtent);
        addExtent = true;
      case AxisDirection.right:
        mainAxisUnit = const Offset(1.0, 0.0);
        crossAxisUnit = const Offset(0.0, 1.0);
        originOffset = offset;
        addExtent = false;
      case AxisDirection.down:
        mainAxisUnit = const Offset(0.0, 1.0);
        crossAxisUnit = const Offset(1.0, 0.0);
        originOffset = offset;
        addExtent = false;
      case AxisDirection.left:
        mainAxisUnit = const Offset(-1.0, 0.0);
        crossAxisUnit = const Offset(0.0, 1.0);
        originOffset = offset + Offset(geometry!.paintExtent, 0.0);
        addExtent = true;
    }

    RenderSliver? child = lastChild;
    while (child != null) {
      final double mainAxisDelta = childMainAxisPosition(child);
      final double crossAxisDelta = childCrossAxisPosition(child);
      Offset childOffset = Offset(
        originOffset.dx + mainAxisUnit.dx * mainAxisDelta + crossAxisUnit.dx * crossAxisDelta,
        originOffset.dy + mainAxisUnit.dy * mainAxisDelta + crossAxisUnit.dy * crossAxisDelta,
      );
      if (addExtent) {
        childOffset += mainAxisUnit * child.geometry!.paintExtent;
      }

      if (child.geometry!.visible) {
        context.paintChild(child, childOffset);
      }
      child = childBefore(child);
    }
  }

  @override
  void applyPaintTransform(RenderSliver child, Matrix4 transform) {
    final SliverPhysicalParentData childParentData = child.parentData! as SliverPhysicalParentData;
    childParentData.applyPaintTransform(transform);
  }

  @override
  bool hitTestChildren(
    SliverHitTestResult result, {
    required double mainAxisPosition,
    required double crossAxisPosition,
  }) {
    RenderSliver? child = firstChild;
    while (child != null) {
      final bool isHit = result.addWithAxisOffset(
        mainAxisPosition: mainAxisPosition,
        crossAxisPosition: crossAxisPosition,
        paintOffset: null,
        mainAxisOffset: childMainAxisPosition(child),
        crossAxisOffset: childCrossAxisPosition(child),
        hitTest: child.hitTest,
      );
      if (isHit) {
        return true;
      }
      child = childAfter(child);
    }
    return false;
  }

  @override
  void visitChildrenForSemantics(RenderObjectVisitor visitor) {
    RenderSliver? child = firstChild;
    while (child != null) {
      if (child.geometry!.visible) {
        visitor(child);
      }
      child = childAfter(child);
    }
  }
}
