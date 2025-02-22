// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
library;

import 'dart:math' as math;
import 'dart:ui';

import 'package:flutter/foundation.dart';
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
    double scrollOffset = 0;
    double layoutOffset = 0;
    double maxPaintExtent = 0;
    double paintOffset = constraints.overlap;

    RenderSliver? child = firstChild;
    while (child != null) {
      final double beforeOffsetPaintExtent = calculatePaintOffset(
        constraints,
        from: 0.0,
        to: scrollOffset,
      );
      child.layout(
        constraints.copyWith(
          scrollOffset: math.max(0.0, constraints.scrollOffset - scrollOffset),
          cacheOrigin: math.min(0.0, constraints.cacheOrigin + scrollOffset),
          overlap: math.max(0.0, _fixPrecisionError(paintOffset - beforeOffsetPaintExtent)),
          remainingPaintExtent: _fixPrecisionError(
            constraints.remainingPaintExtent - beforeOffsetPaintExtent,
          ),
          remainingCacheExtent: _fixPrecisionError(
            constraints.remainingCacheExtent -
                calculateCacheOffset(constraints, from: 0.0, to: scrollOffset),
          ),
          precedingScrollExtent: scrollOffset + constraints.precedingScrollExtent,
        ),
        parentUsesSize: true,
      );
      final SliverGeometry childLayoutGeometry = child.geometry!;
      final double childPaintOffset = layoutOffset + childLayoutGeometry.paintOrigin;
      final SliverPhysicalParentData childParentData =
          child.parentData! as SliverPhysicalParentData;
      childParentData.paintOffset = switch (constraints.axis) {
        Axis.vertical => Offset(0.0, childPaintOffset),
        Axis.horizontal => Offset(childPaintOffset, 0.0),
      };
      scrollOffset += childLayoutGeometry.scrollExtent;
      layoutOffset += childLayoutGeometry.layoutExtent;
      maxPaintExtent += childLayoutGeometry.maxPaintExtent;
      paintOffset = math.max(childPaintOffset + childLayoutGeometry.paintExtent, paintOffset);
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

    final double remainingExtent = math.max(0, scrollOffset - constraints.scrollOffset);
    // If the children's paint extent exceeds the remaining scroll extent of the `RenderSliverMainAxisGroup`,
    // they need to be corrected.
    if (paintOffset > remainingExtent) {
      final double paintCorrection = paintOffset - remainingExtent;
      paintOffset = remainingExtent;
      child = firstChild;
      while (child != null) {
        final SliverGeometry childLayoutGeometry = child.geometry!;
        if (childLayoutGeometry.paintExtent > 0) {
          final SliverPhysicalParentData childParentData =
              child.parentData! as SliverPhysicalParentData;
          childParentData.paintOffset = switch (constraints.axis) {
            Axis.vertical => Offset(0.0, childParentData.paintOffset.dy - paintCorrection),
            Axis.horizontal => Offset(childParentData.paintOffset.dx - paintCorrection, 0.0),
          };
        }
        child = childAfter(child);
      }
    }

    final double cacheExtent = calculateCacheOffset(
      constraints,
      from: math.min(constraints.scrollOffset, 0),
      to: scrollOffset,
    );
    final double paintExtent = clampDouble(paintOffset, 0, constraints.remainingPaintExtent);

    geometry = SliverGeometry(
      scrollExtent: scrollOffset,
      paintExtent: paintExtent,
      cacheExtent: cacheExtent,
      maxPaintExtent: maxPaintExtent,
      hasVisualOverflow:
          scrollOffset > constraints.remainingPaintExtent || constraints.scrollOffset > 0.0,
    );

    // Update the children's paintOffset based on the direction again, which
    // must be done after obtaining the `paintExtent`.
    child = firstChild;
    while (child != null) {
      final SliverPhysicalParentData childParentData =
          child.parentData! as SliverPhysicalParentData;
      childParentData.paintOffset = switch (constraints.axisDirection) {
        AxisDirection.up => Offset(
          0.0,
          paintExtent - childParentData.paintOffset.dy - child.geometry!.paintExtent,
        ),
        AxisDirection.left => Offset(
          paintExtent - childParentData.paintOffset.dx - child.geometry!.paintExtent,
          0.0,
        ),
        AxisDirection.right || AxisDirection.down => childParentData.paintOffset,
      };
      child = childAfter(child);
    }
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    RenderSliver? child = lastChild;
    while (child != null) {
      if (child.geometry!.visible) {
        final SliverPhysicalParentData childParentData =
            child.parentData! as SliverPhysicalParentData;
        context.paintChild(child, offset + childParentData.paintOffset);
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

  static double _fixPrecisionError(double number) {
    return number.abs() < precisionErrorTolerance ? 0.0 : number;
  }
}
