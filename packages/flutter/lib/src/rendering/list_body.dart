// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:flutter/foundation.dart';

import 'box.dart';
import 'object.dart';

/// Parent data for use with [RenderListBody].
class ListBodyParentData extends ContainerBoxParentData<RenderBox> {}

typedef _ChildSizingFunction = double Function(RenderBox child);

/// Displays its children sequentially along a given axis, forcing them to the
/// dimensions of the parent in the other axis.
///
/// This layout algorithm arranges its children linearly along the main axis
/// (either horizontally or vertically). In the cross axis, children are
/// stretched to match the box's cross-axis extent. In the main axis, children
/// are given unlimited space and the box expands its main axis to contain all
/// its children. Because [RenderListBody] boxes expand in the main axis, they
/// must be given unlimited space in the main axis, typically by being contained
/// in a viewport with a scrolling direction that matches the box's main axis.
class RenderListBody extends RenderBox
    with
        ContainerRenderObjectMixin<RenderBox, ListBodyParentData>,
        RenderBoxContainerDefaultsMixin<RenderBox, ListBodyParentData> {
  /// Creates a render object that arranges its children sequentially along a
  /// given axis.
  ///
  /// By default, children are arranged along the vertical axis.
  RenderListBody({List<RenderBox>? children, AxisDirection axisDirection = AxisDirection.down})
    : _axisDirection = axisDirection {
    addAll(children);
  }

  @override
  void setupParentData(RenderBox child) {
    if (child.parentData is! ListBodyParentData) {
      child.parentData = ListBodyParentData();
    }
  }

  /// The direction in which the children are laid out.
  ///
  /// For example, if the [axisDirection] is [AxisDirection.down], each child
  /// will be laid out below the next, vertically.
  AxisDirection get axisDirection => _axisDirection;
  AxisDirection _axisDirection;
  set axisDirection(AxisDirection value) {
    if (_axisDirection == value) {
      return;
    }
    _axisDirection = value;
    markNeedsLayout();
  }

  /// The axis (horizontal or vertical) corresponding to the current
  /// [axisDirection].
  Axis get mainAxis => axisDirectionToAxis(axisDirection);

  @override
  double? computeDryBaseline(covariant BoxConstraints constraints, TextBaseline baseline) {
    assert(_debugCheckConstraints(constraints));
    RenderBox? child;
    final RenderBox? Function(RenderBox) nextChild;
    switch (axisDirection) {
      case AxisDirection.right:
      case AxisDirection.left:
        final BoxConstraints childConstraints = BoxConstraints.tightFor(
          height: constraints.maxHeight,
        );
        BaselineOffset baselineOffset = BaselineOffset.noBaseline;
        for (child = firstChild; child != null; child = childAfter(child)) {
          baselineOffset = baselineOffset.minOf(
            BaselineOffset(child.getDryBaseline(childConstraints, baseline)),
          );
        }
        return baselineOffset.offset;
      case AxisDirection.up:
        child = lastChild;
        nextChild = childBefore;
      case AxisDirection.down:
        child = firstChild;
        nextChild = childAfter;
    }
    final BoxConstraints childConstraints = BoxConstraints.tightFor(width: constraints.maxWidth);
    double mainAxisExtent = 0.0;
    for (; child != null; child = nextChild(child)) {
      final double? childBaseline = child.getDryBaseline(childConstraints, baseline);
      if (childBaseline != null) {
        return childBaseline + mainAxisExtent;
      }
      mainAxisExtent += child.getDryLayout(childConstraints).height;
    }
    return null;
  }

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    assert(_debugCheckConstraints(constraints));
    double mainAxisExtent = 0.0;
    RenderBox? child = firstChild;
    switch (axisDirection) {
      case AxisDirection.right:
      case AxisDirection.left:
        final BoxConstraints innerConstraints = BoxConstraints.tightFor(
          height: constraints.maxHeight,
        );
        while (child != null) {
          final Size childSize = child.getDryLayout(innerConstraints);
          mainAxisExtent += childSize.width;
          child = childAfter(child);
        }
        return constraints.constrain(Size(mainAxisExtent, constraints.maxHeight));
      case AxisDirection.up:
      case AxisDirection.down:
        final BoxConstraints innerConstraints = BoxConstraints.tightFor(
          width: constraints.maxWidth,
        );
        while (child != null) {
          final Size childSize = child.getDryLayout(innerConstraints);
          mainAxisExtent += childSize.height;
          child = childAfter(child);
        }
        return constraints.constrain(Size(constraints.maxWidth, mainAxisExtent));
    }
  }

  bool _debugCheckConstraints(BoxConstraints constraints) {
    assert(() {
      switch (mainAxis) {
        case Axis.horizontal:
          if (!constraints.hasBoundedWidth) {
            return true;
          }
        case Axis.vertical:
          if (!constraints.hasBoundedHeight) {
            return true;
          }
      }
      throw FlutterError.fromParts(<DiagnosticsNode>[
        ErrorSummary('RenderListBody must have unlimited space along its main axis.'),
        ErrorDescription(
          'RenderListBody does not clip or resize its children, so it must be '
          'placed in a parent that does not constrain the main '
          'axis.',
        ),
        ErrorHint(
          'You probably want to put the RenderListBody inside a '
          'RenderViewport with a matching main axis.',
        ),
      ]);
    }());
    assert(() {
      switch (mainAxis) {
        case Axis.horizontal:
          if (constraints.hasBoundedHeight) {
            return true;
          }
        case Axis.vertical:
          if (constraints.hasBoundedWidth) {
            return true;
          }
      }
      // TODO(ianh): Detect if we're actually nested blocks and say something
      // more specific to the exact situation in that case, and don't mention
      // nesting blocks in the negative case.
      throw FlutterError.fromParts(<DiagnosticsNode>[
        ErrorSummary('RenderListBody must have a bounded constraint for its cross axis.'),
        ErrorDescription(
          "RenderListBody forces its children to expand to fit the RenderListBody's container, "
          'so it must be placed in a parent that constrains the cross '
          'axis to a finite dimension.',
        ),
        // TODO(jacobr): this hint is a great candidate to promote to being an
        // automated quick fix in the future.
        ErrorHint(
          'If you are attempting to nest a RenderListBody with '
          'one direction inside one of another direction, you will want to '
          'wrap the inner one inside a box that fixes the dimension in that direction, '
          'for example, a RenderIntrinsicWidth or RenderIntrinsicHeight object. '
          'This is relatively expensive, however.', // (that's why we don't do it automatically)
        ),
      ]);
    }());
    return true;
  }

  @override
  void performLayout() {
    final BoxConstraints constraints = this.constraints;
    assert(_debugCheckConstraints(constraints));
    double mainAxisExtent = 0.0;
    RenderBox? child = firstChild;
    switch (axisDirection) {
      case AxisDirection.right:
        final BoxConstraints innerConstraints = BoxConstraints.tightFor(
          height: constraints.maxHeight,
        );
        while (child != null) {
          child.layout(innerConstraints, parentUsesSize: true);
          final ListBodyParentData childParentData = child.parentData! as ListBodyParentData;
          childParentData.offset = Offset(mainAxisExtent, 0.0);
          mainAxisExtent += child.size.width;
          assert(child.parentData == childParentData);
          child = childParentData.nextSibling;
        }
        size = constraints.constrain(Size(mainAxisExtent, constraints.maxHeight));
      case AxisDirection.left:
        final BoxConstraints innerConstraints = BoxConstraints.tightFor(
          height: constraints.maxHeight,
        );
        while (child != null) {
          child.layout(innerConstraints, parentUsesSize: true);
          final ListBodyParentData childParentData = child.parentData! as ListBodyParentData;
          mainAxisExtent += child.size.width;
          assert(child.parentData == childParentData);
          child = childParentData.nextSibling;
        }
        double position = 0.0;
        child = firstChild;
        while (child != null) {
          final ListBodyParentData childParentData = child.parentData! as ListBodyParentData;
          position += child.size.width;
          childParentData.offset = Offset(mainAxisExtent - position, 0.0);
          assert(child.parentData == childParentData);
          child = childParentData.nextSibling;
        }
        size = constraints.constrain(Size(mainAxisExtent, constraints.maxHeight));
      case AxisDirection.down:
        final BoxConstraints innerConstraints = BoxConstraints.tightFor(
          width: constraints.maxWidth,
        );
        while (child != null) {
          child.layout(innerConstraints, parentUsesSize: true);
          final ListBodyParentData childParentData = child.parentData! as ListBodyParentData;
          childParentData.offset = Offset(0.0, mainAxisExtent);
          mainAxisExtent += child.size.height;
          assert(child.parentData == childParentData);
          child = childParentData.nextSibling;
        }
        size = constraints.constrain(Size(constraints.maxWidth, mainAxisExtent));
      case AxisDirection.up:
        final BoxConstraints innerConstraints = BoxConstraints.tightFor(
          width: constraints.maxWidth,
        );
        while (child != null) {
          child.layout(innerConstraints, parentUsesSize: true);
          final ListBodyParentData childParentData = child.parentData! as ListBodyParentData;
          mainAxisExtent += child.size.height;
          assert(child.parentData == childParentData);
          child = childParentData.nextSibling;
        }
        double position = 0.0;
        child = firstChild;
        while (child != null) {
          final ListBodyParentData childParentData = child.parentData! as ListBodyParentData;
          position += child.size.height;
          childParentData.offset = Offset(0.0, mainAxisExtent - position);
          assert(child.parentData == childParentData);
          child = childParentData.nextSibling;
        }
        size = constraints.constrain(Size(constraints.maxWidth, mainAxisExtent));
    }
    assert(size.isFinite);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(EnumProperty<AxisDirection>('axisDirection', axisDirection));
  }

  double _getIntrinsicCrossAxis(_ChildSizingFunction childSize) {
    double extent = 0.0;
    RenderBox? child = firstChild;
    while (child != null) {
      extent = math.max(extent, childSize(child));
      final ListBodyParentData childParentData = child.parentData! as ListBodyParentData;
      child = childParentData.nextSibling;
    }
    return extent;
  }

  double _getIntrinsicMainAxis(_ChildSizingFunction childSize) {
    double extent = 0.0;
    RenderBox? child = firstChild;
    while (child != null) {
      extent += childSize(child);
      final ListBodyParentData childParentData = child.parentData! as ListBodyParentData;
      child = childParentData.nextSibling;
    }
    return extent;
  }

  @override
  double computeMinIntrinsicWidth(double height) {
    return switch (mainAxis) {
      Axis.horizontal => _getIntrinsicMainAxis(
        (RenderBox child) => child.getMinIntrinsicWidth(height),
      ),
      Axis.vertical => _getIntrinsicCrossAxis(
        (RenderBox child) => child.getMinIntrinsicWidth(height),
      ),
    };
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    return switch (mainAxis) {
      Axis.horizontal => _getIntrinsicMainAxis(
        (RenderBox child) => child.getMaxIntrinsicWidth(height),
      ),
      Axis.vertical => _getIntrinsicCrossAxis(
        (RenderBox child) => child.getMaxIntrinsicWidth(height),
      ),
    };
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    return switch (mainAxis) {
      Axis.horizontal => _getIntrinsicMainAxis(
        (RenderBox child) => child.getMinIntrinsicHeight(width),
      ),
      Axis.vertical => _getIntrinsicCrossAxis(
        (RenderBox child) => child.getMinIntrinsicHeight(width),
      ),
    };
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    return switch (mainAxis) {
      Axis.horizontal => _getIntrinsicMainAxis(
        (RenderBox child) => child.getMaxIntrinsicHeight(width),
      ),
      Axis.vertical => _getIntrinsicCrossAxis(
        (RenderBox child) => child.getMaxIntrinsicHeight(width),
      ),
    };
  }

  @override
  double? computeDistanceToActualBaseline(TextBaseline baseline) {
    return defaultComputeDistanceToFirstActualBaseline(baseline);
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    defaultPaint(context, offset);
  }

  @override
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) {
    return defaultHitTestChildren(result, position: position);
  }
}
