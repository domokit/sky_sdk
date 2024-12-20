// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
library;

import 'dart:math' as math;

import 'package:flutter/foundation.dart' show clampDouble;
import 'basic_types.dart';

/// Position a child box within a container box, either above or below a target
/// point.
///
/// The container's size is described by `size`.
///
/// The target point is specified by `target`, as an offset from the top left of
/// the container.
///
/// The child box's size is given by `childSize`.
///
/// The return value is the suggested distance from the top left of the
/// container box to the top left of the child box.
///
/// The suggested position will be above the target point if `preferBelow` is
/// false, and below the target point if it is true, unless it wouldn't fit on
/// the preferred side but would fit on the other side.
///
/// The suggested position will place the nearest side of the child to the
/// target point `verticalOffset` from the target point (even if it cannot fit
/// given that constraint).
///
/// The suggested position will be at least `margin` away from the edge of the
/// container. If possible, the child will be positioned so that its center is
/// aligned with the target point. If the child cannot fit horizontally within
/// the container given the margin, then the child will be centered in the
/// container.
///
/// Used by [Tooltip] to position a tooltip relative to its parent.
Offset positionDependentBox({
  required Size size,
  required Size childSize,
  required Offset target,
  required bool preferBelow,
  double verticalOffset = 0.0,
  double margin = 10.0,
}) {
  // VERTICAL DIRECTION
  final bool fitsBelow = target.dy + verticalOffset + childSize.height <= size.height - margin;
  final bool fitsAbove = target.dy - verticalOffset - childSize.height >= margin;
  final bool tooltipBelow = fitsAbove == fitsBelow ? preferBelow : fitsBelow;
  final double y;
  if (tooltipBelow) {
    y = math.min(target.dy + verticalOffset, size.height - margin);
  } else {
    y = math.max(target.dy - verticalOffset - childSize.height, margin);
  }
  // HORIZONTAL DIRECTION
  final double flexibleSpace = size.width - childSize.width;
  final double x =
      flexibleSpace <= 2 * margin
          // If there's not enough horizontal space for margin + child, center the
          // child.
          ? flexibleSpace / 2.0
          : clampDouble(target.dx - childSize.width / 2, margin, flexibleSpace - margin);
  return Offset(x, y);
}
