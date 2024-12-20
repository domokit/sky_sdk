// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'proxy_box.dart';
library;

import 'dart:math' as math;

import 'package:flutter/foundation.dart';
import 'package:vector_math/vector_math_64.dart';

import 'box.dart';
import 'layer.dart';
import 'object.dart';

const double _kQuarterTurnsInRadians = math.pi / 2.0;

/// Rotates its child by a integral number of quarter turns.
///
/// Unlike [RenderTransform], which applies a transform just prior to painting,
/// this object applies its rotation prior to layout, which means the entire
/// rotated box consumes only as much space as required by the rotated child.
class RenderRotatedBox extends RenderBox with RenderObjectWithChildMixin<RenderBox> {
  /// Creates a rotated render box.
  RenderRotatedBox({required int quarterTurns, RenderBox? child}) : _quarterTurns = quarterTurns {
    this.child = child;
  }

  /// The number of clockwise quarter turns the child should be rotated.
  int get quarterTurns => _quarterTurns;
  int _quarterTurns;
  set quarterTurns(int value) {
    if (_quarterTurns == value) {
      return;
    }
    _quarterTurns = value;
    markNeedsLayout();
  }

  bool get _isVertical => quarterTurns.isOdd;

  @override
  double computeMinIntrinsicWidth(double height) {
    if (child == null) {
      return 0.0;
    }
    return _isVertical ? child!.getMinIntrinsicHeight(height) : child!.getMinIntrinsicWidth(height);
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    if (child == null) {
      return 0.0;
    }
    return _isVertical ? child!.getMaxIntrinsicHeight(height) : child!.getMaxIntrinsicWidth(height);
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    if (child == null) {
      return 0.0;
    }
    return _isVertical ? child!.getMinIntrinsicWidth(width) : child!.getMinIntrinsicHeight(width);
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    if (child == null) {
      return 0.0;
    }
    return _isVertical ? child!.getMaxIntrinsicWidth(width) : child!.getMaxIntrinsicHeight(width);
  }

  Matrix4? _paintTransform;

  @override
  @protected
  Size computeDryLayout(covariant BoxConstraints constraints) {
    if (child == null) {
      return constraints.smallest;
    }
    final Size childSize = child!.getDryLayout(_isVertical ? constraints.flipped : constraints);
    return _isVertical ? Size(childSize.height, childSize.width) : childSize;
  }

  @override
  void performLayout() {
    _paintTransform = null;
    if (child != null) {
      child!.layout(_isVertical ? constraints.flipped : constraints, parentUsesSize: true);
      size = _isVertical ? Size(child!.size.height, child!.size.width) : child!.size;
      _paintTransform =
          Matrix4.identity()
            ..translate(size.width / 2.0, size.height / 2.0)
            ..rotateZ(_kQuarterTurnsInRadians * (quarterTurns % 4))
            ..translate(-child!.size.width / 2.0, -child!.size.height / 2.0);
    } else {
      size = constraints.smallest;
    }
  }

  @override
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) {
    assert(_paintTransform != null || debugNeedsLayout || child == null);
    if (child == null || _paintTransform == null) {
      return false;
    }
    return result.addWithPaintTransform(
      transform: _paintTransform,
      position: position,
      hitTest: (BoxHitTestResult result, Offset position) {
        return child!.hitTest(result, position: position);
      },
    );
  }

  void _paintChild(PaintingContext context, Offset offset) {
    context.paintChild(child!, offset);
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (child != null) {
      _transformLayer.layer = context.pushTransform(
        needsCompositing,
        offset,
        _paintTransform!,
        _paintChild,
        oldLayer: _transformLayer.layer,
      );
    } else {
      _transformLayer.layer = null;
    }
  }

  final LayerHandle<TransformLayer> _transformLayer = LayerHandle<TransformLayer>();

  @override
  void dispose() {
    _transformLayer.layer = null;
    super.dispose();
  }

  @override
  void applyPaintTransform(RenderBox child, Matrix4 transform) {
    if (_paintTransform != null) {
      transform.multiply(_paintTransform!);
    }
    super.applyPaintTransform(child, transform);
  }
}
