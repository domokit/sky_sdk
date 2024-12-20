// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';

class RenderSolidColorBox extends RenderDecoratedBox {
  RenderSolidColorBox(this.backgroundColor, {this.desiredSize = Size.infinite})
    : super(decoration: BoxDecoration(color: backgroundColor));

  final Size desiredSize;
  final Color backgroundColor;

  @override
  double computeMinIntrinsicWidth(double height) {
    return desiredSize.width == double.infinity ? 0.0 : desiredSize.width;
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    return desiredSize.width == double.infinity ? 0.0 : desiredSize.width;
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    return desiredSize.height == double.infinity ? 0.0 : desiredSize.height;
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    return desiredSize.height == double.infinity ? 0.0 : desiredSize.height;
  }

  @override
  void performLayout() {
    size = constraints.constrain(desiredSize);
  }

  @override
  void handleEvent(PointerEvent event, BoxHitTestEntry entry) {
    if (event is PointerDownEvent) {
      decoration = const BoxDecoration(color: Color(0xFFFF0000));
    } else if (event is PointerUpEvent) {
      decoration = BoxDecoration(color: backgroundColor);
    }
  }
}
