// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

class BorderTabIndicator extends Decoration {
  const BorderTabIndicator({required this.indicatorHeight, required this.textScaleFactor})
    : super();

  final double indicatorHeight;
  final double textScaleFactor;

  @override
  BorderPainter createBoxPainter([VoidCallback? onChanged]) {
    return BorderPainter(this, indicatorHeight, textScaleFactor, onChanged);
  }
}

class BorderPainter extends BoxPainter {
  BorderPainter(
    this.decoration,
    this.indicatorHeight,
    this.textScaleFactor,
    VoidCallback? onChanged,
  ) : assert(indicatorHeight >= 0),
      super(onChanged);

  final BorderTabIndicator decoration;
  final double indicatorHeight;
  final double textScaleFactor;

  @override
  void paint(Canvas canvas, Offset offset, ImageConfiguration configuration) {
    assert(configuration.size != null);
    final double horizontalInset = 16 - 4 * textScaleFactor;
    final Rect rect =
        Offset(
          offset.dx + horizontalInset,
          (configuration.size!.height / 2) - indicatorHeight / 2 - 1,
        ) &
        Size(configuration.size!.width - 2 * horizontalInset, indicatorHeight);
    final Paint paint = Paint();
    paint.color = Colors.white;
    paint.style = PaintingStyle.stroke;
    paint.strokeWidth = 2;
    canvas.drawRRect(RRect.fromRectAndRadius(rect, const Radius.circular(56)), paint);
  }
}
