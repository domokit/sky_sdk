// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import 'colors.dart';

const List<Offset> _vertices = <Offset>[
  Offset(0, -14),
  Offset(-17, 14),
  Offset(17, 14),
  Offset(0, -14),
  Offset(0, -7.37),
  Offset(10.855, 10.48),
  Offset(-10.855, 10.48),
  Offset(0, -7.37),
];

class TriangleCategoryIndicator extends CustomPainter {
  const TriangleCategoryIndicator(this.triangleWidth, this.triangleHeight);

  final double triangleWidth;
  final double triangleHeight;

  @override
  void paint(Canvas canvas, Size size) {
    final Path myPath =
        Path()..addPolygon(
          List<Offset>.from(
            _vertices.map<Offset>((Offset vertex) {
              return Offset(size.width, size.height) / 2 +
                  Offset(vertex.dx * triangleWidth / 34, vertex.dy * triangleHeight / 28);
            }),
          ),
          true,
        );
    final Paint myPaint = Paint()..color = shrinePink400;
    canvas.drawPath(myPath, myPaint);
  }

  @override
  bool shouldRepaint(TriangleCategoryIndicator oldDelegate) => false;

  @override
  bool shouldRebuildSemantics(TriangleCategoryIndicator oldDelegate) => false;
}
