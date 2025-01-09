// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;
import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' hide window;

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

/// Test winding and convexity of a path.
void testMain() {
  group('Convexity', () {
    test('Empty path should be convex', () {
      final SurfacePath path = SurfacePath();
      expect(path.isConvex, isTrue);
    });

    test('Circle should be convex', () {
      final SurfacePath path = SurfacePath();
      path.addOval(const Rect.fromLTRB(0, 0, 20, 20));
      expect(path.isConvex, isTrue);
      // 2nd circle.
      path.addOval(const Rect.fromLTRB(0, 0, 20, 20));
      expect(path.isConvex, isFalse);
    });

    test('addRect should be convex', () {
      SurfacePath path = SurfacePath();
      path.addRect(const Rect.fromLTRB(0, 0, 20, 20));
      expect(path.isConvex, isTrue);

      path = SurfacePath();
      path.addRectWithDirection(const Rect.fromLTRB(0, 0, 20, 20), SPathDirection.kCW, 0);
      expect(path.isConvex, isTrue);

      path = SurfacePath();
      path.addRectWithDirection(const Rect.fromLTRB(0, 0, 20, 20), SPathDirection.kCCW, 0);
      expect(path.isConvex, isTrue);
    });

    test('Quad should be convex', () {
      final SurfacePath path = SurfacePath();
      path.quadraticBezierTo(100, 100, 50, 50);
      expect(path.isConvex, isTrue);
    });

    test('moveto/lineto convexity', () {
      final List<LineTestCase> testCases = <LineTestCase>[
        LineTestCase('', SPathConvexityType.kConvex),
        LineTestCase('0 0', SPathConvexityType.kConvex),
        LineTestCase('0 0 10 10', SPathConvexityType.kConvex),
        LineTestCase('0 0 10 10 20 20 0 0 10 10', SPathConvexityType.kConcave),
        LineTestCase('0 0 10 10 10 20', SPathConvexityType.kConvex),
        LineTestCase('0 0 10 10 10 0', SPathConvexityType.kConvex),
        LineTestCase('0 0 10 10 10 0 0 10', SPathConvexityType.kConcave),
        LineTestCase('0 0 10 0 0 10 -10 -10', SPathConvexityType.kConcave),
      ];

      for (final LineTestCase testCase in testCases) {
        final SurfacePath path = SurfacePath();
        setFromString(path, testCase.pathContent);
        expect(path.convexityType, testCase.convexity);
      }
    });

    test('Convexity of path with infinite points should return unknown', () {
      const List<Offset> nonFinitePts = <Offset>[
        Offset(double.infinity, 0),
        Offset(0, double.infinity),
        Offset.infinite,
        Offset(double.negativeInfinity, 0),
        Offset(0, double.negativeInfinity),
        Offset(double.negativeInfinity, double.negativeInfinity),
        Offset(double.negativeInfinity, double.infinity),
        Offset(double.infinity, double.negativeInfinity),
        Offset(double.nan, 0),
        Offset(0, double.nan),
        Offset(double.nan, double.nan),
      ];
      final int nonFinitePointsCount = nonFinitePts.length;

      const List<Offset> axisAlignedPts = <Offset>[
        Offset(kScalarMax, 0),
        Offset(0, kScalarMax),
        Offset(kScalarMin, 0),
        Offset(0, kScalarMin),
      ];
      final int axisAlignedPointsCount = axisAlignedPts.length;

      final SurfacePath path = SurfacePath();

      for (int index = 0; index < (13 * nonFinitePointsCount * axisAlignedPointsCount); index++) {
        final int i = index % nonFinitePointsCount;
        final int f = index % axisAlignedPointsCount;
        final int g = (f + 1) % axisAlignedPointsCount;
        path.reset();
        switch (index % 13) {
          case 0:
            path.lineTo(nonFinitePts[i].dx, nonFinitePts[i].dy);
          case 1:
            path.quadraticBezierTo(
              nonFinitePts[i].dx,
              nonFinitePts[i].dy,
              nonFinitePts[i].dx,
              nonFinitePts[i].dy,
            );
          case 2:
            path.quadraticBezierTo(
              nonFinitePts[i].dx,
              nonFinitePts[i].dy,
              axisAlignedPts[f].dx,
              axisAlignedPts[f].dy,
            );
          case 3:
            path.quadraticBezierTo(
              axisAlignedPts[f].dx,
              axisAlignedPts[f].dy,
              nonFinitePts[i].dx,
              nonFinitePts[i].dy,
            );
          case 4:
            path.cubicTo(
              nonFinitePts[i].dx,
              nonFinitePts[i].dy,
              axisAlignedPts[f].dx,
              axisAlignedPts[f].dy,
              axisAlignedPts[f].dx,
              axisAlignedPts[f].dy,
            );
          case 5:
            path.cubicTo(
              axisAlignedPts[f].dx,
              axisAlignedPts[f].dy,
              nonFinitePts[i].dx,
              nonFinitePts[i].dy,
              axisAlignedPts[f].dx,
              axisAlignedPts[f].dy,
            );
          case 6:
            path.cubicTo(
              axisAlignedPts[f].dx,
              axisAlignedPts[f].dy,
              axisAlignedPts[f].dx,
              axisAlignedPts[f].dy,
              nonFinitePts[i].dx,
              nonFinitePts[i].dy,
            );
          case 7:
            path.cubicTo(
              nonFinitePts[i].dx,
              nonFinitePts[i].dy,
              nonFinitePts[i].dx,
              nonFinitePts[i].dy,
              axisAlignedPts[f].dx,
              axisAlignedPts[f].dy,
            );
          case 8:
            path.cubicTo(
              nonFinitePts[i].dx,
              nonFinitePts[i].dy,
              axisAlignedPts[f].dx,
              axisAlignedPts[f].dy,
              nonFinitePts[i].dx,
              nonFinitePts[i].dy,
            );
          case 9:
            path.cubicTo(
              axisAlignedPts[f].dx,
              axisAlignedPts[f].dy,
              nonFinitePts[i].dx,
              nonFinitePts[i].dy,
              nonFinitePts[i].dx,
              nonFinitePts[i].dy,
            );
          case 10:
            path.cubicTo(
              nonFinitePts[i].dx,
              nonFinitePts[i].dy,
              nonFinitePts[i].dx,
              nonFinitePts[i].dy,
              nonFinitePts[i].dx,
              nonFinitePts[i].dy,
            );
          case 11:
            path.cubicTo(
              nonFinitePts[i].dx,
              nonFinitePts[i].dy,
              axisAlignedPts[f].dx,
              axisAlignedPts[f].dy,
              axisAlignedPts[g].dx,
              axisAlignedPts[g].dy,
            );
          case 12:
            path.moveTo(nonFinitePts[i].dx, nonFinitePts[i].dy);
        }
        expect(path.convexityType, SPathConvexityType.kUnknown);
      }

      for (int index = 0; index < (11 * axisAlignedPointsCount); ++index) {
        final int f = index % axisAlignedPointsCount;
        final int g = (f + 1) % axisAlignedPointsCount;
        path.reset();
        final int curveSelect = index % 11;
        switch (curveSelect) {
          case 0:
            path.moveTo(axisAlignedPts[f].dx, axisAlignedPts[f].dy);
          case 1:
            path.lineTo(axisAlignedPts[f].dx, axisAlignedPts[f].dy);
          case 2:
            path.quadraticBezierTo(
              axisAlignedPts[f].dx,
              axisAlignedPts[f].dy,
              axisAlignedPts[f].dx,
              axisAlignedPts[f].dy,
            );
          case 3:
            path.quadraticBezierTo(
              axisAlignedPts[f].dx,
              axisAlignedPts[f].dy,
              axisAlignedPts[g].dx,
              axisAlignedPts[g].dy,
            );
          case 4:
            path.quadraticBezierTo(
              axisAlignedPts[g].dx,
              axisAlignedPts[g].dy,
              axisAlignedPts[f].dx,
              axisAlignedPts[f].dy,
            );
          case 5:
            path.cubicTo(
              axisAlignedPts[f].dx,
              axisAlignedPts[f].dy,
              axisAlignedPts[f].dx,
              axisAlignedPts[f].dy,
              axisAlignedPts[f].dx,
              axisAlignedPts[f].dy,
            );
          case 6:
            path.cubicTo(
              axisAlignedPts[f].dx,
              axisAlignedPts[f].dy,
              axisAlignedPts[f].dx,
              axisAlignedPts[f].dy,
              axisAlignedPts[g].dx,
              axisAlignedPts[g].dy,
            );
          case 7:
            path.cubicTo(
              axisAlignedPts[f].dx,
              axisAlignedPts[f].dy,
              axisAlignedPts[g].dx,
              axisAlignedPts[g].dy,
              axisAlignedPts[f].dx,
              axisAlignedPts[f].dy,
            );
          case 8:
            path.cubicTo(
              axisAlignedPts[f].dx,
              axisAlignedPts[f].dy,
              axisAlignedPts[g].dx,
              axisAlignedPts[g].dy,
              axisAlignedPts[g].dx,
              axisAlignedPts[g].dy,
            );
          case 9:
            path.cubicTo(
              axisAlignedPts[g].dx,
              axisAlignedPts[g].dy,
              axisAlignedPts[f].dx,
              axisAlignedPts[f].dy,
              axisAlignedPts[f].dx,
              axisAlignedPts[f].dy,
            );
          case 10:
            path.cubicTo(
              axisAlignedPts[g].dx,
              axisAlignedPts[g].dy,
              axisAlignedPts[f].dx,
              axisAlignedPts[f].dy,
              axisAlignedPts[g].dx,
              axisAlignedPts[g].dy,
            );
        }
        if (curveSelect != 7 && curveSelect != 10) {
          final int result = path.convexityType;
          expect(result, SPathConvexityType.kConvex);
        } else {
          // we make a copy so that we don't cache the result on the passed
          // in path.
          final SurfacePath path2 = SurfacePath.from(path);
          final int c = path2.convexityType;
          assert(SPathConvexityType.kUnknown == c || SPathConvexityType.kConcave == c);
        }
      }
    });

    test('Concave lines path', () {
      final SurfacePath path = SurfacePath();
      path.moveTo(-0.284071773, -0.0622361786);
      path.lineTo(-0.284072, -0.0622351);
      path.lineTo(-0.28407, -0.0622307);
      path.lineTo(-0.284067, -0.0622182);
      path.lineTo(-0.284084, -0.0622269);
      path.lineTo(-0.284072, -0.0622362);
      path.close();
      expect(path.convexityType, SPathConvexityType.kConcave);
    });

    test('Single moveTo origin', () {
      final SurfacePath path = SurfacePath();
      path.moveTo(0, 0);
      expect(path.convexityType, SPathConvexityType.kConvex);
    });

    test('Single diagonal line', () {
      final SurfacePath path = SurfacePath();
      path.moveTo(12, 20);
      path.lineTo(-12, -20);
      expect(path.convexityType, SPathConvexityType.kConvex);
    });

    test('TriLeft', () {
      final SurfacePath path = SurfacePath();
      path.moveTo(0, 0);
      path.lineTo(1, 0);
      path.lineTo(1, 1);
      path.close();
      expect(path.convexityType, SPathConvexityType.kConvex);
    });

    test('TriRight', () {
      final SurfacePath path = SurfacePath();
      path.moveTo(0, 0);
      path.lineTo(-1, 0);
      path.lineTo(1, 1);
      path.close();
      expect(path.convexityType, SPathConvexityType.kConvex);
    });

    test('square', () {
      final SurfacePath path = SurfacePath();
      path.moveTo(0, 0);
      path.lineTo(1, 0);
      path.lineTo(1, 1);
      path.lineTo(0, 1);
      path.close();
      expect(path.convexityType, SPathConvexityType.kConvex);
    });

    test('redundant square', () {
      final SurfacePath redundantSquare = SurfacePath();
      redundantSquare.moveTo(0, 0);
      redundantSquare.lineTo(0, 0);
      redundantSquare.lineTo(0, 0);
      redundantSquare.lineTo(1, 0);
      redundantSquare.lineTo(1, 0);
      redundantSquare.lineTo(1, 0);
      redundantSquare.lineTo(1, 1);
      redundantSquare.lineTo(1, 1);
      redundantSquare.lineTo(1, 1);
      redundantSquare.lineTo(0, 1);
      redundantSquare.lineTo(0, 1);
      redundantSquare.lineTo(0, 1);
      redundantSquare.close();
      expect(redundantSquare.convexityType, SPathConvexityType.kConvex);
    });

    test('bowtie', () {
      final SurfacePath bowTie = SurfacePath();
      bowTie.moveTo(0, 0);
      bowTie.lineTo(0, 0);
      bowTie.lineTo(0, 0);
      bowTie.lineTo(1, 1);
      bowTie.lineTo(1, 1);
      bowTie.lineTo(1, 1);
      bowTie.lineTo(1, 0);
      bowTie.lineTo(1, 0);
      bowTie.lineTo(1, 0);
      bowTie.lineTo(0, 1);
      bowTie.lineTo(0, 1);
      bowTie.lineTo(0, 1);
      bowTie.close();
      expect(bowTie.convexityType, SPathConvexityType.kConcave);
    });

    test('sprial', () {
      final SurfacePath spiral = SurfacePath();
      spiral.moveTo(0, 0);
      spiral.lineTo(100, 0);
      spiral.lineTo(100, 100);
      spiral.lineTo(0, 100);
      spiral.lineTo(0, 50);
      spiral.lineTo(50, 50);
      spiral.lineTo(50, 75);
      spiral.close();
      expect(spiral.convexityType, SPathConvexityType.kConcave);
    });

    test('dent', () {
      final SurfacePath dent = SurfacePath();
      dent.moveTo(0, 0);
      dent.lineTo(100, 100);
      dent.lineTo(0, 100);
      dent.lineTo(-50, 200);
      dent.lineTo(-200, 100);
      dent.close();
      expect(dent.convexityType, SPathConvexityType.kConcave);
    });

    test('degenerate segments1', () {
      final SurfacePath strokedSin = SurfacePath();
      for (int i = 0; i < 2000; i++) {
        final double x = i.toDouble() / 2.0;
        final double y = 500 - (x + math.sin(x / 100) * 40) / 3;
        if (0 == i) {
          strokedSin.moveTo(x, y);
        } else {
          strokedSin.lineTo(x, y);
        }
      }
      expect(strokedSin.convexityType, SPathConvexityType.kConcave);
    });

    /// Regression test for https://github.com/flutter/flutter/issues/66560.
    test('Quadratic', () {
      final SurfacePath path = SurfacePath();
      path.moveTo(100.0, 0.0);
      path.quadraticBezierTo(200.0, 0.0, 200.0, 100.0);
      path.quadraticBezierTo(200.0, 200.0, 100.0, 200.0);
      path.quadraticBezierTo(0.0, 200.0, 0.0, 100.0);
      path.quadraticBezierTo(0.0, 0.0, 100.0, 0.0);
      path.close();
      expect(path.contains(const Offset(100, 20)), isTrue);
      expect(path.contains(const Offset(100, 120)), isTrue);
      expect(path.contains(const Offset(100, -10)), isFalse);
    });
  });
}

class LineTestCase {
  LineTestCase(this.pathContent, this.convexity);
  final String pathContent;
  final int convexity;
}

/// Parses a string of the format "mx my lx1 ly1 lx2 ly2..." into a path
/// with moveTo/lineTo instructions for points.
void setFromString(SurfacePath path, String value) {
  bool first = true;
  final List<String> points = value.split(' ');
  if (points.length < 2) {
    return;
  }
  for (int i = 0; i < points.length; i += 2) {
    if (first) {
      path.moveTo(double.parse(points[i]), double.parse(points[i + 1]));
      first = false;
    } else {
      path.lineTo(double.parse(points[i]), double.parse(points[i + 1]));
    }
  }
}

// Scalar max is based on 32 bit float since [PathRef] stores values in
// Float32List.
const double kScalarMax = 3.402823466e+38;
const double kScalarMin = -kScalarMax;
