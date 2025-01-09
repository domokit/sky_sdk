// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/ui.dart';

import '../common/matchers.dart';
import '../common/test_initialization.dart';

const double kTolerance = 0.1;

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests();
  group('PathMetric length', () {
    test('empty path', () {
      final Path path = Path();
      expect(path.computeMetrics().isEmpty, isTrue);
    });

    test('simple line', () {
      final Path path = Path();
      path.moveTo(100.0, 50.0);
      path.lineTo(200.0, 100.0);
      expect(path.computeMetrics().isEmpty, isFalse);
      final List<PathMetric> metrics = path.computeMetrics().toList();
      expect(metrics.length, 1);
      expect(metrics[0].length, within(distance: kTolerance, from: 111.803));
    });

    test('2 lines', () {
      final Path path = Path();
      path.moveTo(100.0, 50.0);
      path.lineTo(200.0, 50.0);
      path.lineTo(100.0, 200.0);
      expect(path.computeMetrics().isEmpty, isFalse);
      final List<PathMetric> metrics = path.computeMetrics().toList();
      expect(metrics.length, 1);
      expect(metrics[0].length, within(distance: kTolerance, from: 280.277));
    });

    test('2 lines forceClosed', () {
      final Path path = Path();
      path.moveTo(100.0, 50.0);
      path.lineTo(200.0, 50.0);
      path.lineTo(100.0, 200.0);
      expect(path.computeMetrics(forceClosed: true).isEmpty, isFalse);
      final List<PathMetric> metrics = path.computeMetrics(forceClosed: true).toList();
      expect(metrics.length, 1);
      expect(metrics[0].length, within(distance: kTolerance, from: 430.277));
    });

    test('2 subpaths', () {
      final Path path = Path();
      path.moveTo(100.0, 50.0);
      path.lineTo(200.0, 100.0);
      path.moveTo(200.0, 100.0);
      path.lineTo(200.0, 200.0);
      final List<double> contourLengths = computeLengths(path.computeMetrics());
      expect(contourLengths.length, 2);
      expect(contourLengths[0], within(distance: kTolerance, from: 111.803));
      expect(contourLengths[1], within(distance: kTolerance, from: 100.0));
    });

    test('quadratic curve', () {
      final Path path = Path();
      path.moveTo(20, 100);
      path.quadraticBezierTo(80, 10, 140, 110);
      final List<double> contourLengths = computeLengths(path.computeMetrics());
      expect(contourLengths.length, 1);
      expect(contourLengths[0], within(distance: kTolerance, from: 159.473));
    });

    test('cubic curve', () {
      final Path path = Path();
      path.moveTo(20, 100);
      path.cubicTo(80, 10, 120, 90, 140, 40);
      final List<double> contourLengths = computeLengths(path.computeMetrics());
      expect(contourLengths.length, 1);
      expect(contourLengths[0], within(distance: kTolerance, from: 146.567));
    });

    test('addRect', () {
      final Path path = Path();
      path.addRect(const Rect.fromLTRB(20, 30, 220, 130));
      final List<double> contourLengths = computeLengths(path.computeMetrics());
      expect(contourLengths.length, 1);
      expect(contourLengths[0], within(distance: kTolerance, from: 600.0));
    });

    test('addRRect with zero radius', () {
      final Path path = Path();
      path.addRRect(RRect.fromLTRBR(20, 30, 220, 130, Radius.zero));
      final List<double> contourLengths = computeLengths(path.computeMetrics());
      expect(contourLengths.length, 1);
      expect(contourLengths[0], within(distance: kTolerance, from: 600.0));
    });

    test('addRRect with elliptical radius', () {
      final Path path = Path();
      path.addRRect(RRect.fromLTRBR(20, 30, 220, 130, const Radius.elliptical(8, 4)));
      final List<double> contourLengths = computeLengths(path.computeMetrics());
      expect(contourLengths.length, 1);
      expect(contourLengths[0], within(distance: kTolerance, from: 590.408));
    });

    test('arcToPoint < 90 degrees', () {
      const double rx = 100;
      const double ry = 100;
      const double cx = 150;
      const double cy = 100;
      const double startAngle = 0.0;
      const double endAngle = 90.0;
      const double startRad = startAngle * math.pi / 180.0;
      const double endRad = endAngle * math.pi / 180.0;

      final double startX = cx + (rx * math.cos(startRad));
      final double startY = cy + (ry * math.sin(startRad));
      final double endX = cx + (rx * math.cos(endRad));
      final double endY = cy + (ry * math.sin(endRad));

      final bool largeArc = (endAngle - startAngle).abs() > 180.0;
      final Path path =
          Path()
            ..moveTo(startX, startY)
            ..arcToPoint(
              Offset(endX, endY),
              radius: const Radius.elliptical(rx, ry),
              largeArc: largeArc,
            );
      final List<double> contourLengths = computeLengths(path.computeMetrics());
      expect(contourLengths.length, 1);
      expect(contourLengths[0], within(distance: kTolerance, from: 156.827));
    });

    test('arcToPoint 180 degrees', () {
      const double rx = 100;
      const double ry = 100;
      const double cx = 150;
      const double cy = 100;
      const double startAngle = 0.0;
      const double endAngle = 180.0;
      const double startRad = startAngle * math.pi / 180.0;
      const double endRad = endAngle * math.pi / 180.0;

      final double startX = cx + (rx * math.cos(startRad));
      final double startY = cy + (ry * math.sin(startRad));
      final double endX = cx + (rx * math.cos(endRad));
      final double endY = cy + (ry * math.sin(endRad));

      final bool largeArc = (endAngle - startAngle).abs() > 180.0;
      final Path path =
          Path()
            ..moveTo(startX, startY)
            ..arcToPoint(
              Offset(endX, endY),
              radius: const Radius.elliptical(rx, ry),
              largeArc: largeArc,
            );
      final List<double> contourLengths = computeLengths(path.computeMetrics());
      expect(contourLengths.length, 1);
      expect(contourLengths[0], within(distance: kTolerance, from: 313.654));
    });

    test('arcToPoint 270 degrees', () {
      const double rx = 100;
      const double ry = 100;
      const double cx = 150;
      const double cy = 100;
      const double startAngle = 0.0;
      const double endAngle = 270.0;
      const double startRad = startAngle * math.pi / 180.0;
      const double endRad = endAngle * math.pi / 180.0;

      final double startX = cx + (rx * math.cos(startRad));
      final double startY = cy + (ry * math.sin(startRad));
      final double endX = cx + (rx * math.cos(endRad));
      final double endY = cy + (ry * math.sin(endRad));

      final bool largeArc = (endAngle - startAngle).abs() > 180.0;
      final Path path =
          Path()
            ..moveTo(startX, startY)
            ..arcToPoint(
              Offset(endX, endY),
              radius: const Radius.elliptical(rx, ry),
              largeArc: largeArc,
            );
      final List<double> contourLengths = computeLengths(path.computeMetrics());
      expect(contourLengths.length, 1);
      expect(contourLengths[0], within(distance: kTolerance, from: 470.482));
    });

    test('arcToPoint 270 degrees rx!=ry', () {
      const double rx = 100;
      const double ry = 50;
      const double cx = 150;
      const double cy = 100;
      const double startAngle = 0.0;
      const double endAngle = 270.0;
      const double startRad = startAngle * math.pi / 180.0;
      const double endRad = endAngle * math.pi / 180.0;

      final double startX = cx + (rx * math.cos(startRad));
      final double startY = cy + (ry * math.sin(startRad));
      final double endX = cx + (rx * math.cos(endRad));
      final double endY = cy + (ry * math.sin(endRad));

      final bool largeArc = (endAngle - startAngle).abs() > 180.0;
      final Path path =
          Path()
            ..moveTo(startX, startY)
            ..arcToPoint(
              Offset(endX, endY),
              radius: const Radius.elliptical(rx, ry),
              largeArc: largeArc,
            );
      final List<double> contourLengths = computeLengths(path.computeMetrics());
      expect(contourLengths.length, 1);
      expect(contourLengths[0], within(distance: kTolerance, from: 362.733));
    });
  });
}

List<double> computeLengths(PathMetrics pathMetrics) {
  final List<double> lengths = <double>[];
  for (final PathMetric metric in pathMetrics) {
    lengths.add(metric.length);
  }
  return lengths;
}
