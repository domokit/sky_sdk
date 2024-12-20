// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This is testing some of the named constants.
// ignore_for_file: use_named_constants

import 'dart:math' as math show sqrt;
import 'dart:math' show pi;
import 'dart:ui';

import 'package:test/test.dart';

void main() {
  test('OffsetBase.>=', () {
    expect(const Offset(0, 0) >= const Offset(0, -1), true);
    expect(const Offset(0, 0) >= const Offset(-1, 0), true);
    expect(const Offset(0, 0) >= const Offset(-1, -1), true);
    expect(const Offset(0, 0) >= const Offset(0, 0), true);
    expect(const Offset(0, 0) >= const Offset(0, double.nan), false);
    expect(const Offset(0, 0) >= const Offset(double.nan, 0), false);
    expect(const Offset(0, 0) >= const Offset(10, -10), false);
  });

  test('OffsetBase.<=', () {
    expect(const Offset(0, 0) <= const Offset(0, 1), true);
    expect(const Offset(0, 0) <= const Offset(1, 0), true);
    expect(const Offset(0, 0) <= const Offset(0, 0), true);
    expect(const Offset(0, 0) <= const Offset(0, double.nan), false);
    expect(const Offset(0, 0) <= const Offset(double.nan, 0), false);
    expect(const Offset(0, 0) <= const Offset(10, -10), false);
  });

  test('OffsetBase.>', () {
    expect(const Offset(0, 0) > const Offset(-1, -1), true);
    expect(const Offset(0, 0) > const Offset(0, -1), false);
    expect(const Offset(0, 0) > const Offset(-1, 0), false);
    expect(const Offset(0, 0) > const Offset(double.nan, -1), false);
  });

  test('OffsetBase.<', () {
    expect(const Offset(0, 0) < const Offset(1, 1), true);
    expect(const Offset(0, 0) < const Offset(0, 1), false);
    expect(const Offset(0, 0) < const Offset(1, 0), false);
    expect(const Offset(0, 0) < const Offset(double.nan, 1), false);
  });

  test('OffsetBase.==', () {
    expect(const Offset(0, 0), equals(const Offset(0, 0)));
    expect(const Offset(0, 0), isNot(const Offset(1, 0)));
    expect(const Offset(0, 0), isNot(const Offset(0, 1)));
  });

  test('Offset.direction', () {
    expect(const Offset(0.0, 0.0).direction, 0.0);
    expect(const Offset(0.0, 1.0).direction, pi / 2.0);
    expect(const Offset(0.0, -1.0).direction, -pi / 2.0);
    expect(const Offset(1.0, 0.0).direction, 0.0);
    expect(const Offset(1.0, 1.0).direction, pi / 4.0);
    expect(const Offset(1.0, -1.0).direction, -pi / 4.0);
    expect(const Offset(-1.0, 0.0).direction, pi);
    expect(const Offset(-1.0, 1.0).direction, pi * 3.0 / 4.0);
    expect(const Offset(-1.0, -1.0).direction, -pi * 3.0 / 4.0);
  });

  test('Offset.fromDirection', () {
    expect(Offset.fromDirection(0.0, 0.0), const Offset(0.0, 0.0));
    expect(
      Offset.fromDirection(pi / 2.0).dx,
      closeTo(0.0, 1e-12),
    ); // aah, floating point math. i love you so.
    expect(Offset.fromDirection(pi / 2.0).dy, 1.0);
    expect(Offset.fromDirection(-pi / 2.0).dx, closeTo(0.0, 1e-12));
    expect(Offset.fromDirection(-pi / 2.0).dy, -1.0);
    expect(Offset.fromDirection(0.0), const Offset(1.0, 0.0));
    expect(Offset.fromDirection(pi / 4.0).dx, closeTo(1.0 / math.sqrt(2.0), 1e-12));
    expect(Offset.fromDirection(pi / 4.0).dy, closeTo(1.0 / math.sqrt(2.0), 1e-12));
    expect(Offset.fromDirection(-pi / 4.0).dx, closeTo(1.0 / math.sqrt(2.0), 1e-12));
    expect(Offset.fromDirection(-pi / 4.0).dy, closeTo(-1.0 / math.sqrt(2.0), 1e-12));
    expect(Offset.fromDirection(pi).dx, -1.0);
    expect(Offset.fromDirection(pi).dy, closeTo(0.0, 1e-12));
    expect(Offset.fromDirection(pi * 3.0 / 4.0).dx, closeTo(-1.0 / math.sqrt(2.0), 1e-12));
    expect(Offset.fromDirection(pi * 3.0 / 4.0).dy, closeTo(1.0 / math.sqrt(2.0), 1e-12));
    expect(Offset.fromDirection(-pi * 3.0 / 4.0).dx, closeTo(-1.0 / math.sqrt(2.0), 1e-12));
    expect(Offset.fromDirection(-pi * 3.0 / 4.0).dy, closeTo(-1.0 / math.sqrt(2.0), 1e-12));
    expect(Offset.fromDirection(0.0, 2.0), const Offset(2.0, 0.0));
    expect(Offset.fromDirection(pi / 6, 2.0).dx, closeTo(math.sqrt(3.0), 1e-12));
    expect(Offset.fromDirection(pi / 6, 2.0).dy, closeTo(1.0, 1e-12));
  });

  test('Size created from doubles', () {
    const Size size = Size(5.0, 7.0);
    expect(size.width, equals(5.0));
    expect(size.height, equals(7.0));
    expect(size.shortestSide, equals(5.0));
    expect(size.longestSide, equals(7.0));
  });

  test('Size.aspectRatio', () {
    expect(const Size(0.0, 0.0).aspectRatio, 0.0);
    expect(const Size(-0.0, 0.0).aspectRatio, 0.0);
    expect(const Size(0.0, -0.0).aspectRatio, 0.0);
    expect(const Size(-0.0, -0.0).aspectRatio, 0.0);
    expect(const Size(0.0, 1.0).aspectRatio, 0.0);
    expect(const Size(0.0, -1.0).aspectRatio, -0.0);
    expect(const Size(1.0, 0.0).aspectRatio, double.infinity);
    expect(const Size(1.0, 1.0).aspectRatio, 1.0);
    expect(const Size(1.0, -1.0).aspectRatio, -1.0);
    expect(const Size(-1.0, 0.0).aspectRatio, -double.infinity);
    expect(const Size(-1.0, 1.0).aspectRatio, -1.0);
    expect(const Size(-1.0, -1.0).aspectRatio, 1.0);
    expect(const Size(3.0, 4.0).aspectRatio, 3.0 / 4.0);
  });

  test('Rect.toString test', () {
    const Rect r = Rect.fromLTRB(1.0, 3.0, 5.0, 7.0);
    expect(r.toString(), 'Rect.fromLTRB(1.0, 3.0, 5.0, 7.0)');
  });

  test('Rect accessors', () {
    const Rect r = Rect.fromLTRB(1.0, 3.0, 5.0, 7.0);
    expect(r.left, equals(1.0));
    expect(r.top, equals(3.0));
    expect(r.right, equals(5.0));
    expect(r.bottom, equals(7.0));
  });

  test('Rect.fromCenter', () {
    Rect rect = Rect.fromCenter(center: const Offset(1.0, 3.0), width: 5.0, height: 7.0);
    expect(rect.left, -1.5);
    expect(rect.top, -0.5);
    expect(rect.right, 3.5);
    expect(rect.bottom, 6.5);

    rect = Rect.fromCenter(center: const Offset(0.0, 0.0), width: 0.0, height: 0.0);
    expect(rect.left, 0.0);
    expect(rect.top, 0.0);
    expect(rect.right, 0.0);
    expect(rect.bottom, 0.0);

    rect = Rect.fromCenter(center: const Offset(double.nan, 0.0), width: 0.0, height: 0.0);
    expect(rect.left, isNaN);
    expect(rect.top, 0.0);
    expect(rect.right, isNaN);
    expect(rect.bottom, 0.0);

    rect = Rect.fromCenter(center: const Offset(0.0, double.nan), width: 0.0, height: 0.0);
    expect(rect.left, 0.0);
    expect(rect.top, isNaN);
    expect(rect.right, 0.0);
    expect(rect.bottom, isNaN);
  });

  test('Rect created by width and height', () {
    const Rect r = Rect.fromLTWH(1.0, 3.0, 5.0, 7.0);
    expect(r.left, equals(1.0));
    expect(r.top, equals(3.0));
    expect(r.right, equals(6.0));
    expect(r.bottom, equals(10.0));
    expect(r.shortestSide, equals(5.0));
    expect(r.longestSide, equals(7.0));
  });

  test('Rect intersection', () {
    const Rect r1 = Rect.fromLTRB(0.0, 0.0, 100.0, 100.0);
    const Rect r2 = Rect.fromLTRB(50.0, 50.0, 200.0, 200.0);
    final Rect r3 = r1.intersect(r2);
    expect(r3.left, equals(50.0));
    expect(r3.top, equals(50.0));
    expect(r3.right, equals(100.0));
    expect(r3.bottom, equals(100.0));
    final Rect r4 = r2.intersect(r1);
    expect(r4, equals(r3));
  });

  test('Rect expandToInclude overlapping rects', () {
    const Rect r1 = Rect.fromLTRB(0.0, 0.0, 100.0, 100.0);
    const Rect r2 = Rect.fromLTRB(50.0, 50.0, 200.0, 200.0);
    final Rect r3 = r1.expandToInclude(r2);
    expect(r3.left, equals(0.0));
    expect(r3.top, equals(0.0));
    expect(r3.right, equals(200.0));
    expect(r3.bottom, equals(200.0));

    final Rect r4 = r2.expandToInclude(r1);
    expect(r4, equals(r3));
  });

  test('Rect expandToInclude crossing rects', () {
    const Rect r1 = Rect.fromLTRB(50.0, 0.0, 50.0, 200.0);
    const Rect r2 = Rect.fromLTRB(0.0, 50.0, 200.0, 50.0);
    final Rect r3 = r1.expandToInclude(r2);
    expect(r3.left, equals(0.0));
    expect(r3.top, equals(0.0));
    expect(r3.right, equals(200.0));
    expect(r3.bottom, equals(200.0));

    final Rect r4 = r2.expandToInclude(r1);
    expect(r4, equals(r3));
  });

  test('RRect.fromRectXY', () {
    const Rect baseRect = Rect.fromLTWH(1.0, 3.0, 5.0, 7.0);
    final RRect r = RRect.fromRectXY(baseRect, 1.0, 1.0);
    expect(r.left, equals(1.0));
    expect(r.top, equals(3.0));
    expect(r.right, equals(6.0));
    expect(r.bottom, equals(10.0));
    expect(r.shortestSide, equals(5.0));
    expect(r.longestSide, equals(7.0));
  });

  test('RRect.contains()', () {
    final RRect rrect = RRect.fromRectAndCorners(
      const Rect.fromLTRB(1.0, 1.0, 2.0, 2.0),
      topLeft: const Radius.circular(0.5),
      topRight: const Radius.circular(0.25),
      bottomRight: const Radius.elliptical(0.25, 0.75),
    );

    expect(rrect.contains(const Offset(1.0, 1.0)), isFalse);
    expect(rrect.contains(const Offset(1.1, 1.1)), isFalse);
    expect(rrect.contains(const Offset(1.15, 1.15)), isTrue);
    expect(rrect.contains(const Offset(2.0, 1.0)), isFalse);
    expect(rrect.contains(const Offset(1.93, 1.07)), isFalse);
    expect(rrect.contains(const Offset(1.97, 1.7)), isFalse);
    expect(rrect.contains(const Offset(1.7, 1.97)), isTrue);
    expect(rrect.contains(const Offset(1.0, 1.99)), isTrue);
  });

  test('RRect.contains() large radii', () {
    final RRect rrect = RRect.fromRectAndCorners(
      const Rect.fromLTRB(1.0, 1.0, 2.0, 2.0),
      topLeft: const Radius.circular(5000.0),
      topRight: const Radius.circular(2500.0),
      bottomRight: const Radius.elliptical(2500.0, 7500.0),
    );

    expect(rrect.contains(const Offset(1.0, 1.0)), isFalse);
    expect(rrect.contains(const Offset(1.1, 1.1)), isFalse);
    expect(rrect.contains(const Offset(1.15, 1.15)), isTrue);
    expect(rrect.contains(const Offset(2.0, 1.0)), isFalse);
    expect(rrect.contains(const Offset(1.93, 1.07)), isFalse);
    expect(rrect.contains(const Offset(1.97, 1.7)), isFalse);
    expect(rrect.contains(const Offset(1.7, 1.97)), isTrue);
    expect(rrect.contains(const Offset(1.0, 1.99)), isTrue);
  });

  test('RRect.scaleRadii() properly constrained radii should remain unchanged', () {
    final RRect rrect =
        RRect.fromRectAndCorners(
          const Rect.fromLTRB(1.0, 1.0, 2.0, 2.0),
          topLeft: const Radius.circular(0.5),
          topRight: const Radius.circular(0.25),
          bottomRight: const Radius.elliptical(0.25, 0.75),
        ).scaleRadii();

    // check sides
    expect(rrect.left, 1.0);
    expect(rrect.top, 1.0);
    expect(rrect.right, 2.0);
    expect(rrect.bottom, 2.0);

    // check corner radii
    expect(rrect.tlRadiusX, 0.5);
    expect(rrect.tlRadiusY, 0.5);
    expect(rrect.trRadiusX, 0.25);
    expect(rrect.trRadiusY, 0.25);
    expect(rrect.blRadiusX, 0.0);
    expect(rrect.blRadiusY, 0.0);
    expect(rrect.brRadiusX, 0.25);
    expect(rrect.brRadiusY, 0.75);
  });

  test('RRect.scaleRadii() sum of radii that exceed side length should properly scale', () {
    final RRect rrect =
        RRect.fromRectAndCorners(
          const Rect.fromLTRB(1.0, 1.0, 2.0, 2.0),
          topLeft: const Radius.circular(5000.0),
          topRight: const Radius.circular(2500.0),
          bottomRight: const Radius.elliptical(2500.0, 7500.0),
        ).scaleRadii();

    // check sides
    expect(rrect.left, 1.0);
    expect(rrect.top, 1.0);
    expect(rrect.right, 2.0);
    expect(rrect.bottom, 2.0);

    // check corner radii
    expect(rrect.tlRadiusX, 0.5);
    expect(rrect.tlRadiusY, 0.5);
    expect(rrect.trRadiusX, 0.25);
    expect(rrect.trRadiusY, 0.25);
    expect(rrect.blRadiusX, 0.0);
    expect(rrect.blRadiusY, 0.0);
    expect(rrect.brRadiusX, 0.25);
    expect(rrect.brRadiusY, 0.75);
  });

  test('Radius.clamp() operates as expected', () {
    final RRect rrectMin = RRect.fromLTRBR(
      1,
      3,
      5,
      7,
      const Radius.circular(-100).clamp(minimum: Radius.zero),
    );

    expect(rrectMin.left, 1);
    expect(rrectMin.top, 3);
    expect(rrectMin.right, 5);
    expect(rrectMin.bottom, 7);
    expect(rrectMin.trRadius, equals(const Radius.circular(0)));
    expect(rrectMin.blRadius, equals(const Radius.circular(0)));

    final RRect rrectMax = RRect.fromLTRBR(
      1,
      3,
      5,
      7,
      const Radius.circular(100).clamp(maximum: const Radius.circular(10)),
    );

    expect(rrectMax.left, 1);
    expect(rrectMax.top, 3);
    expect(rrectMax.right, 5);
    expect(rrectMax.bottom, 7);
    expect(rrectMax.trRadius, equals(const Radius.circular(10)));
    expect(rrectMax.blRadius, equals(const Radius.circular(10)));

    final RRect rrectMix = RRect.fromLTRBR(
      1,
      3,
      5,
      7,
      const Radius.elliptical(
        -100,
        100,
      ).clamp(minimum: Radius.zero, maximum: const Radius.circular(10)),
    );

    expect(rrectMix.left, 1);
    expect(rrectMix.top, 3);
    expect(rrectMix.right, 5);
    expect(rrectMix.bottom, 7);
    expect(rrectMix.trRadius, equals(const Radius.elliptical(0, 10)));
    expect(rrectMix.blRadius, equals(const Radius.elliptical(0, 10)));

    final RRect rrectMix1 = RRect.fromLTRBR(
      1,
      3,
      5,
      7,
      const Radius.elliptical(
        100,
        -100,
      ).clamp(minimum: Radius.zero, maximum: const Radius.circular(10)),
    );

    expect(rrectMix1.left, 1);
    expect(rrectMix1.top, 3);
    expect(rrectMix1.right, 5);
    expect(rrectMix1.bottom, 7);
    expect(rrectMix1.trRadius, equals(const Radius.elliptical(10, 0)));
    expect(rrectMix1.blRadius, equals(const Radius.elliptical(10, 0)));
  });

  test('Radius.clampValues() operates as expected', () {
    final RRect rrectMin = RRect.fromLTRBR(
      1,
      3,
      5,
      7,
      const Radius.circular(-100).clampValues(minimumX: 0, minimumY: 0),
    );

    expect(rrectMin.left, 1);
    expect(rrectMin.top, 3);
    expect(rrectMin.right, 5);
    expect(rrectMin.bottom, 7);
    expect(rrectMin.trRadius, equals(const Radius.circular(0)));
    expect(rrectMin.blRadius, equals(const Radius.circular(0)));

    final RRect rrectMax = RRect.fromLTRBR(
      1,
      3,
      5,
      7,
      const Radius.circular(100).clampValues(maximumX: 10, maximumY: 20),
    );

    expect(rrectMax.left, 1);
    expect(rrectMax.top, 3);
    expect(rrectMax.right, 5);
    expect(rrectMax.bottom, 7);
    expect(rrectMax.trRadius, equals(const Radius.elliptical(10, 20)));
    expect(rrectMax.blRadius, equals(const Radius.elliptical(10, 20)));

    final RRect rrectMix = RRect.fromLTRBR(
      1,
      3,
      5,
      7,
      const Radius.elliptical(
        -100,
        100,
      ).clampValues(minimumX: 5, minimumY: 6, maximumX: 10, maximumY: 20),
    );

    expect(rrectMix.left, 1);
    expect(rrectMix.top, 3);
    expect(rrectMix.right, 5);
    expect(rrectMix.bottom, 7);
    expect(rrectMix.trRadius, equals(const Radius.elliptical(5, 20)));
    expect(rrectMix.blRadius, equals(const Radius.elliptical(5, 20)));

    final RRect rrectMix2 = RRect.fromLTRBR(
      1,
      3,
      5,
      7,
      const Radius.elliptical(
        100,
        -100,
      ).clampValues(minimumX: 5, minimumY: 6, maximumX: 10, maximumY: 20),
    );

    expect(rrectMix2.left, 1);
    expect(rrectMix2.top, 3);
    expect(rrectMix2.right, 5);
    expect(rrectMix2.bottom, 7);
    expect(rrectMix2.trRadius, equals(const Radius.elliptical(10, 6)));
    expect(rrectMix2.blRadius, equals(const Radius.elliptical(10, 6)));
  });

  test('RRect asserts when corner radii are negative', () {
    expect(() {
      RRect.fromRectAndCorners(
        const Rect.fromLTRB(10.0, 20.0, 30.0, 40.0),
        topLeft: const Radius.circular(-1),
      );
    }, throwsA(const isInstanceOf<AssertionError>()));

    expect(() {
      RRect.fromRectAndCorners(
        const Rect.fromLTRB(10.0, 20.0, 30.0, 40.0),
        topRight: const Radius.circular(-2),
      );
    }, throwsA(const isInstanceOf<AssertionError>()));

    expect(() {
      RRect.fromRectAndCorners(
        const Rect.fromLTRB(10.0, 20.0, 30.0, 40.0),
        bottomLeft: const Radius.circular(-3),
      );
    }, throwsA(const isInstanceOf<AssertionError>()));

    expect(() {
      RRect.fromRectAndCorners(
        const Rect.fromLTRB(10.0, 20.0, 30.0, 40.0),
        bottomRight: const Radius.circular(-4),
      );
    }, throwsA(const isInstanceOf<AssertionError>()));
  });

  test('RRect.inflate clamps when deflating past zero', () {
    RRect rrect = RRect.fromRectAndCorners(
      const Rect.fromLTRB(10.0, 20.0, 30.0, 40.0),
      topLeft: const Radius.circular(1),
      topRight: const Radius.circular(2),
      bottomLeft: const Radius.circular(3),
      bottomRight: const Radius.circular(4),
    ).inflate(-1);

    expect(rrect.tlRadiusX, 0);
    expect(rrect.tlRadiusY, 0);
    expect(rrect.trRadiusX, 1);
    expect(rrect.trRadiusY, 1);
    expect(rrect.blRadiusX, 2);
    expect(rrect.blRadiusY, 2);
    expect(rrect.brRadiusX, 3);
    expect(rrect.brRadiusY, 3);

    rrect = rrect.inflate(-1);
    expect(rrect.tlRadiusX, 0);
    expect(rrect.tlRadiusY, 0);
    expect(rrect.trRadiusX, 0);
    expect(rrect.trRadiusY, 0);
    expect(rrect.blRadiusX, 1);
    expect(rrect.blRadiusY, 1);
    expect(rrect.brRadiusX, 2);
    expect(rrect.brRadiusY, 2);

    rrect = rrect.inflate(-1);
    expect(rrect.tlRadiusX, 0);
    expect(rrect.tlRadiusY, 0);
    expect(rrect.trRadiusX, 0);
    expect(rrect.trRadiusY, 0);
    expect(rrect.blRadiusX, 0);
    expect(rrect.blRadiusY, 0);
    expect(rrect.brRadiusX, 1);
    expect(rrect.brRadiusY, 1);

    rrect = rrect.inflate(-1);
    expect(rrect.tlRadiusX, 0);
    expect(rrect.tlRadiusY, 0);
    expect(rrect.trRadiusX, 0);
    expect(rrect.trRadiusY, 0);
    expect(rrect.blRadiusX, 0);
    expect(rrect.blRadiusY, 0);
    expect(rrect.brRadiusX, 0);
    expect(rrect.brRadiusY, 0);
  });

  test('infinity lerp', () {
    const Offset a = Offset(double.infinity, double.infinity);
    const Offset b = Offset(4, 4);
    final Offset? result = Offset.lerp(a, b, 0.5);
    if (result == null) {
      expect(result != null, true);
    } else {
      expect(result.dx, double.infinity);
      expect(result.dy, double.infinity);
    }
  });

  test('RRect.deflate clamps when deflating past zero', () {
    RRect rrect = RRect.fromRectAndCorners(
      const Rect.fromLTRB(10.0, 20.0, 30.0, 40.0),
      topLeft: const Radius.circular(1),
      topRight: const Radius.circular(2),
      bottomLeft: const Radius.circular(3),
      bottomRight: const Radius.circular(4),
    ).deflate(1);

    expect(rrect.tlRadiusX, 0);
    expect(rrect.tlRadiusY, 0);
    expect(rrect.trRadiusX, 1);
    expect(rrect.trRadiusY, 1);
    expect(rrect.blRadiusX, 2);
    expect(rrect.blRadiusY, 2);
    expect(rrect.brRadiusX, 3);
    expect(rrect.brRadiusY, 3);

    rrect = rrect.deflate(1);
    expect(rrect.tlRadiusX, 0);
    expect(rrect.tlRadiusY, 0);
    expect(rrect.trRadiusX, 0);
    expect(rrect.trRadiusY, 0);
    expect(rrect.blRadiusX, 1);
    expect(rrect.blRadiusY, 1);
    expect(rrect.brRadiusX, 2);
    expect(rrect.brRadiusY, 2);

    rrect = rrect.deflate(1);
    expect(rrect.tlRadiusX, 0);
    expect(rrect.tlRadiusY, 0);
    expect(rrect.trRadiusX, 0);
    expect(rrect.trRadiusY, 0);
    expect(rrect.blRadiusX, 0);
    expect(rrect.blRadiusY, 0);
    expect(rrect.brRadiusX, 1);
    expect(rrect.brRadiusY, 1);

    rrect = rrect.deflate(1);
    expect(rrect.tlRadiusX, 0);
    expect(rrect.tlRadiusY, 0);
    expect(rrect.trRadiusX, 0);
    expect(rrect.trRadiusY, 0);
    expect(rrect.blRadiusX, 0);
    expect(rrect.blRadiusY, 0);
    expect(rrect.brRadiusX, 0);
    expect(rrect.brRadiusY, 0);
  });
}
