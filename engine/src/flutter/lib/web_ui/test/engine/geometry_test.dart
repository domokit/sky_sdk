// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This is testing some of the named constants.
// ignore_for_file: use_named_constants

import 'dart:math' as math show sqrt;
import 'dart:math' show pi;

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';

import 'package:ui/ui.dart';

import '../common/matchers.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
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
    // aah, floating point math. i love you so.
    expect(Offset.fromDirection(pi / 2.0), within(from: const Offset(0.0, 1.0)));
    expect(Offset.fromDirection(-pi / 2.0), within(from: const Offset(0.0, -1.0)));
    expect(Offset.fromDirection(0.0), const Offset(1.0, 0.0));
    expect(
      Offset.fromDirection(pi / 4.0),
      within(from: Offset(1.0 / math.sqrt(2.0), 1.0 / math.sqrt(2.0))),
    );
    expect(
      Offset.fromDirection(-pi / 4.0),
      within(from: Offset(1.0 / math.sqrt(2.0), -1.0 / math.sqrt(2.0))),
    );
    expect(Offset.fromDirection(pi), within(from: const Offset(-1.0, 0.0)));
    expect(
      Offset.fromDirection(pi * 3.0 / 4.0),
      within(from: Offset(-1.0 / math.sqrt(2.0), 1.0 / math.sqrt(2.0))),
    );
    expect(
      Offset.fromDirection(-pi * 3.0 / 4.0),
      within(from: Offset(-1.0 / math.sqrt(2.0), -1.0 / math.sqrt(2.0))),
    );
    expect(Offset.fromDirection(0.0, 2.0), const Offset(2.0, 0.0));
    expect(Offset.fromDirection(pi / 6, 2.0), within(from: Offset(math.sqrt(3.0), 1.0)));
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
    bool assertsEnabled = false;
    assert(() {
      assertsEnabled = true;
      return true;
    }());
    if (!assertsEnabled) {
      return;
    }

    expect(() {
      RRect.fromRectAndCorners(
        const Rect.fromLTRB(10.0, 20.0, 30.0, 40.0),
        topLeft: const Radius.circular(-1),
      );
    }, throwsA(isA<AssertionError>()));

    expect(() {
      RRect.fromRectAndCorners(
        const Rect.fromLTRB(10.0, 20.0, 30.0, 40.0),
        topRight: const Radius.circular(-2),
      );
    }, throwsA(isA<AssertionError>()));

    expect(() {
      RRect.fromRectAndCorners(
        const Rect.fromLTRB(10.0, 20.0, 30.0, 40.0),
        bottomLeft: const Radius.circular(-3),
      );
    }, throwsA(isA<AssertionError>()));

    expect(() {
      RRect.fromRectAndCorners(
        const Rect.fromLTRB(10.0, 20.0, 30.0, 40.0),
        bottomRight: const Radius.circular(-4),
      );
    }, throwsA(isA<AssertionError>()));
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
