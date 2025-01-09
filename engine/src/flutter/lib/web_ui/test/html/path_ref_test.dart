// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart';

// TODO(yjbanov): https://github.com/flutter/flutter/issues/76885
const bool issue76885Exists = true;

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  test('PathRef.getRRect with radius', () {
    final SurfacePath path = SurfacePath();
    final RRect rrect = RRect.fromLTRBR(0, 0, 10, 10, const Radius.circular(2));
    path.addRRect(rrect);
    expect(path.toRoundedRect(), rrect);
  });

  test('PathRef.getRRect with radius larger than rect', () {
    final SurfacePath path = SurfacePath();
    final RRect rrect = RRect.fromLTRBR(0, 0, 10, 10, const Radius.circular(20));
    path.addRRect(rrect);
    expect(
      path.toRoundedRect(),
      // Path.addRRect will correct the radius to fit the dimensions, so when
      // extracting the rrect out of the path we don't get the original.
      RRect.fromLTRBR(0, 0, 10, 10, const Radius.circular(5)),
    );
  });

  test('PathRef.getRRect with zero radius', () {
    final SurfacePath path = SurfacePath();
    final RRect rrect = RRect.fromLTRBR(0, 0, 10, 10, Radius.zero);
    path.addRRect(rrect);
    expect(path.toRoundedRect(), isNull);
    expect(path.toRect(), rrect.outerRect);
  });

  test('PathRef.getRRect elliptical', () {
    final SurfacePath path = SurfacePath();
    final RRect rrect = RRect.fromLTRBR(0, 0, 10, 10, const Radius.elliptical(2, 4));
    path.addRRect(rrect);
    expect(path.toRoundedRect(), rrect);
  });

  test('PathRef.getRRect elliptical zero x', () {
    final SurfacePath path = SurfacePath();
    final RRect rrect = RRect.fromLTRBR(0, 0, 10, 10, const Radius.elliptical(0, 3));
    path.addRRect(rrect);
    expect(path.toRoundedRect(), isNull);
    expect(path.toRect(), rrect.outerRect);
  });

  test('PathRef.getRRect elliptical zero y', () {
    final SurfacePath path = SurfacePath();
    final RRect rrect = RRect.fromLTRBR(0, 0, 10, 10, const Radius.elliptical(3, 0));
    path.addRRect(rrect);
    expect(path.toRoundedRect(), isNull);
    expect(path.toRect(), rrect.outerRect);
  });

  test('PathRef.getRect returns a Rect from a valid Path and null otherwise', () {
    final SurfacePath path = SurfacePath();
    // Draw a line
    path.moveTo(0, 0);
    path.lineTo(10, 0);
    expect(path.pathRef.getRect(), isNull);
    // Draw two other lines to get a valid rectangle
    path.lineTo(10, 10);
    path.lineTo(0, 10);
    expect(path.pathRef.getRect(), const Rect.fromLTWH(0, 0, 10, 10));
  });

  // Regression test for https://github.com/flutter/flutter/issues/111750
  test('PathRef.getRect returns Rect with positive width and height', () {
    final SurfacePath path = SurfacePath();
    // Draw a rectangle starting from bottom right corner
    path.moveTo(10, 10);
    path.lineTo(0, 10);
    path.lineTo(0, 0);
    path.lineTo(10, 0);
    // pathRef.getRect() should return a rectangle with positive height and width
    expect(path.pathRef.getRect(), const Rect.fromLTWH(0, 0, 10, 10));
  });

  // This test demonstrates the issue with attempting to reconstruct an RRect
  // with imprecision introduced by comparing double values. We should fix this
  // by removing the need to reconstruct rrects:
  // https://github.com/flutter/flutter/issues/76885
  test('PathRef.getRRect with nearly zero corner', () {
    final SurfacePath path = SurfacePath();
    final RRect original = RRect.fromLTRBR(0, 0, 10, 10, const Radius.elliptical(0.00000001, 5));
    path.addRRect(original);
    expect(path.toRoundedRect(), original);
  }, skip: issue76885Exists);
}
