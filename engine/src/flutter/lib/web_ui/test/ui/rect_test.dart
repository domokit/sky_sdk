// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/ui.dart';

import '../common/test_initialization.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests();
  test('rect accessors', () {
    const Rect r = Rect.fromLTRB(1.0, 3.0, 5.0, 7.0);
    expect(r.left, equals(1.0));
    expect(r.top, equals(3.0));
    expect(r.right, equals(5.0));
    expect(r.bottom, equals(7.0));
  });

  test('rect created by width and height', () {
    const Rect r = Rect.fromLTWH(1.0, 3.0, 5.0, 7.0);
    expect(r.left, equals(1.0));
    expect(r.top, equals(3.0));
    expect(r.right, equals(6.0));
    expect(r.bottom, equals(10.0));
    expect(r.shortestSide, equals(5.0));
    expect(r.longestSide, equals(7.0));
  });

  test('rect intersection', () {
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

  test('rect expandToInclude overlapping rects', () {
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

  test('rect expandToInclude crossing rects', () {
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

  test('size created from doubles', () {
    const Size size = Size(5.0, 7.0);
    expect(size.width, equals(5.0));
    expect(size.height, equals(7.0));
    expect(size.shortestSide, equals(5.0));
    expect(size.longestSide, equals(7.0));
  });

  test('rounded rect created from rect and radii', () {
    const Rect baseRect = Rect.fromLTWH(1.0, 3.0, 5.0, 7.0);
    final RRect r = RRect.fromRectXY(baseRect, 1.0, 1.0);
    expect(r.left, equals(1.0));
    expect(r.top, equals(3.0));
    expect(r.right, equals(6.0));
    expect(r.bottom, equals(10.0));
    expect(r.shortestSide, equals(5.0));
    expect(r.longestSide, equals(7.0));
  });
}
