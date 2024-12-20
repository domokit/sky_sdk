// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart' hide TypeMatcher, isInstanceOf;
import 'package:ui/ui.dart';

import '../common/test_initialization.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests();
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

  test('RRect.webOnlyUniformRadii returns true when all corner radii are equal', () {
    final RRect rect1 = RRect.fromRectAndCorners(
      const Rect.fromLTWH(1.0, 2.0, 3.0, 4.0),
      topLeft: const Radius.elliptical(5, 5),
      topRight: const Radius.elliptical(5, 5),
      bottomLeft: const Radius.elliptical(5, 5),
      bottomRight: const Radius.elliptical(5, 5),
    );
    expect(rect1.webOnlyUniformRadii, isTrue);

    final RRect rect2 = RRect.fromRectAndCorners(
      const Rect.fromLTWH(1.0, 2.0, 3.0, 4.0),
      topLeft: const Radius.elliptical(1000, 5),
      topRight: const Radius.elliptical(5, 5),
      bottomLeft: const Radius.elliptical(5, 5),
      bottomRight: const Radius.elliptical(5, 5),
    );
    expect(rect2.webOnlyUniformRadii, isFalse);

    final RRect rect3 = RRect.fromRectAndCorners(
      const Rect.fromLTWH(1.0, 2.0, 3.0, 4.0),
      topLeft: const Radius.elliptical(5, 1000),
      topRight: const Radius.elliptical(5, 5),
      bottomLeft: const Radius.elliptical(5, 5),
      bottomRight: const Radius.elliptical(5, 5),
    );
    expect(rect3.webOnlyUniformRadii, isFalse);

    final RRect rect4 = RRect.fromRectAndCorners(
      const Rect.fromLTWH(1.0, 2.0, 3.0, 4.0),
      topLeft: const Radius.elliptical(5, 5),
      topRight: const Radius.elliptical(1000, 5),
      bottomLeft: const Radius.elliptical(5, 5),
      bottomRight: const Radius.elliptical(5, 5),
    );
    expect(rect4.webOnlyUniformRadii, isFalse);

    final RRect rect5 = RRect.fromRectAndCorners(
      const Rect.fromLTWH(1.0, 2.0, 3.0, 4.0),
      topLeft: const Radius.elliptical(5, 5),
      topRight: const Radius.elliptical(5, 1000),
      bottomLeft: const Radius.elliptical(5, 5),
      bottomRight: const Radius.elliptical(5, 5),
    );
    expect(rect5.webOnlyUniformRadii, isFalse);

    final RRect rect6 = RRect.fromRectAndCorners(
      const Rect.fromLTWH(1.0, 2.0, 3.0, 4.0),
      topLeft: const Radius.elliptical(5, 5),
      topRight: const Radius.elliptical(5, 5),
      bottomLeft: const Radius.elliptical(1000, 5),
      bottomRight: const Radius.elliptical(5, 5),
    );
    expect(rect6.webOnlyUniformRadii, isFalse);

    final RRect rect7 = RRect.fromRectAndCorners(
      const Rect.fromLTWH(1.0, 2.0, 3.0, 4.0),
      topLeft: const Radius.elliptical(5, 5),
      topRight: const Radius.elliptical(5, 5),
      bottomLeft: const Radius.elliptical(5, 1000),
      bottomRight: const Radius.elliptical(5, 5),
    );
    expect(rect7.webOnlyUniformRadii, isFalse);

    final RRect rect8 = RRect.fromRectAndCorners(
      const Rect.fromLTWH(1.0, 2.0, 3.0, 4.0),
      topLeft: const Radius.elliptical(5, 5),
      topRight: const Radius.elliptical(5, 5),
      bottomLeft: const Radius.elliptical(5, 5),
      bottomRight: const Radius.elliptical(1000, 5),
    );
    expect(rect8.webOnlyUniformRadii, isFalse);

    final RRect rect9 = RRect.fromRectAndCorners(
      const Rect.fromLTWH(1.0, 2.0, 3.0, 4.0),
      topLeft: const Radius.elliptical(5, 5),
      topRight: const Radius.elliptical(5, 5),
      bottomLeft: const Radius.elliptical(5, 5),
      bottomRight: const Radius.elliptical(5, 1000),
    );
    expect(rect9.webOnlyUniformRadii, isFalse);
  });
}
