// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/painting.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('OvalBorder defaults', () {
    const OvalBorder border = OvalBorder();
    expect(border.side, BorderSide.none);
  });

  test('OvalBorder copyWith, ==, hashCode', () {
    expect(const OvalBorder(), const OvalBorder().copyWith());
    expect(const OvalBorder().hashCode, const OvalBorder().copyWith().hashCode);
    const BorderSide side = BorderSide(width: 10.0, color: Color(0xff123456));
    expect(const OvalBorder().copyWith(side: side), const OvalBorder(side: side));
  });

  test('OvalBorder', () {
    const OvalBorder c10 = OvalBorder(side: BorderSide(width: 10.0));
    const OvalBorder c15 = OvalBorder(side: BorderSide(width: 15.0));
    const OvalBorder c20 = OvalBorder(side: BorderSide(width: 20.0));
    expect(c10.dimensions, const EdgeInsets.all(10.0));
    expect(c10.scale(2.0), c20);
    expect(c20.scale(0.5), c10);
    expect(ShapeBorder.lerp(c10, c20, 0.0), c10);
    expect(ShapeBorder.lerp(c10, c20, 0.5), c15);
    expect(ShapeBorder.lerp(c10, c20, 1.0), c20);
    expect(
      c10.getInnerPath(const Rect.fromLTWH(0, 0, 100, 40)),
      isPathThat(
        includes: const <Offset>[Offset(12, 19), Offset(50, 10), Offset(88, 19), Offset(50, 29)],
        excludes: const <Offset>[Offset(17, 26), Offset(15, 15), Offset(74, 10), Offset(76, 28)],
      ),
    );
    expect(
      c10.getOuterPath(const Rect.fromLTWH(0, 0, 100, 20)),
      isPathThat(
        includes: const <Offset>[Offset(2, 9), Offset(50, 0), Offset(98, 9), Offset(50, 19)],
        excludes: const <Offset>[Offset(7, 16), Offset(10, 2), Offset(84, 1), Offset(86, 18)],
      ),
    );
  });
}
