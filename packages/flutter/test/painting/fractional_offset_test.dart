// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/painting.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('FractionalOffset control test', () {
    const FractionalOffset a = FractionalOffset(0.5, 0.25);
    const FractionalOffset b = FractionalOffset(1.25, 0.75);

    expect(a, hasOneLineDescription);
    expect(a.hashCode, equals(const FractionalOffset(0.5, 0.25).hashCode));
    expect(a.toString(), equals('FractionalOffset(0.5, 0.3)'));

    expect(-a, const FractionalOffset(-0.5, -0.25));
    expect(a - b, const FractionalOffset(-0.75, -0.5));
    expect(a + b, const FractionalOffset(1.75, 1.0));
    expect(a * 2.0, FractionalOffset.centerRight);
    expect(a / 2.0, const FractionalOffset(0.25, 0.125));
    expect(a ~/ 2.0, FractionalOffset.topLeft);
    expect(a % 5.0, const FractionalOffset(0.5, 0.25));
  });

  test('FractionalOffset.lerp()', () {
    const FractionalOffset a = FractionalOffset.topLeft;
    const FractionalOffset b = FractionalOffset.topCenter;
    expect(FractionalOffset.lerp(a, b, 0.25), equals(const FractionalOffset(0.125, 0.0)));

    expect(FractionalOffset.lerp(null, null, 0.25), isNull);
    expect(FractionalOffset.lerp(null, b, 0.25), equals(const FractionalOffset(0.5, 0.5 - 0.125)));
    expect(FractionalOffset.lerp(a, null, 0.25), equals(const FractionalOffset(0.125, 0.125)));
  });

  test('FractionalOffset.lerp identical a,b', () {
    expect(FractionalOffset.lerp(null, null, 0), null);
    const FractionalOffset decoration = FractionalOffset(1, 2);
    expect(identical(FractionalOffset.lerp(decoration, decoration, 0.5), decoration), true);
  });

  test('FractionalOffset.fromOffsetAndSize()', () {
    final FractionalOffset a = FractionalOffset.fromOffsetAndSize(
      const Offset(100.0, 100.0),
      const Size(200.0, 400.0),
    );
    expect(a, const FractionalOffset(0.5, 0.25));
  });

  test('FractionalOffset.fromOffsetAndRect()', () {
    final FractionalOffset a = FractionalOffset.fromOffsetAndRect(
      const Offset(150.0, 120.0),
      const Rect.fromLTWH(50.0, 20.0, 200.0, 400.0),
    );
    expect(a, const FractionalOffset(0.5, 0.25));
  });
}
