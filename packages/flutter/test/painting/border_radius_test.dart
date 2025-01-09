// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/painting.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('BorderRadius control test', () {
    const Rect rect = Rect.fromLTRB(19.0, 23.0, 29.0, 31.0);
    BorderRadius borderRadius;

    borderRadius = const BorderRadius.all(Radius.elliptical(5.0, 7.0));
    expect(borderRadius, hasOneLineDescription);
    expect(borderRadius.topLeft, const Radius.elliptical(5.0, 7.0));
    expect(borderRadius.topRight, const Radius.elliptical(5.0, 7.0));
    expect(borderRadius.bottomLeft, const Radius.elliptical(5.0, 7.0));
    expect(borderRadius.bottomRight, const Radius.elliptical(5.0, 7.0));
    expect(borderRadius.toRRect(rect), RRect.fromRectXY(rect, 5.0, 7.0));

    borderRadius = BorderRadius.circular(3.0);
    expect(borderRadius, hasOneLineDescription);
    expect(borderRadius.topLeft, const Radius.elliptical(3.0, 3.0));
    expect(borderRadius.topRight, const Radius.elliptical(3.0, 3.0));
    expect(borderRadius.bottomLeft, const Radius.elliptical(3.0, 3.0));
    expect(borderRadius.bottomRight, const Radius.elliptical(3.0, 3.0));
    expect(borderRadius.toRRect(rect), RRect.fromRectXY(rect, 3.0, 3.0));

    const Radius radius1 = Radius.elliptical(89.0, 87.0);
    const Radius radius2 = Radius.elliptical(103.0, 107.0);

    borderRadius = const BorderRadius.vertical(top: radius1, bottom: radius2);
    expect(borderRadius, hasOneLineDescription);
    expect(borderRadius.topLeft, radius1);
    expect(borderRadius.topRight, radius1);
    expect(borderRadius.bottomLeft, radius2);
    expect(borderRadius.bottomRight, radius2);
    expect(
      borderRadius.toRRect(rect),
      RRect.fromRectAndCorners(
        rect,
        topLeft: radius1,
        topRight: radius1,
        bottomLeft: radius2,
        bottomRight: radius2,
      ),
    );

    borderRadius = const BorderRadius.horizontal(left: radius1, right: radius2);
    expect(borderRadius, hasOneLineDescription);
    expect(borderRadius.topLeft, radius1);
    expect(borderRadius.topRight, radius2);
    expect(borderRadius.bottomLeft, radius1);
    expect(borderRadius.bottomRight, radius2);
    expect(
      borderRadius.toRRect(rect),
      RRect.fromRectAndCorners(
        rect,
        topLeft: radius1,
        topRight: radius2,
        bottomLeft: radius1,
        bottomRight: radius2,
      ),
    );

    borderRadius = BorderRadius.zero;
    expect(borderRadius, hasOneLineDescription);
    expect(borderRadius.topLeft, Radius.zero);
    expect(borderRadius.topRight, Radius.zero);
    expect(borderRadius.bottomLeft, Radius.zero);
    expect(borderRadius.bottomRight, Radius.zero);
    expect(borderRadius.toRRect(rect), RRect.fromRectAndCorners(rect));

    borderRadius = const BorderRadius.only(topRight: radius1, bottomRight: radius2);
    expect(borderRadius, hasOneLineDescription);
    expect(borderRadius.topLeft, Radius.zero);
    expect(borderRadius.topRight, radius1);
    expect(borderRadius.bottomLeft, Radius.zero);
    expect(borderRadius.bottomRight, radius2);
    expect(
      borderRadius.toRRect(rect),
      RRect.fromRectAndCorners(rect, topRight: radius1, bottomRight: radius2),
    );

    expect(
      const BorderRadius.only(
        topLeft: Radius.elliptical(1.0, 2.0),
      ).subtract(const BorderRadius.only(topLeft: Radius.elliptical(3.0, 5.0))),
      const BorderRadius.only(topLeft: Radius.elliptical(-2.0, -3.0)),
    );
    expect(
      const BorderRadius.only(
        topRight: Radius.elliptical(1.0, 2.0),
      ).add(const BorderRadius.only(topLeft: Radius.elliptical(3.0, 5.0))),
      const BorderRadius.only(
        topLeft: Radius.elliptical(3.0, 5.0),
        topRight: Radius.elliptical(1.0, 2.0),
      ),
    );

    expect(
      const BorderRadius.only(topLeft: Radius.elliptical(1.0, 2.0)) -
          const BorderRadius.only(topLeft: Radius.elliptical(3.0, 5.0)),
      const BorderRadius.only(topLeft: Radius.elliptical(-2.0, -3.0)),
    );
    expect(
      const BorderRadius.only(topRight: Radius.elliptical(1.0, 2.0)) +
          const BorderRadius.only(topLeft: Radius.elliptical(3.0, 5.0)),
      const BorderRadius.only(
        topLeft: Radius.elliptical(3.0, 5.0),
        topRight: Radius.elliptical(1.0, 2.0),
      ),
    );

    expect(
      -const BorderRadius.only(topLeft: Radius.elliptical(1.0, 2.0)),
      const BorderRadius.only(topLeft: Radius.elliptical(-1.0, -2.0)),
    );

    expect(
      const BorderRadius.only(
            topLeft: radius1,
            topRight: radius2,
            bottomLeft: radius2,
            bottomRight: radius1,
          ) *
          0.0,
      BorderRadius.zero,
    );

    expect(BorderRadius.circular(15.0) / 10.0, BorderRadius.circular(1.5));

    expect(BorderRadius.circular(15.0) ~/ 10.0, BorderRadius.circular(1.0));

    expect(BorderRadius.circular(15.0) % 10.0, BorderRadius.circular(5.0));
  });

  test('BorderRadius.lerp() invariants', () {
    final BorderRadius a = BorderRadius.circular(10.0);
    final BorderRadius b = BorderRadius.circular(20.0);
    expect(BorderRadius.lerp(a, b, 0.25), equals(a * 1.25));
    expect(BorderRadius.lerp(a, b, 0.25), equals(b * 0.625));
    expect(BorderRadius.lerp(a, b, 0.25), equals(a + BorderRadius.circular(2.5)));
    expect(BorderRadius.lerp(a, b, 0.25), equals(b - BorderRadius.circular(7.5)));

    expect(BorderRadius.lerp(null, null, 0.25), isNull);
    expect(BorderRadius.lerp(null, b, 0.25), equals(b * 0.25));
    expect(BorderRadius.lerp(a, null, 0.25), equals(a * 0.75));
  });

  test('BorderRadius.lerp identical a,b', () {
    expect(BorderRadius.lerp(null, null, 0), null);
    const BorderRadius border = BorderRadius.zero;
    expect(identical(BorderRadius.lerp(border, border, 0.5), border), true);
  });

  test('BorderRadius.lerp() crazy', () {
    const BorderRadius a = BorderRadius.only(
      topLeft: Radius.elliptical(10.0, 20.0),
      topRight: Radius.elliptical(30.0, 40.0),
      bottomLeft: Radius.elliptical(50.0, 60.0),
    );
    const BorderRadius b = BorderRadius.only(
      topRight: Radius.elliptical(100.0, 110.0),
      bottomLeft: Radius.elliptical(120.0, 130.0),
      bottomRight: Radius.elliptical(140.0, 150.0),
    );
    const BorderRadius c = BorderRadius.only(
      topLeft: Radius.elliptical(5.0, 10.0), // 10,20 -> 0
      topRight: Radius.elliptical(65.0, 75.0), // 30,40 -> 100,110
      bottomLeft: Radius.elliptical(85.0, 95.0), // 50,60 -> 120,130
      bottomRight: Radius.elliptical(70.0, 75.0), // 0,0 -> 140,150
    );
    expect(BorderRadius.lerp(a, b, 0.5), c);
  });

  test('BorderRadiusDirectional control test', () {
    const Rect rect = Rect.fromLTRB(19.0, 23.0, 29.0, 31.0);
    BorderRadiusDirectional borderRadius;

    borderRadius = const BorderRadiusDirectional.all(Radius.elliptical(5.0, 7.0));
    expect(borderRadius, hasOneLineDescription);
    expect(borderRadius.topStart, const Radius.elliptical(5.0, 7.0));
    expect(borderRadius.topEnd, const Radius.elliptical(5.0, 7.0));
    expect(borderRadius.bottomStart, const Radius.elliptical(5.0, 7.0));
    expect(borderRadius.bottomEnd, const Radius.elliptical(5.0, 7.0));
    expect(borderRadius.resolve(TextDirection.ltr).toRRect(rect), RRect.fromRectXY(rect, 5.0, 7.0));
    expect(borderRadius.resolve(TextDirection.rtl).toRRect(rect), RRect.fromRectXY(rect, 5.0, 7.0));

    borderRadius = BorderRadiusDirectional.circular(3.0);
    expect(borderRadius, hasOneLineDescription);
    expect(borderRadius.topStart, const Radius.elliptical(3.0, 3.0));
    expect(borderRadius.topEnd, const Radius.elliptical(3.0, 3.0));
    expect(borderRadius.bottomStart, const Radius.elliptical(3.0, 3.0));
    expect(borderRadius.bottomEnd, const Radius.elliptical(3.0, 3.0));
    expect(borderRadius.resolve(TextDirection.ltr).toRRect(rect), RRect.fromRectXY(rect, 3.0, 3.0));
    expect(borderRadius.resolve(TextDirection.rtl).toRRect(rect), RRect.fromRectXY(rect, 3.0, 3.0));

    const Radius radius1 = Radius.elliptical(89.0, 87.0);
    const Radius radius2 = Radius.elliptical(103.0, 107.0);

    borderRadius = const BorderRadiusDirectional.vertical(top: radius1, bottom: radius2);
    expect(borderRadius, hasOneLineDescription);
    expect(borderRadius.topStart, radius1);
    expect(borderRadius.topEnd, radius1);
    expect(borderRadius.bottomStart, radius2);
    expect(borderRadius.bottomEnd, radius2);
    expect(
      borderRadius.resolve(TextDirection.ltr).toRRect(rect),
      RRect.fromRectAndCorners(
        rect,
        topLeft: radius1,
        topRight: radius1,
        bottomLeft: radius2,
        bottomRight: radius2,
      ),
    );
    expect(
      borderRadius.resolve(TextDirection.rtl).toRRect(rect),
      RRect.fromRectAndCorners(
        rect,
        topLeft: radius1,
        topRight: radius1,
        bottomLeft: radius2,
        bottomRight: radius2,
      ),
    );

    borderRadius = const BorderRadiusDirectional.horizontal(start: radius1, end: radius2);
    expect(borderRadius, hasOneLineDescription);
    expect(borderRadius.topStart, radius1);
    expect(borderRadius.topEnd, radius2);
    expect(borderRadius.bottomStart, radius1);
    expect(borderRadius.bottomEnd, radius2);
    expect(
      borderRadius.resolve(TextDirection.ltr).toRRect(rect),
      RRect.fromRectAndCorners(
        rect,
        topLeft: radius1,
        topRight: radius2,
        bottomLeft: radius1,
        bottomRight: radius2,
      ),
    );
    expect(
      borderRadius.resolve(TextDirection.rtl).toRRect(rect),
      RRect.fromRectAndCorners(
        rect,
        topLeft: radius2,
        topRight: radius1,
        bottomLeft: radius2,
        bottomRight: radius1,
      ),
    );

    borderRadius = BorderRadiusDirectional.zero;
    expect(borderRadius, hasOneLineDescription);
    expect(borderRadius.topStart, Radius.zero);
    expect(borderRadius.topEnd, Radius.zero);
    expect(borderRadius.bottomStart, Radius.zero);
    expect(borderRadius.bottomEnd, Radius.zero);
    expect(borderRadius.resolve(TextDirection.ltr).toRRect(rect), RRect.fromRectAndCorners(rect));
    expect(borderRadius.resolve(TextDirection.rtl).toRRect(rect), RRect.fromRectAndCorners(rect));

    borderRadius = const BorderRadiusDirectional.only(topEnd: radius1, bottomEnd: radius2);
    expect(borderRadius, hasOneLineDescription);
    expect(borderRadius.topStart, Radius.zero);
    expect(borderRadius.topEnd, radius1);
    expect(borderRadius.bottomStart, Radius.zero);
    expect(borderRadius.bottomEnd, radius2);
    expect(
      borderRadius.resolve(TextDirection.ltr).toRRect(rect),
      RRect.fromRectAndCorners(rect, topRight: radius1, bottomRight: radius2),
    );
    expect(
      borderRadius.resolve(TextDirection.rtl).toRRect(rect),
      RRect.fromRectAndCorners(rect, topLeft: radius1, bottomLeft: radius2),
    );

    expect(
      const BorderRadiusDirectional.only(
        topStart: Radius.elliptical(1.0, 2.0),
      ).subtract(const BorderRadiusDirectional.only(topStart: Radius.elliptical(3.0, 5.0))),
      const BorderRadiusDirectional.only(topStart: Radius.elliptical(-2.0, -3.0)),
    );
    expect(
      const BorderRadiusDirectional.only(
        topEnd: Radius.elliptical(1.0, 2.0),
      ).add(const BorderRadiusDirectional.only(topStart: Radius.elliptical(3.0, 5.0))),
      const BorderRadiusDirectional.only(
        topStart: Radius.elliptical(3.0, 5.0),
        topEnd: Radius.elliptical(1.0, 2.0),
      ),
    );

    expect(
      const BorderRadiusDirectional.only(topStart: Radius.elliptical(1.0, 2.0)) -
          const BorderRadiusDirectional.only(topStart: Radius.elliptical(3.0, 5.0)),
      const BorderRadiusDirectional.only(topStart: Radius.elliptical(-2.0, -3.0)),
    );
    expect(
      const BorderRadiusDirectional.only(topEnd: Radius.elliptical(1.0, 2.0)) +
          const BorderRadiusDirectional.only(topStart: Radius.elliptical(3.0, 5.0)),
      const BorderRadiusDirectional.only(
        topStart: Radius.elliptical(3.0, 5.0),
        topEnd: Radius.elliptical(1.0, 2.0),
      ),
    );

    expect(
      -const BorderRadiusDirectional.only(topStart: Radius.elliptical(1.0, 2.0)),
      const BorderRadiusDirectional.only(topStart: Radius.elliptical(-1.0, -2.0)),
    );

    expect(
      const BorderRadiusDirectional.only(
            topStart: radius1,
            topEnd: radius2,
            bottomStart: radius2,
            bottomEnd: radius1,
          ) *
          0.0,
      BorderRadiusDirectional.zero,
    );

    expect(BorderRadiusDirectional.circular(15.0) / 10.0, BorderRadiusDirectional.circular(1.5));

    expect(BorderRadiusDirectional.circular(15.0) ~/ 10.0, BorderRadiusDirectional.circular(1.0));

    expect(BorderRadiusDirectional.circular(15.0) % 10.0, BorderRadiusDirectional.circular(5.0));
  });

  test('BorderRadiusDirectional.lerp() invariants', () {
    final BorderRadiusDirectional a = BorderRadiusDirectional.circular(10.0);
    final BorderRadiusDirectional b = BorderRadiusDirectional.circular(20.0);
    expect(BorderRadiusDirectional.lerp(a, b, 0.25), equals(a * 1.25));
    expect(BorderRadiusDirectional.lerp(a, b, 0.25), equals(b * 0.625));
    expect(
      BorderRadiusDirectional.lerp(a, b, 0.25),
      equals(a + BorderRadiusDirectional.circular(2.5)),
    );
    expect(
      BorderRadiusDirectional.lerp(a, b, 0.25),
      equals(b - BorderRadiusDirectional.circular(7.5)),
    );

    expect(BorderRadiusDirectional.lerp(null, null, 0.25), isNull);
    expect(BorderRadiusDirectional.lerp(null, b, 0.25), equals(b * 0.25));
    expect(BorderRadiusDirectional.lerp(a, null, 0.25), equals(a * 0.75));
  });

  test('BorderRadiusDirectional.lerp identical a,b', () {
    expect(BorderRadiusDirectional.lerp(null, null, 0), null);
    const BorderRadiusDirectional border = BorderRadiusDirectional.zero;
    expect(identical(BorderRadiusDirectional.lerp(border, border, 0.5), border), true);
  });

  test('BorderRadiusDirectional.lerp() crazy', () {
    const BorderRadiusDirectional a = BorderRadiusDirectional.only(
      topStart: Radius.elliptical(10.0, 20.0),
      topEnd: Radius.elliptical(30.0, 40.0),
      bottomStart: Radius.elliptical(50.0, 60.0),
    );
    const BorderRadiusDirectional b = BorderRadiusDirectional.only(
      topEnd: Radius.elliptical(100.0, 110.0),
      bottomStart: Radius.elliptical(120.0, 130.0),
      bottomEnd: Radius.elliptical(140.0, 150.0),
    );
    const BorderRadiusDirectional c = BorderRadiusDirectional.only(
      topStart: Radius.elliptical(5.0, 10.0), // 10,20 -> 0
      topEnd: Radius.elliptical(65.0, 75.0), // 30,40 -> 100,110
      bottomStart: Radius.elliptical(85.0, 95.0), // 50,60 -> 120,130
      bottomEnd: Radius.elliptical(70.0, 75.0), // 0,0 -> 140,150
    );
    expect(BorderRadiusDirectional.lerp(a, b, 0.5), c);
  });

  test('BorderRadiusGeometry.lerp()', () {
    const BorderRadius a = BorderRadius.only(
      topLeft: Radius.elliptical(10.0, 20.0),
      topRight: Radius.elliptical(30.0, 40.0),
      bottomLeft: Radius.elliptical(50.0, 60.0),
    );
    const BorderRadiusDirectional b = BorderRadiusDirectional.only(
      topEnd: Radius.elliptical(100.0, 110.0),
      bottomStart: Radius.elliptical(120.0, 130.0),
      bottomEnd: Radius.elliptical(140.0, 150.0),
    );
    const BorderRadius ltr = BorderRadius.only(
      topLeft: Radius.elliptical(5.0, 10.0), // 10,20 -> 0
      topRight: Radius.elliptical(65.0, 75.0), // 30,40 -> 100,110
      bottomLeft: Radius.elliptical(85.0, 95.0), // 50,60 -> 120,130
      bottomRight: Radius.elliptical(70.0, 75.0), // 0,0 -> 140,150
    );
    const BorderRadius rtl = BorderRadius.only(
      topLeft: Radius.elliptical(55.0, 65.0), // 10,20 -> 100,110
      topRight: Radius.elliptical(15.0, 20.0), // 30,40 -> 0,0
      bottomLeft: Radius.elliptical(95.0, 105.0), // 50,60 -> 140,150
      bottomRight: Radius.elliptical(60.0, 65.0), // 0,0 -> 120,130
    );
    expect(BorderRadiusGeometry.lerp(a, b, 0.5)!.resolve(TextDirection.ltr), ltr);
    expect(BorderRadiusGeometry.lerp(a, b, 0.5)!.resolve(TextDirection.rtl), rtl);
    expect(BorderRadiusGeometry.lerp(a, b, 0.0)!.resolve(TextDirection.ltr), a);
    expect(
      BorderRadiusGeometry.lerp(a, b, 1.0)!.resolve(TextDirection.rtl),
      b.resolve(TextDirection.rtl),
    );
  });

  test('BorderRadiusGeometry.lerp identical a,b', () {
    expect(BorderRadiusDirectional.lerp(null, null, 0), null);
    const BorderRadiusGeometry border = BorderRadius.zero;
    expect(identical(BorderRadiusGeometry.lerp(border, border, 0.5), border), true);
  });

  test('BorderRadiusGeometry subtract', () {
    const BorderRadius a = BorderRadius.only(
      topLeft: Radius.elliptical(10.0, 20.0),
      topRight: Radius.elliptical(30.0, 40.0),
      bottomLeft: Radius.elliptical(50.0, 60.0),
    );
    const BorderRadiusDirectional b = BorderRadiusDirectional.only(
      topEnd: Radius.elliptical(100.0, 110.0),
      bottomStart: Radius.elliptical(120.0, 130.0),
      bottomEnd: Radius.elliptical(140.0, 150.0),
    );
    expect(
      a.subtract(b).resolve(TextDirection.ltr),
      BorderRadius.only(
        topLeft: const Radius.elliptical(10.0, 20.0) - Radius.zero,
        topRight: const Radius.elliptical(30.0, 40.0) - const Radius.elliptical(100.0, 110.0),
        bottomLeft: const Radius.elliptical(50.0, 60.0) - const Radius.elliptical(120.0, 130.0),
        bottomRight: Radius.zero - const Radius.elliptical(140.0, 150.0),
      ),
    );
    expect(
      a.subtract(b).resolve(TextDirection.rtl),
      BorderRadius.only(
        topLeft: const Radius.elliptical(10.0, 20.0) - const Radius.elliptical(100.0, 110.0),
        topRight: const Radius.elliptical(30.0, 40.0) - Radius.zero,
        bottomLeft: const Radius.elliptical(50.0, 60.0) - const Radius.elliptical(140.0, 150.0),
        bottomRight: Radius.zero - const Radius.elliptical(120.0, 130.0),
      ),
    );
  });

  test('BorderRadiusGeometry add', () {
    const BorderRadius a = BorderRadius.only(
      topLeft: Radius.elliptical(10.0, 20.0),
      topRight: Radius.elliptical(30.0, 40.0),
      bottomLeft: Radius.elliptical(50.0, 60.0),
    );
    const BorderRadiusDirectional b = BorderRadiusDirectional.only(
      topEnd: Radius.elliptical(100.0, 110.0),
      bottomStart: Radius.elliptical(120.0, 130.0),
      bottomEnd: Radius.elliptical(140.0, 150.0),
    );
    expect(
      a.add(b).resolve(TextDirection.ltr),
      BorderRadius.only(
        topLeft: const Radius.elliptical(10.0, 20.0) + Radius.zero,
        topRight: const Radius.elliptical(30.0, 40.0) + const Radius.elliptical(100.0, 110.0),
        bottomLeft: const Radius.elliptical(50.0, 60.0) + const Radius.elliptical(120.0, 130.0),
        bottomRight: Radius.zero + const Radius.elliptical(140.0, 150.0),
      ),
    );
    expect(
      a.add(b).resolve(TextDirection.rtl),
      BorderRadius.only(
        topLeft: const Radius.elliptical(10.0, 20.0) + const Radius.elliptical(100.0, 110.0),
        topRight: const Radius.elliptical(30.0, 40.0) + Radius.zero,
        bottomLeft: const Radius.elliptical(50.0, 60.0) + const Radius.elliptical(140.0, 150.0),
        bottomRight: Radius.zero + const Radius.elliptical(120.0, 130.0),
      ),
    );
  });

  test('BorderRadiusGeometry add and multiply', () {
    const BorderRadius a = BorderRadius.only(
      topLeft: Radius.elliptical(10.0, 20.0),
      topRight: Radius.elliptical(30.0, 40.0),
      bottomLeft: Radius.elliptical(50.0, 60.0),
    );
    const BorderRadiusDirectional b = BorderRadiusDirectional.only(
      topEnd: Radius.elliptical(100.0, 110.0),
      bottomStart: Radius.elliptical(120.0, 130.0),
      bottomEnd: Radius.elliptical(140.0, 150.0),
    );
    expect(
      (a.add(b) * 0.5).resolve(TextDirection.ltr),
      BorderRadius.only(
        topLeft: (const Radius.elliptical(10.0, 20.0) + Radius.zero) / 2.0,
        topRight:
            (const Radius.elliptical(30.0, 40.0) + const Radius.elliptical(100.0, 110.0)) / 2.0,
        bottomLeft:
            (const Radius.elliptical(50.0, 60.0) + const Radius.elliptical(120.0, 130.0)) / 2.0,
        bottomRight: (Radius.zero + const Radius.elliptical(140.0, 150.0)) / 2.0,
      ),
    );
    expect(
      (a.add(b) * 0.5).resolve(TextDirection.rtl),
      BorderRadius.only(
        topLeft:
            (const Radius.elliptical(10.0, 20.0) + const Radius.elliptical(100.0, 110.0)) / 2.0,
        topRight: (const Radius.elliptical(30.0, 40.0) + Radius.zero) / 2.0,
        bottomLeft:
            (const Radius.elliptical(50.0, 60.0) + const Radius.elliptical(140.0, 150.0)) / 2.0,
        bottomRight: (Radius.zero + const Radius.elliptical(120.0, 130.0)) / 2.0,
      ),
    );
  });

  test('BorderRadiusGeometry add and subtract', () {
    const BorderRadius a = BorderRadius.only(topLeft: Radius.elliptical(300.0, 500.0));
    const BorderRadiusDirectional b = BorderRadiusDirectional.only(
      topEnd: Radius.elliptical(30.0, 50.0),
    );
    const BorderRadius c = BorderRadius.only(bottomLeft: Radius.elliptical(3.0, 5.0));

    const BorderRadius ltr = BorderRadius.only(
      topLeft: Radius.elliptical(300.0, 500.0), // tL + 0 - 0
      topRight: Radius.elliptical(30.0, 50.0), // 0 + tE - 0
      bottomLeft: Radius.elliptical(-3.0, -5.0), // 0 + 0 - 0
    );
    const BorderRadius rtl = BorderRadius.only(
      topLeft: Radius.elliptical(330.0, 550.0), // 0 + 0 - 0
      bottomLeft: Radius.elliptical(-3.0, -5.0), // 0 + 0 - 0
    );
    expect(a.add(b.subtract(c)).resolve(TextDirection.ltr), ltr);
    expect(a.add(b.subtract(c)).resolve(TextDirection.rtl), rtl);
  });

  test('BorderRadiusGeometry add and subtract, more', () {
    const BorderRadius a = BorderRadius.only(
      topLeft: Radius.elliptical(300.0, 300.0),
      topRight: Radius.elliptical(500.0, 500.0),
      bottomLeft: Radius.elliptical(700.0, 700.0),
      bottomRight: Radius.elliptical(900.0, 900.0),
    );
    const BorderRadiusDirectional b = BorderRadiusDirectional.only(
      topStart: Radius.elliptical(30.0, 30.0),
      topEnd: Radius.elliptical(50.0, 50.0),
      bottomStart: Radius.elliptical(70.0, 70.0),
      bottomEnd: Radius.elliptical(90.0, 90.0),
    );
    const BorderRadius c = BorderRadius.only(
      topLeft: Radius.elliptical(3.0, 3.0),
      topRight: Radius.elliptical(5.0, 5.0),
      bottomLeft: Radius.elliptical(7.0, 7.0),
      bottomRight: Radius.elliptical(9.0, 9.0),
    );

    const BorderRadius ltr = BorderRadius.only(
      topLeft: Radius.elliptical(327.0, 327.0), // tL + tS - tL
      topRight: Radius.elliptical(545.0, 545.0), // tR + tE - tR
      bottomLeft: Radius.elliptical(763.0, 763.0), // bL + bS - bL
      bottomRight: Radius.elliptical(981.0, 981.0), // bR + bE - bR
    );
    const BorderRadius rtl = BorderRadius.only(
      topLeft: Radius.elliptical(347.0, 347.0), // tL + tE - tL
      topRight: Radius.elliptical(525.0, 525.0), // tR + TS - tR
      bottomLeft: Radius.elliptical(783.0, 783.0), // bL + bE + bL
      bottomRight: Radius.elliptical(961.0, 961.0), // bR + bS - bR
    );
    expect(a.add(b.subtract(c)).resolve(TextDirection.ltr), ltr);
    expect(a.add(b.subtract(c)).resolve(TextDirection.rtl), rtl);
  });

  test('BorderRadiusGeometry operators', () {
    const BorderRadius a = BorderRadius.only(
      topLeft: Radius.elliptical(10.0, 20.0),
      topRight: Radius.elliptical(30.0, 40.0),
      bottomLeft: Radius.elliptical(50.0, 60.0),
    );
    const BorderRadiusDirectional b = BorderRadiusDirectional.only(
      topEnd: Radius.elliptical(100.0, 110.0),
      bottomStart: Radius.elliptical(120.0, 130.0),
      bottomEnd: Radius.elliptical(140.0, 150.0),
    );

    const BorderRadius ltr = BorderRadius.only(
      topLeft: Radius.elliptical(5.0, 10.0), // 10,20 -> 0
      topRight: Radius.elliptical(65.0, 75.0), // 30,40 -> 100,110
      bottomLeft: Radius.elliptical(85.0, 95.0), // 50,60 -> 120,130
      bottomRight: Radius.elliptical(70.0, 75.0), // 0,0 -> 140,150
    );
    const BorderRadius rtl = BorderRadius.only(
      topLeft: Radius.elliptical(55.0, 65.0), // 10,20 -> 100,110
      topRight: Radius.elliptical(15.0, 20.0), // 30,40 -> 0,0
      bottomLeft: Radius.elliptical(95.0, 105.0), // 50,60 -> 140,150
      bottomRight: Radius.elliptical(60.0, 65.0), // 0,0 -> 120,130
    );
    expect(a.add(b.subtract(a) * 0.5).resolve(TextDirection.ltr), ltr);
    expect(a.add(b.subtract(a) * 0.5).resolve(TextDirection.rtl), rtl);
    expect(a.add(b.subtract(a) * 0.0).resolve(TextDirection.ltr), a);
    expect(a.add(b.subtract(a) * 1.0).resolve(TextDirection.rtl), b.resolve(TextDirection.rtl));
  });

  test('BorderRadius copyWith, merge, ==, hashCode basics', () {
    const BorderRadius firstRadius = BorderRadius.all(Radius.circular(5.0));
    final BorderRadius secondRadius = firstRadius.copyWith();
    expect(firstRadius, secondRadius);
    expect(firstRadius.hashCode, secondRadius.hashCode);
  });

  test('BorderRadius copyWith parameters', () {
    const Radius radius = Radius.circular(10);
    const BorderRadius borderRadius = BorderRadius.all(radius);
    expect(borderRadius.copyWith(topLeft: Radius.zero).topLeft, Radius.zero);
    expect(borderRadius.copyWith(topLeft: Radius.zero).copyWith(topLeft: radius), borderRadius);
    expect(borderRadius.copyWith(topRight: Radius.zero).topRight, Radius.zero);
    expect(borderRadius.copyWith(topRight: Radius.zero).copyWith(topRight: radius), borderRadius);
    expect(borderRadius.copyWith(bottomLeft: Radius.zero).bottomLeft, Radius.zero);
    expect(
      borderRadius.copyWith(bottomLeft: Radius.zero).copyWith(bottomLeft: radius),
      borderRadius,
    );
    expect(borderRadius.copyWith(bottomRight: Radius.zero).bottomRight, Radius.zero);
    expect(
      borderRadius.copyWith(bottomRight: Radius.zero).copyWith(bottomRight: radius),
      borderRadius,
    );
  });
}
