// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:flutter/painting.dart';
import 'package:flutter_test/flutter_test.dart';

void approxExpect(Alignment a, Alignment b) {
  expect(a.x, moreOrLessEquals(b.x));
  expect(a.y, moreOrLessEquals(b.y));
}

void main() {
  test('Alignment control test', () {
    const Alignment alignment = Alignment(0.5, 0.25);

    expect(alignment, hasOneLineDescription);
    expect(alignment.hashCode, equals(const Alignment(0.5, 0.25).hashCode));

    expect(alignment / 2.0, const Alignment(0.25, 0.125));
    expect(alignment ~/ 2.0, Alignment.center);
    expect(alignment % 5.0, const Alignment(0.5, 0.25));
  });

  test('Alignment.lerp()', () {
    const Alignment a = Alignment.topLeft;
    const Alignment b = Alignment.topCenter;
    expect(Alignment.lerp(a, b, 0.25), equals(const Alignment(-0.75, -1.0)));

    expect(Alignment.lerp(null, null, 0.25), isNull);
    expect(Alignment.lerp(null, b, 0.25), equals(const Alignment(0.0, -0.25)));
    expect(Alignment.lerp(a, null, 0.25), equals(const Alignment(-0.75, -0.75)));
  });

  test('Alignment.lerp identical a,b', () {
    expect(Alignment.lerp(null, null, 0), null);
    const Alignment alignment = Alignment.topLeft;
    expect(identical(Alignment.lerp(alignment, alignment, 0.5), alignment), true);
  });

  test('AlignmentGeometry.lerp identical a,b', () {
    expect(AlignmentGeometry.lerp(null, null, 0), null);
    const AlignmentGeometry alignment = Alignment.topLeft;
    expect(identical(AlignmentGeometry.lerp(alignment, alignment, 0.5), alignment), true);
  });

  test('AlignmentDirectional.lerp identical a,b', () {
    expect(AlignmentDirectional.lerp(null, null, 0), null);
    const AlignmentDirectional alignment = AlignmentDirectional.topStart;
    expect(identical(AlignmentDirectional.lerp(alignment, alignment, 0.5), alignment), true);
  });

  test('AlignmentGeometry invariants', () {
    const AlignmentDirectional topStart = AlignmentDirectional.topStart;
    const AlignmentDirectional topEnd = AlignmentDirectional.topEnd;
    const Alignment center = Alignment.center;
    const Alignment topLeft = Alignment.topLeft;
    const Alignment topRight = Alignment.topRight;
    final List<double> numbers = <double>[0.0, 1.0, -1.0, 2.0, 0.25, 0.5, 100.0, -999.75];

    expect((topEnd * 0.0).add(topRight * 0.0), center);
    expect(topEnd.add(topRight) * 0.0, (topEnd * 0.0).add(topRight * 0.0));
    expect(topStart.add(topLeft), topLeft.add(topStart));
    expect(
      topStart.add(topLeft).resolve(TextDirection.ltr),
      (topStart.resolve(TextDirection.ltr)) + topLeft,
    );
    expect(
      topStart.add(topLeft).resolve(TextDirection.rtl),
      (topStart.resolve(TextDirection.rtl)) + topLeft,
    );
    expect(
      topStart.add(topLeft).resolve(TextDirection.ltr),
      topStart.resolve(TextDirection.ltr).add(topLeft),
    );
    expect(
      topStart.add(topLeft).resolve(TextDirection.rtl),
      topStart.resolve(TextDirection.rtl).add(topLeft),
    );
    expect(topStart.resolve(TextDirection.ltr), topLeft);
    expect(topStart.resolve(TextDirection.rtl), topRight);
    expect(topEnd * 0.0, center);
    expect(topLeft * 0.0, center);
    expect(topStart * 1.0, topStart);
    expect(topEnd * 1.0, topEnd);
    expect(topLeft * 1.0, topLeft);
    expect(topRight * 1.0, topRight);
    for (final double n in numbers) {
      expect((topStart * n).add(topStart), topStart * (n + 1.0));
      expect((topEnd * n).add(topEnd), topEnd * (n + 1.0));
      for (final double m in numbers) {
        expect((topStart * n).add(topStart * m), topStart * (n + m));
      }
    }
    expect(topStart + topStart + topStart, topStart * 3.0); // without using "add"
    for (final TextDirection x in TextDirection.values) {
      expect((topEnd * 0.0).add(topRight * 0.0).resolve(x), center.add(center).resolve(x));
      expect((topEnd * 0.0).add(topLeft).resolve(x), center.add(topLeft).resolve(x));
      expect(
        (topEnd * 0.0).resolve(x).add(topLeft.resolve(x)),
        center.resolve(x).add(topLeft.resolve(x)),
      );
      expect((topEnd * 0.0).resolve(x).add(topLeft), center.resolve(x).add(topLeft));
      expect((topEnd * 0.0).resolve(x), center.resolve(x));
    }
    expect(topStart, isNot(topLeft));
    expect(topEnd, isNot(topLeft));
    expect(topStart, isNot(topRight));
    expect(topEnd, isNot(topRight));
    expect(topStart.add(topLeft), isNot(topLeft));
    expect(topStart.add(topLeft), isNot(topStart));
  });

  test('AlignmentGeometry.resolve()', () {
    expect(
      const AlignmentDirectional(0.25, 0.3).resolve(TextDirection.ltr),
      const Alignment(0.25, 0.3),
    );
    expect(
      const AlignmentDirectional(0.25, 0.3).resolve(TextDirection.rtl),
      const Alignment(-0.25, 0.3),
    );
    expect(
      const AlignmentDirectional(-0.25, 0.3).resolve(TextDirection.ltr),
      const Alignment(-0.25, 0.3),
    );
    expect(
      const AlignmentDirectional(-0.25, 0.3).resolve(TextDirection.rtl),
      const Alignment(0.25, 0.3),
    );
    expect(
      const AlignmentDirectional(1.25, 0.3).resolve(TextDirection.ltr),
      const Alignment(1.25, 0.3),
    );
    expect(
      const AlignmentDirectional(1.25, 0.3).resolve(TextDirection.rtl),
      const Alignment(-1.25, 0.3),
    );
    expect(
      const AlignmentDirectional(0.5, -0.3).resolve(TextDirection.ltr),
      const Alignment(0.5, -0.3),
    );
    expect(
      const AlignmentDirectional(0.5, -0.3).resolve(TextDirection.rtl),
      const Alignment(-0.5, -0.3),
    );
    expect(AlignmentDirectional.center.resolve(TextDirection.ltr), Alignment.center);
    expect(AlignmentDirectional.center.resolve(TextDirection.rtl), Alignment.center);
    expect(AlignmentDirectional.bottomEnd.resolve(TextDirection.ltr), Alignment.bottomRight);
    expect(AlignmentDirectional.bottomEnd.resolve(TextDirection.rtl), Alignment.bottomLeft);
    expect(AlignmentDirectional(nonconst(1.0), 2.0), AlignmentDirectional(nonconst(1.0), 2.0));
    expect(const AlignmentDirectional(1.0, 2.0), isNot(const AlignmentDirectional(2.0, 1.0)));
    expect(
      AlignmentDirectional.centerStart.resolve(TextDirection.ltr),
      AlignmentDirectional.centerEnd.resolve(TextDirection.rtl),
    );
    expect(
      AlignmentDirectional.centerStart.resolve(TextDirection.ltr),
      isNot(AlignmentDirectional.centerEnd.resolve(TextDirection.ltr)),
    );
    expect(
      AlignmentDirectional.centerEnd.resolve(TextDirection.ltr),
      isNot(AlignmentDirectional.centerEnd.resolve(TextDirection.rtl)),
    );
  });

  test('AlignmentGeometry.lerp ad hoc tests', () {
    final AlignmentGeometry mixed1 = const Alignment(
      10.0,
      20.0,
    ).add(const AlignmentDirectional(30.0, 50.0));
    final AlignmentGeometry mixed2 = const Alignment(
      70.0,
      110.0,
    ).add(const AlignmentDirectional(130.0, 170.0));
    final AlignmentGeometry mixed3 = const Alignment(
      25.0,
      42.5,
    ).add(const AlignmentDirectional(55.0, 80.0));

    for (final TextDirection direction in TextDirection.values) {
      expect(
        AlignmentGeometry.lerp(mixed1, mixed2, 0.0)!.resolve(direction),
        mixed1.resolve(direction),
      );
      expect(
        AlignmentGeometry.lerp(mixed1, mixed2, 1.0)!.resolve(direction),
        mixed2.resolve(direction),
      );
      expect(
        AlignmentGeometry.lerp(mixed1, mixed2, 0.25)!.resolve(direction),
        mixed3.resolve(direction),
      );
    }
  });

  test('lerp commutes with resolve', () {
    final List<AlignmentGeometry?> offsets = <AlignmentGeometry?>[
      Alignment.topLeft,
      Alignment.topCenter,
      Alignment.topRight,
      AlignmentDirectional.topStart,
      AlignmentDirectional.topCenter,
      AlignmentDirectional.topEnd,
      Alignment.centerLeft,
      Alignment.center,
      Alignment.centerRight,
      AlignmentDirectional.centerStart,
      AlignmentDirectional.center,
      AlignmentDirectional.centerEnd,
      Alignment.bottomLeft,
      Alignment.bottomCenter,
      Alignment.bottomRight,
      AlignmentDirectional.bottomStart,
      AlignmentDirectional.bottomCenter,
      AlignmentDirectional.bottomEnd,
      const Alignment(-1.0, 0.65),
      const AlignmentDirectional(-1.0, 0.45),
      const AlignmentDirectional(0.125, 0.625),
      const Alignment(0.25, 0.875),
      const Alignment(0.0625, 0.5625).add(const AlignmentDirectional(0.1875, 0.6875)),
      const AlignmentDirectional(2.0, 3.0),
      const Alignment(2.0, 3.0),
      const Alignment(2.0, 3.0).add(const AlignmentDirectional(5.0, 3.0)),
      const Alignment(10.0, 20.0).add(const AlignmentDirectional(30.0, 50.0)),
      const Alignment(70.0, 110.0).add(const AlignmentDirectional(130.0, 170.0)),
      const Alignment(25.0, 42.5).add(const AlignmentDirectional(55.0, 80.0)),
      null,
    ];

    final List<double> times = <double>[0.25, 0.5, 0.75];

    for (final TextDirection direction in TextDirection.values) {
      final Alignment defaultValue = AlignmentDirectional.center.resolve(direction);
      for (final AlignmentGeometry? a in offsets) {
        final Alignment resolvedA = a?.resolve(direction) ?? defaultValue;
        for (final AlignmentGeometry? b in offsets) {
          final Alignment resolvedB = b?.resolve(direction) ?? defaultValue;
          approxExpect(Alignment.lerp(resolvedA, resolvedB, 0.0)!, resolvedA);
          approxExpect(Alignment.lerp(resolvedA, resolvedB, 1.0)!, resolvedB);
          approxExpect(
            (AlignmentGeometry.lerp(a, b, 0.0) ?? defaultValue).resolve(direction),
            resolvedA,
          );
          approxExpect(
            (AlignmentGeometry.lerp(a, b, 1.0) ?? defaultValue).resolve(direction),
            resolvedB,
          );
          for (final double t in times) {
            assert(t > 0.0);
            assert(t < 1.0);
            final Alignment value = (AlignmentGeometry.lerp(a, b, t) ?? defaultValue).resolve(
              direction,
            );
            approxExpect(value, Alignment.lerp(resolvedA, resolvedB, t)!);
            final double minDX = math.min(resolvedA.x, resolvedB.x);
            final double maxDX = math.max(resolvedA.x, resolvedB.x);
            final double minDY = math.min(resolvedA.y, resolvedB.y);
            final double maxDY = math.max(resolvedA.y, resolvedB.y);
            expect(value.x, inInclusiveRange(minDX, maxDX));
            expect(value.y, inInclusiveRange(minDY, maxDY));
          }
        }
      }
    }
  });

  test('AlignmentGeometry add/subtract', () {
    const AlignmentGeometry directional = AlignmentDirectional(1.0, 2.0);
    const AlignmentGeometry normal = Alignment(3.0, 5.0);
    expect(directional.add(normal).resolve(TextDirection.ltr), const Alignment(4.0, 7.0));
    expect(directional.add(normal).resolve(TextDirection.rtl), const Alignment(2.0, 7.0));
    expect(normal.add(normal), normal * 2.0);
    expect(directional.add(directional), directional * 2.0);
  });

  test('AlignmentGeometry operators', () {
    expect(const AlignmentDirectional(1.0, 2.0) * 2.0, const AlignmentDirectional(2.0, 4.0));
    expect(const AlignmentDirectional(1.0, 2.0) / 2.0, const AlignmentDirectional(0.5, 1.0));
    expect(const AlignmentDirectional(1.0, 2.0) % 2.0, AlignmentDirectional.centerEnd);
    expect(const AlignmentDirectional(1.0, 2.0) ~/ 2.0, AlignmentDirectional.bottomCenter);
    for (final TextDirection direction in TextDirection.values) {
      expect(
        Alignment.center.add(const AlignmentDirectional(1.0, 2.0) * 2.0).resolve(direction),
        const AlignmentDirectional(2.0, 4.0).resolve(direction),
      );
      expect(
        Alignment.center.add(const AlignmentDirectional(1.0, 2.0) / 2.0).resolve(direction),
        const AlignmentDirectional(0.5, 1.0).resolve(direction),
      );
      expect(
        Alignment.center.add(const AlignmentDirectional(1.0, 2.0) % 2.0).resolve(direction),
        AlignmentDirectional.centerEnd.resolve(direction),
      );
      expect(
        Alignment.center.add(const AlignmentDirectional(1.0, 2.0) ~/ 2.0).resolve(direction),
        AlignmentDirectional.bottomCenter.resolve(direction),
      );
    }
    expect(const Alignment(1.0, 2.0) * 2.0, const Alignment(2.0, 4.0));
    expect(const Alignment(1.0, 2.0) / 2.0, const Alignment(0.5, 1.0));
    expect(const Alignment(1.0, 2.0) % 2.0, Alignment.centerRight);
    expect(const Alignment(1.0, 2.0) ~/ 2.0, Alignment.bottomCenter);
  });

  test('AlignmentGeometry operators', () {
    expect(const Alignment(1.0, 2.0) + const Alignment(3.0, 5.0), const Alignment(4.0, 7.0));
    expect(const Alignment(1.0, 2.0) - const Alignment(3.0, 5.0), const Alignment(-2.0, -3.0));
    expect(
      const AlignmentDirectional(1.0, 2.0) + const AlignmentDirectional(3.0, 5.0),
      const AlignmentDirectional(4.0, 7.0),
    );
    expect(
      const AlignmentDirectional(1.0, 2.0) - const AlignmentDirectional(3.0, 5.0),
      const AlignmentDirectional(-2.0, -3.0),
    );
  });

  test('AlignmentGeometry toString', () {
    expect(const Alignment(1.0001, 2.0001).toString(), 'Alignment(1.0, 2.0)');
    expect(Alignment.center.toString(), 'Alignment.center');
    expect(
      Alignment.bottomLeft.add(AlignmentDirectional.centerEnd).toString(),
      'Alignment.bottomLeft + AlignmentDirectional.centerEnd',
    );
    expect(const Alignment(0.0001, 0.0001).toString(), 'Alignment(0.0, 0.0)');
    expect(Alignment.center.toString(), 'Alignment.center');
    expect(AlignmentDirectional.center.toString(), 'AlignmentDirectional.center');
    expect(
      Alignment.bottomRight.add(AlignmentDirectional.bottomEnd).toString(),
      'Alignment(1.0, 2.0) + AlignmentDirectional.centerEnd',
    );
  });
}
