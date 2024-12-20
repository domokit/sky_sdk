// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:flutter/painting.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  group('CircularNotchedRectangle', () {
    test("guest and host don't overlap", () {
      const CircularNotchedRectangle shape = CircularNotchedRectangle();
      const Rect host = Rect.fromLTRB(0.0, 100.0, 300.0, 300.0);
      const Rect guest = Rect.fromLTWH(50.0, 50.0, 10.0, 10.0);

      final Path actualPath = shape.getOuterPath(host, guest);
      final Path expectedPath = Path()..addRect(host);

      expect(
        actualPath,
        coversSameAreaAs(expectedPath, areaToCompare: host.inflate(5.0), sampleSize: 40),
      );
    });

    test('guest center above host', () {
      const CircularNotchedRectangle shape = CircularNotchedRectangle();
      const Rect host = Rect.fromLTRB(0.0, 100.0, 300.0, 300.0);
      const Rect guest = Rect.fromLTRB(190.0, 85.0, 210.0, 105.0);

      final Path actualPath = shape.getOuterPath(host, guest);

      expect(pathDoesNotContainCircle(actualPath, guest), isTrue);
    });

    test('guest center below host', () {
      const CircularNotchedRectangle shape = CircularNotchedRectangle();
      const Rect host = Rect.fromLTRB(0.0, 100.0, 300.0, 300.0);
      const Rect guest = Rect.fromLTRB(190.0, 95.0, 210.0, 115.0);

      final Path actualPath = shape.getOuterPath(host, guest);

      expect(pathDoesNotContainCircle(actualPath, guest), isTrue);
    });

    test('inverted guest center above host', () {
      const CircularNotchedRectangle shape = CircularNotchedRectangle(inverted: true);
      const Rect host = Rect.fromLTRB(0.0, 100.0, 300.0, 300.0);
      const Rect guest = Rect.fromLTRB(190.0, 285.0, 210.0, 305.0);

      final Path actualPath = shape.getOuterPath(host, guest);

      expect(pathDoesNotContainCircle(actualPath, guest), isTrue);
    });

    test('inverted guest center below host', () {
      const CircularNotchedRectangle shape = CircularNotchedRectangle(inverted: true);
      const Rect host = Rect.fromLTRB(0.0, 100.0, 300.0, 300.0);
      const Rect guest = Rect.fromLTRB(190.0, 295.0, 210.0, 315.0);

      final Path actualPath = shape.getOuterPath(host, guest);

      expect(pathDoesNotContainCircle(actualPath, guest), isTrue);
    });

    test('no guest is ok', () {
      const Rect host = Rect.fromLTRB(0.0, 100.0, 300.0, 300.0);
      expect(
        const CircularNotchedRectangle().getOuterPath(host, null),
        coversSameAreaAs(
          Path()..addRect(host),
          areaToCompare: host.inflate(800.0),
          sampleSize: 100,
        ),
      );
    });

    test('AutomaticNotchedShape - with guest', () {
      expect(
        const AutomaticNotchedShape(
          RoundedRectangleBorder(),
          RoundedRectangleBorder(),
        ).getOuterPath(
          const Rect.fromLTWH(-200.0, -100.0, 50.0, 100.0),
          const Rect.fromLTWH(-175.0, -110.0, 100.0, 100.0),
        ),
        coversSameAreaAs(
          Path()
            ..moveTo(-200.0, -100.0)
            ..lineTo(-150.0, -100.0)
            ..lineTo(-150.0, -10.0)
            ..lineTo(-175.0, -10.0)
            ..lineTo(-175.0, 0.0)
            ..lineTo(-200.0, 0.0)
            ..close(),
          areaToCompare: const Rect.fromLTWH(-300.0, -300.0, 600.0, 600.0),
          sampleSize: 100,
        ),
      );
    }, skip: isBrowser); // https://github.com/flutter/flutter/issues/44572

    test('AutomaticNotchedShape - no guest', () {
      expect(
        const AutomaticNotchedShape(
          RoundedRectangleBorder(),
          RoundedRectangleBorder(),
        ).getOuterPath(const Rect.fromLTWH(-200.0, -100.0, 50.0, 100.0), null),
        coversSameAreaAs(
          Path()
            ..moveTo(-200.0, -100.0)
            ..lineTo(-150.0, -100.0)
            ..lineTo(-150.0, 0.0)
            ..lineTo(-200.0, 0.0)
            ..close(),
          areaToCompare: const Rect.fromLTWH(-300.0, -300.0, 600.0, 600.0),
          sampleSize: 100,
        ),
      );
    });
  });
}

bool pathDoesNotContainCircle(Path path, Rect circleBounds) {
  assert(circleBounds.width == circleBounds.height);
  final double radius = circleBounds.width / 2.0;

  for (double theta = 0.0; theta <= 2.0 * math.pi; theta += math.pi / 20.0) {
    for (double i = 0.0; i < 1; i += 0.01) {
      final double x = i * radius * math.cos(theta);
      final double y = i * radius * math.sin(theta);
      if (path.contains(Offset(x, y) + circleBounds.center)) {
        return false;
      }
    }
  }
  return true;
}
