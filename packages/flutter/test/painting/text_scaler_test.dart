// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  group('Linear TextScaler', () {
    test('equality', () {
      const TextScaler a = TextScaler.linear(3.0);
      final TextScaler b = TextScaler.noScaling.clamp(minScaleFactor: 3.0);
      // Creates a non-const TextScaler instance.
      final TextScaler c = TextScaler.linear(3.0); // ignore: prefer_const_constructors
      final TextScaler d = TextScaler.noScaling
        .clamp(minScaleFactor: 1, maxScaleFactor: 5)
        .clamp(minScaleFactor: 3, maxScaleFactor: 6);

      final List<TextScaler> list = <TextScaler>[a, b, c, d];
      for (final TextScaler lhs in list) {
        expect(list, everyElement(lhs));
      }
    });

    test('clamping', () {
      expect(TextScaler.noScaling.clamp(minScaleFactor: 3.0), const TextScaler.linear(3.0));
      expect(const TextScaler.linear(5.0).clamp(maxScaleFactor: 3.0), const TextScaler.linear(3.0));
      expect(const TextScaler.linear(5.0).clamp(maxScaleFactor: 3.0), const TextScaler.linear(3.0));
      expect(const TextScaler.linear(5.0).clamp(minScaleFactor: 3.0, maxScaleFactor: 3.0), const TextScaler.linear(3.0));
      // Asserts when min > max.
      expect(
        () => TextScaler.noScaling.clamp(minScaleFactor: 5.0, maxScaleFactor: 4.0),
        throwsA(isA<AssertionError>().having((AssertionError error) => error.toString(), 'message', contains('maxScaleFactor >= minScaleFactor'))),
      );
    });
  });

  group('SystemTextScaler', () {
    testWidgets('equality', (WidgetTester tester) async {
      addTearDown(() => tester.platformDispatcher.clearAllTestValues());

      tester.platformDispatcher.textScaleFactorTestValue = 123;
      final TextScaler scaler1 = MediaQueryData.fromView(tester.view).textScaler;
      tester.platformDispatcher.textScaleFactorTestValue = 345;
      final TextScaler scaler2 = MediaQueryData.fromView(tester.view).textScaler;
      tester.platformDispatcher.textScaleFactorTestValue = 123;
      final TextScaler scaler3 = MediaQueryData.fromView(tester.view).textScaler;
      expect(scaler1, scaler3);
      expect(scaler1, isNot(scaler2));
    });
  });
}
