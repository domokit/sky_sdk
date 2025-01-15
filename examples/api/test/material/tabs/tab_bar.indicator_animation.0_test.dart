// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/tabs/tab_bar.indicator_animation.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('TabBar.indicatorAnimation can customize tab indicator animation', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.IndicatorAnimationExampleApp());

    final RenderBox tabBarBox = tester.firstRenderObject<RenderBox>(find.byType(TabBar));

    late RRect indicatorRRect;

    expect(
      tabBarBox,
      paints..something((Symbol method, List<dynamic> arguments) {
        if (method != #drawRRect) {
          return false;
        }
        indicatorRRect = arguments[0] as RRect;
        return true;
      }),
    );
    expect(indicatorRRect.left, equals(16.0));
    expect(indicatorRRect.top, equals(45.0));
    expect(indicatorRRect.right, closeTo(142.9, 0.1));
    expect(indicatorRRect.bottom, equals(48.0));

    // Tap the long tab.
    await tester.tap(find.text('Very Very Very Long Tab').first);
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));

    expect(
      tabBarBox,
      paints..something((Symbol method, List<dynamic> arguments) {
        if (method != #drawRRect) {
          return false;
        }
        indicatorRRect = arguments[0] as RRect;
        return true;
      }),
    );
    expect(indicatorRRect.left, closeTo(107.5, 0.1));
    expect(indicatorRRect.top, equals(45.0));
    expect(indicatorRRect.right, closeTo(348.2, 0.1));
    expect(indicatorRRect.bottom, equals(48.0));

    // Tap to go to the first tab.
    await tester.tap(find.text('Short Tab').first);
    await tester.pumpAndSettle();

    expect(
      tabBarBox,
      paints..something((Symbol method, List<dynamic> arguments) {
        if (method != #drawRRect) {
          return false;
        }
        indicatorRRect = arguments[0] as RRect;
        return true;
      }),
    );
    expect(indicatorRRect.left, equals(16.0));
    expect(indicatorRRect.top, equals(45.0));
    expect(indicatorRRect.right, closeTo(142.9, 0.1));
    expect(indicatorRRect.bottom, equals(48.0));

    // Select the elastic animation.
    await tester.tap(find.text('Elastic'));
    await tester.pumpAndSettle();

    // Tap the long tab.
    await tester.tap(find.text('Very Very Very Long Tab').first);
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));

    expect(
      tabBarBox,
      paints..something((Symbol method, List<dynamic> arguments) {
        if (method != #drawRRect) {
          return false;
        }
        indicatorRRect = arguments[0] as RRect;
        return true;
      }),
    );
    expect(indicatorRRect.left, closeTo(76.7, 0.1));
    expect(indicatorRRect.top, equals(45.0));
    expect(indicatorRRect.right, closeTo(423.1, 0.1));
    expect(indicatorRRect.bottom, equals(48.0));
  });
}
