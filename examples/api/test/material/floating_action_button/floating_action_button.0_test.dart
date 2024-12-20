// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/floating_action_button/floating_action_button.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('FloatingActionButton', (WidgetTester tester) async {
    await tester.pumpWidget(const example.FloatingActionButtonExampleApp());

    expect(find.byType(FloatingActionButton), findsOneWidget);
    expect(find.byIcon(Icons.navigation), findsOneWidget);

    RawMaterialButton getRawMaterialButtonWidget() {
      return tester.widget<RawMaterialButton>(find.byType(RawMaterialButton));
    }

    Color? getIconColor() {
      final RichText iconRichText = tester.widget<RichText>(
        find.descendant(of: find.byIcon(Icons.navigation), matching: find.byType(RichText)),
      );
      return iconRichText.text.style?.color;
    }

    await tester.tap(find.byType(FloatingActionButton));
    await tester.pumpAndSettle(); // Wait for the animation to finish.
    expect(getRawMaterialButtonWidget().fillColor, Colors.green);

    await tester.tap(find.byType(FloatingActionButton));
    await tester.pumpAndSettle(); // Wait for the animation to finish.
    expect(getRawMaterialButtonWidget().fillColor, Colors.green);
    expect(getIconColor(), Colors.white);

    await tester.tap(find.byType(FloatingActionButton));
    await tester.pumpAndSettle(); // Wait for the animation to finish.
    expect(getRawMaterialButtonWidget().fillColor, Colors.green);
    expect(getIconColor(), Colors.white);
    expect(getRawMaterialButtonWidget().shape, const CircleBorder());
  });
}
