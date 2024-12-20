// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/text_button/text_button.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  // The app being tested loads images via HTTP which the test
  // framework defeats by default.
  setUpAll(() {
    HttpOverrides.global = null;
  });

  testWidgets('TextButtonExample smoke test', (WidgetTester tester) async {
    await tester.pumpWidget(const example.TextButtonExampleApp());
    await tester.pumpAndSettle();

    await tester.tap(find.widgetWithText(TextButton, 'Enabled'));
    await tester.pumpAndSettle();

    await tester.tap(find.widgetWithText(TextButton, 'Disabled'));
    await tester.pumpAndSettle();

    // TextButton.icon buttons are _TextButtonWithIcons rather than TextButtons.
    // For the purposes of this test, just tapping in the right place is OK.

    await tester.tap(find.text('TextButton.icon #1'));
    await tester.pumpAndSettle();

    await tester.tap(find.text('TextButton.icon #2'));
    await tester.pumpAndSettle();

    await tester.tap(find.widgetWithText(TextButton, 'TextButton #3'));
    await tester.pumpAndSettle();

    await tester.tap(find.widgetWithText(TextButton, 'TextButton #4'));
    await tester.pumpAndSettle();

    await tester.tap(find.widgetWithText(TextButton, 'TextButton #5'));
    await tester.pumpAndSettle();

    await tester.tap(find.widgetWithText(TextButton, 'TextButton #6'));
    await tester.pumpAndSettle();

    await tester.tap(find.widgetWithText(TextButton, 'TextButton #7'));
    await tester.pumpAndSettle();

    await tester.tap(find.widgetWithText(TextButton, 'TextButton #8'));
    await tester.pumpAndSettle();

    final Finder smileyButton = find.byType(TextButton).last;
    await tester.tap(smileyButton);
    await tester.pump();

    String smileyButtonImageUrl() {
      final AnimatedContainer container = tester.widget<AnimatedContainer>(
        find.descendant(of: smileyButton, matching: find.byType(AnimatedContainer)),
      );
      final BoxDecoration decoration = container.decoration! as BoxDecoration;
      final NetworkImage image = decoration.image!.image as NetworkImage;
      return image.url;
    }

    // The smiley button's onPressed method changes the button image
    // for one second to simulate a long action running. The button's
    // image changes while the action is running.
    expect(smileyButtonImageUrl().endsWith('text_button_nhu_end.png'), isTrue);
    await tester.pump(const Duration(seconds: 1));
    expect(smileyButtonImageUrl().endsWith('text_button_nhu_default.png'), isTrue);

    // Pressing the smiley button while the one second action is
    // underway starts a new one section action. The button's image
    // doesn't change until the second action has finished.
    await tester.tap(smileyButton);
    await tester.pump(const Duration(milliseconds: 500));
    expect(smileyButtonImageUrl().endsWith('text_button_nhu_end.png'), isTrue);
    await tester.tap(smileyButton); // Second button press.
    await tester.pump(const Duration(milliseconds: 500));
    expect(smileyButtonImageUrl().endsWith('text_button_nhu_end.png'), isTrue);
    await tester.pump(const Duration(milliseconds: 500));
    expect(smileyButtonImageUrl().endsWith('text_button_nhu_default.png'), isTrue);

    await tester.tap(find.byType(Switch).at(0)); // Dark Mode Switch
    await tester.pumpAndSettle();

    await tester.tap(find.byType(Switch).at(1)); // RTL Text Switch
    await tester.pumpAndSettle();
  });
}
