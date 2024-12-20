// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/icon_button/icon_button.3.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('It should select and unselect the icon buttons', (WidgetTester tester) async {
    await tester.pumpWidget(const example.IconButtonToggleApp());

    expect(find.widgetWithIcon(IconButton, Icons.settings_outlined), findsExactly(8));
    final Finder unselectedIconButtons = find.widgetWithIcon(IconButton, Icons.settings_outlined);
    for (int i = 0; i <= 6; i++) {
      final IconButton button = tester.widget<IconButton>(unselectedIconButtons.at(i));
      expect(button.onPressed, i.isEven ? isA<VoidCallback>() : isNull);
      expect(button.isSelected, isFalse);
    }

    // Select the icons buttons.
    for (int i = 0; i <= 3; i++) {
      await tester.tap(unselectedIconButtons.at(2 * i));
    }
    await tester.pump();

    expect(find.widgetWithIcon(IconButton, Icons.settings), findsExactly(8));
    final Finder selectedIconButtons = find.widgetWithIcon(IconButton, Icons.settings);
    for (int i = 0; i <= 6; i++) {
      final IconButton button = tester.widget<IconButton>(selectedIconButtons.at(i));
      expect(button.onPressed, i.isEven ? isA<VoidCallback>() : isNull);
      expect(button.isSelected, isTrue);
    }

    // Unselect the icons buttons.
    for (int i = 0; i <= 3; i++) {
      await tester.tap(selectedIconButtons.at(2 * i));
    }
    await tester.pump();

    expect(find.widgetWithIcon(IconButton, Icons.settings_outlined), findsExactly(8));
    for (int i = 0; i <= 6; i++) {
      final IconButton button = tester.widget<IconButton>(unselectedIconButtons.at(i));
      expect(button.onPressed, i.isEven ? isA<VoidCallback>() : isNull);
      expect(button.isSelected, isFalse);
    }
  });
}
