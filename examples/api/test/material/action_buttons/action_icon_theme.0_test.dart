// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/action_buttons/action_icon_theme.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Action Icon Buttons', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ActionIconThemeExampleApp());

    expect(find.byType(DrawerButton), findsOneWidget);
    final Icon drawerButtonIcon = tester.widget(
      find.descendant(of: find.byType(DrawerButton), matching: find.byType(Icon)),
    );
    expect(drawerButtonIcon.icon, Icons.segment);

    // open next page
    await tester.tap(find.byType(example.NextPageButton));
    await tester.pumpAndSettle();

    expect(find.byType(EndDrawerButton), findsOneWidget);
    final Icon endDrawerButtonIcon = tester.widget(
      find.descendant(of: find.byType(EndDrawerButton), matching: find.byType(Icon)),
    );
    expect(endDrawerButtonIcon.icon, Icons.more_horiz);

    expect(find.byType(BackButton), findsOneWidget);
    final Icon backButtonIcon = tester.widget(
      find.descendant(of: find.byType(BackButton), matching: find.byType(Icon)),
    );
    expect(backButtonIcon.icon, Icons.arrow_back_ios_new_rounded);
  });
}
