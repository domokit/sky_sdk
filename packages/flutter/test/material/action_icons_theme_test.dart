// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('ActionIconThemeData copyWith, ==, hashCode basics', () {
    expect(const ActionIconThemeData(), const ActionIconThemeData().copyWith());
    expect(const ActionIconThemeData().hashCode, const ActionIconThemeData().copyWith().hashCode);
  });

  testWidgets('ActionIconThemeData copyWith overrides all properties', (WidgetTester tester) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/126762.
    Widget originalButtonBuilder(BuildContext context) {
      return const SizedBox();
    }

    Widget newButtonBuilder(BuildContext context) {
      return const Icon(Icons.add);
    }

    // Create a ActionIconThemeData with all properties set.
    final ActionIconThemeData original = ActionIconThemeData(
      backButtonIconBuilder: originalButtonBuilder,
      closeButtonIconBuilder: originalButtonBuilder,
      drawerButtonIconBuilder: originalButtonBuilder,
      endDrawerButtonIconBuilder: originalButtonBuilder,
    );
    // Check if the all properties are copied.
    final ActionIconThemeData copy = original.copyWith();
    expect(copy.backButtonIconBuilder, originalButtonBuilder);
    expect(copy.closeButtonIconBuilder, originalButtonBuilder);
    expect(copy.drawerButtonIconBuilder, originalButtonBuilder);
    expect(copy.endDrawerButtonIconBuilder, originalButtonBuilder);

    // Check if the properties are overridden.
    final ActionIconThemeData overridden = original.copyWith(
      backButtonIconBuilder: newButtonBuilder,
      closeButtonIconBuilder: newButtonBuilder,
      drawerButtonIconBuilder: newButtonBuilder,
      endDrawerButtonIconBuilder: newButtonBuilder,
    );
    expect(overridden.backButtonIconBuilder, newButtonBuilder);
    expect(overridden.closeButtonIconBuilder, newButtonBuilder);
    expect(overridden.drawerButtonIconBuilder, newButtonBuilder);
    expect(overridden.endDrawerButtonIconBuilder, newButtonBuilder);
  });

  test('ActionIconThemeData defaults', () {
    const ActionIconThemeData themeData = ActionIconThemeData();
    expect(themeData.backButtonIconBuilder, null);
    expect(themeData.closeButtonIconBuilder, null);
    expect(themeData.drawerButtonIconBuilder, null);
    expect(themeData.endDrawerButtonIconBuilder, null);
  });

  testWidgets('Default ActionIconThemeData debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const ActionIconThemeData().debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[]);
  });

  testWidgets('ActionIconThemeData implements debugFillProperties', (WidgetTester tester) async {
    Widget actionButtonIconBuilder(BuildContext context) {
      return const Icon(IconData(0));
    }

    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    ActionIconThemeData(
      backButtonIconBuilder: actionButtonIconBuilder,
      closeButtonIconBuilder: actionButtonIconBuilder,
      drawerButtonIconBuilder: actionButtonIconBuilder,
      endDrawerButtonIconBuilder: actionButtonIconBuilder,
    ).debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    final Matcher containsBuilderCallback = contains('Closure: (BuildContext) =>');
    expect(description, <dynamic>[
      allOf(startsWith('backButtonIconBuilder:'), containsBuilderCallback),
      allOf(startsWith('closeButtonIconBuilder:'), containsBuilderCallback),
      allOf(startsWith('drawerButtonIconBuilder:'), containsBuilderCallback),
      allOf(startsWith('endDrawerButtonIconBuilder:'), containsBuilderCallback),
    ]);
  });

  testWidgets('Action buttons use ThemeData action icon theme', (WidgetTester tester) async {
    const Color green = Color(0xff00ff00);
    const IconData icon = IconData(0);

    Widget buildSampleIcon(BuildContext context) {
      return const Icon(icon, size: 20, color: green);
    }

    final ActionIconThemeData actionIconTheme = ActionIconThemeData(
      backButtonIconBuilder: buildSampleIcon,
      closeButtonIconBuilder: buildSampleIcon,
      drawerButtonIconBuilder: buildSampleIcon,
      endDrawerButtonIconBuilder: buildSampleIcon,
    );

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.light(useMaterial3: true).copyWith(actionIconTheme: actionIconTheme),
        home: const Material(
          child: Column(
            children: <Widget>[BackButton(), CloseButton(), DrawerButton(), EndDrawerButton()],
          ),
        ),
      ),
    );

    final Icon backButtonIcon = tester.widget(
      find.descendant(of: find.byType(BackButton), matching: find.byType(Icon)),
    );
    final Icon closeButtonIcon = tester.widget(
      find.descendant(of: find.byType(CloseButton), matching: find.byType(Icon)),
    );
    final Icon drawerButtonIcon = tester.widget(
      find.descendant(of: find.byType(DrawerButton), matching: find.byType(Icon)),
    );
    final Icon endDrawerButtonIcon = tester.widget(
      find.descendant(of: find.byType(EndDrawerButton), matching: find.byType(Icon)),
    );

    expect(backButtonIcon.icon == icon, isTrue);
    expect(closeButtonIcon.icon == icon, isTrue);
    expect(drawerButtonIcon.icon == icon, isTrue);
    expect(endDrawerButtonIcon.icon == icon, isTrue);

    final RichText backButtonIconText = tester.widget(
      find.descendant(of: find.byType(BackButton), matching: find.byType(RichText)),
    );
    final RichText closeButtonIconText = tester.widget(
      find.descendant(of: find.byType(CloseButton), matching: find.byType(RichText)),
    );
    final RichText drawerButtonIconText = tester.widget(
      find.descendant(of: find.byType(DrawerButton), matching: find.byType(RichText)),
    );
    final RichText endDrawerButtonIconText = tester.widget(
      find.descendant(of: find.byType(EndDrawerButton), matching: find.byType(RichText)),
    );

    expect(backButtonIconText.text.style!.color, green);
    expect(closeButtonIconText.text.style!.color, green);
    expect(drawerButtonIconText.text.style!.color, green);
    expect(endDrawerButtonIconText.text.style!.color, green);
  });

  // This test is essentially the same as 'Action buttons use ThemeData action icon theme'. In
  // this case the theme is introduced with the ActionIconTheme widget instead of
  // ThemeData.actionIconTheme.
  testWidgets('Action buttons use ActionIconTheme', (WidgetTester tester) async {
    const Color green = Color(0xff00ff00);
    const IconData icon = IconData(0);

    Widget buildSampleIcon(BuildContext context) {
      return const Icon(icon, size: 20, color: green);
    }

    final ActionIconThemeData actionIconTheme = ActionIconThemeData(
      backButtonIconBuilder: buildSampleIcon,
      closeButtonIconBuilder: buildSampleIcon,
      drawerButtonIconBuilder: buildSampleIcon,
      endDrawerButtonIconBuilder: buildSampleIcon,
    );

    await tester.pumpWidget(
      MaterialApp(
        home: ActionIconTheme(
          data: actionIconTheme,
          child: const Material(
            child: Column(
              children: <Widget>[BackButton(), CloseButton(), DrawerButton(), EndDrawerButton()],
            ),
          ),
        ),
      ),
    );

    final Icon backButtonIcon = tester.widget(
      find.descendant(of: find.byType(BackButton), matching: find.byType(Icon)),
    );
    final Icon closeButtonIcon = tester.widget(
      find.descendant(of: find.byType(CloseButton), matching: find.byType(Icon)),
    );
    final Icon drawerButtonIcon = tester.widget(
      find.descendant(of: find.byType(DrawerButton), matching: find.byType(Icon)),
    );
    final Icon endDrawerButtonIcon = tester.widget(
      find.descendant(of: find.byType(EndDrawerButton), matching: find.byType(Icon)),
    );

    expect(backButtonIcon.icon == icon, isTrue);
    expect(closeButtonIcon.icon == icon, isTrue);
    expect(drawerButtonIcon.icon == icon, isTrue);
    expect(endDrawerButtonIcon.icon == icon, isTrue);

    final RichText backButtonIconText = tester.widget(
      find.descendant(of: find.byType(BackButton), matching: find.byType(RichText)),
    );
    final RichText closeButtonIconText = tester.widget(
      find.descendant(of: find.byType(CloseButton), matching: find.byType(RichText)),
    );
    final RichText drawerButtonIconText = tester.widget(
      find.descendant(of: find.byType(DrawerButton), matching: find.byType(RichText)),
    );
    final RichText endDrawerButtonIconText = tester.widget(
      find.descendant(of: find.byType(EndDrawerButton), matching: find.byType(RichText)),
    );

    expect(backButtonIconText.text.style!.color, green);
    expect(closeButtonIconText.text.style!.color, green);
    expect(drawerButtonIconText.text.style!.color, green);
    expect(endDrawerButtonIconText.text.style!.color, green);
  });
}
