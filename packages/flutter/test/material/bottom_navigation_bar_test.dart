// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'dart:math';
import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:vector_math/vector_math_64.dart' show Vector3;

import '../widgets/feedback_tester.dart';
import '../widgets/semantics_tester.dart';

void main() {
  testWidgets('BottomNavigationBar callback test', (WidgetTester tester) async {
    late int mutatedIndex;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
            onTap: (int index) {
              mutatedIndex = index;
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('Alarm'));

    expect(mutatedIndex, 1);
  });

  testWidgets('Material2 - BottomNavigationBar content test', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
          ),
        ),
      ),
    );

    final RenderBox box = tester.renderObject(find.byType(BottomNavigationBar));
    expect(box.size.height, kBottomNavigationBarHeight);
    expect(find.text('AC'), findsOneWidget);
    expect(find.text('Alarm'), findsOneWidget);
  });

  testWidgets('Material3 - BottomNavigationBar content test', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
          ),
        ),
      ),
    );

    final RenderBox box = tester.renderObject(find.byType(BottomNavigationBar));
    // kBottomNavigationBarHeight is a minimum dimension.
    expect(box.size.height, greaterThanOrEqualTo(kBottomNavigationBarHeight));
    expect(find.text('AC'), findsOneWidget);
    expect(find.text('Alarm'), findsOneWidget);
  });

  testWidgets('Material2 - Fixed BottomNavigationBar defaults', (WidgetTester tester) async {
    const Color primaryColor = Color(0xFF000001);
    const Color unselectedWidgetColor = Color(0xFF000002);

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.light(useMaterial3: false).copyWith(
          colorScheme: const ColorScheme.light().copyWith(primary: primaryColor),
          unselectedWidgetColor: unselectedWidgetColor,
        ),
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.fixed,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
          ),
        ),
      ),
    );

    const double selectedFontSize = 14.0;
    const double unselectedFontSize = 12.0;
    final TextStyle selectedFontStyle =
        tester.renderObject<RenderParagraph>(find.text('AC')).text.style!;
    final TextStyle unselectedFontStyle =
        tester.renderObject<RenderParagraph>(find.text('Alarm')).text.style!;
    final TextStyle selectedIcon = _iconStyle(tester, Icons.ac_unit);
    final TextStyle unselectedIcon = _iconStyle(tester, Icons.access_alarm);
    expect(selectedFontStyle.color, equals(primaryColor));
    expect(selectedFontStyle.fontSize, selectedFontSize);
    expect(selectedFontStyle.fontWeight, equals(FontWeight.w400));
    expect(selectedFontStyle.height, isNull);
    expect(unselectedFontStyle.color, equals(unselectedWidgetColor));
    expect(unselectedFontStyle.fontWeight, equals(FontWeight.w400));
    expect(unselectedFontStyle.height, isNull);
    // Unselected label has a font size of 14 but is scaled down to be font size 12.
    expect(
      tester
          .firstWidget<Transform>(
            find.ancestor(of: find.text('Alarm'), matching: find.byType(Transform)),
          )
          .transform,
      equals(Matrix4.diagonal3(Vector3.all(unselectedFontSize / selectedFontSize))),
    );
    expect(selectedIcon.color, equals(primaryColor));
    expect(selectedIcon.fontSize, equals(24.0));
    expect(unselectedIcon.color, equals(unselectedWidgetColor));
    expect(unselectedIcon.fontSize, equals(24.0));
    // There should not be any [Opacity] or [FadeTransition] widgets
    // since showUnselectedLabels and showSelectedLabels are true.
    final Finder findOpacity = find.descendant(
      of: find.byType(BottomNavigationBar),
      matching: find.byType(Opacity),
    );
    final Finder findFadeTransition = find.descendant(
      of: find.byType(BottomNavigationBar),
      matching: find.byType(FadeTransition),
    );
    expect(findOpacity, findsNothing);
    expect(findFadeTransition, findsNothing);
    expect(_getMaterial(tester).elevation, equals(8.0));
  });

  testWidgets('Material3 - Fixed BottomNavigationBar defaults', (WidgetTester tester) async {
    const Color primaryColor = Color(0xFF000001);
    const Color unselectedWidgetColor = Color(0xFF000002);

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          colorScheme: const ColorScheme.light().copyWith(primary: primaryColor),
          unselectedWidgetColor: unselectedWidgetColor,
        ),
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.fixed,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
          ),
        ),
      ),
    );

    const double selectedFontSize = 14.0;
    const double unselectedFontSize = 12.0;
    final TextStyle selectedFontStyle =
        tester.renderObject<RenderParagraph>(find.text('AC')).text.style!;
    final TextStyle unselectedFontStyle =
        tester.renderObject<RenderParagraph>(find.text('Alarm')).text.style!;
    final TextStyle selectedIcon = _iconStyle(tester, Icons.ac_unit);
    final TextStyle unselectedIcon = _iconStyle(tester, Icons.access_alarm);
    expect(selectedFontStyle.color, equals(primaryColor));
    expect(selectedFontStyle.fontSize, selectedFontSize);
    expect(selectedFontStyle.fontWeight, equals(FontWeight.w400));
    expect(selectedFontStyle.height, 1.43);
    expect(unselectedFontStyle.color, equals(unselectedWidgetColor));
    expect(unselectedFontStyle.fontWeight, equals(FontWeight.w400));
    expect(unselectedFontStyle.height, 1.43);
    // Unselected label has a font size of 14 but is scaled down to be font size 12.
    expect(
      tester
          .firstWidget<Transform>(
            find.ancestor(of: find.text('Alarm'), matching: find.byType(Transform)),
          )
          .transform,
      equals(Matrix4.diagonal3(Vector3.all(unselectedFontSize / selectedFontSize))),
    );
    expect(selectedIcon.color, equals(primaryColor));
    expect(selectedIcon.fontSize, equals(24.0));
    expect(unselectedIcon.color, equals(unselectedWidgetColor));
    expect(unselectedIcon.fontSize, equals(24.0));
    // There should not be any [Opacity] or [FadeTransition] widgets
    // since showUnselectedLabels and showSelectedLabels are true.
    final Finder findOpacity = find.descendant(
      of: find.byType(BottomNavigationBar),
      matching: find.byType(Opacity),
    );
    final Finder findFadeTransition = find.descendant(
      of: find.byType(BottomNavigationBar),
      matching: find.byType(FadeTransition),
    );
    expect(findOpacity, findsNothing);
    expect(findFadeTransition, findsNothing);
    expect(_getMaterial(tester).elevation, equals(8.0));
  });

  testWidgets('Custom selected and unselected font styles', (WidgetTester tester) async {
    const TextStyle selectedTextStyle = TextStyle(fontWeight: FontWeight.w200, fontSize: 18.0);
    const TextStyle unselectedTextStyle = TextStyle(fontWeight: FontWeight.w600, fontSize: 12.0);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.fixed,
            selectedLabelStyle: selectedTextStyle,
            unselectedLabelStyle: unselectedTextStyle,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
          ),
        ),
      ),
    );

    final TextStyle selectedFontStyle =
        tester.renderObject<RenderParagraph>(find.text('AC')).text.style!;
    final TextStyle unselectedFontStyle =
        tester.renderObject<RenderParagraph>(find.text('Alarm')).text.style!;
    expect(selectedFontStyle.fontSize, equals(selectedTextStyle.fontSize));
    expect(selectedFontStyle.fontWeight, equals(selectedTextStyle.fontWeight));
    expect(
      tester
          .firstWidget<Transform>(
            find.ancestor(of: find.text('Alarm'), matching: find.byType(Transform)),
          )
          .transform,
      equals(
        Matrix4.diagonal3(Vector3.all(unselectedTextStyle.fontSize! / selectedTextStyle.fontSize!)),
      ),
    );
    expect(unselectedFontStyle.fontWeight, equals(unselectedTextStyle.fontWeight));
  });

  testWidgets('font size on text styles overrides font size params', (WidgetTester tester) async {
    const TextStyle selectedTextStyle = TextStyle(fontSize: 18.0);
    const TextStyle unselectedTextStyle = TextStyle(fontSize: 12.0);
    const double selectedFontSize = 17.0;
    const double unselectedFontSize = 11.0;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.fixed,
            selectedLabelStyle: selectedTextStyle,
            unselectedLabelStyle: unselectedTextStyle,
            selectedFontSize: selectedFontSize,
            unselectedFontSize: unselectedFontSize,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
          ),
        ),
      ),
    );

    final TextStyle selectedFontStyle =
        tester.renderObject<RenderParagraph>(find.text('AC')).text.style!;
    expect(selectedFontStyle.fontSize, equals(selectedTextStyle.fontSize));
    expect(
      tester
          .firstWidget<Transform>(
            find.ancestor(of: find.text('Alarm'), matching: find.byType(Transform)),
          )
          .transform,
      equals(
        Matrix4.diagonal3(Vector3.all(unselectedTextStyle.fontSize! / selectedTextStyle.fontSize!)),
      ),
    );
  });

  testWidgets('Custom selected and unselected icon themes', (WidgetTester tester) async {
    const IconThemeData selectedIconTheme = IconThemeData(size: 36, color: Color(0x00000001));
    const IconThemeData unselectedIconTheme = IconThemeData(size: 18, color: Color(0x00000002));

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.fixed,
            selectedIconTheme: selectedIconTheme,
            unselectedIconTheme: unselectedIconTheme,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
          ),
        ),
      ),
    );

    final TextStyle selectedIcon = _iconStyle(tester, Icons.ac_unit);
    final TextStyle unselectedIcon = _iconStyle(tester, Icons.access_alarm);
    expect(selectedIcon.color, equals(selectedIconTheme.color));
    expect(selectedIcon.fontSize, equals(selectedIconTheme.size));
    expect(unselectedIcon.color, equals(unselectedIconTheme.color));
    expect(unselectedIcon.fontSize, equals(unselectedIconTheme.size));
  });

  testWidgets('color on icon theme overrides selected and unselected item colors', (
    WidgetTester tester,
  ) async {
    const IconThemeData selectedIconTheme = IconThemeData(size: 36, color: Color(0x00000001));
    const IconThemeData unselectedIconTheme = IconThemeData(size: 18, color: Color(0x00000002));
    const Color selectedItemColor = Color(0x00000003);
    const Color unselectedItemColor = Color(0x00000004);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.fixed,
            selectedIconTheme: selectedIconTheme,
            unselectedIconTheme: unselectedIconTheme,
            selectedItemColor: selectedItemColor,
            unselectedItemColor: unselectedItemColor,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
          ),
        ),
      ),
    );

    final TextStyle selectedFontStyle =
        tester.renderObject<RenderParagraph>(find.text('AC')).text.style!;
    final TextStyle unselectedFontStyle =
        tester.renderObject<RenderParagraph>(find.text('Alarm')).text.style!;
    final TextStyle selectedIcon = _iconStyle(tester, Icons.ac_unit);
    final TextStyle unselectedIcon = _iconStyle(tester, Icons.access_alarm);
    expect(selectedIcon.color, equals(selectedIconTheme.color));
    expect(unselectedIcon.color, equals(unselectedIconTheme.color));
    expect(selectedFontStyle.color, equals(selectedItemColor));
    expect(unselectedFontStyle.color, equals(unselectedItemColor));
  });

  testWidgets('Padding is calculated properly on items - all labels', (WidgetTester tester) async {
    const double selectedFontSize = 16.0;
    const double selectedIconSize = 36.0;
    const double unselectedIconSize = 20.0;
    const IconThemeData selectedIconTheme = IconThemeData(size: selectedIconSize);
    const IconThemeData unselectedIconTheme = IconThemeData(size: unselectedIconSize);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.fixed,
            showSelectedLabels: true,
            showUnselectedLabels: true,
            selectedFontSize: selectedFontSize,
            selectedIconTheme: selectedIconTheme,
            unselectedIconTheme: unselectedIconTheme,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
          ),
        ),
      ),
    );

    final EdgeInsets selectedItemPadding = _itemPadding(tester, Icons.ac_unit);
    expect(selectedItemPadding.top, equals(selectedFontSize / 2.0));
    expect(selectedItemPadding.bottom, equals(selectedFontSize / 2.0));
    final EdgeInsets unselectedItemPadding = _itemPadding(tester, Icons.access_alarm);
    const double expectedUnselectedPadding =
        (selectedIconSize - unselectedIconSize) / 2.0 + selectedFontSize / 2.0;
    expect(unselectedItemPadding.top, equals(expectedUnselectedPadding));
    expect(unselectedItemPadding.bottom, equals(expectedUnselectedPadding));
  });

  testWidgets('Padding is calculated properly on items - selected labels only', (
    WidgetTester tester,
  ) async {
    const double selectedFontSize = 16.0;
    const double selectedIconSize = 36.0;
    const double unselectedIconSize = 20.0;
    const IconThemeData selectedIconTheme = IconThemeData(size: selectedIconSize);
    const IconThemeData unselectedIconTheme = IconThemeData(size: unselectedIconSize);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.fixed,
            showSelectedLabels: true,
            showUnselectedLabels: false,
            selectedFontSize: selectedFontSize,
            selectedIconTheme: selectedIconTheme,
            unselectedIconTheme: unselectedIconTheme,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
          ),
        ),
      ),
    );

    final EdgeInsets selectedItemPadding = _itemPadding(tester, Icons.ac_unit);
    expect(selectedItemPadding.top, equals(selectedFontSize / 2.0));
    expect(selectedItemPadding.bottom, equals(selectedFontSize / 2.0));
    final EdgeInsets unselectedItemPadding = _itemPadding(tester, Icons.access_alarm);
    expect(
      unselectedItemPadding.top,
      equals((selectedIconSize - unselectedIconSize) / 2.0 + selectedFontSize),
    );
    expect(unselectedItemPadding.bottom, equals((selectedIconSize - unselectedIconSize) / 2.0));
  });

  testWidgets('Padding is calculated properly on items - no labels', (WidgetTester tester) async {
    const double selectedFontSize = 16.0;
    const double selectedIconSize = 36.0;
    const double unselectedIconSize = 20.0;
    const IconThemeData selectedIconTheme = IconThemeData(size: selectedIconSize);
    const IconThemeData unselectedIconTheme = IconThemeData(size: unselectedIconSize);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.fixed,
            showSelectedLabels: false,
            showUnselectedLabels: false,
            selectedFontSize: selectedFontSize,
            selectedIconTheme: selectedIconTheme,
            unselectedIconTheme: unselectedIconTheme,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
          ),
        ),
      ),
    );

    final EdgeInsets selectedItemPadding = _itemPadding(tester, Icons.ac_unit);
    expect(selectedItemPadding.top, equals(selectedFontSize));
    expect(selectedItemPadding.bottom, equals(0.0));
    final EdgeInsets unselectedItemPadding = _itemPadding(tester, Icons.access_alarm);
    expect(
      unselectedItemPadding.top,
      equals((selectedIconSize - unselectedIconSize) / 2.0 + selectedFontSize),
    );
    expect(unselectedItemPadding.bottom, equals((selectedIconSize - unselectedIconSize) / 2.0));
  });

  testWidgets('Material2 - Shifting BottomNavigationBar defaults', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.shifting,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
          ),
        ),
      ),
    );

    const double selectedFontSize = 14.0;
    expect(
      tester.renderObject<RenderParagraph>(find.text('AC')).text.style!.fontSize,
      selectedFontSize,
    );
    expect(
      tester.renderObject<RenderParagraph>(find.text('AC')).text.style!.color,
      equals(Colors.white),
    );
    expect(_getOpacity(tester, 'Alarm'), equals(0.0));
    expect(_getMaterial(tester).elevation, equals(8.0));
  });

  testWidgets('Material3 - Shifting BottomNavigationBar defaults', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.shifting,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
          ),
        ),
      ),
    );

    const double selectedFontSize = 14.0;
    expect(
      tester.renderObject<RenderParagraph>(find.text('AC')).text.style!.fontSize,
      selectedFontSize,
    );
    final ThemeData theme = Theme.of(tester.element(find.text('AC')));
    expect(
      tester.renderObject<RenderParagraph>(find.text('AC')).text.style!.color,
      equals(theme.colorScheme.surface),
    );
    expect(_getOpacity(tester, 'Alarm'), equals(0.0));
    expect(_getMaterial(tester).elevation, equals(8.0));
  });

  testWidgets('Fixed BottomNavigationBar custom font size, color', (WidgetTester tester) async {
    const Color primaryColor = Color(0xFF000000);
    const Color unselectedWidgetColor = Color(0xFFD501FF);
    const Color selectedColor = Color(0xFF0004FF);
    const Color unselectedColor = Color(0xFFE5FF00);
    const double selectedFontSize = 18.0;
    const double unselectedFontSize = 14.0;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(primaryColor: primaryColor, unselectedWidgetColor: unselectedWidgetColor),
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.fixed,
            selectedFontSize: selectedFontSize,
            unselectedFontSize: unselectedFontSize,
            selectedItemColor: selectedColor,
            unselectedItemColor: unselectedColor,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
          ),
        ),
      ),
    );

    final TextStyle selectedIcon = _iconStyle(tester, Icons.ac_unit);
    final TextStyle unselectedIcon = _iconStyle(tester, Icons.access_alarm);

    expect(
      tester.renderObject<RenderParagraph>(find.text('AC')).text.style!.fontSize,
      selectedFontSize,
    );
    // Unselected label has a font size of 18 but is scaled down to be font size 14.
    expect(
      tester.renderObject<RenderParagraph>(find.text('Alarm')).text.style!.fontSize,
      selectedFontSize,
    );
    expect(
      tester
          .firstWidget<Transform>(
            find.ancestor(of: find.text('Alarm'), matching: find.byType(Transform)),
          )
          .transform,
      equals(Matrix4.diagonal3(Vector3.all(unselectedFontSize / selectedFontSize))),
    );
    expect(
      tester.renderObject<RenderParagraph>(find.text('AC')).text.style!.color,
      equals(selectedColor),
    );
    expect(
      tester.renderObject<RenderParagraph>(find.text('Alarm')).text.style!.color,
      equals(unselectedColor),
    );
    expect(selectedIcon.color, equals(selectedColor));
    expect(unselectedIcon.color, equals(unselectedColor));
    // There should not be any [Opacity] or [FadeTransition] widgets
    // since showUnselectedLabels and showSelectedLabels are true.
    final Finder findOpacity = find.descendant(
      of: find.byType(BottomNavigationBar),
      matching: find.byType(Opacity),
    );
    final Finder findFadeTransition = find.descendant(
      of: find.byType(BottomNavigationBar),
      matching: find.byType(FadeTransition),
    );
    expect(findOpacity, findsNothing);
    expect(findFadeTransition, findsNothing);
  });

  testWidgets('Shifting BottomNavigationBar custom font size, color', (WidgetTester tester) async {
    const Color primaryColor = Color(0xFF000000);
    const Color unselectedWidgetColor = Color(0xFFD501FF);
    const Color selectedColor = Color(0xFF0004FF);
    const Color unselectedColor = Color(0xFFE5FF00);
    const double selectedFontSize = 18.0;
    const double unselectedFontSize = 14.0;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(primaryColor: primaryColor, unselectedWidgetColor: unselectedWidgetColor),
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.shifting,
            selectedFontSize: selectedFontSize,
            unselectedFontSize: unselectedFontSize,
            selectedItemColor: selectedColor,
            unselectedItemColor: unselectedColor,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
          ),
        ),
      ),
    );

    final TextStyle selectedIcon = _iconStyle(tester, Icons.ac_unit);
    final TextStyle unselectedIcon = _iconStyle(tester, Icons.access_alarm);

    expect(
      tester.renderObject<RenderParagraph>(find.text('AC')).text.style!.fontSize,
      selectedFontSize,
    );
    expect(
      tester.renderObject<RenderParagraph>(find.text('AC')).text.style!.color,
      equals(selectedColor),
    );
    expect(_getOpacity(tester, 'Alarm'), equals(0.0));

    expect(selectedIcon.color, equals(selectedColor));
    expect(unselectedIcon.color, equals(unselectedColor));
  });

  testWidgets(
    'label style color should override itemColor only for the label for BottomNavigationBarType.fixed',
    (WidgetTester tester) async {
      const Color primaryColor = Color(0xFF000000);
      const Color unselectedWidgetColor = Color(0xFFD501FF);
      const Color selectedColor = Color(0xFF0004FF);
      const Color unselectedColor = Color(0xFFE5FF00);
      const Color selectedLabelColor = Color(0xFFFF9900);
      const Color unselectedLabelColor = Color(0xFF92F74E);

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            primaryColor: primaryColor,
            unselectedWidgetColor: unselectedWidgetColor,
          ),
          home: Scaffold(
            bottomNavigationBar: BottomNavigationBar(
              type: BottomNavigationBarType.fixed,
              selectedLabelStyle: const TextStyle(color: selectedLabelColor),
              unselectedLabelStyle: const TextStyle(color: unselectedLabelColor),
              selectedItemColor: selectedColor,
              unselectedItemColor: unselectedColor,
              useLegacyColorScheme: false,
              items: const <BottomNavigationBarItem>[
                BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
                BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
              ],
            ),
          ),
        ),
      );

      final TextStyle selectedIcon = _iconStyle(tester, Icons.ac_unit);
      final TextStyle unselectedIcon = _iconStyle(tester, Icons.access_alarm);

      expect(selectedIcon.color, equals(selectedColor));
      expect(unselectedIcon.color, equals(unselectedColor));
      expect(
        tester.renderObject<RenderParagraph>(find.text('AC')).text.style!.color,
        equals(selectedLabelColor),
      );
      expect(
        tester.renderObject<RenderParagraph>(find.text('Alarm')).text.style!.color,
        equals(unselectedLabelColor),
      );
    },
  );

  testWidgets(
    'label style color should override itemColor only for the label for BottomNavigationBarType.shifting',
    (WidgetTester tester) async {
      const Color primaryColor = Color(0xFF000000);
      const Color unselectedWidgetColor = Color(0xFFD501FF);
      const Color selectedColor = Color(0xFF0004FF);
      const Color unselectedColor = Color(0xFFE5FF00);
      const Color selectedLabelColor = Color(0xFFFF9900);
      const Color unselectedLabelColor = Color(0xFF92F74E);

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            primaryColor: primaryColor,
            unselectedWidgetColor: unselectedWidgetColor,
          ),
          home: Scaffold(
            bottomNavigationBar: BottomNavigationBar(
              type: BottomNavigationBarType.shifting,
              selectedLabelStyle: const TextStyle(color: selectedLabelColor),
              unselectedLabelStyle: const TextStyle(color: unselectedLabelColor),
              selectedItemColor: selectedColor,
              unselectedItemColor: unselectedColor,
              useLegacyColorScheme: false,
              items: const <BottomNavigationBarItem>[
                BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
                BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
              ],
            ),
          ),
        ),
      );

      final TextStyle selectedIcon = _iconStyle(tester, Icons.ac_unit);
      final TextStyle unselectedIcon = _iconStyle(tester, Icons.access_alarm);

      expect(selectedIcon.color, equals(selectedColor));
      expect(unselectedIcon.color, equals(unselectedColor));
      expect(
        tester.renderObject<RenderParagraph>(find.text('AC')).text.style!.color,
        equals(selectedLabelColor),
      );
      expect(
        tester.renderObject<RenderParagraph>(find.text('Alarm')).text.style!.color,
        equals(unselectedLabelColor),
      );
    },
  );

  testWidgets('iconTheme color should override itemColor for BottomNavigationBarType.fixed', (
    WidgetTester tester,
  ) async {
    const Color primaryColor = Color(0xFF000000);
    const Color unselectedWidgetColor = Color(0xFFD501FF);
    const Color selectedColor = Color(0xFF0004FF);
    const Color unselectedColor = Color(0xFFE5FF00);
    const Color selectedLabelColor = Color(0xFFFF9900);
    const Color unselectedLabelColor = Color(0xFF92F74E);
    const Color selectedIconThemeColor = Color(0xFF1E7723);
    const Color unselectedIconThemeColor = Color(0xFF009688);
    const IconThemeData selectedIconTheme = IconThemeData(size: 20, color: selectedIconThemeColor);
    const IconThemeData unselectedIconTheme = IconThemeData(
      size: 18,
      color: unselectedIconThemeColor,
    );
    const TextStyle selectedTextStyle = TextStyle(fontSize: 18.0, color: selectedLabelColor);
    const TextStyle unselectedTextStyle = TextStyle(fontSize: 18.0, color: unselectedLabelColor);

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(primaryColor: primaryColor, unselectedWidgetColor: unselectedWidgetColor),
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.fixed,
            selectedLabelStyle: selectedTextStyle,
            unselectedLabelStyle: unselectedTextStyle,
            selectedIconTheme: selectedIconTheme,
            unselectedIconTheme: unselectedIconTheme,
            selectedItemColor: selectedColor,
            unselectedItemColor: unselectedColor,
            useLegacyColorScheme: false,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
          ),
        ),
      ),
    );

    final TextStyle selectedIcon = _iconStyle(tester, Icons.ac_unit);
    final TextStyle unselectedIcon = _iconStyle(tester, Icons.access_alarm);

    expect(selectedIcon.color, equals(selectedIconThemeColor));
    expect(unselectedIcon.color, equals(unselectedIconThemeColor));
  });

  testWidgets('iconTheme color should override itemColor for BottomNavigationBarType.shifted', (
    WidgetTester tester,
  ) async {
    const Color primaryColor = Color(0xFF000000);
    const Color unselectedWidgetColor = Color(0xFFD501FF);
    const Color selectedLabelColor = Color(0xFFFF9900);
    const Color unselectedLabelColor = Color(0xFF92F74E);
    const Color selectedIconThemeColor = Color(0xFF1E7723);
    const Color unselectedIconThemeColor = Color(0xFF009688);
    const IconThemeData selectedIconTheme = IconThemeData(size: 20, color: selectedIconThemeColor);
    const IconThemeData unselectedIconTheme = IconThemeData(
      size: 18,
      color: unselectedIconThemeColor,
    );
    const TextStyle selectedTextStyle = TextStyle(fontSize: 18.0, color: selectedLabelColor);
    const TextStyle unselectedTextStyle = TextStyle(fontSize: 18.0, color: unselectedLabelColor);

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(primaryColor: primaryColor, unselectedWidgetColor: unselectedWidgetColor),
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.shifting,
            selectedLabelStyle: selectedTextStyle,
            unselectedLabelStyle: unselectedTextStyle,
            selectedIconTheme: selectedIconTheme,
            unselectedIconTheme: unselectedIconTheme,
            useLegacyColorScheme: false,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
          ),
        ),
      ),
    );

    final TextStyle selectedIcon = _iconStyle(tester, Icons.ac_unit);
    final TextStyle unselectedIcon = _iconStyle(tester, Icons.access_alarm);

    expect(selectedIcon.color, equals(selectedIconThemeColor));
    expect(unselectedIcon.color, equals(unselectedIconThemeColor));
  });

  testWidgets('iconTheme color should override itemColor color for BottomNavigationBarType.fixed', (
    WidgetTester tester,
  ) async {
    const Color primaryColor = Color(0xFF000000);
    const Color unselectedWidgetColor = Color(0xFFD501FF);
    const Color selectedIconThemeColor = Color(0xFF1E7723);
    const Color unselectedIconThemeColor = Color(0xFF009688);
    const IconThemeData selectedIconTheme = IconThemeData(size: 20, color: selectedIconThemeColor);
    const IconThemeData unselectedIconTheme = IconThemeData(
      size: 18,
      color: unselectedIconThemeColor,
    );

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(primaryColor: primaryColor, unselectedWidgetColor: unselectedWidgetColor),
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.fixed,
            selectedIconTheme: selectedIconTheme,
            unselectedIconTheme: unselectedIconTheme,
            useLegacyColorScheme: false,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
          ),
        ),
      ),
    );

    final TextStyle selectedIcon = _iconStyle(tester, Icons.ac_unit);
    final TextStyle unselectedIcon = _iconStyle(tester, Icons.access_alarm);

    expect(selectedIcon.color, equals(selectedIconThemeColor));
    expect(unselectedIcon.color, equals(unselectedIconThemeColor));
  });

  testWidgets('iconTheme color should override itemColor for BottomNavigationBarType.shifted', (
    WidgetTester tester,
  ) async {
    const Color primaryColor = Color(0xFF000000);
    const Color unselectedWidgetColor = Color(0xFFD501FF);
    const Color selectedColor = Color(0xFF0004FF);
    const Color unselectedColor = Color(0xFFE5FF00);
    const Color selectedIconThemeColor = Color(0xFF1E7723);
    const Color unselectedIconThemeColor = Color(0xFF009688);
    const IconThemeData selectedIconTheme = IconThemeData(size: 20, color: selectedIconThemeColor);
    const IconThemeData unselectedIconTheme = IconThemeData(
      size: 18,
      color: unselectedIconThemeColor,
    );

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(primaryColor: primaryColor, unselectedWidgetColor: unselectedWidgetColor),
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.shifting,
            selectedIconTheme: selectedIconTheme,
            unselectedIconTheme: unselectedIconTheme,
            selectedItemColor: selectedColor,
            unselectedItemColor: unselectedColor,
            useLegacyColorScheme: false,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
          ),
        ),
      ),
    );

    final TextStyle selectedIcon = _iconStyle(tester, Icons.ac_unit);
    final TextStyle unselectedIcon = _iconStyle(tester, Icons.access_alarm);

    expect(selectedIcon.color, equals(selectedIconThemeColor));
    expect(unselectedIcon.color, equals(unselectedIconThemeColor));
  });

  testWidgets('Fixed BottomNavigationBar can hide unselected labels', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.fixed,
            showUnselectedLabels: false,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
          ),
        ),
      ),
    );

    expect(_getOpacity(tester, 'AC'), equals(1.0));
    expect(_getOpacity(tester, 'Alarm'), equals(0.0));
  });

  testWidgets('Fixed BottomNavigationBar can update background color', (WidgetTester tester) async {
    const Color color = Colors.yellow;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.fixed,
            backgroundColor: color,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
          ),
        ),
      ),
    );

    expect(_getMaterial(tester).color, equals(color));
  });

  testWidgets('Shifting BottomNavigationBar background color is overridden by item color', (
    WidgetTester tester,
  ) async {
    const Color itemColor = Colors.yellow;
    const Color backgroundColor = Colors.blue;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.shifting,
            backgroundColor: backgroundColor,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(
                icon: Icon(Icons.ac_unit),
                label: 'AC',
                backgroundColor: itemColor,
              ),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
          ),
        ),
      ),
    );

    expect(_getMaterial(tester).color, equals(itemColor));
  });

  testWidgets('Specifying both selectedItemColor and fixedColor asserts', (
    WidgetTester tester,
  ) async {
    expect(() {
      return BottomNavigationBar(
        selectedItemColor: Colors.black,
        fixedColor: Colors.black,
        items: const <BottomNavigationBarItem>[
          BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
          BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
        ],
      );
    }, throwsAssertionError);
  });

  testWidgets('Fixed BottomNavigationBar uses fixedColor when selectedItemColor not provided', (
    WidgetTester tester,
  ) async {
    const Color fixedColor = Colors.black;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.fixed,
            fixedColor: fixedColor,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
          ),
        ),
      ),
    );

    expect(
      tester.renderObject<RenderParagraph>(find.text('AC')).text.style!.color,
      equals(fixedColor),
    );
  });

  testWidgets('setting selectedFontSize to zero hides all labels', (WidgetTester tester) async {
    const double customElevation = 3.0;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.fixed,
            elevation: customElevation,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
          ),
        ),
      ),
    );

    expect(_getMaterial(tester).elevation, equals(customElevation));
  });

  testWidgets('Material2 - BottomNavigationBar adds bottom padding to height', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: MediaQuery(
          data: const MediaQueryData(viewPadding: EdgeInsets.only(bottom: 40.0)),
          child: Scaffold(
            bottomNavigationBar: BottomNavigationBar(
              items: const <BottomNavigationBarItem>[
                BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
                BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
              ],
            ),
          ),
        ),
      ),
    );

    const double expectedHeight = kBottomNavigationBarHeight + 40.0;
    expect(tester.getSize(find.byType(BottomNavigationBar)).height, expectedHeight);
  });

  testWidgets('Material3 - BottomNavigationBar adds bottom padding to height', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: MediaQuery(
          data: const MediaQueryData(viewPadding: EdgeInsets.only(bottom: 40.0)),
          child: Scaffold(
            bottomNavigationBar: BottomNavigationBar(
              items: const <BottomNavigationBarItem>[
                BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
                BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
              ],
            ),
          ),
        ),
      ),
    );

    const double expectedMinHeight = kBottomNavigationBarHeight + 40.0;
    expect(tester.getSize(find.byType(BottomNavigationBar)).height >= expectedMinHeight, isTrue);
  });

  testWidgets('BottomNavigationBar adds bottom padding to height with a custom font size', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: MediaQuery(
          data: const MediaQueryData(viewPadding: EdgeInsets.only(bottom: 40.0)),
          child: Scaffold(
            bottomNavigationBar: BottomNavigationBar(
              selectedFontSize: 8,
              items: const <BottomNavigationBarItem>[
                BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
                BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
              ],
            ),
          ),
        ),
      ),
    );

    const double expectedHeight = kBottomNavigationBarHeight + 40.0;
    expect(tester.getSize(find.byType(BottomNavigationBar)).height, expectedHeight);
  });

  testWidgets('BottomNavigationBar height will not change when toggle keyboard', (
    WidgetTester tester,
  ) async {
    final Widget child = Scaffold(
      bottomNavigationBar: BottomNavigationBar(
        selectedFontSize: 8,
        items: const <BottomNavigationBarItem>[
          BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
          BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
        ],
      ),
    );

    // Test the bar height is correct when not showing the keyboard.
    await tester.pumpWidget(
      MaterialApp(
        home: MediaQuery(
          data: const MediaQueryData(
            viewPadding: EdgeInsets.only(bottom: 40.0),
            padding: EdgeInsets.only(bottom: 40.0),
          ),
          child: child,
        ),
      ),
    );

    // Expect the height is the correct.
    const double expectedHeight = kBottomNavigationBarHeight + 40.0;
    expect(tester.getSize(find.byType(BottomNavigationBar)).height, expectedHeight);

    // Now we show the keyboard.
    await tester.pumpWidget(
      MaterialApp(
        home: MediaQuery(
          data: const MediaQueryData(
            viewPadding: EdgeInsets.only(bottom: 40.0),
            viewInsets: EdgeInsets.only(bottom: 336.0),
          ),
          child: child,
        ),
      ),
    );

    // Expect the height is the same.
    expect(tester.getSize(find.byType(BottomNavigationBar)).height, expectedHeight);
  });

  testWidgets('BottomNavigationBar action size test', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.shifting,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
          ),
        ),
      ),
    );

    Iterable<RenderBox> actions = tester.renderObjectList(find.byType(InkResponse));
    expect(actions.length, 2);
    expect(actions.elementAt(0).size.width, 480.0);
    expect(actions.elementAt(1).size.width, 320.0);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            currentIndex: 1,
            type: BottomNavigationBarType.shifting,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
          ),
        ),
      ),
    );

    await tester.pump(const Duration(milliseconds: 200));

    actions = tester.renderObjectList(find.byType(InkResponse));
    expect(actions.length, 2);
    expect(actions.elementAt(0).size.width, 320.0);
    expect(actions.elementAt(1).size.width, 480.0);
  });

  testWidgets('BottomNavigationBar multiple taps test', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.shifting,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
              BottomNavigationBarItem(icon: Icon(Icons.access_time), label: 'Time'),
              BottomNavigationBarItem(icon: Icon(Icons.add), label: 'Add'),
            ],
          ),
        ),
      ),
    );

    // We want to make sure that the last label does not get displaced,
    // irrespective of how many taps happen on the first N - 1 labels and how
    // they grow.

    Iterable<RenderBox> actions = tester.renderObjectList(find.byType(InkResponse));
    final Offset originalOrigin = actions.elementAt(3).localToGlobal(Offset.zero);

    await tester.tap(find.text('AC'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));

    actions = tester.renderObjectList(find.byType(InkResponse));
    expect(actions.elementAt(3).localToGlobal(Offset.zero), equals(originalOrigin));

    await tester.tap(find.text('Alarm'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));

    actions = tester.renderObjectList(find.byType(InkResponse));
    expect(actions.elementAt(3).localToGlobal(Offset.zero), equals(originalOrigin));

    await tester.tap(find.text('Time'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));

    actions = tester.renderObjectList(find.byType(InkResponse));
    expect(actions.elementAt(3).localToGlobal(Offset.zero), equals(originalOrigin));
  });

  testWidgets('BottomNavigationBar inherits shadowed app theme for shifting navbar', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(brightness: Brightness.light),
        home: Theme(
          data: ThemeData(brightness: Brightness.dark),
          child: Scaffold(
            bottomNavigationBar: BottomNavigationBar(
              type: BottomNavigationBarType.shifting,
              items: const <BottomNavigationBarItem>[
                BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
                BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
                BottomNavigationBarItem(icon: Icon(Icons.access_time), label: 'Time'),
                BottomNavigationBarItem(icon: Icon(Icons.add), label: 'Add'),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text('Alarm'));
    await tester.pump(const Duration(seconds: 1));
    expect(Theme.of(tester.element(find.text('Alarm'))).brightness, equals(Brightness.dark));
  });

  testWidgets('BottomNavigationBar inherits shadowed app theme for fixed navbar', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(brightness: Brightness.light),
        home: Theme(
          data: ThemeData(brightness: Brightness.dark),
          child: Scaffold(
            bottomNavigationBar: BottomNavigationBar(
              type: BottomNavigationBarType.fixed,
              items: const <BottomNavigationBarItem>[
                BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
                BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
                BottomNavigationBarItem(icon: Icon(Icons.access_time), label: 'Time'),
                BottomNavigationBarItem(icon: Icon(Icons.add), label: 'Add'),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text('Alarm'));
    await tester.pump(const Duration(seconds: 1));
    expect(Theme.of(tester.element(find.text('Alarm'))).brightness, equals(Brightness.dark));
  });

  testWidgets('BottomNavigationBar iconSize test', (WidgetTester tester) async {
    late double builderIconSize;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            iconSize: 12.0,
            items: <BottomNavigationBarItem>[
              const BottomNavigationBarItem(label: 'A', icon: Icon(Icons.ac_unit)),
              BottomNavigationBarItem(
                label: 'B',
                icon: Builder(
                  builder: (BuildContext context) {
                    builderIconSize = IconTheme.of(context).size!;
                    return SizedBox(width: builderIconSize, height: builderIconSize);
                  },
                ),
              ),
            ],
          ),
        ),
      ),
    );

    final RenderBox box = tester.renderObject(find.byType(Icon));
    expect(box.size.width, equals(12.0));
    expect(box.size.height, equals(12.0));
    expect(builderIconSize, 12.0);
  });

  testWidgets('Material2 - BottomNavigationBar responds to textScaleFactor', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.fixed,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(label: 'A', icon: Icon(Icons.ac_unit)),
              BottomNavigationBarItem(label: 'B', icon: Icon(Icons.battery_alert)),
            ],
          ),
        ),
      ),
    );
    final RenderBox defaultBox = tester.renderObject(find.byType(BottomNavigationBar));
    expect(defaultBox.size.height, equals(kBottomNavigationBarHeight));
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.shifting,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(label: 'A', icon: Icon(Icons.ac_unit)),
              BottomNavigationBarItem(label: 'B', icon: Icon(Icons.battery_alert)),
            ],
          ),
        ),
      ),
    );
    final RenderBox shiftingBox = tester.renderObject(find.byType(BottomNavigationBar));
    expect(shiftingBox.size.height, equals(kBottomNavigationBarHeight));
    await tester.pumpWidget(
      MaterialApp(
        home: MediaQuery.withClampedTextScaling(
          minScaleFactor: 2.0,
          maxScaleFactor: 2.0,
          child: Scaffold(
            bottomNavigationBar: BottomNavigationBar(
              items: const <BottomNavigationBarItem>[
                BottomNavigationBarItem(label: 'A', icon: Icon(Icons.ac_unit)),
                BottomNavigationBarItem(label: 'B', icon: Icon(Icons.battery_alert)),
              ],
            ),
          ),
        ),
      ),
    );

    final RenderBox box = tester.renderObject(find.byType(BottomNavigationBar));
    expect(box.size.height, equals(56.0));
  });

  testWidgets('Material3 - BottomNavigationBar responds to textScaleFactor', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.fixed,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(label: 'A', icon: Icon(Icons.ac_unit)),
              BottomNavigationBarItem(label: 'B', icon: Icon(Icons.battery_alert)),
            ],
          ),
        ),
      ),
    );
    final RenderBox defaultBox = tester.renderObject(find.byType(BottomNavigationBar));
    // kBottomNavigationBarHeight is a minimum dimension.
    expect(defaultBox.size.height, greaterThanOrEqualTo(kBottomNavigationBarHeight));

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.shifting,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(label: 'A', icon: Icon(Icons.ac_unit)),
              BottomNavigationBarItem(label: 'B', icon: Icon(Icons.battery_alert)),
            ],
          ),
        ),
      ),
    );
    final RenderBox shiftingBox = tester.renderObject(find.byType(BottomNavigationBar));
    // kBottomNavigationBarHeight is a minimum dimension.
    expect(shiftingBox.size.height, greaterThanOrEqualTo(kBottomNavigationBarHeight));

    await tester.pumpWidget(
      MaterialApp(
        home: MediaQuery.withClampedTextScaling(
          minScaleFactor: 2.0,
          maxScaleFactor: 2.0,
          child: Scaffold(
            bottomNavigationBar: BottomNavigationBar(
              items: const <BottomNavigationBarItem>[
                BottomNavigationBarItem(label: 'A', icon: Icon(Icons.ac_unit)),
                BottomNavigationBarItem(label: 'B', icon: Icon(Icons.battery_alert)),
              ],
            ),
          ),
        ),
      ),
    );

    final RenderBox box = tester.renderObject(find.byType(BottomNavigationBar));
    // kBottomNavigationBarHeight is a minimum dimension.
    expect(box.size.height, greaterThanOrEqualTo(kBottomNavigationBarHeight));
  });

  testWidgets(
    'Material2 - BottomNavigationBar does not grow with textScaleFactor when labels are provided',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Scaffold(
            bottomNavigationBar: BottomNavigationBar(
              type: BottomNavigationBarType.fixed,
              items: const <BottomNavigationBarItem>[
                BottomNavigationBarItem(label: 'A', icon: Icon(Icons.ac_unit)),
                BottomNavigationBarItem(label: 'B', icon: Icon(Icons.battery_alert)),
              ],
            ),
          ),
        ),
      );

      final RenderBox defaultBox = tester.renderObject(find.byType(BottomNavigationBar));
      expect(defaultBox.size.height, equals(kBottomNavigationBarHeight));

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            bottomNavigationBar: BottomNavigationBar(
              type: BottomNavigationBarType.shifting,
              items: const <BottomNavigationBarItem>[
                BottomNavigationBarItem(label: 'A', icon: Icon(Icons.ac_unit)),
                BottomNavigationBarItem(label: 'B', icon: Icon(Icons.battery_alert)),
              ],
            ),
          ),
        ),
      );

      final RenderBox shiftingBox = tester.renderObject(find.byType(BottomNavigationBar));
      expect(shiftingBox.size.height, equals(kBottomNavigationBarHeight));

      await tester.pumpWidget(
        MaterialApp(
          home: MediaQuery(
            data: const MediaQueryData(textScaler: TextScaler.linear(2.0)),
            child: Scaffold(
              bottomNavigationBar: BottomNavigationBar(
                items: const <BottomNavigationBarItem>[
                  BottomNavigationBarItem(label: 'A', icon: Icon(Icons.ac_unit)),
                  BottomNavigationBarItem(label: 'B', icon: Icon(Icons.battery_alert)),
                ],
              ),
            ),
          ),
        ),
      );

      final RenderBox box = tester.renderObject(find.byType(BottomNavigationBar));
      expect(box.size.height, equals(kBottomNavigationBarHeight));
    },
  );

  testWidgets(
    'Material3 - BottomNavigationBar does not grow with textScaleFactor when labels are provided',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            bottomNavigationBar: BottomNavigationBar(
              type: BottomNavigationBarType.fixed,
              items: const <BottomNavigationBarItem>[
                BottomNavigationBarItem(label: 'A', icon: Icon(Icons.ac_unit)),
                BottomNavigationBarItem(label: 'B', icon: Icon(Icons.battery_alert)),
              ],
            ),
          ),
        ),
      );

      final RenderBox defaultBox = tester.renderObject(find.byType(BottomNavigationBar));
      // kBottomNavigationBarHeight is a minimum dimension.
      expect(defaultBox.size.height, greaterThanOrEqualTo(kBottomNavigationBarHeight));

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            bottomNavigationBar: BottomNavigationBar(
              type: BottomNavigationBarType.shifting,
              items: const <BottomNavigationBarItem>[
                BottomNavigationBarItem(label: 'A', icon: Icon(Icons.ac_unit)),
                BottomNavigationBarItem(label: 'B', icon: Icon(Icons.battery_alert)),
              ],
            ),
          ),
        ),
      );

      final RenderBox shiftingBox = tester.renderObject(find.byType(BottomNavigationBar));
      // kBottomNavigationBarHeight is a minimum dimension.
      expect(shiftingBox.size.height, greaterThanOrEqualTo(kBottomNavigationBarHeight));

      await tester.pumpWidget(
        MaterialApp(
          home: MediaQuery(
            data: const MediaQueryData(textScaler: TextScaler.linear(2.0)),
            child: Scaffold(
              bottomNavigationBar: BottomNavigationBar(
                items: const <BottomNavigationBarItem>[
                  BottomNavigationBarItem(label: 'A', icon: Icon(Icons.ac_unit)),
                  BottomNavigationBarItem(label: 'B', icon: Icon(Icons.battery_alert)),
                ],
              ),
            ),
          ),
        ),
      );

      final RenderBox box = tester.renderObject(find.byType(BottomNavigationBar));
      expect(box.size.height, equals(defaultBox.size.height));
      expect(box.size.height, equals(shiftingBox.size.height));
    },
  );

  testWidgets(
    'Material2 - BottomNavigationBar shows tool tips with text scaling on long press when labels are provided',
    (WidgetTester tester) async {
      const String label = 'Foo';

      Widget buildApp({required double textScaleFactor}) {
        return MediaQuery.withClampedTextScaling(
          minScaleFactor: textScaleFactor,
          maxScaleFactor: textScaleFactor,
          child: Localizations(
            locale: const Locale('en', 'US'),
            delegates: const <LocalizationsDelegate<dynamic>>[
              DefaultMaterialLocalizations.delegate,
              DefaultWidgetsLocalizations.delegate,
            ],
            child: Directionality(
              textDirection: TextDirection.ltr,
              child: Navigator(
                onGenerateRoute: (RouteSettings settings) {
                  return MaterialPageRoute<void>(
                    builder: (BuildContext context) {
                      return MaterialApp(
                        theme: ThemeData(useMaterial3: false),
                        home: Scaffold(
                          bottomNavigationBar: BottomNavigationBar(
                            items: const <BottomNavigationBarItem>[
                              BottomNavigationBarItem(
                                label: label,
                                icon: Icon(Icons.ac_unit),
                                tooltip: label,
                              ),
                              BottomNavigationBarItem(label: 'B', icon: Icon(Icons.battery_alert)),
                            ],
                          ),
                        ),
                      );
                    },
                  );
                },
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildApp(textScaleFactor: 1.0));
      expect(find.text(label), findsOneWidget);
      await tester.longPress(find.text(label));
      expect(find.text(label), findsNWidgets(2));
      expect(tester.getSize(find.text(label).last), equals(const Size(42.0, 14.0)));
      await tester.pumpAndSettle(const Duration(seconds: 2));

      await tester.pumpWidget(buildApp(textScaleFactor: 4.0));
      expect(find.text(label), findsOneWidget);
      await tester.longPress(find.text(label));
      expect(tester.getSize(find.text(label).last), equals(const Size(168.0, 56.0)));
    },
  );

  testWidgets(
    'Material3 - BottomNavigationBar shows tool tips with text scaling on long press when labels are provided',
    (WidgetTester tester) async {
      const String label = 'Foo';

      Widget buildApp({required TextScaler textScaler}) {
        return MediaQuery(
          data: MediaQueryData(textScaler: textScaler),
          child: Localizations(
            locale: const Locale('en', 'US'),
            delegates: const <LocalizationsDelegate<dynamic>>[
              DefaultMaterialLocalizations.delegate,
              DefaultWidgetsLocalizations.delegate,
            ],
            child: Directionality(
              textDirection: TextDirection.ltr,
              child: Navigator(
                onGenerateRoute: (RouteSettings settings) {
                  return MaterialPageRoute<void>(
                    builder: (BuildContext context) {
                      return MaterialApp(
                        home: Scaffold(
                          bottomNavigationBar: BottomNavigationBar(
                            items: const <BottomNavigationBarItem>[
                              BottomNavigationBarItem(
                                label: label,
                                icon: Icon(Icons.ac_unit),
                                tooltip: label,
                              ),
                              BottomNavigationBarItem(label: 'B', icon: Icon(Icons.battery_alert)),
                            ],
                          ),
                        ),
                      );
                    },
                  );
                },
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildApp(textScaler: TextScaler.noScaling));
      expect(find.text(label), findsOneWidget);
      await tester.longPress(find.text(label));
      expect(find.text(label), findsNWidgets(2));
      expect(tester.getSize(find.text(label).last).height, equals(20.0));
      await tester.pumpAndSettle(const Duration(seconds: 2));

      await tester.pumpWidget(buildApp(textScaler: const TextScaler.linear(4.0)));
      expect(find.text(label), findsOneWidget);
      await tester.longPress(find.text(label));
      expect(tester.getSize(find.text(label).last).height, equals(80.0));
    },
  );

  testWidgets('Different behaviour of tool tip in BottomNavigationBarItem', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(label: 'A', tooltip: 'A tooltip', icon: Icon(Icons.ac_unit)),
              BottomNavigationBarItem(label: 'B', icon: Icon(Icons.battery_alert)),
              BottomNavigationBarItem(label: 'C', icon: Icon(Icons.cake), tooltip: ''),
            ],
          ),
        ),
      ),
    );

    expect(find.text('A'), findsOneWidget);
    await tester.longPress(find.text('A'));
    expect(find.byTooltip('A tooltip'), findsOneWidget);

    expect(find.text('B'), findsOneWidget);
    await tester.longPress(find.text('B'));
    expect(find.byTooltip('B'), findsNothing);

    expect(find.text('C'), findsOneWidget);
    await tester.longPress(find.text('C'));
    expect(find.byTooltip('C'), findsNothing);
  });

  testWidgets('BottomNavigationBar limits width of tiles with long labels', (
    WidgetTester tester,
  ) async {
    final String longTextA = List<String>.generate(100, (int index) => 'A').toString();
    final String longTextB = List<String>.generate(100, (int index) => 'B').toString();

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            items: <BottomNavigationBarItem>[
              BottomNavigationBarItem(label: longTextA, icon: const Icon(Icons.ac_unit)),
              BottomNavigationBarItem(label: longTextB, icon: const Icon(Icons.battery_alert)),
            ],
          ),
        ),
      ),
    );

    final RenderBox box = tester.renderObject(find.byType(BottomNavigationBar));
    expect(box.size.height, greaterThanOrEqualTo(kBottomNavigationBarHeight));

    final RenderBox itemBoxA = tester.renderObject(find.text(longTextA));
    expect(itemBoxA.size.width, equals(400.0));
    final RenderBox itemBoxB = tester.renderObject(find.text(longTextB));
    expect(itemBoxB.size.width, equals(400.0));
  });

  testWidgets('Material2 - BottomNavigationBar paints circles', (WidgetTester tester) async {
    await tester.pumpWidget(
      boilerplate(
        useMaterial3: false,
        textDirection: TextDirection.ltr,
        bottomNavigationBar: BottomNavigationBar(
          items: const <BottomNavigationBarItem>[
            BottomNavigationBarItem(label: 'A', icon: Icon(Icons.ac_unit)),
            BottomNavigationBarItem(label: 'B', icon: Icon(Icons.battery_alert)),
          ],
        ),
      ),
    );

    final RenderBox box = tester.renderObject(find.byType(BottomNavigationBar));
    expect(box, isNot(paints..circle()));

    await tester.tap(find.text('A'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 20));
    expect(box, paints..circle(x: 200.0));

    await tester.tap(find.text('B'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 20));
    expect(
      box,
      paints
        ..circle(x: 200.0)
        ..translate(x: 400.0)
        ..circle(x: 200.0),
    );

    // Now we flip the directionality and verify that the circles switch positions.
    await tester.pumpWidget(
      boilerplate(
        useMaterial3: false,
        textDirection: TextDirection.rtl,
        bottomNavigationBar: BottomNavigationBar(
          items: const <BottomNavigationBarItem>[
            BottomNavigationBarItem(label: 'A', icon: Icon(Icons.ac_unit)),
            BottomNavigationBarItem(label: 'B', icon: Icon(Icons.battery_alert)),
          ],
        ),
      ),
    );

    expect(
      box,
      paints
        ..translate()
        ..save()
        ..translate(x: 400.0)
        ..circle(x: 200.0)
        ..restore()
        ..circle(x: 200.0),
    );

    await tester.tap(find.text('A'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 20));
    expect(
      box,
      paints
        ..translate(x: 0.0, y: 0.0)
        ..save()
        ..translate(x: 400.0)
        ..circle(x: 200.0)
        ..restore()
        ..circle(x: 200.0)
        ..translate(x: 400.0)
        ..circle(x: 200.0),
    );
  });

  testWidgets('BottomNavigationBar inactiveIcon shown', (WidgetTester tester) async {
    const Key filled = Key('filled');
    const Key stroked = Key('stroked');
    int selectedItem = 0;

    await tester.pumpWidget(
      boilerplate(
        textDirection: TextDirection.ltr,
        bottomNavigationBar: BottomNavigationBar(
          currentIndex: selectedItem,
          items: const <BottomNavigationBarItem>[
            BottomNavigationBarItem(
              activeIcon: Icon(Icons.favorite, key: filled),
              icon: Icon(Icons.favorite_border, key: stroked),
              label: 'Favorite',
            ),
            BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
          ],
        ),
      ),
    );

    expect(find.byKey(filled), findsOneWidget);
    expect(find.byKey(stroked), findsNothing);
    selectedItem = 1;

    await tester.pumpWidget(
      boilerplate(
        textDirection: TextDirection.ltr,
        bottomNavigationBar: BottomNavigationBar(
          currentIndex: selectedItem,
          items: const <BottomNavigationBarItem>[
            BottomNavigationBarItem(
              activeIcon: Icon(Icons.favorite, key: filled),
              icon: Icon(Icons.favorite_border, key: stroked),
              label: 'Favorite',
            ),
            BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
          ],
        ),
      ),
    );

    expect(find.byKey(filled), findsNothing);
    expect(find.byKey(stroked), findsOneWidget);
  });

  testWidgets('BottomNavigationBar.fixed semantics', (WidgetTester tester) async {
    await tester.pumpWidget(
      boilerplate(
        textDirection: TextDirection.ltr,
        bottomNavigationBar: BottomNavigationBar(
          items: const <BottomNavigationBarItem>[
            BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
            BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            BottomNavigationBarItem(icon: Icon(Icons.hot_tub), label: 'Hot Tub'),
          ],
        ),
      ),
    );

    expect(
      tester.getSemantics(find.text('AC')),
      matchesSemantics(
        label: 'AC\nTab 1 of 3',
        textDirection: TextDirection.ltr,
        isButton: true,
        isFocusable: true,
        isSelected: true,
        hasSelectedState: true,
        hasTapAction: true,
        hasFocusAction: true,
      ),
    );
    expect(
      tester.getSemantics(find.text('Alarm')),
      matchesSemantics(
        label: 'Alarm\nTab 2 of 3',
        textDirection: TextDirection.ltr,
        isButton: true,
        isFocusable: true,
        hasSelectedState: true,
        hasTapAction: true,
        hasFocusAction: true,
      ),
    );
    expect(
      tester.getSemantics(find.text('Hot Tub')),
      matchesSemantics(
        label: 'Hot Tub\nTab 3 of 3',
        textDirection: TextDirection.ltr,
        isButton: true,
        isFocusable: true,
        hasSelectedState: true,
        hasTapAction: true,
        hasFocusAction: true,
      ),
    );
  });

  testWidgets('BottomNavigationBar.shifting semantics', (WidgetTester tester) async {
    await tester.pumpWidget(
      boilerplate(
        textDirection: TextDirection.ltr,
        bottomNavigationBar: BottomNavigationBar(
          type: BottomNavigationBarType.shifting,
          items: const <BottomNavigationBarItem>[
            BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
            BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            BottomNavigationBarItem(icon: Icon(Icons.hot_tub), label: 'Hot Tub'),
          ],
        ),
      ),
    );

    expect(
      tester.getSemantics(find.text('AC')),
      matchesSemantics(
        label: 'AC\nTab 1 of 3',
        textDirection: TextDirection.ltr,
        isButton: true,
        isFocusable: true,
        isSelected: true,
        hasSelectedState: true,
        hasTapAction: true,
        hasFocusAction: true,
      ),
    );
    expect(
      tester.getSemantics(find.text('Alarm')),
      matchesSemantics(
        label: 'Alarm\nTab 2 of 3',
        textDirection: TextDirection.ltr,
        isButton: true,
        isFocusable: true,
        hasSelectedState: true,
        hasTapAction: true,
        hasFocusAction: true,
      ),
    );
    expect(
      tester.getSemantics(find.text('Hot Tub')),
      matchesSemantics(
        label: 'Hot Tub\nTab 3 of 3',
        textDirection: TextDirection.ltr,
        isButton: true,
        isFocusable: true,
        hasSelectedState: true,
        hasTapAction: true,
        hasFocusAction: true,
      ),
    );
  });

  testWidgets('BottomNavigationBar handles items.length changes', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/10322

    Widget buildFrame(int itemCount) {
      return MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.fixed,
            items: List<BottomNavigationBarItem>.generate(itemCount, (int itemIndex) {
              return BottomNavigationBarItem(
                icon: const Icon(Icons.android),
                label: 'item $itemIndex',
              );
            }),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(3));
    expect(find.text('item 0'), findsOneWidget);
    expect(find.text('item 1'), findsOneWidget);
    expect(find.text('item 2'), findsOneWidget);
    expect(find.text('item 3'), findsNothing);

    await tester.pumpWidget(buildFrame(4));
    expect(find.text('item 0'), findsOneWidget);
    expect(find.text('item 1'), findsOneWidget);
    expect(find.text('item 2'), findsOneWidget);
    expect(find.text('item 3'), findsOneWidget);

    await tester.pumpWidget(buildFrame(2));
    expect(find.text('item 0'), findsOneWidget);
    expect(find.text('item 1'), findsOneWidget);
    expect(find.text('item 2'), findsNothing);
    expect(find.text('item 3'), findsNothing);
  });

  testWidgets('BottomNavigationBar change backgroundColor test', (WidgetTester tester) async {
    // Regression test for: https://github.com/flutter/flutter/issues/19653

    Color backgroundColor = Colors.red;

    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Scaffold(
              body: Center(
                child: ElevatedButton(
                  child: const Text('green'),
                  onPressed: () {
                    setState(() {
                      backgroundColor = Colors.green;
                    });
                  },
                ),
              ),
              bottomNavigationBar: BottomNavigationBar(
                type: BottomNavigationBarType.shifting,
                items: <BottomNavigationBarItem>[
                  BottomNavigationBarItem(
                    label: 'Page 1',
                    backgroundColor: backgroundColor,
                    icon: const Icon(Icons.dashboard),
                  ),
                  BottomNavigationBarItem(
                    label: 'Page 2',
                    backgroundColor: backgroundColor,
                    icon: const Icon(Icons.menu),
                  ),
                ],
              ),
            );
          },
        ),
      ),
    );

    final Finder backgroundMaterial = find.descendant(
      of: find.byType(BottomNavigationBar),
      matching: find.byWidgetPredicate((Widget w) {
        if (w is Material) {
          return w.type == MaterialType.canvas;
        }
        return false;
      }),
    );

    expect(backgroundColor, Colors.red);
    expect(tester.widget<Material>(backgroundMaterial).color, Colors.red);
    await tester.tap(find.text('green'));
    await tester.pumpAndSettle();
    expect(backgroundColor, Colors.green);
    expect(tester.widget<Material>(backgroundMaterial).color, Colors.green);
  });

  group('Material2 - BottomNavigationBar shifting backgroundColor with transition', () {
    // Regression test for: https://github.com/flutter/flutter/issues/22226
    Widget runTest() {
      int currentIndex = 0;
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Scaffold(
              bottomNavigationBar: RepaintBoundary(
                child: BottomNavigationBar(
                  type: BottomNavigationBarType.shifting,
                  currentIndex: currentIndex,
                  onTap: (int index) {
                    setState(() {
                      currentIndex = index;
                    });
                  },
                  items: const <BottomNavigationBarItem>[
                    BottomNavigationBarItem(
                      label: 'Red',
                      backgroundColor: Colors.red,
                      icon: Icon(Icons.dashboard),
                    ),
                    BottomNavigationBarItem(
                      label: 'Green',
                      backgroundColor: Colors.green,
                      icon: Icon(Icons.menu),
                    ),
                  ],
                ),
              ),
            );
          },
        ),
      );
    }

    for (int pump = 1; pump < 9; pump++) {
      testWidgets('pump $pump', (WidgetTester tester) async {
        await tester.pumpWidget(runTest());
        await tester.tap(find.text('Green'));

        for (int i = 0; i < pump; i++) {
          await tester.pump(const Duration(milliseconds: 30));
        }
        await expectLater(
          find.byType(BottomNavigationBar),
          matchesGoldenFile('m2_bottom_navigation_bar.shifting_transition.${pump - 1}.png'),
        );
      });
    }
  });

  group('Material3 - BottomNavigationBar shifting backgroundColor with transition', () {
    // Regression test for: https://github.com/flutter/flutter/issues/22226
    Widget runTest() {
      int currentIndex = 0;

      return MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Scaffold(
              bottomNavigationBar: RepaintBoundary(
                child: BottomNavigationBar(
                  type: BottomNavigationBarType.shifting,
                  currentIndex: currentIndex,
                  onTap: (int index) {
                    setState(() {
                      currentIndex = index;
                    });
                  },
                  items: const <BottomNavigationBarItem>[
                    BottomNavigationBarItem(
                      label: 'Red',
                      backgroundColor: Colors.red,
                      icon: Icon(Icons.dashboard),
                    ),
                    BottomNavigationBarItem(
                      label: 'Green',
                      backgroundColor: Colors.green,
                      icon: Icon(Icons.menu),
                    ),
                  ],
                ),
              ),
            );
          },
        ),
      );
    }

    for (int pump = 1; pump < 9; pump++) {
      testWidgets('pump $pump', (WidgetTester tester) async {
        await tester.pumpWidget(runTest());
        await tester.tap(find.text('Green'));

        for (int i = 0; i < pump; i++) {
          await tester.pump(const Duration(milliseconds: 30));
        }
        await expectLater(
          find.byType(BottomNavigationBar),
          matchesGoldenFile('m3_bottom_navigation_bar.shifting_transition.${pump - 1}.png'),
        );
      });
    }
  });

  testWidgets('BottomNavigationBar item label should not be nullable', (WidgetTester tester) async {
    expect(() {
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            type: BottomNavigationBarType.shifting,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm)),
            ],
          ),
        ),
      );
    }, throwsAssertionError);
  });

  testWidgets('BottomNavigationBar [showSelectedLabels]=false and [showUnselectedLabels]=false '
      'for shifting navbar, expect that there is no rendered text', (WidgetTester tester) async {
    final Widget widget = MaterialApp(
      home: StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return Scaffold(
            bottomNavigationBar: BottomNavigationBar(
              showSelectedLabels: false,
              showUnselectedLabels: false,
              type: BottomNavigationBarType.shifting,
              items: const <BottomNavigationBarItem>[
                BottomNavigationBarItem(
                  label: 'Red',
                  backgroundColor: Colors.red,
                  icon: Icon(Icons.dashboard),
                ),
                BottomNavigationBarItem(
                  label: 'Green',
                  backgroundColor: Colors.green,
                  icon: Icon(Icons.menu),
                ),
              ],
            ),
          );
        },
      ),
    );
    await tester.pumpWidget(widget);
    expect(find.text('Red'), findsOneWidget);
    expect(find.text('Green'), findsOneWidget);
    expect(tester.widget<Visibility>(find.byType(Visibility).first).visible, false);
    expect(tester.widget<Visibility>(find.byType(Visibility).last).visible, false);
  });

  testWidgets('BottomNavigationBar [showSelectedLabels]=false and [showUnselectedLabels]=false '
      'for fixed navbar, expect that there is no rendered text', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Scaffold(
              bottomNavigationBar: BottomNavigationBar(
                showSelectedLabels: false,
                showUnselectedLabels: false,
                type: BottomNavigationBarType.fixed,
                items: const <BottomNavigationBarItem>[
                  BottomNavigationBarItem(
                    label: 'Red',
                    backgroundColor: Colors.red,
                    icon: Icon(Icons.dashboard),
                  ),
                  BottomNavigationBarItem(
                    label: 'Green',
                    backgroundColor: Colors.green,
                    icon: Icon(Icons.menu),
                  ),
                ],
              ),
            );
          },
        ),
      ),
    );
    expect(find.text('Red'), findsOneWidget);
    expect(find.text('Green'), findsOneWidget);
    expect(tester.widget<Visibility>(find.byType(Visibility).first).visible, false);
    expect(tester.widget<Visibility>(find.byType(Visibility).last).visible, false);
  });

  testWidgets(
    'BottomNavigationBar.fixed [showSelectedLabels]=false and [showUnselectedLabels]=false semantics',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        boilerplate(
          textDirection: TextDirection.ltr,
          bottomNavigationBar: BottomNavigationBar(
            showSelectedLabels: false,
            showUnselectedLabels: false,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'Red'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Green'),
            ],
          ),
        ),
      );

      expect(
        tester.getSemantics(find.text('Red')),
        matchesSemantics(
          label: 'Red\nTab 1 of 2',
          textDirection: TextDirection.ltr,
          isButton: true,
          isFocusable: true,
          isSelected: true,
          hasSelectedState: true,
          hasTapAction: true,
          hasFocusAction: true,
        ),
      );
      expect(
        tester.getSemantics(find.text('Green')),
        matchesSemantics(
          label: 'Green\nTab 2 of 2',
          textDirection: TextDirection.ltr,
          isButton: true,
          isFocusable: true,
          hasSelectedState: true,
          hasTapAction: true,
          hasFocusAction: true,
        ),
      );
    },
  );

  testWidgets(
    'BottomNavigationBar.shifting [showSelectedLabels]=false and [showUnselectedLabels]=false semantics',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        boilerplate(
          textDirection: TextDirection.ltr,
          bottomNavigationBar: BottomNavigationBar(
            showSelectedLabels: false,
            showUnselectedLabels: false,
            type: BottomNavigationBarType.shifting,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'Red'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Green'),
            ],
          ),
        ),
      );

      expect(
        tester.getSemantics(find.text('Red')),
        matchesSemantics(
          label: 'Red\nTab 1 of 2',
          textDirection: TextDirection.ltr,
          isButton: true,
          isFocusable: true,
          hasSelectedState: true,
          isSelected: true,
          hasTapAction: true,
          hasFocusAction: true,
        ),
      );
      expect(
        tester.getSemantics(find.text('Green')),
        matchesSemantics(
          label: 'Green\nTab 2 of 2',
          textDirection: TextDirection.ltr,
          isButton: true,
          hasSelectedState: true,
          isFocusable: true,
          hasTapAction: true,
          hasFocusAction: true,
        ),
      );
    },
  );

  testWidgets('BottomNavigationBar changes mouse cursor when the tile is hovered over', (
    WidgetTester tester,
  ) async {
    // Test BottomNavigationBar() constructor
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: MouseRegion(
            cursor: SystemMouseCursors.forbidden,
            child: BottomNavigationBar(
              mouseCursor: SystemMouseCursors.text,
              items: const <BottomNavigationBarItem>[
                BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
                BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
              ],
            ),
          ),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: tester.getCenter(find.text('AC')));

    await tester.pumpAndSettle();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );

    // Test default cursor
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: MouseRegion(
            cursor: SystemMouseCursors.forbidden,
            child: BottomNavigationBar(
              items: const <BottomNavigationBarItem>[
                BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
                BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
              ],
            ),
          ),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );
  });

  group('feedback', () {
    late FeedbackTester feedback;

    setUp(() {
      feedback = FeedbackTester();
    });

    tearDown(() {
      feedback.dispose();
    });

    Widget feedbackBoilerplate({bool? enableFeedback, bool? enableFeedbackTheme}) {
      return MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBarTheme(
            data: BottomNavigationBarThemeData(enableFeedback: enableFeedbackTheme),
            child: BottomNavigationBar(
              enableFeedback: enableFeedback,
              items: const <BottomNavigationBarItem>[
                BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
                BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
              ],
            ),
          ),
        ),
      );
    }

    testWidgets('BottomNavigationBar with enabled feedback', (WidgetTester tester) async {
      const bool enableFeedback = true;

      await tester.pumpWidget(feedbackBoilerplate(enableFeedback: enableFeedback));

      await tester.tap(find.byType(InkResponse).first);
      await tester.pumpAndSettle();
      expect(feedback.clickSoundCount, 1);
      expect(feedback.hapticCount, 0);
    });

    testWidgets('BottomNavigationBar with disabled feedback', (WidgetTester tester) async {
      const bool enableFeedback = false;

      await tester.pumpWidget(feedbackBoilerplate(enableFeedback: enableFeedback));

      await tester.tap(find.byType(InkResponse).first);
      await tester.pumpAndSettle();
      expect(feedback.clickSoundCount, 0);
      expect(feedback.hapticCount, 0);
    });

    testWidgets('BottomNavigationBar with enabled feedback by default', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(feedbackBoilerplate());

      await tester.tap(find.byType(InkResponse).first);
      await tester.pumpAndSettle();
      expect(feedback.clickSoundCount, 1);
      expect(feedback.hapticCount, 0);
    });

    testWidgets('BottomNavigationBar with disabled feedback using BottomNavigationBarTheme', (
      WidgetTester tester,
    ) async {
      const bool enableFeedbackTheme = false;

      await tester.pumpWidget(feedbackBoilerplate(enableFeedbackTheme: enableFeedbackTheme));

      await tester.tap(find.byType(InkResponse).first);
      await tester.pumpAndSettle();
      expect(feedback.clickSoundCount, 0);
      expect(feedback.hapticCount, 0);
    });

    testWidgets(
      'BottomNavigationBar.enableFeedback overrides BottomNavigationBarTheme.enableFeedback',
      (WidgetTester tester) async {
        const bool enableFeedbackTheme = false;
        const bool enableFeedback = true;

        await tester.pumpWidget(
          feedbackBoilerplate(
            enableFeedbackTheme: enableFeedbackTheme,
            enableFeedback: enableFeedback,
          ),
        );

        await tester.tap(find.byType(InkResponse).first);
        await tester.pumpAndSettle();
        expect(feedback.clickSoundCount, 1);
        expect(feedback.hapticCount, 0);
      },
    );
  });

  testWidgets('BottomNavigationBar excludes semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(label: 'A', icon: Icon(Icons.ac_unit)),
              BottomNavigationBarItem(label: 'B', icon: Icon(Icons.battery_alert)),
            ],
          ),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              textDirection: TextDirection.ltr,
              children: <TestSemantics>[
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                      children: <TestSemantics>[
                        TestSemantics(
                          children: <TestSemantics>[
                            TestSemantics(
                              flags: <SemanticsFlag>[
                                SemanticsFlag.isButton,
                                SemanticsFlag.isFocusable,
                                SemanticsFlag.hasSelectedState,
                                SemanticsFlag.isSelected,
                              ],
                              actions: <SemanticsAction>[
                                SemanticsAction.tap,
                                SemanticsAction.focus,
                              ],
                              label: 'A\nTab 1 of 2',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              flags: <SemanticsFlag>[
                                SemanticsFlag.isButton,
                                SemanticsFlag.isFocusable,
                                SemanticsFlag.hasSelectedState,
                              ],
                              actions: <SemanticsAction>[
                                SemanticsAction.tap,
                                SemanticsAction.focus,
                              ],
                              label: 'B\nTab 2 of 2',
                              textDirection: TextDirection.ltr,
                            ),
                          ],
                        ),
                      ],
                    ),
                  ],
                ),
              ],
            ),
          ],
        ),
        ignoreId: true,
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('Material2 - BottomNavigationBar default layout', (WidgetTester tester) async {
    final Key icon0 = UniqueKey();
    final Key icon1 = UniqueKey();

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Builder(
          builder: (BuildContext context) {
            return Scaffold(
              bottomNavigationBar: BottomNavigationBar(
                items: <BottomNavigationBarItem>[
                  BottomNavigationBarItem(
                    icon: SizedBox(key: icon0, width: 200, height: 10),
                    label: 'Title0',
                  ),
                  BottomNavigationBarItem(
                    icon: SizedBox(key: icon1, width: 200, height: 10),
                    label: 'Title1',
                  ),
                ],
              ),
            );
          },
        ),
      ),
    );
    expect(
      tester.getSize(find.byType(BottomNavigationBar)),
      const Size(800, kBottomNavigationBarHeight),
    );
    expect(
      tester.getRect(find.byType(BottomNavigationBar)),
      const Rect.fromLTRB(0, 600 - kBottomNavigationBarHeight, 800, 600),
    );

    // The height of the navigation bar is kBottomNavigationBarHeight = 56
    // The top of the navigation bar is 600 - 56 = 544
    // The top and bottom of the selected item is defined by its centered icon/label column:
    //   top = 544 + ((56 - (10 + 10)) / 2) = 562
    //   bottom = top + 10 + 10 = 582
    expect(tester.getRect(find.byKey(icon0)).top, 560.0);
    expect(tester.getRect(find.text('Title0')).bottom, 584.0);

    // The items are padded horizontally according to
    // MainAxisAlignment.spaceAround. Left/right padding is:
    // 800 - (200 * 2) / 4 = 100
    // The layout of the unselected item's label is slightly different; not
    // checking that here.
    expect(tester.getRect(find.text('Title0')), const Rect.fromLTRB(158.0, 570.0, 242.0, 584.0));
    expect(tester.getRect(find.byKey(icon0)), const Rect.fromLTRB(100.0, 560.0, 300.0, 570.0));
    expect(tester.getRect(find.byKey(icon1)), const Rect.fromLTRB(500.0, 560.0, 700.0, 570.0));
  });

  testWidgets('Material3 - BottomNavigationBar default layout', (WidgetTester tester) async {
    final Key icon0 = UniqueKey();
    final Key icon1 = UniqueKey();
    const double iconHeight = 10;

    await tester.pumpWidget(
      MaterialApp(
        home: Builder(
          builder: (BuildContext context) {
            return Scaffold(
              bottomNavigationBar: BottomNavigationBar(
                items: <BottomNavigationBarItem>[
                  BottomNavigationBarItem(
                    icon: SizedBox(key: icon0, width: 200, height: iconHeight),
                    label: 'Title0',
                  ),
                  BottomNavigationBarItem(
                    icon: SizedBox(key: icon1, width: 200, height: iconHeight),
                    label: 'Title1',
                  ),
                ],
              ),
            );
          },
        ),
      ),
    );
    expect(
      tester.getSize(find.byType(BottomNavigationBar)),
      const Size(800, kBottomNavigationBarHeight),
    );
    expect(
      tester.getRect(find.byType(BottomNavigationBar)),
      const Rect.fromLTRB(0, 600 - kBottomNavigationBarHeight, 800, 600),
    );

    const double navigationBarTop = 600 - kBottomNavigationBarHeight; // 544
    const double selectedFontSize = 14.0;
    const double m3LineHeight = 1.43;
    final double labelHeight = (selectedFontSize * m3LineHeight).floorToDouble(); // 20
    const double navigationTileVerticalPadding = selectedFontSize / 2; // 7.0
    final double navigationTileHeight =
        iconHeight + labelHeight + 2 * navigationTileVerticalPadding;

    // Navigation tiles parent is a Row with crossAxisAlignment set to center.
    final double navigationTileVerticalOffset =
        (kBottomNavigationBarHeight - navigationTileHeight) / 2;

    final double iconTop =
        navigationBarTop + navigationTileVerticalOffset + navigationTileVerticalPadding;
    final double labelBottom = 600 - (navigationTileVerticalOffset + navigationTileVerticalPadding);

    expect(tester.getRect(find.byKey(icon0)).top, iconTop);
    expect(tester.getRect(find.text('Title0')).bottom, labelBottom);

    // The items are padded horizontally according to
    // MainAxisAlignment.spaceAround. Left/right padding is:
    // 800 - (200 * 2) / 4 = 100
    // The layout of the unselected item's label is slightly different; not
    // checking that here.
    final double firstLabelWidth = tester.getSize(find.text('Title0')).width;
    const double itemsWidth = 800 / 2; // 2 items.
    const double firstLabelCenter = itemsWidth / 2;
    expect(
      tester.getRect(find.text('Title0')),
      Rect.fromLTRB(
        firstLabelCenter - firstLabelWidth / 2,
        labelBottom - labelHeight,
        firstLabelCenter + firstLabelWidth / 2,
        labelBottom,
      ),
    );
    expect(
      tester.getRect(find.byKey(icon0)),
      Rect.fromLTRB(100.0, iconTop, 300.0, iconTop + iconHeight),
    );
    expect(
      tester.getRect(find.byKey(icon1)),
      Rect.fromLTRB(500.0, iconTop, 700.0, iconTop + iconHeight),
    );
  });

  testWidgets('Material2 - BottomNavigationBar centered landscape layout', (
    WidgetTester tester,
  ) async {
    final Key icon0 = UniqueKey();
    final Key icon1 = UniqueKey();

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Builder(
          builder: (BuildContext context) {
            return Scaffold(
              bottomNavigationBar: BottomNavigationBar(
                landscapeLayout: BottomNavigationBarLandscapeLayout.centered,
                items: <BottomNavigationBarItem>[
                  BottomNavigationBarItem(
                    icon: SizedBox(key: icon0, width: 200, height: 10),
                    label: 'Title0',
                  ),
                  BottomNavigationBarItem(
                    icon: SizedBox(key: icon1, width: 200, height: 10),
                    label: 'Title1',
                  ),
                ],
              ),
            );
          },
        ),
      ),
    );

    expect(
      tester.getSize(find.byType(BottomNavigationBar)),
      const Size(800, kBottomNavigationBarHeight),
    );
    expect(
      tester.getRect(find.byType(BottomNavigationBar)),
      const Rect.fromLTRB(0, 600 - kBottomNavigationBarHeight, 800, 600),
    );

    // The items are laid out as in the default case, within width = 600
    // (the "portrait" width) and the result is centered with the
    // landscape width = 800.
    // So item 0's left edges are:
    // ((800 - 600) / 2) + ((600 - 400) / 4) = 150.
    // Item 1's right edge is:
    // 800 - 150 = 650
    // The layout of the unselected item's label is slightly different; not
    // checking that here.
    expect(tester.getRect(find.text('Title0')), const Rect.fromLTRB(208.0, 570.0, 292.0, 584.0));
    expect(tester.getRect(find.byKey(icon0)), const Rect.fromLTRB(150.0, 560.0, 350.0, 570.0));
    expect(tester.getRect(find.byKey(icon1)), const Rect.fromLTRB(450.0, 560.0, 650.0, 570.0));
  });

  testWidgets('Material3 - BottomNavigationBar centered landscape layout', (
    WidgetTester tester,
  ) async {
    final Key icon0 = UniqueKey();
    final Key icon1 = UniqueKey();
    const double iconWidth = 200;
    const double iconHeight = 10;

    await tester.pumpWidget(
      MaterialApp(
        home: Builder(
          builder: (BuildContext context) {
            return Scaffold(
              bottomNavigationBar: BottomNavigationBar(
                landscapeLayout: BottomNavigationBarLandscapeLayout.centered,
                items: <BottomNavigationBarItem>[
                  BottomNavigationBarItem(
                    icon: SizedBox(key: icon0, width: iconWidth, height: iconHeight),
                    label: 'Title0',
                  ),
                  BottomNavigationBarItem(
                    icon: SizedBox(key: icon1, width: iconWidth, height: iconHeight),
                    label: 'Title1',
                  ),
                ],
              ),
            );
          },
        ),
      ),
    );

    expect(
      tester.getSize(find.byType(BottomNavigationBar)),
      const Size(800, kBottomNavigationBarHeight),
    );
    expect(
      tester.getRect(find.byType(BottomNavigationBar)),
      const Rect.fromLTRB(0, 600 - kBottomNavigationBarHeight, 800, 600),
    );

    const double navigationBarTop = 600 - kBottomNavigationBarHeight; // 544
    const double selectedFontSize = 14.0;
    const double m3LineHeight = 1.43;
    final double labelHeight = (selectedFontSize * m3LineHeight).floorToDouble(); // 20
    const double navigationTileVerticalPadding = selectedFontSize / 2; // 7.0
    final double navigationTileHeight =
        iconHeight + labelHeight + 2 * navigationTileVerticalPadding;

    // Navigation tiles parent is a Row with crossAxisAlignment sets to center.
    final double navigationTileVerticalOffset =
        (kBottomNavigationBarHeight - navigationTileHeight) / 2;

    final double iconTop =
        navigationBarTop + navigationTileVerticalOffset + navigationTileVerticalPadding;
    final double labelBottom = 600 - (navigationTileVerticalOffset + navigationTileVerticalPadding);

    // The items are laid out as in the default case, within width = 600
    // (the "portrait" width) and the result is centered with the
    // landscape width = 800.
    // So item 0's left edges are:
    // ((800 - 600) / 2) + ((600 - 400) / 4) = 150.
    // Item 1's right edge is:
    // 800 - 150 = 650
    // The layout of the unselected item's label is slightly different; not
    // checking that here.
    final double firstLabelWidth = tester.getSize(find.text('Title0')).width;
    const double itemWidth = iconWidth; // 200
    const double firstItemLeft = 150;
    const double firstLabelCenter = firstItemLeft + itemWidth / 2; // 250

    expect(
      tester.getRect(find.text('Title0')),
      Rect.fromLTRB(
        firstLabelCenter - firstLabelWidth / 2,
        labelBottom - labelHeight,
        firstLabelCenter + firstLabelWidth / 2,
        labelBottom,
      ),
    );
    expect(
      tester.getRect(find.byKey(icon0)),
      Rect.fromLTRB(150.0, iconTop, 350.0, iconTop + iconHeight),
    );
    expect(
      tester.getRect(find.byKey(icon1)),
      Rect.fromLTRB(450.0, iconTop, 650.0, iconTop + iconHeight),
    );
  });

  testWidgets('Material2 - BottomNavigationBar linear landscape layout', (
    WidgetTester tester,
  ) async {
    final Key icon0 = UniqueKey();
    final Key icon1 = UniqueKey();

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Builder(
          builder: (BuildContext context) {
            return Scaffold(
              bottomNavigationBar: BottomNavigationBar(
                landscapeLayout: BottomNavigationBarLandscapeLayout.linear,
                items: <BottomNavigationBarItem>[
                  BottomNavigationBarItem(
                    icon: SizedBox(key: icon0, width: 100, height: 20),
                    label: 'Title0',
                  ),
                  BottomNavigationBarItem(
                    icon: SizedBox(key: icon1, width: 100, height: 20),
                    label: 'Title1',
                  ),
                ],
              ),
            );
          },
        ),
      ),
    );

    expect(
      tester.getSize(find.byType(BottomNavigationBar)),
      const Size(800, kBottomNavigationBarHeight),
    );
    expect(
      tester.getRect(find.byType(BottomNavigationBar)),
      const Rect.fromLTRB(0, 600 - kBottomNavigationBarHeight, 800, 600),
    );

    // The items are laid out as in the default case except each
    // item's icon/label is arranged in a row, with 8 pixels in
    // between the icon and label.  The layout of the unselected
    // item's label is slightly different; not checking that here.
    expect(tester.getRect(find.text('Title0')), const Rect.fromLTRB(212.0, 565.0, 296.0, 579.0));
    expect(tester.getRect(find.byKey(icon0)), const Rect.fromLTRB(104.0, 562.0, 204.0, 582.0));
    expect(tester.getRect(find.byKey(icon1)), const Rect.fromLTRB(504.0, 562.0, 604.0, 582.0));
  });

  testWidgets('Material3 - BottomNavigationBar linear landscape layout', (
    WidgetTester tester,
  ) async {
    final Key icon0 = UniqueKey();
    final Key icon1 = UniqueKey();
    const double iconWidth = 100;
    const double iconHeight = 20;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(),
        home: Builder(
          builder: (BuildContext context) {
            return Scaffold(
              bottomNavigationBar: BottomNavigationBar(
                landscapeLayout: BottomNavigationBarLandscapeLayout.linear,
                items: <BottomNavigationBarItem>[
                  BottomNavigationBarItem(
                    icon: SizedBox(key: icon0, width: iconWidth, height: iconHeight),
                    label: 'Title0',
                  ),
                  BottomNavigationBarItem(
                    icon: SizedBox(key: icon1, width: iconWidth, height: iconHeight),
                    label: 'Title1',
                  ),
                ],
              ),
            );
          },
        ),
      ),
    );

    expect(
      tester.getSize(find.byType(BottomNavigationBar)),
      const Size(800, kBottomNavigationBarHeight),
    );
    expect(
      tester.getRect(find.byType(BottomNavigationBar)),
      const Rect.fromLTRB(0, 600 - kBottomNavigationBarHeight, 800, 600),
    );

    const double navigationBarTop = 600 - kBottomNavigationBarHeight; // 544
    const double selectedFontSize = 14.0;
    const double m3LineHeight = 1.43;
    final double labelHeight = (selectedFontSize * m3LineHeight).floorToDouble(); // 20
    const double navigationTileVerticalPadding = selectedFontSize / 2; // 7.0
    // Icon and label are in the same row.
    final double navigationTileHeight =
        max(iconHeight, labelHeight) + 2 * navigationTileVerticalPadding;

    // Navigation tiles parent is a Row with crossAxisAlignment sets to center.
    final double navigationTileVerticalOffset =
        (kBottomNavigationBarHeight - navigationTileHeight) / 2;

    final double iconTop =
        navigationBarTop + navigationTileVerticalOffset + navigationTileVerticalPadding;
    final double labelBottom = 600 - (navigationTileVerticalOffset + navigationTileVerticalPadding);

    // The items are laid out as in the default case except each
    // item's icon/label is arranged in a row, with 8 pixels in
    // between the icon and label.  The layout of the unselected
    // item's label is slightly different; not checking that here.
    const double itemFullWith = 800 / 2; // Two items in the navigation bar.
    const double separatorWidth = 8;
    final double firstLabelWidth = tester.getSize(find.text('Title0')).width;
    final double firstItemContentWidth = iconWidth + separatorWidth + firstLabelWidth;
    final double firstItemLeft = itemFullWith / 2 - firstItemContentWidth / 2;
    final double secondLabelWidth = tester.getSize(find.text('Title1')).width;
    final double secondItemContentWidth = iconWidth + separatorWidth + secondLabelWidth;
    final double secondItemLeft = itemFullWith + itemFullWith / 2 - secondItemContentWidth / 2;

    expect(
      tester.getRect(find.text('Title0')),
      Rect.fromLTRB(
        firstItemLeft + iconWidth + separatorWidth,
        labelBottom - labelHeight,
        firstItemLeft + iconWidth + separatorWidth + firstLabelWidth,
        labelBottom,
      ),
    );
    expect(
      tester.getRect(find.byKey(icon0)),
      Rect.fromLTRB(firstItemLeft, iconTop, firstItemLeft + iconWidth, iconTop + iconHeight),
    );
    expect(
      tester.getRect(find.byKey(icon1)),
      Rect.fromLTRB(secondItemLeft, iconTop, secondItemLeft + iconWidth, iconTop + iconHeight),
    );
  });

  testWidgets('BottomNavigationBar linear landscape layout label RenderFlex overflow', (
    WidgetTester tester,
  ) async {
    //Regression test for https://github.com/flutter/flutter/issues/112163

    tester.view.physicalSize = const Size(540, 340);

    await tester.pumpWidget(
      MaterialApp(
        home: Builder(
          builder: (BuildContext context) {
            return Scaffold(
              bottomNavigationBar: BottomNavigationBar(
                type: BottomNavigationBarType.fixed,
                landscapeLayout: BottomNavigationBarLandscapeLayout.linear,
                items: const <BottomNavigationBarItem>[
                  BottomNavigationBarItem(
                    icon: Icon(Icons.home_rounded),
                    label: 'Home Challenges',
                    backgroundColor: Colors.grey,
                    tooltip: '',
                  ),
                  BottomNavigationBarItem(
                    icon: Icon(Icons.date_range_rounded),
                    label: 'Daily Challenges',
                    backgroundColor: Colors.grey,
                    tooltip: '',
                  ),
                  BottomNavigationBarItem(
                    icon: Icon(Icons.wind_power),
                    label: 'Awards Challenges',
                    backgroundColor: Colors.grey,
                    tooltip: '',
                  ),
                  BottomNavigationBarItem(
                    icon: Icon(Icons.bar_chart_rounded),
                    label: 'Statistics Challenges',
                    backgroundColor: Colors.grey,
                    tooltip: '',
                  ),
                ],
              ),
            );
          },
        ),
      ),
    );

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('bottom_navigation_bar.label_overflow.png'),
    );

    addTearDown(tester.view.resetPhysicalSize);
  });

  testWidgets('BottomNavigationBar keys passed through', (WidgetTester tester) async {
    const Key key1 = Key('key1');
    const Key key2 = Key('key2');

    await tester.pumpWidget(
      boilerplate(
        textDirection: TextDirection.ltr,
        bottomNavigationBar: BottomNavigationBar(
          items: const <BottomNavigationBarItem>[
            BottomNavigationBarItem(
              key: key1,
              icon: Icon(Icons.favorite_border),
              label: 'Favorite',
            ),
            BottomNavigationBarItem(key: key2, icon: Icon(Icons.access_alarm), label: 'Alarm'),
          ],
        ),
      ),
    );

    expect(find.byKey(key1), findsOneWidget);
    expect(find.byKey(key2), findsOneWidget);
  });
}

Widget boilerplate({
  Widget? bottomNavigationBar,
  required TextDirection textDirection,
  bool? useMaterial3,
}) {
  return MaterialApp(
    theme: ThemeData(useMaterial3: useMaterial3),
    home: Localizations(
      locale: const Locale('en', 'US'),
      delegates: const <LocalizationsDelegate<dynamic>>[
        DefaultMaterialLocalizations.delegate,
        DefaultWidgetsLocalizations.delegate,
      ],
      child: Directionality(
        textDirection: textDirection,
        child: MediaQuery(
          data: const MediaQueryData(),
          child: Material(child: Scaffold(bottomNavigationBar: bottomNavigationBar)),
        ),
      ),
    ),
  );
}

double _getOpacity(WidgetTester tester, String textValue) {
  final FadeTransition opacityWidget = tester.widget<FadeTransition>(
    find.ancestor(of: find.text(textValue), matching: find.byType(FadeTransition)).first,
  );
  return opacityWidget.opacity.value;
}

Material _getMaterial(WidgetTester tester) {
  return tester.firstWidget<Material>(
    find.descendant(of: find.byType(BottomNavigationBar), matching: find.byType(Material)),
  );
}

TextStyle _iconStyle(WidgetTester tester, IconData icon) {
  final RichText iconRichText = tester.widget<RichText>(
    find.descendant(of: find.byIcon(icon), matching: find.byType(RichText)),
  );
  return iconRichText.text.style!;
}

EdgeInsets _itemPadding(WidgetTester tester, IconData icon) {
  return tester
      .widget<Padding>(
        find
            .descendant(
              of: find.ancestor(of: find.byIcon(icon), matching: find.byType(InkResponse)),
              matching: find.byType(Padding),
            )
            .first,
      )
      .padding
      .resolve(TextDirection.ltr);
}
