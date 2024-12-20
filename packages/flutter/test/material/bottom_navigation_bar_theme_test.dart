// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:vector_math/vector_math_64.dart' show Vector3;

void main() {
  test('BottomNavigationBarThemeData copyWith, ==, hashCode basics', () {
    expect(const BottomNavigationBarThemeData(), const BottomNavigationBarThemeData().copyWith());
    expect(
      const BottomNavigationBarThemeData().hashCode,
      const BottomNavigationBarThemeData().copyWith().hashCode,
    );
  });

  test('BottomNavigationBarThemeData lerp special cases', () {
    const BottomNavigationBarThemeData data = BottomNavigationBarThemeData();
    expect(identical(BottomNavigationBarThemeData.lerp(data, data, 0.5), data), true);
  });

  test('BottomNavigationBarThemeData defaults', () {
    const BottomNavigationBarThemeData themeData = BottomNavigationBarThemeData();
    expect(themeData.backgroundColor, null);
    expect(themeData.elevation, null);
    expect(themeData.selectedIconTheme, null);
    expect(themeData.unselectedIconTheme, null);
    expect(themeData.selectedItemColor, null);
    expect(themeData.unselectedItemColor, null);
    expect(themeData.selectedLabelStyle, null);
    expect(themeData.unselectedLabelStyle, null);
    expect(themeData.showSelectedLabels, null);
    expect(themeData.showUnselectedLabels, null);
    expect(themeData.type, null);
    expect(themeData.landscapeLayout, null);
    expect(themeData.mouseCursor, null);

    const BottomNavigationBarTheme theme = BottomNavigationBarTheme(
      data: BottomNavigationBarThemeData(),
      child: SizedBox(),
    );
    expect(theme.data.backgroundColor, null);
    expect(theme.data.elevation, null);
    expect(theme.data.selectedIconTheme, null);
    expect(theme.data.unselectedIconTheme, null);
    expect(theme.data.selectedItemColor, null);
    expect(theme.data.unselectedItemColor, null);
    expect(theme.data.selectedLabelStyle, null);
    expect(theme.data.unselectedLabelStyle, null);
    expect(theme.data.showSelectedLabels, null);
    expect(theme.data.showUnselectedLabels, null);
    expect(theme.data.type, null);
    expect(themeData.landscapeLayout, null);
    expect(themeData.mouseCursor, null);
  });

  testWidgets('Default BottomNavigationBarThemeData debugFillProperties', (
    WidgetTester tester,
  ) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const BottomNavigationBarThemeData().debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[]);
  });

  testWidgets('BottomNavigationBarThemeData implements debugFillProperties', (
    WidgetTester tester,
  ) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const BottomNavigationBarThemeData(
      backgroundColor: Color(0xfffffff0),
      elevation: 10.0,
      selectedIconTheme: IconThemeData(size: 1.0),
      unselectedIconTheme: IconThemeData(size: 2.0),
      selectedItemColor: Color(0xfffffff1),
      unselectedItemColor: Color(0xfffffff2),
      selectedLabelStyle: TextStyle(fontSize: 3.0),
      unselectedLabelStyle: TextStyle(fontSize: 4.0),
      showSelectedLabels: true,
      showUnselectedLabels: true,
      type: BottomNavigationBarType.fixed,
      mouseCursor: MaterialStateMouseCursor.clickable,
    ).debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description[0], 'backgroundColor: ${const Color(0xfffffff0)}');
    expect(description[1], 'elevation: 10.0');

    // Ignore instance address for IconThemeData.
    expect(description[2].contains('selectedIconTheme: IconThemeData'), isTrue);
    expect(description[2].contains('(size: 1.0)'), isTrue);
    expect(description[3].contains('unselectedIconTheme: IconThemeData'), isTrue);
    expect(description[3].contains('(size: 2.0)'), isTrue);

    expect(description[4], 'selectedItemColor: ${const Color(0xfffffff1)}');
    expect(description[5], 'unselectedItemColor: ${const Color(0xfffffff2)}');
    expect(description[6], 'selectedLabelStyle: TextStyle(inherit: true, size: 3.0)');
    expect(description[7], 'unselectedLabelStyle: TextStyle(inherit: true, size: 4.0)');
    expect(description[8], 'showSelectedLabels: true');
    expect(description[9], 'showUnselectedLabels: true');
    expect(description[10], 'type: BottomNavigationBarType.fixed');
    expect(description[11], 'mouseCursor: WidgetStateMouseCursor(clickable)');
  });

  testWidgets('BottomNavigationBar is themeable', (WidgetTester tester) async {
    const Color backgroundColor = Color(0xFF000001);
    const Color selectedItemColor = Color(0xFF000002);
    const Color unselectedItemColor = Color(0xFF000003);
    const IconThemeData selectedIconTheme = IconThemeData(size: 10);
    const IconThemeData unselectedIconTheme = IconThemeData(size: 11);
    const TextStyle selectedTextStyle = TextStyle(fontSize: 22);
    const TextStyle unselectedTextStyle = TextStyle(fontSize: 21);
    const double elevation = 9.0;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          bottomNavigationBarTheme: BottomNavigationBarThemeData(
            backgroundColor: backgroundColor,
            selectedItemColor: selectedItemColor,
            unselectedItemColor: unselectedItemColor,
            selectedIconTheme: selectedIconTheme,
            unselectedIconTheme: unselectedIconTheme,
            elevation: elevation,
            showUnselectedLabels: true,
            showSelectedLabels: true,
            type: BottomNavigationBarType.fixed,
            selectedLabelStyle: selectedTextStyle,
            unselectedLabelStyle: unselectedTextStyle,
            mouseCursor: MaterialStateProperty.resolveWith<MouseCursor?>((
              Set<MaterialState> states,
            ) {
              if (states.contains(MaterialState.selected)) {
                return SystemMouseCursors.grab;
              }
              return SystemMouseCursors.move;
            }),
          ),
        ),
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

    final Finder findACTransform = find.descendant(
      of: find.byType(BottomNavigationBar),
      matching: find.ancestor(of: find.text('AC'), matching: find.byType(Transform)),
    );
    final Finder findAlarmTransform = find.descendant(
      of: find.byType(BottomNavigationBar),
      matching: find.ancestor(of: find.text('Alarm'), matching: find.byType(Transform)),
    );
    final TextStyle selectedFontStyle =
        tester.renderObject<RenderParagraph>(find.text('AC')).text.style!;
    final TextStyle selectedIcon = _iconStyle(tester, Icons.ac_unit);
    final TextStyle unselectedIcon = _iconStyle(tester, Icons.access_alarm);
    expect(selectedFontStyle.fontSize, selectedFontStyle.fontSize);
    // Unselected label has a font size of 22 but is scaled down to be font size 21.
    expect(
      tester.firstWidget<Transform>(findAlarmTransform).transform,
      equals(
        Matrix4.diagonal3(Vector3.all(unselectedTextStyle.fontSize! / selectedTextStyle.fontSize!)),
      ),
    );
    expect(selectedIcon.color, equals(selectedItemColor));
    expect(selectedIcon.fontSize, equals(selectedIconTheme.size));
    expect(unselectedIcon.color, equals(unselectedItemColor));
    expect(unselectedIcon.fontSize, equals(unselectedIconTheme.size));
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
    expect(_material(tester).elevation, equals(elevation));
    expect(_material(tester).color, equals(backgroundColor));

    final Offset selectedBarItem = tester.getCenter(findACTransform);
    final Offset unselectedBarItem = tester.getCenter(findAlarmTransform);
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(selectedBarItem);
    await tester.pumpAndSettle();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.grab,
    );
    await gesture.moveTo(unselectedBarItem);
    await tester.pumpAndSettle();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.move,
    );
  });

  testWidgets('BottomNavigationBar properties are taken over the theme values', (
    WidgetTester tester,
  ) async {
    const Color themeBackgroundColor = Color(0xFF000001);
    const Color themeSelectedItemColor = Color(0xFF000002);
    const Color themeUnselectedItemColor = Color(0xFF000003);
    const IconThemeData themeSelectedIconTheme = IconThemeData(size: 10);
    const IconThemeData themeUnselectedIconTheme = IconThemeData(size: 11);
    const TextStyle themeSelectedTextStyle = TextStyle(fontSize: 22);
    const TextStyle themeUnselectedTextStyle = TextStyle(fontSize: 21);
    const double themeElevation = 9.0;
    const BottomNavigationBarLandscapeLayout themeLandscapeLayout =
        BottomNavigationBarLandscapeLayout.centered;
    const MaterialStateMouseCursor themeCursor = MaterialStateMouseCursor.clickable;

    const Color backgroundColor = Color(0xFF000004);
    const Color selectedItemColor = Color(0xFF000005);
    const Color unselectedItemColor = Color(0xFF000006);
    const IconThemeData selectedIconTheme = IconThemeData(size: 15);
    const IconThemeData unselectedIconTheme = IconThemeData(size: 16);
    const TextStyle selectedTextStyle = TextStyle(fontSize: 25);
    const TextStyle unselectedTextStyle = TextStyle(fontSize: 26);
    const double elevation = 7.0;
    const BottomNavigationBarLandscapeLayout landscapeLayout =
        BottomNavigationBarLandscapeLayout.spread;
    const MaterialStateMouseCursor cursor = MaterialStateMouseCursor.textable;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          bottomNavigationBarTheme: const BottomNavigationBarThemeData(
            backgroundColor: themeBackgroundColor,
            selectedItemColor: themeSelectedItemColor,
            unselectedItemColor: themeUnselectedItemColor,
            selectedIconTheme: themeSelectedIconTheme,
            unselectedIconTheme: themeUnselectedIconTheme,
            elevation: themeElevation,
            showUnselectedLabels: false,
            showSelectedLabels: false,
            type: BottomNavigationBarType.shifting,
            selectedLabelStyle: themeSelectedTextStyle,
            unselectedLabelStyle: themeUnselectedTextStyle,
            landscapeLayout: themeLandscapeLayout,
            mouseCursor: themeCursor,
          ),
        ),
        home: Scaffold(
          bottomNavigationBar: BottomNavigationBar(
            backgroundColor: backgroundColor,
            selectedItemColor: selectedItemColor,
            unselectedItemColor: unselectedItemColor,
            selectedIconTheme: selectedIconTheme,
            unselectedIconTheme: unselectedIconTheme,
            elevation: elevation,
            showUnselectedLabels: true,
            showSelectedLabels: true,
            type: BottomNavigationBarType.fixed,
            selectedLabelStyle: selectedTextStyle,
            unselectedLabelStyle: unselectedTextStyle,
            landscapeLayout: landscapeLayout,
            mouseCursor: cursor,
            items: const <BottomNavigationBarItem>[
              BottomNavigationBarItem(icon: Icon(Icons.ac_unit), label: 'AC'),
              BottomNavigationBarItem(icon: Icon(Icons.access_alarm), label: 'Alarm'),
            ],
          ),
        ),
      ),
    );

    Finder findDescendantOfBottomNavigationBar(Finder finder) {
      return find.descendant(of: find.byType(BottomNavigationBar), matching: finder);
    }

    final TextStyle selectedFontStyle =
        tester.renderObject<RenderParagraph>(find.text('AC')).text.style!;
    final TextStyle selectedIcon = _iconStyle(tester, Icons.ac_unit);
    final TextStyle unselectedIcon = _iconStyle(tester, Icons.access_alarm);
    expect(selectedFontStyle.fontSize, selectedFontStyle.fontSize);
    // Unselected label has a font size of 22 but is scaled down to be font size 21.
    expect(
      tester
          .firstWidget<Transform>(
            findDescendantOfBottomNavigationBar(
              find.ancestor(of: find.text('Alarm'), matching: find.byType(Transform)),
            ),
          )
          .transform,
      equals(
        Matrix4.diagonal3(Vector3.all(unselectedTextStyle.fontSize! / selectedTextStyle.fontSize!)),
      ),
    );
    expect(selectedIcon.color, equals(selectedItemColor));
    expect(selectedIcon.fontSize, equals(selectedIconTheme.size));
    expect(unselectedIcon.color, equals(unselectedItemColor));
    expect(unselectedIcon.fontSize, equals(unselectedIconTheme.size));
    // There should not be any [Opacity] or [FadeTransition] widgets
    // since showUnselectedLabels and showSelectedLabels are true.
    final Finder findOpacity = findDescendantOfBottomNavigationBar(find.byType(Opacity));
    final Finder findFadeTransition = findDescendantOfBottomNavigationBar(
      find.byType(FadeTransition),
    );
    expect(findOpacity, findsNothing);
    expect(findFadeTransition, findsNothing);
    expect(_material(tester).elevation, equals(elevation));
    expect(_material(tester).color, equals(backgroundColor));

    final Offset barItem = tester.getCenter(
      findDescendantOfBottomNavigationBar(
        find.ancestor(of: find.text('AC'), matching: find.byType(Transform)),
      ),
    );
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(barItem);
    await tester.pumpAndSettle();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );
  });

  testWidgets('BottomNavigationBarTheme can be used to hide all labels', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/66738.
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          bottomNavigationBarTheme: const BottomNavigationBarThemeData(
            showSelectedLabels: false,
            showUnselectedLabels: false,
          ),
        ),
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

    final Finder findVisibility = find.descendant(
      of: find.byType(BottomNavigationBar),
      matching: find.byType(Visibility),
    );

    expect(findVisibility, findsNWidgets(2));
    expect(tester.widget<Visibility>(findVisibility.at(0)).visible, false);
    expect(tester.widget<Visibility>(findVisibility.at(1)).visible, false);
  });

  testWidgets('BottomNavigationBarTheme can be used to hide selected labels', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/66738.
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          bottomNavigationBarTheme: const BottomNavigationBarThemeData(
            showSelectedLabels: false,
            showUnselectedLabels: true,
          ),
        ),
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

    final Finder findFadeTransition = find.descendant(
      of: find.byType(BottomNavigationBar),
      matching: find.byType(FadeTransition),
    );

    expect(findFadeTransition, findsNWidgets(2));
    expect(tester.widget<FadeTransition>(findFadeTransition.at(0)).opacity.value, 0.0);
    expect(tester.widget<FadeTransition>(findFadeTransition.at(1)).opacity.value, 1.0);
  });

  testWidgets('BottomNavigationBarTheme can be used to hide unselected labels', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          bottomNavigationBarTheme: const BottomNavigationBarThemeData(
            showSelectedLabels: true,
            showUnselectedLabels: false,
          ),
        ),
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

    final Finder findFadeTransition = find.descendant(
      of: find.byType(BottomNavigationBar),
      matching: find.byType(FadeTransition),
    );

    expect(findFadeTransition, findsNWidgets(2));
    expect(tester.widget<FadeTransition>(findFadeTransition.at(0)).opacity.value, 1.0);
    expect(tester.widget<FadeTransition>(findFadeTransition.at(1)).opacity.value, 0.0);
  });
}

TextStyle _iconStyle(WidgetTester tester, IconData icon) {
  final RichText iconRichText = tester.widget<RichText>(
    find.descendant(of: find.byIcon(icon), matching: find.byType(RichText)),
  );
  return iconRichText.text.style!;
}

Material _material(WidgetTester tester) {
  return tester.firstWidget<Material>(
    find.descendant(of: find.byType(BottomNavigationBar), matching: find.byType(Material)),
  );
}
