// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('copyWith, ==, hashCode basics', () {
    expect(const NavigationBarThemeData(), const NavigationBarThemeData().copyWith());
    expect(
      const NavigationBarThemeData().hashCode,
      const NavigationBarThemeData().copyWith().hashCode,
    );
  });

  test('NavigationBarThemeData lerp special cases', () {
    expect(NavigationBarThemeData.lerp(null, null, 0), null);
    const NavigationBarThemeData data = NavigationBarThemeData();
    expect(identical(NavigationBarThemeData.lerp(data, data, 0.5), data), true);
  });

  testWidgets('Default debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const NavigationBarThemeData().debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[]);
  });

  testWidgets('NavigationBarThemeData implements debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const NavigationBarThemeData(
      height: 200.0,
      backgroundColor: Color(0x00000099),
      elevation: 20.0,
      shadowColor: Color(0x00000098),
      surfaceTintColor: Color(0x00000097),
      indicatorColor: Color(0x00000096),
      indicatorShape: CircleBorder(),
      labelTextStyle: MaterialStatePropertyAll<TextStyle>(TextStyle(fontSize: 7.0)),
      iconTheme: MaterialStatePropertyAll<IconThemeData>(IconThemeData(color: Color(0x00000097))),
      labelBehavior: NavigationDestinationLabelBehavior.alwaysHide,
      overlayColor: MaterialStatePropertyAll<Color>(Color(0x00000095)),
      labelPadding: EdgeInsets.all(8),
    ).debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(
      description,
      equalsIgnoringHashCodes(<String>[
        'height: 200.0',
        'backgroundColor: Color(alpha: 0.0000, red: 0.0000, green: 0.0000, blue: 0.6000, colorSpace: ColorSpace.sRGB)',
        'elevation: 20.0',
        'shadowColor: Color(alpha: 0.0000, red: 0.0000, green: 0.0000, blue: 0.5961, colorSpace: ColorSpace.sRGB)',
        'surfaceTintColor: Color(alpha: 0.0000, red: 0.0000, green: 0.0000, blue: 0.5922, colorSpace: ColorSpace.sRGB)',
        'indicatorColor: Color(alpha: 0.0000, red: 0.0000, green: 0.0000, blue: 0.5882, colorSpace: ColorSpace.sRGB)',
        'indicatorShape: CircleBorder(BorderSide(width: 0.0, style: none))',
        'labelTextStyle: WidgetStatePropertyAll(TextStyle(inherit: true, size: 7.0))',
        'iconTheme: WidgetStatePropertyAll(IconThemeData#fd5c3(color: Color(alpha: 0.0000, red: 0.0000, green: 0.0000, blue: 0.5922, colorSpace: ColorSpace.sRGB)))',
        'labelBehavior: NavigationDestinationLabelBehavior.alwaysHide',
        'overlayColor: WidgetStatePropertyAll(Color(alpha: 0.0000, red: 0.0000, green: 0.0000, blue: 0.5843, colorSpace: ColorSpace.sRGB))',
        'labelPadding: EdgeInsets.all(8.0)',
      ]),
    );
  });

  testWidgets(
    'NavigationBarThemeData values are used when no NavigationBar properties are specified',
    (WidgetTester tester) async {
      const double height = 200.0;
      const Color backgroundColor = Color(0x00000001);
      const double elevation = 42.0;
      const Color indicatorColor = Color(0x00000002);
      const ShapeBorder indicatorShape = CircleBorder();
      const double selectedIconSize = 25.0;
      const double unselectedIconSize = 23.0;
      const Color selectedIconColor = Color(0x00000003);
      const Color unselectedIconColor = Color(0x00000004);
      const double selectedIconOpacity = 0.99;
      const double unselectedIconOpacity = 0.98;
      const double selectedLabelFontSize = 13.0;
      const double unselectedLabelFontSize = 11.0;
      const NavigationDestinationLabelBehavior labelBehavior =
          NavigationDestinationLabelBehavior.alwaysShow;
      const EdgeInsetsGeometry labelPadding = EdgeInsets.all(8);

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            bottomNavigationBar: NavigationBarTheme(
              data: NavigationBarThemeData(
                height: height,
                backgroundColor: backgroundColor,
                elevation: elevation,
                indicatorColor: indicatorColor,
                indicatorShape: indicatorShape,
                iconTheme: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
                  if (states.contains(MaterialState.selected)) {
                    return const IconThemeData(
                      size: selectedIconSize,
                      color: selectedIconColor,
                      opacity: selectedIconOpacity,
                    );
                  }
                  return const IconThemeData(
                    size: unselectedIconSize,
                    color: unselectedIconColor,
                    opacity: unselectedIconOpacity,
                  );
                }),
                labelTextStyle: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
                  if (states.contains(MaterialState.selected)) {
                    return const TextStyle(fontSize: selectedLabelFontSize);
                  }
                  return const TextStyle(fontSize: unselectedLabelFontSize);
                }),
                labelBehavior: labelBehavior,
                labelPadding: labelPadding,
              ),
              child: NavigationBar(destinations: _destinations()),
            ),
          ),
        ),
      );

      expect(_barHeight(tester), height);
      expect(_barMaterial(tester).color, backgroundColor);
      expect(_barMaterial(tester).elevation, elevation);
      expect(_indicator(tester)?.color, indicatorColor);
      expect(_indicator(tester)?.shape, indicatorShape);
      expect(_selectedIconTheme(tester).size, selectedIconSize);
      expect(_selectedIconTheme(tester).color, selectedIconColor);
      expect(_selectedIconTheme(tester).opacity, selectedIconOpacity);
      expect(_unselectedIconTheme(tester).size, unselectedIconSize);
      expect(_unselectedIconTheme(tester).color, unselectedIconColor);
      expect(_unselectedIconTheme(tester).opacity, unselectedIconOpacity);
      expect(_selectedLabelStyle(tester).fontSize, selectedLabelFontSize);
      expect(_unselectedLabelStyle(tester).fontSize, unselectedLabelFontSize);
      expect(_labelBehavior(tester), labelBehavior);
      expect(_getLabelPadding(tester, 'Abc'), labelPadding);
      expect(_getLabelPadding(tester, 'Def'), labelPadding);
    },
  );

  testWidgets(
    'NavigationBar values take priority over NavigationBarThemeData values when both properties are specified',
    (WidgetTester tester) async {
      const double height = 200.0;
      const Color backgroundColor = Color(0x00000001);
      const double elevation = 42.0;
      const NavigationDestinationLabelBehavior labelBehavior =
          NavigationDestinationLabelBehavior.alwaysShow;
      const EdgeInsetsGeometry labelPadding = EdgeInsets.symmetric(horizontal: 16.0);

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            bottomNavigationBar: NavigationBarTheme(
              data: const NavigationBarThemeData(
                height: 100.0,
                elevation: 18.0,
                backgroundColor: Color(0x00000099),
                labelBehavior: NavigationDestinationLabelBehavior.alwaysHide,
                labelPadding: EdgeInsets.all(8),
              ),
              child: NavigationBar(
                height: height,
                elevation: elevation,
                backgroundColor: backgroundColor,
                labelBehavior: labelBehavior,
                labelPadding: labelPadding,
                destinations: _destinations(),
              ),
            ),
          ),
        ),
      );

      expect(_barHeight(tester), height);
      expect(_barMaterial(tester).color, backgroundColor);
      expect(_barMaterial(tester).elevation, elevation);
      expect(_labelBehavior(tester), labelBehavior);
      expect(_getLabelPadding(tester, 'Abc'), labelPadding);
      expect(_getLabelPadding(tester, 'Def'), labelPadding);
    },
  );

  testWidgets('Custom label style renders ink ripple properly', (WidgetTester tester) async {
    Widget buildWidget({NavigationDestinationLabelBehavior? labelBehavior}) {
      return MaterialApp(
        theme: ThemeData(
          navigationBarTheme: const NavigationBarThemeData(
            labelTextStyle: MaterialStatePropertyAll<TextStyle>(
              TextStyle(fontSize: 25, color: Color(0xff0000ff)),
            ),
          ),
          useMaterial3: true,
        ),
        home: Scaffold(
          bottomNavigationBar: Center(
            child: NavigationBar(
              labelBehavior: labelBehavior,
              destinations: const <Widget>[
                NavigationDestination(icon: SizedBox(), label: 'AC'),
                NavigationDestination(icon: SizedBox(), label: 'Alarm'),
              ],
              onDestinationSelected: (int i) {},
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildWidget());

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byType(NavigationDestination).last));
    await tester.pumpAndSettle();

    await expectLater(
      find.byType(NavigationBar),
      matchesGoldenFile('indicator_custom_label_style.png'),
    );
  });

  testWidgets(
    'NavigationBar respects NavigationBarTheme.overlayColor in active/pressed/hovered states',
    (WidgetTester tester) async {
      tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
      const Color hoverColor = Color(0xff0000ff);
      const Color focusColor = Color(0xff00ffff);
      const Color pressedColor = Color(0xffff00ff);
      final MaterialStateProperty<Color?> overlayColor = MaterialStateProperty.resolveWith<Color>((
        Set<MaterialState> states,
      ) {
        if (states.contains(MaterialState.hovered)) {
          return hoverColor;
        }
        if (states.contains(MaterialState.focused)) {
          return focusColor;
        }
        if (states.contains(MaterialState.pressed)) {
          return pressedColor;
        }
        return Colors.transparent;
      });

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(navigationBarTheme: NavigationBarThemeData(overlayColor: overlayColor)),
          home: Scaffold(
            bottomNavigationBar: RepaintBoundary(
              child: NavigationBar(
                destinations: const <Widget>[
                  NavigationDestination(icon: Icon(Icons.ac_unit), label: 'AC'),
                  NavigationDestination(icon: Icon(Icons.access_alarm), label: 'Alarm'),
                ],
                onDestinationSelected: (int i) {},
              ),
            ),
          ),
        ),
      );

      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture.addPointer();
      await gesture.moveTo(tester.getCenter(find.byType(NavigationIndicator).last));
      await tester.pumpAndSettle();

      final RenderObject inkFeatures = tester.allRenderObjects.firstWhere(
        (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
      );

      // Test hovered state.
      expect(
        inkFeatures,
        kIsWeb
            ? (paints
              ..rrect()
              ..rrect()
              ..circle(color: hoverColor))
            : (paints..circle(color: hoverColor)),
      );

      await gesture.down(tester.getCenter(find.byType(NavigationIndicator).last));
      await tester.pumpAndSettle();

      // Test pressed state.
      expect(
        inkFeatures,
        kIsWeb
            ? (paints
              ..circle()
              ..circle()
              ..circle(color: pressedColor))
            : (paints
              ..circle()
              ..circle(color: pressedColor)),
      );

      await gesture.up();
      await tester.pumpAndSettle();

      // Press tab to focus the navigation bar.
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pumpAndSettle();

      // Test focused state.
      expect(
        inkFeatures,
        kIsWeb
            ? (paints
              ..circle()
              ..circle(color: focusColor))
            : (paints
              ..circle()
              ..circle(color: focusColor)),
      );
    },
  );
}

List<NavigationDestination> _destinations() {
  return const <NavigationDestination>[
    NavigationDestination(
      icon: Icon(Icons.favorite_border),
      selectedIcon: Icon(Icons.favorite),
      label: 'Abc',
    ),
    NavigationDestination(
      icon: Icon(Icons.star_border),
      selectedIcon: Icon(Icons.star),
      label: 'Def',
    ),
  ];
}

double _barHeight(WidgetTester tester) {
  return tester.getRect(find.byType(NavigationBar)).height;
}

Material _barMaterial(WidgetTester tester) {
  return tester.firstWidget<Material>(
    find.descendant(of: find.byType(NavigationBar), matching: find.byType(Material)),
  );
}

ShapeDecoration? _indicator(WidgetTester tester) {
  return tester
          .firstWidget<Container>(
            find.descendant(of: find.byType(FadeTransition), matching: find.byType(Container)),
          )
          .decoration
      as ShapeDecoration?;
}

IconThemeData _selectedIconTheme(WidgetTester tester) {
  return _iconTheme(tester, Icons.favorite);
}

IconThemeData _unselectedIconTheme(WidgetTester tester) {
  return _iconTheme(tester, Icons.star_border);
}

IconThemeData _iconTheme(WidgetTester tester, IconData icon) {
  return tester
      .firstWidget<IconTheme>(
        find.ancestor(of: find.byIcon(icon), matching: find.byType(IconTheme)),
      )
      .data;
}

TextStyle _selectedLabelStyle(WidgetTester tester) {
  return tester
      .widget<RichText>(find.descendant(of: find.text('Abc'), matching: find.byType(RichText)))
      .text
      .style!;
}

TextStyle _unselectedLabelStyle(WidgetTester tester) {
  return tester
      .widget<RichText>(find.descendant(of: find.text('Def'), matching: find.byType(RichText)))
      .text
      .style!;
}

NavigationDestinationLabelBehavior _labelBehavior(WidgetTester tester) {
  if (_opacityAboveLabel('Abc').evaluate().isNotEmpty &&
      _opacityAboveLabel('Def').evaluate().isNotEmpty) {
    return _labelOpacity(tester, 'Abc') == 1
        ? NavigationDestinationLabelBehavior.onlyShowSelected
        : NavigationDestinationLabelBehavior.alwaysHide;
  } else {
    return NavigationDestinationLabelBehavior.alwaysShow;
  }
}

Finder _opacityAboveLabel(String text) {
  return find.ancestor(of: find.text(text), matching: find.byType(Opacity));
}

// Only valid when labelBehavior != alwaysShow.
double _labelOpacity(WidgetTester tester, String text) {
  final Opacity opacityWidget = tester.widget<Opacity>(
    find.ancestor(of: find.text(text), matching: find.byType(Opacity)),
  );
  return opacityWidget.opacity;
}

EdgeInsetsGeometry _getLabelPadding(WidgetTester tester, String text) {
  return tester
      .widget<Padding>(find.ancestor(of: find.text(text), matching: find.byType(Padding)).first)
      .padding;
}
