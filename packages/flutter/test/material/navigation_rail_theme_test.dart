// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('copyWith, ==, hashCode basics', () {
    expect(const NavigationRailThemeData(), const NavigationRailThemeData().copyWith());
    expect(
      const NavigationRailThemeData().hashCode,
      const NavigationRailThemeData().copyWith().hashCode,
    );
  });

  testWidgets(
    'Material3 - Default values are used when no NavigationRail or NavigationRailThemeData properties are specified',
    (WidgetTester tester) async {
      final ThemeData theme = ThemeData.light(useMaterial3: true);
      // Material 3 defaults
      await tester.pumpWidget(
        MaterialApp(
          theme: theme,
          home: Scaffold(body: NavigationRail(selectedIndex: 0, destinations: _destinations())),
        ),
      );

      expect(_railMaterial(tester).color, theme.colorScheme.surface);
      expect(_railMaterial(tester).elevation, 0);
      expect(_destinationSize(tester).width, 80.0);
      expect(_selectedIconTheme(tester).size, 24.0);
      expect(_selectedIconTheme(tester).color, theme.colorScheme.onSecondaryContainer);
      expect(_selectedIconTheme(tester).opacity, null);
      expect(_unselectedIconTheme(tester).size, 24.0);
      expect(_unselectedIconTheme(tester).color, theme.colorScheme.onSurfaceVariant);
      expect(_unselectedIconTheme(tester).opacity, null);
      expect(_selectedLabelStyle(tester).fontSize, 14.0);
      expect(_unselectedLabelStyle(tester).fontSize, 14.0);
      expect(_destinationsAlign(tester).alignment, Alignment.topCenter);
      expect(_labelType(tester), NavigationRailLabelType.none);
      expect(find.byType(NavigationIndicator), findsWidgets);
      expect(_indicatorDecoration(tester)?.color, theme.colorScheme.secondaryContainer);
      expect(_indicatorDecoration(tester)?.shape, const StadiumBorder());
      final InkResponse inkResponse =
          tester.allWidgets.firstWhere(
                (Widget object) => object.runtimeType.toString() == '_IndicatorInkWell',
              )
              as InkResponse;
      expect(inkResponse.customBorder, const StadiumBorder());
    },
  );

  testWidgets(
    'Material2 - Default values are used when no NavigationRail or NavigationRailThemeData properties are specified',
    (WidgetTester tester) async {
      // This test can be removed when `useMaterial3` is deprecated.
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData.light().copyWith(useMaterial3: false),
          home: Scaffold(body: NavigationRail(selectedIndex: 0, destinations: _destinations())),
        ),
      );

      expect(_railMaterial(tester).color, ThemeData().colorScheme.surface);
      expect(_railMaterial(tester).elevation, 0);
      expect(_destinationSize(tester).width, 72.0);
      expect(_selectedIconTheme(tester).size, 24.0);
      expect(_selectedIconTheme(tester).color, ThemeData().colorScheme.primary);
      expect(_selectedIconTheme(tester).opacity, 1.0);
      expect(_unselectedIconTheme(tester).size, 24.0);
      expect(_unselectedIconTheme(tester).color, ThemeData().colorScheme.onSurface);
      expect(_unselectedIconTheme(tester).opacity, 0.64);
      expect(_selectedLabelStyle(tester).fontSize, 14.0);
      expect(_unselectedLabelStyle(tester).fontSize, 14.0);
      expect(_destinationsAlign(tester).alignment, Alignment.topCenter);
      expect(_labelType(tester), NavigationRailLabelType.none);
      expect(find.byType(NavigationIndicator), findsNothing);
    },
  );

  testWidgets(
    'NavigationRailThemeData values are used when no NavigationRail properties are specified',
    (WidgetTester tester) async {
      const Color backgroundColor = Color(0x00000001);
      const double elevation = 7.0;
      const double selectedIconSize = 25.0;
      const double unselectedIconSize = 23.0;
      const Color selectedIconColor = Color(0x00000002);
      const Color unselectedIconColor = Color(0x00000003);
      const double selectedIconOpacity = 0.99;
      const double unselectedIconOpacity = 0.98;
      const double selectedLabelFontSize = 13.0;
      const double unselectedLabelFontSize = 11.0;
      const double groupAlignment = 0.0;
      const NavigationRailLabelType labelType = NavigationRailLabelType.all;
      const bool useIndicator = true;
      const Color indicatorColor = Color(0x00000004);
      const ShapeBorder indicatorShape = RoundedRectangleBorder();

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: NavigationRailTheme(
              data: const NavigationRailThemeData(
                backgroundColor: backgroundColor,
                elevation: elevation,
                selectedIconTheme: IconThemeData(
                  size: selectedIconSize,
                  color: selectedIconColor,
                  opacity: selectedIconOpacity,
                ),
                unselectedIconTheme: IconThemeData(
                  size: unselectedIconSize,
                  color: unselectedIconColor,
                  opacity: unselectedIconOpacity,
                ),
                selectedLabelTextStyle: TextStyle(fontSize: selectedLabelFontSize),
                unselectedLabelTextStyle: TextStyle(fontSize: unselectedLabelFontSize),
                groupAlignment: groupAlignment,
                labelType: labelType,
                useIndicator: useIndicator,
                indicatorColor: indicatorColor,
                indicatorShape: indicatorShape,
              ),
              child: NavigationRail(selectedIndex: 0, destinations: _destinations()),
            ),
          ),
        ),
      );

      expect(_railMaterial(tester).color, backgroundColor);
      expect(_railMaterial(tester).elevation, elevation);
      expect(_selectedIconTheme(tester).size, selectedIconSize);
      expect(_selectedIconTheme(tester).color, selectedIconColor);
      expect(_selectedIconTheme(tester).opacity, selectedIconOpacity);
      expect(_unselectedIconTheme(tester).size, unselectedIconSize);
      expect(_unselectedIconTheme(tester).color, unselectedIconColor);
      expect(_unselectedIconTheme(tester).opacity, unselectedIconOpacity);
      expect(_selectedLabelStyle(tester).fontSize, selectedLabelFontSize);
      expect(_unselectedLabelStyle(tester).fontSize, unselectedLabelFontSize);
      expect(_destinationsAlign(tester).alignment, Alignment.center);
      expect(_labelType(tester), labelType);
      expect(find.byType(NavigationIndicator), findsWidgets);
      expect(_indicatorDecoration(tester)?.color, indicatorColor);
      expect(_indicatorDecoration(tester)?.shape, indicatorShape);
    },
  );

  testWidgets(
    'NavigationRail values take priority over NavigationRailThemeData values when both properties are specified',
    (WidgetTester tester) async {
      const Color backgroundColor = Color(0x00000001);
      const double elevation = 7.0;
      const double selectedIconSize = 25.0;
      const double unselectedIconSize = 23.0;
      const Color selectedIconColor = Color(0x00000002);
      const Color unselectedIconColor = Color(0x00000003);
      const double selectedIconOpacity = 0.99;
      const double unselectedIconOpacity = 0.98;
      const double selectedLabelFontSize = 13.0;
      const double unselectedLabelFontSize = 11.0;
      const double groupAlignment = 0.0;
      const NavigationRailLabelType labelType = NavigationRailLabelType.all;
      const bool useIndicator = true;
      const Color indicatorColor = Color(0x00000004);

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: NavigationRailTheme(
              data: const NavigationRailThemeData(
                backgroundColor: Color(0x00000099),
                elevation: 5,
                selectedIconTheme: IconThemeData(
                  size: 31.0,
                  color: Color(0x00000098),
                  opacity: 0.81,
                ),
                unselectedIconTheme: IconThemeData(
                  size: 37.0,
                  color: Color(0x00000097),
                  opacity: 0.82,
                ),
                selectedLabelTextStyle: TextStyle(fontSize: 9.0),
                unselectedLabelTextStyle: TextStyle(fontSize: 7.0),
                groupAlignment: 1.0,
                labelType: NavigationRailLabelType.selected,
                useIndicator: false,
                indicatorColor: Color(0x00000096),
              ),
              child: NavigationRail(
                selectedIndex: 0,
                destinations: _destinations(),
                backgroundColor: backgroundColor,
                elevation: elevation,
                selectedIconTheme: const IconThemeData(
                  size: selectedIconSize,
                  color: selectedIconColor,
                  opacity: selectedIconOpacity,
                ),
                unselectedIconTheme: const IconThemeData(
                  size: unselectedIconSize,
                  color: unselectedIconColor,
                  opacity: unselectedIconOpacity,
                ),
                selectedLabelTextStyle: const TextStyle(fontSize: selectedLabelFontSize),
                unselectedLabelTextStyle: const TextStyle(fontSize: unselectedLabelFontSize),
                groupAlignment: groupAlignment,
                labelType: labelType,
                useIndicator: useIndicator,
                indicatorColor: indicatorColor,
              ),
            ),
          ),
        ),
      );

      expect(_railMaterial(tester).color, backgroundColor);
      expect(_railMaterial(tester).elevation, elevation);
      expect(_selectedIconTheme(tester).size, selectedIconSize);
      expect(_selectedIconTheme(tester).color, selectedIconColor);
      expect(_selectedIconTheme(tester).opacity, selectedIconOpacity);
      expect(_unselectedIconTheme(tester).size, unselectedIconSize);
      expect(_unselectedIconTheme(tester).color, unselectedIconColor);
      expect(_unselectedIconTheme(tester).opacity, unselectedIconOpacity);
      expect(_selectedLabelStyle(tester).fontSize, selectedLabelFontSize);
      expect(_unselectedLabelStyle(tester).fontSize, unselectedLabelFontSize);
      expect(_destinationsAlign(tester).alignment, Alignment.center);
      expect(_labelType(tester), labelType);
      expect(find.byType(NavigationIndicator), findsWidgets);
      expect(_indicatorDecoration(tester)?.color, indicatorColor);
    },
  );

  // Regression test for https://github.com/flutter/flutter/issues/118618.
  testWidgets('NavigationRailThemeData lerps correctly with null iconThemes', (
    WidgetTester tester,
  ) async {
    final NavigationRailThemeData lerp =
        NavigationRailThemeData.lerp(
          const NavigationRailThemeData(),
          const NavigationRailThemeData(),
          0.5,
        )!;

    expect(lerp.selectedIconTheme, isNull);
    expect(lerp.unselectedIconTheme, isNull);
  });

  testWidgets('Default debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const NavigationRailThemeData().debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[]);
  });

  testWidgets('Custom debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const NavigationRailThemeData(
      backgroundColor: Color(0x00000099),
      elevation: 5,
      selectedIconTheme: IconThemeData(color: Color(0x00000098)),
      unselectedIconTheme: IconThemeData(color: Color(0x00000097)),
      selectedLabelTextStyle: TextStyle(fontSize: 9.0),
      unselectedLabelTextStyle: TextStyle(fontSize: 7.0),
      groupAlignment: 1.0,
      labelType: NavigationRailLabelType.selected,
      useIndicator: true,
      indicatorColor: Color(0x00000096),
      indicatorShape: CircleBorder(),
    ).debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description[0], 'backgroundColor: ${const Color(0x00000099)}');
    expect(description[1], 'elevation: 5.0');
    expect(description[2], 'unselectedLabelTextStyle: TextStyle(inherit: true, size: 7.0)');
    expect(description[3], 'selectedLabelTextStyle: TextStyle(inherit: true, size: 9.0)');

    // Ignore instance address for IconThemeData.
    expect(description[4].contains('unselectedIconTheme: IconThemeData'), isTrue);
    expect(description[4].contains('(color: ${const Color(0x00000097)})'), isTrue);
    expect(description[5].contains('selectedIconTheme: IconThemeData'), isTrue);
    expect(description[5].contains('(color: ${const Color(0x00000098)})'), isTrue);

    expect(description[6], 'groupAlignment: 1.0');
    expect(description[7], 'labelType: NavigationRailLabelType.selected');
    expect(description[8], 'useIndicator: true');
    expect(description[9], 'indicatorColor: ${const Color(0x00000096)}');
    expect(description[10], 'indicatorShape: CircleBorder(BorderSide(width: 0.0, style: none))');
  });
}

List<NavigationRailDestination> _destinations() {
  return const <NavigationRailDestination>[
    NavigationRailDestination(
      icon: Icon(Icons.favorite_border),
      selectedIcon: Icon(Icons.favorite),
      label: Text('Abc'),
    ),
    NavigationRailDestination(
      icon: Icon(Icons.star_border),
      selectedIcon: Icon(Icons.star),
      label: Text('Def'),
    ),
  ];
}

Material _railMaterial(WidgetTester tester) {
  // The first material is for the rail, and the rest are for the destinations.
  return tester.firstWidget<Material>(
    find.descendant(of: find.byType(NavigationRail), matching: find.byType(Material)),
  );
}

ShapeDecoration? _indicatorDecoration(WidgetTester tester) {
  return tester
          .firstWidget<Container>(
            find.descendant(of: find.byType(NavigationIndicator), matching: find.byType(Container)),
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
  // The first IconTheme is the one added by the navigation rail.
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

Size _destinationSize(WidgetTester tester) {
  return tester.getSize(
    find.ancestor(of: find.byIcon(Icons.favorite), matching: find.byType(Material)).first,
  );
}

Align _destinationsAlign(WidgetTester tester) {
  // The first Expanded widget is the one within the main Column for the rail
  // content.
  return tester.firstWidget<Align>(
    find.descendant(of: find.byType(Expanded), matching: find.byType(Align)),
  );
}

NavigationRailLabelType _labelType(WidgetTester tester) {
  if (_visibilityAboveLabel('Abc').evaluate().isNotEmpty &&
      _visibilityAboveLabel('Def').evaluate().isNotEmpty) {
    return _labelVisibility(tester, 'Abc')
        ? NavigationRailLabelType.selected
        : NavigationRailLabelType.none;
  } else {
    return NavigationRailLabelType.all;
  }
}

Finder _visibilityAboveLabel(String text) {
  return find.ancestor(of: find.text(text), matching: find.byType(Visibility));
}

// Only valid when labelType != all.
bool _labelVisibility(WidgetTester tester, String text) {
  final Visibility visibilityWidget = tester.widget<Visibility>(
    find.ancestor(of: find.text(text), matching: find.byType(Visibility)),
  );
  return visibilityWidget.visible;
}
