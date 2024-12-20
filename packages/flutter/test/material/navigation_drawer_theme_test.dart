// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('NavigationDrawerThemeData copyWith, ==, hashCode, basics', () {
    expect(const NavigationDrawerThemeData(), const NavigationDrawerThemeData().copyWith());
    expect(
      const NavigationDrawerThemeData().hashCode,
      const NavigationDrawerThemeData().copyWith().hashCode,
    );
  });

  test('NavigationDrawerThemeData lerp special cases', () {
    expect(NavigationDrawerThemeData.lerp(null, null, 0), null);
    const NavigationDrawerThemeData data = NavigationDrawerThemeData();
    expect(identical(NavigationDrawerThemeData.lerp(data, data, 0.5), data), true);
  });

  testWidgets('Default debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const NavigationDrawerThemeData().debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[]);
  });

  testWidgets('NavigationDrawerThemeData implements debugFillProperties', (
    WidgetTester tester,
  ) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const NavigationDrawerThemeData(
      tileHeight: 50,
      backgroundColor: Color(0x00000099),
      elevation: 5.0,
      shadowColor: Color(0x00000098),
      surfaceTintColor: Color(0x00000097),
      indicatorColor: Color(0x00000096),
      indicatorShape: RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(2.0))),
      indicatorSize: Size(10, 10),
      labelTextStyle: MaterialStatePropertyAll<TextStyle>(TextStyle(fontSize: 7.0)),
      iconTheme: MaterialStatePropertyAll<IconThemeData>(IconThemeData(color: Color(0x00000095))),
    ).debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(
      description,
      equalsIgnoringHashCodes(<String>[
        'tileHeight: 50.0',
        'backgroundColor: ${const Color(0x00000099)}',
        'elevation: 5.0',
        'shadowColor: ${const Color(0x00000098)}',
        'surfaceTintColor: ${const Color(0x00000097)}',
        'indicatorColor: ${const Color(0x00000096)}',
        'indicatorShape: RoundedRectangleBorder(BorderSide(width: 0.0, style: none), BorderRadius.circular(2.0))',
        'indicatorSize: Size(10.0, 10.0)',
        'labelTextStyle: WidgetStatePropertyAll(TextStyle(inherit: true, size: 7.0))',
        'iconTheme: WidgetStatePropertyAll(IconThemeData#00000(color: ${const Color(0x00000095)}))',
      ]),
    );
  });

  testWidgets(
    'NavigationDrawerThemeData values are used when no NavigationDrawer properties are specified',
    (WidgetTester tester) async {
      final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
      const NavigationDrawerThemeData navigationDrawerTheme = NavigationDrawerThemeData(
        backgroundColor: Color(0x00000001),
        elevation: 7.0,
        shadowColor: Color(0x00000002),
        surfaceTintColor: Color(0x00000003),
        indicatorColor: Color(0x00000004),
        indicatorShape: RoundedRectangleBorder(
          borderRadius: BorderRadius.only(topRight: Radius.circular(16.0)),
        ),
        labelTextStyle: MaterialStatePropertyAll<TextStyle>(TextStyle(fontSize: 7.0)),
        iconTheme: MaterialStatePropertyAll<IconThemeData>(IconThemeData(color: Color(0x00000005))),
      );

      await tester.pumpWidget(
        _buildWidget(
          scaffoldKey,
          NavigationDrawer(
            children: const <Widget>[
              Text('Headline'),
              NavigationDrawerDestination(icon: Icon(Icons.ac_unit), label: Text('AC')),
              NavigationDrawerDestination(icon: Icon(Icons.access_alarm), label: Text('Alarm')),
            ],
            onDestinationSelected: (int i) {},
          ),
          theme: ThemeData(navigationDrawerTheme: navigationDrawerTheme),
        ),
      );
      scaffoldKey.currentState!.openDrawer();
      await tester.pump(const Duration(seconds: 1));

      // Test drawer Material.
      expect(_getMaterial(tester).color, navigationDrawerTheme.backgroundColor);
      expect(_getMaterial(tester).surfaceTintColor, navigationDrawerTheme.surfaceTintColor);
      expect(_getMaterial(tester).shadowColor, navigationDrawerTheme.shadowColor);
      expect(_getMaterial(tester).elevation, 7);
      // Test indicator decoration.
      expect(_getIndicatorDecoration(tester)?.color, navigationDrawerTheme.indicatorColor);
      expect(_getIndicatorDecoration(tester)?.shape, navigationDrawerTheme.indicatorShape);
      // Test icon.
      expect(
        _iconStyle(tester, Icons.ac_unit)?.color,
        navigationDrawerTheme.iconTheme?.resolve(<MaterialState>{})?.color,
      );
      expect(
        _iconStyle(tester, Icons.access_alarm)?.color,
        navigationDrawerTheme.iconTheme?.resolve(<MaterialState>{})?.color,
      );
      // Test label.
      expect(
        _labelStyle(tester, 'AC'),
        navigationDrawerTheme.labelTextStyle?.resolve(<MaterialState>{}),
      );
      expect(
        _labelStyle(tester, 'Alarm'),
        navigationDrawerTheme.labelTextStyle?.resolve(<MaterialState>{}),
      );
    },
  );

  testWidgets(
    'NavigationDrawer values take priority over NavigationDrawerThemeData values when both properties are specified',
    (WidgetTester tester) async {
      final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
      const NavigationDrawerThemeData navigationDrawerTheme = NavigationDrawerThemeData(
        backgroundColor: Color(0x00000001),
        elevation: 7.0,
        shadowColor: Color(0x00000002),
        surfaceTintColor: Color(0x00000003),
        indicatorColor: Color(0x00000004),
        indicatorShape: RoundedRectangleBorder(
          borderRadius: BorderRadius.only(topRight: Radius.circular(16.0)),
        ),
        labelTextStyle: MaterialStatePropertyAll<TextStyle>(TextStyle(fontSize: 7.0)),
        iconTheme: MaterialStatePropertyAll<IconThemeData>(IconThemeData(color: Color(0x00000005))),
      );
      const Color backgroundColor = Color(0x00000009);
      const double elevation = 14.0;
      const Color shadowColor = Color(0x00000008);
      const Color surfaceTintColor = Color(0x00000007);
      const RoundedRectangleBorder indicatorShape = RoundedRectangleBorder(
        borderRadius: BorderRadius.all(Radius.circular(32.0)),
      );
      const Color indicatorColor = Color(0x00000006);

      await tester.pumpWidget(
        _buildWidget(
          scaffoldKey,
          NavigationDrawer(
            backgroundColor: backgroundColor,
            elevation: elevation,
            shadowColor: shadowColor,
            surfaceTintColor: surfaceTintColor,
            indicatorShape: indicatorShape,
            indicatorColor: indicatorColor,
            children: const <Widget>[
              Text('Headline'),
              NavigationDrawerDestination(icon: Icon(Icons.ac_unit), label: Text('AC')),
              NavigationDrawerDestination(icon: Icon(Icons.access_alarm), label: Text('Alarm')),
            ],
            onDestinationSelected: (int i) {},
          ),
          theme: ThemeData(navigationDrawerTheme: navigationDrawerTheme),
        ),
      );
      scaffoldKey.currentState!.openDrawer();
      await tester.pump(const Duration(seconds: 1));

      // Test drawer Material.
      expect(_getMaterial(tester).color, backgroundColor);
      expect(_getMaterial(tester).surfaceTintColor, surfaceTintColor);
      expect(_getMaterial(tester).shadowColor, shadowColor);
      expect(_getMaterial(tester).elevation, elevation);
      // Test indicator decoration.
      expect(_getIndicatorDecoration(tester)?.color, indicatorColor);
      expect(_getIndicatorDecoration(tester)?.shape, indicatorShape);
    },
  );

  testWidgets('Local NavigationDrawerTheme takes priority over ThemeData.navigationDrawerTheme', (
    WidgetTester tester,
  ) async {
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
    const Color backgroundColor = Color(0x00000009);
    const double elevation = 7.0;
    const Color shadowColor = Color(0x00000008);
    const Color surfaceTintColor = Color(0x00000007);
    const Color iconColor = Color(0x00000006);
    const TextStyle labelStyle = TextStyle(fontSize: 7.0);
    const ShapeBorder indicatorShape = CircleBorder();
    const Color indicatorColor = Color(0x00000005);

    await tester.pumpWidget(
      _buildWidget(
        scaffoldKey,
        NavigationDrawerTheme(
          data: const NavigationDrawerThemeData(
            backgroundColor: backgroundColor,
            elevation: elevation,
            shadowColor: shadowColor,
            surfaceTintColor: surfaceTintColor,
            indicatorShape: indicatorShape,
            indicatorColor: indicatorColor,
            labelTextStyle: MaterialStatePropertyAll<TextStyle>(TextStyle(fontSize: 7.0)),
            iconTheme: MaterialStatePropertyAll<IconThemeData>(IconThemeData(color: iconColor)),
          ),
          child: NavigationDrawer(
            children: const <Widget>[
              Text('Headline'),
              NavigationDrawerDestination(icon: Icon(Icons.ac_unit), label: Text('AC')),
              NavigationDrawerDestination(icon: Icon(Icons.access_alarm), label: Text('Alarm')),
            ],
            onDestinationSelected: (int i) {},
          ),
        ),
        theme: ThemeData(
          navigationDrawerTheme: const NavigationDrawerThemeData(
            backgroundColor: Color(0x00000001),
            elevation: 7.0,
            shadowColor: Color(0x00000002),
            surfaceTintColor: Color(0x00000003),
            indicatorColor: Color(0x00000004),
            indicatorShape: RoundedRectangleBorder(
              borderRadius: BorderRadius.only(topRight: Radius.circular(16.0)),
            ),
            labelTextStyle: MaterialStatePropertyAll<TextStyle>(TextStyle(fontSize: 7.0)),
            iconTheme: MaterialStatePropertyAll<IconThemeData>(
              IconThemeData(color: Color(0x00000005)),
            ),
          ),
        ),
      ),
    );
    scaffoldKey.currentState!.openDrawer();
    await tester.pump(const Duration(seconds: 1));

    // Test drawer Material.
    expect(_getMaterial(tester).color, backgroundColor);
    expect(_getMaterial(tester).surfaceTintColor, surfaceTintColor);
    expect(_getMaterial(tester).shadowColor, shadowColor);
    expect(_getMaterial(tester).elevation, elevation);
    // Test indicator decoration.
    expect(_getIndicatorDecoration(tester)?.color, indicatorColor);
    expect(_getIndicatorDecoration(tester)?.shape, indicatorShape);
    // Test icon.
    expect(_iconStyle(tester, Icons.ac_unit)?.color, iconColor);
    expect(_iconStyle(tester, Icons.access_alarm)?.color, iconColor);
    // Test label.
    expect(_labelStyle(tester, 'AC'), labelStyle);
    expect(_labelStyle(tester, 'Alarm'), labelStyle);
  });
}

Widget _buildWidget(GlobalKey<ScaffoldState> scaffoldKey, Widget child, {ThemeData? theme}) {
  return MaterialApp(
    theme: theme,
    home: Scaffold(key: scaffoldKey, drawer: child, body: Container()),
  );
}

Material _getMaterial(WidgetTester tester) {
  return tester.firstWidget<Material>(
    find.descendant(of: find.byType(NavigationDrawer), matching: find.byType(Material)),
  );
}

ShapeDecoration? _getIndicatorDecoration(WidgetTester tester) {
  return tester
          .firstWidget<Container>(
            find.descendant(of: find.byType(FadeTransition), matching: find.byType(Container)),
          )
          .decoration
      as ShapeDecoration?;
}

TextStyle? _iconStyle(WidgetTester tester, IconData icon) {
  return tester
      .widget<RichText>(find.descendant(of: find.byIcon(icon), matching: find.byType(RichText)))
      .text
      .style;
}

TextStyle? _labelStyle(WidgetTester tester, String label) {
  return tester
      .widget<RichText>(find.descendant(of: find.text(label), matching: find.byType(RichText)))
      .text
      .style;
}
