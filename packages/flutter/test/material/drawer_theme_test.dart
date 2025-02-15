// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('copyWith, ==, hashCode basics', () {
    expect(const DrawerThemeData(), const DrawerThemeData().copyWith());
    expect(const DrawerThemeData().hashCode, const DrawerThemeData().copyWith().hashCode);
  });

  test('DrawerThemeData lerp special cases', () {
    expect(DrawerThemeData.lerp(null, null, 0), null);
    const DrawerThemeData data = DrawerThemeData();
    expect(identical(DrawerThemeData.lerp(data, data, 0.5), data), true);
  });

  testWidgets('Default debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const DrawerThemeData().debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[]);
  });

  testWidgets('Custom debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const DrawerThemeData(
      backgroundColor: Color(0x00000099),
      scrimColor: Color(0x00000098),
      elevation: 5.0,
      shadowColor: Color(0x00000097),
      surfaceTintColor: Color(0x00000096),
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(2.0))),
      width: 200.0,
      clipBehavior: Clip.hardEdge,
    ).debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[
      'backgroundColor: ${const Color(0x00000099)}',
      'scrimColor: ${const Color(0x00000098)}',
      'elevation: 5.0',
      'shadowColor: ${const Color(0x00000097)}',
      'surfaceTintColor: ${const Color(0x00000096)}',
      'shape: RoundedRectangleBorder(BorderSide(width: 0.0, style: none), BorderRadius.circular(2.0))',
      'width: 200.0',
      'clipBehavior: Clip.hardEdge',
    ]);
  });

  testWidgets(
    'Material2 - Default values are used when no Drawer or DrawerThemeData properties are specified',
    (WidgetTester tester) async {
      final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
      final ThemeData theme = ThemeData(useMaterial3: false);
      await tester.pumpWidget(
        MaterialApp(theme: theme, home: Scaffold(key: scaffoldKey, drawer: const Drawer())),
      );
      scaffoldKey.currentState!.openDrawer();
      await tester.pumpAndSettle();

      expect(_drawerMaterial(tester).color, null);
      expect(_drawerMaterial(tester).elevation, 16.0);
      expect(_drawerMaterial(tester).shadowColor, theme.shadowColor);
      expect(_drawerMaterial(tester).surfaceTintColor, null);
      expect(_drawerMaterial(tester).shape, null);
      expect(_scrim(tester).color, Colors.black54);
      expect(_drawerRenderBox(tester).size.width, 304.0);
      expect(_drawerMaterial(tester).clipBehavior, Clip.none);
    },
  );

  testWidgets(
    'Material3 - Default values are used when no Drawer or DrawerThemeData properties are specified',
    (WidgetTester tester) async {
      final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
      final ThemeData theme = ThemeData();
      await tester.pumpWidget(
        MaterialApp(theme: theme, home: Scaffold(key: scaffoldKey, drawer: const Drawer())),
      );
      scaffoldKey.currentState!.openDrawer();
      await tester.pumpAndSettle();

      expect(_drawerMaterial(tester).color, theme.colorScheme.surfaceContainerLow);
      expect(_drawerMaterial(tester).elevation, 1.0);
      expect(_drawerMaterial(tester).shadowColor, Colors.transparent);
      expect(_drawerMaterial(tester).surfaceTintColor, Colors.transparent);
      expect(
        _drawerMaterial(tester).shape,
        const RoundedRectangleBorder(
          borderRadius: BorderRadius.horizontal(right: Radius.circular(16.0)),
        ),
      );
      expect(_scrim(tester).color, Colors.black54);
      expect(_drawerRenderBox(tester).size.width, 304.0);
      expect(_drawerMaterial(tester).clipBehavior, Clip.hardEdge);
    },
  );

  testWidgets(
    'Material2 - Default values are used when no Drawer or DrawerThemeData properties are specified in end drawer',
    (WidgetTester tester) async {
      final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
      final ThemeData theme = ThemeData(useMaterial3: false);
      await tester.pumpWidget(
        MaterialApp(theme: theme, home: Scaffold(key: scaffoldKey, endDrawer: const Drawer())),
      );
      scaffoldKey.currentState!.openEndDrawer();
      await tester.pumpAndSettle();

      expect(_drawerMaterial(tester).color, null);
      expect(_drawerMaterial(tester).elevation, 16.0);
      expect(_drawerMaterial(tester).shadowColor, theme.shadowColor);
      expect(_drawerMaterial(tester).surfaceTintColor, null);
      expect(_drawerMaterial(tester).shape, null);
      expect(_scrim(tester).color, Colors.black54);
      expect(_drawerRenderBox(tester).size.width, 304.0);
      expect(_drawerMaterial(tester).clipBehavior, Clip.none);
    },
  );

  testWidgets(
    'Material3 - Default values are used when no Drawer or DrawerThemeData properties are specified in end drawer',
    (WidgetTester tester) async {
      final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
      final ThemeData theme = ThemeData();
      await tester.pumpWidget(
        MaterialApp(theme: theme, home: Scaffold(key: scaffoldKey, endDrawer: const Drawer())),
      );
      scaffoldKey.currentState!.openEndDrawer();
      await tester.pumpAndSettle();

      expect(_drawerMaterial(tester).color, theme.colorScheme.surfaceContainerLow);
      expect(_drawerMaterial(tester).elevation, 1.0);
      expect(_drawerMaterial(tester).shadowColor, Colors.transparent);
      expect(_drawerMaterial(tester).surfaceTintColor, Colors.transparent);
      expect(
        _drawerMaterial(tester).shape,
        const RoundedRectangleBorder(
          borderRadius: BorderRadius.horizontal(left: Radius.circular(16.0)),
        ),
      );
      expect(_scrim(tester).color, Colors.black54);
      expect(_drawerRenderBox(tester).size.width, 304.0);
      expect(_drawerMaterial(tester).clipBehavior, Clip.hardEdge);
    },
  );

  testWidgets('DrawerThemeData values are used when no Drawer properties are specified', (
    WidgetTester tester,
  ) async {
    const Color backgroundColor = Color(0x00000001);
    const Color scrimColor = Color(0x00000002);
    const double elevation = 7.0;
    const Color shadowColor = Color(0x00000003);
    const Color surfaceTintColor = Color(0x00000004);
    const RoundedRectangleBorder shape = RoundedRectangleBorder(
      borderRadius: BorderRadius.all(Radius.circular(16.0)),
    );
    const double width = 200.0;
    const Clip clipBehavior = Clip.antiAlias;

    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          drawerTheme: const DrawerThemeData(
            backgroundColor: backgroundColor,
            scrimColor: scrimColor,
            elevation: elevation,
            shadowColor: shadowColor,
            surfaceTintColor: surfaceTintColor,
            shape: shape,
            width: width,
            clipBehavior: clipBehavior,
          ),
        ),
        home: Scaffold(key: scaffoldKey, drawer: const Drawer()),
      ),
    );
    scaffoldKey.currentState!.openDrawer();
    await tester.pumpAndSettle();

    expect(_drawerMaterial(tester).color, backgroundColor);
    expect(_drawerMaterial(tester).elevation, elevation);
    expect(_drawerMaterial(tester).shadowColor, shadowColor);
    expect(_drawerMaterial(tester).surfaceTintColor, surfaceTintColor);
    expect(_drawerMaterial(tester).shape, shape);
    expect(_scrim(tester).color, scrimColor);
    expect(_drawerRenderBox(tester).size.width, width);
    expect(_drawerMaterial(tester).clipBehavior, clipBehavior);
  });

  testWidgets(
    'Drawer values take priority over DrawerThemeData values when both properties are specified',
    (WidgetTester tester) async {
      const Color backgroundColor = Color(0x00000001);
      const Color scrimColor = Color(0x00000002);
      const double elevation = 7.0;
      const Color shadowColor = Color(0x00000003);
      const Color surfaceTintColor = Color(0x00000004);
      const RoundedRectangleBorder shape = RoundedRectangleBorder(
        borderRadius: BorderRadius.all(Radius.circular(16.0)),
      );
      const double width = 200.0;
      const Clip clipBehavior = Clip.antiAlias;

      final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            drawerTheme: const DrawerThemeData(
              backgroundColor: Color(0x00000005),
              scrimColor: Color(0x00000006),
              elevation: 13.0,
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(29.0))),
              width: 400.0,
              clipBehavior: Clip.antiAliasWithSaveLayer,
            ),
          ),
          home: Scaffold(
            key: scaffoldKey,
            drawerScrimColor: scrimColor,
            drawer: const Drawer(
              backgroundColor: backgroundColor,
              elevation: elevation,
              shadowColor: shadowColor,
              surfaceTintColor: surfaceTintColor,
              shape: shape,
              width: width,
              clipBehavior: clipBehavior,
            ),
          ),
        ),
      );
      scaffoldKey.currentState!.openDrawer();
      await tester.pumpAndSettle();

      expect(_drawerMaterial(tester).color, backgroundColor);
      expect(_drawerMaterial(tester).elevation, elevation);
      expect(_drawerMaterial(tester).shadowColor, shadowColor);
      expect(_drawerMaterial(tester).surfaceTintColor, surfaceTintColor);
      expect(_drawerMaterial(tester).shape, shape);
      expect(_scrim(tester).color, scrimColor);
      expect(_drawerRenderBox(tester).size.width, width);
      expect(_drawerMaterial(tester).clipBehavior, clipBehavior);
    },
  );

  testWidgets(
    'DrawerTheme values take priority over ThemeData.drawerTheme values when both properties are specified',
    (WidgetTester tester) async {
      const Color backgroundColor = Color(0x00000001);
      const Color scrimColor = Color(0x00000002);
      const double elevation = 7.0;
      const Color shadowColor = Color(0x00000003);
      const Color surfaceTintColor = Color(0x00000004);
      const RoundedRectangleBorder shape = RoundedRectangleBorder(
        borderRadius: BorderRadius.all(Radius.circular(16.0)),
      );
      const double width = 200.0;
      const Clip clipBehavior = Clip.antiAlias;

      final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            drawerTheme: const DrawerThemeData(
              backgroundColor: Color(0x00000005),
              scrimColor: Color(0x00000006),
              elevation: 13.0,
              shadowColor: Color(0x00000007),
              surfaceTintColor: Color(0x00000007),
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(29.0))),
              width: 400.0,
              clipBehavior: Clip.antiAliasWithSaveLayer,
            ),
          ),
          home: DrawerTheme(
            data: const DrawerThemeData(
              backgroundColor: backgroundColor,
              scrimColor: scrimColor,
              elevation: elevation,
              shadowColor: shadowColor,
              surfaceTintColor: surfaceTintColor,
              shape: shape,
              width: width,
              clipBehavior: clipBehavior,
            ),
            child: Scaffold(key: scaffoldKey, drawer: const Drawer()),
          ),
        ),
      );
      scaffoldKey.currentState!.openDrawer();
      await tester.pumpAndSettle();

      expect(_drawerMaterial(tester).color, backgroundColor);
      expect(_drawerMaterial(tester).elevation, elevation);
      expect(_drawerMaterial(tester).shadowColor, shadowColor);
      expect(_drawerMaterial(tester).surfaceTintColor, surfaceTintColor);
      expect(_drawerMaterial(tester).shape, shape);
      expect(_scrim(tester).color, scrimColor);
      expect(_drawerRenderBox(tester).size.width, width);
      expect(_drawerMaterial(tester).clipBehavior, clipBehavior);
    },
  );
}

Material _drawerMaterial(WidgetTester tester) {
  return tester.firstWidget<Material>(
    find.descendant(of: find.byType(Drawer), matching: find.byType(Material)),
  );
}

// The scrim is a Container within a Semantics node labeled "Dismiss",
// within a DrawerController.
ColoredBox _scrim(WidgetTester tester) {
  return tester.widget<ColoredBox>(
    find.descendant(
      of: find.descendant(
        of: find.byType(DrawerController),
        matching: find.byWidgetPredicate((Widget widget) {
          return widget is Semantics && widget.properties.label == 'Dismiss';
        }),
      ),
      matching: find.byType(ColoredBox),
    ),
  );
}

// The RenderBox representing the Drawer.
RenderBox _drawerRenderBox(WidgetTester tester) {
  return tester.renderObject(find.byType(Drawer));
}
