// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('BottomAppBarTheme lerp special cases', () {
    expect(BottomAppBarTheme.lerp(null, null, 0), const BottomAppBarTheme());
    const BottomAppBarTheme data = BottomAppBarTheme();
    expect(identical(BottomAppBarTheme.lerp(data, data, 0.5), data), true);
  });

  group('Material 2 tests', () {
    testWidgets('Material2 - BAB theme overrides color', (WidgetTester tester) async {
      const Color themedColor = Colors.black87;
      const BottomAppBarTheme theme = BottomAppBarTheme(color: themedColor);

      await tester.pumpWidget(_withTheme(theme, useMaterial3: false));

      final PhysicalShape widget = _getBabRenderObject(tester);
      expect(widget.color, themedColor);
    });

    testWidgets('Material2 - BAB color - Widget', (WidgetTester tester) async {
      const Color babThemeColor = Colors.black87;
      const Color babColor = Colors.pink;
      const BottomAppBarTheme theme = BottomAppBarTheme(color: babThemeColor);

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false, bottomAppBarTheme: theme),
          home: const Scaffold(body: BottomAppBar(color: babColor)),
        ),
      );

      final PhysicalShape widget = _getBabRenderObject(tester);
      expect(widget.color, babColor);
    });

    testWidgets('Material2 - BAB color - BabTheme', (WidgetTester tester) async {
      const Color babThemeColor = Colors.black87;
      const BottomAppBarTheme theme = BottomAppBarTheme(color: babThemeColor);

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false, bottomAppBarTheme: theme),
          home: const Scaffold(body: BottomAppBar()),
        ),
      );

      final PhysicalShape widget = _getBabRenderObject(tester);
      expect(widget.color, babThemeColor);
    });

    testWidgets('Material2 - BAB color - Theme', (WidgetTester tester) async {
      const Color themeColor = Colors.white10;

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            useMaterial3: false,
            bottomAppBarTheme: const BottomAppBarTheme(color: themeColor),
          ),
          home: const Scaffold(body: BottomAppBar()),
        ),
      );

      final PhysicalShape widget = _getBabRenderObject(tester);
      expect(widget.color, themeColor);
    });

    testWidgets('Material2 - BAB color - Default', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: const Scaffold(body: BottomAppBar()),
        ),
      );

      final PhysicalShape widget = _getBabRenderObject(tester);

      expect(widget.color, Colors.white);
    });

    testWidgets('Material2 - BAB theme customizes shape', (WidgetTester tester) async {
      const BottomAppBarTheme theme = BottomAppBarTheme(
        color: Colors.white30,
        shape: CircularNotchedRectangle(),
        elevation: 1.0,
      );

      await tester.pumpWidget(_withTheme(theme, useMaterial3: false));

      await expectLater(
        find.byKey(_painterKey),
        matchesGoldenFile('bottom_app_bar_theme.custom_shape.png'),
      );
    });

    testWidgets('Material2 - BAB theme does not affect defaults', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: const Scaffold(body: BottomAppBar()),
        ),
      );

      final PhysicalShape widget = _getBabRenderObject(tester);

      expect(widget.color, Colors.white);
      expect(widget.elevation, equals(8.0));
    });
  });

  group('Material 3 tests', () {
    testWidgets('Material3 - BAB theme overrides color', (WidgetTester tester) async {
      const Color themedColor = Colors.black87;
      const BottomAppBarTheme theme = BottomAppBarTheme(color: themedColor, elevation: 0);
      await tester.pumpWidget(_withTheme(theme, useMaterial3: true));

      final PhysicalShape widget = _getBabRenderObject(tester);
      expect(widget.color, themedColor);
    });

    testWidgets('Material3 - BAB color - Widget', (WidgetTester tester) async {
      const Color babThemeColor = Colors.black87;
      const Color babColor = Colors.pink;
      const BottomAppBarTheme theme = BottomAppBarTheme(color: babThemeColor);

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(bottomAppBarTheme: theme),
          home: const Scaffold(
            body: BottomAppBar(color: babColor, surfaceTintColor: Colors.transparent),
          ),
        ),
      );

      final PhysicalShape widget = _getBabRenderObject(tester);
      expect(widget.color, babColor);
    });

    testWidgets('Material3 - BAB color - BabTheme', (WidgetTester tester) async {
      const Color babThemeColor = Colors.black87;
      const BottomAppBarTheme theme = BottomAppBarTheme(color: babThemeColor);

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(bottomAppBarTheme: theme),
          home: const Scaffold(body: BottomAppBar(surfaceTintColor: Colors.transparent)),
        ),
      );

      final PhysicalShape widget = _getBabRenderObject(tester);
      expect(widget.color, babThemeColor);
    });

    testWidgets('Material3 - BAB theme does not affect defaults', (WidgetTester tester) async {
      final ThemeData theme = ThemeData();
      await tester.pumpWidget(
        MaterialApp(
          theme: theme,
          home: const Scaffold(body: BottomAppBar(surfaceTintColor: Colors.transparent)),
        ),
      );

      final PhysicalShape widget = _getBabRenderObject(tester);

      expect(widget.color, theme.colorScheme.surfaceContainer);
      expect(widget.elevation, equals(3.0));
    });

    testWidgets('Material3 - BAB theme overrides surfaceTintColor', (WidgetTester tester) async {
      const Color color = Colors.blue; // base color that the surface tint will be applied to
      const Color babThemeSurfaceTintColor = Colors.black87;
      const BottomAppBarTheme theme = BottomAppBarTheme(
        color: color,
        surfaceTintColor: babThemeSurfaceTintColor,
        elevation: 0,
      );
      await tester.pumpWidget(_withTheme(theme, useMaterial3: true));

      final PhysicalShape widget = _getBabRenderObject(tester);
      expect(widget.color, ElevationOverlay.applySurfaceTint(color, babThemeSurfaceTintColor, 0));
    });

    testWidgets('Material3 - BAB theme overrides shadowColor', (WidgetTester tester) async {
      const Color babThemeShadowColor = Colors.yellow;
      const BottomAppBarTheme theme = BottomAppBarTheme(
        shadowColor: babThemeShadowColor,
        elevation: 0,
      );
      await tester.pumpWidget(_withTheme(theme, useMaterial3: true));

      final PhysicalShape widget = _getBabRenderObject(tester);
      expect(widget.shadowColor, babThemeShadowColor);
    });

    testWidgets('Material3 - BAB surfaceTintColor - Widget', (WidgetTester tester) async {
      const Color color = Colors.white10; // base color that the surface tint will be applied to
      const Color babThemeSurfaceTintColor = Colors.black87;
      const Color babSurfaceTintColor = Colors.pink;
      const BottomAppBarTheme theme = BottomAppBarTheme(surfaceTintColor: babThemeSurfaceTintColor);
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(bottomAppBarTheme: theme),
          home: const Scaffold(
            body: BottomAppBar(color: color, surfaceTintColor: babSurfaceTintColor),
          ),
        ),
      );

      final PhysicalShape widget = _getBabRenderObject(tester);
      expect(widget.color, ElevationOverlay.applySurfaceTint(color, babSurfaceTintColor, 3.0));
    });

    testWidgets('Material3 - BAB surfaceTintColor - BabTheme', (WidgetTester tester) async {
      const Color color = Colors.blue; // base color that the surface tint will be applied to
      const Color babThemeColor = Colors.black87;
      const BottomAppBarTheme theme = BottomAppBarTheme(surfaceTintColor: babThemeColor);

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(bottomAppBarTheme: theme),
          home: const Scaffold(body: BottomAppBar(color: color)),
        ),
      );

      final PhysicalShape widget = _getBabRenderObject(tester);
      expect(widget.color, ElevationOverlay.applySurfaceTint(color, babThemeColor, 3.0));
    });
  });
}

PhysicalShape _getBabRenderObject(WidgetTester tester) {
  return tester.widget<PhysicalShape>(
    find.descendant(of: find.byType(BottomAppBar), matching: find.byType(PhysicalShape)),
  );
}

final Key _painterKey = UniqueKey();

Widget _withTheme(BottomAppBarTheme theme, {required bool useMaterial3}) {
  return MaterialApp(
    theme: ThemeData(useMaterial3: useMaterial3, bottomAppBarTheme: theme),
    home: Scaffold(
      floatingActionButton: const FloatingActionButton(onPressed: null),
      floatingActionButtonLocation: FloatingActionButtonLocation.centerDocked,
      bottomNavigationBar: RepaintBoundary(
        key: _painterKey,
        child: const BottomAppBar(
          child: Row(
            children: <Widget>[Icon(Icons.add), Expanded(child: SizedBox()), Icon(Icons.add)],
          ),
        ),
      ),
    ),
  );
}
