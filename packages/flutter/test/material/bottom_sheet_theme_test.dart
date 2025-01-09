// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('BottomSheetThemeData copyWith, ==, hashCode basics', () {
    expect(const BottomSheetThemeData(), const BottomSheetThemeData().copyWith());
    expect(const BottomSheetThemeData().hashCode, const BottomSheetThemeData().copyWith().hashCode);
  });

  test('BottomSheetThemeData lerp special cases', () {
    expect(BottomSheetThemeData.lerp(null, null, 0), null);
    const BottomSheetThemeData data = BottomSheetThemeData();
    expect(identical(BottomSheetThemeData.lerp(data, data, 0.5), data), true);
  });

  test('BottomSheetThemeData lerp special cases', () {
    expect(BottomSheetThemeData.lerp(null, null, 0), null);
    const BottomSheetThemeData data = BottomSheetThemeData();
    expect(identical(BottomSheetThemeData.lerp(data, data, 0.5), data), true);
  });

  test('BottomSheetThemeData null fields by default', () {
    const BottomSheetThemeData bottomSheetTheme = BottomSheetThemeData();
    expect(bottomSheetTheme.backgroundColor, null);
    expect(bottomSheetTheme.shadowColor, null);
    expect(bottomSheetTheme.elevation, null);
    expect(bottomSheetTheme.shape, null);
    expect(bottomSheetTheme.clipBehavior, null);
    expect(bottomSheetTheme.constraints, null);
    expect(bottomSheetTheme.dragHandleColor, null);
    expect(bottomSheetTheme.dragHandleSize, null);
  });

  testWidgets('Default BottomSheetThemeData debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const BottomSheetThemeData().debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[]);
  });

  testWidgets('BottomSheetThemeData implements debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const BottomSheetThemeData(
      backgroundColor: Color(0xFFFFFFFF),
      elevation: 2.0,
      shadowColor: Color(0xFF00FFFF),
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(2.0))),
      clipBehavior: Clip.antiAlias,
      constraints: BoxConstraints(minWidth: 200, maxWidth: 640),
      dragHandleColor: Color(0xFFFFFFFF),
      dragHandleSize: Size(20, 20),
    ).debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[
      'backgroundColor: ${const Color(0xffffffff)}',
      'elevation: 2.0',
      'shadowColor: ${const Color(0xff00ffff)}',
      'shape: RoundedRectangleBorder(BorderSide(width: 0.0, style: none), BorderRadius.circular(2.0))',
      'dragHandleColor: ${const Color(0xffffffff)}',
      'dragHandleSize: Size(20.0, 20.0)',
      'clipBehavior: Clip.antiAlias',
      'constraints: BoxConstraints(200.0<=w<=640.0, 0.0<=h<=Infinity)',
    ]);
  });

  testWidgets('Material3 - Passing no BottomSheetThemeData returns defaults', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: Scaffold(
          body: BottomSheet(
            onClosing: () {},
            builder: (BuildContext context) {
              return Container();
            },
          ),
        ),
      ),
    );

    final Material material = tester.widget<Material>(
      find.descendant(of: find.byType(BottomSheet), matching: find.byType(Material)),
    );

    final ThemeData theme = Theme.of(tester.element(find.byType(Scaffold)));
    expect(material.color, theme.colorScheme.surfaceContainerLow);
    expect(material.elevation, 1.0);
    expect(
      material.shape,
      const RoundedRectangleBorder(borderRadius: BorderRadius.vertical(top: Radius.circular(28.0))),
    );
    expect(material.clipBehavior, Clip.none);
  });

  testWidgets('Material2 - Passing no BottomSheetThemeData returns defaults', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: BottomSheet(
            onClosing: () {},
            builder: (BuildContext context) {
              return Container();
            },
          ),
        ),
      ),
    );

    final Material material = tester.widget<Material>(
      find.descendant(of: find.byType(BottomSheet), matching: find.byType(Material)),
    );
    expect(material.color, null);
    expect(material.elevation, 0.0);
    expect(material.shape, null);
    expect(material.clipBehavior, Clip.none);
  });

  testWidgets('BottomSheet uses values from BottomSheetThemeData', (WidgetTester tester) async {
    final BottomSheetThemeData bottomSheetTheme = _bottomSheetTheme();

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(bottomSheetTheme: bottomSheetTheme),
        home: Scaffold(
          body: BottomSheet(
            onClosing: () {},
            builder: (BuildContext context) {
              return Container();
            },
          ),
        ),
      ),
    );

    final Material material = tester.widget<Material>(
      find.descendant(of: find.byType(BottomSheet), matching: find.byType(Material)),
    );
    expect(material.color, bottomSheetTheme.backgroundColor);
    expect(material.elevation, bottomSheetTheme.elevation);
    expect(material.shape, bottomSheetTheme.shape);
    expect(material.clipBehavior, bottomSheetTheme.clipBehavior);
  });

  testWidgets('BottomSheet widget properties take priority over theme', (
    WidgetTester tester,
  ) async {
    const Color backgroundColor = Colors.purple;
    const Color shadowColor = Colors.blue;
    const double elevation = 7.0;
    const ShapeBorder shape = RoundedRectangleBorder(
      borderRadius: BorderRadius.all(Radius.circular(9.0)),
    );
    const Clip clipBehavior = Clip.hardEdge;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(bottomSheetTheme: _bottomSheetTheme()),
        home: Scaffold(
          body: BottomSheet(
            backgroundColor: backgroundColor,
            shadowColor: shadowColor,
            elevation: elevation,
            shape: shape,
            clipBehavior: Clip.hardEdge,
            onClosing: () {},
            builder: (BuildContext context) {
              return Container();
            },
          ),
        ),
      ),
    );

    final Material material = tester.widget<Material>(
      find.descendant(of: find.byType(BottomSheet), matching: find.byType(Material)),
    );
    expect(material.color, backgroundColor);
    expect(material.shadowColor, shadowColor);
    expect(material.elevation, elevation);
    expect(material.shape, shape);
    expect(material.clipBehavior, clipBehavior);
  });

  testWidgets('Modal bottom sheet-specific parameters are used for modal bottom sheets', (
    WidgetTester tester,
  ) async {
    const double modalElevation = 5.0;
    const double persistentElevation = 7.0;
    const Color modalBackgroundColor = Colors.yellow;
    const Color modalBarrierColor = Colors.blue;
    const Color persistentBackgroundColor = Colors.red;
    const BottomSheetThemeData bottomSheetTheme = BottomSheetThemeData(
      elevation: persistentElevation,
      modalElevation: modalElevation,
      backgroundColor: persistentBackgroundColor,
      modalBackgroundColor: modalBackgroundColor,
      modalBarrierColor: modalBarrierColor,
    );

    await tester.pumpWidget(bottomSheetWithElevations(bottomSheetTheme));
    await tester.tap(find.text('Show Modal'));
    await tester.pumpAndSettle();

    final Material material = tester.widget<Material>(
      find.descendant(of: find.byType(BottomSheet), matching: find.byType(Material)),
    );
    expect(material.elevation, modalElevation);
    expect(material.color, modalBackgroundColor);

    final ModalBarrier modalBarrier = tester.widget(find.byType(ModalBarrier).last);
    expect(modalBarrier.color, modalBarrierColor);
  });

  testWidgets(
    'General bottom sheet parameters take priority over modal bottom sheet-specific parameters for persistent bottom sheets',
    (WidgetTester tester) async {
      const double modalElevation = 5.0;
      const double persistentElevation = 7.0;
      const Color modalBackgroundColor = Colors.yellow;
      const Color persistentBackgroundColor = Colors.red;
      const BottomSheetThemeData bottomSheetTheme = BottomSheetThemeData(
        elevation: persistentElevation,
        modalElevation: modalElevation,
        backgroundColor: persistentBackgroundColor,
        modalBackgroundColor: modalBackgroundColor,
      );

      await tester.pumpWidget(bottomSheetWithElevations(bottomSheetTheme));
      await tester.tap(find.text('Show Persistent'));
      await tester.pumpAndSettle();

      final Material material = tester.widget<Material>(
        find.descendant(of: find.byType(BottomSheet), matching: find.byType(Material)),
      );
      expect(material.elevation, persistentElevation);
      expect(material.color, persistentBackgroundColor);
    },
  );

  testWidgets(
    "Material3 - Modal bottom sheet-specific parameters don't apply to persistent bottom sheets",
    (WidgetTester tester) async {
      const double modalElevation = 5.0;
      const Color modalBackgroundColor = Colors.yellow;
      const BottomSheetThemeData bottomSheetTheme = BottomSheetThemeData(
        modalElevation: modalElevation,
        modalBackgroundColor: modalBackgroundColor,
      );

      await tester.pumpWidget(bottomSheetWithElevations(bottomSheetTheme));
      await tester.tap(find.text('Show Persistent'));
      await tester.pumpAndSettle();

      final Material material = tester.widget<Material>(
        find.descendant(of: find.byType(BottomSheet), matching: find.byType(Material)),
      );
      expect(material.elevation, 1.0);
      final ThemeData theme = Theme.of(tester.element(find.byType(Scaffold)));
      expect(material.color, theme.colorScheme.surfaceContainerLow);
    },
  );

  testWidgets(
    "Material2 - Modal bottom sheet-specific parameters don't apply to persistent bottom sheets",
    (WidgetTester tester) async {
      const double modalElevation = 5.0;
      const Color modalBackgroundColor = Colors.yellow;
      const BottomSheetThemeData bottomSheetTheme = BottomSheetThemeData(
        modalElevation: modalElevation,
        modalBackgroundColor: modalBackgroundColor,
      );

      await tester.pumpWidget(bottomSheetWithElevations(bottomSheetTheme, useMaterial3: false));
      await tester.tap(find.text('Show Persistent'));
      await tester.pumpAndSettle();

      final Material material = tester.widget<Material>(
        find.descendant(of: find.byType(BottomSheet), matching: find.byType(Material)),
      );
      expect(material.elevation, 0);
      expect(material.color, null);
    },
  );

  testWidgets('Modal bottom sheets respond to theme changes', (WidgetTester tester) async {
    const double lightElevation = 5.0;
    const double darkElevation = 3.0;
    const Color lightBackgroundColor = Colors.green;
    const Color darkBackgroundColor = Colors.grey;
    const Color lightShadowColor = Colors.blue;
    const Color darkShadowColor = Colors.purple;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.light().copyWith(
          bottomSheetTheme: const BottomSheetThemeData(
            elevation: lightElevation,
            backgroundColor: lightBackgroundColor,
            shadowColor: lightShadowColor,
          ),
        ),
        darkTheme: ThemeData.dark().copyWith(
          bottomSheetTheme: const BottomSheetThemeData(
            elevation: darkElevation,
            backgroundColor: darkBackgroundColor,
            shadowColor: darkShadowColor,
          ),
        ),
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              return Column(
                children: <Widget>[
                  RawMaterialButton(
                    child: const Text('Show Modal'),
                    onPressed: () {
                      showModalBottomSheet<void>(
                        context: context,
                        builder: (BuildContext context) {
                          return const Text('This is a modal bottom sheet.');
                        },
                      );
                    },
                  ),
                ],
              );
            },
          ),
        ),
      ),
    );
    await tester.tap(find.text('Show Modal'));
    await tester.pumpAndSettle();

    final Material lightMaterial = tester.widget<Material>(
      find.descendant(of: find.byType(BottomSheet), matching: find.byType(Material)),
    );
    expect(lightMaterial.elevation, lightElevation);
    expect(lightMaterial.color, lightBackgroundColor);
    expect(lightMaterial.shadowColor, lightShadowColor);

    // Simulate the user changing to dark theme
    tester.binding.platformDispatcher.platformBrightnessTestValue = Brightness.dark;
    await tester.pumpAndSettle();

    final Material darkMaterial = tester.widget<Material>(
      find.descendant(of: find.byType(BottomSheet), matching: find.byType(Material)),
    );
    expect(darkMaterial.elevation, darkElevation);
    expect(darkMaterial.color, darkBackgroundColor);
    expect(darkMaterial.shadowColor, darkShadowColor);
  });
}

Widget bottomSheetWithElevations(
  BottomSheetThemeData bottomSheetTheme, {
  bool useMaterial3 = true,
}) {
  return MaterialApp(
    theme: ThemeData(bottomSheetTheme: bottomSheetTheme, useMaterial3: useMaterial3),
    home: Scaffold(
      body: Builder(
        builder: (BuildContext context) {
          return Column(
            children: <Widget>[
              RawMaterialButton(
                child: const Text('Show Modal'),
                onPressed: () {
                  showModalBottomSheet<void>(
                    context: context,
                    builder: (BuildContext _) {
                      return const Text('This is a modal bottom sheet.');
                    },
                  );
                },
              ),
              RawMaterialButton(
                child: const Text('Show Persistent'),
                onPressed: () {
                  showBottomSheet(
                    context: context,
                    builder: (BuildContext _) {
                      return const Text('This is a persistent bottom sheet.');
                    },
                  );
                },
              ),
            ],
          );
        },
      ),
    ),
  );
}

BottomSheetThemeData _bottomSheetTheme() {
  return const BottomSheetThemeData(
    backgroundColor: Colors.orange,
    elevation: 12.0,
    shape: BeveledRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(12))),
    clipBehavior: Clip.antiAlias,
  );
}
