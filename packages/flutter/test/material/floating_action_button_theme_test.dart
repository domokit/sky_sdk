// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('FloatingActionButtonThemeData copyWith, ==, hashCode basics', () {
    expect(const FloatingActionButtonThemeData(), const FloatingActionButtonThemeData().copyWith());
    expect(
      const FloatingActionButtonThemeData().hashCode,
      const FloatingActionButtonThemeData().copyWith().hashCode,
    );
  });

  test('FloatingActionButtonThemeData lerp special cases', () {
    expect(FloatingActionButtonThemeData.lerp(null, null, 0), null);
    const FloatingActionButtonThemeData data = FloatingActionButtonThemeData();
    expect(identical(FloatingActionButtonThemeData.lerp(data, data, 0.5), data), true);
  });

  testWidgets(
    'Material3: Default values are used when no FloatingActionButton or FloatingActionButtonThemeData properties are specified',
    (WidgetTester tester) async {
      const ColorScheme colorScheme = ColorScheme.light();
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData.from(colorScheme: colorScheme),
          home: Scaffold(
            floatingActionButton: FloatingActionButton(
              onPressed: () {},
              child: const Icon(Icons.add),
            ),
          ),
        ),
      );

      expect(_getRawMaterialButton(tester).fillColor, colorScheme.primaryContainer);
      expect(_getRichText(tester).text.style!.color, colorScheme.onPrimaryContainer);

      // These defaults come directly from the [FloatingActionButton].
      expect(_getRawMaterialButton(tester).elevation, 6);
      expect(_getRawMaterialButton(tester).highlightElevation, 6);
      expect(
        _getRawMaterialButton(tester).shape,
        const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(16.0))),
      );
      expect(
        _getRawMaterialButton(tester).splashColor,
        colorScheme.onPrimaryContainer.withOpacity(0.1),
      );
      expect(
        _getRawMaterialButton(tester).constraints,
        const BoxConstraints.tightFor(width: 56.0, height: 56.0),
      );
      expect(_getIconSize(tester).width, 24.0);
      expect(_getIconSize(tester).height, 24.0);
    },
  );

  testWidgets(
    'Material2: Default values are used when no FloatingActionButton or FloatingActionButtonThemeData properties are specified',
    (WidgetTester tester) async {
      const ColorScheme colorScheme = ColorScheme.light();
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData.from(useMaterial3: false, colorScheme: colorScheme),
          home: Scaffold(
            floatingActionButton: FloatingActionButton(
              onPressed: () {},
              child: const Icon(Icons.add),
            ),
          ),
        ),
      );

      expect(_getRawMaterialButton(tester).fillColor, colorScheme.secondary);
      expect(_getRichText(tester).text.style!.color, colorScheme.onSecondary);

      // These defaults come directly from the [FloatingActionButton].
      expect(_getRawMaterialButton(tester).elevation, 6);
      expect(_getRawMaterialButton(tester).highlightElevation, 12);
      expect(_getRawMaterialButton(tester).shape, const CircleBorder());
      expect(_getRawMaterialButton(tester).splashColor, ThemeData().splashColor);
      expect(
        _getRawMaterialButton(tester).constraints,
        const BoxConstraints.tightFor(width: 56.0, height: 56.0),
      );
      expect(_getIconSize(tester).width, 24.0);
      expect(_getIconSize(tester).height, 24.0);
    },
  );

  testWidgets(
    'FloatingActionButtonThemeData values are used when no FloatingActionButton properties are specified',
    (WidgetTester tester) async {
      const Color backgroundColor = Color(0xBEEFBEEF);
      const Color foregroundColor = Color(0xFACEFACE);
      const Color splashColor = Color(0xCAFEFEED);
      const double elevation = 7;
      const double disabledElevation = 1;
      const double highlightElevation = 13;
      const ShapeBorder shape = StadiumBorder();
      const BoxConstraints constraints = BoxConstraints.tightFor(width: 100.0, height: 100.0);

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData().copyWith(
            floatingActionButtonTheme: const FloatingActionButtonThemeData(
              backgroundColor: backgroundColor,
              foregroundColor: foregroundColor,
              splashColor: splashColor,
              elevation: elevation,
              disabledElevation: disabledElevation,
              highlightElevation: highlightElevation,
              shape: shape,
              sizeConstraints: constraints,
            ),
          ),
          home: Scaffold(
            floatingActionButton: FloatingActionButton(
              onPressed: () {},
              child: const Icon(Icons.add),
            ),
          ),
        ),
      );

      expect(_getRawMaterialButton(tester).fillColor, backgroundColor);
      expect(_getRichText(tester).text.style!.color, foregroundColor);
      expect(_getRawMaterialButton(tester).elevation, elevation);
      expect(_getRawMaterialButton(tester).disabledElevation, disabledElevation);
      expect(_getRawMaterialButton(tester).highlightElevation, highlightElevation);
      expect(_getRawMaterialButton(tester).shape, shape);
      expect(_getRawMaterialButton(tester).splashColor, splashColor);
      expect(_getRawMaterialButton(tester).constraints, constraints);
    },
  );

  testWidgets(
    'FloatingActionButton values take priority over FloatingActionButtonThemeData values when both properties are specified',
    (WidgetTester tester) async {
      const Color backgroundColor = Color(0x00000001);
      const Color foregroundColor = Color(0x00000002);
      const Color splashColor = Color(0x00000003);
      const double elevation = 7;
      const double disabledElevation = 1;
      const double highlightElevation = 13;
      const ShapeBorder shape = StadiumBorder();

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData().copyWith(
            floatingActionButtonTheme: const FloatingActionButtonThemeData(
              backgroundColor: Color(0x00000004),
              foregroundColor: Color(0x00000005),
              splashColor: Color(0x00000006),
              elevation: 23,
              disabledElevation: 11,
              highlightElevation: 43,
              shape: BeveledRectangleBorder(),
            ),
          ),
          home: Scaffold(
            floatingActionButton: FloatingActionButton(
              onPressed: () {},
              backgroundColor: backgroundColor,
              foregroundColor: foregroundColor,
              splashColor: splashColor,
              elevation: elevation,
              disabledElevation: disabledElevation,
              highlightElevation: highlightElevation,
              shape: shape,
              child: const Icon(Icons.add),
            ),
          ),
        ),
      );

      expect(_getRawMaterialButton(tester).fillColor, backgroundColor);
      expect(_getRichText(tester).text.style!.color, foregroundColor);
      expect(_getRawMaterialButton(tester).elevation, elevation);
      expect(_getRawMaterialButton(tester).disabledElevation, disabledElevation);
      expect(_getRawMaterialButton(tester).highlightElevation, highlightElevation);
      expect(_getRawMaterialButton(tester).shape, shape);
      expect(_getRawMaterialButton(tester).splashColor, splashColor);
    },
  );

  testWidgets('FloatingActionButton uses a custom shape when specified in the theme', (
    WidgetTester tester,
  ) async {
    const ShapeBorder customShape = BeveledRectangleBorder();

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          floatingActionButton: FloatingActionButton(onPressed: () {}, shape: customShape),
        ),
      ),
    );

    expect(_getRawMaterialButton(tester).shape, customShape);
  });

  testWidgets('FloatingActionButton.small uses custom constraints when specified in the theme', (
    WidgetTester tester,
  ) async {
    const BoxConstraints constraints = BoxConstraints.tightFor(width: 100.0, height: 100.0);
    const double iconSize = 24.0;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData().copyWith(
          floatingActionButtonTheme: const FloatingActionButtonThemeData(
            smallSizeConstraints: constraints,
          ),
        ),
        home: Scaffold(
          floatingActionButton: FloatingActionButton.small(
            onPressed: () {},
            child: const Icon(Icons.add),
          ),
        ),
      ),
    );

    expect(_getRawMaterialButton(tester).constraints, constraints);
    expect(_getIconSize(tester).width, iconSize);
    expect(_getIconSize(tester).height, iconSize);
  });

  testWidgets('FloatingActionButton.large uses custom constraints when specified in the theme', (
    WidgetTester tester,
  ) async {
    const BoxConstraints constraints = BoxConstraints.tightFor(width: 100.0, height: 100.0);
    const double iconSize = 36.0;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData().copyWith(
          floatingActionButtonTheme: const FloatingActionButtonThemeData(
            largeSizeConstraints: constraints,
          ),
        ),
        home: Scaffold(
          floatingActionButton: FloatingActionButton.large(
            onPressed: () {},
            child: const Icon(Icons.add),
          ),
        ),
      ),
    );

    expect(_getRawMaterialButton(tester).constraints, constraints);
    expect(_getIconSize(tester).width, iconSize);
    expect(_getIconSize(tester).height, iconSize);
  });

  testWidgets(
    'Material3: FloatingActionButton.extended uses custom properties when specified in the theme',
    (WidgetTester tester) async {
      const ColorScheme colorScheme = ColorScheme.light();
      const Key iconKey = Key('icon');
      const Key labelKey = Key('label');
      const BoxConstraints constraints = BoxConstraints.tightFor(height: 100.0);
      const double iconLabelSpacing = 33.0;
      const EdgeInsetsDirectional padding = EdgeInsetsDirectional.only(start: 5.0, end: 6.0);
      const TextStyle textStyle = TextStyle(letterSpacing: 2.0);

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(colorScheme: colorScheme).copyWith(
            floatingActionButtonTheme: const FloatingActionButtonThemeData(
              extendedSizeConstraints: constraints,
              extendedIconLabelSpacing: iconLabelSpacing,
              extendedPadding: padding,
              extendedTextStyle: textStyle,
            ),
          ),
          home: Scaffold(
            floatingActionButton: FloatingActionButton.extended(
              onPressed: () {},
              label: const Text('Extended', key: labelKey),
              icon: const Icon(Icons.add, key: iconKey),
            ),
          ),
        ),
      );

      expect(_getRawMaterialButton(tester).constraints, constraints);
      expect(
        tester.getTopLeft(find.byKey(labelKey)).dx - tester.getTopRight(find.byKey(iconKey)).dx,
        iconLabelSpacing,
      );
      expect(
        tester.getTopLeft(find.byKey(iconKey)).dx -
            tester.getTopLeft(find.byType(FloatingActionButton)).dx,
        padding.start,
      );
      expect(
        tester.getTopRight(find.byType(FloatingActionButton)).dx -
            tester.getTopRight(find.byKey(labelKey)).dx,
        padding.end,
      );
      expect(
        _getRawMaterialButton(tester).textStyle,
        textStyle.copyWith(color: colorScheme.onPrimaryContainer),
      );
    },
  );

  testWidgets(
    'Material2: FloatingActionButton.extended uses custom properties when specified in the theme',
    (WidgetTester tester) async {
      const Key iconKey = Key('icon');
      const Key labelKey = Key('label');
      const BoxConstraints constraints = BoxConstraints.tightFor(height: 100.0);
      const double iconLabelSpacing = 33.0;
      const EdgeInsetsDirectional padding = EdgeInsetsDirectional.only(start: 5.0, end: 6.0);
      const TextStyle textStyle = TextStyle(letterSpacing: 2.0);

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false).copyWith(
            floatingActionButtonTheme: const FloatingActionButtonThemeData(
              extendedSizeConstraints: constraints,
              extendedIconLabelSpacing: iconLabelSpacing,
              extendedPadding: padding,
              extendedTextStyle: textStyle,
            ),
          ),
          home: Scaffold(
            floatingActionButton: FloatingActionButton.extended(
              onPressed: () {},
              label: const Text('Extended', key: labelKey),
              icon: const Icon(Icons.add, key: iconKey),
            ),
          ),
        ),
      );

      expect(_getRawMaterialButton(tester).constraints, constraints);
      expect(
        tester.getTopLeft(find.byKey(labelKey)).dx - tester.getTopRight(find.byKey(iconKey)).dx,
        iconLabelSpacing,
      );
      expect(
        tester.getTopLeft(find.byKey(iconKey)).dx -
            tester.getTopLeft(find.byType(FloatingActionButton)).dx,
        padding.start,
      );
      expect(
        tester.getTopRight(find.byType(FloatingActionButton)).dx -
            tester.getTopRight(find.byKey(labelKey)).dx,
        padding.end,
      );
      // The color comes from the default color scheme's onSecondary value.
      expect(
        _getRawMaterialButton(tester).textStyle,
        textStyle.copyWith(color: const Color(0xffffffff)),
      );
    },
  );

  testWidgets(
    'Material3: FloatingActionButton.extended custom properties takes priority over FloatingActionButtonThemeData spacing',
    (WidgetTester tester) async {
      const ColorScheme colorScheme = ColorScheme.light();
      const Key iconKey = Key('icon');
      const Key labelKey = Key('label');
      const double iconLabelSpacing = 33.0;
      const EdgeInsetsDirectional padding = EdgeInsetsDirectional.only(start: 5.0, end: 6.0);
      const TextStyle textStyle = TextStyle(letterSpacing: 2.0);

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(colorScheme: colorScheme).copyWith(
            floatingActionButtonTheme: const FloatingActionButtonThemeData(
              extendedIconLabelSpacing: 25.0,
              extendedPadding: EdgeInsetsDirectional.only(start: 7.0, end: 8.0),
              extendedTextStyle: TextStyle(letterSpacing: 3.0),
            ),
          ),
          home: Scaffold(
            floatingActionButton: FloatingActionButton.extended(
              onPressed: () {},
              label: const Text('Extended', key: labelKey),
              icon: const Icon(Icons.add, key: iconKey),
              extendedIconLabelSpacing: iconLabelSpacing,
              extendedPadding: padding,
              extendedTextStyle: textStyle,
            ),
          ),
        ),
      );

      expect(
        tester.getTopLeft(find.byKey(labelKey)).dx - tester.getTopRight(find.byKey(iconKey)).dx,
        iconLabelSpacing,
      );
      expect(
        tester.getTopLeft(find.byKey(iconKey)).dx -
            tester.getTopLeft(find.byType(FloatingActionButton)).dx,
        padding.start,
      );
      expect(
        tester.getTopRight(find.byType(FloatingActionButton)).dx -
            tester.getTopRight(find.byKey(labelKey)).dx,
        padding.end,
      );
      expect(
        _getRawMaterialButton(tester).textStyle,
        textStyle.copyWith(color: colorScheme.onPrimaryContainer),
      );
    },
  );

  testWidgets(
    'Material2: FloatingActionButton.extended custom properties takes priority over FloatingActionButtonThemeData spacing',
    (WidgetTester tester) async {
      const Key iconKey = Key('icon');
      const Key labelKey = Key('label');
      const double iconLabelSpacing = 33.0;
      const EdgeInsetsDirectional padding = EdgeInsetsDirectional.only(start: 5.0, end: 6.0);
      const TextStyle textStyle = TextStyle(letterSpacing: 2.0);

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false).copyWith(
            floatingActionButtonTheme: const FloatingActionButtonThemeData(
              extendedIconLabelSpacing: 25.0,
              extendedPadding: EdgeInsetsDirectional.only(start: 7.0, end: 8.0),
              extendedTextStyle: TextStyle(letterSpacing: 3.0),
            ),
          ),
          home: Scaffold(
            floatingActionButton: FloatingActionButton.extended(
              onPressed: () {},
              label: const Text('Extended', key: labelKey),
              icon: const Icon(Icons.add, key: iconKey),
              extendedIconLabelSpacing: iconLabelSpacing,
              extendedPadding: padding,
              extendedTextStyle: textStyle,
            ),
          ),
        ),
      );

      expect(
        tester.getTopLeft(find.byKey(labelKey)).dx - tester.getTopRight(find.byKey(iconKey)).dx,
        iconLabelSpacing,
      );
      expect(
        tester.getTopLeft(find.byKey(iconKey)).dx -
            tester.getTopLeft(find.byType(FloatingActionButton)).dx,
        padding.start,
      );
      expect(
        tester.getTopRight(find.byType(FloatingActionButton)).dx -
            tester.getTopRight(find.byKey(labelKey)).dx,
        padding.end,
      );
      // The color comes from the default color scheme's onSecondary value.
      expect(
        _getRawMaterialButton(tester).textStyle,
        textStyle.copyWith(color: const Color(0xffffffff)),
      );
    },
  );

  testWidgets('default FloatingActionButton debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const FloatingActionButtonThemeData().debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[]);
  });

  testWidgets('Material implements debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const FloatingActionButtonThemeData(
      foregroundColor: Color(0xFEEDFEED),
      backgroundColor: Color(0xCAFECAFE),
      focusColor: Color(0xFEEDFEE1),
      hoverColor: Color(0xFEEDFEE2),
      splashColor: Color(0xFEEDFEE3),
      elevation: 23,
      focusElevation: 9,
      hoverElevation: 10,
      disabledElevation: 11,
      highlightElevation: 43,
      shape: BeveledRectangleBorder(),
      enableFeedback: true,
      iconSize: 42,
      sizeConstraints: BoxConstraints.tightFor(width: 100.0, height: 100.0),
      smallSizeConstraints: BoxConstraints.tightFor(width: 101.0, height: 101.0),
      largeSizeConstraints: BoxConstraints.tightFor(width: 102.0, height: 102.0),
      extendedSizeConstraints: BoxConstraints(minHeight: 103.0, maxHeight: 103.0),
      extendedIconLabelSpacing: 12,
      extendedPadding: EdgeInsetsDirectional.only(start: 7.0, end: 8.0),
      extendedTextStyle: TextStyle(letterSpacing: 2.0),
      mouseCursor: MaterialStateMouseCursor.clickable,
    ).debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[
      'foregroundColor: ${const Color(0xfeedfeed)}',
      'backgroundColor: ${const Color(0xcafecafe)}',
      'focusColor: ${const Color(0xfeedfee1)}',
      'hoverColor: ${const Color(0xfeedfee2)}',
      'splashColor: ${const Color(0xfeedfee3)}',
      'elevation: 23.0',
      'focusElevation: 9.0',
      'hoverElevation: 10.0',
      'disabledElevation: 11.0',
      'highlightElevation: 43.0',
      'shape: BeveledRectangleBorder(BorderSide(width: 0.0, style: none), BorderRadius.zero)',
      'enableFeedback: true',
      'iconSize: 42.0',
      'sizeConstraints: BoxConstraints(w=100.0, h=100.0)',
      'smallSizeConstraints: BoxConstraints(w=101.0, h=101.0)',
      'largeSizeConstraints: BoxConstraints(w=102.0, h=102.0)',
      'extendedSizeConstraints: BoxConstraints(0.0<=w<=Infinity, h=103.0)',
      'extendedIconLabelSpacing: 12.0',
      'extendedPadding: EdgeInsetsDirectional(7.0, 0.0, 8.0, 0.0)',
      'extendedTextStyle: TextStyle(inherit: true, letterSpacing: 2.0)',
      'mouseCursor: WidgetStateMouseCursor(clickable)',
    ]);
  });

  testWidgets(
    'FloatingActionButton.mouseCursor uses FloatingActionButtonThemeData.mouseCursor when specified.',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData().copyWith(
            floatingActionButtonTheme: FloatingActionButtonThemeData(
              mouseCursor: MaterialStateProperty.all(SystemMouseCursors.text),
            ),
          ),
          home: Scaffold(
            floatingActionButton: FloatingActionButton(
              onPressed: () {},
              child: const Icon(Icons.add),
            ),
          ),
        ),
      );

      await tester.pumpAndSettle();
      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture.addPointer();
      addTearDown(gesture.removePointer);
      await gesture.moveTo(tester.getCenter(find.byType(FloatingActionButton)));
      await tester.pumpAndSettle();
      expect(
        RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
        SystemMouseCursors.text,
      );
    },
  );
}

RawMaterialButton _getRawMaterialButton(WidgetTester tester) {
  return tester.widget<RawMaterialButton>(
    find.descendant(
      of: find.byType(FloatingActionButton),
      matching: find.byType(RawMaterialButton),
    ),
  );
}

RichText _getRichText(WidgetTester tester) {
  return tester.widget<RichText>(
    find.descendant(of: find.byType(FloatingActionButton), matching: find.byType(RichText)),
  );
}

SizedBox _getIconSize(WidgetTester tester) {
  return tester.widget<SizedBox>(
    find.descendant(
      of: find.descendant(of: find.byType(FloatingActionButton), matching: find.byType(Icon)),
      matching: find.byType(SizedBox),
    ),
  );
}
