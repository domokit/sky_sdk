// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('RadioThemeData copyWith, ==, hashCode basics', () {
    expect(const RadioThemeData(), const RadioThemeData().copyWith());
    expect(const RadioThemeData().hashCode, const RadioThemeData().copyWith().hashCode);
  });

  test('RadioThemeData lerp special cases', () {
    expect(RadioThemeData.lerp(null, null, 0), const RadioThemeData());
    const RadioThemeData data = RadioThemeData();
    expect(identical(RadioThemeData.lerp(data, data, 0.5), data), true);
  });

  test('RadioThemeData defaults', () {
    const RadioThemeData themeData = RadioThemeData();
    expect(themeData.mouseCursor, null);
    expect(themeData.fillColor, null);
    expect(themeData.overlayColor, null);
    expect(themeData.splashRadius, null);
    expect(themeData.materialTapTargetSize, null);
    expect(themeData.visualDensity, null);

    const RadioTheme theme = RadioTheme(data: RadioThemeData(), child: SizedBox());
    expect(theme.data.mouseCursor, null);
    expect(theme.data.fillColor, null);
    expect(theme.data.overlayColor, null);
    expect(theme.data.splashRadius, null);
    expect(theme.data.materialTapTargetSize, null);
    expect(theme.data.visualDensity, null);
  });

  testWidgets('Default RadioThemeData debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const RadioThemeData().debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[]);
  });

  testWidgets('RadioThemeData implements debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const RadioThemeData(
      mouseCursor: MaterialStatePropertyAll<MouseCursor>(SystemMouseCursors.click),
      fillColor: MaterialStatePropertyAll<Color>(Color(0xfffffff0)),
      overlayColor: MaterialStatePropertyAll<Color>(Color(0xfffffff1)),
      splashRadius: 1.0,
      materialTapTargetSize: MaterialTapTargetSize.shrinkWrap,
      visualDensity: VisualDensity.standard,
    ).debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(
      description,
      equalsIgnoringHashCodes(<String>[
        'mouseCursor: WidgetStatePropertyAll(SystemMouseCursor(click))',
        'fillColor: WidgetStatePropertyAll(${const Color(0xfffffff0)})',
        'overlayColor: WidgetStatePropertyAll(${const Color(0xfffffff1)})',
        'splashRadius: 1.0',
        'materialTapTargetSize: MaterialTapTargetSize.shrinkWrap',
        'visualDensity: VisualDensity#00000(h: 0.0, v: 0.0)',
      ]),
    );
  });

  testWidgets('Radio is themeable', (WidgetTester tester) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;

    const MouseCursor mouseCursor = SystemMouseCursors.text;
    const Color defaultFillColor = Color(0xfffffff0);
    const Color selectedFillColor = Color(0xfffffff1);
    const Color focusOverlayColor = Color(0xfffffff2);
    const Color hoverOverlayColor = Color(0xfffffff3);
    const double splashRadius = 1.0;
    const MaterialTapTargetSize materialTapTargetSize = MaterialTapTargetSize.shrinkWrap;
    const VisualDensity visualDensity = VisualDensity(horizontal: 1, vertical: 1);

    Widget buildRadio({bool selected = false, bool autofocus = false}) {
      return MaterialApp(
        theme: ThemeData(
          radioTheme: RadioThemeData(
            mouseCursor: const MaterialStatePropertyAll<MouseCursor>(mouseCursor),
            fillColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
              if (states.contains(MaterialState.selected)) {
                return selectedFillColor;
              }
              return defaultFillColor;
            }),
            overlayColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
              if (states.contains(MaterialState.focused)) {
                return focusOverlayColor;
              }
              if (states.contains(MaterialState.hovered)) {
                return hoverOverlayColor;
              }
              return null;
            }),
            splashRadius: splashRadius,
            materialTapTargetSize: materialTapTargetSize,
            visualDensity: visualDensity,
          ),
        ),
        home: Scaffold(
          body: Radio<int>(
            onChanged: (int? int) {},
            value: selected ? 1 : 0,
            groupValue: 1,
            autofocus: autofocus,
          ),
        ),
      );
    }

    // Radio.
    await tester.pumpWidget(buildRadio());
    await tester.pumpAndSettle();
    expect(_getRadioMaterial(tester), paints..circle(color: defaultFillColor));
    // Size from MaterialTapTargetSize.shrinkWrap with added VisualDensity.
    expect(tester.getSize(_findRadio()), const Size(40.0, 40.0) + visualDensity.baseSizeAdjustment);

    // Selected radio.
    await tester.pumpWidget(buildRadio(selected: true));
    await tester.pumpAndSettle();
    expect(_getRadioMaterial(tester), paints..circle(color: selectedFillColor));

    // Radio with hover.
    await tester.pumpWidget(buildRadio());
    await _pointGestureToRadio(tester);
    await tester.pumpAndSettle();
    expect(_getRadioMaterial(tester), paints..circle(color: hoverOverlayColor));
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );

    // Radio with focus.
    await tester.pumpWidget(buildRadio(autofocus: true));
    await tester.pumpAndSettle();
    expect(
      _getRadioMaterial(tester),
      paints..circle(color: focusOverlayColor, radius: splashRadius),
    );
  });

  testWidgets('Radio properties are taken over the theme values', (WidgetTester tester) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;

    const MouseCursor themeMouseCursor = SystemMouseCursors.click;
    const Color themeDefaultFillColor = Color(0xfffffff0);
    const Color themeSelectedFillColor = Color(0xfffffff1);
    const Color themeFocusOverlayColor = Color(0xfffffff2);
    const Color themeHoverOverlayColor = Color(0xfffffff3);
    const double themeSplashRadius = 1.0;
    const MaterialTapTargetSize themeMaterialTapTargetSize = MaterialTapTargetSize.padded;
    const VisualDensity themeVisualDensity = VisualDensity.standard;

    const MouseCursor mouseCursor = SystemMouseCursors.text;
    const Color defaultFillColor = Color(0xfffffff0);
    const Color selectedFillColor = Color(0xfffffff1);
    const Color focusColor = Color(0xfffffff2);
    const Color hoverColor = Color(0xfffffff3);
    const double splashRadius = 2.0;
    const MaterialTapTargetSize materialTapTargetSize = MaterialTapTargetSize.shrinkWrap;
    const VisualDensity visualDensity = VisualDensity(horizontal: 1, vertical: 1);

    Widget buildRadio({bool selected = false, bool autofocus = false}) {
      return MaterialApp(
        theme: ThemeData(
          radioTheme: RadioThemeData(
            mouseCursor: const MaterialStatePropertyAll<MouseCursor>(themeMouseCursor),
            fillColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
              if (states.contains(MaterialState.selected)) {
                return themeSelectedFillColor;
              }
              return themeDefaultFillColor;
            }),
            overlayColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
              if (states.contains(MaterialState.focused)) {
                return themeFocusOverlayColor;
              }
              if (states.contains(MaterialState.hovered)) {
                return themeHoverOverlayColor;
              }
              return null;
            }),
            splashRadius: themeSplashRadius,
            materialTapTargetSize: themeMaterialTapTargetSize,
            visualDensity: themeVisualDensity,
          ),
        ),
        home: Scaffold(
          body: Radio<int>(
            onChanged: (int? int) {},
            value: selected ? 0 : 1,
            groupValue: 0,
            autofocus: autofocus,
            mouseCursor: mouseCursor,
            fillColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
              if (states.contains(MaterialState.selected)) {
                return selectedFillColor;
              }
              return defaultFillColor;
            }),
            focusColor: focusColor,
            hoverColor: hoverColor,
            splashRadius: splashRadius,
            materialTapTargetSize: materialTapTargetSize,
            visualDensity: visualDensity,
          ),
        ),
      );
    }

    // Radio.
    await tester.pumpWidget(buildRadio());
    await tester.pumpAndSettle();
    expect(_getRadioMaterial(tester), paints..circle(color: defaultFillColor));
    // Size from MaterialTapTargetSize.shrinkWrap with added VisualDensity.
    expect(tester.getSize(_findRadio()), const Size(40.0, 40.0) + visualDensity.baseSizeAdjustment);

    // Selected radio.
    await tester.pumpWidget(buildRadio(selected: true));
    await tester.pumpAndSettle();
    expect(_getRadioMaterial(tester), paints..circle(color: selectedFillColor));

    // Radio with hover.
    await tester.pumpWidget(buildRadio());
    await _pointGestureToRadio(tester);
    await tester.pumpAndSettle();
    expect(_getRadioMaterial(tester), paints..circle(color: hoverColor));
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );

    // Radio with focus.
    await tester.pumpWidget(buildRadio(autofocus: true));
    await tester.pumpAndSettle();
    expect(_getRadioMaterial(tester), paints..circle(color: focusColor, radius: splashRadius));
  });

  testWidgets('Radio activeColor property is taken over the theme', (WidgetTester tester) async {
    const Color themeDefaultFillColor = Color(0xfffffff0);
    const Color themeSelectedFillColor = Color(0xfffffff1);

    const Color selectedFillColor = Color(0xfffffff1);

    Widget buildRadio({bool selected = false, bool autofocus = false}) {
      return MaterialApp(
        theme: ThemeData(
          radioTheme: RadioThemeData(
            fillColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
              if (states.contains(MaterialState.selected)) {
                return themeSelectedFillColor;
              }
              return themeDefaultFillColor;
            }),
          ),
        ),
        home: Scaffold(
          body: Radio<int>(
            onChanged: (int? int) {},
            value: selected ? 0 : 1,
            groupValue: 0,
            autofocus: autofocus,
            activeColor: selectedFillColor,
          ),
        ),
      );
    }

    // Radio.
    await tester.pumpWidget(buildRadio());
    await tester.pumpAndSettle();
    expect(_getRadioMaterial(tester), paints..circle(color: themeDefaultFillColor));

    // Selected radio.
    await tester.pumpWidget(buildRadio(selected: true));
    await tester.pumpAndSettle();
    expect(_getRadioMaterial(tester), paints..circle(color: selectedFillColor));
  });

  testWidgets('Radio theme overlay color resolves in active/pressed states', (
    WidgetTester tester,
  ) async {
    const Color activePressedOverlayColor = Color(0xFF000001);
    const Color inactivePressedOverlayColor = Color(0xFF000002);

    Color? getOverlayColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.pressed)) {
        if (states.contains(MaterialState.selected)) {
          return activePressedOverlayColor;
        }
        return inactivePressedOverlayColor;
      }
      return null;
    }

    const double splashRadius = 24.0;

    Widget buildRadio({required bool active}) {
      return MaterialApp(
        theme: ThemeData(
          radioTheme: RadioThemeData(
            overlayColor: MaterialStateProperty.resolveWith(getOverlayColor),
            splashRadius: splashRadius,
          ),
        ),
        home: Scaffold(body: Radio<int>(value: active ? 1 : 0, groupValue: 1, onChanged: (_) {})),
      );
    }

    await tester.pumpWidget(buildRadio(active: false));
    await tester.press(_findRadio());
    await tester.pumpAndSettle();

    expect(
      _getRadioMaterial(tester),
      paints..circle(color: inactivePressedOverlayColor, radius: splashRadius),
      reason: 'Inactive pressed Radio should have overlay color: $inactivePressedOverlayColor',
    );

    await tester.pumpWidget(buildRadio(active: true));
    await tester.press(_findRadio());
    await tester.pumpAndSettle();

    expect(
      _getRadioMaterial(tester),
      paints..circle(color: activePressedOverlayColor, radius: splashRadius),
      reason: 'Active pressed Radio should have overlay color: $activePressedOverlayColor',
    );
  });

  testWidgets('Local RadioTheme can override global RadioTheme', (WidgetTester tester) async {
    const Color globalThemeFillColor = Color(0xfffffff1);
    const Color localThemeFillColor = Color(0xffff0000);

    Widget buildRadio({required bool active}) {
      return MaterialApp(
        theme: ThemeData(
          radioTheme: const RadioThemeData(
            fillColor: MaterialStatePropertyAll<Color>(globalThemeFillColor),
          ),
        ),
        home: Scaffold(
          body: RadioTheme(
            data: const RadioThemeData(
              fillColor: MaterialStatePropertyAll<Color>(localThemeFillColor),
            ),
            child: Radio<int>(value: active ? 1 : 0, groupValue: 1, onChanged: (_) {}),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildRadio(active: true));
    await tester.pumpAndSettle();
    expect(_getRadioMaterial(tester), paints..circle(color: localThemeFillColor));
  });
}

Finder _findRadio() {
  return find.byWidgetPredicate((Widget widget) => widget is Radio<int>);
}

Future<void> _pointGestureToRadio(WidgetTester tester) async {
  final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
  await gesture.addPointer();
  addTearDown(gesture.removePointer);
  await gesture.moveTo(tester.getCenter(_findRadio()));
}

MaterialInkController? _getRadioMaterial(WidgetTester tester) {
  return Material.of(tester.element(_findRadio()));
}
