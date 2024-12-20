// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('CheckboxThemeData copyWith, ==, hashCode basics', () {
    expect(const CheckboxThemeData(), const CheckboxThemeData().copyWith());
    expect(const CheckboxThemeData().hashCode, const CheckboxThemeData().copyWith().hashCode);
  });

  test('CheckboxThemeData lerp special cases', () {
    expect(CheckboxThemeData.lerp(null, null, 0), const CheckboxThemeData());
    const CheckboxThemeData data = CheckboxThemeData();
    expect(identical(CheckboxThemeData.lerp(data, data, 0.5), data), true);
  });

  test('CheckboxThemeData defaults', () {
    const CheckboxThemeData themeData = CheckboxThemeData();
    expect(themeData.mouseCursor, null);
    expect(themeData.fillColor, null);
    expect(themeData.checkColor, null);
    expect(themeData.overlayColor, null);
    expect(themeData.splashRadius, null);
    expect(themeData.materialTapTargetSize, null);
    expect(themeData.visualDensity, null);

    const CheckboxTheme theme = CheckboxTheme(data: CheckboxThemeData(), child: SizedBox());
    expect(theme.data.mouseCursor, null);
    expect(theme.data.fillColor, null);
    expect(theme.data.checkColor, null);
    expect(theme.data.overlayColor, null);
    expect(theme.data.splashRadius, null);
    expect(theme.data.materialTapTargetSize, null);
    expect(theme.data.visualDensity, null);
  });

  testWidgets('Default CheckboxThemeData debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const CheckboxThemeData().debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[]);
  });

  testWidgets('CheckboxThemeData implements debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const CheckboxThemeData(
      mouseCursor: MaterialStatePropertyAll<MouseCursor?>(SystemMouseCursors.click),
      fillColor: MaterialStatePropertyAll<Color>(Color(0xfffffff0)),
      checkColor: MaterialStatePropertyAll<Color>(Color(0xfffffff1)),
      overlayColor: MaterialStatePropertyAll<Color>(Color(0xfffffff2)),
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
        'checkColor: WidgetStatePropertyAll(${const Color(0xfffffff1)})',
        'overlayColor: WidgetStatePropertyAll(${const Color(0xfffffff2)})',
        'splashRadius: 1.0',
        'materialTapTargetSize: MaterialTapTargetSize.shrinkWrap',
        'visualDensity: VisualDensity#00000(h: 0.0, v: 0.0)',
      ]),
    );
  });

  testWidgets('Checkbox is themeable', (WidgetTester tester) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;

    const MouseCursor mouseCursor = SystemMouseCursors.text;
    const Color defaultFillColor = Color(0xfffffff0);
    const Color selectedFillColor = Color(0xfffffff1);
    const Color defaultCheckColor = Color(0xfffffff2);
    const Color focusedCheckColor = Color(0xfffffff3);
    const Color focusOverlayColor = Color(0xfffffff4);
    const Color hoverOverlayColor = Color(0xfffffff5);
    const double splashRadius = 1.0;
    const MaterialTapTargetSize materialTapTargetSize = MaterialTapTargetSize.shrinkWrap;
    const VisualDensity visualDensity = VisualDensity(vertical: 1.0, horizontal: 1.0);

    Widget buildCheckbox({bool selected = false, bool autofocus = false}) {
      return MaterialApp(
        theme: ThemeData(
          checkboxTheme: CheckboxThemeData(
            mouseCursor: const MaterialStatePropertyAll<MouseCursor?>(mouseCursor),
            fillColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
              if (states.contains(MaterialState.selected)) {
                return selectedFillColor;
              }
              return defaultFillColor;
            }),
            checkColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
              if (states.contains(MaterialState.focused)) {
                return focusedCheckColor;
              }
              return defaultCheckColor;
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
          body: Checkbox(onChanged: (bool? value) {}, value: selected, autofocus: autofocus),
        ),
      );
    }

    // Checkbox.
    await tester.pumpWidget(buildCheckbox());
    await tester.pumpAndSettle();
    expect(_getCheckboxMaterial(tester), paints..path(color: defaultFillColor));
    // Size from MaterialTapTargetSize.shrinkWrap with added VisualDensity.
    expect(
      tester.getSize(find.byType(Checkbox)),
      const Size(40.0, 40.0) + visualDensity.baseSizeAdjustment,
    );

    // Selected checkbox.
    await tester.pumpWidget(buildCheckbox(selected: true));
    await tester.pumpAndSettle();
    expect(_getCheckboxMaterial(tester), paints..path(color: selectedFillColor));
    expect(
      _getCheckboxMaterial(tester),
      paints
        ..path(color: selectedFillColor)
        ..path(color: defaultCheckColor),
    );

    // Checkbox with hover.
    await tester.pumpWidget(buildCheckbox());
    await _pointGestureToCheckbox(tester);
    await tester.pumpAndSettle();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );
    expect(_getCheckboxMaterial(tester), paints..circle(color: hoverOverlayColor));

    // Checkbox with focus.
    await tester.pumpWidget(buildCheckbox(autofocus: true, selected: true));
    await tester.pumpAndSettle();
    expect(
      _getCheckboxMaterial(tester),
      paints..circle(color: focusOverlayColor, radius: splashRadius),
    );
    expect(
      _getCheckboxMaterial(tester),
      paints
        ..path(color: selectedFillColor)
        ..path(color: focusedCheckColor),
    );
  });

  testWidgets('Checkbox properties are taken over the theme values', (WidgetTester tester) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;

    const MouseCursor themeMouseCursor = SystemMouseCursors.click;
    const Color themeDefaultFillColor = Color(0xfffffff0);
    const Color themeSelectedFillColor = Color(0xfffffff1);
    const Color themeCheckColor = Color(0xfffffff2);
    const Color themeFocusOverlayColor = Color(0xfffffff3);
    const Color themeHoverOverlayColor = Color(0xfffffff4);
    const double themeSplashRadius = 1.0;
    const MaterialTapTargetSize themeMaterialTapTargetSize = MaterialTapTargetSize.padded;
    const VisualDensity themeVisualDensity = VisualDensity.standard;

    const MouseCursor mouseCursor = SystemMouseCursors.text;
    const Color defaultFillColor = Color(0xfffffff5);
    const Color selectedFillColor = Color(0xfffffff6);
    const Color checkColor = Color(0xfffffff7);
    const Color focusColor = Color(0xfffffff8);
    const Color hoverColor = Color(0xfffffff9);
    const double splashRadius = 2.0;
    const MaterialTapTargetSize materialTapTargetSize = MaterialTapTargetSize.shrinkWrap;
    const VisualDensity visualDensity = VisualDensity.standard;

    Widget buildCheckbox({bool selected = false, bool autofocus = false}) {
      return MaterialApp(
        theme: ThemeData(
          checkboxTheme: CheckboxThemeData(
            mouseCursor: const MaterialStatePropertyAll<MouseCursor?>(themeMouseCursor),
            fillColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
              if (states.contains(MaterialState.selected)) {
                return themeSelectedFillColor;
              }
              return themeDefaultFillColor;
            }),
            checkColor: const MaterialStatePropertyAll<Color?>(themeCheckColor),
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
          body: Checkbox(
            onChanged: (bool? value) {},
            value: selected,
            autofocus: autofocus,
            mouseCursor: mouseCursor,
            fillColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
              if (states.contains(MaterialState.selected)) {
                return selectedFillColor;
              }
              return defaultFillColor;
            }),
            checkColor: checkColor,
            focusColor: focusColor,
            hoverColor: hoverColor,
            splashRadius: splashRadius,
            materialTapTargetSize: materialTapTargetSize,
            visualDensity: visualDensity,
          ),
        ),
      );
    }

    // Checkbox.
    await tester.pumpWidget(buildCheckbox());
    await tester.pumpAndSettle();
    expect(_getCheckboxMaterial(tester), paints..path(color: defaultFillColor));
    // Size from MaterialTapTargetSize.shrinkWrap with added VisualDensity.
    expect(
      tester.getSize(find.byType(Checkbox)),
      const Size(40.0, 40.0) + visualDensity.baseSizeAdjustment,
    );

    // Selected checkbox.
    await tester.pumpWidget(buildCheckbox(selected: true));
    await tester.pumpAndSettle();
    expect(_getCheckboxMaterial(tester), paints..path(color: selectedFillColor));
    expect(
      _getCheckboxMaterial(tester),
      paints
        ..path(color: selectedFillColor)
        ..path(color: checkColor),
    );

    // Checkbox with hover.
    await tester.pumpWidget(buildCheckbox());
    await _pointGestureToCheckbox(tester);
    await tester.pumpAndSettle();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );
    expect(_getCheckboxMaterial(tester), paints..circle(color: hoverColor));

    // Checkbox with focus.
    await tester.pumpWidget(buildCheckbox(autofocus: true));
    await tester.pumpAndSettle();
    expect(_getCheckboxMaterial(tester), paints..circle(color: focusColor, radius: splashRadius));
  });

  testWidgets('Checkbox activeColor property is taken over the theme', (WidgetTester tester) async {
    const Color themeSelectedFillColor = Color(0xfffffff1);
    const Color themeDefaultFillColor = Color(0xfffffff0);
    const Color selectedFillColor = Color(0xfffffff6);

    Widget buildCheckbox({bool selected = false}) {
      return MaterialApp(
        theme: ThemeData(
          checkboxTheme: CheckboxThemeData(
            fillColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
              if (states.contains(MaterialState.selected)) {
                return themeSelectedFillColor;
              }
              return themeDefaultFillColor;
            }),
          ),
        ),
        home: Scaffold(
          body: Checkbox(
            onChanged: (bool? value) {},
            value: selected,
            activeColor: selectedFillColor,
          ),
        ),
      );
    }

    // Unselected checkbox.
    await tester.pumpWidget(buildCheckbox());
    await tester.pumpAndSettle();
    expect(_getCheckboxMaterial(tester), paints..path(color: themeDefaultFillColor));

    // Selected checkbox.
    await tester.pumpWidget(buildCheckbox(selected: true));
    await tester.pumpAndSettle();
    expect(_getCheckboxMaterial(tester), paints..path(color: selectedFillColor));
  });

  testWidgets('Checkbox theme overlay color resolves in active/pressed states', (
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

    Widget buildCheckbox({required bool active}) {
      return MaterialApp(
        theme: ThemeData(
          checkboxTheme: CheckboxThemeData(
            overlayColor: MaterialStateProperty.resolveWith(getOverlayColor),
            splashRadius: splashRadius,
          ),
        ),
        home: Scaffold(body: Checkbox(value: active, onChanged: (_) {})),
      );
    }

    await tester.pumpWidget(buildCheckbox(active: false));
    final TestGesture gesture1 = await tester.startGesture(tester.getCenter(find.byType(Checkbox)));
    await tester.pumpAndSettle();

    expect(
      _getCheckboxMaterial(tester),
      paints..circle(color: inactivePressedOverlayColor, radius: splashRadius),
      reason: 'Inactive pressed Checkbox should have overlay color: $inactivePressedOverlayColor',
    );

    await tester.pumpWidget(buildCheckbox(active: true));
    final TestGesture gesture2 = await tester.startGesture(tester.getCenter(find.byType(Checkbox)));
    await tester.pumpAndSettle();

    expect(
      _getCheckboxMaterial(tester),
      paints..circle(color: activePressedOverlayColor, radius: splashRadius),
      reason: 'Active pressed Checkbox should have overlay color: $activePressedOverlayColor',
    );

    // Finish gesture to release resources.
    await gesture1.up();
    await gesture2.up();
    await tester.pumpAndSettle();
  });

  testWidgets('Local CheckboxTheme can override global CheckboxTheme', (WidgetTester tester) async {
    const Color globalThemeFillColor = Color(0xfffffff1);
    const Color globalThemeCheckColor = Color(0xff000000);
    const Color localThemeFillColor = Color(0xffff0000);
    const Color localThemeCheckColor = Color(0xffffffff);

    Widget buildCheckbox({required bool active}) {
      return MaterialApp(
        theme: ThemeData(
          checkboxTheme: const CheckboxThemeData(
            checkColor: MaterialStatePropertyAll<Color>(globalThemeCheckColor),
            fillColor: MaterialStatePropertyAll<Color>(globalThemeFillColor),
          ),
        ),
        home: Scaffold(
          body: CheckboxTheme(
            data: const CheckboxThemeData(
              fillColor: MaterialStatePropertyAll<Color>(localThemeFillColor),
              checkColor: MaterialStatePropertyAll<Color>(localThemeCheckColor),
            ),
            child: Checkbox(value: active, onChanged: (_) {}),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildCheckbox(active: true));
    await tester.pumpAndSettle();
    expect(_getCheckboxMaterial(tester), paints..path(color: localThemeFillColor));
    expect(
      _getCheckboxMaterial(tester),
      paints
        ..path(color: localThemeFillColor)
        ..path(color: localThemeCheckColor),
    );
  });

  test('CheckboxThemeData lerp with null parameters', () {
    final CheckboxThemeData lerped = CheckboxThemeData.lerp(null, null, 0.25);

    expect(lerped.mouseCursor, null);
    expect(lerped.fillColor, null);
    expect(lerped.checkColor, null);
    expect(lerped.overlayColor, null);
    expect(lerped.splashRadius, null);
    expect(lerped.materialTapTargetSize, null);
    expect(lerped.visualDensity, null);
    expect(lerped.shape, null);
    expect(lerped.side, null);
  });

  test('CheckboxThemeData lerp from populated to null parameters', () {
    final CheckboxThemeData theme = CheckboxThemeData(
      fillColor: MaterialStateProperty.all(const Color(0xfffffff0)),
      checkColor: MaterialStateProperty.all(const Color(0xfffffff1)),
      overlayColor: MaterialStateProperty.all(const Color(0xfffffff2)),
      splashRadius: 3.0,
      materialTapTargetSize: MaterialTapTargetSize.shrinkWrap,
      visualDensity: const VisualDensity(vertical: 1.0, horizontal: 1.0),
      shape: const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(4.0))),
      side: const BorderSide(width: 4.0),
    );
    final CheckboxThemeData lerped = CheckboxThemeData.lerp(theme, null, 0.5);

    expect(lerped.fillColor!.resolve(<MaterialState>{}), isSameColorAs(const Color(0x80fffff0)));
    expect(lerped.checkColor!.resolve(<MaterialState>{}), isSameColorAs(const Color(0x80fffff1)));
    expect(lerped.overlayColor!.resolve(<MaterialState>{}), isSameColorAs(const Color(0x80fffff2)));
    expect(lerped.splashRadius, 1.5);
    expect(lerped.materialTapTargetSize, null);
    expect(lerped.visualDensity, null);
    expect(
      lerped.shape,
      const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(2.0))),
    );
    // Returns null if either lerp value is null.
    expect(lerped.side, null);
  });

  test('CheckboxThemeData lerp from populated parameters', () {
    final CheckboxThemeData themeA = CheckboxThemeData(
      fillColor: MaterialStateProperty.all(const Color(0xfffffff0)),
      checkColor: MaterialStateProperty.all(const Color(0xfffffff1)),
      overlayColor: MaterialStateProperty.all(const Color(0xfffffff2)),
      splashRadius: 3.0,
      materialTapTargetSize: MaterialTapTargetSize.shrinkWrap,
      visualDensity: const VisualDensity(vertical: 1.0, horizontal: 1.0),
      shape: const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(4.0))),
      side: const BorderSide(width: 4.0),
    );
    final CheckboxThemeData themeB = CheckboxThemeData(
      fillColor: MaterialStateProperty.all(const Color(0xfffffff3)),
      checkColor: MaterialStateProperty.all(const Color(0xfffffff4)),
      overlayColor: MaterialStateProperty.all(const Color(0xfffffff5)),
      splashRadius: 9.0,
      materialTapTargetSize: MaterialTapTargetSize.shrinkWrap,
      visualDensity: const VisualDensity(vertical: 2.0, horizontal: 2.0),
      shape: const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(1.0))),
      side: const BorderSide(width: 3.0),
    );
    final CheckboxThemeData lerped = CheckboxThemeData.lerp(themeA, themeB, 0.5);

    expect(lerped.fillColor!.resolve(<MaterialState>{}), isSameColorAs(const Color(0xfffffff1)));
    expect(lerped.checkColor!.resolve(<MaterialState>{}), isSameColorAs(const Color(0xfffffff2)));
    expect(lerped.overlayColor!.resolve(<MaterialState>{}), isSameColorAs(const Color(0xfffffff3)));
    expect(lerped.splashRadius, 6);
    expect(lerped.materialTapTargetSize, MaterialTapTargetSize.shrinkWrap);
    expect(lerped.visualDensity, const VisualDensity(vertical: 2.0, horizontal: 2.0));
    expect(
      lerped.shape,
      const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(2.5))),
    );
    expect(lerped.side, const BorderSide(width: 3.5));
  });

  testWidgets('MaterialStateBorderSide properly lerp in CheckboxThemeData.side', (
    WidgetTester tester,
  ) async {
    late ColorScheme colorScheme;

    Widget buildCheckbox({required Color seedColor}) {
      colorScheme = ColorScheme.fromSeed(seedColor: seedColor);
      return MaterialApp(
        theme: ThemeData(
          colorScheme: colorScheme,
          checkboxTheme: CheckboxThemeData(
            side: MaterialStateBorderSide.resolveWith((Set<MaterialState> states) {
              return BorderSide(color: colorScheme.primary, width: 4.0);
            }),
          ),
        ),
        home: Scaffold(body: Checkbox(value: false, onChanged: (_) {})),
      );
    }

    await tester.pumpWidget(buildCheckbox(seedColor: Colors.red));
    await tester.pumpAndSettle();

    RenderBox getCheckboxRenderBox() {
      return tester.renderObject<RenderBox>(find.byType(Checkbox));
    }

    expect(getCheckboxRenderBox(), paints..drrect(color: colorScheme.primary));

    await tester.pumpWidget(buildCheckbox(seedColor: Colors.blue));
    await tester.pump(kPressTimeout);

    expect(getCheckboxRenderBox(), paints..drrect(color: colorScheme.primary));
  });
}

Future<void> _pointGestureToCheckbox(WidgetTester tester) async {
  final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
  await gesture.addPointer();
  addTearDown(gesture.removePointer);
  await gesture.moveTo(tester.getCenter(find.byType(Checkbox)));
}

MaterialInkController? _getCheckboxMaterial(WidgetTester tester) {
  return Material.of(tester.element(find.byType(Checkbox)));
}
