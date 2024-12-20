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

import '../widgets/feedback_tester.dart';

/// Adds the basic requirements for a Chip.
Widget wrapForChip({
  required Widget child,
  TextDirection textDirection = TextDirection.ltr,
  TextScaler textScaler = TextScaler.noScaling,
  ThemeData? theme,
}) {
  return MaterialApp(
    theme: theme,
    home: Directionality(
      textDirection: textDirection,
      child: MediaQuery(
        data: MediaQueryData(textScaler: textScaler),
        child: Material(child: child),
      ),
    ),
  );
}

Future<void> pumpCheckmarkChip(
  WidgetTester tester, {
  required Widget chip,
  Color? themeColor,
  ThemeData? theme,
}) async {
  await tester.pumpWidget(
    wrapForChip(
      theme: theme,
      child: Builder(
        builder: (BuildContext context) {
          final ChipThemeData chipTheme = ChipTheme.of(context);
          return ChipTheme(
            data: themeColor == null ? chipTheme : chipTheme.copyWith(checkmarkColor: themeColor),
            child: chip,
          );
        },
      ),
    ),
  );
}

Widget selectedFilterChip({Color? checkmarkColor}) {
  return FilterChip(
    label: const Text('InputChip'),
    selected: true,
    showCheckmark: true,
    checkmarkColor: checkmarkColor,
    onSelected: (bool _) {},
  );
}

void expectCheckmarkColor(Finder finder, Color color) {
  expect(
    finder,
    paints
      // Physical model path
      ..path()
      // The first layer that is painted is the selection overlay. We do not care
      // how it is painted but it has to be added it to this pattern so that the
      // check mark can be checked next.
      ..rrect()
      // The second layer that is painted is the check mark.
      ..path(color: color),
  );
}

RenderBox getMaterialBox(WidgetTester tester, Finder type) {
  return tester.firstRenderObject<RenderBox>(
    find.descendant(of: type, matching: find.byType(CustomPaint)),
  );
}

void checkChipMaterialClipBehavior(WidgetTester tester, Clip clipBehavior) {
  final Iterable<Material> materials = tester.widgetList<Material>(find.byType(Material));
  // There should be two Material widgets, first Material is from the "_wrapForChip" and
  // last Material is from the "RawChip".
  expect(materials.length, 2);
  // The last Material from `RawChip` should have the clip behavior.
  expect(materials.last.clipBehavior, clipBehavior);
}

Material getMaterial(WidgetTester tester) {
  return tester.widget<Material>(
    find.descendant(of: find.byType(FilterChip), matching: find.byType(Material)),
  );
}

IconThemeData getIconData(WidgetTester tester) {
  final IconTheme iconTheme = tester.firstWidget(
    find.descendant(of: find.byType(RawChip), matching: find.byType(IconTheme)),
  );
  return iconTheme.data;
}

DefaultTextStyle getLabelStyle(WidgetTester tester, String labelText) {
  return tester.widget(
    find.ancestor(of: find.text(labelText), matching: find.byType(DefaultTextStyle)).first,
  );
}

// Finds any container of a tooltip.
Finder findTooltipContainer(String tooltipText) {
  return find.ancestor(of: find.text(tooltipText), matching: find.byType(Container));
}

void main() {
  testWidgets('Material2 - FilterChip defaults', (WidgetTester tester) async {
    final ThemeData theme = ThemeData(useMaterial3: false);
    const String label = 'filter chip';

    // Test enabled FilterChip defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: FilterChip(onSelected: (bool valueChanged) {}, label: const Text(label)),
          ),
        ),
      ),
    );

    // Test default chip size.
    expect(tester.getSize(find.byType(FilterChip)), const Size(178.0, 48.0));

    // Test default label style.
    expect(
      getLabelStyle(tester, label).style.color,
      theme.textTheme.bodyLarge!.color!.withAlpha(0xde),
    );

    Material chipMaterial = getMaterial(tester);
    expect(chipMaterial.elevation, 0);
    expect(chipMaterial.shadowColor, Colors.black);
    expect(chipMaterial.shape, const StadiumBorder());

    ShapeDecoration decoration =
        tester.widget<Ink>(find.byType(Ink)).decoration! as ShapeDecoration;
    expect(decoration.color, Colors.black.withAlpha(0x1f));

    // Test disabled FilterChip defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: const Material(child: FilterChip(onSelected: null, label: Text(label))),
      ),
    );
    await tester.pumpAndSettle();

    chipMaterial = getMaterial(tester);
    expect(chipMaterial.elevation, 0);
    expect(chipMaterial.shadowColor, Colors.black);
    expect(chipMaterial.shape, const StadiumBorder());

    decoration = tester.widget<Ink>(find.byType(Ink)).decoration! as ShapeDecoration;
    expect(decoration.color, Colors.black38);

    // Test selected enabled FilterChip defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Material(
          child: FilterChip(
            selected: true,
            onSelected: (bool valueChanged) {},
            label: const Text(label),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();

    chipMaterial = getMaterial(tester);
    expect(chipMaterial.elevation, 0);
    expect(chipMaterial.shadowColor, Colors.black);
    expect(chipMaterial.shape, const StadiumBorder());

    decoration = tester.widget<Ink>(find.byType(Ink)).decoration! as ShapeDecoration;
    expect(decoration.color, Colors.black.withAlpha(0x3d));

    // Test selected disabled FilterChip defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: const Material(
          child: FilterChip(selected: true, onSelected: null, label: Text(label)),
        ),
      ),
    );
    await tester.pumpAndSettle();

    chipMaterial = getMaterial(tester);
    expect(chipMaterial.elevation, 0);
    expect(chipMaterial.shadowColor, Colors.black);
    expect(chipMaterial.shape, const StadiumBorder());

    decoration = tester.widget<Ink>(find.byType(Ink)).decoration! as ShapeDecoration;
    expect(decoration.color, Colors.black.withAlpha(0x3d));
  });

  testWidgets('Material3 - FilterChip defaults', (WidgetTester tester) async {
    final ThemeData theme = ThemeData();
    const String label = 'filter chip';

    // Test enabled FilterChip defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: FilterChip(onSelected: (bool valueChanged) {}, label: const Text(label)),
          ),
        ),
      ),
    );

    // Test default chip size.
    expect(
      tester.getSize(find.byType(FilterChip)),
      within(distance: 0.001, from: const Size(189.1, 48.0)),
    );
    // Test default label style.
    expect(
      getLabelStyle(tester, label).style.color!.value,
      theme.colorScheme.onSurfaceVariant.value,
    );

    Material chipMaterial = getMaterial(tester);
    expect(chipMaterial.elevation, 0);
    expect(chipMaterial.shadowColor, Colors.transparent);
    expect(chipMaterial.surfaceTintColor, Colors.transparent);
    expect(
      chipMaterial.shape,
      RoundedRectangleBorder(
        borderRadius: const BorderRadius.all(Radius.circular(8.0)),
        side: BorderSide(color: theme.colorScheme.outlineVariant),
      ),
    );

    ShapeDecoration decoration =
        tester.widget<Ink>(find.byType(Ink)).decoration! as ShapeDecoration;
    expect(decoration.color, null);

    // Test disabled FilterChip defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: const Material(child: FilterChip(onSelected: null, label: Text(label))),
      ),
    );
    await tester.pumpAndSettle();

    chipMaterial = getMaterial(tester);
    expect(chipMaterial.elevation, 0);
    expect(chipMaterial.shadowColor, Colors.transparent);
    expect(chipMaterial.surfaceTintColor, Colors.transparent);
    expect(
      chipMaterial.shape,
      RoundedRectangleBorder(
        borderRadius: const BorderRadius.all(Radius.circular(8.0)),
        side: BorderSide(color: theme.colorScheme.onSurface.withOpacity(0.12)),
      ),
    );

    decoration = tester.widget<Ink>(find.byType(Ink)).decoration! as ShapeDecoration;
    expect(decoration.color, null);

    // Test selected enabled FilterChip defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Material(
          child: FilterChip(
            selected: true,
            onSelected: (bool valueChanged) {},
            label: const Text(label),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();

    chipMaterial = getMaterial(tester);
    expect(chipMaterial.elevation, 0);
    expect(chipMaterial.shadowColor, null);
    expect(chipMaterial.surfaceTintColor, Colors.transparent);
    expect(
      chipMaterial.shape,
      const RoundedRectangleBorder(
        borderRadius: BorderRadius.all(Radius.circular(8.0)),
        side: BorderSide(color: Colors.transparent),
      ),
    );

    decoration = tester.widget<Ink>(find.byType(Ink)).decoration! as ShapeDecoration;
    expect(decoration.color, theme.colorScheme.secondaryContainer);

    // Test selected disabled FilterChip defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: const Material(
          child: FilterChip(selected: true, onSelected: null, label: Text(label)),
        ),
      ),
    );
    await tester.pumpAndSettle();

    chipMaterial = getMaterial(tester);
    expect(chipMaterial.elevation, 0);
    expect(chipMaterial.shadowColor, null);
    expect(chipMaterial.surfaceTintColor, Colors.transparent);
    expect(
      chipMaterial.shape,
      const RoundedRectangleBorder(
        borderRadius: BorderRadius.all(Radius.circular(8.0)),
        side: BorderSide(color: Colors.transparent),
      ),
    );

    decoration = tester.widget<Ink>(find.byType(Ink)).decoration! as ShapeDecoration;
    expect(decoration.color, theme.colorScheme.onSurface.withOpacity(0.12));
  });

  testWidgets('Material3 - FilterChip.elevated defaults', (WidgetTester tester) async {
    final ThemeData theme = ThemeData();
    const String label = 'filter chip';

    // Test enabled FilterChip.elevated defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: FilterChip.elevated(
              onSelected: (bool valueChanged) {},
              label: const Text(label),
            ),
          ),
        ),
      ),
    );

    // Test default chip size.
    expect(
      tester.getSize(find.byType(FilterChip)),
      within(distance: 0.001, from: const Size(189.1, 48.0)),
    );
    // Test default label style.
    expect(
      getLabelStyle(tester, 'filter chip').style.color!.value,
      theme.colorScheme.onSurfaceVariant.value,
    );

    Material chipMaterial = getMaterial(tester);
    expect(chipMaterial.elevation, 1);
    expect(chipMaterial.shadowColor, theme.colorScheme.shadow);
    expect(chipMaterial.surfaceTintColor, Colors.transparent);
    expect(
      chipMaterial.shape,
      const RoundedRectangleBorder(
        borderRadius: BorderRadius.all(Radius.circular(8.0)),
        side: BorderSide(color: Colors.transparent),
      ),
    );

    ShapeDecoration decoration =
        tester.widget<Ink>(find.byType(Ink)).decoration! as ShapeDecoration;
    expect(decoration.color, theme.colorScheme.surfaceContainerLow);

    // Test disabled FilterChip.elevated defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: const Material(child: FilterChip.elevated(onSelected: null, label: Text(label))),
      ),
    );
    await tester.pumpAndSettle();

    chipMaterial = getMaterial(tester);
    expect(chipMaterial.elevation, 0);
    expect(chipMaterial.shadowColor, theme.colorScheme.shadow);
    expect(chipMaterial.surfaceTintColor, Colors.transparent);
    expect(
      chipMaterial.shape,
      const RoundedRectangleBorder(
        borderRadius: BorderRadius.all(Radius.circular(8.0)),
        side: BorderSide(color: Colors.transparent),
      ),
    );

    decoration = tester.widget<Ink>(find.byType(Ink)).decoration! as ShapeDecoration;
    expect(decoration.color, theme.colorScheme.onSurface.withOpacity(0.12));

    // Test selected enabled FilterChip.elevated defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Material(
          child: FilterChip.elevated(
            selected: true,
            onSelected: (bool valueChanged) {},
            label: const Text(label),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();

    chipMaterial = getMaterial(tester);
    expect(chipMaterial.elevation, 1);
    expect(chipMaterial.shadowColor, null);
    expect(chipMaterial.surfaceTintColor, Colors.transparent);
    expect(
      chipMaterial.shape,
      const RoundedRectangleBorder(
        borderRadius: BorderRadius.all(Radius.circular(8.0)),
        side: BorderSide(color: Colors.transparent),
      ),
    );

    decoration = tester.widget<Ink>(find.byType(Ink)).decoration! as ShapeDecoration;
    expect(decoration.color, theme.colorScheme.secondaryContainer);

    // Test selected disabled FilterChip.elevated defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: const Material(
          child: FilterChip.elevated(selected: true, onSelected: null, label: Text(label)),
        ),
      ),
    );
    await tester.pumpAndSettle();

    chipMaterial = getMaterial(tester);
    expect(chipMaterial.elevation, 0);
    expect(chipMaterial.shadowColor, null);
    expect(chipMaterial.surfaceTintColor, Colors.transparent);
    expect(
      chipMaterial.shape,
      const RoundedRectangleBorder(
        borderRadius: BorderRadius.all(Radius.circular(8.0)),
        side: BorderSide(color: Colors.transparent),
      ),
    );

    decoration = tester.widget<Ink>(find.byType(Ink)).decoration! as ShapeDecoration;
    expect(decoration.color, theme.colorScheme.onSurface.withOpacity(0.12));
  });

  testWidgets('FilterChip.color resolves material states', (WidgetTester tester) async {
    const Color disabledSelectedColor = Color(0xffffff00);
    const Color disabledColor = Color(0xff00ff00);
    const Color backgroundColor = Color(0xff0000ff);
    const Color selectedColor = Color(0xffff0000);
    final MaterialStateProperty<Color?> color = MaterialStateProperty.resolveWith((
      Set<MaterialState> states,
    ) {
      if (states.contains(MaterialState.disabled) && states.contains(MaterialState.selected)) {
        return disabledSelectedColor;
      }
      if (states.contains(MaterialState.disabled)) {
        return disabledColor;
      }
      if (states.contains(MaterialState.selected)) {
        return selectedColor;
      }
      return backgroundColor;
    });
    Widget buildApp({required bool enabled, required bool selected}) {
      return wrapForChip(
        child: Column(
          children: <Widget>[
            FilterChip(
              onSelected: enabled ? (bool value) {} : null,
              selected: selected,
              color: color,
              label: const Text('FilterChip'),
            ),
            FilterChip.elevated(
              onSelected: enabled ? (bool value) {} : null,
              selected: selected,
              color: color,
              label: const Text('FilterChip.elevated'),
            ),
          ],
        ),
      );
    }

    // Test enabled state.
    await tester.pumpWidget(buildApp(enabled: true, selected: false));

    // Enabled FilterChip should have the provided backgroundColor.
    expect(
      getMaterialBox(tester, find.byType(RawChip).first),
      paints..rrect(color: backgroundColor),
    );
    // Enabled elevated FilterChip should have the provided backgroundColor.
    expect(
      getMaterialBox(tester, find.byType(RawChip).last),
      paints..rrect(color: backgroundColor),
    );

    // Test disabled state.
    await tester.pumpWidget(buildApp(enabled: false, selected: false));
    await tester.pumpAndSettle();

    // Disabled FilterChip should have the provided disabledColor.
    expect(getMaterialBox(tester, find.byType(RawChip).first), paints..rrect(color: disabledColor));
    // Disabled elevated FilterChip should have the provided disabledColor.
    expect(getMaterialBox(tester, find.byType(RawChip).last), paints..rrect(color: disabledColor));

    // Test enabled & selected state.
    await tester.pumpWidget(buildApp(enabled: true, selected: true));
    await tester.pumpAndSettle();

    // Enabled & selected FilterChip should have the provided selectedColor.
    expect(getMaterialBox(tester, find.byType(RawChip).first), paints..rrect(color: selectedColor));
    // Enabled & selected elevated FilterChip should have the provided selectedColor.
    expect(getMaterialBox(tester, find.byType(RawChip).last), paints..rrect(color: selectedColor));

    // Test disabled & selected state.
    await tester.pumpWidget(buildApp(enabled: false, selected: true));
    await tester.pumpAndSettle();

    // Disabled & selected FilterChip should have the provided disabledSelectedColor.
    expect(
      getMaterialBox(tester, find.byType(RawChip).first),
      paints..rrect(color: disabledSelectedColor),
    );
    // Disabled & selected elevated FilterChip should have the
    // provided disabledSelectedColor.
    expect(
      getMaterialBox(tester, find.byType(RawChip).last),
      paints..rrect(color: disabledSelectedColor),
    );
  });

  testWidgets('FilterChip uses provided state color properties', (WidgetTester tester) async {
    const Color disabledColor = Color(0xff00ff00);
    const Color backgroundColor = Color(0xff0000ff);
    const Color selectedColor = Color(0xffff0000);
    Widget buildApp({required bool enabled, required bool selected}) {
      return wrapForChip(
        child: Column(
          children: <Widget>[
            FilterChip(
              onSelected: enabled ? (bool value) {} : null,
              selected: selected,
              disabledColor: disabledColor,
              backgroundColor: backgroundColor,
              selectedColor: selectedColor,
              label: const Text('FilterChip'),
            ),
            FilterChip.elevated(
              onSelected: enabled ? (bool value) {} : null,
              selected: selected,
              disabledColor: disabledColor,
              backgroundColor: backgroundColor,
              selectedColor: selectedColor,
              label: const Text('FilterChip.elevated'),
            ),
          ],
        ),
      );
    }

    // Test enabled state.
    await tester.pumpWidget(buildApp(enabled: true, selected: false));

    // Enabled FilterChip should have the provided backgroundColor.
    expect(
      getMaterialBox(tester, find.byType(RawChip).first),
      paints..rrect(color: backgroundColor),
    );
    // Enabled elevated FilterChip should have the provided backgroundColor.
    expect(
      getMaterialBox(tester, find.byType(RawChip).last),
      paints..rrect(color: backgroundColor),
    );

    // Test disabled state.
    await tester.pumpWidget(buildApp(enabled: false, selected: false));
    await tester.pumpAndSettle();

    // Disabled FilterChip should have the provided disabledColor.
    expect(getMaterialBox(tester, find.byType(RawChip).first), paints..rrect(color: disabledColor));
    // Disabled elevated FilterChip should have the provided disabledColor.
    expect(getMaterialBox(tester, find.byType(RawChip).last), paints..rrect(color: disabledColor));

    // Test enabled & selected state.
    await tester.pumpWidget(buildApp(enabled: true, selected: true));
    await tester.pumpAndSettle();

    // Enabled & selected FilterChip should have the provided selectedColor.
    expect(getMaterialBox(tester, find.byType(RawChip).first), paints..rrect(color: selectedColor));
    // Enabled & selected elevated FilterChip should have the provided selectedColor.
    expect(getMaterialBox(tester, find.byType(RawChip).last), paints..rrect(color: selectedColor));
  });

  testWidgets('FilterChip can be tapped', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: FilterChip(onSelected: (bool valueChanged) {}, label: const Text('filter chip')),
        ),
      ),
    );

    await tester.tap(find.byType(FilterChip));
    expect(tester.takeException(), null);
  });

  testWidgets(
    'Material2 - Filter chip check mark color is determined by platform brightness when light',
    (WidgetTester tester) async {
      await pumpCheckmarkChip(
        tester,
        chip: selectedFilterChip(),
        theme: ThemeData(useMaterial3: false),
      );

      expectCheckmarkColor(find.byType(FilterChip), Colors.black.withAlpha(0xde));
    },
  );

  testWidgets(
    'Material3 - Filter chip check mark color is determined by platform brightness when light',
    (WidgetTester tester) async {
      final ThemeData theme = ThemeData();
      await pumpCheckmarkChip(tester, chip: selectedFilterChip(), theme: theme);

      expectCheckmarkColor(find.byType(FilterChip), theme.colorScheme.onSecondaryContainer);
    },
  );

  testWidgets(
    'Material2 - Filter chip check mark color is determined by platform brightness when dark',
    (WidgetTester tester) async {
      await pumpCheckmarkChip(
        tester,
        chip: selectedFilterChip(),
        theme: ThemeData.dark(useMaterial3: false),
      );

      expectCheckmarkColor(find.byType(FilterChip), Colors.white.withAlpha(0xde));
    },
  );

  testWidgets(
    'Material3 - Filter chip check mark color is determined by platform brightness when dark',
    (WidgetTester tester) async {
      final ThemeData theme = ThemeData(brightness: Brightness.dark);
      await pumpCheckmarkChip(tester, chip: selectedFilterChip(), theme: theme);

      expectCheckmarkColor(find.byType(FilterChip), theme.colorScheme.onSecondaryContainer);
    },
  );

  testWidgets('Filter chip check mark color can be set by the chip theme', (
    WidgetTester tester,
  ) async {
    await pumpCheckmarkChip(
      tester,
      chip: selectedFilterChip(),
      themeColor: const Color(0xff00ff00),
    );

    expectCheckmarkColor(find.byType(FilterChip), const Color(0xff00ff00));
  });

  testWidgets('Filter chip check mark color can be set by the chip constructor', (
    WidgetTester tester,
  ) async {
    await pumpCheckmarkChip(
      tester,
      chip: selectedFilterChip(checkmarkColor: const Color(0xff00ff00)),
    );

    expectCheckmarkColor(find.byType(FilterChip), const Color(0xff00ff00));
  });

  testWidgets(
    'Filter chip check mark color is set by chip constructor even when a theme color is specified',
    (WidgetTester tester) async {
      await pumpCheckmarkChip(
        tester,
        chip: selectedFilterChip(checkmarkColor: const Color(0xffff0000)),
        themeColor: const Color(0xff00ff00),
      );

      expectCheckmarkColor(find.byType(FilterChip), const Color(0xffff0000));
    },
  );

  testWidgets('FilterChip clipBehavior properly passes through to the Material', (
    WidgetTester tester,
  ) async {
    const Text label = Text('label');
    await tester.pumpWidget(wrapForChip(child: FilterChip(label: label, onSelected: (bool b) {})));
    checkChipMaterialClipBehavior(tester, Clip.none);

    await tester.pumpWidget(
      wrapForChip(
        child: FilterChip(label: label, onSelected: (bool b) {}, clipBehavior: Clip.antiAlias),
      ),
    );
    checkChipMaterialClipBehavior(tester, Clip.antiAlias);
  });

  testWidgets('Material3 - width should not change with selection', (WidgetTester tester) async {
    // Regression tests for: https://github.com/flutter/flutter/issues/110645

    // For the text "FilterChip" the chip should default to 175 regardless of selection.
    const int expectedWidth = 175;

    // Unselected
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: FilterChip(
              label: const Text('FilterChip'),
              showCheckmark: false,
              onSelected: (bool _) {},
            ),
          ),
        ),
      ),
    );
    expect(tester.getSize(find.byType(FilterChip)).width, expectedWidth);

    // Selected
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: FilterChip(
              label: const Text('FilterChip'),
              showCheckmark: false,
              selected: true,
              onSelected: (bool _) {},
            ),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    expect(tester.getSize(find.byType(FilterChip)).width, expectedWidth);
  });

  testWidgets('FilterChip uses provided iconTheme', (WidgetTester tester) async {
    final ThemeData theme = ThemeData();

    Widget buildChip({IconThemeData? iconTheme}) {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: FilterChip(
            iconTheme: iconTheme,
            avatar: const Icon(Icons.add),
            label: const Text('FilterChip'),
            onSelected: (bool _) {},
          ),
        ),
      );
    }

    // Test default icon theme.
    await tester.pumpWidget(buildChip());

    expect(getIconData(tester).color, theme.colorScheme.primary);

    // Test provided icon theme.
    await tester.pumpWidget(buildChip(iconTheme: const IconThemeData(color: Color(0xff00ff00))));

    expect(getIconData(tester).color, const Color(0xff00ff00));
  });

  testWidgets('Material3 - FilterChip supports delete button', (WidgetTester tester) async {
    final ThemeData theme = ThemeData();
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: FilterChip(
              onDeleted: () {},
              onSelected: (bool valueChanged) {},
              label: const Text('FilterChip'),
            ),
          ),
        ),
      ),
    );

    // Test the chip size with delete button.
    expect(find.text('FilterChip'), findsOneWidget);
    expect(tester.getSize(find.byType(FilterChip)), const Size(195.0, 48.0));

    // Test the delete button icon.
    expect(tester.getSize(find.byIcon(Icons.clear)), const Size(18.0, 18.0));
    expect(getIconData(tester).color, theme.colorScheme.onSurfaceVariant);

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: FilterChip.elevated(
              onDeleted: () {},
              onSelected: (bool valueChanged) {},
              label: const Text('Elevated FilterChip'),
            ),
          ),
        ),
      ),
    );

    // Test the elevated chip size with delete button.
    expect(find.text('Elevated FilterChip'), findsOneWidget);
    expect(
      tester.getSize(find.byType(FilterChip)),
      within(distance: 0.001, from: const Size(321.9, 48.0)),
    );

    // Test the delete button icon.
    expect(tester.getSize(find.byIcon(Icons.clear)), const Size(18.0, 18.0));
    expect(getIconData(tester).color, theme.colorScheme.onSurfaceVariant);
  }, skip: kIsWeb && !isSkiaWeb); // https://github.com/flutter/flutter/issues/99933

  testWidgets('Material2 - FilterChip supports delete button', (WidgetTester tester) async {
    final ThemeData theme = ThemeData(useMaterial3: false);
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: FilterChip(
              onDeleted: () {},
              onSelected: (bool valueChanged) {},
              label: const Text('FilterChip'),
            ),
          ),
        ),
      ),
    );

    // Test the chip size with delete button.
    expect(find.text('FilterChip'), findsOneWidget);
    expect(tester.getSize(find.byType(FilterChip)), const Size(188.0, 48.0));

    // Test the delete button icon.
    expect(tester.getSize(find.byIcon(Icons.cancel)), const Size(18.0, 18.0));
    expect(getIconData(tester).color, theme.iconTheme.color?.withAlpha(0xde));

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: FilterChip.elevated(
              onDeleted: () {},
              onSelected: (bool valueChanged) {},
              label: const Text('Elevated FilterChip'),
            ),
          ),
        ),
      ),
    );

    // Test the elevated chip size with delete button.
    expect(find.text('Elevated FilterChip'), findsOneWidget);
    expect(tester.getSize(find.byType(FilterChip)), const Size(314.0, 48.0));

    // Test the delete button icon.
    expect(tester.getSize(find.byIcon(Icons.cancel)), const Size(18.0, 18.0));
    expect(getIconData(tester).color, theme.iconTheme.color?.withAlpha(0xde));
  });

  testWidgets('Customize FilterChip delete button', (WidgetTester tester) async {
    Widget buildChip({
      Widget? deleteIcon,
      Color? deleteIconColor,
      String? deleteButtonTooltipMessage,
    }) {
      return MaterialApp(
        home: Material(
          child: Center(
            child: FilterChip(
              deleteIcon: deleteIcon,
              deleteIconColor: deleteIconColor,
              deleteButtonTooltipMessage: deleteButtonTooltipMessage,
              onDeleted: () {},
              onSelected: (bool valueChanged) {},
              label: const Text('FilterChip'),
            ),
          ),
        ),
      );
    }

    // Test the custom delete icon.
    await tester.pumpWidget(buildChip(deleteIcon: const Icon(Icons.delete)));

    expect(find.byIcon(Icons.clear), findsNothing);
    expect(find.byIcon(Icons.delete), findsOneWidget);

    // Test the custom delete icon color.
    await tester.pumpWidget(
      buildChip(deleteIcon: const Icon(Icons.delete), deleteIconColor: const Color(0xff00ff00)),
    );
    await tester.pumpAndSettle();

    expect(find.byIcon(Icons.clear), findsNothing);
    expect(find.byIcon(Icons.delete), findsOneWidget);
    expect(getIconData(tester).color, const Color(0xff00ff00));

    // Test the custom delete button tooltip message.
    await tester.pumpWidget(buildChip(deleteButtonTooltipMessage: 'Delete FilterChip'));
    await tester.pumpAndSettle();

    // Hover over the delete icon of the chip
    final TestGesture gesture = await tester.startGesture(
      tester.getCenter(find.byIcon(Icons.clear)),
    );

    await tester.pumpAndSettle();

    // Verify the tooltip message is set.
    expect(find.widgetWithText(Tooltip, 'Delete FilterChip'), findsOneWidget);

    await gesture.up();
  });

  testWidgets('FilterChip delete button control test', (WidgetTester tester) async {
    final FeedbackTester feedback = FeedbackTester();
    final List<String> deletedButtonStrings = <String>[];
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: FilterChip(
              onDeleted: () {
                deletedButtonStrings.add('A');
              },
              onSelected: (bool valueChanged) {},
              label: const Text('FilterChip'),
            ),
          ),
        ),
      ),
    );

    expect(feedback.clickSoundCount, 0);

    expect(deletedButtonStrings, isEmpty);
    await tester.tap(find.byIcon(Icons.clear));
    expect(deletedButtonStrings, equals(<String>['A']));

    await tester.pumpAndSettle(const Duration(seconds: 1));
    expect(feedback.clickSoundCount, 1);

    await tester.tap(find.byIcon(Icons.clear));
    expect(deletedButtonStrings, equals(<String>['A', 'A']));

    await tester.pumpAndSettle(const Duration(seconds: 1));
    expect(feedback.clickSoundCount, 2);

    feedback.dispose();
  });

  testWidgets('Delete button is visible on disabled FilterChip', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrapForChip(
        child: FilterChip(label: const Text('Label'), onSelected: null, onDeleted: () {}),
      ),
    );

    // Delete button should be visible.
    await expectLater(
      find.byType(RawChip),
      matchesGoldenFile('filter_chip.disabled.delete_button.png'),
    );
  });

  testWidgets('Delete button tooltip is not shown on disabled FilterChip', (
    WidgetTester tester,
  ) async {
    Widget buildChip({bool enabled = true}) {
      return wrapForChip(
        child: FilterChip(
          onSelected: enabled ? (bool value) {} : null,
          label: const Text('Label'),
          onDeleted: () {},
        ),
      );
    }

    // Test enabled chip.
    await tester.pumpWidget(buildChip());

    final Offset deleteButtonLocation = tester.getCenter(find.byType(Icon));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.moveTo(deleteButtonLocation);
    await tester.pump();

    // Delete button tooltip should be visible.
    expect(findTooltipContainer('Delete'), findsOneWidget);

    // Test disabled chip.
    await tester.pumpWidget(buildChip(enabled: false));
    await tester.pump();

    // Delete button tooltip should not be visible.
    expect(findTooltipContainer('Delete'), findsNothing);
  });

  testWidgets('Material3 - Default FilterChip icon colors', (WidgetTester tester) async {
    const IconData flatAvatar = Icons.favorite;
    const IconData flatDeleteIcon = Icons.delete;
    const IconData elevatedAvatar = Icons.house;
    const IconData elevatedDeleteIcon = Icons.clear_all;
    final ThemeData theme = ThemeData();

    Widget buildChips({required bool selected}) {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Column(
            children: <Widget>[
              FilterChip(
                avatar: const Icon(flatAvatar),
                deleteIcon: const Icon(flatDeleteIcon),
                onSelected: (bool valueChanged) {},
                label: const Text('FilterChip'),
                selected: selected,
                onDeleted: () {},
              ),
              FilterChip.elevated(
                avatar: const Icon(elevatedAvatar),
                deleteIcon: const Icon(elevatedDeleteIcon),
                onSelected: (bool valueChanged) {},
                label: const Text('Elevated FilterChip'),
                selected: selected,
                onDeleted: () {},
              ),
            ],
          ),
        ),
      );
    }

    Color getIconColor(WidgetTester tester, IconData icon) {
      return tester
          .firstWidget<IconTheme>(
            find.ancestor(of: find.byIcon(icon), matching: find.byType(IconTheme)),
          )
          .data
          .color!;
    }

    // Test unselected state.
    await tester.pumpWidget(buildChips(selected: false));
    // Check the flat chip icon colors.
    expect(getIconColor(tester, flatAvatar), theme.colorScheme.primary);
    expect(getIconColor(tester, flatDeleteIcon), theme.colorScheme.onSurfaceVariant);
    // Check the elevated chip icon colors.
    expect(getIconColor(tester, elevatedAvatar), theme.colorScheme.primary);
    expect(getIconColor(tester, elevatedDeleteIcon), theme.colorScheme.onSurfaceVariant);

    // Test selected state.
    await tester.pumpWidget(buildChips(selected: true));
    // Check the flat chip icon colors.
    expect(getIconColor(tester, flatAvatar), theme.colorScheme.onSecondaryContainer);
    expect(getIconColor(tester, flatDeleteIcon), theme.colorScheme.onSecondaryContainer);
    // Check the elevated chip icon colors.
    expect(getIconColor(tester, elevatedAvatar), theme.colorScheme.onSecondaryContainer);
    expect(getIconColor(tester, elevatedDeleteIcon), theme.colorScheme.onSecondaryContainer);
  });

  testWidgets('FilterChip avatar layout constraints can be customized', (
    WidgetTester tester,
  ) async {
    const double border = 1.0;
    const double iconSize = 18.0;
    const double labelPadding = 8.0;
    const double padding = 8.0;
    const Size labelSize = Size(100, 100);

    Widget buildChip({BoxConstraints? avatarBoxConstraints}) {
      return wrapForChip(
        child: Center(
          child: FilterChip(
            avatarBoxConstraints: avatarBoxConstraints,
            avatar: const Icon(Icons.favorite),
            label: Container(
              width: labelSize.width,
              height: labelSize.width,
              color: const Color(0xFFFF0000),
            ),
            onSelected: (bool value) {},
          ),
        ),
      );
    }

    // Test default avatar layout constraints.
    await tester.pumpWidget(buildChip());

    expect(tester.getSize(find.byType(FilterChip)).width, equals(234.0));
    expect(tester.getSize(find.byType(FilterChip)).height, equals(118.0));

    // Calculate the distance between avatar and chip edges.
    Offset chipTopLeft = tester.getTopLeft(find.byWidget(getMaterial(tester)));
    final Offset avatarCenter = tester.getCenter(find.byIcon(Icons.favorite));
    expect(chipTopLeft.dx, avatarCenter.dx - (labelSize.width / 2) - padding - border);
    expect(chipTopLeft.dy, avatarCenter.dy - (labelSize.width / 2) - padding - border);

    // Calculate the distance between avatar and label.
    Offset labelTopLeft = tester.getTopLeft(find.byType(Container));
    expect(labelTopLeft.dx, avatarCenter.dx + (labelSize.width / 2) + labelPadding);

    // Test custom avatar layout constraints.
    await tester.pumpWidget(buildChip(avatarBoxConstraints: const BoxConstraints.tightForFinite()));
    await tester.pump();

    expect(tester.getSize(find.byType(FilterChip)).width, equals(152.0));
    expect(tester.getSize(find.byType(FilterChip)).height, equals(118.0));

    // Calculate the distance between avatar and chip edges.
    chipTopLeft = tester.getTopLeft(find.byWidget(getMaterial(tester)));
    expect(chipTopLeft.dx, avatarCenter.dx - (iconSize / 2) - padding - border);
    expect(chipTopLeft.dy, avatarCenter.dy - (labelSize.width / 2) - padding - border);

    // Calculate the distance between avatar and label.
    labelTopLeft = tester.getTopLeft(find.byType(Container));
    expect(labelTopLeft.dx, avatarCenter.dx + (iconSize / 2) + labelPadding);
  });

  testWidgets('FilterChip delete icon layout constraints can be customized', (
    WidgetTester tester,
  ) async {
    const double border = 1.0;
    const double iconSize = 18.0;
    const double labelPadding = 8.0;
    const double padding = 8.0;
    const Size labelSize = Size(100, 100);

    Widget buildChip({BoxConstraints? deleteIconBoxConstraints}) {
      return wrapForChip(
        child: Center(
          child: FilterChip(
            deleteIconBoxConstraints: deleteIconBoxConstraints,
            onDeleted: () {},
            label: Container(
              width: labelSize.width,
              height: labelSize.width,
              color: const Color(0xFFFF0000),
            ),
            onSelected: (bool value) {},
          ),
        ),
      );
    }

    // Test default delete icon layout constraints.
    await tester.pumpWidget(buildChip());

    expect(tester.getSize(find.byType(FilterChip)).width, equals(234.0));
    expect(tester.getSize(find.byType(FilterChip)).height, equals(118.0));

    // Calculate the distance between delete icon and chip edges.
    Offset chipTopRight = tester.getTopRight(find.byWidget(getMaterial(tester)));
    final Offset deleteIconCenter = tester.getCenter(find.byIcon(Icons.clear));
    expect(chipTopRight.dx, deleteIconCenter.dx + (labelSize.width / 2) + padding + border);
    expect(chipTopRight.dy, deleteIconCenter.dy - (labelSize.width / 2) - padding - border);

    // Calculate the distance between delete icon and label.
    Offset labelTopRight = tester.getTopRight(find.byType(Container));
    expect(labelTopRight.dx, deleteIconCenter.dx - (labelSize.width / 2) - labelPadding);

    // Test custom avatar layout constraints.
    await tester.pumpWidget(
      buildChip(deleteIconBoxConstraints: const BoxConstraints.tightForFinite()),
    );
    await tester.pump();

    expect(tester.getSize(find.byType(FilterChip)).width, equals(152.0));
    expect(tester.getSize(find.byType(FilterChip)).height, equals(118.0));

    // Calculate the distance between delete icon and chip edges.
    chipTopRight = tester.getTopRight(find.byWidget(getMaterial(tester)));
    expect(chipTopRight.dx, deleteIconCenter.dx + (iconSize / 2) + padding + border);
    expect(chipTopRight.dy, deleteIconCenter.dy - (labelSize.width / 2) - padding - border);

    // Calculate the distance between delete icon and label.
    labelTopRight = tester.getTopRight(find.byType(Container));
    expect(labelTopRight.dx, deleteIconCenter.dx - (iconSize / 2) - labelPadding);
  });

  testWidgets('FilterChip.chipAnimationStyle is passed to RawChip', (WidgetTester tester) async {
    final ChipAnimationStyle chipAnimationStyle = ChipAnimationStyle(
      enableAnimation: AnimationStyle.noAnimation,
      selectAnimation: AnimationStyle(duration: Durations.extralong4),
    );

    await tester.pumpWidget(
      wrapForChip(
        child: Center(
          child: FilterChip(
            chipAnimationStyle: chipAnimationStyle,
            onSelected: (bool value) {},
            label: const Text('FilterChip'),
          ),
        ),
      ),
    );

    expect(tester.widget<RawChip>(find.byType(RawChip)).chipAnimationStyle, chipAnimationStyle);
  });

  testWidgets('Elevated FilterChip.chipAnimationStyle is passed to RawChip', (
    WidgetTester tester,
  ) async {
    final ChipAnimationStyle chipAnimationStyle = ChipAnimationStyle(
      enableAnimation: AnimationStyle.noAnimation,
      selectAnimation: AnimationStyle(duration: Durations.extralong4),
    );

    await tester.pumpWidget(
      wrapForChip(
        child: Center(
          child: FilterChip.elevated(
            chipAnimationStyle: chipAnimationStyle,
            onSelected: (bool value) {},
            label: const Text('FilterChip'),
          ),
        ),
      ),
    );

    expect(tester.widget<RawChip>(find.byType(RawChip)).chipAnimationStyle, chipAnimationStyle);
  });

  testWidgets('FilterChip mouse cursor behavior', (WidgetTester tester) async {
    const SystemMouseCursor customCursor = SystemMouseCursors.grab;

    await tester.pumpWidget(
      wrapForChip(
        child: Center(
          child: FilterChip(
            mouseCursor: customCursor,
            label: const Text('Chip'),
            onSelected: (bool value) {},
          ),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: const Offset(10, 10));
    await tester.pump();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );

    final Offset chip = tester.getCenter(find.text('Chip'));
    await gesture.moveTo(chip);
    await tester.pump();
    expect(RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1), customCursor);
  });

  testWidgets('Mouse cursor resolves in focused/unfocused/disabled states', (
    WidgetTester tester,
  ) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    final FocusNode focusNode = FocusNode(debugLabel: 'Chip');
    addTearDown(focusNode.dispose);

    Widget buildChip({required bool enabled}) {
      return wrapForChip(
        child: Center(
          child: FilterChip(
            mouseCursor: const WidgetStateMouseCursor.fromMap(<WidgetStatesConstraint, MouseCursor>{
              WidgetState.disabled: SystemMouseCursors.forbidden,
              WidgetState.focused: SystemMouseCursors.grab,
              WidgetState.selected: SystemMouseCursors.click,
              WidgetState.any: SystemMouseCursors.basic,
            }),
            focusNode: focusNode,
            label: const Text('Chip'),
            onSelected: enabled ? (bool value) {} : null,
          ),
        ),
      );
    }

    // Unfocused case.
    await tester.pumpWidget(buildChip(enabled: true));
    final TestGesture gesture1 = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    addTearDown(gesture1.removePointer);
    await gesture1.addPointer(location: tester.getCenter(find.text('Chip')));
    await tester.pump();
    await gesture1.moveTo(tester.getCenter(find.text('Chip')));
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );

    // Focused case.
    focusNode.requestFocus();
    await tester.pump();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.grab,
    );

    // Disabled case.
    await tester.pumpWidget(buildChip(enabled: false));
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.forbidden,
    );
  });
}
