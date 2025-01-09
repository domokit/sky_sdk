// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

RenderBox getMaterialBox(WidgetTester tester, Finder type) {
  return tester.firstRenderObject<RenderBox>(
    find.descendant(of: type, matching: find.byType(CustomPaint)),
  );
}

Material getMaterial(WidgetTester tester) {
  return tester.widget<Material>(
    find.descendant(of: find.byType(ChoiceChip), matching: find.byType(Material)),
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

/// Adds the basic requirements for a Chip.
Widget wrapForChip({
  required Widget child,
  TextDirection textDirection = TextDirection.ltr,
  TextScaler textScaler = TextScaler.noScaling,
  Brightness brightness = Brightness.light,
  bool? useMaterial3,
}) {
  return MaterialApp(
    theme: ThemeData(brightness: brightness, useMaterial3: useMaterial3),
    home: Directionality(
      textDirection: textDirection,
      child: MediaQuery(
        data: MediaQueryData(textScaler: textScaler),
        child: Material(child: child),
      ),
    ),
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

void main() {
  testWidgets('Material2 - ChoiceChip defaults', (WidgetTester tester) async {
    final ThemeData theme = ThemeData(useMaterial3: false);
    const String label = 'choice chip';

    // Test enabled ChoiceChip defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: ChoiceChip(
              selected: false,
              onSelected: (bool valueChanged) {},
              label: const Text(label),
            ),
          ),
        ),
      ),
    );

    // Test default chip size.
    expect(tester.getSize(find.byType(ChoiceChip)), const Size(178.0, 48.0));
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

    // Test disabled ChoiceChip defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: const Material(child: ChoiceChip(selected: false, label: Text(label))),
      ),
    );
    await tester.pumpAndSettle();

    chipMaterial = getMaterial(tester);
    expect(chipMaterial.elevation, 0);
    expect(chipMaterial.shadowColor, Colors.black);
    expect(chipMaterial.shape, const StadiumBorder());

    decoration = tester.widget<Ink>(find.byType(Ink)).decoration! as ShapeDecoration;
    expect(decoration.color, Colors.black38);

    // Test selected enabled ChoiceChip defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Material(
          child: ChoiceChip(
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

    // Test selected disabled ChoiceChip defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: const Material(child: ChoiceChip(selected: true, label: Text(label))),
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

  testWidgets('Material3 - ChoiceChip defaults', (WidgetTester tester) async {
    final ThemeData theme = ThemeData();
    const String label = 'choice chip';

    // Test enabled ChoiceChip defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: ChoiceChip(
              selected: false,
              onSelected: (bool valueChanged) {},
              label: const Text(label),
            ),
          ),
        ),
      ),
    );

    // Test default chip size.
    expect(
      tester.getSize(find.byType(ChoiceChip)),
      within(distance: 0.01, from: const Size(189.1, 48.0)),
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

    // Test disabled ChoiceChip defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: const Material(child: ChoiceChip(selected: false, label: Text(label))),
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

    // Test selected enabled ChoiceChip defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Material(
          child: ChoiceChip(
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

    // Test selected disabled ChoiceChip defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: const Material(child: ChoiceChip(selected: true, label: Text(label))),
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

  testWidgets('Material3 - ChoiceChip.elevated defaults', (WidgetTester tester) async {
    final ThemeData theme = ThemeData();
    const String label = 'choice chip';

    // Test enabled ChoiceChip.elevated defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: ChoiceChip.elevated(
              selected: false,
              onSelected: (bool valueChanged) {},
              label: const Text(label),
            ),
          ),
        ),
      ),
    );

    // Test default chip size.
    expect(
      tester.getSize(find.byType(ChoiceChip)),
      within(distance: 0.01, from: const Size(189.1, 48.0)),
    );
    // Test default label style.
    expect(
      getLabelStyle(tester, label).style.color!.value,
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

    // Test disabled ChoiceChip.elevated defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: const Material(child: ChoiceChip.elevated(selected: false, label: Text(label))),
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

    // Test selected enabled ChoiceChip.elevated defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Material(
          child: ChoiceChip.elevated(
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

    // Test selected disabled ChoiceChip.elevated defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: const Material(child: ChoiceChip.elevated(selected: false, label: Text(label))),
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
  });

  testWidgets('ChoiceChip.color resolves material states', (WidgetTester tester) async {
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
        useMaterial3: true,
        child: Column(
          children: <Widget>[
            ChoiceChip(
              onSelected: enabled ? (bool value) {} : null,
              selected: selected,
              color: color,
              label: const Text('ChoiceChip'),
            ),
            ChoiceChip.elevated(
              onSelected: enabled ? (bool value) {} : null,
              selected: selected,
              color: color,
              label: const Text('ChoiceChip.elevated'),
            ),
          ],
        ),
      );
    }

    // Test enabled state.
    await tester.pumpWidget(buildApp(enabled: true, selected: false));

    // Enabled ChoiceChip should have the provided backgroundColor.
    expect(
      getMaterialBox(tester, find.byType(RawChip).first),
      paints..rrect(color: backgroundColor),
    );
    // Enabled elevated ChoiceChip should have the provided backgroundColor.
    expect(
      getMaterialBox(tester, find.byType(RawChip).last),
      paints..rrect(color: backgroundColor),
    );

    // Test disabled state.
    await tester.pumpWidget(buildApp(enabled: false, selected: false));
    await tester.pumpAndSettle();

    // Disabled ChoiceChip should have the provided disabledColor.
    expect(getMaterialBox(tester, find.byType(RawChip).first), paints..rrect(color: disabledColor));
    // Disabled elevated ChoiceChip should have the provided disabledColor.
    expect(getMaterialBox(tester, find.byType(RawChip).last), paints..rrect(color: disabledColor));

    // Test enabled & selected state.
    await tester.pumpWidget(buildApp(enabled: true, selected: true));
    await tester.pumpAndSettle();

    // Enabled & selected ChoiceChip should have the provided selectedColor.
    expect(getMaterialBox(tester, find.byType(RawChip).first), paints..rrect(color: selectedColor));
    // Enabled & selected elevated ChoiceChip should have the provided selectedColor.
    expect(getMaterialBox(tester, find.byType(RawChip).last), paints..rrect(color: selectedColor));

    // Test disabled & selected state.
    await tester.pumpWidget(buildApp(enabled: false, selected: true));
    await tester.pumpAndSettle();

    // Disabled & selected ChoiceChip should have the provided disabledSelectedColor.
    expect(
      getMaterialBox(tester, find.byType(RawChip).first),
      paints..rrect(color: disabledSelectedColor),
    );
    // Disabled & selected elevated ChoiceChip should have the provided disabledSelectedColor.
    expect(
      getMaterialBox(tester, find.byType(RawChip).last),
      paints..rrect(color: disabledSelectedColor),
    );
  });

  testWidgets('ChoiceChip uses provided state color properties', (WidgetTester tester) async {
    const Color disabledColor = Color(0xff00ff00);
    const Color backgroundColor = Color(0xff0000ff);
    const Color selectedColor = Color(0xffff0000);
    Widget buildApp({required bool enabled, required bool selected}) {
      return wrapForChip(
        useMaterial3: true,
        child: Column(
          children: <Widget>[
            ChoiceChip(
              onSelected: enabled ? (bool value) {} : null,
              selected: selected,
              disabledColor: disabledColor,
              backgroundColor: backgroundColor,
              selectedColor: selectedColor,
              label: const Text('ChoiceChip'),
            ),
            ChoiceChip.elevated(
              onSelected: enabled ? (bool value) {} : null,
              selected: selected,
              disabledColor: disabledColor,
              backgroundColor: backgroundColor,
              selectedColor: selectedColor,
              label: const Text('ChoiceChip.elevated'),
            ),
          ],
        ),
      );
    }

    // Test enabled chips.
    await tester.pumpWidget(buildApp(enabled: true, selected: false));

    // Enabled ChoiceChip should have the provided backgroundColor.
    expect(
      getMaterialBox(tester, find.byType(RawChip).first),
      paints..rrect(color: backgroundColor),
    );
    // Enabled elevated ChoiceChip should have the provided backgroundColor.
    expect(
      getMaterialBox(tester, find.byType(RawChip).last),
      paints..rrect(color: backgroundColor),
    );

    // Test disabled chips.
    await tester.pumpWidget(buildApp(enabled: false, selected: false));
    await tester.pumpAndSettle();

    // Disabled ChoiceChip should have the provided disabledColor.
    expect(getMaterialBox(tester, find.byType(RawChip).first), paints..rrect(color: disabledColor));
    // Disabled elevated ChoiceChip should have the provided disabledColor.
    expect(getMaterialBox(tester, find.byType(RawChip).last), paints..rrect(color: disabledColor));

    // Test enabled & selected chips.
    await tester.pumpWidget(buildApp(enabled: true, selected: true));
    await tester.pumpAndSettle();

    // Enabled & selected ChoiceChip should have the provided selectedColor.
    expect(getMaterialBox(tester, find.byType(RawChip).first), paints..rrect(color: selectedColor));
    // Enabled & selected elevated ChoiceChip should have the provided selectedColor.
    expect(getMaterialBox(tester, find.byType(RawChip).last), paints..rrect(color: selectedColor));
  });

  testWidgets('ChoiceChip can be tapped', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(child: ChoiceChip(selected: false, label: Text('choice chip'))),
      ),
    );

    await tester.tap(find.byType(ChoiceChip));
    expect(tester.takeException(), null);
  });

  testWidgets('ChoiceChip clipBehavior properly passes through to the Material', (
    WidgetTester tester,
  ) async {
    const Text label = Text('label');
    await tester.pumpWidget(wrapForChip(child: const ChoiceChip(label: label, selected: false)));
    checkChipMaterialClipBehavior(tester, Clip.none);

    await tester.pumpWidget(
      wrapForChip(
        child: const ChoiceChip(label: label, selected: false, clipBehavior: Clip.antiAlias),
      ),
    );
    checkChipMaterialClipBehavior(tester, Clip.antiAlias);
  });

  testWidgets('ChoiceChip passes iconTheme property to RawChip', (WidgetTester tester) async {
    const IconThemeData iconTheme = IconThemeData(color: Colors.red);
    await tester.pumpWidget(
      wrapForChip(
        child: const ChoiceChip(label: Text('Test'), selected: true, iconTheme: iconTheme),
      ),
    );
    final RawChip rawChip = tester.widget(find.byType(RawChip));
    expect(rawChip.iconTheme, iconTheme);
  });

  testWidgets('ChoiceChip passes showCheckmark from ChipTheme to RawChip', (
    WidgetTester tester,
  ) async {
    const bool showCheckmark = false;
    await tester.pumpWidget(
      wrapForChip(
        child: const ChipTheme(
          data: ChipThemeData(showCheckmark: showCheckmark),
          child: ChoiceChip(label: Text('Test'), selected: true),
        ),
      ),
    );
    final RawChip rawChip = tester.widget(find.byType(RawChip));
    expect(rawChip.showCheckmark, showCheckmark);
  });

  testWidgets('ChoiceChip passes checkmark properties to RawChip', (WidgetTester tester) async {
    const bool showCheckmark = false;
    const Color checkmarkColor = Color(0xff0000ff);
    await tester.pumpWidget(
      wrapForChip(
        child: const ChoiceChip(
          label: Text('Test'),
          selected: true,
          showCheckmark: showCheckmark,
          checkmarkColor: checkmarkColor,
        ),
      ),
    );
    final RawChip rawChip = tester.widget(find.byType(RawChip));
    expect(rawChip.showCheckmark, showCheckmark);
    expect(rawChip.checkmarkColor, checkmarkColor);
  });

  testWidgets('ChoiceChip uses provided iconTheme', (WidgetTester tester) async {
    final ThemeData theme = ThemeData();

    Widget buildChip({IconThemeData? iconTheme}) {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: ChoiceChip(
            iconTheme: iconTheme,
            avatar: const Icon(Icons.add),
            label: const Text('Test'),
            selected: false,
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

  testWidgets('ChoiceChip avatar layout constraints can be customized', (
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
          child: ChoiceChip(
            avatarBoxConstraints: avatarBoxConstraints,
            avatar: const Icon(Icons.favorite),
            label: Container(
              width: labelSize.width,
              height: labelSize.width,
              color: const Color(0xFFFF0000),
            ),
            selected: false,
          ),
        ),
      );
    }

    // Test default avatar layout constraints.
    await tester.pumpWidget(buildChip());

    expect(tester.getSize(find.byType(ChoiceChip)).width, equals(234.0));
    expect(tester.getSize(find.byType(ChoiceChip)).height, equals(118.0));

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

    expect(tester.getSize(find.byType(ChoiceChip)).width, equals(152.0));
    expect(tester.getSize(find.byType(ChoiceChip)).height, equals(118.0));

    // Calculate the distance between avatar and chip edges.
    chipTopLeft = tester.getTopLeft(find.byWidget(getMaterial(tester)));
    expect(chipTopLeft.dx, avatarCenter.dx - (iconSize / 2) - padding - border);
    expect(chipTopLeft.dy, avatarCenter.dy - (labelSize.width / 2) - padding - border);

    // Calculate the distance between avatar and label.
    labelTopLeft = tester.getTopLeft(find.byType(Container));
    expect(labelTopLeft.dx, avatarCenter.dx + (iconSize / 2) + labelPadding);
  });

  testWidgets('ChoiceChip.chipAnimationStyle is passed to RawChip', (WidgetTester tester) async {
    final ChipAnimationStyle chipAnimationStyle = ChipAnimationStyle(
      enableAnimation: AnimationStyle(duration: Durations.extralong4),
      selectAnimation: AnimationStyle.noAnimation,
    );

    await tester.pumpWidget(
      wrapForChip(
        child: Center(
          child: ChoiceChip(
            chipAnimationStyle: chipAnimationStyle,
            selected: true,
            label: const Text('ChoiceChip'),
          ),
        ),
      ),
    );

    expect(tester.widget<RawChip>(find.byType(RawChip)).chipAnimationStyle, chipAnimationStyle);
  });

  testWidgets('Elevated ChoiceChip.chipAnimationStyle is passed to RawChip', (
    WidgetTester tester,
  ) async {
    final ChipAnimationStyle chipAnimationStyle = ChipAnimationStyle(
      enableAnimation: AnimationStyle(duration: Durations.extralong4),
      selectAnimation: AnimationStyle.noAnimation,
    );

    await tester.pumpWidget(
      wrapForChip(
        child: Center(
          child: ChoiceChip.elevated(
            chipAnimationStyle: chipAnimationStyle,
            selected: true,
            label: const Text('ChoiceChip'),
          ),
        ),
      ),
    );

    expect(tester.widget<RawChip>(find.byType(RawChip)).chipAnimationStyle, chipAnimationStyle);
  });

  testWidgets('ChoiceChip mouse cursor behavior', (WidgetTester tester) async {
    const SystemMouseCursor customCursor = SystemMouseCursors.grab;

    await tester.pumpWidget(
      wrapForChip(
        child: const Center(
          child: ChoiceChip(selected: false, mouseCursor: customCursor, label: Text('Chip')),
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
          child: ChoiceChip(
            mouseCursor: const WidgetStateMouseCursor.fromMap(<WidgetStatesConstraint, MouseCursor>{
              WidgetState.disabled: SystemMouseCursors.forbidden,
              WidgetState.focused: SystemMouseCursors.grab,
              WidgetState.selected: SystemMouseCursors.click,
              WidgetState.any: SystemMouseCursors.basic,
            }),
            focusNode: focusNode,
            label: const Text('Chip'),
            onSelected: enabled ? (bool value) {} : null,
            selected: false,
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
