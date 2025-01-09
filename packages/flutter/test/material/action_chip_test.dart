// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

/// Adds the basic requirements for a Chip.
Widget wrapForChip({
  required Widget child,
  TextDirection textDirection = TextDirection.ltr,
  TextScaler textScaler = TextScaler.noScaling,
  Brightness brightness = Brightness.light,
}) {
  return MaterialApp(
    theme: ThemeData(brightness: brightness),
    home: Directionality(
      textDirection: textDirection,
      child: MediaQuery(
        data: MediaQueryData(textScaler: textScaler),
        child: Material(child: child),
      ),
    ),
  );
}

RenderBox getMaterialBox(WidgetTester tester, Finder type) {
  return tester.firstRenderObject<RenderBox>(
    find.descendant(of: type, matching: find.byType(CustomPaint)),
  );
}

Material getMaterial(WidgetTester tester) {
  return tester.widget<Material>(
    find.descendant(of: find.byType(ActionChip), matching: find.byType(Material)),
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

void checkChipMaterialClipBehavior(WidgetTester tester, Clip clipBehavior) {
  final Iterable<Material> materials = tester.widgetList<Material>(find.byType(Material));
  // There should be two Material widgets, first Material is from the "_wrapForChip" and
  // last Material is from the "RawChip".
  expect(materials.length, 2);
  // The last Material from `RawChip` should have the clip behavior.
  expect(materials.last.clipBehavior, clipBehavior);
}

void main() {
  testWidgets('Material2 - ActionChip defaults', (WidgetTester tester) async {
    final ThemeData theme = ThemeData(useMaterial3: false);
    const String label = 'action chip';

    // Test enabled ActionChip defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Material(
          child: Center(child: ActionChip(onPressed: () {}, label: const Text(label))),
        ),
      ),
    );

    // Test default chip size.
    expect(tester.getSize(find.byType(ActionChip)), const Size(178.0, 48.0));
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

    // Test disabled ActionChip defaults.
    await tester.pumpWidget(
      MaterialApp(theme: theme, home: const Material(child: ActionChip(label: Text(label)))),
    );
    await tester.pumpAndSettle();

    chipMaterial = getMaterial(tester);
    expect(chipMaterial.elevation, 0);
    expect(chipMaterial.shadowColor, Colors.black);
    expect(chipMaterial.shape, const StadiumBorder());

    decoration = tester.widget<Ink>(find.byType(Ink)).decoration! as ShapeDecoration;
    expect(decoration.color, Colors.black38);
  });

  testWidgets('Material3 - ActionChip defaults', (WidgetTester tester) async {
    final ThemeData theme = ThemeData();
    const String label = 'action chip';

    // Test enabled ActionChip defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Material(
          child: Center(child: ActionChip(onPressed: () {}, label: const Text(label))),
        ),
      ),
    );

    // Test default chip size.
    expect(
      tester.getSize(find.byType(ActionChip)),
      within<Size>(distance: 0.01, from: const Size(189.1, 48.0)),
    );
    // Test default label style.
    expect(getLabelStyle(tester, label).style.color!.value, theme.colorScheme.onSurface.value);

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

    // Test disabled ActionChip defaults.
    await tester.pumpWidget(
      MaterialApp(theme: theme, home: const Material(child: ActionChip(label: Text(label)))),
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
  });

  testWidgets('Material3 - ActionChip.elevated defaults', (WidgetTester tester) async {
    final ThemeData theme = ThemeData();
    const String label = 'action chip';

    // Test enabled ActionChip defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Material(
          child: Center(child: ActionChip.elevated(onPressed: () {}, label: const Text(label))),
        ),
      ),
    );

    // Test default chip size.
    expect(
      tester.getSize(find.byType(ActionChip)),
      within<Size>(distance: 0.01, from: const Size(189.1, 48.0)),
    );
    // Test default label style.
    expect(getLabelStyle(tester, label).style.color!.value, theme.colorScheme.onSurface.value);

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

    // Test disabled ActionChip.elevated defaults.
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: const Material(child: ActionChip.elevated(label: Text(label))),
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

  testWidgets('ActionChip.color resolves material states', (WidgetTester tester) async {
    const Color disabledColor = Color(0xff00ff00);
    const Color backgroundColor = Color(0xff0000ff);
    final MaterialStateProperty<Color?> color = MaterialStateProperty.resolveWith((
      Set<MaterialState> states,
    ) {
      if (states.contains(MaterialState.disabled)) {
        return disabledColor;
      }
      return backgroundColor;
    });
    Widget buildApp({required bool enabled, required bool selected}) {
      return wrapForChip(
        child: Column(
          children: <Widget>[
            ActionChip(
              onPressed: enabled ? () {} : null,
              color: color,
              label: const Text('ActionChip'),
            ),
            ActionChip.elevated(
              onPressed: enabled ? () {} : null,
              color: color,
              label: const Text('ActionChip.elevated'),
            ),
          ],
        ),
      );
    }

    // Test enabled state.
    await tester.pumpWidget(buildApp(enabled: true, selected: false));

    // Enabled ActionChip should have the provided backgroundColor.
    expect(
      getMaterialBox(tester, find.byType(RawChip).first),
      paints..rrect(color: backgroundColor),
    );
    // Enabled elevated ActionChip should have the provided backgroundColor.
    expect(
      getMaterialBox(tester, find.byType(RawChip).last),
      paints..rrect(color: backgroundColor),
    );

    // Test disabled state.
    await tester.pumpWidget(buildApp(enabled: false, selected: false));
    await tester.pumpAndSettle();

    // Disabled ActionChip should have the provided disabledColor.
    expect(getMaterialBox(tester, find.byType(RawChip).first), paints..rrect(color: disabledColor));
    // Disabled elevated ActionChip should have the provided disabledColor.
    expect(getMaterialBox(tester, find.byType(RawChip).last), paints..rrect(color: disabledColor));
  });

  testWidgets('ActionChip uses provided state color properties', (WidgetTester tester) async {
    const Color disabledColor = Color(0xff00ff00);
    const Color backgroundColor = Color(0xff0000ff);
    Widget buildApp({required bool enabled, required bool selected}) {
      return wrapForChip(
        child: Column(
          children: <Widget>[
            ActionChip(
              onPressed: enabled ? () {} : null,
              disabledColor: disabledColor,
              backgroundColor: backgroundColor,
              label: const Text('ActionChip'),
            ),
            ActionChip.elevated(
              onPressed: enabled ? () {} : null,
              disabledColor: disabledColor,
              backgroundColor: backgroundColor,
              label: const Text('ActionChip.elevated'),
            ),
          ],
        ),
      );
    }

    // Test enabled state.
    await tester.pumpWidget(buildApp(enabled: true, selected: false));

    // Enabled ActionChip should have the provided backgroundColor.
    expect(
      getMaterialBox(tester, find.byType(RawChip).first),
      paints..rrect(color: backgroundColor),
    );
    // Enabled elevated ActionChip should have the provided backgroundColor.
    expect(
      getMaterialBox(tester, find.byType(RawChip).last),
      paints..rrect(color: backgroundColor),
    );

    // Test disabled state.
    await tester.pumpWidget(buildApp(enabled: false, selected: false));
    await tester.pumpAndSettle();

    // Disabled ActionChip should have the provided disabledColor.
    expect(getMaterialBox(tester, find.byType(RawChip).first), paints..rrect(color: disabledColor));
    // Disabled elevated ActionChip should have the provided disabledColor.
    expect(getMaterialBox(tester, find.byType(RawChip).last), paints..rrect(color: disabledColor));
  });

  testWidgets('ActionChip can be tapped', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(child: ActionChip(onPressed: () {}, label: const Text('action chip'))),
      ),
    );

    await tester.tap(find.byType(ActionChip));
    expect(tester.takeException(), null);
  });

  testWidgets('ActionChip clipBehavior properly passes through to the Material', (
    WidgetTester tester,
  ) async {
    const Text label = Text('label');
    await tester.pumpWidget(wrapForChip(child: ActionChip(label: label, onPressed: () {})));
    checkChipMaterialClipBehavior(tester, Clip.none);

    await tester.pumpWidget(
      wrapForChip(child: ActionChip(label: label, clipBehavior: Clip.antiAlias, onPressed: () {})),
    );
    checkChipMaterialClipBehavior(tester, Clip.antiAlias);
  });

  testWidgets('ActionChip uses provided iconTheme', (WidgetTester tester) async {
    Widget buildChip({IconThemeData? iconTheme}) {
      return MaterialApp(
        home: Material(
          child: ActionChip(
            iconTheme: iconTheme,
            avatar: const Icon(Icons.add),
            onPressed: () {},
            label: const Text('action chip'),
          ),
        ),
      );
    }

    // Test default icon theme.
    await tester.pumpWidget(buildChip());

    expect(getIconData(tester).color, ThemeData().colorScheme.primary);

    // Test provided icon theme.
    await tester.pumpWidget(buildChip(iconTheme: const IconThemeData(color: Color(0xff00ff00))));

    expect(getIconData(tester).color, const Color(0xff00ff00));
  });

  testWidgets('ActionChip avatar layout constraints can be customized', (
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
          child: ActionChip(
            avatarBoxConstraints: avatarBoxConstraints,
            avatar: const Icon(Icons.favorite),
            label: Container(
              width: labelSize.width,
              height: labelSize.width,
              color: const Color(0xFFFF0000),
            ),
          ),
        ),
      );
    }

    // Test default avatar layout constraints.
    await tester.pumpWidget(buildChip());

    expect(tester.getSize(find.byType(ActionChip)).width, equals(234.0));
    expect(tester.getSize(find.byType(ActionChip)).height, equals(118.0));

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

    expect(tester.getSize(find.byType(ActionChip)).width, equals(152.0));
    expect(tester.getSize(find.byType(ActionChip)).height, equals(118.0));

    // Calculate the distance between avatar and chip edges.
    chipTopLeft = tester.getTopLeft(find.byWidget(getMaterial(tester)));
    expect(chipTopLeft.dx, avatarCenter.dx - (iconSize / 2) - padding - border);
    expect(chipTopLeft.dy, avatarCenter.dy - (labelSize.width / 2) - padding - border);

    // Calculate the distance between avatar and label.
    labelTopLeft = tester.getTopLeft(find.byType(Container));
    expect(labelTopLeft.dx, avatarCenter.dx + (iconSize / 2) + labelPadding);
  });

  testWidgets('ActionChip.chipAnimationStyle is passed to RawChip', (WidgetTester tester) async {
    final ChipAnimationStyle chipAnimationStyle = ChipAnimationStyle(
      enableAnimation: AnimationStyle(duration: Durations.extralong4),
      selectAnimation: AnimationStyle.noAnimation,
    );

    await tester.pumpWidget(
      wrapForChip(
        child: Center(
          child: ActionChip(
            chipAnimationStyle: chipAnimationStyle,
            label: const Text('ActionChip'),
          ),
        ),
      ),
    );

    expect(tester.widget<RawChip>(find.byType(RawChip)).chipAnimationStyle, chipAnimationStyle);
  });

  testWidgets('Elevated ActionChip.chipAnimationStyle is passed to RawChip', (
    WidgetTester tester,
  ) async {
    final ChipAnimationStyle chipAnimationStyle = ChipAnimationStyle(
      enableAnimation: AnimationStyle(duration: Durations.extralong4),
      selectAnimation: AnimationStyle.noAnimation,
    );

    await tester.pumpWidget(
      wrapForChip(
        child: Center(
          child: ActionChip.elevated(
            chipAnimationStyle: chipAnimationStyle,
            label: const Text('ActionChip'),
          ),
        ),
      ),
    );

    expect(tester.widget<RawChip>(find.byType(RawChip)).chipAnimationStyle, chipAnimationStyle);
  });

  testWidgets('ActionChip mouse cursor behavior', (WidgetTester tester) async {
    const SystemMouseCursor customCursor = SystemMouseCursors.grab;

    await tester.pumpWidget(
      wrapForChip(
        child: const Center(child: ActionChip(mouseCursor: customCursor, label: Text('Chip'))),
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
          child: ActionChip(
            mouseCursor: const WidgetStateMouseCursor.fromMap(<WidgetStatesConstraint, MouseCursor>{
              WidgetState.disabled: SystemMouseCursors.forbidden,
              WidgetState.focused: SystemMouseCursors.grab,
              WidgetState.any: SystemMouseCursors.basic,
            }),
            focusNode: focusNode,
            label: const Text('Chip'),
            onPressed: enabled ? () {} : null,
          ),
        ),
      );
    }

    await tester.pumpWidget(buildChip(enabled: true));

    // Unfocused case.
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
