// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

/// Adds the basic requirements for a Chip.
Widget wrapForChip({
  required Widget child,
  TextDirection textDirection = TextDirection.ltr,
  double textScaleFactor = 1.0,
  ThemeData? theme,
}) {
  return MaterialApp(
    theme: theme,
    home: Directionality(
      textDirection: textDirection,
      child: MediaQuery.withClampedTextScaling(
        minScaleFactor: textScaleFactor,
        maxScaleFactor: textScaleFactor,
        child: Material(child: child),
      ),
    ),
  );
}

Widget selectedInputChip({Color? checkmarkColor, bool enabled = false}) {
  return InputChip(
    label: const Text('InputChip'),
    selected: true,
    isEnabled: enabled,
    // When [enabled] is true we also need to provide one of the chip
    // callbacks, otherwise the chip would have a 'disabled'
    // [MaterialState], which is not the intention.
    onSelected: enabled ? (_) {} : null,
    showCheckmark: true,
    checkmarkColor: checkmarkColor,
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

void expectCheckmarkColor(Finder finder, Color color) {
  expect(
    finder,
    paints
      // Physical model layer path
      ..path()
      // The first layer that is painted is the selection overlay. We do not care
      // how it is painted but it has to be added it to this pattern so that the
      // check mark can be checked next.
      ..rrect()
      // The second layer that is painted is the check mark.
      ..path(color: color),
  );
}

RenderBox getMaterialBox(WidgetTester tester) {
  return tester.firstRenderObject<RenderBox>(
    find.descendant(of: find.byType(InputChip), matching: find.byType(CustomPaint)),
  );
}

Material getMaterial(WidgetTester tester) {
  return tester.widget<Material>(
    find.descendant(of: find.byType(InputChip), matching: find.byType(Material)),
  );
}

IconThemeData getIconData(WidgetTester tester) {
  final IconTheme iconTheme = tester.firstWidget(
    find.descendant(of: find.byType(RawChip), matching: find.byType(IconTheme)),
  );
  return iconTheme.data;
}

void checkChipMaterialClipBehavior(WidgetTester tester, Clip clipBehavior) {
  final Iterable<Material> materials = tester.widgetList<Material>(find.byType(Material));
  // There should be two Material widgets, first Material is from the "_wrapForChip" and
  // last Material is from the "RawChip".
  expect(materials.length, 2);
  // The last Material from `RawChip` should have the clip behavior.
  expect(materials.last.clipBehavior, clipBehavior);
}

// Finds any container of a tooltip.
Finder findTooltipContainer(String tooltipText) {
  return find.ancestor(of: find.text(tooltipText), matching: find.byType(Container));
}

void main() {
  testWidgets('InputChip.color resolves material states', (WidgetTester tester) async {
    const Color disabledSelectedColor = Color(0xffffff00);
    const Color disabledColor = Color(0xff00ff00);
    const Color backgroundColor = Color(0xff0000ff);
    const Color selectedColor = Color(0xffff0000);
    Widget buildApp({required bool enabled, required bool selected}) {
      return wrapForChip(
        child: InputChip(
          onSelected: enabled ? (bool value) {} : null,
          selected: selected,
          color: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
            if (states.contains(MaterialState.disabled) &&
                states.contains(MaterialState.selected)) {
              return disabledSelectedColor;
            }
            if (states.contains(MaterialState.disabled)) {
              return disabledColor;
            }
            if (states.contains(MaterialState.selected)) {
              return selectedColor;
            }
            return backgroundColor;
          }),
          label: const Text('InputChip'),
        ),
      );
    }

    // Test enabled chip.
    await tester.pumpWidget(buildApp(enabled: true, selected: false));

    // Enabled chip should have the provided backgroundColor.
    expect(getMaterialBox(tester), paints..rrect(color: backgroundColor));

    // Test disabled chip.
    await tester.pumpWidget(buildApp(enabled: false, selected: false));
    await tester.pumpAndSettle();

    // Disabled chip should have the provided disabledColor.
    expect(getMaterialBox(tester), paints..rrect(color: disabledColor));

    // Test enabled & selected chip.
    await tester.pumpWidget(buildApp(enabled: true, selected: true));
    await tester.pumpAndSettle();

    // Enabled & selected chip should have the provided selectedColor.
    expect(getMaterialBox(tester), paints..rrect(color: selectedColor));

    // Test disabled & selected chip.
    await tester.pumpWidget(buildApp(enabled: false, selected: true));
    await tester.pumpAndSettle();

    // Disabled & selected chip should have the provided disabledSelectedColor.
    expect(getMaterialBox(tester), paints..rrect(color: disabledSelectedColor));
  });

  testWidgets('InputChip uses provided state color properties', (WidgetTester tester) async {
    const Color disabledColor = Color(0xff00ff00);
    const Color backgroundColor = Color(0xff0000ff);
    const Color selectedColor = Color(0xffff0000);
    Widget buildApp({required bool enabled, required bool selected}) {
      return wrapForChip(
        child: InputChip(
          onSelected: enabled ? (bool value) {} : null,
          selected: selected,
          disabledColor: disabledColor,
          backgroundColor: backgroundColor,
          selectedColor: selectedColor,
          label: const Text('InputChip'),
        ),
      );
    }

    // Test enabled chip.
    await tester.pumpWidget(buildApp(enabled: true, selected: false));

    // Enabled chip should have the provided backgroundColor.
    expect(getMaterialBox(tester), paints..rrect(color: backgroundColor));

    // Test disabled chip.
    await tester.pumpWidget(buildApp(enabled: false, selected: false));
    await tester.pumpAndSettle();

    // Disabled chip should have the provided disabledColor.
    expect(getMaterialBox(tester), paints..rrect(color: disabledColor));

    // Test enabled & selected chip.
    await tester.pumpWidget(buildApp(enabled: true, selected: true));
    await tester.pumpAndSettle();

    // Enabled & selected chip should have the provided selectedColor.
    expect(getMaterialBox(tester), paints..rrect(color: selectedColor));
  });

  testWidgets('InputChip can be tapped', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(home: Material(child: InputChip(label: Text('input chip')))),
    );

    await tester.tap(find.byType(InputChip));
    expect(tester.takeException(), null);
  });

  testWidgets('loses focus when disabled', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'InputChip');
    await tester.pumpWidget(
      wrapForChip(
        child: InputChip(
          focusNode: focusNode,
          autofocus: true,
          shape: const RoundedRectangleBorder(),
          avatar: const CircleAvatar(child: Text('A')),
          label: const Text('Chip A'),
          onPressed: () {},
        ),
      ),
    );
    await tester.pump();
    expect(focusNode.hasPrimaryFocus, isTrue);

    await tester.pumpWidget(
      wrapForChip(
        child: InputChip(
          focusNode: focusNode,
          autofocus: true,
          shape: const RoundedRectangleBorder(),
          avatar: const CircleAvatar(child: Text('A')),
          label: const Text('Chip A'),
        ),
      ),
    );
    await tester.pump();
    expect(focusNode.hasPrimaryFocus, isFalse);

    focusNode.dispose();
  });

  testWidgets('cannot be traversed to when disabled', (WidgetTester tester) async {
    final FocusNode focusNode1 = FocusNode(debugLabel: 'InputChip 1');
    final FocusNode focusNode2 = FocusNode(debugLabel: 'InputChip 2');
    await tester.pumpWidget(
      wrapForChip(
        child: FocusScope(
          child: Column(
            children: <Widget>[
              InputChip(
                focusNode: focusNode1,
                autofocus: true,
                label: const Text('Chip A'),
                onPressed: () {},
              ),
              InputChip(focusNode: focusNode2, autofocus: true, label: const Text('Chip B')),
            ],
          ),
        ),
      ),
    );
    await tester.pump();
    expect(focusNode1.hasPrimaryFocus, isTrue);
    expect(focusNode2.hasPrimaryFocus, isFalse);

    expect(focusNode1.nextFocus(), isFalse);

    await tester.pump();
    expect(focusNode1.hasPrimaryFocus, isTrue);
    expect(focusNode2.hasPrimaryFocus, isFalse);

    focusNode1.dispose();
    focusNode2.dispose();
  });

  testWidgets(
    'Material2 - Input chip disabled check mark color is determined by platform brightness when light',
    (WidgetTester tester) async {
      await pumpCheckmarkChip(
        tester,
        chip: selectedInputChip(),
        theme: ThemeData(useMaterial3: false),
      );

      expectCheckmarkColor(find.byType(InputChip), Colors.black.withAlpha(0xde));
    },
  );

  testWidgets(
    'Material3 - Input chip disabled check mark color is determined by platform brightness when light',
    (WidgetTester tester) async {
      final ThemeData theme = ThemeData();
      await pumpCheckmarkChip(tester, chip: selectedInputChip(), theme: theme);

      expectCheckmarkColor(find.byType(InputChip), theme.colorScheme.onSurface);
    },
  );

  testWidgets(
    'Material2 - Input chip disabled check mark color is determined by platform brightness when dark',
    (WidgetTester tester) async {
      await pumpCheckmarkChip(
        tester,
        chip: selectedInputChip(),
        theme: ThemeData.dark(useMaterial3: false),
      );

      expectCheckmarkColor(find.byType(InputChip), Colors.white.withAlpha(0xde));
    },
  );

  testWidgets(
    'Material3 - Input chip disabled check mark color is determined by platform brightness when dark',
    (WidgetTester tester) async {
      final ThemeData theme = ThemeData.dark();
      await pumpCheckmarkChip(tester, chip: selectedInputChip(), theme: theme);

      expectCheckmarkColor(find.byType(InputChip), theme.colorScheme.onSurface);
    },
  );

  testWidgets('Input chip check mark color can be set by the chip theme', (
    WidgetTester tester,
  ) async {
    await pumpCheckmarkChip(tester, chip: selectedInputChip(), themeColor: const Color(0xff00ff00));

    expectCheckmarkColor(find.byType(InputChip), const Color(0xff00ff00));
  });

  testWidgets('Input chip check mark color can be set by the chip constructor', (
    WidgetTester tester,
  ) async {
    await pumpCheckmarkChip(
      tester,
      chip: selectedInputChip(checkmarkColor: const Color(0xff00ff00)),
    );

    expectCheckmarkColor(find.byType(InputChip), const Color(0xff00ff00));
  });

  testWidgets(
    'Input chip check mark color is set by chip constructor even when a theme color is specified',
    (WidgetTester tester) async {
      await pumpCheckmarkChip(
        tester,
        chip: selectedInputChip(checkmarkColor: const Color(0xffff0000)),
        themeColor: const Color(0xff00ff00),
      );

      expectCheckmarkColor(find.byType(InputChip), const Color(0xffff0000));
    },
  );

  testWidgets('InputChip clipBehavior properly passes through to the Material', (
    WidgetTester tester,
  ) async {
    const Text label = Text('label');
    await tester.pumpWidget(wrapForChip(child: const InputChip(label: label)));
    checkChipMaterialClipBehavior(tester, Clip.none);

    await tester.pumpWidget(
      wrapForChip(child: const InputChip(label: label, clipBehavior: Clip.antiAlias)),
    );
    checkChipMaterialClipBehavior(tester, Clip.antiAlias);
  });

  testWidgets('Material3 - Input chip has correct selected color when enabled', (
    WidgetTester tester,
  ) async {
    final ThemeData theme = ThemeData();
    await pumpCheckmarkChip(tester, chip: selectedInputChip(enabled: true), theme: theme);

    final RenderBox materialBox = getMaterialBox(tester);
    expect(materialBox, paints..rrect(color: theme.colorScheme.secondaryContainer));
  });

  testWidgets('Material3 - Input chip has correct selected color when disabled', (
    WidgetTester tester,
  ) async {
    final ThemeData theme = ThemeData();
    await pumpCheckmarkChip(tester, chip: selectedInputChip(), theme: theme);

    final RenderBox materialBox = getMaterialBox(tester);
    expect(materialBox, paints..path(color: theme.colorScheme.onSurface));
  });

  testWidgets('InputChip uses provided iconTheme', (WidgetTester tester) async {
    final ThemeData theme = ThemeData();

    Widget buildChip({IconThemeData? iconTheme}) {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: InputChip(
            iconTheme: iconTheme,
            avatar: const Icon(Icons.add),
            label: const Text('Test'),
          ),
        ),
      );
    }

    // Test default icon theme.
    await tester.pumpWidget(buildChip());

    expect(getIconData(tester).color, theme.colorScheme.onSurfaceVariant);

    // Test provided icon theme.
    await tester.pumpWidget(buildChip(iconTheme: const IconThemeData(color: Color(0xff00ff00))));

    expect(getIconData(tester).color, const Color(0xff00ff00));
  });

  testWidgets('Delete button is visible on disabled InputChip', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrapForChip(child: InputChip(isEnabled: false, label: const Text('Label'), onDeleted: () {})),
    );

    // Delete button should be visible.
    await expectLater(
      find.byType(RawChip),
      matchesGoldenFile('input_chip.disabled.delete_button.png'),
    );
  });

  testWidgets('Delete button tooltip is not shown on disabled InputChip', (
    WidgetTester tester,
  ) async {
    Widget buildChip({bool enabled = true}) {
      return wrapForChip(
        child: InputChip(isEnabled: enabled, label: const Text('Label'), onDeleted: () {}),
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

  testWidgets('InputChip avatar layout constraints can be customized', (WidgetTester tester) async {
    const double border = 1.0;
    const double iconSize = 18.0;
    const double labelPadding = 8.0;
    const double padding = 8.0;
    const Size labelSize = Size(100, 100);

    Widget buildChip({BoxConstraints? avatarBoxConstraints}) {
      return wrapForChip(
        child: Center(
          child: InputChip(
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

    expect(tester.getSize(find.byType(InputChip)).width, equals(234.0));
    expect(tester.getSize(find.byType(InputChip)).height, equals(118.0));

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

    expect(tester.getSize(find.byType(InputChip)).width, equals(152.0));
    expect(tester.getSize(find.byType(InputChip)).height, equals(118.0));

    // Calculate the distance between avatar and chip edges.
    chipTopLeft = tester.getTopLeft(find.byWidget(getMaterial(tester)));
    expect(chipTopLeft.dx, avatarCenter.dx - (iconSize / 2) - padding - border);
    expect(chipTopLeft.dy, avatarCenter.dy - (labelSize.width / 2) - padding - border);

    // Calculate the distance between avatar and label.
    labelTopLeft = tester.getTopLeft(find.byType(Container));
    expect(labelTopLeft.dx, avatarCenter.dx + (iconSize / 2) + labelPadding);
  });

  testWidgets('InputChip delete icon layout constraints can be customized', (
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
          child: InputChip(
            deleteIconBoxConstraints: deleteIconBoxConstraints,
            onDeleted: () {},
            label: Container(
              width: labelSize.width,
              height: labelSize.width,
              color: const Color(0xFFFF0000),
            ),
          ),
        ),
      );
    }

    // Test default delete icon layout constraints.
    await tester.pumpWidget(buildChip());

    expect(tester.getSize(find.byType(InputChip)).width, equals(234.0));
    expect(tester.getSize(find.byType(InputChip)).height, equals(118.0));

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

    expect(tester.getSize(find.byType(InputChip)).width, equals(152.0));
    expect(tester.getSize(find.byType(InputChip)).height, equals(118.0));

    // Calculate the distance between delete icon and chip edges.
    chipTopRight = tester.getTopRight(find.byWidget(getMaterial(tester)));
    expect(chipTopRight.dx, deleteIconCenter.dx + (iconSize / 2) + padding + border);
    expect(chipTopRight.dy, deleteIconCenter.dy - (labelSize.width / 2) - padding - border);

    // Calculate the distance between delete icon and label.
    labelTopRight = tester.getTopRight(find.byType(Container));
    expect(labelTopRight.dx, deleteIconCenter.dx - (iconSize / 2) - labelPadding);
  });

  testWidgets('InputChip.chipAnimationStyle is passed to RawChip', (WidgetTester tester) async {
    final ChipAnimationStyle chipAnimationStyle = ChipAnimationStyle(
      enableAnimation: AnimationStyle(duration: Durations.short2),
      selectAnimation: AnimationStyle.noAnimation,
    );

    await tester.pumpWidget(
      wrapForChip(
        child: Center(
          child: InputChip(chipAnimationStyle: chipAnimationStyle, label: const Text('InputChip')),
        ),
      ),
    );

    expect(tester.widget<RawChip>(find.byType(RawChip)).chipAnimationStyle, chipAnimationStyle);
  });

  testWidgets('InputChip mouse cursor behavior', (WidgetTester tester) async {
    const SystemMouseCursor customCursor = SystemMouseCursors.grab;

    await tester.pumpWidget(
      wrapForChip(
        child: const Center(child: InputChip(mouseCursor: customCursor, label: Text('Chip'))),
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
          child: InputChip(
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
