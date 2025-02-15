// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'dart:async';
import 'dart:ui' as ui;

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import '../widgets/semantics_tester.dart';

void main() {
  final ThemeData theme = ThemeData();

  testWidgets('Switch can toggle on tap', (WidgetTester tester) async {
    final Key switchKey = UniqueKey();
    bool value = false;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return MaterialApp(
              theme: theme,
              home: Material(
                child: Center(
                  child: Switch(
                    dragStartBehavior: DragStartBehavior.down,
                    key: switchKey,
                    value: value,
                    onChanged: (bool newValue) {
                      setState(() {
                        value = newValue;
                      });
                    },
                  ),
                ),
              ),
            );
          },
        ),
      ),
    );

    expect(value, isFalse);
    await tester.tap(find.byKey(switchKey));
    expect(value, isTrue);
  });

  testWidgets('Switch size is configurable by ThemeData.materialTapTargetSize', (
    WidgetTester tester,
  ) async {
    final bool material3 = theme.useMaterial3;
    await tester.pumpWidget(
      Theme(
        data: theme.copyWith(materialTapTargetSize: MaterialTapTargetSize.padded),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: Switch(
                dragStartBehavior: DragStartBehavior.down,
                value: true,
                onChanged: (bool newValue) {},
              ),
            ),
          ),
        ),
      ),
    );

    // switch width = trackWidth - 2 * trackRadius + _kSwitchMinSize
    // M2 width = 33 - 2 * 7 + 40
    // M3 width = 52 - 2 * 16 + 40
    expect(
      tester.getSize(find.byType(Switch)),
      material3 ? const Size(60.0, 48.0) : const Size(59.0, 48.0),
    );

    await tester.pumpWidget(
      Theme(
        data: theme.copyWith(materialTapTargetSize: MaterialTapTargetSize.shrinkWrap),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: Switch(
                dragStartBehavior: DragStartBehavior.down,
                value: true,
                onChanged: (bool newValue) {},
              ),
            ),
          ),
        ),
      ),
    );

    expect(
      tester.getSize(find.byType(Switch)),
      material3 ? const Size(60.0, 40.0) : const Size(59.0, 40.0),
    );
  });

  testWidgets('Material2 - Switch does not get distorted upon changing constraints with parent', (
    WidgetTester tester,
  ) async {
    const double maxWidth = 300;
    const double maxHeight = 100;

    const ValueKey<String> boundaryKey = ValueKey<String>('switch container');

    Widget buildSwitch({required double width, required double height}) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: Directionality(
            textDirection: TextDirection.ltr,
            child: SizedBox(
              width: maxWidth,
              height: maxHeight,
              child: RepaintBoundary(
                key: boundaryKey,
                child: SizedBox(
                  width: width,
                  height: height,
                  child: Switch(
                    dragStartBehavior: DragStartBehavior.down,
                    value: true,
                    onChanged: (_) {},
                  ),
                ),
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSwitch(width: maxWidth, height: maxHeight));
    await expectLater(find.byKey(boundaryKey), matchesGoldenFile('m2_switch_test.big.on.png'));

    await tester.pumpWidget(buildSwitch(width: 20, height: 10));
    await expectLater(find.byKey(boundaryKey), matchesGoldenFile('m2_switch_test.small.on.png'));
  });

  testWidgets('Material3 - Switch does not get distorted upon changing constraints with parent', (
    WidgetTester tester,
  ) async {
    const double maxWidth = 300;
    const double maxHeight = 100;

    const ValueKey<String> boundaryKey = ValueKey<String>('switch container');

    Widget buildSwitch({required double width, required double height}) {
      return MaterialApp(
        home: Scaffold(
          body: Directionality(
            textDirection: TextDirection.ltr,
            child: SizedBox(
              width: maxWidth,
              height: maxHeight,
              child: RepaintBoundary(
                key: boundaryKey,
                child: SizedBox(
                  width: width,
                  height: height,
                  child: Switch(
                    dragStartBehavior: DragStartBehavior.down,
                    value: true,
                    onChanged: (_) {},
                  ),
                ),
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSwitch(width: maxWidth, height: maxHeight));
    await expectLater(find.byKey(boundaryKey), matchesGoldenFile('m3_switch_test.big.on.png'));

    await tester.pumpWidget(buildSwitch(width: 20, height: 10));
    await expectLater(find.byKey(boundaryKey), matchesGoldenFile('m3_switch_test.small.on.png'));
  });

  testWidgets('Switch can drag (LTR)', (WidgetTester tester) async {
    bool value = false;

    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: Switch(
                    dragStartBehavior: DragStartBehavior.down,
                    value: value,
                    onChanged: (bool newValue) {
                      setState(() {
                        value = newValue;
                      });
                    },
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );

    expect(value, isFalse);

    await tester.drag(find.byType(Switch), const Offset(-30.0, 0.0));

    expect(value, isFalse);

    await tester.drag(find.byType(Switch), const Offset(30.0, 0.0));

    expect(value, isTrue);

    await tester.pump();
    await tester.drag(find.byType(Switch), const Offset(30.0, 0.0));

    expect(value, isTrue);

    await tester.pump();
    await tester.drag(find.byType(Switch), const Offset(-30.0, 0.0));

    expect(value, isFalse);
  });

  testWidgets('Switch can drag with dragStartBehavior', (WidgetTester tester) async {
    bool value = false;

    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: Switch(
                    dragStartBehavior: DragStartBehavior.down,
                    value: value,
                    onChanged: (bool newValue) {
                      setState(() {
                        value = newValue;
                      });
                    },
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );

    expect(value, isFalse);
    await tester.drag(find.byType(Switch), const Offset(-30.0, 0.0));
    expect(value, isFalse);

    await tester.drag(find.byType(Switch), const Offset(30.0, 0.0));
    expect(value, isTrue);
    await tester.pump();
    await tester.drag(find.byType(Switch), const Offset(30.0, 0.0));
    expect(value, isTrue);
    await tester.pump();
    await tester.drag(find.byType(Switch), const Offset(-30.0, 0.0));
    expect(value, isFalse);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Material(
              child: Center(
                child: Switch(
                  value: value,
                  onChanged: (bool newValue) {
                    setState(() {
                      value = newValue;
                    });
                  },
                ),
              ),
            );
          },
        ),
      ),
    );
    await tester.pumpAndSettle();
    final Rect switchRect = tester.getRect(find.byType(Switch));

    TestGesture gesture = await tester.startGesture(switchRect.center);
    // We have to execute the drag in two frames because the first update will
    // just set the start position.
    await gesture.moveBy(const Offset(20.0, 0.0));
    await gesture.moveBy(const Offset(20.0, 0.0));
    expect(value, isFalse);
    await gesture.up();
    expect(value, isTrue);
    await tester.pump();

    gesture = await tester.startGesture(switchRect.center);
    await gesture.moveBy(const Offset(20.0, 0.0));
    await gesture.moveBy(const Offset(20.0, 0.0));
    expect(value, isTrue);
    await gesture.up();
    expect(value, isTrue);
    await tester.pump();

    gesture = await tester.startGesture(switchRect.center);
    await gesture.moveBy(const Offset(-20.0, 0.0));
    await gesture.moveBy(const Offset(-20.0, 0.0));
    expect(value, isTrue);
    await gesture.up();
    expect(value, isFalse);
  });

  testWidgets('Switch can drag (RTL)', (WidgetTester tester) async {
    bool value = false;

    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Directionality(
          textDirection: TextDirection.rtl,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: Switch(
                    dragStartBehavior: DragStartBehavior.down,
                    value: value,
                    onChanged: (bool newValue) {
                      setState(() {
                        value = newValue;
                      });
                    },
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );

    await tester.drag(find.byType(Switch), const Offset(30.0, 0.0));

    expect(value, isFalse);

    await tester.drag(find.byType(Switch), const Offset(-30.0, 0.0));

    expect(value, isTrue);

    await tester.pump();
    await tester.drag(find.byType(Switch), const Offset(-30.0, 0.0));

    expect(value, isTrue);

    await tester.pump();
    await tester.drag(find.byType(Switch), const Offset(30.0, 0.0));

    expect(value, isFalse);
  });

  testWidgets('Material2 - Switch has default colors when enabled', (WidgetTester tester) async {
    bool value = false;
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: Switch(
                    dragStartBehavior: DragStartBehavior.down,
                    value: value,
                    onChanged: (bool newValue) {
                      setState(() {
                        value = newValue;
                      });
                    },
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          color: const Color(0x52000000), // Black with 32% opacity
          rrect: RRect.fromLTRBR(13.0, 17.0, 46.0, 31.0, const Radius.circular(7.0)),
        )
        ..rrect(color: const Color(0x00000000))
        ..rrect(color: const Color(0x33000000))
        ..rrect(color: const Color(0x24000000))
        ..rrect(color: const Color(0x1f000000))
        ..rrect(color: Colors.grey.shade50),
      reason: 'Inactive enabled switch should match these colors',
    );
    await tester.drag(find.byType(Switch), const Offset(-30.0, 0.0));
    await tester.pump();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          color: const Color(0x802196f3),
          rrect: RRect.fromLTRBR(13.0, 17.0, 46.0, 31.0, const Radius.circular(7.0)),
        )
        ..rrect(color: const Color(0x00000000))
        ..rrect(color: const Color(0x33000000))
        ..rrect(color: const Color(0x24000000))
        ..rrect(color: const Color(0x1f000000))
        ..rrect(color: const Color(0xff2196f3)),
      reason: 'Active enabled switch should match these colors',
    );
  });

  testWidgets('Material3 - Switch has default colors when enabled', (WidgetTester tester) async {
    final ThemeData theme = ThemeData();
    final ColorScheme colors = theme.colorScheme;
    bool value = false;
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: Switch(
                    dragStartBehavior: DragStartBehavior.down,
                    value: value,
                    onChanged: (bool newValue) {
                      setState(() {
                        value = newValue;
                      });
                    },
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..save()
        ..rrect(
          style: PaintingStyle.fill,
          color: colors.surfaceContainerHighest,
          rrect: RRect.fromLTRBR(4.0, 8.0, 56.0, 40.0, const Radius.circular(16.0)),
        )
        ..rrect(
          style: PaintingStyle.stroke,
          color: colors.outline,
          rrect: RRect.fromLTRBR(5.0, 9.0, 55.0, 39.0, const Radius.circular(16.0)),
        )
        ..rrect(color: colors.outline), // thumb color
      reason: 'Inactive enabled switch should match these colors',
    );
    await tester.drag(find.byType(Switch), const Offset(-30.0, 0.0));
    await tester.pump();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..save()
        ..rrect(
          style: PaintingStyle.fill,
          color: colors.primary,
          rrect: RRect.fromLTRBR(4.0, 8.0, 56.0, 40.0, const Radius.circular(16.0)),
        )
        ..rrect()
        ..rrect(color: colors.onPrimary), // thumb color
      reason: 'Active enabled switch should match these colors',
    );
  });

  testWidgets('Switch.adaptive(Cupertino) has default colors when enabled', (
    WidgetTester tester,
  ) async {
    final ThemeData theme = ThemeData();
    final ColorScheme colors = theme.colorScheme;
    bool value = false;
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: Switch.adaptive(
                    dragStartBehavior: DragStartBehavior.down,
                    value: value,
                    onChanged: (bool newValue) {
                      setState(() {
                        value = newValue;
                      });
                    },
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..save()
        ..rrect(
          style: PaintingStyle.fill,
          color: colors.surfaceContainerHighest,
          rrect: RRect.fromLTRBR(4.0, 8.0, 56.0, 40.0, const Radius.circular(16.0)),
        )
        ..rrect(
          style: PaintingStyle.stroke,
          color: colors.outline,
          rrect: RRect.fromLTRBR(5.0, 9.0, 55.0, 39.0, const Radius.circular(16.0)),
        )
        ..rrect(color: colors.outline), // thumb color
      reason: 'Inactive enabled switch should match these colors',
    );
    await tester.drag(find.byType(Switch), const Offset(-30.0, 0.0));
    await tester.pump();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..save()
        ..rrect(
          style: PaintingStyle.fill,
          color: colors.primary,
          rrect: RRect.fromLTRBR(4.0, 8.0, 56.0, 40.0, const Radius.circular(16.0)),
        )
        ..rrect()
        ..rrect(color: colors.onPrimary), // thumb color
      reason: 'Active enabled switch should match these colors',
    );
  });

  testWidgets('Material2 - Switch has default colors when disabled', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: const Directionality(
          textDirection: TextDirection.rtl,
          child: Material(child: Center(child: Switch(value: false, onChanged: null))),
        ),
      ),
    );

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          color: Colors.black12,
          rrect: RRect.fromLTRBR(13.0, 17.0, 46.0, 31.0, const Radius.circular(7.0)),
        )
        ..rrect(color: const Color(0x00000000))
        ..rrect(color: const Color(0x33000000))
        ..rrect(color: const Color(0x24000000))
        ..rrect(color: const Color(0x1f000000))
        ..rrect(color: Colors.grey.shade400),
      reason: 'Inactive disabled switch should match these colors',
    );

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: const Directionality(
          textDirection: TextDirection.rtl,
          child: Material(child: Center(child: Switch(value: true, onChanged: null))),
        ),
      ),
    );

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          color: Colors.black12,
          rrect: RRect.fromLTRBR(13.0, 17.0, 46.0, 31.0, const Radius.circular(7.0)),
        )
        ..rrect(color: const Color(0x00000000))
        ..rrect(color: const Color(0x33000000))
        ..rrect(color: const Color(0x24000000))
        ..rrect(color: const Color(0x1f000000))
        ..rrect(color: Colors.grey.shade400),
      reason: 'Active disabled switch should match these colors',
    );
  });

  testWidgets('Material3 - Inactive Switch has default colors when disabled', (
    WidgetTester tester,
  ) async {
    final ThemeData themeData = ThemeData();
    final ColorScheme colors = themeData.colorScheme;

    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: const Directionality(
          textDirection: TextDirection.rtl,
          child: Material(child: Center(child: Switch(value: false, onChanged: null))),
        ),
      ),
    );

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..save()
        ..rrect(
          style: PaintingStyle.fill,
          color: colors.surfaceContainerHighest.withOpacity(0.12),
          rrect: RRect.fromLTRBR(4.0, 8.0, 56.0, 40.0, const Radius.circular(16.0)),
        )
        ..rrect(
          style: PaintingStyle.stroke,
          color: colors.onSurface.withOpacity(0.12),
          rrect: RRect.fromLTRBR(5.0, 9.0, 55.0, 39.0, const Radius.circular(16.0)),
        )
        ..rrect(
          color: Color.alphaBlend(colors.onSurface.withOpacity(0.38), colors.surface),
        ), // thumb color
      reason: 'Inactive disabled switch should match these colors',
    );
  });

  testWidgets('Material3 - Active Switch has default colors when disabled', (
    WidgetTester tester,
  ) async {
    final ThemeData themeData = ThemeData();
    final ColorScheme colors = themeData.colorScheme;
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: const Directionality(
          textDirection: TextDirection.rtl,
          child: Material(child: Center(child: Switch(value: true, onChanged: null))),
        ),
      ),
    );

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..save()
        ..rrect(
          style: PaintingStyle.fill,
          color: colors.onSurface.withOpacity(0.12),
          rrect: RRect.fromLTRBR(4.0, 8.0, 56.0, 40.0, const Radius.circular(16.0)),
        )
        ..rrect()
        ..rrect(color: colors.surface), // thumb color
      reason: 'Active disabled switch should match these colors',
    );
  });

  testWidgets('Material2 - Switch default overlayColor resolves hovered/focused state', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'Switch');
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    Finder findSwitch() {
      return find.byWidgetPredicate((Widget widget) => widget is Switch);
    }

    MaterialInkController? getSwitchMaterial(WidgetTester tester) {
      return Material.of(tester.element(findSwitch()));
    }

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(body: Switch(focusNode: focusNode, value: true, onChanged: (_) {})),
      ),
    );

    // Focused.
    focusNode.requestFocus();
    await tester.pumpAndSettle();

    expect(getSwitchMaterial(tester), paints..circle(color: theme.focusColor));

    // On both hovered and focused, the overlay color should show hovered overlay color.
    final Offset center = tester.getCenter(find.byType(Switch));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    addTearDown(gesture.removePointer);
    await gesture.moveTo(center);
    await tester.pumpAndSettle();

    expect(getSwitchMaterial(tester), paints..circle(color: theme.hoverColor));

    focusNode.dispose();
  });

  testWidgets('Material3 - Switch default overlayColor resolves hovered/focused state', (
    WidgetTester tester,
  ) async {
    final ThemeData theme = ThemeData();
    final FocusNode focusNode = FocusNode(debugLabel: 'Switch');
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;

    Finder findSwitch() {
      return find.byWidgetPredicate((Widget widget) => widget is Switch);
    }

    MaterialInkController? getSwitchMaterial(WidgetTester tester) {
      return Material.of(tester.element(findSwitch()));
    }

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Scaffold(body: Switch(focusNode: focusNode, value: true, onChanged: (_) {})),
      ),
    );

    // Focused.
    focusNode.requestFocus();
    await tester.pumpAndSettle();

    expect(
      getSwitchMaterial(tester),
      paints..circle(color: theme.colorScheme.primary.withOpacity(0.1)),
    );

    // On both hovered and focused, the overlay color should show hovered overlay color.
    final Offset center = tester.getCenter(find.byType(Switch));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    addTearDown(gesture.removePointer);
    await gesture.moveTo(center);
    await tester.pumpAndSettle();

    expect(
      getSwitchMaterial(tester),
      paints..circle(color: theme.colorScheme.primary.withOpacity(0.08)),
    );

    focusNode.dispose();
  });

  testWidgets('Material2 - Switch can be set color', (WidgetTester tester) async {
    bool value = false;
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: Switch(
                    dragStartBehavior: DragStartBehavior.down,
                    value: value,
                    onChanged: (bool newValue) {
                      setState(() {
                        value = newValue;
                      });
                    },
                    activeColor: Colors.red[500],
                    activeTrackColor: Colors.green[500],
                    inactiveThumbColor: Colors.yellow[500],
                    inactiveTrackColor: Colors.blue[500],
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          color: Colors.blue[500],
          rrect: RRect.fromLTRBR(13.0, 17.0, 46.0, 31.0, const Radius.circular(7.0)),
        )
        ..rrect(color: const Color(0x00000000))
        ..rrect(color: const Color(0x33000000))
        ..rrect(color: const Color(0x24000000))
        ..rrect(color: const Color(0x1f000000))
        ..rrect(color: Colors.yellow[500]),
    );
    await tester.drag(find.byType(Switch), const Offset(-30.0, 0.0));
    await tester.pump();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          color: Colors.green[500],
          rrect: RRect.fromLTRBR(13.0, 17.0, 46.0, 31.0, const Radius.circular(7.0)),
        )
        ..rrect(color: const Color(0x00000000))
        ..rrect(color: const Color(0x33000000))
        ..rrect(color: const Color(0x24000000))
        ..rrect(color: const Color(0x1f000000))
        ..rrect(color: Colors.red[500]),
    );
  });

  testWidgets('Material3 - Switch can be set color', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData();
    final ColorScheme colors = themeData.colorScheme;

    bool value = false;
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: Switch(
                    dragStartBehavior: DragStartBehavior.down,
                    value: value,
                    onChanged: (bool newValue) {
                      setState(() {
                        value = newValue;
                      });
                    },
                    activeColor: Colors.red[500],
                    activeTrackColor: Colors.green[500],
                    inactiveThumbColor: Colors.yellow[500],
                    inactiveTrackColor: Colors.blue[500],
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          style: PaintingStyle.fill,
          color: Colors.blue[500],
          rrect: RRect.fromLTRBR(4.0, 8.0, 56.0, 40.0, const Radius.circular(16.0)),
        )
        ..rrect(style: PaintingStyle.stroke, color: colors.outline)
        ..rrect(color: Colors.yellow[500]), // thumb color
    );
    await tester.drag(find.byType(Switch), const Offset(-30.0, 0.0));
    await tester.pump();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          style: PaintingStyle.fill,
          color: Colors.green[500],
          rrect: RRect.fromLTRBR(4.0, 8.0, 56.0, 40.0, const Radius.circular(16.0)),
        )
        ..rrect()
        ..rrect(color: Colors.red[500]), // thumb color
    );
  });

  testWidgets('Drag ends after animation completes', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/17773

    bool value = false;
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: Switch(
                    dragStartBehavior: DragStartBehavior.down,
                    value: value,
                    onChanged: (bool newValue) {
                      setState(() {
                        value = newValue;
                      });
                    },
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );

    expect(value, isFalse);

    final Rect switchRect = tester.getRect(find.byType(Switch));
    final TestGesture gesture = await tester.startGesture(switchRect.centerLeft);
    await tester.pump();
    await gesture.moveBy(Offset(switchRect.width, 0.0));
    await tester.pump();
    await gesture.up();
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 200));

    expect(value, isTrue);
    expect(tester.hasRunningAnimations, false);
  });

  testWidgets('can veto switch dragging result', (WidgetTester tester) async {
    bool value = false;

    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: Switch(
                    dragStartBehavior: DragStartBehavior.down,
                    value: value,
                    onChanged: (bool newValue) {
                      setState(() {
                        value = value || newValue;
                      });
                    },
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );

    // Move a little to the right, not past the middle.
    TestGesture gesture = await tester.startGesture(tester.getRect(find.byType(Switch)).center);
    await gesture.moveBy(const Offset(kTouchSlop + 0.1, 0.0));
    await tester.pump();
    await gesture.moveBy(const Offset(-kTouchSlop + 5.1, 0.0));
    await tester.pump();
    await gesture.up();
    await tester.pump();
    expect(value, isFalse);
    final ToggleableStateMixin state = tester.state<ToggleableStateMixin>(
      find.descendant(
        of: find.byType(Switch),
        matching: find.byWidgetPredicate(
          (Widget widget) => widget.runtimeType.toString() == '_MaterialSwitch',
        ),
      ),
    );
    expect(state.position.value, lessThan(0.5));
    await tester.pump();
    await tester.pumpAndSettle();
    expect(value, isFalse);
    expect(state.position.value, 0);

    // Move past the middle.
    gesture = await tester.startGesture(tester.getRect(find.byType(Switch)).center);
    await gesture.moveBy(const Offset(kTouchSlop + 0.1, 0.0));
    await tester.pump();
    await gesture.up();
    await tester.pump();
    expect(value, isTrue);
    expect(state.position.value, greaterThan(0.5));

    await tester.pump();
    await tester.pumpAndSettle();
    expect(value, isTrue);
    expect(state.position.value, 1.0);

    // Now move back to the left, the revert animation should play.
    gesture = await tester.startGesture(tester.getRect(find.byType(Switch)).center);
    await gesture.moveBy(const Offset(-kTouchSlop - 0.1, 0.0));
    await tester.pump();
    await gesture.up();
    await tester.pump();
    expect(value, isTrue);
    expect(state.position.value, lessThan(0.5));

    await tester.pump();
    await tester.pumpAndSettle();
    expect(value, isTrue);
    expect(state.position.value, 1.0);
  });

  testWidgets('switch has semantic events', (WidgetTester tester) async {
    dynamic semanticEvent;
    bool value = false;
    tester.binding.defaultBinaryMessenger.setMockDecodedMessageHandler<dynamic>(
      SystemChannels.accessibility,
      (dynamic message) async {
        semanticEvent = message;
      },
    );
    final SemanticsTester semanticsTester = SemanticsTester(tester);

    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: Switch(
                    value: value,
                    onChanged: (bool newValue) {
                      setState(() {
                        value = newValue;
                      });
                    },
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );
    await tester.tap(find.byType(Switch));
    final RenderObject object = tester.firstRenderObject(find.byType(Switch));

    expect(value, true);
    expect(semanticEvent, <String, dynamic>{
      'type': 'tap',
      'nodeId': object.debugSemantics!.id,
      'data': <String, dynamic>{},
    });
    expect(object.debugSemantics!.getSemanticsData().hasAction(SemanticsAction.tap), true);

    semanticsTester.dispose();
    tester.binding.defaultBinaryMessenger.setMockDecodedMessageHandler<dynamic>(
      SystemChannels.accessibility,
      null,
    );
  });

  testWidgets('switch sends semantic events from parent if fully merged', (
    WidgetTester tester,
  ) async {
    dynamic semanticEvent;
    bool value = false;
    tester.binding.defaultBinaryMessenger.setMockDecodedMessageHandler<dynamic>(
      SystemChannels.accessibility,
      (dynamic message) async {
        semanticEvent = message;
      },
    );
    final SemanticsTester semanticsTester = SemanticsTester(tester);

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            void onChanged(bool newValue) {
              setState(() {
                value = newValue;
              });
            }

            return Material(
              child: MergeSemantics(
                child: ListTile(
                  leading: const Text('test'),
                  onTap: () {
                    onChanged(!value);
                  },
                  trailing: Switch(value: value, onChanged: onChanged),
                ),
              ),
            );
          },
        ),
      ),
    );
    await tester.tap(find.byType(MergeSemantics));
    final RenderObject object = tester.firstRenderObject(find.byType(MergeSemantics));

    expect(value, true);
    expect(semanticEvent, <String, dynamic>{
      'type': 'tap',
      'nodeId': object.debugSemantics!.id,
      'data': <String, dynamic>{},
    });
    expect(object.debugSemantics!.getSemanticsData().hasAction(SemanticsAction.tap), true);

    semanticsTester.dispose();
    tester.binding.defaultBinaryMessenger.setMockDecodedMessageHandler<dynamic>(
      SystemChannels.accessibility,
      null,
    );
  });

  testWidgets('Switch.adaptive', (WidgetTester tester) async {
    bool value = false;
    const Color activeTrackColor = Color(0xffff1200);
    const Color inactiveTrackColor = Color(0xffff12ff);
    const Color thumbColor = Color(0xffffff00);
    const Color focusColor = Color(0xff00ff00);

    Widget buildFrame(TargetPlatform platform) {
      return MaterialApp(
        theme: ThemeData(platform: platform),
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Material(
              child: Center(
                child: Switch.adaptive(
                  value: value,
                  activeColor: activeTrackColor,
                  inactiveTrackColor: inactiveTrackColor,
                  thumbColor: const MaterialStatePropertyAll<Color?>(thumbColor),
                  focusColor: focusColor,
                  onChanged: (bool newValue) {
                    setState(() {
                      value = newValue;
                    });
                  },
                ),
              ),
            );
          },
        ),
      );
    }

    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    ]) {
      value = false;
      await tester.pumpWidget(buildFrame(platform));
      expect(find.byType(Switch), findsOneWidget, reason: 'on ${platform.name}');
      expect(find.byType(CupertinoSwitch), findsNothing);

      final Switch adaptiveSwitch = tester.widget(find.byType(Switch));
      expect(adaptiveSwitch.activeColor, activeTrackColor, reason: 'on ${platform.name}');
      expect(adaptiveSwitch.inactiveTrackColor, inactiveTrackColor, reason: 'on ${platform.name}');
      expect(
        adaptiveSwitch.thumbColor?.resolve(<MaterialState>{}),
        thumbColor,
        reason: 'on ${platform.name}',
      );
      expect(adaptiveSwitch.focusColor, focusColor, reason: 'on ${platform.name}');

      expect(value, isFalse, reason: 'on ${platform.name}');
      await tester.tap(find.byType(Switch));
      expect(value, isTrue, reason: 'on ${platform.name}');
    }

    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.android,
      TargetPlatform.fuchsia,
      TargetPlatform.linux,
      TargetPlatform.windows,
    ]) {
      value = false;
      await tester.pumpWidget(buildFrame(platform));
      await tester.pumpAndSettle(); // Finish the theme change animation.
      expect(find.byType(CupertinoSwitch), findsNothing);
      expect(value, isFalse, reason: 'on ${platform.name}');
      await tester.tap(find.byType(Switch));
      expect(value, isTrue, reason: 'on ${platform.name}');
    }
  });

  testWidgets('Switch.adaptive default mouse cursor(Cupertino)', (WidgetTester tester) async {
    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    ]) {
      await tester.pumpWidget(buildAdaptiveSwitch(platform: platform, value: false));
      final Size switchSize = tester.getSize(find.byType(Switch));
      expect(switchSize, const Size(60.0, 48.0));
      final TestGesture gesture = await tester.createGesture(
        kind: PointerDeviceKind.mouse,
        pointer: 1,
      );
      await gesture.addPointer(location: tester.getCenter(find.byType(Switch)));
      addTearDown(gesture.removePointer);
      await tester.pump();
      await gesture.moveTo(tester.getCenter(find.byType(Switch)));
      expect(
        RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
        kIsWeb ? SystemMouseCursors.click : SystemMouseCursors.basic,
      );

      await tester.pumpWidget(buildAdaptiveSwitch(platform: platform));
      expect(
        RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
        kIsWeb ? SystemMouseCursors.click : SystemMouseCursors.basic,
      );

      // Test disabled switch.
      await tester.pumpWidget(
        buildAdaptiveSwitch(platform: platform, enabled: false, value: false),
      );
      expect(
        RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
        SystemMouseCursors.basic,
      );
      await gesture.removePointer(location: tester.getCenter(find.byType(Switch)));
      await tester.pump();
    }
  });

  testWidgets('Switch.adaptive default thumb/track color and size(Cupertino)', (
    WidgetTester tester,
  ) async {
    const Color thumbColor = Colors.white;
    const Color inactiveTrackColor = Color.fromARGB(
      40,
      120,
      120,
      128,
    ); // Default inactive track color.
    const Color activeTrackColor = Color.fromARGB(255, 52, 199, 89); // Default active track color.
    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    ]) {
      // Switches have same sizes on both platform but they are more compact on macOS.
      final RRect trackRRect =
          platform == TargetPlatform.iOS
              ? RRect.fromLTRBR(4.5, 8.5, 55.5, 39.5, const Radius.circular(15.5))
              : RRect.fromLTRBR(4.5, 4.5, 55.5, 35.5, const Radius.circular(15.5));
      final RRect inactiveThumbRRect =
          platform == TargetPlatform.iOS
              ? RRect.fromLTRBR(6.0, 10.0, 34.0, 38.0, const Radius.circular(14.0))
              : RRect.fromLTRBR(6.0, 6.0, 34.0, 34.0, const Radius.circular(14.0));
      final RRect activeThumbRRect =
          platform == TargetPlatform.iOS
              ? RRect.fromLTRBR(26.0, 10.0, 54.0, 38.0, const Radius.circular(14.0))
              : RRect.fromLTRBR(26.0, 6.0, 54.0, 34.0, const Radius.circular(14.0));
      await tester.pumpWidget(Container());
      await tester.pumpWidget(buildAdaptiveSwitch(platform: platform, value: false));
      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(
            color: inactiveTrackColor,
            rrect: trackRRect,
          ) // Default cupertino inactive track color
          ..rrect(color: const Color(0x00000000))
          ..rrect(color: const Color(0x26000000))
          ..rrect(color: const Color(0x0f000000))
          ..rrect(color: const Color(0x0a000000)) // Thumb border color(only cupertino)
          ..rrect(color: thumbColor, rrect: inactiveThumbRRect),
        reason: 'Inactive enabled switch should have default track and thumb color',
      );
      expect(find.byType(Opacity), findsOneWidget);
      expect(tester.widget<Opacity>(find.byType(Opacity)).opacity, 1.0);

      await tester.pumpWidget(Container());
      await tester.pumpWidget(buildAdaptiveSwitch(platform: platform));
      await tester.pump();
      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(
            color: activeTrackColor,
            rrect: trackRRect,
          ) // Default cupertino active track color
          ..rrect(color: const Color(0x00000000))
          ..rrect(color: const Color(0x26000000))
          ..rrect(color: const Color(0x0f000000))
          ..rrect(color: const Color(0x0a000000)) // Thumb border color(only cupertino)
          ..rrect(color: thumbColor, rrect: activeThumbRRect),
        reason: 'Active enabled switch should have default track and thumb color',
      );
      expect(find.byType(Opacity), findsOneWidget);
      expect(tester.widget<Opacity>(find.byType(Opacity)).opacity, 1.0);

      // Test disabled switch.
      await tester.pumpWidget(Container());
      await tester.pumpWidget(
        buildAdaptiveSwitch(platform: platform, enabled: false, value: false),
      );
      await tester.pump();
      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(
            color: inactiveTrackColor,
            rrect: trackRRect,
          ) // Default cupertino inactive track color
          ..rrect(color: const Color(0x00000000))
          ..rrect(color: const Color(0x26000000))
          ..rrect(color: const Color(0x0f000000))
          ..rrect(color: const Color(0x0a000000)) // Thumb border color(only cupertino)
          ..rrect(color: thumbColor, rrect: inactiveThumbRRect),
        reason: 'Inactive disabled switch should have default track and thumb color',
      );
      expect(find.byType(Opacity), findsOneWidget);
      expect(tester.widget<Opacity>(find.byType(Opacity)).opacity, 0.5);

      await tester.pumpWidget(Container());
      await tester.pumpWidget(buildAdaptiveSwitch(platform: platform, enabled: false));
      await tester.pump();
      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(
            color: activeTrackColor,
            rrect: trackRRect,
          ) // Default cupertino active track color
          ..rrect(color: const Color(0x00000000))
          ..rrect(color: const Color(0x26000000))
          ..rrect(color: const Color(0x0f000000))
          ..rrect(color: const Color(0x0a000000)) // Thumb border color(only cupertino)
          ..rrect(color: thumbColor, rrect: activeThumbRRect),
        reason: 'Active disabled switch should have default track and thumb color',
      );
      expect(find.byType(Opacity), findsOneWidget);
      expect(tester.widget<Opacity>(find.byType(Opacity)).opacity, 0.5);
    }
  });

  testWidgets('Default Switch.adaptive are not affected by '
      'ThemeData.switchThemeData on iOS/macOS', (WidgetTester tester) async {
    const Color defaultThumbColor = Colors.white;
    const Color defaultInactiveTrackColor = Color.fromARGB(40, 120, 120, 128);
    const Color defaultActiveTrackColor = Color.fromARGB(255, 52, 199, 89);
    const Color updatedThumbColor = Colors.red;
    const Color updatedTrackColor = Colors.green;
    const SwitchThemeData overallSwitchTheme = SwitchThemeData(
      thumbColor: MaterialStatePropertyAll<Color>(updatedThumbColor),
      trackColor: MaterialStatePropertyAll<Color>(updatedTrackColor),
    );
    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    ]) {
      await tester.pumpWidget(Container());
      await tester.pumpWidget(
        buildAdaptiveSwitch(platform: platform, overallSwitchThemeData: overallSwitchTheme),
      );
      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(color: defaultActiveTrackColor)
          ..rrect(color: const Color(0x00000000))
          ..rrect(color: const Color(0x26000000))
          ..rrect(color: const Color(0x0f000000))
          ..rrect(color: const Color(0x0a000000)) // Thumb border color(only cupertino)
          ..rrect(color: defaultThumbColor),
        reason: 'Active enabled switch should still have default track and thumb color',
      );

      await tester.pumpWidget(Container());
      await tester.pumpWidget(
        buildAdaptiveSwitch(
          platform: platform,
          value: false,
          overallSwitchThemeData: overallSwitchTheme,
        ),
      );
      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(color: defaultInactiveTrackColor)
          ..rrect(color: const Color(0x00000000))
          ..rrect(color: const Color(0x26000000))
          ..rrect(color: const Color(0x0f000000))
          ..rrect(color: const Color(0x0a000000)) // Thumb border color(only cupertino)
          ..rrect(color: defaultThumbColor),
        reason: 'Inactive enabled switch should have default track and thumb color',
      );

      await tester.pumpWidget(Container());
      await tester.pumpWidget(
        buildAdaptiveSwitch(
          platform: platform,
          enabled: false,
          value: false,
          overallSwitchThemeData: overallSwitchTheme,
        ),
      );
      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(color: defaultInactiveTrackColor)
          ..rrect(color: const Color(0x00000000))
          ..rrect(color: const Color(0x26000000))
          ..rrect(color: const Color(0x0f000000))
          ..rrect(color: const Color(0x0a000000)) // Thumb border color(only cupertino)
          ..rrect(color: defaultThumbColor),
        reason: 'Inactive disabled switch should have default track and thumb color',
      );
    }

    await tester.pumpWidget(Container());
    await tester.pumpWidget(
      buildAdaptiveSwitch(
        platform: TargetPlatform.android,
        overallSwitchThemeData: overallSwitchTheme,
      ),
    );
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(color: Color(updatedTrackColor.value))
        ..rrect()
        ..rrect(color: Color(updatedThumbColor.value)),
      reason: 'Switch.adaptive is affected by SwitchTheme on other platforms',
    );
  });

  testWidgets('Default Switch.adaptive are not affected by '
      'SwitchThemeData on iOS/macOS', (WidgetTester tester) async {
    const Color defaultThumbColor = Colors.white;
    const Color defaultInactiveTrackColor = Color.fromARGB(40, 120, 120, 128);
    const Color defaultActiveTrackColor = Color.fromARGB(255, 52, 199, 89);
    const Color updatedThumbColor = Colors.red;
    const Color updatedTrackColor = Colors.green;
    const SwitchThemeData switchTheme = SwitchThemeData(
      thumbColor: MaterialStatePropertyAll<Color>(updatedThumbColor),
      trackColor: MaterialStatePropertyAll<Color>(updatedTrackColor),
    );
    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    ]) {
      await tester.pumpWidget(Container());
      await tester.pumpWidget(
        buildAdaptiveSwitch(platform: platform, switchThemeData: switchTheme),
      );
      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(color: defaultActiveTrackColor)
          ..rrect(color: const Color(0x00000000))
          ..rrect(color: const Color(0x26000000))
          ..rrect(color: const Color(0x0f000000))
          ..rrect(color: const Color(0x0a000000)) // Thumb border color(only cupertino)
          ..rrect(color: defaultThumbColor),
        reason: 'Active enabled switch should still have default track and thumb color',
      );

      await tester.pumpWidget(Container());
      await tester.pumpWidget(
        buildAdaptiveSwitch(platform: platform, value: false, switchThemeData: switchTheme),
      );
      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(color: defaultInactiveTrackColor)
          ..rrect(color: const Color(0x00000000))
          ..rrect(color: const Color(0x26000000))
          ..rrect(color: const Color(0x0f000000))
          ..rrect(color: const Color(0x0a000000)) // Thumb border color(only cupertino)
          ..rrect(color: defaultThumbColor),
        reason: 'Inactive enabled switch should have default track and thumb color',
      );

      await tester.pumpWidget(Container());
      await tester.pumpWidget(
        buildAdaptiveSwitch(
          platform: platform,
          enabled: false,
          value: false,
          switchThemeData: switchTheme,
        ),
      );
      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(color: defaultInactiveTrackColor)
          ..rrect(color: const Color(0x00000000))
          ..rrect(color: const Color(0x26000000))
          ..rrect(color: const Color(0x0f000000))
          ..rrect(color: const Color(0x0a000000)) // Thumb border color(only cupertino)
          ..rrect(color: defaultThumbColor),
        reason: 'Inactive disabled switch should have default track and thumb color',
      );
    }

    await tester.pumpWidget(Container());
    await tester.pumpWidget(
      buildAdaptiveSwitch(platform: TargetPlatform.android, switchThemeData: switchTheme),
    );
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(color: Color(updatedTrackColor.value))
        ..rrect()
        ..rrect(color: Color(updatedThumbColor.value)),
      reason: 'Switch.adaptive is affected by SwitchTheme on other platforms',
    );
  });

  testWidgets('Override default adaptive SwitchThemeData on iOS/macOS', (
    WidgetTester tester,
  ) async {
    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    ]) {
      await tester.pumpWidget(Container());
      await tester.pumpWidget(
        buildAdaptiveSwitch(
          platform: platform,
          switchThemeData: const SwitchThemeData(
            thumbColor: MaterialStatePropertyAll<Color>(Colors.yellow),
            trackColor: MaterialStatePropertyAll<Color>(Colors.brown),
          ),
          switchThemeAdaptation: const _SwitchThemeAdaptation(),
        ),
      );

      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(color: Color(Colors.deepPurple.value))
          ..rrect()
          ..rrect()
          ..rrect()
          ..rrect()
          ..rrect(color: Color(Colors.lightGreen.value)),
      );
    }

    // Other platforms should not be affected by the adaptive switch theme.
    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.android,
      TargetPlatform.fuchsia,
      TargetPlatform.linux,
      TargetPlatform.windows,
    ]) {
      await tester.pumpWidget(Container());
      await tester.pumpWidget(
        buildAdaptiveSwitch(
          platform: platform,
          switchThemeData: const SwitchThemeData(
            thumbColor: MaterialStatePropertyAll<Color>(Colors.yellow),
            trackColor: MaterialStatePropertyAll<Color>(Colors.brown),
          ),
          switchThemeAdaptation: const _SwitchThemeAdaptation(),
        ),
      );

      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(color: Color(Colors.brown.value))
          ..rrect()
          ..rrect(color: Color(Colors.yellow.value)),
      );
    }
  });

  testWidgets('Switch.adaptive default focus color(Cupertino)', (WidgetTester tester) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    final FocusNode node = FocusNode();
    addTearDown(node.dispose);
    await tester.pumpWidget(
      buildAdaptiveSwitch(platform: TargetPlatform.macOS, autofocus: true, focusNode: node),
    );
    await tester.pumpAndSettle();
    expect(node.hasPrimaryFocus, isTrue);
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(color: const Color(0xff34c759)) // Track color
        ..rrect()
        ..rrect(
          color: const Color(0xcc6ef28f),
          strokeWidth: 3.5,
          style: PaintingStyle.stroke,
        ) // Focused outline
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect(color: const Color(0xffffffff)), // Thumb color
    );

    await tester.pumpWidget(
      buildAdaptiveSwitch(
        platform: TargetPlatform.macOS,
        autofocus: true,
        focusNode: node,
        focusColor: Colors.red,
      ),
    );
    await tester.pumpAndSettle();
    expect(node.hasPrimaryFocus, isTrue);
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(color: const Color(0xff34c759)) // Track color
        ..rrect()
        ..rrect(
          color: Color(Colors.red.value),
          strokeWidth: 3.5,
          style: PaintingStyle.stroke,
        ) // Focused outline
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect(color: const Color(0xffffffff)), // Thumb color
    );
  });

  testWidgets('Material2 - Switch is focusable and has correct focus color', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'Switch');
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    bool value = true;
    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Switch(
                  value: value,
                  onChanged:
                      enabled
                          ? (bool newValue) {
                            setState(() {
                              value = newValue;
                            });
                          }
                          : null,
                  focusColor: Colors.orange[500],
                  autofocus: true,
                  focusNode: focusNode,
                );
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildApp());

    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isTrue);
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          color: const Color(0x802196f3),
          rrect: RRect.fromLTRBR(13.0, 17.0, 46.0, 31.0, const Radius.circular(7.0)),
        )
        ..circle(color: Colors.orange[500])
        ..rrect(color: const Color(0x33000000))
        ..rrect(color: const Color(0x24000000))
        ..rrect(color: const Color(0x1f000000))
        ..rrect(color: const Color(0xff2196f3)),
    );

    // Check the false value.
    value = false;
    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isTrue);
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          color: const Color(0x52000000),
          rrect: RRect.fromLTRBR(13.0, 17.0, 46.0, 31.0, const Radius.circular(7.0)),
        )
        ..circle(color: Colors.orange[500])
        ..rrect(color: const Color(0x33000000))
        ..rrect(color: const Color(0x24000000))
        ..rrect(color: const Color(0x1f000000))
        ..rrect(color: const Color(0xfffafafa)),
    );

    // Check what happens when disabled.
    value = false;
    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isFalse);
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          color: const Color(0x1f000000),
          rrect: RRect.fromLTRBR(13.0, 17.0, 46.0, 31.0, const Radius.circular(7.0)),
        )
        ..rrect(color: const Color(0x00000000))
        ..rrect(color: const Color(0x33000000))
        ..rrect(color: const Color(0x24000000))
        ..rrect(color: const Color(0x1f000000))
        ..rrect(color: const Color(0xffbdbdbd)),
    );

    focusNode.dispose();
  });

  testWidgets('Material3 - Switch is focusable and has correct focus color', (
    WidgetTester tester,
  ) async {
    final ThemeData themeData = ThemeData();
    final ColorScheme colors = themeData.colorScheme;
    final FocusNode focusNode = FocusNode(debugLabel: 'Switch');
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    bool value = true;
    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        theme: themeData,
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Switch(
                  value: value,
                  onChanged:
                      enabled
                          ? (bool newValue) {
                            setState(() {
                              value = newValue;
                            });
                          }
                          : null,
                  focusColor: Colors.orange[500],
                  autofocus: true,
                  focusNode: focusNode,
                );
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildApp());

    // active, enabled switch
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isTrue);
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          style: PaintingStyle.fill,
          color: colors.primary,
          rrect: RRect.fromLTRBR(4.0, 8.0, 56.0, 40.0, const Radius.circular(16.0)),
        )
        ..circle(color: Colors.orange[500]),
    );

    // Check the false value: inactive enabled switch
    value = false;
    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isTrue);
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          style: PaintingStyle.fill,
          color: colors.surfaceContainerHighest,
          rrect: RRect.fromLTRBR(4.0, 8.0, 56.0, 40.0, const Radius.circular(16.0)),
        )
        ..rrect(
          style: PaintingStyle.stroke,
          color: colors.outline,
          rrect: RRect.fromLTRBR(5.0, 9.0, 55.0, 39.0, const Radius.circular(16.0)),
        )
        ..circle(color: Colors.orange[500]),
    );

    // Check what happens when disabled: inactive disabled switch.
    value = false;
    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isFalse);
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          style: PaintingStyle.fill,
          color: colors.surfaceContainerHighest.withOpacity(0.12),
          rrect: RRect.fromLTRBR(4.0, 8.0, 56.0, 40.0, const Radius.circular(16.0)),
        )
        ..rrect(
          style: PaintingStyle.stroke,
          color: colors.onSurface.withOpacity(0.12),
          rrect: RRect.fromLTRBR(5.0, 9.0, 55.0, 39.0, const Radius.circular(16.0)),
        )
        ..rrect(color: Color.alphaBlend(colors.onSurface.withOpacity(0.38), colors.surface)),
    );

    focusNode.dispose();
  });

  testWidgets('Switch with splash radius set', (WidgetTester tester) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    const double splashRadius = 30;
    Widget buildApp() {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: Switch(
              value: true,
              onChanged: (bool newValue) {},
              focusColor: Colors.orange[500],
              autofocus: true,
              splashRadius: splashRadius,
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints..circle(color: Colors.orange[500], radius: splashRadius),
    );
  });

  testWidgets('Material2 - Switch can be hovered and has correct hover color', (
    WidgetTester tester,
  ) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    bool value = true;
    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Switch(
                  value: value,
                  onChanged:
                      enabled
                          ? (bool newValue) {
                            setState(() {
                              value = newValue;
                            });
                          }
                          : null,
                  hoverColor: Colors.orange[500],
                );
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          color: const Color(0x802196f3),
          rrect: RRect.fromLTRBR(13.0, 17.0, 46.0, 31.0, const Radius.circular(7.0)),
        )
        ..rrect(color: const Color(0x00000000))
        ..rrect(color: const Color(0x33000000))
        ..rrect(color: const Color(0x24000000))
        ..rrect(color: const Color(0x1f000000))
        ..rrect(color: const Color(0xff2196f3)),
    );

    // Start hovering
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    addTearDown(gesture.removePointer);
    await gesture.moveTo(tester.getCenter(find.byType(Switch)));

    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          color: const Color(0x802196f3),
          rrect: RRect.fromLTRBR(13.0, 17.0, 46.0, 31.0, const Radius.circular(7.0)),
        )
        ..circle(color: Colors.orange[500])
        ..rrect(color: const Color(0x33000000))
        ..rrect(color: const Color(0x24000000))
        ..rrect(color: const Color(0x1f000000))
        ..rrect(color: const Color(0xff2196f3)),
    );

    // Check what happens when disabled.
    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          color: const Color(0x1f000000),
          rrect: RRect.fromLTRBR(13.0, 17.0, 46.0, 31.0, const Radius.circular(7.0)),
        )
        ..rrect(color: const Color(0x00000000))
        ..rrect(color: const Color(0x33000000))
        ..rrect(color: const Color(0x24000000))
        ..rrect(color: const Color(0x1f000000))
        ..rrect(color: const Color(0xffbdbdbd)),
    );
  });

  testWidgets('Material3 - Switch can be hovered and has correct hover color', (
    WidgetTester tester,
  ) async {
    final ThemeData themeData = ThemeData();
    final ColorScheme colors = themeData.colorScheme;
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    bool value = true;
    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        theme: themeData,
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Switch(
                  value: value,
                  onChanged:
                      enabled
                          ? (bool newValue) {
                            setState(() {
                              value = newValue;
                            });
                          }
                          : null,
                  hoverColor: Colors.orange[500],
                );
              },
            ),
          ),
        ),
      );
    }

    // active enabled switch
    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          color: colors.primary,
          rrect: RRect.fromLTRBR(4.0, 8.0, 56.0, 40.0, const Radius.circular(16.0)),
        )
        ..rrect()
        ..rrect(color: colors.onPrimary),
    );

    // Start hovering
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    addTearDown(gesture.removePointer);
    await gesture.moveTo(tester.getCenter(find.byType(Switch)));

    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          color: colors.primary,
          rrect: RRect.fromLTRBR(4.0, 8.0, 56.0, 40.0, const Radius.circular(16.0)),
        )
        ..circle(color: Colors.orange[500]),
    );

    // Check what happens for disabled active switch
    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          color: colors.onSurface.withOpacity(0.12),
          rrect: RRect.fromLTRBR(4.0, 8.0, 56.0, 40.0, const Radius.circular(16.0)),
        )
        ..rrect()
        ..rrect(color: colors.surface.withOpacity(1.0)),
    );
  });

  testWidgets('Switch can be toggled by keyboard shortcuts', (WidgetTester tester) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    bool value = true;
    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Switch(
                  value: value,
                  onChanged:
                      enabled
                          ? (bool newValue) {
                            setState(() {
                              value = newValue;
                            });
                          }
                          : null,
                  focusColor: Colors.orange[500],
                  autofocus: true,
                );
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pumpAndSettle();
    // On web, switches don't respond to the enter key.
    expect(value, kIsWeb ? isTrue : isFalse);
    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pumpAndSettle();
    expect(value, isTrue);
    await tester.sendKeyEvent(LogicalKeyboardKey.space);
    await tester.pumpAndSettle();
    expect(value, isFalse);
    await tester.sendKeyEvent(LogicalKeyboardKey.space);
    await tester.pumpAndSettle();
    expect(value, isTrue);
  });

  testWidgets('Switch changes mouse cursor when hovered', (WidgetTester tester) async {
    // Test Switch.adaptive() constructor
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Scaffold(
          body: Align(
            alignment: Alignment.topLeft,
            child: Material(
              child: MouseRegion(
                cursor: SystemMouseCursors.forbidden,
                child: Switch.adaptive(
                  mouseCursor: SystemMouseCursors.text,
                  value: true,
                  onChanged: (_) {},
                ),
              ),
            ),
          ),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: tester.getCenter(find.byType(Switch)));
    addTearDown(gesture.removePointer);

    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );

    // Test Switch() constructor
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Scaffold(
          body: Align(
            alignment: Alignment.topLeft,
            child: Material(
              child: MouseRegion(
                cursor: SystemMouseCursors.forbidden,
                child: Switch(mouseCursor: SystemMouseCursors.text, value: true, onChanged: (_) {}),
              ),
            ),
          ),
        ),
      ),
    );

    await gesture.moveTo(tester.getCenter(find.byType(Switch)));
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );

    // Test default cursor
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Scaffold(
          body: Align(
            alignment: Alignment.topLeft,
            child: Material(
              child: MouseRegion(
                cursor: SystemMouseCursors.forbidden,
                child: Switch(value: true, onChanged: (_) {}),
              ),
            ),
          ),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );

    // Test default cursor when disabled
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: const Scaffold(
          body: Align(
            alignment: Alignment.topLeft,
            child: Material(
              child: MouseRegion(
                cursor: SystemMouseCursors.forbidden,
                child: Switch(value: true, onChanged: null),
              ),
            ),
          ),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );

    await tester.pumpAndSettle();
  });

  testWidgets('Material switch should not recreate its render object when disabled', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/61247.
    bool value = true;
    bool enabled = true;
    late StateSetter stateSetter;
    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              stateSetter = setState;
              return Material(
                child: Center(
                  child: Switch(
                    value: value,
                    onChanged:
                        !enabled
                            ? null
                            : (bool newValue) {
                              setState(() {
                                value = newValue;
                              });
                            },
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );

    final ToggleableStateMixin oldSwitchState = tester.state(
      find.byWidgetPredicate((Widget widget) => widget.runtimeType.toString() == '_MaterialSwitch'),
    );

    stateSetter(() {
      value = false;
    });
    await tester.pump();
    // Disable the switch when the implicit animation begins.
    stateSetter(() {
      enabled = false;
    });
    await tester.pump();

    final ToggleableStateMixin updatedSwitchState = tester.state(
      find.byWidgetPredicate((Widget widget) => widget.runtimeType.toString() == '_MaterialSwitch'),
    );

    expect(updatedSwitchState.isInteractive, false);
    expect(updatedSwitchState, oldSwitchState);
    expect(updatedSwitchState.position.isCompleted, false);
    expect(updatedSwitchState.position.isDismissed, false);
  });

  testWidgets('Material2 - Switch thumb color resolves in active/enabled states', (
    WidgetTester tester,
  ) async {
    const Color activeEnabledThumbColor = Color(0xFF000001);
    const Color activeDisabledThumbColor = Color(0xFF000002);
    const Color inactiveEnabledThumbColor = Color(0xFF000003);
    const Color inactiveDisabledThumbColor = Color(0xFF000004);

    Color getThumbColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        if (states.contains(MaterialState.selected)) {
          return activeDisabledThumbColor;
        }
        return inactiveDisabledThumbColor;
      }
      if (states.contains(MaterialState.selected)) {
        return activeEnabledThumbColor;
      }
      return inactiveEnabledThumbColor;
    }

    final MaterialStateProperty<Color> thumbColor = MaterialStateColor.resolveWith(getThumbColor);

    Widget buildSwitch({required bool enabled, required bool active}) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: Material(
            child: Center(
              child: Switch(
                thumbColor: thumbColor,
                value: active,
                onChanged: enabled ? (_) {} : null,
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSwitch(enabled: false, active: false));

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          color: Colors.black12,
          rrect: RRect.fromLTRBR(13.0, 17.0, 46.0, 31.0, const Radius.circular(7.0)),
        )
        ..rrect(color: const Color(0x00000000))
        ..rrect(color: const Color(0x33000000))
        ..rrect(color: const Color(0x24000000))
        ..rrect(color: const Color(0x1f000000))
        ..rrect(color: inactiveDisabledThumbColor),
      reason: 'Inactive disabled switch should default track and custom thumb color',
    );

    await tester.pumpWidget(buildSwitch(enabled: false, active: true));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          color: Colors.black12,
          rrect: RRect.fromLTRBR(13.0, 17.0, 46.0, 31.0, const Radius.circular(7.0)),
        )
        ..rrect(color: const Color(0x00000000))
        ..rrect(color: const Color(0x33000000))
        ..rrect(color: const Color(0x24000000))
        ..rrect(color: const Color(0x1f000000))
        ..rrect(color: activeDisabledThumbColor),
      reason: 'Active disabled switch should match these colors',
    );

    await tester.pumpWidget(buildSwitch(enabled: true, active: false));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          color: const Color(0x52000000), // Black with 32% opacity,
          rrect: RRect.fromLTRBR(13.0, 17.0, 46.0, 31.0, const Radius.circular(7.0)),
        )
        ..rrect(color: const Color(0x00000000))
        ..rrect(color: const Color(0x33000000))
        ..rrect(color: const Color(0x24000000))
        ..rrect(color: const Color(0x1f000000))
        ..rrect(color: inactiveEnabledThumbColor),
      reason: 'Inactive enabled switch should match these colors',
    );

    await tester.pumpWidget(buildSwitch(enabled: false, active: false));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          color: Colors.black12,
          rrect: RRect.fromLTRBR(13.0, 17.0, 46.0, 31.0, const Radius.circular(7.0)),
        )
        ..rrect(color: const Color(0x00000000))
        ..rrect(color: const Color(0x33000000))
        ..rrect(color: const Color(0x24000000))
        ..rrect(color: const Color(0x1f000000))
        ..rrect(color: inactiveDisabledThumbColor),
      reason: 'Inactive disabled switch should match these colors',
    );
  });

  testWidgets('Material3 - Switch thumb color resolves in active/enabled states', (
    WidgetTester tester,
  ) async {
    final ThemeData themeData = ThemeData();
    final ColorScheme colors = themeData.colorScheme;
    const Color activeEnabledThumbColor = Color(0xFF000001);
    const Color activeDisabledThumbColor = Color(0xFF000002);
    const Color inactiveEnabledThumbColor = Color(0xFF000003);
    const Color inactiveDisabledThumbColor = Color(0xFF000004);

    Color getThumbColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        if (states.contains(MaterialState.selected)) {
          return activeDisabledThumbColor;
        }
        return inactiveDisabledThumbColor;
      }
      if (states.contains(MaterialState.selected)) {
        return activeEnabledThumbColor;
      }
      return inactiveEnabledThumbColor;
    }

    final MaterialStateProperty<Color> thumbColor = MaterialStateColor.resolveWith(getThumbColor);

    Widget buildSwitch({required bool enabled, required bool active}) {
      return Theme(
        data: themeData,
        child: Directionality(
          textDirection: TextDirection.rtl,
          child: Material(
            child: Center(
              child: Switch(
                thumbColor: thumbColor,
                value: active,
                onChanged: enabled ? (_) {} : null,
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSwitch(enabled: false, active: false));

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          style: PaintingStyle.fill,
          color: colors.surfaceContainerHighest.withOpacity(0.12),
          rrect: RRect.fromLTRBR(4.0, 8.0, 56.0, 40.0, const Radius.circular(16.0)),
        )
        ..rrect(
          style: PaintingStyle.stroke,
          color: colors.onSurface.withOpacity(0.12),
          rrect: RRect.fromLTRBR(5.0, 9.0, 55.0, 39.0, const Radius.circular(16.0)),
        )
        ..rrect(color: inactiveDisabledThumbColor),
      reason: 'Inactive disabled switch should default track and custom thumb color',
    );

    await tester.pumpWidget(buildSwitch(enabled: false, active: true));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          style: PaintingStyle.fill,
          color: colors.onSurface.withOpacity(0.12),
          rrect: RRect.fromLTRBR(4.0, 8.0, 56.0, 40.0, const Radius.circular(16.0)),
        )
        ..rrect()
        ..rrect(color: activeDisabledThumbColor),
      reason: 'Active disabled switch should match these colors',
    );

    await tester.pumpWidget(buildSwitch(enabled: true, active: false));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          style: PaintingStyle.fill,
          color: colors.surfaceContainerHighest,
          rrect: RRect.fromLTRBR(4.0, 8.0, 56.0, 40.0, const Radius.circular(16.0)),
        )
        ..rrect()
        ..rrect(color: inactiveEnabledThumbColor),
      reason: 'Inactive enabled switch should match these colors',
    );

    await tester.pumpWidget(buildSwitch(enabled: true, active: true));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          style: PaintingStyle.fill,
          color: colors.primary,
          rrect: RRect.fromLTRBR(4.0, 8.0, 56.0, 40.0, const Radius.circular(16.0)),
        )
        ..rrect()
        ..rrect(color: activeEnabledThumbColor),
      reason: 'Active enabled switch should match these colors',
    );
  });

  testWidgets('Material2 - Switch thumb color resolves in hovered/focused states', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'Switch');
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    const Color hoveredThumbColor = Color(0xFF000001);
    const Color focusedThumbColor = Color(0xFF000002);

    Color getThumbColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.hovered)) {
        return hoveredThumbColor;
      }
      if (states.contains(MaterialState.focused)) {
        return focusedThumbColor;
      }
      return Colors.transparent;
    }

    final MaterialStateProperty<Color> thumbColor = MaterialStateColor.resolveWith(getThumbColor);

    Widget buildSwitch() {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Center(
            child: Switch(
              focusNode: focusNode,
              autofocus: true,
              value: true,
              thumbColor: thumbColor,
              onChanged: (_) {},
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSwitch());
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isTrue);
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          color: const Color(0x802196f3),
          rrect: RRect.fromLTRBR(13.0, 17.0, 46.0, 31.0, const Radius.circular(7.0)),
        )
        ..circle() // Radial reaction
        ..rrect(color: const Color(0x33000000))
        ..rrect(color: const Color(0x24000000))
        ..rrect(color: const Color(0x1f000000))
        ..rrect(color: focusedThumbColor),
      reason: 'Inactive disabled switch should default track and custom thumb color',
    );

    // Start hovering
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    addTearDown(gesture.removePointer);
    await gesture.moveTo(tester.getCenter(find.byType(Switch)));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          color: const Color(0x802196f3),
          rrect: RRect.fromLTRBR(13.0, 17.0, 46.0, 31.0, const Radius.circular(7.0)),
        )
        ..circle()
        ..rrect(color: const Color(0x33000000))
        ..rrect(color: const Color(0x24000000))
        ..rrect(color: const Color(0x1f000000))
        ..rrect(color: hoveredThumbColor),
      reason: 'Inactive disabled switch should default track and custom thumb color',
    );

    focusNode.dispose();
  });

  testWidgets('Material3 - Switch thumb color resolves in hovered/focused states', (
    WidgetTester tester,
  ) async {
    final ThemeData themeData = ThemeData();
    final ColorScheme colors = themeData.colorScheme;
    final FocusNode focusNode = FocusNode(debugLabel: 'Switch');
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    const Color hoveredThumbColor = Color(0xFF000001);
    const Color focusedThumbColor = Color(0xFF000002);

    Color getThumbColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.hovered)) {
        return hoveredThumbColor;
      }
      if (states.contains(MaterialState.focused)) {
        return focusedThumbColor;
      }
      return Colors.transparent;
    }

    final MaterialStateProperty<Color> thumbColor = MaterialStateColor.resolveWith(getThumbColor);

    Widget buildSwitch() {
      return MaterialApp(
        theme: themeData,
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: Material(
            child: Center(
              child: Switch(
                focusNode: focusNode,
                autofocus: true,
                value: true,
                thumbColor: thumbColor,
                onChanged: (_) {},
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSwitch());
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isTrue);
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          style: PaintingStyle.fill,
          color: colors.primary,
          rrect: RRect.fromLTRBR(4.0, 8.0, 56.0, 40.0, const Radius.circular(16.0)),
        )
        ..circle(color: colors.primary.withOpacity(0.1))
        ..rrect(color: focusedThumbColor),
      reason: 'active enabled switch should default track and custom thumb color',
    );

    // Start hovering
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    addTearDown(gesture.removePointer);
    await gesture.moveTo(tester.getCenter(find.byType(Switch)));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          style: PaintingStyle.fill,
          color: colors.primary,
          rrect: RRect.fromLTRBR(4.0, 8.0, 56.0, 40.0, const Radius.circular(16.0)),
        )
        ..circle(color: colors.primary.withOpacity(0.08))
        ..rrect(color: hoveredThumbColor),
      reason: 'active enabled switch should default track and custom thumb color',
    );

    focusNode.dispose();
  });

  testWidgets('Material2 - Track color resolves in active/enabled states', (
    WidgetTester tester,
  ) async {
    const Color activeEnabledTrackColor = Color(0xFF000001);
    const Color activeDisabledTrackColor = Color(0xFF000002);
    const Color inactiveEnabledTrackColor = Color(0xFF000003);
    const Color inactiveDisabledTrackColor = Color(0xFF000004);

    Color getTrackColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        if (states.contains(MaterialState.selected)) {
          return activeDisabledTrackColor;
        }
        return inactiveDisabledTrackColor;
      }
      if (states.contains(MaterialState.selected)) {
        return activeEnabledTrackColor;
      }
      return inactiveEnabledTrackColor;
    }

    final MaterialStateProperty<Color> trackColor = MaterialStateColor.resolveWith(getTrackColor);

    Widget buildSwitch({required bool enabled, required bool active}) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Center(
            child: Switch(
              trackColor: trackColor,
              value: active,
              onChanged: enabled ? (_) {} : null,
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSwitch(enabled: false, active: false));

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints..rrect(
        color: inactiveDisabledTrackColor,
        rrect: RRect.fromLTRBR(13.0, 17.0, 46.0, 31.0, const Radius.circular(7.0)),
      ),
      reason: 'Inactive disabled switch track should use this value',
    );

    await tester.pumpWidget(buildSwitch(enabled: false, active: true));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints..rrect(
        color: activeDisabledTrackColor,
        rrect: RRect.fromLTRBR(13.0, 17.0, 46.0, 31.0, const Radius.circular(7.0)),
      ),
      reason: 'Active disabled switch should match these colors',
    );

    await tester.pumpWidget(buildSwitch(enabled: true, active: false));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints..rrect(
        color: inactiveEnabledTrackColor,
        rrect: RRect.fromLTRBR(13.0, 17.0, 46.0, 31.0, const Radius.circular(7.0)),
      ),
      reason: 'Inactive enabled switch should match these colors',
    );

    await tester.pumpWidget(buildSwitch(enabled: false, active: false));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints..rrect(
        color: inactiveDisabledTrackColor,
        rrect: RRect.fromLTRBR(13.0, 17.0, 46.0, 31.0, const Radius.circular(7.0)),
      ),
      reason: 'Inactive disabled switch should match these colors',
    );
  });

  testWidgets('Material3 - Track color resolves in active/enabled states', (
    WidgetTester tester,
  ) async {
    final ThemeData themeData = ThemeData();
    const Color activeEnabledTrackColor = Color(0xFF000001);
    const Color activeDisabledTrackColor = Color(0xFF000002);
    const Color inactiveEnabledTrackColor = Color(0xFF000003);
    const Color inactiveDisabledTrackColor = Color(0xFF000004);

    Color getTrackColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        if (states.contains(MaterialState.selected)) {
          return activeDisabledTrackColor;
        }
        return inactiveDisabledTrackColor;
      }
      if (states.contains(MaterialState.selected)) {
        return activeEnabledTrackColor;
      }
      return inactiveEnabledTrackColor;
    }

    final MaterialStateProperty<Color> trackColor = MaterialStateColor.resolveWith(getTrackColor);

    Widget buildSwitch({required bool enabled, required bool active}) {
      return Theme(
        data: themeData,
        child: Directionality(
          textDirection: TextDirection.rtl,
          child: Material(
            child: Center(
              child: Switch(
                trackColor: trackColor,
                value: active,
                onChanged: enabled ? (_) {} : null,
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSwitch(enabled: false, active: false));

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints..rrect(
        color: inactiveDisabledTrackColor,
        rrect: RRect.fromLTRBR(4.0, 8.0, 56.0, 40.0, const Radius.circular(16.0)),
      ),
      reason: 'Inactive disabled switch track should use this value',
    );

    await tester.pumpWidget(buildSwitch(enabled: false, active: true));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints..rrect(
        color: activeDisabledTrackColor,
        rrect: RRect.fromLTRBR(4.0, 8.0, 56.0, 40.0, const Radius.circular(16.0)),
      ),
      reason: 'Active disabled switch should match these colors',
    );

    await tester.pumpWidget(buildSwitch(enabled: true, active: false));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints..rrect(
        color: inactiveEnabledTrackColor,
        rrect: RRect.fromLTRBR(4.0, 8.0, 56.0, 40.0, const Radius.circular(16.0)),
      ),
      reason: 'Inactive enabled switch should match these colors',
    );

    await tester.pumpWidget(buildSwitch(enabled: true, active: true));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints..rrect(
        color: activeEnabledTrackColor,
        rrect: RRect.fromLTRBR(4.0, 8.0, 56.0, 40.0, const Radius.circular(16.0)),
      ),
      reason: 'Active enabled switch should match these colors',
    );
  });

  testWidgets('Material2 - Switch track color resolves in hovered/focused states', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'Switch');
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    const Color hoveredTrackColor = Color(0xFF000001);
    const Color focusedTrackColor = Color(0xFF000002);

    Color getTrackColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.hovered)) {
        return hoveredTrackColor;
      }
      if (states.contains(MaterialState.focused)) {
        return focusedTrackColor;
      }
      return Colors.transparent;
    }

    final MaterialStateProperty<Color> trackColor = MaterialStateColor.resolveWith(getTrackColor);

    Widget buildSwitch() {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: Material(
            child: Center(
              child: Switch(
                focusNode: focusNode,
                autofocus: true,
                value: true,
                trackColor: trackColor,
                onChanged: (_) {},
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSwitch());
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isTrue);
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints..rrect(
        color: focusedTrackColor,
        rrect: RRect.fromLTRBR(13.0, 17.0, 46.0, 31.0, const Radius.circular(7.0)),
      ),
      reason: 'Inactive enabled switch should match these colors',
    );

    // Start hovering
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    addTearDown(gesture.removePointer);
    await gesture.moveTo(tester.getCenter(find.byType(Switch)));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints..rrect(
        color: hoveredTrackColor,
        rrect: RRect.fromLTRBR(13.0, 17.0, 46.0, 31.0, const Radius.circular(7.0)),
      ),
      reason: 'Inactive enabled switch should match these colors',
    );

    focusNode.dispose();
  });

  testWidgets('Material3 - Switch track color resolves in hovered/focused states', (
    WidgetTester tester,
  ) async {
    final ThemeData themeData = ThemeData();
    final FocusNode focusNode = FocusNode(debugLabel: 'Switch');
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    const Color hoveredTrackColor = Color(0xFF000001);
    const Color focusedTrackColor = Color(0xFF000002);

    Color getTrackColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.hovered)) {
        return hoveredTrackColor;
      }
      if (states.contains(MaterialState.focused)) {
        return focusedTrackColor;
      }
      return Colors.transparent;
    }

    final MaterialStateProperty<Color> trackColor = MaterialStateColor.resolveWith(getTrackColor);

    Widget buildSwitch() {
      return Theme(
        data: themeData,
        child: Directionality(
          textDirection: TextDirection.rtl,
          child: Material(
            child: Center(
              child: Switch(
                focusNode: focusNode,
                autofocus: true,
                value: true,
                trackColor: trackColor,
                onChanged: (_) {},
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSwitch());
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isTrue);
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints..rrect(
        color: focusedTrackColor,
        rrect: RRect.fromLTRBR(4.0, 8.0, 56.0, 40.0, const Radius.circular(16.0)),
      ),
      reason: 'Active enabled switch should match these colors',
    );

    // Start hovering
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    addTearDown(gesture.removePointer);
    await gesture.moveTo(tester.getCenter(find.byType(Switch)));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints..rrect(
        color: hoveredTrackColor,
        rrect: RRect.fromLTRBR(4.0, 8.0, 56.0, 40.0, const Radius.circular(16.0)),
      ),
      reason: 'Active enabled switch should match these colors',
    );

    focusNode.dispose();
  });

  testWidgets('Material2 - Switch thumb color is blended against surface color', (
    WidgetTester tester,
  ) async {
    final Color activeDisabledThumbColor = Colors.blue.withOpacity(.60);
    final ThemeData theme = ThemeData.light(useMaterial3: false);

    Color getThumbColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return activeDisabledThumbColor;
      }
      return Colors.black;
    }

    final MaterialStateProperty<Color> thumbColor = MaterialStateColor.resolveWith(getThumbColor);

    Widget buildSwitch({required bool enabled, required bool active}) {
      return Directionality(
        textDirection: TextDirection.rtl,
        child: Theme(
          data: theme,
          child: Material(
            child: Center(
              child: Switch(
                thumbColor: thumbColor,
                value: active,
                onChanged: enabled ? (_) {} : null,
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSwitch(enabled: false, active: true));

    final Color expectedThumbColor = Color.alphaBlend(
      activeDisabledThumbColor,
      theme.colorScheme.surface,
    );

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          color: Colors.black12,
          rrect: RRect.fromLTRBR(13.0, 17.0, 46.0, 31.0, const Radius.circular(7.0)),
        )
        ..rrect(color: const Color(0x00000000))
        ..rrect(color: const Color(0x33000000))
        ..rrect(color: const Color(0x24000000))
        ..rrect(color: const Color(0x1f000000))
        ..rrect(color: expectedThumbColor),
      reason: 'Active disabled thumb color should be blended on top of surface color',
    );
  });

  testWidgets('Material3 - Switch thumb color is blended against surface color', (
    WidgetTester tester,
  ) async {
    final Color activeDisabledThumbColor = Colors.blue.withOpacity(.60);
    final ThemeData theme = ThemeData();
    final ColorScheme colors = theme.colorScheme;

    Color getThumbColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return activeDisabledThumbColor;
      }
      return Colors.black;
    }

    final MaterialStateProperty<Color> thumbColor = MaterialStateColor.resolveWith(getThumbColor);

    Widget buildSwitch({required bool enabled, required bool active}) {
      return Directionality(
        textDirection: TextDirection.rtl,
        child: Theme(
          data: theme,
          child: Material(
            child: Center(
              child: Switch(
                thumbColor: thumbColor,
                value: active,
                onChanged: enabled ? (_) {} : null,
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSwitch(enabled: false, active: true));

    final Color expectedThumbColor = Color.alphaBlend(
      activeDisabledThumbColor,
      theme.colorScheme.surface,
    );

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(
          color: colors.onSurface.withOpacity(0.12),
          rrect: RRect.fromLTRBR(4.0, 8.0, 56.0, 40.0, const Radius.circular(16.0)),
        )
        ..rrect()
        ..rrect(color: expectedThumbColor),
      reason: 'Active disabled thumb color should be blended on top of surface color',
    );
  });

  testWidgets('Switch overlay color resolves in active/pressed/focused/hovered states', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'Switch');
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;

    const Color activeThumbColor = Color(0xFF000000);
    const Color inactiveThumbColor = Color(0xFF000010);
    const Color activePressedOverlayColor = Color(0xFF000001);
    const Color inactivePressedOverlayColor = Color(0xFF000002);
    const Color hoverOverlayColor = Color(0xFF000003);
    const Color focusOverlayColor = Color(0xFF000004);
    const Color hoverColor = Color(0xFF000005);
    const Color focusColor = Color(0xFF000006);

    Color? getOverlayColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.pressed)) {
        if (states.contains(MaterialState.selected)) {
          return activePressedOverlayColor;
        }
        return inactivePressedOverlayColor;
      }
      if (states.contains(MaterialState.hovered)) {
        return hoverOverlayColor;
      }
      if (states.contains(MaterialState.focused)) {
        return focusOverlayColor;
      }
      return null;
    }

    const double splashRadius = 24.0;

    Widget buildSwitch({bool active = false, bool focused = false, bool useOverlay = true}) {
      return MaterialApp(
        theme: theme,
        home: Scaffold(
          body: Switch(
            focusNode: focusNode,
            autofocus: focused,
            value: active,
            onChanged: (_) {},
            thumbColor: MaterialStateProperty.resolveWith<Color>((Set<MaterialState> states) {
              if (states.contains(MaterialState.selected)) {
                return activeThumbColor;
              }
              return inactiveThumbColor;
            }),
            overlayColor: useOverlay ? MaterialStateProperty.resolveWith(getOverlayColor) : null,
            hoverColor: hoverColor,
            focusColor: focusColor,
            splashRadius: splashRadius,
          ),
        ),
      );
    }

    // test inactive Switch, and overlayColor is set to null.
    await tester.pumpWidget(buildSwitch(useOverlay: false));
    await tester.press(find.byType(Switch));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect()
        ..circle(color: inactiveThumbColor.withAlpha(kRadialReactionAlpha), radius: splashRadius),
      reason: 'Default inactive pressed Switch should have overlay color from thumbColor',
    );

    // test active Switch, and overlayColor is set to null.
    await tester.pumpWidget(buildSwitch(active: true, useOverlay: false));
    await tester.press(find.byType(Switch));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect()
        ..circle(color: activeThumbColor.withAlpha(kRadialReactionAlpha), radius: splashRadius),
      reason: 'Default active pressed Switch should have overlay color from thumbColor',
    );

    // test inactive Switch with an overlayColor
    await tester.pumpWidget(buildSwitch());
    await tester.press(find.byType(Switch));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect()
        ..circle(color: inactivePressedOverlayColor, radius: splashRadius),
      reason: 'Inactive pressed Switch should have overlay color: $inactivePressedOverlayColor',
    );

    // test active Switch with an overlayColor
    await tester.pumpWidget(buildSwitch(active: true));
    await tester.press(find.byType(Switch));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect()
        ..circle(color: activePressedOverlayColor, radius: splashRadius),
      reason: 'Active pressed Switch should have overlay color: $activePressedOverlayColor',
    );

    await tester.pumpWidget(buildSwitch(focused: true));
    await tester.pumpAndSettle();

    expect(focusNode.hasPrimaryFocus, isTrue);
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect()
        ..circle(color: focusOverlayColor, radius: splashRadius),
      reason: 'Focused Switch should use overlay color $focusOverlayColor over $focusColor',
    );

    // Start hovering
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    addTearDown(gesture.removePointer);
    await gesture.moveTo(tester.getCenter(find.byType(Switch)));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect()
        ..circle(color: hoverOverlayColor, radius: splashRadius),
      reason: 'Hovered Switch should use overlay color $hoverOverlayColor over $hoverColor',
    );

    focusNode.dispose();
  });

  testWidgets('Do not crash when widget disappears while pointer is down', (
    WidgetTester tester,
  ) async {
    Widget buildSwitch(bool show) {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(child: show ? Switch(value: true, onChanged: (_) {}) : Container()),
        ),
      );
    }

    await tester.pumpWidget(buildSwitch(true));
    final Offset center = tester.getCenter(find.byType(Switch));
    // Put a pointer down on the screen.
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pump();
    // While the pointer is down, the widget disappears.
    await tester.pumpWidget(buildSwitch(false));
    expect(find.byType(Switch), findsNothing);
    // Release pointer after widget disappeared.
    await gesture.up();
  });

  testWidgets('disabled switch shows tooltip', (WidgetTester tester) async {
    const String longPressTooltip = 'long press tooltip';
    const String tapTooltip = 'tap tooltip';
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: Tooltip(message: longPressTooltip, child: Switch(onChanged: null, value: true)),
        ),
      ),
    );

    // Default tooltip shows up after long pressed.
    final Finder tooltip0 = find.byType(Tooltip);
    expect(find.text(longPressTooltip), findsNothing);

    await tester.tap(tooltip0);
    await tester.pump(const Duration(milliseconds: 10));
    expect(find.text(longPressTooltip), findsNothing);

    final TestGesture gestureLongPress = await tester.startGesture(tester.getCenter(tooltip0));
    await tester.pump();
    await tester.pump(kLongPressTimeout);
    await gestureLongPress.up();
    await tester.pump();

    expect(find.text(longPressTooltip), findsOneWidget);

    // Tooltip shows up after tapping when set triggerMode to TooltipTriggerMode.tap.
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: Tooltip(
            triggerMode: TooltipTriggerMode.tap,
            message: tapTooltip,
            child: Switch(onChanged: null, value: true),
          ),
        ),
      ),
    );

    await tester.pump(const Duration(days: 1));
    await tester.pumpAndSettle();
    expect(find.text(tapTooltip), findsNothing);
    expect(find.text(longPressTooltip), findsNothing);

    final Finder tooltip1 = find.byType(Tooltip);
    await tester.tap(tooltip1);
    await tester.pump(const Duration(milliseconds: 10));
    expect(find.text(tapTooltip), findsOneWidget);
  });

  group('with image', () {
    late ui.Image image;

    setUp(() async {
      image = await createTestImage(width: 100, height: 100);
    });

    testWidgets('thumb image shows up', (WidgetTester tester) async {
      imageCache.clear();
      final _TestImageProvider provider1 = _TestImageProvider();
      final _TestImageProvider provider2 = _TestImageProvider();

      expect(provider1.loadCallCount, 0);
      expect(provider2.loadCallCount, 0);

      bool value1 = true;
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(platform: TargetPlatform.android),
          home: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Switch(
                  activeThumbImage: provider1,
                  inactiveThumbImage: provider2,
                  value: value1,
                  onChanged: (bool val) {
                    setState(() {
                      value1 = val;
                    });
                  },
                ),
              );
            },
          ),
        ),
      );

      expect(provider1.loadCallCount, 1);
      expect(provider2.loadCallCount, 0);
      expect(imageCache.liveImageCount, 1);
      await tester.tap(find.byType(Switch));
      await tester.pumpAndSettle();
      expect(provider1.loadCallCount, 1);
      expect(provider2.loadCallCount, 1);
      expect(imageCache.liveImageCount, 2);
    });

    testWidgets('do not crash when imageProvider completes after Switch is disposed', (
      WidgetTester tester,
    ) async {
      final DelayedImageProvider imageProvider = DelayedImageProvider(image);

      await tester.pumpWidget(
        MaterialApp(
          theme: theme,
          home: Material(
            child: Center(
              child: Switch(value: true, onChanged: null, inactiveThumbImage: imageProvider),
            ),
          ),
        ),
      );

      expect(find.byType(Switch), findsOneWidget);

      // Dispose the switch by taking down the tree.
      await tester.pumpWidget(Container());
      expect(find.byType(Switch), findsNothing);

      imageProvider.complete();
      expect(tester.takeException(), isNull);
    });

    testWidgets('do not crash when previous imageProvider completes after Switch is disposed', (
      WidgetTester tester,
    ) async {
      final DelayedImageProvider imageProvider1 = DelayedImageProvider(image);
      final DelayedImageProvider imageProvider2 = DelayedImageProvider(image);

      Future<void> buildSwitch(ImageProvider imageProvider) {
        return tester.pumpWidget(
          MaterialApp(
            theme: theme,
            home: Material(
              child: Center(
                child: Switch(value: true, onChanged: null, inactiveThumbImage: imageProvider),
              ),
            ),
          ),
        );
      }

      await buildSwitch(imageProvider1);
      expect(find.byType(Switch), findsOneWidget);
      // Replace the ImageProvider.
      await buildSwitch(imageProvider2);
      expect(find.byType(Switch), findsOneWidget);

      // Dispose the switch by taking down the tree.
      await tester.pumpWidget(Container());
      expect(find.byType(Switch), findsNothing);

      // Completing the replaced ImageProvider shouldn't crash.
      imageProvider1.complete();
      expect(tester.takeException(), isNull);

      imageProvider2.complete();
      expect(tester.takeException(), isNull);
    });
  });

  group('Switch M3 only tests', () {
    testWidgets('M3 Switch has a 300-millisecond animation in total', (WidgetTester tester) async {
      final ThemeData theme = ThemeData();
      bool value = false;
      await tester.pumpWidget(
        MaterialApp(
          theme: theme,
          home: Directionality(
            textDirection: TextDirection.rtl,
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Material(
                  child: Center(
                    child: Switch(
                      value: value,
                      onChanged: (bool newValue) {
                        setState(() {
                          value = newValue;
                        });
                      },
                    ),
                  ),
                );
              },
            ),
          ),
        ),
      );
      expect(value, isFalse);

      final Rect switchRect = tester.getRect(find.byType(Switch));
      final TestGesture gesture = await tester.startGesture(switchRect.centerLeft);
      await tester.pump();
      await gesture.up();
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 200)); // M2 animation duration
      expect(tester.hasRunningAnimations, true);
      await tester.pump(const Duration(milliseconds: 101));
      expect(tester.hasRunningAnimations, false);
    });

    testWidgets('M3 Switch has a stadium shape in the middle of the track', (
      WidgetTester tester,
    ) async {
      final ThemeData theme = ThemeData(colorSchemeSeed: Colors.deepPurple);
      bool value = false;
      await tester.pumpWidget(
        MaterialApp(
          theme: theme,
          home: Directionality(
            textDirection: TextDirection.ltr,
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Material(
                  child: Center(
                    child: Switch(
                      value: value,
                      onChanged: (bool newValue) {
                        setState(() {
                          value = newValue;
                        });
                      },
                    ),
                  ),
                );
              },
            ),
          ),
        ),
      );
      expect(value, isFalse);

      final Rect switchRect = tester.getRect(find.byType(Switch));
      final TestGesture gesture = await tester.startGesture(switchRect.centerLeft);
      await tester.pump();
      await gesture.up();
      await tester.pump();
      // After 33 milliseconds, the switch thumb moves to the middle
      // and has a stadium shape with a size of (34x22).
      await tester.pump(const Duration(milliseconds: 33));
      expect(tester.hasRunningAnimations, true);

      await expectLater(find.byType(Switch), matchesGoldenFile('switch_test.m3.transition.png'));
    });

    testWidgets('M3 Switch thumb bounces in the end of the animation', (WidgetTester tester) async {
      final ThemeData theme = ThemeData();
      bool value = false;
      await tester.pumpWidget(
        MaterialApp(
          theme: theme,
          home: Directionality(
            textDirection: TextDirection.ltr,
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Material(
                  child: Center(
                    child: Switch(
                      value: value,
                      onChanged: (bool newValue) {
                        setState(() {
                          value = newValue;
                        });
                      },
                    ),
                  ),
                );
              },
            ),
          ),
        ),
      );
      expect(value, isFalse);

      final Rect switchRect = tester.getRect(find.byType(Switch));
      final TestGesture gesture = await tester.startGesture(switchRect.centerLeft);
      await tester.pump();
      await gesture.up();
      await tester.pump();
      // The value on y axis is greater than 1 when t > 0.375
      // 300 * 0.375 = 112.5
      await tester.pump(const Duration(milliseconds: 113));
      final ToggleableStateMixin state = tester.state<ToggleableStateMixin>(
        find.descendant(
          of: find.byType(Switch),
          matching: find.byWidgetPredicate(
            (Widget widget) => widget.runtimeType.toString() == '_MaterialSwitch',
          ),
        ),
      );
      expect(tester.hasRunningAnimations, true);
      expect(state.position.value, greaterThan(1));
    });

    testWidgets('Switch thumb shows correct pressed color - M3', (WidgetTester tester) async {
      final ThemeData themeData = ThemeData();
      final ColorScheme colors = themeData.colorScheme;
      Widget buildApp({bool enabled = true, bool value = true}) {
        return MaterialApp(
          theme: themeData,
          home: Material(
            child: Center(
              child: StatefulBuilder(
                builder: (BuildContext context, StateSetter setState) {
                  return Switch(
                    value: value,
                    onChanged:
                        enabled
                            ? (bool newValue) {
                              setState(() {
                                value = newValue;
                              });
                            }
                            : null,
                  );
                },
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildApp());
      await tester.press(find.byType(Switch));
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(
            color: colors.primary, // track color
            style: PaintingStyle.fill,
          )
          ..rrect(
            color: Colors.transparent, // track outline color
            style: PaintingStyle.stroke,
          )
          ..rrect(
            color: colors.primaryContainer,
            rrect: RRect.fromLTRBR(26.0, 10.0, 54.0, 38.0, const Radius.circular(14.0)),
          ),
      );

      await tester.pumpWidget(Container());
      await tester.pumpWidget(buildApp(value: false));
      await tester.press(find.byType(Switch));
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(
            color: colors.surfaceContainerHighest, // track color
            style: PaintingStyle.fill,
          )
          ..rrect(
            color: colors.outline, // track outline color
            style: PaintingStyle.stroke,
          )
          ..rrect(color: colors.onSurfaceVariant),
      );

      await tester.pumpWidget(Container());
      await tester.pumpWidget(buildApp(enabled: false));
      await tester.press(find.byType(Switch));
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(
            color: colors.onSurface.withOpacity(0.12), // track color
            style: PaintingStyle.fill,
          )
          ..rrect(
            color: Colors.transparent, // track outline color
            style: PaintingStyle.stroke,
          )
          ..rrect(color: colors.surface.withOpacity(1.0)),
      );

      await tester.pumpWidget(Container());
      await tester.pumpWidget(buildApp(enabled: false, value: false));
      await tester.press(find.byType(Switch));
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(
            color: colors.surfaceContainerHighest.withOpacity(0.12), // track color
            style: PaintingStyle.fill,
          )
          ..rrect(
            color: colors.onSurface.withOpacity(0.12), // track outline color
            style: PaintingStyle.stroke,
          )
          ..rrect(color: Color.alphaBlend(colors.onSurface.withOpacity(0.38), colors.surface)),
      );
    }, variant: TargetPlatformVariant.mobile());

    testWidgets('Track outline color resolves in active/enabled states', (
      WidgetTester tester,
    ) async {
      const Color activeEnabledTrackOutlineColor = Color(0xFF000001);
      const Color activeDisabledTrackOutlineColor = Color(0xFF000002);
      const Color inactiveEnabledTrackOutlineColor = Color(0xFF000003);
      const Color inactiveDisabledTrackOutlineColor = Color(0xFF000004);

      Color getTrackOutlineColor(Set<MaterialState> states) {
        if (states.contains(MaterialState.disabled)) {
          if (states.contains(MaterialState.selected)) {
            return activeDisabledTrackOutlineColor;
          }
          return inactiveDisabledTrackOutlineColor;
        }
        if (states.contains(MaterialState.selected)) {
          return activeEnabledTrackOutlineColor;
        }
        return inactiveEnabledTrackOutlineColor;
      }

      final MaterialStateProperty<Color> trackOutlineColor = MaterialStateColor.resolveWith(
        getTrackOutlineColor,
      );

      Widget buildSwitch({required bool enabled, required bool active}) {
        return Directionality(
          textDirection: TextDirection.rtl,
          child: Material(
            child: Center(
              child: Switch(
                trackOutlineColor: trackOutlineColor,
                value: active,
                onChanged: enabled ? (_) {} : null,
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildSwitch(enabled: false, active: false));

      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(style: PaintingStyle.fill)
          ..rrect(color: inactiveDisabledTrackOutlineColor, style: PaintingStyle.stroke),
        reason: 'Inactive disabled switch track outline should use this value',
      );

      await tester.pumpWidget(buildSwitch(enabled: false, active: true));
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(style: PaintingStyle.fill)
          ..rrect(color: activeDisabledTrackOutlineColor, style: PaintingStyle.stroke),
        reason: 'Active disabled switch track outline should match these colors',
      );

      await tester.pumpWidget(buildSwitch(enabled: true, active: false));
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(style: PaintingStyle.fill)
          ..rrect(color: inactiveEnabledTrackOutlineColor),
        reason: 'Inactive enabled switch track outline should match these colors',
      );

      await tester.pumpWidget(buildSwitch(enabled: true, active: true));
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(style: PaintingStyle.fill)
          ..rrect(color: activeEnabledTrackOutlineColor),
        reason: 'Active enabled switch track outline should match these colors',
      );
    });

    testWidgets('Switch track outline color resolves in hovered/focused states', (
      WidgetTester tester,
    ) async {
      final FocusNode focusNode = FocusNode(debugLabel: 'Switch');
      tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
      const Color hoveredTrackOutlineColor = Color(0xFF000001);
      const Color focusedTrackOutlineColor = Color(0xFF000002);

      Color getTrackOutlineColor(Set<MaterialState> states) {
        if (states.contains(MaterialState.hovered)) {
          return hoveredTrackOutlineColor;
        }
        if (states.contains(MaterialState.focused)) {
          return focusedTrackOutlineColor;
        }
        return Colors.transparent;
      }

      final MaterialStateProperty<Color> trackOutlineColor = MaterialStateColor.resolveWith(
        getTrackOutlineColor,
      );

      Widget buildSwitch() {
        return Directionality(
          textDirection: TextDirection.rtl,
          child: Material(
            child: Center(
              child: Switch(
                focusNode: focusNode,
                autofocus: true,
                value: true,
                trackOutlineColor: trackOutlineColor,
                onChanged: (_) {},
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildSwitch());
      await tester.pumpAndSettle();
      expect(focusNode.hasPrimaryFocus, isTrue);
      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(style: PaintingStyle.fill)
          ..rrect(color: focusedTrackOutlineColor, style: PaintingStyle.stroke),
        reason: 'Active enabled switch track outline should match this color',
      );

      // Start hovering
      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture.addPointer();
      addTearDown(gesture.removePointer);
      await gesture.moveTo(tester.getCenter(find.byType(Switch)));
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(style: PaintingStyle.fill)
          ..rrect(color: hoveredTrackOutlineColor, style: PaintingStyle.stroke),
        reason: 'Active enabled switch track outline should match this color',
      );

      focusNode.dispose();
    });

    testWidgets('Track outline width resolves in active/enabled states', (
      WidgetTester tester,
    ) async {
      const double activeEnabledTrackOutlineWidth = 1.0;
      const double activeDisabledTrackOutlineWidth = 2.0;
      const double inactiveEnabledTrackOutlineWidth = 3.0;
      const double inactiveDisabledTrackOutlineWidth = 4.0;

      double getTrackOutlineWidth(Set<MaterialState> states) {
        if (states.contains(MaterialState.disabled)) {
          if (states.contains(MaterialState.selected)) {
            return activeDisabledTrackOutlineWidth;
          }
          return inactiveDisabledTrackOutlineWidth;
        }
        if (states.contains(MaterialState.selected)) {
          return activeEnabledTrackOutlineWidth;
        }
        return inactiveEnabledTrackOutlineWidth;
      }

      final MaterialStateProperty<double> trackOutlineWidth = MaterialStateProperty.resolveWith(
        getTrackOutlineWidth,
      );

      Widget buildSwitch({required bool enabled, required bool active}) {
        return MaterialApp(
          home: Material(
            child: Center(
              child: Switch(
                trackOutlineWidth: trackOutlineWidth,
                value: active,
                onChanged: enabled ? (_) {} : null,
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildSwitch(enabled: false, active: false));

      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(style: PaintingStyle.fill)
          ..rrect(strokeWidth: inactiveDisabledTrackOutlineWidth, style: PaintingStyle.stroke),
        reason: 'Inactive disabled switch track outline width should be 4.0',
      );

      await tester.pumpWidget(buildSwitch(enabled: false, active: true));
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(style: PaintingStyle.fill)
          ..rrect(strokeWidth: activeDisabledTrackOutlineWidth, style: PaintingStyle.stroke),
        reason: 'Active disabled switch track outline width should be 2.0',
      );

      await tester.pumpWidget(buildSwitch(enabled: true, active: false));
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(style: PaintingStyle.fill)
          ..rrect(strokeWidth: inactiveEnabledTrackOutlineWidth, style: PaintingStyle.stroke),
        reason: 'Inactive enabled switch track outline width should be 3.0',
      );

      await tester.pumpWidget(buildSwitch(enabled: true, active: true));
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(style: PaintingStyle.fill)
          ..rrect(strokeWidth: activeEnabledTrackOutlineWidth, style: PaintingStyle.stroke),
        reason: 'Active enabled switch track outline width should be 1.0',
      );
    });

    testWidgets('Switch track outline width resolves in hovered/focused states', (
      WidgetTester tester,
    ) async {
      final FocusNode focusNode = FocusNode(debugLabel: 'Switch');
      tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
      const double hoveredTrackOutlineWidth = 4.0;
      const double focusedTrackOutlineWidth = 6.0;

      double getTrackOutlineWidth(Set<MaterialState> states) {
        if (states.contains(MaterialState.hovered)) {
          return hoveredTrackOutlineWidth;
        }
        if (states.contains(MaterialState.focused)) {
          return focusedTrackOutlineWidth;
        }
        return 8.0;
      }

      final MaterialStateProperty<double> trackOutlineWidth = MaterialStateProperty.resolveWith(
        getTrackOutlineWidth,
      );

      Widget buildSwitch() {
        return MaterialApp(
          home: Material(
            child: Center(
              child: Switch(
                focusNode: focusNode,
                autofocus: true,
                value: true,
                trackOutlineWidth: trackOutlineWidth,
                onChanged: (_) {},
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildSwitch());
      await tester.pumpAndSettle();
      expect(focusNode.hasPrimaryFocus, isTrue);
      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(style: PaintingStyle.fill)
          ..rrect(strokeWidth: focusedTrackOutlineWidth, style: PaintingStyle.stroke),
        reason: 'Active enabled switch track outline width should be 6.0',
      );

      // Start hovering
      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture.addPointer();
      addTearDown(gesture.removePointer);
      await gesture.moveTo(tester.getCenter(find.byType(Switch)));
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect(style: PaintingStyle.fill)
          ..rrect(strokeWidth: hoveredTrackOutlineWidth, style: PaintingStyle.stroke),
        reason: 'Active enabled switch track outline width should be 4.0',
      );

      focusNode.dispose();
    });

    testWidgets('Switch can set icon - M3', (WidgetTester tester) async {
      final ThemeData themeData = ThemeData(
        colorSchemeSeed: const Color(0xff6750a4),
        brightness: Brightness.light,
      );

      MaterialStateProperty<Icon?> thumbIcon(Icon? activeIcon, Icon? inactiveIcon) {
        return MaterialStateProperty.resolveWith<Icon?>((Set<MaterialState> states) {
          if (states.contains(MaterialState.selected)) {
            return activeIcon;
          }
          return inactiveIcon;
        });
      }

      Widget buildSwitch({
        required bool enabled,
        required bool active,
        Icon? activeIcon,
        Icon? inactiveIcon,
      }) {
        return Theme(
          data: themeData,
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Material(
              child: Center(
                child: Switch(
                  thumbIcon: thumbIcon(activeIcon, inactiveIcon),
                  value: active,
                  onChanged: enabled ? (_) {} : null,
                ),
              ),
            ),
          ),
        );
      }

      // active icon shows when switch is on.
      await tester.pumpWidget(
        buildSwitch(enabled: true, active: true, activeIcon: const Icon(Icons.close)),
      );
      await tester.pumpAndSettle();
      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect()
          ..rrect()
          ..paragraph(offset: const Offset(32.0, 16.0)),
      );

      // inactive icon shows when switch is off.
      await tester.pumpWidget(
        buildSwitch(enabled: true, active: false, inactiveIcon: const Icon(Icons.close)),
      );
      await tester.pumpAndSettle();
      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect()
          ..rrect()
          ..rrect()
          ..paragraph(offset: const Offset(12.0, 16.0)),
      );

      // active icon doesn't show when switch is off.
      await tester.pumpWidget(
        buildSwitch(enabled: true, active: false, activeIcon: const Icon(Icons.check)),
      );
      await tester.pumpAndSettle();
      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect()
          ..rrect()
          ..rrect(),
      );

      // inactive icon doesn't show when switch is on.
      await tester.pumpWidget(
        buildSwitch(enabled: true, active: true, inactiveIcon: const Icon(Icons.check)),
      );
      await tester.pumpAndSettle();
      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect()
          ..rrect()
          ..restore(),
      );

      // without icon
      await tester.pumpWidget(buildSwitch(enabled: true, active: false));
      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect()
          ..rrect()
          ..rrect()
          ..restore(),
      );
    });
  });

  testWidgets('Switch.adaptive(Cupertino) is focusable and has correct focus color', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'Switch.adaptive');
    addTearDown(focusNode.dispose);
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    bool value = true;
    const Color focusColor = Color(0xffff0000);

    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        theme: ThemeData(platform: TargetPlatform.iOS),
        home: Material(
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Center(
                child: Switch.adaptive(
                  value: value,
                  onChanged:
                      enabled
                          ? (bool newValue) {
                            setState(() {
                              value = newValue;
                            });
                          }
                          : null,
                  focusColor: focusColor,
                  focusNode: focusNode,
                  autofocus: true,
                ),
              );
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();

    expect(focusNode.hasPrimaryFocus, isTrue);
    expect(
      find.byType(Switch),
      paints
        ..rrect(color: const Color(0xff34c759))
        ..rrect(color: const Color(0x00000000))
        ..rrect(color: focusColor)
        ..clipRRect()
        ..rrect(color: const Color(0x26000000))
        ..rrect(color: const Color(0x0f000000))
        ..rrect(color: const Color(0x0a000000))
        ..rrect(color: const Color(0xffffffff)),
    );

    // Check the false value.
    value = false;
    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();

    expect(focusNode.hasPrimaryFocus, isTrue);
    expect(
      find.byType(Switch),
      paints
        ..rrect(color: const Color(0x28787880))
        ..rrect(color: const Color(0x00000000))
        ..rrect(color: focusColor)
        ..clipRRect()
        ..rrect(color: const Color(0x26000000))
        ..rrect(color: const Color(0x0f000000))
        ..rrect(color: const Color(0x0a000000))
        ..rrect(color: const Color(0xffffffff)),
    );

    // Check what happens when disabled.
    value = false;
    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pumpAndSettle();

    expect(focusNode.hasPrimaryFocus, isFalse);
    expect(
      find.byType(Switch),
      paints
        ..rrect(color: const Color(0x28787880))
        ..clipRRect()
        ..rrect(color: const Color(0x26000000))
        ..rrect(color: const Color(0x0f000000))
        ..rrect(color: const Color(0x0a000000))
        ..rrect(color: const Color(0xffffffff)),
    );
  });

  testWidgets('Switch.onFocusChange callback', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'Switch');
    bool focused = false;
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: Switch(
              value: true,
              focusNode: focusNode,
              onFocusChange: (bool value) {
                focused = value;
              },
              onChanged: (bool newValue) {},
            ),
          ),
        ),
      ),
    );

    focusNode.requestFocus();
    await tester.pump();
    expect(focused, isTrue);
    expect(focusNode.hasFocus, isTrue);

    focusNode.unfocus();
    await tester.pump();
    expect(focused, isFalse);
    expect(focusNode.hasFocus, isFalse);

    focusNode.dispose();
  });

  testWidgets('Switch.padding is respected', (WidgetTester tester) async {
    Widget buildSwitch({EdgeInsets? padding}) {
      return MaterialApp(
        home: Material(
          child: Center(child: Switch(padding: padding, value: true, onChanged: (_) {})),
        ),
      );
    }

    await tester.pumpWidget(buildSwitch());

    expect(tester.getSize(find.byType(Switch)), const Size(60.0, 48.0));

    await tester.pumpWidget(buildSwitch(padding: EdgeInsets.zero));

    expect(tester.getSize(find.byType(Switch)), const Size(52.0, 48.0));

    await tester.pumpWidget(buildSwitch(padding: const EdgeInsets.all(4.0)));

    expect(tester.getSize(find.byType(Switch)), const Size(60.0, 56.0));
  });
}

class DelayedImageProvider extends ImageProvider<DelayedImageProvider> {
  DelayedImageProvider(this.image);

  final ui.Image image;

  final Completer<ImageInfo> _completer = Completer<ImageInfo>();

  @override
  Future<DelayedImageProvider> obtainKey(ImageConfiguration configuration) {
    return SynchronousFuture<DelayedImageProvider>(this);
  }

  @override
  ImageStreamCompleter loadImage(DelayedImageProvider key, ImageDecoderCallback decode) {
    return OneFrameImageStreamCompleter(_completer.future);
  }

  Future<void> complete() async {
    _completer.complete(ImageInfo(image: image));
  }

  @override
  String toString() => '${describeIdentity(this)}()';
}

class _TestImageProvider extends ImageProvider<Object> {
  _TestImageProvider({ImageStreamCompleter? streamCompleter}) {
    _streamCompleter = streamCompleter ?? OneFrameImageStreamCompleter(_completer.future);
  }

  final Completer<ImageInfo> _completer = Completer<ImageInfo>();
  late ImageStreamCompleter _streamCompleter;

  bool get loadCalled => _loadCallCount > 0;
  int get loadCallCount => _loadCallCount;
  int _loadCallCount = 0;

  @override
  Future<Object> obtainKey(ImageConfiguration configuration) {
    return SynchronousFuture<_TestImageProvider>(this);
  }

  @override
  void resolveStreamForKey(
    ImageConfiguration configuration,
    ImageStream stream,
    Object key,
    ImageErrorListener handleError,
  ) {
    super.resolveStreamForKey(configuration, stream, key, handleError);
  }

  @override
  ImageStreamCompleter loadImage(Object key, ImageDecoderCallback decode) {
    _loadCallCount += 1;
    return _streamCompleter;
  }

  void complete(ui.Image image) {
    _completer.complete(ImageInfo(image: image));
  }

  void fail(Object exception, StackTrace? stackTrace) {
    _completer.completeError(exception, stackTrace);
  }

  @override
  String toString() => '${describeIdentity(this)}()';
}

Widget buildAdaptiveSwitch({
  required TargetPlatform platform,
  bool enabled = true,
  bool value = true,
  bool autofocus = false,
  FocusNode? focusNode,
  Color? focusColor,
  SwitchThemeData? overallSwitchThemeData,
  SwitchThemeData? switchThemeData,
  Adaptation<SwitchThemeData>? switchThemeAdaptation,
}) {
  final Widget adaptiveSwitch = Switch.adaptive(
    focusNode: focusNode,
    autofocus: autofocus,
    focusColor: focusColor,
    value: value,
    onChanged: enabled ? (_) {} : null,
  );

  return MaterialApp(
    theme: ThemeData(
      platform: platform,
      switchTheme: overallSwitchThemeData,
      adaptations:
          switchThemeAdaptation == null ? null : <Adaptation<Object>>[switchThemeAdaptation],
    ),
    home: StatefulBuilder(
      builder: (BuildContext context, StateSetter setState) {
        return Material(
          child: Center(
            child:
                switchThemeData == null
                    ? adaptiveSwitch
                    : SwitchTheme(data: switchThemeData, child: adaptiveSwitch),
          ),
        );
      },
    ),
  );
}

class _SwitchThemeAdaptation extends Adaptation<SwitchThemeData> {
  const _SwitchThemeAdaptation();

  @override
  SwitchThemeData adapt(ThemeData theme, SwitchThemeData defaultValue) {
    switch (theme.platform) {
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.windows:
      case TargetPlatform.linux:
        return defaultValue;
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
        return const SwitchThemeData(
          thumbColor: MaterialStatePropertyAll<Color>(Colors.lightGreen),
          trackColor: MaterialStatePropertyAll<Color>(Colors.deepPurple),
        );
    }
  }
}
