// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter/src/gestures/constants.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/semantics_tester.dart';

void main() {
  final ThemeData theme = ThemeData();
  setUp(() {
    debugResetSemanticsIdCounter();
  });

  testWidgets('Checkbox size is configurable by ThemeData.materialTapTargetSize', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      Theme(
        data: theme.copyWith(materialTapTargetSize: MaterialTapTargetSize.padded),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(child: Checkbox(value: true, onChanged: (bool? newValue) {})),
          ),
        ),
      ),
    );

    expect(tester.getSize(find.byType(Checkbox)), const Size(48.0, 48.0));

    await tester.pumpWidget(
      Theme(
        data: theme.copyWith(materialTapTargetSize: MaterialTapTargetSize.shrinkWrap),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(child: Checkbox(value: true, onChanged: (bool? newValue) {})),
          ),
        ),
      ),
    );

    expect(tester.getSize(find.byType(Checkbox)), const Size(40.0, 40.0));
  });

  testWidgets('Checkbox semantics', (WidgetTester tester) async {
    final SemanticsHandle handle = tester.ensureSemantics();

    await tester.pumpWidget(
      Theme(data: theme, child: Material(child: Checkbox(value: false, onChanged: (bool? b) {}))),
    );

    expect(
      tester.getSemantics(find.byType(Focus).last),
      matchesSemantics(
        hasCheckedState: true,
        hasEnabledState: true,
        isEnabled: true,
        hasTapAction: true,
        hasFocusAction: true,
        isFocusable: true,
      ),
    );

    await tester.pumpWidget(
      Theme(data: theme, child: Material(child: Checkbox(value: true, onChanged: (bool? b) {}))),
    );

    expect(
      tester.getSemantics(find.byType(Focus).last),
      matchesSemantics(
        hasCheckedState: true,
        hasEnabledState: true,
        isChecked: true,
        isEnabled: true,
        hasTapAction: true,
        hasFocusAction: true,
        isFocusable: true,
      ),
    );

    await tester.pumpWidget(
      Theme(data: theme, child: const Material(child: Checkbox(value: false, onChanged: null))),
    );

    expect(
      tester.getSemantics(find.byType(Checkbox)),
      matchesSemantics(
        hasCheckedState: true,
        hasEnabledState: true,
        // isFocusable is delayed by 1 frame.
        isFocusable: true,
        hasFocusAction: true,
      ),
    );

    await tester.pump();
    // isFocusable should be false now after the 1 frame delay.
    expect(
      tester.getSemantics(find.byType(Checkbox)),
      matchesSemantics(hasCheckedState: true, hasEnabledState: true),
    );

    await tester.pumpWidget(
      Theme(data: theme, child: const Material(child: Checkbox(value: true, onChanged: null))),
    );

    expect(
      tester.getSemantics(find.byType(Checkbox)),
      matchesSemantics(hasCheckedState: true, hasEnabledState: true, isChecked: true),
    );

    await tester.pumpWidget(
      Theme(
        data: theme,
        child: const Material(child: Checkbox(value: null, tristate: true, onChanged: null)),
      ),
    );

    expect(
      tester.getSemantics(find.byType(Checkbox)),
      matchesSemantics(hasCheckedState: true, hasEnabledState: true, isCheckStateMixed: true),
    );

    await tester.pumpWidget(
      Theme(
        data: theme,
        child: const Material(child: Checkbox(value: true, tristate: true, onChanged: null)),
      ),
    );

    expect(
      tester.getSemantics(find.byType(Checkbox)),
      matchesSemantics(hasCheckedState: true, hasEnabledState: true, isChecked: true),
    );

    await tester.pumpWidget(
      Theme(
        data: theme,
        child: const Material(child: Checkbox(value: false, tristate: true, onChanged: null)),
      ),
    );

    expect(
      tester.getSemantics(find.byType(Checkbox)),
      matchesSemantics(hasCheckedState: true, hasEnabledState: true),
    );

    // Check if semanticLabel is there.
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Theme(
          data: theme,
          child: Material(
            child: Checkbox(semanticLabel: 'checkbox', value: true, onChanged: (bool? b) {}),
          ),
        ),
      ),
    );

    expect(
      tester.getSemantics(find.byType(Focus).last),
      matchesSemantics(
        label: 'checkbox',
        textDirection: TextDirection.ltr,
        hasCheckedState: true,
        hasEnabledState: true,
        isChecked: true,
        isEnabled: true,
        hasTapAction: true,
        hasFocusAction: true,
        isFocusable: true,
      ),
    );
    handle.dispose();
  });

  testWidgets('Can wrap Checkbox with Semantics', (WidgetTester tester) async {
    final SemanticsHandle handle = tester.ensureSemantics();

    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Material(
          child: Semantics(
            label: 'foo',
            textDirection: TextDirection.ltr,
            child: Checkbox(value: false, onChanged: (bool? b) {}),
          ),
        ),
      ),
    );

    expect(
      tester.getSemantics(find.byType(Focus).last),
      matchesSemantics(
        label: 'foo',
        textDirection: TextDirection.ltr,
        hasCheckedState: true,
        hasEnabledState: true,
        isEnabled: true,
        hasTapAction: true,
        hasFocusAction: true,
        isFocusable: true,
      ),
    );
    handle.dispose();
  });

  testWidgets('Checkbox tristate: true', (WidgetTester tester) async {
    bool? checkBoxValue;

    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Material(
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Checkbox(
                tristate: true,
                value: checkBoxValue,
                onChanged: (bool? value) {
                  setState(() {
                    checkBoxValue = value;
                  });
                },
              );
            },
          ),
        ),
      ),
    );

    expect(tester.widget<Checkbox>(find.byType(Checkbox)).value, null);

    await tester.tap(find.byType(Checkbox));
    await tester.pumpAndSettle();
    expect(checkBoxValue, false);

    await tester.tap(find.byType(Checkbox));
    await tester.pumpAndSettle();
    expect(checkBoxValue, true);

    await tester.tap(find.byType(Checkbox));
    await tester.pumpAndSettle();
    expect(checkBoxValue, null);

    checkBoxValue = true;
    await tester.pumpAndSettle();
    expect(checkBoxValue, true);

    checkBoxValue = null;
    await tester.pumpAndSettle();
    expect(checkBoxValue, null);
  });

  testWidgets('has semantics for tristate', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Material(
          child: Checkbox(tristate: true, value: null, onChanged: (bool? newValue) {}),
        ),
      ),
    );

    expect(
      semantics.nodesWith(
        flags: <SemanticsFlag>[
          SemanticsFlag.hasCheckedState,
          SemanticsFlag.hasEnabledState,
          SemanticsFlag.isEnabled,
          SemanticsFlag.isFocusable,
          SemanticsFlag.isCheckStateMixed,
        ],
        actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
      ),
      hasLength(1),
    );

    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Material(
          child: Checkbox(tristate: true, value: true, onChanged: (bool? newValue) {}),
        ),
      ),
    );

    expect(
      semantics.nodesWith(
        flags: <SemanticsFlag>[
          SemanticsFlag.hasCheckedState,
          SemanticsFlag.hasEnabledState,
          SemanticsFlag.isEnabled,
          SemanticsFlag.isChecked,
          SemanticsFlag.isFocusable,
        ],
        actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
      ),
      hasLength(1),
    );

    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Material(
          child: Checkbox(tristate: true, value: false, onChanged: (bool? newValue) {}),
        ),
      ),
    );

    expect(
      semantics.nodesWith(
        flags: <SemanticsFlag>[
          SemanticsFlag.hasCheckedState,
          SemanticsFlag.hasEnabledState,
          SemanticsFlag.isEnabled,
          SemanticsFlag.isFocusable,
        ],
        actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
      ),
      hasLength(1),
    );

    semantics.dispose();
  });

  testWidgets('has semantic events', (WidgetTester tester) async {
    dynamic semanticEvent;
    bool? checkboxValue = false;
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
        child: Material(
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Checkbox(
                value: checkboxValue,
                onChanged: (bool? value) {
                  setState(() {
                    checkboxValue = value;
                  });
                },
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.byType(Checkbox));
    final RenderObject object = tester.firstRenderObject(find.byType(Checkbox));

    expect(checkboxValue, true);
    expect(semanticEvent, <String, dynamic>{
      'type': 'tap',
      'nodeId': object.debugSemantics!.id,
      'data': <String, dynamic>{},
    });
    expect(object.debugSemantics!.getSemanticsData().hasAction(SemanticsAction.tap), true);

    tester.binding.defaultBinaryMessenger.setMockDecodedMessageHandler<dynamic>(
      SystemChannels.accessibility,
      null,
    );
    semanticsTester.dispose();
  });

  testWidgets('Material2 - Checkbox tristate rendering, programmatic transitions', (
    WidgetTester tester,
  ) async {
    final ThemeData theme = ThemeData(useMaterial3: false);
    Widget buildFrame(bool? checkboxValue) {
      return Theme(
        data: theme,
        child: Material(
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Checkbox(tristate: true, value: checkboxValue, onChanged: (bool? value) {});
            },
          ),
        ),
      );
    }

    RenderBox getCheckboxRenderer() {
      return tester.renderObject<RenderBox>(find.byType(Checkbox));
    }

    await tester.pumpWidget(buildFrame(false));
    await tester.pumpAndSettle();
    expect(
      getCheckboxRenderer(),
      paints..path(color: Colors.transparent),
    ); // paint transparent border
    expect(getCheckboxRenderer(), isNot(paints..line())); // null is rendered as a line (a "dash")
    expect(getCheckboxRenderer(), paints..drrect()); // empty checkbox

    await tester.pumpWidget(buildFrame(true));
    await tester.pumpAndSettle();
    expect(
      getCheckboxRenderer(),
      paints
        ..path(color: theme.colorScheme.secondary)
        ..path(color: const Color(0xFFFFFFFF)),
    ); // checkmark is rendered as a path

    await tester.pumpWidget(buildFrame(false));
    await tester.pumpAndSettle();
    expect(
      getCheckboxRenderer(),
      paints..path(color: Colors.transparent),
    ); // paint transparent border
    expect(getCheckboxRenderer(), isNot(paints..line())); // null is rendered as a line (a "dash")
    expect(getCheckboxRenderer(), paints..drrect()); // empty checkbox

    await tester.pumpWidget(buildFrame(null));
    await tester.pumpAndSettle();
    expect(getCheckboxRenderer(), paints..line()); // null is rendered as a line (a "dash")

    await tester.pumpWidget(buildFrame(true));
    await tester.pumpAndSettle();
    expect(
      getCheckboxRenderer(),
      paints
        ..path(color: theme.colorScheme.secondary)
        ..path(color: const Color(0xFFFFFFFF)),
    ); // checkmark is rendered as a path

    await tester.pumpWidget(buildFrame(null));
    await tester.pumpAndSettle();
    expect(getCheckboxRenderer(), paints..line()); // null is rendered as a line (a "dash")
  });

  testWidgets('Material3 - Checkbox tristate rendering, programmatic transitions', (
    WidgetTester tester,
  ) async {
    final ThemeData theme = ThemeData();
    Widget buildFrame(bool? checkboxValue) {
      return Theme(
        data: theme,
        child: Material(
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Checkbox(tristate: true, value: checkboxValue, onChanged: (bool? value) {});
            },
          ),
        ),
      );
    }

    RenderBox getCheckboxRenderer() {
      return tester.renderObject<RenderBox>(find.byType(Checkbox));
    }

    await tester.pumpWidget(buildFrame(false));
    await tester.pumpAndSettle();
    expect(
      getCheckboxRenderer(),
      paints..path(color: Colors.transparent),
    ); // paint transparent border
    expect(getCheckboxRenderer(), isNot(paints..line())); // null is rendered as a line (a "dash")
    expect(getCheckboxRenderer(), paints..drrect()); // empty checkbox

    await tester.pumpWidget(buildFrame(true));
    await tester.pumpAndSettle();
    expect(
      getCheckboxRenderer(),
      paints
        ..path(color: theme.colorScheme.primary)
        ..path(color: theme.colorScheme.onPrimary),
    ); // checkmark is rendered as a path

    await tester.pumpWidget(buildFrame(false));
    await tester.pumpAndSettle();
    expect(
      getCheckboxRenderer(),
      paints..path(color: Colors.transparent),
    ); // paint transparent border
    expect(getCheckboxRenderer(), isNot(paints..line())); // null is rendered as a line (a "dash")
    expect(getCheckboxRenderer(), paints..drrect()); // empty checkbox

    await tester.pumpWidget(buildFrame(null));
    await tester.pumpAndSettle();
    expect(getCheckboxRenderer(), paints..line()); // null is rendered as a line (a "dash")

    await tester.pumpWidget(buildFrame(true));
    await tester.pumpAndSettle();
    expect(
      getCheckboxRenderer(),
      paints
        ..path(color: theme.colorScheme.primary)
        ..path(color: theme.colorScheme.onPrimary),
    ); // checkmark is rendered as a path

    await tester.pumpWidget(buildFrame(null));
    await tester.pumpAndSettle();
    expect(getCheckboxRenderer(), paints..line()); // null is rendered as a line (a "dash")
  });

  testWidgets('Material2 - Checkbox color rendering', (WidgetTester tester) async {
    ThemeData theme = ThemeData(useMaterial3: false);
    const Color borderColor = Color(0xff2196f3);
    Color checkColor = const Color(0xffFFFFFF);
    Color activeColor;

    Widget buildFrame({Color? activeColor, Color? checkColor, ThemeData? themeData}) {
      return Material(
        child: Theme(
          data: themeData ?? theme,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Checkbox(
                value: true,
                activeColor: activeColor,
                checkColor: checkColor,
                onChanged: (bool? value) {},
              );
            },
          ),
        ),
      );
    }

    RenderBox getCheckboxRenderer() {
      return tester.renderObject<RenderBox>(find.byType(Checkbox));
    }

    await tester.pumpWidget(buildFrame(checkColor: checkColor));
    await tester.pumpAndSettle();
    expect(
      getCheckboxRenderer(),
      paints
        ..path(color: borderColor)
        ..path(color: checkColor),
    ); // paints's color is 0xFFFFFFFF (default color)

    checkColor = const Color(0xFF000000);

    await tester.pumpWidget(buildFrame(checkColor: checkColor));
    await tester.pumpAndSettle();
    expect(
      getCheckboxRenderer(),
      paints
        ..path(color: borderColor)
        ..path(color: checkColor),
    ); // paints's color is 0xFF000000 (params)

    activeColor = const Color(0xFF00FF00);

    final ColorScheme colorScheme = const ColorScheme.light().copyWith(secondary: activeColor);
    theme = theme.copyWith(colorScheme: colorScheme);
    await tester.pumpWidget(buildFrame(themeData: theme));
    await tester.pumpAndSettle();
    expect(
      getCheckboxRenderer(),
      paints..path(color: activeColor),
    ); // paints's color is 0xFF00FF00 (theme)

    activeColor = const Color(0xFF000000);

    await tester.pumpWidget(buildFrame(activeColor: activeColor));
    await tester.pumpAndSettle();
    expect(getCheckboxRenderer(), paints..path(color: activeColor));
  });

  testWidgets('Material3 - Checkbox color rendering', (WidgetTester tester) async {
    ThemeData theme = ThemeData();
    const Color borderColor = Color(0xFF6750A4);
    Color checkColor = const Color(0xffFFFFFF);
    Color activeColor;

    Widget buildFrame({Color? activeColor, Color? checkColor, ThemeData? themeData}) {
      return Material(
        child: Theme(
          data: themeData ?? theme,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Checkbox(
                value: true,
                activeColor: activeColor,
                checkColor: checkColor,
                onChanged: (bool? value) {},
              );
            },
          ),
        ),
      );
    }

    RenderBox getCheckboxRenderer() {
      return tester.renderObject<RenderBox>(find.byType(Checkbox));
    }

    await tester.pumpWidget(buildFrame(checkColor: checkColor));
    await tester.pumpAndSettle();
    expect(
      getCheckboxRenderer(),
      paints
        ..path(color: borderColor)
        ..path(color: checkColor),
    ); // paints's color is 0xFFFFFFFF (default color)

    checkColor = const Color(0xFF000000);

    await tester.pumpWidget(buildFrame(checkColor: checkColor));
    await tester.pumpAndSettle();
    expect(
      getCheckboxRenderer(),
      paints
        ..path(color: borderColor)
        ..path(color: checkColor),
    ); // paints's color is 0xFF000000 (params)

    activeColor = const Color(0xFF00FF00);

    final ColorScheme colorScheme = const ColorScheme.light().copyWith(primary: activeColor);
    theme = theme.copyWith(colorScheme: colorScheme);
    await tester.pumpWidget(buildFrame(themeData: theme));
    await tester.pumpAndSettle();
    expect(
      getCheckboxRenderer(),
      paints..path(color: activeColor),
    ); // paints's color is 0xFF00FF00 (theme)

    activeColor = const Color(0xFF000000);

    await tester.pumpWidget(buildFrame(activeColor: activeColor));
    await tester.pumpAndSettle();
    expect(getCheckboxRenderer(), paints..path(color: activeColor));
  });

  testWidgets('Material2 - Checkbox is focusable and has correct focus color', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'Checkbox');
    addTearDown(focusNode.dispose);
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    bool? value = true;
    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Checkbox(
                  value: value,
                  onChanged:
                      enabled
                          ? (bool? newValue) {
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
      Material.of(tester.element(find.byType(Checkbox))),
      paints
        ..circle(color: Colors.orange[500])
        ..path(color: const Color(0xff2196f3))
        ..path(color: Colors.white),
    );

    // Check the false value.
    value = false;
    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isTrue);
    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints
        ..circle(color: Colors.orange[500])
        ..drrect(
          color: const Color(0x8a000000),
          outer: RRect.fromLTRBR(15.0, 15.0, 33.0, 33.0, const Radius.circular(1.0)),
          inner: RRect.fromLTRBR(17.0, 17.0, 31.0, 31.0, Radius.zero),
        ),
    );

    // Check what happens when disabled.
    value = false;
    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isFalse);
    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints..drrect(
        color: const Color(0x61000000),
        outer: RRect.fromLTRBR(15.0, 15.0, 33.0, 33.0, const Radius.circular(1.0)),
        inner: RRect.fromLTRBR(17.0, 17.0, 31.0, 31.0, Radius.zero),
      ),
    );
  });

  testWidgets('Material3 - Checkbox is focusable and has correct focus color', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'Checkbox');
    addTearDown(focusNode.dispose);
    final ThemeData theme = ThemeData();
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    bool? value = true;
    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Checkbox(
                  value: value,
                  onChanged:
                      enabled
                          ? (bool? newValue) {
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
      Material.of(tester.element(find.byType(Checkbox))),
      paints
        ..circle(color: Colors.orange[500])
        ..path(color: theme.colorScheme.primary)
        ..path(color: theme.colorScheme.onPrimary),
    );

    // Check the false value.
    value = false;
    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isTrue);
    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints
        ..circle(color: Colors.orange[500])
        ..drrect(
          color: theme.colorScheme.onSurface,
          outer: RRect.fromLTRBR(15.0, 15.0, 33.0, 33.0, const Radius.circular(2.0)),
          inner: RRect.fromLTRBR(17.0, 17.0, 31.0, 31.0, Radius.zero),
        ),
    );

    // Check what happens when disabled.
    value = false;
    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isFalse);
    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints..drrect(
        color: theme.colorScheme.onSurface.withOpacity(0.38),
        outer: RRect.fromLTRBR(15.0, 15.0, 33.0, 33.0, const Radius.circular(2.0)),
        inner: RRect.fromLTRBR(17.0, 17.0, 31.0, 31.0, Radius.zero),
      ),
    );
  });

  testWidgets('Checkbox with splash radius set', (WidgetTester tester) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    const double splashRadius = 30;
    Widget buildApp() {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Checkbox(
                  value: false,
                  onChanged: (bool? newValue) {},
                  focusColor: Colors.orange[500],
                  autofocus: true,
                  splashRadius: splashRadius,
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
      Material.of(tester.element(find.byType(Checkbox))),
      paints..circle(color: Colors.orange[500], radius: splashRadius),
    );
  });

  testWidgets('Checkbox starts the splash in center, even when tap is on the corner', (
    WidgetTester tester,
  ) async {
    Widget buildApp() {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Checkbox(value: false, onChanged: (bool? newValue) {});
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildApp());
    final Offset checkboxTopLeftGlobal = tester.getTopLeft(find.byType(Checkbox));
    final Offset checkboxCenterGlobal = tester.getCenter(find.byType(Checkbox));
    final Offset checkboxCenterLocal = checkboxCenterGlobal - checkboxTopLeftGlobal;
    final TestGesture gesture = await tester.startGesture(checkboxTopLeftGlobal);
    await tester.pump();
    // Wait for the splash to be drawn, but not long enough for it to animate towards the center, since
    // we want to catch it in its starting position.
    await tester.pump(const Duration(milliseconds: 1));
    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints..circle(x: checkboxCenterLocal.dx, y: checkboxCenterLocal.dy),
    );

    // Finish gesture to release resources.
    await gesture.up();
    await tester.pumpAndSettle();
  });

  testWidgets('Material2 - Checkbox can be hovered and has correct hover color', (
    WidgetTester tester,
  ) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    bool? value = true;
    final ThemeData theme = ThemeData(useMaterial3: false);
    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Checkbox(
                  value: value,
                  onChanged:
                      enabled
                          ? (bool? newValue) {
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
      Material.of(tester.element(find.byType(Checkbox))),
      paints
        ..path(color: const Color(0xff2196f3))
        ..path(color: const Color(0xffffffff), style: PaintingStyle.stroke, strokeWidth: 2.0),
    );

    // Start hovering
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    addTearDown(gesture.removePointer);
    await gesture.moveTo(tester.getCenter(find.byType(Checkbox)));

    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints
        ..circle(color: Colors.orange[500])
        ..path(color: const Color(0xff2196f3))
        ..path(color: const Color(0xffffffff), style: PaintingStyle.stroke, strokeWidth: 2.0),
    );

    // Check what happens when disabled.
    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints
        ..path(color: const Color(0x61000000))
        ..path(color: const Color(0xffffffff), style: PaintingStyle.stroke, strokeWidth: 2.0),
    );
  });

  testWidgets('Material3 - Checkbox can be hovered and has correct hover color', (
    WidgetTester tester,
  ) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    bool? value = true;
    final ThemeData theme = ThemeData();
    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Checkbox(
                  value: value,
                  onChanged:
                      enabled
                          ? (bool? newValue) {
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
      Material.of(tester.element(find.byType(Checkbox))),
      paints
        ..path(color: const Color(0xff6750a4))
        ..path(color: theme.colorScheme.onPrimary, style: PaintingStyle.stroke, strokeWidth: 2.0),
    );

    // Start hovering
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    addTearDown(gesture.removePointer);
    await gesture.moveTo(tester.getCenter(find.byType(Checkbox)));

    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints
        ..circle(color: Colors.orange[500])
        ..path(color: const Color(0xff6750a4))
        ..path(color: theme.colorScheme.onPrimary, style: PaintingStyle.stroke, strokeWidth: 2.0),
    );

    // Check what happens when disabled.
    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints
        ..path(color: theme.colorScheme.onSurface.withOpacity(0.38))
        ..path(color: theme.colorScheme.surface, style: PaintingStyle.stroke, strokeWidth: 2.0),
    );
  });

  testWidgets('Checkbox can be toggled by keyboard shortcuts', (WidgetTester tester) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    bool? value = true;
    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Checkbox(
                  value: value,
                  onChanged:
                      enabled
                          ? (bool? newValue) {
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
    // On web, checkboxes don't respond to the enter key.
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

  testWidgets(
    'Material3 - Checkbox visual density cannot be overriden by ThemeData.visualDensity',
    (WidgetTester tester) async {
      const Key key = Key('test');
      Widget buldCheckbox() {
        return MaterialApp(
          theme: theme.copyWith(visualDensity: VisualDensity.compact),
          home: Material(
            child: Center(child: Checkbox(key: key, value: true, onChanged: (bool? value) {})),
          ),
        );
      }

      await tester.pumpWidget(buldCheckbox());
      await tester.pumpAndSettle();
      final RenderBox box = tester.renderObject(find.byKey(key));
      expect(box.size, equals(const Size(48, 48)));
    },
  );

  testWidgets(
    'Material3 - Checkbox with MaterialTapTargetSize.padded meets Material Guidelines on desktop',
    (WidgetTester tester) async {
      const Key key = Key('test');
      Widget buldCheckbox() {
        return MaterialApp(
          theme: theme,
          home: Material(
            child: Center(
              child: Checkbox(
                key: key,
                materialTapTargetSize: MaterialTapTargetSize.padded,
                value: true,
                onChanged: (bool? value) {},
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buldCheckbox());
      await tester.pumpAndSettle();
      final RenderBox box = tester.renderObject(find.byKey(key));
      expect(box.size, equals(const Size(48, 48)));
    },
    variant: TargetPlatformVariant.desktop(),
  );

  testWidgets('Checkbox responds to density changes', (WidgetTester tester) async {
    const Key key = Key('test');
    Future<void> buildTest({VisualDensity? visualDensity}) async {
      return tester.pumpWidget(
        MaterialApp(
          theme: theme,
          home: Material(
            child: Center(
              child: Checkbox(
                visualDensity: visualDensity,
                key: key,
                onChanged: (bool? value) {},
                value: true,
              ),
            ),
          ),
        ),
      );
    }

    // Test the default visual density.
    await buildTest();
    await tester.pumpAndSettle();
    final RenderBox box = tester.renderObject(find.byKey(key));
    expect(box.size, equals(const Size(48, 48)));

    await buildTest(visualDensity: VisualDensity.standard);
    await tester.pumpAndSettle();
    expect(box.size, equals(const Size(48, 48)));

    await buildTest(visualDensity: VisualDensity.compact);
    await tester.pumpAndSettle();
    expect(box.size, equals(const Size(40, 40)));

    await buildTest(visualDensity: const VisualDensity(horizontal: 3.0, vertical: 3.0));
    await tester.pumpAndSettle();
    expect(box.size, equals(const Size(60, 60)));

    await buildTest(visualDensity: const VisualDensity(horizontal: -3.0, vertical: -3.0));
    await tester.pumpAndSettle();
    expect(box.size, equals(const Size(36, 36)));

    await buildTest(visualDensity: const VisualDensity(horizontal: 3.0, vertical: -3.0));
    await tester.pumpAndSettle();
    expect(box.size, equals(const Size(60, 36)));
  });

  testWidgets('Checkbox stops hover animation when removed from the tree.', (
    WidgetTester tester,
  ) async {
    const Key checkboxKey = Key('checkbox');
    bool? checkboxVal = true;

    await tester.pumpWidget(
      Theme(
        data: ThemeData(materialTapTargetSize: MaterialTapTargetSize.padded),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: StatefulBuilder(
                builder:
                    (_, StateSetter setState) => Checkbox(
                      key: checkboxKey,
                      value: checkboxVal,
                      onChanged:
                          (bool? newValue) => setState(() {
                            checkboxVal = newValue;
                          }),
                    ),
              ),
            ),
          ),
        ),
      ),
    );

    expect(find.byKey(checkboxKey), findsOneWidget);
    final Offset checkboxCenter = tester.getCenter(find.byKey(checkboxKey));
    final TestGesture testGesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await testGesture.moveTo(checkboxCenter);

    await tester.pump(); // start animation
    await tester.pump(
      const Duration(milliseconds: 25),
    ); // hover animation duration is 50 ms. It is half-way.

    await tester.pumpWidget(
      Theme(
        data: ThemeData(materialTapTargetSize: MaterialTapTargetSize.padded),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(child: Center(child: Container())),
        ),
      ),
    );

    // Hover animation should not trigger an exception when the checkbox is removed
    // before the hover animation should complete.
    expect(tester.takeException(), isNull);

    await testGesture.removePointer();
  });

  testWidgets('Checkbox changes mouse cursor when hovered', (WidgetTester tester) async {
    // Test Checkbox() constructor
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Scaffold(
          body: Align(
            alignment: Alignment.topLeft,
            child: Material(
              child: MouseRegion(
                cursor: SystemMouseCursors.forbidden,
                child: Checkbox(
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
    await gesture.addPointer(location: tester.getCenter(find.byType(Checkbox)));
    addTearDown(gesture.removePointer);

    await tester.pump();

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
                child: Checkbox(value: true, onChanged: (_) {}),
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
                child: Checkbox(value: true, onChanged: null),
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

    // Test cursor when tristate
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: const Scaffold(
          body: Align(
            alignment: Alignment.topLeft,
            child: Material(
              child: MouseRegion(
                cursor: SystemMouseCursors.forbidden,
                child: Checkbox(
                  value: null,
                  tristate: true,
                  onChanged: null,
                  mouseCursor: _SelectedGrabMouseCursor(),
                ),
              ),
            ),
          ),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.grab,
    );

    await tester.pumpAndSettle();
  });

  testWidgets('Checkbox fill color resolves in enabled/disabled states', (
    WidgetTester tester,
  ) async {
    const Color activeEnabledFillColor = Color(0xFF000001);
    const Color activeDisabledFillColor = Color(0xFF000002);

    Color getFillColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return activeDisabledFillColor;
      }
      return activeEnabledFillColor;
    }

    final MaterialStateProperty<Color> fillColor = MaterialStateColor.resolveWith(getFillColor);

    Widget buildFrame({required bool enabled}) {
      return Material(
        child: Theme(
          data: theme,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Checkbox(
                value: true,
                fillColor: fillColor,
                onChanged: enabled ? (bool? value) {} : null,
              );
            },
          ),
        ),
      );
    }

    RenderBox getCheckboxRenderer() {
      return tester.renderObject<RenderBox>(find.byType(Checkbox));
    }

    await tester.pumpWidget(buildFrame(enabled: true));
    await tester.pumpAndSettle();
    expect(getCheckboxRenderer(), paints..path(color: activeEnabledFillColor));

    await tester.pumpWidget(buildFrame(enabled: false));
    await tester.pumpAndSettle();
    expect(getCheckboxRenderer(), paints..path(color: activeDisabledFillColor));
  });

  testWidgets('Checkbox fill color resolves in hovered/focused states', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'checkbox');
    addTearDown(focusNode.dispose);

    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    const Color hoveredFillColor = Color(0xFF000001);
    const Color focusedFillColor = Color(0xFF000002);

    Color getFillColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.hovered)) {
        return hoveredFillColor;
      }
      if (states.contains(MaterialState.focused)) {
        return focusedFillColor;
      }
      return Colors.transparent;
    }

    final MaterialStateProperty<Color> fillColor = MaterialStateColor.resolveWith(getFillColor);

    Widget buildFrame() {
      return Material(
        child: Theme(
          data: theme,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Checkbox(
                focusNode: focusNode,
                autofocus: true,
                value: true,
                fillColor: fillColor,
                onChanged: (bool? value) {},
              );
            },
          ),
        ),
      );
    }

    RenderBox getCheckboxRenderer() {
      return tester.renderObject<RenderBox>(find.byType(Checkbox));
    }

    await tester.pumpWidget(buildFrame());
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isTrue);
    expect(getCheckboxRenderer(), paints..path(color: focusedFillColor));

    // Start hovering
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    addTearDown(gesture.removePointer);
    await gesture.moveTo(tester.getCenter(find.byType(Checkbox)));
    await tester.pumpAndSettle();

    expect(getCheckboxRenderer(), paints..path(color: hoveredFillColor));
  });

  testWidgets('Checkbox respects shape and side', (WidgetTester tester) async {
    const RoundedRectangleBorder roundedRectangleBorder = RoundedRectangleBorder(
      borderRadius: BorderRadius.all(Radius.circular(5)),
    );

    const BorderSide side = BorderSide(width: 4, color: Color(0xfff44336));

    Widget buildApp() {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Checkbox(
                  value: false,
                  onChanged: (bool? newValue) {},
                  shape: roundedRectangleBorder,
                  side: side,
                );
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();

    expect(tester.widget<Checkbox>(find.byType(Checkbox)).shape, roundedRectangleBorder);
    expect(tester.widget<Checkbox>(find.byType(Checkbox)).side, side);
    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints..drrect(
        color: const Color(0xfff44336),
        outer: RRect.fromLTRBR(15.0, 15.0, 33.0, 33.0, const Radius.circular(5)),
        inner: RRect.fromLTRBR(19.0, 19.0, 29.0, 29.0, const Radius.circular(1)),
      ),
    );
  });

  testWidgets(
    'Material2 - Checkbox default overlay color in active/pressed/focused/hovered states',
    (WidgetTester tester) async {
      final FocusNode focusNode = FocusNode(debugLabel: 'Checkbox');
      addTearDown(focusNode.dispose);
      tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;

      final ThemeData theme = ThemeData(useMaterial3: false);
      final ColorScheme colors = theme.colorScheme;
      Widget buildCheckbox({bool active = false, bool focused = false}) {
        return MaterialApp(
          theme: theme,
          home: Scaffold(
            body: Checkbox(
              focusNode: focusNode,
              autofocus: focused,
              value: active,
              onChanged: (_) {},
            ),
          ),
        );
      }

      await tester.pumpWidget(buildCheckbox());
      final TestGesture gesture1 = await tester.startGesture(
        tester.getCenter(find.byType(Checkbox)),
      );
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Checkbox))),
        paints..circle(color: theme.unselectedWidgetColor.withAlpha(kRadialReactionAlpha)),
        reason:
            'Default inactive pressed Checkbox should have overlay color from default fillColor',
      );

      await tester.pumpWidget(buildCheckbox(active: true));
      final TestGesture gesture2 = await tester.startGesture(
        tester.getCenter(find.byType(Checkbox)),
      );
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Checkbox))),
        paints..circle(color: colors.secondary.withAlpha(kRadialReactionAlpha)),
        reason: 'Default active pressed Checkbox should have overlay color from default fillColor',
      );

      await tester.pumpWidget(Container()); // reset test
      await tester.pumpWidget(buildCheckbox(focused: true));
      await tester.pumpAndSettle();

      expect(focusNode.hasPrimaryFocus, isTrue);
      expect(
        Material.of(tester.element(find.byType(Checkbox))),
        paints..circle(color: theme.focusColor),
        reason: 'Focused Checkbox should use default focused overlay color',
      );

      await tester.pumpWidget(Container()); // reset test
      await tester.pumpWidget(buildCheckbox());
      final TestGesture gesture3 = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture3.addPointer();
      addTearDown(gesture3.removePointer);
      await gesture3.moveTo(tester.getCenter(find.byType(Checkbox)));
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Checkbox))),
        paints..circle(color: theme.hoverColor),
        reason: 'Hovered Checkbox should use default hovered overlay color',
      );

      // Finish gestures to release resources.
      await gesture1.up();
      await gesture2.up();
      await tester.pumpAndSettle();
    },
  );

  testWidgets(
    'Material3 - Checkbox default overlay color in active/pressed/focused/hovered states',
    (WidgetTester tester) async {
      final FocusNode focusNode = FocusNode(debugLabel: 'Checkbox');
      addTearDown(focusNode.dispose);
      tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;

      final ThemeData theme = ThemeData();
      final ColorScheme colors = theme.colorScheme;
      Widget buildCheckbox({bool active = false, bool focused = false}) {
        return MaterialApp(
          theme: theme,
          home: Scaffold(
            body: Checkbox(
              focusNode: focusNode,
              autofocus: focused,
              value: active,
              onChanged: (_) {},
            ),
          ),
        );
      }

      await tester.pumpWidget(buildCheckbox());
      final TestGesture gesture1 = await tester.startGesture(
        tester.getCenter(find.byType(Checkbox)),
      );
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Checkbox))),
        paints..circle(color: colors.primary.withOpacity(0.1)),
        reason:
            'Default inactive pressed Checkbox should have overlay color from default fillColor',
      );

      await tester.pumpWidget(buildCheckbox(active: true));
      final TestGesture gesture2 = await tester.startGesture(
        tester.getCenter(find.byType(Checkbox)),
      );
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Checkbox))),
        paints..circle(color: colors.onSurface.withOpacity(0.1)),
        reason: 'Default active pressed Checkbox should have overlay color from default fillColor',
      );

      await tester.pumpWidget(Container()); // reset test
      await tester.pumpWidget(buildCheckbox(focused: true));
      await tester.pumpAndSettle();

      expect(focusNode.hasPrimaryFocus, isTrue);
      expect(
        Material.of(tester.element(find.byType(Checkbox))),
        paints..circle(color: colors.onSurface.withOpacity(0.1)),
        reason: 'Focused Checkbox should use default focused overlay color',
      );

      await tester.pumpWidget(Container()); // reset test
      await tester.pumpWidget(buildCheckbox());
      final TestGesture gesture3 = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture3.addPointer();
      addTearDown(gesture3.removePointer);
      await gesture3.moveTo(tester.getCenter(find.byType(Checkbox)));
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Checkbox))),
        paints..circle(color: colors.onSurface.withOpacity(0.08)),
        reason: 'Hovered Checkbox should use default hovered overlay color',
      );

      // Finish gestures to release resources.
      await gesture1.up();
      await gesture2.up();
      await tester.pumpAndSettle();
    },
  );

  testWidgets('Checkbox overlay color resolves in active/pressed/focused/hovered states', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'Checkbox');
    addTearDown(focusNode.dispose);
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;

    const Color fillColor = Color(0xFF000000);
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

    Widget buildCheckbox({bool active = false, bool focused = false, bool useOverlay = true}) {
      return MaterialApp(
        theme: theme,
        home: Scaffold(
          body: Checkbox(
            focusNode: focusNode,
            autofocus: focused,
            value: active,
            onChanged: (_) {},
            fillColor: const MaterialStatePropertyAll<Color>(fillColor),
            overlayColor: useOverlay ? MaterialStateProperty.resolveWith(getOverlayColor) : null,
            hoverColor: hoverColor,
            focusColor: focusColor,
            splashRadius: splashRadius,
          ),
        ),
      );
    }

    await tester.pumpWidget(buildCheckbox(useOverlay: false));
    final TestGesture gesture1 = await tester.startGesture(tester.getCenter(find.byType(Checkbox)));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints..circle(color: fillColor.withAlpha(kRadialReactionAlpha), radius: splashRadius),
      reason: 'Default inactive pressed Checkbox should have overlay color from fillColor',
    );

    await tester.pumpWidget(buildCheckbox(active: true, useOverlay: false));
    final TestGesture gesture2 = await tester.startGesture(tester.getCenter(find.byType(Checkbox)));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints..circle(color: fillColor.withAlpha(kRadialReactionAlpha), radius: splashRadius),
      reason: 'Default active pressed Checkbox should have overlay color from fillColor',
    );

    await tester.pumpWidget(buildCheckbox());
    final TestGesture gesture3 = await tester.startGesture(tester.getCenter(find.byType(Checkbox)));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints..circle(color: inactivePressedOverlayColor, radius: splashRadius),
      reason: 'Inactive pressed Checkbox should have overlay color: $inactivePressedOverlayColor',
    );

    await tester.pumpWidget(buildCheckbox(active: true));
    final TestGesture gesture4 = await tester.startGesture(tester.getCenter(find.byType(Checkbox)));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints..circle(color: activePressedOverlayColor, radius: splashRadius),
      reason: 'Active pressed Checkbox should have overlay color: $activePressedOverlayColor',
    );

    await tester.pumpWidget(Container()); // reset test
    await tester.pumpWidget(buildCheckbox(focused: true));
    await tester.pumpAndSettle();

    expect(focusNode.hasPrimaryFocus, isTrue);
    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints..circle(color: focusOverlayColor, radius: splashRadius),
      reason: 'Focused Checkbox should use overlay color $focusOverlayColor over $focusColor',
    );

    // Start hovering
    final TestGesture gesture5 = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture5.addPointer();
    addTearDown(gesture5.removePointer);
    await gesture5.moveTo(tester.getCenter(find.byType(Checkbox)));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints..circle(color: hoverOverlayColor, radius: splashRadius),
      reason: 'Hovered Checkbox should use overlay color $hoverOverlayColor over $hoverColor',
    );

    // Finish gestures to release resources.
    await gesture1.up();
    await gesture2.up();
    await gesture3.up();
    await gesture4.up();
    await tester.pumpAndSettle();
  });

  testWidgets('Tristate Checkbox overlay color resolves in pressed active/inactive states', (
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
    TestGesture gesture;
    bool? value = false;

    Widget buildTristateCheckbox() {
      return MaterialApp(
        theme: theme,
        home: Scaffold(
          body: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Checkbox(
                value: value,
                tristate: true,
                onChanged: (bool? v) {
                  setState(() {
                    value = v;
                  });
                },
                overlayColor: MaterialStateProperty.resolveWith(getOverlayColor),
                splashRadius: splashRadius,
              );
            },
          ),
        ),
      );
    }

    // The checkbox is inactive.
    await tester.pumpWidget(buildTristateCheckbox());
    gesture = await tester.press(find.byType(Checkbox));
    await tester.pumpAndSettle();

    expect(value, false);
    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints..circle(color: inactivePressedOverlayColor, radius: splashRadius),
      reason: 'Inactive pressed Checkbox should have overlay color: $inactivePressedOverlayColor',
    );

    // The checkbox is active.
    await gesture.up();
    gesture = await tester.press(find.byType(Checkbox));
    await tester.pumpAndSettle();

    expect(value, true);
    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints..circle(color: activePressedOverlayColor, radius: splashRadius),
      reason: 'Active pressed Checkbox should have overlay color: $activePressedOverlayColor',
    );

    // The checkbox is active in tri-state.
    await gesture.up();
    gesture = await tester.press(find.byType(Checkbox));
    await tester.pumpAndSettle();

    expect(value, null);
    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints..circle(color: activePressedOverlayColor, radius: splashRadius),
      reason:
          'Active (tristate) pressed Checkbox should have overlay color: $activePressedOverlayColor',
    );

    // The checkbox is inactive again.
    await gesture.up();
    gesture = await tester.press(find.byType(Checkbox));
    await tester.pumpAndSettle();

    expect(value, false);
    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints..circle(color: inactivePressedOverlayColor, radius: splashRadius),
      reason: 'Inactive pressed Checkbox should have overlay color: $inactivePressedOverlayColor',
    );

    await gesture.up();
  });

  testWidgets('Do not crash when widget disappears while pointer is down', (
    WidgetTester tester,
  ) async {
    Widget buildCheckbox(bool show) {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(child: show ? Checkbox(value: true, onChanged: (_) {}) : Container()),
        ),
      );
    }

    await tester.pumpWidget(buildCheckbox(true));
    final Offset center = tester.getCenter(find.byType(Checkbox));
    // Put a pointer down on the screen.
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pump();
    // While the pointer is down, the widget disappears.
    await tester.pumpWidget(buildCheckbox(false));
    expect(find.byType(Checkbox), findsNothing);
    // Release pointer after widget disappeared.
    await gesture.up();
  });

  testWidgets('Checkbox BorderSide side only applies when unselected in M2', (
    WidgetTester tester,
  ) async {
    const Color borderColor = Color(0xfff44336);
    const Color activeColor = Color(0xff123456);
    const BorderSide side = BorderSide(width: 4, color: borderColor);

    Widget buildApp({bool? value, bool enabled = true}) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Center(
            child: Checkbox(
              value: value,
              tristate: value == null,
              activeColor: activeColor,
              onChanged: enabled ? (bool? newValue) {} : null,
              side: side,
            ),
          ),
        ),
      );
    }

    RenderBox getCheckboxRenderer() {
      return tester.renderObject<RenderBox>(find.byType(Checkbox));
    }

    void expectBorder() {
      expect(
        getCheckboxRenderer(),
        paints..drrect(
          color: borderColor,
          outer: RRect.fromLTRBR(15, 15, 33, 33, const Radius.circular(1)),
          inner: RRect.fromLTRBR(19, 19, 29, 29, Radius.zero),
        ),
      );
    }

    // Checkbox is unselected, so the specified BorderSide appears.

    await tester.pumpWidget(buildApp(value: false));
    await tester.pumpAndSettle();
    expectBorder();

    await tester.pumpWidget(buildApp(value: false, enabled: false));
    await tester.pumpAndSettle();
    expectBorder();

    // Checkbox is selected/indeterminate, so the specified BorderSide is transparent

    await tester.pumpWidget(buildApp(value: true));
    await tester.pumpAndSettle();
    expect(getCheckboxRenderer(), paints..drrect(color: Colors.transparent));
    expect(getCheckboxRenderer(), paints..path(color: activeColor)); // checkbox fill

    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expect(getCheckboxRenderer(), paints..drrect(color: Colors.transparent));
    expect(getCheckboxRenderer(), paints..path(color: activeColor)); // checkbox fill
  });

  testWidgets('Material2 - Checkbox MaterialStateBorderSide applies unconditionally', (
    WidgetTester tester,
  ) async {
    const Color borderColor = Color(0xfff44336);
    const BorderSide side = BorderSide(width: 4, color: borderColor);
    final ThemeData theme = ThemeData(useMaterial3: false);

    Widget buildApp({bool? value, bool enabled = true}) {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: Checkbox(
              value: value,
              tristate: value == null,
              onChanged: enabled ? (bool? newValue) {} : null,
              side: MaterialStateBorderSide.resolveWith((Set<MaterialState> states) => side),
            ),
          ),
        ),
      );
    }

    void expectBorder() {
      expect(
        tester.renderObject<RenderBox>(find.byType(Checkbox)),
        paints..drrect(
          color: borderColor,
          outer: RRect.fromLTRBR(15, 15, 33, 33, const Radius.circular(1)),
          inner: RRect.fromLTRBR(19, 19, 29, 29, Radius.zero),
        ),
      );
    }

    await tester.pumpWidget(buildApp(value: false));
    await tester.pumpAndSettle();
    expectBorder();

    await tester.pumpWidget(buildApp(value: false, enabled: false));
    await tester.pumpAndSettle();
    expectBorder();

    await tester.pumpWidget(buildApp(value: true));
    await tester.pumpAndSettle();
    expectBorder();

    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expectBorder();
  });

  testWidgets('Material3 - Checkbox MaterialStateBorderSide applies unconditionally', (
    WidgetTester tester,
  ) async {
    const Color borderColor = Color(0xfff44336);
    const BorderSide side = BorderSide(width: 4, color: borderColor);
    final ThemeData theme = ThemeData();

    Widget buildApp({bool? value, bool enabled = true}) {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: Checkbox(
              value: value,
              tristate: value == null,
              onChanged: enabled ? (bool? newValue) {} : null,
              side: MaterialStateBorderSide.resolveWith((Set<MaterialState> states) => side),
            ),
          ),
        ),
      );
    }

    void expectBorder() {
      expect(
        tester.renderObject<RenderBox>(find.byType(Checkbox)),
        paints..drrect(
          color: borderColor,
          outer: RRect.fromLTRBR(15, 15, 33, 33, const Radius.circular(2)),
          inner: RRect.fromLTRBR(19, 19, 29, 29, Radius.zero),
        ),
      );
    }

    await tester.pumpWidget(buildApp(value: false));
    await tester.pumpAndSettle();
    expectBorder();

    await tester.pumpWidget(buildApp(value: false, enabled: false));
    await tester.pumpAndSettle();
    expectBorder();

    await tester.pumpWidget(buildApp(value: true));
    await tester.pumpAndSettle();
    expectBorder();

    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expectBorder();
  });

  testWidgets('disabled checkbox shows tooltip', (WidgetTester tester) async {
    const String longPressTooltip = 'long press tooltip';
    const String tapTooltip = 'tap tooltip';
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: Tooltip(message: longPressTooltip, child: Checkbox(value: true, onChanged: null)),
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
            child: Checkbox(value: true, onChanged: null),
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

  testWidgets('Material3 - Checkbox has default error color when isError is set to true', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'Checkbox');
    addTearDown(focusNode.dispose);
    final ThemeData themeData = ThemeData();
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    bool? value = true;
    Widget buildApp({bool autoFocus = true}) {
      return MaterialApp(
        theme: themeData,
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Checkbox(
                  isError: true,
                  value: value,
                  onChanged: (bool? newValue) {
                    setState(() {
                      value = newValue;
                    });
                  },
                  autofocus: autoFocus,
                  focusNode: focusNode,
                );
              },
            ),
          ),
        ),
      );
    }

    // Focused
    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isTrue);
    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints
        ..circle(color: themeData.colorScheme.error.withOpacity(0.1))
        ..path(color: themeData.colorScheme.error)
        ..path(color: themeData.colorScheme.onError),
    );

    // Default color
    await tester.pumpWidget(Container());
    await tester.pumpWidget(buildApp(autoFocus: false));
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isFalse);
    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints
        ..path(color: themeData.colorScheme.error)
        ..path(color: themeData.colorScheme.onError),
    );

    // Start hovering
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    addTearDown(gesture.removePointer);
    await gesture.moveTo(tester.getCenter(find.byType(Checkbox)));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints
        ..circle(color: themeData.colorScheme.error.withOpacity(0.08))
        ..path(color: themeData.colorScheme.error),
    );

    // Start pressing
    final TestGesture gestureLongPress = await tester.startGesture(
      tester.getCenter(find.byType(Checkbox)),
    );
    await tester.pump();
    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints
        ..circle(color: themeData.colorScheme.error.withOpacity(0.1))
        ..path(color: themeData.colorScheme.error),
    );
    await gestureLongPress.up();
    await tester.pump();
  });

  testWidgets('Material3 - Checkbox MaterialStateBorderSide applies in error states', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'Checkbox');
    addTearDown(focusNode.dispose);
    final ThemeData themeData = ThemeData();
    const Color borderColor = Color(0xffffeb3b);
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    bool? value = false;
    Widget buildApp({bool autoFocus = true}) {
      return MaterialApp(
        theme: themeData,
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Checkbox(
                  isError: true,
                  side: MaterialStateBorderSide.resolveWith((Set<MaterialState> states) {
                    if (states.contains(MaterialState.error)) {
                      return const BorderSide(color: borderColor, width: 4);
                    }
                    return const BorderSide(color: Colors.red, width: 2);
                  }),
                  value: value,
                  onChanged: (bool? newValue) {
                    setState(() {
                      value = newValue;
                    });
                  },
                  autofocus: autoFocus,
                  focusNode: focusNode,
                );
              },
            ),
          ),
        ),
      );
    }

    void expectBorder() {
      expect(
        tester.renderObject<RenderBox>(find.byType(Checkbox)),
        paints..drrect(
          color: borderColor,
          outer: RRect.fromLTRBR(15, 15, 33, 33, const Radius.circular(2)),
          inner: RRect.fromLTRBR(19, 19, 29, 29, Radius.zero),
        ),
      );
    }

    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expectBorder();

    // Focused
    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isTrue);
    expectBorder();

    // Default color
    await tester.pumpWidget(Container());
    await tester.pumpWidget(buildApp(autoFocus: false));
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isFalse);
    expectBorder();

    // Start hovering
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    addTearDown(gesture.removePointer);
    await gesture.moveTo(tester.getCenter(find.byType(Checkbox)));
    await tester.pumpAndSettle();
    expectBorder();

    // Start pressing
    final TestGesture gestureLongPress = await tester.startGesture(
      tester.getCenter(find.byType(Checkbox)),
    );
    await tester.pump();
    expectBorder();
    await gestureLongPress.up();
    await tester.pump();
  });

  testWidgets('Material3 - Checkbox has correct default shape', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData();

    Widget buildApp() {
      return MaterialApp(
        theme: themeData,
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Checkbox(value: false, onChanged: (bool? newValue) {});
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();

    final OutlinedBorder? expectedShape = themeData.checkboxTheme.shape;
    expect(tester.widget<Checkbox>(find.byType(Checkbox)).shape, expectedShape);
    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints..drrect(
        outer: RRect.fromLTRBR(15.0, 15.0, 33.0, 33.0, const Radius.circular(2)),
        inner: RRect.fromLTRBR(17.0, 17.0, 31.0, 31.0, Radius.zero),
      ),
    );
  });

  testWidgets('Checkbox.adaptive shows the correct platform widget', (WidgetTester tester) async {
    Widget buildApp(TargetPlatform platform) {
      return MaterialApp(
        theme: ThemeData(platform: platform),
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Checkbox.adaptive(value: false, onChanged: (bool? newValue) {});
              },
            ),
          ),
        ),
      );
    }

    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    ]) {
      await tester.pumpWidget(buildApp(platform));
      await tester.pumpAndSettle();

      expect(find.byType(CupertinoCheckbox), findsOneWidget);
    }

    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.android,
      TargetPlatform.fuchsia,
      TargetPlatform.linux,
      TargetPlatform.windows,
    ]) {
      await tester.pumpWidget(buildApp(platform));
      await tester.pumpAndSettle();

      expect(find.byType(CupertinoCheckbox), findsNothing);
    }
  });

  testWidgets(
    'Checkbox.adaptive respects Checkbox.mouseCursor on iOS/macOS',
    (WidgetTester tester) async {
      Widget buildApp({MouseCursor? mouseCursor}) {
        return MaterialApp(
          home: Material(
            child: Checkbox.adaptive(
              value: true,
              onChanged: (bool? newValue) {},
              mouseCursor: mouseCursor,
            ),
          ),
        );
      }

      await tester.pumpWidget(buildApp());
      final TestGesture gesture = await tester.createGesture(
        kind: PointerDeviceKind.mouse,
        pointer: 1,
      );
      await gesture.addPointer(location: tester.getCenter(find.byType(CupertinoCheckbox)));
      await tester.pump();
      await gesture.moveTo(tester.getCenter(find.byType(CupertinoCheckbox)));
      expect(
        RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
        kIsWeb ? SystemMouseCursors.click : SystemMouseCursors.basic,
      );

      // Test mouse cursor can be configured.
      await tester.pumpWidget(buildApp(mouseCursor: SystemMouseCursors.click));
      expect(
        RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
        SystemMouseCursors.click,
      );

      // Test Checkbox.adaptive can resolve a WidgetStateMouseCursor.
      await tester.pumpWidget(buildApp(mouseCursor: const _SelectedGrabMouseCursor()));
      expect(
        RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
        SystemMouseCursors.grab,
      );

      await gesture.removePointer();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets('Material2 - Checkbox respects fillColor when it is unchecked', (
    WidgetTester tester,
  ) async {
    final ThemeData theme = ThemeData(useMaterial3: false);
    const Color activeBackgroundColor = Color(0xff123456);
    const Color inactiveBackgroundColor = Color(0xff654321);

    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: Checkbox(
              fillColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
                if (states.contains(MaterialState.selected)) {
                  return activeBackgroundColor;
                }
                return inactiveBackgroundColor;
              }),
              value: false,
              onChanged: enabled ? (bool? newValue) {} : null,
            ),
          ),
        ),
      );
    }

    RenderBox getCheckboxRenderer() {
      return tester.renderObject<RenderBox>(find.byType(Checkbox));
    }

    // Checkbox is unselected, so the default BorderSide appears and fillColor is checkbox's background color.
    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expect(getCheckboxRenderer(), paints..drrect(color: theme.unselectedWidgetColor));
    expect(getCheckboxRenderer(), paints..path(color: inactiveBackgroundColor));

    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pumpAndSettle();
    expect(getCheckboxRenderer(), paints..drrect(color: theme.disabledColor));
    expect(getCheckboxRenderer(), paints..path(color: inactiveBackgroundColor));
  });

  testWidgets('Material3 - Checkbox respects fillColor when it is unchecked', (
    WidgetTester tester,
  ) async {
    final ThemeData theme = ThemeData();
    const Color activeBackgroundColor = Color(0xff123456);
    const Color inactiveBackgroundColor = Color(0xff654321);

    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: Checkbox(
              fillColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
                if (states.contains(MaterialState.selected)) {
                  return activeBackgroundColor;
                }
                return inactiveBackgroundColor;
              }),
              value: false,
              onChanged: enabled ? (bool? newValue) {} : null,
            ),
          ),
        ),
      );
    }

    RenderBox getCheckboxRenderer() {
      return tester.renderObject<RenderBox>(find.byType(Checkbox));
    }

    // Checkbox is unselected, so the default BorderSide appears and fillColor is checkbox's background color.
    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expect(getCheckboxRenderer(), paints..drrect(color: theme.colorScheme.onSurfaceVariant));
    expect(getCheckboxRenderer(), paints..path(color: inactiveBackgroundColor));

    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pumpAndSettle();
    expect(
      getCheckboxRenderer(),
      paints..drrect(color: theme.colorScheme.onSurface.withOpacity(0.38)),
    );
    expect(getCheckboxRenderer(), paints..path(color: inactiveBackgroundColor));
  });
}

class _SelectedGrabMouseCursor extends MaterialStateMouseCursor {
  const _SelectedGrabMouseCursor();

  @override
  MouseCursor resolve(Set<MaterialState> states) {
    if (states.contains(MaterialState.selected)) {
      return SystemMouseCursors.grab;
    }
    return SystemMouseCursors.basic;
  }

  @override
  String get debugDescription => '_SelectedGrabMouseCursor()';
}
