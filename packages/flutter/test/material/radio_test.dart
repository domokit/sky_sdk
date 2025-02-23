// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

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

  testWidgets('Radio control test', (WidgetTester tester) async {
    final Key key = UniqueKey();
    final List<int?> log = <int?>[];

    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Material(
          child: Center(child: Radio<int>(key: key, value: 1, groupValue: 2, onChanged: log.add)),
        ),
      ),
    );

    await tester.tap(find.byKey(key));

    expect(log, equals(<int>[1]));
    log.clear();

    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Material(
          child: Center(
            child: Radio<int>(
              key: key,
              value: 1,
              groupValue: 1,
              onChanged: log.add,
              activeColor: Colors.green[500],
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(key));

    expect(log, isEmpty);

    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Material(
          child: Center(child: Radio<int>(key: key, value: 1, groupValue: 2, onChanged: null)),
        ),
      ),
    );

    await tester.tap(find.byKey(key));

    expect(log, isEmpty);
  });

  testWidgets('Radio can be toggled when toggleable is set', (WidgetTester tester) async {
    final Key key = UniqueKey();
    final List<int?> log = <int?>[];

    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Material(
          child: Center(
            child: Radio<int>(
              key: key,
              value: 1,
              groupValue: 2,
              onChanged: log.add,
              toggleable: true,
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(key));

    expect(log, equals(<int>[1]));
    log.clear();

    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Material(
          child: Center(
            child: Radio<int>(
              key: key,
              value: 1,
              groupValue: 1,
              onChanged: log.add,
              toggleable: true,
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(key));

    expect(log, equals(<int?>[null]));
    log.clear();

    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Material(
          child: Center(
            child: Radio<int>(
              key: key,
              value: 1,
              groupValue: null,
              onChanged: log.add,
              toggleable: true,
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(key));

    expect(log, equals(<int>[1]));
  });

  testWidgets('Radio size is configurable by ThemeData.materialTapTargetSize', (
    WidgetTester tester,
  ) async {
    final Key key1 = UniqueKey();
    await tester.pumpWidget(
      Theme(
        data: theme.copyWith(materialTapTargetSize: MaterialTapTargetSize.padded),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: Radio<bool>(
                key: key1,
                groupValue: true,
                value: true,
                onChanged: (bool? newValue) {},
              ),
            ),
          ),
        ),
      ),
    );

    expect(tester.getSize(find.byKey(key1)), const Size(48.0, 48.0));

    final Key key2 = UniqueKey();
    await tester.pumpWidget(
      Theme(
        data: theme.copyWith(materialTapTargetSize: MaterialTapTargetSize.shrinkWrap),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: Radio<bool>(
                key: key2,
                groupValue: true,
                value: true,
                onChanged: (bool? newValue) {},
              ),
            ),
          ),
        ),
      ),
    );

    expect(tester.getSize(find.byKey(key2)), const Size(40.0, 40.0));
  });

  testWidgets('Radio selected semantics - platform adaptive', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Material(child: Radio<int>(value: 1, groupValue: 1, onChanged: (int? i) {})),
      ),
    );
    final bool isCupertino =
        defaultTargetPlatform == TargetPlatform.iOS ||
        defaultTargetPlatform == TargetPlatform.macOS;
    expect(
      semantics,
      includesNodeWith(
        flags: <SemanticsFlag>[
          SemanticsFlag.isInMutuallyExclusiveGroup,
          SemanticsFlag.hasCheckedState,
          SemanticsFlag.hasEnabledState,
          SemanticsFlag.isEnabled,
          SemanticsFlag.isFocusable,
          SemanticsFlag.isChecked,
          if (isCupertino) SemanticsFlag.hasSelectedState,
          if (isCupertino) SemanticsFlag.isSelected,
        ],
        actions: <SemanticsAction>[
          SemanticsAction.tap,
          if (defaultTargetPlatform != TargetPlatform.iOS) SemanticsAction.focus,
        ],
      ),
    );
    semantics.dispose();
  }, variant: TargetPlatformVariant.all());

  testWidgets('Radio semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Material(child: Radio<int>(value: 1, groupValue: 2, onChanged: (int? i) {})),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              flags: <SemanticsFlag>[
                SemanticsFlag.isInMutuallyExclusiveGroup,
                SemanticsFlag.hasCheckedState,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isEnabled,
                SemanticsFlag.isFocusable,
              ],
              actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
            ),
          ],
        ),
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );

    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Material(child: Radio<int>(value: 2, groupValue: 2, onChanged: (int? i) {})),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              flags: <SemanticsFlag>[
                SemanticsFlag.isInMutuallyExclusiveGroup,
                SemanticsFlag.hasCheckedState,
                SemanticsFlag.isChecked,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isEnabled,
                SemanticsFlag.isFocusable,
              ],
              actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
            ),
          ],
        ),
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );

    await tester.pumpWidget(
      Theme(
        data: theme,
        child: const Material(child: Radio<int>(value: 1, groupValue: 2, onChanged: null)),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              flags: <SemanticsFlag>[
                SemanticsFlag.hasCheckedState,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isInMutuallyExclusiveGroup,
                SemanticsFlag.isFocusable, // This flag is delayed by 1 frame.
              ],
              actions: <SemanticsAction>[SemanticsAction.focus],
            ),
          ],
        ),
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );

    await tester.pump();

    // Now the isFocusable should be gone.
    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              flags: <SemanticsFlag>[
                SemanticsFlag.hasCheckedState,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isInMutuallyExclusiveGroup,
              ],
            ),
          ],
        ),
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );

    await tester.pumpWidget(
      Theme(
        data: theme,
        child: const Material(child: Radio<int>(value: 2, groupValue: 2, onChanged: null)),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              flags: <SemanticsFlag>[
                SemanticsFlag.hasCheckedState,
                SemanticsFlag.isChecked,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isInMutuallyExclusiveGroup,
              ],
            ),
          ],
        ),
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('has semantic events', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final Key key = UniqueKey();
    dynamic semanticEvent;
    int? radioValue = 2;
    tester.binding.defaultBinaryMessenger.setMockDecodedMessageHandler<dynamic>(
      SystemChannels.accessibility,
      (dynamic message) async {
        semanticEvent = message;
      },
    );

    await tester.pumpWidget(
      Theme(
        data: theme,
        child: Material(
          child: Radio<int>(
            key: key,
            value: 1,
            groupValue: radioValue,
            onChanged: (int? i) {
              radioValue = i;
            },
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(key));
    final RenderObject object = tester.firstRenderObject(find.byKey(key));

    expect(radioValue, 1);
    expect(semanticEvent, <String, dynamic>{
      'type': 'tap',
      'nodeId': object.debugSemantics!.id,
      'data': <String, dynamic>{},
    });
    expect(object.debugSemantics!.getSemanticsData().hasAction(SemanticsAction.tap), true);

    semantics.dispose();
    tester.binding.defaultBinaryMessenger.setMockDecodedMessageHandler<dynamic>(
      SystemChannels.accessibility,
      null,
    );
  });

  testWidgets('Material2 - Radio ink ripple is displayed correctly', (WidgetTester tester) async {
    final Key painterKey = UniqueKey();
    const Key radioKey = Key('radio');

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: RepaintBoundary(
            key: painterKey,
            child: Center(
              child: Container(
                width: 100,
                height: 100,
                color: Colors.white,
                child: Radio<int>(
                  key: radioKey,
                  value: 1,
                  groupValue: 1,
                  onChanged: (int? value) {},
                ),
              ),
            ),
          ),
        ),
      ),
    );

    await tester.press(find.byKey(radioKey));
    await tester.pumpAndSettle();
    await expectLater(find.byKey(painterKey), matchesGoldenFile('m2_radio.ink_ripple.png'));
  });

  testWidgets('Material3 - Radio ink ripple is displayed correctly', (WidgetTester tester) async {
    final Key painterKey = UniqueKey();
    const Key radioKey = Key('radio');

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: RepaintBoundary(
            key: painterKey,
            child: Center(
              child: Container(
                width: 100,
                height: 100,
                color: Colors.white,
                child: Radio<int>(
                  key: radioKey,
                  value: 1,
                  groupValue: 1,
                  onChanged: (int? value) {},
                ),
              ),
            ),
          ),
        ),
      ),
    );

    await tester.press(find.byKey(radioKey));
    await tester.pumpAndSettle();
    await expectLater(find.byKey(painterKey), matchesGoldenFile('m3_radio.ink_ripple.png'));
  });

  testWidgets('Radio with splash radius set', (WidgetTester tester) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    const double splashRadius = 30;
    Widget buildApp() {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Container(
                  width: 100,
                  height: 100,
                  color: Colors.white,
                  child: Radio<int>(
                    value: 0,
                    onChanged: (int? newValue) {},
                    focusColor: Colors.orange[500],
                    autofocus: true,
                    groupValue: 0,
                    splashRadius: splashRadius,
                  ),
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
      Material.of(tester.element(find.byWidgetPredicate((Widget widget) => widget is Radio<int>))),
      paints..circle(color: Colors.orange[500], radius: splashRadius),
    );
  });

  testWidgets('Material2 - Radio is focusable and has correct focus color', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'Radio');
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    int? groupValue = 0;
    const Key radioKey = Key('radio');
    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Container(
                  width: 100,
                  height: 100,
                  color: Colors.white,
                  child: Radio<int>(
                    key: radioKey,
                    value: 0,
                    onChanged:
                        enabled
                            ? (int? newValue) {
                              setState(() {
                                groupValue = newValue;
                              });
                            }
                            : null,
                    focusColor: Colors.orange[500],
                    autofocus: true,
                    focusNode: focusNode,
                    groupValue: groupValue,
                  ),
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
      Material.of(tester.element(find.byKey(radioKey))),
      paints
        ..rect(
          color: const Color(0xffffffff),
          rect: const Rect.fromLTRB(350.0, 250.0, 450.0, 350.0),
        )
        ..circle(color: Colors.orange[500])
        ..circle(color: const Color(0xff2196f3))
        ..circle(color: const Color(0xff2196f3)),
    );

    // Check when the radio isn't selected.
    groupValue = 1;
    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isTrue);
    expect(
      Material.of(tester.element(find.byKey(radioKey))),
      paints
        ..rect(
          color: const Color(0xffffffff),
          rect: const Rect.fromLTRB(350.0, 250.0, 450.0, 350.0),
        )
        ..circle(color: Colors.orange[500])
        ..circle(color: const Color(0x8a000000), style: PaintingStyle.stroke, strokeWidth: 2.0),
    );

    // Check when the radio is selected, but disabled.
    groupValue = 0;
    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isFalse);
    expect(
      Material.of(tester.element(find.byKey(radioKey))),
      paints
        ..rect(
          color: const Color(0xffffffff),
          rect: const Rect.fromLTRB(350.0, 250.0, 450.0, 350.0),
        )
        ..circle(color: const Color(0x61000000))
        ..circle(color: const Color(0x61000000)),
    );
    focusNode.dispose();
  });

  testWidgets('Material3 - Radio is focusable and has correct focus color', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'Radio');
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    int? groupValue = 0;
    const Key radioKey = Key('radio');
    final ThemeData theme = ThemeData();
    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Container(
                  width: 100,
                  height: 100,
                  color: Colors.white,
                  child: Radio<int>(
                    key: radioKey,
                    value: 0,
                    onChanged:
                        enabled
                            ? (int? newValue) {
                              setState(() {
                                groupValue = newValue;
                              });
                            }
                            : null,
                    focusColor: Colors.orange[500],
                    autofocus: true,
                    focusNode: focusNode,
                    groupValue: groupValue,
                  ),
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
      Material.of(tester.element(find.byKey(radioKey))),
      paints
        ..rect(
          color: const Color(0xffffffff),
          rect: const Rect.fromLTRB(350.0, 250.0, 450.0, 350.0),
        )
        ..circle(color: Colors.orange[500])
        ..circle(color: theme.colorScheme.primary)
        ..circle(color: theme.colorScheme.primary),
    );

    // Check when the radio isn't selected.
    groupValue = 1;
    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isTrue);
    expect(
      Material.of(tester.element(find.byKey(radioKey))),
      paints
        ..rect()
        ..circle(color: Colors.orange[500])
        ..circle(color: theme.colorScheme.onSurface),
    );

    // Check when the radio is selected, but disabled.
    groupValue = 0;
    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isFalse);
    expect(
      Material.of(tester.element(find.byKey(radioKey))),
      paints
        ..rect(
          color: const Color(0xffffffff),
          rect: const Rect.fromLTRB(350.0, 250.0, 450.0, 350.0),
        )
        ..circle(color: theme.colorScheme.onSurface.withOpacity(0.38))
        ..circle(color: theme.colorScheme.onSurface.withOpacity(0.38)),
    );
    focusNode.dispose();
  });

  testWidgets('Material2 - Radio can be hovered and has correct hover color', (
    WidgetTester tester,
  ) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    int? groupValue = 0;
    const Key radioKey = Key('radio');
    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Container(
                  width: 100,
                  height: 100,
                  color: Colors.white,
                  child: Radio<int>(
                    key: radioKey,
                    value: 0,
                    onChanged:
                        enabled
                            ? (int? newValue) {
                              setState(() {
                                groupValue = newValue;
                              });
                            }
                            : null,
                    hoverColor: Colors.orange[500],
                    groupValue: groupValue,
                  ),
                );
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildApp());

    await tester.pump();
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byKey(radioKey))),
      paints
        ..rect(
          color: const Color(0xffffffff),
          rect: const Rect.fromLTRB(350.0, 250.0, 450.0, 350.0),
        )
        ..circle(color: const Color(0xff2196f3))
        ..circle(color: const Color(0xff2196f3)),
    );

    // Start hovering
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.moveTo(tester.getCenter(find.byKey(radioKey)));

    // Check when the radio isn't selected.
    groupValue = 1;
    await tester.pumpWidget(buildApp());
    await tester.pump();
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byKey(radioKey))),
      paints
        ..rect(
          color: const Color(0xffffffff),
          rect: const Rect.fromLTRB(350.0, 250.0, 450.0, 350.0),
        )
        ..circle(color: Colors.orange[500])
        ..circle(color: const Color(0x8a000000), style: PaintingStyle.stroke, strokeWidth: 2.0),
    );

    // Check when the radio is selected, but disabled.
    groupValue = 0;
    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pump();
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byKey(radioKey))),
      paints
        ..rect(
          color: const Color(0xffffffff),
          rect: const Rect.fromLTRB(350.0, 250.0, 450.0, 350.0),
        )
        ..circle(color: const Color(0x61000000))
        ..circle(color: const Color(0x61000000)),
    );
  });

  testWidgets('Material3 - Radio can be hovered and has correct hover color', (
    WidgetTester tester,
  ) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    int? groupValue = 0;
    const Key radioKey = Key('radio');
    final ThemeData theme = ThemeData();
    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Container(
                  width: 100,
                  height: 100,
                  color: Colors.white,
                  child: Radio<int>(
                    key: radioKey,
                    value: 0,
                    onChanged:
                        enabled
                            ? (int? newValue) {
                              setState(() {
                                groupValue = newValue;
                              });
                            }
                            : null,
                    hoverColor: Colors.orange[500],
                    groupValue: groupValue,
                  ),
                );
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildApp());

    await tester.pump();
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byKey(radioKey))),
      paints
        ..rect(
          color: const Color(0xffffffff),
          rect: const Rect.fromLTRB(350.0, 250.0, 450.0, 350.0),
        )
        ..circle(color: theme.colorScheme.primary)
        ..circle(color: theme.colorScheme.primary),
    );

    // Start hovering
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.moveTo(tester.getCenter(find.byKey(radioKey)));

    // Check when the radio isn't selected.
    groupValue = 1;
    await tester.pumpWidget(buildApp());
    await tester.pump();
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byKey(radioKey))),
      paints
        ..rect(
          color: const Color(0xffffffff),
          rect: const Rect.fromLTRB(350.0, 250.0, 450.0, 350.0),
        )
        ..circle(color: Colors.orange[500])
        ..circle(color: theme.colorScheme.onSurface, style: PaintingStyle.stroke, strokeWidth: 2.0),
    );

    // Check when the radio is selected, but disabled.
    groupValue = 0;
    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pump();
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byKey(radioKey))),
      paints
        ..rect(
          color: const Color(0xffffffff),
          rect: const Rect.fromLTRB(350.0, 250.0, 450.0, 350.0),
        )
        ..circle(color: theme.colorScheme.onSurface.withOpacity(0.38))
        ..circle(color: theme.colorScheme.onSurface.withOpacity(0.38)),
    );
  });

  testWidgets('Radio can be controlled by keyboard shortcuts', (WidgetTester tester) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    int? groupValue = 1;
    const Key radioKey0 = Key('radio0');
    const Key radioKey1 = Key('radio1');
    const Key radioKey2 = Key('radio2');
    final FocusNode focusNode2 = FocusNode(debugLabel: 'radio2');
    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Container(
                  width: 200,
                  height: 100,
                  color: Colors.white,
                  child: Row(
                    children: <Widget>[
                      Radio<int>(
                        key: radioKey0,
                        value: 0,
                        onChanged:
                            enabled
                                ? (int? newValue) {
                                  setState(() {
                                    groupValue = newValue;
                                  });
                                }
                                : null,
                        hoverColor: Colors.orange[500],
                        groupValue: groupValue,
                        autofocus: true,
                      ),
                      Radio<int>(
                        key: radioKey1,
                        value: 1,
                        onChanged:
                            enabled
                                ? (int? newValue) {
                                  setState(() {
                                    groupValue = newValue;
                                  });
                                }
                                : null,
                        hoverColor: Colors.orange[500],
                        groupValue: groupValue,
                      ),
                      Radio<int>(
                        key: radioKey2,
                        value: 2,
                        onChanged:
                            enabled
                                ? (int? newValue) {
                                  setState(() {
                                    groupValue = newValue;
                                  });
                                }
                                : null,
                        hoverColor: Colors.orange[500],
                        groupValue: groupValue,
                        focusNode: focusNode2,
                      ),
                    ],
                  ),
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
    // On web, radios don't respond to the enter key.
    expect(groupValue, kIsWeb ? equals(1) : equals(0));

    focusNode2.requestFocus();
    await tester.pumpAndSettle();

    await tester.sendKeyEvent(LogicalKeyboardKey.space);
    await tester.pumpAndSettle();
    expect(groupValue, equals(2));

    focusNode2.dispose();
  });

  testWidgets('Radio responds to density changes.', (WidgetTester tester) async {
    const Key key = Key('test');
    Future<void> buildTest(VisualDensity visualDensity) async {
      return tester.pumpWidget(
        MaterialApp(
          theme: theme,
          home: Material(
            child: Center(
              child: Radio<int>(
                visualDensity: visualDensity,
                key: key,
                onChanged: (int? value) {},
                value: 0,
                groupValue: 0,
              ),
            ),
          ),
        ),
      );
    }

    await buildTest(VisualDensity.standard);
    final RenderBox box = tester.renderObject(find.byKey(key));
    await tester.pumpAndSettle();
    expect(box.size, equals(const Size(48, 48)));

    await buildTest(const VisualDensity(horizontal: 3.0, vertical: 3.0));
    await tester.pumpAndSettle();
    expect(box.size, equals(const Size(60, 60)));

    await buildTest(const VisualDensity(horizontal: -3.0, vertical: -3.0));
    await tester.pumpAndSettle();
    expect(box.size, equals(const Size(36, 36)));

    await buildTest(const VisualDensity(horizontal: 3.0, vertical: -3.0));
    await tester.pumpAndSettle();
    expect(box.size, equals(const Size(60, 36)));
  });

  testWidgets('Radio changes mouse cursor when hovered', (WidgetTester tester) async {
    const Key key = ValueKey<int>(1);
    // Test Radio() constructor
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Scaffold(
          body: Align(
            alignment: Alignment.topLeft,
            child: Material(
              child: MouseRegion(
                cursor: SystemMouseCursors.forbidden,
                child: Radio<int>(
                  key: key,
                  mouseCursor: SystemMouseCursors.text,
                  value: 1,
                  onChanged: (int? v) {},
                  groupValue: 2,
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
    await gesture.addPointer(location: tester.getCenter(find.byKey(key)));
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
                child: Radio<int>(value: 1, onChanged: (int? v) {}, groupValue: 2),
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
                child: Radio<int>(value: 1, onChanged: null, groupValue: 2),
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
  });

  testWidgets('Radio button fill color resolves in enabled/disabled states', (
    WidgetTester tester,
  ) async {
    const Color activeEnabledFillColor = Color(0xFF000001);
    const Color activeDisabledFillColor = Color(0xFF000002);
    const Color inactiveEnabledFillColor = Color(0xFF000003);
    const Color inactiveDisabledFillColor = Color(0xFF000004);

    Color getFillColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        if (states.contains(MaterialState.selected)) {
          return activeDisabledFillColor;
        }
        return inactiveDisabledFillColor;
      }
      if (states.contains(MaterialState.selected)) {
        return activeEnabledFillColor;
      }
      return inactiveEnabledFillColor;
    }

    final MaterialStateProperty<Color> fillColor = MaterialStateColor.resolveWith(getFillColor);

    int? groupValue = 0;
    const Key radioKey = Key('radio');
    Widget buildApp({required bool enabled}) {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Container(
                  width: 100,
                  height: 100,
                  color: Colors.white,
                  child: Radio<int>(
                    key: radioKey,
                    value: 0,
                    fillColor: fillColor,
                    onChanged:
                        enabled
                            ? (int? newValue) {
                              setState(() {
                                groupValue = newValue;
                              });
                            }
                            : null,
                    groupValue: groupValue,
                  ),
                );
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildApp(enabled: true));

    // Selected and enabled.
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byKey(radioKey))),
      paints
        ..rect(
          color: const Color(0xffffffff),
          rect: const Rect.fromLTRB(350.0, 250.0, 450.0, 350.0),
        )
        ..circle(color: activeEnabledFillColor)
        ..circle(color: activeEnabledFillColor),
    );

    // Check when the radio isn't selected.
    groupValue = 1;
    await tester.pumpWidget(buildApp(enabled: true));
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byKey(radioKey))),
      paints
        ..rect(
          color: const Color(0xffffffff),
          rect: const Rect.fromLTRB(350.0, 250.0, 450.0, 350.0),
        )
        ..circle(color: inactiveEnabledFillColor, style: PaintingStyle.stroke, strokeWidth: 2.0),
    );

    // Check when the radio is selected, but disabled.
    groupValue = 0;
    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byKey(radioKey))),
      paints
        ..rect(
          color: const Color(0xffffffff),
          rect: const Rect.fromLTRB(350.0, 250.0, 450.0, 350.0),
        )
        ..circle(color: activeDisabledFillColor)
        ..circle(color: activeDisabledFillColor),
    );

    // Check when the radio is unselected and disabled.
    groupValue = 1;
    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byKey(radioKey))),
      paints
        ..rect(
          color: const Color(0xffffffff),
          rect: const Rect.fromLTRB(350.0, 250.0, 450.0, 350.0),
        )
        ..circle(color: inactiveDisabledFillColor, style: PaintingStyle.stroke, strokeWidth: 2.0),
    );
  });

  testWidgets('Material2 - Radio fill color resolves in hovered/focused states', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'radio');
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

    int? groupValue = 0;
    const Key radioKey = Key('radio');
    final ThemeData theme = ThemeData(useMaterial3: false);
    Widget buildApp() {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Container(
                  width: 100,
                  height: 100,
                  color: Colors.white,
                  child: Radio<int>(
                    autofocus: true,
                    focusNode: focusNode,
                    key: radioKey,
                    value: 0,
                    fillColor: fillColor,
                    onChanged: (int? newValue) {
                      setState(() {
                        groupValue = newValue;
                      });
                    },
                    groupValue: groupValue,
                  ),
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
      Material.of(tester.element(find.byKey(radioKey))),
      paints
        ..rect(
          color: const Color(0xffffffff),
          rect: const Rect.fromLTRB(350.0, 250.0, 450.0, 350.0),
        )
        ..circle(color: Colors.black12)
        ..circle(color: focusedFillColor),
    );

    // Start hovering
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    addTearDown(gesture.removePointer);
    await gesture.moveTo(tester.getCenter(find.byKey(radioKey)));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byKey(radioKey))),
      paints
        ..rect(
          color: const Color(0xffffffff),
          rect: const Rect.fromLTRB(350.0, 250.0, 450.0, 350.0),
        )
        ..circle(color: theme.hoverColor)
        ..circle(color: hoveredFillColor),
    );

    focusNode.dispose();
  });

  testWidgets('Material3 - Radio fill color resolves in hovered/focused states', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'radio');
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

    int? groupValue = 0;
    const Key radioKey = Key('radio');
    final ThemeData theme = ThemeData();
    Widget buildApp() {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Container(
                  width: 100,
                  height: 100,
                  color: Colors.white,
                  child: Radio<int>(
                    autofocus: true,
                    focusNode: focusNode,
                    key: radioKey,
                    value: 0,
                    fillColor: fillColor,
                    onChanged: (int? newValue) {
                      setState(() {
                        groupValue = newValue;
                      });
                    },
                    groupValue: groupValue,
                  ),
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
      Material.of(tester.element(find.byKey(radioKey))),
      paints
        ..rect()
        ..circle(color: theme.colorScheme.primary.withOpacity(0.1))
        ..circle(color: focusedFillColor),
    );

    // Start hovering
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    addTearDown(gesture.removePointer);
    await gesture.moveTo(tester.getCenter(find.byKey(radioKey)));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byKey(radioKey))),
      paints
        ..rect(
          color: const Color(0xffffffff),
          rect: const Rect.fromLTRB(350.0, 250.0, 450.0, 350.0),
        )
        ..circle(color: theme.colorScheme.primary.withOpacity(0.08))
        ..circle(color: hoveredFillColor),
    );

    focusNode.dispose();
  });

  testWidgets('Radio overlay color resolves in active/pressed/focused/hovered states', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'Radio');
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

    Finder findRadio() {
      return find.byWidgetPredicate((Widget widget) => widget is Radio<bool>);
    }

    MaterialInkController? getRadioMaterial(WidgetTester tester) {
      return Material.of(tester.element(findRadio()));
    }

    Widget buildRadio({bool active = false, bool focused = false, bool useOverlay = true}) {
      return MaterialApp(
        theme: theme,
        home: Scaffold(
          body: Radio<bool>(
            focusNode: focusNode,
            autofocus: focused,
            value: active,
            groupValue: true,
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

    await tester.pumpWidget(buildRadio(useOverlay: false));
    await tester.press(findRadio());
    await tester.pumpAndSettle();

    expect(
      getRadioMaterial(tester),
      paints..circle(color: fillColor.withAlpha(kRadialReactionAlpha), radius: splashRadius),
      reason: 'Default inactive pressed Radio should have overlay color from fillColor',
    );

    await tester.pumpWidget(buildRadio(active: true, useOverlay: false));
    await tester.press(findRadio());
    await tester.pumpAndSettle();

    expect(
      getRadioMaterial(tester),
      paints..circle(color: fillColor.withAlpha(kRadialReactionAlpha), radius: splashRadius),
      reason: 'Default active pressed Radio should have overlay color from fillColor',
    );

    await tester.pumpWidget(buildRadio());
    await tester.press(findRadio());
    await tester.pumpAndSettle();

    expect(
      getRadioMaterial(tester),
      paints..circle(color: inactivePressedOverlayColor, radius: splashRadius),
      reason: 'Inactive pressed Radio should have overlay color: $inactivePressedOverlayColor',
    );

    await tester.pumpWidget(buildRadio(active: true));
    await tester.press(findRadio());
    await tester.pumpAndSettle();

    expect(
      getRadioMaterial(tester),
      paints..circle(color: activePressedOverlayColor, radius: splashRadius),
      reason: 'Active pressed Radio should have overlay color: $activePressedOverlayColor',
    );

    await tester.pumpWidget(Container());
    await tester.pumpWidget(buildRadio(focused: true));
    await tester.pumpAndSettle();

    expect(focusNode.hasPrimaryFocus, isTrue);
    expect(
      getRadioMaterial(tester),
      paints..circle(color: focusOverlayColor, radius: splashRadius),
      reason: 'Focused Radio should use overlay color $focusOverlayColor over $focusColor',
    );

    // Start hovering
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    addTearDown(gesture.removePointer);
    await gesture.moveTo(tester.getCenter(findRadio()));
    await tester.pumpAndSettle();

    expect(
      getRadioMaterial(tester),
      paints..circle(color: hoverOverlayColor, radius: splashRadius),
      reason: 'Hovered Radio should use overlay color $hoverOverlayColor over $hoverColor',
    );

    focusNode.dispose();
  });

  testWidgets('Do not crash when widget disappears while pointer is down', (
    WidgetTester tester,
  ) async {
    final Key key = UniqueKey();

    Widget buildRadio(bool show) {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child:
                show
                    ? Radio<bool>(key: key, value: true, groupValue: false, onChanged: (_) {})
                    : Container(),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildRadio(true));
    final Offset center = tester.getCenter(find.byKey(key));
    // Put a pointer down on the screen.
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pump();
    // While the pointer is down, the widget disappears.
    await tester.pumpWidget(buildRadio(false));
    expect(find.byKey(key), findsNothing);
    // Release pointer after widget disappeared.
    await gesture.up();
  });

  testWidgets('disabled radio shows tooltip', (WidgetTester tester) async {
    const String longPressTooltip = 'long press tooltip';
    const String tapTooltip = 'tap tooltip';
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: const Material(
          child: Tooltip(
            message: longPressTooltip,
            child: Radio<bool>(value: true, groupValue: false, onChanged: null),
          ),
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
      MaterialApp(
        theme: theme,
        home: const Material(
          child: Tooltip(
            triggerMode: TooltipTriggerMode.tap,
            message: tapTooltip,
            child: Radio<bool>(value: true, groupValue: false, onChanged: null),
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

  testWidgets('Material2 - Radio button default colors', (WidgetTester tester) async {
    Widget buildRadio({bool enabled = true, bool selected = true}) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: Radio<bool>(value: true, groupValue: true, onChanged: enabled ? (_) {} : null),
        ),
      );
    }

    await tester.pumpWidget(buildRadio());
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Radio<bool>))),
      paints
        ..circle(color: const Color(0xFF2196F3)) // Outer circle - primary value
        ..circle(color: const Color(0xFF2196F3))
        ..restore(), // Inner circle - primary value
    );

    await tester.pumpWidget(Container());
    await tester.pumpWidget(buildRadio(selected: false));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Radio<bool>))),
      paints
        ..save()
        ..circle(color: const Color(0xFF2196F3))
        ..restore(),
    );

    await tester.pumpWidget(Container());
    await tester.pumpWidget(buildRadio(enabled: false));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Radio<bool>))),
      paints..circle(color: Colors.black38),
    );
  });

  testWidgets('Material3 - Radio button default colors', (WidgetTester tester) async {
    final ThemeData theme = ThemeData();
    Widget buildRadio({bool enabled = true, bool selected = true}) {
      return MaterialApp(
        theme: theme,
        home: Scaffold(
          body: Radio<bool>(value: true, groupValue: true, onChanged: enabled ? (_) {} : null),
        ),
      );
    }

    await tester.pumpWidget(buildRadio());
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Radio<bool>))),
      paints
        ..circle(color: theme.colorScheme.primary) // Outer circle - primary value
        ..circle(color: theme.colorScheme.primary)
        ..restore(), // Inner circle - primary value
    );

    await tester.pumpWidget(Container());
    await tester.pumpWidget(buildRadio(selected: false));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Radio<bool>))),
      paints
        ..save()
        ..circle(color: theme.colorScheme.primary)
        ..restore(),
    );

    await tester.pumpWidget(Container());
    await tester.pumpWidget(buildRadio(enabled: false));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Radio<bool>))),
      paints..circle(color: theme.colorScheme.onSurface.withOpacity(0.38)),
    );
  });

  testWidgets('Material2 - Radio button default overlay colors in hover/focus/press states', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'Radio');
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;

    final ThemeData theme = ThemeData(useMaterial3: false);
    final ColorScheme colors = theme.colorScheme;
    Widget buildRadio({bool enabled = true, bool focused = false, bool selected = true}) {
      return MaterialApp(
        theme: theme,
        home: Scaffold(
          body: Radio<bool>(
            focusNode: focusNode,
            autofocus: focused,
            value: true,
            groupValue: selected,
            onChanged: enabled ? (_) {} : null,
          ),
        ),
      );
    }

    // default selected radio
    await tester.pumpWidget(buildRadio());
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Radio<bool>))),
      paints..circle(color: colors.secondary),
    );

    // selected radio in pressed state
    await tester.pumpWidget(buildRadio());
    final TestGesture gesture1 = await tester.startGesture(
      tester.getCenter(find.byType(Radio<bool>)),
    );
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Radio<bool>))),
      paints
        ..circle(color: colors.secondary.withAlpha(0x1F))
        ..circle(color: colors.secondary),
    );

    // unselected radio in pressed state
    await tester.pumpWidget(buildRadio(selected: false));
    final TestGesture gesture2 = await tester.startGesture(
      tester.getCenter(find.byType(Radio<bool>)),
    );
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Radio<bool>))),
      paints
        ..circle(color: theme.unselectedWidgetColor.withAlpha(0x1F))
        ..circle(color: theme.unselectedWidgetColor),
    );

    // selected radio in focused state
    await tester.pumpWidget(Container()); // reset test
    await tester.pumpWidget(buildRadio(focused: true));
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isTrue);

    expect(
      Material.of(tester.element(find.byType(Radio<bool>))),
      paints
        ..circle(color: theme.focusColor)
        ..circle(color: colors.secondary),
    );

    // unselected radio in focused state
    await tester.pumpWidget(Container()); // reset test
    await tester.pumpWidget(buildRadio(focused: true, selected: false));
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isTrue);

    expect(
      Material.of(tester.element(find.byType(Radio<bool>))),
      paints
        ..circle(color: theme.focusColor)
        ..circle(color: theme.unselectedWidgetColor),
    );

    // selected radio in hovered state
    await tester.pumpWidget(Container()); // reset test
    await tester.pumpWidget(buildRadio());
    final TestGesture gesture3 = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture3.addPointer();
    await gesture3.moveTo(tester.getCenter(find.byType(Radio<bool>)));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Radio<bool>))),
      paints
        ..circle(color: theme.hoverColor)
        ..circle(color: colors.secondary),
    );

    focusNode.dispose();

    // Finish gesture to release resources.
    await gesture1.up();
    await gesture2.up();
    await tester.pumpAndSettle();
  });

  testWidgets('Material3 - Radio button default overlay colors in hover/focus/press states', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'Radio');
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;

    final ThemeData theme = ThemeData();
    final ColorScheme colors = theme.colorScheme;
    Widget buildRadio({bool enabled = true, bool focused = false, bool selected = true}) {
      return MaterialApp(
        theme: theme,
        home: Scaffold(
          body: Radio<bool>(
            focusNode: focusNode,
            autofocus: focused,
            value: true,
            groupValue: selected,
            onChanged: enabled ? (_) {} : null,
          ),
        ),
      );
    }

    // default selected radio
    await tester.pumpWidget(buildRadio());
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Radio<bool>))),
      paints..circle(color: colors.primary.withOpacity(1)),
    );

    // selected radio in pressed state
    await tester.pumpWidget(buildRadio());
    final TestGesture gesture1 = await tester.startGesture(
      tester.getCenter(find.byType(Radio<bool>)),
    );
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Radio<bool>))),
      paints
        ..circle(color: colors.onSurface.withOpacity(0.1))
        ..circle(color: colors.primary.withOpacity(1)),
    );

    // unselected radio in pressed state
    await tester.pumpWidget(buildRadio(selected: false));
    final TestGesture gesture2 = await tester.startGesture(
      tester.getCenter(find.byType(Radio<bool>)),
    );
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Radio<bool>))),
      paints
        ..circle(color: colors.primary.withOpacity(0.1))
        ..circle(color: colors.onSurfaceVariant.withOpacity(1)),
    );

    // selected radio in focused state
    await tester.pumpWidget(Container()); // reset test
    await tester.pumpWidget(buildRadio(focused: true));
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isTrue);

    expect(
      Material.of(tester.element(find.byType(Radio<bool>))),
      paints
        ..circle(color: colors.primary.withOpacity(0.1))
        ..circle(color: colors.primary.withOpacity(1)),
    );

    // unselected radio in focused state
    await tester.pumpWidget(Container()); // reset test
    await tester.pumpWidget(buildRadio(focused: true, selected: false));
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isTrue);

    expect(
      Material.of(tester.element(find.byType(Radio<bool>))),
      paints
        ..circle(color: colors.onSurface.withOpacity(0.1))
        ..circle(color: colors.onSurface.withOpacity(1)),
    );

    // selected radio in hovered state
    await tester.pumpWidget(Container()); // reset test
    await tester.pumpWidget(buildRadio());
    final TestGesture gesture3 = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture3.addPointer();
    await gesture3.moveTo(tester.getCenter(find.byType(Radio<bool>)));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Radio<bool>))),
      paints
        ..circle(color: colors.primary.withOpacity(0.08))
        ..circle(color: colors.primary.withOpacity(1)),
    );

    focusNode.dispose();

    // Finish gesture to release resources.
    await gesture1.up();
    await gesture2.up();
    await tester.pumpAndSettle();
  });

  testWidgets('Radio.adaptive shows the correct platform widget', (WidgetTester tester) async {
    Widget buildApp(TargetPlatform platform) {
      return MaterialApp(
        theme: ThemeData(platform: platform),
        home: Material(
          child: Center(child: Radio<int>.adaptive(value: 1, groupValue: 2, onChanged: (_) {})),
        ),
      );
    }

    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    ]) {
      await tester.pumpWidget(buildApp(platform));
      await tester.pumpAndSettle();

      expect(find.byType(CupertinoRadio<int>), findsOneWidget);
    }

    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.android,
      TargetPlatform.fuchsia,
      TargetPlatform.linux,
      TargetPlatform.windows,
    ]) {
      await tester.pumpWidget(buildApp(platform));
      await tester.pumpAndSettle();

      expect(find.byType(CupertinoRadio<int>), findsNothing);
    }
  });

  testWidgets('Material2 - Radio default overlayColor and fillColor resolves pressed state', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'Radio');
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    final ThemeData theme = ThemeData(useMaterial3: false);

    Finder findRadio() {
      return find.byWidgetPredicate((Widget widget) => widget is Radio<bool>);
    }

    MaterialInkController? getRadioMaterial(WidgetTester tester) {
      return Material.of(tester.element(findRadio()));
    }

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Scaffold(
          body: Radio<bool>(focusNode: focusNode, value: true, groupValue: true, onChanged: (_) {}),
        ),
      ),
    );

    // Hover
    final Offset center = tester.getCenter(find.byType(Radio<bool>));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    addTearDown(gesture.removePointer);
    await gesture.moveTo(center);
    await tester.pumpAndSettle();

    expect(
      getRadioMaterial(tester),
      paints
        ..circle(color: theme.hoverColor)
        ..circle(color: theme.colorScheme.secondary),
    );

    // Highlighted (pressed).
    await gesture.down(center);
    await tester.pumpAndSettle();

    expect(
      getRadioMaterial(tester),
      paints
        ..circle(color: theme.colorScheme.secondary.withAlpha(kRadialReactionAlpha))
        ..circle(color: theme.colorScheme.secondary),
    );
    // Remove pressed and hovered states
    await gesture.up();
    await tester.pumpAndSettle();
    await gesture.moveTo(const Offset(0, 50));
    await tester.pumpAndSettle();

    // Focused.
    focusNode.requestFocus();
    await tester.pumpAndSettle();

    expect(
      getRadioMaterial(tester),
      paints
        ..circle(color: theme.focusColor)
        ..circle(color: theme.colorScheme.secondary),
    );
    focusNode.dispose();
  });

  testWidgets('Material3 - Radio default overlayColor and fillColor resolves pressed state', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'Radio');
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    final ThemeData theme = ThemeData();

    Finder findRadio() {
      return find.byWidgetPredicate((Widget widget) => widget is Radio<bool>);
    }

    MaterialInkController? getRadioMaterial(WidgetTester tester) {
      return Material.of(tester.element(findRadio()));
    }

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Scaffold(
          body: Radio<bool>(focusNode: focusNode, value: true, groupValue: true, onChanged: (_) {}),
        ),
      ),
    );

    // Hover
    final Offset center = tester.getCenter(find.byType(Radio<bool>));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    addTearDown(gesture.removePointer);
    await gesture.moveTo(center);
    await tester.pumpAndSettle();

    expect(
      getRadioMaterial(tester),
      paints
        ..circle(color: theme.colorScheme.primary.withOpacity(0.08))
        ..circle(color: theme.colorScheme.primary),
    );

    // Highlighted (pressed).
    await gesture.down(center);
    await tester.pumpAndSettle();

    expect(
      getRadioMaterial(tester),
      paints
        ..circle(color: theme.colorScheme.onSurface.withOpacity(0.1))
        ..circle(color: theme.colorScheme.primary),
    );
    // Remove pressed and hovered states
    await gesture.up();
    await tester.pumpAndSettle();
    await gesture.moveTo(const Offset(0, 50));
    await tester.pumpAndSettle();

    // Focused.
    focusNode.requestFocus();
    await tester.pumpAndSettle();

    expect(
      getRadioMaterial(tester),
      paints
        ..circle(color: theme.colorScheme.primary.withOpacity(0.1))
        ..circle(color: theme.colorScheme.primary),
    );
    focusNode.dispose();
  });
}
