// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// reduced-test-set:
//   This file is run as part of a reduced test set in CI on Mac and Windows
//   machines.
@Tags(<String>['reduced-test-set'])
library;

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/semantics_tester.dart';

void main() {
  testWidgets('Radio control test', (WidgetTester tester) async {
    final Key key = UniqueKey();
    final List<int?> log = <int?>[];

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoRadio<int>(key: key, value: 1, groupValue: 2, onChanged: log.add),
        ),
      ),
    );

    await tester.tap(find.byKey(key));

    expect(log, equals(<int>[1]));
    log.clear();

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoRadio<int>(
            key: key,
            value: 1,
            groupValue: 1,
            onChanged: log.add,
            activeColor: CupertinoColors.systemGreen,
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(key));

    expect(log, isEmpty);

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoRadio<int>(key: key, value: 1, groupValue: 2, onChanged: null),
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
      CupertinoApp(
        home: Center(
          child: CupertinoRadio<int>(
            key: key,
            value: 1,
            groupValue: 2,
            onChanged: log.add,
            toggleable: true,
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(key));

    expect(log, equals(<int>[1]));
    log.clear();

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoRadio<int>(
            key: key,
            value: 1,
            groupValue: 1,
            onChanged: log.add,
            toggleable: true,
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(key));

    expect(log, equals(<int?>[null]));
    log.clear();

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoRadio<int>(
            key: key,
            value: 1,
            groupValue: null,
            onChanged: log.add,
            toggleable: true,
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(key));

    expect(log, equals(<int>[1]));
  });

  testWidgets('Radio selected semantics - platform adaptive', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(child: CupertinoRadio<int>(value: 1, groupValue: 1, onChanged: (int? i) {})),
      ),
    );

    final bool isApple =
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
          if (isApple) SemanticsFlag.hasSelectedState,
          if (isApple) SemanticsFlag.isSelected,
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
      CupertinoApp(
        home: Center(child: CupertinoRadio<int>(value: 1, groupValue: 2, onChanged: (int? i) {})),
      ),
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
        isInMutuallyExclusiveGroup: true,
      ),
    );

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(child: CupertinoRadio<int>(value: 2, groupValue: 2, onChanged: (int? i) {})),
      ),
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
        isInMutuallyExclusiveGroup: true,
        isChecked: true,
      ),
    );

    await tester.pumpWidget(
      const CupertinoApp(
        home: Center(child: CupertinoRadio<int>(value: 1, groupValue: 2, onChanged: null)),
      ),
    );

    expect(
      tester.getSemantics(find.byType(Focus).last),
      matchesSemantics(
        hasCheckedState: true,
        hasEnabledState: true,
        isFocusable: true,
        isInMutuallyExclusiveGroup: true,
        hasFocusAction: true,
      ),
    );

    await tester.pump();

    // Now the isFocusable should be gone.
    expect(
      tester.getSemantics(find.byType(Focus).last),
      matchesSemantics(
        hasCheckedState: true,
        hasEnabledState: true,
        isInMutuallyExclusiveGroup: true,
      ),
    );

    await tester.pumpWidget(
      const CupertinoApp(
        home: Center(child: CupertinoRadio<int>(value: 2, groupValue: 2, onChanged: null)),
      ),
    );

    expect(
      tester.getSemantics(find.byType(Focus).last),
      matchesSemantics(
        hasCheckedState: true,
        hasEnabledState: true,
        isChecked: true,
        isInMutuallyExclusiveGroup: true,
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
      CupertinoApp(
        home: Center(
          child: CupertinoRadio<int>(
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

  testWidgets('Radio can be controlled by keyboard shortcuts', (WidgetTester tester) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    int? groupValue = 1;
    const Key radioKey0 = Key('radio0');
    const Key radioKey1 = Key('radio1');
    const Key radioKey2 = Key('radio2');
    final FocusNode focusNode2 = FocusNode(debugLabel: 'radio2');
    addTearDown(focusNode2.dispose);
    Widget buildApp({bool enabled = true}) {
      return CupertinoApp(
        home: Center(
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return SizedBox(
                width: 200,
                height: 100,
                child: Row(
                  children: <Widget>[
                    CupertinoRadio<int>(
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
                      groupValue: groupValue,
                      autofocus: true,
                    ),
                    CupertinoRadio<int>(
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
                      groupValue: groupValue,
                    ),
                    CupertinoRadio<int>(
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
                      groupValue: groupValue,
                      focusNode: focusNode2,
                    ),
                  ],
                ),
              );
            },
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
  });

  testWidgets('Show a checkmark when useCheckmarkStyle is true', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(child: CupertinoRadio<int>(value: 1, groupValue: 1, onChanged: (int? i) {})),
      ),
    );
    await tester.pumpAndSettle();

    // Has no checkmark when useCheckmarkStyle is false
    expect(
      tester.firstRenderObject<RenderBox>(find.byType(CupertinoRadio<int>)),
      isNot(paints..path()),
    );

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoRadio<int>(
            value: 1,
            groupValue: 2,
            useCheckmarkStyle: true,
            onChanged: (int? i) {},
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();

    // Has no checkmark when group value doesn't match the value
    expect(
      tester.firstRenderObject<RenderBox>(find.byType(CupertinoRadio<int>)),
      isNot(paints..path()),
    );

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoRadio<int>(
            value: 1,
            groupValue: 1,
            useCheckmarkStyle: true,
            onChanged: (int? i) {},
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();

    // Draws a path to show the checkmark when toggled on
    expect(tester.firstRenderObject<RenderBox>(find.byType(CupertinoRadio<int>)), paints..path());
  });

  testWidgets('Do not crash when widget disappears while pointer is down', (
    WidgetTester tester,
  ) async {
    final Key key = UniqueKey();

    Widget buildRadio(bool show) {
      return CupertinoApp(
        home: Center(
          child:
              show
                  ? CupertinoRadio<bool>(
                    key: key,
                    value: true,
                    groupValue: false,
                    onChanged: (_) {},
                  )
                  : Container(),
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

  testWidgets('Radio has correct default active/inactive/fill/border colors in light mode', (
    WidgetTester tester,
  ) async {
    Widget buildRadio({required int value, required int groupValue}) {
      return CupertinoApp(
        home: Center(
          child: RepaintBoundary(
            child: CupertinoRadio<int>(
              value: value,
              groupValue: groupValue,
              onChanged: (int? i) {},
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildRadio(value: 1, groupValue: 1));
    await expectLater(
      find.byType(CupertinoRadio<int>),
      matchesGoldenFile('radio.light_theme.selected.png'),
    );
    await tester.pumpWidget(buildRadio(value: 1, groupValue: 2));
    await expectLater(
      find.byType(CupertinoRadio<int>),
      matchesGoldenFile('radio.light_theme.unselected.png'),
    );
  });

  testWidgets('Radio has correct default active/inactive/fill/border colors in dark mode', (
    WidgetTester tester,
  ) async {
    Widget buildRadio({required int value, required int groupValue, bool enabled = true}) {
      return CupertinoApp(
        theme: const CupertinoThemeData(brightness: Brightness.dark),
        home: Center(
          child: RepaintBoundary(
            child: CupertinoRadio<int>(
              value: value,
              groupValue: groupValue,
              onChanged: enabled ? (int? i) {} : null,
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildRadio(value: 1, groupValue: 1));
    await expectLater(
      find.byType(CupertinoRadio<int>),
      matchesGoldenFile('radio.dark_theme.selected.png'),
    );
    await tester.pumpWidget(buildRadio(value: 1, groupValue: 2));
    await expectLater(
      find.byType(CupertinoRadio<int>),
      matchesGoldenFile('radio.dark_theme.unselected.png'),
    );
  });

  testWidgets(
    'Disabled radio has correct default active/inactive/fill/border colors in light mode',
    (WidgetTester tester) async {
      Widget buildRadio({required int value, required int groupValue}) {
        return CupertinoApp(
          home: Center(
            child: RepaintBoundary(
              child: CupertinoRadio<int>(value: value, groupValue: groupValue, onChanged: null),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildRadio(value: 1, groupValue: 1));
      await expectLater(
        find.byType(CupertinoRadio<int>),
        matchesGoldenFile('radio.disabled_light_theme.selected.png'),
      );
      await tester.pumpWidget(buildRadio(value: 1, groupValue: 2));
      await expectLater(
        find.byType(CupertinoRadio<int>),
        matchesGoldenFile('radio.disabled_light_theme.unselected.png'),
      );
    },
  );

  testWidgets(
    'Disabled radio has correct default active/inactive/fill/border colors in dark mode',
    (WidgetTester tester) async {
      Widget buildRadio({required int value, required int groupValue}) {
        return CupertinoApp(
          theme: const CupertinoThemeData(brightness: Brightness.dark),
          home: Center(
            child: RepaintBoundary(
              child: CupertinoRadio<int>(value: value, groupValue: groupValue, onChanged: null),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildRadio(value: 1, groupValue: 1));
      await expectLater(
        find.byType(CupertinoRadio<int>),
        matchesGoldenFile('radio.disabled_dark_theme.selected.png'),
      );
      await tester.pumpWidget(buildRadio(value: 1, groupValue: 2));
      await expectLater(
        find.byType(CupertinoRadio<int>),
        matchesGoldenFile('radio.disabled_dark_theme.unselected.png'),
      );
    },
  );

  testWidgets('Radio can set inactive/active/fill colors', (WidgetTester tester) async {
    const Color inactiveBorderColor = Color(0xffd1d1d6);
    const Color activeColor = Color(0x0000000A);
    const Color fillColor = Color(0x0000000B);
    const Color inactiveColor = Color(0x0000000C);
    const double innerRadius = 2.975;
    const double outerRadius = 7.0;

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoRadio<int>(
            value: 1,
            groupValue: 2,
            onChanged: (int? i) {},
            activeColor: activeColor,
            fillColor: fillColor,
            inactiveColor: inactiveColor,
          ),
        ),
      ),
    );

    expect(
      find.byType(CupertinoRadio<int>),
      paints
        ..circle(radius: outerRadius, style: PaintingStyle.fill, color: inactiveColor)
        ..circle(radius: outerRadius, style: PaintingStyle.stroke, color: inactiveBorderColor),
      reason: 'Unselected radio button should use inactive and border colors',
    );

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoRadio<int>(
            value: 1,
            groupValue: 1,
            onChanged: (int? i) {},
            activeColor: activeColor,
            fillColor: fillColor,
            inactiveColor: inactiveColor,
          ),
        ),
      ),
    );

    expect(
      find.byType(CupertinoRadio<int>),
      paints
        ..circle(radius: outerRadius, style: PaintingStyle.fill, color: activeColor)
        ..circle(radius: innerRadius, style: PaintingStyle.fill, color: fillColor),
      reason: 'Selected radio button should use active and fill colors',
    );
  });

  testWidgets('Radio is slightly darkened when pressed in light mode', (WidgetTester tester) async {
    const Color activeInnerColor = Color(0xffffffff);
    const Color activeOuterColor = Color(0xff007aff);
    const Color inactiveBorderColor = Color(0xffd1d1d6);
    const Color inactiveOuterColor = Color(0xffffffff);
    const double innerRadius = 2.975;
    const double outerRadius = 7.0;
    const Color pressedShadowColor = Color(0x26ffffff);

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(child: CupertinoRadio<int>(value: 1, groupValue: 2, onChanged: (int? i) {})),
      ),
    );

    final TestGesture gesture1 = await tester.startGesture(
      tester.getCenter(find.byType(CupertinoRadio<int>)),
    );
    await tester.pump();

    expect(
      find.byType(CupertinoRadio<int>),
      paints
        ..circle(radius: outerRadius, style: PaintingStyle.fill, color: inactiveOuterColor)
        ..circle(radius: outerRadius, style: PaintingStyle.fill, color: pressedShadowColor)
        ..circle(radius: outerRadius, style: PaintingStyle.stroke, color: inactiveBorderColor),
      reason: 'Unselected pressed radio button is slightly darkened',
    );

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(child: CupertinoRadio<int>(value: 2, groupValue: 2, onChanged: (int? i) {})),
      ),
    );

    final TestGesture gesture2 = await tester.startGesture(
      tester.getCenter(find.byType(CupertinoRadio<int>)),
    );
    await tester.pump();

    expect(
      find.byType(CupertinoRadio<int>),
      paints
        ..circle(radius: outerRadius, style: PaintingStyle.fill, color: activeOuterColor)
        ..circle(radius: outerRadius, style: PaintingStyle.fill, color: pressedShadowColor)
        ..circle(radius: innerRadius, style: PaintingStyle.fill, color: activeInnerColor),
      reason: 'Selected pressed radio button is slightly darkened',
    );

    // Finish gestures to release resources.
    await gesture1.up();
    await gesture2.up();
    await tester.pump();
  });

  testWidgets('Radio is slightly lightened when pressed in dark mode', (WidgetTester tester) async {
    const Color activeInnerColor = Color(0xffffffff);
    const Color activeOuterColor = Color(0xff007aff);
    const Color inactiveBorderColor = Color(0x40000000);
    const double innerRadius = 2.975;
    const double outerRadius = 7.0;
    const Color pressedShadowColor = Color(0x26ffffff);

    await tester.pumpWidget(
      CupertinoApp(
        theme: const CupertinoThemeData(brightness: Brightness.dark),
        home: Center(child: CupertinoRadio<int>(value: 1, groupValue: 2, onChanged: (int? i) {})),
      ),
    );

    final TestGesture gesture1 = await tester.startGesture(
      tester.getCenter(find.byType(CupertinoRadio<int>)),
    );
    await tester.pump();

    expect(
      find.byType(CupertinoRadio<int>),
      paints
        ..path()
        ..circle(radius: outerRadius, style: PaintingStyle.fill, color: pressedShadowColor)
        ..circle(radius: outerRadius, style: PaintingStyle.stroke, color: inactiveBorderColor),
      reason: 'Unselected pressed radio button is slightly lightened',
    );

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(child: CupertinoRadio<int>(value: 2, groupValue: 2, onChanged: (int? i) {})),
      ),
    );

    final TestGesture gesture2 = await tester.startGesture(
      tester.getCenter(find.byType(CupertinoRadio<int>)),
    );
    await tester.pump();

    expect(
      find.byType(CupertinoRadio<int>),
      paints
        ..circle(radius: outerRadius, style: PaintingStyle.fill, color: activeOuterColor)
        ..circle(radius: outerRadius, style: PaintingStyle.fill, color: pressedShadowColor)
        ..circle(radius: innerRadius, style: PaintingStyle.fill, color: activeInnerColor),
      reason: 'Selected pressed radio button is slightly lightened',
    );

    // Finish gestures to release resources.
    await gesture1.up();
    await gesture2.up();
    await tester.pump();
  });

  testWidgets('Radio is focusable and has correct focus colors', (WidgetTester tester) async {
    const Color activeInnerColor = Color(0xffffffff);
    const Color activeOuterColor = Color(0xff007aff);
    final Color defaultFocusColor =
        HSLColor.fromColor(CupertinoColors.activeBlue.withOpacity(kCupertinoFocusColorOpacity))
            .withLightness(kCupertinoFocusColorBrightness)
            .withSaturation(kCupertinoFocusColorSaturation)
            .toColor();
    const double innerRadius = 2.975;
    const double outerRadius = 7.0;
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    final FocusNode node = FocusNode();
    addTearDown(node.dispose);

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoRadio<int>(
            value: 1,
            groupValue: 1,
            onChanged: (int? i) {},
            focusNode: node,
            autofocus: true,
          ),
        ),
      ),
    );

    await tester.pump();
    expect(node.hasPrimaryFocus, isTrue);
    expect(
      find.byType(CupertinoRadio<int>),
      paints
        ..circle(radius: outerRadius, style: PaintingStyle.fill, color: activeOuterColor)
        ..circle(radius: innerRadius, style: PaintingStyle.fill, color: activeInnerColor)
        ..circle(strokeWidth: 3.0, style: PaintingStyle.stroke, color: defaultFocusColor),
      reason: 'Radio is focusable and shows the default focus color',
    );
  });

  testWidgets('Radio can configure a focus color', (WidgetTester tester) async {
    const Color activeInnerColor = Color(0xffffffff);
    const Color activeOuterColor = Color(0xff007aff);
    const Color focusColor = Color(0x0000000A);
    const double innerRadius = 2.975;
    const double outerRadius = 7.0;
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    final FocusNode node = FocusNode();
    addTearDown(node.dispose);

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoRadio<int>(
            value: 1,
            groupValue: 1,
            onChanged: (int? i) {},
            focusColor: focusColor,
            focusNode: node,
            autofocus: true,
          ),
        ),
      ),
    );

    await tester.pump();
    expect(node.hasPrimaryFocus, isTrue);
    expect(
      find.byType(CupertinoRadio<int>),
      paints
        ..circle(radius: outerRadius, style: PaintingStyle.fill, color: activeOuterColor)
        ..circle(radius: innerRadius, style: PaintingStyle.fill, color: activeInnerColor)
        ..circle(strokeWidth: 3.0, style: PaintingStyle.stroke, color: focusColor),
      reason: 'Radio configures the color of the focus outline',
    );
  });

  testWidgets('Radio configures mouse cursor', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoRadio<int>(
            value: 1,
            groupValue: 1,
            onChanged: (int? i) {},
            mouseCursor: SystemMouseCursors.forbidden,
          ),
        ),
      ),
    );
    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    addTearDown(gesture.removePointer);
    await gesture.addPointer(location: tester.getCenter(find.byType(CupertinoRadio<int>)));
    await tester.pump();
    await gesture.moveTo(tester.getCenter(find.byType(CupertinoRadio<int>)));
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.forbidden,
    );
  });

  testWidgets('Mouse cursor resolves in disabled/hovered/focused states', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'Radio');
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoRadio<int>(
            value: 1,
            groupValue: 1,
            onChanged: (int? i) {},
            mouseCursor: const _RadioMouseCursor(),
            focusNode: focusNode,
          ),
        ),
      ),
    );
    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    addTearDown(gesture.removePointer);
    await gesture.addPointer(location: tester.getCenter(find.byType(CupertinoRadio<int>)));
    await tester.pump();

    // Test hovered case.
    await gesture.moveTo(tester.getCenter(find.byType(CupertinoRadio<int>)));
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );

    // Test focused case.
    focusNode.requestFocus();
    await tester.pump();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );

    // Test disabled case.
    await tester.pumpWidget(
      const CupertinoApp(
        home: Center(
          child: CupertinoRadio<int>(
            value: 1,
            groupValue: 1,
            onChanged: null,
            mouseCursor: _RadioMouseCursor(),
          ),
        ),
      ),
    );

    await tester.pump();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.forbidden,
    );
    focusNode.dispose();
  });

  testWidgets('Radio default mouse cursor', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(child: CupertinoRadio<int>(value: 1, groupValue: 1, onChanged: (int? i) {})),
      ),
    );
    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    addTearDown(gesture.removePointer);
    await gesture.addPointer(location: tester.getCenter(find.byType(CupertinoRadio<int>)));
    await tester.pump();
    await gesture.moveTo(tester.getCenter(find.byType(CupertinoRadio<int>)));
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      kIsWeb ? SystemMouseCursors.click : SystemMouseCursors.basic,
    );
  });
}

class _RadioMouseCursor extends WidgetStateMouseCursor {
  const _RadioMouseCursor();

  @override
  MouseCursor resolve(Set<WidgetState> states) {
    if (states.contains(WidgetState.disabled)) {
      return SystemMouseCursors.forbidden;
    }
    if (states.contains(WidgetState.focused)) {
      return SystemMouseCursors.basic;
    }
    return SystemMouseCursors.click;
  }

  @override
  String get debugDescription => '_RadioMouseCursor()';
}
