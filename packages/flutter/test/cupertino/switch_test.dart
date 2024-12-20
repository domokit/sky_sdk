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
  testWidgets('Switch can toggle on tap', (WidgetTester tester) async {
    final Key switchKey = UniqueKey();
    bool value = false;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Center(
              child: CupertinoSwitch(
                key: switchKey,
                value: value,
                dragStartBehavior: DragStartBehavior.down,
                onChanged: (bool newValue) {
                  setState(() {
                    value = newValue;
                  });
                },
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

  testWidgets('CupertinoSwitch can be toggled by keyboard shortcuts', (WidgetTester tester) async {
    bool value = true;
    Widget buildApp({bool enabled = true}) {
      return CupertinoApp(
        home: CupertinoPageScaffold(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return CupertinoSwitch(
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
    await tester.pumpAndSettle();
    expect(value, isTrue);
    await tester.sendKeyEvent(LogicalKeyboardKey.tab);
    await tester.pumpAndSettle();
    await tester.sendKeyEvent(LogicalKeyboardKey.space);
    await tester.pumpAndSettle();
    expect(value, isFalse);
    await tester.sendKeyEvent(LogicalKeyboardKey.space);
    await tester.pumpAndSettle();
    expect(value, isTrue);
  });

  testWidgets(
    'Switch emits light haptic vibration on tap',
    (WidgetTester tester) async {
      final Key switchKey = UniqueKey();
      bool value = false;

      final List<MethodCall> log = <MethodCall>[];

      tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.platform, (
        MethodCall methodCall,
      ) async {
        log.add(methodCall);
        return null;
      });

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Center(
                child: CupertinoSwitch(
                  key: switchKey,
                  value: value,
                  dragStartBehavior: DragStartBehavior.down,
                  onChanged: (bool newValue) {
                    setState(() {
                      value = newValue;
                    });
                  },
                ),
              );
            },
          ),
        ),
      );

      await tester.tap(find.byKey(switchKey));
      await tester.pump();

      expect(log, hasLength(1));
      expect(
        log.single,
        isMethodCall('HapticFeedback.vibrate', arguments: 'HapticFeedbackType.lightImpact'),
      );
    },
    variant: TargetPlatformVariant.only(TargetPlatform.iOS),
  );

  testWidgets(
    'Using other widgets that rebuild the switch will not cause vibrations',
    (WidgetTester tester) async {
      final Key switchKey = UniqueKey();
      final Key switchKey2 = UniqueKey();
      bool value = false;
      bool value2 = false;
      final List<MethodCall> log = <MethodCall>[];

      tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.platform, (
        MethodCall methodCall,
      ) async {
        log.add(methodCall);
        return null;
      });

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Center(
                child: Column(
                  children: <Widget>[
                    CupertinoSwitch(
                      key: switchKey,
                      value: value,
                      onChanged: (bool newValue) {
                        setState(() {
                          value = newValue;
                        });
                      },
                    ),
                    CupertinoSwitch(
                      key: switchKey2,
                      value: value2,
                      onChanged: (bool newValue) {
                        setState(() {
                          value2 = newValue;
                        });
                      },
                    ),
                  ],
                ),
              );
            },
          ),
        ),
      );

      await tester.tap(find.byKey(switchKey));
      await tester.pump();

      expect(log, hasLength(1));
      expect(
        log[0],
        isMethodCall('HapticFeedback.vibrate', arguments: 'HapticFeedbackType.lightImpact'),
      );

      await tester.tap(find.byKey(switchKey2));
      await tester.pump();

      expect(log, hasLength(2));
      expect(
        log[1],
        isMethodCall('HapticFeedback.vibrate', arguments: 'HapticFeedbackType.lightImpact'),
      );

      await tester.tap(find.byKey(switchKey));
      await tester.pump();

      expect(log, hasLength(3));
      expect(
        log[2],
        isMethodCall('HapticFeedback.vibrate', arguments: 'HapticFeedbackType.lightImpact'),
      );

      await tester.tap(find.byKey(switchKey2));
      await tester.pump();

      expect(log, hasLength(4));
      expect(
        log[3],
        isMethodCall('HapticFeedback.vibrate', arguments: 'HapticFeedbackType.lightImpact'),
      );
    },
    variant: TargetPlatformVariant.only(TargetPlatform.iOS),
  );

  testWidgets('Haptic vibration triggers on drag', (WidgetTester tester) async {
    bool value = false;
    final List<MethodCall> log = <MethodCall>[];

    tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.platform, (
      MethodCall methodCall,
    ) async {
      log.add(methodCall);
      return null;
    });

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Center(
              child: CupertinoSwitch(
                value: value,
                dragStartBehavior: DragStartBehavior.down,
                onChanged: (bool newValue) {
                  setState(() {
                    value = newValue;
                  });
                },
              ),
            );
          },
        ),
      ),
    );

    await tester.drag(find.byType(CupertinoSwitch), const Offset(30.0, 0.0));
    expect(value, isTrue);
    await tester.pump();

    expect(log, hasLength(1));
    expect(
      log[0],
      isMethodCall('HapticFeedback.vibrate', arguments: 'HapticFeedbackType.lightImpact'),
    );
  }, variant: TargetPlatformVariant.only(TargetPlatform.iOS));

  testWidgets(
    'No haptic vibration triggers from a programmatic value change',
    (WidgetTester tester) async {
      final Key switchKey = UniqueKey();
      bool value = false;

      final List<MethodCall> log = <MethodCall>[];
      tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.platform, (
        MethodCall methodCall,
      ) async {
        log.add(methodCall);
        return null;
      });

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Center(
                child: Column(
                  children: <Widget>[
                    CupertinoButton(
                      child: const Text('Button'),
                      onPressed: () {
                        setState(() {
                          value = !value;
                        });
                      },
                    ),
                    CupertinoSwitch(
                      key: switchKey,
                      value: value,
                      onChanged: (bool newValue) {
                        setState(() {
                          value = newValue;
                        });
                      },
                    ),
                  ],
                ),
              );
            },
          ),
        ),
      );

      expect(value, isFalse);

      await tester.tap(find.byType(CupertinoButton));
      expect(value, isTrue);
      await tester.pump();

      expect(log, hasLength(0));
    },
    variant: TargetPlatformVariant.only(TargetPlatform.iOS),
  );

  testWidgets('Switch can drag (LTR)', (WidgetTester tester) async {
    bool value = false;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Center(
              child: CupertinoSwitch(
                value: value,
                onChanged: (bool newValue) {
                  setState(() {
                    value = newValue;
                  });
                },
              ),
            );
          },
        ),
      ),
    );

    expect(value, isFalse);

    await tester.drag(find.byType(CupertinoSwitch), const Offset(-48.0, 0.0));

    expect(value, isFalse);

    await tester.drag(find.byType(CupertinoSwitch), const Offset(48.0, 0.0));

    expect(value, isTrue);

    await tester.pump();
    await tester.drag(find.byType(CupertinoSwitch), const Offset(48.0, 0.0));

    expect(value, isTrue);

    await tester.pump();
    await tester.drag(find.byType(CupertinoSwitch), const Offset(-48.0, 0.0));

    expect(value, isFalse);
  });

  testWidgets('Switch can drag with dragStartBehavior', (WidgetTester tester) async {
    bool value = false;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Center(
              child: CupertinoSwitch(
                value: value,
                dragStartBehavior: DragStartBehavior.down,
                onChanged: (bool newValue) {
                  setState(() {
                    value = newValue;
                  });
                },
              ),
            );
          },
        ),
      ),
    );

    expect(value, isFalse);
    await tester.drag(find.byType(CupertinoSwitch), const Offset(-30.0, 0.0));
    expect(value, isFalse);

    await tester.drag(find.byType(CupertinoSwitch), const Offset(30.0, 0.0));
    expect(value, isTrue);
    await tester.pump();
    await tester.drag(find.byType(CupertinoSwitch), const Offset(30.0, 0.0));
    expect(value, isTrue);
    await tester.pump();
    await tester.drag(find.byType(CupertinoSwitch), const Offset(-30.0, 0.0));
    expect(value, isFalse);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Center(
              child: CupertinoSwitch(
                value: value,
                onChanged: (bool newValue) {
                  setState(() {
                    value = newValue;
                  });
                },
              ),
            );
          },
        ),
      ),
    );
    await tester.pumpAndSettle();
    final Rect switchRect = tester.getRect(find.byType(CupertinoSwitch));
    expect(value, isFalse);

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
    await tester.pump();

    gesture = await tester.startGesture(switchRect.center);
    await gesture.moveBy(const Offset(-20.0, 0.0));
    await gesture.moveBy(const Offset(-20.0, 0.0));
    expect(value, isTrue);
    await gesture.up();
    expect(value, isFalse);
    await tester.pump();
  });

  testWidgets('Switch can drag (RTL)', (WidgetTester tester) async {
    bool value = false;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.rtl,
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Center(
              child: CupertinoSwitch(
                value: value,
                dragStartBehavior: DragStartBehavior.down,
                onChanged: (bool newValue) {
                  setState(() {
                    value = newValue;
                  });
                },
              ),
            );
          },
        ),
      ),
    );

    expect(value, isFalse);

    await tester.drag(find.byType(CupertinoSwitch), const Offset(30.0, 0.0));

    expect(value, isFalse);

    await tester.drag(find.byType(CupertinoSwitch), const Offset(-30.0, 0.0));

    expect(value, isTrue);

    await tester.pump();
    await tester.drag(find.byType(CupertinoSwitch), const Offset(-30.0, 0.0));

    expect(value, isTrue);

    await tester.pump();
    await tester.drag(find.byType(CupertinoSwitch), const Offset(30.0, 0.0));

    expect(value, isFalse);
  });

  testWidgets('can veto switch dragging result', (WidgetTester tester) async {
    bool value = false;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Material(
              child: Center(
                child: CupertinoSwitch(
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
    );

    // Move a little to the right, not past the middle.
    TestGesture gesture = await tester.startGesture(
      tester.getRect(find.byType(CupertinoSwitch)).center,
    );
    await gesture.moveBy(const Offset(kTouchSlop + 0.1, 0.0));
    await tester.pump();
    await gesture.moveBy(const Offset(-kTouchSlop + 5.1, 0.0));
    await tester.pump();
    await gesture.up();
    await tester.pump();
    expect(value, isFalse);
    final CurvedAnimation position =
        (tester.state(find.byType(CupertinoSwitch)) as dynamic).position as CurvedAnimation;
    expect(position.value, lessThan(0.5));
    await tester.pump();
    await tester.pumpAndSettle();
    expect(value, isFalse);
    expect(position.value, 0);

    // Move past the middle.
    gesture = await tester.startGesture(tester.getRect(find.byType(CupertinoSwitch)).center);
    await gesture.moveBy(const Offset(kTouchSlop + 0.1, 0.0));
    await tester.pump();
    await gesture.up();
    await tester.pump();
    expect(value, isTrue);
    expect(position.value, greaterThan(0.5));

    await tester.pump();
    await tester.pumpAndSettle();
    expect(value, isTrue);
    expect(position.value, 1.0);

    // Now move back to the left, the revert animation should play.
    gesture = await tester.startGesture(tester.getRect(find.byType(CupertinoSwitch)).center);
    await gesture.moveBy(const Offset(-kTouchSlop - 0.1, 0.0));
    await tester.pump();
    await gesture.up();
    await tester.pump();
    expect(value, isTrue);
    expect(position.value, lessThan(0.5));

    await tester.pump();
    await tester.pumpAndSettle();
    expect(value, isTrue);
    expect(position.value, 1.0);
  });

  testWidgets('Switch is translucent when disabled', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: CupertinoSwitch(
            value: false,
            dragStartBehavior: DragStartBehavior.down,
            onChanged: null,
          ),
        ),
      ),
    );

    expect(find.byType(Opacity), findsOneWidget);
    expect(tester.widget<Opacity>(find.byType(Opacity).first).opacity, 0.5);
  });

  testWidgets('Switch is using track color when set', (WidgetTester tester) async {
    const Color trackColor = Color(0xFF00FF00);

    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: CupertinoSwitch(
            value: false,
            trackColor: trackColor,
            dragStartBehavior: DragStartBehavior.down,
            onChanged: null,
          ),
        ),
      ),
    );

    expect(find.byType(CupertinoSwitch), findsOneWidget);
    expect(tester.widget<CupertinoSwitch>(find.byType(CupertinoSwitch)).trackColor, trackColor);
    expect(find.byType(CupertinoSwitch), paints..rrect(color: trackColor));
  });

  testWidgets('Switch is using default thumb color', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(child: CupertinoSwitch(value: false, onChanged: null)),
      ),
    );

    expect(find.byType(CupertinoSwitch), findsOneWidget);
    expect(tester.widget<CupertinoSwitch>(find.byType(CupertinoSwitch)).thumbColor, null);
    expect(
      find.byType(CupertinoSwitch),
      paints
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect(color: CupertinoColors.white),
    );
  });

  testWidgets('Switch is using thumb color when set', (WidgetTester tester) async {
    const Color thumbColor = Color(0xFF000000);
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: CupertinoSwitch(value: false, thumbColor: thumbColor, onChanged: null),
        ),
      ),
    );

    expect(find.byType(CupertinoSwitch), findsOneWidget);
    expect(tester.widget<CupertinoSwitch>(find.byType(CupertinoSwitch)).thumbColor, thumbColor);
    expect(
      find.byType(CupertinoSwitch),
      paints
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect(color: thumbColor),
    );
  });

  testWidgets('Switch can set active/inactive thumb colors', (WidgetTester tester) async {
    bool value = false;
    const Color activeThumbColor = Color(0xff00000A);
    const Color inactiveThumbColor = Color(0xff00000B);

    await tester.pumpWidget(
      CupertinoApp(
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return CupertinoPageScaffold(
                child: Center(
                  child: CupertinoSwitch(
                    dragStartBehavior: DragStartBehavior.down,
                    value: value,
                    onChanged: (bool newValue) {
                      setState(() {
                        value = newValue;
                      });
                    },
                    thumbColor: activeThumbColor,
                    inactiveThumbColor: inactiveThumbColor,
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );
    expect(
      find.byType(CupertinoSwitch),
      paints
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect(color: inactiveThumbColor),
    );
    await tester.drag(find.byType(CupertinoSwitch), const Offset(-30.0, 0.0));
    await tester.pump();
    expect(
      find.byType(CupertinoSwitch),
      paints
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect(color: activeThumbColor),
    );
  });

  testWidgets('Switch is opaque when enabled', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: CupertinoSwitch(
            value: false,
            dragStartBehavior: DragStartBehavior.down,
            onChanged: (bool newValue) {},
          ),
        ),
      ),
    );

    expect(find.byType(Opacity), findsOneWidget);
    expect(tester.widget<Opacity>(find.byType(Opacity).first).opacity, 1.0);
  });

  testWidgets('Switch turns translucent after becoming disabled', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: CupertinoSwitch(
            value: false,
            dragStartBehavior: DragStartBehavior.down,
            onChanged: (bool newValue) {},
          ),
        ),
      ),
    );

    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: CupertinoSwitch(
            value: false,
            dragStartBehavior: DragStartBehavior.down,
            onChanged: null,
          ),
        ),
      ),
    );

    expect(find.byType(Opacity), findsOneWidget);
    expect(tester.widget<Opacity>(find.byType(Opacity).first).opacity, 0.5);
  });

  testWidgets('Switch turns opaque after becoming enabled', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: CupertinoSwitch(
            value: false,
            dragStartBehavior: DragStartBehavior.down,
            onChanged: null,
          ),
        ),
      ),
    );

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: CupertinoSwitch(
            value: false,
            dragStartBehavior: DragStartBehavior.down,
            onChanged: (bool newValue) {},
          ),
        ),
      ),
    );

    expect(find.byType(Opacity), findsOneWidget);
    expect(tester.widget<Opacity>(find.byType(Opacity).first).opacity, 1.0);
  });

  testWidgets('Switch renders correctly before, during, and after being tapped', (
    WidgetTester tester,
  ) async {
    final Key switchKey = UniqueKey();
    bool value = false;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Center(
              child: RepaintBoundary(
                child: CupertinoSwitch(
                  key: switchKey,
                  value: value,
                  dragStartBehavior: DragStartBehavior.down,
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

    await expectLater(find.byKey(switchKey), matchesGoldenFile('switch.tap.off.png'));

    await tester.tap(find.byKey(switchKey));
    expect(value, isTrue);

    // Kick off animation, then advance to intermediate frame.
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 60));
    await expectLater(find.byKey(switchKey), matchesGoldenFile('switch.tap.turningOn.png'));

    await tester.pumpAndSettle();
    await expectLater(find.byKey(switchKey), matchesGoldenFile('switch.tap.on.png'));
  });

  PaintPattern onLabelPaintPattern({required int alpha, bool isRtl = false}) =>
      paints..rect(
        rect: Rect.fromLTWH(isRtl ? 43.5 : 14.5, 14.5, 1.0, 10.0),
        color: const Color(0xffffffff).withAlpha(alpha),
        style: PaintingStyle.fill,
      );

  PaintPattern offLabelPaintPattern({
    required int alpha,
    bool highContrast = false,
    bool isRtl = false,
  }) =>
      paints..circle(
        x: isRtl ? 16.0 : 43.0,
        y: 19.5,
        radius: 5.0,
        color: (highContrast ? const Color(0xffffffff) : const Color(0xffb3b3b3)).withAlpha(alpha),
        strokeWidth: 1.0,
        style: PaintingStyle.stroke,
      );

  testWidgets('Switch renders switch labels correctly before, during, and after being tapped', (
    WidgetTester tester,
  ) async {
    final Key switchKey = UniqueKey();
    bool value = false;
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(onOffSwitchLabels: true),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Center(
                child: RepaintBoundary(
                  child: CupertinoSwitch(
                    key: switchKey,
                    value: value,
                    dragStartBehavior: DragStartBehavior.down,
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

    final RenderObject switchRenderObject =
        tester.element(find.byType(CupertinoSwitch)).renderObject!;

    expect(switchRenderObject, offLabelPaintPattern(alpha: 255));
    expect(switchRenderObject, onLabelPaintPattern(alpha: 0));

    await tester.tap(find.byKey(switchKey));
    expect(value, isTrue);

    // Kick off animation, then advance to intermediate frame.
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 60));
    expect(switchRenderObject, onLabelPaintPattern(alpha: 131));
    expect(switchRenderObject, offLabelPaintPattern(alpha: 124));

    await tester.pumpAndSettle();
    expect(switchRenderObject, onLabelPaintPattern(alpha: 255));
    expect(switchRenderObject, offLabelPaintPattern(alpha: 0));
  });

  testWidgets(
    'Switch renders switch labels correctly before, during, and after being tapped in high contrast',
    (WidgetTester tester) async {
      final Key switchKey = UniqueKey();
      bool value = false;
      await tester.pumpWidget(
        MediaQuery(
          data: const MediaQueryData(onOffSwitchLabels: true, highContrast: true),
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Center(
                  child: RepaintBoundary(
                    child: CupertinoSwitch(
                      key: switchKey,
                      value: value,
                      dragStartBehavior: DragStartBehavior.down,
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

      final RenderObject switchRenderObject =
          tester.element(find.byType(CupertinoSwitch)).renderObject!;

      expect(switchRenderObject, offLabelPaintPattern(highContrast: true, alpha: 255));
      expect(switchRenderObject, onLabelPaintPattern(alpha: 0));

      await tester.tap(find.byKey(switchKey));
      expect(value, isTrue);

      // Kick off animation, then advance to intermediate frame.
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 60));
      expect(switchRenderObject, onLabelPaintPattern(alpha: 131));
      expect(switchRenderObject, offLabelPaintPattern(highContrast: true, alpha: 124));

      await tester.pumpAndSettle();
      expect(switchRenderObject, onLabelPaintPattern(alpha: 255));
      expect(switchRenderObject, offLabelPaintPattern(highContrast: true, alpha: 0));
    },
  );

  testWidgets(
    'Switch renders switch labels correctly before, during, and after being tapped with direction rtl',
    (WidgetTester tester) async {
      final Key switchKey = UniqueKey();
      bool value = false;
      await tester.pumpWidget(
        MediaQuery(
          data: const MediaQueryData(onOffSwitchLabels: true),
          child: Directionality(
            textDirection: TextDirection.rtl,
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Center(
                  child: RepaintBoundary(
                    child: CupertinoSwitch(
                      key: switchKey,
                      value: value,
                      dragStartBehavior: DragStartBehavior.down,
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

      final RenderObject switchRenderObject =
          tester.element(find.byType(CupertinoSwitch)).renderObject!;

      expect(switchRenderObject, offLabelPaintPattern(isRtl: true, alpha: 255));
      expect(switchRenderObject, onLabelPaintPattern(isRtl: true, alpha: 0));

      await tester.tap(find.byKey(switchKey));
      expect(value, isTrue);

      // Kick off animation, then advance to intermediate frame.
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 60));
      expect(switchRenderObject, onLabelPaintPattern(isRtl: true, alpha: 131));
      expect(switchRenderObject, offLabelPaintPattern(isRtl: true, alpha: 124));

      await tester.pumpAndSettle();
      expect(switchRenderObject, onLabelPaintPattern(isRtl: true, alpha: 255));
      expect(switchRenderObject, offLabelPaintPattern(isRtl: true, alpha: 0));
    },
  );

  testWidgets('Switch renders correctly in dark mode', (WidgetTester tester) async {
    final Key switchKey = UniqueKey();
    bool value = false;
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(platformBrightness: Brightness.dark),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Center(
                child: RepaintBoundary(
                  child: CupertinoSwitch(
                    key: switchKey,
                    value: value,
                    dragStartBehavior: DragStartBehavior.down,
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

    await expectLater(find.byKey(switchKey), matchesGoldenFile('switch.tap.off.dark.png'));

    await tester.tap(find.byKey(switchKey));
    expect(value, isTrue);

    await tester.pumpAndSettle();
    await expectLater(find.byKey(switchKey), matchesGoldenFile('switch.tap.on.dark.png'));
  });

  testWidgets('Switch can apply the ambient theme and be opted out', (WidgetTester tester) async {
    final Key switchKey = UniqueKey();
    bool value = false;
    await tester.pumpWidget(
      CupertinoTheme(
        data: const CupertinoThemeData(primaryColor: Colors.amber, applyThemeToAll: true),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Center(
                child: RepaintBoundary(
                  child: Column(
                    children: <Widget>[
                      CupertinoSwitch(
                        key: switchKey,
                        value: value,
                        dragStartBehavior: DragStartBehavior.down,
                        applyTheme: true,
                        onChanged: (bool newValue) {
                          setState(() {
                            value = newValue;
                          });
                        },
                      ),
                      CupertinoSwitch(
                        value: value,
                        dragStartBehavior: DragStartBehavior.down,
                        applyTheme: false,
                        onChanged: (bool newValue) {
                          setState(() {
                            value = newValue;
                          });
                        },
                      ),
                    ],
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );

    await expectLater(find.byType(Column), matchesGoldenFile('switch.tap.off.themed.png'));

    await tester.tap(find.byKey(switchKey));
    expect(value, isTrue);

    await tester.pumpAndSettle();
    await expectLater(find.byType(Column), matchesGoldenFile('switch.tap.on.themed.png'));
  });

  testWidgets('Hovering over switch updates cursor to clickable on Web', (
    WidgetTester tester,
  ) async {
    const bool value = false;
    // Disabled CupertinoSwitch does not update cursor on Web.
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return const Center(
              child: CupertinoSwitch(
                value: value,
                dragStartBehavior: DragStartBehavior.down,
                onChanged: null,
              ),
            );
          },
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    final Offset cupertinoSwitch = tester.getCenter(find.byType(CupertinoSwitch));
    await gesture.addPointer(location: cupertinoSwitch);
    await tester.pumpAndSettle();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );

    // Enabled CupertinoSwitch updates cursor when hovering on Web.
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Center(
              child: CupertinoSwitch(
                value: value,
                dragStartBehavior: DragStartBehavior.down,
                onChanged: (bool newValue) {},
              ),
            );
          },
        ),
      ),
    );

    await gesture.moveTo(const Offset(10, 10));
    await tester.pumpAndSettle();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );

    await gesture.moveTo(cupertinoSwitch);
    await tester.pumpAndSettle();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      kIsWeb ? SystemMouseCursors.click : SystemMouseCursors.basic,
    );
  });

  testWidgets('Switch configures mouse cursor', (WidgetTester tester) async {
    const bool value = false;
    const Offset switchSize = Offset(51.0, 31.0);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Center(
              child: CupertinoSwitch(
                value: value,
                dragStartBehavior: DragStartBehavior.down,
                mouseCursor: WidgetStateProperty.all(SystemMouseCursors.forbidden),
                onChanged: (bool newValue) {},
              ),
            );
          },
        ),
      ),
    );
    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    // The pointer is not pointing at the switch.
    await gesture.addPointer(location: tester.getCenter(find.byType(CupertinoSwitch)) + switchSize);
    await tester.pump();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );
    // The pointer now points at the switch.
    await gesture.moveTo(tester.getCenter(find.byType(CupertinoSwitch)));
    await tester.pump();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.forbidden,
    );
  });

  testWidgets('CupertinoSwitch is focusable and has correct focus color', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'CupertinoSwitch');
    addTearDown(focusNode.dispose);
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    bool value = true;
    const Color focusColor = Color(0xffff0000);

    Widget buildApp({bool enabled = true}) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Center(
              child: CupertinoSwitch(
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
      );
    }

    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();

    expect(focusNode.hasPrimaryFocus, isTrue);
    expect(
      find.byType(CupertinoSwitch),
      paints
        ..rrect(color: const Color(0xff34c759))
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
      find.byType(CupertinoSwitch),
      paints
        ..rrect(color: const Color(0x28787880))
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
      find.byType(CupertinoSwitch),
      paints
        ..rrect(color: const Color(0x28787880))
        ..clipRRect()
        ..rrect(color: const Color(0x26000000))
        ..rrect(color: const Color(0x0f000000))
        ..rrect(color: const Color(0x0a000000))
        ..rrect(color: const Color(0xffffffff)),
    );
  });

  testWidgets('CupertinoSwitch.onFocusChange callback', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'CupertinoSwitch');
    addTearDown(focusNode.dispose);
    bool focused = false;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: CupertinoSwitch(
            value: true,
            focusNode: focusNode,
            onFocusChange: (bool value) {
              focused = value;
            },
            onChanged: (bool newValue) {},
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
  });

  testWidgets('Switch has semantic events', (WidgetTester tester) async {
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
      Directionality(
        textDirection: TextDirection.ltr,
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Material(
              child: Center(
                child: CupertinoSwitch(
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
    await tester.tap(find.byType(CupertinoSwitch));
    final RenderObject object = tester.firstRenderObject(find.byType(CupertinoSwitch));

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

  testWidgets('Switch sends semantic events from parent if fully merged', (
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
      CupertinoApp(
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
                  title: const Text('test'),
                  onTap: () {
                    onChanged(!value);
                  },
                  trailing: CupertinoSwitch(value: value, onChanged: onChanged),
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

  testWidgets('Track outline color resolves in active/enabled states', (WidgetTester tester) async {
    const Color activeEnabledTrackOutlineColor = Color(0xFF000001);
    const Color activeDisabledTrackOutlineColor = Color(0xFF000002);
    const Color inactiveEnabledTrackOutlineColor = Color(0xFF000003);
    const Color inactiveDisabledTrackOutlineColor = Color(0xFF000004);

    Color getTrackOutlineColor(Set<WidgetState> states) {
      if (states.contains(WidgetState.disabled)) {
        if (states.contains(WidgetState.selected)) {
          return activeDisabledTrackOutlineColor;
        }
        return inactiveDisabledTrackOutlineColor;
      }
      if (states.contains(WidgetState.selected)) {
        return activeEnabledTrackOutlineColor;
      }
      return inactiveEnabledTrackOutlineColor;
    }

    final WidgetStateProperty<Color> trackOutlineColor = WidgetStateColor.resolveWith(
      getTrackOutlineColor,
    );

    Widget buildSwitch({required bool enabled, required bool active}) {
      return Directionality(
        textDirection: TextDirection.rtl,
        child: CupertinoPageScaffold(
          child: Center(
            child: CupertinoSwitch(
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
      find.byType(CupertinoSwitch),
      paints
        ..rrect(style: PaintingStyle.fill)
        ..rrect(color: inactiveDisabledTrackOutlineColor, style: PaintingStyle.stroke),
      reason: 'Inactive disabled switch track outline should use this value',
    );

    await tester.pumpWidget(buildSwitch(enabled: false, active: true));
    await tester.pumpAndSettle();

    expect(
      find.byType(CupertinoSwitch),
      paints
        ..rrect(style: PaintingStyle.fill)
        ..rrect(color: activeDisabledTrackOutlineColor, style: PaintingStyle.stroke),
      reason: 'Active disabled switch track outline should match these colors',
    );

    await tester.pumpWidget(buildSwitch(enabled: true, active: false));
    await tester.pumpAndSettle();

    expect(
      find.byType(CupertinoSwitch),
      paints
        ..rrect(style: PaintingStyle.fill)
        ..rrect(color: inactiveEnabledTrackOutlineColor),
      reason: 'Inactive enabled switch track outline should match these colors',
    );

    await tester.pumpWidget(buildSwitch(enabled: true, active: true));
    await tester.pumpAndSettle();

    expect(
      find.byType(CupertinoSwitch),
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

    Color getTrackOutlineColor(Set<WidgetState> states) {
      if (states.contains(WidgetState.hovered)) {
        return hoveredTrackOutlineColor;
      }
      if (states.contains(WidgetState.focused)) {
        return focusedTrackOutlineColor;
      }
      return Colors.transparent;
    }

    final WidgetStateProperty<Color> trackOutlineColor = WidgetStateColor.resolveWith(
      getTrackOutlineColor,
    );

    Widget buildSwitch() {
      return Directionality(
        textDirection: TextDirection.rtl,
        child: Material(
          child: Center(
            child: CupertinoSwitch(
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
      find.byType(CupertinoSwitch),
      paints
        ..rrect(style: PaintingStyle.fill)
        ..rrect(color: focusedTrackOutlineColor, style: PaintingStyle.stroke),
      reason: 'Active enabled switch track outline should match this color',
    );

    // Start hovering.
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byType(CupertinoSwitch)));
    await tester.pumpAndSettle();

    expect(
      find.byType(CupertinoSwitch),
      paints
        ..rrect(style: PaintingStyle.fill)
        ..rrect(color: hoveredTrackOutlineColor, style: PaintingStyle.stroke),
      reason: 'Active enabled switch track outline should match this color',
    );

    focusNode.dispose();
  });

  testWidgets('Track outline width resolves in active/enabled states', (WidgetTester tester) async {
    const double activeEnabledTrackOutlineWidth = 1.0;
    const double activeDisabledTrackOutlineWidth = 2.0;
    const double inactiveEnabledTrackOutlineWidth = 3.0;
    const double inactiveDisabledTrackOutlineWidth = 4.0;

    double getTrackOutlineWidth(Set<WidgetState> states) {
      if (states.contains(WidgetState.disabled)) {
        if (states.contains(WidgetState.selected)) {
          return activeDisabledTrackOutlineWidth;
        }
        return inactiveDisabledTrackOutlineWidth;
      }
      if (states.contains(WidgetState.selected)) {
        return activeEnabledTrackOutlineWidth;
      }
      return inactiveEnabledTrackOutlineWidth;
    }

    final WidgetStateProperty<double> trackOutlineWidth = WidgetStateProperty.resolveWith(
      getTrackOutlineWidth,
    );
    const WidgetStateProperty<Color> trackOutlineColor = WidgetStatePropertyAll<Color>(
      Color(0xFFFFFFFF),
    );

    Widget buildSwitch({required bool enabled, required bool active}) {
      return CupertinoApp(
        home: CupertinoPageScaffold(
          child: Center(
            child: CupertinoSwitch(
              trackOutlineWidth: trackOutlineWidth,
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
      find.byType(CupertinoSwitch),
      paints
        ..rrect(style: PaintingStyle.fill)
        ..rrect(strokeWidth: inactiveDisabledTrackOutlineWidth, style: PaintingStyle.stroke),
      reason: 'Inactive disabled switch track outline width should be 4.0',
    );

    await tester.pumpWidget(buildSwitch(enabled: false, active: true));
    await tester.pumpAndSettle();

    expect(
      find.byType(CupertinoSwitch),
      paints
        ..rrect(style: PaintingStyle.fill)
        ..rrect(strokeWidth: activeDisabledTrackOutlineWidth, style: PaintingStyle.stroke),
      reason: 'Active disabled switch track outline width should be 2.0',
    );

    await tester.pumpWidget(buildSwitch(enabled: true, active: false));
    await tester.pumpAndSettle();

    expect(
      find.byType(CupertinoSwitch),
      paints
        ..rrect(style: PaintingStyle.fill)
        ..rrect(strokeWidth: inactiveEnabledTrackOutlineWidth, style: PaintingStyle.stroke),
      reason: 'Inactive enabled switch track outline width should be 3.0',
    );

    await tester.pumpWidget(buildSwitch(enabled: true, active: true));
    await tester.pumpAndSettle();

    expect(
      find.byType(CupertinoSwitch),
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

    double getTrackOutlineWidth(Set<WidgetState> states) {
      if (states.contains(WidgetState.hovered)) {
        return hoveredTrackOutlineWidth;
      }
      if (states.contains(WidgetState.focused)) {
        return focusedTrackOutlineWidth;
      }
      return 8.0;
    }

    final WidgetStateProperty<double> trackOutlineWidth = WidgetStateProperty.resolveWith(
      getTrackOutlineWidth,
    );
    const WidgetStateProperty<Color> trackOutlineColor = WidgetStatePropertyAll<Color>(
      Color(0xFFFFFFFF),
    );

    Widget buildSwitch() {
      return MaterialApp(
        home: Material(
          child: Center(
            child: CupertinoSwitch(
              focusNode: focusNode,
              autofocus: true,
              value: true,
              trackOutlineWidth: trackOutlineWidth,
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
      find.byType(CupertinoSwitch),
      paints
        ..rrect(style: PaintingStyle.fill)
        ..rrect(strokeWidth: focusedTrackOutlineWidth, style: PaintingStyle.stroke)
        ..rrect(strokeWidth: 3.5, style: PaintingStyle.stroke),
      reason: 'Active enabled switch track outline width should be 6.0',
    );

    // Start hovering.
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byType(CupertinoSwitch)));
    await tester.pumpAndSettle();

    expect(
      find.byType(CupertinoSwitch),
      paints
        ..rrect(style: PaintingStyle.fill)
        ..rrect(strokeWidth: hoveredTrackOutlineWidth, style: PaintingStyle.stroke),
      reason: 'Active enabled switch track outline width should be 4.0',
    );

    focusNode.dispose();
  });

  testWidgets('Switch can set icon', (WidgetTester tester) async {
    WidgetStateProperty<Icon?> thumbIcon(Icon? activeIcon, Icon? inactiveIcon) {
      return WidgetStateProperty.resolveWith<Icon?>((Set<WidgetState> states) {
        if (states.contains(WidgetState.selected)) {
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
      return Directionality(
        textDirection: TextDirection.ltr,
        child: CupertinoPageScaffold(
          child: Center(
            child: CupertinoSwitch(
              thumbIcon: thumbIcon(activeIcon, inactiveIcon),
              value: active,
              onChanged: enabled ? (_) {} : null,
            ),
          ),
        ),
      );
    }

    // The active icon shows when the switch is on.
    await tester.pumpWidget(
      buildSwitch(enabled: true, active: true, activeIcon: const Icon(Icons.close)),
    );
    await tester.pumpAndSettle();
    expect(
      find.byType(CupertinoSwitch),
      paints
        ..rrect()
        ..rrect()
        ..paragraph(offset: const Offset(31.5, 11.5)),
    );

    // The inactive icon shows when the switch is off.
    await tester.pumpWidget(
      buildSwitch(enabled: true, active: false, inactiveIcon: const Icon(Icons.close)),
    );
    await tester.pumpAndSettle();
    expect(
      find.byType(CupertinoSwitch),
      paints
        ..rrect()
        ..rrect()
        ..rrect()
        ..paragraph(offset: const Offset(11.5, 11.5)),
    );

    // The active icon doesn't show when the switch is off.
    await tester.pumpWidget(
      buildSwitch(enabled: true, active: false, activeIcon: const Icon(Icons.check)),
    );
    await tester.pumpAndSettle();
    expect(
      find.byType(CupertinoSwitch),
      paints
        ..rrect()
        ..rrect()
        ..rrect(),
    );

    // The inactive icon doesn't show when the switch is on.
    await tester.pumpWidget(
      buildSwitch(enabled: true, active: true, inactiveIcon: const Icon(Icons.check)),
    );
    await tester.pumpAndSettle();
    expect(
      find.byType(CupertinoSwitch),
      paints
        ..rrect()
        ..rrect()
        ..restore(),
    );

    // No icons are shown.
    await tester.pumpWidget(buildSwitch(enabled: true, active: false));
    expect(
      find.byType(CupertinoSwitch),
      paints
        ..rrect()
        ..rrect()
        ..rrect()
        ..restore(),
    );
  });

  group('with image', () {
    late ui.Image image;

    setUp(() async {
      image = await createTestImage(width: 100, height: 100);
    });

    testWidgets('Thumb images show up when set', (WidgetTester tester) async {
      imageCache.clear();
      final _TestImageProvider provider1 = _TestImageProvider();
      final _TestImageProvider provider2 = _TestImageProvider();

      expect(provider1.loadCallCount, 0);
      expect(provider2.loadCallCount, 0);

      bool value1 = true;
      await tester.pumpWidget(
        CupertinoApp(
          home: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return CupertinoPageScaffold(
                child: CupertinoSwitch(
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
      await tester.tap(find.byType(CupertinoSwitch));
      await tester.pumpAndSettle();
      expect(provider1.loadCallCount, 1);
      expect(provider2.loadCallCount, 1);
      expect(imageCache.liveImageCount, 2);
    });

    testWidgets('Does not crash when imageProvider completes after switch is disposed', (
      WidgetTester tester,
    ) async {
      final DelayedImageProvider imageProvider = DelayedImageProvider(image);

      await tester.pumpWidget(
        CupertinoApp(
          home: CupertinoPageScaffold(
            child: Center(
              child: CupertinoSwitch(
                value: true,
                onChanged: null,
                inactiveThumbImage: imageProvider,
              ),
            ),
          ),
        ),
      );

      expect(find.byType(CupertinoSwitch), findsOneWidget);

      // Dispose the switch by taking down the tree.
      await tester.pumpWidget(Container());
      expect(find.byType(CupertinoSwitch), findsNothing);

      imageProvider.complete();
      expect(tester.takeException(), isNull);
    });

    testWidgets('Does not crash when previous imageProvider completes after switch is disposed', (
      WidgetTester tester,
    ) async {
      final DelayedImageProvider imageProvider1 = DelayedImageProvider(image);
      final DelayedImageProvider imageProvider2 = DelayedImageProvider(image);

      Future<void> buildSwitch(ImageProvider imageProvider) {
        return tester.pumpWidget(
          CupertinoApp(
            home: CupertinoPageScaffold(
              child: Center(
                child: CupertinoSwitch(
                  value: true,
                  onChanged: null,
                  inactiveThumbImage: imageProvider,
                ),
              ),
            ),
          ),
        );
      }

      await buildSwitch(imageProvider1);
      expect(find.byType(CupertinoSwitch), findsOneWidget);
      // Replace the ImageProvider.
      await buildSwitch(imageProvider2);
      expect(find.byType(CupertinoSwitch), findsOneWidget);

      // Dispose the switch by taking down the tree.
      await tester.pumpWidget(Container());
      expect(find.byType(CupertinoSwitch), findsNothing);

      // Completing the replaced ImageProvider shouldn't crash.
      imageProvider1.complete();
      expect(tester.takeException(), isNull);

      imageProvider2.complete();
      expect(tester.takeException(), isNull);
    });

    testWidgets('Switch uses inactive track color when set', (WidgetTester tester) async {
      const Color inactiveTrackColor = Color(0xFF00FF00);

      await tester.pumpWidget(
        const Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: CupertinoSwitch(
              value: false,
              inactiveTrackColor: inactiveTrackColor,
              dragStartBehavior: DragStartBehavior.down,
              onChanged: null,
            ),
          ),
        ),
      );

      expect(find.byType(CupertinoSwitch), findsOneWidget);
      expect(
        tester.widget<CupertinoSwitch>(find.byType(CupertinoSwitch)).inactiveTrackColor,
        inactiveTrackColor,
      );
      expect(find.byType(CupertinoSwitch), paints..rrect(color: inactiveTrackColor));
    });

    testWidgets('Switch uses active track color when set', (WidgetTester tester) async {
      const Color activeTrackColor = Color(0xFF00FF00);

      await tester.pumpWidget(
        const Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: CupertinoSwitch(
              value: true,
              activeTrackColor: activeTrackColor,
              dragStartBehavior: DragStartBehavior.down,
              onChanged: null,
            ),
          ),
        ),
      );

      expect(find.byType(CupertinoSwitch), findsOneWidget);
      expect(
        tester.widget<CupertinoSwitch>(find.byType(CupertinoSwitch)).activeTrackColor,
        activeTrackColor,
      );
      expect(find.byType(CupertinoSwitch), paints..rrect(color: activeTrackColor));
    });
  });
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
