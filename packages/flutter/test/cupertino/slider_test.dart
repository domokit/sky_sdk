// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/semantics_tester.dart';

const CupertinoDynamicColor _kSystemFill = CupertinoDynamicColor(
  color: Color.fromARGB(51, 120, 120, 128),
  darkColor: Color.fromARGB(91, 120, 120, 128),
  highContrastColor: Color.fromARGB(71, 120, 120, 128),
  darkHighContrastColor: Color.fromARGB(112, 120, 120, 128),
  elevatedColor: Color.fromARGB(51, 120, 120, 128),
  darkElevatedColor: Color.fromARGB(91, 120, 120, 128),
  highContrastElevatedColor: Color.fromARGB(71, 120, 120, 128),
  darkHighContrastElevatedColor: Color.fromARGB(112, 120, 120, 128),
);

void main() {
  Future<void> dragSlider(WidgetTester tester, Key sliderKey) {
    final Offset topLeft = tester.getTopLeft(find.byKey(sliderKey));
    const double unit = CupertinoThumbPainter.radius;
    const double delta = 3.0 * unit;
    return tester.dragFrom(topLeft + const Offset(unit, unit), const Offset(delta, 0.0));
  }

  testWidgets('Slider does not move when tapped (LTR)', (WidgetTester tester) async {
    final Key sliderKey = UniqueKey();
    double value = 0.0;

    await tester.pumpWidget(
      CupertinoApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: CupertinoSlider(
                    key: sliderKey,
                    value: value,
                    onChanged: (double newValue) {
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

    expect(value, equals(0.0));
    await tester.tap(find.byKey(sliderKey), warnIfMissed: false);
    expect(value, equals(0.0));
    await tester.pump(); // No animation should start.
    // Check the transientCallbackCount before tearing down the widget to ensure
    // that no animation is running.
    expect(SchedulerBinding.instance.transientCallbackCount, equals(0));
  });

  testWidgets('Slider does not move when tapped (RTL)', (WidgetTester tester) async {
    final Key sliderKey = UniqueKey();
    double value = 0.0;

    await tester.pumpWidget(
      CupertinoApp(
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: CupertinoSlider(
                    key: sliderKey,
                    value: value,
                    onChanged: (double newValue) {
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

    expect(value, equals(0.0));
    await tester.tap(find.byKey(sliderKey), warnIfMissed: false);
    expect(value, equals(0.0));
    await tester.pump(); // No animation should start.
    // Check the transientCallbackCount before tearing down the widget to ensure
    // that no animation is running.
    expect(SchedulerBinding.instance.transientCallbackCount, equals(0));
  });

  testWidgets('Slider calls onChangeStart once when interaction begins', (
    WidgetTester tester,
  ) async {
    final Key sliderKey = UniqueKey();
    double value = 0.0;
    int numberOfTimesOnChangeStartIsCalled = 0;

    await tester.pumpWidget(
      CupertinoApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: CupertinoSlider(
                    key: sliderKey,
                    value: value,
                    onChanged: (double newValue) {
                      setState(() {
                        value = newValue;
                      });
                    },
                    onChangeStart: (double value) {
                      numberOfTimesOnChangeStartIsCalled++;
                    },
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );

    await dragSlider(tester, sliderKey);

    expect(numberOfTimesOnChangeStartIsCalled, equals(1));

    await tester.pump(); // No animation should start.
    // Check the transientCallbackCount before tearing down the widget to ensure
    // that no animation is running.
    expect(SchedulerBinding.instance.transientCallbackCount, equals(0));
  });

  testWidgets('Slider calls onChangeEnd once after interaction has ended', (
    WidgetTester tester,
  ) async {
    final Key sliderKey = UniqueKey();
    double value = 0.0;
    int numberOfTimesOnChangeEndIsCalled = 0;

    await tester.pumpWidget(
      CupertinoApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: CupertinoSlider(
                    key: sliderKey,
                    value: value,
                    onChanged: (double newValue) {
                      setState(() {
                        value = newValue;
                      });
                    },
                    onChangeEnd: (double value) {
                      numberOfTimesOnChangeEndIsCalled++;
                    },
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );

    await dragSlider(tester, sliderKey);

    expect(numberOfTimesOnChangeEndIsCalled, equals(1));

    await tester.pump(); // No animation should start.
    // Check the transientCallbackCount before tearing down the widget to ensure
    // that no animation is running.
    expect(SchedulerBinding.instance.transientCallbackCount, equals(0));
  });

  testWidgets('Slider moves when dragged (LTR)', (WidgetTester tester) async {
    final Key sliderKey = UniqueKey();
    double value = 0.0;
    late double startValue;
    late double endValue;

    await tester.pumpWidget(
      CupertinoApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: CupertinoSlider(
                    key: sliderKey,
                    value: value,
                    onChanged: (double newValue) {
                      setState(() {
                        value = newValue;
                      });
                    },
                    onChangeStart: (double value) {
                      startValue = value;
                    },
                    onChangeEnd: (double value) {
                      endValue = value;
                    },
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );

    expect(value, equals(0.0));

    final Offset topLeft = tester.getTopLeft(find.byKey(sliderKey));
    const double unit = CupertinoThumbPainter.radius;
    const double delta = 3.0 * unit;
    await tester.dragFrom(topLeft + const Offset(unit, unit), const Offset(delta, 0.0));

    final Size size = tester.getSize(find.byKey(sliderKey));
    final double finalValue = delta / (size.width - 2.0 * (8.0 + CupertinoThumbPainter.radius));
    expect(startValue, equals(0.0));
    expect(value, equals(finalValue));
    expect(endValue, equals(finalValue));

    await tester.pump(); // No animation should start.
    // Check the transientCallbackCount before tearing down the widget to ensure
    // that no animation is running.
    expect(SchedulerBinding.instance.transientCallbackCount, equals(0));
  });

  testWidgets('Slider moves when dragged (RTL)', (WidgetTester tester) async {
    final Key sliderKey = UniqueKey();
    double value = 0.0;
    late double startValue;
    late double endValue;

    await tester.pumpWidget(
      CupertinoApp(
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: CupertinoSlider(
                    key: sliderKey,
                    value: value,
                    onChanged: (double newValue) {
                      setState(() {
                        value = newValue;
                      });
                    },
                    onChangeStart: (double value) {
                      setState(() {
                        startValue = value;
                      });
                    },
                    onChangeEnd: (double value) {
                      setState(() {
                        endValue = value;
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

    expect(value, equals(0.0));

    final Offset bottomRight = tester.getBottomRight(find.byKey(sliderKey));
    const double unit = CupertinoThumbPainter.radius;
    const double delta = 3.0 * unit;
    await tester.dragFrom(bottomRight - const Offset(unit, unit), const Offset(-delta, 0.0));

    final Size size = tester.getSize(find.byKey(sliderKey));
    final double finalValue = delta / (size.width - 2.0 * (8.0 + CupertinoThumbPainter.radius));
    expect(startValue, equals(0.0));
    expect(value, equals(finalValue));
    expect(endValue, equals(finalValue));

    await tester.pump(); // No animation should start.
    // Check the transientCallbackCount before tearing down the widget to ensure
    // that no animation is running.
    expect(SchedulerBinding.instance.transientCallbackCount, equals(0));
  });

  testWidgets('Slider Semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: CupertinoSlider(value: 0.5, onChanged: (double v) {}),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              value: '50%',
              increasedValue: '60%',
              decreasedValue: '40%',
              textDirection: TextDirection.ltr,
              flags: <SemanticsFlag>[SemanticsFlag.isSlider],
              actions: SemanticsAction.decrease.index | SemanticsAction.increase.index,
            ),
          ],
        ),
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );

    // Disable slider
    await tester.pumpWidget(
      const MediaQuery(
        data: MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: CupertinoSlider(value: 0.5, onChanged: null),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(id: 1, flags: <SemanticsFlag>[SemanticsFlag.isSlider]),
          ],
        ),
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('Slider Semantics can be updated', (WidgetTester tester) async {
    final SemanticsHandle handle = tester.ensureSemantics();
    double value = 0.5;
    await tester.pumpWidget(
      CupertinoApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: CupertinoSlider(value: value, onChanged: (double v) {}),
        ),
      ),
    );

    expect(
      tester.getSemantics(find.byType(CupertinoSlider)),
      matchesSemantics(
        isSlider: true,
        hasIncreaseAction: true,
        hasDecreaseAction: true,
        value: '50%',
        increasedValue: '60%',
        decreasedValue: '40%',
        textDirection: TextDirection.ltr,
      ),
    );

    value = 0.6;
    await tester.pumpWidget(
      CupertinoApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: CupertinoSlider(value: value, onChanged: (double v) {}),
        ),
      ),
    );

    expect(
      tester.getSemantics(find.byType(CupertinoSlider)),
      matchesSemantics(
        isSlider: true,
        hasIncreaseAction: true,
        hasDecreaseAction: true,
        value: '60%',
        increasedValue: '70%',
        decreasedValue: '50%',
        textDirection: TextDirection.ltr,
      ),
    );

    handle.dispose();
  });

  testWidgets('Slider respects themes', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(home: Center(child: CupertinoSlider(onChanged: (double value) {}, value: 0.5))),
    );
    expect(
      find.byType(CupertinoSlider),
      // First line it paints is blue.
      paints..rrect(color: CupertinoColors.systemBlue.color),
    );

    await tester.pumpWidget(
      CupertinoApp(
        theme: const CupertinoThemeData(brightness: Brightness.dark),
        home: Center(child: CupertinoSlider(onChanged: (double value) {}, value: 0.5)),
      ),
    );

    expect(
      find.byType(CupertinoSlider),
      paints..rrect(color: CupertinoColors.systemBlue.darkColor),
    );
  });

  testWidgets('Themes can be overridden', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        theme: const CupertinoThemeData(brightness: Brightness.dark),
        home: Center(
          child: CupertinoSlider(
            activeColor: CupertinoColors.activeGreen,
            onChanged: (double value) {},
            value: 0.5,
          ),
        ),
      ),
    );
    expect(
      find.byType(CupertinoSlider),
      paints..rrect(color: CupertinoColors.systemGreen.darkColor),
    );
  });

  testWidgets('Themes can be overridden by dynamic colors', (WidgetTester tester) async {
    const CupertinoDynamicColor activeColor = CupertinoDynamicColor(
      color: Color(0x00000001),
      darkColor: Color(0x00000002),
      elevatedColor: Color(0x00000003),
      highContrastColor: Color(0x00000004),
      darkElevatedColor: Color(0x00000005),
      darkHighContrastColor: Color(0x00000006),
      highContrastElevatedColor: Color(0x00000007),
      darkHighContrastElevatedColor: Color(0x00000008),
    );

    Widget withTraits(
      Brightness brightness,
      CupertinoUserInterfaceLevelData level,
      bool highContrast,
    ) {
      return CupertinoTheme(
        data: CupertinoThemeData(brightness: brightness),
        child: CupertinoUserInterfaceLevel(
          data: level,
          child: MediaQuery(
            data: MediaQueryData(highContrast: highContrast),
            child: Center(
              child: CupertinoSlider(
                activeColor: activeColor,
                onChanged: (double value) {},
                value: 0.5,
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(
      CupertinoApp(home: withTraits(Brightness.light, CupertinoUserInterfaceLevelData.base, false)),
    );
    expect(find.byType(CupertinoSlider), paints..rrect(color: activeColor.color));

    await tester.pumpWidget(
      CupertinoApp(home: withTraits(Brightness.dark, CupertinoUserInterfaceLevelData.base, false)),
    );
    expect(find.byType(CupertinoSlider), paints..rrect(color: activeColor.darkColor));

    await tester.pumpWidget(
      CupertinoApp(
        home: withTraits(Brightness.dark, CupertinoUserInterfaceLevelData.elevated, false),
      ),
    );
    expect(find.byType(CupertinoSlider), paints..rrect(color: activeColor.darkElevatedColor));

    await tester.pumpWidget(
      CupertinoApp(home: withTraits(Brightness.dark, CupertinoUserInterfaceLevelData.base, true)),
    );
    expect(find.byType(CupertinoSlider), paints..rrect(color: activeColor.darkHighContrastColor));

    await tester.pumpWidget(
      CupertinoApp(
        home: withTraits(Brightness.dark, CupertinoUserInterfaceLevelData.elevated, true),
      ),
    );
    expect(
      find.byType(CupertinoSlider),
      paints..rrect(color: activeColor.darkHighContrastElevatedColor),
    );

    await tester.pumpWidget(
      CupertinoApp(home: withTraits(Brightness.light, CupertinoUserInterfaceLevelData.base, true)),
    );
    expect(find.byType(CupertinoSlider), paints..rrect(color: activeColor.highContrastColor));

    await tester.pumpWidget(
      CupertinoApp(
        home: withTraits(Brightness.light, CupertinoUserInterfaceLevelData.elevated, false),
      ),
    );
    expect(find.byType(CupertinoSlider), paints..rrect(color: activeColor.elevatedColor));

    await tester.pumpWidget(
      CupertinoApp(
        home: withTraits(Brightness.light, CupertinoUserInterfaceLevelData.elevated, true),
      ),
    );
    expect(
      find.byType(CupertinoSlider),
      paints..rrect(color: activeColor.highContrastElevatedColor),
    );
  });

  testWidgets('track color is dynamic', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        theme: const CupertinoThemeData(brightness: Brightness.light),
        home: Center(
          child: CupertinoSlider(
            activeColor: CupertinoColors.activeGreen,
            onChanged: (double value) {},
            value: 0,
          ),
        ),
      ),
    );

    expect(find.byType(CupertinoSlider), paints..rrect(color: _kSystemFill.color));

    expect(find.byType(CupertinoSlider), isNot(paints..rrect(color: _kSystemFill.darkColor)));

    await tester.pumpWidget(
      CupertinoApp(
        theme: const CupertinoThemeData(brightness: Brightness.dark),
        home: Center(
          child: CupertinoSlider(
            activeColor: CupertinoColors.activeGreen,
            onChanged: (double value) {},
            value: 0,
          ),
        ),
      ),
    );

    expect(find.byType(CupertinoSlider), paints..rrect(color: _kSystemFill.darkColor));

    expect(find.byType(CupertinoSlider), isNot(paints..rrect(color: _kSystemFill.color)));
  });

  testWidgets('Thumb color can be overridden', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoSlider(
            thumbColor: CupertinoColors.systemPurple,
            onChanged: (double value) {},
            value: 0,
          ),
        ),
      ),
    );

    expect(
      find.byType(CupertinoSlider),
      paints
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect(color: CupertinoColors.systemPurple.color),
    );

    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoSlider(
            thumbColor: CupertinoColors.activeOrange,
            onChanged: (double value) {},
            value: 0,
          ),
        ),
      ),
    );

    expect(
      find.byType(CupertinoSlider),
      paints
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect(color: CupertinoColors.activeOrange.color),
    );
  });

  testWidgets('Hovering over Cupertino slider thumb updates cursor to clickable on Web', (
    WidgetTester tester,
  ) async {
    final Key sliderKey = UniqueKey();
    double value = 0.0;

    await tester.pumpWidget(
      CupertinoApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: CupertinoSlider(
                    key: sliderKey,
                    value: value,
                    onChanged: (double newValue) {
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

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: const Offset(10, 10));
    await tester.pumpAndSettle();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );

    final Offset topLeft = tester.getTopLeft(find.byKey(sliderKey));
    await gesture.moveTo(topLeft + const Offset(15, 0));
    addTearDown(gesture.removePointer);
    await tester.pumpAndSettle();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      kIsWeb ? SystemMouseCursors.click : SystemMouseCursors.basic,
    );
  });
}
