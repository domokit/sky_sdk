// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';
import 'package:flutter/src/physics/utils.dart' show nearEqual;
import 'package:flutter_test/flutter_test.dart';

import '../widgets/semantics_tester.dart';

// A thumb shape that also logs its repaint center.
class LoggingThumbShape extends SliderComponentShape {
  LoggingThumbShape(this.log);

  final List<Offset> log;

  @override
  Size getPreferredSize(bool isEnabled, bool isDiscrete) {
    return const Size(10.0, 10.0);
  }

  @override
  void paint(
    PaintingContext context,
    Offset thumbCenter, {
    required Animation<double> activationAnimation,
    required Animation<double> enableAnimation,
    required bool isDiscrete,
    required TextPainter labelPainter,
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    required TextDirection textDirection,
    required double value,
    required double textScaleFactor,
    required Size sizeWithOverflow,
  }) {
    log.add(thumbCenter);
    final Paint thumbPaint = Paint()..color = Colors.red;
    context.canvas.drawCircle(thumbCenter, 5.0, thumbPaint);
  }
}

// A value indicator shape to log labelPainter text.
class LoggingValueIndicatorShape extends SliderComponentShape {
  LoggingValueIndicatorShape(this.logLabel);

  final List<InlineSpan> logLabel;

  @override
  Size getPreferredSize(bool isEnabled, bool isDiscrete) {
    return const Size(10.0, 10.0);
  }

  @override
  void paint(
    PaintingContext context,
    Offset offset, {
    required Animation<double> activationAnimation,
    required Animation<double> enableAnimation,
    required bool isDiscrete,
    required TextPainter labelPainter,
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    required TextDirection textDirection,
    required double value,
    required double textScaleFactor,
    required Size sizeWithOverflow,
  }) {
    logLabel.add(labelPainter.text!);
  }
}

class TallSliderTickMarkShape extends SliderTickMarkShape {
  @override
  Size getPreferredSize({required SliderThemeData sliderTheme, required bool isEnabled}) {
    return const Size(10.0, 200.0);
  }

  @override
  void paint(
    PaintingContext context,
    Offset offset, {
    required Offset thumbCenter,
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    required Animation<double> enableAnimation,
    required bool isEnabled,
    required TextDirection textDirection,
  }) {
    final Paint paint = Paint()..color = Colors.red;
    context.canvas.drawRect(Rect.fromLTWH(offset.dx, offset.dy, 10.0, 20.0), paint);
  }
}

class _StateDependentMouseCursor extends MaterialStateMouseCursor {
  const _StateDependentMouseCursor({
    this.disabled = SystemMouseCursors.none,
    this.dragged = SystemMouseCursors.none,
    this.hovered = SystemMouseCursors.none,
  });

  final MouseCursor disabled;
  final MouseCursor hovered;
  final MouseCursor dragged;

  @override
  MouseCursor resolve(Set<MaterialState> states) {
    if (states.contains(MaterialState.disabled)) {
      return disabled;
    }
    if (states.contains(MaterialState.dragged)) {
      return dragged;
    }
    if (states.contains(MaterialState.hovered)) {
      return hovered;
    }
    return SystemMouseCursors.none;
  }

  @override
  String get debugDescription => '_StateDependentMouseCursor';
}

void main() {
  testWidgets('The initial value should respect the discrete value', (WidgetTester tester) async {
    final Key sliderKey = UniqueKey();
    double value = 0.20;
    final List<Offset> log = <Offset>[];
    final LoggingThumbShape loggingThumb = LoggingThumbShape(log);
    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              final SliderThemeData sliderTheme = SliderTheme.of(
                context,
              ).copyWith(thumbShape: loggingThumb);
              return Material(
                child: Center(
                  child: SliderTheme(
                    data: sliderTheme,
                    child: Slider(
                      key: sliderKey,
                      value: value,
                      divisions: 4,
                      onChanged: (double newValue) {
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
      ),
    );

    expect(value, equals(0.20));
    expect(log.length, 1);
    expect(log[0], const Offset(213.0, 300.0));
  });

  testWidgets('Slider can move when tapped (LTR)', (WidgetTester tester) async {
    final Key sliderKey = UniqueKey();
    double value = 0.0;
    double? startValue;
    double? endValue;

    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: Slider(
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
    await tester.tap(find.byKey(sliderKey));
    expect(value, equals(0.5));
    expect(startValue, equals(0.0));
    expect(endValue, equals(0.5));
    startValue = null;
    endValue = null;
    await tester.pump(); // No animation should start.
    expect(SchedulerBinding.instance.transientCallbackCount, equals(0));

    final Offset topLeft = tester.getTopLeft(find.byKey(sliderKey));
    final Offset bottomRight = tester.getBottomRight(find.byKey(sliderKey));

    final Offset target = topLeft + (bottomRight - topLeft) / 4.0;
    await tester.tapAt(target);
    expect(value, moreOrLessEquals(0.25, epsilon: 0.05));
    expect(startValue, equals(0.5));
    expect(endValue, moreOrLessEquals(0.25, epsilon: 0.05));
    await tester.pump(); // No animation should start.
    expect(SchedulerBinding.instance.transientCallbackCount, equals(0));
  });

  testWidgets('Slider can move when tapped (RTL)', (WidgetTester tester) async {
    final Key sliderKey = UniqueKey();
    double value = 0.0;

    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: Slider(
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
    await tester.tap(find.byKey(sliderKey));
    expect(value, equals(0.5));
    await tester.pump(); // No animation should start.
    expect(SchedulerBinding.instance.transientCallbackCount, equals(0));

    final Offset topLeft = tester.getTopLeft(find.byKey(sliderKey));
    final Offset bottomRight = tester.getBottomRight(find.byKey(sliderKey));

    final Offset target = topLeft + (bottomRight - topLeft) / 4.0;
    await tester.tapAt(target);
    expect(value, moreOrLessEquals(0.75, epsilon: 0.05));
    await tester.pump(); // No animation should start.
    expect(SchedulerBinding.instance.transientCallbackCount, equals(0));
  });

  testWidgets("Slider doesn't send duplicate change events if tapped on the same value", (
    WidgetTester tester,
  ) async {
    final Key sliderKey = UniqueKey();
    double value = 0.0;
    late double startValue;
    late double endValue;
    int updates = 0;
    int startValueUpdates = 0;
    int endValueUpdates = 0;

    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: Slider(
                    key: sliderKey,
                    value: value,
                    onChanged: (double newValue) {
                      setState(() {
                        updates++;
                        value = newValue;
                      });
                    },
                    onChangeStart: (double value) {
                      startValueUpdates++;
                      startValue = value;
                    },
                    onChangeEnd: (double value) {
                      endValueUpdates++;
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
    await tester.tap(find.byKey(sliderKey));
    expect(value, equals(0.5));
    expect(startValue, equals(0.0));
    expect(endValue, equals(0.5));
    await tester.pump();
    await tester.tap(find.byKey(sliderKey));
    expect(value, equals(0.5));
    await tester.pump();
    expect(updates, equals(1));
    expect(startValueUpdates, equals(2));
    expect(endValueUpdates, equals(2));
  });

  testWidgets('Value indicator shows for a bit after being tapped', (WidgetTester tester) async {
    final Key sliderKey = UniqueKey();
    double value = 0.0;

    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: Slider(
                    key: sliderKey,
                    value: value,
                    divisions: 4,
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
    await tester.tap(find.byKey(sliderKey));
    expect(value, equals(0.5));
    await tester.pump(const Duration(milliseconds: 100));
    // Starts with the position animation and value indicator
    expect(SchedulerBinding.instance.transientCallbackCount, equals(2));
    await tester.pump(const Duration(milliseconds: 100));
    // Value indicator is longer than position.
    expect(SchedulerBinding.instance.transientCallbackCount, equals(1));
    await tester.pump(const Duration(milliseconds: 100));
    expect(SchedulerBinding.instance.transientCallbackCount, equals(0));
    await tester.pump(const Duration(milliseconds: 100));
    expect(SchedulerBinding.instance.transientCallbackCount, equals(0));
    await tester.pump(const Duration(milliseconds: 100));
    // Shown for long enough, value indicator is animated closed.
    expect(SchedulerBinding.instance.transientCallbackCount, equals(1));
    await tester.pump(const Duration(milliseconds: 101));
    expect(SchedulerBinding.instance.transientCallbackCount, equals(0));
  });

  testWidgets('Discrete Slider repaints and animates when dragged', (WidgetTester tester) async {
    final Key sliderKey = UniqueKey();
    double value = 0.0;
    final List<Offset> log = <Offset>[];
    final LoggingThumbShape loggingThumb = LoggingThumbShape(log);
    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              final SliderThemeData sliderTheme = SliderTheme.of(
                context,
              ).copyWith(thumbShape: loggingThumb);
              return Material(
                child: Center(
                  child: SliderTheme(
                    data: sliderTheme,
                    child: Slider(
                      key: sliderKey,
                      value: value,
                      divisions: 4,
                      onChanged: (double newValue) {
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
      ),
    );

    final List<Offset> expectedLog = <Offset>[
      const Offset(26.0, 300.0),
      const Offset(26.0, 300.0),
      const Offset(400.0, 300.0),
    ];
    final TestGesture gesture = await tester.startGesture(tester.getCenter(find.byKey(sliderKey)));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));
    expect(value, equals(0.5));
    expect(log.length, 3);
    expect(log, orderedEquals(expectedLog));
    await gesture.moveBy(const Offset(-500.0, 0.0));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 10));
    expect(value, equals(0.0));
    expect(log.length, 5);
    expect(log.last.dx, moreOrLessEquals(386.6, epsilon: 0.1));
    // With no more gesture or value changes, the thumb position should still
    // be redrawn in the animated position.
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 10));
    expect(value, equals(0.0));
    expect(log.length, 7);
    expect(log.last.dx, moreOrLessEquals(344.8, epsilon: 0.1));
    // Final position.
    await tester.pump(const Duration(milliseconds: 80));
    expectedLog.add(const Offset(26.0, 300.0));
    expect(value, equals(0.0));
    expect(log.length, 8);
    expect(log.last.dx, moreOrLessEquals(26.0, epsilon: 0.1));
    await gesture.up();
  });

  testWidgets("Slider doesn't send duplicate change events if tapped on the same value", (
    WidgetTester tester,
  ) async {
    final Key sliderKey = UniqueKey();
    double value = 0.0;
    int updates = 0;

    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: Slider(
                    key: sliderKey,
                    value: value,
                    onChanged: (double newValue) {
                      setState(() {
                        updates++;
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
    await tester.tap(find.byKey(sliderKey));
    expect(value, equals(0.5));
    await tester.pump();
    await tester.tap(find.byKey(sliderKey));
    expect(value, equals(0.5));
    await tester.pump();
    expect(updates, equals(1));
  });

  testWidgets('Discrete Slider repaints when dragged', (WidgetTester tester) async {
    final Key sliderKey = UniqueKey();
    double value = 0.0;
    final List<Offset> log = <Offset>[];
    final LoggingThumbShape loggingThumb = LoggingThumbShape(log);
    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              final SliderThemeData sliderTheme = SliderTheme.of(
                context,
              ).copyWith(thumbShape: loggingThumb);
              return Material(
                child: Center(
                  child: SliderTheme(
                    data: sliderTheme,
                    child: Slider(
                      key: sliderKey,
                      value: value,
                      divisions: 4,
                      onChanged: (double newValue) {
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
      ),
    );

    final List<Offset> expectedLog = <Offset>[
      const Offset(26.0, 300.0),
      const Offset(26.0, 300.0),
      const Offset(400.0, 300.0),
    ];
    final TestGesture gesture = await tester.startGesture(tester.getCenter(find.byKey(sliderKey)));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));
    expect(value, equals(0.5));
    expect(log.length, 3);
    expect(log, orderedEquals(expectedLog));
    await gesture.moveBy(const Offset(-500.0, 0.0));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 10));
    expect(value, equals(0.0));
    expect(log.length, 5);
    expect(log.last.dx, moreOrLessEquals(386.6, epsilon: 0.1));
    // With no more gesture or value changes, the thumb position should still
    // be redrawn in the animated position.
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 10));
    expect(value, equals(0.0));
    expect(log.length, 7);
    expect(log.last.dx, moreOrLessEquals(344.8, epsilon: 0.1));
    // Final position.
    await tester.pump(const Duration(milliseconds: 80));
    expectedLog.add(const Offset(26.0, 300.0));
    expect(value, equals(0.0));
    expect(log.length, 8);
    expect(log.last.dx, moreOrLessEquals(26.0, epsilon: 0.1));
    await gesture.up();
  });

  testWidgets('Slider take on discrete values', (WidgetTester tester) async {
    final Key sliderKey = UniqueKey();
    double value = 0.0;

    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: SizedBox(
                    width: 144.0 + 2 * 16.0, // _kPreferredTotalWidth
                    child: Slider(
                      key: sliderKey,
                      max: 100.0,
                      divisions: 10,
                      value: value,
                      onChanged: (double newValue) {
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
      ),
    );

    expect(value, equals(0.0));
    await tester.tap(find.byKey(sliderKey));
    expect(value, equals(50.0));
    await tester.drag(find.byKey(sliderKey), const Offset(5.0, 0.0));
    expect(value, equals(50.0));
    await tester.drag(find.byKey(sliderKey), const Offset(40.0, 0.0));
    expect(value, equals(80.0));

    await tester.pump(); // Starts animation.
    expect(SchedulerBinding.instance.transientCallbackCount, greaterThan(0));
    await tester.pump(const Duration(milliseconds: 200));
    await tester.pump(const Duration(milliseconds: 200));
    await tester.pump(const Duration(milliseconds: 200));
    await tester.pump(const Duration(milliseconds: 200));
    // Animation complete.
    expect(SchedulerBinding.instance.transientCallbackCount, equals(0));
  });

  testWidgets('Slider can be given zero values', (WidgetTester tester) async {
    final List<double> log = <double>[];
    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Slider(
              value: 0.0,
              onChanged: (double newValue) {
                log.add(newValue);
              },
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(Slider));
    expect(log, <double>[0.5]);
    log.clear();

    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Slider(
              value: 0.0,
              max: 0.0,
              onChanged: (double newValue) {
                log.add(newValue);
              },
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(Slider));
    expect(log, <double>[]);
    log.clear();
  });

  testWidgets('Slider can tap in vertical scroller', (WidgetTester tester) async {
    double value = 0.0;
    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: ListView(
              children: <Widget>[
                Slider(
                  value: value,
                  onChanged: (double newValue) {
                    value = newValue;
                  },
                ),
                Container(height: 2000.0),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(Slider));
    expect(value, equals(0.5));
  });

  testWidgets('Slider drags immediately (LTR)', (WidgetTester tester) async {
    double value = 0.0;
    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: Slider(
                value: value,
                onChanged: (double newValue) {
                  value = newValue;
                },
              ),
            ),
          ),
        ),
      ),
    );

    final Offset center = tester.getCenter(find.byType(Slider));
    final TestGesture gesture = await tester.startGesture(center);

    expect(value, equals(0.5));

    await gesture.moveBy(const Offset(1.0, 0.0));

    expect(value, greaterThan(0.5));

    await gesture.up();
  });

  testWidgets('Slider drags immediately (RTL)', (WidgetTester tester) async {
    double value = 0.0;
    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: Material(
            child: Center(
              child: Slider(
                value: value,
                onChanged: (double newValue) {
                  value = newValue;
                },
              ),
            ),
          ),
        ),
      ),
    );

    final Offset center = tester.getCenter(find.byType(Slider));
    final TestGesture gesture = await tester.startGesture(center);

    expect(value, equals(0.5));

    await gesture.moveBy(const Offset(1.0, 0.0));

    expect(value, lessThan(0.5));

    await gesture.up();
  });

  testWidgets('Slider onChangeStart and onChangeEnd fire once', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/28115

    int startFired = 0;
    int endFired = 0;
    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: GestureDetector(
                onHorizontalDragUpdate: (_) {},
                child: Slider(
                  value: 0.0,
                  onChanged: (double newValue) {},
                  onChangeStart: (double value) {
                    startFired += 1;
                  },
                  onChangeEnd: (double value) {
                    endFired += 1;
                  },
                ),
              ),
            ),
          ),
        ),
      ),
    );

    await tester.timedDrag(
      find.byType(Slider),
      const Offset(20.0, 0.0),
      const Duration(milliseconds: 100),
    );

    expect(startFired, equals(1));
    expect(endFired, equals(1));
  });

  testWidgets('Slider sizing', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(child: Center(child: Slider(value: 0.5, onChanged: null))),
        ),
      ),
    );
    expect(tester.renderObject<RenderBox>(find.byType(Slider)).size, const Size(800.0, 600.0));

    await tester.pumpWidget(
      const MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(child: IntrinsicWidth(child: Slider(value: 0.5, onChanged: null))),
          ),
        ),
      ),
    );
    expect(
      tester.renderObject<RenderBox>(find.byType(Slider)).size,
      const Size(144.0 + 2.0 * 24.0, 600.0),
    );

    await tester.pumpWidget(
      const MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: OverflowBox(
                maxWidth: double.infinity,
                maxHeight: double.infinity,
                child: Slider(value: 0.5, onChanged: null),
              ),
            ),
          ),
        ),
      ),
    );
    expect(
      tester.renderObject<RenderBox>(find.byType(Slider)).size,
      const Size(144.0 + 2.0 * 24.0, 48.0),
    );
  });

  testWidgets('Slider respects textScaleFactor', (WidgetTester tester) async {
    debugDisableShadows = false;
    try {
      final Key sliderKey = UniqueKey();
      double value = 0.0;

      Widget buildSlider({
        required double textScaleFactor,
        bool isDiscrete = true,
        ShowValueIndicator show = ShowValueIndicator.onlyForDiscrete,
      }) {
        return MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Directionality(
            textDirection: TextDirection.ltr,
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return MediaQuery(
                  data: MediaQueryData(textScaler: TextScaler.linear(textScaleFactor)),
                  child: Material(
                    child: Theme(
                      data: Theme.of(context).copyWith(
                        sliderTheme: Theme.of(
                          context,
                        ).sliderTheme.copyWith(showValueIndicator: show),
                      ),
                      child: Center(
                        child: OverflowBox(
                          maxWidth: double.infinity,
                          maxHeight: double.infinity,
                          child: Slider(
                            key: sliderKey,
                            max: 100.0,
                            divisions: isDiscrete ? 10 : null,
                            label: '${value.round()}',
                            value: value,
                            onChanged: (double newValue) {
                              setState(() {
                                value = newValue;
                              });
                            },
                          ),
                        ),
                      ),
                    ),
                  ),
                );
              },
            ),
          ),
        );
      }

      await tester.pumpWidget(buildSlider(textScaleFactor: 1.0));
      Offset center = tester.getCenter(find.byType(Slider));
      TestGesture gesture = await tester.startGesture(center);
      await tester.pumpAndSettle();

      expect(
        tester.renderObject(find.byType(Overlay)),
        paints
          ..path(
            includes: const <Offset>[
              Offset.zero,
              Offset(0.0, -8.0),
              Offset(-276.0, -16.0),
              Offset(-216.0, -16.0),
            ],
            color: const Color(0xf55f5f5f),
          )
          ..paragraph(),
      );

      await gesture.up();
      await tester.pumpAndSettle();

      await tester.pumpWidget(buildSlider(textScaleFactor: 2.0));
      center = tester.getCenter(find.byType(Slider));
      gesture = await tester.startGesture(center);
      await tester.pumpAndSettle();

      expect(
        tester.renderObject(find.byType(Overlay)),
        paints
          ..path(
            includes: const <Offset>[
              Offset.zero,
              Offset(0.0, -8.0),
              Offset(-304.0, -16.0),
              Offset(-216.0, -16.0),
            ],
            color: const Color(0xf55f5f5f),
          )
          ..paragraph(),
      );

      await gesture.up();
      await tester.pumpAndSettle();

      // Check continuous
      await tester.pumpWidget(
        buildSlider(
          textScaleFactor: 1.0,
          isDiscrete: false,
          show: ShowValueIndicator.onlyForContinuous,
        ),
      );
      center = tester.getCenter(find.byType(Slider));
      gesture = await tester.startGesture(center);
      await tester.pumpAndSettle();

      expect(
        tester.renderObject(find.byType(Overlay)),
        paints
          ..path(
            includes: const <Offset>[
              Offset.zero,
              Offset(0.0, -8.0),
              Offset(-276.0, -16.0),
              Offset(-216.0, -16.0),
            ],
            color: const Color(0xf55f5f5f),
          )
          ..paragraph(),
      );

      await gesture.up();
      await tester.pumpAndSettle();

      await tester.pumpWidget(
        buildSlider(
          textScaleFactor: 2.0,
          isDiscrete: false,
          show: ShowValueIndicator.onlyForContinuous,
        ),
      );
      center = tester.getCenter(find.byType(Slider));
      gesture = await tester.startGesture(center);
      await tester.pumpAndSettle();

      expect(
        tester.renderObject(find.byType(Overlay)),
        paints
          ..path(
            includes: const <Offset>[
              Offset.zero,
              Offset(0.0, -8.0),
              Offset(-276.0, -16.0),
              Offset(-216.0, -16.0),
            ],
            color: const Color(0xf55f5f5f),
          )
          ..paragraph(),
      );

      await gesture.up();
      await tester.pumpAndSettle();
    } finally {
      debugDisableShadows = true;
    }
  });

  testWidgets('Slider value indicator respects bold text', (WidgetTester tester) async {
    final Key sliderKey = UniqueKey();
    double value = 0.0;
    final List<InlineSpan> log = <InlineSpan>[];
    final LoggingValueIndicatorShape loggingValueIndicatorShape = LoggingValueIndicatorShape(log);

    Widget buildSlider({bool boldText = false}) {
      return MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return MediaQuery(
                data: MediaQueryData(boldText: boldText),
                child: Material(
                  child: Theme(
                    data: Theme.of(context).copyWith(
                      sliderTheme: Theme.of(context).sliderTheme.copyWith(
                        showValueIndicator: ShowValueIndicator.always,
                        valueIndicatorShape: loggingValueIndicatorShape,
                      ),
                    ),
                    child: Center(
                      child: OverflowBox(
                        maxWidth: double.infinity,
                        maxHeight: double.infinity,
                        child: Slider(
                          key: sliderKey,
                          max: 100.0,
                          divisions: 4,
                          label: '${value.round()}',
                          value: value,
                          onChanged: (double newValue) {
                            setState(() {
                              value = newValue;
                            });
                          },
                        ),
                      ),
                    ),
                  ),
                ),
              );
            },
          ),
        ),
      );
    }

    // Normal text
    await tester.pumpWidget(buildSlider());
    Offset center = tester.getCenter(find.byType(Slider));
    TestGesture gesture = await tester.startGesture(center);
    await tester.pumpAndSettle();

    expect(log.last.toPlainText(), '50');
    expect(log.last.style!.fontWeight, FontWeight.w500);

    await gesture.up();
    await tester.pumpAndSettle();

    // Bold text
    await tester.pumpWidget(buildSlider(boldText: true));
    center = tester.getCenter(find.byType(Slider));
    gesture = await tester.startGesture(center);
    await tester.pumpAndSettle();

    expect(log.last.toPlainText(), '50');
    expect(log.last.style!.fontWeight, FontWeight.w700);

    await gesture.up();
    await tester.pumpAndSettle();
  });

  testWidgets('Tick marks are skipped when they are too dense', (WidgetTester tester) async {
    Widget buildSlider({required int divisions}) {
      return MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: Slider(
                max: 100.0,
                divisions: divisions,
                value: 0.25,
                onChanged: (double newValue) {},
              ),
            ),
          ),
        ),
      );
    }

    // Pump a slider with a reasonable amount of divisions to verify that the
    // tick marks are drawn when the number of tick marks is not too dense.
    await tester.pumpWidget(buildSlider(divisions: 4));

    final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));

    // 5 tick marks and a thumb.
    expect(material, paintsExactlyCountTimes(#drawCircle, 6));

    // 200 divisions will produce a tick interval off less than 6,
    // which would be too dense to draw.
    await tester.pumpWidget(buildSlider(divisions: 200));

    // No tick marks are drawn because they are too dense, but the thumb is
    // still drawn.
    expect(material, paintsExactlyCountTimes(#drawCircle, 1));
  });

  testWidgets('Slider has correct animations when reparented', (WidgetTester tester) async {
    final Key sliderKey = GlobalKey(debugLabel: 'A');
    double value = 0.0;

    Widget buildSlider(int parents) {
      Widget createParents(int parents, StateSetter setState) {
        Widget slider = Slider(
          key: sliderKey,
          value: value,
          divisions: 4,
          onChanged: (double newValue) {
            setState(() {
              value = newValue;
            });
          },
        );

        for (int i = 0; i < parents; ++i) {
          slider = Column(children: <Widget>[slider]);
        }
        return slider;
      }

      return MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(child: createParents(parents, setState));
            },
          ),
        ),
      );
    }

    Future<void> testReparenting(bool reparent) async {
      final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));
      final Offset center = tester.getCenter(find.byType(Slider));
      // Move to 0.0.
      TestGesture gesture = await tester.startGesture(Offset.zero);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      expect(SchedulerBinding.instance.transientCallbackCount, equals(0));
      expect(
        material,
        paints
          ..circle(x: 26.0, y: 24.0, radius: 1.0)
          ..circle(x: 213.0, y: 24.0, radius: 1.0)
          ..circle(x: 400.0, y: 24.0, radius: 1.0)
          ..circle(x: 587.0, y: 24.0, radius: 1.0)
          ..circle(x: 774.0, y: 24.0, radius: 1.0)
          ..circle(x: 26.0, y: 24.0, radius: 10.0),
      );

      gesture = await tester.startGesture(center);
      await tester.pump();
      // Wait for animations to start.
      await tester.pump(const Duration(milliseconds: 25));
      expect(SchedulerBinding.instance.transientCallbackCount, equals(2));
      expect(
        material,
        paints
          ..circle(x: 112.7431640625, y: 24.0, radius: 5.687664985656738)
          ..circle(x: 26.0, y: 24.0, radius: 1.0)
          ..circle(x: 213.0, y: 24.0, radius: 1.0)
          ..circle(x: 400.0, y: 24.0, radius: 1.0)
          ..circle(x: 587.0, y: 24.0, radius: 1.0)
          ..circle(x: 774.0, y: 24.0, radius: 1.0)
          ..circle(x: 112.7431640625, y: 24.0, radius: 10.0),
      );

      // Reparenting in the middle of an animation should do nothing.
      if (reparent) {
        await tester.pumpWidget(buildSlider(2));
      }

      // Move a little further in the animations.
      await tester.pump(const Duration(milliseconds: 10));
      expect(SchedulerBinding.instance.transientCallbackCount, equals(2));
      expect(
        material,
        paints
          ..circle(x: 191.130521774292, y: 24.0, radius: 12.0)
          ..circle(x: 26.0, y: 24.0, radius: 1.0)
          ..circle(x: 213.0, y: 24.0, radius: 1.0)
          ..circle(x: 400.0, y: 24.0, radius: 1.0)
          ..circle(x: 587.0, y: 24.0, radius: 1.0)
          ..circle(x: 774.0, y: 24.0, radius: 1.0)
          ..circle(x: 191.130521774292, y: 24.0, radius: 10.0),
      );
      // Wait for animations to finish.
      await tester.pumpAndSettle();
      expect(SchedulerBinding.instance.transientCallbackCount, equals(0));
      expect(
        material,
        paints
          ..circle(x: 400.0, y: 24.0, radius: 24.0)
          ..circle(x: 26.0, y: 24.0, radius: 1.0)
          ..circle(x: 213.0, y: 24.0, radius: 1.0)
          ..circle(x: 400.0, y: 24.0, radius: 1.0)
          ..circle(x: 587.0, y: 24.0, radius: 1.0)
          ..circle(x: 774.0, y: 24.0, radius: 1.0)
          ..circle(x: 400.0, y: 24.0, radius: 10.0),
      );
      await gesture.up();
      await tester.pumpAndSettle();
      expect(SchedulerBinding.instance.transientCallbackCount, equals(0));
      expect(
        material,
        paints
          ..circle(x: 26.0, y: 24.0, radius: 1.0)
          ..circle(x: 213.0, y: 24.0, radius: 1.0)
          ..circle(x: 400.0, y: 24.0, radius: 1.0)
          ..circle(x: 587.0, y: 24.0, radius: 1.0)
          ..circle(x: 774.0, y: 24.0, radius: 1.0)
          ..circle(x: 400.0, y: 24.0, radius: 10.0),
      );
    }

    await tester.pumpWidget(buildSlider(1));
    // Do it once without reparenting in the middle of an animation
    await testReparenting(false);
    // Now do it again with reparenting in the middle of an animation.
    await testReparenting(true);
  });

  testWidgets(
    'Slider Semantics',
    (WidgetTester tester) async {
      final SemanticsTester semantics = SemanticsTester(tester);

      await tester.pumpWidget(
        MaterialApp(
          home: Directionality(
            textDirection: TextDirection.ltr,
            child: Material(child: Slider(value: 0.5, onChanged: (double v) {})),
          ),
        ),
      );

      await tester.pumpAndSettle();

      expect(
        semantics,
        hasSemantics(
          TestSemantics.root(
            children: <TestSemantics>[
              TestSemantics(
                id: 1,
                textDirection: TextDirection.ltr,
                children: <TestSemantics>[
                  TestSemantics(
                    id: 2,
                    children: <TestSemantics>[
                      TestSemantics(
                        id: 3,
                        flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                        children: <TestSemantics>[
                          TestSemantics(
                            id: 4,
                            flags: <SemanticsFlag>[
                              SemanticsFlag.hasEnabledState,
                              SemanticsFlag.isEnabled,
                              SemanticsFlag.isFocusable,
                              SemanticsFlag.isSlider,
                            ],
                            actions: <SemanticsAction>[
                              SemanticsAction.focus,
                              SemanticsAction.increase,
                              SemanticsAction.decrease,
                            ],
                            value: '50%',
                            increasedValue: '55%',
                            decreasedValue: '45%',
                            textDirection: TextDirection.ltr,
                          ),
                        ],
                      ),
                    ],
                  ),
                ],
              ),
            ],
          ),
          ignoreRect: true,
          ignoreTransform: true,
        ),
      );

      // Disable slider
      await tester.pumpWidget(
        const MaterialApp(
          home: Directionality(
            textDirection: TextDirection.ltr,
            child: Material(child: Slider(value: 0.5, onChanged: null)),
          ),
        ),
      );

      expect(
        semantics,
        hasSemantics(
          TestSemantics.root(
            children: <TestSemantics>[
              TestSemantics(
                id: 1,
                textDirection: TextDirection.ltr,
                children: <TestSemantics>[
                  TestSemantics(
                    id: 2,
                    children: <TestSemantics>[
                      TestSemantics(
                        id: 3,
                        flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                        children: <TestSemantics>[
                          TestSemantics(
                            id: 4,
                            flags: <SemanticsFlag>[
                              SemanticsFlag.hasEnabledState,
                              // isFocusable is delayed by 1 frame.
                              SemanticsFlag.isFocusable,
                              SemanticsFlag.isSlider,
                            ],
                            actions: <SemanticsAction>[SemanticsAction.focus],
                            value: '50%',
                            increasedValue: '55%',
                            decreasedValue: '45%',
                            textDirection: TextDirection.ltr,
                          ),
                        ],
                      ),
                    ],
                  ),
                ],
              ),
            ],
          ),
          ignoreRect: true,
          ignoreTransform: true,
        ),
      );

      await tester.pump();
      expect(
        semantics,
        hasSemantics(
          TestSemantics.root(
            children: <TestSemantics>[
              TestSemantics(
                id: 1,
                textDirection: TextDirection.ltr,
                children: <TestSemantics>[
                  TestSemantics(
                    id: 2,
                    children: <TestSemantics>[
                      TestSemantics(
                        id: 3,
                        flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                        children: <TestSemantics>[
                          TestSemantics(
                            id: 4,
                            flags: <SemanticsFlag>[
                              SemanticsFlag.hasEnabledState,
                              SemanticsFlag.isSlider,
                            ],
                            value: '50%',
                            increasedValue: '55%',
                            decreasedValue: '45%',
                            textDirection: TextDirection.ltr,
                          ),
                        ],
                      ),
                    ],
                  ),
                ],
              ),
            ],
          ),
          ignoreRect: true,
          ignoreTransform: true,
        ),
      );

      semantics.dispose();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.fuchsia,
      TargetPlatform.linux,
    }),
  );

  testWidgets(
    'Slider Semantics',
    (WidgetTester tester) async {
      final SemanticsTester semantics = SemanticsTester(tester);

      await tester.pumpWidget(
        MaterialApp(
          home: Theme(
            data: ThemeData.light(),
            child: Directionality(
              textDirection: TextDirection.ltr,
              child: Material(child: Slider(value: 100.0, max: 200.0, onChanged: (double v) {})),
            ),
          ),
        ),
      );

      expect(
        semantics,
        hasSemantics(
          TestSemantics.root(
            children: <TestSemantics>[
              TestSemantics(
                id: 1,
                textDirection: TextDirection.ltr,
                children: <TestSemantics>[
                  TestSemantics(
                    id: 2,
                    children: <TestSemantics>[
                      TestSemantics(
                        id: 3,
                        flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                        children: <TestSemantics>[
                          TestSemantics(
                            id: 4,
                            flags: <SemanticsFlag>[
                              SemanticsFlag.hasEnabledState,
                              SemanticsFlag.isEnabled,
                              SemanticsFlag.isFocusable,
                              SemanticsFlag.isSlider,
                            ],
                            actions: <SemanticsAction>[
                              if (defaultTargetPlatform != TargetPlatform.iOS)
                                SemanticsAction.focus,
                              SemanticsAction.increase,
                              SemanticsAction.decrease,
                            ],
                            value: '50%',
                            increasedValue: '60%',
                            decreasedValue: '40%',
                            textDirection: TextDirection.ltr,
                          ),
                        ],
                      ),
                    ],
                  ),
                ],
              ),
            ],
          ),
          ignoreRect: true,
          ignoreTransform: true,
        ),
      );

      // Disable slider
      await tester.pumpWidget(
        const MaterialApp(
          home: Directionality(
            textDirection: TextDirection.ltr,
            child: Material(child: Slider(value: 0.5, onChanged: null)),
          ),
        ),
      );

      expect(
        semantics,
        hasSemantics(
          TestSemantics.root(
            children: <TestSemantics>[
              TestSemantics(
                id: 1,
                textDirection: TextDirection.ltr,
                children: <TestSemantics>[
                  TestSemantics(
                    id: 2,
                    children: <TestSemantics>[
                      TestSemantics(
                        id: 3,
                        flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                        children: <TestSemantics>[
                          TestSemantics(
                            id: 5,
                            flags: <SemanticsFlag>[
                              SemanticsFlag.hasEnabledState,
                              SemanticsFlag.isSlider,
                            ],
                            value: '50%',
                            increasedValue: '60%',
                            decreasedValue: '40%',
                            textDirection: TextDirection.ltr,
                          ),
                        ],
                      ),
                    ],
                  ),
                ],
              ),
            ],
          ),
          ignoreRect: true,
          ignoreTransform: true,
        ),
      );
      semantics.dispose();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets('Slider Semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(child: Slider(value: 0.5, onChanged: (double v) {})),
        ),
      ),
    );

    await tester.pumpAndSettle();

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              id: 1,
              textDirection: TextDirection.ltr,
              children: <TestSemantics>[
                TestSemantics(
                  id: 2,
                  children: <TestSemantics>[
                    TestSemantics(
                      id: 3,
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                      children: <TestSemantics>[
                        TestSemantics(
                          id: 4,
                          flags: <SemanticsFlag>[
                            SemanticsFlag.hasEnabledState,
                            SemanticsFlag.isEnabled,
                            SemanticsFlag.isFocusable,
                            SemanticsFlag.isSlider,
                          ],
                          actions: <SemanticsAction>[
                            SemanticsAction.focus,
                            SemanticsAction.increase,
                            SemanticsAction.decrease,
                            SemanticsAction.didGainAccessibilityFocus,
                          ],
                          value: '50%',
                          increasedValue: '55%',
                          decreasedValue: '45%',
                          textDirection: TextDirection.ltr,
                        ),
                      ],
                    ),
                  ],
                ),
              ],
            ),
          ],
        ),
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );

    // Disable slider
    await tester.pumpWidget(
      const MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(child: Slider(value: 0.5, onChanged: null)),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              id: 1,
              textDirection: TextDirection.ltr,
              children: <TestSemantics>[
                TestSemantics(
                  id: 2,
                  children: <TestSemantics>[
                    TestSemantics(
                      id: 3,
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                      children: <TestSemantics>[
                        TestSemantics(
                          id: 4,
                          flags: <SemanticsFlag>[
                            SemanticsFlag.hasEnabledState,
                            // isFocusable is delayed by 1 frame.
                            SemanticsFlag.isFocusable,
                            SemanticsFlag.isSlider,
                          ],
                          actions: <SemanticsAction>[
                            SemanticsAction.focus,
                            SemanticsAction.didGainAccessibilityFocus,
                          ],
                          value: '50%',
                          increasedValue: '55%',
                          decreasedValue: '45%',
                          textDirection: TextDirection.ltr,
                        ),
                      ],
                    ),
                  ],
                ),
              ],
            ),
          ],
        ),
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );

    await tester.pump();
    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              id: 1,
              textDirection: TextDirection.ltr,
              children: <TestSemantics>[
                TestSemantics(
                  id: 2,
                  children: <TestSemantics>[
                    TestSemantics(
                      id: 3,
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                      children: <TestSemantics>[
                        TestSemantics(
                          id: 4,
                          flags: <SemanticsFlag>[
                            SemanticsFlag.hasEnabledState,
                            SemanticsFlag.isSlider,
                          ],
                          actions: <SemanticsAction>[SemanticsAction.didGainAccessibilityFocus],
                          value: '50%',
                          increasedValue: '55%',
                          decreasedValue: '45%',
                          textDirection: TextDirection.ltr,
                        ),
                      ],
                    ),
                  ],
                ),
              ],
            ),
          ],
        ),
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );

    semantics.dispose();
  }, variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.windows}));

  testWidgets('Slider semantics with custom formatter', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Slider(
              value: 40.0,
              max: 200.0,
              divisions: 10,
              semanticFormatterCallback: (double value) => value.round().toString(),
              onChanged: (double v) {},
            ),
          ),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              id: 1,
              textDirection: TextDirection.ltr,
              children: <TestSemantics>[
                TestSemantics(
                  id: 2,
                  children: <TestSemantics>[
                    TestSemantics(
                      id: 3,
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                      children: <TestSemantics>[
                        TestSemantics(
                          id: 4,
                          flags: <SemanticsFlag>[
                            SemanticsFlag.hasEnabledState,
                            SemanticsFlag.isEnabled,
                            SemanticsFlag.isFocusable,
                            SemanticsFlag.isSlider,
                          ],
                          actions: <SemanticsAction>[
                            SemanticsAction.focus,
                            SemanticsAction.increase,
                            SemanticsAction.decrease,
                          ],
                          value: '40',
                          increasedValue: '60',
                          decreasedValue: '20',
                          textDirection: TextDirection.ltr,
                        ),
                      ],
                    ),
                  ],
                ),
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

  // Regression test for https://github.com/flutter/flutter/issues/101868
  testWidgets('Slider.label info should not write to semantic node', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Slider(
              value: 40.0,
              max: 200.0,
              divisions: 10,
              semanticFormatterCallback: (double value) => value.round().toString(),
              onChanged: (double v) {},
              label: 'Bingo',
            ),
          ),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              id: 1,
              textDirection: TextDirection.ltr,
              children: <TestSemantics>[
                TestSemantics(
                  id: 2,
                  children: <TestSemantics>[
                    TestSemantics(
                      id: 3,
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                      children: <TestSemantics>[
                        TestSemantics(
                          id: 4,
                          flags: <SemanticsFlag>[
                            SemanticsFlag.hasEnabledState,
                            SemanticsFlag.isEnabled,
                            SemanticsFlag.isFocusable,
                            SemanticsFlag.isSlider,
                          ],
                          actions: <SemanticsAction>[
                            SemanticsAction.focus,
                            SemanticsAction.increase,
                            SemanticsAction.decrease,
                          ],
                          value: '40',
                          increasedValue: '60',
                          decreasedValue: '20',
                          textDirection: TextDirection.ltr,
                        ),
                      ],
                    ),
                  ],
                ),
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

  testWidgets('Material3 - Slider is focusable and has correct focus color', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'Slider');
    addTearDown(focusNode.dispose);
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    final ThemeData theme = ThemeData();
    double value = 0.5;
    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Slider(
                  value: value,
                  onChanged:
                      enabled
                          ? (double newValue) {
                            setState(() {
                              value = newValue;
                            });
                          }
                          : null,
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

    // Check that the overlay shows when focused.
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isTrue);
    expect(
      Material.of(tester.element(find.byType(Slider))),
      paints..circle(color: theme.colorScheme.primary.withOpacity(0.1)),
    );

    // Check that the overlay does not show when unfocused and disabled.
    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isFalse);
    expect(
      Material.of(tester.element(find.byType(Slider))),
      isNot(paints..circle(color: theme.colorScheme.primary.withOpacity(0.1))),
    );
  });

  testWidgets('Slider has correct focus color from overlayColor property', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'Slider');
    addTearDown(focusNode.dispose);
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    double value = 0.5;
    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Slider(
                  value: value,
                  overlayColor: MaterialStateColor.resolveWith((Set<MaterialState> states) {
                    if (states.contains(MaterialState.focused)) {
                      return Colors.purple[500]!;
                    }

                    return Colors.transparent;
                  }),
                  onChanged:
                      enabled
                          ? (double newValue) {
                            setState(() {
                              value = newValue;
                            });
                          }
                          : null,
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

    // Check that the overlay shows when focused.
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isTrue);
    expect(
      Material.of(tester.element(find.byType(Slider))),
      paints..circle(color: Colors.purple[500]),
    );

    // Check that the overlay does not show when focused and disabled.
    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isFalse);
    expect(
      Material.of(tester.element(find.byType(Slider))),
      isNot(paints..circle(color: Colors.purple[500])),
    );
  });

  testWidgets('Slider can be hovered and has correct hover color', (WidgetTester tester) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    final ThemeData theme = ThemeData(useMaterial3: true);
    double value = 0.5;
    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Slider(
                  value: value,
                  onChanged:
                      enabled
                          ? (double newValue) {
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

    // Slider does not have overlay when enabled and not hovered.
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Slider))),
      isNot(paints..circle(color: Colors.orange[500])),
    );

    // Start hovering.
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byType(Slider)));

    // Slider has overlay when enabled and hovered.
    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Slider))),
      paints..circle(color: theme.colorScheme.primary.withOpacity(0.08)),
    );

    // Slider still shows correct hovered color after pressing/dragging
    await gesture.down(tester.getCenter(find.byType(Slider)));
    await tester.pump();
    await gesture.up();
    await tester.pumpAndSettle();
    await gesture.moveTo(const Offset(0.0, 100.0));
    await tester.pumpAndSettle();
    await gesture.moveTo(tester.getCenter(find.byType(Slider)));
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Slider))),
      paints..circle(color: theme.colorScheme.primary.withOpacity(0.08)),
    );

    // Slider does not have an overlay when disabled and hovered.
    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Slider))),
      isNot(paints..circle(color: Colors.orange[500])),
    );
  });

  testWidgets('Slider has correct hovered color from overlayColor property', (
    WidgetTester tester,
  ) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    double value = 0.5;
    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Slider(
                  value: value,
                  overlayColor: MaterialStateColor.resolveWith((Set<MaterialState> states) {
                    if (states.contains(MaterialState.hovered)) {
                      return Colors.cyan[500]!;
                    }

                    return Colors.transparent;
                  }),
                  onChanged:
                      enabled
                          ? (double newValue) {
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

    // Slider does not have overlay when enabled and not hovered.
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Slider))),
      isNot(paints..circle(color: Colors.cyan[500])),
    );

    // Start hovering.
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byType(Slider)));

    // Slider has overlay when enabled and hovered.
    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Slider))),
      paints..circle(color: Colors.cyan[500]),
    );

    // Slider does not have an overlay when disabled and hovered.
    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Slider))),
      isNot(paints..circle(color: Colors.cyan[500])),
    );
  });

  testWidgets('Material3 - Slider is draggable and has correct dragged color', (
    WidgetTester tester,
  ) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    double value = 0.5;
    final ThemeData theme = ThemeData();
    final Key sliderKey = UniqueKey();
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Slider(
                  key: sliderKey,
                  value: value,
                  focusNode: focusNode,
                  onChanged:
                      enabled
                          ? (double newValue) {
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

    // Slider does not have overlay when enabled and not dragged.
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Slider))),
      isNot(paints..circle(color: theme.colorScheme.primary.withOpacity(0.1))),
    );

    // Start dragging.
    final TestGesture drag = await tester.startGesture(tester.getCenter(find.byKey(sliderKey)));
    await tester.pump(kPressTimeout);

    // Less than configured touch slop, more than default touch slop
    await drag.moveBy(const Offset(19.0, 0));
    await tester.pump();

    // Slider has overlay when enabled and dragged.
    expect(
      Material.of(tester.element(find.byType(Slider))),
      paints..circle(color: theme.colorScheme.primary.withOpacity(0.1)),
    );

    await drag.up();
    await tester.pumpAndSettle();

    // Slider without focus doesn't have overlay when enabled and dragged.
    expect(focusNode.hasFocus, false);
    expect(
      Material.of(tester.element(find.byType(Slider))),
      isNot(paints..circle(color: theme.colorScheme.primary.withOpacity(0.1))),
    );

    // Slider has overlay when enabled, dragged and focused.
    focusNode.requestFocus();
    await tester.pumpAndSettle();

    expect(focusNode.hasFocus, true);
    expect(
      Material.of(tester.element(find.byType(Slider))),
      paints..circle(color: theme.colorScheme.primary.withOpacity(0.1)),
    );
  });

  testWidgets('Slider has correct dragged color from overlayColor property', (
    WidgetTester tester,
  ) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    double value = 0.5;
    final Key sliderKey = UniqueKey();
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Slider(
                  key: sliderKey,
                  value: value,
                  focusNode: focusNode,
                  overlayColor: MaterialStateColor.resolveWith((Set<MaterialState> states) {
                    if (states.contains(MaterialState.dragged)) {
                      return Colors.lime[500]!;
                    }

                    return Colors.transparent;
                  }),
                  onChanged:
                      enabled
                          ? (double newValue) {
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

    // Slider does not have overlay when enabled and not dragged.
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Slider))),
      isNot(paints..circle(color: Colors.lime[500])),
    );

    // Start dragging.
    final TestGesture drag = await tester.startGesture(tester.getCenter(find.byKey(sliderKey)));
    await tester.pump(kPressTimeout);

    // Less than configured touch slop, more than default touch slop
    await drag.moveBy(const Offset(19.0, 0));
    await tester.pump();

    // Slider has overlay when enabled and dragged.
    expect(
      Material.of(tester.element(find.byType(Slider))),
      paints..circle(color: Colors.lime[500]),
    );

    await drag.up();
    await tester.pumpAndSettle();

    // Slider without focus doesn't have overlay when enabled and dragged.
    expect(focusNode.hasFocus, false);
    expect(
      Material.of(tester.element(find.byType(Slider))),
      isNot(paints..circle(color: Colors.lime[500])),
    );
  });

  testWidgets('OverlayColor property is correctly applied when activeColor is also provided', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'Slider');
    addTearDown(focusNode.dispose);
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    double value = 0.5;
    const Color activeColor = Color(0xffff0000);
    const Color overlayColor = Color(0xff0000ff);

    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Slider(
                  value: value,
                  activeColor: activeColor,
                  overlayColor: const MaterialStatePropertyAll<Color?>(overlayColor),
                  onChanged:
                      enabled
                          ? (double newValue) {
                            setState(() {
                              value = newValue;
                            });
                          }
                          : null,
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

    final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));
    // Check that thumb color is using active color.
    expect(material, paints..circle(color: activeColor));

    focusNode.requestFocus();
    await tester.pumpAndSettle();

    // Check that the overlay shows when focused.
    expect(focusNode.hasPrimaryFocus, isTrue);
    expect(Material.of(tester.element(find.byType(Slider))), paints..circle(color: overlayColor));

    // Check that the overlay does not show when focused and disabled.
    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isFalse);
    expect(
      Material.of(tester.element(find.byType(Slider))),
      isNot(paints..circle(color: overlayColor)),
    );
  });

  testWidgets(
    'Slider can be incremented and decremented by keyboard shortcuts - LTR',
    (WidgetTester tester) async {
      tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
      double startValue = 0.0;
      double currentValue = 0.5;
      double endValue = 0.0;
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: StatefulBuilder(
                builder: (BuildContext context, StateSetter setState) {
                  return Slider(
                    value: currentValue,
                    onChangeStart: (double newValue) {
                      setState(() {
                        startValue = newValue;
                      });
                    },
                    onChanged: (double newValue) {
                      setState(() {
                        currentValue = newValue;
                      });
                    },
                    onChangeEnd: (double newValue) {
                      setState(() {
                        endValue = newValue;
                      });
                    },
                    autofocus: true,
                  );
                },
              ),
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pumpAndSettle();
      expect(startValue, 0.5);
      expect(currentValue, 0.55);
      expect(endValue, 0.55);

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      await tester.pumpAndSettle();
      expect(startValue, 0.55);
      expect(currentValue, 0.5);
      expect(endValue, 0.5);

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
      await tester.pumpAndSettle();
      expect(startValue, 0.5);
      expect(currentValue, 0.55);
      expect(endValue, 0.55);

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      await tester.pumpAndSettle();
      expect(startValue, 0.55);
      expect(currentValue, 0.5);
      expect(endValue, 0.5);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.fuchsia,
      TargetPlatform.linux,
      TargetPlatform.windows,
    }),
  );

  testWidgets(
    'Slider can be incremented and decremented by keyboard shortcuts - LTR',
    (WidgetTester tester) async {
      tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
      double startValue = 0.0;
      double currentValue = 0.5;
      double endValue = 0.0;
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: StatefulBuilder(
                builder: (BuildContext context, StateSetter setState) {
                  return Slider(
                    value: currentValue,
                    onChangeStart: (double newValue) {
                      setState(() {
                        startValue = newValue;
                      });
                    },
                    onChanged: (double newValue) {
                      setState(() {
                        currentValue = newValue;
                      });
                    },
                    onChangeEnd: (double newValue) {
                      setState(() {
                        endValue = newValue;
                      });
                    },
                    autofocus: true,
                  );
                },
              ),
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pumpAndSettle();
      expect(startValue, 0.5);
      expect(currentValue, 0.6);
      expect(endValue, 0.6);

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      await tester.pumpAndSettle();
      expect(startValue, 0.6);
      expect(currentValue, 0.5);
      expect(endValue, 0.5);

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
      await tester.pumpAndSettle();
      expect(startValue, 0.5);
      expect(currentValue, 0.6);
      expect(endValue, 0.6);

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      await tester.pumpAndSettle();
      expect(startValue, 0.6);
      expect(currentValue, 0.5);
      expect(endValue, 0.5);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'Slider can be incremented and decremented by keyboard shortcuts - RTL',
    (WidgetTester tester) async {
      tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
      double startValue = 0.0;
      double currentValue = 0.5;
      double endValue = 0.0;
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: StatefulBuilder(
                builder: (BuildContext context, StateSetter setState) {
                  return Directionality(
                    textDirection: TextDirection.rtl,
                    child: Slider(
                      value: currentValue,
                      onChangeStart: (double newValue) {
                        setState(() {
                          startValue = newValue;
                        });
                      },
                      onChanged: (double newValue) {
                        setState(() {
                          currentValue = newValue;
                        });
                      },
                      onChangeEnd: (double newValue) {
                        setState(() {
                          endValue = newValue;
                        });
                      },
                      autofocus: true,
                    ),
                  );
                },
              ),
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pumpAndSettle();
      expect(startValue, 0.5);
      expect(currentValue, 0.45);
      expect(endValue, 0.45);

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      await tester.pumpAndSettle();
      expect(startValue, 0.45);
      expect(currentValue, 0.5);
      expect(endValue, 0.5);

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
      await tester.pumpAndSettle();
      expect(startValue, 0.5);
      expect(currentValue, 0.55);
      expect(endValue, 0.55);

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      await tester.pumpAndSettle();
      expect(startValue, 0.55);
      expect(currentValue, 0.5);
      expect(endValue, 0.5);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.fuchsia,
      TargetPlatform.linux,
      TargetPlatform.windows,
    }),
  );

  testWidgets(
    'Slider can be incremented and decremented by keyboard shortcuts - RTL',
    (WidgetTester tester) async {
      tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
      double startValue = 0.0;
      double currentValue = 0.5;
      double endValue = 0.0;
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: StatefulBuilder(
                builder: (BuildContext context, StateSetter setState) {
                  return Directionality(
                    textDirection: TextDirection.rtl,
                    child: Slider(
                      value: currentValue,
                      onChangeStart: (double newValue) {
                        setState(() {
                          startValue = newValue;
                        });
                      },
                      onChanged: (double newValue) {
                        setState(() {
                          currentValue = newValue;
                        });
                      },
                      onChangeEnd: (double newValue) {
                        setState(() {
                          endValue = newValue;
                        });
                      },
                      autofocus: true,
                    ),
                  );
                },
              ),
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pumpAndSettle();
      expect(startValue, 0.5);
      expect(currentValue, 0.4);
      expect(endValue, 0.4);

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      await tester.pumpAndSettle();
      expect(startValue, 0.4);
      expect(currentValue, 0.5);
      expect(endValue, 0.5);

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
      await tester.pumpAndSettle();
      expect(startValue, 0.5);
      expect(currentValue, 0.6);
      expect(endValue, 0.6);

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      await tester.pumpAndSettle();
      expect(startValue, 0.6);
      expect(currentValue, 0.5);
      expect(endValue, 0.5);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets('In directional nav, Slider can be navigated out of by using up and down arrows', (
    WidgetTester tester,
  ) async {
    const Map<ShortcutActivator, Intent> shortcuts = <ShortcutActivator, Intent>{
      SingleActivator(LogicalKeyboardKey.arrowLeft): DirectionalFocusIntent(
        TraversalDirection.left,
      ),
      SingleActivator(LogicalKeyboardKey.arrowRight): DirectionalFocusIntent(
        TraversalDirection.right,
      ),
      SingleActivator(LogicalKeyboardKey.arrowDown): DirectionalFocusIntent(
        TraversalDirection.down,
      ),
      SingleActivator(LogicalKeyboardKey.arrowUp): DirectionalFocusIntent(TraversalDirection.up),
    };

    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    double topSliderValue = 0.5;
    double bottomSliderValue = 0.5;
    await tester.pumpWidget(
      MaterialApp(
        home: Shortcuts(
          shortcuts: shortcuts,
          child: Material(
            child: Center(
              child: StatefulBuilder(
                builder: (BuildContext context, StateSetter setState) {
                  return MediaQuery(
                    data: const MediaQueryData(navigationMode: NavigationMode.directional),
                    child: Column(
                      children: <Widget>[
                        Slider(
                          value: topSliderValue,
                          onChanged: (double newValue) {
                            setState(() {
                              topSliderValue = newValue;
                            });
                          },
                          autofocus: true,
                        ),
                        Slider(
                          value: bottomSliderValue,
                          onChanged: (double newValue) {
                            setState(() {
                              bottomSliderValue = newValue;
                            });
                          },
                        ),
                      ],
                    ),
                  );
                },
              ),
            ),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();

    // The top slider is auto-focused and can be adjusted with left and right arrow keys.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pumpAndSettle();
    expect(topSliderValue, 0.55, reason: 'focused top Slider increased after first arrowRight');
    expect(
      bottomSliderValue,
      0.5,
      reason: 'unfocused bottom Slider unaffected by first arrowRight',
    );

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    await tester.pumpAndSettle();
    expect(topSliderValue, 0.5, reason: 'focused top Slider decreased after first arrowLeft');
    expect(bottomSliderValue, 0.5, reason: 'unfocused bottom Slider unaffected by first arrowLeft');

    // Pressing the down-arrow key moves focus down to the bottom slider
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pumpAndSettle();
    expect(topSliderValue, 0.5, reason: 'arrowDown unfocuses top Slider, does not alter its value');
    expect(
      bottomSliderValue,
      0.5,
      reason: 'arrowDown focuses bottom Slider, does not alter its value',
    );

    // The bottom slider is now focused and can be adjusted with left and right arrow keys.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pumpAndSettle();
    expect(topSliderValue, 0.5, reason: 'unfocused top Slider unaffected by second arrowRight');
    expect(bottomSliderValue, 0.55, reason: 'focused bottom Slider increased by second arrowRight');

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    await tester.pumpAndSettle();
    expect(topSliderValue, 0.5, reason: 'unfocused top Slider unaffected by second arrowLeft');
    expect(bottomSliderValue, 0.5, reason: 'focused bottom Slider decreased by second arrowLeft');

    // Pressing the up-arrow key moves focus back up to the top slider
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.pumpAndSettle();
    expect(topSliderValue, 0.5, reason: 'arrowUp focuses top Slider, does not alter its value');
    expect(
      bottomSliderValue,
      0.5,
      reason: 'arrowUp unfocuses bottom Slider, does not alter its value',
    );

    // The top slider is now focused again and can be adjusted with left and right arrow keys.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pumpAndSettle();
    expect(topSliderValue, 0.55, reason: 'focused top Slider increased after third arrowRight');
    expect(
      bottomSliderValue,
      0.5,
      reason: 'unfocused bottom Slider unaffected by third arrowRight',
    );

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    await tester.pumpAndSettle();
    expect(topSliderValue, 0.5, reason: 'focused top Slider decreased after third arrowRight');
    expect(
      bottomSliderValue,
      0.5,
      reason: 'unfocused bottom Slider unaffected by third arrowRight',
    );
  });

  testWidgets(
    'Slider gains keyboard focus when it gains semantics focus on Windows',
    (WidgetTester tester) async {
      final SemanticsTester semantics = SemanticsTester(tester);
      final SemanticsOwner semanticsOwner = tester.binding.pipelineOwner.semanticsOwner!;
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);
      await tester.pumpWidget(
        MaterialApp(
          home: Material(child: Slider(value: 0.5, onChanged: (double _) {}, focusNode: focusNode)),
        ),
      );

      expect(
        semantics,
        hasSemantics(
          TestSemantics.root(
            children: <TestSemantics>[
              TestSemantics(
                id: 1,
                textDirection: TextDirection.ltr,
                children: <TestSemantics>[
                  TestSemantics(
                    id: 2,
                    children: <TestSemantics>[
                      TestSemantics(
                        id: 3,
                        flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                        children: <TestSemantics>[
                          TestSemantics(
                            id: 4,
                            flags: <SemanticsFlag>[
                              SemanticsFlag.hasEnabledState,
                              SemanticsFlag.isEnabled,
                              SemanticsFlag.isFocusable,
                              SemanticsFlag.isSlider,
                            ],
                            actions: <SemanticsAction>[
                              SemanticsAction.focus,
                              SemanticsAction.increase,
                              SemanticsAction.decrease,
                              SemanticsAction.didGainAccessibilityFocus,
                            ],
                            value: '50%',
                            increasedValue: '55%',
                            decreasedValue: '45%',
                            textDirection: TextDirection.ltr,
                          ),
                        ],
                      ),
                    ],
                  ),
                ],
              ),
            ],
          ),
          ignoreRect: true,
          ignoreTransform: true,
        ),
      );

      expect(focusNode.hasFocus, isFalse);
      semanticsOwner.performAction(4, SemanticsAction.didGainAccessibilityFocus);
      await tester.pumpAndSettle();
      expect(focusNode.hasFocus, isTrue);
      semantics.dispose();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.windows}),
  );

  testWidgets('Value indicator appears when it should', (WidgetTester tester) async {
    final ThemeData baseTheme = ThemeData(
      platform: TargetPlatform.android,
      primarySwatch: Colors.blue,
    );
    SliderThemeData theme = baseTheme.sliderTheme;
    double value = 0.45;
    Widget buildApp({required SliderThemeData sliderTheme, int? divisions, bool enabled = true}) {
      final ValueChanged<double>? onChanged = enabled ? (double d) => value = d : null;
      return MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: Theme(
                data: baseTheme,
                child: SliderTheme(
                  data: sliderTheme,
                  child: Slider(
                    value: value,
                    label: '$value',
                    divisions: divisions,
                    onChanged: onChanged,
                  ),
                ),
              ),
            ),
          ),
        ),
      );
    }

    Future<void> expectValueIndicator({
      required bool isVisible,
      required SliderThemeData theme,
      int? divisions,
      bool enabled = true,
    }) async {
      // Discrete enabled widget.
      await tester.pumpWidget(buildApp(sliderTheme: theme, divisions: divisions, enabled: enabled));
      final Offset center = tester.getCenter(find.byType(Slider));
      final TestGesture gesture = await tester.startGesture(center);
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();

      final RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));
      expect(
        valueIndicatorBox,
        isVisible
            ? (paints
              ..path(color: theme.valueIndicatorColor)
              ..paragraph())
            : isNot(
              paints
                ..path(color: theme.valueIndicatorColor)
                ..paragraph(),
            ),
      );
      await gesture.up();
    }

    // Default (showValueIndicator set to onlyForDiscrete).
    await expectValueIndicator(isVisible: true, theme: theme, divisions: 3);
    await expectValueIndicator(isVisible: false, theme: theme, divisions: 3, enabled: false);
    await expectValueIndicator(isVisible: false, theme: theme);
    await expectValueIndicator(isVisible: false, theme: theme, enabled: false);

    // With showValueIndicator set to onlyForContinuous.
    theme = theme.copyWith(showValueIndicator: ShowValueIndicator.onlyForContinuous);
    await expectValueIndicator(isVisible: false, theme: theme, divisions: 3);
    await expectValueIndicator(isVisible: false, theme: theme, divisions: 3, enabled: false);
    await expectValueIndicator(isVisible: true, theme: theme);
    await expectValueIndicator(isVisible: false, theme: theme, enabled: false);

    // discrete enabled widget with showValueIndicator set to always.
    theme = theme.copyWith(showValueIndicator: ShowValueIndicator.always);
    await expectValueIndicator(isVisible: true, theme: theme, divisions: 3);
    await expectValueIndicator(isVisible: false, theme: theme, divisions: 3, enabled: false);
    await expectValueIndicator(isVisible: true, theme: theme);
    await expectValueIndicator(isVisible: false, theme: theme, enabled: false);

    // discrete enabled widget with showValueIndicator set to never.
    theme = theme.copyWith(showValueIndicator: ShowValueIndicator.never);
    await expectValueIndicator(isVisible: false, theme: theme, divisions: 3);
    await expectValueIndicator(isVisible: false, theme: theme, divisions: 3, enabled: false);
    await expectValueIndicator(isVisible: false, theme: theme);
    await expectValueIndicator(isVisible: false, theme: theme, enabled: false);
  });

  testWidgets("Slider doesn't start any animations after dispose", (WidgetTester tester) async {
    final Key sliderKey = UniqueKey();
    double value = 0.0;
    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: Slider(
                    key: sliderKey,
                    value: value,
                    divisions: 4,
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

    final TestGesture gesture = await tester.startGesture(tester.getCenter(find.byKey(sliderKey)));
    await tester.pumpAndSettle();
    expect(value, equals(0.5));
    await gesture.moveBy(const Offset(-500.0, 0.0));
    await tester.pumpAndSettle();
    // Change the tree to dispose the original widget.
    await tester.pumpWidget(Container());
    expect(await tester.pumpAndSettle(), equals(1));
    await gesture.up();
  });

  testWidgets(
    'Slider removes value indicator from overlay if Slider gets disposed without value indicator animation completing.',
    (WidgetTester tester) async {
      final Key sliderKey = UniqueKey();
      const Color fillColor = Color(0xf55f5f5f);
      double value = 0.0;

      Widget buildApp({int? divisions, bool enabled = true}) {
        return MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Scaffold(
            body: Builder(
              // The builder is used to pass the context from the MaterialApp widget
              // to the [Navigator]. This context is required in order for the
              // Navigator to work.
              builder: (BuildContext context) {
                return Column(
                  children: <Widget>[
                    Slider(
                      key: sliderKey,
                      max: 100.0,
                      divisions: divisions,
                      label: '${value.round()}',
                      value: value,
                      onChanged: (double newValue) {
                        value = newValue;
                      },
                    ),
                    ElevatedButton(
                      child: const Text('Next'),
                      onPressed: () {
                        Navigator.of(context).pushReplacement(
                          MaterialPageRoute<void>(
                            builder: (BuildContext context) {
                              return ElevatedButton(
                                child: const Text('Inner page'),
                                onPressed: () {
                                  Navigator.of(context).pop();
                                },
                              );
                            },
                          ),
                        );
                      },
                    ),
                  ],
                );
              },
            ),
          ),
        );
      }

      await tester.pumpWidget(buildApp(divisions: 3));

      final RenderObject valueIndicatorBox = tester.renderObject(find.byType(Overlay));
      final Offset topRight = tester.getTopRight(find.byType(Slider)).translate(-24, 0);
      final TestGesture gesture = await tester.startGesture(topRight);
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();

      expect(find.byType(Slider), isNotNull);
      expect(
        valueIndicatorBox,
        paints
          // Represents the raised button with text, next.
          ..path(color: Colors.black)
          ..paragraph()
          // Represents the Slider.
          ..path(color: fillColor)
          ..paragraph(),
      );

      expect(valueIndicatorBox, paintsExactlyCountTimes(#drawPath, 4));
      expect(valueIndicatorBox, paintsExactlyCountTimes(#drawParagraph, 2));

      await tester.tap(find.text('Next'));
      await tester.pumpAndSettle();

      expect(find.byType(Slider), findsNothing);
      expect(
        valueIndicatorBox,
        isNot(
          paints
            ..path(color: fillColor)
            ..paragraph(),
        ),
      );

      // Represents the ElevatedButton with inner Text, inner page.
      expect(valueIndicatorBox, paintsExactlyCountTimes(#drawPath, 2));
      expect(valueIndicatorBox, paintsExactlyCountTimes(#drawParagraph, 1));

      // Don't stop holding the value indicator.
      await gesture.up();
      await tester.pumpAndSettle();
    },
  );

  testWidgets('Slider.adaptive', (WidgetTester tester) async {
    double value = 0.5;

    Widget buildFrame(TargetPlatform platform) {
      return MaterialApp(
        theme: ThemeData(platform: platform),
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Material(
              child: Center(
                child: Slider.adaptive(
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
      );
    }

    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    ]) {
      value = 0.5;
      await tester.pumpWidget(buildFrame(platform));
      expect(find.byType(Slider), findsOneWidget);
      expect(find.byType(CupertinoSlider), findsOneWidget);

      expect(value, 0.5, reason: 'on ${platform.name}');
      final TestGesture gesture = await tester.startGesture(
        tester.getCenter(find.byType(CupertinoSlider)),
      );
      // Drag to the right end of the track.
      await gesture.moveBy(const Offset(600.0, 0.0));
      expect(value, 1.0, reason: 'on ${platform.name}');
      await gesture.up();
    }

    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.android,
      TargetPlatform.fuchsia,
      TargetPlatform.linux,
      TargetPlatform.windows,
    ]) {
      value = 0.5;
      await tester.pumpWidget(buildFrame(platform));
      await tester.pumpAndSettle(); // Finish the theme change animation.
      expect(find.byType(Slider), findsOneWidget);
      expect(find.byType(CupertinoSlider), findsNothing);

      expect(value, 0.5, reason: 'on ${platform.name}');
      final TestGesture gesture = await tester.startGesture(tester.getCenter(find.byType(Slider)));
      // Drag to the right end of the track.
      await gesture.moveBy(const Offset(600.0, 0.0));
      expect(value, 1.0, reason: 'on ${platform.name}');
      await gesture.up();
    }
  });

  testWidgets('Slider respects height from theme', (WidgetTester tester) async {
    final Key sliderKey = UniqueKey();
    double value = 0.0;
    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              final SliderThemeData sliderTheme = SliderTheme.of(
                context,
              ).copyWith(tickMarkShape: TallSliderTickMarkShape());
              return Material(
                child: Center(
                  child: IntrinsicHeight(
                    child: SliderTheme(
                      data: sliderTheme,
                      child: Slider(
                        key: sliderKey,
                        value: value,
                        divisions: 4,
                        onChanged: (double newValue) {
                          setState(() {
                            value = newValue;
                          });
                        },
                      ),
                    ),
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );

    final RenderBox renderObject = tester.renderObject<RenderBox>(find.byType(Slider));
    expect(renderObject.size.height, 200);
  });

  testWidgets('Slider changes mouse cursor when hovered', (WidgetTester tester) async {
    // Test Slider() constructor
    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: MouseRegion(
                cursor: SystemMouseCursors.forbidden,
                child: Slider(
                  mouseCursor: SystemMouseCursors.text,
                  value: 0.5,
                  onChanged: (double newValue) {},
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
    await gesture.addPointer(location: tester.getCenter(find.byType(Slider)));

    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );

    // Test Slider.adaptive() constructor
    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: MouseRegion(
                cursor: SystemMouseCursors.forbidden,
                child: Slider.adaptive(
                  mouseCursor: SystemMouseCursors.text,
                  value: 0.5,
                  onChanged: (double newValue) {},
                ),
              ),
            ),
          ),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );

    // Test default cursor
    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: MouseRegion(
                cursor: SystemMouseCursors.forbidden,
                child: Slider(value: 0.5, onChanged: (double newValue) {}),
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
  });

  testWidgets('Slider MaterialStateMouseCursor resolves correctly', (WidgetTester tester) async {
    const MouseCursor disabledCursor = SystemMouseCursors.basic;
    const MouseCursor hoveredCursor = SystemMouseCursors.grab;
    const MouseCursor draggedCursor = SystemMouseCursors.move;

    Widget buildFrame({required bool enabled}) {
      return MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: MouseRegion(
                cursor: SystemMouseCursors.forbidden,
                child: Slider(
                  mouseCursor: const _StateDependentMouseCursor(
                    disabled: disabledCursor,
                    hovered: hoveredCursor,
                    dragged: draggedCursor,
                  ),
                  value: 0.5,
                  onChanged: enabled ? (double newValue) {} : null,
                ),
              ),
            ),
          ),
        ),
      );
    }

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: Offset.zero);

    await tester.pumpWidget(buildFrame(enabled: false));
    expect(RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1), disabledCursor);

    await tester.pumpWidget(buildFrame(enabled: true));
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.none,
    );

    await gesture.moveTo(tester.getCenter(find.byType(Slider))); // start hover
    await tester.pumpAndSettle();
    expect(RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1), hoveredCursor);

    await tester.timedDrag(
      find.byType(Slider),
      const Offset(20.0, 0.0),
      const Duration(milliseconds: 100),
    );
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.move,
    );
  });

  testWidgets('Slider implements debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();

    const Slider(
      activeColor: Colors.blue,
      divisions: 10,
      inactiveColor: Colors.grey,
      secondaryActiveColor: Colors.blueGrey,
      label: 'Set a value',
      max: 100.0,
      onChanged: null,
      value: 50.0,
      secondaryTrackValue: 75.0,
    ).debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[
      'value: 50.0',
      'secondaryTrackValue: 75.0',
      'disabled',
      'min: 0.0',
      'max: 100.0',
      'divisions: 10',
      'label: "Set a value"',
      'activeColor: MaterialColor(primary value: ${const Color(0xff2196f3)})',
      'inactiveColor: MaterialColor(primary value: ${const Color(0xff9e9e9e)})',
      'secondaryActiveColor: MaterialColor(primary value: ${const Color(0xff607d8b)})',
    ]);
  });

  testWidgets('Slider track paints correctly when the shape is rectangular', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          sliderTheme: const SliderThemeData(trackShape: RectangularSliderTrackShape()),
        ),
        home: const Directionality(
          textDirection: TextDirection.ltr,
          child: Material(child: Center(child: Slider(value: 0.5, onChanged: null))),
        ),
      ),
    );

    // _RenderSlider is the last render object in the tree.
    final RenderObject renderObject = tester.allRenderObjects.last;

    // The active track rect should start at 24.0 pixels,
    // and there should not have a gap between active and inactive track.
    expect(
      renderObject,
      paints
        ..rect(rect: const Rect.fromLTRB(24.0, 298.0, 400.0, 302.0)) // active track Rect.
        ..rect(rect: const Rect.fromLTRB(400.0, 298.0, 776.0, 302.0)), // inactive track Rect.
    );
  });

  testWidgets('SliderTheme change should trigger re-layout', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/118955
    double sliderValue = 0.0;
    Widget buildFrame(ThemeMode themeMode) {
      return MaterialApp(
        themeMode: themeMode,
        theme: ThemeData(brightness: Brightness.light, useMaterial3: true),
        darkTheme: ThemeData(brightness: Brightness.dark, useMaterial3: true),
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: SizedBox(
                height: 10.0,
                width: 10.0,
                child: Slider(
                  value: sliderValue,
                  label: 'label',
                  onChanged: (double value) => sliderValue = value,
                ),
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(ThemeMode.light));

    // _RenderSlider is the last render object in the tree.
    final RenderObject renderObject = tester.allRenderObjects.last;
    expect(renderObject.debugNeedsLayout, false);

    await tester.pumpWidget(buildFrame(ThemeMode.dark));
    await tester.pump(
      const Duration(milliseconds: 100), // to let the theme animate
      EnginePhase.build,
    );

    expect(renderObject.debugNeedsLayout, true);

    // Pump the rest of the frames to complete the test.
    await tester.pumpAndSettle();
  });

  testWidgets('Slider can be painted in a narrower constraint', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: SizedBox(
                height: 10.0,
                width: 10.0,
                child: Slider(value: 0.5, onChanged: null),
              ),
            ),
          ),
        ),
      ),
    );

    // _RenderSlider is the last render object in the tree.
    final RenderObject renderObject = tester.allRenderObjects.last;

    expect(
      renderObject,
      paints
        // Inactive track RRect.
        ..rrect(rrect: RRect.fromLTRBR(3.0, 3.0, 24.0, 7.0, const Radius.circular(2.0)))
        // Active track RRect.
        ..rrect(rrect: RRect.fromLTRBR(-14.0, 2.0, 7.0, 8.0, const Radius.circular(3.0)))
        // Thumb.
        ..circle(x: 5.0, y: 5.0, radius: 10.0),
    );
  });

  testWidgets('Update the divisions and value at the same time for Slider', (
    WidgetTester tester,
  ) async {
    // Regress test for https://github.com/flutter/flutter/issues/65943
    Widget buildFrame(double maxValue) {
      return MaterialApp(
        home: Material(
          child: Center(
            child: Slider.adaptive(
              value: 5,
              max: maxValue,
              divisions: maxValue.toInt(),
              onChanged: (double newValue) {},
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(10));

    // _RenderSlider is the last render object in the tree.
    final RenderObject renderObject = tester.allRenderObjects.last;

    // Update the divisions from 10 to 15, the thumb should be paint at the correct position.
    await tester.pumpWidget(buildFrame(15));
    await tester.pumpAndSettle(); // Finish the animation.

    late RRect activeTrackRRect;
    expect(
      renderObject,
      paints
        ..rrect()
        ..something((Symbol method, List<dynamic> arguments) {
          if (method != #drawRRect) {
            return false;
          }
          activeTrackRRect = arguments[0] as RRect;
          return true;
        }),
    );

    const double padding = 4.0;
    // The thumb should at one-third(5 / 15) of the Slider.
    // The right of the active track shape is the position of the thumb.
    // 24.0 is the default margin, (800.0 - 24.0 - 24.0) is the slider's width.
    expect(
      nearEqual(
        activeTrackRRect.right,
        (800.0 - 24.0 - 24.0 + (padding / 2)) * (5 / 15) + 24.0 + padding / 2,
        0.01,
      ),
      true,
    );
  });

  testWidgets('Slider paints thumbColor', (WidgetTester tester) async {
    const Color color = Color(0xffffc107);

    final Widget sliderAdaptive = MaterialApp(
      theme: ThemeData(platform: TargetPlatform.iOS),
      home: Material(child: Slider(value: 0, onChanged: (double newValue) {}, thumbColor: color)),
    );

    await tester.pumpWidget(sliderAdaptive);
    await tester.pumpAndSettle();

    final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));
    expect(material, paints..circle(color: color));
  });

  testWidgets('Slider.adaptive paints thumbColor on Android', (WidgetTester tester) async {
    const Color color = Color(0xffffc107);

    final Widget sliderAdaptive = MaterialApp(
      theme: ThemeData(platform: TargetPlatform.android),
      home: Material(
        child: Slider.adaptive(value: 0, onChanged: (double newValue) {}, thumbColor: color),
      ),
    );

    await tester.pumpWidget(sliderAdaptive);
    await tester.pumpAndSettle();

    final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));
    expect(material, paints..circle(color: color));
  });

  testWidgets('If thumbColor is null, it defaults to CupertinoColors.white', (
    WidgetTester tester,
  ) async {
    final Widget sliderAdaptive = MaterialApp(
      theme: ThemeData(platform: TargetPlatform.iOS),
      home: Material(child: Slider.adaptive(value: 0, onChanged: (double newValue) {})),
    );

    await tester.pumpWidget(sliderAdaptive);
    await tester.pumpAndSettle();

    final MaterialInkController material = Material.of(
      tester.element(find.byType(CupertinoSlider)),
    );
    expect(
      material,
      paints
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect(color: CupertinoColors.white),
    );
  });

  testWidgets('Slider.adaptive passes thumbColor to CupertinoSlider', (WidgetTester tester) async {
    const Color color = Color(0xffffc107);

    final Widget sliderAdaptive = MaterialApp(
      theme: ThemeData(platform: TargetPlatform.iOS),
      home: Material(
        child: Slider.adaptive(value: 0, onChanged: (double newValue) {}, thumbColor: color),
      ),
    );

    await tester.pumpWidget(sliderAdaptive);
    await tester.pumpAndSettle();

    final MaterialInkController material = Material.of(
      tester.element(find.byType(CupertinoSlider)),
    );
    expect(
      material,
      paints
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect(color: color),
    );
  });

  // Regression test for https://github.com/flutter/flutter/issues/103566
  testWidgets('Drag gesture uses provided gesture settings', (WidgetTester tester) async {
    double value = 0.5;
    bool dragStarted = false;
    final Key sliderKey = UniqueKey();

    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: GestureDetector(
                    behavior: HitTestBehavior.deferToChild,
                    onHorizontalDragStart: (DragStartDetails details) {
                      dragStarted = true;
                    },
                    child: MediaQuery(
                      data: MediaQuery.of(
                        context,
                      ).copyWith(gestureSettings: const DeviceGestureSettings(touchSlop: 20)),
                      child: Slider(
                        value: value,
                        key: sliderKey,
                        onChanged: (double newValue) {
                          setState(() {
                            value = newValue;
                          });
                        },
                      ),
                    ),
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );

    TestGesture drag = await tester.startGesture(tester.getCenter(find.byKey(sliderKey)));
    await tester.pump(kPressTimeout);

    // Less than configured touch slop, more than default touch slop
    await drag.moveBy(const Offset(19.0, 0));
    await tester.pump();

    expect(value, 0.5);
    expect(dragStarted, true);

    dragStarted = false;

    await drag.up();
    await tester.pumpAndSettle();

    drag = await tester.startGesture(tester.getCenter(find.byKey(sliderKey)));
    await tester.pump(kPressTimeout);

    bool sliderEnd = false;

    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: GestureDetector(
                    behavior: HitTestBehavior.deferToChild,
                    onHorizontalDragStart: (DragStartDetails details) {
                      dragStarted = true;
                    },
                    child: MediaQuery(
                      data: MediaQuery.of(
                        context,
                      ).copyWith(gestureSettings: const DeviceGestureSettings(touchSlop: 10)),
                      child: Slider(
                        value: value,
                        key: sliderKey,
                        onChanged: (double newValue) {
                          setState(() {
                            value = newValue;
                          });
                        },
                        onChangeEnd: (double endValue) {
                          sliderEnd = true;
                        },
                      ),
                    ),
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );

    // More than touch slop.
    await drag.moveBy(const Offset(12.0, 0));

    await drag.up();
    await tester.pumpAndSettle();

    expect(sliderEnd, true);
    expect(dragStarted, false);
  });

  // Regression test for https://github.com/flutter/flutter/issues/139281
  testWidgets('Slider does not request focus when the value is changed', (
    WidgetTester tester,
  ) async {
    double value = 0.5;
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Slider(
                  value: value,
                  onChanged: (double newValue) {
                    setState(() {
                      value = newValue;
                    });
                  },
                );
              },
            ),
          ),
        ),
      ),
    );
    // Initially, the slider does not have focus whe enabled and not tapped.
    await tester.pumpAndSettle();
    expect(value, equals(0.5));
    // Get FocusNode from the state of the slider to include auto-generated FocusNode.
    // ignore: invalid_assignment
    final FocusNode focusNode = (tester.firstState(find.byType(Slider)) as dynamic).focusNode;
    // The slider does not have focus.
    expect(focusNode.hasFocus, false);
    final Offset sliderCenter = tester.getCenter(find.byType(Slider));
    final Offset tapLocation = Offset(sliderCenter.dx + 50, sliderCenter.dy);
    // Tap on the slider to change the value.
    final TestGesture gesture = await tester.createGesture();
    await gesture.addPointer();
    await gesture.down(tapLocation);
    await gesture.up();
    await tester.pumpAndSettle();
    expect(value, isNot(equals(0.5)));
    // The slider does not have focus after the value is changed.
    expect(focusNode.hasFocus, false);
  });

  // Regression test for https://github.com/flutter/flutter/issues/139281
  testWidgets('Overlay remains when Slider thumb is interacted', (WidgetTester tester) async {
    double value = 0.5;
    const Color overlayColor = Color(0xffff0000);
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Slider(
                  value: value,
                  overlayColor: const MaterialStatePropertyAll<Color?>(overlayColor),
                  onChanged: (double newValue) {
                    setState(() {
                      value = newValue;
                    });
                  },
                );
              },
            ),
          ),
        ),
      ),
    );
    // Slider does not have overlay when enabled and not tapped.
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Slider))),
      isNot(paints..circle(color: overlayColor)),
    );
    final Offset sliderCenter = tester.getCenter(find.byType(Slider));
    // Tap and hold down on the thumb to keep it active.
    final TestGesture gesture = await tester.createGesture();
    await gesture.addPointer();
    await gesture.down(sliderCenter);
    await tester.pumpAndSettle();
    expect(Material.of(tester.element(find.byType(Slider))), paints..circle(color: overlayColor));
    // Hover on the slider but outside the thumb.
    await gesture.moveTo(tester.getTopLeft(find.byType(Slider)));
    await tester.pumpAndSettle();
    expect(Material.of(tester.element(find.byType(Slider))), paints..circle(color: overlayColor));
    // Tap up on the slider.
    await gesture.up();
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Slider))),
      isNot(paints..circle(color: overlayColor)),
    );
  });

  testWidgets(
    'Overlay appear only when hovered on the thumb on desktop',
    (WidgetTester tester) async {
      double value = 0.5;
      const Color overlayColor = Color(0xffff0000);

      Widget buildApp({bool enabled = true}) {
        return MaterialApp(
          home: Material(
            child: Center(
              child: StatefulBuilder(
                builder: (BuildContext context, StateSetter setState) {
                  return Slider(
                    value: value,
                    overlayColor: const MaterialStatePropertyAll<Color?>(overlayColor),
                    onChanged:
                        enabled
                            ? (double newValue) {
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

      // Slider does not have overlay when enabled and not hovered.
      await tester.pumpAndSettle();
      expect(
        Material.of(tester.element(find.byType(Slider))),
        isNot(paints..circle(color: overlayColor)),
      );

      // Hover on the slider but outside the thumb.
      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture.addPointer();
      await gesture.moveTo(tester.getTopLeft(find.byType(Slider)));

      await tester.pumpWidget(buildApp());
      await tester.pumpAndSettle();
      expect(
        Material.of(tester.element(find.byType(Slider))),
        isNot(paints..circle(color: overlayColor)),
      );

      // Hover on the thumb.
      await gesture.moveTo(tester.getCenter(find.byType(Slider)));
      await tester.pumpAndSettle();
      expect(Material.of(tester.element(find.byType(Slider))), paints..circle(color: overlayColor));

      // Hover on the slider but outside the thumb.
      await gesture.moveTo(tester.getBottomRight(find.byType(Slider)));
      await tester.pumpAndSettle();
      expect(
        Material.of(tester.element(find.byType(Slider))),
        isNot(paints..circle(color: overlayColor)),
      );
    },
    variant: TargetPlatformVariant.desktop(),
  );

  testWidgets('Overlay remains when Slider is in focus on desktop', (WidgetTester tester) async {
    double value = 0.5;
    const Color overlayColor = Color(0xffff0000);
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Slider(
                  value: value,
                  focusNode: focusNode,
                  overlayColor: const MaterialStatePropertyAll<Color?>(overlayColor),
                  onChanged:
                      enabled
                          ? (double newValue) {
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

    // Slider does not have overlay when enabled and not tapped.
    await tester.pumpAndSettle();
    expect(focusNode.hasFocus, false);
    expect(
      Material.of(tester.element(find.byType(Slider))),
      isNot(paints..circle(color: overlayColor)),
    );

    final Offset sliderCenter = tester.getCenter(find.byType(Slider));
    Offset tapLocation = Offset(sliderCenter.dx + 50, sliderCenter.dy);

    // Tap somewhere to bring overlay.
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.down(tapLocation);
    await gesture.up();
    focusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(focusNode.hasFocus, true);
    expect(Material.of(tester.element(find.byType(Slider))), paints..circle(color: overlayColor));

    tapLocation = Offset(sliderCenter.dx - 50, sliderCenter.dy);
    await gesture.down(tapLocation);
    await gesture.up();
    await tester.pumpAndSettle();
    expect(focusNode.hasFocus, true);
    // Overlay is removed when adjusted with a tap.
    expect(
      Material.of(tester.element(find.byType(Slider))),
      isNot(paints..circle(color: overlayColor)),
    );
  }, variant: TargetPlatformVariant.desktop());

  // Regression test for https://github.com/flutter/flutter/issues/123313, which only occurs on desktop platforms.
  testWidgets(
    'Value indicator disappears after adjusting the slider on desktop',
    (WidgetTester tester) async {
      final ThemeData theme = ThemeData(useMaterial3: true);
      const double currentValue = 0.5;
      await tester.pumpWidget(
        MaterialApp(
          theme: theme,
          home: Material(
            child: Center(
              child: Slider(
                value: currentValue,
                divisions: 5,
                label: currentValue.toStringAsFixed(1),
                onChanged: (_) {},
              ),
            ),
          ),
        ),
      );

      // Slider does not show value indicator initially.
      await tester.pumpAndSettle();
      RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));
      expect(
        valueIndicatorBox,
        isNot(
          paints
            ..scale()
            ..path(color: theme.colorScheme.primary),
        ),
      );

      final Offset sliderCenter = tester.getCenter(find.byType(Slider));
      final Offset tapLocation = Offset(sliderCenter.dx + 50, sliderCenter.dy);

      // Tap the slider by mouse to bring up the value indicator.
      await tester.tapAt(tapLocation, kind: PointerDeviceKind.mouse);
      await tester.pumpAndSettle();

      // Value indicator is visible.
      valueIndicatorBox = tester.renderObject(find.byType(Overlay));
      expect(
        valueIndicatorBox,
        paints
          ..scale()
          ..path(color: theme.colorScheme.primary),
      );

      // Wait for the value indicator to disappear.
      await tester.pumpAndSettle(const Duration(seconds: 2));

      // Value indicator is no longer visible.
      expect(
        valueIndicatorBox,
        isNot(
          paints
            ..scale()
            ..path(color: theme.colorScheme.primary),
        ),
      );
    },
    variant: TargetPlatformVariant.desktop(),
  );

  testWidgets(
    'Value indicator remains when Slider is in focus on desktop',
    (WidgetTester tester) async {
      double value = 0.5;
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);

      Widget buildApp({bool enabled = true}) {
        return MaterialApp(
          theme: ThemeData(
            sliderTheme: const SliderThemeData(showValueIndicator: ShowValueIndicator.always),
          ),
          home: Material(
            child: Center(
              child: StatefulBuilder(
                builder: (BuildContext context, StateSetter setState) {
                  return Slider(
                    value: value,
                    focusNode: focusNode,
                    divisions: 5,
                    label: value.toStringAsFixed(1),
                    onChanged:
                        enabled
                            ? (double newValue) {
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

      // Slider does not show value indicator without focus.
      await tester.pumpAndSettle();
      expect(focusNode.hasFocus, false);
      RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));
      expect(
        valueIndicatorBox,
        isNot(
          paints
            ..path(color: const Color(0xff000000))
            ..paragraph(),
        ),
      );

      final Offset sliderCenter = tester.getCenter(find.byType(Slider));
      final Offset tapLocation = Offset(sliderCenter.dx + 50, sliderCenter.dy);

      // Tap somewhere to bring value indicator.
      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture.addPointer();
      await gesture.down(tapLocation);
      await gesture.up();
      focusNode.requestFocus();
      await tester.pumpAndSettle();
      expect(focusNode.hasFocus, true);
      valueIndicatorBox = tester.renderObject(find.byType(Overlay));
      expect(
        valueIndicatorBox,
        paints
          ..path(color: const Color(0xff000000))
          ..paragraph(),
      );

      focusNode.unfocus();
      await tester.pumpAndSettle();
      expect(focusNode.hasFocus, false);
      expect(
        valueIndicatorBox,
        isNot(
          paints
            ..path(color: const Color(0xff000000))
            ..paragraph(),
        ),
      );
    },
    variant: TargetPlatformVariant.desktop(),
  );

  testWidgets('Event on Slider should perform no-op if already unmounted', (
    WidgetTester tester,
  ) async {
    // Test covering crashing found in Google internal issue b/192329942.
    double value = 0.0;
    final ValueNotifier<bool> shouldShowSliderListenable = ValueNotifier<bool>(true);
    addTearDown(shouldShowSliderListenable.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: ValueListenableBuilder<bool>(
                    valueListenable: shouldShowSliderListenable,
                    builder: (BuildContext context, bool shouldShowSlider, _) {
                      return GestureDetector(
                        behavior: HitTestBehavior.translucent,
                        // Note: it is important that `onTap` is non-null so
                        // [GestureDetector] will register tap events.
                        onTap: () {},
                        child:
                            shouldShowSlider
                                ? Slider(
                                  value: value,
                                  onChanged: (double newValue) {
                                    setState(() {
                                      value = newValue;
                                    });
                                  },
                                )
                                : const SizedBox.expand(),
                      );
                    },
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );

    // Move Slider.
    final TestGesture gesture = await tester.startGesture(
      tester.getRect(find.byType(Slider)).center,
    );
    await gesture.moveBy(const Offset(1.0, 0.0));
    await tester.pumpAndSettle();

    // Hide Slider. Slider will dispose and unmount.
    shouldShowSliderListenable.value = false;
    await tester.pumpAndSettle();

    // Move Slider after unmounted.
    await gesture.moveBy(const Offset(1.0, 0.0));
    await tester.pumpAndSettle();

    expect(tester.takeException(), null);
  });

  group('Material 2', () {
    // These tests are only relevant for Material 2. Once Material 2
    // support is deprecated and the APIs are removed, these tests
    // can be deleted.

    testWidgets('Slider can be hovered and has correct hover color', (WidgetTester tester) async {
      tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
      final ThemeData theme = ThemeData(useMaterial3: false);
      double value = 0.5;
      Widget buildApp({bool enabled = true}) {
        return MaterialApp(
          theme: theme,
          home: Material(
            child: Center(
              child: StatefulBuilder(
                builder: (BuildContext context, StateSetter setState) {
                  return Slider(
                    value: value,
                    onChanged:
                        enabled
                            ? (double newValue) {
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

      // Slider does not have overlay when enabled and not hovered.
      await tester.pumpAndSettle();
      expect(
        Material.of(tester.element(find.byType(Slider))),
        isNot(paints..circle(color: theme.colorScheme.primary.withOpacity(0.12))),
      );

      // Start hovering.
      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture.addPointer();
      await gesture.moveTo(tester.getCenter(find.byType(Slider)));

      // Slider has overlay when enabled and hovered.
      await tester.pumpWidget(buildApp());
      await tester.pumpAndSettle();
      expect(
        Material.of(tester.element(find.byType(Slider))),
        paints..circle(color: theme.colorScheme.primary.withOpacity(0.12)),
      );

      // Slider does not have an overlay when disabled and hovered.
      await tester.pumpWidget(buildApp(enabled: false));
      await tester.pumpAndSettle();
      expect(
        Material.of(tester.element(find.byType(Slider))),
        isNot(paints..circle(color: theme.colorScheme.primary.withOpacity(0.12))),
      );
    });

    testWidgets('Material2 - Slider is focusable and has correct focus color', (
      WidgetTester tester,
    ) async {
      final FocusNode focusNode = FocusNode(debugLabel: 'Slider');
      addTearDown(focusNode.dispose);
      tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
      final ThemeData theme = ThemeData(useMaterial3: false);
      double value = 0.5;
      Widget buildApp({bool enabled = true}) {
        return MaterialApp(
          theme: theme,
          home: Material(
            child: Center(
              child: StatefulBuilder(
                builder: (BuildContext context, StateSetter setState) {
                  return Slider(
                    value: value,
                    onChanged:
                        enabled
                            ? (double newValue) {
                              setState(() {
                                value = newValue;
                              });
                            }
                            : null,
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

      // Check that the overlay shows when focused.
      await tester.pumpAndSettle();
      expect(focusNode.hasPrimaryFocus, isTrue);
      expect(
        Material.of(tester.element(find.byType(Slider))),
        paints..circle(color: theme.colorScheme.primary.withOpacity(0.12)),
      );

      // Check that the overlay does not show when unfocused and disabled.
      await tester.pumpWidget(buildApp(enabled: false));
      await tester.pumpAndSettle();
      expect(focusNode.hasPrimaryFocus, isFalse);
      expect(
        Material.of(tester.element(find.byType(Slider))),
        isNot(paints..circle(color: theme.colorScheme.primary.withOpacity(0.12))),
      );
    });

    testWidgets('Material2 - Slider is draggable and has correct dragged color', (
      WidgetTester tester,
    ) async {
      tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
      double value = 0.5;
      final ThemeData theme = ThemeData(useMaterial3: false);
      final Key sliderKey = UniqueKey();
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);

      Widget buildApp({bool enabled = true}) {
        return MaterialApp(
          theme: theme,
          home: Material(
            child: Center(
              child: StatefulBuilder(
                builder: (BuildContext context, StateSetter setState) {
                  return Slider(
                    key: sliderKey,
                    value: value,
                    focusNode: focusNode,
                    onChanged:
                        enabled
                            ? (double newValue) {
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

      // Slider does not have overlay when enabled and not dragged.
      await tester.pumpAndSettle();
      expect(
        Material.of(tester.element(find.byType(Slider))),
        isNot(paints..circle(color: theme.colorScheme.primary.withOpacity(0.12))),
      );

      // Start dragging.
      final TestGesture drag = await tester.startGesture(tester.getCenter(find.byKey(sliderKey)));
      await tester.pump(kPressTimeout);

      // Less than configured touch slop, more than default touch slop
      await drag.moveBy(const Offset(19.0, 0));
      await tester.pump();

      // Slider has overlay when enabled and dragged.
      expect(
        Material.of(tester.element(find.byType(Slider))),
        paints..circle(color: theme.colorScheme.primary.withOpacity(0.12)),
      );

      await drag.up();
      await tester.pumpAndSettle();

      // Slider without focus doesn't have overlay when enabled and dragged.
      expect(focusNode.hasFocus, false);
      expect(
        Material.of(tester.element(find.byType(Slider))),
        isNot(paints..circle(color: theme.colorScheme.primary.withOpacity(0.12))),
      );
    });
  });

  group('Slider.allowedInteraction', () {
    testWidgets('SliderInteraction.tapOnly', (WidgetTester tester) async {
      double value = 1.0;
      final Key sliderKey = UniqueKey();
      // (slider's left padding (overlayRadius), windowHeight / 2)
      const Offset startOfTheSliderTrack = Offset(24, 300);
      const Offset centerOfTheSlideTrack = Offset(400, 300);
      final List<String> logs = <String>[];

      Widget buildWidget() => MaterialApp(
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext _, StateSetter setState) {
                return Slider(
                  value: value,
                  key: sliderKey,
                  allowedInteraction: SliderInteraction.tapOnly,
                  onChangeStart: (double newValue) {
                    logs.add('onChangeStart');
                  },
                  onChanged: (double newValue) {
                    logs.add('onChanged');
                    setState(() {
                      value = newValue;
                    });
                  },
                  onChangeEnd: (double newValue) {
                    logs.add('onChangeEnd');
                  },
                );
              },
            ),
          ),
        ),
      );

      // allow tap only
      await tester.pumpWidget(buildWidget());

      expect(logs, isEmpty);

      // test tap
      final TestGesture gesture = await tester.startGesture(centerOfTheSlideTrack);
      await tester.pump();
      // changes from 1.0 -> 0.5
      expect(value, 0.5);
      expect(logs, <String>['onChangeStart', 'onChanged']);

      // test slide
      await gesture.moveTo(startOfTheSliderTrack);
      await tester.pump();
      // has no effect, remains 0.5
      expect(value, 0.5);
      expect(logs, <String>['onChangeStart', 'onChanged']);

      await gesture.up();
      await tester.pump();
      expect(logs, <String>['onChangeStart', 'onChanged', 'onChangeEnd']);
    });

    testWidgets('SliderInteraction.tapAndSlide (default)', (WidgetTester tester) async {
      double value = 1.0;
      final Key sliderKey = UniqueKey();
      // (slider's left padding (overlayRadius), windowHeight / 2)
      const Offset startOfTheSliderTrack = Offset(24, 300);
      const Offset centerOfTheSlideTrack = Offset(400, 300);
      const Offset endOfTheSliderTrack = Offset(800 - 24, 300);
      final List<String> logs = <String>[];

      Widget buildWidget() => MaterialApp(
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext _, StateSetter setState) {
                return Slider(
                  value: value,
                  key: sliderKey,
                  onChangeStart: (double newValue) {
                    logs.add('onChangeStart');
                  },
                  onChanged: (double newValue) {
                    logs.add('onChanged');
                    setState(() {
                      value = newValue;
                    });
                  },
                  onChangeEnd: (double newValue) {
                    logs.add('onChangeEnd');
                  },
                );
              },
            ),
          ),
        ),
      );

      await tester.pumpWidget(buildWidget());

      expect(logs, isEmpty);

      // Test tap.
      final TestGesture gesture = await tester.startGesture(centerOfTheSlideTrack);
      await tester.pump();
      // changes from 1.0 -> 0.5
      expect(value, 0.5);
      expect(logs, <String>['onChangeStart', 'onChanged']);

      // test slide
      await gesture.moveTo(startOfTheSliderTrack);
      await tester.pump();
      // changes from 0.5 -> 0.0
      expect(value, 0.0);
      await gesture.moveTo(endOfTheSliderTrack);
      await tester.pump();
      // changes from 0.0 -> 1.0
      expect(value, 1.0);
      expect(logs, <String>['onChangeStart', 'onChanged', 'onChanged', 'onChanged']);

      await gesture.up();
      await tester.pump();

      expect(logs, <String>['onChangeStart', 'onChanged', 'onChanged', 'onChanged', 'onChangeEnd']);
    });

    testWidgets('SliderInteraction.slideOnly', (WidgetTester tester) async {
      double value = 1.0;
      final Key sliderKey = UniqueKey();
      // (slider's left padding (overlayRadius), windowHeight / 2)
      const Offset startOfTheSliderTrack = Offset(24, 300);
      const Offset centerOfTheSlideTrack = Offset(400, 300);
      const Offset endOfTheSliderTrack = Offset(800 - 24, 300);
      final List<String> logs = <String>[];

      Widget buildApp() {
        return MaterialApp(
          home: Material(
            child: Center(
              child: StatefulBuilder(
                builder: (BuildContext _, StateSetter setState) {
                  return Slider(
                    value: value,
                    key: sliderKey,
                    allowedInteraction: SliderInteraction.slideOnly,
                    onChangeStart: (double newValue) {
                      logs.add('onChangeStart');
                    },
                    onChanged: (double newValue) {
                      logs.add('onChanged');
                      setState(() {
                        value = newValue;
                      });
                    },
                    onChangeEnd: (double newValue) {
                      logs.add('onChangeEnd');
                    },
                  );
                },
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildApp());

      expect(logs, isEmpty);

      // test tap
      final TestGesture gesture = await tester.startGesture(centerOfTheSlideTrack);
      await tester.pump();
      // has no effect as tap is disabled, remains 1.0
      expect(value, 1.0);
      expect(logs, <String>['onChangeStart']);

      // test slide
      await gesture.moveTo(startOfTheSliderTrack);
      await tester.pump();
      // changes from 1.0 -> 0.5
      expect(value, 0.5);
      await gesture.moveTo(endOfTheSliderTrack);
      await tester.pump();
      // changes from 0.0 -> 1.0
      expect(value, 1.0);
      expect(logs, <String>['onChangeStart', 'onChanged', 'onChanged']);

      await gesture.up();
      await tester.pump();

      expect(logs, <String>['onChangeStart', 'onChanged', 'onChanged', 'onChangeEnd']);
    });

    testWidgets('SliderInteraction.slideThumb', (WidgetTester tester) async {
      double value = 1.0;
      final Key sliderKey = UniqueKey();
      // (slider's left padding (overlayRadius), windowHeight / 2)
      const Offset startOfTheSliderTrack = Offset(24, 300);
      const Offset centerOfTheSliderTrack = Offset(400, 300);
      const Offset endOfTheSliderTrack = Offset(800 - 24, 300);
      final List<String> logs = <String>[];

      Widget buildApp() {
        return MaterialApp(
          home: Material(
            child: Center(
              child: StatefulBuilder(
                builder: (BuildContext _, StateSetter setState) {
                  return Slider(
                    value: value,
                    key: sliderKey,
                    allowedInteraction: SliderInteraction.slideThumb,
                    onChangeStart: (double newValue) {
                      logs.add('onChangeStart');
                    },
                    onChanged: (double newValue) {
                      logs.add('onChanged');
                      setState(() {
                        value = newValue;
                      });
                    },
                    onChangeEnd: (double newValue) {
                      logs.add('onChangeEnd');
                    },
                  );
                },
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildApp());

      expect(logs, isEmpty);

      // test tap
      final TestGesture gesture = await tester.startGesture(centerOfTheSliderTrack);
      await tester.pump();
      // has no effect, remains 1.0
      expect(value, 1.0);
      expect(logs, isEmpty);

      // test slide
      await gesture.moveTo(startOfTheSliderTrack);
      await tester.pump();
      // has no effect, remains 1.0
      expect(value, 1.0);
      expect(logs, isEmpty);

      // test slide thumb
      await gesture.up();
      await gesture.down(endOfTheSliderTrack); // where the thumb is
      await tester.pump();
      // has no effect, remains 1.0
      expect(value, 1.0);
      expect(logs, <String>['onChangeStart']);

      await gesture.moveTo(centerOfTheSliderTrack);
      await tester.pump();
      // changes from 1.0 -> 0.5
      expect(value, 0.5);
      expect(logs, <String>['onChangeStart', 'onChanged']);

      // test tap inside overlay but not on thumb, then slide
      await gesture.up();
      // default overlay radius is 12, so 10 is inside the overlay
      await gesture.down(centerOfTheSliderTrack.translate(-10, 0));
      await tester.pump();
      // changes from 1.0 -> 0.5
      expect(value, 0.5);
      expect(logs, <String>['onChangeStart', 'onChanged', 'onChangeEnd', 'onChangeStart']);

      await gesture.moveTo(endOfTheSliderTrack.translate(-10, 0));
      await tester.pump();
      // changes from 0.5 -> 1.0
      expect(value, 1.0);
      expect(logs, <String>[
        'onChangeStart',
        'onChanged',
        'onChangeEnd',
        'onChangeStart',
        'onChanged',
      ]);

      await gesture.up();
      await tester.pump();

      expect(logs, <String>[
        'onChangeStart',
        'onChanged',
        'onChangeEnd',
        'onChangeStart',
        'onChanged',
        'onChangeEnd',
      ]);
    });
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/143524.
  testWidgets('Discrete Slider.onChanged is called only once', (WidgetTester tester) async {
    int onChangeCallbackCount = 0;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: Slider(
              max: 5,
              divisions: 5,
              value: 0,
              onChanged: (double newValue) {
                onChangeCallbackCount++;
              },
            ),
          ),
        ),
      ),
    );

    final TestGesture gesture = await tester.startGesture(tester.getTopLeft(find.byType(Slider)));
    await tester.pump(kLongPressTimeout);
    await gesture.moveBy(const Offset(160.0, 0.0));
    await gesture.moveBy(const Offset(1.0, 0.0));
    await gesture.moveBy(const Offset(1.0, 0.0));
    expect(onChangeCallbackCount, 1);
  });

  testWidgets('Skip drawing ValueIndicator shape when label painter text is null', (
    WidgetTester tester,
  ) async {
    double sliderValue = 10;

    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, void Function(void Function()) setState) {
            return Material(
              child: Slider(
                value: sliderValue,
                max: 100,
                label: sliderValue > 50 ? null : sliderValue.toString(),
                divisions: 10,
                onChanged: (double value) {
                  setState(() {
                    sliderValue = value;
                  });
                },
              ),
            );
          },
        ),
      ),
    );

    final RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));

    // Calculate a specific position on the Slider.
    final Rect sliderRect = tester.getRect(find.byType(Slider));
    final Offset tapPositionLeft = Offset(
      sliderRect.left + sliderRect.width * 0.25,
      sliderRect.center.dy,
    );
    final Offset tapPositionRight = Offset(
      sliderRect.left + sliderRect.width * 0.75,
      sliderRect.center.dy,
    );

    // Tap on the 25% position of the Slider.
    await tester.tapAt(tapPositionLeft);
    await tester.pumpAndSettle();
    expect(valueIndicatorBox, paintsExactlyCountTimes(#drawPath, 2));

    // Tap on the 75% position of the Slider.
    await tester.tapAt(tapPositionRight);
    await tester.pumpAndSettle();
    expect(valueIndicatorBox, paintsExactlyCountTimes(#drawPath, 1));
  });

  testWidgets('Slider value indicator is shown when using arrow keys', (WidgetTester tester) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    final ThemeData theme = ThemeData();
    double startValue = 0.0;
    double currentValue = 0.5;
    double endValue = 0.0;

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Slider(
                  value: currentValue,
                  divisions: 5,
                  label: currentValue.toStringAsFixed(1),
                  onChangeStart: (double newValue) {
                    setState(() {
                      startValue = newValue;
                    });
                  },
                  onChanged: (double newValue) {
                    setState(() {
                      currentValue = newValue;
                    });
                  },
                  onChangeEnd: (double newValue) {
                    setState(() {
                      endValue = newValue;
                    });
                  },
                  autofocus: true,
                );
              },
            ),
          ),
        ),
      ),
    );

    // Slider shows value indicator initially on focus.
    await tester.pumpAndSettle();
    RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));
    expect(
      valueIndicatorBox,
      paints
        ..scale()
        ..path(color: theme.colorScheme.primary),
    );

    // Right arrow (increase)
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pumpAndSettle();
    expect(startValue, 0.6);
    expect(currentValue.toStringAsFixed(1), '0.8');
    expect(endValue.toStringAsFixed(1), '0.8');

    // Value indicator is visible.
    valueIndicatorBox = tester.renderObject(find.byType(Overlay));
    expect(
      valueIndicatorBox,
      paints
        ..scale()
        ..path(color: theme.colorScheme.primary),
    );

    // Left arrow (decrease)
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    await tester.pumpAndSettle();
    expect(startValue, 0.8);
    expect(currentValue.toStringAsFixed(1), '0.6');
    expect(endValue.toStringAsFixed(1), '0.6');

    // Value indicator is still visible.
    valueIndicatorBox = tester.renderObject(find.byType(Overlay));
    expect(
      valueIndicatorBox,
      paints
        ..scale()
        ..path(color: theme.colorScheme.primary),
    );

    // Up arrow (increase)
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.pumpAndSettle();
    expect(startValue, 0.6);
    expect(currentValue.toStringAsFixed(1), '0.8');
    expect(endValue.toStringAsFixed(1), '0.8');

    // Value indicator is still visible.
    valueIndicatorBox = tester.renderObject(find.byType(Overlay));
    expect(
      valueIndicatorBox,
      paints
        ..scale()
        ..path(color: theme.colorScheme.primary),
    );

    // Down arrow (decrease)
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pumpAndSettle();
    expect(startValue, 0.8);
    expect(currentValue.toStringAsFixed(1), '0.6');
    expect(endValue.toStringAsFixed(1), '0.6');

    // Value indicator is still visible.
    valueIndicatorBox = tester.renderObject(find.byType(Overlay));
    expect(
      valueIndicatorBox,
      paints
        ..scale()
        ..path(color: theme.colorScheme.primary),
    );
  }, variant: TargetPlatformVariant.desktop());

  testWidgets('Value indicator label is shown when focused', (WidgetTester tester) async {
    double value = 0.5;
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    Widget buildApp() {
      return MaterialApp(
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Slider(
                  value: value,
                  focusNode: focusNode,
                  divisions: 5,
                  label: value.toStringAsFixed(1),
                  onChanged: (double newValue) {
                    setState(() {
                      value = newValue;
                    });
                  },
                );
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildApp());

    // Slider does not show value indicator without focus.
    await tester.pumpAndSettle();
    expect(focusNode.hasFocus, false);
    RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));
    expect(
      valueIndicatorBox,
      isNot(
        paints
          ..path(color: const Color(0xff000000))
          ..paragraph(),
      ),
    );

    focusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(focusNode.hasFocus, true);

    // Slider shows value indicator when focused.
    valueIndicatorBox = tester.renderObject(find.byType(Overlay));
    expect(
      valueIndicatorBox,
      paints
        ..path(color: const Color(0xff000000))
        ..paragraph(),
    );
  }, variant: TargetPlatformVariant.desktop());

  testWidgets('Slider.padding can override the default Slider padding', (
    WidgetTester tester,
  ) async {
    Widget buildSlider({EdgeInsetsGeometry? padding}) {
      return MaterialApp(
        home: Material(
          child: Center(
            child: IntrinsicHeight(
              child: Slider(padding: padding, value: 0.5, onChanged: (double value) {}),
            ),
          ),
        ),
      );
    }

    RenderBox sliderRenderBox() {
      return tester.allRenderObjects.firstWhere(
            (RenderObject object) => object.runtimeType.toString() == '_RenderSlider',
          )
          as RenderBox;
    }

    // Test Slider height and tracks spacing with zero padding.
    await tester.pumpWidget(buildSlider(padding: EdgeInsets.zero));
    await tester.pumpAndSettle();

    // The height equals to the default thumb height.
    expect(sliderRenderBox().size, const Size(800, 20));
    expect(
      find.byType(Slider),
      paints
        // Inactive track.
        ..rrect(rrect: RRect.fromLTRBR(398.0, 8.0, 800.0, 12.0, const Radius.circular(2.0)))
        // Active track.
        ..rrect(rrect: RRect.fromLTRBR(0.0, 7.0, 402.0, 13.0, const Radius.circular(3.0))),
    );

    // Test Slider height and tracks spacing with directional padding.
    const double startPadding = 100;
    const double endPadding = 20;
    await tester.pumpWidget(
      buildSlider(padding: const EdgeInsetsDirectional.only(start: startPadding, end: endPadding)),
    );
    await tester.pumpAndSettle();

    expect(sliderRenderBox().size, const Size(800 - startPadding - endPadding, 20));
    expect(
      find.byType(Slider),
      paints
        // Inactive track.
        ..rrect(rrect: RRect.fromLTRBR(338.0, 8.0, 680.0, 12.0, const Radius.circular(2.0)))
        // Active track.
        ..rrect(rrect: RRect.fromLTRBR(0.0, 7.0, 342.0, 13.0, const Radius.circular(3.0))),
    );

    // Test Slider height and tracks spacing with top and bottom padding.
    const double topPadding = 100;
    const double bottomPadding = 20;
    const double trackHeight = 20;
    await tester.pumpWidget(
      buildSlider(
        padding: const EdgeInsetsDirectional.only(top: topPadding, bottom: bottomPadding),
      ),
    );
    await tester.pumpAndSettle();

    expect(
      tester.getSize(find.byType(Slider)),
      const Size(800, topPadding + trackHeight + bottomPadding),
    );
    expect(sliderRenderBox().size, const Size(800, 20));
    expect(
      find.byType(Slider),
      paints
        // Inactive track.
        ..rrect(rrect: RRect.fromLTRBR(398.0, 8.0, 800.0, 12.0, const Radius.circular(2.0)))
        // Active track.
        ..rrect(rrect: RRect.fromLTRBR(0.0, 7.0, 402.0, 13.0, const Radius.circular(3.0))),
    );
  });

  testWidgets('Default Slider when year2023 is false', (WidgetTester tester) async {
    debugDisableShadows = false;
    try {
      final ThemeData theme = ThemeData();
      final ColorScheme colorScheme = theme.colorScheme;
      final Color activeTrackColor = colorScheme.primary;
      final Color inactiveTrackColor = colorScheme.secondaryContainer;
      final Color secondaryActiveTrackColor = colorScheme.primary.withOpacity(0.54);
      final Color disabledActiveTrackColor = colorScheme.onSurface.withOpacity(0.38);
      final Color disabledInactiveTrackColor = colorScheme.onSurface.withOpacity(0.12);
      final Color disabledSecondaryActiveTrackColor = colorScheme.onSurface.withOpacity(0.38);
      final Color activeTickMarkColor = colorScheme.onPrimary;
      final Color inactiveTickMarkColor = colorScheme.onSecondaryContainer;
      final Color disabledActiveTickMarkColor = colorScheme.onInverseSurface;
      final Color disabledInactiveTickMarkColor = colorScheme.onSurface;
      final Color thumbColor = colorScheme.primary;
      final Color disabledThumbColor = colorScheme.onSurface.withOpacity(0.38);
      final Color valueIndicatorColor = colorScheme.inverseSurface;
      double value = 0.45;
      Widget buildApp({int? divisions, bool enabled = true}) {
        final ValueChanged<double>? onChanged =
            !enabled
                ? null
                : (double d) {
                  value = d;
                };
        return MaterialApp(
          home: Directionality(
            textDirection: TextDirection.ltr,
            child: Material(
              child: Center(
                child: Theme(
                  data: theme,
                  child: Slider(
                    year2023: false,
                    value: value,
                    secondaryTrackValue: 0.75,
                    label: '$value',
                    divisions: divisions,
                    onChanged: onChanged,
                  ),
                ),
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildApp());

      final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));

      // Test default track shape.
      const Radius trackOuterCornerRadius = Radius.circular(8.0);
      const Radius trackInnerCornerRadius = Radius.circular(2.0);
      expect(
        material,
        paints
          // Active track.
          ..rrect(
            rrect: RRect.fromLTRBAndCorners(
              24.0,
              292.0,
              356.4,
              308.0,
              topLeft: trackOuterCornerRadius,
              topRight: trackInnerCornerRadius,
              bottomRight: trackInnerCornerRadius,
              bottomLeft: trackOuterCornerRadius,
            ),
            color: activeTrackColor,
          )
          // Inactive track.
          ..rrect(
            rrect: RRect.fromLTRBAndCorners(
              368.4,
              292.0,
              776.0,
              308.0,
              topLeft: trackInnerCornerRadius,
              topRight: trackOuterCornerRadius,
              bottomRight: trackOuterCornerRadius,
              bottomLeft: trackInnerCornerRadius,
            ),
            color: inactiveTrackColor,
          ),
      );

      // Test default colors for enabled slider.
      expect(
        material,
        paints
          ..circle()
          ..rrect(color: thumbColor),
      );
      expect(
        material,
        isNot(
          paints
            ..circle()
            ..circle(color: disabledThumbColor),
        ),
      );
      expect(material, isNot(paints..rrect(color: disabledActiveTrackColor)));
      expect(material, isNot(paints..rrect(color: disabledInactiveTrackColor)));
      expect(material, isNot(paints..rrect(color: disabledSecondaryActiveTrackColor)));

      // Test defaults colors for discrete slider.
      await tester.pumpWidget(buildApp(divisions: 3));
      expect(
        material,
        paints
          ..rrect(color: activeTrackColor)
          ..rrect(color: inactiveTrackColor)
          ..rrect(color: secondaryActiveTrackColor)
          ..circle(color: activeTickMarkColor)
          ..circle(color: activeTickMarkColor)
          ..circle(color: inactiveTickMarkColor)
          ..circle(color: inactiveTickMarkColor),
      );
      expect(material, isNot(paints..circle(color: disabledThumbColor)));
      expect(material, isNot(paints..rrect(color: disabledActiveTrackColor)));
      expect(material, isNot(paints..rrect(color: disabledInactiveTrackColor)));
      expect(material, isNot(paints..rrect(color: disabledSecondaryActiveTrackColor)));

      // Test defaults colors for disabled slider.
      await tester.pumpWidget(buildApp(enabled: false));
      await tester.pumpAndSettle();
      expect(
        material,
        paints
          ..rrect(color: disabledActiveTrackColor)
          ..rrect(color: disabledInactiveTrackColor)
          ..rrect(color: disabledSecondaryActiveTrackColor),
      );
      expect(
        material,
        paints
          ..circle()
          ..rrect(color: disabledThumbColor),
      );
      expect(
        material,
        isNot(
          paints
            ..circle()
            ..rrect(color: thumbColor),
        ),
      );
      expect(material, isNot(paints..rrect(color: activeTrackColor)));
      expect(material, isNot(paints..rrect(color: inactiveTrackColor)));
      expect(material, isNot(paints..rrect(color: secondaryActiveTrackColor)));

      // Test defaults colors for disabled discrete slider.
      await tester.pumpWidget(buildApp(divisions: 3, enabled: false));
      expect(
        material,
        paints
          ..rrect(color: disabledActiveTrackColor)
          ..rrect(color: disabledInactiveTrackColor)
          ..rrect(color: disabledSecondaryActiveTrackColor)
          ..circle(color: disabledActiveTickMarkColor)
          ..circle(color: disabledActiveTickMarkColor)
          ..circle(color: disabledInactiveTickMarkColor)
          ..circle(color: disabledInactiveTickMarkColor)
          ..rrect(color: disabledThumbColor),
      );
      expect(
        material,
        isNot(
          paints
            ..circle()
            ..rrect(color: thumbColor),
        ),
      );
      expect(material, isNot(paints..rrect(color: activeTrackColor)));
      expect(material, isNot(paints..rrect(color: inactiveTrackColor)));
      expect(material, isNot(paints..rrect(color: secondaryActiveTrackColor)));

      await tester.pumpWidget(buildApp(divisions: 3));
      await tester.pumpAndSettle();

      final Offset center = tester.getCenter(find.byType(Slider));
      final TestGesture gesture = await tester.startGesture(center);
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();

      final RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));
      expect(
        valueIndicatorBox,
        paints
          ..scale()
          ..rrect(color: valueIndicatorColor),
      );
      await gesture.up();
    } finally {
      debugDisableShadows = true;
    }
  });

  testWidgets('Slider value indicator text when year2023 is false', (WidgetTester tester) async {
    const double value = 50;
    final List<InlineSpan> log = <InlineSpan>[];
    final LoggingValueIndicatorShape loggingValueIndicatorShape = LoggingValueIndicatorShape(log);
    final ThemeData theme = ThemeData(
      sliderTheme: SliderThemeData(valueIndicatorShape: loggingValueIndicatorShape),
    );

    Widget buildSlider() {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: Slider(
              year2023: false,
              max: 100.0,
              divisions: 4,
              label: '${value.round()}',
              value: value,
              onChanged: (double newValue) {},
            ),
          ),
        ),
      );
    }

    // Normal text
    await tester.pumpWidget(buildSlider());
    final Offset center = tester.getCenter(find.byType(Slider));
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pumpAndSettle();

    expect(log.last.toPlainText(), '50');
    expect(log.last.style!.fontSize, 14.0);
    expect(log.last.style!.color, theme.colorScheme.onInverseSurface);

    await gesture.up();
    await tester.pumpAndSettle();
  });
}
