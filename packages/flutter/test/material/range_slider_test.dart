// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/src/physics/utils.dart' show nearEqual;
import 'package:flutter_test/flutter_test.dart';

import '../widgets/semantics_tester.dart';

void main() {
  // Regression test for https://github.com/flutter/flutter/issues/105833
  testWidgets('Drag gesture uses provided gesture settings', (WidgetTester tester) async {
    RangeValues values = const RangeValues(0.1, 0.5);
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
                      child: RangeSlider(
                        key: sliderKey,
                        values: values,
                        onChanged: (RangeValues newValues) {
                          setState(() {
                            values = newValues;
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

    expect(values, const RangeValues(0.1, 0.5));
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
                      child: RangeSlider(
                        key: sliderKey,
                        values: values,
                        onChanged: (RangeValues newValues) {
                          setState(() {
                            values = newValues;
                          });
                        },
                        onChangeEnd: (RangeValues newValues) {
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

  testWidgets('Range Slider can move when tapped (continuous LTR)', (WidgetTester tester) async {
    RangeValues values = const RangeValues(0.3, 0.7);

    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: RangeSlider(
                    values: values,
                    onChanged: (RangeValues newValues) {
                      setState(() {
                        values = newValues;
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

    // No thumbs get select when tapping between the thumbs outside the touch
    // boundaries
    expect(values, equals(const RangeValues(0.3, 0.7)));
    //  taps at 0.5
    await tester.tap(find.byType(RangeSlider));
    await tester.pump();
    expect(values, equals(const RangeValues(0.3, 0.7)));

    // Get the bounds of the track by finding the slider edges and translating
    // inwards by the overlay radius.
    final Offset topLeft = tester.getTopLeft(find.byType(RangeSlider)).translate(24, 0);
    final Offset bottomRight = tester.getBottomRight(find.byType(RangeSlider)).translate(-24, 0);

    // The start thumb is selected when tapping the left inactive track.
    final Offset leftTarget = topLeft + (bottomRight - topLeft) * 0.1;
    await tester.tapAt(leftTarget);
    expect(values.start, moreOrLessEquals(0.1, epsilon: 0.01));
    expect(values.end, equals(0.7));

    // The end thumb is selected when tapping the right inactive track.
    await tester.pump();
    final Offset rightTarget = topLeft + (bottomRight - topLeft) * 0.9;
    await tester.tapAt(rightTarget);
    expect(values.start, moreOrLessEquals(0.1, epsilon: 0.01));
    expect(values.end, moreOrLessEquals(0.9, epsilon: 0.01));
  });

  testWidgets('Range Slider can move when tapped (continuous RTL)', (WidgetTester tester) async {
    RangeValues values = const RangeValues(0.3, 0.7);

    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: RangeSlider(
                    values: values,
                    onChanged: (RangeValues newValues) {
                      setState(() {
                        values = newValues;
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

    // No thumbs get select when tapping between the thumbs outside the touch
    // boundaries
    expect(values, equals(const RangeValues(0.3, 0.7)));
    // taps at 0.5
    await tester.tap(find.byType(RangeSlider));
    await tester.pump();
    expect(values, equals(const RangeValues(0.3, 0.7)));

    // Get the bounds of the track by finding the slider edges and translating
    // inwards by the overlay radius.
    final Offset topLeft = tester.getTopLeft(find.byType(RangeSlider)).translate(24, 0);
    final Offset bottomRight = tester.getBottomRight(find.byType(RangeSlider)).translate(-24, 0);

    // The end thumb is selected when tapping the left inactive track.
    final Offset leftTarget = topLeft + (bottomRight - topLeft) * 0.1;
    await tester.tapAt(leftTarget);
    expect(values.start, 0.3);
    expect(values.end, moreOrLessEquals(0.9, epsilon: 0.01));

    // The start thumb is selected when tapping the right inactive track.
    await tester.pump();
    final Offset rightTarget = topLeft + (bottomRight - topLeft) * 0.9;
    await tester.tapAt(rightTarget);
    expect(values.start, moreOrLessEquals(0.1, epsilon: 0.01));
    expect(values.end, moreOrLessEquals(0.9, epsilon: 0.01));
  });

  testWidgets('Range Slider can move when tapped (discrete LTR)', (WidgetTester tester) async {
    RangeValues values = const RangeValues(30, 70);

    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: RangeSlider(
                    values: values,
                    max: 100.0,
                    divisions: 10,
                    onChanged: (RangeValues newValues) {
                      setState(() {
                        values = newValues;
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

    // No thumbs get select when tapping between the thumbs outside the touch
    // boundaries
    expect(values, equals(const RangeValues(30, 70)));
    // taps at 0.5
    await tester.tap(find.byType(RangeSlider));
    await tester.pumpAndSettle();
    expect(values, equals(const RangeValues(30, 70)));

    // Get the bounds of the track by finding the slider edges and translating
    // inwards by the overlay radius.
    final Offset topLeft = tester.getTopLeft(find.byType(RangeSlider)).translate(24, 0);
    final Offset bottomRight = tester.getBottomRight(find.byType(RangeSlider)).translate(-24, 0);

    // The start thumb is selected when tapping the left inactive track.
    final Offset leftTarget = topLeft + (bottomRight - topLeft) * 0.1;
    await tester.tapAt(leftTarget);
    await tester.pumpAndSettle();
    expect(values.start.round(), equals(10));
    expect(values.end.round(), equals(70));

    // The end thumb is selected when tapping the right inactive track.
    await tester.pump();
    final Offset rightTarget = topLeft + (bottomRight - topLeft) * 0.9;
    await tester.tapAt(rightTarget);
    await tester.pumpAndSettle();
    expect(values.start.round(), equals(10));
    expect(values.end.round(), equals(90));
  });

  testWidgets('Range Slider can move when tapped (discrete RTL)', (WidgetTester tester) async {
    RangeValues values = const RangeValues(30, 70);

    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: RangeSlider(
                    values: values,
                    max: 100,
                    divisions: 10,
                    onChanged: (RangeValues newValues) {
                      setState(() {
                        values = newValues;
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

    // No thumbs get select when tapping between the thumbs outside the touch
    // boundaries
    expect(values, equals(const RangeValues(30, 70)));
    // taps at 0.5
    await tester.tap(find.byType(RangeSlider));
    await tester.pumpAndSettle();
    expect(values, equals(const RangeValues(30, 70)));

    // Get the bounds of the track by finding the slider edges and translating
    // inwards by the overlay radius.
    final Offset topLeft = tester.getTopLeft(find.byType(RangeSlider)).translate(24, 0);
    final Offset bottomRight = tester.getBottomRight(find.byType(RangeSlider)).translate(-24, 0);

    // The start thumb is selected when tapping the left inactive track.
    final Offset leftTarget = topLeft + (bottomRight - topLeft) * 0.1;
    await tester.tapAt(leftTarget);
    await tester.pumpAndSettle();
    expect(values.start.round(), equals(30));
    expect(values.end.round(), equals(90));

    // The end thumb is selected when tapping the right inactive track.
    await tester.pump();
    final Offset rightTarget = topLeft + (bottomRight - topLeft) * 0.9;
    await tester.tapAt(rightTarget);
    await tester.pumpAndSettle();
    expect(values.start.round(), equals(10));
    expect(values.end.round(), equals(90));
  });

  testWidgets('Range Slider thumbs can be dragged to the min and max (continuous LTR)', (
    WidgetTester tester,
  ) async {
    RangeValues values = const RangeValues(0.3, 0.7);

    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: RangeSlider(
                    values: values,
                    onChanged: (RangeValues newValues) {
                      setState(() {
                        values = newValues;
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

    // Get the bounds of the track by finding the slider edges and translating
    // inwards by the overlay radius.
    final Offset topLeft = tester.getTopLeft(find.byType(RangeSlider)).translate(24, 0);
    final Offset bottomRight = tester.getBottomRight(find.byType(RangeSlider)).translate(-24, 0);

    // Drag the start thumb to the min.
    final Offset leftTarget = topLeft + (bottomRight - topLeft) * 0.3;
    await tester.dragFrom(leftTarget, topLeft + (bottomRight - topLeft) * -0.4);
    expect(values.start, equals(0));

    // Drag the end thumb to the max.
    await tester.pumpAndSettle();
    final Offset rightTarget = topLeft + (bottomRight - topLeft) * 0.7;
    await tester.dragFrom(rightTarget, topLeft + (bottomRight - topLeft) * 0.4);
    expect(values.end, equals(1));
  });

  testWidgets('Range Slider thumbs can be dragged to the min and max (continuous RTL)', (
    WidgetTester tester,
  ) async {
    RangeValues values = const RangeValues(0.3, 0.7);

    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: RangeSlider(
                    values: values,
                    onChanged: (RangeValues newValues) {
                      setState(() {
                        values = newValues;
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

    // Get the bounds of the track by finding the slider edges and translating
    // inwards by the overlay radius.
    final Offset topLeft = tester.getTopLeft(find.byType(RangeSlider)).translate(24, 0);
    final Offset bottomRight = tester.getBottomRight(find.byType(RangeSlider)).translate(-24, 0);

    // Drag the end thumb to the max.
    final Offset leftTarget = topLeft + (bottomRight - topLeft) * 0.3;
    await tester.dragFrom(leftTarget, topLeft + (bottomRight - topLeft) * -0.4);
    expect(values.end, equals(1));

    // Drag the start thumb to the min.
    await tester.pumpAndSettle();
    final Offset rightTarget = topLeft + (bottomRight - topLeft) * 0.7;
    await tester.dragFrom(rightTarget, topLeft + (bottomRight - topLeft) * 0.4);
    expect(values.start, equals(0));
  });

  testWidgets('Range Slider thumbs can be dragged to the min and max (discrete LTR)', (
    WidgetTester tester,
  ) async {
    RangeValues values = const RangeValues(30, 70);

    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: RangeSlider(
                    values: values,
                    max: 100,
                    divisions: 10,
                    onChanged: (RangeValues newValues) {
                      setState(() {
                        values = newValues;
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

    // Get the bounds of the track by finding the slider edges and translating
    // inwards by the overlay radius.
    final Offset topLeft = tester.getTopLeft(find.byType(RangeSlider)).translate(24, 0);
    final Offset bottomRight = tester.getBottomRight(find.byType(RangeSlider)).translate(-24, 0);

    // Drag the start thumb to the min.
    final Offset leftTarget = topLeft + (bottomRight - topLeft) * 0.3;
    await tester.dragFrom(leftTarget, topLeft + (bottomRight - topLeft) * -0.4);
    expect(values.start, equals(0));

    // Drag the end thumb to the max.
    await tester.pumpAndSettle();
    final Offset rightTarget = topLeft + (bottomRight - topLeft) * 0.7;
    await tester.dragFrom(rightTarget, topLeft + (bottomRight - topLeft) * 0.4);
    expect(values.end, equals(100));
  });

  testWidgets('Range Slider thumbs can be dragged to the min and max (discrete RTL)', (
    WidgetTester tester,
  ) async {
    RangeValues values = const RangeValues(30, 70);

    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: RangeSlider(
                    values: values,
                    max: 100,
                    divisions: 10,
                    onChanged: (RangeValues newValues) {
                      setState(() {
                        values = newValues;
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

    // Get the bounds of the track by finding the slider edges and translating
    // inwards by the overlay radius.
    final Offset topLeft = tester.getTopLeft(find.byType(RangeSlider)).translate(24, 0);
    final Offset bottomRight = tester.getBottomRight(find.byType(RangeSlider)).translate(-24, 0);

    // Drag the end thumb to the max.
    final Offset leftTarget = topLeft + (bottomRight - topLeft) * 0.3;
    await tester.dragFrom(leftTarget, topLeft + (bottomRight - topLeft) * -0.4);
    expect(values.end, equals(100));

    // Drag the start thumb to the min.
    await tester.pumpAndSettle();
    final Offset rightTarget = topLeft + (bottomRight - topLeft) * 0.7;
    await tester.dragFrom(rightTarget, topLeft + (bottomRight - topLeft) * 0.4);
    expect(values.start, equals(0));
  });

  testWidgets(
    'Range Slider thumbs can be dragged together and the start thumb can be dragged apart (continuous LTR)',
    (WidgetTester tester) async {
      RangeValues values = const RangeValues(0.3, 0.7);

      await tester.pumpWidget(
        MaterialApp(
          home: Directionality(
            textDirection: TextDirection.ltr,
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Material(
                  child: Center(
                    child: RangeSlider(
                      values: values,
                      onChanged: (RangeValues newValues) {
                        setState(() {
                          values = newValues;
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

      // Get the bounds of the track by finding the slider edges and translating
      // inwards by the overlay radius.
      final Offset topLeft = tester.getTopLeft(find.byType(RangeSlider)).translate(24, 0);
      final Offset bottomRight = tester.getBottomRight(find.byType(RangeSlider)).translate(-24, 0);
      final Offset middle = topLeft + bottomRight / 2;

      // Drag the start thumb towards the center.
      final Offset leftTarget = topLeft + (bottomRight - topLeft) * 0.3;
      await tester.dragFrom(leftTarget, middle - leftTarget);
      expect(values.start, moreOrLessEquals(0.5, epsilon: 0.05));

      // Drag the end thumb towards the center.
      await tester.pumpAndSettle();
      final Offset rightTarget = topLeft + (bottomRight - topLeft) * 0.7;
      await tester.dragFrom(rightTarget, middle - rightTarget);
      expect(values.end, moreOrLessEquals(0.5, epsilon: 0.05));

      // Drag the start thumb apart.
      await tester.pumpAndSettle();
      await tester.dragFrom(middle, -(bottomRight - topLeft) * 0.3);
      expect(values.start, moreOrLessEquals(0.2, epsilon: 0.05));
    },
  );

  testWidgets(
    'Range Slider thumbs can be dragged together and the start thumb can be dragged apart (continuous RTL)',
    (WidgetTester tester) async {
      RangeValues values = const RangeValues(0.3, 0.7);

      await tester.pumpWidget(
        MaterialApp(
          home: Directionality(
            textDirection: TextDirection.rtl,
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Material(
                  child: Center(
                    child: RangeSlider(
                      values: values,
                      onChanged: (RangeValues newValues) {
                        setState(() {
                          values = newValues;
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

      // Get the bounds of the track by finding the slider edges and translating
      // inwards by the overlay radius.
      final Offset topLeft = tester.getTopLeft(find.byType(RangeSlider)).translate(24, 0);
      final Offset bottomRight = tester.getBottomRight(find.byType(RangeSlider)).translate(-24, 0);
      final Offset middle = topLeft + bottomRight / 2;

      // Drag the end thumb towards the center.
      final Offset leftTarget = topLeft + (bottomRight - topLeft) * 0.3;
      await tester.dragFrom(leftTarget, middle - leftTarget);
      expect(values.end, moreOrLessEquals(0.5, epsilon: 0.05));

      // Drag the start thumb towards the center.
      await tester.pumpAndSettle();
      final Offset rightTarget = topLeft + (bottomRight - topLeft) * 0.7;
      await tester.dragFrom(rightTarget, middle - rightTarget);
      expect(values.start, moreOrLessEquals(0.5, epsilon: 0.05));

      // Drag the start thumb apart.
      await tester.pumpAndSettle();
      await tester.dragFrom(middle, (bottomRight - topLeft) * 0.3);
      expect(values.start, moreOrLessEquals(0.2, epsilon: 0.05));
    },
  );

  testWidgets(
    'Range Slider thumbs can be dragged together and the start thumb can be dragged apart (discrete LTR)',
    (WidgetTester tester) async {
      RangeValues values = const RangeValues(30, 70);

      await tester.pumpWidget(
        MaterialApp(
          home: Directionality(
            textDirection: TextDirection.ltr,
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Material(
                  child: Center(
                    child: RangeSlider(
                      values: values,
                      max: 100,
                      divisions: 10,
                      onChanged: (RangeValues newValues) {
                        setState(() {
                          values = newValues;
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

      // Get the bounds of the track by finding the slider edges and translating
      // inwards by the overlay radius.
      final Offset topLeft = tester.getTopLeft(find.byType(RangeSlider)).translate(24, 0);
      final Offset bottomRight = tester.getBottomRight(find.byType(RangeSlider)).translate(-24, 0);
      final Offset middle = topLeft + bottomRight / 2;

      // Drag the start thumb towards the center.
      final Offset leftTarget = topLeft + (bottomRight - topLeft) * 0.3;
      await tester.dragFrom(leftTarget, middle - leftTarget);
      expect(values.start, moreOrLessEquals(50, epsilon: 0.01));

      // Drag the end thumb towards the center.
      await tester.pumpAndSettle();
      final Offset rightTarget = topLeft + (bottomRight - topLeft) * 0.7;
      await tester.dragFrom(rightTarget, middle - rightTarget);
      expect(values.end, moreOrLessEquals(50, epsilon: 0.01));

      // Drag the start thumb apart.
      await tester.pumpAndSettle();
      await tester.dragFrom(middle, -(bottomRight - topLeft) * 0.3);
      expect(values.start, moreOrLessEquals(20, epsilon: 0.01));
    },
  );

  testWidgets(
    'Range Slider thumbs can be dragged together and the start thumb can be dragged apart (discrete RTL)',
    (WidgetTester tester) async {
      RangeValues values = const RangeValues(30, 70);

      await tester.pumpWidget(
        MaterialApp(
          home: Directionality(
            textDirection: TextDirection.rtl,
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Material(
                  child: Center(
                    child: RangeSlider(
                      values: values,
                      max: 100,
                      divisions: 10,
                      onChanged: (RangeValues newValues) {
                        setState(() {
                          values = newValues;
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

      // Get the bounds of the track by finding the slider edges and translating
      // inwards by the overlay radius.
      final Offset topLeft = tester.getTopLeft(find.byType(RangeSlider)).translate(24, 0);
      final Offset bottomRight = tester.getBottomRight(find.byType(RangeSlider)).translate(-24, 0);
      final Offset middle = topLeft + bottomRight / 2;

      // Drag the end thumb towards the center.
      final Offset leftTarget = topLeft + (bottomRight - topLeft) * 0.3;
      await tester.dragFrom(leftTarget, middle - leftTarget);
      expect(values.end, moreOrLessEquals(50, epsilon: 0.01));

      // Drag the start thumb towards the center.
      await tester.pumpAndSettle();
      final Offset rightTarget = topLeft + (bottomRight - topLeft) * 0.7;
      await tester.dragFrom(rightTarget, middle - rightTarget);
      expect(values.start, moreOrLessEquals(50, epsilon: 0.01));

      // Drag the start thumb apart.
      await tester.pumpAndSettle();
      await tester.dragFrom(middle, (bottomRight - topLeft) * 0.3);
      expect(values.start, moreOrLessEquals(20, epsilon: 0.01));
    },
  );

  testWidgets(
    'Range Slider thumbs can be dragged together and the end thumb can be dragged apart (continuous LTR)',
    (WidgetTester tester) async {
      RangeValues values = const RangeValues(0.3, 0.7);

      await tester.pumpWidget(
        MaterialApp(
          home: Directionality(
            textDirection: TextDirection.ltr,
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Material(
                  child: Center(
                    child: RangeSlider(
                      values: values,
                      onChanged: (RangeValues newValues) {
                        setState(() {
                          values = newValues;
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

      // Get the bounds of the track by finding the slider edges and translating
      // inwards by the overlay radius.
      final Offset topLeft = tester.getTopLeft(find.byType(RangeSlider)).translate(24, 0);
      final Offset bottomRight = tester.getBottomRight(find.byType(RangeSlider)).translate(-24, 0);
      final Offset middle = topLeft + bottomRight / 2;

      // Drag the start thumb towards the center.
      final Offset leftTarget = topLeft + (bottomRight - topLeft) * 0.3;
      await tester.dragFrom(leftTarget, middle - leftTarget);
      expect(values.start, moreOrLessEquals(0.5, epsilon: 0.05));

      // Drag the end thumb towards the center.
      await tester.pumpAndSettle();
      final Offset rightTarget = topLeft + (bottomRight - topLeft) * 0.7;
      await tester.dragFrom(rightTarget, middle - rightTarget);
      expect(values.end, moreOrLessEquals(0.5, epsilon: 0.05));

      // Drag the end thumb apart.
      await tester.pumpAndSettle();
      await tester.dragFrom(middle, (bottomRight - topLeft) * 0.3);
      expect(values.end, moreOrLessEquals(0.8, epsilon: 0.05));
    },
  );

  testWidgets(
    'Range Slider thumbs can be dragged together and the end thumb can be dragged apart (continuous RTL)',
    (WidgetTester tester) async {
      RangeValues values = const RangeValues(0.3, 0.7);

      await tester.pumpWidget(
        MaterialApp(
          home: Directionality(
            textDirection: TextDirection.rtl,
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Material(
                  child: Center(
                    child: RangeSlider(
                      values: values,
                      onChanged: (RangeValues newValues) {
                        setState(() {
                          values = newValues;
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

      // Get the bounds of the track by finding the slider edges and translating
      // inwards by the overlay radius.
      final Offset topLeft = tester.getTopLeft(find.byType(RangeSlider)).translate(24, 0);
      final Offset bottomRight = tester.getBottomRight(find.byType(RangeSlider)).translate(-24, 0);
      final Offset middle = topLeft + bottomRight / 2;

      // Drag the end thumb towards the center.
      final Offset leftTarget = topLeft + (bottomRight - topLeft) * 0.3;
      await tester.dragFrom(leftTarget, middle - leftTarget);
      expect(values.end, moreOrLessEquals(0.5, epsilon: 0.05));

      // Drag the start thumb towards the center.
      await tester.pumpAndSettle();
      final Offset rightTarget = topLeft + (bottomRight - topLeft) * 0.7;
      await tester.dragFrom(rightTarget, middle - rightTarget);
      expect(values.start, moreOrLessEquals(0.5, epsilon: 0.05));

      // Drag the end thumb apart.
      await tester.pumpAndSettle();
      await tester.dragFrom(middle, -(bottomRight - topLeft) * 0.3);
      expect(values.end, moreOrLessEquals(0.8, epsilon: 0.05));
    },
  );

  testWidgets(
    'Range Slider thumbs can be dragged together and the end thumb can be dragged apart (discrete LTR)',
    (WidgetTester tester) async {
      RangeValues values = const RangeValues(30, 70);

      await tester.pumpWidget(
        MaterialApp(
          home: Directionality(
            textDirection: TextDirection.ltr,
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Material(
                  child: Center(
                    child: RangeSlider(
                      values: values,
                      max: 100,
                      divisions: 10,
                      onChanged: (RangeValues newValues) {
                        setState(() {
                          values = newValues;
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

      // Get the bounds of the track by finding the slider edges and translating
      // inwards by the overlay radius.
      final Offset topLeft = tester.getTopLeft(find.byType(RangeSlider)).translate(24, 0);
      final Offset bottomRight = tester.getBottomRight(find.byType(RangeSlider)).translate(-24, 0);
      final Offset middle = topLeft + bottomRight / 2;

      // Drag the start thumb towards the center.
      final Offset leftTarget = topLeft + (bottomRight - topLeft) * 0.3;
      await tester.dragFrom(leftTarget, middle - leftTarget);
      expect(values.start, moreOrLessEquals(50, epsilon: 0.01));

      // Drag the end thumb towards the center.
      await tester.pumpAndSettle();
      final Offset rightTarget = topLeft + (bottomRight - topLeft) * 0.7;
      await tester.dragFrom(rightTarget, middle - rightTarget);
      expect(values.end, moreOrLessEquals(50, epsilon: 0.01));

      // Drag the end thumb apart.
      await tester.pumpAndSettle();
      await tester.dragFrom(middle, (bottomRight - topLeft) * 0.3);
      expect(values.end, moreOrLessEquals(80, epsilon: 0.01));
    },
  );

  testWidgets(
    'Range Slider thumbs can be dragged together and the end thumb can be dragged apart (discrete RTL)',
    (WidgetTester tester) async {
      RangeValues values = const RangeValues(30, 70);

      await tester.pumpWidget(
        MaterialApp(
          home: Directionality(
            textDirection: TextDirection.rtl,
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Material(
                  child: Center(
                    child: RangeSlider(
                      values: values,
                      max: 100,
                      divisions: 10,
                      onChanged: (RangeValues newValues) {
                        setState(() {
                          values = newValues;
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

      // Get the bounds of the track by finding the slider edges and translating
      // inwards by the overlay radius.
      final Offset topLeft = tester.getTopLeft(find.byType(RangeSlider)).translate(24, 0);
      final Offset bottomRight = tester.getBottomRight(find.byType(RangeSlider)).translate(-24, 0);
      final Offset middle = topLeft + bottomRight / 2;

      // Drag the end thumb towards the center.
      final Offset leftTarget = topLeft + (bottomRight - topLeft) * 0.3;
      await tester.dragFrom(leftTarget, middle - leftTarget);
      expect(values.end, moreOrLessEquals(50, epsilon: 0.01));

      // Drag the start thumb towards the center.
      await tester.pumpAndSettle();
      final Offset rightTarget = topLeft + (bottomRight - topLeft) * 0.7;
      await tester.dragFrom(rightTarget, middle - rightTarget);
      expect(values.start, moreOrLessEquals(50, epsilon: 0.01));

      // Drag the end thumb apart.
      await tester.pumpAndSettle();
      await tester.dragFrom(middle, -(bottomRight - topLeft) * 0.3);
      expect(values.end, moreOrLessEquals(80, epsilon: 0.01));
    },
  );

  testWidgets(
    'Range Slider onChangeEnd and onChangeStart are called on an interaction initiated by tap',
    (WidgetTester tester) async {
      RangeValues values = const RangeValues(30, 70);
      RangeValues? startValues;
      RangeValues? endValues;

      await tester.pumpWidget(
        MaterialApp(
          home: Directionality(
            textDirection: TextDirection.ltr,
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Material(
                  child: Center(
                    child: RangeSlider(
                      values: values,
                      max: 100,
                      onChanged: (RangeValues newValues) {
                        setState(() {
                          values = newValues;
                        });
                      },
                      onChangeStart: (RangeValues newValues) {
                        startValues = newValues;
                      },
                      onChangeEnd: (RangeValues newValues) {
                        endValues = newValues;
                      },
                    ),
                  ),
                );
              },
            ),
          ),
        ),
      );

      // Get the bounds of the track by finding the slider edges and translating
      // inwards by the overlay radius.
      final Offset topLeft = tester.getTopLeft(find.byType(RangeSlider)).translate(24, 0);
      final Offset bottomRight = tester.getBottomRight(find.byType(RangeSlider)).translate(-24, 0);

      // Drag the start thumb towards the center.
      final Offset leftTarget = topLeft + (bottomRight - topLeft) * 0.3;
      expect(startValues, null);
      expect(endValues, null);
      await tester.dragFrom(leftTarget, (bottomRight - topLeft) * 0.2);
      expect(startValues!.start, moreOrLessEquals(30, epsilon: 1));
      expect(startValues!.end, moreOrLessEquals(70, epsilon: 1));
      expect(values.start, moreOrLessEquals(50, epsilon: 1));
      expect(values.end, moreOrLessEquals(70, epsilon: 1));
      expect(endValues!.start, moreOrLessEquals(50, epsilon: 1));
      expect(endValues!.end, moreOrLessEquals(70, epsilon: 1));
    },
  );

  testWidgets(
    'Range Slider onChangeEnd and onChangeStart are called on an interaction initiated by drag',
    (WidgetTester tester) async {
      RangeValues values = const RangeValues(30, 70);
      late RangeValues startValues;
      late RangeValues endValues;

      await tester.pumpWidget(
        MaterialApp(
          home: Directionality(
            textDirection: TextDirection.ltr,
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Material(
                  child: Center(
                    child: RangeSlider(
                      values: values,
                      max: 100,
                      onChanged: (RangeValues newValues) {
                        setState(() {
                          values = newValues;
                        });
                      },
                      onChangeStart: (RangeValues newValues) {
                        startValues = newValues;
                      },
                      onChangeEnd: (RangeValues newValues) {
                        endValues = newValues;
                      },
                    ),
                  ),
                );
              },
            ),
          ),
        ),
      );

      // Get the bounds of the track by finding the slider edges and translating
      // inwards by the overlay radius
      final Offset topLeft = tester.getTopLeft(find.byType(RangeSlider)).translate(24, 0);
      final Offset bottomRight = tester.getBottomRight(find.byType(RangeSlider)).translate(-24, 0);

      // Drag the thumbs together.
      final Offset leftTarget = topLeft + (bottomRight - topLeft) * 0.3;
      await tester.dragFrom(leftTarget, (bottomRight - topLeft) * 0.2);
      await tester.pumpAndSettle();
      final Offset rightTarget = topLeft + (bottomRight - topLeft) * 0.7;
      await tester.dragFrom(rightTarget, (bottomRight - topLeft) * -0.2);
      await tester.pumpAndSettle();
      expect(values.start, moreOrLessEquals(50, epsilon: 1));
      expect(values.end, moreOrLessEquals(51, epsilon: 1));

      // Drag the end thumb to the right.
      final Offset middleTarget = topLeft + (bottomRight - topLeft) * 0.5;
      await tester.dragFrom(middleTarget, (bottomRight - topLeft) * 0.4);
      await tester.pumpAndSettle();
      expect(startValues.start, moreOrLessEquals(50, epsilon: 1));
      expect(startValues.end, moreOrLessEquals(51, epsilon: 1));
      expect(endValues.start, moreOrLessEquals(50, epsilon: 1));
      expect(endValues.end, moreOrLessEquals(90, epsilon: 1));
    },
  );

  ThemeData buildTheme() {
    return ThemeData(
      platform: TargetPlatform.android,
      primarySwatch: Colors.blue,
      sliderTheme: const SliderThemeData(
        disabledThumbColor: Color(0xff000001),
        disabledActiveTickMarkColor: Color(0xff000002),
        disabledActiveTrackColor: Color(0xff000003),
        disabledInactiveTickMarkColor: Color(0xff000004),
        disabledInactiveTrackColor: Color(0xff000005),
        activeTrackColor: Color(0xff000006),
        activeTickMarkColor: Color(0xff000007),
        inactiveTrackColor: Color(0xff000008),
        inactiveTickMarkColor: Color(0xff000009),
        overlayColor: Color(0xff000010),
        thumbColor: Color(0xff000011),
        valueIndicatorColor: Color(0xff000012),
      ),
    );
  }

  Widget buildThemedApp({
    required ThemeData theme,
    Color? activeColor,
    Color? inactiveColor,
    int? divisions,
    bool enabled = true,
  }) {
    RangeValues values = const RangeValues(0.5, 0.75);
    final ValueChanged<RangeValues>? onChanged =
        !enabled
            ? null
            : (RangeValues newValues) {
              values = newValues;
            };
    return MaterialApp(
      home: Directionality(
        textDirection: TextDirection.ltr,
        child: Material(
          child: Center(
            child: Theme(
              data: theme,
              child: RangeSlider(
                values: values,
                labels: RangeLabels(values.start.toStringAsFixed(2), values.end.toStringAsFixed(2)),
                divisions: divisions,
                activeColor: activeColor,
                inactiveColor: inactiveColor,
                onChanged: onChanged,
              ),
            ),
          ),
        ),
      ),
    );
  }

  testWidgets(
    'Range Slider uses the right theme colors for the right shapes for a default enabled slider',
    (WidgetTester tester) async {
      final ThemeData theme = buildTheme();
      final SliderThemeData sliderTheme = theme.sliderTheme;

      await tester.pumpWidget(buildThemedApp(theme: theme));

      final RenderBox sliderBox = tester.firstRenderObject<RenderBox>(find.byType(RangeSlider));

      // Check default theme for enabled widget.
      expect(
        sliderBox,
        paints
          ..rrect(color: sliderTheme.inactiveTrackColor)
          ..rrect(color: sliderTheme.inactiveTrackColor)
          ..rrect(color: sliderTheme.activeTrackColor),
      );
      expect(
        sliderBox,
        paints
          ..circle(color: sliderTheme.thumbColor)
          ..circle(color: sliderTheme.thumbColor),
      );
      expect(sliderBox, isNot(paints..circle(color: sliderTheme.disabledThumbColor)));
      expect(sliderBox, isNot(paints..rect(color: sliderTheme.disabledActiveTrackColor)));
      expect(sliderBox, isNot(paints..rect(color: sliderTheme.disabledInactiveTrackColor)));
      expect(sliderBox, isNot(paints..circle(color: sliderTheme.activeTickMarkColor)));
      expect(sliderBox, isNot(paints..circle(color: sliderTheme.inactiveTickMarkColor)));
    },
  );

  testWidgets(
    'Range Slider uses the right theme colors for the right shapes when setting the active color',
    (WidgetTester tester) async {
      const Color activeColor = Color(0xcafefeed);
      final ThemeData theme = buildTheme();
      final SliderThemeData sliderTheme = theme.sliderTheme;

      await tester.pumpWidget(buildThemedApp(theme: theme, activeColor: activeColor));

      final RenderBox sliderBox = tester.firstRenderObject<RenderBox>(find.byType(RangeSlider));

      expect(
        sliderBox,
        paints
          ..rrect(color: sliderTheme.inactiveTrackColor)
          ..rrect(color: sliderTheme.inactiveTrackColor)
          ..rrect(color: activeColor),
      );
      expect(
        sliderBox,
        paints
          ..circle(color: activeColor)
          ..circle(color: activeColor),
      );
      expect(sliderBox, isNot(paints..circle(color: sliderTheme.thumbColor)));
      expect(sliderBox, isNot(paints..circle(color: sliderTheme.disabledThumbColor)));
      expect(sliderBox, isNot(paints..rect(color: sliderTheme.disabledActiveTrackColor)));
      expect(sliderBox, isNot(paints..rect(color: sliderTheme.disabledInactiveTrackColor)));
    },
  );

  testWidgets(
    'Range Slider uses the right theme colors for the right shapes when setting the inactive color',
    (WidgetTester tester) async {
      const Color inactiveColor = Color(0xdeadbeef);
      final ThemeData theme = buildTheme();
      final SliderThemeData sliderTheme = theme.sliderTheme;

      await tester.pumpWidget(buildThemedApp(theme: theme, inactiveColor: inactiveColor));

      final RenderBox sliderBox = tester.firstRenderObject<RenderBox>(find.byType(RangeSlider));

      expect(
        sliderBox,
        paints
          ..rrect(color: inactiveColor)
          ..rrect(color: inactiveColor)
          ..rrect(color: sliderTheme.activeTrackColor),
      );
      expect(
        sliderBox,
        paints
          ..circle(color: sliderTheme.thumbColor)
          ..circle(color: sliderTheme.thumbColor),
      );
      expect(sliderBox, isNot(paints..circle(color: sliderTheme.disabledThumbColor)));
      expect(sliderBox, isNot(paints..rect(color: sliderTheme.disabledActiveTrackColor)));
      expect(sliderBox, isNot(paints..rect(color: sliderTheme.disabledInactiveTrackColor)));
    },
  );

  testWidgets(
    'Range Slider uses the right theme colors for the right shapes with active and inactive colors',
    (WidgetTester tester) async {
      const Color activeColor = Color(0xcafefeed);
      const Color inactiveColor = Color(0xdeadbeef);
      final ThemeData theme = buildTheme();
      final SliderThemeData sliderTheme = theme.sliderTheme;

      await tester.pumpWidget(
        buildThemedApp(theme: theme, activeColor: activeColor, inactiveColor: inactiveColor),
      );

      final RenderBox sliderBox = tester.firstRenderObject<RenderBox>(find.byType(RangeSlider));

      expect(
        sliderBox,
        paints
          ..rrect(color: inactiveColor)
          ..rrect(color: inactiveColor)
          ..rrect(color: activeColor),
      );
      expect(
        sliderBox,
        paints
          ..circle(color: activeColor)
          ..circle(color: activeColor),
      );
      expect(sliderBox, isNot(paints..circle(color: sliderTheme.thumbColor)));
      expect(sliderBox, isNot(paints..circle(color: sliderTheme.disabledThumbColor)));
      expect(sliderBox, isNot(paints..rect(color: sliderTheme.disabledActiveTrackColor)));
      expect(sliderBox, isNot(paints..rect(color: sliderTheme.disabledInactiveTrackColor)));
    },
  );

  testWidgets(
    'Range Slider uses the right theme colors for the right shapes for a discrete slider',
    (WidgetTester tester) async {
      final ThemeData theme = buildTheme();
      final SliderThemeData sliderTheme = theme.sliderTheme;

      await tester.pumpWidget(buildThemedApp(theme: theme, divisions: 3));

      final RenderBox sliderBox = tester.firstRenderObject<RenderBox>(find.byType(RangeSlider));

      expect(
        sliderBox,
        paints
          ..rrect(color: sliderTheme.inactiveTrackColor)
          ..rrect(color: sliderTheme.inactiveTrackColor)
          ..rrect(color: sliderTheme.activeTrackColor),
      );
      expect(
        sliderBox,
        paints
          ..circle(color: sliderTheme.inactiveTickMarkColor)
          ..circle(color: sliderTheme.inactiveTickMarkColor)
          ..circle(color: sliderTheme.activeTickMarkColor)
          ..circle(color: sliderTheme.inactiveTickMarkColor)
          ..circle(color: sliderTheme.thumbColor)
          ..circle(color: sliderTheme.thumbColor),
      );
      expect(sliderBox, isNot(paints..circle(color: sliderTheme.disabledThumbColor)));
      expect(sliderBox, isNot(paints..rect(color: sliderTheme.disabledActiveTrackColor)));
      expect(sliderBox, isNot(paints..rect(color: sliderTheme.disabledInactiveTrackColor)));
    },
  );

  testWidgets(
    'Range Slider uses the right theme colors for the right shapes for a discrete slider with active and inactive colors',
    (WidgetTester tester) async {
      const Color activeColor = Color(0xcafefeed);
      const Color inactiveColor = Color(0xdeadbeef);
      final ThemeData theme = buildTheme();
      final SliderThemeData sliderTheme = theme.sliderTheme;

      await tester.pumpWidget(
        buildThemedApp(
          theme: theme,
          activeColor: activeColor,
          inactiveColor: inactiveColor,
          divisions: 3,
        ),
      );

      final RenderBox sliderBox = tester.firstRenderObject<RenderBox>(find.byType(RangeSlider));

      expect(
        sliderBox,
        paints
          ..rrect(color: inactiveColor)
          ..rrect(color: inactiveColor)
          ..rrect(color: activeColor),
      );
      expect(
        sliderBox,
        paints
          ..circle(color: activeColor)
          ..circle(color: activeColor)
          ..circle(color: inactiveColor)
          ..circle(color: activeColor)
          ..circle(color: activeColor)
          ..circle(color: activeColor),
      );
      expect(sliderBox, isNot(paints..circle(color: sliderTheme.thumbColor)));
      expect(sliderBox, isNot(paints..circle(color: sliderTheme.disabledThumbColor)));
      expect(sliderBox, isNot(paints..rect(color: sliderTheme.disabledActiveTrackColor)));
      expect(sliderBox, isNot(paints..rect(color: sliderTheme.disabledInactiveTrackColor)));
      expect(sliderBox, isNot(paints..circle(color: sliderTheme.activeTickMarkColor)));
      expect(sliderBox, isNot(paints..circle(color: sliderTheme.inactiveTickMarkColor)));
    },
  );

  testWidgets(
    'Range Slider uses the right theme colors for the right shapes for a default disabled slider',
    (WidgetTester tester) async {
      final ThemeData theme = buildTheme();
      final SliderThemeData sliderTheme = theme.sliderTheme;

      await tester.pumpWidget(buildThemedApp(theme: theme, enabled: false));

      final RenderBox sliderBox = tester.firstRenderObject<RenderBox>(find.byType(RangeSlider));

      expect(
        sliderBox,
        paints
          ..rrect(color: sliderTheme.disabledInactiveTrackColor)
          ..rrect(color: sliderTheme.disabledInactiveTrackColor)
          ..rrect(color: sliderTheme.disabledActiveTrackColor),
      );
      expect(sliderBox, isNot(paints..circle(color: sliderTheme.thumbColor)));
      expect(sliderBox, isNot(paints..rect(color: sliderTheme.activeTrackColor)));
      expect(sliderBox, isNot(paints..rect(color: sliderTheme.inactiveTrackColor)));
    },
  );

  testWidgets(
    'Range Slider uses the right theme colors for the right shapes for a disabled slider with active and inactive colors',
    (WidgetTester tester) async {
      const Color activeColor = Color(0xcafefeed);
      const Color inactiveColor = Color(0xdeadbeef);
      final ThemeData theme = buildTheme();
      final SliderThemeData sliderTheme = theme.sliderTheme;

      await tester.pumpWidget(
        buildThemedApp(
          theme: theme,
          activeColor: activeColor,
          inactiveColor: inactiveColor,
          enabled: false,
        ),
      );

      final RenderBox sliderBox = tester.firstRenderObject<RenderBox>(find.byType(RangeSlider));

      expect(
        sliderBox,
        paints
          ..rrect(color: sliderTheme.disabledInactiveTrackColor)
          ..rrect(color: sliderTheme.disabledInactiveTrackColor)
          ..rrect(color: sliderTheme.disabledActiveTrackColor),
      );
      expect(sliderBox, isNot(paints..circle(color: sliderTheme.thumbColor)));
      expect(sliderBox, isNot(paints..rect(color: sliderTheme.activeTrackColor)));
      expect(sliderBox, isNot(paints..rect(color: sliderTheme.inactiveTrackColor)));
    },
  );

  testWidgets(
    'Range Slider uses the right theme colors for the right shapes when the value indicators are showing',
    (WidgetTester tester) async {
      final ThemeData theme = buildTheme();
      final SliderThemeData sliderTheme = theme.sliderTheme;
      RangeValues values = const RangeValues(0.5, 0.75);

      Widget buildApp({
        Color? activeColor,
        Color? inactiveColor,
        int? divisions,
        bool enabled = true,
      }) {
        final ValueChanged<RangeValues>? onChanged =
            !enabled
                ? null
                : (RangeValues newValues) {
                  values = newValues;
                };
        return MaterialApp(
          home: Directionality(
            textDirection: TextDirection.ltr,
            child: Material(
              child: Center(
                child: Theme(
                  data: theme,
                  child: RangeSlider(
                    values: values,
                    labels: RangeLabels(
                      values.start.toStringAsFixed(2),
                      values.end.toStringAsFixed(2),
                    ),
                    divisions: divisions,
                    activeColor: activeColor,
                    inactiveColor: inactiveColor,
                    onChanged: onChanged,
                  ),
                ),
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildApp(divisions: 3));

      final RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));

      final Offset topRight = tester.getTopRight(find.byType(RangeSlider)).translate(-24, 0);
      final TestGesture gesture = await tester.startGesture(topRight);
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();
      expect(values.end, equals(1));
      expect(
        valueIndicatorBox,
        paints
          ..path(color: Colors.black) // shadow
          ..path(color: Colors.black) // shadow
          ..path(color: sliderTheme.valueIndicatorColor)
          ..paragraph(),
      );
      await gesture.up();
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();
    },
  );

  testWidgets(
    'Range Slider removes value indicator from overlay if Slider gets disposed without value indicator animation completing.',
    (WidgetTester tester) async {
      RangeValues values = const RangeValues(0.5, 0.75);
      const Color fillColor = Color(0xf55f5f5f);

      Widget buildApp({
        Color? activeColor,
        Color? inactiveColor,
        int? divisions,
        bool enabled = true,
      }) {
        void onChanged(RangeValues newValues) {
          values = newValues;
        }

        return MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Scaffold(
            // The builder is used to pass the context from the MaterialApp widget
            // to the [Navigator]. This context is required in order for the
            // Navigator to work.
            body: Builder(
              builder: (BuildContext context) {
                return Column(
                  children: <Widget>[
                    RangeSlider(
                      values: values,
                      labels: RangeLabels(
                        values.start.toStringAsFixed(2),
                        values.end.toStringAsFixed(2),
                      ),
                      divisions: divisions,
                      onChanged: onChanged,
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
      final Offset topRight = tester.getTopRight(find.byType(RangeSlider)).translate(-24, 0);
      final TestGesture gesture = await tester.startGesture(topRight);
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();

      expect(
        valueIndicatorBox,
        paints
          // Represents the raised button wth next text.
          ..path(color: Colors.black)
          ..paragraph()
          // Represents the range slider.
          ..path(color: fillColor)
          ..paragraph()
          ..path(color: fillColor)
          ..paragraph(),
      );

      // Represents the Raised Button and Range Slider.
      expect(valueIndicatorBox, paintsExactlyCountTimes(#drawPath, 6));
      expect(valueIndicatorBox, paintsExactlyCountTimes(#drawParagraph, 3));

      await tester.tap(find.text('Next'));
      await tester.pumpAndSettle();

      expect(find.byType(RangeSlider), findsNothing);
      expect(
        valueIndicatorBox,
        isNot(
          paints
            ..path(color: fillColor)
            ..paragraph()
            ..path(color: fillColor)
            ..paragraph(),
        ),
      );

      // Represents the raised button with inner page text.
      expect(valueIndicatorBox, paintsExactlyCountTimes(#drawPath, 2));
      expect(valueIndicatorBox, paintsExactlyCountTimes(#drawParagraph, 1));

      // Don't stop holding the value indicator.
      await gesture.up();
      await tester.pumpAndSettle();
    },
  );

  testWidgets('Range Slider top thumb gets stroked when overlapping', (WidgetTester tester) async {
    RangeValues values = const RangeValues(0.3, 0.7);

    final ThemeData theme = ThemeData(
      platform: TargetPlatform.android,
      primarySwatch: Colors.blue,
      sliderTheme: const SliderThemeData(
        thumbColor: Color(0xff000001),
        overlappingShapeStrokeColor: Color(0xff000002),
      ),
    );
    final SliderThemeData sliderTheme = theme.sliderTheme;

    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: Theme(
                    data: theme,
                    child: RangeSlider(
                      values: values,
                      onChanged: (RangeValues newValues) {
                        setState(() {
                          values = newValues;
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

    final RenderBox sliderBox = tester.firstRenderObject<RenderBox>(find.byType(RangeSlider));

    // Get the bounds of the track by finding the slider edges and translating
    // inwards by the overlay radius.
    final Offset topLeft = tester.getTopLeft(find.byType(RangeSlider)).translate(24, 0);
    final Offset bottomRight = tester.getBottomRight(find.byType(RangeSlider)).translate(-24, 0);
    final Offset middle = topLeft + bottomRight / 2;

    // Drag the thumbs towards the center.
    final Offset leftTarget = topLeft + (bottomRight - topLeft) * 0.3;
    await tester.dragFrom(leftTarget, middle - leftTarget);
    await tester.pumpAndSettle();
    final Offset rightTarget = topLeft + (bottomRight - topLeft) * 0.7;
    await tester.dragFrom(rightTarget, middle - rightTarget);
    expect(values.start, moreOrLessEquals(0.5, epsilon: 0.03));
    expect(values.end, moreOrLessEquals(0.5, epsilon: 0.03));
    await tester.pumpAndSettle();

    expect(
      sliderBox,
      paints
        ..circle(color: sliderTheme.thumbColor)
        ..circle(color: sliderTheme.overlappingShapeStrokeColor)
        ..circle(color: sliderTheme.thumbColor),
    );
  });

  testWidgets('Range Slider top value indicator gets stroked when overlapping', (
    WidgetTester tester,
  ) async {
    RangeValues values = const RangeValues(0.3, 0.7);

    final ThemeData theme = ThemeData(
      platform: TargetPlatform.android,
      primarySwatch: Colors.blue,
      sliderTheme: const SliderThemeData(
        valueIndicatorColor: Color(0xff000001),
        overlappingShapeStrokeColor: Color(0xff000002),
        showValueIndicator: ShowValueIndicator.always,
      ),
    );
    final SliderThemeData sliderTheme = theme.sliderTheme;

    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: Theme(
                    data: theme,
                    child: RangeSlider(
                      values: values,
                      labels: RangeLabels(
                        values.start.toStringAsFixed(2),
                        values.end.toStringAsFixed(2),
                      ),
                      onChanged: (RangeValues newValues) {
                        setState(() {
                          values = newValues;
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

    final RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));

    // Get the bounds of the track by finding the slider edges and translating
    // inwards by the overlay radius.
    final Offset topLeft = tester.getTopLeft(find.byType(RangeSlider)).translate(24, 0);
    final Offset bottomRight = tester.getBottomRight(find.byType(RangeSlider)).translate(-24, 0);
    final Offset middle = topLeft + bottomRight / 2;

    // Drag the thumbs towards the center.
    final Offset leftTarget = topLeft + (bottomRight - topLeft) * 0.3;
    await tester.dragFrom(leftTarget, middle - leftTarget);
    await tester.pumpAndSettle();
    final Offset rightTarget = topLeft + (bottomRight - topLeft) * 0.7;
    await tester.dragFrom(rightTarget, middle - rightTarget);
    await tester.pumpAndSettle();
    expect(values.start, moreOrLessEquals(0.5, epsilon: 0.03));
    expect(values.end, moreOrLessEquals(0.5, epsilon: 0.03));
    final TestGesture gesture = await tester.startGesture(middle);
    await tester.pumpAndSettle();

    expect(
      valueIndicatorBox,
      paints
        ..path(color: Colors.black) // shadow
        ..path(color: Colors.black) // shadow
        ..path(color: sliderTheme.valueIndicatorColor)
        ..paragraph(),
    );

    await gesture.up();
  });

  testWidgets(
    'Range Slider top value indicator gets stroked when overlapping with large text scale',
    (WidgetTester tester) async {
      RangeValues values = const RangeValues(0.3, 0.7);

      final ThemeData theme = ThemeData(
        platform: TargetPlatform.android,
        primarySwatch: Colors.blue,
        sliderTheme: const SliderThemeData(
          valueIndicatorColor: Color(0xff000001),
          overlappingShapeStrokeColor: Color(0xff000002),
          showValueIndicator: ShowValueIndicator.always,
        ),
      );
      final SliderThemeData sliderTheme = theme.sliderTheme;

      await tester.pumpWidget(
        MaterialApp(
          home: Directionality(
            textDirection: TextDirection.ltr,
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return MediaQuery(
                  data: const MediaQueryData(textScaler: TextScaler.linear(2)),
                  child: Material(
                    child: Center(
                      child: Theme(
                        data: theme,
                        child: RangeSlider(
                          values: values,
                          labels: RangeLabels(
                            values.start.toStringAsFixed(2),
                            values.end.toStringAsFixed(2),
                          ),
                          onChanged: (RangeValues newValues) {
                            setState(() {
                              values = newValues;
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

      final RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));

      // Get the bounds of the track by finding the slider edges and translating
      // inwards by the overlay radius.
      final Offset topLeft = tester.getTopLeft(find.byType(RangeSlider)).translate(24, 0);
      final Offset bottomRight = tester.getBottomRight(find.byType(RangeSlider)).translate(-24, 0);
      final Offset middle = topLeft + bottomRight / 2;

      // Drag the thumbs towards the center.
      final Offset leftTarget = topLeft + (bottomRight - topLeft) * 0.3;
      await tester.dragFrom(leftTarget, middle - leftTarget);
      await tester.pumpAndSettle();
      final Offset rightTarget = topLeft + (bottomRight - topLeft) * 0.7;
      await tester.dragFrom(rightTarget, middle - rightTarget);
      await tester.pumpAndSettle();
      expect(values.start, moreOrLessEquals(0.5, epsilon: 0.03));
      expect(values.end, moreOrLessEquals(0.5, epsilon: 0.03));
      final TestGesture gesture = await tester.startGesture(middle);
      await tester.pumpAndSettle();

      expect(
        valueIndicatorBox,
        paints
          ..path(color: Colors.black) // shadow
          ..path(color: Colors.black) // shadow
          ..path(color: sliderTheme.valueIndicatorColor)
          ..paragraph(),
      );

      await gesture.up();
    },
  );

  testWidgets('Range Slider thumb gets stroked when overlapping', (WidgetTester tester) async {
    RangeValues values = const RangeValues(0.3, 0.7);

    final ThemeData theme = ThemeData(
      platform: TargetPlatform.android,
      primarySwatch: Colors.blue,
      sliderTheme: const SliderThemeData(
        valueIndicatorColor: Color(0xff000001),
        showValueIndicator: ShowValueIndicator.onlyForContinuous,
      ),
    );
    final SliderThemeData sliderTheme = theme.sliderTheme;

    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: Theme(
                    data: theme,
                    child: RangeSlider(
                      values: values,
                      labels: RangeLabels(
                        values.start.toStringAsFixed(2),
                        values.end.toStringAsFixed(2),
                      ),
                      onChanged: (RangeValues newValues) {
                        setState(() {
                          values = newValues;
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

    // Get the bounds of the track by finding the slider edges and translating
    // inwards by the overlay radius.
    final Offset topLeft = tester.getTopLeft(find.byType(RangeSlider)).translate(24, 0);
    final Offset bottomRight = tester.getBottomRight(find.byType(RangeSlider)).translate(-24, 0);
    final Offset middle = topLeft + bottomRight / 2;

    // Drag the thumbs towards the center.
    final Offset leftTarget = topLeft + (bottomRight - topLeft) * 0.3;
    await tester.dragFrom(leftTarget, middle - leftTarget);
    await tester.pumpAndSettle();
    final Offset rightTarget = topLeft + (bottomRight - topLeft) * 0.7;
    await tester.dragFrom(rightTarget, middle - rightTarget);
    await tester.pumpAndSettle();
    expect(values.start, moreOrLessEquals(0.5, epsilon: 0.03));
    expect(values.end, moreOrLessEquals(0.5, epsilon: 0.03));
    final TestGesture gesture = await tester.startGesture(middle);
    await tester.pumpAndSettle();

    /// The first circle is the thumb, the second one is the overlapping shape
    /// circle, and the last one is the second thumb.
    expect(
      find.byType(RangeSlider),
      paints
        ..circle()
        ..circle(color: sliderTheme.overlappingShapeStrokeColor)
        ..circle(),
    );

    await gesture.up();

    expect(
      find.byType(RangeSlider),
      paints
        ..circle()
        ..circle(color: sliderTheme.overlappingShapeStrokeColor)
        ..circle(),
    );
  });

  // Regression test for https://github.com/flutter/flutter/issues/101868
  testWidgets('RangeSlider.label info should not write to semantic node', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Theme(
          data: ThemeData.light(),
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Material(
              child: RangeSlider(
                values: const RangeValues(10.0, 12.0),
                max: 100.0,
                onChanged: (RangeValues v) {},
                labels: const RangeLabels('Begin', 'End'),
              ),
            ),
          ),
        ),
      ),
    );

    await tester.pumpAndSettle();

    expect(
      tester.getSemantics(find.byType(RangeSlider)),
      matchesSemantics(
        scopesRoute: true,
        children: <Matcher>[
          matchesSemantics(
            children: <Matcher>[
              matchesSemantics(
                isEnabled: true,
                isSlider: true,
                hasEnabledState: true,
                hasIncreaseAction: true,
                hasDecreaseAction: true,
                value: '10%',
                increasedValue: '10%',
                decreasedValue: '5%',
                label: '',
              ),
              matchesSemantics(
                isEnabled: true,
                isSlider: true,
                hasEnabledState: true,
                hasIncreaseAction: true,
                hasDecreaseAction: true,
                value: '12%',
                increasedValue: '17%',
                decreasedValue: '12%',
                label: '',
              ),
            ],
          ),
        ],
      ),
    );
  });

  testWidgets('Range Slider Semantics - ltr', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Theme(
          data: ThemeData.light(),
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Material(
              child: RangeSlider(
                values: const RangeValues(10.0, 30.0),
                max: 100.0,
                onChanged: (RangeValues v) {},
              ),
            ),
          ),
        ),
      ),
    );

    await tester.pumpAndSettle();

    final SemanticsNode semanticsNode = tester.getSemantics(find.byType(RangeSlider));
    expect(
      semanticsNode,
      matchesSemantics(
        scopesRoute: true,
        children: <Matcher>[
          matchesSemantics(
            children: <Matcher>[
              matchesSemantics(
                isEnabled: true,
                isSlider: true,
                hasEnabledState: true,
                hasIncreaseAction: true,
                hasDecreaseAction: true,
                value: '10%',
                increasedValue: '15%',
                decreasedValue: '5%',
                rect: const Rect.fromLTRB(75.2, 276.0, 123.2, 324.0),
              ),
              matchesSemantics(
                isEnabled: true,
                isSlider: true,
                hasEnabledState: true,
                hasIncreaseAction: true,
                hasDecreaseAction: true,
                value: '30%',
                increasedValue: '35%',
                decreasedValue: '25%',
                rect: const Rect.fromLTRB(225.6, 276.0, 273.6, 324.0),
              ),
            ],
          ),
        ],
      ),
    );

    // TODO(tahatesser): This is a workaround for matching
    // the semantics node rects by avoiding floating point errors.
    // https://github.com/flutter/flutter/issues/115079
    // Get semantics node rects.
    final List<Rect> rects = <Rect>[];
    semanticsNode.visitChildren((SemanticsNode node) {
      node.visitChildren((SemanticsNode node) {
        // Round rect values to avoid floating point errors.
        rects.add(
          Rect.fromLTRB(
            node.rect.left.roundToDouble(),
            node.rect.top.roundToDouble(),
            node.rect.right.roundToDouble(),
            node.rect.bottom.roundToDouble(),
          ),
        );
        return true;
      });
      return true;
    });
    // Test that the semantics node rect sizes are correct.
    expect(rects, <Rect>[
      const Rect.fromLTRB(75.0, 276.0, 123.0, 324.0),
      const Rect.fromLTRB(226.0, 276.0, 274.0, 324.0),
    ]);
  });

  testWidgets('Range Slider Semantics - rtl', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Theme(
          data: ThemeData.light(),
          child: Directionality(
            textDirection: TextDirection.rtl,
            child: Material(
              child: RangeSlider(
                values: const RangeValues(10.0, 30.0),
                max: 100.0,
                onChanged: (RangeValues v) {},
              ),
            ),
          ),
        ),
      ),
    );

    await tester.pumpAndSettle();

    final SemanticsNode semanticsNode = tester.getSemantics(find.byType(RangeSlider));
    expect(
      semanticsNode,
      matchesSemantics(
        scopesRoute: true,
        children: <Matcher>[
          matchesSemantics(
            children: <Matcher>[
              matchesSemantics(
                isEnabled: true,
                isSlider: true,
                hasEnabledState: true,
                hasIncreaseAction: true,
                hasDecreaseAction: true,
                value: '10%',
                increasedValue: '15%',
                decreasedValue: '5%',
              ),
              matchesSemantics(
                isEnabled: true,
                isSlider: true,
                hasEnabledState: true,
                hasIncreaseAction: true,
                hasDecreaseAction: true,
                value: '30%',
                increasedValue: '35%',
                decreasedValue: '25%',
              ),
            ],
          ),
        ],
      ),
    );

    // TODO(tahatesser): This is a workaround for matching
    // the semantics node rects by avoiding floating point errors.
    // https://github.com/flutter/flutter/issues/115079
    // Get semantics node rects.
    final List<Rect> rects = <Rect>[];
    semanticsNode.visitChildren((SemanticsNode node) {
      node.visitChildren((SemanticsNode node) {
        // Round rect values to avoid floating point errors.
        rects.add(
          Rect.fromLTRB(
            node.rect.left.roundToDouble(),
            node.rect.top.roundToDouble(),
            node.rect.right.roundToDouble(),
            node.rect.bottom.roundToDouble(),
          ),
        );
        return true;
      });
      return true;
    });
    // Test that the semantics node rect sizes are correct.
    expect(rects, <Rect>[
      const Rect.fromLTRB(526.0, 276.0, 574.0, 324.0),
      const Rect.fromLTRB(677.0, 276.0, 725.0, 324.0),
    ]);
  });

  testWidgets('Range Slider implements debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();

    RangeSlider(
      activeColor: Colors.blue,
      divisions: 4,
      inactiveColor: Colors.grey,
      labels: const RangeLabels('lowerValue', 'upperValue'),
      max: 100.0,
      onChanged: null,
      values: const RangeValues(25.0, 75.0),
    ).debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[
      'valueStart: 25.0',
      'valueEnd: 75.0',
      'disabled',
      'min: 0.0',
      'max: 100.0',
      'divisions: 4',
      'labelStart: "lowerValue"',
      'labelEnd: "upperValue"',
      'activeColor: MaterialColor(primary value: ${const Color(0xff2196f3)})',
      'inactiveColor: MaterialColor(primary value: ${const Color(0xff9e9e9e)})',
    ]);
  });

  testWidgets(
    'Range Slider can be painted in a narrower constraint when track shape is RoundedRectRange',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Directionality(
            textDirection: TextDirection.ltr,
            child: Material(
              child: Center(
                child: SizedBox(
                  height: 10.0,
                  width: 0.0,
                  child: RangeSlider(values: const RangeValues(0.25, 0.5), onChanged: null),
                ),
              ),
            ),
          ),
        ),
      );

      // _RenderRangeSlider is the last render object in the tree.
      final RenderObject renderObject = tester.allRenderObjects.last;

      expect(
        renderObject,
        paints
          // left inactive track RRect
          ..rrect(
            rrect: RRect.fromLTRBAndCorners(
              -24.0,
              3.0,
              -12.0,
              7.0,
              topLeft: const Radius.circular(2.0),
              bottomLeft: const Radius.circular(2.0),
            ),
          )
          // right inactive track RRect
          ..rrect(
            rrect: RRect.fromLTRBAndCorners(
              0.0,
              3.0,
              24.0,
              7.0,
              topRight: const Radius.circular(2.0),
              bottomRight: const Radius.circular(2.0),
            ),
          )
          // active track RRect
          ..rrect(rrect: RRect.fromLTRBR(-14.0, 2.0, 2.0, 8.0, const Radius.circular(2.0)))
          // thumbs
          ..circle(x: -12.0, y: 5.0, radius: 10.0)
          ..circle(x: 0.0, y: 5.0, radius: 10.0),
      );
    },
  );

  testWidgets(
    'Range Slider can be painted in a narrower constraint when track shape is Rectangular',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            sliderTheme: const SliderThemeData(rangeTrackShape: RectangularRangeSliderTrackShape()),
          ),
          home: Directionality(
            textDirection: TextDirection.ltr,
            child: Material(
              child: Center(
                child: SizedBox(
                  height: 10.0,
                  width: 0.0,
                  child: RangeSlider(values: const RangeValues(0.25, 0.5), onChanged: null),
                ),
              ),
            ),
          ),
        ),
      );

      // _RenderRangeSlider is the last render object in the tree.
      final RenderObject renderObject = tester.allRenderObjects.last;

      //There should no gap between the inactive track and active track.
      expect(
        renderObject,
        paints
          // left inactive track RRect
          ..rect(rect: const Rect.fromLTRB(-24.0, 3.0, -12.0, 7.0))
          // active track RRect
          ..rect(rect: const Rect.fromLTRB(-12.0, 3.0, 0.0, 7.0))
          // right inactive track RRect
          ..rect(rect: const Rect.fromLTRB(0.0, 3.0, 24.0, 7.0))
          // thumbs
          ..circle(x: -12.0, y: 5.0, radius: 10.0)
          ..circle(x: 0.0, y: 5.0, radius: 10.0),
      );
    },
  );

  testWidgets('Update the divisions and values at the same time for RangeSlider', (
    WidgetTester tester,
  ) async {
    // Regress test for https://github.com/flutter/flutter/issues/65943
    Widget buildFrame(double maxValue) {
      return MaterialApp(
        home: Material(
          child: Center(
            child: RangeSlider(
              values: const RangeValues(5, 8),
              max: maxValue,
              divisions: maxValue.toInt(),
              onChanged: (RangeValues newValue) {},
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(10));

    // _RenderRangeSlider is the last render object in the tree.
    final RenderObject renderObject = tester.allRenderObjects.last;

    // Update the divisions from 10 to 15, the thumbs should be paint at the correct position.
    await tester.pumpWidget(buildFrame(15));
    await tester.pumpAndSettle(); // Finish the animation.

    late RRect activeTrackRRect;
    expect(
      renderObject,
      paints
        ..rrect()
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
    // The 1st thumb should at one-third(5 / 15) of the Slider.
    // The 2nd thumb should at (8 / 15) of the Slider.
    // The left of the active track shape is the position of the 1st thumb.
    // The right of the active track shape is the position of the 2nd thumb.
    // 24.0 is the default margin, (800.0 - 24.0 - 24.0 - padding) is the slider's width.
    // Where the padding value equals to the track height.
    expect(
      nearEqual(activeTrackRRect.left, (800.0 - 24.0 - 24.0 - padding) * (5 / 15) + 24.0, 0.01),
      true,
    );
    expect(
      nearEqual(
        activeTrackRRect.right,
        (800.0 - 24.0 - 24.0 - padding) * (8 / 15) + 24.0 + padding,
        0.01,
      ),
      true,
    );
  });

  testWidgets('RangeSlider changes mouse cursor when hovered', (WidgetTester tester) async {
    const RangeValues values = RangeValues(50, 70);

    // Test default cursor.
    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: MouseRegion(
                cursor: SystemMouseCursors.forbidden,
                child: RangeSlider(values: values, max: 100.0, onChanged: (RangeValues values) {}),
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
    await gesture.addPointer(location: tester.getCenter(find.byType(RangeSlider)));

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );

    // Test custom cursor.
    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: MouseRegion(
                cursor: SystemMouseCursors.forbidden,
                child: RangeSlider(
                  values: values,
                  max: 100.0,
                  mouseCursor: const MaterialStatePropertyAll<MouseCursor?>(
                    SystemMouseCursors.text,
                  ),
                  onChanged: (RangeValues values) {},
                ),
              ),
            ),
          ),
        ),
      ),
    );

    await tester.pump();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );
  });

  testWidgets('RangeSlider MaterialStateMouseCursor resolves correctly', (
    WidgetTester tester,
  ) async {
    RangeValues values = const RangeValues(50, 70);
    const MouseCursor disabledCursor = SystemMouseCursors.basic;
    const MouseCursor hoveredCursor = SystemMouseCursors.grab;
    const MouseCursor draggedCursor = SystemMouseCursors.move;

    Widget buildFrame({required bool enabled}) {
      return MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Center(
                  child: MouseRegion(
                    cursor: SystemMouseCursors.forbidden,
                    child: RangeSlider(
                      mouseCursor: MaterialStateProperty.resolveWith<MouseCursor?>((
                        Set<MaterialState> states,
                      ) {
                        if (states.contains(MaterialState.disabled)) {
                          return disabledCursor;
                        }
                        if (states.contains(MaterialState.dragged)) {
                          return draggedCursor;
                        }
                        if (states.contains(MaterialState.hovered)) {
                          return hoveredCursor;
                        }

                        return SystemMouseCursors.none;
                      }),
                      values: values,
                      max: 100.0,
                      onChanged:
                          enabled
                              ? (RangeValues newValues) {
                                setState(() {
                                  values = newValues;
                                });
                              }
                              : null,
                      onChangeStart: enabled ? (RangeValues newValues) {} : null,
                      onChangeEnd: enabled ? (RangeValues newValues) {} : null,
                    ),
                  ),
                );
              },
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

    await gesture.moveTo(tester.getCenter(find.byType(RangeSlider))); // start hover
    await tester.pumpAndSettle();
    expect(RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1), hoveredCursor);

    await tester.timedDrag(
      find.byType(RangeSlider),
      const Offset(20.0, 0.0),
      const Duration(milliseconds: 100),
    );
    expect(RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1), draggedCursor);
  });

  testWidgets('RangeSlider can be hovered and has correct hover color', (
    WidgetTester tester,
  ) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    RangeValues values = const RangeValues(50, 70);
    final ThemeData theme = ThemeData();

    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: RangeSlider(
                    values: values,
                    max: 100.0,
                    onChanged:
                        enabled
                            ? (RangeValues newValues) {
                              setState(() {
                                values = newValues;
                              });
                            }
                            : null,
                  ),
                ),
              );
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildApp());

    // RangeSlider does not have overlay when enabled and not hovered.
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(RangeSlider))),
      isNot(paints..circle(color: theme.colorScheme.primary.withOpacity(0.12))),
    );

    // Start hovering.
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byType(RangeSlider)));

    // RangeSlider has overlay when enabled and hovered.
    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(RangeSlider))),
      paints..circle(color: theme.colorScheme.primary.withOpacity(0.12)),
    );

    // RangeSlider does not have an overlay when disabled and hovered.
    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(RangeSlider))),
      isNot(paints..circle(color: theme.colorScheme.primary.withOpacity(0.12))),
    );
  });

  testWidgets('RangeSlider is draggable and has correct dragged color', (
    WidgetTester tester,
  ) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    RangeValues values = const RangeValues(50, 70);
    final ThemeData theme = ThemeData();

    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: RangeSlider(
                    values: values,
                    max: 100.0,
                    onChanged:
                        enabled
                            ? (RangeValues newValues) {
                              setState(() {
                                values = newValues;
                              });
                            }
                            : null,
                  ),
                ),
              );
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildApp());

    // RangeSlider does not have overlay when enabled and not dragged.
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(RangeSlider))),
      isNot(paints..circle(color: theme.colorScheme.primary.withOpacity(0.12))),
    );

    // Start dragging.
    final TestGesture drag = await tester.startGesture(tester.getCenter(find.byType(RangeSlider)));
    await tester.pump(kPressTimeout);

    // Less than configured touch slop, more than default touch slop
    await drag.moveBy(const Offset(19.0, 0));
    await tester.pump();

    // RangeSlider has overlay when enabled and dragged.
    expect(
      Material.of(tester.element(find.byType(RangeSlider))),
      paints..circle(color: theme.colorScheme.primary.withOpacity(0.12)),
    );
  });

  testWidgets('RangeSlider overlayColor supports hovered and dragged states', (
    WidgetTester tester,
  ) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    RangeValues values = const RangeValues(50, 70);
    const Color hoverColor = Color(0xffff0000);
    const Color draggedColor = Color(0xff0000ff);

    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Material(
                child: Center(
                  child: RangeSlider(
                    values: values,
                    max: 100.0,
                    overlayColor: MaterialStateProperty.resolveWith<Color?>((
                      Set<MaterialState> states,
                    ) {
                      if (states.contains(MaterialState.hovered)) {
                        return hoverColor;
                      }
                      if (states.contains(MaterialState.dragged)) {
                        return draggedColor;
                      }

                      return null;
                    }),
                    onChanged:
                        enabled
                            ? (RangeValues newValues) {
                              setState(() {
                                values = newValues;
                              });
                            }
                            : null,
                    onChangeStart: enabled ? (RangeValues newValues) {} : null,
                    onChangeEnd: enabled ? (RangeValues newValues) {} : null,
                  ),
                ),
              );
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildApp());

    // RangeSlider does not have overlay when enabled and not hovered.
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(RangeSlider))),
      isNot(paints..circle(color: hoverColor)),
    );

    // Hover on the range slider but outside the thumb.
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getTopLeft(find.byType(RangeSlider)));

    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(RangeSlider))),
      isNot(paints..circle(color: hoverColor)),
    );

    // Hover on the thumb.
    await gesture.moveTo(tester.getCenter(find.byType(RangeSlider)));
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(RangeSlider))),
      paints..circle(color: hoverColor),
    );

    // Hover on the slider but outside the thumb.
    await gesture.moveTo(tester.getBottomRight(find.byType(RangeSlider)));
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(RangeSlider))),
      isNot(paints..circle(color: hoverColor)),
    );

    // Reset range slider values.
    values = const RangeValues(50, 70);

    // RangeSlider does not have overlay when enabled and not dragged.
    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(RangeSlider))),
      isNot(paints..circle(color: draggedColor)),
    );

    // Start dragging.
    final TestGesture drag = await tester.startGesture(tester.getCenter(find.byType(RangeSlider)));
    await tester.pump(kPressTimeout);

    // Less than configured touch slop, more than default touch slop.
    await drag.moveBy(const Offset(19.0, 0));
    await tester.pump();

    // RangeSlider has overlay when enabled and dragged.
    expect(
      Material.of(tester.element(find.byType(RangeSlider))),
      paints..circle(color: draggedColor),
    );

    // Stop dragging.
    await drag.up();
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(RangeSlider))),
      isNot(paints..circle(color: draggedColor)),
    );
  });

  testWidgets('RangeSlider onChangeStart and onChangeEnd fire once', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/128433

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
                child: RangeSlider(
                  values: const RangeValues(40, 80),
                  max: 100,
                  onChanged: (RangeValues newValue) {},
                  onChangeStart: (RangeValues value) {
                    startFired += 1;
                  },
                  onChangeEnd: (RangeValues value) {
                    endFired += 1;
                  },
                ),
              ),
            ),
          ),
        ),
      ),
    );

    await tester.timedDragFrom(
      tester.getTopLeft(find.byType(RangeSlider)),
      const Offset(100.0, 0.0),
      const Duration(milliseconds: 500),
    );

    expect(startFired, equals(1));
    expect(endFired, equals(1));
  });

  testWidgets('RangeSlider in a ListView does not throw an exception', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/126648

    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: ListView(
              children: <Widget>[
                const SizedBox(height: 600, child: Placeholder()),
                RangeSlider(
                  values: const RangeValues(40, 80),
                  max: 100,
                  onChanged: (RangeValues newValue) {},
                ),
              ],
            ),
          ),
        ),
      ),
    );

    // No exception should be thrown.
    expect(tester.takeException(), null);
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/141953.
  testWidgets('Semantic nodes do not throw an error after clearSemantics', (
    WidgetTester tester,
  ) async {
    SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Material(
          child: RangeSlider(
            values: const RangeValues(40, 80),
            max: 100,
            onChanged: (RangeValues newValue) {},
          ),
        ),
      ),
    );

    // Dispose the semantics to trigger clearSemantics.
    semantics.dispose();
    await tester.pumpAndSettle();

    expect(tester.takeException(), isNull);

    // Initialize the semantics again.
    semantics = SemanticsTester(tester);
    await tester.pumpAndSettle();

    expect(tester.takeException(), isNull);

    semantics.dispose();
  }, semanticsEnabled: false);
}
