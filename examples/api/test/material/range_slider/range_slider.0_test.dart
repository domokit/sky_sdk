// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/range_slider/range_slider.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('The range slider should have 5 divisions from 0 to 100', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.RangeSliderExampleApp());

    expect(find.widgetWithText(AppBar, 'RangeSlider Sample'), findsOne);
    expect(
      find.byWidgetPredicate(
        (Widget widget) => widget is RangeSlider && widget.values == const RangeValues(40, 80),
      ),
      findsOne,
    );

    final Rect rangeSliderRect = tester.getRect(find.byType(RangeSlider));

    final double y = rangeSliderRect.centerRight.dy;
    final double startX = rangeSliderRect.centerLeft.dx;
    final double endX = rangeSliderRect.centerRight.dx;

    // Drag the start to 0.
    final TestGesture drag = await tester.startGesture(Offset(startX + (endX - startX) * 0.4, y));
    await tester.pump(kPressTimeout);
    await drag.moveTo(rangeSliderRect.centerLeft);
    await drag.up();
    await tester.pump();

    expect(
      find.byWidgetPredicate(
        (Widget widget) => widget is RangeSlider && widget.values == const RangeValues(0, 80),
      ),
      findsOne,
    );

    // Drag the start to 20.
    await drag.down(rangeSliderRect.centerLeft);
    await tester.pump(kPressTimeout);
    await drag.moveTo(Offset(startX + (endX - startX) * 0.2, y));
    await drag.up();
    await tester.pump();

    expect(
      find.byWidgetPredicate(
        (Widget widget) => widget is RangeSlider && widget.values == const RangeValues(20, 80),
      ),
      findsOne,
    );

    // Drag the end to 60.
    await drag.down(Offset(startX + (endX - startX) * 0.8, y));
    await tester.pump(kPressTimeout);
    await drag.moveTo(Offset(startX + (endX - startX) * 0.6, y));
    await drag.up();
    await tester.pump();

    expect(
      find.byWidgetPredicate(
        (Widget widget) => widget is RangeSlider && widget.values == const RangeValues(20, 60),
      ),
      findsOne,
    );

    // Drag the end to 100.
    await drag.down(Offset(startX + (endX - startX) * 0.6, y));
    await tester.pump(kPressTimeout);
    await drag.moveTo(rangeSliderRect.centerRight);
    await drag.up();
    await tester.pump();

    expect(
      find.byWidgetPredicate(
        (Widget widget) => widget is RangeSlider && widget.values == const RangeValues(20, 100),
      ),
      findsOne,
    );
  });
}
