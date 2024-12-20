// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/slider/cupertino_slider.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  Future<void> dragSlider(WidgetTester tester, Key sliderKey) {
    final Offset topLeft = tester.getTopLeft(find.byKey(sliderKey));
    const double unit = CupertinoThumbPainter.radius;
    const double delta = 3.0 * unit;
    return tester.dragFrom(topLeft + const Offset(unit, unit), const Offset(delta, 0.0));
  }

  testWidgets('Can change value using CupertinoSlider', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CupertinoSliderApp());

    // Check for the initial slider value.
    expect(find.text('0.0'), findsOneWidget);

    await dragSlider(tester, const Key('slider'));
    await tester.pumpAndSettle();

    // Check for the updated slider value.
    expect(find.text('40.0'), findsOneWidget);
  });
}
