// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/slider/slider.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Slider shows secondary track', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SliderApp());

    expect(find.byType(Slider), findsNWidgets(2));

    final Finder slider1Finder = find.byType(Slider).at(0);
    final Finder slider2Finder = find.byType(Slider).at(1);

    Slider slider1 = tester.widget(slider1Finder);
    Slider slider2 = tester.widget(slider2Finder);
    expect(slider1.secondaryTrackValue, slider2.value);

    const double targetValue = 0.8;
    final Rect rect = tester.getRect(slider2Finder);
    final Offset target = Offset(
      rect.left + (rect.right - rect.left) * targetValue,
      rect.top + (rect.bottom - rect.top) / 2,
    );
    await tester.tapAt(target);
    await tester.pump();

    slider1 = tester.widget(slider1Finder);
    slider2 = tester.widget(slider2Finder);
    expect(slider1.secondaryTrackValue, closeTo(targetValue, 0.05));
    expect(slider1.secondaryTrackValue, slider2.value);
  });
}
