// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/slider/slider.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Sliders can change their value', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SliderExampleApp());

    expect(find.byType(Slider), findsNWidgets(2));

    Finder sliderFinder = find.byType(Slider).first;
    Slider slider = tester.widget<Slider>(sliderFinder);
    expect(slider.value, equals(20));

    await tester.tapAt(tester.getCenter(sliderFinder));
    await tester.pump();

    slider = tester.widget(sliderFinder);
    expect(slider.value, equals(50));

    sliderFinder = find.byType(Slider).last;
    slider = tester.widget(sliderFinder);
    expect(slider.value, equals(60));

    await tester.tapAt(tester.getTopLeft(sliderFinder));
    await tester.pump();

    slider = tester.widget(sliderFinder);
    expect(slider.value, equals(0));
  });

  testWidgets('Sliders year2023 flag can be toggled', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SliderExampleApp());

    Slider slider = tester.widget<Slider>(find.byType(Slider).first);
    expect(slider.year2023, true);
    Slider discreteSlider = tester.widget<Slider>(find.byType(Slider).last);
    expect(discreteSlider.year2023, true);

    await tester.tap(find.byType(SwitchListTile));
    await tester.pumpAndSettle();

    slider = tester.widget<Slider>(find.byType(Slider).first);
    expect(slider.year2023, false);
    discreteSlider = tester.widget<Slider>(find.byType(Slider).last);
    expect(discreteSlider.year2023, false);
  });
}
