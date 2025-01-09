// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/carousel/carousel.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  // The app being tested loads images via HTTP which the test
  // framework defeats by default.
  setUpAll(() {
    HttpOverrides.global = null;
  });

  testWidgets('Carousel Smoke Test', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CarouselExampleApp());

    expect(find.widgetWithText(example.HeroLayoutCard, 'Through the Pane'), findsOneWidget);
    final Finder firstCarousel = find.byType(CarouselView).first;
    await tester.drag(firstCarousel, const Offset(150, 0));
    await tester.pumpAndSettle();
    expect(find.widgetWithText(example.HeroLayoutCard, 'The Flow'), findsOneWidget);

    await tester.drag(firstCarousel, const Offset(0, -200));
    await tester.pumpAndSettle();

    expect(find.widgetWithText(CarouselView, 'Cameras'), findsOneWidget);
    expect(find.widgetWithText(CarouselView, 'Lighting'), findsOneWidget);
    expect(find.widgetWithText(CarouselView, 'Climate'), findsOneWidget);
    expect(find.widgetWithText(CarouselView, 'Wifi'), findsOneWidget);

    await tester.drag(find.widgetWithText(CarouselView, 'Cameras'), const Offset(0, -200));
    await tester.pumpAndSettle();

    expect(find.text('Uncontained layout'), findsOneWidget);
    expect(find.widgetWithText(CarouselView, 'Show 0'), findsOneWidget);
    expect(find.widgetWithText(CarouselView, 'Show 1'), findsOneWidget);
  });
}
