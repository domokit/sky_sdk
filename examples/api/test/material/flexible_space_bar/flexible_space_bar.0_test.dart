// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/flexible_space_bar/flexible_space_bar.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  // The app being tested loads images via HTTP which the test
  // framework defeats by default.
  setUpAll(() {
    HttpOverrides.global = null;
  });

  testWidgets('The app bar stretches when over-scrolled', (WidgetTester tester) async {
    await tester.pumpWidget(const example.FlexibleSpaceBarExampleApp());

    expect(find.text('Flight Report'), findsOne);

    expect(find.widgetWithText(ListTile, 'Sunday'), findsOne);
    expect(find.widgetWithText(ListTile, 'Monday'), findsOne);
    expect(find.text('sunny, h: 80, l: 65'), findsExactly(2));
    expect(find.byIcon(Icons.wb_sunny), findsExactly(2));

    final Finder appBarContainer = find.byType(Image);
    final Size sizeBeforeScroll = tester.getSize(appBarContainer);
    final Offset target = tester.getCenter(find.byType(ListTile).first);
    final TestGesture gesture = await tester.startGesture(target);
    await gesture.moveBy(const Offset(0.0, 100.0));
    await tester.pump(const Duration(milliseconds: 10));
    await gesture.up();
    final Size sizeAfterScroll = tester.getSize(appBarContainer);

    expect(sizeBeforeScroll.height, lessThan(sizeAfterScroll.height));
    // Verifies ScrollBehavior.dragDevices is correctly set.
  }, variant: TargetPlatformVariant.all());
}
