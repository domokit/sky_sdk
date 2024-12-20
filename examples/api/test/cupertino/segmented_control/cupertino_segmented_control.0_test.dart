// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/segmented_control/cupertino_segmented_control.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Verify initial state', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SegmentedControlApp());

    // Midnight is the default selected segment.
    expect(find.text('Selected Segment: midnight'), findsOneWidget);

    // All segments are enabled and can be selected.
    await tester.tap(find.text('Viridian'));
    await tester.pumpAndSettle();
    expect(find.text('Selected Segment: viridian'), findsOneWidget);

    await tester.tap(find.text('Cerulean'));
    await tester.pumpAndSettle();
    expect(find.text('Selected Segment: cerulean'), findsOneWidget);

    await tester.tap(find.text('Midnight'));
    await tester.pumpAndSettle();
    expect(find.text('Selected Segment: midnight'), findsOneWidget);

    // Verify that the first CupertinoSwitch is off.
    final Finder firstSwitchFinder = find.byType(CupertinoSwitch).first;
    final CupertinoSwitch firstSwitch = tester.widget<CupertinoSwitch>(firstSwitchFinder);
    expect(firstSwitch.value, false);

    // Verify that the second CupertinoSwitch is on.
    final Finder secondSwitchFinder = find.byType(CupertinoSwitch).last;
    final CupertinoSwitch secondSwitch = tester.widget<CupertinoSwitch>(secondSwitchFinder);
    expect(secondSwitch.value, true);
  });

  testWidgets('Can change a selected segmented control', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SegmentedControlApp());

    expect(find.text('Selected Segment: midnight'), findsOneWidget);

    await tester.tap(find.text('Cerulean'));
    await tester.pumpAndSettle();

    expect(find.text('Selected Segment: cerulean'), findsOneWidget);
  });

  testWidgets('Can not select on a disabled segment', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SegmentedControlApp());

    // Toggle on the first CupertinoSwitch to disable the first segment.
    final Finder firstSwitchFinder = find.byType(CupertinoSwitch).first;
    await tester.tap(firstSwitchFinder);
    await tester.pumpAndSettle();
    final CupertinoSwitch firstSwitch = tester.widget<CupertinoSwitch>(firstSwitchFinder);
    expect(firstSwitch.value, true);

    // Tap on the second segment then tap back on the first segment.
    // Verify that the selected segment is still the second segment.
    await tester.tap(find.text('Viridian'));
    await tester.pumpAndSettle();
    expect(find.text('Selected Segment: viridian'), findsOneWidget);

    await tester.tap(find.text('Midnight'));
    await tester.pumpAndSettle();
    expect(find.text('Selected Segment: viridian'), findsOneWidget);
  });

  testWidgets('Can not select on all disabled segments', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SegmentedControlApp());

    // Toggle off the second CupertinoSwitch to disable all segments.
    final Finder secondSwitchFinder = find.byType(CupertinoSwitch).last;
    await tester.tap(secondSwitchFinder);
    await tester.pumpAndSettle();
    final CupertinoSwitch secondSwitch = tester.widget<CupertinoSwitch>(secondSwitchFinder);
    expect(secondSwitch.value, false);

    // Tap on the second segment and verify that the selected segment is still the first segment.
    await tester.tap(find.text('Viridian'));
    await tester.pumpAndSettle();
    expect(find.text('Selected Segment: midnight'), findsOneWidget);

    // Tap on the third segment and verify that the selected segment is still the first segment.
    await tester.tap(find.text('Cerulean'));
    await tester.pumpAndSettle();
    expect(find.text('Selected Segment: midnight'), findsOneWidget);
  });
}
