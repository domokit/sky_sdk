// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:display_cutout_rotation/main.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:integration_test/integration_test.dart';

void main() {
  IntegrationTestWidgetsFlutterBinding.ensureInitialized();

  group('end-to-end test', () {
    // Test assumes that the device already has enabled
    // "com.android.internal.display.cutout.emulation.tall".
    testWidgets('cutout should be on top in portrait mode', (WidgetTester tester) async {
      // Force rotation
      await setOrientationAndWaitUntilRotation(tester, DeviceOrientation.portraitUp);
      // Load app widget.
      await tester.pumpWidget(const MyApp());
      final BuildContext context = tester.element(find.byType(Text));
      final Iterable<DisplayFeature> displayFeatures = getCutouts(tester, context);
      // Test is expecting one cutout setup in the test harness.
      expect(displayFeatures.length, 1, reason: 'Single cutout display feature expected');
      // Verify that app code thinks there is a top cutout.
      expect(
        displayFeatures.first.bounds.top,
        0,
        reason:
            'cutout should start at the top, does the test device have a '
            'camera cutout or window inset?',
      );
    });

    testWidgets('cutout should be on left in landscape left', (WidgetTester tester) async {
      await setOrientationAndWaitUntilRotation(tester, DeviceOrientation.landscapeLeft);
      // Load app widget.
      await tester.pumpWidget(const MyApp());
      final BuildContext context = tester.element(find.byType(Text));
      // Verify that app code thinks there is a left cutout.
      final Iterable<DisplayFeature> displayFeatures = getCutouts(tester, context);

      // Test is expecting one cutout setup in the test harness.
      expect(displayFeatures.length, 1, reason: 'Single cutout display feature expected');
      expect(
        displayFeatures.first.bounds.left,
        0,
        reason:
            'cutout should start at the left, does the test device have a '
            'camera cutout or window inset?',
      );
    });

    testWidgets('cutout handles rotation', (WidgetTester tester) async {
      await setOrientationAndWaitUntilRotation(tester, DeviceOrientation.portraitUp);
      const MyApp widgetUnderTest = MyApp();
      // Load app widget.
      await tester.pumpWidget(widgetUnderTest);
      BuildContext context = tester.element(find.byType(Text));
      Iterable<DisplayFeature> displayFeatures = getCutouts(tester, context);
      // Test is expecting one cutout setup in the test harness.
      expect(displayFeatures.length, 1, reason: 'Single cutout display feature expected');
      // Verify that app code thinks there is a top cutout.
      expect(
        displayFeatures.first.bounds.top,
        0,
        reason:
            'cutout should start at the top, does the test device have a '
            'camera cutout or window inset?',
      );
      await setOrientationAndWaitUntilRotation(tester, DeviceOrientation.landscapeLeft);
      await tester.pumpWidget(widgetUnderTest);

      // Requery for display features after rotation.
      context = tester.element(find.byType(Text));
      displayFeatures = getCutouts(tester, context);
      // Test is expecting one cutout setup in the test harness.
      expect(displayFeatures.length, 1, reason: 'Single cutout display feature expected');
      expect(
        displayFeatures.first.bounds.left,
        0,
        reason: 'cutout should start at the left or handle camera',
      );
    });

    tearDown(() {
      // After each test reset to device perfered orientations to avoid
      // test pollution.
      SystemChrome.setPreferredOrientations(<DeviceOrientation>[]);
    });
  });
}

/*
 * Force rotation then poll to ensure rotation has happened.
 *
 * Rotations have an async communication to engine which then has an async
 * communication to the android operating system.
 */
Future<void> setOrientationAndWaitUntilRotation(
  WidgetTester tester,
  DeviceOrientation orientation,
) async {
  SystemChrome.setPreferredOrientations(<DeviceOrientation>[orientation]);
  Orientation expectedOrientation;
  switch (orientation) {
    case DeviceOrientation.portraitUp:
    case DeviceOrientation.portraitDown:
      expectedOrientation = Orientation.portrait;
    case DeviceOrientation.landscapeRight:
    case DeviceOrientation.landscapeLeft:
      expectedOrientation = Orientation.landscape;
  }
  while (true) {
    final BuildContext context = tester.element(find.byType(Text));
    if (expectedOrientation == MediaQuery.of(context).orientation) {
      break;
    }
    await tester.pumpAndSettle();
  }
}

Iterable<DisplayFeature> getCutouts(WidgetTester tester, BuildContext context) {
  final List<DisplayFeature> displayFeatures = MediaQuery.of(context).displayFeatures;
  return displayFeatures.where(
    (DisplayFeature feature) => feature.type == DisplayFeatureType.cutout,
  );
}
