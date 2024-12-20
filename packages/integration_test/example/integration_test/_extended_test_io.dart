// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This is a basic Flutter widget test.
//
// To perform an interaction with a widget in your test, use the WidgetTester
// utility that Flutter provides. For example, you can send tap and scroll
// gestures. You can also use WidgetTester to find child widgets in the widget
// tree, read text, and verify that the values of widget properties are correct.

import 'dart:io' show Platform;
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:integration_test/integration_test.dart';

import 'package:integration_test_example/main.dart' as app;

void main() {
  final IntegrationTestWidgetsFlutterBinding binding =
      IntegrationTestWidgetsFlutterBinding.ensureInitialized();

  testWidgets('verify text', (WidgetTester tester) async {
    // Build our app.
    app.main();

    // Pump a frame.
    await tester.pumpAndSettle();

    // Verify that platform version is retrieved.
    expect(
      find.byWidgetPredicate(
        (Widget widget) =>
            widget is Text && widget.data!.startsWith('Platform: ${Platform.operatingSystem}'),
      ),
      findsOneWidget,
    );
  });

  testWidgets('verify screenshot', (WidgetTester tester) async {
    // Build our app.
    app.main();

    // On Android, this is required prior to taking the screenshot.
    await binding.convertFlutterSurfaceToImage();

    // Pump a frame before taking the screenshot.
    await tester.pumpAndSettle();
    final List<int> firstPng = await binding.takeScreenshot('first');
    expect(firstPng.isNotEmpty, isTrue);

    // Pump another frame before taking the screenshot.
    await tester.pumpAndSettle();
    final List<int> secondPng = await binding.takeScreenshot('second');
    expect(secondPng.isNotEmpty, isTrue);

    expect(listEquals(firstPng, secondPng), isTrue);
  });
}
