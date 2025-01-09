// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_gallery/gallery/app.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  final TestWidgetsFlutterBinding binding = TestWidgetsFlutterBinding.ensureInitialized();
  if (binding is LiveTestWidgetsFlutterBinding) {
    binding.framePolicy = LiveTestWidgetsFlutterBindingFramePolicy.fullyLive;
  }

  // Regression test for https://github.com/flutter/flutter/pull/5168
  testWidgets('Pesto appbar heroics', (WidgetTester tester) async {
    await tester.pumpWidget(
      // The bug only manifests itself when the screen's orientation is portrait
      const Center(child: SizedBox(width: 450.0, height: 800.0, child: GalleryApp(testMode: true))),
    );
    await tester.pump(); // see https://github.com/flutter/flutter/issues/1865
    await tester.pump(); // triggers a frame

    await tester.tap(find.text('Studies'));
    await tester.pumpAndSettle();

    await tester.tap(find.text('Pesto'));
    await tester.pumpAndSettle();

    await tester.tap(find.text('Roasted Chicken'));
    await tester.pumpAndSettle();

    await tester.drag(find.text('Roasted Chicken'), const Offset(0.0, -300.0));
    await tester.pumpAndSettle();

    Navigator.pop(find.byType(Scaffold).evaluate().single);
    await tester.pumpAndSettle();
  });

  testWidgets('Pesto can be scrolled all the way down', (WidgetTester tester) async {
    await tester.pumpWidget(const GalleryApp(testMode: true));
    await tester.pump(); // see https://github.com/flutter/flutter/issues/1865
    await tester.pump(); // triggers a frame

    await tester.tap(find.text('Studies'));
    await tester.pumpAndSettle();

    await tester.tap(find.text('Pesto'));
    await tester.pumpAndSettle();

    await tester.fling(find.text('Roasted Chicken'), const Offset(0.0, -200.0), 10000.0);
    await tester.pumpAndSettle(); // start and finish fling
    expect(find.text('Spanakopita'), findsOneWidget);
  });
}
