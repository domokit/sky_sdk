// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_gallery/gallery/app.dart' show GalleryApp;
import 'package:flutter_test/flutter_test.dart';

void main() {
  final TestWidgetsFlutterBinding binding = TestWidgetsFlutterBinding.ensureInitialized();
  if (binding is LiveTestWidgetsFlutterBinding) {
    binding.framePolicy = LiveTestWidgetsFlutterBindingFramePolicy.fullyLive;
  }

  testWidgets(
    'Flutter Gallery app simple smoke test',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const GalleryApp(
          testMode: true,
        ), // builds the app and schedules a frame but doesn't trigger one
      );
      await tester.pump(); // see https://github.com/flutter/flutter/issues/1865
      await tester.pump(); // triggers a frame

      final Finder showOptionsPageButton = find.byTooltip('Toggle options page');

      // Show the options page
      await tester.tap(showOptionsPageButton);
      await tester.pumpAndSettle();

      // Switch to the dark theme: the first switch control
      await tester.tap(find.byType(Switch).first);
      await tester.pumpAndSettle();

      // Close the options page
      expect(showOptionsPageButton, findsOneWidget);
      await tester.tap(showOptionsPageButton);
      await tester.pumpAndSettle();

      // Show the studies (aka "vignettes", aka "demos")
      await tester.tap(find.text('Studies'));
      await tester.pumpAndSettle();

      // Show the Contact profile demo and scroll it upwards
      await tester.tap(find.text('Contact profile'));
      await tester.pumpAndSettle();

      await tester.drag(find.text('(650) 555-1234'), const Offset(0.0, -50.0));
      await tester.pump(const Duration(milliseconds: 200));
      await tester.drag(find.text('(650) 555-1234'), const Offset(0.0, -50.0));
      await tester.pump(const Duration(milliseconds: 200));
      await tester.drag(find.text('(650) 555-1234'), const Offset(0.0, -50.0));
      await tester.pump(const Duration(milliseconds: 200));
      await tester.drag(find.text('(650) 555-1234'), const Offset(0.0, -50.0));
      await tester.pump(const Duration(milliseconds: 200));
      await tester.drag(find.text('(650) 555-1234'), const Offset(0.0, -50.0));
      await tester.pump(const Duration(milliseconds: 200));
      await tester.drag(find.text('(650) 555-1234'), const Offset(0.0, -50.0));
      await tester.pump(const Duration(milliseconds: 200));

      await tester.pump(const Duration(hours: 100)); // for testing
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.macOS,
    }),
  );
}
