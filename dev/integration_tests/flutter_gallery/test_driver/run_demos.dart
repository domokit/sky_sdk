// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_gallery/demo_lists.dart';
import 'package:flutter_test/flutter_test.dart';

/// The demos we don't run as part of the integration test.
///
/// Demo names are formatted as 'DEMO_NAME@DEMO_CATEGORY' (see
/// `demo_lists.dart` for more examples).
final List<String> kSkippedDemos = <String>[
  // This demo is flaky on CI due to hitting the network.
  // See: https://github.com/flutter/flutter/issues/100497
  'Video@Media',
];

/// Scrolls each demo menu item into view, launches it, then returns to the
/// home screen twice.
Future<void> runDemos(List<String> demos, WidgetController controller) async {
  final Finder demoList = find.byType(Scrollable);
  String? currentDemoCategory;

  for (final String demo in demos) {
    if (kSkippedDemos.contains(demo)) {
      continue;
    }

    final String demoName = demo.substring(0, demo.indexOf('@'));
    final String demoCategory = demo.substring(demo.indexOf('@') + 1);
    print('> $demo');
    await controller.pump(const Duration(milliseconds: 250));

    final Finder demoCategoryItem = find.text(demoCategory);
    if (currentDemoCategory == null) {
      await controller.scrollUntilVisible(demoCategoryItem, 48.0);
      await controller.tap(demoCategoryItem);
      await controller.pumpAndSettle();
    } else if (currentDemoCategory != demoCategory) {
      await controller.tap(find.byTooltip('Back'));
      await controller.pumpAndSettle();
      await controller.scrollUntilVisible(demoCategoryItem, 48.0);
      await controller.tap(demoCategoryItem);
      await controller.pumpAndSettle();
      // Scroll back to the top
      await controller.drag(demoList, const Offset(0.0, 10000.0));
      await controller.pumpAndSettle();
    }
    currentDemoCategory = demoCategory;

    Future<void> pageBack() {
      Finder backButton = find.byTooltip('Back');
      if (backButton.evaluate().isEmpty) {
        backButton = find.byType(CupertinoNavigationBarBackButton);
      }
      return controller.tap(backButton);
    }

    for (int i = 0; i < 2; i += 1) {
      final Finder demoItem = find.text(demoName);
      await controller.scrollUntilVisible(demoItem, 48.0);
      await controller.pumpAndSettle();
      if (demoItem.evaluate().isEmpty) {
        print('Failed to find $demoItem');
        print('All available elements:');
        print(controller.allElements.toList().join('\n'));
        print('App structure:');
        debugDumpApp();
        throw TestFailure('Failed to find element');
      }
      await controller.tap(demoItem); // Launch the demo

      if (kUnsynchronizedDemos.contains(demo)) {
        // These tests have animation, pumpAndSettle cannot be used.
        // This time is questionable. 600ms is the tested reasonable result.
        await controller.pump(const Duration(milliseconds: 600));
        await controller.pump();
        await pageBack();
      } else {
        await controller.pumpAndSettle();
        // page back
        await pageBack();
      }
      await controller.pumpAndSettle();
    }

    print('< Success');
  }

  // Return to the home screen
  await controller.tap(find.byTooltip('Back'));
  await controller.pumpAndSettle();
}
