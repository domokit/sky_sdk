// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_gallery/gallery/app.dart' show GalleryApp;
import 'package:flutter_test/flutter_test.dart';

Future<String> mockUpdateUrlFetcher() {
  // A real implementation would connect to the network to retrieve this value
  return Future<String>.value('http://www.example.com/');
}

void main() {
  final TestWidgetsFlutterBinding binding = TestWidgetsFlutterBinding.ensureInitialized();
  if (binding is LiveTestWidgetsFlutterBinding) {
    binding.framePolicy = LiveTestWidgetsFlutterBindingFramePolicy.fullyLive;
  }

  // Regression test for https://github.com/flutter/flutter/pull/5168
  testWidgets('update dialog', (WidgetTester tester) async {
    await tester.pumpWidget(
      const GalleryApp(testMode: true, updateUrlFetcher: mockUpdateUrlFetcher),
    );
    await tester.pump(); // see https://github.com/flutter/flutter/issues/1865
    await tester.pump(); // triggers a frame

    expect(find.text('UPDATE'), findsOneWidget);

    await tester.tap(find.text('NO THANKS'));
    await tester.pump();

    await tester.tap(find.text('Studies'));
    await tester.pump(); // Launch
    await tester.pump(const Duration(seconds: 1)); // transition is complete

    final Finder backButton = find.byTooltip('Back');
    expect(backButton, findsOneWidget);
    await tester.tap(backButton);
    await tester.pump(); // Start the pop "back" operation.
    await tester.pump(); // Complete the willPop() Future.
    await tester.pump(const Duration(seconds: 1)); // transition is complete
    //await tester.pumpUntilNoTransientCallbacks(const Duration(seconds: 1));

    expect(find.text('UPDATE'), findsNothing);
  });
}
