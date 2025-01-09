// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/services.dart';
import 'package:flutter_gallery/demo/cupertino/cupertino_navigation_demo.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Navigation demo golden', (WidgetTester tester) async {
    // The point is to mainly test the cupertino icons that we don't have a
    // dependency against in the flutter/cupertino package directly.

    final Future<ByteData> font = rootBundle.load(
      'packages/cupertino_icons/assets/CupertinoIcons.ttf',
    );

    await (FontLoader('packages/cupertino_icons/CupertinoIcons')..addFont(font)).load();

    await tester.pumpWidget(CupertinoApp(home: CupertinoNavigationDemo(randomSeed: 123456)));

    await expectLater(
      find.byType(CupertinoNavigationDemo),
      matchesGoldenFile('cupertino_navigation_demo.screen.1.png'),
    );

    await tester.pump(); // Need a new frame after loading fonts to refresh layout.
    // Tap some row to go to the next page.
    await tester.tap(find.text('Buy this cool color').first);
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 600));

    await expectLater(
      find.byType(CupertinoNavigationDemo),
      matchesGoldenFile('cupertino_navigation_demo.screen.2.png'),
    );
  });
}
