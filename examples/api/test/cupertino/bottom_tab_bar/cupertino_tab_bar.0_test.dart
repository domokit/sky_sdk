// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/bottom_tab_bar/cupertino_tab_bar.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Can switch between tabs', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CupertinoTabBarApp());

    expect(find.byType(CupertinoTabBar), findsOneWidget);
    expect(find.text('Content of tab 0'), findsOneWidget);

    await tester.tap(find.text('Contacts'));
    await tester.pump();

    expect(find.text('Content of tab 2'), findsOneWidget);
  });
}
