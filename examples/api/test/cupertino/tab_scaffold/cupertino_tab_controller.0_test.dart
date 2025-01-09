// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/tab_scaffold/cupertino_tab_controller.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Can switch tabs using CupertinoTabController', (WidgetTester tester) async {
    await tester.pumpWidget(const example.TabControllerApp());

    expect(find.text('Content of tab 0'), findsOneWidget);
    await tester.tap(find.byIcon(CupertinoIcons.star_circle_fill));
    await tester.pumpAndSettle();
    expect(find.text('Content of tab 1'), findsOneWidget);

    await tester.tap(find.text('Go to first tab'));
    await tester.pumpAndSettle();
    expect(find.text('Content of tab 0'), findsOneWidget);
  });
}
