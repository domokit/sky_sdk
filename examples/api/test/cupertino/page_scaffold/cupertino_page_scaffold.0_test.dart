// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/page_scaffold/cupertino_page_scaffold.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Can increment counter', (WidgetTester tester) async {
    await tester.pumpWidget(const example.PageScaffoldApp());

    expect(find.byType(CupertinoPageScaffold), findsOneWidget);
    expect(find.text('You have pressed the button 0 times.'), findsOneWidget);
    await tester.tap(find.byType(CupertinoButton));
    await tester.pumpAndSettle();
    expect(find.text('You have pressed the button 1 times.'), findsOneWidget);
  });
}
