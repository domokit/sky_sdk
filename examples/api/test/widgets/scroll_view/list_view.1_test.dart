// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/scroll_view/list_view.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('tapping Reverse button should reverse ListView', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ListViewExampleApp());
    final Finder listView = find.byType(ListView);
    final Finder reverseFinder = find.text('Reverse items');
    expect(listView, findsOneWidget);
    expect(reverseFinder, findsOneWidget);
    final Finder keepAliveItemFinder = find.byType(example.KeepAliveItem);
    example.KeepAliveItem firstWidget = tester.firstWidget(keepAliveItemFinder);
    expect(firstWidget.data, '1');
    await tester.tap(reverseFinder);
    await tester.pump();
    firstWidget = tester.firstWidget(keepAliveItemFinder);
    expect(firstWidget.data, '5');
  });
}
