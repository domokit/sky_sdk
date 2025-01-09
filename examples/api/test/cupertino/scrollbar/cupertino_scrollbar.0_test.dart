// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/scrollbar/cupertino_scrollbar.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('List view displays CupertinoScrollbar', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ScrollbarApp());

    expect(find.text('Item 0'), findsOneWidget);
    final TestGesture gesture = await tester.startGesture(tester.getCenter(find.byType(ListView)));
    await gesture.moveBy(const Offset(0.0, -100.0));
    await tester.pumpAndSettle();
    expect(find.text('Item 0'), findsNothing);

    final Finder scrollbar = find.byType(CupertinoScrollbar);
    expect(scrollbar, findsOneWidget);
    expect(tester.getTopLeft(scrollbar).dy, 0.0);
    expect(tester.getBottomLeft(scrollbar).dy, 600.0);
  });
}
