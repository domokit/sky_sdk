// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/scrollbar/scrollbar.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('The scrollbar thumb should be visible at all time', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ScrollbarExampleApp());
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 10)); // Wait for the thumb to start appearing.

    expect(find.widgetWithText(AppBar, 'Scrollbar Sample'), findsOne);

    expect(find.text('item 0'), findsOne);
    expect(find.text('item 9'), findsNothing);
    expect(find.byType(Scrollbar), paints..rect());

    await tester.fling(find.byType(Scrollbar).last, const Offset(0, -300), 10.0);

    expect(find.text('item 0'), findsNothing);
    expect(find.text('item 9'), findsOne);
    expect(find.byType(Scrollbar), paints..rect());
  });
}
