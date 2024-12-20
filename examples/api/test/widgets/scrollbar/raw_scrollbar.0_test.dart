// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/scrollbar/raw_scrollbar.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('There are two scrollbars', (WidgetTester tester) async {
    await tester.pumpWidget(const example.RawScrollbarExampleApp());

    expect(find.widgetWithText(AppBar, 'RawScrollbar Sample'), findsOne);
    expect(find.byType(Scrollbar), findsExactly(2));

    expect(find.text('Scrollable 1 : Index 0'), findsOne);
    expect(find.text('Scrollable 2 : Index 0'), findsOne);

    final TestPointer pointer = TestPointer(1, PointerDeviceKind.mouse);
    pointer.hover(tester.getCenter(find.byType(ListView).first));
    await tester.sendEventToBinding(pointer.scroll(const Offset(0.0, 1000)));
    await tester.pumpAndSettle();

    expect(find.text('Scrollable 1 : Index 40'), findsOne);
    expect(find.text('Scrollable 2 : Index 0'), findsOne);

    pointer.hover(tester.getCenter(find.byType(ListView).last));
    await tester.sendEventToBinding(pointer.scroll(const Offset(0.0, 1000)));
    await tester.pumpAndSettle();

    expect(find.text('Scrollable 1 : Index 40'), findsOne);
    expect(find.text('Scrollable 2 : Index 20'), findsOne);
  });
}
