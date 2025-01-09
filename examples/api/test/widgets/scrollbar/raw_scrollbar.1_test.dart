// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/scrollbar/raw_scrollbar.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('When thumbVisibility is true, the scrollbar thumb remains visible', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.RawScrollbarExampleApp());

    expect(find.widgetWithText(AppBar, 'RawScrollbar Sample'), findsOne);
    expect(find.byType(RawScrollbar), findsOne);
    expect(find.byType(GridView), findsOne);
    expect(find.byType(RawScrollbar), paints..clipRect());

    expect(find.text('item 0'), findsOne);
    expect(find.text('item 1'), findsOne);
    expect(find.text('item 2'), findsOne);

    final TestPointer pointer = TestPointer(1, PointerDeviceKind.mouse);
    pointer.hover(tester.getCenter(find.byType(GridView)));
    await tester.sendEventToBinding(pointer.scroll(const Offset(0.0, 1000)));
    await tester.pumpAndSettle();

    expect(find.byType(RawScrollbar), paints..clipRect());

    expect(find.text('item 15'), findsOne);
    expect(find.text('item 16'), findsOne);
    expect(find.text('item 17'), findsOne);
  });
}
