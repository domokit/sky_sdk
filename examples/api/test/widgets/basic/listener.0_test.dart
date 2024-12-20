// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/basic/listener.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Listener detects press & release, and cursor location', (WidgetTester tester) async {
    await tester.pumpWidget(const MaterialApp(home: example.ListenerExample()));

    expect(find.text('0 presses\n0 releases'), findsOneWidget);
    expect(find.text('The cursor is here: (0.00, 0.00)'), findsOneWidget);

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.down(tester.getCenter(find.byType(ColoredBox)));
    await tester.pump();

    expect(find.text('1 presses\n0 releases'), findsOneWidget);
    expect(find.text('The cursor is here: (400.00, 300.00)'), findsOneWidget);

    await gesture.up();
    await tester.pump();

    expect(find.text('1 presses\n1 releases'), findsOneWidget);
    expect(find.text('The cursor is here: (400.00, 300.00)'), findsOneWidget);
  });
}
