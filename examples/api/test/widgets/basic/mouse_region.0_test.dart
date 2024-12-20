// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/basic/mouse_region.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('MouseRegion detects mouse entries, exists, and location', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.MouseRegionApp());

    expect(find.text('0 Entries\n0 Exits'), findsOneWidget);
    expect(find.text('The cursor is here: (0.00, 0.00)'), findsOneWidget);

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byType(ColoredBox)));
    await tester.pump();

    expect(find.text('1 Entries\n0 Exits'), findsOneWidget);
    expect(find.text('The cursor is here: (400.00, 328.00)'), findsOneWidget);

    await gesture.moveTo(tester.getCenter(find.byType(ColoredBox)) + const Offset(50.0, 30.0));
    await tester.pump();

    expect(find.text('The cursor is here: (450.00, 358.00)'), findsOneWidget);

    await gesture.moveTo(Offset.zero);
    await tester.pump();

    expect(find.text('1 Entries\n1 Exits'), findsOneWidget);
  });
}
