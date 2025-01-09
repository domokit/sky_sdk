// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/animated_grid/sliver_animated_grid.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('SliverAnimatedGrid example', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SliverAnimatedGridSample());

    expect(find.text('1'), findsOneWidget);
    expect(find.text('2'), findsOneWidget);
    expect(find.text('3'), findsOneWidget);
    expect(find.text('4'), findsOneWidget);
    expect(find.text('5'), findsOneWidget);
    expect(find.text('6'), findsOneWidget);
    expect(find.text('7'), findsNothing);

    await tester.tap(find.byIcon(Icons.add_circle));
    await tester.pumpAndSettle();

    expect(find.text('7'), findsOneWidget);

    await tester.tap(find.byIcon(Icons.remove_circle));
    await tester.pumpAndSettle();

    expect(find.text('7'), findsNothing);

    await tester.tap(find.text('2'));
    await tester.pumpAndSettle();

    await tester.tap(find.byIcon(Icons.remove_circle));
    await tester.pumpAndSettle();

    expect(find.text('2'), findsNothing);
    expect(find.text('6'), findsOneWidget);
  });
}
