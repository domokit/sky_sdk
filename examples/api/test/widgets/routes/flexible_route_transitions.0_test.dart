// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_api_samples/widgets/routes/flexible_route_transitions.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Flexible Transitions App is able to build', (WidgetTester tester) async {
    await tester.pumpWidget(const example.FlexibleRouteTransitionsApp());

    expect(find.text('Zoom Page'), findsOneWidget);
    expect(find.text('Zoom Transition'), findsOneWidget);
    expect(find.text('Crazy Vertical Transition'), findsOneWidget);
    expect(find.text('Cupertino Transition'), findsOneWidget);

    await tester.tap(find.text('Cupertino Transition'));

    await tester.pumpAndSettle();

    expect(find.text('Zoom Page'), findsNothing);
    expect(find.text('Cupertino Page'), findsOneWidget);
  });

  testWidgets('A vertical slide animation is passed to the previous route', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.FlexibleRouteTransitionsApp());

    expect(find.text('Zoom Page'), findsOneWidget);

    // Save the Y coordinate of the page title.
    double lastYPosition = tester.getTopLeft(find.text('Zoom Page')).dy;

    await tester.tap(find.text('Crazy Vertical Transition'));

    await tester.pump();

    await tester.pump(const Duration(milliseconds: 10));
    // The current Y coordinate of the page title should be lower than it was
    // before as the page slides upwards.
    expect(tester.getTopLeft(find.text('Zoom Page')).dy, lessThan(lastYPosition));
    lastYPosition = tester.getTopLeft(find.text('Zoom Page')).dy;

    await tester.pump(const Duration(milliseconds: 10));
    expect(tester.getTopLeft(find.text('Zoom Page')).dy, lessThan(lastYPosition));
    lastYPosition = tester.getTopLeft(find.text('Zoom Page')).dy;

    await tester.pump(const Duration(milliseconds: 10));
    expect(tester.getTopLeft(find.text('Zoom Page')).dy, lessThan(lastYPosition));
    lastYPosition = tester.getTopLeft(find.text('Zoom Page')).dy;

    await tester.pumpAndSettle();

    expect(find.text('Crazy Vertical Page'), findsOneWidget);
  });
}
