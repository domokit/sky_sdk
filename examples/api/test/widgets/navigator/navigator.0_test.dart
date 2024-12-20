// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_api_samples/widgets/navigator/navigator.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('It should show the home page', (WidgetTester tester) async {
    await tester.pumpWidget(const example.NavigatorExampleApp());

    expect(find.text('Home Page'), findsOne);
  });

  testWidgets('It should start from the sign up page and follow the flow to reach the home page', (
    WidgetTester tester,
  ) async {
    tester.platformDispatcher.defaultRouteNameTestValue = '/signup';
    await tester.pumpWidget(const example.NavigatorExampleApp());

    expect(find.text('Collect Personal Info Page'), findsOne);

    await tester.tap(find.text('Collect Personal Info Page'));
    await tester.pumpAndSettle();

    expect(find.text('Choose Credentials Page'), findsOne);

    await tester.tap(find.text('Choose Credentials Page'));
    await tester.pumpAndSettle();

    expect(find.text('Home Page'), findsOne);
  });
}
