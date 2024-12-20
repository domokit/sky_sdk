// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_api_samples/widgets/app_lifecycle_listener/app_lifecycle_listener.1.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('AppLifecycleListener example', (WidgetTester tester) async {
    await tester.pumpWidget(const example.AppLifecycleListenerExample());

    expect(find.text('Do Not Allow Exit'), findsOneWidget);
    expect(find.text('Allow Exit'), findsOneWidget);
    expect(find.text('Quit'), findsOneWidget);
    expect(find.textContaining('Exit Request:'), findsOneWidget);
    await tester.tap(find.text('Quit'));
    await tester.pump();
    // Responding to the quit request happens in a Future that we don't have
    // visibility for, so to avoid a flaky test with a delay, we just check to
    // see if the request string prefix is still there, rather than the request
    // response string. Testing it wasn't worth exposing a Completer in the
    // example code.
    expect(find.textContaining('Exit Request:'), findsOneWidget);
  });
}
