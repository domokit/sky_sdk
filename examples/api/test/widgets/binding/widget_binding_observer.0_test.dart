// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_api_samples/widgets/binding/widget_binding_observer.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('App tracks lifecycle states', (WidgetTester tester) async {
    Future<void> setAppLifeCycleState(AppLifecycleState state) async {
      final ByteData? message = const StringCodec().encodeMessage(state.toString());
      await tester.binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/lifecycle',
        message,
        (_) {},
      );
    }

    await tester.pumpWidget(const example.WidgetBindingObserverExampleApp());

    expect(find.text('There are no AppLifecycleStates to show.'), findsOneWidget);

    await setAppLifeCycleState(AppLifecycleState.resumed);
    await tester.pumpAndSettle();
    expect(find.text('state is: AppLifecycleState.resumed'), findsOneWidget);

    await setAppLifeCycleState(AppLifecycleState.inactive);
    await tester.pumpAndSettle();
    expect(find.text('state is: AppLifecycleState.inactive'), findsOneWidget);

    await setAppLifeCycleState(AppLifecycleState.paused);
    await tester.pumpAndSettle();
    // Can't look for paused text here because rendering is paused.

    await setAppLifeCycleState(AppLifecycleState.inactive);
    await tester.pumpAndSettle();
    expect(find.text('state is: AppLifecycleState.inactive'), findsNWidgets(2));

    await setAppLifeCycleState(AppLifecycleState.resumed);
    await tester.pumpAndSettle();
    expect(find.text('state is: AppLifecycleState.resumed'), findsNWidgets(2));
  });
}
