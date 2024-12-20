// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

import 'package:flutter_api_samples/widgets/navigator_pop_handler/navigator_pop_handler.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

import '../navigator_utils.dart';

void main() {
  bool? lastFrameworkHandlesBack;
  setUp(() async {
    lastFrameworkHandlesBack = null;
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      (MethodCall methodCall) async {
        if (methodCall.method == 'SystemNavigator.setFrameworkHandlesBack') {
          expect(methodCall.arguments, isA<bool>());
          lastFrameworkHandlesBack = methodCall.arguments as bool;
        }
        return;
      },
    );
    await TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.handlePlatformMessage(
      'flutter/lifecycle',
      const StringCodec().encodeMessage(AppLifecycleState.resumed.toString()),
      (ByteData? data) {},
    );
  });

  tearDown(() {
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      null,
    );
  });

  testWidgets('Can go back with system back gesture', (WidgetTester tester) async {
    await tester.pumpWidget(const example.NavigatorPopHandlerApp());

    expect(find.text('Nested Navigators Example'), findsOneWidget);
    expect(find.text('Nested Navigators Page One'), findsNothing);
    expect(find.text('Nested Navigators Page Two'), findsNothing);
    if (!kIsWeb && defaultTargetPlatform == TargetPlatform.android) {
      expect(lastFrameworkHandlesBack, isFalse);
    }

    await tester.tap(find.text('Nested Navigator route'));
    await tester.pumpAndSettle();

    expect(find.text('Nested Navigators Example'), findsNothing);
    expect(find.text('Nested Navigators Page One'), findsOneWidget);
    expect(find.text('Nested Navigators Page Two'), findsNothing);
    if (!kIsWeb && defaultTargetPlatform == TargetPlatform.android) {
      expect(lastFrameworkHandlesBack, isTrue);
    }

    await tester.tap(find.text('Go to another route in this nested Navigator'));
    await tester.pumpAndSettle();

    expect(find.text('Nested Navigators Example'), findsNothing);
    expect(find.text('Nested Navigators Page One'), findsNothing);
    expect(find.text('Nested Navigators Page Two'), findsOneWidget);
    if (!kIsWeb && defaultTargetPlatform == TargetPlatform.android) {
      expect(lastFrameworkHandlesBack, isTrue);
    }

    await simulateSystemBack();
    await tester.pumpAndSettle();

    expect(find.text('Nested Navigators Example'), findsNothing);
    expect(find.text('Nested Navigators Page One'), findsOneWidget);
    expect(find.text('Nested Navigators Page Two'), findsNothing);
    if (!kIsWeb && defaultTargetPlatform == TargetPlatform.android) {
      expect(lastFrameworkHandlesBack, isTrue);
    }

    await simulateSystemBack();
    await tester.pumpAndSettle();

    expect(find.text('Nested Navigators Example'), findsOneWidget);
    expect(find.text('Nested Navigators Page One'), findsNothing);
    expect(find.text('Nested Navigators Page Two'), findsNothing);
    if (!kIsWeb && defaultTargetPlatform == TargetPlatform.android) {
      expect(lastFrameworkHandlesBack, isFalse);
    }
  });

  testWidgets('restoring the app preserves the navigation stack', (WidgetTester tester) async {
    await tester.pumpWidget(const example.NavigatorPopHandlerApp());

    expect(find.text('Nested Navigators Example'), findsOneWidget);
    expect(find.text('Nested Navigators Page One'), findsNothing);
    expect(find.text('Nested Navigators Page Two'), findsNothing);
    if (!kIsWeb && defaultTargetPlatform == TargetPlatform.android) {
      expect(lastFrameworkHandlesBack, isFalse);
    }

    await tester.tap(find.text('Nested Navigator route'));
    await tester.pumpAndSettle();

    expect(find.text('Nested Navigators Example'), findsNothing);
    expect(find.text('Nested Navigators Page One'), findsOneWidget);
    expect(find.text('Nested Navigators Page Two'), findsNothing);
    if (!kIsWeb && defaultTargetPlatform == TargetPlatform.android) {
      expect(lastFrameworkHandlesBack, isTrue);
    }

    await tester.tap(find.text('Go to another route in this nested Navigator'));
    await tester.pumpAndSettle();

    expect(find.text('Nested Navigators Example'), findsNothing);
    expect(find.text('Nested Navigators Page One'), findsNothing);
    expect(find.text('Nested Navigators Page Two'), findsOneWidget);
    if (!kIsWeb && defaultTargetPlatform == TargetPlatform.android) {
      expect(lastFrameworkHandlesBack, isTrue);
    }

    await tester.restartAndRestore();

    expect(find.text('Nested Navigators Example'), findsNothing);
    expect(find.text('Nested Navigators Page One'), findsNothing);
    expect(find.text('Nested Navigators Page Two'), findsOneWidget);
    if (!kIsWeb && defaultTargetPlatform == TargetPlatform.android) {
      expect(lastFrameworkHandlesBack, isTrue);
    }
  });
}
