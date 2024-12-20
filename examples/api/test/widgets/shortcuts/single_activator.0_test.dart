// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart';
import 'package:flutter_api_samples/widgets/shortcuts/single_activator.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  Future<void> pressControlC(WidgetTester tester) async {
    await tester.sendKeyDownEvent(LogicalKeyboardKey.control);
    await tester.sendKeyDownEvent(LogicalKeyboardKey.keyC);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.keyC);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.control);
  }

  group('SingleActivatorExampleApp', () {
    testWidgets('displays correct labels', (WidgetTester tester) async {
      await tester.pumpWidget(const example.SingleActivatorExampleApp());

      expect(find.text('Add to the counter by pressing Ctrl+C'), findsOneWidget);
      expect(find.text('count: 0'), findsOneWidget);
    });

    testWidgets('updates counter when Ctrl-C combination pressed', (WidgetTester tester) async {
      await tester.pumpWidget(const example.SingleActivatorExampleApp());

      for (int counter = 0; counter < 10; counter++) {
        expect(find.text('count: $counter'), findsOneWidget);

        await pressControlC(tester);
        await tester.pump();
      }
    });
  });
}
