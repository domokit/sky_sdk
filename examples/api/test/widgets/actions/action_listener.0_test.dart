// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/actions/action_listener.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('ActionListener can be enabled, triggered, and disabled', (
    WidgetTester tester,
  ) async {
    final List<String?> log = <String?>[];

    final DebugPrintCallback originalDebugPrint = debugPrint;
    debugPrint = (String? message, {int? wrapWidth}) {
      log.add(message);
    };
    try {
      await tester.pumpWidget(const example.ActionListenerExampleApp());

      expect(find.widgetWithText(AppBar, 'ActionListener Sample'), findsOne);
      expect(find.widgetWithText(OutlinedButton, 'Enable'), findsOne);

      await tester.tap(find.byType(OutlinedButton));
      await tester.pump();

      expect(find.widgetWithText(OutlinedButton, 'Disable'), findsOne);
      expect(find.widgetWithText(ElevatedButton, 'Call Action Listener'), findsOne);
      expect(log, const <String?>['Action Listener was added']);

      await tester.tap(find.text('Call Action Listener'));
      await tester.pumpAndSettle();

      expect(find.widgetWithText(SnackBar, 'Action Listener Called'), findsOne);

      await tester.tap(find.text('Disable'));
      await tester.pump();

      expect(find.widgetWithText(OutlinedButton, 'Enable'), findsOne);
      expect(log, const <String?>['Action Listener was added', 'Action Listener was removed']);
    } finally {
      debugPrint = originalDebugPrint;
    }
  });
}
