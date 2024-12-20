// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter_test/flutter_test.dart';

class TestTestBinding extends AutomatedTestWidgetsFlutterBinding {
  @override
  DebugPrintCallback get debugPrintOverride => testPrint;
  static void testPrint(String? message, {int? wrapWidth}) {
    print(message);
  }
}

Future<void> helperFunction(WidgetTester tester) async {
  await tester.pump();
}

void main() {
  TestTestBinding();
  testWidgets('TestAsyncUtils - handling unguarded async helper functions', (
    WidgetTester tester,
  ) async {
    helperFunction(tester);
    helperFunction(tester);
    // this should fail
  });
}
