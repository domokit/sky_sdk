// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_test/flutter_test.dart';
import 'package:integration_test/integration_test.dart';
import 'package:web_e2e_tests/profile_diagnostics_main.dart' as app;

void main() {
  IntegrationTestWidgetsFlutterBinding.ensureInitialized();

  testWidgets('App build method exception should form valid FlutterErrorDetails', (
    WidgetTester tester,
  ) async {
    app.main();
    await tester.pumpAndSettle();
    final dynamic appError = tester.takeException();
    expect(appError, isA<TypeError>());
  });
}
