// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@Tags(<String>['reduced-test-set'])
library;

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:integration_test/integration_test.dart';

import 'package:integration_test_example/main.dart' as app;

/// To run:
///
/// ```sh
/// # Be in this directory
/// cd dev/packages/integration_test/example
///
/// flutter test integration_test/matches_golden_test.dart
/// ```
///
/// To run on a particular device, see `flutter -d`.
void main() {
  IntegrationTestWidgetsFlutterBinding.ensureInitialized();

  // TODO(matanlurey): Make this automatic as part of the bootstrap.
  VmServiceProxyGoldenFileComparator.useIfRunningOnDevice();

  testWidgets('can use matchesGoldenFile with integration_test', (WidgetTester tester) async {
    // Build our app and trigger a frame.
    app.main();

    // TODO(matanlurey): Is this necessary?
    await tester.pumpAndSettle();

    // Take a screenshot.
    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('integration_test_matches_golden_file.png'),
    );
  });
}
