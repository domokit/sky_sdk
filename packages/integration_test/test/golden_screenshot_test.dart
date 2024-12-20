// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@Tags(<String>['reduced-test-set'])
library;

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:integration_test/integration_test.dart';

void main() {
  IntegrationTestWidgetsFlutterBinding.ensureInitialized();

  testWidgets('a screenshot of a Placeholder', (WidgetTester tester) async {
    await tester.pumpWidget(const Placeholder());

    await expectLater(find.byType(Placeholder), matchesGoldenFile('placeholder.png'));
  });
}
