// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

// Regression test for https://github.com/flutter/flutter/issues/111285
void main() {
  testWidgets('Can load asset from same package without error', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(home: Scaffold(body: Image.asset('assets/foo.png', package: 'integration_ui'))),
    );
    await tester.pumpAndSettle();

    // If this asset couldn't be loaded, the exception message would be
    // "asset failed to load"
    expect(tester.takeException().toString(), contains('Invalid image data'));
  });
}
