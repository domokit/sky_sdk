// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/badge/badge.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Verify Badges have label and count', (WidgetTester tester) async {
    await tester.pumpWidget(const example.BadgeExampleApp());
    // Verify that two Badge(s) are present
    expect(find.byType(Badge), findsNWidgets(2));

    // Verify that Badge.count displays label 999+ when count is greater than 999
    expect(find.text('999+'), findsOneWidget);

    // Verify that Badge displays custom label
    expect(find.text('Your label'), findsOneWidget);
  });
}
