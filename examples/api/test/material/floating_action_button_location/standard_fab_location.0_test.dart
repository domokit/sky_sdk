// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/floating_action_button_location/standard_fab_location.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('The FloatingActionButton should have a right padding', (WidgetTester tester) async {
    await tester.pumpWidget(const example.StandardFabLocationExampleApp());

    expect(find.widgetWithIcon(FloatingActionButton, Icons.add), findsOne);
    final double right = tester.getCenter(find.byType(FloatingActionButton)).dx;
    expect(right, closeTo(706, 1));
  });
}
