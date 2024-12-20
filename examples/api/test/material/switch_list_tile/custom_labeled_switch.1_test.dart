// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/switch_list_tile/custom_labeled_switch.1.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Tapping LabeledSwitch toggles the switch', (WidgetTester tester) async {
    await tester.pumpWidget(const example.LabeledSwitchApp());

    // Switch is initially off.
    Switch switchWidget = tester.widget(find.byType(Switch));
    expect(switchWidget.value, isFalse);

    // Tap to toggle the switch.
    await tester.tap(find.byType(example.LabeledSwitch));
    await tester.pumpAndSettle();

    // Switch is now on.
    switchWidget = tester.widget(find.byType(Switch));
    expect(switchWidget.value, isTrue);
  });
}
