// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/switch/switch.3.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Can toggle switch', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SwitchApp());

    final Finder switchFinder = find.byType(Switch).first;
    Switch materialSwitch = tester.widget<Switch>(switchFinder);
    expect(materialSwitch.value, true);

    await tester.tap(switchFinder);
    await tester.pumpAndSettle();
    materialSwitch = tester.widget<Switch>(switchFinder);
    expect(materialSwitch.value, false);
  });
}
