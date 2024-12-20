// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/switch/cupertino_switch.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Toggling cupertino switch updates icon', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CupertinoSwitchApp());

    final Finder switchFinder = find.byType(CupertinoSwitch);
    CupertinoSwitch cupertinoSwitch = tester.widget<CupertinoSwitch>(switchFinder);
    expect(cupertinoSwitch.value, true);

    await tester.tap(switchFinder);
    await tester.pumpAndSettle();
    cupertinoSwitch = tester.widget<CupertinoSwitch>(switchFinder);
    expect(cupertinoSwitch.value, false);
  });
}
