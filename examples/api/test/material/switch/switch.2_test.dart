// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/switch/switch.2.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Switch thumb icon supports material states', (WidgetTester tester) async {
    const Set<MaterialState> selected = <MaterialState>{MaterialState.selected};
    const Set<MaterialState> unselected = <MaterialState>{};

    await tester.pumpWidget(const example.SwitchApp());

    Switch materialSwitch = tester.widget<Switch>(find.byType(Switch).first);
    expect(materialSwitch.thumbIcon, null);

    materialSwitch = tester.widget<Switch>(find.byType(Switch).last);
    expect(materialSwitch.thumbIcon, isNotNull);
    expect(materialSwitch.thumbIcon!.resolve(selected)!.icon, Icons.check);
    expect(materialSwitch.thumbIcon!.resolve(unselected)!.icon, Icons.close);
  });
}
