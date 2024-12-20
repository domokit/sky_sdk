// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/basic/flow.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Clicking on the menu icon opens the Flow menu', (WidgetTester tester) async {
    await tester.pumpWidget(const example.FlowApp());

    // The menu icon is in the top left corner of the screen.
    Offset menuIcon = tester.getCenter(find.byIcon(Icons.menu));
    expect(menuIcon, const Offset(80.0, 144.0));

    // The home icon is also in the top left corner of the screen.
    Offset homeIcon = tester.getCenter(find.byIcon(Icons.home));
    expect(homeIcon, const Offset(80.0, 144.0));

    // Tap the menu icon to open the flow menu.
    await tester.tapAt(menuIcon);
    await tester.pumpAndSettle();

    // The home icon is still in the top left corner of the screen.
    homeIcon = tester.getCenter(find.byIcon(Icons.home));
    expect(homeIcon, const Offset(80.0, 144.0));

    // The menu icon is now in the top right corner of the screen.
    menuIcon = tester.getCenter(find.byIcon(Icons.menu));
    expect(menuIcon, const Offset(720.0, 144.0));
  });
}
