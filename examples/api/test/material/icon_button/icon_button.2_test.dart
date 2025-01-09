// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/icon_button/icon_button.2.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('IconButton Types', (WidgetTester tester) async {
    await tester.pumpWidget(const example.IconButtonApp());
    expect(find.widgetWithIcon(IconButton, Icons.filter_drama), findsNWidgets(8));
    final Finder iconButtons = find.widgetWithIcon(IconButton, Icons.filter_drama);
    for (int i = 0; i <= 3; i++) {
      expect(tester.widget<IconButton>(iconButtons.at(i)).onPressed is VoidCallback, isTrue);
    }
    for (int i = 4; i <= 7; i++) {
      expect(tester.widget<IconButton>(iconButtons.at(i)).onPressed, isNull);
    }
  });
}
