// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter_api_samples/material/checkbox_list_tile/checkbox_list_tile.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('CheckboxListTile can be checked', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CheckboxListTileApp());

    CheckboxListTile checkboxListTile = tester.widget(find.byType(CheckboxListTile));
    expect(checkboxListTile.value, isFalse);

    await tester.tap(find.byType(CheckboxListTile));
    await tester.pump();
    timeDilation = 1.0;

    checkboxListTile = tester.widget(find.byType(CheckboxListTile));
    expect(checkboxListTile.value, isTrue);

    await tester.tap(find.byType(CheckboxListTile));
    await tester.pump();

    checkboxListTile = tester.widget(find.byType(CheckboxListTile));
    expect(checkboxListTile.value, isFalse);
  });
}
