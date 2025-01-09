// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('TextField works correctly when changing helperText', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(child: TextField(decoration: InputDecoration(helperText: 'Awesome'))),
      ),
    );
    expect(find.text('Awesome'), findsNWidgets(1));
    await tester.pump(const Duration(milliseconds: 100));
    expect(find.text('Awesome'), findsNWidgets(1));
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(child: TextField(decoration: InputDecoration(errorText: 'Awesome'))),
      ),
    );
    expect(find.text('Awesome'), findsNWidgets(2));
  });
}
