// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/data_table/data_table.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('DataTable is scrollable', (WidgetTester tester) async {
    await tester.pumpWidget(const example.DataTableExampleApp());

    expect(find.byType(SingleChildScrollView), findsOneWidget);

    expect(tester.getTopLeft(find.text('Row 5')), const Offset(66.0, 366.0));

    await tester.drag(find.byType(SingleChildScrollView), const Offset(0.0, -200.0));
    await tester.pumpAndSettle();

    expect(tester.getTopLeft(find.text('Row 5')), const Offset(66.0, 186.0));
  });
}
