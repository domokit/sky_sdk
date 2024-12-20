// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/data_table/data_table.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('DataTable Smoke Test', (WidgetTester tester) async {
    await tester.pumpWidget(const example.DataTableExampleApp());
    expect(find.widgetWithText(AppBar, 'DataTable Sample'), findsOneWidget);
    expect(find.byType(DataTable), findsOneWidget);
    final DataTable dataTable = tester.widget<DataTable>(find.byType(DataTable));
    expect(dataTable.columns.length, 3);
    expect(dataTable.rows.length, 3);
    for (int i = 0; i < dataTable.rows.length; i++) {
      expect(dataTable.rows[i].cells.length, 3);
    }
    expect(find.text('Name'), findsOneWidget);
    expect(find.text('Age'), findsOneWidget);
    expect(find.text('Role'), findsOneWidget);
    expect(find.text('Sarah'), findsOneWidget);
    expect(find.text('19'), findsOneWidget);
    expect(find.text('Student'), findsOneWidget);
    expect(find.text('Janine'), findsOneWidget);
    expect(find.text('43'), findsOneWidget);
    expect(find.text('Professor'), findsOneWidget);
    expect(find.text('William'), findsOneWidget);
    expect(find.text('27'), findsOneWidget);
    expect(find.text('Associate Professor'), findsOneWidget);
  });
}
