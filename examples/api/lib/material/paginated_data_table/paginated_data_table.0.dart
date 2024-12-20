// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [PaginatedDataTable].

class MyDataSource extends DataTableSource {
  @override
  int get rowCount => 3;

  @override
  DataRow? getRow(int index) {
    switch (index) {
      case 0:
        return const DataRow(
          cells: <DataCell>[
            DataCell(Text('Sarah')),
            DataCell(Text('19')),
            DataCell(Text('Student')),
          ],
        );
      case 1:
        return const DataRow(
          cells: <DataCell>[
            DataCell(Text('Janine')),
            DataCell(Text('43')),
            DataCell(Text('Professor')),
          ],
        );
      case 2:
        return const DataRow(
          cells: <DataCell>[
            DataCell(Text('William')),
            DataCell(Text('27')),
            DataCell(Text('Associate Professor')),
          ],
        );
      default:
        return null;
    }
  }

  @override
  bool get isRowCountApproximate => false;

  @override
  int get selectedRowCount => 0;
}

final DataTableSource dataSource = MyDataSource();

void main() => runApp(const DataTableExampleApp());

class DataTableExampleApp extends StatelessWidget {
  const DataTableExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(
      home: SingleChildScrollView(padding: EdgeInsets.all(12.0), child: DataTableExample()),
    );
  }
}

class DataTableExample extends StatelessWidget {
  const DataTableExample({super.key});

  @override
  Widget build(BuildContext context) {
    return PaginatedDataTable(
      columns: const <DataColumn>[
        DataColumn(label: Text('Name')),
        DataColumn(label: Text('Age')),
        DataColumn(label: Text('Role')),
      ],
      source: dataSource,
    );
  }
}
