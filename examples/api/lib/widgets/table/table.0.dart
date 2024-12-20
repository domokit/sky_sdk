// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [Table].

void main() => runApp(const TableExampleApp());

class TableExampleApp extends StatelessWidget {
  const TableExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(appBar: AppBar(title: const Text('Table Sample')), body: const TableExample()),
    );
  }
}

class TableExample extends StatelessWidget {
  const TableExample({super.key});

  @override
  Widget build(BuildContext context) {
    return Table(
      border: TableBorder.all(),
      columnWidths: const <int, TableColumnWidth>{
        0: IntrinsicColumnWidth(),
        1: FlexColumnWidth(),
        2: FixedColumnWidth(64),
      },
      defaultVerticalAlignment: TableCellVerticalAlignment.middle,
      children: <TableRow>[
        TableRow(
          children: <Widget>[
            Container(height: 32, color: Colors.green),
            TableCell(
              verticalAlignment: TableCellVerticalAlignment.top,
              child: Container(height: 32, width: 32, color: Colors.red),
            ),
            Container(height: 64, color: Colors.blue),
          ],
        ),
        TableRow(
          decoration: const BoxDecoration(color: Colors.grey),
          children: <Widget>[
            Container(height: 64, width: 128, color: Colors.purple),
            Container(height: 32, color: Colors.yellow),
            Center(child: Container(height: 32, width: 32, color: Colors.orange)),
          ],
        ),
      ],
    );
  }
}
