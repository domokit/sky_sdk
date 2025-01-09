// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [PaginatedDataTable].

class MyDataSource extends DataTableSource {
  static const List<int> _displayIndexToRawIndex = <int>[0, 3, 4, 5, 6];

  late List<List<Comparable<Object>>> sortedData;
  void setData(List<List<Comparable<Object>>> rawData, int sortColumn, bool sortAscending) {
    sortedData =
        rawData.toList()..sort((List<Comparable<Object>> a, List<Comparable<Object>> b) {
          final Comparable<Object> cellA = a[_displayIndexToRawIndex[sortColumn]];
          final Comparable<Object> cellB = b[_displayIndexToRawIndex[sortColumn]];
          return cellA.compareTo(cellB) * (sortAscending ? 1 : -1);
        });
    notifyListeners();
  }

  @override
  int get rowCount => sortedData.length;

  static DataCell cellFor(Object data) {
    String value;
    if (data is DateTime) {
      value =
          '${data.year}-${data.month.toString().padLeft(2, '0')}-${data.day.toString().padLeft(2, '0')}';
    } else {
      value = data.toString();
    }
    return DataCell(Text(value));
  }

  @override
  DataRow? getRow(int index) {
    return DataRow.byIndex(
      index: sortedData[index][0] as int,
      cells: <DataCell>[
        cellFor('S${sortedData[index][1]}E${sortedData[index][2].toString().padLeft(2, '0')}'),
        cellFor(sortedData[index][3]),
        cellFor(sortedData[index][4]),
        cellFor(sortedData[index][5]),
        cellFor(sortedData[index][6]),
      ],
    );
  }

  @override
  bool get isRowCountApproximate => false;

  @override
  int get selectedRowCount => 0;
}

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

class DataTableExample extends StatefulWidget {
  const DataTableExample({super.key});

  @override
  State<DataTableExample> createState() => _DataTableExampleState();
}

class _DataTableExampleState extends State<DataTableExample> {
  final MyDataSource dataSource = MyDataSource()..setData(episodes, 0, true);

  int _columnIndex = 0;
  bool _columnAscending = true;

  void _sort(int columnIndex, bool ascending) {
    setState(() {
      _columnIndex = columnIndex;
      _columnAscending = ascending;
      dataSource.setData(episodes, _columnIndex, _columnAscending);
    });
  }

  @override
  Widget build(BuildContext context) {
    return PaginatedDataTable(
      sortColumnIndex: _columnIndex,
      sortAscending: _columnAscending,
      columns: <DataColumn>[
        DataColumn(label: const Text('Episode'), onSort: _sort),
        DataColumn(label: const Text('Title'), onSort: _sort),
        DataColumn(label: const Text('Director'), onSort: _sort),
        DataColumn(label: const Text('Writer(s)'), onSort: _sort),
        DataColumn(label: const Text('Air Date'), onSort: _sort),
      ],
      source: dataSource,
    );
  }
}

final List<List<Comparable<Object>>> episodes = <List<Comparable<Object>>>[
  <Comparable<Object>>[
    1,
    1,
    1,
    'Strange New Worlds',
    'Akiva Goldsman',
    'Akiva Goldsman, Alex Kurtzman, Jenny Lumet',
    DateTime(2022, 5, 5),
  ],
  <Comparable<Object>>[
    2,
    1,
    2,
    'Children of the Comet',
    'Maja Vrvilo',
    'Henry Alonso Myers, Sarah Tarkoff',
    DateTime(2022, 5, 12),
  ],
  <Comparable<Object>>[
    3,
    1,
    3,
    'Ghosts of Illyria',
    'Leslie Hope',
    'Akela Cooper, Bill Wolkoff',
    DateTime(2022, 5, 19),
  ],
  <Comparable<Object>>[
    4,
    1,
    4,
    'Memento Mori',
    'Dan Liu',
    'Davy Perez, Beau DeMayo',
    DateTime(2022, 5, 26),
  ],
  <Comparable<Object>>[
    5,
    1,
    5,
    'Spock Amok',
    'Rachel Leiterman',
    'Henry Alonso Myers, Robin Wasserman',
    DateTime(2022, 6, 2),
  ],
  <Comparable<Object>>[
    6,
    1,
    6,
    'Lift Us Where Suffering Cannot Reach',
    'Andi Armaganian',
    'Robin Wasserman, Bill Wolkoff',
    DateTime(2022, 6, 9),
  ],
  <Comparable<Object>>[
    7,
    1,
    7,
    'The Serene Squall',
    'Sydney Freeland',
    'Beau DeMayo, Sarah Tarkoff',
    DateTime(2022, 6, 16),
  ],
  <Comparable<Object>>[
    8,
    1,
    8,
    'The Elysian Kingdom',
    'Amanda Row',
    'Akela Cooper, Onitra Johnson',
    DateTime(2022, 6, 23),
  ],
  <Comparable<Object>>[
    9,
    1,
    9,
    'All Those Who Wander',
    'Christopher J. Byrne',
    'Davy Perez',
    DateTime(2022, 6, 30),
  ],
  <Comparable<Object>>[
    10,
    2,
    10,
    'A Quality of Mercy',
    'Chris Fisher',
    'Henry Alonso Myers, Akiva Goldsman',
    DateTime(2022, 7, 7),
  ],
  <Comparable<Object>>[
    11,
    2,
    1,
    'The Broken Circle',
    'Chris Fisher',
    'Henry Alonso Myers, Akiva Goldsman',
    DateTime(2023, 6, 15),
  ],
  <Comparable<Object>>[
    12,
    2,
    2,
    'Ad Astra per Aspera',
    'Valerie Weiss',
    'Dana Horgan',
    DateTime(2023, 6, 22),
  ],
  <Comparable<Object>>[
    13,
    2,
    3,
    'Tomorrow and Tomorrow and Tomorrow',
    'Amanda Row',
    'David Reed',
    DateTime(2023, 6, 29),
  ],
  <Comparable<Object>>[
    14,
    2,
    4,
    'Among the Lotus Eaters',
    'Eduardo Sánchez',
    'Kirsten Beyer, Davy Perez',
    DateTime(2023, 7, 6),
  ],
  <Comparable<Object>>[
    15,
    2,
    5,
    'Charades',
    'Jordan Canning',
    'Kathryn Lyn, Henry Alonso Myers',
    DateTime(2023, 7, 13),
  ],
  <Comparable<Object>>[
    16,
    2,
    6,
    'Lost in Translation',
    'Dan Liu',
    'Onitra Johnson, David Reed',
    DateTime(2023, 7, 20),
  ],
  <Comparable<Object>>[
    17,
    2,
    7,
    'Those Old Scientists',
    'Jonathan Frakes',
    'Kathryn Lyn, Bill Wolkoff',
    DateTime(2023, 7, 22),
  ],
  <Comparable<Object>>[18, 2, 8, 'Under the Cloak of War', '', 'Davy Perez', DateTime(2023, 7, 27)],
  <Comparable<Object>>[
    19,
    2,
    9,
    'Subspace Rhapsody',
    '',
    'Dana Horgan, Bill Wolkoff',
    DateTime(2023, 8, 3),
  ],
  <Comparable<Object>>[20, 2, 10, 'Hegemony', '', 'Henry Alonso Myers', DateTime(2023, 8, 10)],
];
