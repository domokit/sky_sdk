// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@TestOn('!chrome')
library;

import 'dart:math' as math;

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:vector_math/vector_math_64.dart' show Matrix3;

import 'data_table_test_utils.dart';

void main() {
  testWidgets('DataTable control test', (WidgetTester tester) async {
    final List<String> log = <String>[];

    Widget buildTable({int? sortColumnIndex, bool sortAscending = true}) {
      return DataTable(
        sortColumnIndex: sortColumnIndex,
        sortAscending: sortAscending,
        onSelectAll: (bool? value) {
          log.add('select-all: $value');
        },
        columns: <DataColumn>[
          const DataColumn(label: Text('Name'), tooltip: 'Name'),
          DataColumn(
            label: const Text('Calories'),
            tooltip: 'Calories',
            numeric: true,
            onSort: (int columnIndex, bool ascending) {
              log.add('column-sort: $columnIndex $ascending');
            },
          ),
        ],
        rows:
            kDesserts.map<DataRow>((Dessert dessert) {
              return DataRow(
                key: ValueKey<String>(dessert.name),
                onSelectChanged: (bool? selected) {
                  log.add('row-selected: ${dessert.name}');
                },
                onLongPress: () {
                  log.add('onLongPress: ${dessert.name}');
                },
                cells: <DataCell>[
                  DataCell(Text(dessert.name)),
                  DataCell(
                    Text('${dessert.calories}'),
                    showEditIcon: true,
                    onTap: () {
                      log.add('cell-tap: ${dessert.calories}');
                    },
                    onDoubleTap: () {
                      log.add('cell-doubleTap: ${dessert.calories}');
                    },
                    onLongPress: () {
                      log.add('cell-longPress: ${dessert.calories}');
                    },
                    onTapCancel: () {
                      log.add('cell-tapCancel: ${dessert.calories}');
                    },
                    onTapDown: (TapDownDetails details) {
                      log.add('cell-tapDown: ${dessert.calories}');
                    },
                  ),
                ],
              );
            }).toList(),
      );
    }

    await tester.pumpWidget(MaterialApp(home: Material(child: buildTable())));

    await tester.tap(find.byType(Checkbox).first);

    expect(log, <String>['select-all: true']);
    log.clear();

    await tester.tap(find.text('Cupcake'));

    expect(log, <String>['row-selected: Cupcake']);
    log.clear();

    await tester.longPress(find.text('Cupcake'));

    expect(log, <String>['onLongPress: Cupcake']);
    log.clear();

    await tester.tap(find.text('Calories'));

    expect(log, <String>['column-sort: 1 true']);
    log.clear();

    await tester.pumpWidget(MaterialApp(home: Material(child: buildTable(sortColumnIndex: 1))));
    await tester.pumpAndSettle(const Duration(milliseconds: 200));
    await tester.tap(find.text('Calories'));

    expect(log, <String>['column-sort: 1 false']);
    log.clear();

    await tester.pumpWidget(
      MaterialApp(home: Material(child: buildTable(sortColumnIndex: 1, sortAscending: false))),
    );
    await tester.pumpAndSettle(const Duration(milliseconds: 200));

    await tester.tap(find.text('375'));
    await tester.pump(const Duration(milliseconds: 100));
    await tester.tap(find.text('375'));

    expect(log, <String>['cell-doubleTap: 375']);
    log.clear();

    await tester.longPress(find.text('375'));
    // The tap down is triggered on gesture down.
    // Then, the cancel is triggered when the gesture arena
    // recognizes that the long press overrides the tap event
    // so it triggers a tap cancel, followed by the long press.
    expect(log, <String>['cell-tapDown: 375', 'cell-tapCancel: 375', 'cell-longPress: 375']);
    log.clear();

    TestGesture gesture = await tester.startGesture(tester.getRect(find.text('375')).center);
    await tester.pump(const Duration(milliseconds: 100));
    // onTapDown callback is registered.
    expect(log, equals(<String>['cell-tapDown: 375']));
    await gesture.up();

    await tester.pump(const Duration(seconds: 1));
    // onTap callback is registered after the gesture is removed.
    expect(log, equals(<String>['cell-tapDown: 375', 'cell-tap: 375']));
    log.clear();

    // dragging off the bounds of the cell calls the cancel callback
    gesture = await tester.startGesture(tester.getRect(find.text('375')).center);
    await tester.pump(const Duration(milliseconds: 100));
    await gesture.moveBy(const Offset(0.0, 200.0));
    await gesture.cancel();
    expect(log, equals(<String>['cell-tapDown: 375', 'cell-tapCancel: 375']));

    log.clear();

    await tester.tap(find.byType(Checkbox).last);

    expect(log, <String>['row-selected: KitKat']);
    log.clear();
  });

  testWidgets('DataTable control test - tristate', (WidgetTester tester) async {
    final List<String> log = <String>[];
    const int numItems = 3;
    Widget buildTable(List<bool> selected, {int? disabledIndex}) {
      return DataTable(
        onSelectAll: (bool? value) {
          log.add('select-all: $value');
        },
        columns: const <DataColumn>[DataColumn(label: Text('Name'), tooltip: 'Name')],
        rows: List<DataRow>.generate(
          numItems,
          (int index) => DataRow(
            cells: <DataCell>[DataCell(Text('Row $index'))],
            selected: selected[index],
            onSelectChanged:
                index == disabledIndex
                    ? null
                    : (bool? value) {
                      log.add('row-selected: $index');
                    },
          ),
        ),
      );
    }

    // Tapping the parent checkbox when no rows are selected, selects all.
    await tester.pumpWidget(
      MaterialApp(home: Material(child: buildTable(<bool>[false, false, false]))),
    );
    await tester.tap(find.byType(Checkbox).first);

    expect(log, <String>['select-all: true']);
    log.clear();

    // Tapping the parent checkbox when some rows are selected, selects all.
    await tester.pumpWidget(
      MaterialApp(home: Material(child: buildTable(<bool>[true, false, true]))),
    );
    await tester.tap(find.byType(Checkbox).first);

    expect(log, <String>['select-all: true']);
    log.clear();

    // Tapping the parent checkbox when all rows are selected, deselects all.
    await tester.pumpWidget(
      MaterialApp(home: Material(child: buildTable(<bool>[true, true, true]))),
    );
    await tester.tap(find.byType(Checkbox).first);

    expect(log, <String>['select-all: false']);
    log.clear();

    // Tapping the parent checkbox when all rows are selected and one is
    // disabled, deselects all.
    await tester.pumpWidget(
      MaterialApp(home: Material(child: buildTable(<bool>[true, true, false], disabledIndex: 2))),
    );
    await tester.tap(find.byType(Checkbox).first);

    expect(log, <String>['select-all: false']);
    log.clear();
  });

  testWidgets('DataTable control test - no checkboxes', (WidgetTester tester) async {
    final List<String> log = <String>[];

    Widget buildTable({bool checkboxes = false}) {
      return DataTable(
        showCheckboxColumn: checkboxes,
        onSelectAll: (bool? value) {
          log.add('select-all: $value');
        },
        columns: const <DataColumn>[
          DataColumn(label: Text('Name'), tooltip: 'Name'),
          DataColumn(label: Text('Calories'), tooltip: 'Calories', numeric: true),
        ],
        rows:
            kDesserts.map<DataRow>((Dessert dessert) {
              return DataRow(
                key: ValueKey<String>(dessert.name),
                onSelectChanged: (bool? selected) {
                  log.add('row-selected: ${dessert.name}');
                },
                cells: <DataCell>[
                  DataCell(Text(dessert.name)),
                  DataCell(
                    Text('${dessert.calories}'),
                    showEditIcon: true,
                    onTap: () {
                      log.add('cell-tap: ${dessert.calories}');
                    },
                  ),
                ],
              );
            }).toList(),
      );
    }

    await tester.pumpWidget(MaterialApp(home: Material(child: buildTable())));

    expect(find.byType(Checkbox), findsNothing);
    await tester.tap(find.text('Cupcake'));

    expect(log, <String>['row-selected: Cupcake']);
    log.clear();

    await tester.pumpWidget(MaterialApp(home: Material(child: buildTable(checkboxes: true))));

    await tester.pumpAndSettle(const Duration(milliseconds: 200));
    final Finder checkboxes = find.byType(Checkbox);
    expect(checkboxes, findsNWidgets(11));
    await tester.tap(checkboxes.first);

    expect(log, <String>['select-all: true']);
    log.clear();
  });

  testWidgets('DataTable overflow test - header', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: DataTable(
            headingTextStyle: const TextStyle(
              fontSize: 14.0,
              letterSpacing: 0.0, // Will overflow if letter spacing is larger than 0.0.
            ),
            columns: <DataColumn>[DataColumn(label: Text('X' * 2000))],
            rows: const <DataRow>[
              DataRow(cells: <DataCell>[DataCell(Text('X'))]),
            ],
          ),
        ),
      ),
    );

    expect(tester.renderObject<RenderBox>(find.byType(Text).first).size.width, greaterThan(800.0));
    expect(tester.renderObject<RenderBox>(find.byType(Row).first).size.width, greaterThan(800.0));
    expect(
      tester.takeException(),
      isNull,
    ); // column overflows table, but text doesn't overflow cell
  });

  testWidgets('DataTable overflow test - header with spaces', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: DataTable(
            columns: <DataColumn>[
              DataColumn(
                label: Text('X ' * 2000), // has soft wrap points, but they should be ignored
              ),
            ],
            rows: const <DataRow>[
              DataRow(cells: <DataCell>[DataCell(Text('X'))]),
            ],
          ),
        ),
      ),
    );
    expect(tester.renderObject<RenderBox>(find.byType(Text).first).size.width, greaterThan(800.0));
    expect(tester.renderObject<RenderBox>(find.byType(Row).first).size.width, greaterThan(800.0));
    expect(
      tester.takeException(),
      isNull,
    ); // column overflows table, but text doesn't overflow cell
  }, skip: true); // https://github.com/flutter/flutter/issues/13512

  testWidgets('DataTable overflow test', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: DataTable(
            columns: const <DataColumn>[DataColumn(label: Text('X'))],
            rows: <DataRow>[
              DataRow(cells: <DataCell>[DataCell(Text('X' * 2000))]),
            ],
          ),
        ),
      ),
    );
    expect(tester.renderObject<RenderBox>(find.byType(Text).first).size.width, lessThan(800.0));
    expect(tester.renderObject<RenderBox>(find.byType(Row).first).size.width, greaterThan(800.0));
    expect(tester.takeException(), isNull); // cell overflows table, but text doesn't overflow cell
  });

  testWidgets('DataTable overflow test', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: DataTable(
            columns: const <DataColumn>[DataColumn(label: Text('X'))],
            rows: <DataRow>[
              DataRow(
                cells: <DataCell>[
                  DataCell(
                    Text('X ' * 2000), // wraps
                  ),
                ],
              ),
            ],
          ),
        ),
      ),
    );
    expect(tester.renderObject<RenderBox>(find.byType(Text).first).size.width, lessThan(800.0));
    expect(tester.renderObject<RenderBox>(find.byType(Row).first).size.width, lessThan(800.0));
    expect(tester.takeException(), isNull);
  });

  testWidgets('DataTable column onSort test', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: DataTable(
            columns: const <DataColumn>[DataColumn(label: Text('Dessert'))],
            rows: const <DataRow>[
              DataRow(
                cells: <DataCell>[
                  DataCell(
                    Text('Lollipop'), // wraps
                  ),
                ],
              ),
            ],
          ),
        ),
      ),
    );
    await tester.tap(find.text('Dessert'));
    await tester.pump();
    expect(tester.takeException(), isNull);
  });

  testWidgets('DataTable sort indicator orientation', (WidgetTester tester) async {
    Widget buildTable({bool sortAscending = true}) {
      return DataTable(
        sortColumnIndex: 0,
        sortAscending: sortAscending,
        columns: <DataColumn>[
          DataColumn(
            label: const Text('Name'),
            tooltip: 'Name',
            onSort: (int columnIndex, bool ascending) {},
          ),
        ],
        rows:
            kDesserts.map<DataRow>((Dessert dessert) {
              return DataRow(cells: <DataCell>[DataCell(Text(dessert.name))]);
            }).toList(),
      );
    }

    // Check for ascending list
    await tester.pumpWidget(MaterialApp(home: Material(child: buildTable())));
    final Finder iconFinder = find.descendant(
      of: find.byType(DataTable),
      matching: find.widgetWithIcon(Transform, Icons.arrow_upward),
    );
    // The `tester.widget` ensures that there is exactly one upward arrow.
    Transform transformOfArrow = tester.widget<Transform>(iconFinder);
    expect(transformOfArrow.transform.getRotation(), equals(Matrix3.identity()));

    // Check for descending list.
    await tester.pumpWidget(MaterialApp(home: Material(child: buildTable(sortAscending: false))));
    await tester.pumpAndSettle();
    // The `tester.widget` ensures that there is exactly one upward arrow.
    transformOfArrow = tester.widget<Transform>(iconFinder);
    expect(transformOfArrow.transform.getRotation(), equals(Matrix3.rotationZ(math.pi)));
  });

  testWidgets('DataTable sort indicator orientation does not change on state update', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/43724
    Widget buildTable({String title = 'Name1'}) {
      return DataTable(
        sortColumnIndex: 0,
        columns: <DataColumn>[
          DataColumn(
            label: Text(title),
            tooltip: 'Name',
            onSort: (int columnIndex, bool ascending) {},
          ),
        ],
        rows:
            kDesserts.map<DataRow>((Dessert dessert) {
              return DataRow(cells: <DataCell>[DataCell(Text(dessert.name))]);
            }).toList(),
      );
    }

    // Check for ascending list
    await tester.pumpWidget(MaterialApp(home: Material(child: buildTable())));
    final Finder iconFinder = find.descendant(
      of: find.byType(DataTable),
      matching: find.widgetWithIcon(Transform, Icons.arrow_upward),
    );
    // The `tester.widget` ensures that there is exactly one upward arrow.
    Transform transformOfArrow = tester.widget<Transform>(iconFinder);
    expect(transformOfArrow.transform.getRotation(), equals(Matrix3.identity()));

    // Cause a rebuild by updating the widget
    await tester.pumpWidget(MaterialApp(home: Material(child: buildTable(title: 'Name2'))));
    await tester.pumpAndSettle();
    // The `tester.widget` ensures that there is exactly one upward arrow.
    transformOfArrow = tester.widget<Transform>(iconFinder);
    expect(
      transformOfArrow.transform.getRotation(),
      equals(Matrix3.identity()), // Should not have changed
    );
  });

  testWidgets('DataTable sort indicator orientation does not change on state update - reverse', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/43724
    Widget buildTable({String title = 'Name1'}) {
      return DataTable(
        sortColumnIndex: 0,
        sortAscending: false,
        columns: <DataColumn>[
          DataColumn(
            label: Text(title),
            tooltip: 'Name',
            onSort: (int columnIndex, bool ascending) {},
          ),
        ],
        rows:
            kDesserts.map<DataRow>((Dessert dessert) {
              return DataRow(cells: <DataCell>[DataCell(Text(dessert.name))]);
            }).toList(),
      );
    }

    // Check for ascending list
    await tester.pumpWidget(MaterialApp(home: Material(child: buildTable())));
    final Finder iconFinder = find.descendant(
      of: find.byType(DataTable),
      matching: find.widgetWithIcon(Transform, Icons.arrow_upward),
    );
    // The `tester.widget` ensures that there is exactly one upward arrow.
    Transform transformOfArrow = tester.widget<Transform>(iconFinder);
    expect(transformOfArrow.transform.getRotation(), equals(Matrix3.rotationZ(math.pi)));

    // Cause a rebuild by updating the widget
    await tester.pumpWidget(MaterialApp(home: Material(child: buildTable(title: 'Name2'))));
    await tester.pumpAndSettle();
    // The `tester.widget` ensures that there is exactly one upward arrow.
    transformOfArrow = tester.widget<Transform>(iconFinder);
    expect(
      transformOfArrow.transform.getRotation(),
      equals(Matrix3.rotationZ(math.pi)), // Should not have changed
    );
  });

  testWidgets('DataTable row onSelectChanged test', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: DataTable(
            columns: const <DataColumn>[DataColumn(label: Text('Dessert'))],
            rows: const <DataRow>[
              DataRow(
                cells: <DataCell>[
                  DataCell(
                    Text('Lollipop'), // wraps
                  ),
                ],
              ),
            ],
          ),
        ),
      ),
    );
    await tester.tap(find.text('Lollipop'));
    await tester.pump();
    expect(tester.takeException(), isNull);
  });

  testWidgets('DataTable custom row height', (WidgetTester tester) async {
    Widget buildCustomTable({
      int? sortColumnIndex,
      bool sortAscending = true,
      double? dataRowMinHeight,
      double? dataRowMaxHeight,
      double headingRowHeight = 56.0,
    }) {
      return DataTable(
        sortColumnIndex: sortColumnIndex,
        sortAscending: sortAscending,
        onSelectAll: (bool? value) {},
        dataRowMinHeight: dataRowMinHeight,
        dataRowMaxHeight: dataRowMaxHeight,
        headingRowHeight: headingRowHeight,
        columns: <DataColumn>[
          const DataColumn(label: Text('Name'), tooltip: 'Name'),
          DataColumn(
            label: const Text('Calories'),
            tooltip: 'Calories',
            numeric: true,
            onSort: (int columnIndex, bool ascending) {},
          ),
        ],
        rows:
            kDesserts.map<DataRow>((Dessert dessert) {
              return DataRow(
                key: ValueKey<String>(dessert.name),
                onSelectChanged: (bool? selected) {},
                cells: <DataCell>[
                  DataCell(Text(dessert.name)),
                  DataCell(Text('${dessert.calories}'), showEditIcon: true, onTap: () {}),
                ],
              );
            }).toList(),
      );
    }

    // DEFAULT VALUES
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: DataTable(
            onSelectAll: (bool? value) {},
            columns: <DataColumn>[
              const DataColumn(label: Text('Name'), tooltip: 'Name'),
              DataColumn(
                label: const Text('Calories'),
                tooltip: 'Calories',
                numeric: true,
                onSort: (int columnIndex, bool ascending) {},
              ),
            ],
            rows:
                kDesserts.map<DataRow>((Dessert dessert) {
                  return DataRow(
                    key: ValueKey<String>(dessert.name),
                    onSelectChanged: (bool? selected) {},
                    cells: <DataCell>[
                      DataCell(Text(dessert.name)),
                      DataCell(Text('${dessert.calories}'), showEditIcon: true, onTap: () {}),
                    ],
                  );
                }).toList(),
          ),
        ),
      ),
    );

    // The finder matches with the Container of the cell content, as well as the
    // Container wrapping the whole table. The first one is used to test row
    // heights.
    Finder findFirstContainerFor(String text) => find.widgetWithText(Container, text).first;

    expect(tester.getSize(findFirstContainerFor('Name')).height, 56.0);
    expect(tester.getSize(findFirstContainerFor('Frozen yogurt')).height, kMinInteractiveDimension);

    // CUSTOM VALUES
    await tester.pumpWidget(
      MaterialApp(home: Material(child: buildCustomTable(headingRowHeight: 48.0))),
    );
    expect(tester.getSize(findFirstContainerFor('Name')).height, 48.0);

    await tester.pumpWidget(
      MaterialApp(home: Material(child: buildCustomTable(headingRowHeight: 64.0))),
    );
    expect(tester.getSize(findFirstContainerFor('Name')).height, 64.0);

    await tester.pumpWidget(
      MaterialApp(
        home: Material(child: buildCustomTable(dataRowMinHeight: 30.0, dataRowMaxHeight: 30.0)),
      ),
    );
    expect(tester.getSize(findFirstContainerFor('Frozen yogurt')).height, 30.0);

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: buildCustomTable(dataRowMinHeight: 0.0, dataRowMaxHeight: double.infinity),
        ),
      ),
    );
    expect(tester.getSize(findFirstContainerFor('Frozen yogurt')).height, greaterThan(0.0));
  });

  testWidgets('DataTable custom row height one row taller than others', (
    WidgetTester tester,
  ) async {
    const String multilineText = 'Line one.\nLine two.\nLine three.\nLine four.';

    Widget buildCustomTable({double? dataRowMinHeight, double? dataRowMaxHeight}) {
      return DataTable(
        dataRowMinHeight: dataRowMinHeight,
        dataRowMaxHeight: dataRowMaxHeight,
        columns: const <DataColumn>[
          DataColumn(label: Text('SingleRowColumn')),
          DataColumn(label: Text('MultiRowColumn')),
        ],
        rows: const <DataRow>[
          DataRow(
            cells: <DataCell>[
              DataCell(Text('Data')),
              DataCell(Column(children: <Widget>[Text(multilineText)])),
            ],
          ),
        ],
      );
    }

    Finder findFirstContainerFor(String text) => find.widgetWithText(Container, text).first;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: buildCustomTable(dataRowMinHeight: 0.0, dataRowMaxHeight: double.infinity),
        ),
      ),
    );

    final double singleLineRowHeight = tester.getSize(findFirstContainerFor('Data')).height;
    final double multilineRowHeight = tester.getSize(findFirstContainerFor(multilineText)).height;

    expect(multilineRowHeight, greaterThan(singleLineRowHeight));
  });

  testWidgets('DataTable custom row height - separate test for deprecated dataRowHeight', (
    WidgetTester tester,
  ) async {
    Widget buildCustomTable({double dataRowHeight = 48.0}) {
      return DataTable(
        onSelectAll: (bool? value) {},
        dataRowHeight: dataRowHeight,
        columns: <DataColumn>[
          const DataColumn(label: Text('Name'), tooltip: 'Name'),
          DataColumn(
            label: const Text('Calories'),
            tooltip: 'Calories',
            numeric: true,
            onSort: (int columnIndex, bool ascending) {},
          ),
        ],
        rows:
            kDesserts.map<DataRow>((Dessert dessert) {
              return DataRow(
                key: ValueKey<String>(dessert.name),
                onSelectChanged: (bool? selected) {},
                cells: <DataCell>[
                  DataCell(Text(dessert.name)),
                  DataCell(Text('${dessert.calories}'), showEditIcon: true, onTap: () {}),
                ],
              );
            }).toList(),
      );
    }

    // The finder matches with the Container of the cell content, as well as the
    // Container wrapping the whole table. The first one is used to test row
    // heights.
    Finder findFirstContainerFor(String text) => find.widgetWithText(Container, text).first;

    // CUSTOM VALUES
    await tester.pumpWidget(
      MaterialApp(home: Material(child: buildCustomTable(dataRowHeight: 30.0))),
    );
    expect(tester.getSize(findFirstContainerFor('Frozen yogurt')).height, 30.0);
  });

  testWidgets('DataTable custom horizontal padding - checkbox', (WidgetTester tester) async {
    const double defaultHorizontalMargin = 24.0;
    const double defaultColumnSpacing = 56.0;
    const double customHorizontalMargin = 10.0;
    const double customColumnSpacing = 15.0;
    Finder cellContent;
    Finder checkbox;
    Finder padding;

    Widget buildDefaultTable({int? sortColumnIndex, bool sortAscending = true}) {
      return DataTable(
        sortColumnIndex: sortColumnIndex,
        sortAscending: sortAscending,
        onSelectAll: (bool? value) {},
        columns: <DataColumn>[
          const DataColumn(label: Text('Name'), tooltip: 'Name'),
          DataColumn(
            label: const Text('Calories'),
            tooltip: 'Calories',
            numeric: true,
            onSort: (int columnIndex, bool ascending) {},
          ),
          DataColumn(
            label: const Text('Fat'),
            tooltip: 'Fat',
            numeric: true,
            onSort: (int columnIndex, bool ascending) {},
          ),
        ],
        rows:
            kDesserts.map<DataRow>((Dessert dessert) {
              return DataRow(
                key: ValueKey<String>(dessert.name),
                onSelectChanged: (bool? selected) {},
                cells: <DataCell>[
                  DataCell(Text(dessert.name)),
                  DataCell(Text('${dessert.calories}'), showEditIcon: true, onTap: () {}),
                  DataCell(Text('${dessert.fat}'), showEditIcon: true, onTap: () {}),
                ],
              );
            }).toList(),
      );
    }

    // DEFAULT VALUES
    await tester.pumpWidget(MaterialApp(home: Material(child: buildDefaultTable())));

    // default checkbox padding
    checkbox = find.byType(Checkbox).first;
    padding = find.ancestor(of: checkbox, matching: find.byType(Padding));
    expect(tester.getRect(checkbox).left - tester.getRect(padding).left, defaultHorizontalMargin);
    expect(
      tester.getRect(padding).right - tester.getRect(checkbox).right,
      defaultHorizontalMargin / 2,
    );

    // default first column padding
    padding = find.widgetWithText(Padding, 'Frozen yogurt');
    cellContent = find.widgetWithText(
      Align,
      'Frozen yogurt',
    ); // DataTable wraps its DataCells in an Align widget
    expect(
      tester.getRect(cellContent).left - tester.getRect(padding).left,
      defaultHorizontalMargin / 2,
    );
    expect(
      tester.getRect(padding).right - tester.getRect(cellContent).right,
      defaultColumnSpacing / 2,
    );

    // default middle column padding
    padding = find.widgetWithText(Padding, '159');
    cellContent = find.widgetWithText(Align, '159');
    expect(
      tester.getRect(cellContent).left - tester.getRect(padding).left,
      defaultColumnSpacing / 2,
    );
    expect(
      tester.getRect(padding).right - tester.getRect(cellContent).right,
      defaultColumnSpacing / 2,
    );

    // default last column padding
    padding = find.widgetWithText(Padding, '6.0');
    cellContent = find.widgetWithText(Align, '6.0');
    expect(
      tester.getRect(cellContent).left - tester.getRect(padding).left,
      defaultColumnSpacing / 2,
    );
    expect(
      tester.getRect(padding).right - tester.getRect(cellContent).right,
      defaultHorizontalMargin,
    );

    Widget buildCustomTable({
      int? sortColumnIndex,
      bool sortAscending = true,
      double? horizontalMargin,
      double? columnSpacing,
    }) {
      return DataTable(
        sortColumnIndex: sortColumnIndex,
        sortAscending: sortAscending,
        onSelectAll: (bool? value) {},
        horizontalMargin: horizontalMargin,
        columnSpacing: columnSpacing,
        columns: <DataColumn>[
          const DataColumn(label: Text('Name'), tooltip: 'Name'),
          DataColumn(
            label: const Text('Calories'),
            tooltip: 'Calories',
            numeric: true,
            onSort: (int columnIndex, bool ascending) {},
          ),
          DataColumn(
            label: const Text('Fat'),
            tooltip: 'Fat',
            numeric: true,
            onSort: (int columnIndex, bool ascending) {},
          ),
        ],
        rows:
            kDesserts.map<DataRow>((Dessert dessert) {
              return DataRow(
                key: ValueKey<String>(dessert.name),
                onSelectChanged: (bool? selected) {},
                cells: <DataCell>[
                  DataCell(Text(dessert.name)),
                  DataCell(Text('${dessert.calories}'), showEditIcon: true, onTap: () {}),
                  DataCell(Text('${dessert.fat}'), showEditIcon: true, onTap: () {}),
                ],
              );
            }).toList(),
      );
    }

    // CUSTOM VALUES
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: buildCustomTable(
            horizontalMargin: customHorizontalMargin,
            columnSpacing: customColumnSpacing,
          ),
        ),
      ),
    );

    // custom checkbox padding
    checkbox = find.byType(Checkbox).first;
    padding = find.ancestor(of: checkbox, matching: find.byType(Padding));
    expect(tester.getRect(checkbox).left - tester.getRect(padding).left, customHorizontalMargin);
    expect(
      tester.getRect(padding).right - tester.getRect(checkbox).right,
      customHorizontalMargin / 2,
    );

    // custom first column padding
    padding = find.widgetWithText(Padding, 'Frozen yogurt').first;
    cellContent = find.widgetWithText(
      Align,
      'Frozen yogurt',
    ); // DataTable wraps its DataCells in an Align widget
    expect(
      tester.getRect(cellContent).left - tester.getRect(padding).left,
      customHorizontalMargin / 2,
    );
    expect(
      tester.getRect(padding).right - tester.getRect(cellContent).right,
      customColumnSpacing / 2,
    );

    // custom middle column padding
    padding = find.widgetWithText(Padding, '159');
    cellContent = find.widgetWithText(Align, '159');
    expect(
      tester.getRect(cellContent).left - tester.getRect(padding).left,
      customColumnSpacing / 2,
    );
    expect(
      tester.getRect(padding).right - tester.getRect(cellContent).right,
      customColumnSpacing / 2,
    );

    // custom last column padding
    padding = find.widgetWithText(Padding, '6.0');
    cellContent = find.widgetWithText(Align, '6.0');
    expect(
      tester.getRect(cellContent).left - tester.getRect(padding).left,
      customColumnSpacing / 2,
    );
    expect(
      tester.getRect(padding).right - tester.getRect(cellContent).right,
      customHorizontalMargin,
    );
  });

  testWidgets('DataTable custom horizontal padding - no checkbox', (WidgetTester tester) async {
    const double defaultHorizontalMargin = 24.0;
    const double defaultColumnSpacing = 56.0;
    const double customHorizontalMargin = 10.0;
    const double customColumnSpacing = 15.0;
    Finder cellContent;
    Finder padding;

    Widget buildDefaultTable({int? sortColumnIndex, bool sortAscending = true}) {
      return DataTable(
        sortColumnIndex: sortColumnIndex,
        sortAscending: sortAscending,
        columns: <DataColumn>[
          const DataColumn(label: Text('Name'), tooltip: 'Name'),
          DataColumn(
            label: const Text('Calories'),
            tooltip: 'Calories',
            numeric: true,
            onSort: (int columnIndex, bool ascending) {},
          ),
          DataColumn(
            label: const Text('Fat'),
            tooltip: 'Fat',
            numeric: true,
            onSort: (int columnIndex, bool ascending) {},
          ),
        ],
        rows:
            kDesserts.map<DataRow>((Dessert dessert) {
              return DataRow(
                key: ValueKey<String>(dessert.name),
                cells: <DataCell>[
                  DataCell(Text(dessert.name)),
                  DataCell(Text('${dessert.calories}'), showEditIcon: true, onTap: () {}),
                  DataCell(Text('${dessert.fat}'), showEditIcon: true, onTap: () {}),
                ],
              );
            }).toList(),
      );
    }

    // DEFAULT VALUES
    await tester.pumpWidget(MaterialApp(home: Material(child: buildDefaultTable())));

    // default first column padding
    padding = find.widgetWithText(Padding, 'Frozen yogurt');
    cellContent = find.widgetWithText(
      Align,
      'Frozen yogurt',
    ); // DataTable wraps its DataCells in an Align widget
    expect(
      tester.getRect(cellContent).left - tester.getRect(padding).left,
      defaultHorizontalMargin,
    );
    expect(
      tester.getRect(padding).right - tester.getRect(cellContent).right,
      defaultColumnSpacing / 2,
    );

    // default middle column padding
    padding = find.widgetWithText(Padding, '159');
    cellContent = find.widgetWithText(Align, '159');
    expect(
      tester.getRect(cellContent).left - tester.getRect(padding).left,
      defaultColumnSpacing / 2,
    );
    expect(
      tester.getRect(padding).right - tester.getRect(cellContent).right,
      defaultColumnSpacing / 2,
    );

    // default last column padding
    padding = find.widgetWithText(Padding, '6.0');
    cellContent = find.widgetWithText(Align, '6.0');
    expect(
      tester.getRect(cellContent).left - tester.getRect(padding).left,
      defaultColumnSpacing / 2,
    );
    expect(
      tester.getRect(padding).right - tester.getRect(cellContent).right,
      defaultHorizontalMargin,
    );

    Widget buildCustomTable({
      int? sortColumnIndex,
      bool sortAscending = true,
      double? horizontalMargin,
      double? columnSpacing,
    }) {
      return DataTable(
        sortColumnIndex: sortColumnIndex,
        sortAscending: sortAscending,
        horizontalMargin: horizontalMargin,
        columnSpacing: columnSpacing,
        columns: <DataColumn>[
          const DataColumn(label: Text('Name'), tooltip: 'Name'),
          DataColumn(
            label: const Text('Calories'),
            tooltip: 'Calories',
            numeric: true,
            onSort: (int columnIndex, bool ascending) {},
          ),
          DataColumn(
            label: const Text('Fat'),
            tooltip: 'Fat',
            numeric: true,
            onSort: (int columnIndex, bool ascending) {},
          ),
        ],
        rows:
            kDesserts.map<DataRow>((Dessert dessert) {
              return DataRow(
                key: ValueKey<String>(dessert.name),
                cells: <DataCell>[
                  DataCell(Text(dessert.name)),
                  DataCell(Text('${dessert.calories}'), showEditIcon: true, onTap: () {}),
                  DataCell(Text('${dessert.fat}'), showEditIcon: true, onTap: () {}),
                ],
              );
            }).toList(),
      );
    }

    // CUSTOM VALUES
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: buildCustomTable(
            horizontalMargin: customHorizontalMargin,
            columnSpacing: customColumnSpacing,
          ),
        ),
      ),
    );

    // custom first column padding
    padding = find.widgetWithText(Padding, 'Frozen yogurt');
    cellContent = find.widgetWithText(
      Align,
      'Frozen yogurt',
    ); // DataTable wraps its DataCells in an Align widget
    expect(tester.getRect(cellContent).left - tester.getRect(padding).left, customHorizontalMargin);
    expect(
      tester.getRect(padding).right - tester.getRect(cellContent).right,
      customColumnSpacing / 2,
    );

    // custom middle column padding
    padding = find.widgetWithText(Padding, '159');
    cellContent = find.widgetWithText(Align, '159');
    expect(
      tester.getRect(cellContent).left - tester.getRect(padding).left,
      customColumnSpacing / 2,
    );
    expect(
      tester.getRect(padding).right - tester.getRect(cellContent).right,
      customColumnSpacing / 2,
    );

    // custom last column padding
    padding = find.widgetWithText(Padding, '6.0');
    cellContent = find.widgetWithText(Align, '6.0');
    expect(
      tester.getRect(cellContent).left - tester.getRect(padding).left,
      customColumnSpacing / 2,
    );
    expect(
      tester.getRect(padding).right - tester.getRect(cellContent).right,
      customHorizontalMargin,
    );
  });

  testWidgets('DataTable set border width test', (WidgetTester tester) async {
    const List<DataColumn> columns = <DataColumn>[
      DataColumn(label: Text('column1')),
      DataColumn(label: Text('column2')),
    ];

    const List<DataCell> cells = <DataCell>[DataCell(Text('cell1')), DataCell(Text('cell2'))];

    const List<DataRow> rows = <DataRow>[DataRow(cells: cells), DataRow(cells: cells)];

    // no thickness provided - border should be default: i.e "1.0" as it
    // set in DataTable constructor
    await tester.pumpWidget(
      MaterialApp(home: Material(child: DataTable(columns: columns, rows: rows))),
    );

    Table table = tester.widget(find.byType(Table));
    TableRow tableRow = table.children.last;
    BoxDecoration boxDecoration = tableRow.decoration! as BoxDecoration;
    expect(boxDecoration.border!.top.width, 1.0);

    const double thickness = 4.2;
    await tester.pumpWidget(
      MaterialApp(
        home: Material(child: DataTable(dividerThickness: thickness, columns: columns, rows: rows)),
      ),
    );
    table = tester.widget(find.byType(Table));
    tableRow = table.children.last;
    boxDecoration = tableRow.decoration! as BoxDecoration;
    expect(boxDecoration.border!.top.width, thickness);
  });

  testWidgets('DataTable set show bottom border', (WidgetTester tester) async {
    const List<DataColumn> columns = <DataColumn>[
      DataColumn(label: Text('column1')),
      DataColumn(label: Text('column2')),
    ];

    const List<DataCell> cells = <DataCell>[DataCell(Text('cell1')), DataCell(Text('cell2'))];

    const List<DataRow> rows = <DataRow>[DataRow(cells: cells), DataRow(cells: cells)];

    await tester.pumpWidget(
      MaterialApp(
        home: Material(child: DataTable(showBottomBorder: true, columns: columns, rows: rows)),
      ),
    );

    Table table = tester.widget(find.byType(Table));
    TableRow tableRow = table.children.last;
    BoxDecoration boxDecoration = tableRow.decoration! as BoxDecoration;
    expect(boxDecoration.border!.bottom.width, 1.0);

    await tester.pumpWidget(
      MaterialApp(home: Material(child: DataTable(columns: columns, rows: rows))),
    );
    table = tester.widget(find.byType(Table));
    tableRow = table.children.last;
    boxDecoration = tableRow.decoration! as BoxDecoration;
    expect(boxDecoration.border!.bottom.width, 0.0);
  });

  testWidgets('DataTable column heading cell - with and without sorting', (
    WidgetTester tester,
  ) async {
    Widget buildTable({int? sortColumnIndex, bool sortEnabled = true}) {
      return DataTable(
        sortColumnIndex: sortColumnIndex,
        columns: <DataColumn>[
          DataColumn(
            label: const Expanded(child: Center(child: Text('Name'))),
            tooltip: 'Name',
            onSort: sortEnabled ? (_, __) {} : null,
          ),
        ],
        rows: const <DataRow>[
          DataRow(cells: <DataCell>[DataCell(Text('A long desert name'))]),
        ],
      );
    }

    // Start with without sorting
    await tester.pumpWidget(MaterialApp(home: Material(child: buildTable(sortEnabled: false))));

    {
      final Finder nameText = find.text('Name');
      expect(nameText, findsOneWidget);
      final Finder nameCell =
          find.ancestor(of: find.text('Name'), matching: find.byType(Container)).first;
      expect(tester.getCenter(nameText), equals(tester.getCenter(nameCell)));
      expect(find.descendant(of: nameCell, matching: find.byType(Icon)), findsNothing);
    }

    // Turn on sorting
    await tester.pumpWidget(MaterialApp(home: Material(child: buildTable())));

    {
      final Finder nameText = find.text('Name');
      expect(nameText, findsOneWidget);
      final Finder nameCell =
          find.ancestor(of: find.text('Name'), matching: find.byType(Container)).first;
      expect(find.descendant(of: nameCell, matching: find.byType(Icon)), findsOneWidget);
    }

    // Turn off sorting again
    await tester.pumpWidget(MaterialApp(home: Material(child: buildTable(sortEnabled: false))));

    {
      final Finder nameText = find.text('Name');
      expect(nameText, findsOneWidget);
      final Finder nameCell =
          find.ancestor(of: find.text('Name'), matching: find.byType(Container)).first;
      expect(tester.getCenter(nameText), equals(tester.getCenter(nameCell)));
      expect(find.descendant(of: nameCell, matching: find.byType(Icon)), findsNothing);
    }
  });

  testWidgets('DataTable correctly renders with a mouse', (WidgetTester tester) async {
    // Regression test for a bug described in
    // https://github.com/flutter/flutter/pull/43735#issuecomment-589459947
    // Filed at https://github.com/flutter/flutter/issues/51152
    Widget buildTable({int? sortColumnIndex}) {
      return DataTable(
        sortColumnIndex: sortColumnIndex,
        columns: <DataColumn>[
          const DataColumn(
            label: Expanded(child: Center(child: Text('column1'))),
            tooltip: 'Column1',
          ),
          DataColumn(
            label: const Expanded(child: Center(child: Text('column2'))),
            tooltip: 'Column2',
            onSort: (_, __) {},
          ),
        ],
        rows: const <DataRow>[
          DataRow(cells: <DataCell>[DataCell(Text('Content1')), DataCell(Text('Content2'))]),
        ],
      );
    }

    await tester.pumpWidget(MaterialApp(home: Material(child: buildTable())));

    expect(tester.renderObject(find.text('column1')).attached, true);
    expect(tester.renderObject(find.text('column2')).attached, true);

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: Offset.zero);

    await tester.pumpAndSettle();
    expect(tester.renderObject(find.text('column1')).attached, true);
    expect(tester.renderObject(find.text('column2')).attached, true);

    // Wait for the tooltip timer to expire to prevent it scheduling a new frame
    // after the view is destroyed, which causes exceptions.
    await tester.pumpAndSettle(const Duration(seconds: 1));
  });

  testWidgets('DataRow renders default selected row colors', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData.light();
    Widget buildTable({bool selected = false}) {
      return MaterialApp(
        theme: themeData,
        home: Material(
          child: DataTable(
            columns: const <DataColumn>[DataColumn(label: Text('Column1'))],
            rows: <DataRow>[
              DataRow(
                onSelectChanged: (bool? checked) {},
                selected: selected,
                cells: const <DataCell>[DataCell(Text('Content1'))],
              ),
            ],
          ),
        ),
      );
    }

    BoxDecoration lastTableRowBoxDecoration() {
      final Table table = tester.widget(find.byType(Table));
      final TableRow tableRow = table.children.last;
      return tableRow.decoration! as BoxDecoration;
    }

    await tester.pumpWidget(buildTable());
    expect(lastTableRowBoxDecoration().color, null);

    await tester.pumpWidget(buildTable(selected: true));
    expect(lastTableRowBoxDecoration().color, themeData.colorScheme.primary.withOpacity(0.08));
  });

  testWidgets('DataRow renders checkbox with colors from CheckboxTheme', (
    WidgetTester tester,
  ) async {
    const Color fillColor = Color(0xFF00FF00);
    const Color checkColor = Color(0xFF0000FF);

    final ThemeData themeData = ThemeData(
      checkboxTheme: const CheckboxThemeData(
        fillColor: MaterialStatePropertyAll<Color?>(fillColor),
        checkColor: MaterialStatePropertyAll<Color?>(checkColor),
      ),
    );
    Widget buildTable() {
      return MaterialApp(
        theme: themeData,
        home: Material(
          child: DataTable(
            columns: const <DataColumn>[DataColumn(label: Text('Column1'))],
            rows: <DataRow>[
              DataRow(
                selected: true,
                onSelectChanged: (bool? checked) {},
                cells: const <DataCell>[DataCell(Text('Content1'))],
              ),
            ],
          ),
        ),
      );
    }

    await tester.pumpWidget(buildTable());

    expect(
      Material.of(tester.element(find.byType(Checkbox).last)),
      paints
        ..path()
        ..path(color: fillColor)
        ..path(color: checkColor),
    );
  });

  testWidgets('DataRow renders custom colors when selected', (WidgetTester tester) async {
    const Color selectedColor = Colors.green;
    const Color defaultColor = Colors.red;

    Widget buildTable({bool selected = false}) {
      return Material(
        child: DataTable(
          columns: const <DataColumn>[DataColumn(label: Text('Column1'))],
          rows: <DataRow>[
            DataRow(
              selected: selected,
              color: MaterialStateProperty.resolveWith<Color>((Set<MaterialState> states) {
                if (states.contains(MaterialState.selected)) {
                  return selectedColor;
                }
                return defaultColor;
              }),
              cells: const <DataCell>[DataCell(Text('Content1'))],
            ),
          ],
        ),
      );
    }

    BoxDecoration lastTableRowBoxDecoration() {
      final Table table = tester.widget(find.byType(Table));
      final TableRow tableRow = table.children.last;
      return tableRow.decoration! as BoxDecoration;
    }

    await tester.pumpWidget(MaterialApp(home: buildTable()));
    expect(lastTableRowBoxDecoration().color, defaultColor);

    await tester.pumpWidget(MaterialApp(home: buildTable(selected: true)));
    expect(lastTableRowBoxDecoration().color, selectedColor);
  });

  testWidgets('DataRow renders custom colors when disabled', (WidgetTester tester) async {
    const Color disabledColor = Colors.grey;
    const Color defaultColor = Colors.red;

    Widget buildTable({bool disabled = false}) {
      return Material(
        child: DataTable(
          columns: const <DataColumn>[DataColumn(label: Text('Column1'))],
          rows: <DataRow>[
            DataRow(
              cells: const <DataCell>[DataCell(Text('Content1'))],
              onSelectChanged: (bool? value) {},
            ),
            DataRow(
              color: MaterialStateProperty.resolveWith<Color>((Set<MaterialState> states) {
                if (states.contains(MaterialState.disabled)) {
                  return disabledColor;
                }
                return defaultColor;
              }),
              cells: const <DataCell>[DataCell(Text('Content2'))],
              onSelectChanged: disabled ? null : (bool? value) {},
            ),
          ],
        ),
      );
    }

    BoxDecoration lastTableRowBoxDecoration() {
      final Table table = tester.widget(find.byType(Table));
      final TableRow tableRow = table.children.last;
      return tableRow.decoration! as BoxDecoration;
    }

    await tester.pumpWidget(MaterialApp(home: buildTable()));
    expect(lastTableRowBoxDecoration().color, defaultColor);

    await tester.pumpWidget(MaterialApp(home: buildTable(disabled: true)));
    expect(lastTableRowBoxDecoration().color, disabledColor);
  });

  testWidgets('Material2 - DataRow renders custom colors when pressed', (
    WidgetTester tester,
  ) async {
    const Color pressedColor = Color(0xff4caf50);
    Widget buildTable() {
      return DataTable(
        columns: const <DataColumn>[DataColumn(label: Text('Column1'))],
        rows: <DataRow>[
          DataRow(
            color: MaterialStateProperty.resolveWith<Color>((Set<MaterialState> states) {
              if (states.contains(MaterialState.pressed)) {
                return pressedColor;
              }
              return Colors.transparent;
            }),
            onSelectChanged: (bool? value) {},
            cells: const <DataCell>[DataCell(Text('Content1'))],
          ),
        ],
      );
    }

    await tester.pumpWidget(
      MaterialApp(theme: ThemeData(useMaterial3: false), home: Material(child: buildTable())),
    );

    final TestGesture gesture = await tester.startGesture(tester.getCenter(find.text('Content1')));
    await tester.pump(const Duration(milliseconds: 200)); // splash is well underway
    final RenderBox box = Material.of(tester.element(find.byType(InkWell))) as RenderBox;
    expect(box, paints..circle(x: 68.0, y: 24.0, color: pressedColor));
    await gesture.up();
  });

  testWidgets('Material3 - DataRow renders custom colors when pressed', (
    WidgetTester tester,
  ) async {
    const Color pressedColor = Color(0xff4caf50);
    Widget buildTable() {
      return DataTable(
        columns: const <DataColumn>[DataColumn(label: Text('Column1'))],
        rows: <DataRow>[
          DataRow(
            color: MaterialStateProperty.resolveWith<Color>((Set<MaterialState> states) {
              if (states.contains(MaterialState.pressed)) {
                return pressedColor;
              }
              return Colors.transparent;
            }),
            onSelectChanged: (bool? value) {},
            cells: const <DataCell>[DataCell(Text('Content1'))],
          ),
        ],
      );
    }

    await tester.pumpWidget(MaterialApp(theme: ThemeData(), home: Material(child: buildTable())));

    final TestGesture gesture = await tester.startGesture(tester.getCenter(find.text('Content1')));
    await tester.pump(const Duration(milliseconds: 200)); // splash is well underway
    final RenderBox box = Material.of(tester.element(find.byType(InkWell))) as RenderBox;
    // Material 3 uses the InkSparkle which uses a shader, so we can't capture
    // the effect with paint methods.
    expect(
      box,
      paints
        ..rect()
        ..rect(
          rect: const Rect.fromLTRB(0.0, 56.0, 800.0, 104.0),
          color: pressedColor.withOpacity(0.0),
        ),
    );
    await gesture.up();
  });

  testWidgets('DataTable can render inside an AlertDialog', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: AlertDialog(
            content: DataTable(
              columns: const <DataColumn>[DataColumn(label: Text('Col1'))],
              rows: const <DataRow>[
                DataRow(cells: <DataCell>[DataCell(Text('1'))]),
              ],
            ),
            scrollable: true,
          ),
        ),
      ),
    );

    expect(tester.takeException(), isNull);
  });

  testWidgets('DataTable renders with border and background decoration', (
    WidgetTester tester,
  ) async {
    const double width = 800;
    const double height = 600;
    const double borderHorizontal = 5.0;
    const double borderVertical = 10.0;
    const Color borderColor = Color(0xff2196f3);
    const Color backgroundColor = Color(0xfff5f5f5);

    await tester.pumpWidget(
      MaterialApp(
        home: DataTable(
          decoration: const BoxDecoration(
            color: backgroundColor,
            border: Border.symmetric(
              vertical: BorderSide(width: borderVertical, color: borderColor),
              horizontal: BorderSide(width: borderHorizontal, color: borderColor),
            ),
          ),
          columns: const <DataColumn>[DataColumn(label: Text('Col1'))],
          rows: const <DataRow>[
            DataRow(cells: <DataCell>[DataCell(Text('1'))]),
          ],
        ),
      ),
    );

    expect(
      find.ancestor(of: find.byType(Table), matching: find.byType(Container)),
      paints..rect(
        rect: const Rect.fromLTRB(
          borderVertical / 2,
          borderHorizontal / 2,
          width - borderVertical / 2,
          height - borderHorizontal / 2,
        ),
        color: backgroundColor,
      ),
    );
    expect(
      find.ancestor(of: find.byType(Table), matching: find.byType(Container)),
      paints..path(color: borderColor),
    );
    expect(tester.getTopLeft(find.byType(Table)), const Offset(borderVertical, borderHorizontal));
    expect(
      tester.getBottomRight(find.byType(Table)),
      const Offset(width - borderVertical, height - borderHorizontal),
    );
  });

  testWidgets('checkboxHorizontalMargin properly applied', (WidgetTester tester) async {
    const double customCheckboxHorizontalMargin = 15.0;
    const double customHorizontalMargin = 10.0;
    Finder cellContent;
    Finder checkbox;
    Finder padding;

    Widget buildCustomTable({
      int? sortColumnIndex,
      bool sortAscending = true,
      double? horizontalMargin,
      double? checkboxHorizontalMargin,
    }) {
      return DataTable(
        sortColumnIndex: sortColumnIndex,
        sortAscending: sortAscending,
        onSelectAll: (bool? value) {},
        horizontalMargin: horizontalMargin,
        checkboxHorizontalMargin: checkboxHorizontalMargin,
        columns: <DataColumn>[
          const DataColumn(label: Text('Name'), tooltip: 'Name'),
          DataColumn(
            label: const Text('Calories'),
            tooltip: 'Calories',
            numeric: true,
            onSort: (int columnIndex, bool ascending) {},
          ),
          DataColumn(
            label: const Text('Fat'),
            tooltip: 'Fat',
            numeric: true,
            onSort: (int columnIndex, bool ascending) {},
          ),
        ],
        rows:
            kDesserts.map<DataRow>((Dessert dessert) {
              return DataRow(
                key: ValueKey<String>(dessert.name),
                onSelectChanged: (bool? selected) {},
                cells: <DataCell>[
                  DataCell(Text(dessert.name)),
                  DataCell(Text('${dessert.calories}'), showEditIcon: true, onTap: () {}),
                  DataCell(Text('${dessert.fat}'), showEditIcon: true, onTap: () {}),
                ],
              );
            }).toList(),
      );
    }

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: buildCustomTable(
            checkboxHorizontalMargin: customCheckboxHorizontalMargin,
            horizontalMargin: customHorizontalMargin,
          ),
        ),
      ),
    );

    // Custom checkbox padding.
    checkbox = find.byType(Checkbox).first;
    padding = find.ancestor(of: checkbox, matching: find.byType(Padding));
    expect(
      tester.getRect(checkbox).left - tester.getRect(padding).left,
      customCheckboxHorizontalMargin,
    );
    expect(
      tester.getRect(padding).right - tester.getRect(checkbox).right,
      customCheckboxHorizontalMargin,
    );

    // First column padding.
    padding = find.widgetWithText(Padding, 'Frozen yogurt').first;
    cellContent = find.widgetWithText(
      Align,
      'Frozen yogurt',
    ); // DataTable wraps its DataCells in an Align widget.
    expect(tester.getRect(cellContent).left - tester.getRect(padding).left, customHorizontalMargin);
  });

  testWidgets('DataRow is disabled when onSelectChanged is not set', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: DataTable(
            columns: const <DataColumn>[
              DataColumn(label: Text('Col1')),
              DataColumn(label: Text('Col2')),
            ],
            rows: <DataRow>[
              DataRow(
                cells: const <DataCell>[DataCell(Text('Hello')), DataCell(Text('world'))],
                onSelectChanged: (bool? value) {},
              ),
              const DataRow(cells: <DataCell>[DataCell(Text('Bug')), DataCell(Text('report'))]),
              const DataRow(cells: <DataCell>[DataCell(Text('GitHub')), DataCell(Text('issue'))]),
            ],
          ),
        ),
      ),
    );

    expect(find.widgetWithText(TableRowInkWell, 'Hello'), findsOneWidget);
    expect(find.widgetWithText(TableRowInkWell, 'Bug'), findsNothing);
    expect(find.widgetWithText(TableRowInkWell, 'GitHub'), findsNothing);
  });

  testWidgets('DataTable set interior border test', (WidgetTester tester) async {
    const List<DataColumn> columns = <DataColumn>[
      DataColumn(label: Text('column1')),
      DataColumn(label: Text('column2')),
    ];

    const List<DataCell> cells = <DataCell>[DataCell(Text('cell1')), DataCell(Text('cell2'))];

    const List<DataRow> rows = <DataRow>[DataRow(cells: cells), DataRow(cells: cells)];

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: DataTable(
            border: TableBorder.all(width: 2, color: Colors.red),
            columns: columns,
            rows: rows,
          ),
        ),
      ),
    );

    final Finder finder = find.byType(DataTable);
    expect(tester.getSize(finder), equals(const Size(800, 600)));

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: DataTable(
            border: TableBorder.all(color: Colors.red),
            columns: columns,
            rows: rows,
          ),
        ),
      ),
    );

    Table table = tester.widget(find.byType(Table));
    TableBorder? tableBorder = table.border;
    expect(tableBorder!.top.color, Colors.red);
    expect(tableBorder.bottom.width, 1);

    await tester.pumpWidget(
      MaterialApp(home: Material(child: DataTable(columns: columns, rows: rows))),
    );

    table = tester.widget(find.byType(Table));
    tableBorder = table.border;
    expect(tableBorder?.bottom.width, null);
    expect(tableBorder?.top.color, null);
  });

  // Regression test for https://github.com/flutter/flutter/issues/100952
  testWidgets('Do not crashes when paint borders in a narrow space', (WidgetTester tester) async {
    const List<DataColumn> columns = <DataColumn>[
      DataColumn(label: Text('column1')),
      DataColumn(label: Text('column2')),
    ];

    const List<DataCell> cells = <DataCell>[DataCell(Text('cell1')), DataCell(Text('cell2'))];

    const List<DataRow> rows = <DataRow>[DataRow(cells: cells), DataRow(cells: cells)];

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: SizedBox(
              width: 117.0,
              child: DataTable(
                border: TableBorder.all(width: 2, color: Colors.red),
                columns: columns,
                rows: rows,
              ),
            ),
          ),
        ),
      ),
    );

    // Go without crashes.
  });

  testWidgets('DataTable clip behavior', (WidgetTester tester) async {
    const Color selectedColor = Colors.green;
    const Color defaultColor = Colors.red;
    const BorderRadius borderRadius = BorderRadius.all(Radius.circular(30));

    Widget buildTable({bool selected = false, required Clip clipBehavior}) {
      return Material(
        child: DataTable(
          clipBehavior: clipBehavior,
          border: TableBorder.all(borderRadius: borderRadius),
          columns: const <DataColumn>[DataColumn(label: Text('Column1'))],
          rows: <DataRow>[
            DataRow(
              selected: selected,
              color: MaterialStateProperty.resolveWith<Color>((Set<MaterialState> states) {
                if (states.contains(MaterialState.selected)) {
                  return selectedColor;
                }
                return defaultColor;
              }),
              cells: const <DataCell>[DataCell(Text('Content1'))],
            ),
          ],
        ),
      );
    }

    // Test default clip behavior.
    await tester.pumpWidget(MaterialApp(home: buildTable(clipBehavior: Clip.none)));

    Material material = tester.widget<Material>(find.byType(Material).last);
    expect(material.clipBehavior, Clip.none);
    expect(material.borderRadius, borderRadius);

    await tester.pumpWidget(MaterialApp(home: buildTable(clipBehavior: Clip.hardEdge)));

    material = tester.widget<Material>(find.byType(Material).last);
    expect(material.clipBehavior, Clip.hardEdge);
    expect(material.borderRadius, borderRadius);
  });

  testWidgets('DataTable dataRowMinHeight smaller or equal dataRowMaxHeight validation', (
    WidgetTester tester,
  ) async {
    DataTable createDataTable() => DataTable(
      columns: const <DataColumn>[DataColumn(label: Text('Column1'))],
      rows: const <DataRow>[],
      dataRowMinHeight: 2.0,
      dataRowMaxHeight: 1.0,
    );

    expect(
      () => createDataTable(),
      throwsA(
        predicate(
          (AssertionError e) => e.toString().contains('dataRowMaxHeight >= dataRowMinHeight'),
        ),
      ),
    );
  });

  testWidgets(
    'DataTable dataRowHeight is not used together with dataRowMinHeight or dataRowMaxHeight',
    (WidgetTester tester) async {
      DataTable createDataTable({
        double? dataRowHeight,
        double? dataRowMinHeight,
        double? dataRowMaxHeight,
      }) => DataTable(
        columns: const <DataColumn>[DataColumn(label: Text('Column1'))],
        rows: const <DataRow>[],
        dataRowHeight: dataRowHeight,
        dataRowMinHeight: dataRowMinHeight,
        dataRowMaxHeight: dataRowMaxHeight,
      );

      expect(
        () => createDataTable(dataRowHeight: 1.0, dataRowMinHeight: 2.0, dataRowMaxHeight: 2.0),
        throwsA(
          predicate(
            (AssertionError e) => e.toString().contains(
              'dataRowHeight == null || (dataRowMinHeight == null && dataRowMaxHeight == null)',
            ),
          ),
        ),
      );

      expect(
        () => createDataTable(dataRowHeight: 1.0, dataRowMaxHeight: 2.0),
        throwsA(
          predicate(
            (AssertionError e) => e.toString().contains(
              'dataRowHeight == null || (dataRowMinHeight == null && dataRowMaxHeight == null)',
            ),
          ),
        ),
      );

      expect(
        () => createDataTable(dataRowHeight: 1.0, dataRowMinHeight: 2.0),
        throwsA(
          predicate(
            (AssertionError e) => e.toString().contains(
              'dataRowHeight == null || (dataRowMinHeight == null && dataRowMaxHeight == null)',
            ),
          ),
        ),
      );
    },
  );

  group('TableRowInkWell', () {
    testWidgets('can handle secondary taps', (WidgetTester tester) async {
      bool secondaryTapped = false;
      bool secondaryTappedDown = false;

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Table(
              children: <TableRow>[
                TableRow(
                  children: <Widget>[
                    TableRowInkWell(
                      onSecondaryTap: () {
                        secondaryTapped = true;
                      },
                      onSecondaryTapDown: (TapDownDetails details) {
                        secondaryTappedDown = true;
                      },
                      child: const SizedBox(width: 100.0, height: 100.0),
                    ),
                  ],
                ),
              ],
            ),
          ),
        ),
      );

      expect(secondaryTapped, isFalse);
      expect(secondaryTappedDown, isFalse);

      expect(find.byType(TableRowInkWell), findsOneWidget);
      await tester.tap(find.byType(TableRowInkWell), buttons: kSecondaryMouseButton);
      await tester.pumpAndSettle();

      expect(secondaryTapped, isTrue);
      expect(secondaryTappedDown, isTrue);
    });
  });

  testWidgets('Heading cell cursor resolves MaterialStateMouseCursor correctly', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DataTable(
            sortColumnIndex: 0,
            columns: <DataColumn>[
              // This column can be sorted.
              DataColumn(
                mouseCursor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
                  if (states.contains(MaterialState.disabled)) {
                    return SystemMouseCursors.forbidden;
                  }
                  return SystemMouseCursors.copy;
                }),

                onSort: (int columnIndex, bool ascending) {},
                label: const Text('A'),
              ),
              // This column cannot be sorted.
              DataColumn(
                mouseCursor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
                  if (states.contains(MaterialState.disabled)) {
                    return SystemMouseCursors.forbidden;
                  }
                  return SystemMouseCursors.copy;
                }),
                label: const Text('B'),
              ),
            ],
            rows: const <DataRow>[
              DataRow(cells: <DataCell>[DataCell(Text('Data 1')), DataCell(Text('Data 2'))]),
              DataRow(cells: <DataCell>[DataCell(Text('Data 3')), DataCell(Text('Data 4'))]),
            ],
          ),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: tester.getCenter(find.text('A')));
    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.copy,
    );

    await gesture.moveTo(tester.getCenter(find.text('B')));
    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.forbidden,
    );
  });

  testWidgets('DataRow cursor resolves MaterialStateMouseCursor correctly', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DataTable(
            sortColumnIndex: 0,
            columns: <DataColumn>[
              DataColumn(label: const Text('A'), onSort: (int columnIndex, bool ascending) {}),
              const DataColumn(label: Text('B')),
            ],
            rows: <DataRow>[
              // This row can be selected.
              DataRow(
                mouseCursor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
                  if (states.contains(MaterialState.selected)) {
                    return SystemMouseCursors.copy;
                  }
                  return SystemMouseCursors.forbidden;
                }),
                onSelectChanged: (bool? selected) {},
                cells: const <DataCell>[DataCell(Text('Data 1')), DataCell(Text('Data 2'))],
              ),
              // This row is selected.
              DataRow(
                selected: true,
                onSelectChanged: (bool? selected) {},
                mouseCursor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
                  if (states.contains(MaterialState.selected)) {
                    return SystemMouseCursors.copy;
                  }
                  return SystemMouseCursors.forbidden;
                }),
                cells: const <DataCell>[DataCell(Text('Data 3')), DataCell(Text('Data 4'))],
              ),
            ],
          ),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: tester.getCenter(find.text('Data 1')));
    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.forbidden,
    );

    await gesture.moveTo(tester.getCenter(find.text('Data 3')));
    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.copy,
    );
  });

  testWidgets("DataRow cursor doesn't update checkbox cursor", (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DataTable(
            sortColumnIndex: 0,
            columns: <DataColumn>[
              DataColumn(label: const Text('A'), onSort: (int columnIndex, bool ascending) {}),
              const DataColumn(label: Text('B')),
            ],
            rows: <DataRow>[
              DataRow(
                onSelectChanged: (bool? selected) {},
                mouseCursor: const MaterialStatePropertyAll<MouseCursor>(SystemMouseCursors.copy),
                cells: const <DataCell>[DataCell(Text('Data')), DataCell(Text('Data 2'))],
              ),
            ],
          ),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: tester.getCenter(find.byType(Checkbox).last));
    await tester.pump();

    // Test that the checkbox cursor is not changed.
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );

    await gesture.moveTo(tester.getCenter(find.text('Data')));
    await tester.pump();

    // Test that cursor is updated for the row.
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.copy,
    );
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/114470.
  testWidgets('DataTable text styles are merged with default text style', (
    WidgetTester tester,
  ) async {
    late DefaultTextStyle defaultTextStyle;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext context) {
              defaultTextStyle = DefaultTextStyle.of(context);
              return DataTable(
                headingTextStyle: const TextStyle(),
                dataTextStyle: const TextStyle(),
                columns: const <DataColumn>[
                  DataColumn(label: Text('Header 1')),
                  DataColumn(label: Text('Header 2')),
                ],
                rows: const <DataRow>[
                  DataRow(cells: <DataCell>[DataCell(Text('Data 1')), DataCell(Text('Data 2'))]),
                ],
              );
            },
          ),
        ),
      ),
    );

    final TextStyle? headingTextStyle = _getTextRenderObject(tester, 'Header 1').text.style;
    expect(headingTextStyle, defaultTextStyle.style);

    final TextStyle? dataTextStyle = _getTextRenderObject(tester, 'Data 1').text.style;
    expect(dataTextStyle, defaultTextStyle.style);
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/143340.
  testWidgets('DataColumn label can be centered', (WidgetTester tester) async {
    const double horizontalMargin = 24.0;

    Widget buildTable({MainAxisAlignment? headingRowAlignment, bool sortEnabled = false}) {
      return MaterialApp(
        home: Material(
          child: DataTable(
            columns: <DataColumn>[
              DataColumn(
                headingRowAlignment: headingRowAlignment,
                onSort: sortEnabled ? (int columnIndex, bool ascending) {} : null,
                label: const Text('Header'),
              ),
            ],
            rows: const <DataRow>[
              DataRow(cells: <DataCell>[DataCell(Text('Data'))]),
            ],
          ),
        ),
      );
    }

    // Test mainAxisAlignment without sort arrow.
    await tester.pumpWidget(buildTable());

    Offset headerTopLeft = tester.getTopLeft(find.text('Header'));
    expect(headerTopLeft.dx, equals(horizontalMargin));

    // Test mainAxisAlignment.center without sort arrow.
    await tester.pumpWidget(buildTable(headingRowAlignment: MainAxisAlignment.center));

    Offset headerCenter = tester.getCenter(find.text('Header'));
    expect(headerCenter.dx, equals(400));

    // Test mainAxisAlignment with sort arrow.
    await tester.pumpWidget(buildTable(sortEnabled: true));

    headerTopLeft = tester.getTopLeft(find.text('Header'));
    expect(headerTopLeft.dx, equals(horizontalMargin));

    // Test mainAxisAlignment.center with sort arrow.
    await tester.pumpWidget(
      buildTable(headingRowAlignment: MainAxisAlignment.center, sortEnabled: true),
    );

    headerCenter = tester.getCenter(find.text('Header'));
    expect(headerCenter.dx, equals(400));
  });

  testWidgets('DataTable with custom column widths - checkbox', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SizedBox(
            width: 500,
            child: DataTable(
              columns: const <DataColumn>[
                DataColumn(
                  label: Text('Flex Numeric'),
                  columnWidth: FlexColumnWidth(),
                  numeric: true,
                ),
                DataColumn(label: Text('Numeric'), numeric: true),
                DataColumn(label: Text('Text')),
              ],
              rows: <DataRow>[
                DataRow(
                  onSelectChanged: (bool? value) {},
                  cells: const <DataCell>[
                    DataCell(Text('1')),
                    DataCell(Text('1')),
                    DataCell(Text('D')),
                  ],
                ),
              ],
            ),
          ),
        ),
      ),
    );

    final Table table = tester.widget(find.byType(Table));
    expect(table.columnWidths![0], isA<FixedColumnWidth>()); // Checkbox column
    expect(table.columnWidths![1], const FlexColumnWidth());
    expect(table.columnWidths![2], const IntrinsicColumnWidth());
    expect(table.columnWidths![3], const IntrinsicColumnWidth(flex: 1));
  });

  testWidgets('DataTable with custom column widths - no checkbox', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SizedBox(
            width: 500,
            child: DataTable(
              columns: const <DataColumn>[
                DataColumn(
                  label: Text('Flex Numeric'),
                  columnWidth: FlexColumnWidth(),
                  numeric: true,
                ),
                DataColumn(label: Text('Numeric'), numeric: true),
                DataColumn(label: Text('Text')),
              ],
              rows: const <DataRow>[
                DataRow(
                  cells: <DataCell>[DataCell(Text('1')), DataCell(Text('1')), DataCell(Text('D'))],
                ),
              ],
            ),
          ),
        ),
      ),
    );

    final Table table = tester.widget(find.byType(Table));
    expect(table.columnWidths![0], const FlexColumnWidth());
    expect(table.columnWidths![1], const IntrinsicColumnWidth());
    expect(table.columnWidths![2], const IntrinsicColumnWidth(flex: 1));
  });
}

RenderParagraph _getTextRenderObject(WidgetTester tester, String text) {
  return tester.renderObject(find.text(text));
}
