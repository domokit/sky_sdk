// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'rendering_tester.dart';

RenderBox sizedBox(double width, double height) {
  return RenderConstrainedBox(additionalConstraints: BoxConstraints.tight(Size(width, height)));
}

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  test('Table control test; tight', () {
    RenderTable table;
    layout(table = RenderTable(textDirection: TextDirection.ltr));

    expect(table.size.width, equals(800.0));
    expect(table.size.height, equals(600.0));

    expect(table, hasAGoodToStringDeep);
    expect(
      table.toStringDeep(minLevel: DiagnosticLevel.info),
      equalsIgnoringHashCodes(
        'RenderTable#00000 NEEDS-PAINT\n'
        ' │ parentData: <none>\n'
        ' │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        ' │ semantic boundary\n'
        ' │ size: Size(800.0, 600.0)\n'
        ' │ default column width: FlexColumnWidth(1.0)\n'
        ' │ table size: 0×0\n'
        ' │ column offsets: unknown\n'
        ' │ row offsets: []\n'
        ' │\n'
        ' └─table is empty\n',
      ),
    );
  });

  test('Table control test; loose', () {
    RenderTable table;
    layout(RenderPositionedBox(child: table = RenderTable(textDirection: TextDirection.ltr)));

    expect(table.size, equals(Size.zero));
  });

  test('Table control test: constrained flex columns', () {
    final RenderTable table = RenderTable(textDirection: TextDirection.ltr);
    final List<RenderBox> children = List<RenderBox>.generate(6, (_) => RenderPositionedBox());

    table.setFlatChildren(6, children);
    layout(table, constraints: const BoxConstraints.tightFor(width: 100.0));

    const double expectedWidth = 100.0 / 6;
    for (final RenderBox child in children) {
      expect(child.size.width, moreOrLessEquals(expectedWidth));
    }
  });

  test('Table test: combinations', () {
    RenderTable table;
    layout(
      RenderPositionedBox(
        child:
            table = RenderTable(
              columns: 5,
              rows: 5,
              defaultColumnWidth: const IntrinsicColumnWidth(),
              textDirection: TextDirection.ltr,
              defaultVerticalAlignment: TableCellVerticalAlignment.baseline,
              textBaseline: TextBaseline.alphabetic,
            ),
      ),
    );

    expect(table.size, equals(Size.zero));

    table.setChild(2, 4, sizedBox(100.0, 200.0));

    pumpFrame();

    expect(table.size, equals(const Size(100.0, 200.0)));

    table.setChild(0, 0, sizedBox(10.0, 30.0));
    table.setChild(1, 0, sizedBox(20.0, 20.0));
    table.setChild(2, 0, sizedBox(30.0, 10.0));

    pumpFrame();

    expect(table.size, equals(const Size(130.0, 230.0)));

    expect(table, hasAGoodToStringDeep);
    expect(
      table.toStringDeep(minLevel: DiagnosticLevel.info),
      equalsIgnoringHashCodes(
        'RenderTable#00000 relayoutBoundary=up1 NEEDS-PAINT NEEDS-COMPOSITING-BITS-UPDATE\n'
        ' │ parentData: offset=Offset(335.0, 185.0) (can use size)\n'
        ' │ constraints: BoxConstraints(0.0<=w<=800.0, 0.0<=h<=600.0)\n'
        ' │ semantic boundary\n'
        ' │ size: Size(130.0, 230.0)\n'
        ' │ default column width: IntrinsicColumnWidth(flex: null)\n'
        ' │ table size: 5×5\n'
        ' │ column offsets: 0.0, 10.0, 30.0, 130.0, 130.0\n'
        ' │ row offsets: 0.0, 30.0, 30.0, 30.0, 30.0, 230.0\n'
        ' │\n'
        ' ├─child (0, 0): RenderConstrainedBox#00000 relayoutBoundary=up2 NEEDS-PAINT\n'
        ' │   parentData: offset=Offset(0.0, 0.0); default vertical alignment\n'
        ' │     (can use size)\n'
        ' │   constraints: BoxConstraints(w=10.0, 0.0<=h<=Infinity)\n'
        ' │   size: Size(10.0, 30.0)\n'
        ' │   additionalConstraints: BoxConstraints(w=10.0, h=30.0)\n'
        ' │\n'
        ' ├─child (1, 0): RenderConstrainedBox#00000 relayoutBoundary=up2 NEEDS-PAINT\n'
        ' │   parentData: offset=Offset(10.0, 0.0); default vertical alignment\n'
        ' │     (can use size)\n'
        ' │   constraints: BoxConstraints(w=20.0, 0.0<=h<=Infinity)\n'
        ' │   size: Size(20.0, 20.0)\n'
        ' │   additionalConstraints: BoxConstraints(w=20.0, h=20.0)\n'
        ' │\n'
        ' ├─child (2, 0): RenderConstrainedBox#00000 relayoutBoundary=up2 NEEDS-PAINT\n'
        ' │   parentData: offset=Offset(30.0, 0.0); default vertical alignment\n'
        ' │     (can use size)\n'
        ' │   constraints: BoxConstraints(w=100.0, 0.0<=h<=Infinity)\n'
        ' │   size: Size(100.0, 10.0)\n'
        ' │   additionalConstraints: BoxConstraints(w=30.0, h=10.0)\n'
        ' │\n'
        ' ├─child (3, 0) is null\n'
        ' ├─child (4, 0) is null\n'
        ' ├─child (0, 1) is null\n'
        ' ├─child (1, 1) is null\n'
        ' ├─child (2, 1) is null\n'
        ' ├─child (3, 1) is null\n'
        ' ├─child (4, 1) is null\n'
        ' ├─child (0, 2) is null\n'
        ' ├─child (1, 2) is null\n'
        ' ├─child (2, 2) is null\n'
        ' ├─child (3, 2) is null\n'
        ' ├─child (4, 2) is null\n'
        ' ├─child (0, 3) is null\n'
        ' ├─child (1, 3) is null\n'
        ' ├─child (2, 3) is null\n'
        ' ├─child (3, 3) is null\n'
        ' ├─child (4, 3) is null\n'
        ' ├─child (0, 4) is null\n'
        ' ├─child (1, 4) is null\n'
        ' ├─child (2, 4): RenderConstrainedBox#00000 relayoutBoundary=up2 NEEDS-PAINT\n'
        ' │   parentData: offset=Offset(30.0, 30.0); default vertical alignment\n'
        ' │     (can use size)\n'
        ' │   constraints: BoxConstraints(w=100.0, 0.0<=h<=Infinity)\n'
        ' │   size: Size(100.0, 200.0)\n'
        ' │   additionalConstraints: BoxConstraints(w=100.0, h=200.0)\n'
        ' │\n'
        ' ├─child (3, 4) is null\n'
        ' └─child (4, 4) is null\n',
      ),
    );
  });

  test('Table test: removing cells', () {
    RenderTable table;
    RenderBox child;
    table = RenderTable(columns: 5, rows: 5, textDirection: TextDirection.ltr);
    table.setChild(4, 4, child = sizedBox(10.0, 10.0));

    layout(table);

    expect(child.attached, isTrue);
    table.rows = 4;
    expect(child.attached, isFalse);
  });

  test('Table test: replacing cells', () {
    RenderTable table;
    final RenderBox child1 = RenderPositionedBox();
    final RenderBox child2 = RenderPositionedBox();
    final RenderBox child3 = RenderPositionedBox();
    table = RenderTable(textDirection: TextDirection.ltr);
    table.setFlatChildren(3, <RenderBox>[
      child1,
      RenderPositionedBox(),
      child2,
      RenderPositionedBox(),
      child3,
      RenderPositionedBox(),
    ]);
    expect(table.rows, equals(2));
    layout(table);
    table.setFlatChildren(3, <RenderBox>[
      RenderPositionedBox(),
      child1,
      RenderPositionedBox(),
      child2,
      RenderPositionedBox(),
      child3,
    ]);
    pumpFrame();
    table.setFlatChildren(3, <RenderBox>[
      RenderPositionedBox(),
      child1,
      RenderPositionedBox(),
      child2,
      RenderPositionedBox(),
      child3,
    ]);
    pumpFrame();
    expect(table.columns, equals(3));
    expect(table.rows, equals(2));
  });

  test('Table border painting', () {
    final RenderTable table = RenderTable(
      textDirection: TextDirection.rtl,
      border: TableBorder.all(),
    );
    layout(table);
    table.setFlatChildren(1, <RenderBox>[]);
    pumpFrame();
    expect(
      table,
      paints
        ..path()
        ..path()
        ..path()
        ..path(),
    );
    table.setFlatChildren(1, <RenderBox>[RenderPositionedBox()]);
    pumpFrame();
    expect(
      table,
      paints
        ..path()
        ..path()
        ..path()
        ..path(),
    );
    table.setFlatChildren(1, <RenderBox>[RenderPositionedBox(), RenderPositionedBox()]);
    pumpFrame();
    expect(
      table,
      paints
        ..path()
        ..path()
        ..path()
        ..path()
        ..path(),
    );
    table.setFlatChildren(2, <RenderBox>[RenderPositionedBox(), RenderPositionedBox()]);
    pumpFrame();
    expect(
      table,
      paints
        ..path()
        ..path()
        ..path()
        ..path()
        ..path(),
    );
    table.setFlatChildren(2, <RenderBox>[
      RenderPositionedBox(),
      RenderPositionedBox(),
      RenderPositionedBox(),
      RenderPositionedBox(),
    ]);
    pumpFrame();
    expect(
      table,
      paints
        ..path()
        ..path()
        ..path()
        ..path()
        ..path()
        ..path(),
    );
    table.setFlatChildren(3, <RenderBox>[
      RenderPositionedBox(),
      RenderPositionedBox(),
      RenderPositionedBox(),
      RenderPositionedBox(),
      RenderPositionedBox(),
      RenderPositionedBox(),
    ]);
    pumpFrame();
    expect(
      table,
      paints
        ..path()
        ..path()
        ..path()
        ..path()
        ..path()
        ..path(),
    );
  });

  test('Table flex sizing', () {
    const BoxConstraints cellConstraints = BoxConstraints.tightFor(width: 100, height: 100);
    final RenderTable table = RenderTable(
      textDirection: TextDirection.rtl,
      children: <List<RenderBox>>[
        List<RenderBox>.generate(
          7,
          (int _) => RenderConstrainedBox(additionalConstraints: cellConstraints),
        ),
      ],
      columnWidths: const <int, TableColumnWidth>{
        0: FlexColumnWidth(),
        1: FlexColumnWidth(0.123),
        2: FlexColumnWidth(0.123),
        3: FlexColumnWidth(0.123),
        4: FlexColumnWidth(0.123),
        5: FlexColumnWidth(0.123),
        6: FlexColumnWidth(0.123),
      },
    );

    layout(table, constraints: BoxConstraints.tight(const Size(800.0, 600.0)));
    expect(table.hasSize, true);
  });

  test('Table paints a borderRadius', () {
    final RenderTable table = RenderTable(
      textDirection: TextDirection.ltr,
      border: TableBorder.all(borderRadius: const BorderRadius.all(Radius.circular(8.0))),
    );
    layout(table);
    table.setFlatChildren(2, <RenderBox>[
      RenderPositionedBox(),
      RenderPositionedBox(),
      RenderPositionedBox(),
      RenderPositionedBox(),
    ]);
    pumpFrame();
    expect(
      table,
      paints
        ..path()
        ..path()
        ..drrect(
          outer: RRect.fromLTRBR(0.0, 0.0, 800.0, 0.0, const Radius.circular(8.0)),
          inner: RRect.fromLTRBR(1.0, 1.0, 799.0, -1.0, const Radius.circular(7.0)),
        ),
    );
  });

  test('MaxColumnWidth.flex returns the correct result', () {
    MaxColumnWidth columnWidth = const MaxColumnWidth(
      FixedColumnWidth(100), // returns null from .flex
      FlexColumnWidth(), // returns 1 from .flex
    );
    final double? flexValue = columnWidth.flex(<RenderBox>[]);
    expect(flexValue, 1.0);

    // Swap a and b, check for same result.
    columnWidth = const MaxColumnWidth(
      FlexColumnWidth(), // returns 1 from .flex
      FixedColumnWidth(100), // returns null from .flex
    );
    // Same result.
    expect(columnWidth.flex(<RenderBox>[]), flexValue);
  });

  test('MinColumnWidth.flex returns the correct result', () {
    MinColumnWidth columnWidth = const MinColumnWidth(
      FixedColumnWidth(100), // returns null from .flex
      FlexColumnWidth(), // returns 1 from .flex
    );
    final double? flexValue = columnWidth.flex(<RenderBox>[]);
    expect(flexValue, 1.0);

    // Swap a and b, check for same result.
    columnWidth = const MinColumnWidth(
      FlexColumnWidth(), // returns 1 from .flex
      FixedColumnWidth(100), // returns null from .flex
    );
    // Same result.
    expect(columnWidth.flex(<RenderBox>[]), flexValue);
  });

  test('TableRows with different constraints, but vertically with intrinsicHeight', () {
    const BoxConstraints firstConstraints = BoxConstraints.tightFor(width: 100, height: 100);
    const BoxConstraints secondConstraints = BoxConstraints.tightFor(width: 200, height: 200);

    final RenderTable table = RenderTable(
      textDirection: TextDirection.rtl,
      defaultVerticalAlignment: TableCellVerticalAlignment.intrinsicHeight,
      children: <List<RenderBox>>[
        <RenderBox>[
          RenderConstrainedBox(additionalConstraints: firstConstraints),
          RenderConstrainedBox(additionalConstraints: secondConstraints),
        ],
      ],
      columnWidths: const <int, TableColumnWidth>{0: FlexColumnWidth(), 1: FlexColumnWidth()},
    );

    const Size size = Size(300.0, 300.0);

    // Layout the table with a fixed size.
    layout(table, constraints: BoxConstraints.tight(size));

    // Make sure the table has a size and that the children are filled vertically to the highest cell.
    expect(table.size, equals(size));
    expect(table.defaultVerticalAlignment, TableCellVerticalAlignment.intrinsicHeight);
  });
}
