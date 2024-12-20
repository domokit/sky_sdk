// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('DataTableThemeData copyWith, ==, hashCode basics', () {
    expect(const DataTableThemeData(), const DataTableThemeData().copyWith());
    expect(const DataTableThemeData().hashCode, const DataTableThemeData().copyWith().hashCode);
  });

  test('DataTableThemeData copyWith dataRowHeight', () {
    const DataTableThemeData themeData = DataTableThemeData(
      dataRowMinHeight: 10,
      dataRowMaxHeight: 10,
    );
    expect(themeData, themeData.copyWith());
    expect(
      themeData.copyWith(dataRowMinHeight: 20, dataRowMaxHeight: 20),
      themeData.copyWith(dataRowHeight: 20),
    );
  });

  test('DataTableThemeData lerp special cases', () {
    const DataTableThemeData data = DataTableThemeData();
    expect(identical(DataTableThemeData.lerp(data, data, 0.5), data), true);
  });

  test('DataTableThemeData defaults', () {
    const DataTableThemeData themeData = DataTableThemeData();
    expect(themeData.decoration, null);
    expect(themeData.dataRowColor, null);
    expect(themeData.dataRowHeight, null);
    expect(themeData.dataRowMinHeight, null);
    expect(themeData.dataRowMaxHeight, null);
    expect(themeData.dataTextStyle, null);
    expect(themeData.headingRowColor, null);
    expect(themeData.headingRowHeight, null);
    expect(themeData.headingTextStyle, null);
    expect(themeData.horizontalMargin, null);
    expect(themeData.columnSpacing, null);
    expect(themeData.dividerThickness, null);
    expect(themeData.checkboxHorizontalMargin, null);
    expect(themeData.headingCellCursor, null);
    expect(themeData.dataRowCursor, null);
    expect(themeData.headingRowAlignment, null);

    const DataTableTheme theme = DataTableTheme(data: DataTableThemeData(), child: SizedBox());
    expect(theme.data.decoration, null);
    expect(theme.data.dataRowColor, null);
    expect(theme.data.dataRowHeight, null);
    expect(theme.data.dataRowMinHeight, null);
    expect(theme.data.dataRowMaxHeight, null);
    expect(theme.data.dataTextStyle, null);
    expect(theme.data.headingRowColor, null);
    expect(theme.data.headingRowHeight, null);
    expect(theme.data.headingTextStyle, null);
    expect(theme.data.horizontalMargin, null);
    expect(theme.data.columnSpacing, null);
    expect(theme.data.dividerThickness, null);
    expect(theme.data.checkboxHorizontalMargin, null);
    expect(theme.data.headingCellCursor, null);
    expect(theme.data.dataRowCursor, null);
    expect(theme.data.headingRowAlignment, null);
  });

  testWidgets('Default DataTableThemeData debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const DataTableThemeData().debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[]);
  });

  testWidgets('DataTableThemeData implements debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    DataTableThemeData(
      decoration: const BoxDecoration(color: Color(0xfffffff0)),
      dataRowColor: MaterialStateProperty.resolveWith<Color>(
        (Set<MaterialState> states) => const Color(0xfffffff1),
      ),
      dataRowMinHeight: 41.0,
      dataRowMaxHeight: 42.0,
      dataTextStyle: const TextStyle(fontSize: 12.0),
      headingRowColor: MaterialStateProperty.resolveWith<Color>(
        (Set<MaterialState> states) => const Color(0xfffffff2),
      ),
      headingRowHeight: 52.0,
      headingTextStyle: const TextStyle(fontSize: 14.0),
      horizontalMargin: 3.0,
      columnSpacing: 4.0,
      dividerThickness: 5.0,
      checkboxHorizontalMargin: 6.0,
      headingCellCursor: const MaterialStatePropertyAll<MouseCursor>(SystemMouseCursors.grab),
      dataRowCursor: const MaterialStatePropertyAll<MouseCursor>(SystemMouseCursors.forbidden),
      headingRowAlignment: MainAxisAlignment.center,
    ).debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description[0], 'decoration: BoxDecoration(color: ${const Color(0xfffffff0)})');
    expect(description[1], "dataRowColor: Instance of '_WidgetStatePropertyWith<Color>'");
    expect(description[2], 'dataRowMinHeight: 41.0');
    expect(description[3], 'dataRowMaxHeight: 42.0');
    expect(description[4], 'dataTextStyle: TextStyle(inherit: true, size: 12.0)');
    expect(description[5], "headingRowColor: Instance of '_WidgetStatePropertyWith<Color>'");
    expect(description[6], 'headingRowHeight: 52.0');
    expect(description[7], 'headingTextStyle: TextStyle(inherit: true, size: 14.0)');
    expect(description[8], 'horizontalMargin: 3.0');
    expect(description[9], 'columnSpacing: 4.0');
    expect(description[10], 'dividerThickness: 5.0');
    expect(description[11], 'checkboxHorizontalMargin: 6.0');
    expect(description[12], 'headingCellCursor: WidgetStatePropertyAll(SystemMouseCursor(grab))');
    expect(description[13], 'dataRowCursor: WidgetStatePropertyAll(SystemMouseCursor(forbidden))');
    expect(description[14], 'headingRowAlignment: center');
  });

  testWidgets('DataTable is themeable', (WidgetTester tester) async {
    const BoxDecoration decoration = BoxDecoration(color: Color(0xfffffff0));
    const MaterialStateProperty<Color> dataRowColor = MaterialStatePropertyAll<Color>(
      Color(0xfffffff1),
    );
    const double minMaxDataRowHeight = 41.0;
    const TextStyle dataTextStyle = TextStyle(fontSize: 12.5);
    const MaterialStateProperty<Color> headingRowColor = MaterialStatePropertyAll<Color>(
      Color(0xfffffff2),
    );
    const double headingRowHeight = 52.0;
    const TextStyle headingTextStyle = TextStyle(fontSize: 14.5);
    const double horizontalMargin = 3.0;
    const double columnSpacing = 4.0;
    const double dividerThickness = 5.0;
    const MaterialStateProperty<MouseCursor> headingCellCursor =
        MaterialStatePropertyAll<MouseCursor>(SystemMouseCursors.grab);
    const MaterialStateProperty<MouseCursor> dataRowCursor = MaterialStatePropertyAll<MouseCursor>(
      SystemMouseCursors.forbidden,
    );

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          dataTableTheme: const DataTableThemeData(
            decoration: decoration,
            dataRowColor: dataRowColor,
            dataRowMinHeight: minMaxDataRowHeight,
            dataRowMaxHeight: minMaxDataRowHeight,
            dataTextStyle: dataTextStyle,
            headingRowColor: headingRowColor,
            headingRowHeight: headingRowHeight,
            headingTextStyle: headingTextStyle,
            horizontalMargin: horizontalMargin,
            columnSpacing: columnSpacing,
            dividerThickness: dividerThickness,
            headingCellCursor: headingCellCursor,
            dataRowCursor: dataRowCursor,
          ),
        ),
        home: Scaffold(
          body: DataTable(
            sortColumnIndex: 0,
            showCheckboxColumn: false,
            columns: <DataColumn>[
              DataColumn(label: const Text('A'), onSort: (int columnIndex, bool ascending) {}),
              const DataColumn(label: Text('B')),
            ],
            rows: <DataRow>[
              DataRow(
                cells: const <DataCell>[DataCell(Text('Data')), DataCell(Text('Data 2'))],
                onSelectChanged: (bool? value) {},
              ),
            ],
          ),
        ),
      ),
    );

    final Finder tableContainerFinder = find.ancestor(
      of: find.byType(Table),
      matching: find.byType(Container),
    );
    expect(tester.widgetList<Container>(tableContainerFinder).first.decoration, decoration);

    final TextStyle dataRowTextStyle =
        tester.renderObject<RenderParagraph>(find.text('Data')).text.style!;
    expect(dataRowTextStyle.fontSize, dataTextStyle.fontSize);
    expect(
      _tableRowBoxDecoration(tester: tester, index: 1).color,
      dataRowColor.resolve(<MaterialState>{}),
    );
    expect(_tableRowBoxDecoration(tester: tester, index: 1).border!.top.width, dividerThickness);
    expect(tester.getSize(_findFirstContainerFor('Data')).height, minMaxDataRowHeight);

    final TextStyle headingRowTextStyle =
        tester.renderObject<RenderParagraph>(find.text('A')).text.style!;
    expect(headingRowTextStyle.fontSize, headingTextStyle.fontSize);
    expect(
      _tableRowBoxDecoration(tester: tester, index: 0).color,
      headingRowColor.resolve(<MaterialState>{}),
    );

    expect(tester.getSize(_findFirstContainerFor('A')).height, headingRowHeight);
    expect(tester.getTopLeft(find.text('A')).dx, horizontalMargin);
    expect(
      tester.getTopLeft(find.text('Data 2')).dx - tester.getTopRight(find.text('Data')).dx,
      columnSpacing,
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: tester.getCenter(find.text('A')));
    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.grab,
    );

    await gesture.moveTo(tester.getCenter(find.text('Data')));
    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.forbidden,
    );
  });

  testWidgets('DataTable is themeable - separate test for deprecated dataRowHeight', (
    WidgetTester tester,
  ) async {
    const double dataRowHeight = 51.0;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(dataTableTheme: const DataTableThemeData(dataRowHeight: dataRowHeight)),
        home: Scaffold(
          body: DataTable(
            sortColumnIndex: 0,
            columns: <DataColumn>[
              DataColumn(label: const Text('A'), onSort: (int columnIndex, bool ascending) {}),
              const DataColumn(label: Text('B')),
            ],
            rows: const <DataRow>[
              DataRow(cells: <DataCell>[DataCell(Text('Data')), DataCell(Text('Data 2'))]),
            ],
          ),
        ),
      ),
    );

    expect(tester.getSize(_findFirstContainerFor('Data')).height, dataRowHeight);
  });

  testWidgets('DataTable properties are taken over the theme values', (WidgetTester tester) async {
    const BoxDecoration themeDecoration = BoxDecoration(color: Color(0xfffffff1));
    const MaterialStateProperty<Color> themeDataRowColor = MaterialStatePropertyAll<Color>(
      Color(0xfffffff0),
    );
    const double minMaxThemeDataRowHeight = 50.0;
    const TextStyle themeDataTextStyle = TextStyle(fontSize: 11.5);
    const MaterialStateProperty<Color> themeHeadingRowColor = MaterialStatePropertyAll<Color>(
      Color(0xfffffff1),
    );
    const double themeHeadingRowHeight = 51.0;
    const TextStyle themeHeadingTextStyle = TextStyle(fontSize: 13.5);
    const double themeHorizontalMargin = 2.0;
    const double themeColumnSpacing = 3.0;
    const double themeDividerThickness = 4.0;
    const MaterialStateProperty<MouseCursor> themeHeadingCellCursor =
        MaterialStatePropertyAll<MouseCursor>(SystemMouseCursors.copy);
    const MaterialStateProperty<MouseCursor> themeDataRowCursor =
        MaterialStatePropertyAll<MouseCursor>(SystemMouseCursors.copy);

    const BoxDecoration decoration = BoxDecoration(color: Color(0xfffffff0));
    const MaterialStateProperty<Color> dataRowColor = MaterialStatePropertyAll<Color>(
      Color(0xfffffff1),
    );
    const double minMaxDataRowHeight = 51.0;
    const TextStyle dataTextStyle = TextStyle(fontSize: 12.5);
    const MaterialStateProperty<Color> headingRowColor = MaterialStatePropertyAll<Color>(
      Color(0xfffffff2),
    );
    const double headingRowHeight = 52.0;
    const TextStyle headingTextStyle = TextStyle(fontSize: 14.5);
    const double horizontalMargin = 3.0;
    const double columnSpacing = 4.0;
    const double dividerThickness = 5.0;
    const MaterialStateProperty<MouseCursor> headingCellCursor =
        MaterialStatePropertyAll<MouseCursor>(SystemMouseCursors.forbidden);
    const MaterialStateProperty<MouseCursor> dataRowCursor = MaterialStatePropertyAll<MouseCursor>(
      SystemMouseCursors.forbidden,
    );

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          dataTableTheme: const DataTableThemeData(
            decoration: themeDecoration,
            dataRowColor: themeDataRowColor,
            dataRowMinHeight: minMaxThemeDataRowHeight,
            dataRowMaxHeight: minMaxThemeDataRowHeight,
            dataTextStyle: themeDataTextStyle,
            headingRowColor: themeHeadingRowColor,
            headingRowHeight: themeHeadingRowHeight,
            headingTextStyle: themeHeadingTextStyle,
            horizontalMargin: themeHorizontalMargin,
            columnSpacing: themeColumnSpacing,
            dividerThickness: themeDividerThickness,
            headingCellCursor: themeHeadingCellCursor,
            dataRowCursor: themeDataRowCursor,
          ),
        ),
        home: Scaffold(
          body: DataTable(
            showCheckboxColumn: false,
            decoration: decoration,
            dataRowColor: dataRowColor,
            dataRowMinHeight: minMaxDataRowHeight,
            dataRowMaxHeight: minMaxDataRowHeight,
            dataTextStyle: dataTextStyle,
            headingRowColor: headingRowColor,
            headingRowHeight: headingRowHeight,
            headingTextStyle: headingTextStyle,
            horizontalMargin: horizontalMargin,
            columnSpacing: columnSpacing,
            dividerThickness: dividerThickness,
            sortColumnIndex: 0,
            columns: <DataColumn>[
              DataColumn(
                label: const Text('A'),
                mouseCursor: headingCellCursor,
                onSort: (int columnIndex, bool ascending) {},
              ),
              const DataColumn(label: Text('B')),
            ],
            rows: <DataRow>[
              DataRow(
                mouseCursor: dataRowCursor,
                onSelectChanged: (bool? selected) {},
                cells: const <DataCell>[DataCell(Text('Data')), DataCell(Text('Data 2'))],
              ),
            ],
          ),
        ),
      ),
    );

    final Finder tableContainerFinder = find.ancestor(
      of: find.byType(Table),
      matching: find.byType(Container),
    );
    expect(tester.widget<Container>(tableContainerFinder).decoration, decoration);

    final TextStyle dataRowTextStyle =
        tester.renderObject<RenderParagraph>(find.text('Data')).text.style!;
    expect(dataRowTextStyle.fontSize, dataTextStyle.fontSize);
    expect(
      _tableRowBoxDecoration(tester: tester, index: 1).color,
      dataRowColor.resolve(<MaterialState>{}),
    );
    expect(_tableRowBoxDecoration(tester: tester, index: 1).border!.top.width, dividerThickness);
    expect(tester.getSize(_findFirstContainerFor('Data')).height, minMaxDataRowHeight);

    final TextStyle headingRowTextStyle =
        tester.renderObject<RenderParagraph>(find.text('A')).text.style!;
    expect(headingRowTextStyle.fontSize, headingTextStyle.fontSize);
    expect(
      _tableRowBoxDecoration(tester: tester, index: 0).color,
      headingRowColor.resolve(<MaterialState>{}),
    );

    expect(tester.getSize(_findFirstContainerFor('A')).height, headingRowHeight);
    expect(tester.getTopLeft(find.text('A')).dx, horizontalMargin);
    expect(
      tester.getTopLeft(find.text('Data 2')).dx - tester.getTopRight(find.text('Data')).dx,
      columnSpacing,
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: tester.getCenter(find.text('A')));
    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      headingCellCursor.resolve(<MaterialState>{}),
    );

    await gesture.moveTo(tester.getCenter(find.text('Data')));
    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      dataRowCursor.resolve(<MaterialState>{}),
    );
  });

  testWidgets(
    'DataTable properties are taken over the theme values - separate test for deprecated dataRowHeight',
    (WidgetTester tester) async {
      const double themeDataRowHeight = 50.0;
      const double dataRowHeight = 51.0;

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            dataTableTheme: const DataTableThemeData(dataRowHeight: themeDataRowHeight),
          ),
          home: Scaffold(
            body: DataTable(
              dataRowHeight: dataRowHeight,
              sortColumnIndex: 0,
              columns: <DataColumn>[
                DataColumn(label: const Text('A'), onSort: (int columnIndex, bool ascending) {}),
                const DataColumn(label: Text('B')),
              ],
              rows: const <DataRow>[
                DataRow(cells: <DataCell>[DataCell(Text('Data')), DataCell(Text('Data 2'))]),
              ],
            ),
          ),
        ),
      );

      expect(tester.getSize(_findFirstContainerFor('Data')).height, dataRowHeight);
    },
  );

  testWidgets('Local DataTableTheme can override global DataTableTheme', (
    WidgetTester tester,
  ) async {
    const BoxDecoration globalThemeDecoration = BoxDecoration(color: Color(0xfffffff1));
    const MaterialStateProperty<Color> globalThemeDataRowColor = MaterialStatePropertyAll<Color>(
      Color(0xfffffff0),
    );
    const double minMaxGlobalThemeDataRowHeight = 50.0;
    const TextStyle globalThemeDataTextStyle = TextStyle(fontSize: 11.5);
    const MaterialStateProperty<Color> globalThemeHeadingRowColor = MaterialStatePropertyAll<Color>(
      Color(0xfffffff1),
    );
    const double globalThemeHeadingRowHeight = 51.0;
    const TextStyle globalThemeHeadingTextStyle = TextStyle(fontSize: 13.5);
    const double globalThemeHorizontalMargin = 2.0;
    const double globalThemeColumnSpacing = 3.0;
    const double globalThemeDividerThickness = 4.0;
    const MaterialStateProperty<MouseCursor> globalHeadingCellCursor =
        MaterialStatePropertyAll<MouseCursor>(SystemMouseCursors.allScroll);
    const MaterialStateProperty<MouseCursor> globalDataRowCursor =
        MaterialStatePropertyAll<MouseCursor>(SystemMouseCursors.allScroll);

    const BoxDecoration localThemeDecoration = BoxDecoration(color: Color(0xfffffff0));
    const MaterialStateProperty<Color> localThemeDataRowColor = MaterialStatePropertyAll<Color>(
      Color(0xfffffff1),
    );
    const double minMaxLocalThemeDataRowHeight = 51.0;
    const TextStyle localThemeDataTextStyle = TextStyle(fontSize: 12.5);
    const MaterialStateProperty<Color> localThemeHeadingRowColor = MaterialStatePropertyAll<Color>(
      Color(0xfffffff2),
    );
    const double localThemeHeadingRowHeight = 52.0;
    const TextStyle localThemeHeadingTextStyle = TextStyle(fontSize: 14.5);
    const double localThemeHorizontalMargin = 3.0;
    const double localThemeColumnSpacing = 4.0;
    const double localThemeDividerThickness = 5.0;
    const MaterialStateProperty<MouseCursor> localHeadingCellCursor =
        MaterialStatePropertyAll<MouseCursor>(SystemMouseCursors.move);
    const MaterialStateProperty<MouseCursor> localDataRowCursor =
        MaterialStatePropertyAll<MouseCursor>(SystemMouseCursors.move);

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          dataTableTheme: const DataTableThemeData(
            decoration: globalThemeDecoration,
            dataRowColor: globalThemeDataRowColor,
            dataRowMinHeight: minMaxGlobalThemeDataRowHeight,
            dataRowMaxHeight: minMaxGlobalThemeDataRowHeight,
            dataTextStyle: globalThemeDataTextStyle,
            headingRowColor: globalThemeHeadingRowColor,
            headingRowHeight: globalThemeHeadingRowHeight,
            headingTextStyle: globalThemeHeadingTextStyle,
            horizontalMargin: globalThemeHorizontalMargin,
            columnSpacing: globalThemeColumnSpacing,
            dividerThickness: globalThemeDividerThickness,
            headingCellCursor: globalHeadingCellCursor,
            dataRowCursor: globalDataRowCursor,
          ),
        ),
        home: Scaffold(
          body: DataTableTheme(
            data: const DataTableThemeData(
              decoration: localThemeDecoration,
              dataRowColor: localThemeDataRowColor,
              dataRowMinHeight: minMaxLocalThemeDataRowHeight,
              dataRowMaxHeight: minMaxLocalThemeDataRowHeight,
              dataTextStyle: localThemeDataTextStyle,
              headingRowColor: localThemeHeadingRowColor,
              headingRowHeight: localThemeHeadingRowHeight,
              headingTextStyle: localThemeHeadingTextStyle,
              horizontalMargin: localThemeHorizontalMargin,
              columnSpacing: localThemeColumnSpacing,
              dividerThickness: localThemeDividerThickness,
              headingCellCursor: localHeadingCellCursor,
              dataRowCursor: localDataRowCursor,
            ),
            child: DataTable(
              showCheckboxColumn: false,
              sortColumnIndex: 0,
              columns: <DataColumn>[
                DataColumn(label: const Text('A'), onSort: (int columnIndex, bool ascending) {}),
                const DataColumn(label: Text('B')),
              ],
              rows: <DataRow>[
                DataRow(
                  onSelectChanged: (bool? selected) {},
                  cells: const <DataCell>[DataCell(Text('Data')), DataCell(Text('Data 2'))],
                ),
              ],
            ),
          ),
        ),
      ),
    );

    final Finder tableContainerFinder = find.ancestor(
      of: find.byType(Table),
      matching: find.byType(Container),
    );
    expect(
      tester.widgetList<Container>(tableContainerFinder).first.decoration,
      localThemeDecoration,
    );

    final TextStyle dataRowTextStyle =
        tester.renderObject<RenderParagraph>(find.text('Data')).text.style!;
    expect(dataRowTextStyle.fontSize, localThemeDataTextStyle.fontSize);
    expect(
      _tableRowBoxDecoration(tester: tester, index: 1).color,
      localThemeDataRowColor.resolve(<MaterialState>{}),
    );
    expect(
      _tableRowBoxDecoration(tester: tester, index: 1).border!.top.width,
      localThemeDividerThickness,
    );
    expect(tester.getSize(_findFirstContainerFor('Data')).height, minMaxLocalThemeDataRowHeight);

    final TextStyle headingRowTextStyle =
        tester.renderObject<RenderParagraph>(find.text('A')).text.style!;
    expect(headingRowTextStyle.fontSize, localThemeHeadingTextStyle.fontSize);
    expect(
      _tableRowBoxDecoration(tester: tester, index: 0).color,
      localThemeHeadingRowColor.resolve(<MaterialState>{}),
    );

    expect(tester.getSize(_findFirstContainerFor('A')).height, localThemeHeadingRowHeight);
    expect(tester.getTopLeft(find.text('A')).dx, localThemeHorizontalMargin);
    expect(
      tester.getTopLeft(find.text('Data 2')).dx - tester.getTopRight(find.text('Data')).dx,
      localThemeColumnSpacing,
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: tester.getCenter(find.text('A')));
    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      localHeadingCellCursor.resolve(<MaterialState>{}),
    );

    await gesture.moveTo(tester.getCenter(find.text('Data')));
    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      localDataRowCursor.resolve(<MaterialState>{}),
    );
  });

  testWidgets(
    'Local DataTableTheme can override global DataTableTheme - separate test for deprecated dataRowHeight',
    (WidgetTester tester) async {
      const double globalThemeDataRowHeight = 50.0;
      const double localThemeDataRowHeight = 51.0;

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            dataTableTheme: const DataTableThemeData(dataRowHeight: globalThemeDataRowHeight),
          ),
          home: Scaffold(
            body: DataTableTheme(
              data: const DataTableThemeData(dataRowHeight: localThemeDataRowHeight),
              child: DataTable(
                sortColumnIndex: 0,
                columns: <DataColumn>[
                  DataColumn(label: const Text('A'), onSort: (int columnIndex, bool ascending) {}),
                  const DataColumn(label: Text('B')),
                ],
                rows: const <DataRow>[
                  DataRow(cells: <DataCell>[DataCell(Text('Data')), DataCell(Text('Data 2'))]),
                ],
              ),
            ),
          ),
        ),
      );

      expect(tester.getSize(_findFirstContainerFor('Data')).height, localThemeDataRowHeight);
    },
  );

  // This is a regression test for https://github.com/flutter/flutter/issues/143340.
  testWidgets('DataColumn label can be centered with DataTableTheme.headingRowAlignment', (
    WidgetTester tester,
  ) async {
    const double horizontalMargin = 24.0;

    Widget buildTable({MainAxisAlignment? headingRowAlignment, bool sortEnabled = false}) {
      return MaterialApp(
        theme: ThemeData(
          dataTableTheme: DataTableThemeData(headingRowAlignment: headingRowAlignment),
        ),
        home: Material(
          child: DataTable(
            columns: <DataColumn>[
              DataColumn(
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
    await tester.pumpAndSettle();

    Offset headerCenter = tester.getCenter(find.text('Header'));
    expect(headerCenter.dx, equals(400));

    // Test mainAxisAlignment with sort arrow.
    await tester.pumpWidget(buildTable(sortEnabled: true));
    await tester.pumpAndSettle();

    headerTopLeft = tester.getTopLeft(find.text('Header'));
    expect(headerTopLeft.dx, equals(horizontalMargin));

    // Test mainAxisAlignment.center with sort arrow.
    await tester.pumpWidget(
      buildTable(headingRowAlignment: MainAxisAlignment.center, sortEnabled: true),
    );
    await tester.pumpAndSettle();

    headerCenter = tester.getCenter(find.text('Header'));
    expect(headerCenter.dx, equals(400));
  });
}

BoxDecoration _tableRowBoxDecoration({required WidgetTester tester, required int index}) {
  final Table table = tester.widget(find.byType(Table));
  final TableRow tableRow = table.children[index];
  return tableRow.decoration! as BoxDecoration;
}

// The finder matches with the Container of the cell content, as well as the
// Container wrapping the whole table. The first one is used to test row
// heights.
Finder _findFirstContainerFor(String text) => find.widgetWithText(Container, text).first;
