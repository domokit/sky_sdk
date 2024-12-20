// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'card_theme.dart';
/// @docImport 'data_table_theme.dart';
/// @docImport 'text_button.dart';
library;

import 'dart:math' as math;

import 'package:flutter/gestures.dart' show DragStartBehavior;
import 'package:flutter/widgets.dart';

import 'card.dart';
import 'constants.dart';
import 'data_table.dart';
import 'data_table_source.dart';
import 'debug.dart';
import 'dropdown.dart';
import 'icon_button.dart';
import 'icons.dart';
import 'ink_decoration.dart';
import 'material_localizations.dart';
import 'material_state.dart';
import 'progress_indicator.dart';
import 'theme.dart';

/// A table that follows the
/// [Material 2](https://material.io/go/design-data-tables)
/// design specification, using multiple pages to display data.
///
/// A paginated data table shows [rowsPerPage] rows of data per page and
/// provides controls for showing other pages.
///
/// Data is read lazily from a [DataTableSource]. The widget is presented
/// as a [Card].
///
/// If the [key] is a [PageStorageKey], the [initialFirstRowIndex] is persisted
/// to [PageStorage].
///
/// {@tool dartpad}
///
/// This sample shows how to display a [DataTable] with three columns: name,
/// age, and role. The columns are defined by three [DataColumn] objects. The
/// table contains three rows of data for three example users, the data for
/// which is defined by three [DataRow] objects.
///
/// ** See code in examples/api/lib/material/paginated_data_table/paginated_data_table.0.dart **
/// {@end-tool}
///
/// {@tool dartpad}
///
/// This example shows how paginated data tables can supported sorted data.
///
/// ** See code in examples/api/lib/material/paginated_data_table/paginated_data_table.1.dart **
/// {@end-tool}
///
/// See also:
///
///  * [DataTable], which is not paginated.
///  * `TableView` from the
///    [two_dimensional_scrollables](https://pub.dev/packages/two_dimensional_scrollables)
///    package, for displaying large amounts of data without pagination.
///  * <https://material.io/go/design-data-tables>
class PaginatedDataTable extends StatefulWidget {
  /// Creates a widget describing a paginated [DataTable] on a [Card].
  ///
  /// The [header] should give the card's header, typically a [Text] widget.
  ///
  /// The [columns] argument must be a list of as many [DataColumn] objects as
  /// the table is to have columns, ignoring the leading checkbox column if any.
  /// The [columns] argument must have a length greater than zero and cannot be
  /// null.
  ///
  /// If the table is sorted, the column that provides the current primary key
  /// should be specified by index in [sortColumnIndex], 0 meaning the first
  /// column in [columns], 1 being the next one, and so forth.
  ///
  /// The actual sort order can be specified using [sortAscending]; if the sort
  /// order is ascending, this should be true (the default), otherwise it should
  /// be false.
  ///
  /// The [source] should be a long-lived [DataTableSource]. The same source
  /// should be provided each time a particular [PaginatedDataTable] widget is
  /// created; avoid creating a new [DataTableSource] with each new instance of
  /// the [PaginatedDataTable] widget unless the data table really is to now
  /// show entirely different data from a new source.
  ///
  /// Themed by [DataTableTheme]. [DataTableThemeData.decoration] is ignored.
  /// To modify the border or background color of the [PaginatedDataTable], use
  /// [CardTheme], since a [Card] wraps the inner [DataTable].
  PaginatedDataTable({
    super.key,
    this.header,
    this.actions,
    required this.columns,
    this.sortColumnIndex,
    this.sortAscending = true,
    this.onSelectAll,
    @Deprecated(
      'Migrate to use dataRowMinHeight and dataRowMaxHeight instead. '
      'This feature was deprecated after v3.7.0-5.0.pre.',
    )
    double? dataRowHeight,
    double? dataRowMinHeight,
    double? dataRowMaxHeight,
    this.headingRowHeight = 56.0,
    this.horizontalMargin = 24.0,
    this.columnSpacing = 56.0,
    this.showCheckboxColumn = true,
    this.showFirstLastButtons = false,
    this.initialFirstRowIndex = 0,
    this.onPageChanged,
    this.rowsPerPage = defaultRowsPerPage,
    this.availableRowsPerPage = const <int>[
      defaultRowsPerPage,
      defaultRowsPerPage * 2,
      defaultRowsPerPage * 5,
      defaultRowsPerPage * 10,
    ],
    this.onRowsPerPageChanged,
    this.dragStartBehavior = DragStartBehavior.start,
    this.arrowHeadColor,
    required this.source,
    this.checkboxHorizontalMargin,
    this.controller,
    this.primary,
    this.headingRowColor,
    this.dividerThickness,
    this.showEmptyRows = true,
  }) : assert(actions == null || (header != null)),
       assert(columns.isNotEmpty),
       assert(
         sortColumnIndex == null || (sortColumnIndex >= 0 && sortColumnIndex < columns.length),
       ),
       assert(
         dataRowMinHeight == null ||
             dataRowMaxHeight == null ||
             dataRowMaxHeight >= dataRowMinHeight,
       ),
       assert(
         dataRowHeight == null || (dataRowMinHeight == null && dataRowMaxHeight == null),
         'dataRowHeight ($dataRowHeight) must not be set if dataRowMinHeight ($dataRowMinHeight) or dataRowMaxHeight ($dataRowMaxHeight) are set.',
       ),
       dataRowMinHeight = dataRowHeight ?? dataRowMinHeight,
       dataRowMaxHeight = dataRowHeight ?? dataRowMaxHeight,
       assert(rowsPerPage > 0),
       assert(dividerThickness == null || dividerThickness >= 0),
       assert(() {
         if (onRowsPerPageChanged != null) {
           assert(availableRowsPerPage.contains(rowsPerPage));
         }
         return true;
       }()),
       assert(
         !(controller != null && (primary ?? false)),
         'Primary ScrollViews obtain their ScrollController via inheritance from a PrimaryScrollController widget. '
         'You cannot both set primary to true and pass an explicit controller.',
       );

  /// The table card's optional header.
  ///
  /// This is typically a [Text] widget, but can also be a [Row] of
  /// [TextButton]s. To show icon buttons at the top end side of the table with
  /// a header, set the [actions] property.
  ///
  /// If items in the table are selectable, then, when the selection is not
  /// empty, the header is replaced by a count of the selected items. The
  /// [actions] are still visible when items are selected.
  final Widget? header;

  /// Icon buttons to show at the top end side of the table. The [header] must
  /// not be null to show the actions.
  ///
  /// Typically, the exact actions included in this list will vary based on
  /// whether any rows are selected or not.
  ///
  /// These should be size 24.0 with default padding (8.0).
  final List<Widget>? actions;

  /// The configuration and labels for the columns in the table.
  final List<DataColumn> columns;

  /// The current primary sort key's column.
  ///
  /// See [DataTable.sortColumnIndex] for details.
  ///
  /// The direction of the sort is specified using [sortAscending].
  final int? sortColumnIndex;

  /// Whether the column mentioned in [sortColumnIndex], if any, is sorted
  /// in ascending order.
  ///
  /// See [DataTable.sortAscending] for details.
  final bool sortAscending;

  /// Invoked when the user selects or unselects every row, using the
  /// checkbox in the heading row.
  ///
  /// See [DataTable.onSelectAll].
  final ValueSetter<bool?>? onSelectAll;

  /// The height of each row (excluding the row that contains column headings).
  ///
  /// This value is optional and defaults to kMinInteractiveDimension if not
  /// specified.
  @Deprecated(
    'Migrate to use dataRowMinHeight and dataRowMaxHeight instead. '
    'This feature was deprecated after v3.7.0-5.0.pre.',
  )
  double? get dataRowHeight => dataRowMinHeight == dataRowMaxHeight ? dataRowMinHeight : null;

  /// The minimum height of each row (excluding the row that contains column headings).
  ///
  /// This value is optional and defaults to [kMinInteractiveDimension] if not
  /// specified.
  final double? dataRowMinHeight;

  /// The maximum height of each row (excluding the row that contains column headings).
  ///
  /// This value is optional and defaults to [kMinInteractiveDimension] if not
  /// specified.
  final double? dataRowMaxHeight;

  /// The height of the heading row.
  ///
  /// This value is optional and defaults to 56.0 if not specified.
  final double headingRowHeight;

  /// The horizontal margin between the edges of the table and the content
  /// in the first and last cells of each row.
  ///
  /// When a checkbox is displayed, it is also the margin between the checkbox
  /// the content in the first data column.
  ///
  /// This value defaults to 24.0 to adhere to the Material Design specifications.
  ///
  /// If [checkboxHorizontalMargin] is null, then [horizontalMargin] is also the
  /// margin between the edge of the table and the checkbox, as well as the
  /// margin between the checkbox and the content in the first data column.
  final double horizontalMargin;

  /// The horizontal margin between the contents of each data column.
  ///
  /// This value defaults to 56.0 to adhere to the Material Design specifications.
  final double columnSpacing;

  /// {@macro flutter.material.dataTable.showCheckboxColumn}
  final bool showCheckboxColumn;

  /// Flag to display the pagination buttons to go to the first and last pages.
  final bool showFirstLastButtons;

  /// The index of the first row to display when the widget is first created.
  final int? initialFirstRowIndex;

  /// {@macro flutter.material.dataTable.dividerThickness}
  ///
  /// If null, [DataTableThemeData.dividerThickness] is used. This value
  /// defaults to 1.0.
  final double? dividerThickness;

  /// Invoked when the user switches to another page.
  ///
  /// The value is the index of the first row on the currently displayed page.
  final ValueChanged<int>? onPageChanged;

  /// The number of rows to show on each page.
  ///
  /// See also:
  ///
  ///  * [onRowsPerPageChanged]
  ///  * [defaultRowsPerPage]
  final int rowsPerPage;

  /// The default value for [rowsPerPage].
  ///
  /// Useful when initializing the field that will hold the current
  /// [rowsPerPage], when implemented [onRowsPerPageChanged].
  static const int defaultRowsPerPage = 10;

  /// The options to offer for the rowsPerPage.
  ///
  /// The current [rowsPerPage] must be a value in this list.
  ///
  /// The values in this list should be sorted in ascending order.
  final List<int> availableRowsPerPage;

  /// Invoked when the user selects a different number of rows per page.
  ///
  /// If this is null, then the value given by [rowsPerPage] will be used
  /// and no affordance will be provided to change the value.
  final ValueChanged<int?>? onRowsPerPageChanged;

  /// The data source which provides data to show in each row.
  ///
  /// This object should generally have a lifetime longer than the
  /// [PaginatedDataTable] widget itself; it should be reused each time the
  /// [PaginatedDataTable] constructor is called.
  final DataTableSource source;

  /// {@macro flutter.widgets.scrollable.dragStartBehavior}
  final DragStartBehavior dragStartBehavior;

  /// Horizontal margin around the checkbox, if it is displayed.
  ///
  /// If null, then [horizontalMargin] is used as the margin between the edge
  /// of the table and the checkbox, as well as the margin between the checkbox
  /// and the content in the first data column. This value defaults to 24.0.
  final double? checkboxHorizontalMargin;

  /// Defines the color of the arrow heads in the footer.
  final Color? arrowHeadColor;

  /// {@macro flutter.widgets.scroll_view.controller}
  final ScrollController? controller;

  /// {@macro flutter.widgets.scroll_view.primary}
  final bool? primary;

  /// {@macro flutter.material.dataTable.headingRowColor}
  final MaterialStateProperty<Color?>? headingRowColor;

  /// Controls the visibility of empty rows on the last page of a
  /// [PaginatedDataTable].
  ///
  /// Defaults to `true`, which means empty rows will be populated on the
  /// last page of the table if there is not enough content.
  /// When set to `false`, empty rows will not be created.
  final bool showEmptyRows;

  @override
  PaginatedDataTableState createState() => PaginatedDataTableState();
}

/// Holds the state of a [PaginatedDataTable].
///
/// The table can be programmatically paged using the [pageTo] method.
class PaginatedDataTableState extends State<PaginatedDataTable> {
  late int _firstRowIndex;
  late int _rowCount;
  late bool _rowCountApproximate;
  int _selectedRowCount = 0;
  final Map<int, DataRow?> _rows = <int, DataRow?>{};

  @protected
  @override
  void initState() {
    super.initState();
    _firstRowIndex =
        PageStorage.maybeOf(context)?.readState(context) as int? ??
        widget.initialFirstRowIndex ??
        0;
    widget.source.addListener(_handleDataSourceChanged);
    _handleDataSourceChanged();
  }

  @protected
  @override
  void didUpdateWidget(PaginatedDataTable oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.source != widget.source) {
      oldWidget.source.removeListener(_handleDataSourceChanged);
      widget.source.addListener(_handleDataSourceChanged);
      _updateCaches();
    }
  }

  @protected
  @override
  void reassemble() {
    super.reassemble();
    // This function is called during hot reload.
    //
    // Normally, if the data source changes, it would notify its listeners and
    // thus trigger _handleDataSourceChanged(), which clears the row cache and
    // causes the widget to rebuild.
    //
    // During a hot reload, though, a data source can change in ways that will
    // invalidate the row cache (e.g. adding or removing columns) without ever
    // triggering a notification, leaving the PaginatedDataTable in an invalid
    // state. This method handles this case by clearing the cache any time the
    // widget is involved in a hot reload.
    _updateCaches();
  }

  @protected
  @override
  void dispose() {
    widget.source.removeListener(_handleDataSourceChanged);
    super.dispose();
  }

  void _handleDataSourceChanged() {
    setState(_updateCaches);
  }

  void _updateCaches() {
    _rowCount = widget.source.rowCount;
    _rowCountApproximate = widget.source.isRowCountApproximate;
    _selectedRowCount = widget.source.selectedRowCount;
    _rows.clear();
  }

  /// Ensures that the given row is visible.
  void pageTo(int rowIndex) {
    final int oldFirstRowIndex = _firstRowIndex;
    setState(() {
      final int rowsPerPage = widget.rowsPerPage;
      _firstRowIndex = (rowIndex ~/ rowsPerPage) * rowsPerPage;
    });
    if ((widget.onPageChanged != null) && (oldFirstRowIndex != _firstRowIndex)) {
      widget.onPageChanged!(_firstRowIndex);
    }
  }

  DataRow _getBlankRowFor(int index) {
    return DataRow.byIndex(
      index: index,
      cells: widget.columns.map<DataCell>((DataColumn column) => DataCell.empty).toList(),
    );
  }

  DataRow _getProgressIndicatorRowFor(int index) {
    bool haveProgressIndicator = false;
    final List<DataCell> cells =
        widget.columns.map<DataCell>((DataColumn column) {
          if (!column.numeric) {
            haveProgressIndicator = true;
            return const DataCell(CircularProgressIndicator());
          }
          return DataCell.empty;
        }).toList();
    if (!haveProgressIndicator) {
      haveProgressIndicator = true;
      cells[0] = const DataCell(CircularProgressIndicator());
    }
    return DataRow.byIndex(index: index, cells: cells);
  }

  List<DataRow> _getRows(int firstRowIndex, int rowsPerPage) {
    final List<DataRow> result = <DataRow>[];
    final int nextPageFirstRowIndex = firstRowIndex + rowsPerPage;
    bool haveProgressIndicator = false;
    for (int index = firstRowIndex; index < nextPageFirstRowIndex; index += 1) {
      DataRow? row;
      if (index < _rowCount || _rowCountApproximate) {
        row = _rows.putIfAbsent(index, () => widget.source.getRow(index));
        if (row == null && !haveProgressIndicator) {
          row ??= _getProgressIndicatorRowFor(index);
          haveProgressIndicator = true;
        }
      }

      if (widget.showEmptyRows) {
        row ??= _getBlankRowFor(index);
      }

      if (row != null) {
        result.add(row);
      }
    }
    return result;
  }

  void _handleFirst() {
    pageTo(0);
  }

  void _handlePrevious() {
    pageTo(math.max(_firstRowIndex - widget.rowsPerPage, 0));
  }

  void _handleNext() {
    pageTo(_firstRowIndex + widget.rowsPerPage);
  }

  void _handleLast() {
    pageTo(((_rowCount - 1) / widget.rowsPerPage).floor() * widget.rowsPerPage);
  }

  bool _isNextPageUnavailable() =>
      !_rowCountApproximate && (_firstRowIndex + widget.rowsPerPage >= _rowCount);

  final GlobalKey _tableKey = GlobalKey();

  @protected
  @override
  Widget build(BuildContext context) {
    // TODO(ianh): This whole build function doesn't handle RTL yet.
    assert(debugCheckHasMaterialLocalizations(context));
    final ThemeData themeData = Theme.of(context);
    final MaterialLocalizations localizations = MaterialLocalizations.of(context);
    // HEADER
    final List<Widget> headerWidgets = <Widget>[];
    if (_selectedRowCount == 0 && widget.header != null) {
      headerWidgets.add(Expanded(child: widget.header!));
    } else if (widget.header != null) {
      headerWidgets.add(
        Expanded(child: Text(localizations.selectedRowCountTitle(_selectedRowCount))),
      );
    }
    if (widget.actions != null) {
      headerWidgets.addAll(
        widget.actions!.map<Widget>((Widget action) {
          return Padding(
            // 8.0 is the default padding of an icon button
            padding: const EdgeInsetsDirectional.only(start: 24.0 - 8.0 * 2.0),
            child: action,
          );
        }).toList(),
      );
    }

    // FOOTER
    final TextStyle? footerTextStyle = themeData.textTheme.bodySmall;
    final List<Widget> footerWidgets = <Widget>[];
    if (widget.onRowsPerPageChanged != null) {
      final List<Widget> availableRowsPerPage =
          widget.availableRowsPerPage
              .where((int value) => value <= _rowCount || value == widget.rowsPerPage)
              .map<DropdownMenuItem<int>>((int value) {
                return DropdownMenuItem<int>(value: value, child: Text('$value'));
              })
              .toList();
      footerWidgets.addAll(<Widget>[
        // Match trailing padding, in case we overflow and end up scrolling.
        const SizedBox(width: 14.0),
        Text(localizations.rowsPerPageTitle),
        ConstrainedBox(
          constraints: const BoxConstraints(minWidth: 64.0), // 40.0 for the text, 24.0 for the icon
          child: Align(
            alignment: AlignmentDirectional.centerEnd,
            child: DropdownButtonHideUnderline(
              child: DropdownButton<int>(
                items: availableRowsPerPage.cast<DropdownMenuItem<int>>(),
                value: widget.rowsPerPage,
                onChanged: widget.onRowsPerPageChanged,
                style: footerTextStyle,
              ),
            ),
          ),
        ),
      ]);
    }
    footerWidgets.addAll(<Widget>[
      const SizedBox(width: 32.0),
      Text(
        localizations.pageRowsInfoTitle(
          _firstRowIndex + 1,
          math.min(_firstRowIndex + widget.rowsPerPage, _rowCount),
          _rowCount,
          _rowCountApproximate,
        ),
      ),
      const SizedBox(width: 32.0),
      if (widget.showFirstLastButtons)
        IconButton(
          icon: Icon(Icons.skip_previous, color: widget.arrowHeadColor),
          padding: EdgeInsets.zero,
          tooltip: localizations.firstPageTooltip,
          onPressed: _firstRowIndex <= 0 ? null : _handleFirst,
        ),
      IconButton(
        icon: Icon(Icons.chevron_left, color: widget.arrowHeadColor),
        padding: EdgeInsets.zero,
        tooltip: localizations.previousPageTooltip,
        onPressed: _firstRowIndex <= 0 ? null : _handlePrevious,
      ),
      const SizedBox(width: 24.0),
      IconButton(
        icon: Icon(Icons.chevron_right, color: widget.arrowHeadColor),
        padding: EdgeInsets.zero,
        tooltip: localizations.nextPageTooltip,
        onPressed: _isNextPageUnavailable() ? null : _handleNext,
      ),
      if (widget.showFirstLastButtons)
        IconButton(
          icon: Icon(Icons.skip_next, color: widget.arrowHeadColor),
          padding: EdgeInsets.zero,
          tooltip: localizations.lastPageTooltip,
          onPressed: _isNextPageUnavailable() ? null : _handleLast,
        ),
      const SizedBox(width: 14.0),
    ]);

    // CARD
    return Card(
      semanticContainer: false,
      child: LayoutBuilder(
        builder: (BuildContext context, BoxConstraints constraints) {
          return Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: <Widget>[
              if (headerWidgets.isNotEmpty)
                Semantics(
                  container: true,
                  child: DefaultTextStyle(
                    // These typographic styles aren't quite the regular ones. We pick the closest ones from the regular
                    // list and then tweak them appropriately.
                    // See https://material.io/design/components/data-tables.html#tables-within-cards
                    style:
                        _selectedRowCount > 0
                            ? themeData.textTheme.titleMedium!.copyWith(
                              color: themeData.colorScheme.secondary,
                            )
                            : themeData.textTheme.titleLarge!.copyWith(fontWeight: FontWeight.w400),
                    child: IconTheme.merge(
                      data: const IconThemeData(opacity: 0.54),
                      child: Ink(
                        height: 64.0,
                        color: _selectedRowCount > 0 ? themeData.secondaryHeaderColor : null,
                        child: Padding(
                          padding: const EdgeInsetsDirectional.only(start: 24, end: 14.0),
                          child: Row(
                            mainAxisAlignment: MainAxisAlignment.end,
                            children: headerWidgets,
                          ),
                        ),
                      ),
                    ),
                  ),
                ),
              SingleChildScrollView(
                scrollDirection: Axis.horizontal,
                primary: widget.primary,
                controller: widget.controller,
                dragStartBehavior: widget.dragStartBehavior,
                child: ConstrainedBox(
                  constraints: BoxConstraints(minWidth: constraints.minWidth),
                  child: DataTable(
                    key: _tableKey,
                    columns: widget.columns,
                    sortColumnIndex: widget.sortColumnIndex,
                    sortAscending: widget.sortAscending,
                    onSelectAll: widget.onSelectAll,
                    dividerThickness: widget.dividerThickness,
                    // Make sure no decoration is set on the DataTable
                    // from the theme, as its already wrapped in a Card.
                    decoration: const BoxDecoration(),
                    dataRowMinHeight: widget.dataRowMinHeight,
                    dataRowMaxHeight: widget.dataRowMaxHeight,
                    headingRowHeight: widget.headingRowHeight,
                    horizontalMargin: widget.horizontalMargin,
                    checkboxHorizontalMargin: widget.checkboxHorizontalMargin,
                    columnSpacing: widget.columnSpacing,
                    showCheckboxColumn: widget.showCheckboxColumn,
                    showBottomBorder: true,
                    rows: _getRows(_firstRowIndex, widget.rowsPerPage),
                    headingRowColor: widget.headingRowColor,
                  ),
                ),
              ),
              if (!widget.showEmptyRows)
                SizedBox(
                  height:
                      (widget.dataRowMaxHeight ?? kMinInteractiveDimension) *
                      (widget.rowsPerPage - _rowCount + _firstRowIndex).clamp(
                        0,
                        widget.rowsPerPage,
                      ),
                ),
              DefaultTextStyle(
                style: footerTextStyle!,
                child: IconTheme.merge(
                  data: const IconThemeData(opacity: 0.54),
                  child: SizedBox(
                    // TODO(bkonyi): this won't handle text zoom correctly,
                    //  https://github.com/flutter/flutter/issues/48522
                    height: 56.0,
                    child: SingleChildScrollView(
                      dragStartBehavior: widget.dragStartBehavior,
                      scrollDirection: Axis.horizontal,
                      reverse: true,
                      child: Row(children: footerWidgets),
                    ),
                  ),
                ),
              ),
            ],
          );
        },
      ),
    );
  }
}
