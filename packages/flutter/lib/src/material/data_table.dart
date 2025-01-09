// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'paginated_data_table.dart';
/// @docImport 'text_theme.dart';
library;

import 'dart:math' as math;

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

import 'checkbox.dart';
import 'constants.dart';
import 'data_table_theme.dart';
import 'debug.dart';
import 'divider.dart';
import 'dropdown.dart';
import 'icons.dart';
import 'ink_well.dart';
import 'material.dart';
import 'material_state.dart';
import 'theme.dart';
import 'tooltip.dart';

// Examples can assume:
// late BuildContext context;
// late List<DataColumn> _columns;
// late List<DataRow> _rows;

/// Signature for [DataColumn.onSort] callback.
typedef DataColumnSortCallback = void Function(int columnIndex, bool ascending);

/// Column configuration for a [DataTable].
///
/// One column configuration must be provided for each column to
/// display in the table. The list of [DataColumn] objects is passed
/// as the `columns` argument to the [DataTable.new] constructor.
@immutable
class DataColumn {
  /// Creates the configuration for a column of a [DataTable].
  const DataColumn({
    required this.label,
    this.columnWidth,
    this.tooltip,
    this.numeric = false,
    this.onSort,
    this.mouseCursor,
    this.headingRowAlignment,
  });

  /// The column heading.
  ///
  /// Typically, this will be a [Text] widget. It could also be an
  /// [Icon] (typically using size 18), or a [Row] with an icon and
  /// some text.
  ///
  /// The [label] is placed within a [Row] along with the
  /// sort indicator (if applicable). By default, [label] only occupy minimal
  /// space. It is recommended to place the label content in an [Expanded] or
  /// [Flexible] as [label] to control how the content flexes. Otherwise,
  /// an exception will occur when the available space is insufficient.
  ///
  /// By default, [DefaultTextStyle.softWrap] of this subtree will be set to false.
  /// Use [DefaultTextStyle.merge] to override it if needed.
  ///
  /// The label should not include the sort indicator.
  final Widget label;

  /// How the horizontal extents of this column of the table should be determined.
  ///
  /// The [FixedColumnWidth] class can be used to specify a specific width in
  /// pixels. This is the cheapest way to size a table's columns.
  ///
  /// The layout performance of the table depends critically on which column
  /// sizing algorithms are used here. In particular, [IntrinsicColumnWidth] is
  /// quite expensive because it needs to measure each cell in the column to
  /// determine the intrinsic size of the column.
  ///
  /// If this property is `null`, the table applies a default behavior:
  /// - If the table has exactly one column identified as the only text column
  ///   (i.e., all the rest are numeric), that column uses `IntrinsicColumnWidth(flex: 1.0)`.
  /// - All other columns use `IntrinsicColumnWidth()`.
  final TableColumnWidth? columnWidth;

  /// The column heading's tooltip.
  ///
  /// This is a longer description of the column heading, for cases
  /// where the heading might have been abbreviated to keep the column
  /// width to a reasonable size.
  final String? tooltip;

  /// Whether this column represents numeric data or not.
  ///
  /// The contents of cells of columns containing numeric data are
  /// right-aligned.
  final bool numeric;

  /// Called when the user asks to sort the table using this column.
  ///
  /// If null, the column will not be considered sortable.
  ///
  /// See [DataTable.sortColumnIndex] and [DataTable.sortAscending].
  final DataColumnSortCallback? onSort;

  bool get _debugInteractive => onSort != null;

  /// The cursor for a mouse pointer when it enters or is hovering over the
  /// heading row.
  ///
  /// [WidgetStateProperty.resolve] is used for the following [WidgetState]s:
  ///
  ///  * [WidgetState.disabled].
  ///
  /// If this is null, then the value of [DataTableThemeData.headingCellCursor]
  /// is used. If that's null, then [WidgetStateMouseCursor.clickable] is used.
  ///
  /// See also:
  ///  * [WidgetStateMouseCursor], which can be used to create a [MouseCursor].
  final MaterialStateProperty<MouseCursor?>? mouseCursor;

  /// Defines the horizontal layout of the [label] and sort indicator in the
  /// heading row.
  ///
  /// If [headingRowAlignment] value is [MainAxisAlignment.center] and [onSort] is
  /// not null, then a [SizedBox] with a width of sort arrow icon size and sort
  /// arrow padding will be placed before the [label] to ensure the label is
  /// centered in the column.
  ///
  /// If null, then defaults to [MainAxisAlignment.start].
  final MainAxisAlignment? headingRowAlignment;
}

/// Row configuration and cell data for a [DataTable].
///
/// One row configuration must be provided for each row to
/// display in the table. The list of [DataRow] objects is passed
/// as the `rows` argument to the [DataTable.new] constructor.
///
/// The data for this row of the table is provided in the [cells]
/// property of the [DataRow] object.
@immutable
class DataRow {
  /// Creates the configuration for a row of a [DataTable].
  const DataRow({
    this.key,
    this.selected = false,
    this.onSelectChanged,
    this.onLongPress,
    this.color,
    this.mouseCursor,
    required this.cells,
  });

  /// Creates the configuration for a row of a [DataTable], deriving
  /// the key from a row index.
  DataRow.byIndex({
    int? index,
    this.selected = false,
    this.onSelectChanged,
    this.onLongPress,
    this.color,
    this.mouseCursor,
    required this.cells,
  }) : key = ValueKey<int?>(index);

  /// A [Key] that uniquely identifies this row. This is used to
  /// ensure that if a row is added or removed, any stateful widgets
  /// related to this row (e.g. an in-progress checkbox animation)
  /// remain on the right row visually.
  ///
  /// If the table never changes once created, no key is necessary.
  final LocalKey? key;

  /// Called when the user selects or unselects a selectable row.
  ///
  /// If this is not null, then the row is selectable. The current
  /// selection state of the row is given by [selected].
  ///
  /// If any row is selectable, then the table's heading row will have
  /// a checkbox that can be checked to select all selectable rows
  /// (and which is checked if all the rows are selected), and each
  /// subsequent row will have a checkbox to toggle just that row.
  ///
  /// A row whose [onSelectChanged] callback is null is ignored for
  /// the purposes of determining the state of the "all" checkbox,
  /// and its checkbox is disabled.
  ///
  /// If a [DataCell] in the row has its [DataCell.onTap] callback defined,
  /// that callback behavior overrides the gesture behavior of the row for
  /// that particular cell.
  final ValueChanged<bool?>? onSelectChanged;

  /// Called if the row is long-pressed.
  ///
  /// If a [DataCell] in the row has its [DataCell.onTap], [DataCell.onDoubleTap],
  /// [DataCell.onLongPress], [DataCell.onTapCancel] or [DataCell.onTapDown] callback defined,
  /// that callback behavior overrides the gesture behavior of the row for
  /// that particular cell.
  final GestureLongPressCallback? onLongPress;

  /// Whether the row is selected.
  ///
  /// If [onSelectChanged] is non-null for any row in the table, then
  /// a checkbox is shown at the start of each row. If the row is
  /// selected (true), the checkbox will be checked and the row will
  /// be highlighted.
  ///
  /// Otherwise, the checkbox, if present, will not be checked.
  final bool selected;

  /// The data for this row.
  ///
  /// There must be exactly as many cells as there are columns in the
  /// table.
  final List<DataCell> cells;

  /// The color for the row.
  ///
  /// By default, the color is transparent unless selected. Selected rows has
  /// a grey translucent color.
  ///
  /// The effective color can depend on the [WidgetState] state, if the
  /// row is selected, pressed, hovered, focused, disabled or enabled. The
  /// color is painted as an overlay to the row. To make sure that the row's
  /// [InkWell] is visible (when pressed, hovered and focused), it is
  /// recommended to use a translucent color.
  ///
  /// If [onSelectChanged] or [onLongPress] is null, the row's [InkWell] will be disabled.
  ///
  /// ```dart
  /// DataRow(
  ///   color: WidgetStateProperty.resolveWith<Color?>((Set<WidgetState> states) {
  ///     if (states.contains(WidgetState.selected)) {
  ///       return Theme.of(context).colorScheme.primary.withOpacity(0.08);
  ///     }
  ///     return null;  // Use the default value.
  ///   }),
  ///   cells: const <DataCell>[
  ///     // ...
  ///   ],
  /// )
  /// ```
  ///
  /// See also:
  ///
  ///  * The Material Design specification for overlay colors and how they
  ///    match a component's state:
  ///    <https://material.io/design/interaction/states.html#anatomy>.
  final MaterialStateProperty<Color?>? color;

  /// The cursor for a mouse pointer when it enters or is hovering over the
  /// data row.
  ///
  /// [WidgetStateProperty.resolve] is used for the following [WidgetState]s:
  ///
  ///  * [WidgetState.selected].
  ///
  /// If this is null, then the value of [DataTableThemeData.dataRowCursor]
  /// is used. If that's null, then [WidgetStateMouseCursor.clickable] is used.
  ///
  /// See also:
  ///  * [WidgetStateMouseCursor], which can be used to create a [MouseCursor].
  final MaterialStateProperty<MouseCursor?>? mouseCursor;

  bool get _debugInteractive =>
      onSelectChanged != null || cells.any((DataCell cell) => cell._debugInteractive);
}

/// The data for a cell of a [DataTable].
///
/// One list of [DataCell] objects must be provided for each [DataRow]
/// in the [DataTable], in the new [DataRow] constructor's `cells`
/// argument.
@immutable
class DataCell {
  /// Creates an object to hold the data for a cell in a [DataTable].
  ///
  /// The first argument is the widget to show for the cell, typically
  /// a [Text] or [DropdownButton] widget.
  ///
  /// If the cell has no data, then a [Text] widget with placeholder
  /// text should be provided instead, and then the [placeholder]
  /// argument should be set to true.
  const DataCell(
    this.child, {
    this.placeholder = false,
    this.showEditIcon = false,
    this.onTap,
    this.onLongPress,
    this.onTapDown,
    this.onDoubleTap,
    this.onTapCancel,
  });

  /// A cell that has no content and has zero width and height.
  static const DataCell empty = DataCell(SizedBox.shrink());

  /// The data for the row.
  ///
  /// Typically a [Text] widget or a [DropdownButton] widget.
  ///
  /// If the cell has no data, then a [Text] widget with placeholder
  /// text should be provided instead, and [placeholder] should be set
  /// to true.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  /// Whether the [child] is actually a placeholder.
  ///
  /// If this is true, the default text style for the cell is changed
  /// to be appropriate for placeholder text.
  final bool placeholder;

  /// Whether to show an edit icon at the end of the cell.
  ///
  /// This does not make the cell actually editable; the caller must
  /// implement editing behavior if desired (initiated from the
  /// [onTap] callback).
  ///
  /// If this is set, [onTap] should also be set, otherwise tapping
  /// the icon will have no effect.
  final bool showEditIcon;

  /// Called if the cell is tapped.
  ///
  /// If non-null, tapping the cell will call this callback. If
  /// null (including [onDoubleTap], [onLongPress], [onTapCancel] and [onTapDown]),
  /// tapping the cell will attempt to select the row (if
  /// [DataRow.onSelectChanged] is provided).
  final GestureTapCallback? onTap;

  /// Called when the cell is double tapped.
  ///
  /// If non-null, tapping the cell will call this callback. If
  /// null (including [onTap], [onLongPress], [onTapCancel] and [onTapDown]),
  /// tapping the cell will attempt to select the row (if
  /// [DataRow.onSelectChanged] is provided).
  final GestureTapCallback? onDoubleTap;

  /// Called if the cell is long-pressed.
  ///
  /// If non-null, tapping the cell will invoke this callback. If
  /// null (including [onDoubleTap], [onTap], [onTapCancel] and [onTapDown]),
  /// tapping the cell will attempt to select the row (if
  /// [DataRow.onSelectChanged] is provided).
  final GestureLongPressCallback? onLongPress;

  /// Called if the cell is tapped down.
  ///
  /// If non-null, tapping the cell will call this callback. If
  /// null (including [onTap] [onDoubleTap], [onLongPress] and [onTapCancel]),
  /// tapping the cell will attempt to select the row (if
  /// [DataRow.onSelectChanged] is provided).
  final GestureTapDownCallback? onTapDown;

  /// Called if the user cancels a tap was started on cell.
  ///
  /// If non-null, canceling the tap gesture will invoke this callback.
  /// If null (including [onTap], [onDoubleTap] and [onLongPress]),
  /// tapping the cell will attempt to select the
  /// row (if [DataRow.onSelectChanged] is provided).
  final GestureTapCancelCallback? onTapCancel;

  bool get _debugInteractive =>
      onTap != null ||
      onDoubleTap != null ||
      onLongPress != null ||
      onTapDown != null ||
      onTapCancel != null;
}

/// A data table that follows the
/// [Material 2](https://material.io/go/design-data-tables)
/// design specification.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=ktTajqbhIcY}
///
/// ## Performance considerations
///
/// Columns are sized automatically based on the table's contents.
/// It's expensive to display large amounts of data with this widget,
/// since it must be measured twice: once to negotiate each column's
/// dimensions, and again when the table is laid out.
///
/// A [SingleChildScrollView] mounts and paints the entire child, even
/// when only some of it is visible. For a table that effectively handles
/// large amounts of data, here are some other options to consider:
///
///  * `TableView`, a widget from the
///    [two_dimensional_scrollables](https://pub.dev/packages/two_dimensional_scrollables)
///    package.
///  * [PaginatedDataTable], which automatically splits the data into
///    multiple pages.
///  * [CustomScrollView], for greater control over scrolling effects.
///
/// {@tool dartpad}
/// This sample shows how to display a [DataTable] with three columns: name, age, and
/// role. The columns are defined by three [DataColumn] objects. The table
/// contains three rows of data for three example users, the data for which
/// is defined by three [DataRow] objects.
///
/// ![](https://flutter.github.io/assets-for-api-docs/assets/material/data_table.png)
///
/// ** See code in examples/api/lib/material/data_table/data_table.0.dart **
/// {@end-tool}
///
///
/// {@tool dartpad}
/// This sample shows how to display a [DataTable] with alternate colors per
/// row, and a custom color for when the row is selected.
///
/// ** See code in examples/api/lib/material/data_table/data_table.1.dart **
/// {@end-tool}
///
/// [DataTable] can be sorted on the basis of any column in [columns] in
/// ascending or descending order. If [sortColumnIndex] is non-null, then the
/// table will be sorted by the values in the specified column. The boolean
/// [sortAscending] flag controls the sort order.
///
/// See also:
///
///  * [DataColumn], which describes a column in the data table.
///  * [DataRow], which contains the data for a row in the data table.
///  * [DataCell], which contains the data for a single cell in the data table.
///  * [PaginatedDataTable], which shows part of the data in a data table and
///    provides controls for paging through the remainder of the data.
///  * `TableView` from the
///    [two_dimensional_scrollables](https://pub.dev/packages/two_dimensional_scrollables)
///    package, for displaying large amounts of data without pagination.
///  * <https://material.io/go/design-data-tables>
class DataTable extends StatelessWidget {
  /// Creates a widget describing a data table.
  ///
  /// The [columns] argument must be a list of as many [DataColumn]
  /// objects as the table is to have columns, ignoring the leading
  /// checkbox column if any. The [columns] argument must have a
  /// length greater than zero.
  ///
  /// The [rows] argument must be a list of as many [DataRow] objects
  /// as the table is to have rows, ignoring the leading heading row
  /// that contains the column headings (derived from the [columns]
  /// argument). There may be zero rows, but the rows argument must
  /// not be null.
  ///
  /// Each [DataRow] object in [rows] must have as many [DataCell]
  /// objects in the [DataRow.cells] list as the table has columns.
  ///
  /// If the table is sorted, the column that provides the current
  /// primary key should be specified by index in [sortColumnIndex], 0
  /// meaning the first column in [columns], 1 being the next one, and
  /// so forth.
  ///
  /// The actual sort order can be specified using [sortAscending]; if
  /// the sort order is ascending, this should be true (the default),
  /// otherwise it should be false.
  DataTable({
    super.key,
    required this.columns,
    this.sortColumnIndex,
    this.sortAscending = true,
    this.onSelectAll,
    this.decoration,
    this.dataRowColor,
    @Deprecated(
      'Migrate to use dataRowMinHeight and dataRowMaxHeight instead. '
      'This feature was deprecated after v3.7.0-5.0.pre.',
    )
    double? dataRowHeight,
    double? dataRowMinHeight,
    double? dataRowMaxHeight,
    this.dataTextStyle,
    this.headingRowColor,
    this.headingRowHeight,
    this.headingTextStyle,
    this.horizontalMargin,
    this.columnSpacing,
    this.showCheckboxColumn = true,
    this.showBottomBorder = false,
    this.dividerThickness,
    required this.rows,
    this.checkboxHorizontalMargin,
    this.border,
    this.clipBehavior = Clip.none,
  }) : assert(columns.isNotEmpty),
       assert(
         sortColumnIndex == null || (sortColumnIndex >= 0 && sortColumnIndex < columns.length),
       ),
       assert(
         !rows.any((DataRow row) => row.cells.length != columns.length),
         'All rows must have the same number of cells as there are header cells (${columns.length})',
       ),
       assert(dividerThickness == null || dividerThickness >= 0),
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
       _onlyTextColumn = _initOnlyTextColumn(columns);

  /// The configuration and labels for the columns in the table.
  final List<DataColumn> columns;

  /// The current primary sort key's column.
  ///
  /// If non-null, indicates that the indicated column is the column
  /// by which the data is sorted. The number must correspond to the
  /// index of the relevant column in [columns].
  ///
  /// Setting this will cause the relevant column to have a sort
  /// indicator displayed.
  ///
  /// When this is null, it implies that the table's sort order does
  /// not correspond to any of the columns.
  ///
  /// The direction of the sort is specified using [sortAscending].
  final int? sortColumnIndex;

  /// Whether the column mentioned in [sortColumnIndex], if any, is sorted
  /// in ascending order.
  ///
  /// If true, the order is ascending (meaning the rows with the
  /// smallest values for the current sort column are first in the
  /// table).
  ///
  /// If false, the order is descending (meaning the rows with the
  /// smallest values for the current sort column are last in the
  /// table).
  ///
  /// Ascending order is represented by an upwards-facing arrow.
  final bool sortAscending;

  /// Invoked when the user selects or unselects every row, using the
  /// checkbox in the heading row.
  ///
  /// If this is null, then the [DataRow.onSelectChanged] callback of
  /// every row in the table is invoked appropriately instead.
  ///
  /// To control whether a particular row is selectable or not, see
  /// [DataRow.onSelectChanged]. This callback is only relevant if any
  /// row is selectable.
  final ValueSetter<bool?>? onSelectAll;

  /// {@template flutter.material.dataTable.decoration}
  /// The background and border decoration for the table.
  /// {@endtemplate}
  ///
  /// If null, [DataTableThemeData.decoration] is used. By default there is no
  /// decoration.
  final Decoration? decoration;

  /// {@template flutter.material.dataTable.dataRowColor}
  /// The background color for the data rows.
  ///
  /// The effective background color can be made to depend on the
  /// [WidgetState] state, i.e. if the row is selected, pressed, hovered,
  /// focused, disabled or enabled. The color is painted as an overlay to the
  /// row. To make sure that the row's [InkWell] is visible (when pressed,
  /// hovered and focused), it is recommended to use a translucent background
  /// color.
  ///
  /// If [DataRow.onSelectChanged] or [DataRow.onLongPress] is null, the row's
  /// [InkWell] will be disabled.
  /// {@endtemplate}
  ///
  /// If null, [DataTableThemeData.dataRowColor] is used. By default, the
  /// background color is transparent unless selected. Selected rows have a grey
  /// translucent color. To set a different color for individual rows, see
  /// [DataRow.color].
  ///
  /// {@template flutter.material.DataTable.dataRowColor}
  /// ```dart
  /// DataTable(
  ///   dataRowColor: WidgetStateProperty.resolveWith<Color?>((Set<WidgetState> states) {
  ///     if (states.contains(WidgetState.selected)) {
  ///       return Theme.of(context).colorScheme.primary.withOpacity(0.08);
  ///     }
  ///     return null;  // Use the default value.
  ///   }),
  ///   columns: _columns,
  ///   rows: _rows,
  /// )
  /// ```
  ///
  /// See also:
  ///
  ///  * The Material Design specification for overlay colors and how they
  ///    match a component's state:
  ///    <https://material.io/design/interaction/states.html#anatomy>.
  /// {@endtemplate}
  final MaterialStateProperty<Color?>? dataRowColor;

  /// {@template flutter.material.dataTable.dataRowHeight}
  /// The height of each row (excluding the row that contains column headings).
  /// {@endtemplate}
  ///
  /// If null, [DataTableThemeData.dataRowHeight] is used. This value defaults
  /// to [kMinInteractiveDimension] to adhere to the Material Design
  /// specifications.
  @Deprecated(
    'Migrate to use dataRowMinHeight and dataRowMaxHeight instead. '
    'This feature was deprecated after v3.7.0-5.0.pre.',
  )
  double? get dataRowHeight => dataRowMinHeight == dataRowMaxHeight ? dataRowMinHeight : null;

  /// {@template flutter.material.dataTable.dataRowMinHeight}
  /// The minimum height of each row (excluding the row that contains column headings).
  /// {@endtemplate}
  ///
  /// If null, [DataTableThemeData.dataRowMinHeight] is used. This value defaults
  /// to [kMinInteractiveDimension] to adhere to the Material Design
  /// specifications.
  final double? dataRowMinHeight;

  /// {@template flutter.material.dataTable.dataRowMaxHeight}
  /// The maximum height of each row (excluding the row that contains column headings).
  /// {@endtemplate}
  ///
  /// If null, [DataTableThemeData.dataRowMaxHeight] is used. This value defaults
  /// to [kMinInteractiveDimension] to adhere to the Material Design
  /// specifications.
  final double? dataRowMaxHeight;

  /// {@template flutter.material.dataTable.dataTextStyle}
  /// The text style for data rows.
  /// {@endtemplate}
  ///
  /// If null, [DataTableThemeData.dataTextStyle] is used. By default, the text
  /// style is [TextTheme.bodyMedium].
  final TextStyle? dataTextStyle;

  /// {@template flutter.material.dataTable.headingRowColor}
  /// The background color for the heading row.
  ///
  /// The effective background color can be made to depend on the
  /// [WidgetState] state, i.e. if the row is pressed, hovered, focused when
  /// sorted. The color is painted as an overlay to the row. To make sure that
  /// the row's [InkWell] is visible (when pressed, hovered and focused), it is
  /// recommended to use a translucent color.
  /// {@endtemplate}
  ///
  /// If null, [DataTableThemeData.headingRowColor] is used.
  ///
  /// {@template flutter.material.DataTable.headingRowColor}
  /// ```dart
  /// DataTable(
  ///   columns: _columns,
  ///   rows: _rows,
  ///   headingRowColor: WidgetStateProperty.resolveWith<Color?>((Set<WidgetState> states) {
  ///     if (states.contains(WidgetState.hovered)) {
  ///       return Theme.of(context).colorScheme.primary.withOpacity(0.08);
  ///     }
  ///     return null;  // Use the default value.
  ///   }),
  /// )
  /// ```
  ///
  /// See also:
  ///
  ///  * The Material Design specification for overlay colors and how they
  ///    match a component's state:
  ///    <https://material.io/design/interaction/states.html#anatomy>.
  /// {@endtemplate}
  final MaterialStateProperty<Color?>? headingRowColor;

  /// {@template flutter.material.dataTable.headingRowHeight}
  /// The height of the heading row.
  /// {@endtemplate}
  ///
  /// If null, [DataTableThemeData.headingRowHeight] is used. This value
  /// defaults to 56.0 to adhere to the Material Design specifications.
  final double? headingRowHeight;

  /// {@template flutter.material.dataTable.headingTextStyle}
  /// The text style for the heading row.
  /// {@endtemplate}
  ///
  /// If null, [DataTableThemeData.headingTextStyle] is used. By default, the
  /// text style is [TextTheme.titleSmall].
  final TextStyle? headingTextStyle;

  /// {@template flutter.material.dataTable.horizontalMargin}
  /// The horizontal margin between the edges of the table and the content
  /// in the first and last cells of each row.
  ///
  /// When a checkbox is displayed, it is also the margin between the checkbox
  /// the content in the first data column.
  /// {@endtemplate}
  ///
  /// If null, [DataTableThemeData.horizontalMargin] is used. This value
  /// defaults to 24.0 to adhere to the Material Design specifications.
  ///
  /// If [checkboxHorizontalMargin] is null, then [horizontalMargin] is also the
  /// margin between the edge of the table and the checkbox, as well as the
  /// margin between the checkbox and the content in the first data column.
  final double? horizontalMargin;

  /// {@template flutter.material.dataTable.columnSpacing}
  /// The horizontal margin between the contents of each data column.
  /// {@endtemplate}
  ///
  /// If null, [DataTableThemeData.columnSpacing] is used. This value defaults
  /// to 56.0 to adhere to the Material Design specifications.
  final double? columnSpacing;

  /// {@template flutter.material.dataTable.showCheckboxColumn}
  /// Whether the widget should display checkboxes for selectable rows.
  ///
  /// If true, a [Checkbox] will be placed at the beginning of each row that is
  /// selectable. However, if [DataRow.onSelectChanged] is not set for any row,
  /// checkboxes will not be placed, even if this value is true.
  ///
  /// If false, all rows will not display a [Checkbox].
  /// {@endtemplate}
  final bool showCheckboxColumn;

  /// The data to show in each row (excluding the row that contains
  /// the column headings).
  ///
  /// The list may be empty.
  final List<DataRow> rows;

  /// {@template flutter.material.dataTable.dividerThickness}
  /// The width of the divider that appears between [TableRow]s.
  ///
  /// Must be greater than or equal to zero.
  /// {@endtemplate}
  ///
  /// If null, [DataTableThemeData.dividerThickness] is used. This value
  /// defaults to 1.0.
  final double? dividerThickness;

  /// Whether a border at the bottom of the table is displayed.
  ///
  /// By default, a border is not shown at the bottom to allow for a border
  /// around the table defined by [decoration].
  final bool showBottomBorder;

  /// {@template flutter.material.dataTable.checkboxHorizontalMargin}
  /// Horizontal margin around the checkbox, if it is displayed.
  /// {@endtemplate}
  ///
  /// If null, [DataTableThemeData.checkboxHorizontalMargin] is used. If that is
  /// also null, then [horizontalMargin] is used as the margin between the edge
  /// of the table and the checkbox, as well as the margin between the checkbox
  /// and the content in the first data column. This value defaults to 24.0.
  final double? checkboxHorizontalMargin;

  /// The style to use when painting the boundary and interior divisions of the table.
  final TableBorder? border;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// This can be used to clip the content within the border of the [DataTable].
  ///
  /// Defaults to [Clip.none].
  final Clip clipBehavior;

  // Set by the constructor to the index of the only Column that is
  // non-numeric, if there is exactly one, otherwise null.
  final int? _onlyTextColumn;
  static int? _initOnlyTextColumn(List<DataColumn> columns) {
    int? result;
    for (int index = 0; index < columns.length; index += 1) {
      final DataColumn column = columns[index];
      if (!column.numeric) {
        if (result != null) {
          return null;
        }
        result = index;
      }
    }
    return result;
  }

  bool get _debugInteractive {
    return columns.any((DataColumn column) => column._debugInteractive) ||
        rows.any((DataRow row) => row._debugInteractive);
  }

  static final LocalKey _headingRowKey = UniqueKey();

  void _handleSelectAll(bool? checked, bool someChecked) {
    // If some checkboxes are checked, all checkboxes are selected. Otherwise,
    // use the new checked value but default to false if it's null.
    final bool effectiveChecked = someChecked || (checked ?? false);
    if (onSelectAll != null) {
      onSelectAll!(effectiveChecked);
    } else {
      for (final DataRow row in rows) {
        if (row.onSelectChanged != null && row.selected != effectiveChecked) {
          row.onSelectChanged!(effectiveChecked);
        }
      }
    }
  }

  /// The default height of the heading row.
  static const double _headingRowHeight = 56.0;

  /// The default horizontal margin between the edges of the table and the content
  /// in the first and last cells of each row.
  static const double _horizontalMargin = 24.0;

  /// The default horizontal margin between the contents of each data column.
  static const double _columnSpacing = 56.0;

  /// The default padding between the heading content and sort arrow.
  static const double _sortArrowPadding = 2.0;

  /// The default divider thickness.
  static const double _dividerThickness = 1.0;

  static const Duration _sortArrowAnimationDuration = Duration(milliseconds: 150);

  Widget _buildCheckbox({
    required BuildContext context,
    required bool? checked,
    required VoidCallback? onRowTap,
    required ValueChanged<bool?>? onCheckboxChanged,
    required MaterialStateProperty<Color?>? overlayColor,
    required bool tristate,
    MouseCursor? rowMouseCursor,
  }) {
    final ThemeData themeData = Theme.of(context);
    final double effectiveHorizontalMargin =
        horizontalMargin ?? themeData.dataTableTheme.horizontalMargin ?? _horizontalMargin;
    final double effectiveCheckboxHorizontalMarginStart =
        checkboxHorizontalMargin ??
        themeData.dataTableTheme.checkboxHorizontalMargin ??
        effectiveHorizontalMargin;
    final double effectiveCheckboxHorizontalMarginEnd =
        checkboxHorizontalMargin ??
        themeData.dataTableTheme.checkboxHorizontalMargin ??
        effectiveHorizontalMargin / 2.0;
    Widget contents = Semantics(
      container: true,
      child: Padding(
        padding: EdgeInsetsDirectional.only(
          start: effectiveCheckboxHorizontalMarginStart,
          end: effectiveCheckboxHorizontalMarginEnd,
        ),
        child: Center(
          child: Checkbox(value: checked, onChanged: onCheckboxChanged, tristate: tristate),
        ),
      ),
    );
    if (onRowTap != null) {
      contents = TableRowInkWell(
        onTap: onRowTap,
        overlayColor: overlayColor,
        mouseCursor: rowMouseCursor,
        child: contents,
      );
    }
    return TableCell(verticalAlignment: TableCellVerticalAlignment.fill, child: contents);
  }

  Widget _buildHeadingCell({
    required BuildContext context,
    required EdgeInsetsGeometry padding,
    required Widget label,
    required String? tooltip,
    required bool numeric,
    required VoidCallback? onSort,
    required bool sorted,
    required bool ascending,
    required MaterialStateProperty<Color?>? overlayColor,
    required MouseCursor? mouseCursor,
    required MainAxisAlignment headingRowAlignment,
  }) {
    final ThemeData themeData = Theme.of(context);
    final DataTableThemeData dataTableTheme = DataTableTheme.of(context);
    label = Row(
      textDirection: numeric ? TextDirection.rtl : null,
      mainAxisAlignment: headingRowAlignment,
      children: <Widget>[
        if (headingRowAlignment == MainAxisAlignment.center && onSort != null)
          const SizedBox(width: _SortArrowState._arrowIconSize + _sortArrowPadding),
        label,
        if (onSort != null) ...<Widget>[
          _SortArrow(
            visible: sorted,
            up: sorted ? ascending : null,
            duration: _sortArrowAnimationDuration,
          ),
          const SizedBox(width: _sortArrowPadding),
        ],
      ],
    );

    final TextStyle effectiveHeadingTextStyle =
        headingTextStyle ??
        dataTableTheme.headingTextStyle ??
        themeData.dataTableTheme.headingTextStyle ??
        themeData.textTheme.titleSmall!;
    final double effectiveHeadingRowHeight =
        headingRowHeight ??
        dataTableTheme.headingRowHeight ??
        themeData.dataTableTheme.headingRowHeight ??
        _headingRowHeight;
    label = Container(
      padding: padding,
      height: effectiveHeadingRowHeight,
      alignment: numeric ? Alignment.centerRight : AlignmentDirectional.centerStart,
      child: AnimatedDefaultTextStyle(
        style: DefaultTextStyle.of(context).style.merge(effectiveHeadingTextStyle),
        softWrap: false,
        duration: _sortArrowAnimationDuration,
        child: label,
      ),
    );
    if (tooltip != null) {
      label = Tooltip(message: tooltip, child: label);
    }

    // TODO(dkwingsmt): Only wrap Inkwell if onSort != null. Blocked by
    // https://github.com/flutter/flutter/issues/51152
    label = InkWell(
      onTap: onSort,
      overlayColor: overlayColor,
      mouseCursor: mouseCursor,
      child: label,
    );
    return label;
  }

  Widget _buildDataCell({
    required BuildContext context,
    required EdgeInsetsGeometry padding,
    required Widget label,
    required bool numeric,
    required bool placeholder,
    required bool showEditIcon,
    required GestureTapCallback? onTap,
    required VoidCallback? onSelectChanged,
    required GestureTapCallback? onDoubleTap,
    required GestureLongPressCallback? onLongPress,
    required GestureTapDownCallback? onTapDown,
    required GestureTapCancelCallback? onTapCancel,
    required MaterialStateProperty<Color?>? overlayColor,
    required GestureLongPressCallback? onRowLongPress,
    required MouseCursor? mouseCursor,
  }) {
    final ThemeData themeData = Theme.of(context);
    final DataTableThemeData dataTableTheme = DataTableTheme.of(context);
    if (showEditIcon) {
      const Widget icon = Icon(Icons.edit, size: 18.0);
      label = Expanded(child: label);
      label = Row(
        textDirection: numeric ? TextDirection.rtl : null,
        children: <Widget>[label, icon],
      );
    }

    final TextStyle effectiveDataTextStyle =
        dataTextStyle ??
        dataTableTheme.dataTextStyle ??
        themeData.dataTableTheme.dataTextStyle ??
        themeData.textTheme.bodyMedium!;
    final double effectiveDataRowMinHeight =
        dataRowMinHeight ??
        dataTableTheme.dataRowMinHeight ??
        themeData.dataTableTheme.dataRowMinHeight ??
        kMinInteractiveDimension;
    final double effectiveDataRowMaxHeight =
        dataRowMaxHeight ??
        dataTableTheme.dataRowMaxHeight ??
        themeData.dataTableTheme.dataRowMaxHeight ??
        kMinInteractiveDimension;
    label = Container(
      padding: padding,
      constraints: BoxConstraints(
        minHeight: effectiveDataRowMinHeight,
        maxHeight: effectiveDataRowMaxHeight,
      ),
      alignment: numeric ? Alignment.centerRight : AlignmentDirectional.centerStart,
      child: DefaultTextStyle(
        style: DefaultTextStyle.of(context).style
            .merge(effectiveDataTextStyle)
            .copyWith(color: placeholder ? effectiveDataTextStyle.color!.withOpacity(0.6) : null),
        child: DropdownButtonHideUnderline(child: label),
      ),
    );
    if (onTap != null ||
        onDoubleTap != null ||
        onLongPress != null ||
        onTapDown != null ||
        onTapCancel != null) {
      label = InkWell(
        onTap: onTap,
        onDoubleTap: onDoubleTap,
        onLongPress: onLongPress,
        onTapCancel: onTapCancel,
        onTapDown: onTapDown,
        overlayColor: overlayColor,
        child: label,
      );
    } else if (onSelectChanged != null || onRowLongPress != null) {
      label = TableRowInkWell(
        onTap: onSelectChanged,
        onLongPress: onRowLongPress,
        overlayColor: overlayColor,
        mouseCursor: mouseCursor,
        child: label,
      );
    }
    return label;
  }

  @override
  Widget build(BuildContext context) {
    assert(!_debugInteractive || debugCheckHasMaterial(context));

    final ThemeData theme = Theme.of(context);
    final DataTableThemeData dataTableTheme = DataTableTheme.of(context);
    final MaterialStateProperty<Color?>? effectiveHeadingRowColor =
        headingRowColor ?? dataTableTheme.headingRowColor ?? theme.dataTableTheme.headingRowColor;
    final MaterialStateProperty<Color?>? effectiveDataRowColor =
        dataRowColor ?? dataTableTheme.dataRowColor ?? theme.dataTableTheme.dataRowColor;
    final MaterialStateProperty<Color?> defaultRowColor = MaterialStateProperty.resolveWith((
      Set<MaterialState> states,
    ) {
      if (states.contains(MaterialState.selected)) {
        return theme.colorScheme.primary.withOpacity(0.08);
      }
      return null;
    });
    final bool anyRowSelectable = rows.any((DataRow row) => row.onSelectChanged != null);
    final bool displayCheckboxColumn = showCheckboxColumn && anyRowSelectable;
    final Iterable<DataRow> rowsWithCheckbox =
        displayCheckboxColumn
            ? rows.where((DataRow row) => row.onSelectChanged != null)
            : <DataRow>[];
    final Iterable<DataRow> rowsChecked = rowsWithCheckbox.where((DataRow row) => row.selected);
    final bool allChecked = displayCheckboxColumn && rowsChecked.length == rowsWithCheckbox.length;
    final bool anyChecked = displayCheckboxColumn && rowsChecked.isNotEmpty;
    final bool someChecked = anyChecked && !allChecked;
    final double effectiveHorizontalMargin =
        horizontalMargin ??
        dataTableTheme.horizontalMargin ??
        theme.dataTableTheme.horizontalMargin ??
        _horizontalMargin;
    final double effectiveCheckboxHorizontalMarginStart =
        checkboxHorizontalMargin ??
        dataTableTheme.checkboxHorizontalMargin ??
        theme.dataTableTheme.checkboxHorizontalMargin ??
        effectiveHorizontalMargin;
    final double effectiveCheckboxHorizontalMarginEnd =
        checkboxHorizontalMargin ??
        dataTableTheme.checkboxHorizontalMargin ??
        theme.dataTableTheme.checkboxHorizontalMargin ??
        effectiveHorizontalMargin / 2.0;
    final double effectiveColumnSpacing =
        columnSpacing ??
        dataTableTheme.columnSpacing ??
        theme.dataTableTheme.columnSpacing ??
        _columnSpacing;

    final List<TableColumnWidth> tableColumns = List<TableColumnWidth>.filled(
      columns.length + (displayCheckboxColumn ? 1 : 0),
      const _NullTableColumnWidth(),
    );
    final List<TableRow> tableRows = List<TableRow>.generate(
      rows.length + 1, // the +1 is for the header row
      (int index) {
        final bool isSelected = index > 0 && rows[index - 1].selected;
        final bool isDisabled =
            index > 0 && anyRowSelectable && rows[index - 1].onSelectChanged == null;
        final Set<MaterialState> states = <MaterialState>{
          if (isSelected) MaterialState.selected,
          if (isDisabled) MaterialState.disabled,
        };
        final Color? resolvedDataRowColor =
            index > 0 ? (rows[index - 1].color ?? effectiveDataRowColor)?.resolve(states) : null;
        final Color? resolvedHeadingRowColor = effectiveHeadingRowColor?.resolve(<MaterialState>{});
        final Color? rowColor = index > 0 ? resolvedDataRowColor : resolvedHeadingRowColor;
        final BorderSide borderSide = Divider.createBorderSide(
          context,
          width:
              dividerThickness ??
              dataTableTheme.dividerThickness ??
              theme.dataTableTheme.dividerThickness ??
              _dividerThickness,
        );
        final Border? border =
            showBottomBorder
                ? Border(bottom: borderSide)
                : index == 0
                ? null
                : Border(top: borderSide);
        return TableRow(
          key: index == 0 ? _headingRowKey : rows[index - 1].key,
          decoration: BoxDecoration(
            border: border,
            color: rowColor ?? defaultRowColor.resolve(states),
          ),
          children: List<Widget>.filled(tableColumns.length, const _NullWidget()),
        );
      },
    );

    int rowIndex;

    int displayColumnIndex = 0;
    if (displayCheckboxColumn) {
      tableColumns[0] = FixedColumnWidth(
        effectiveCheckboxHorizontalMarginStart +
            Checkbox.width +
            effectiveCheckboxHorizontalMarginEnd,
      );
      tableRows[0].children[0] = _buildCheckbox(
        context: context,
        checked: someChecked ? null : allChecked,
        onRowTap: null,
        onCheckboxChanged: (bool? checked) => _handleSelectAll(checked, someChecked),
        overlayColor: null,
        tristate: true,
      );
      rowIndex = 1;
      for (final DataRow row in rows) {
        final Set<MaterialState> states = <MaterialState>{if (row.selected) MaterialState.selected};
        tableRows[rowIndex].children[0] = _buildCheckbox(
          context: context,
          checked: row.selected,
          onRowTap:
              row.onSelectChanged == null ? null : () => row.onSelectChanged?.call(!row.selected),
          onCheckboxChanged: row.onSelectChanged,
          overlayColor: row.color ?? effectiveDataRowColor,
          rowMouseCursor:
              row.mouseCursor?.resolve(states) ?? dataTableTheme.dataRowCursor?.resolve(states),
          tristate: false,
        );
        rowIndex += 1;
      }
      displayColumnIndex += 1;
    }

    for (int dataColumnIndex = 0; dataColumnIndex < columns.length; dataColumnIndex += 1) {
      final DataColumn column = columns[dataColumnIndex];

      final double paddingStart = switch (dataColumnIndex) {
        0 when displayCheckboxColumn && checkboxHorizontalMargin == null =>
          effectiveHorizontalMargin / 2.0,
        0 => effectiveHorizontalMargin,
        _ => effectiveColumnSpacing / 2.0,
      };

      final double paddingEnd;
      if (dataColumnIndex == columns.length - 1) {
        paddingEnd = effectiveHorizontalMargin;
      } else {
        paddingEnd = effectiveColumnSpacing / 2.0;
      }

      final EdgeInsetsDirectional padding = EdgeInsetsDirectional.only(
        start: paddingStart,
        end: paddingEnd,
      );
      if (column.columnWidth != null) {
        tableColumns[displayColumnIndex] = column.columnWidth!;
      } else if (dataColumnIndex == _onlyTextColumn) {
        tableColumns[displayColumnIndex] = const IntrinsicColumnWidth(flex: 1.0);
      } else {
        tableColumns[displayColumnIndex] = const IntrinsicColumnWidth();
      }

      final Set<MaterialState> headerStates = <MaterialState>{
        if (column.onSort == null) MaterialState.disabled,
      };
      tableRows[0].children[displayColumnIndex] = _buildHeadingCell(
        context: context,
        padding: padding,
        label: column.label,
        tooltip: column.tooltip,
        numeric: column.numeric,
        onSort:
            column.onSort != null
                ? () => column.onSort!(
                  dataColumnIndex,
                  sortColumnIndex != dataColumnIndex || !sortAscending,
                )
                : null,
        sorted: dataColumnIndex == sortColumnIndex,
        ascending: sortAscending,
        overlayColor: effectiveHeadingRowColor,
        mouseCursor:
            column.mouseCursor?.resolve(headerStates) ??
            dataTableTheme.headingCellCursor?.resolve(headerStates),
        headingRowAlignment:
            column.headingRowAlignment ??
            dataTableTheme.headingRowAlignment ??
            MainAxisAlignment.start,
      );
      rowIndex = 1;
      for (final DataRow row in rows) {
        final Set<MaterialState> states = <MaterialState>{if (row.selected) MaterialState.selected};
        final DataCell cell = row.cells[dataColumnIndex];
        tableRows[rowIndex].children[displayColumnIndex] = _buildDataCell(
          context: context,
          padding: padding,
          label: cell.child,
          numeric: column.numeric,
          placeholder: cell.placeholder,
          showEditIcon: cell.showEditIcon,
          onTap: cell.onTap,
          onDoubleTap: cell.onDoubleTap,
          onLongPress: cell.onLongPress,
          onTapCancel: cell.onTapCancel,
          onTapDown: cell.onTapDown,
          onSelectChanged:
              row.onSelectChanged == null ? null : () => row.onSelectChanged?.call(!row.selected),
          overlayColor: row.color ?? effectiveDataRowColor,
          onRowLongPress: row.onLongPress,
          mouseCursor:
              row.mouseCursor?.resolve(states) ?? dataTableTheme.dataRowCursor?.resolve(states),
        );
        rowIndex += 1;
      }
      displayColumnIndex += 1;
    }

    return Container(
      decoration: decoration ?? dataTableTheme.decoration ?? theme.dataTableTheme.decoration,
      child: Material(
        type: MaterialType.transparency,
        borderRadius: border?.borderRadius,
        clipBehavior: clipBehavior,
        child: Table(
          columnWidths: tableColumns.asMap(),
          defaultVerticalAlignment: TableCellVerticalAlignment.middle,
          children: tableRows,
          border: border,
        ),
      ),
    );
  }
}

/// A rectangular area of a Material that responds to touch but clips
/// its ink splashes to the current table row of the nearest table.
///
/// Must have an ancestor [Material] widget in which to cause ink
/// reactions and an ancestor [Table] widget to establish a row.
///
/// The [TableRowInkWell] must be in the same coordinate space (modulo
/// translations) as the [Table]. If it's rotated or scaled or
/// otherwise transformed, it will not be able to describe the
/// rectangle of the row in its own coordinate system as a [Rect], and
/// thus the splash will not occur. (In general, this is easy to
/// achieve: just put the [TableRowInkWell] as the direct child of the
/// [Table], and put the other contents of the cell inside it.)
///
/// See also:
///
///  * [DataTable], which makes use of [TableRowInkWell] when
///    [DataRow.onSelectChanged] is defined and [DataCell.onTap]
///    is not.
class TableRowInkWell extends InkResponse {
  /// Creates an ink well for a table row.
  const TableRowInkWell({
    super.key,
    super.child,
    super.onTap,
    super.onDoubleTap,
    super.onLongPress,
    super.onHighlightChanged,
    super.onSecondaryTap,
    super.onSecondaryTapDown,
    super.overlayColor,
    super.mouseCursor,
  }) : super(containedInkWell: true, highlightShape: BoxShape.rectangle);

  @override
  RectCallback getRectCallback(RenderBox referenceBox) {
    return () {
      RenderObject cell = referenceBox;
      RenderObject? table = cell.parent;
      final Matrix4 transform = Matrix4.identity();
      while (table is RenderObject && table is! RenderTable) {
        table.applyPaintTransform(cell, transform);
        assert(table == cell.parent);
        cell = table;
        table = table.parent;
      }
      if (table is RenderTable) {
        final TableCellParentData cellParentData = cell.parentData! as TableCellParentData;
        assert(cellParentData.y != null);
        final Rect rect = table.getRowBox(cellParentData.y!);
        // The rect is in the table's coordinate space. We need to change it to the
        // TableRowInkWell's coordinate space.
        table.applyPaintTransform(cell, transform);
        final Offset? offset = MatrixUtils.getAsTranslation(transform);
        if (offset != null) {
          return rect.shift(-offset);
        }
      }
      return Rect.zero;
    };
  }

  @override
  bool debugCheckContext(BuildContext context) {
    assert(debugCheckHasTable(context));
    return super.debugCheckContext(context);
  }
}

class _SortArrow extends StatefulWidget {
  const _SortArrow({required this.visible, required this.up, required this.duration});

  final bool visible;

  final bool? up;

  final Duration duration;

  @override
  _SortArrowState createState() => _SortArrowState();
}

class _SortArrowState extends State<_SortArrow> with TickerProviderStateMixin {
  late final AnimationController _opacityController;
  late final CurvedAnimation _opacityAnimation;

  late final AnimationController _orientationController;
  late final Animation<double> _orientationAnimation;
  double _orientationOffset = 0.0;

  bool? _up;

  static final Animatable<double> _turnTween = Tween<double>(
    begin: 0.0,
    end: math.pi,
  ).chain(CurveTween(curve: Curves.easeIn));

  @override
  void initState() {
    super.initState();
    _up = widget.up;
    _opacityAnimation = CurvedAnimation(
      parent: _opacityController = AnimationController(duration: widget.duration, vsync: this),
      curve: Curves.fastOutSlowIn,
    )..addListener(_rebuild);
    _opacityController.value = widget.visible ? 1.0 : 0.0;
    _orientationController = AnimationController(duration: widget.duration, vsync: this);
    _orientationAnimation =
        _orientationController.drive(_turnTween)
          ..addListener(_rebuild)
          ..addStatusListener(_resetOrientationAnimation);
    if (widget.visible) {
      _orientationOffset = widget.up! ? 0.0 : math.pi;
    }
  }

  void _rebuild() {
    setState(() {
      // The animations changed, so we need to rebuild.
    });
  }

  void _resetOrientationAnimation(AnimationStatus status) {
    if (status.isCompleted) {
      assert(_orientationAnimation.value == math.pi);
      _orientationOffset += math.pi;
      _orientationController.value = 0.0; // TODO(ianh): This triggers a pointless rebuild.
    }
  }

  @override
  void didUpdateWidget(_SortArrow oldWidget) {
    super.didUpdateWidget(oldWidget);
    bool skipArrow = false;
    final bool? newUp = widget.up ?? _up;
    if (oldWidget.visible != widget.visible) {
      if (widget.visible && _opacityController.isDismissed) {
        _orientationController.stop();
        _orientationController.value = 0.0;
        _orientationOffset = newUp! ? 0.0 : math.pi;
        skipArrow = true;
      }
      if (widget.visible) {
        _opacityController.forward();
      } else {
        _opacityController.reverse();
      }
    }
    if ((_up != newUp) && !skipArrow) {
      if (_orientationController.isDismissed) {
        _orientationController.forward();
      } else {
        _orientationController.reverse();
      }
    }
    _up = newUp;
  }

  @override
  void dispose() {
    _opacityController.dispose();
    _orientationController.dispose();
    _opacityAnimation.dispose();
    super.dispose();
  }

  static const double _arrowIconBaselineOffset = -1.5;
  static const double _arrowIconSize = 16.0;

  @override
  Widget build(BuildContext context) {
    return FadeTransition(
      opacity: _opacityAnimation,
      child: Transform(
        transform: Matrix4.rotationZ(_orientationOffset + _orientationAnimation.value)
          ..setTranslationRaw(0.0, _arrowIconBaselineOffset, 0.0),
        alignment: Alignment.center,
        child: const Icon(Icons.arrow_upward, size: _arrowIconSize),
      ),
    );
  }
}

class _NullTableColumnWidth extends TableColumnWidth {
  const _NullTableColumnWidth();

  @override
  double maxIntrinsicWidth(Iterable<RenderBox> cells, double containerWidth) =>
      throw UnimplementedError();

  @override
  double minIntrinsicWidth(Iterable<RenderBox> cells, double containerWidth) =>
      throw UnimplementedError();
}

class _NullWidget extends Widget {
  const _NullWidget();

  @override
  Element createElement() => throw UnimplementedError();
}
