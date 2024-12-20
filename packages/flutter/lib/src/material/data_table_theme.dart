// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'data_table.dart';
library;

import 'dart:ui' show lerpDouble;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'material_state.dart';
import 'theme.dart';

// Examples can assume:
// late BuildContext context;

/// Defines default property values for descendant [DataTable]
/// widgets.
///
/// Descendant widgets obtain the current [DataTableThemeData] object
/// using `DataTableTheme.of(context)`. Instances of
/// [DataTableThemeData] can be customized with
/// [DataTableThemeData.copyWith].
///
/// Typically a [DataTableThemeData] is specified as part of the
/// overall [Theme] with [ThemeData.dataTableTheme].
///
/// All [DataTableThemeData] properties are `null` by default. When
/// null, the [DataTable] will use the values from [ThemeData] if they exist,
/// otherwise it will provide its own defaults based on the overall [Theme]'s
/// textTheme and colorScheme. See the individual [DataTable] properties for
/// details.
///
/// See also:
///
///  * [ThemeData], which describes the overall theme information for the
///    application.
@immutable
class DataTableThemeData with Diagnosticable {
  /// Creates a theme that can be used for [ThemeData.dataTableTheme].
  const DataTableThemeData({
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
    this.dividerThickness,
    this.checkboxHorizontalMargin,
    this.headingCellCursor,
    this.dataRowCursor,
    this.headingRowAlignment,
  }) : assert(
         dataRowMinHeight == null ||
             dataRowMaxHeight == null ||
             dataRowMaxHeight >= dataRowMinHeight,
       ),
       assert(
         dataRowHeight == null || (dataRowMinHeight == null && dataRowMaxHeight == null),
         'dataRowHeight ($dataRowHeight) must not be set if dataRowMinHeight ($dataRowMinHeight) or dataRowMaxHeight ($dataRowMaxHeight) are set.',
       ),
       dataRowMinHeight = dataRowHeight ?? dataRowMinHeight,
       dataRowMaxHeight = dataRowHeight ?? dataRowMaxHeight;

  /// {@macro flutter.material.dataTable.decoration}
  final Decoration? decoration;

  /// {@macro flutter.material.dataTable.dataRowColor}
  /// {@macro flutter.material.DataTable.dataRowColor}
  final MaterialStateProperty<Color?>? dataRowColor;

  /// {@macro flutter.material.dataTable.dataRowHeight}
  @Deprecated(
    'Migrate to use dataRowMinHeight and dataRowMaxHeight instead. '
    'This feature was deprecated after v3.7.0-5.0.pre.',
  )
  double? get dataRowHeight => dataRowMinHeight == dataRowMaxHeight ? dataRowMinHeight : null;

  /// {@macro flutter.material.dataTable.dataRowMinHeight}
  final double? dataRowMinHeight;

  /// {@macro flutter.material.dataTable.dataRowMaxHeight}
  final double? dataRowMaxHeight;

  /// {@macro flutter.material.dataTable.dataTextStyle}
  final TextStyle? dataTextStyle;

  /// {@macro flutter.material.dataTable.headingRowColor}
  /// {@macro flutter.material.DataTable.headingRowColor}
  final MaterialStateProperty<Color?>? headingRowColor;

  /// {@macro flutter.material.dataTable.headingRowHeight}
  final double? headingRowHeight;

  /// {@macro flutter.material.dataTable.headingTextStyle}
  final TextStyle? headingTextStyle;

  /// {@macro flutter.material.dataTable.horizontalMargin}
  final double? horizontalMargin;

  /// {@macro flutter.material.dataTable.columnSpacing}
  final double? columnSpacing;

  /// {@macro flutter.material.dataTable.dividerThickness}
  final double? dividerThickness;

  /// {@macro flutter.material.dataTable.checkboxHorizontalMargin}
  final double? checkboxHorizontalMargin;

  /// If specified, overrides the default value of [DataColumn.mouseCursor].
  final MaterialStateProperty<MouseCursor?>? headingCellCursor;

  /// If specified, overrides the default value of [DataRow.mouseCursor].
  final MaterialStateProperty<MouseCursor?>? dataRowCursor;

  /// If specified, overrides the default value of [DataColumn.headingRowAlignment].
  final MainAxisAlignment? headingRowAlignment;

  /// Creates a copy of this object but with the given fields replaced with the
  /// new values.
  DataTableThemeData copyWith({
    Decoration? decoration,
    MaterialStateProperty<Color?>? dataRowColor,
    @Deprecated(
      'Migrate to use dataRowMinHeight and dataRowMaxHeight instead. '
      'This feature was deprecated after v3.7.0-5.0.pre.',
    )
    double? dataRowHeight,
    double? dataRowMinHeight,
    double? dataRowMaxHeight,
    TextStyle? dataTextStyle,
    MaterialStateProperty<Color?>? headingRowColor,
    double? headingRowHeight,
    TextStyle? headingTextStyle,
    double? horizontalMargin,
    double? columnSpacing,
    double? dividerThickness,
    double? checkboxHorizontalMargin,
    MaterialStateProperty<MouseCursor?>? headingCellCursor,
    MaterialStateProperty<MouseCursor?>? dataRowCursor,
    MainAxisAlignment? headingRowAlignment,
  }) {
    assert(
      dataRowHeight == null || (dataRowMinHeight == null && dataRowMaxHeight == null),
      'dataRowHeight ($dataRowHeight) must not be set if dataRowMinHeight ($dataRowMinHeight) or dataRowMaxHeight ($dataRowMaxHeight) are set.',
    );
    dataRowMinHeight = dataRowHeight ?? dataRowMinHeight;
    dataRowMaxHeight = dataRowHeight ?? dataRowMaxHeight;

    return DataTableThemeData(
      decoration: decoration ?? this.decoration,
      dataRowColor: dataRowColor ?? this.dataRowColor,
      dataRowMinHeight: dataRowMinHeight ?? this.dataRowMinHeight,
      dataRowMaxHeight: dataRowMaxHeight ?? this.dataRowMaxHeight,
      dataTextStyle: dataTextStyle ?? this.dataTextStyle,
      headingRowColor: headingRowColor ?? this.headingRowColor,
      headingRowHeight: headingRowHeight ?? this.headingRowHeight,
      headingTextStyle: headingTextStyle ?? this.headingTextStyle,
      horizontalMargin: horizontalMargin ?? this.horizontalMargin,
      columnSpacing: columnSpacing ?? this.columnSpacing,
      dividerThickness: dividerThickness ?? this.dividerThickness,
      checkboxHorizontalMargin: checkboxHorizontalMargin ?? this.checkboxHorizontalMargin,
      headingCellCursor: headingCellCursor ?? this.headingCellCursor,
      dataRowCursor: dataRowCursor ?? this.dataRowCursor,
      headingRowAlignment: headingRowAlignment ?? this.headingRowAlignment,
    );
  }

  /// Linearly interpolate between two [DataTableThemeData]s.
  ///
  /// {@macro dart.ui.shadow.lerp}
  static DataTableThemeData lerp(DataTableThemeData a, DataTableThemeData b, double t) {
    if (identical(a, b)) {
      return a;
    }
    return DataTableThemeData(
      decoration: Decoration.lerp(a.decoration, b.decoration, t),
      dataRowColor: MaterialStateProperty.lerp<Color?>(
        a.dataRowColor,
        b.dataRowColor,
        t,
        Color.lerp,
      ),
      dataRowMinHeight: lerpDouble(a.dataRowMinHeight, b.dataRowMinHeight, t),
      dataRowMaxHeight: lerpDouble(a.dataRowMaxHeight, b.dataRowMaxHeight, t),
      dataTextStyle: TextStyle.lerp(a.dataTextStyle, b.dataTextStyle, t),
      headingRowColor: MaterialStateProperty.lerp<Color?>(
        a.headingRowColor,
        b.headingRowColor,
        t,
        Color.lerp,
      ),
      headingRowHeight: lerpDouble(a.headingRowHeight, b.headingRowHeight, t),
      headingTextStyle: TextStyle.lerp(a.headingTextStyle, b.headingTextStyle, t),
      horizontalMargin: lerpDouble(a.horizontalMargin, b.horizontalMargin, t),
      columnSpacing: lerpDouble(a.columnSpacing, b.columnSpacing, t),
      dividerThickness: lerpDouble(a.dividerThickness, b.dividerThickness, t),
      checkboxHorizontalMargin: lerpDouble(
        a.checkboxHorizontalMargin,
        b.checkboxHorizontalMargin,
        t,
      ),
      headingCellCursor: t < 0.5 ? a.headingCellCursor : b.headingCellCursor,
      dataRowCursor: t < 0.5 ? a.dataRowCursor : b.dataRowCursor,
      headingRowAlignment: t < 0.5 ? a.headingRowAlignment : b.headingRowAlignment,
    );
  }

  @override
  int get hashCode => Object.hash(
    decoration,
    dataRowColor,
    dataRowMinHeight,
    dataRowMaxHeight,
    dataTextStyle,
    headingRowColor,
    headingRowHeight,
    headingTextStyle,
    horizontalMargin,
    columnSpacing,
    dividerThickness,
    checkboxHorizontalMargin,
    headingCellCursor,
    dataRowCursor,
    headingRowAlignment,
  );

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is DataTableThemeData &&
        other.decoration == decoration &&
        other.dataRowColor == dataRowColor &&
        other.dataRowMinHeight == dataRowMinHeight &&
        other.dataRowMaxHeight == dataRowMaxHeight &&
        other.dataTextStyle == dataTextStyle &&
        other.headingRowColor == headingRowColor &&
        other.headingRowHeight == headingRowHeight &&
        other.headingTextStyle == headingTextStyle &&
        other.horizontalMargin == horizontalMargin &&
        other.columnSpacing == columnSpacing &&
        other.dividerThickness == dividerThickness &&
        other.checkboxHorizontalMargin == checkboxHorizontalMargin &&
        other.headingCellCursor == headingCellCursor &&
        other.dataRowCursor == dataRowCursor &&
        other.headingRowAlignment == headingRowAlignment;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<Decoration>('decoration', decoration, defaultValue: null));
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<Color?>>(
        'dataRowColor',
        dataRowColor,
        defaultValue: null,
      ),
    );
    properties.add(DoubleProperty('dataRowMinHeight', dataRowMinHeight, defaultValue: null));
    properties.add(DoubleProperty('dataRowMaxHeight', dataRowMaxHeight, defaultValue: null));
    properties.add(
      DiagnosticsProperty<TextStyle>('dataTextStyle', dataTextStyle, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<Color?>>(
        'headingRowColor',
        headingRowColor,
        defaultValue: null,
      ),
    );
    properties.add(DoubleProperty('headingRowHeight', headingRowHeight, defaultValue: null));
    properties.add(
      DiagnosticsProperty<TextStyle>('headingTextStyle', headingTextStyle, defaultValue: null),
    );
    properties.add(DoubleProperty('horizontalMargin', horizontalMargin, defaultValue: null));
    properties.add(DoubleProperty('columnSpacing', columnSpacing, defaultValue: null));
    properties.add(DoubleProperty('dividerThickness', dividerThickness, defaultValue: null));
    properties.add(
      DoubleProperty('checkboxHorizontalMargin', checkboxHorizontalMargin, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<MouseCursor?>?>(
        'headingCellCursor',
        headingCellCursor,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<MouseCursor?>?>(
        'dataRowCursor',
        dataRowCursor,
        defaultValue: null,
      ),
    );
    properties.add(
      EnumProperty<MainAxisAlignment>(
        'headingRowAlignment',
        headingRowAlignment,
        defaultValue: null,
      ),
    );
  }
}

/// Applies a data table theme to descendant [DataTable] widgets.
///
/// Descendant widgets obtain the current theme's [DataTableTheme] object using
/// [DataTableTheme.of]. When a widget uses [DataTableTheme.of], it is
/// automatically rebuilt if the theme later changes.
///
/// A data table theme can be specified as part of the overall Material
/// theme using [ThemeData.dataTableTheme].
///
/// See also:
///
///  * [DataTableThemeData], which describes the actual configuration
///    of a data table theme.
class DataTableTheme extends InheritedWidget {
  /// Constructs a data table theme that configures all descendant
  /// [DataTable] widgets.
  const DataTableTheme({super.key, required this.data, required super.child});

  /// The properties used for all descendant [DataTable] widgets.
  final DataTableThemeData data;

  /// Returns the configuration [data] from the closest
  /// [DataTableTheme] ancestor. If there is no ancestor, it returns
  /// [ThemeData.dataTableTheme]. Applications can assume that the
  /// returned value will not be null.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// DataTableThemeData theme = DataTableTheme.of(context);
  /// ```
  static DataTableThemeData of(BuildContext context) {
    final DataTableTheme? dataTableTheme =
        context.dependOnInheritedWidgetOfExactType<DataTableTheme>();
    return dataTableTheme?.data ?? Theme.of(context).dataTableTheme;
  }

  @override
  bool updateShouldNotify(DataTableTheme oldWidget) => data != oldWidget.data;
}
