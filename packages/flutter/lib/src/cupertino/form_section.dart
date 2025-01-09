// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'form_row.dart';
/// @docImport 'text_form_field_row.dart';
library;

import 'package:flutter/widgets.dart';

import 'colors.dart';
import 'list_section.dart';

// Used for iOS "Inset Grouped" margin, determined from SwiftUI's Forms in
// iOS 14.2 SDK.
const EdgeInsetsDirectional _kFormDefaultInsetGroupedRowsMargin = EdgeInsetsDirectional.fromSTEB(
  20.0,
  0.0,
  20.0,
  10.0,
);

/// An iOS-style form section.
///
/// The base constructor for [CupertinoFormSection] constructs an
/// edge-to-edge style section which includes an iOS-style header, rows,
/// the dividers between rows, and borders on top and bottom of the rows.
///
/// The [CupertinoFormSection.insetGrouped] constructor creates a round-edged and
/// padded section that is commonly seen in notched-displays like iPhone X and
/// beyond. Creates an iOS-style header, rows, and the dividers
/// between rows. Does not create borders on top and bottom of the rows.
///
/// The [header] parameter sets the form section header. The section header lies
/// above the [children] rows, with margins that match the iOS style.
///
/// The [footer] parameter sets the form section footer. The section footer
/// lies below the [children] rows.
///
/// The [children] parameter is required and sets the list of rows shown in
/// the section. The [children] parameter takes a list, as opposed to a more
/// efficient builder function that lazy builds, because forms are intended to
/// be short in row count. It is recommended that only [CupertinoFormRow] and
/// [CupertinoTextFormFieldRow] widgets be included in the [children] list in
/// order to retain the iOS look.
///
/// The [margin] parameter sets the spacing around the content area of the
/// section encapsulating [children].
///
/// The [decoration] parameter sets the decoration around [children].
/// If null, defaults to [CupertinoColors.secondarySystemGroupedBackground].
/// If null, defaults to 10.0 circular radius when constructing with
/// [CupertinoFormSection.insetGrouped]. Defaults to zero radius for the
/// standard [CupertinoFormSection] constructor.
///
/// The [backgroundColor] parameter sets the background color behind the section.
/// If null, defaults to [CupertinoColors.systemGroupedBackground].
///
/// {@macro flutter.material.Material.clipBehavior}
///
/// See also:
///
///  * [CupertinoFormRow], an iOS-style list tile, a typical child of
///    [CupertinoFormSection].
///  * [CupertinoListSection], an iOS-style list section.
class CupertinoFormSection extends StatelessWidget {
  /// Creates a section that mimics standard iOS forms.
  ///
  /// The base constructor for [CupertinoFormSection] constructs an
  /// edge-to-edge style section which includes an iOS-style header,
  /// rows, the dividers between rows, and borders on top and bottom of the rows.
  ///
  /// The [header] parameter sets the form section header. The section header
  /// lies above the [children] rows, with margins that match the iOS style.
  ///
  /// The [footer] parameter sets the form section footer. The section footer
  /// lies below the [children] rows.
  ///
  /// The [children] parameter is required and sets the list of rows shown in
  /// the section. The [children] parameter takes a list, as opposed to a more
  /// efficient builder function that lazy builds, because forms are intended to
  /// be short in row count. It is recommended that only [CupertinoFormRow] and
  /// [CupertinoTextFormFieldRow] widgets be included in the [children] list in
  /// order to retain the iOS look.
  ///
  /// The [margin] parameter sets the spacing around the content area of the
  /// section encapsulating [children], and defaults to zero padding.
  ///
  /// The [decoration] parameter sets the decoration around [children].
  /// If null, defaults to [CupertinoColors.secondarySystemGroupedBackground].
  /// If null, defaults to 10.0 circular radius when constructing with
  /// [CupertinoFormSection.insetGrouped]. Defaults to zero radius for the
  /// standard [CupertinoFormSection] constructor.
  ///
  /// The [backgroundColor] parameter sets the background color behind the
  /// section. If null, defaults to [CupertinoColors.systemGroupedBackground].
  ///
  /// {@macro flutter.material.Material.clipBehavior}
  const CupertinoFormSection({
    super.key,
    required this.children,
    this.header,
    this.footer,
    this.margin = EdgeInsets.zero,
    this.backgroundColor = CupertinoColors.systemGroupedBackground,
    this.decoration,
    this.clipBehavior = Clip.none,
  }) : _type = CupertinoListSectionType.base,
       assert(children.length > 0);

  /// Creates a section that mimics standard "Inset Grouped" iOS forms.
  ///
  /// The [CupertinoFormSection.insetGrouped] constructor creates a round-edged and
  /// padded section that is commonly seen in notched-displays like iPhone X and
  /// beyond. Creates an iOS-style header, rows, and the dividers
  /// between rows. Does not create borders on top and bottom of the rows.
  ///
  /// The [header] parameter sets the form section header. The section header
  /// lies above the [children] rows, with margins that match the iOS style.
  ///
  /// The [footer] parameter sets the form section footer. The section footer
  /// lies below the [children] rows.
  ///
  /// The [children] parameter is required and sets the list of rows shown in
  /// the section. The [children] parameter takes a list, as opposed to a more
  /// efficient builder function that lazy builds, because forms are intended to
  /// be short in row count. It is recommended that only [CupertinoFormRow] and
  /// [CupertinoTextFormFieldRow] widgets be included in the [children] list in
  /// order to retain the iOS look.
  ///
  /// The [margin] parameter sets the spacing around the content area of the
  /// section encapsulating [children], and defaults to the standard
  /// notched-style iOS form padding.
  ///
  /// The [decoration] parameter sets the decoration around [children].
  /// If null, defaults to [CupertinoColors.secondarySystemGroupedBackground].
  /// If null, defaults to 10.0 circular radius when constructing with
  /// [CupertinoFormSection.insetGrouped]. Defaults to zero radius for the
  /// standard [CupertinoFormSection] constructor.
  ///
  /// The [backgroundColor] parameter sets the background color behind the
  /// section. If null, defaults to [CupertinoColors.systemGroupedBackground].
  ///
  /// {@macro flutter.material.Material.clipBehavior}
  const CupertinoFormSection.insetGrouped({
    super.key,
    required this.children,
    this.header,
    this.footer,
    this.margin = _kFormDefaultInsetGroupedRowsMargin,
    this.backgroundColor = CupertinoColors.systemGroupedBackground,
    this.decoration,
    this.clipBehavior = Clip.none,
  }) : _type = CupertinoListSectionType.insetGrouped,
       assert(children.length > 0);

  final CupertinoListSectionType _type;

  /// Sets the form section header. The section header lies above the
  /// [children] rows.
  final Widget? header;

  /// Sets the form section footer. The section footer lies below the
  /// [children] rows.
  final Widget? footer;

  /// Margin around the content area of the section encapsulating [children].
  ///
  /// Defaults to zero padding if constructed with standard
  /// [CupertinoFormSection] constructor. Defaults to the standard notched-style
  /// iOS margin when constructing with [CupertinoFormSection.insetGrouped].
  final EdgeInsetsGeometry margin;

  /// The list of rows in the section.
  ///
  /// This takes a list, as opposed to a more efficient builder function that
  /// lazy builds, because forms are intended to be short in row count. It is
  /// recommended that only [CupertinoFormRow] and [CupertinoTextFormFieldRow]
  /// widgets be included in the [children] list in order to retain the iOS look.
  final List<Widget> children;

  /// Sets the decoration around [children].
  ///
  /// If null, background color defaults to
  /// [CupertinoColors.secondarySystemGroupedBackground].
  ///
  /// If null, border radius defaults to 10.0 circular radius when constructing
  /// with [CupertinoFormSection.insetGrouped]. Defaults to zero radius for the
  /// standard [CupertinoFormSection] constructor.
  final BoxDecoration? decoration;

  /// Sets the background color behind the section.
  ///
  /// Defaults to [CupertinoColors.systemGroupedBackground].
  final Color backgroundColor;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.none].
  final Clip clipBehavior;

  @override
  Widget build(BuildContext context) {
    final Widget? headerWidget =
        header == null
            ? null
            : DefaultTextStyle(
              style: TextStyle(
                fontSize: 13.0,
                color: CupertinoColors.secondaryLabel.resolveFrom(context),
              ),
              child: header!,
            );

    final Widget? footerWidget =
        footer == null
            ? null
            : DefaultTextStyle(
              style: TextStyle(
                fontSize: 13.0,
                color: CupertinoColors.secondaryLabel.resolveFrom(context),
              ),
              child: footer!,
            );

    switch (_type) {
      case CupertinoListSectionType.base:
        return CupertinoListSection(
          header: headerWidget,
          footer: footerWidget,
          margin: margin,
          backgroundColor: backgroundColor,
          decoration: decoration,
          clipBehavior: clipBehavior,
          hasLeading: false,
          children: children,
        );
      case CupertinoListSectionType.insetGrouped:
        return CupertinoListSection.insetGrouped(
          header: headerWidget,
          footer: footerWidget,
          margin: margin,
          backgroundColor: backgroundColor,
          decoration: decoration,
          clipBehavior: clipBehavior,
          hasLeading: false,
          children: children,
        );
    }
  }
}
