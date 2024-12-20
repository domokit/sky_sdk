// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'form_row.dart';
/// @docImport 'form_section.dart';
/// @docImport 'list_tile.dart';
/// @docImport 'text_form_field_row.dart';
library;

import 'package:flutter/widgets.dart';

import 'colors.dart';
import 'theme.dart';

// Margin on top of the list section. This was eyeballed from iOS 14.4 Simulator
// and should be always present on top of the edge-to-edge variant.
const double _kMarginTop = 22.0;

// Standard header margin, determined from SwiftUI's Forms in iOS 14.2 SDK.
const EdgeInsetsDirectional _kDefaultHeaderMargin = EdgeInsetsDirectional.fromSTEB(
  20.0,
  0.0,
  20.0,
  6.0,
);

// Header margin for inset grouped variant, determined from iOS 14.4 Simulator.
const EdgeInsetsDirectional _kInsetGroupedDefaultHeaderMargin = EdgeInsetsDirectional.fromSTEB(
  20.0,
  16.0,
  20.0,
  6.0,
);

// Standard footer margin, determined from SwiftUI's Forms in iOS 14.2 SDK.
const EdgeInsetsDirectional _kDefaultFooterMargin = EdgeInsetsDirectional.fromSTEB(
  20.0,
  0.0,
  20.0,
  0.0,
);

// Footer margin for inset grouped variant, determined from iOS 14.4 Simulator.
const EdgeInsetsDirectional _kInsetGroupedDefaultFooterMargin = EdgeInsetsDirectional.fromSTEB(
  20.0,
  0.0,
  20.0,
  10.0,
);

// Margin around children in edge-to-edge variant, determined from iOS 14.4
// Simulator.
const EdgeInsets _kDefaultRowsMargin = EdgeInsets.only(bottom: 8.0);

// Used for iOS "Inset Grouped" margin, determined from SwiftUI's Forms in
// iOS 14.2 SDK.
const EdgeInsetsDirectional _kDefaultInsetGroupedRowsMargin = EdgeInsetsDirectional.fromSTEB(
  20.0,
  20.0,
  20.0,
  10.0,
);

// Used for iOS "Inset Grouped" margin, determined from SwiftUI's Forms in
// iOS 14.2 SDK.
const EdgeInsetsDirectional _kDefaultInsetGroupedRowsMarginWithHeader =
    EdgeInsetsDirectional.fromSTEB(20.0, 0.0, 20.0, 10.0);

// Used for iOS "Inset Grouped" border radius, estimated from SwiftUI's Forms in
// iOS 14.2 SDK.
// TODO(edrisian): This should be a rounded rectangle once that shape is added.
const BorderRadius _kDefaultInsetGroupedBorderRadius = BorderRadius.all(Radius.circular(10.0));

// The margin of divider used in base list section. Estimated from iOS 14.4 SDK
// Settings app.
const double _kBaseDividerMargin = 20.0;

// Additional margin of divider used in base list section with list tiles with
// leading widgets. Estimated from iOS 14.4 SDK Settings app.
const double _kBaseAdditionalDividerMargin = 44.0;

// The margin of divider used in inset grouped version of list section.
// Estimated from iOS 14.4 SDK Reminders app.
const double _kInsetDividerMargin = 14.0;

// Additional margin of divider used in inset grouped version of list section.
// Estimated from iOS 14.4 SDK Reminders app.
const double _kInsetAdditionalDividerMargin = 42.0;

// Additional margin of divider used in inset grouped version of list section
// when there is no leading widgets. Estimated from iOS 14.4 SDK Notes app.
const double _kInsetAdditionalDividerMarginWithoutLeading = 14.0;

// Color of header and footer text in edge-to-edge variant.
const Color _kHeaderFooterColor = CupertinoDynamicColor(
  color: Color.fromRGBO(108, 108, 108, 1.0),
  darkColor: Color.fromRGBO(142, 142, 146, 1.0),
  highContrastColor: Color.fromRGBO(74, 74, 77, 1.0),
  darkHighContrastColor: Color.fromRGBO(176, 176, 183, 1.0),
  elevatedColor: Color.fromRGBO(108, 108, 108, 1.0),
  darkElevatedColor: Color.fromRGBO(142, 142, 146, 1.0),
  highContrastElevatedColor: Color.fromRGBO(108, 108, 108, 1.0),
  darkHighContrastElevatedColor: Color.fromRGBO(142, 142, 146, 1.0),
);

/// Denotes what type of the list section a [CupertinoListSection] is.
///
/// This is for internal use only.
enum CupertinoListSectionType {
  /// A basic form of [CupertinoListSection].
  base,

  /// An inset-grouped style of [CupertinoListSection].
  insetGrouped,
}

/// An iOS-style list section.
///
/// The [CupertinoListSection] is a container for children widgets. These are
/// most often [CupertinoListTile]s.
///
/// The base constructor for [CupertinoListSection] constructs an
/// edge-to-edge style section which includes an iOS-style header, the dividers
/// between rows, and borders on top and bottom of the rows. An example of such
/// list section are sections in iOS Settings app.
///
/// The [CupertinoListSection.insetGrouped] constructor creates a round-edged
/// and padded section that is seen in iOS Notes and Reminders apps. It creates
/// an iOS-style header, and the dividers between rows. Does not create borders
/// on top and bottom of the rows.
///
/// The section [header] lies above the [children] rows, with margins and style
/// that match the iOS style.
///
/// The section [footer] lies below the [children] rows and is used to provide
/// additional information for current list section.
///
/// The [children] is the list of widgets to be displayed in this list section.
/// Typically, the children are of type [CupertinoListTile], however these is
/// not enforced.
///
/// The [margin] is used to provide spacing around the content area of the
/// section encapsulating [children].
///
/// The [decoration] of [children] specifies how they should be decorated. If it
/// is not provided in constructor, the background color of [children] defaults
/// to [CupertinoColors.secondarySystemGroupedBackground] and border radius of
/// children group defaults to 10.0 circular radius when constructing with
/// [CupertinoListSection.insetGrouped]. Defaults to zero radius for the
/// standard [CupertinoListSection] constructor.
///
/// The [dividerMargin] and [additionalDividerMargin] specify the starting
/// margin of the divider between list tiles. The [dividerMargin] is always
/// present, but [additionalDividerMargin] is only added to the [dividerMargin]
/// if `hasLeading` is set to true in the constructor, which is the default
/// value.
///
/// The [backgroundColor] of the section defaults to
/// [CupertinoColors.systemGroupedBackground].
///
/// {@macro flutter.material.Material.clipBehavior}
///
/// {@tool dartpad}
/// Creates a base [CupertinoListSection] containing [CupertinoListTile]s with
/// `leading`, `title`, `additionalInfo` and `trailing` widgets.
///
/// ** See code in examples/api/lib/cupertino/list_section/list_section_base.0.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// Creates an "Inset Grouped" [CupertinoListSection] containing
/// notched [CupertinoListTile]s with `leading`, `title`, `additionalInfo` and
/// `trailing` widgets.
///
/// ** See code in examples/api/lib/cupertino/list_section/list_section_inset.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [CupertinoListTile], an iOS-style list tile, a typical child of
///    [CupertinoListSection].
///  * [CupertinoFormSection], an iOS-style form section.
class CupertinoListSection extends StatelessWidget {
  /// Creates a section that mimics standard iOS forms.
  ///
  /// The base constructor for [CupertinoListSection] constructs an
  /// edge-to-edge style section which includes an iOS-style header, the dividers
  /// between rows, and borders on top and bottom of the rows. An example of such
  /// list section are sections in iOS Settings app.
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
  /// [CupertinoListSection.insetGrouped]. Defaults to zero radius for the
  /// standard [CupertinoListSection] constructor.
  ///
  /// The [backgroundColor] parameter sets the background color behind the
  /// section. If null, defaults to [CupertinoColors.systemGroupedBackground].
  ///
  /// The [dividerMargin] parameter sets the starting offset of the divider
  /// between rows.
  ///
  /// The [additionalDividerMargin] parameter adds additional margin to existing
  /// [dividerMargin] when [hasLeading] is set to true. By default, it offsets
  /// for the width of leading and space between leading and title of
  /// [CupertinoListTile], but it can be overwritten for custom look.
  ///
  /// The [hasLeading] parameter specifies whether children [CupertinoListTile]
  /// widgets contain leading or not. Used for calculating correct starting
  /// margin for the divider between rows.
  ///
  /// The [topMargin] is used to specify the margin above the list section. It
  /// matches the iOS look by default.
  ///
  /// {@macro flutter.material.Material.clipBehavior}
  const CupertinoListSection({
    super.key,
    this.children,
    this.header,
    this.footer,
    this.margin = _kDefaultRowsMargin,
    this.backgroundColor = CupertinoColors.systemGroupedBackground,
    this.decoration,
    this.clipBehavior = Clip.none,
    this.dividerMargin = _kBaseDividerMargin,
    double? additionalDividerMargin,
    this.topMargin = _kMarginTop,
    bool hasLeading = true,
    this.separatorColor,
  }) : assert((children != null && children.length > 0) || header != null),
       type = CupertinoListSectionType.base,
       additionalDividerMargin =
           additionalDividerMargin ?? (hasLeading ? _kBaseAdditionalDividerMargin : 0.0);

  /// Creates a section that mimics standard "Inset Grouped" iOS list section.
  ///
  /// The [CupertinoListSection.insetGrouped] constructor creates a round-edged
  /// and padded section that is seen in iOS Notes and Reminders apps. It creates
  /// an iOS-style header, and the dividers between rows. Does not create borders
  /// on top and bottom of the rows.
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
  /// be short in row count. It is recommended that only [CupertinoListTile]
  /// widget be included in the [children] list in order to retain the iOS look.
  ///
  /// The [margin] parameter sets the spacing around the content area of the
  /// section encapsulating [children], and defaults to the standard
  /// notched-style iOS form padding.
  ///
  /// The [decoration] parameter sets the decoration around [children].
  /// If null, defaults to [CupertinoColors.secondarySystemGroupedBackground].
  /// If null, defaults to 10.0 circular radius when constructing with
  /// [CupertinoListSection.insetGrouped]. Defaults to zero radius for the
  /// standard [CupertinoListSection] constructor.
  ///
  /// The [backgroundColor] parameter sets the background color behind the
  /// section. If null, defaults to [CupertinoColors.systemGroupedBackground].
  ///
  /// The [dividerMargin] parameter sets the starting offset of the divider
  /// between rows.
  ///
  /// The [additionalDividerMargin] parameter adds additional margin to existing
  /// [dividerMargin] when [hasLeading] is set to true. By default, it offsets
  /// for the width of leading and space between leading and title of
  /// [CupertinoListTile], but it can be overwritten for custom look.
  ///
  /// The [hasLeading] parameter specifies whether children [CupertinoListTile]
  /// widgets contain leading or not. Used for calculating correct starting
  /// margin for the divider between rows.
  ///
  /// {@macro flutter.material.Material.clipBehavior}
  const CupertinoListSection.insetGrouped({
    super.key,
    this.children,
    this.header,
    this.footer,
    EdgeInsetsGeometry? margin,
    this.backgroundColor = CupertinoColors.systemGroupedBackground,
    this.decoration,
    this.clipBehavior = Clip.hardEdge,
    this.dividerMargin = _kInsetDividerMargin,
    double? additionalDividerMargin,
    this.topMargin,
    bool hasLeading = true,
    this.separatorColor,
  }) : assert((children != null && children.length > 0) || header != null),
       type = CupertinoListSectionType.insetGrouped,
       additionalDividerMargin =
           additionalDividerMargin ??
           (hasLeading
               ? _kInsetAdditionalDividerMargin
               : _kInsetAdditionalDividerMarginWithoutLeading),
       margin =
           margin ??
           (header == null
               ? _kDefaultInsetGroupedRowsMargin
               : _kDefaultInsetGroupedRowsMarginWithHeader);

  /// The type of list section, either base or inset grouped.
  ///
  /// This member is public for testing purposes only and cannot be set
  /// manually. Instead, use a corresponding constructors.
  @visibleForTesting
  final CupertinoListSectionType type;

  /// Sets the form section header. The section header lies above the [children]
  /// rows. Usually a [Text] widget.
  final Widget? header;

  /// Sets the form section footer. The section footer lies below the [children]
  /// rows. Usually a [Text] widget.
  final Widget? footer;

  /// Margin around the content area of the section encapsulating [children].
  ///
  /// Defaults to zero padding if constructed with standard
  /// [CupertinoListSection] constructor. Defaults to the standard notched-style
  /// iOS margin when constructing with [CupertinoListSection.insetGrouped].
  final EdgeInsetsGeometry margin;

  /// The list of rows in the section. Usually a list of [CupertinoListTile]s.
  ///
  /// This takes a list, as opposed to a more efficient builder function that
  /// lazy builds, because such lists are intended to be short in row count.
  /// It is recommended that only [CupertinoListTile] widget be included in the
  /// [children] list in order to retain the iOS look.
  final List<Widget>? children;

  /// Sets the decoration around [children].
  ///
  /// If null, background color defaults to
  /// [CupertinoColors.secondarySystemGroupedBackground].
  ///
  /// If null, border radius defaults to 10.0 circular radius when constructing
  /// with [CupertinoListSection.insetGrouped]. Defaults to zero radius for the
  /// standard [CupertinoListSection] constructor.
  final BoxDecoration? decoration;

  /// Sets the background color behind the section.
  ///
  /// Defaults to [CupertinoColors.systemGroupedBackground].
  final Color backgroundColor;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.hardEdge].
  final Clip clipBehavior;

  /// The starting offset of a margin between two list tiles.
  final double dividerMargin;

  /// Additional starting inset of the divider used between rows. This is used
  /// when adding a leading icon to children and a divider should start at the
  /// text inset instead of the icon.
  final double additionalDividerMargin;

  /// Margin above the list section. Only used in edge-to-edge variant and it
  /// matches iOS style by default.
  final double? topMargin;

  /// Sets the color for the dividers between rows, and borders on top and
  /// bottom of the rows.
  ///
  /// If null, defaults to [CupertinoColors.separator].
  final Color? separatorColor;

  @override
  Widget build(BuildContext context) {
    final Color dividerColor = separatorColor ?? CupertinoColors.separator.resolveFrom(context);
    final double dividerHeight = 1.0 / MediaQuery.devicePixelRatioOf(context);

    // Long divider is used for wrapping the top and bottom of rows.
    // Only used in CupertinoListSectionType.base mode.
    final Widget longDivider = Container(color: dividerColor, height: dividerHeight);

    // Short divider is used between rows.
    final Widget shortDivider = Container(
      margin: EdgeInsetsDirectional.only(start: dividerMargin + additionalDividerMargin),
      color: dividerColor,
      height: dividerHeight,
    );

    TextStyle style = CupertinoTheme.of(context).textTheme.textStyle;

    Widget? headerWidget, footerWidget;
    switch (type) {
      case CupertinoListSectionType.base:
        style = style.merge(
          TextStyle(
            fontSize: 13.0,
            color: CupertinoDynamicColor.resolve(_kHeaderFooterColor, context),
          ),
        );
        if (header != null) {
          headerWidget = DefaultTextStyle(style: style, child: header!);
        }
        if (footer != null) {
          footerWidget = DefaultTextStyle(style: style, child: footer!);
        }
      case CupertinoListSectionType.insetGrouped:
        if (header != null) {
          headerWidget = DefaultTextStyle(
            style: style.merge(const TextStyle(fontSize: 20.0, fontWeight: FontWeight.bold)),
            child: header!,
          );
        }
        if (footer != null) {
          footerWidget = DefaultTextStyle(style: style, child: footer!);
        }
    }

    Widget? decoratedChildrenGroup;
    if (children != null && children!.isNotEmpty) {
      // We construct childrenWithDividers as follows:
      // Insert a short divider between all rows.
      // If it is a `CupertinoListSectionType.base` type, add a long divider
      // to the top and bottom of the rows.
      final List<Widget> childrenWithDividers = <Widget>[];

      if (type == CupertinoListSectionType.base) {
        childrenWithDividers.add(longDivider);
      }

      children!.sublist(0, children!.length - 1).forEach((Widget widget) {
        childrenWithDividers.add(widget);
        childrenWithDividers.add(shortDivider);
      });

      childrenWithDividers.add(children!.last);
      if (type == CupertinoListSectionType.base) {
        childrenWithDividers.add(longDivider);
      }

      final BorderRadius childrenGroupBorderRadius = switch (type) {
        CupertinoListSectionType.insetGrouped => _kDefaultInsetGroupedBorderRadius,
        CupertinoListSectionType.base => BorderRadius.zero,
      };

      decoratedChildrenGroup = DecoratedBox(
        decoration:
            decoration ??
            BoxDecoration(
              color: CupertinoDynamicColor.resolve(
                decoration?.color ?? CupertinoColors.secondarySystemGroupedBackground,
                context,
              ),
              borderRadius: childrenGroupBorderRadius,
            ),
        child: Column(children: childrenWithDividers),
      );

      decoratedChildrenGroup = Padding(
        padding: margin,
        child:
            clipBehavior == Clip.none
                ? decoratedChildrenGroup
                : ClipRRect(
                  borderRadius: childrenGroupBorderRadius,
                  clipBehavior: clipBehavior,
                  child: decoratedChildrenGroup,
                ),
      );
    }

    return DecoratedBox(
      decoration: BoxDecoration(color: CupertinoDynamicColor.resolve(backgroundColor, context)),
      child: Column(
        children: <Widget>[
          if (type == CupertinoListSectionType.base) SizedBox(height: topMargin),
          if (headerWidget != null)
            Align(
              alignment: AlignmentDirectional.centerStart,
              child: Padding(
                padding:
                    type == CupertinoListSectionType.base
                        ? _kDefaultHeaderMargin
                        : _kInsetGroupedDefaultHeaderMargin,
                child: headerWidget,
              ),
            ),
          if (decoratedChildrenGroup != null) decoratedChildrenGroup,
          if (footerWidget != null)
            Align(
              alignment: AlignmentDirectional.centerStart,
              child: Padding(
                padding:
                    type == CupertinoListSectionType.base
                        ? _kDefaultFooterMargin
                        : _kInsetGroupedDefaultFooterMargin,
                child: footerWidget,
              ),
            ),
        ],
      ),
    );
  }
}
