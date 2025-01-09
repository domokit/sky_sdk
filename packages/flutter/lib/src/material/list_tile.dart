// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'card.dart';
/// @docImport 'checkbox.dart';
/// @docImport 'checkbox_list_tile.dart';
/// @docImport 'circle_avatar.dart';
/// @docImport 'drawer.dart';
/// @docImport 'expansion_tile.dart';
/// @docImport 'material.dart';
/// @docImport 'radio.dart';
/// @docImport 'radio_list_tile.dart';
/// @docImport 'scaffold.dart';
/// @docImport 'switch.dart';
/// @docImport 'switch_list_tile.dart';
library;

import 'dart:math' as math;

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

import 'color_scheme.dart';
import 'colors.dart';
import 'constants.dart';
import 'debug.dart';
import 'divider.dart';
import 'icon_button.dart';
import 'icon_button_theme.dart';
import 'ink_decoration.dart';
import 'ink_well.dart';
import 'list_tile_theme.dart';
import 'material_state.dart';
import 'text_theme.dart';
import 'theme.dart';
import 'theme_data.dart';

// Examples can assume:
// int _act = 1;

typedef _Sizes = ({double titleY, BoxConstraints textConstraints, Size tileSize});
typedef _PositionChild = void Function(RenderBox child, Offset offset);

/// Defines the title font used for [ListTile] descendants of a [ListTileTheme].
///
/// List tiles that appear in a [Drawer] use the theme's [TextTheme.bodyLarge]
/// text style, which is a little smaller than the theme's [TextTheme.titleMedium]
/// text style, which is used by default.
enum ListTileStyle {
  /// Use a title font that's appropriate for a [ListTile] in a list.
  list,

  /// Use a title font that's appropriate for a [ListTile] that appears in a [Drawer].
  drawer,
}

/// Where to place the control in widgets that use [ListTile] to position a
/// control next to a label.
///
/// See also:
///
///  * [CheckboxListTile], which combines a [ListTile] with a [Checkbox].
///  * [RadioListTile], which combines a [ListTile] with a [Radio] button.
///  * [SwitchListTile], which combines a [ListTile] with a [Switch].
///  * [ExpansionTile], which combines a [ListTile] with a button that expands
///    or collapses the tile to reveal or hide the children.
enum ListTileControlAffinity {
  /// Position the control on the leading edge, and the secondary widget, if
  /// any, on the trailing edge.
  leading,

  /// Position the control on the trailing edge, and the secondary widget, if
  /// any, on the leading edge.
  trailing,

  /// Position the control relative to the text in the fashion that is typical
  /// for the current platform, and place the secondary widget on the opposite
  /// side.
  platform,
}

/// Defines how [ListTile.leading] and [ListTile.trailing] are
/// vertically aligned relative to the [ListTile]'s titles
/// ([ListTile.title] and [ListTile.subtitle]).
///
/// See also:
///
///  * [ListTile.titleAlignment], to configure the title alignment for an
///    individual [ListTile].
///  * [ListTileThemeData.titleAlignment], to configure the title alignment
///    for all of the [ListTile]s under a [ListTileTheme].
///  * [ThemeData.listTileTheme], to configure the [ListTileTheme]
///    for an entire app.
enum ListTileTitleAlignment {
  /// The top of the [ListTile.leading] and [ListTile.trailing] widgets are
  /// placed [ListTile.minVerticalPadding] below the top of the [ListTile.title]
  /// if [ListTile.isThreeLine] is true, otherwise they're centered relative
  /// to the [ListTile.title] and [ListTile.subtitle] widgets.
  ///
  /// This is the default when [ThemeData.useMaterial3] is true.
  threeLine,

  /// The tops of the [ListTile.leading] and [ListTile.trailing] widgets are
  /// placed 16 pixels below the top of the [ListTile.title] widget,
  /// if the [ListTile]'s overall height is greater than 72, otherwise the
  /// [ListTile.trailing] widget is centered relative to the [ListTile.title] and
  /// [ListTile.subtitle] widgets, and the [ListTile.leading] widget is 16 pixels
  /// below the top of [ListTile.title], or center-aligned with [ListTile.title],
  /// whichever makes the [ListTile.leading] closer to the top edge of [ListTile.title].
  ///
  /// This is the default when [ThemeData.useMaterial3] is false.
  titleHeight,

  /// The tops of the [ListTile.leading] and [ListTile.trailing] widgets are
  /// placed [ListTile.minVerticalPadding] below the top of the [ListTile.title].
  top,

  /// The [ListTile.leading] and [ListTile.trailing] widgets are
  /// centered relative to the [ListTile]'s titles.
  center,

  /// The bottoms of the [ListTile.leading] and [ListTile.trailing] widgets are
  /// placed [ListTile.minVerticalPadding] above the bottom of the [ListTile]'s
  /// titles.
  bottom;

  // If isLeading is true the y offset is for the leading widget, otherwise it's
  // for the trailing child.
  double _yOffsetFor(
    double childHeight,
    double tileHeight,
    _RenderListTile listTile,
    bool isLeading,
  ) {
    return switch (this) {
      ListTileTitleAlignment.threeLine =>
        listTile.isThreeLine
            ? ListTileTitleAlignment.top._yOffsetFor(childHeight, tileHeight, listTile, isLeading)
            : ListTileTitleAlignment.center._yOffsetFor(
              childHeight,
              tileHeight,
              listTile,
              isLeading,
            ),
      // This attempts to implement the redlines for the vertical position of the
      // leading and trailing icons on the spec page:
      //   https://m2.material.io/components/lists#specs
      //
      // For large tiles (> 72dp), both leading and trailing controls should be
      // a fixed distance from top. As per guidelines this is set to 16dp.
      ListTileTitleAlignment.titleHeight when tileHeight > 72.0 => 16.0,
      // For smaller tiles, trailing should always be centered. Leading can be
      // centered or closer to the top. It should never be further than 16dp
      // to the top.
      ListTileTitleAlignment.titleHeight =>
        isLeading
            ? math.min((tileHeight - childHeight) / 2.0, 16.0)
            : (tileHeight - childHeight) / 2.0,
      ListTileTitleAlignment.top => listTile.minVerticalPadding,
      ListTileTitleAlignment.center => (tileHeight - childHeight) / 2.0,
      ListTileTitleAlignment.bottom => tileHeight - childHeight - listTile.minVerticalPadding,
    };
  }
}

/// A single fixed-height row that typically contains some text as well as
/// a leading or trailing icon.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=l8dj0yPBvgQ}
///
/// A list tile contains one to three lines of text optionally flanked by icons or
/// other widgets, such as check boxes. The icons (or other widgets) for the
/// tile are defined with the [leading] and [trailing] parameters. The first
/// line of text is not optional and is specified with [title]. The value of
/// [subtitle], which _is_ optional, will occupy the space allocated for an
/// additional line of text, or two lines if [isThreeLine] is true. If [dense]
/// is true then the overall height of this tile and the size of the
/// [DefaultTextStyle]s that wrap the [title] and [subtitle] widget are reduced.
///
/// It is the responsibility of the caller to ensure that [title] does not wrap,
/// and to ensure that [subtitle] doesn't wrap (if [isThreeLine] is false) or
/// wraps to two lines (if it is true).
///
/// The heights of the [leading] and [trailing] widgets are constrained
/// according to the
/// [Material spec](https://material.io/design/components/lists.html).
/// An exception is made for one-line ListTiles for accessibility. Please
/// see the example below to see how to adhere to both Material spec and
/// accessibility requirements.
///
/// The [leading] and [trailing] widgets can expand as far as they wish
/// horizontally, so ensure that they are properly constrained.
///
/// List tiles are typically used in [ListView]s, or arranged in [Column]s in
/// [Drawer]s and [Card]s.
///
/// This widget requires a [Material] widget ancestor in the tree to paint
/// itself on, which is typically provided by the app's [Scaffold].
/// The [tileColor], [selectedTileColor], [focusColor], and [hoverColor]
/// are not painted by the [ListTile] itself but by the [Material] widget
/// ancestor. In this case, one can wrap a [Material] widget around the
/// [ListTile], e.g.:
///
/// {@tool snippet}
/// ```dart
/// const ColoredBox(
///   color: Colors.green,
///   child: Material(
///     child: ListTile(
///       title: Text('ListTile with red background'),
///       tileColor: Colors.red,
///     ),
///   ),
/// )
/// ```
/// {@end-tool}
///
/// ## Performance considerations when wrapping [ListTile] with [Material]
///
/// Wrapping a large number of [ListTile]s individually with [Material]s
/// is expensive. Consider only wrapping the [ListTile]s that require it
/// or include a common [Material] ancestor where possible.
///
/// [ListTile] must be wrapped in a [Material] widget to animate [tileColor],
/// [selectedTileColor], [focusColor], and [hoverColor] as these colors
/// are not drawn by the list tile itself but by the material widget ancestor.
///
/// {@tool dartpad}
/// This example showcases how [ListTile] needs to be wrapped in a [Material]
/// widget to animate colors.
///
/// ** See code in examples/api/lib/material/list_tile/list_tile.0.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example uses a [ListView] to demonstrate different configurations of
/// [ListTile]s in [Card]s.
///
/// ![Different variations of ListTile](https://flutter.github.io/assets-for-api-docs/assets/material/list_tile.png)
///
/// ** See code in examples/api/lib/material/list_tile/list_tile.1.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This sample shows the creation of a [ListTile] using [ThemeData.useMaterial3] flag,
/// as described in: https://m3.material.io/components/lists/overview.
///
/// ** See code in examples/api/lib/material/list_tile/list_tile.2.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This sample shows [ListTile]'s [textColor] and [iconColor] can use
/// [WidgetStateColor] color to change the color of the text and icon
/// when the [ListTile] is enabled, selected, or disabled.
///
/// ** See code in examples/api/lib/material/list_tile/list_tile.3.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This sample shows [ListTile.titleAlignment] can be used to configure the
/// [leading] and [trailing] widgets alignment relative to the [title] and
/// [subtitle] widgets.
///
/// ** See code in examples/api/lib/material/list_tile/list_tile.4.dart **
/// {@end-tool}
///
/// {@tool snippet}
/// To use a [ListTile] within a [Row], it needs to be wrapped in an
/// [Expanded] widget. [ListTile] requires fixed width constraints,
/// whereas a [Row] does not constrain its children.
///
/// ```dart
/// const Row(
///   children: <Widget>[
///     Expanded(
///       child: ListTile(
///         leading: FlutterLogo(),
///         title: Text('These ListTiles are expanded '),
///       ),
///     ),
///     Expanded(
///       child: ListTile(
///         trailing: FlutterLogo(),
///         title: Text('to fill the available space.'),
///       ),
///     ),
///   ],
/// )
/// ```
/// {@end-tool}
/// {@tool snippet}
///
/// Tiles can be much more elaborate. Here is a tile which can be tapped, but
/// which is disabled when the `_act` variable is not 2. When the tile is
/// tapped, the whole row has an ink splash effect (see [InkWell]).
///
/// ```dart
/// ListTile(
///   leading: const Icon(Icons.flight_land),
///   title: const Text("Trix's airplane"),
///   subtitle: _act != 2 ? const Text('The airplane is only in Act II.') : null,
///   enabled: _act == 2,
///   onTap: () { /* react to the tile being tapped */ }
/// )
/// ```
/// {@end-tool}
///
/// To be accessible, tappable [leading] and [trailing] widgets have to
/// be at least 48x48 in size. However, to adhere to the Material spec,
/// [trailing] and [leading] widgets in one-line ListTiles should visually be
/// at most 32 ([dense]: true) or 40 ([dense]: false) in height, which may
/// conflict with the accessibility requirement.
///
/// For this reason, a one-line ListTile allows the height of [leading]
/// and [trailing] widgets to be constrained by the height of the ListTile.
/// This allows for the creation of tappable [leading] and [trailing] widgets
/// that are large enough, but it is up to the developer to ensure that
/// their widgets follow the Material spec.
///
/// {@tool snippet}
///
/// Here is an example of a one-line, non-[dense] ListTile with a
/// tappable leading widget that adheres to accessibility requirements and
/// the Material spec. To adjust the use case below for a one-line, [dense]
/// ListTile, adjust the vertical padding to 8.0.
///
/// ```dart
/// ListTile(
///   leading: GestureDetector(
///     behavior: HitTestBehavior.translucent,
///     onTap: () {},
///     child: Container(
///       width: 48,
///       height: 48,
///       padding: const EdgeInsets.symmetric(vertical: 4.0),
///       alignment: Alignment.center,
///       child: const CircleAvatar(),
///     ),
///   ),
///   title: const Text('title'),
///   dense: false,
/// )
/// ```
/// {@end-tool}
///
/// ## The ListTile layout isn't exactly what I want
///
/// If the way ListTile pads and positions its elements isn't quite what
/// you're looking for, it's easy to create custom list items with a
/// combination of other widgets, such as [Row]s and [Column]s.
///
/// {@tool dartpad}
/// Here is an example of a custom list item that resembles a YouTube-related
/// video list item created with [Expanded] and [Container] widgets.
///
/// ** See code in examples/api/lib/material/list_tile/custom_list_item.0.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// Here is an example of an article list item with multiline titles and
/// subtitles. It utilizes [Row]s and [Column]s, as well as [Expanded] and
/// [AspectRatio] widgets to organize its layout.
///
/// ** See code in examples/api/lib/material/list_tile/custom_list_item.1.dart **
/// {@end-tool}
///
/// See also:
///
///  * [ListTileTheme], which defines visual properties for [ListTile]s.
///  * [ListView], which can display an arbitrary number of [ListTile]s
///    in a scrolling list.
///  * [CircleAvatar], which shows an icon representing a person and is often
///    used as the [leading] element of a ListTile.
///  * [Card], which can be used with [Column] to show a few [ListTile]s.
///  * [Divider], which can be used to separate [ListTile]s.
///  * [ListTile.divideTiles], a utility for inserting [Divider]s in between [ListTile]s.
///  * [CheckboxListTile], [RadioListTile], and [SwitchListTile], widgets
///    that combine [ListTile] with other controls.
///  * Material 3 [ListTile] specifications are referenced from <https://m3.material.io/components/lists/specs>
///    and Material 2 [ListTile] specifications are referenced from <https://material.io/design/components/lists.html>
///  * Cookbook: [Use lists](https://docs.flutter.dev/cookbook/lists/basic-list)
///  * Cookbook: [Implement swipe to dismiss](https://docs.flutter.dev/cookbook/gestures/dismissible)
class ListTile extends StatelessWidget {
  /// Creates a list tile.
  ///
  /// If [isThreeLine] is true, then [subtitle] must not be null.
  ///
  /// Requires one of its ancestors to be a [Material] widget.
  const ListTile({
    super.key,
    this.leading,
    this.title,
    this.subtitle,
    this.trailing,
    this.isThreeLine = false,
    this.dense,
    this.visualDensity,
    this.shape,
    this.style,
    this.selectedColor,
    this.iconColor,
    this.textColor,
    this.titleTextStyle,
    this.subtitleTextStyle,
    this.leadingAndTrailingTextStyle,
    this.contentPadding,
    this.enabled = true,
    this.onTap,
    this.onLongPress,
    this.onFocusChange,
    this.mouseCursor,
    this.selected = false,
    this.focusColor,
    this.hoverColor,
    this.splashColor,
    this.focusNode,
    this.autofocus = false,
    this.tileColor,
    this.selectedTileColor,
    this.enableFeedback,
    this.horizontalTitleGap,
    this.minVerticalPadding,
    this.minLeadingWidth,
    this.minTileHeight,
    this.titleAlignment,
    this.internalAddSemanticForOnTap = true,
  }) : assert(!isThreeLine || subtitle != null);

  /// A widget to display before the title.
  ///
  /// Typically an [Icon] or a [CircleAvatar] widget.
  final Widget? leading;

  /// The primary content of the list tile.
  ///
  /// Typically a [Text] widget.
  ///
  /// This should not wrap. To enforce the single line limit, use
  /// [Text.maxLines].
  final Widget? title;

  /// Additional content displayed below the title.
  ///
  /// Typically a [Text] widget.
  ///
  /// If [isThreeLine] is false, this should not wrap.
  ///
  /// If [isThreeLine] is true, this should be configured to take a maximum of
  /// two lines. For example, you can use [Text.maxLines] to enforce the number
  /// of lines.
  ///
  /// The subtitle's default [TextStyle] depends on [TextTheme.bodyMedium] except
  /// [TextStyle.color]. The [TextStyle.color] depends on the value of [enabled]
  /// and [selected].
  ///
  /// When [enabled] is false, the text color is set to [ThemeData.disabledColor].
  ///
  /// When [selected] is false, the text color is set to [ListTileTheme.textColor]
  /// if it's not null and to [TextTheme.bodySmall]'s color if [ListTileTheme.textColor]
  /// is null.
  final Widget? subtitle;

  /// A widget to display after the title.
  ///
  /// Typically an [Icon] widget.
  ///
  /// To show right-aligned metadata (assuming left-to-right reading order;
  /// left-aligned for right-to-left reading order), consider using a [Row] with
  /// [CrossAxisAlignment.baseline] alignment whose first item is [Expanded] and
  /// whose second child is the metadata text, instead of using the [trailing]
  /// property.
  final Widget? trailing;

  /// Whether this list tile is intended to display three lines of text.
  ///
  /// If true, then [subtitle] must be non-null (since it is expected to give
  /// the second and third lines of text).
  ///
  /// If false, the list tile is treated as having one line if the subtitle is
  /// null and treated as having two lines if the subtitle is non-null.
  ///
  /// When using a [Text] widget for [title] and [subtitle], you can enforce
  /// line limits using [Text.maxLines].
  final bool isThreeLine;

  /// {@template flutter.material.ListTile.dense}
  /// Whether this list tile is part of a vertically dense list.
  ///
  /// If this property is null then its value is based on [ListTileTheme.dense].
  ///
  /// Dense list tiles default to a smaller height.
  ///
  /// It is not recommended to set [dense] to true when [ThemeData.useMaterial3] is true.
  /// {@endtemplate}
  final bool? dense;

  /// Defines how compact the list tile's layout will be.
  ///
  /// {@macro flutter.material.themedata.visualDensity}
  ///
  /// See also:
  ///
  ///  * [ThemeData.visualDensity], which specifies the [visualDensity] for all
  ///    widgets within a [Theme].
  final VisualDensity? visualDensity;

  /// {@template flutter.material.ListTile.shape}
  /// Defines the tile's [InkWell.customBorder] and [Ink.decoration] shape.
  /// {@endtemplate}
  ///
  /// If this property is null then [ListTileThemeData.shape] is used. If that
  /// is also null then a rectangular [Border] will be used.
  ///
  /// See also:
  ///
  /// * [ListTileTheme.of], which returns the nearest [ListTileTheme]'s
  ///   [ListTileThemeData].
  final ShapeBorder? shape;

  /// Defines the color used for icons and text when the list tile is selected.
  ///
  /// If this property is null then [ListTileThemeData.selectedColor]
  /// is used. If that is also null then [ColorScheme.primary] is used.
  ///
  /// See also:
  ///
  /// * [ListTileTheme.of], which returns the nearest [ListTileTheme]'s
  ///   [ListTileThemeData].
  final Color? selectedColor;

  /// Defines the default color for [leading] and [trailing] icons.
  ///
  /// If this property is null and [selected] is false then [ListTileThemeData.iconColor]
  /// is used. If that is also null and [ThemeData.useMaterial3] is true, [ColorScheme.onSurfaceVariant]
  /// is used, otherwise if [ThemeData.brightness] is [Brightness.light], [Colors.black54] is used,
  /// and if [ThemeData.brightness] is [Brightness.dark], the value is null.
  ///
  /// If this property is null and [selected] is true then [ListTileThemeData.selectedColor]
  /// is used. If that is also null then [ColorScheme.primary] is used.
  ///
  /// If this color is a [WidgetStateColor] it will be resolved against
  /// [WidgetState.selected] and [WidgetState.disabled] states.
  ///
  /// See also:
  ///
  /// * [ListTileTheme.of], which returns the nearest [ListTileTheme]'s
  ///   [ListTileThemeData].
  final Color? iconColor;

  /// Defines the text color for the [title], [subtitle], [leading], and [trailing].
  ///
  /// If this property is null and [selected] is false then [ListTileThemeData.textColor]
  /// is used. If that is also null then default text color is used for the [title], [subtitle]
  /// [leading], and [trailing]. Except for [subtitle], if [ThemeData.useMaterial3] is false,
  /// [TextTheme.bodySmall] is used.
  ///
  /// If this property is null and [selected] is true then [ListTileThemeData.selectedColor]
  /// is used. If that is also null then [ColorScheme.primary] is used.
  ///
  /// If this color is a [WidgetStateColor] it will be resolved against
  /// [WidgetState.selected] and [WidgetState.disabled] states.
  ///
  /// See also:
  ///
  /// * [ListTileTheme.of], which returns the nearest [ListTileTheme]'s
  ///   [ListTileThemeData].
  final Color? textColor;

  /// The text style for ListTile's [title].
  ///
  /// If this property is null, then [ListTileThemeData.titleTextStyle] is used.
  /// If that is also null and [ThemeData.useMaterial3] is true, [TextTheme.bodyLarge]
  /// with [ColorScheme.onSurface] will be used. Otherwise, If ListTile style is
  /// [ListTileStyle.list], [TextTheme.titleMedium] will be used and if ListTile style
  /// is [ListTileStyle.drawer], [TextTheme.bodyLarge] will be used.
  final TextStyle? titleTextStyle;

  /// The text style for ListTile's [subtitle].
  ///
  /// If this property is null, then [ListTileThemeData.subtitleTextStyle] is used.
  /// If that is also null and [ThemeData.useMaterial3] is true, [TextTheme.bodyMedium]
  /// with [ColorScheme.onSurfaceVariant] will be used, otherwise [TextTheme.bodyMedium]
  /// with [TextTheme.bodySmall] color will be used.
  final TextStyle? subtitleTextStyle;

  /// The text style for ListTile's [leading] and [trailing].
  ///
  /// If this property is null, then [ListTileThemeData.leadingAndTrailingTextStyle] is used.
  /// If that is also null and [ThemeData.useMaterial3] is true, [TextTheme.labelSmall]
  /// with [ColorScheme.onSurfaceVariant] will be used, otherwise [TextTheme.bodyMedium]
  /// will be used.
  final TextStyle? leadingAndTrailingTextStyle;

  /// Defines the font used for the [title].
  ///
  /// If this property is null then [ListTileThemeData.style] is used. If that
  /// is also null then [ListTileStyle.list] is used.
  ///
  /// See also:
  ///
  /// * [ListTileTheme.of], which returns the nearest [ListTileTheme]'s
  ///   [ListTileThemeData].
  final ListTileStyle? style;

  /// The tile's internal padding.
  ///
  /// Insets a [ListTile]'s contents: its [leading], [title], [subtitle],
  /// and [trailing] widgets.
  ///
  /// If null, `EdgeInsets.symmetric(horizontal: 16.0)` is used.
  final EdgeInsetsGeometry? contentPadding;

  /// Whether this list tile is interactive.
  ///
  /// If false, this list tile is styled with the disabled color from the
  /// current [Theme] and the [onTap] and [onLongPress] callbacks are
  /// inoperative.
  final bool enabled;

  /// Called when the user taps this list tile.
  ///
  /// Inoperative if [enabled] is false.
  final GestureTapCallback? onTap;

  /// Called when the user long-presses on this list tile.
  ///
  /// Inoperative if [enabled] is false.
  final GestureLongPressCallback? onLongPress;

  /// {@macro flutter.material.inkwell.onFocusChange}
  final ValueChanged<bool>? onFocusChange;

  /// {@template flutter.material.ListTile.mouseCursor}
  /// The cursor for a mouse pointer when it enters or is hovering over the
  /// widget.
  ///
  /// If [mouseCursor] is a [WidgetStateMouseCursor],
  /// [WidgetStateProperty.resolve] is used for the following [WidgetState]s:
  ///
  ///  * [WidgetState.selected].
  ///  * [WidgetState.disabled].
  /// {@endtemplate}
  ///
  /// If null, then the value of [ListTileThemeData.mouseCursor] is used. If
  /// that is also null, then [WidgetStateMouseCursor.clickable] is used.
  final MouseCursor? mouseCursor;

  /// If this tile is also [enabled] then icons and text are rendered with the same color.
  ///
  /// By default the selected color is the theme's primary color. The selected color
  /// can be overridden with a [ListTileTheme].
  ///
  /// {@tool dartpad}
  /// Here is an example of using a [StatefulWidget] to keep track of the
  /// selected index, and using that to set the [selected] property on the
  /// corresponding [ListTile].
  ///
  /// ** See code in examples/api/lib/material/list_tile/list_tile.selected.0.dart **
  /// {@end-tool}
  final bool selected;

  /// The color for the tile's [Material] when it has the input focus.
  final Color? focusColor;

  /// The color for the tile's [Material] when a pointer is hovering over it.
  final Color? hoverColor;

  /// The color of splash for the tile's [Material].
  final Color? splashColor;

  /// {@macro flutter.widgets.Focus.focusNode}
  final FocusNode? focusNode;

  /// {@macro flutter.widgets.Focus.autofocus}
  final bool autofocus;

  /// {@template flutter.material.ListTile.tileColor}
  /// Defines the background color of `ListTile` when [selected] is false.
  ///
  /// If this property is null and [selected] is false then [ListTileThemeData.tileColor]
  /// is used. If that is also null and [selected] is true, [selectedTileColor] is used.
  /// When that is also null, the [ListTileTheme.selectedTileColor] is used, otherwise
  /// [Colors.transparent] is used.
  ///
  /// {@endtemplate}
  final Color? tileColor;

  /// Defines the background color of `ListTile` when [selected] is true.
  ///
  /// When the value if null, the [selectedTileColor] is set to [ListTileTheme.selectedTileColor]
  /// if it's not null and to [Colors.transparent] if it's null.
  final Color? selectedTileColor;

  /// {@template flutter.material.ListTile.enableFeedback}
  /// Whether detected gestures should provide acoustic and/or haptic feedback.
  ///
  /// For example, on Android a tap will produce a clicking sound and a
  /// long-press will produce a short vibration, when feedback is enabled.
  ///
  /// When null, the default value is true.
  /// {@endtemplate}
  ///
  /// See also:
  ///
  ///  * [Feedback] for providing platform-specific feedback to certain actions.
  final bool? enableFeedback;

  /// The horizontal gap between the titles and the leading/trailing widgets.
  ///
  /// If null, then the value of [ListTileTheme.horizontalTitleGap] is used. If
  /// that is also null, then a default value of 16 is used.
  final double? horizontalTitleGap;

  /// The minimum padding on the top and bottom of the title and subtitle widgets.
  ///
  /// If null, then the value of [ListTileTheme.minVerticalPadding] is used. If
  /// that is also null, then a default value of 4 is used.
  final double? minVerticalPadding;

  /// The minimum width allocated for the [ListTile.leading] widget.
  ///
  /// If null, then the value of [ListTileTheme.minLeadingWidth] is used. If
  /// that is also null, then a default value of 40 is used.
  final double? minLeadingWidth;

  /// {@template flutter.material.ListTile.minTileHeight}
  /// The minimum height allocated for the [ListTile] widget.
  ///
  /// If this is null, default tile heights are 56.0, 72.0, and 88.0 for one,
  /// two, and three lines of text respectively. If `isDense` is true, these
  /// defaults are changed to 48.0, 64.0, and 76.0. A visual density value or
  /// a large title will also adjust the default tile heights.
  /// {@endtemplate}
  final double? minTileHeight;

  /// Defines how [ListTile.leading] and [ListTile.trailing] are
  /// vertically aligned relative to the [ListTile]'s titles
  /// ([ListTile.title] and [ListTile.subtitle]).
  ///
  /// If this property is null then [ListTileThemeData.titleAlignment]
  /// is used. If that is also null then [ListTileTitleAlignment.threeLine]
  /// is used.
  ///
  /// See also:
  ///
  /// * [ListTileTheme.of], which returns the nearest [ListTileTheme]'s
  ///   [ListTileThemeData].
  final ListTileTitleAlignment? titleAlignment;

  /// Whether to add button:true to the semantics if onTap is provided.
  /// This is a temporary flag to help changing the behavior of ListTile onTap semantics.
  ///
  // TODO(hangyujin): Remove this flag after fixing related g3 tests and flipping
  // the default value to true.
  final bool internalAddSemanticForOnTap;

  /// Add a one pixel border in between each tile. If color isn't specified the
  /// [ThemeData.dividerColor] of the context's [Theme] is used.
  ///
  /// See also:
  ///
  ///  * [Divider], which you can use to obtain this effect manually.
  static Iterable<Widget> divideTiles({
    BuildContext? context,
    required Iterable<Widget> tiles,
    Color? color,
  }) {
    assert(color != null || context != null);
    tiles = tiles.toList();

    if (tiles.isEmpty || tiles.length == 1) {
      return tiles;
    }

    Widget wrapTile(Widget tile) {
      return DecoratedBox(
        position: DecorationPosition.foreground,
        decoration: BoxDecoration(
          border: Border(bottom: Divider.createBorderSide(context, color: color)),
        ),
        child: tile,
      );
    }

    return <Widget>[...tiles.take(tiles.length - 1).map(wrapTile), tiles.last];
  }

  bool _isDenseLayout(ThemeData theme, ListTileThemeData tileTheme) {
    return dense ?? tileTheme.dense ?? theme.listTileTheme.dense ?? false;
  }

  Color _tileBackgroundColor(
    ThemeData theme,
    ListTileThemeData tileTheme,
    ListTileThemeData defaults,
  ) {
    final Color? color =
        selected
            ? selectedTileColor ??
                tileTheme.selectedTileColor ??
                theme.listTileTheme.selectedTileColor
            : tileColor ?? tileTheme.tileColor ?? theme.listTileTheme.tileColor;
    return color ?? defaults.tileColor!;
  }

  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasMaterial(context));
    final ThemeData theme = Theme.of(context);
    final ListTileThemeData tileTheme = ListTileTheme.of(context);
    final ListTileStyle listTileStyle =
        style ?? tileTheme.style ?? theme.listTileTheme.style ?? ListTileStyle.list;
    final ListTileThemeData defaults =
        theme.useMaterial3
            ? _LisTileDefaultsM3(context)
            : _LisTileDefaultsM2(context, listTileStyle);
    final Set<MaterialState> states = <MaterialState>{
      if (!enabled) MaterialState.disabled,
      if (selected) MaterialState.selected,
    };

    Color? resolveColor(
      Color? explicitColor,
      Color? selectedColor,
      Color? enabledColor, [
      Color? disabledColor,
    ]) {
      return _IndividualOverrides(
        explicitColor: explicitColor,
        selectedColor: selectedColor,
        enabledColor: enabledColor,
        disabledColor: disabledColor,
      ).resolve(states);
    }

    final Color? effectiveIconColor =
        resolveColor(iconColor, selectedColor, iconColor) ??
        resolveColor(tileTheme.iconColor, tileTheme.selectedColor, tileTheme.iconColor) ??
        resolveColor(
          theme.listTileTheme.iconColor,
          theme.listTileTheme.selectedColor,
          theme.listTileTheme.iconColor,
        ) ??
        resolveColor(
          defaults.iconColor,
          defaults.selectedColor,
          defaults.iconColor,
          theme.disabledColor,
        );
    final Color? effectiveColor =
        resolveColor(textColor, selectedColor, textColor) ??
        resolveColor(tileTheme.textColor, tileTheme.selectedColor, tileTheme.textColor) ??
        resolveColor(
          theme.listTileTheme.textColor,
          theme.listTileTheme.selectedColor,
          theme.listTileTheme.textColor,
        ) ??
        resolveColor(
          defaults.textColor,
          defaults.selectedColor,
          defaults.textColor,
          theme.disabledColor,
        );
    final IconThemeData iconThemeData = IconThemeData(color: effectiveIconColor);
    final IconButtonThemeData iconButtonThemeData = IconButtonThemeData(
      style: IconButton.styleFrom(foregroundColor: effectiveIconColor),
    );

    TextStyle? leadingAndTrailingStyle;
    if (leading != null || trailing != null) {
      leadingAndTrailingStyle =
          leadingAndTrailingTextStyle ??
          tileTheme.leadingAndTrailingTextStyle ??
          defaults.leadingAndTrailingTextStyle!;
      final Color? leadingAndTrailingTextColor = effectiveColor;
      leadingAndTrailingStyle = leadingAndTrailingStyle.copyWith(
        color: leadingAndTrailingTextColor,
      );
    }

    Widget? leadingIcon;
    if (leading != null) {
      leadingIcon = AnimatedDefaultTextStyle(
        style: leadingAndTrailingStyle!,
        duration: kThemeChangeDuration,
        child: leading!,
      );
    }

    TextStyle titleStyle = titleTextStyle ?? tileTheme.titleTextStyle ?? defaults.titleTextStyle!;
    final Color? titleColor = effectiveColor;
    titleStyle = titleStyle.copyWith(
      color: titleColor,
      fontSize: _isDenseLayout(theme, tileTheme) ? 13.0 : null,
    );
    final Widget titleText = AnimatedDefaultTextStyle(
      style: titleStyle,
      duration: kThemeChangeDuration,
      child: title ?? const SizedBox(),
    );

    Widget? subtitleText;
    TextStyle? subtitleStyle;
    if (subtitle != null) {
      subtitleStyle =
          subtitleTextStyle ?? tileTheme.subtitleTextStyle ?? defaults.subtitleTextStyle!;
      final Color? subtitleColor = effectiveColor;
      subtitleStyle = subtitleStyle.copyWith(
        color: subtitleColor,
        fontSize: _isDenseLayout(theme, tileTheme) ? 12.0 : null,
      );
      subtitleText = AnimatedDefaultTextStyle(
        style: subtitleStyle,
        duration: kThemeChangeDuration,
        child: subtitle!,
      );
    }

    Widget? trailingIcon;
    if (trailing != null) {
      trailingIcon = AnimatedDefaultTextStyle(
        style: leadingAndTrailingStyle!,
        duration: kThemeChangeDuration,
        child: trailing!,
      );
    }

    final TextDirection textDirection = Directionality.of(context);
    final EdgeInsets resolvedContentPadding =
        contentPadding?.resolve(textDirection) ??
        tileTheme.contentPadding?.resolve(textDirection) ??
        defaults.contentPadding!.resolve(textDirection);

    // Show basic cursor when ListTile isn't enabled or gesture callbacks are null.
    final Set<MaterialState> mouseStates = <MaterialState>{
      if (!enabled || (onTap == null && onLongPress == null)) MaterialState.disabled,
    };
    final MouseCursor effectiveMouseCursor =
        MaterialStateProperty.resolveAs<MouseCursor?>(mouseCursor, mouseStates) ??
        tileTheme.mouseCursor?.resolve(mouseStates) ??
        MaterialStateMouseCursor.clickable.resolve(mouseStates);

    final ListTileTitleAlignment effectiveTitleAlignment =
        titleAlignment ??
        tileTheme.titleAlignment ??
        (theme.useMaterial3
            ? ListTileTitleAlignment.threeLine
            : ListTileTitleAlignment.titleHeight);

    return InkWell(
      customBorder: shape ?? tileTheme.shape,
      onTap: enabled ? onTap : null,
      onLongPress: enabled ? onLongPress : null,
      onFocusChange: onFocusChange,
      mouseCursor: effectiveMouseCursor,
      canRequestFocus: enabled,
      focusNode: focusNode,
      focusColor: focusColor,
      hoverColor: hoverColor,
      splashColor: splashColor,
      autofocus: autofocus,
      enableFeedback: enableFeedback ?? tileTheme.enableFeedback ?? true,
      child: Semantics(
        button: internalAddSemanticForOnTap && (onTap != null || onLongPress != null),
        selected: selected,
        enabled: enabled,
        child: Ink(
          decoration: ShapeDecoration(
            shape: shape ?? tileTheme.shape ?? const Border(),
            color: _tileBackgroundColor(theme, tileTheme, defaults),
          ),
          child: SafeArea(
            top: false,
            bottom: false,
            minimum: resolvedContentPadding,
            child: IconTheme.merge(
              data: iconThemeData,
              child: IconButtonTheme(
                data: iconButtonThemeData,
                child: _ListTile(
                  leading: leadingIcon,
                  title: titleText,
                  subtitle: subtitleText,
                  trailing: trailingIcon,
                  isDense: _isDenseLayout(theme, tileTheme),
                  visualDensity: visualDensity ?? tileTheme.visualDensity ?? theme.visualDensity,
                  isThreeLine: isThreeLine,
                  textDirection: textDirection,
                  titleBaselineType:
                      titleStyle.textBaseline ?? defaults.titleTextStyle!.textBaseline!,
                  subtitleBaselineType:
                      subtitleStyle?.textBaseline ?? defaults.subtitleTextStyle!.textBaseline!,
                  horizontalTitleGap: horizontalTitleGap ?? tileTheme.horizontalTitleGap ?? 16,
                  minVerticalPadding:
                      minVerticalPadding ??
                      tileTheme.minVerticalPadding ??
                      defaults.minVerticalPadding!,
                  minLeadingWidth:
                      minLeadingWidth ?? tileTheme.minLeadingWidth ?? defaults.minLeadingWidth!,
                  minTileHeight: minTileHeight ?? tileTheme.minTileHeight,
                  titleAlignment: effectiveTitleAlignment,
                ),
              ),
            ),
          ),
        ),
      ),
    );
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      FlagProperty(
        'isThreeLine',
        value: isThreeLine,
        ifTrue: 'THREE_LINE',
        ifFalse: 'TWO_LINE',
        showName: true,
        defaultValue: false,
      ),
    );
    properties.add(
      FlagProperty('dense', value: dense, ifTrue: 'true', ifFalse: 'false', showName: true),
    );
    properties.add(
      DiagnosticsProperty<VisualDensity>('visualDensity', visualDensity, defaultValue: null),
    );
    properties.add(DiagnosticsProperty<ShapeBorder>('shape', shape, defaultValue: null));
    properties.add(DiagnosticsProperty<ListTileStyle>('style', style, defaultValue: null));
    properties.add(ColorProperty('selectedColor', selectedColor, defaultValue: null));
    properties.add(ColorProperty('iconColor', iconColor, defaultValue: null));
    properties.add(ColorProperty('textColor', textColor, defaultValue: null));
    properties.add(
      DiagnosticsProperty<TextStyle>('titleTextStyle', titleTextStyle, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<TextStyle>('subtitleTextStyle', subtitleTextStyle, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<TextStyle>(
        'leadingAndTrailingTextStyle',
        leadingAndTrailingTextStyle,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<EdgeInsetsGeometry>('contentPadding', contentPadding, defaultValue: null),
    );
    properties.add(
      FlagProperty(
        'enabled',
        value: enabled,
        ifTrue: 'true',
        ifFalse: 'false',
        showName: true,
        defaultValue: true,
      ),
    );
    properties.add(DiagnosticsProperty<Function>('onTap', onTap, defaultValue: null));
    properties.add(DiagnosticsProperty<Function>('onLongPress', onLongPress, defaultValue: null));
    properties.add(
      DiagnosticsProperty<MouseCursor>('mouseCursor', mouseCursor, defaultValue: null),
    );
    properties.add(
      FlagProperty(
        'selected',
        value: selected,
        ifTrue: 'true',
        ifFalse: 'false',
        showName: true,
        defaultValue: false,
      ),
    );
    properties.add(ColorProperty('focusColor', focusColor, defaultValue: null));
    properties.add(ColorProperty('hoverColor', hoverColor, defaultValue: null));
    properties.add(DiagnosticsProperty<FocusNode>('focusNode', focusNode, defaultValue: null));
    properties.add(
      FlagProperty(
        'autofocus',
        value: autofocus,
        ifTrue: 'true',
        ifFalse: 'false',
        showName: true,
        defaultValue: false,
      ),
    );
    properties.add(ColorProperty('tileColor', tileColor, defaultValue: null));
    properties.add(ColorProperty('selectedTileColor', selectedTileColor, defaultValue: null));
    properties.add(
      FlagProperty(
        'enableFeedback',
        value: enableFeedback,
        ifTrue: 'true',
        ifFalse: 'false',
        showName: true,
      ),
    );
    properties.add(DoubleProperty('horizontalTitleGap', horizontalTitleGap, defaultValue: null));
    properties.add(DoubleProperty('minVerticalPadding', minVerticalPadding, defaultValue: null));
    properties.add(DoubleProperty('minLeadingWidth', minLeadingWidth, defaultValue: null));
    properties.add(
      DiagnosticsProperty<ListTileTitleAlignment>(
        'titleAlignment',
        titleAlignment,
        defaultValue: null,
      ),
    );
  }
}

class _IndividualOverrides extends MaterialStateProperty<Color?> {
  _IndividualOverrides({
    this.explicitColor,
    this.enabledColor,
    this.selectedColor,
    this.disabledColor,
  });

  final Color? explicitColor;
  final Color? enabledColor;
  final Color? selectedColor;
  final Color? disabledColor;

  @override
  Color? resolve(Set<MaterialState> states) {
    if (explicitColor is MaterialStateColor) {
      return MaterialStateProperty.resolveAs<Color?>(explicitColor, states);
    }
    if (states.contains(MaterialState.disabled)) {
      return disabledColor;
    }
    if (states.contains(MaterialState.selected)) {
      return selectedColor;
    }
    return enabledColor;
  }
}

// Identifies the children of a _ListTileElement.
enum _ListTileSlot { leading, title, subtitle, trailing }

class _ListTile extends SlottedMultiChildRenderObjectWidget<_ListTileSlot, RenderBox> {
  const _ListTile({
    this.leading,
    required this.title,
    this.subtitle,
    this.trailing,
    required this.isThreeLine,
    required this.isDense,
    required this.visualDensity,
    required this.textDirection,
    required this.titleBaselineType,
    required this.horizontalTitleGap,
    required this.minVerticalPadding,
    required this.minLeadingWidth,
    this.minTileHeight,
    this.subtitleBaselineType,
    required this.titleAlignment,
  });

  final Widget? leading;
  final Widget title;
  final Widget? subtitle;
  final Widget? trailing;
  final bool isThreeLine;
  final bool isDense;
  final VisualDensity visualDensity;
  final TextDirection textDirection;
  final TextBaseline titleBaselineType;
  final TextBaseline? subtitleBaselineType;
  final double horizontalTitleGap;
  final double minVerticalPadding;
  final double minLeadingWidth;
  final double? minTileHeight;
  final ListTileTitleAlignment titleAlignment;

  @override
  Iterable<_ListTileSlot> get slots => _ListTileSlot.values;

  @override
  Widget? childForSlot(_ListTileSlot slot) {
    return switch (slot) {
      _ListTileSlot.leading => leading,
      _ListTileSlot.title => title,
      _ListTileSlot.subtitle => subtitle,
      _ListTileSlot.trailing => trailing,
    };
  }

  @override
  _RenderListTile createRenderObject(BuildContext context) {
    return _RenderListTile(
      isThreeLine: isThreeLine,
      isDense: isDense,
      visualDensity: visualDensity,
      textDirection: textDirection,
      titleBaselineType: titleBaselineType,
      subtitleBaselineType: subtitleBaselineType,
      horizontalTitleGap: horizontalTitleGap,
      minVerticalPadding: minVerticalPadding,
      minLeadingWidth: minLeadingWidth,
      minTileHeight: minTileHeight,
      titleAlignment: titleAlignment,
    );
  }

  @override
  void updateRenderObject(BuildContext context, _RenderListTile renderObject) {
    renderObject
      ..isThreeLine = isThreeLine
      ..isDense = isDense
      ..visualDensity = visualDensity
      ..textDirection = textDirection
      ..titleBaselineType = titleBaselineType
      ..subtitleBaselineType = subtitleBaselineType
      ..horizontalTitleGap = horizontalTitleGap
      ..minLeadingWidth = minLeadingWidth
      ..minTileHeight = minTileHeight
      ..minVerticalPadding = minVerticalPadding
      ..titleAlignment = titleAlignment;
  }
}

class _RenderListTile extends RenderBox
    with SlottedContainerRenderObjectMixin<_ListTileSlot, RenderBox> {
  _RenderListTile({
    required bool isDense,
    required VisualDensity visualDensity,
    required bool isThreeLine,
    required TextDirection textDirection,
    required TextBaseline titleBaselineType,
    TextBaseline? subtitleBaselineType,
    required double horizontalTitleGap,
    required double minVerticalPadding,
    required double minLeadingWidth,
    double? minTileHeight,
    required ListTileTitleAlignment titleAlignment,
  }) : _isDense = isDense,
       _visualDensity = visualDensity,
       _isThreeLine = isThreeLine,
       _textDirection = textDirection,
       _titleBaselineType = titleBaselineType,
       _subtitleBaselineType = subtitleBaselineType,
       _horizontalTitleGap = horizontalTitleGap,
       _minVerticalPadding = minVerticalPadding,
       _minLeadingWidth = minLeadingWidth,
       _minTileHeight = minTileHeight,
       _titleAlignment = titleAlignment;

  RenderBox? get leading => childForSlot(_ListTileSlot.leading);
  RenderBox get title => childForSlot(_ListTileSlot.title)!;
  RenderBox? get subtitle => childForSlot(_ListTileSlot.subtitle);
  RenderBox? get trailing => childForSlot(_ListTileSlot.trailing);

  // The returned list is ordered for hit testing.
  @override
  Iterable<RenderBox> get children {
    final RenderBox? title = childForSlot(_ListTileSlot.title);
    return <RenderBox>[
      if (leading != null) leading!,
      if (title != null) title,
      if (subtitle != null) subtitle!,
      if (trailing != null) trailing!,
    ];
  }

  bool get isDense => _isDense;
  bool _isDense;
  set isDense(bool value) {
    if (_isDense == value) {
      return;
    }
    _isDense = value;
    markNeedsLayout();
  }

  VisualDensity get visualDensity => _visualDensity;
  VisualDensity _visualDensity;
  set visualDensity(VisualDensity value) {
    if (_visualDensity == value) {
      return;
    }
    _visualDensity = value;
    markNeedsLayout();
  }

  bool get isThreeLine => _isThreeLine;
  bool _isThreeLine;
  set isThreeLine(bool value) {
    if (_isThreeLine == value) {
      return;
    }
    _isThreeLine = value;
    markNeedsLayout();
  }

  TextDirection get textDirection => _textDirection;
  TextDirection _textDirection;
  set textDirection(TextDirection value) {
    if (_textDirection == value) {
      return;
    }
    _textDirection = value;
    markNeedsLayout();
  }

  TextBaseline get titleBaselineType => _titleBaselineType;
  TextBaseline _titleBaselineType;
  set titleBaselineType(TextBaseline value) {
    if (_titleBaselineType == value) {
      return;
    }
    _titleBaselineType = value;
    markNeedsLayout();
  }

  TextBaseline? get subtitleBaselineType => _subtitleBaselineType;
  TextBaseline? _subtitleBaselineType;
  set subtitleBaselineType(TextBaseline? value) {
    if (_subtitleBaselineType == value) {
      return;
    }
    _subtitleBaselineType = value;
    markNeedsLayout();
  }

  double get horizontalTitleGap => _horizontalTitleGap;
  double _horizontalTitleGap;
  double get _effectiveHorizontalTitleGap => _horizontalTitleGap + visualDensity.horizontal * 2.0;

  set horizontalTitleGap(double value) {
    if (_horizontalTitleGap == value) {
      return;
    }
    _horizontalTitleGap = value;
    markNeedsLayout();
  }

  double get minVerticalPadding => _minVerticalPadding;
  double _minVerticalPadding;

  set minVerticalPadding(double value) {
    if (_minVerticalPadding == value) {
      return;
    }
    _minVerticalPadding = value;
    markNeedsLayout();
  }

  double get minLeadingWidth => _minLeadingWidth;
  double _minLeadingWidth;

  set minLeadingWidth(double value) {
    if (_minLeadingWidth == value) {
      return;
    }
    _minLeadingWidth = value;
    markNeedsLayout();
  }

  double? _minTileHeight;
  double? get minTileHeight => _minTileHeight;
  set minTileHeight(double? value) {
    if (_minTileHeight == value) {
      return;
    }
    _minTileHeight = value;
    markNeedsLayout();
  }

  ListTileTitleAlignment get titleAlignment => _titleAlignment;
  ListTileTitleAlignment _titleAlignment;
  set titleAlignment(ListTileTitleAlignment value) {
    if (_titleAlignment == value) {
      return;
    }
    _titleAlignment = value;
    markNeedsLayout();
  }

  @override
  bool get sizedByParent => false;

  static double _minWidth(RenderBox? box, double height) {
    return box == null ? 0.0 : box.getMinIntrinsicWidth(height);
  }

  static double _maxWidth(RenderBox? box, double height) {
    return box == null ? 0.0 : box.getMaxIntrinsicWidth(height);
  }

  @override
  double computeMinIntrinsicWidth(double height) {
    final double leadingWidth =
        leading != null
            ? math.max(leading!.getMinIntrinsicWidth(height), _minLeadingWidth) +
                _effectiveHorizontalTitleGap
            : 0.0;
    return leadingWidth +
        math.max(_minWidth(title, height), _minWidth(subtitle, height)) +
        _maxWidth(trailing, height);
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    final double leadingWidth =
        leading != null
            ? math.max(leading!.getMaxIntrinsicWidth(height), _minLeadingWidth) +
                _effectiveHorizontalTitleGap
            : 0.0;
    return leadingWidth +
        math.max(_maxWidth(title, height), _maxWidth(subtitle, height)) +
        _maxWidth(trailing, height);
  }

  // The target tile height to use if _minTileHeight is not specified.
  double get _defaultTileHeight {
    final Offset baseDensity = visualDensity.baseSizeAdjustment;
    return baseDensity.dy +
        switch ((isThreeLine, subtitle != null)) {
          (true, _) => isDense ? 76.0 : 88.0, // 3 lines,
          (false, true) => isDense ? 64.0 : 72.0, // 2 lines
          (false, false) => isDense ? 48.0 : 56.0, // 1 line,
        };
  }

  double get _targetTileHeight => _minTileHeight ?? _defaultTileHeight;

  @override
  double computeMinIntrinsicHeight(double width) {
    return math.max(
      _targetTileHeight,
      title.getMinIntrinsicHeight(width) + (subtitle?.getMinIntrinsicHeight(width) ?? 0.0),
    );
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    return getMinIntrinsicHeight(width);
  }

  @override
  double? computeDistanceToActualBaseline(TextBaseline baseline) {
    final BoxParentData parentData = title.parentData! as BoxParentData;
    final BaselineOffset offset =
        BaselineOffset(title.getDistanceToActualBaseline(baseline)) + parentData.offset.dy;
    return offset.offset;
  }

  BoxConstraints get maxIconHeightConstraint => BoxConstraints(
    // One-line trailing and leading widget heights do not follow
    // Material specifications, but this sizing is required to adhere
    // to accessibility requirements for smallest tappable widget.
    // Two- and three-line trailing widget heights are constrained
    // properly according to the Material spec.
    maxHeight: (isDense ? 48.0 : 56.0) + visualDensity.baseSizeAdjustment.dy,
  );

  static void _positionBox(RenderBox box, Offset offset) {
    final BoxParentData parentData = box.parentData! as BoxParentData;
    parentData.offset = offset;
  }

  // Implements _RenderListTile's layout algorithm. If `positionChild` is not null,
  // it will be called on each child with that child's layout offset.
  //
  // All of the dimensions below were taken from the Material Design spec:
  // https://material.io/design/components/lists.html#specs
  _Sizes _computeSizes(
    ChildBaselineGetter getBaseline,
    ChildLayouter getSize,
    BoxConstraints constraints, {
    _PositionChild? positionChild,
  }) {
    final BoxConstraints looseConstraints = constraints.loosen();
    final double tileWidth = looseConstraints.maxWidth;
    final BoxConstraints iconConstraints = looseConstraints.enforce(maxIconHeightConstraint);
    final RenderBox? leading = this.leading;
    final RenderBox? trailing = this.trailing;

    final Size? leadingSize = leading == null ? null : getSize(leading, iconConstraints);
    final Size? trailingSize = trailing == null ? null : getSize(trailing, iconConstraints);

    assert(
      tileWidth != leadingSize?.width || tileWidth == 0.0,
      'Leading widget consumes entire tile width. Please use a sized widget, '
      'or consider replacing ListTile with a custom widget '
      '(see https://api.flutter.dev/flutter/material/ListTile-class.html#material.ListTile.4)',
    );
    assert(
      tileWidth != trailingSize?.width || tileWidth == 0.0,
      'Trailing widget consumes entire tile width. Please use a sized widget, '
      'or consider replacing ListTile with a custom widget '
      '(see https://api.flutter.dev/flutter/material/ListTile-class.html#material.ListTile.4)',
    );

    final double titleStart =
        leadingSize == null
            ? 0.0
            : math.max(_minLeadingWidth, leadingSize.width) + _effectiveHorizontalTitleGap;

    final double adjustedTrailingWidth =
        trailingSize == null
            ? 0.0
            : math.max(trailingSize.width + _effectiveHorizontalTitleGap, 32.0);

    final BoxConstraints textConstraints = looseConstraints.tighten(
      width: tileWidth - titleStart - adjustedTrailingWidth,
    );

    final RenderBox? subtitle = this.subtitle;
    final double titleHeight = getSize(title, textConstraints).height;

    final bool isLTR = switch (textDirection) {
      TextDirection.ltr => true,
      TextDirection.rtl => false,
    };

    final double titleY;
    final double tileHeight;
    if (subtitle == null) {
      tileHeight = math.max(_targetTileHeight, titleHeight + 2.0 * _minVerticalPadding);
      titleY = (tileHeight - titleHeight) / 2.0;
    } else {
      final double subtitleHeight = getSize(subtitle, textConstraints).height;
      final double titleBaseline =
          getBaseline(title, textConstraints, titleBaselineType) ?? titleHeight;
      final double subtitleBaseline =
          getBaseline(subtitle, textConstraints, subtitleBaselineType!) ?? subtitleHeight;

      final double targetTitleY =
          (isThreeLine ? (isDense ? 22.0 : 28.0) : (isDense ? 28.0 : 32.0)) - titleBaseline;
      final double targetSubtitleY =
          (isThreeLine ? (isDense ? 42.0 : 48.0) : (isDense ? 48.0 : 52.0)) +
          visualDensity.vertical * 2.0 -
          subtitleBaseline;
      // Prevent the title and the subtitle from overlapping by moving them away from
      // each other by the same distance.
      final double halfOverlap = math.max(targetTitleY + titleHeight - targetSubtitleY, 0) / 2;
      final double idealTitleY = targetTitleY - halfOverlap;
      final double idealSubtitleY = targetSubtitleY + halfOverlap;
      // However if either component can't maintain the minimal padding from the top/bottom edges, the ListTile enters "compat mode".
      final bool compact =
          idealTitleY < minVerticalPadding ||
          idealSubtitleY + subtitleHeight + minVerticalPadding > _targetTileHeight;

      // Position subtitle.
      positionChild?.call(
        subtitle,
        Offset(
          isLTR ? titleStart : adjustedTrailingWidth,
          compact ? minVerticalPadding + titleHeight : idealSubtitleY,
        ),
      );
      tileHeight =
          compact ? 2 * _minVerticalPadding + titleHeight + subtitleHeight : _targetTileHeight;
      titleY = compact ? minVerticalPadding : idealTitleY;
    }

    if (positionChild != null) {
      positionChild(title, Offset(isLTR ? titleStart : adjustedTrailingWidth, titleY));

      if (leading != null && leadingSize != null) {
        positionChild(
          leading,
          Offset(
            isLTR ? 0.0 : tileWidth - leadingSize.width,
            titleAlignment._yOffsetFor(leadingSize.height, tileHeight, this, true),
          ),
        );
      }

      if (trailing != null && trailingSize != null) {
        positionChild(
          trailing,
          Offset(
            isLTR ? tileWidth - trailingSize.width : 0.0,
            titleAlignment._yOffsetFor(trailingSize.height, tileHeight, this, false),
          ),
        );
      }
    }

    return (
      titleY: titleY,
      textConstraints: textConstraints,
      tileSize: Size(tileWidth, tileHeight),
    );
  }

  @override
  double? computeDryBaseline(covariant BoxConstraints constraints, TextBaseline baseline) {
    final _Sizes sizes = _computeSizes(
      ChildLayoutHelper.getDryBaseline,
      ChildLayoutHelper.dryLayoutChild,
      constraints,
    );
    final BaselineOffset titleBaseline =
        BaselineOffset(title.getDryBaseline(sizes.textConstraints, baseline)) + sizes.titleY;
    return titleBaseline.offset;
  }

  @override
  Size computeDryLayout(BoxConstraints constraints) {
    return constraints.constrain(
      _computeSizes(
        ChildLayoutHelper.getDryBaseline,
        ChildLayoutHelper.dryLayoutChild,
        constraints,
      ).tileSize,
    );
  }

  @override
  void performLayout() {
    final Size tileSize =
        _computeSizes(
          ChildLayoutHelper.getBaseline,
          ChildLayoutHelper.layoutChild,
          constraints,
          positionChild: _positionBox,
        ).tileSize;

    size = constraints.constrain(tileSize);
    assert(size.width == constraints.constrainWidth(tileSize.width));
    assert(size.height == constraints.constrainHeight(tileSize.height));
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    void doPaint(RenderBox? child) {
      if (child != null) {
        final BoxParentData parentData = child.parentData! as BoxParentData;
        context.paintChild(child, parentData.offset + offset);
      }
    }

    doPaint(leading);
    doPaint(title);
    doPaint(subtitle);
    doPaint(trailing);
  }

  @override
  bool hitTestSelf(Offset position) => true;

  @override
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) {
    for (final RenderBox child in children) {
      final BoxParentData parentData = child.parentData! as BoxParentData;
      final bool isHit = result.addWithPaintOffset(
        offset: parentData.offset,
        position: position,
        hitTest: (BoxHitTestResult result, Offset transformed) {
          assert(transformed == position - parentData.offset);
          return child.hitTest(result, position: transformed);
        },
      );
      if (isHit) {
        return true;
      }
    }
    return false;
  }
}

class _LisTileDefaultsM2 extends ListTileThemeData {
  _LisTileDefaultsM2(this.context, ListTileStyle style)
    : super(
        contentPadding: const EdgeInsets.symmetric(horizontal: 16.0),
        minLeadingWidth: 40,
        minVerticalPadding: 4,
        shape: const Border(),
        style: style,
      );

  final BuildContext context;
  late final ThemeData _theme = Theme.of(context);
  late final TextTheme _textTheme = _theme.textTheme;

  @override
  Color? get tileColor => Colors.transparent;

  @override
  TextStyle? get titleTextStyle => switch (style!) {
    ListTileStyle.drawer => _textTheme.bodyLarge,
    ListTileStyle.list => _textTheme.titleMedium,
  };

  @override
  TextStyle? get subtitleTextStyle =>
      _textTheme.bodyMedium!.copyWith(color: _textTheme.bodySmall!.color);

  @override
  TextStyle? get leadingAndTrailingTextStyle => _textTheme.bodyMedium;

  @override
  Color? get selectedColor => _theme.colorScheme.primary;

  @override
  Color? get iconColor => switch (_theme.brightness) {
    // For the sake of backwards compatibility, the default for unselected
    // tiles is Colors.black45 rather than colorScheme.onSurface.withAlpha(0x73).
    Brightness.light => Colors.black45,
    // null -> use current icon theme color
    Brightness.dark => null,
  };
}

// BEGIN GENERATED TOKEN PROPERTIES - LisTile

// Do not edit by hand. The code between the "BEGIN GENERATED" and
// "END GENERATED" comments are generated from data in the Material
// Design token database by the script:
//   dev/tools/gen_defaults/bin/gen_defaults.dart.

// dart format off
class _LisTileDefaultsM3 extends ListTileThemeData {
  _LisTileDefaultsM3(this.context)
    : super(
        contentPadding: const EdgeInsetsDirectional.only(start: 16.0, end: 24.0),
        minLeadingWidth: 24,
        minVerticalPadding: 8,
        shape: const RoundedRectangleBorder(),
      );

  final BuildContext context;
  late final ThemeData _theme = Theme.of(context);
  late final ColorScheme _colors = _theme.colorScheme;
  late final TextTheme _textTheme = _theme.textTheme;

  @override
  Color? get tileColor =>  Colors.transparent;

  @override
  TextStyle? get titleTextStyle => _textTheme.bodyLarge!.copyWith(color: _colors.onSurface);

  @override
  TextStyle? get subtitleTextStyle => _textTheme.bodyMedium!.copyWith(color: _colors.onSurfaceVariant);

  @override
  TextStyle? get leadingAndTrailingTextStyle => _textTheme.labelSmall!.copyWith(color: _colors.onSurfaceVariant);

  @override
  Color? get selectedColor => _colors.primary;

  @override
  Color? get iconColor => _colors.onSurfaceVariant;
}
// dart format on

// END GENERATED TOKEN PROPERTIES - LisTile
