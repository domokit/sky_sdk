// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'button.dart';
/// @docImport 'list_section.dart';
/// @docImport 'switch.dart';
library;

import 'dart:async';

import 'package:flutter/widgets.dart';

import 'colors.dart';
import 'icons.dart';
import 'theme.dart';

// These constants were eyeballed from iOS 14.4 Settings app for base, Notes for
// notched without leading, and Reminders app for notched with leading.
const double _kLeadingSize = 28.0;
const double _kNotchedLeadingSize = 30.0;
const double _kMinHeight = _kLeadingSize + 2 * 8.0;
const double _kMinHeightWithSubtitle = _kLeadingSize + 2 * 10.0;
const double _kNotchedMinHeight = _kNotchedLeadingSize + 2 * 12.0;
const double _kNotchedMinHeightWithoutLeading = _kNotchedLeadingSize + 2 * 10.0;
const EdgeInsetsDirectional _kPadding = EdgeInsetsDirectional.only(start: 20.0, end: 14.0);
const EdgeInsetsDirectional _kPaddingWithSubtitle = EdgeInsetsDirectional.only(
  start: 20.0,
  end: 14.0,
);
const EdgeInsets _kNotchedPadding = EdgeInsets.symmetric(horizontal: 14.0);
const EdgeInsetsDirectional _kNotchedPaddingWithoutLeading = EdgeInsetsDirectional.fromSTEB(
  28.0,
  10.0,
  14.0,
  10.0,
);
const double _kLeadingToTitle = 16.0;
const double _kNotchedLeadingToTitle = 12.0;
const double _kNotchedTitleToSubtitle = 3.0;
const double _kAdditionalInfoToTrailing = 6.0;
const double _kNotchedTitleWithSubtitleFontSize = 16.0;
const double _kSubtitleFontSize = 12.0;
const double _kNotchedSubtitleFontSize = 14.0;

enum _CupertinoListTileType { base, notched }

/// An iOS-style list tile.
///
/// The [CupertinoListTile] is a Cupertino equivalent of Material [ListTile].
/// It comes in two forms, an old-fashioned edge-to-edge variant known from iOS
/// Settings app and in a new, "Inset Grouped" form, known from either iOS Notes
/// or Reminders app. The first is constructed using default constructor, and
/// the latter using named constructor [CupertinoListTile.notched].
///
/// The [title], [subtitle], and [additionalInfo] are usually [Text] widgets.
/// They are all limited to one line so it is a responsibility of the caller to
/// take care of text wrapping.
///
/// The size of [leading] is by default constrained to match the iOS size,
/// depending of the type of list tile. This can however be overridden by
/// providing [leadingSize]. The [trailing] widget is not constrained and is
/// therefore a responsibility of the caller to ensure reasonable size of the
/// [trailing] widget.
///
/// The background color of the tile can be set with [backgroundColor] for the
/// state before tile was tapped and with [backgroundColorActivated] for the
/// state after the tile was tapped. By default, both values are set to match
/// the default iOS appearance.
///
/// The [padding] and [leadingToTitle] are by default set to match iOS but can
/// be overwritten if necessary.
///
/// The [onTap] callback provides an option to react to taps anywhere inside the
/// list tile. This can be used to navigate routes and according to iOS
/// behavior it should not be used for example to toggle the [CupertinoSwitch]
/// in the trailing widget.
///
/// {@tool dartpad}
/// This example uses a [ListView] to demonstrate different configurations of
/// [CupertinoListTile]s.
///
/// ** See code in examples/api/lib/cupertino/list_tile/cupertino_list_tile.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [CupertinoListSection], an iOS-style list that is a typical container for
///    [CupertinoListTile].
///  * [ListTile], a Material Design list tile.
class CupertinoListTile extends StatefulWidget {
  /// Creates an edge-to-edge iOS-style list tile like the tiles in iOS Settings
  /// app.
  ///
  /// The [title] parameter is required. It is used to convey the most important
  /// information of list tile. It is typically a [Text].
  ///
  /// The [subtitle] parameter is used to display additional information. It is
  /// placed below the [title].
  ///
  /// The [additionalInfo] parameter is used to display additional information.
  /// It is placed at the end of the tile, before the [trailing] if supplied.
  ///
  /// The [leading] parameter is typically an [Icon] or an [Image] and it comes
  /// at the start of the tile. If omitted in all list tiles, a `hasLeading` of
  /// enclosing [CupertinoListSection] should be set to `false` to ensure
  /// correct margin of divider between tiles.
  ///
  /// The [trailing] parameter is typically a [CupertinoListTileChevron], an
  /// [Icon], or a [CupertinoButton]. It is placed at the very end of the tile.
  ///
  /// The [onTap] parameter is used to provide an action that is called when the
  /// tile is tapped. It is mainly used for navigating to a new route. It should
  /// not be used to toggle a trailing [CupertinoSwitch] and similar use cases
  /// because when tile is tapped, it switches the background color and remains
  /// changed. This is according to iOS behavior.
  ///
  /// The [backgroundColor] provides a custom background color for the tile in
  /// a state before tapped. By default, it matches the theme's background color
  /// which is by default a [CupertinoColors.systemBackground].
  ///
  /// The [backgroundColorActivated] provides a custom background color for the
  /// tile after it was tapped. By default, it matches the theme's background
  /// color which is by default a [CupertinoColors.systemGrey4].
  ///
  /// The [padding] parameter sets the padding of the content inside the tile.
  /// It defaults to a value that matches the iOS look, depending on a type of
  /// [CupertinoListTile]. For native look, it should not be provided.
  ///
  /// The [leadingSize] constrains the width and height of the leading widget.
  /// By default, it is set to a value that matches the iOS look, depending on a
  /// type of [CupertinoListTile]. For native look, it should not be provided.
  ///
  /// The [leadingToTitle] specifies the horizontal space between [leading] and
  /// [title] widgets. By default, it is set to a value that matched the iOS
  /// look, depending on a type of [CupertinoListTile]. For native look, it
  /// should not be provided.
  const CupertinoListTile({
    super.key,
    required this.title,
    this.subtitle,
    this.additionalInfo,
    this.leading,
    this.trailing,
    this.onTap,
    this.backgroundColor,
    this.backgroundColorActivated,
    this.padding,
    this.leadingSize = _kLeadingSize,
    this.leadingToTitle = _kLeadingToTitle,
  }) : _type = _CupertinoListTileType.base;

  /// Creates a notched iOS-style list tile like the tiles in iOS Notes app or
  /// Reminders app.
  ///
  /// The [title] parameter is required. It is used to convey the most important
  /// information of list tile. It is typically a [Text].
  ///
  /// The [subtitle] parameter is used to display additional information. It is
  /// placed below the [title].
  ///
  /// The [additionalInfo] parameter is used to display additional information.
  /// It is placed at the end of the tile, before the [trailing] if supplied.
  ///
  /// The [leading] parameter is typically an [Icon] or an [Image] and it comes
  /// at the start of the tile. If omitted in all list tiles, a `hasLeading` of
  /// enclosing [CupertinoListSection] should be set to `false` to ensure
  /// correct margin of divider between tiles. For Notes-like tile appearance,
  /// the [leading] can be left `null`.
  ///
  /// The [trailing] parameter is typically a [CupertinoListTileChevron], an
  /// [Icon], or a [CupertinoButton]. It is placed at the very end of the tile.
  /// For Notes-like tile appearance, the [trailing] can be left `null`.
  ///
  /// The [onTap] parameter is used to provide an action that is called when the
  /// tile is tapped. It is mainly used for navigating to a new route. It should
  /// not be used to toggle a trailing [CupertinoSwitch] and similar use cases
  /// because when tile is tapped, it switches the background color and remains
  /// changed. This is according to iOS behavior.
  ///
  /// The [backgroundColor] provides a custom background color for the tile in
  /// a state before tapped. By default, it matches the theme's background color
  /// which is by default a [CupertinoColors.systemBackground].
  ///
  /// The [backgroundColorActivated] provides a custom background color for the
  /// tile after it was tapped. By default, it matches the theme's background
  /// color which is by default a [CupertinoColors.systemGrey4].
  ///
  /// The [padding] parameter sets the padding of the content inside the tile.
  /// It defaults to a value that matches the iOS look, depending on a type of
  /// [CupertinoListTile]. For native look, it should not be provided.
  ///
  /// The [leadingSize] constrains the width and height of the leading widget.
  /// By default, it is set to a value that matches the iOS look, depending on a
  /// type of [CupertinoListTile]. For native look, it should not be provided.
  ///
  /// The [leadingToTitle] specifies the horizontal space between [leading] and
  /// [title] widgets. By default, it is set to a value that matched the iOS
  /// look, depending on a type of [CupertinoListTile]. For native look, it
  /// should not be provided.
  const CupertinoListTile.notched({
    super.key,
    required this.title,
    this.subtitle,
    this.additionalInfo,
    this.leading,
    this.trailing,
    this.onTap,
    this.backgroundColor,
    this.backgroundColorActivated,
    this.padding,
    this.leadingSize = _kNotchedLeadingSize,
    this.leadingToTitle = _kNotchedLeadingToTitle,
  }) : _type = _CupertinoListTileType.notched;

  final _CupertinoListTileType _type;

  /// A [title] is used to convey the central information. Usually a [Text].
  final Widget title;

  /// A [subtitle] is used to display additional information. It is located
  /// below [title]. Usually a [Text] widget.
  final Widget? subtitle;

  /// Similar to [subtitle], an [additionalInfo] is used to display additional
  /// information. However, instead of being displayed below [title], it is
  /// displayed on the right, before [trailing]. Usually a [Text] widget.
  final Widget? additionalInfo;

  /// A widget displayed at the start of the [CupertinoListTile]. This is
  /// typically an `Icon` or an `Image`.
  final Widget? leading;

  /// A widget displayed at the end of the [CupertinoListTile]. This is usually
  /// a right chevron icon (e.g. `CupertinoListTileChevron`), or an `Icon`.
  final Widget? trailing;

  /// The [onTap] function is called when a user taps on [CupertinoListTile]. If
  /// left `null`, the [CupertinoListTile] will not react on taps. If this is a
  /// `Future<void> Function()`, then the [CupertinoListTile] remains activated
  /// until the returned future is awaited. This is according to iOS behavior.
  /// However, if this function is a `void Function()`, then the tile is active
  /// only for the duration of invocation.
  final FutureOr<void> Function()? onTap;

  /// The [backgroundColor] of the tile in normal state. Once the tile is
  /// tapped, the background color switches to [backgroundColorActivated]. It is
  /// set to match the iOS look by default.
  final Color? backgroundColor;

  /// The [backgroundColorActivated] is the background color of the tile after
  /// the tile was tapped. It is set to match the iOS look by default.
  final Color? backgroundColorActivated;

  /// Padding of the content inside [CupertinoListTile].
  final EdgeInsetsGeometry? padding;

  /// The [leadingSize] is used to constrain the width and height of [leading]
  /// widget.
  final double leadingSize;

  /// The horizontal space between [leading] widget and [title].
  final double leadingToTitle;

  @override
  State<CupertinoListTile> createState() => _CupertinoListTileState();
}

class _CupertinoListTileState extends State<CupertinoListTile> {
  bool _tapped = false;

  @override
  Widget build(BuildContext context) {
    final TextStyle textStyle = CupertinoTheme.of(context).textTheme.textStyle;
    final TextStyle coloredStyle = textStyle.copyWith(
      color: CupertinoColors.secondaryLabel.resolveFrom(context),
    );

    final bool baseType = switch (widget._type) {
      _CupertinoListTileType.base => true,
      _CupertinoListTileType.notched => false,
    };
    final Widget title = DefaultTextStyle(
      style:
          baseType || widget.subtitle == null
              ? textStyle
              : textStyle.copyWith(
                fontWeight: FontWeight.w600,
                fontSize: widget.leading == null ? _kNotchedTitleWithSubtitleFontSize : null,
              ),
      maxLines: 1,
      overflow: TextOverflow.ellipsis,
      child: widget.title,
    );

    final EdgeInsetsGeometry padding =
        widget.padding ??
        switch (widget._type) {
          _CupertinoListTileType.base when widget.subtitle != null => _kPaddingWithSubtitle,
          _CupertinoListTileType.notched when widget.leading != null => _kNotchedPadding,
          _CupertinoListTileType.base => _kPadding,
          _CupertinoListTileType.notched => _kNotchedPaddingWithoutLeading,
        };

    // The color for default state tile is set to either what user provided or
    // null and it will resolve to the correct color provided by context. But if
    // the tile was tapped, it is set to what user provided or if null to the
    // default color that matched the iOS-style.
    Color? backgroundColor = widget.backgroundColor;
    if (_tapped) {
      backgroundColor =
          widget.backgroundColorActivated ?? CupertinoColors.systemGrey4.resolveFrom(context);
    }

    final double minHeight = switch (widget._type) {
      _CupertinoListTileType.base when widget.subtitle != null => _kMinHeightWithSubtitle,
      _CupertinoListTileType.notched when widget.leading != null => _kNotchedMinHeight,
      _CupertinoListTileType.base => _kMinHeight,
      _CupertinoListTileType.notched => _kNotchedMinHeightWithoutLeading,
    };

    final Widget child = Container(
      constraints: BoxConstraints(minWidth: double.infinity, minHeight: minHeight),
      color: backgroundColor,
      child: Padding(
        padding: padding,
        child: Row(
          children: <Widget>[
            if (widget.leading case final Widget leading) ...<Widget>[
              SizedBox.square(dimension: widget.leadingSize, child: Center(child: leading)),
              SizedBox(width: widget.leadingToTitle),
            ] else
              SizedBox(height: widget.leadingSize),
            Expanded(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                crossAxisAlignment: CrossAxisAlignment.start,
                children: <Widget>[
                  title,
                  if (widget.subtitle case final Widget subtitle) ...<Widget>[
                    const SizedBox(height: _kNotchedTitleToSubtitle),
                    DefaultTextStyle(
                      style: coloredStyle.copyWith(
                        fontSize: baseType ? _kSubtitleFontSize : _kNotchedSubtitleFontSize,
                      ),
                      maxLines: 1,
                      overflow: TextOverflow.ellipsis,
                      child: subtitle,
                    ),
                  ],
                ],
              ),
            ),
            if (widget.additionalInfo case final Widget additionalInfo) ...<Widget>[
              DefaultTextStyle(style: coloredStyle, maxLines: 1, child: additionalInfo),
              if (widget.trailing != null) const SizedBox(width: _kAdditionalInfoToTrailing),
            ],
            if (widget.trailing != null) widget.trailing!,
          ],
        ),
      ),
    );

    if (widget.onTap == null) {
      return child;
    }

    return GestureDetector(
      onTapDown:
          (_) => setState(() {
            _tapped = true;
          }),
      onTapCancel:
          () => setState(() {
            _tapped = false;
          }),
      onTap: () async {
        await widget.onTap!();
        if (mounted) {
          setState(() {
            _tapped = false;
          });
        }
      },
      behavior: HitTestBehavior.opaque,
      child: child,
    );
  }
}

/// A typical iOS trailing widget used to denote that a `CupertinoListTile` is a
/// button with an action.
///
/// The [CupertinoListTileChevron] is meant as a convenience implementation of
/// trailing right chevron.
class CupertinoListTileChevron extends StatelessWidget {
  /// Creates a typical widget used to denote that a `CupertinoListTile` is a
  /// button with action.
  const CupertinoListTileChevron({super.key});

  @override
  Widget build(BuildContext context) {
    return Icon(
      CupertinoIcons.right_chevron,
      size: CupertinoTheme.of(context).textTheme.textStyle.fontSize,
      color: CupertinoColors.systemGrey2.resolveFrom(context),
    );
  }
}
