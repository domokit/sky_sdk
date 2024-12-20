// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:ui';
/// @docImport 'package:flutter/semantics.dart';
/// @docImport 'package:flutter/services.dart';
///
/// @docImport 'app.dart';
/// @docImport 'text_button.dart';
library;

import 'dart:ui' show clampDouble, lerpDouble;

import 'package:flutter/cupertino.dart';

import 'color_scheme.dart';
import 'colors.dart';
import 'debug.dart';
import 'dialog_theme.dart';
import 'ink_well.dart';
import 'material.dart';
import 'material_localizations.dart';
import 'text_theme.dart';
import 'theme.dart';
import 'theme_data.dart';

// Examples can assume:
// enum Department { treasury, state }
// late BuildContext context;

const EdgeInsets _defaultInsetPadding = EdgeInsets.symmetric(horizontal: 40.0, vertical: 24.0);

/// A Material Design dialog.
///
/// This dialog widget does not have any opinion about the contents of the
/// dialog. Rather than using this widget directly, consider using [AlertDialog]
/// or [SimpleDialog], which implement specific kinds of Material Design
/// dialogs.
///
/// {@tool dartpad}
/// This sample shows the creation of [Dialog] and [Dialog.fullscreen] widgets.
///
/// ** See code in examples/api/lib/material/dialog/dialog.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [AlertDialog], for dialogs that have a message and some buttons.
///  * [SimpleDialog], for dialogs that offer a variety of options.
///  * [showDialog], which actually displays the dialog and returns its result.
///  * <https://material.io/design/components/dialogs.html>
class Dialog extends StatelessWidget {
  /// Creates a dialog.
  ///
  /// Typically used in conjunction with [showDialog].
  const Dialog({
    super.key,
    this.backgroundColor,
    this.elevation,
    this.shadowColor,
    this.surfaceTintColor,
    this.insetAnimationDuration = const Duration(milliseconds: 100),
    this.insetAnimationCurve = Curves.decelerate,
    this.insetPadding,
    this.clipBehavior,
    this.shape,
    this.alignment,
    this.child,
  }) : assert(elevation == null || elevation >= 0.0),
       _fullscreen = false;

  /// Creates a fullscreen dialog.
  ///
  /// Typically used in conjunction with [showDialog].
  const Dialog.fullscreen({
    super.key,
    this.backgroundColor,
    this.insetAnimationDuration = Duration.zero,
    this.insetAnimationCurve = Curves.decelerate,
    this.child,
  }) : elevation = 0,
       shadowColor = null,
       surfaceTintColor = null,
       insetPadding = EdgeInsets.zero,
       clipBehavior = Clip.none,
       shape = null,
       alignment = null,
       _fullscreen = true;

  /// {@template flutter.material.dialog.backgroundColor}
  /// The background color of the surface of this [Dialog].
  ///
  /// This sets the [Material.color] on this [Dialog]'s [Material].
  ///
  /// If null, then the [DialogThemeData.backgroundColor] is used. If that is
  /// also null, defaults to [ColorScheme.surfaceContainerHigh]. If
  /// [ThemeData.useMaterial3] is false, defaults to [Colors.grey] with a shade
  ///  of 800 in dark theme and [Colors.white] in light theme.
  ///
  /// If [Dialog.fullscreen] is used, defaults to [ColorScheme.surface].
  /// {@endtemplate}
  final Color? backgroundColor;

  /// {@template flutter.material.dialog.elevation}
  /// The z-coordinate of this [Dialog].
  ///
  /// Controls how far above the parent the dialog will appear. Elevation is
  /// represented by a drop shadow if [shadowColor] is non null,
  /// and a surface tint overlay on the background color if [surfaceTintColor] is
  /// non null.
  ///
  /// If null then [DialogThemeData.elevation] is used, and if that is null then
  /// the elevation will match the Material Design specification for Dialogs.
  ///
  /// See also:
  ///   * [Material.elevation], which describes how [elevation] effects the
  ///     drop shadow or surface tint overlay.
  ///   * [shadowColor], color of the drop shadow used to indicate the elevation.
  ///   * [surfaceTintColor], color of an overlay on top of the background
  ///     color used to indicate the elevation.
  ///   * <https://m3.material.io/components/dialogs/overview>, the Material
  ///     Design specification for dialogs.
  /// {@endtemplate}
  final double? elevation;

  /// {@template flutter.material.dialog.shadowColor}
  /// The color used to paint a drop shadow under the dialog's [Material],
  /// which reflects the dialog's [elevation].
  ///
  /// If null and [ThemeData.useMaterial3] is true then no drop shadow will
  /// be rendered.
  ///
  /// If null and [ThemeData.useMaterial3] is false then it will default to
  /// [ThemeData.shadowColor].
  ///
  /// See also:
  ///   * [Material.shadowColor], which describes how the drop shadow is painted.
  ///   * [elevation], which affects how the drop shadow is painted.
  ///   * [surfaceTintColor], which can be used to indicate elevation through
  ///     tinting the background color.
  /// {@endtemplate}
  final Color? shadowColor;

  /// {@template flutter.material.dialog.surfaceTintColor}
  /// The color used as a surface tint overlay on the dialog's background color,
  /// which reflects the dialog's [elevation].
  ///
  /// If [ThemeData.useMaterial3] is false property has no effect.
  ///
  /// This is not recommended for use. [Material 3 spec](https://m3.material.io/styles/color/the-color-system/color-roles)
  /// introduced a set of tone-based surfaces and surface containers in its [ColorScheme],
  /// which provide more flexibility. The intention is to eventually remove surface tint color from
  /// the framework.
  ///
  /// defaults to [Colors.transparent].
  ///
  /// To disable this feature, set [surfaceTintColor] to [Colors.transparent].
  ///
  /// See also:
  ///   * [Material.surfaceTintColor], which describes how the surface tint will
  ///     be applied to the background color of the dialog.
  ///   * [elevation], which affects the opacity of the surface tint.
  ///   * [shadowColor], which can be used to indicate elevation through
  ///     a drop shadow.
  /// {@endtemplate}
  final Color? surfaceTintColor;

  /// {@template flutter.material.dialog.insetAnimationDuration}
  /// The duration of the animation to show when the system keyboard intrudes
  /// into the space that the dialog is placed in.
  ///
  /// Defaults to 100 milliseconds when [Dialog] is used, and [Duration.zero]
  /// when [Dialog.fullscreen] is used.
  /// {@endtemplate}
  final Duration insetAnimationDuration;

  /// {@template flutter.material.dialog.insetAnimationCurve}
  /// The curve to use for the animation shown when the system keyboard intrudes
  /// into the space that the dialog is placed in.
  ///
  /// Defaults to [Curves.decelerate].
  /// {@endtemplate}
  final Curve insetAnimationCurve;

  /// {@template flutter.material.dialog.insetPadding}
  /// The amount of padding added to [MediaQueryData.viewInsets] on the outside
  /// of the dialog. This defines the minimum space between the screen's edges
  /// and the dialog.
  ///
  /// Defaults to `EdgeInsets.symmetric(horizontal: 40.0, vertical: 24.0)`.
  /// {@endtemplate}
  final EdgeInsets? insetPadding;

  /// {@template flutter.material.dialog.clipBehavior}
  /// Controls how the contents of the dialog are clipped (or not) to the given
  /// [shape].
  ///
  /// See the enum [Clip] for details of all possible options and their common
  /// use cases.
  ///
  /// If null, then [DialogThemeData.clipBehavior] is used. If that is also null,
  /// defaults to [Clip.none].
  /// {@endtemplate}
  final Clip? clipBehavior;

  /// {@template flutter.material.dialog.shape}
  /// The shape of this dialog's border.
  ///
  /// Defines the dialog's [Material.shape].
  ///
  /// The default shape is a [RoundedRectangleBorder] with a radius of 4.0
  /// {@endtemplate}
  final ShapeBorder? shape;

  /// {@template flutter.material.dialog.alignment}
  /// How to align the [Dialog].
  ///
  /// If null, then [DialogThemeData.alignment] is used. If that is also null, the
  /// default is [Alignment.center].
  /// {@endtemplate}
  final AlignmentGeometry? alignment;

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget? child;

  /// This value is used to determine if this is a fullscreen dialog.
  final bool _fullscreen;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final DialogThemeData dialogTheme = DialogTheme.of(context);
    final EdgeInsets effectivePadding =
        MediaQuery.viewInsetsOf(context) +
        (insetPadding ?? dialogTheme.insetPadding ?? _defaultInsetPadding);
    final DialogThemeData defaults =
        theme.useMaterial3
            ? (_fullscreen ? _DialogFullscreenDefaultsM3(context) : _DialogDefaultsM3(context))
            : _DialogDefaultsM2(context);

    Widget dialogChild;

    if (_fullscreen) {
      dialogChild = Material(
        color: backgroundColor ?? dialogTheme.backgroundColor ?? defaults.backgroundColor,
        child: child,
      );
    } else {
      dialogChild = Align(
        alignment: alignment ?? dialogTheme.alignment ?? defaults.alignment!,
        child: ConstrainedBox(
          constraints: const BoxConstraints(minWidth: 280.0),
          child: Material(
            color: backgroundColor ?? dialogTheme.backgroundColor ?? defaults.backgroundColor,
            elevation: elevation ?? dialogTheme.elevation ?? defaults.elevation!,
            shadowColor: shadowColor ?? dialogTheme.shadowColor ?? defaults.shadowColor,
            surfaceTintColor:
                surfaceTintColor ?? dialogTheme.surfaceTintColor ?? defaults.surfaceTintColor,
            shape: shape ?? dialogTheme.shape ?? defaults.shape!,
            type: MaterialType.card,
            clipBehavior: clipBehavior ?? dialogTheme.clipBehavior ?? defaults.clipBehavior!,
            child: child,
          ),
        ),
      );
    }

    return AnimatedPadding(
      padding: effectivePadding,
      duration: insetAnimationDuration,
      curve: insetAnimationCurve,
      child: MediaQuery.removeViewInsets(
        removeLeft: true,
        removeTop: true,
        removeRight: true,
        removeBottom: true,
        context: context,
        child: dialogChild,
      ),
    );
  }
}

/// A Material Design alert dialog.
///
/// An alert dialog (also known as a basic dialog) informs the user about
/// situations that require acknowledgment. An alert dialog has an optional
/// title and an optional list of actions. The title is displayed above the
/// content and the actions are displayed below the content.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=75CsnyRXf5I}
///
/// For dialogs that offer the user a choice between several options, consider
/// using a [SimpleDialog].
///
/// Typically passed as the child widget to [showDialog], which displays the
/// dialog.
///
/// {@animation 350 622 https://flutter.github.io/assets-for-api-docs/assets/material/alert_dialog.mp4}
///
/// {@tool snippet}
///
/// This snippet shows a method in a [State] which, when called, displays a dialog box
/// and returns a [Future] that completes when the dialog is dismissed.
///
/// ```dart
/// Future<void> _showMyDialog() async {
///   return showDialog<void>(
///     context: context,
///     barrierDismissible: false, // user must tap button!
///     builder: (BuildContext context) {
///       return AlertDialog(
///         title: const Text('AlertDialog Title'),
///         content: const SingleChildScrollView(
///           child: ListBody(
///             children: <Widget>[
///               Text('This is a demo alert dialog.'),
///               Text('Would you like to approve of this message?'),
///             ],
///           ),
///         ),
///         actions: <Widget>[
///           TextButton(
///             child: const Text('Approve'),
///             onPressed: () {
///               Navigator.of(context).pop();
///             },
///           ),
///         ],
///       );
///     },
///   );
/// }
/// ```
/// {@end-tool}
///
/// {@tool dartpad}
/// This demo shows a [TextButton] which when pressed, calls [showDialog]. When called, this method
/// displays a Material dialog above the current contents of the app and returns
/// a [Future] that completes when the dialog is dismissed.
///
/// ** See code in examples/api/lib/material/dialog/alert_dialog.0.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This sample shows the creation of [AlertDialog], as described in:
/// https://m3.material.io/components/dialogs/overview
///
/// ** See code in examples/api/lib/material/dialog/alert_dialog.1.dart **
/// {@end-tool}
///
/// ## Alert dialogs and scrolling
///
/// By default, alert dialogs size themselves to contain their children.
///
/// If the content is too large to fit on the screen vertically, the dialog will
/// display the title and actions, and let the _[content]_ overflow. This is
/// rarely desired. Consider using a scrolling widget for [content], such as
/// [SingleChildScrollView], to avoid overflow.
///
/// Because the dialog attempts to size itself to the contents, the [content]
/// must support reporting its intrinsic dimensions. In particular, this means
/// that lazily-rendered widgets such as [ListView], [GridView], and
/// [CustomScrollView], will not work in an [AlertDialog] unless they are
/// wrapped in a widget that forces a particular size (e.g. a [SizedBox]).
///
/// For finer-grained control over the sizing of a dialog, consider using
/// [Dialog] directly.
///
/// See also:
///
///  * [SimpleDialog], which handles the scrolling of the contents but has no [actions].
///  * [Dialog], on which [AlertDialog] and [SimpleDialog] are based.
///  * [CupertinoAlertDialog], an iOS-styled alert dialog.
///  * [showDialog], which actually displays the dialog and returns its result.
///  * <https://material.io/design/components/dialogs.html#alert-dialog>
///  * <https://m3.material.io/components/dialogs>
class AlertDialog extends StatelessWidget {
  /// Creates an alert dialog.
  ///
  /// Typically used in conjunction with [showDialog].
  ///
  /// The [titlePadding] and [contentPadding] default to null, which implies a
  /// default that depends on the values of the other properties. See the
  /// documentation of [titlePadding] and [contentPadding] for details.
  const AlertDialog({
    super.key,
    this.icon,
    this.iconPadding,
    this.iconColor,
    this.title,
    this.titlePadding,
    this.titleTextStyle,
    this.content,
    this.contentPadding,
    this.contentTextStyle,
    this.actions,
    this.actionsPadding,
    this.actionsAlignment,
    this.actionsOverflowAlignment,
    this.actionsOverflowDirection,
    this.actionsOverflowButtonSpacing,
    this.buttonPadding,
    this.backgroundColor,
    this.elevation,
    this.shadowColor,
    this.surfaceTintColor,
    this.semanticLabel,
    this.insetPadding,
    this.clipBehavior,
    this.shape,
    this.alignment,
    this.scrollable = false,
  });

  /// Creates an adaptive [AlertDialog] based on whether the target platform is
  /// iOS or macOS, following Material design's
  /// [Cross-platform guidelines](https://material.io/design/platform-guidance/cross-platform-adaptation.html).
  ///
  /// On iOS and macOS, this constructor creates a [CupertinoAlertDialog]. On
  /// other platforms, this creates a Material design [AlertDialog].
  ///
  /// Typically passed as a child of [showAdaptiveDialog], which will display
  /// the alert differently based on platform.
  ///
  /// If a [CupertinoAlertDialog] is created only these parameters are used:
  /// [title], [content], [actions], [scrollController],
  /// [actionScrollController], [insetAnimationDuration], and
  /// [insetAnimationCurve]. If a material [AlertDialog] is created,
  /// [scrollController], [actionScrollController], [insetAnimationDuration],
  /// and [insetAnimationCurve] are ignored.
  ///
  /// The target platform is based on the current [Theme]: [ThemeData.platform].
  ///
  /// {@tool dartpad}
  /// This demo shows a [TextButton] which when pressed, calls [showAdaptiveDialog].
  /// When called, this method displays an adaptive dialog above the current
  /// contents of the app, with different behaviors depending on target platform.
  ///
  /// [CupertinoDialogAction] is conditionally used as the child to show more
  /// platform specific design.
  ///
  /// ** See code in examples/api/lib/material/dialog/adaptive_alert_dialog.0.dart **
  /// {@end-tool}
  const factory AlertDialog.adaptive({
    Key? key,
    Widget? icon,
    EdgeInsetsGeometry? iconPadding,
    Color? iconColor,
    Widget? title,
    EdgeInsetsGeometry? titlePadding,
    TextStyle? titleTextStyle,
    Widget? content,
    EdgeInsetsGeometry? contentPadding,
    TextStyle? contentTextStyle,
    List<Widget>? actions,
    EdgeInsetsGeometry? actionsPadding,
    MainAxisAlignment? actionsAlignment,
    OverflowBarAlignment? actionsOverflowAlignment,
    VerticalDirection? actionsOverflowDirection,
    double? actionsOverflowButtonSpacing,
    EdgeInsetsGeometry? buttonPadding,
    Color? backgroundColor,
    double? elevation,
    Color? shadowColor,
    Color? surfaceTintColor,
    String? semanticLabel,
    EdgeInsets insetPadding,
    Clip? clipBehavior,
    ShapeBorder? shape,
    AlignmentGeometry? alignment,
    bool scrollable,
    ScrollController? scrollController,
    ScrollController? actionScrollController,
    Duration insetAnimationDuration,
    Curve insetAnimationCurve,
  }) = _AdaptiveAlertDialog;

  /// An optional icon to display at the top of the dialog.
  ///
  /// Typically, an [Icon] widget. Providing an icon centers the [title]'s text.
  final Widget? icon;

  /// Color for the [Icon] in the [icon] of this [AlertDialog].
  ///
  /// If null, [DialogThemeData.iconColor] is used. If that is null, defaults to
  /// color scheme's [ColorScheme.secondary] if [ThemeData.useMaterial3] is
  /// true, black otherwise.
  final Color? iconColor;

  /// Padding around the [icon].
  ///
  /// If there is no [icon], no padding will be provided. Otherwise, this
  /// padding is used.
  ///
  /// This property defaults to providing 24 pixels on the top, left, and right
  /// of the [icon]. If [title] is _not_ null, 16 pixels of bottom padding is
  /// added to separate the [icon] from the [title]. If the [title] is null and
  /// [content] is _not_ null, then no bottom padding is provided (but see
  /// [contentPadding]). In any other case 24 pixels of bottom padding is
  /// added.
  final EdgeInsetsGeometry? iconPadding;

  /// The (optional) title of the dialog is displayed in a large font at the top
  /// of the dialog, below the (optional) [icon].
  ///
  /// Typically a [Text] widget.
  final Widget? title;

  /// Padding around the title.
  ///
  /// If there is no title, no padding will be provided. Otherwise, this padding
  /// is used.
  ///
  /// This property defaults to providing 24 pixels on the top, left, and right
  /// of the title. If the [content] is not null, then no bottom padding is
  /// provided (but see [contentPadding]). If it _is_ null, then an extra 20
  /// pixels of bottom padding is added to separate the [title] from the
  /// [actions].
  final EdgeInsetsGeometry? titlePadding;

  /// Style for the text in the [title] of this [AlertDialog].
  ///
  /// If null, [DialogThemeData.titleTextStyle] is used. If that's null, defaults to
  /// [TextTheme.headlineSmall] of [ThemeData.textTheme] if
  /// [ThemeData.useMaterial3] is true, [TextTheme.titleLarge] otherwise.
  final TextStyle? titleTextStyle;

  /// The (optional) content of the dialog is displayed in the center of the
  /// dialog in a lighter font.
  ///
  /// Typically this is a [SingleChildScrollView] that contains the dialog's
  /// message. As noted in the [AlertDialog] documentation, it's important
  /// to use a [SingleChildScrollView] if there's any risk that the content
  /// will not fit, as the contents will otherwise overflow the dialog.
  ///
  /// The [content] must support reporting its intrinsic dimensions. In
  /// particular, [ListView], [GridView], and [CustomScrollView] cannot be used
  /// here unless they are first wrapped in a widget that itself can report
  /// intrinsic dimensions, such as a [SizedBox].
  final Widget? content;

  /// Padding around the content.
  ///
  /// If there is no [content], no padding will be provided. Otherwise, this
  /// padding is used.
  ///
  /// This property defaults to providing a padding of 20 pixels above the
  /// [content] to separate the [content] from the [title], and 24 pixels on the
  /// left, right, and bottom to separate the [content] from the other edges of
  /// the dialog.
  ///
  /// If [ThemeData.useMaterial3] is true, the top padding separating the
  /// content from the title defaults to 16 pixels instead of 20 pixels.
  final EdgeInsetsGeometry? contentPadding;

  /// Style for the text in the [content] of this [AlertDialog].
  ///
  /// If null, [DialogThemeData.contentTextStyle] is used. If that's null, defaults
  /// to [TextTheme.bodyMedium] of [ThemeData.textTheme] if
  /// [ThemeData.useMaterial3] is true, [TextTheme.titleMedium] otherwise.
  final TextStyle? contentTextStyle;

  /// The (optional) set of actions that are displayed at the bottom of the
  /// dialog with an [OverflowBar].
  ///
  /// Typically this is a list of [TextButton] widgets. It is recommended to
  /// set the [Text.textAlign] to [TextAlign.end] for the [Text] within the
  /// [TextButton], so that buttons whose labels wrap to an extra line align
  /// with the overall [OverflowBar]'s alignment within the dialog.
  ///
  /// If the [title] is not null but the [content] _is_ null, then an extra 20
  /// pixels of padding is added above the [OverflowBar] to separate the [title]
  /// from the [actions].
  final List<Widget>? actions;

  /// Padding around the set of [actions] at the bottom of the dialog.
  ///
  /// Typically used to provide padding to the button bar between the button bar
  /// and the edges of the dialog.
  ///
  /// The [buttonPadding] may contribute to the padding on the edges of
  /// [actions] as well.
  ///
  /// If there are no [actions], then no padding will be included.
  ///
  /// {@tool snippet}
  /// This is an example of a set of actions aligned with the content widget.
  /// ```dart
  /// AlertDialog(
  ///   title: const Text('Title'),
  ///   content: Container(width: 200, height: 200, color: Colors.green),
  ///   actions: <Widget>[
  ///     ElevatedButton(onPressed: () {}, child: const Text('Button 1')),
  ///     ElevatedButton(onPressed: () {}, child: const Text('Button 2')),
  ///   ],
  ///   actionsPadding: const EdgeInsets.symmetric(horizontal: 8.0),
  /// )
  /// ```
  /// {@end-tool}
  ///
  /// See also:
  ///
  /// * [OverflowBar], which [actions] configures to lay itself out.
  final EdgeInsetsGeometry? actionsPadding;

  /// Defines the horizontal layout of the [actions] according to the same
  /// rules as for [Row.mainAxisAlignment].
  ///
  /// This parameter is passed along to the dialog's [OverflowBar].
  ///
  /// If this parameter is null (the default) then [MainAxisAlignment.end]
  /// is used.
  final MainAxisAlignment? actionsAlignment;

  /// The horizontal alignment of [actions] within the vertical
  /// "overflow" layout.
  ///
  /// If the dialog's [actions] do not fit into a single row, then they
  /// are arranged in a column. This parameter controls the horizontal
  /// alignment of widgets in the case of an overflow.
  ///
  /// If this parameter is null (the default) then [OverflowBarAlignment.end]
  /// is used.
  ///
  /// See also:
  ///
  /// * [OverflowBar], which [actions] configures to lay itself out.
  final OverflowBarAlignment? actionsOverflowAlignment;

  /// The vertical direction of [actions] if the children overflow
  /// horizontally.
  ///
  /// If the dialog's [actions] do not fit into a single row, then they
  /// are arranged in a column. The first action is at the top of the
  /// column if this property is set to [VerticalDirection.down], since it
  /// "starts" at the top and "ends" at the bottom. On the other hand,
  /// the first action will be at the bottom of the column if this
  /// property is set to [VerticalDirection.up], since it "starts" at the
  /// bottom and "ends" at the top.
  ///
  /// See also:
  ///
  /// * [OverflowBar], which [actions] configures to lay itself out.
  final VerticalDirection? actionsOverflowDirection;

  /// The spacing between [actions] when the [OverflowBar] switches to a column
  /// layout because the actions don't fit horizontally.
  ///
  /// If the widgets in [actions] do not fit into a single row, they are
  /// arranged into a column. This parameter provides additional vertical space
  /// between buttons when it does overflow.
  ///
  /// The button spacing may appear to be more than the value provided. This is
  /// because most buttons adhere to the [MaterialTapTargetSize] of 48px. So,
  /// even though a button might visually be 36px in height, it might still take
  /// up to 48px vertically.
  ///
  /// If null then no spacing will be added in between buttons in an overflow
  /// state.
  final double? actionsOverflowButtonSpacing;

  /// The padding that surrounds each button in [actions].
  ///
  /// This is different from [actionsPadding], which defines the padding
  /// between the entire button bar and the edges of the dialog.
  ///
  /// If this property is null, then it will default to
  /// 8.0 logical pixels on the left and right.
  final EdgeInsetsGeometry? buttonPadding;

  /// {@macro flutter.material.dialog.backgroundColor}
  final Color? backgroundColor;

  /// {@macro flutter.material.dialog.elevation}
  final double? elevation;

  /// {@macro flutter.material.dialog.shadowColor}
  final Color? shadowColor;

  /// {@macro flutter.material.dialog.surfaceTintColor}
  final Color? surfaceTintColor;

  /// The semantic label of the dialog used by accessibility frameworks to
  /// announce screen transitions when the dialog is opened and closed.
  ///
  /// In iOS, if this label is not provided, a semantic label will be inferred
  /// from the [title] if it is not null.
  ///
  /// In Android, if this label is not provided, the dialog will use the
  /// [MaterialLocalizations.alertDialogLabel] as its label.
  ///
  /// See also:
  ///
  ///  * [SemanticsConfiguration.namesRoute], for a description of how this
  ///    value is used.
  final String? semanticLabel;

  /// {@macro flutter.material.dialog.insetPadding}
  final EdgeInsets? insetPadding;

  /// {@macro flutter.material.dialog.clipBehavior}
  final Clip? clipBehavior;

  /// {@macro flutter.material.dialog.shape}
  final ShapeBorder? shape;

  /// {@macro flutter.material.dialog.alignment}
  final AlignmentGeometry? alignment;

  /// Determines whether the [title] and [content] widgets are wrapped in a
  /// scrollable.
  ///
  /// This configuration is used when the [title] and [content] are expected
  /// to overflow. Both [title] and [content] are wrapped in a scroll view,
  /// allowing all overflowed content to be visible while still showing the
  /// button bar.
  final bool scrollable;

  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasMaterialLocalizations(context));
    final ThemeData theme = Theme.of(context);

    final DialogThemeData dialogTheme = DialogTheme.of(context);
    final DialogThemeData defaults =
        theme.useMaterial3 ? _DialogDefaultsM3(context) : _DialogDefaultsM2(context);

    String? label = semanticLabel;
    switch (theme.platform) {
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
        break;
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        label ??= MaterialLocalizations.of(context).alertDialogLabel;
    }

    // The paddingScaleFactor is used to adjust the padding of Dialog's
    // children.
    const double fontSizeToScale = 14.0;
    final double effectiveTextScale =
        MediaQuery.textScalerOf(context).scale(fontSizeToScale) / fontSizeToScale;
    final double paddingScaleFactor = _scalePadding(effectiveTextScale);
    final TextDirection? textDirection = Directionality.maybeOf(context);

    Widget? iconWidget;
    Widget? titleWidget;
    Widget? contentWidget;
    Widget? actionsWidget;

    if (icon != null) {
      final bool belowIsTitle = title != null;
      final bool belowIsContent = !belowIsTitle && content != null;
      final EdgeInsets defaultIconPadding = EdgeInsets.only(
        left: 24.0,
        top: 24.0,
        right: 24.0,
        bottom:
            belowIsTitle
                ? 16.0
                : belowIsContent
                ? 0.0
                : 24.0,
      );
      final EdgeInsets effectiveIconPadding =
          iconPadding?.resolve(textDirection) ?? defaultIconPadding;
      iconWidget = Padding(
        padding: EdgeInsets.only(
          left: effectiveIconPadding.left * paddingScaleFactor,
          right: effectiveIconPadding.right * paddingScaleFactor,
          top: effectiveIconPadding.top * paddingScaleFactor,
          bottom: effectiveIconPadding.bottom,
        ),
        child: IconTheme(
          data: IconThemeData(color: iconColor ?? dialogTheme.iconColor ?? defaults.iconColor),
          child: icon!,
        ),
      );
    }

    if (title != null) {
      final EdgeInsets defaultTitlePadding = EdgeInsets.only(
        left: 24.0,
        top: icon == null ? 24.0 : 0.0,
        right: 24.0,
        bottom: content == null ? 20.0 : 0.0,
      );
      final EdgeInsets effectiveTitlePadding =
          titlePadding?.resolve(textDirection) ?? defaultTitlePadding;
      titleWidget = Padding(
        padding: EdgeInsets.only(
          left: effectiveTitlePadding.left * paddingScaleFactor,
          right: effectiveTitlePadding.right * paddingScaleFactor,
          top:
              icon == null
                  ? effectiveTitlePadding.top * paddingScaleFactor
                  : effectiveTitlePadding.top,
          bottom: effectiveTitlePadding.bottom,
        ),
        child: DefaultTextStyle(
          style: titleTextStyle ?? dialogTheme.titleTextStyle ?? defaults.titleTextStyle!,
          textAlign: icon == null ? TextAlign.start : TextAlign.center,
          child: Semantics(
            // For iOS platform, the focus always lands on the title.
            // Set nameRoute to false to avoid title being announce twice.
            namesRoute: label == null && theme.platform != TargetPlatform.iOS,
            container: true,
            child: title,
          ),
        ),
      );
    }

    if (content != null) {
      final EdgeInsets defaultContentPadding = EdgeInsets.only(
        left: 24.0,
        top: theme.useMaterial3 ? 16.0 : 20.0,
        right: 24.0,
        bottom: 24.0,
      );
      final EdgeInsets effectiveContentPadding =
          contentPadding?.resolve(textDirection) ?? defaultContentPadding;
      contentWidget = Padding(
        padding: EdgeInsets.only(
          left: effectiveContentPadding.left * paddingScaleFactor,
          right: effectiveContentPadding.right * paddingScaleFactor,
          top:
              title == null && icon == null
                  ? effectiveContentPadding.top * paddingScaleFactor
                  : effectiveContentPadding.top,
          bottom: effectiveContentPadding.bottom,
        ),
        child: DefaultTextStyle(
          style: contentTextStyle ?? dialogTheme.contentTextStyle ?? defaults.contentTextStyle!,
          child: Semantics(container: true, explicitChildNodes: true, child: content),
        ),
      );
    }

    if (actions != null) {
      final double spacing = (buttonPadding?.horizontal ?? 16) / 2;
      actionsWidget = Padding(
        padding:
            actionsPadding ??
            dialogTheme.actionsPadding ??
            (theme.useMaterial3
                ? defaults.actionsPadding!
                : defaults.actionsPadding!.add(EdgeInsets.all(spacing))),
        child: OverflowBar(
          alignment: actionsAlignment ?? MainAxisAlignment.end,
          spacing: spacing,
          overflowAlignment: actionsOverflowAlignment ?? OverflowBarAlignment.end,
          overflowDirection: actionsOverflowDirection ?? VerticalDirection.down,
          overflowSpacing: actionsOverflowButtonSpacing ?? 0,
          children: actions!,
        ),
      );
    }

    List<Widget> columnChildren;
    if (scrollable) {
      columnChildren = <Widget>[
        if (title != null || content != null)
          Flexible(
            child: SingleChildScrollView(
              child: Column(
                mainAxisSize: MainAxisSize.min,
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: <Widget>[
                  if (icon != null) iconWidget!,
                  if (title != null) titleWidget!,
                  if (content != null) contentWidget!,
                ],
              ),
            ),
          ),
        if (actions != null) actionsWidget!,
      ];
    } else {
      columnChildren = <Widget>[
        if (icon != null) iconWidget!,
        if (title != null) titleWidget!,
        if (content != null) Flexible(child: contentWidget!),
        if (actions != null) actionsWidget!,
      ];
    }

    Widget dialogChild = IntrinsicWidth(
      child: Column(
        mainAxisSize: MainAxisSize.min,
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: columnChildren,
      ),
    );

    if (label != null) {
      dialogChild = Semantics(
        scopesRoute: true,
        explicitChildNodes: true,
        namesRoute: true,
        label: label,
        child: dialogChild,
      );
    }

    return Dialog(
      backgroundColor: backgroundColor,
      elevation: elevation,
      shadowColor: shadowColor,
      surfaceTintColor: surfaceTintColor,
      insetPadding: insetPadding,
      clipBehavior: clipBehavior,
      shape: shape,
      alignment: alignment,
      child: dialogChild,
    );
  }
}

class _AdaptiveAlertDialog extends AlertDialog {
  const _AdaptiveAlertDialog({
    super.key,
    super.icon,
    super.iconPadding,
    super.iconColor,
    super.title,
    super.titlePadding,
    super.titleTextStyle,
    super.content,
    super.contentPadding,
    super.contentTextStyle,
    super.actions,
    super.actionsPadding,
    super.actionsAlignment,
    super.actionsOverflowAlignment,
    super.actionsOverflowDirection,
    super.actionsOverflowButtonSpacing,
    super.buttonPadding,
    super.backgroundColor,
    super.elevation,
    super.shadowColor,
    super.surfaceTintColor,
    super.semanticLabel,
    super.insetPadding,
    super.clipBehavior,
    super.shape,
    super.alignment,
    super.scrollable = false,
    this.scrollController,
    this.actionScrollController,
    this.insetAnimationDuration = const Duration(milliseconds: 100),
    this.insetAnimationCurve = Curves.decelerate,
  });

  final ScrollController? scrollController;
  final ScrollController? actionScrollController;
  final Duration insetAnimationDuration;
  final Curve insetAnimationCurve;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    switch (theme.platform) {
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        break;
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
        return CupertinoAlertDialog(
          title: title,
          content: content,
          actions: actions ?? <Widget>[],
          scrollController: scrollController,
          actionScrollController: actionScrollController,
          insetAnimationDuration: insetAnimationDuration,
          insetAnimationCurve: insetAnimationCurve,
        );
    }
    return super.build(context);
  }
}

/// An option used in a [SimpleDialog].
///
/// A simple dialog offers the user a choice between several options. This
/// widget is commonly used to represent each of the options. If the user
/// selects this option, the widget will call the [onPressed] callback, which
/// typically uses [Navigator.pop] to close the dialog.
///
/// The padding on a [SimpleDialogOption] is configured to combine with the
/// default [SimpleDialog.contentPadding] so that each option ends up 8 pixels
/// from the other vertically, with 20 pixels of spacing between the dialog's
/// title and the first option, and 24 pixels of spacing between the last option
/// and the bottom of the dialog.
///
/// {@tool snippet}
///
/// ```dart
/// SimpleDialogOption(
///   onPressed: () { Navigator.pop(context, Department.treasury); },
///   child: const Text('Treasury department'),
/// )
/// ```
/// {@end-tool}
///
/// See also:
///
///  * [SimpleDialog], for a dialog in which to use this widget.
///  * [showDialog], which actually displays the dialog and returns its result.
///  * [TextButton], which are commonly used as actions in other kinds of
///    dialogs, such as [AlertDialog]s.
///  * <https://material.io/design/components/dialogs.html#simple-dialog>
class SimpleDialogOption extends StatelessWidget {
  /// Creates an option for a [SimpleDialog].
  const SimpleDialogOption({super.key, this.onPressed, this.padding, this.child});

  /// The callback that is called when this option is selected.
  ///
  /// If this is set to null, the option cannot be selected.
  ///
  /// When used in a [SimpleDialog], this will typically call [Navigator.pop]
  /// with a value for [showDialog] to complete its future with.
  final VoidCallback? onPressed;

  /// The widget below this widget in the tree.
  ///
  /// Typically a [Text] widget.
  final Widget? child;

  /// The amount of space to surround the [child] with.
  ///
  /// Defaults to EdgeInsets.symmetric(vertical: 8.0, horizontal: 24.0).
  final EdgeInsets? padding;

  @override
  Widget build(BuildContext context) {
    return InkWell(
      onTap: onPressed,
      child: Padding(
        padding: padding ?? const EdgeInsets.symmetric(vertical: 8.0, horizontal: 24.0),
        child: child,
      ),
    );
  }
}

/// A simple Material Design dialog.
///
/// A simple dialog offers the user a choice between several options. A simple
/// dialog has an optional title that is displayed above the choices.
///
/// Choices are normally represented using [SimpleDialogOption] widgets. If
/// other widgets are used, see [contentPadding] for notes regarding the
/// conventions for obtaining the spacing expected by Material Design.
///
/// For dialogs that inform the user about a situation, consider using an
/// [AlertDialog].
///
/// Typically passed as the child widget to [showDialog], which displays the
/// dialog.
///
/// {@animation 350 622 https://flutter.github.io/assets-for-api-docs/assets/material/simple_dialog.mp4}
///
/// {@tool snippet}
///
/// In this example, the user is asked to select between two options. These
/// options are represented as an enum. The [showDialog] method here returns
/// a [Future] that completes to a value of that enum. If the user cancels
/// the dialog (e.g. by hitting the back button on Android, or tapping on the
/// mask behind the dialog) then the future completes with the null value.
///
/// The return value in this example is used as the index for a switch statement.
/// One advantage of using an enum as the return value and then using that to
/// drive a switch statement is that the analyzer will flag any switch statement
/// that doesn't mention every value in the enum.
///
/// ```dart
/// Future<void> _askedToLead() async {
///   switch (await showDialog<Department>(
///     context: context,
///     builder: (BuildContext context) {
///       return SimpleDialog(
///         title: const Text('Select assignment'),
///         children: <Widget>[
///           SimpleDialogOption(
///             onPressed: () { Navigator.pop(context, Department.treasury); },
///             child: const Text('Treasury department'),
///           ),
///           SimpleDialogOption(
///             onPressed: () { Navigator.pop(context, Department.state); },
///             child: const Text('State department'),
///           ),
///         ],
///       );
///     }
///   )) {
///     case Department.treasury:
///       // Let's go.
///       // ...
///     break;
///     case Department.state:
///       // ...
///     break;
///     case null:
///       // dialog dismissed
///     break;
///   }
/// }
/// ```
/// {@end-tool}
///
/// See also:
///
///  * [SimpleDialogOption], which are options used in this type of dialog.
///  * [AlertDialog], for dialogs that have a row of buttons below the body.
///  * [Dialog], on which [SimpleDialog] and [AlertDialog] are based.
///  * [showDialog], which actually displays the dialog and returns its result.
///  * <https://material.io/design/components/dialogs.html#simple-dialog>
class SimpleDialog extends StatelessWidget {
  /// Creates a simple dialog.
  ///
  /// Typically used in conjunction with [showDialog].
  const SimpleDialog({
    super.key,
    this.title,
    this.titlePadding = const EdgeInsets.fromLTRB(24.0, 24.0, 24.0, 0.0),
    this.titleTextStyle,
    this.children,
    this.contentPadding = const EdgeInsets.fromLTRB(0.0, 12.0, 0.0, 16.0),
    this.backgroundColor,
    this.elevation,
    this.shadowColor,
    this.surfaceTintColor,
    this.semanticLabel,
    this.insetPadding,
    this.clipBehavior,
    this.shape,
    this.alignment,
  });

  /// The (optional) title of the dialog is displayed in a large font at the top
  /// of the dialog.
  ///
  /// Typically a [Text] widget.
  final Widget? title;

  /// Padding around the title.
  ///
  /// If there is no title, no padding will be provided.
  ///
  /// By default, this provides the recommend Material Design padding of 24
  /// pixels around the left, top, and right edges of the title.
  ///
  /// See [contentPadding] for the conventions regarding padding between the
  /// [title] and the [children].
  final EdgeInsetsGeometry titlePadding;

  /// Style for the text in the [title] of this [SimpleDialog].
  ///
  /// If null, [DialogThemeData.titleTextStyle] is used. If that's null, defaults to
  /// [TextTheme.titleLarge] of [ThemeData.textTheme].
  final TextStyle? titleTextStyle;

  /// The (optional) content of the dialog is displayed in a
  /// [SingleChildScrollView] underneath the title.
  ///
  /// Typically a list of [SimpleDialogOption]s.
  final List<Widget>? children;

  /// Padding around the content.
  ///
  /// By default, this is 12 pixels on the top and 16 pixels on the bottom. This
  /// is intended to be combined with children that have 24 pixels of padding on
  /// the left and right, and 8 pixels of padding on the top and bottom, so that
  /// the content ends up being indented 20 pixels from the title, 24 pixels
  /// from the bottom, and 24 pixels from the sides.
  ///
  /// The [SimpleDialogOption] widget uses such padding.
  ///
  /// If there is no [title], the [contentPadding] should be adjusted so that
  /// the top padding ends up being 24 pixels.
  final EdgeInsetsGeometry contentPadding;

  /// {@macro flutter.material.dialog.backgroundColor}
  final Color? backgroundColor;

  /// {@macro flutter.material.dialog.elevation}
  final double? elevation;

  /// {@macro flutter.material.dialog.shadowColor}
  final Color? shadowColor;

  /// {@macro flutter.material.dialog.surfaceTintColor}
  final Color? surfaceTintColor;

  /// The semantic label of the dialog used by accessibility frameworks to
  /// announce screen transitions when the dialog is opened and closed.
  ///
  /// If this label is not provided, a semantic label will be inferred from the
  /// [title] if it is not null. If there is no title, the label will be taken
  /// from [MaterialLocalizations.dialogLabel].
  ///
  /// See also:
  ///
  ///  * [SemanticsConfiguration.namesRoute], for a description of how this
  ///    value is used.
  final String? semanticLabel;

  /// {@macro flutter.material.dialog.insetPadding}
  final EdgeInsets? insetPadding;

  /// {@macro flutter.material.dialog.clipBehavior}
  final Clip? clipBehavior;

  /// {@macro flutter.material.dialog.shape}
  final ShapeBorder? shape;

  /// {@macro flutter.material.dialog.shape}
  final AlignmentGeometry? alignment;

  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasMaterialLocalizations(context));
    final ThemeData theme = Theme.of(context);

    String? label = semanticLabel;
    switch (theme.platform) {
      case TargetPlatform.macOS:
      case TargetPlatform.iOS:
        break;
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        label ??= MaterialLocalizations.of(context).dialogLabel;
    }

    // The paddingScaleFactor is used to adjust the padding of Dialog
    // children.
    final TextStyle defaultTextStyle =
        titleTextStyle ?? DialogTheme.of(context).titleTextStyle ?? theme.textTheme.titleLarge!;
    final double fontSize = defaultTextStyle.fontSize ?? kDefaultFontSize;
    final double fontSizeToScale = fontSize == 0.0 ? kDefaultFontSize : fontSize;
    final double effectiveTextScale =
        MediaQuery.textScalerOf(context).scale(fontSizeToScale) / fontSizeToScale;
    final double paddingScaleFactor = _scalePadding(effectiveTextScale);
    final TextDirection? textDirection = Directionality.maybeOf(context);

    Widget? titleWidget;
    if (title != null) {
      final EdgeInsets effectiveTitlePadding = titlePadding.resolve(textDirection);
      titleWidget = Padding(
        padding: EdgeInsets.only(
          left: effectiveTitlePadding.left * paddingScaleFactor,
          right: effectiveTitlePadding.right * paddingScaleFactor,
          top: effectiveTitlePadding.top * paddingScaleFactor,
          bottom:
              children == null
                  ? effectiveTitlePadding.bottom * paddingScaleFactor
                  : effectiveTitlePadding.bottom,
        ),
        child: DefaultTextStyle(
          style: defaultTextStyle,
          child: Semantics(
            // For iOS platform, the focus always lands on the title.
            // Set nameRoute to false to avoid title being announce twice.
            namesRoute: label == null && theme.platform != TargetPlatform.iOS,
            container: true,
            child: title,
          ),
        ),
      );
    }

    Widget? contentWidget;
    if (children != null) {
      final EdgeInsets effectiveContentPadding = contentPadding.resolve(textDirection);
      contentWidget = Flexible(
        child: SingleChildScrollView(
          padding: EdgeInsets.only(
            left: effectiveContentPadding.left * paddingScaleFactor,
            right: effectiveContentPadding.right * paddingScaleFactor,
            top:
                title == null
                    ? effectiveContentPadding.top * paddingScaleFactor
                    : effectiveContentPadding.top,
            bottom: effectiveContentPadding.bottom * paddingScaleFactor,
          ),
          child: ListBody(children: children!),
        ),
      );
    }

    Widget dialogChild = IntrinsicWidth(
      stepWidth: 56.0,
      child: ConstrainedBox(
        constraints: const BoxConstraints(minWidth: 280.0),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: <Widget>[if (title != null) titleWidget!, if (children != null) contentWidget!],
        ),
      ),
    );

    if (label != null) {
      dialogChild = Semantics(
        scopesRoute: true,
        explicitChildNodes: true,
        namesRoute: true,
        label: label,
        child: dialogChild,
      );
    }
    return Dialog(
      backgroundColor: backgroundColor,
      elevation: elevation,
      shadowColor: shadowColor,
      surfaceTintColor: surfaceTintColor,
      insetPadding: insetPadding,
      clipBehavior: clipBehavior,
      shape: shape,
      alignment: alignment,
      child: dialogChild,
    );
  }
}

Widget _buildMaterialDialogTransitions(
  BuildContext context,
  Animation<double> animation,
  Animation<double> secondaryAnimation,
  Widget child,
) {
  return child;
}

/// Displays a Material dialog above the current contents of the app, with
/// Material entrance and exit animations, modal barrier color, and modal
/// barrier behavior (dialog is dismissible with a tap on the barrier).
///
/// This function takes a `builder` which typically builds a [Dialog] widget.
/// Content below the dialog is dimmed with a [ModalBarrier]. The widget
/// returned by the `builder` does not share a context with the location that
/// [showDialog] is originally called from. Use a [StatefulBuilder] or a
/// custom [StatefulWidget] if the dialog needs to update dynamically.
///
/// The `context` argument is used to look up the [Navigator] and [Theme] for
/// the dialog. It is only used when the method is called. Its corresponding
/// widget can be safely removed from the tree before the dialog is closed.
///
/// The `barrierDismissible` argument is used to indicate whether tapping on the
/// barrier will dismiss the dialog. It is `true` by default and can not be `null`.
///
/// The `barrierColor` argument is used to specify the color of the modal
/// barrier that darkens everything below the dialog. If `null` the `barrierColor`
/// field from `DialogThemeData` is used. If that is `null` the default color
/// `Colors.black54` is used.
///
/// The `useSafeArea` argument is used to indicate if the dialog should only
/// display in 'safe' areas of the screen not used by the operating system
/// (see [SafeArea] for more details). It is `true` by default, which means
/// the dialog will not overlap operating system areas. If it is set to `false`
/// the dialog will only be constrained by the screen size. It can not be `null`.
///
/// The `useRootNavigator` argument is used to determine whether to push the
/// dialog to the [Navigator] furthest from or nearest to the given `context`.
/// By default, `useRootNavigator` is `true` and the dialog route created by
/// this method is pushed to the root navigator. It can not be `null`.
///
/// The `routeSettings` argument is passed to [showGeneralDialog],
/// see [RouteSettings] for details.
///
/// If not null, the `traversalEdgeBehavior` argument specifies the transfer of
/// focus beyond the first and the last items of the dialog route. By default,
/// [TraversalEdgeBehavior.closedLoop] is used, because it's typical for dialogs
/// to allow users to cycle through dialog widgets without leaving the dialog.
///
/// {@macro flutter.widgets.RawDialogRoute}
///
/// If the application has multiple [Navigator] objects, it may be necessary to
/// call `Navigator.of(context, rootNavigator: true).pop(result)` to close the
/// dialog rather than just `Navigator.pop(context, result)`.
///
/// Returns a [Future] that resolves to the value (if any) that was passed to
/// [Navigator.pop] when the dialog was closed.
///
/// {@tool dartpad}
/// This sample demonstrates how to use [showDialog] to display a dialog box.
///
/// ** See code in examples/api/lib/material/dialog/show_dialog.0.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This sample shows the creation of [showDialog], as described in:
/// https://m3.material.io/components/dialogs/overview
///
/// ** See code in examples/api/lib/material/dialog/show_dialog.1.dart **
/// {@end-tool}
///
/// ### State Restoration in Dialogs
///
/// Using this method will not enable state restoration for the dialog. In order
/// to enable state restoration for a dialog, use [Navigator.restorablePush]
/// or [Navigator.restorablePushNamed] with [DialogRoute].
///
/// For more information about state restoration, see [RestorationManager].
///
/// {@tool dartpad}
/// This sample demonstrates how to create a restorable Material dialog. This is
/// accomplished by enabling state restoration by specifying
/// [MaterialApp.restorationScopeId] and using [Navigator.restorablePush] to
/// push [DialogRoute] when the button is tapped.
///
/// {@macro flutter.widgets.RestorationManager}
///
/// ** See code in examples/api/lib/material/dialog/show_dialog.2.dart **
/// {@end-tool}
///
/// See also:
///
///  * [AlertDialog], for dialogs that have a row of buttons below a body.
///  * [SimpleDialog], which handles the scrolling of the contents and does
///    not show buttons below its body.
///  * [Dialog], on which [SimpleDialog] and [AlertDialog] are based.
///  * [showCupertinoDialog], which displays an iOS-style dialog.
///  * [showGeneralDialog], which allows for customization of the dialog popup.
///  * [DisplayFeatureSubScreen], which documents the specifics of how
///    [DisplayFeature]s can split the screen into sub-screens.
///  * <https://material.io/design/components/dialogs.html>
///  * <https://m3.material.io/components/dialogs>
Future<T?> showDialog<T>({
  required BuildContext context,
  required WidgetBuilder builder,
  bool barrierDismissible = true,
  Color? barrierColor,
  String? barrierLabel,
  bool useSafeArea = true,
  bool useRootNavigator = true,
  RouteSettings? routeSettings,
  Offset? anchorPoint,
  TraversalEdgeBehavior? traversalEdgeBehavior,
}) {
  assert(_debugIsActive(context));
  assert(debugCheckHasMaterialLocalizations(context));

  final CapturedThemes themes = InheritedTheme.capture(
    from: context,
    to: Navigator.of(context, rootNavigator: useRootNavigator).context,
  );

  return Navigator.of(context, rootNavigator: useRootNavigator).push<T>(
    DialogRoute<T>(
      context: context,
      builder: builder,
      barrierColor:
          barrierColor ??
          DialogTheme.of(context).barrierColor ??
          Theme.of(context).dialogTheme.barrierColor ??
          Colors.black54,
      barrierDismissible: barrierDismissible,
      barrierLabel: barrierLabel,
      useSafeArea: useSafeArea,
      settings: routeSettings,
      themes: themes,
      anchorPoint: anchorPoint,
      traversalEdgeBehavior: traversalEdgeBehavior ?? TraversalEdgeBehavior.closedLoop,
    ),
  );
}

/// Displays either a Material or Cupertino dialog depending on platform.
///
/// On most platforms this function will act the same as [showDialog], except
/// for iOS and macOS, in which case it will act the same as
/// [showCupertinoDialog].
///
/// On Cupertino platforms, [barrierColor], [useSafeArea], and
/// [traversalEdgeBehavior] are ignored.
Future<T?> showAdaptiveDialog<T>({
  required BuildContext context,
  required WidgetBuilder builder,
  bool? barrierDismissible,
  Color? barrierColor,
  String? barrierLabel,
  bool useSafeArea = true,
  bool useRootNavigator = true,
  RouteSettings? routeSettings,
  Offset? anchorPoint,
  TraversalEdgeBehavior? traversalEdgeBehavior,
}) {
  final ThemeData theme = Theme.of(context);
  switch (theme.platform) {
    case TargetPlatform.android:
    case TargetPlatform.fuchsia:
    case TargetPlatform.linux:
    case TargetPlatform.windows:
      return showDialog<T>(
        context: context,
        builder: builder,
        barrierDismissible: barrierDismissible ?? true,
        barrierColor: barrierColor,
        barrierLabel: barrierLabel,
        useSafeArea: useSafeArea,
        useRootNavigator: useRootNavigator,
        routeSettings: routeSettings,
        anchorPoint: anchorPoint,
        traversalEdgeBehavior: traversalEdgeBehavior,
      );
    case TargetPlatform.iOS:
    case TargetPlatform.macOS:
      return showCupertinoDialog<T>(
        context: context,
        builder: builder,
        barrierDismissible: barrierDismissible ?? false,
        barrierLabel: barrierLabel,
        useRootNavigator: useRootNavigator,
        anchorPoint: anchorPoint,
        routeSettings: routeSettings,
      );
  }
}

bool _debugIsActive(BuildContext context) {
  if (context is Element && !context.debugIsActive) {
    throw FlutterError.fromParts(<DiagnosticsNode>[
      ErrorSummary('This BuildContext is no longer valid.'),
      ErrorDescription(
        'The showDialog function context parameter is a BuildContext that is no longer valid.',
      ),
      ErrorHint(
        'This can commonly occur when the showDialog function is called after awaiting a Future. '
        'In this situation the BuildContext might refer to a widget that has already been disposed during the await. '
        'Consider using a parent context instead.',
      ),
    ]);
  }
  return true;
}

/// A dialog route with Material entrance and exit animations,
/// modal barrier color, and modal barrier behavior (dialog is dismissible
/// with a tap on the barrier).
///
/// It is used internally by [showDialog] or can be directly pushed
/// onto the [Navigator] stack to enable state restoration. See
/// [showDialog] for a state restoration app example.
///
/// This function takes a `builder` which typically builds a [Dialog] widget.
/// Content below the dialog is dimmed with a [ModalBarrier]. The widget
/// returned by the `builder` does not share a context with the location that
/// `showDialog` is originally called from. Use a [StatefulBuilder] or a
/// custom [StatefulWidget] if the dialog needs to update dynamically.
///
/// The `context` argument is used to look up
/// [MaterialLocalizations.modalBarrierDismissLabel], which provides the
/// modal with a localized accessibility label that will be used for the
/// modal's barrier. However, a custom `barrierLabel` can be passed in as well.
///
/// The `barrierDismissible` argument is used to indicate whether tapping on the
/// barrier will dismiss the dialog. It is `true` by default and cannot be `null`.
///
/// The `barrierColor` argument is used to specify the color of the modal
/// barrier that darkens everything below the dialog. If `null`, the default
/// color `Colors.black54` is used.
///
/// The `useSafeArea` argument is used to indicate if the dialog should only
/// display in 'safe' areas of the screen not used by the operating system
/// (see [SafeArea] for more details). It is `true` by default, which means
/// the dialog will not overlap operating system areas. If it is set to `false`
/// the dialog will only be constrained by the screen size. It can not be `null`.
///
/// The `settings` argument define the settings for this route. See
/// [RouteSettings] for details.
///
/// {@macro flutter.widgets.RawDialogRoute}
///
/// See also:
///
///  * [showDialog], which is a way to display a DialogRoute.
///  * [showGeneralDialog], which allows for customization of the dialog popup.
///  * [showCupertinoDialog], which displays an iOS-style dialog.
///  * [DisplayFeatureSubScreen], which documents the specifics of how
///    [DisplayFeature]s can split the screen into sub-screens.
class DialogRoute<T> extends RawDialogRoute<T> {
  /// A dialog route with Material entrance and exit animations,
  /// modal barrier color, and modal barrier behavior (dialog is dismissible
  /// with a tap on the barrier).
  DialogRoute({
    required BuildContext context,
    required WidgetBuilder builder,
    CapturedThemes? themes,
    super.barrierColor = Colors.black54,
    super.barrierDismissible,
    String? barrierLabel,
    bool useSafeArea = true,
    super.settings,
    super.requestFocus,
    super.anchorPoint,
    super.traversalEdgeBehavior,
  }) : super(
         pageBuilder: (
           BuildContext buildContext,
           Animation<double> animation,
           Animation<double> secondaryAnimation,
         ) {
           final Widget pageChild = Builder(builder: builder);
           Widget dialog = themes?.wrap(pageChild) ?? pageChild;
           if (useSafeArea) {
             dialog = SafeArea(child: dialog);
           }
           return dialog;
         },
         barrierLabel: barrierLabel ?? MaterialLocalizations.of(context).modalBarrierDismissLabel,
         transitionDuration: const Duration(milliseconds: 150),
         transitionBuilder: _buildMaterialDialogTransitions,
       );

  CurvedAnimation? _curvedAnimation;

  void _setAnimation(Animation<double> animation) {
    if (_curvedAnimation?.parent != animation) {
      _curvedAnimation?.dispose();
      _curvedAnimation = CurvedAnimation(parent: animation, curve: Curves.easeOut);
    }
  }

  @override
  Widget buildTransitions(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
    Widget child,
  ) {
    _setAnimation(animation);
    return FadeTransition(
      opacity: _curvedAnimation!,
      child: super.buildTransitions(context, animation, secondaryAnimation, child),
    );
  }

  @override
  void dispose() {
    _curvedAnimation?.dispose();
    super.dispose();
  }
}

double _scalePadding(double textScaleFactor) {
  final double clampedTextScaleFactor = clampDouble(textScaleFactor, 1.0, 2.0);
  // The final padding scale factor is clamped between 1/3 and 1. For example,
  // a non-scaled padding of 24 will produce a padding between 24 and 8.
  return lerpDouble(1.0, 1.0 / 3.0, clampedTextScaleFactor - 1.0)!;
}

// Hand coded defaults based on Material Design 2.
class _DialogDefaultsM2 extends DialogThemeData {
  _DialogDefaultsM2(this.context)
    : super(
        alignment: Alignment.center,
        elevation: 24.0,
        shape: const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(4.0))),
        clipBehavior: Clip.none,
      );

  final BuildContext context;
  late final ThemeData theme = Theme.of(context);
  late final TextTheme textTheme = theme.textTheme;
  late final IconThemeData iconTheme = theme.iconTheme;

  @override
  Color? get iconColor => iconTheme.color;

  @override
  Color? get backgroundColor =>
      theme.brightness == Brightness.dark ? Colors.grey[800]! : Colors.white;

  @override
  Color? get shadowColor => theme.shadowColor;

  @override
  TextStyle? get titleTextStyle => textTheme.titleLarge;

  @override
  TextStyle? get contentTextStyle => textTheme.titleMedium;

  @override
  EdgeInsetsGeometry? get actionsPadding => EdgeInsets.zero;
}

// BEGIN GENERATED TOKEN PROPERTIES - Dialog

// Do not edit by hand. The code between the "BEGIN GENERATED" and
// "END GENERATED" comments are generated from data in the Material
// Design token database by the script:
//   dev/tools/gen_defaults/bin/gen_defaults.dart.

// dart format off
class _DialogDefaultsM3 extends DialogThemeData {
  _DialogDefaultsM3(this.context)
    : super(
        alignment: Alignment.center,
        elevation: 6.0,
        shape: const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(28.0))),
        clipBehavior: Clip.none,
      );

  final BuildContext context;
  late final ColorScheme _colors = Theme.of(context).colorScheme;
  late final TextTheme _textTheme = Theme.of(context).textTheme;

  @override
  Color? get iconColor => _colors.secondary;

  @override
  Color? get backgroundColor => _colors.surfaceContainerHigh;

  @override
  Color? get shadowColor => Colors.transparent;

  @override
  Color? get surfaceTintColor => Colors.transparent;

  @override
  TextStyle? get titleTextStyle => _textTheme.headlineSmall;

  @override
  TextStyle? get contentTextStyle => _textTheme.bodyMedium;

  @override
  EdgeInsetsGeometry? get actionsPadding => const EdgeInsets.only(left: 24.0, right: 24.0, bottom: 24.0);
}
// dart format on

// END GENERATED TOKEN PROPERTIES - Dialog

// BEGIN GENERATED TOKEN PROPERTIES - DialogFullscreen

// Do not edit by hand. The code between the "BEGIN GENERATED" and
// "END GENERATED" comments are generated from data in the Material
// Design token database by the script:
//   dev/tools/gen_defaults/bin/gen_defaults.dart.

// dart format off
class _DialogFullscreenDefaultsM3 extends DialogThemeData {
  const _DialogFullscreenDefaultsM3(this.context): super(clipBehavior: Clip.none);

  final BuildContext context;

  @override
  Color? get backgroundColor => Theme.of(context).colorScheme.surface;
}
// dart format on

// END GENERATED TOKEN PROPERTIES - DialogFullscreen
