// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'bottom_navigation_bar.dart';
/// @docImport 'floating_action_button.dart';
library;

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

import 'color_scheme.dart';
import 'colors.dart';
import 'icon_button.dart';
import 'icons.dart';
import 'material.dart';
import 'material_localizations.dart';
import 'material_state.dart';
import 'scaffold.dart';
import 'snack_bar_theme.dart';
import 'text_button.dart';
import 'text_button_theme.dart';
import 'theme.dart';

// Examples can assume:
// late BuildContext context;

const double _singleLineVerticalPadding = 14.0;
const Duration _snackBarTransitionDuration = Duration(milliseconds: 250);
const Duration _snackBarDisplayDuration = Duration(milliseconds: 4000);
const Curve _snackBarHeightCurve = Curves.fastOutSlowIn;
const Curve _snackBarM3HeightCurve = Curves.easeInOutQuart;

const Curve _snackBarFadeInCurve = Interval(0.4, 1.0);
const Curve _snackBarM3FadeInCurve = Interval(0.4, 0.6, curve: Curves.easeInCirc);
const Curve _snackBarFadeOutCurve = Interval(0.72, 1.0, curve: Curves.fastOutSlowIn);

/// Specify how a [SnackBar] was closed.
///
/// The [ScaffoldMessengerState.showSnackBar] function returns a
/// [ScaffoldFeatureController]. The value of the controller's closed property
/// is a Future that resolves to a SnackBarClosedReason. Applications that need
/// to know how a snackbar was closed can use this value.
///
/// Example:
///
/// ```dart
/// ScaffoldMessenger.of(context).showSnackBar(
///   const SnackBar(
///     content: Text('He likes me. I think he likes me.'),
///   )
/// ).closed.then((SnackBarClosedReason reason) {
///    // ...
/// });
/// ```
enum SnackBarClosedReason {
  /// The snack bar was closed after the user tapped a [SnackBarAction].
  action,

  /// The snack bar was closed through a [SemanticsAction.dismiss].
  dismiss,

  /// The snack bar was closed by a user's swipe.
  swipe,

  /// The snack bar was closed by the [ScaffoldFeatureController] close callback
  /// or by calling [ScaffoldMessengerState.hideCurrentSnackBar] directly.
  hide,

  /// The snack bar was closed by an call to [ScaffoldMessengerState.removeCurrentSnackBar].
  remove,

  /// The snack bar was closed because its timer expired.
  timeout,
}

/// A button for a [SnackBar], known as an "action".
///
/// Snack bar actions are always enabled. Instead of disabling a snack bar
/// action, avoid including it in the snack bar in the first place.
///
/// Snack bar actions can only be pressed once. Subsequent presses are ignored.
///
/// See also:
///
///  * [SnackBar]
///  * <https://material.io/design/components/snackbars.html>
class SnackBarAction extends StatefulWidget {
  /// Creates an action for a [SnackBar].
  const SnackBarAction({
    super.key,
    this.textColor,
    this.disabledTextColor,
    this.backgroundColor,
    this.disabledBackgroundColor,
    required this.label,
    required this.onPressed,
  }) : assert(
         backgroundColor is! MaterialStateColor || disabledBackgroundColor == null,
         'disabledBackgroundColor must not be provided when background color is '
         'a MaterialStateColor',
       );

  /// The button label color. If not provided, defaults to
  /// [SnackBarThemeData.actionTextColor].
  ///
  /// If [textColor] is a [WidgetStateColor], then the text color will be
  /// resolved against the set of [WidgetState]s that the action text
  /// is in, thus allowing for different colors for states such as pressed,
  /// hovered and others.
  final Color? textColor;

  /// The button background fill color. If not provided, defaults to
  /// [SnackBarThemeData.actionBackgroundColor].
  ///
  /// If [backgroundColor] is a [WidgetStateColor], then the text color will
  /// be resolved against the set of [WidgetState]s that the action text is
  /// in, thus allowing for different colors for the states.
  final Color? backgroundColor;

  /// The button disabled label color. This color is shown after the
  /// [SnackBarAction] is dismissed.
  final Color? disabledTextColor;

  /// The button disabled background color. This color is shown after the
  /// [SnackBarAction] is dismissed.
  ///
  /// If not provided, defaults to [SnackBarThemeData.disabledActionBackgroundColor].
  final Color? disabledBackgroundColor;

  /// The button label.
  final String label;

  /// The callback to be called when the button is pressed.
  ///
  /// This callback will be called at most once each time this action is
  /// displayed in a [SnackBar].
  final VoidCallback onPressed;

  @override
  State<SnackBarAction> createState() => _SnackBarActionState();
}

class _SnackBarActionState extends State<SnackBarAction> {
  bool _haveTriggeredAction = false;

  void _handlePressed() {
    if (_haveTriggeredAction) {
      return;
    }
    setState(() {
      _haveTriggeredAction = true;
    });
    widget.onPressed();
    ScaffoldMessenger.of(context).hideCurrentSnackBar(reason: SnackBarClosedReason.action);
  }

  @override
  Widget build(BuildContext context) {
    final SnackBarThemeData defaults =
        Theme.of(context).useMaterial3
            ? _SnackbarDefaultsM3(context)
            : _SnackbarDefaultsM2(context);
    final SnackBarThemeData snackBarTheme = Theme.of(context).snackBarTheme;

    MaterialStateColor resolveForegroundColor() {
      if (widget.textColor != null) {
        if (widget.textColor is MaterialStateColor) {
          return widget.textColor! as MaterialStateColor;
        }
      } else if (snackBarTheme.actionTextColor != null) {
        if (snackBarTheme.actionTextColor is MaterialStateColor) {
          return snackBarTheme.actionTextColor! as MaterialStateColor;
        }
      } else if (defaults.actionTextColor != null) {
        if (defaults.actionTextColor is MaterialStateColor) {
          return defaults.actionTextColor! as MaterialStateColor;
        }
      }

      return MaterialStateColor.resolveWith((Set<MaterialState> states) {
        if (states.contains(MaterialState.disabled)) {
          return widget.disabledTextColor ??
              snackBarTheme.disabledActionTextColor ??
              defaults.disabledActionTextColor!;
        }
        return widget.textColor ?? snackBarTheme.actionTextColor ?? defaults.actionTextColor!;
      });
    }

    MaterialStateColor? resolveBackgroundColor() {
      if (widget.backgroundColor is MaterialStateColor) {
        return widget.backgroundColor! as MaterialStateColor;
      }
      if (snackBarTheme.actionBackgroundColor is MaterialStateColor) {
        return snackBarTheme.actionBackgroundColor! as MaterialStateColor;
      }
      return MaterialStateColor.resolveWith((Set<MaterialState> states) {
        if (states.contains(MaterialState.disabled)) {
          return widget.disabledBackgroundColor ??
              snackBarTheme.disabledActionBackgroundColor ??
              Colors.transparent;
        }
        return widget.backgroundColor ?? snackBarTheme.actionBackgroundColor ?? Colors.transparent;
      });
    }

    return TextButton(
      style: TextButton.styleFrom(overlayColor: resolveForegroundColor()).copyWith(
        foregroundColor: resolveForegroundColor(),
        backgroundColor: resolveBackgroundColor(),
      ),
      onPressed: _haveTriggeredAction ? null : _handlePressed,
      child: Text(widget.label),
    );
  }
}

/// A lightweight message with an optional action which briefly displays at the
/// bottom of the screen.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=zpO6n_oZWw0}
///
/// To display a snack bar, call `ScaffoldMessenger.of(context).showSnackBar()`,
/// passing an instance of [SnackBar] that describes the message.
///
/// To control how long the [SnackBar] remains visible, specify a [duration].
///
/// A SnackBar with an action will not time out when TalkBack or VoiceOver are
/// enabled. This is controlled by [AccessibilityFeatures.accessibleNavigation].
///
/// During page transitions, the [SnackBar] will smoothly animate to its
/// location on the other page. For example if the [SnackBar.behavior] is set to
/// [SnackBarBehavior.floating] and the next page has a floating action button,
/// while the current one does not, the [SnackBar] will smoothly animate above
/// the floating action button. It also works in the case of a back gesture
/// transition.
///
/// {@tool dartpad}
/// Here is an example of a [SnackBar] with an [action] button implemented using
/// [SnackBarAction].
///
/// ** See code in examples/api/lib/material/snack_bar/snack_bar.0.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// Here is an example of a customized [SnackBar]. It utilizes
/// [behavior], [shape], [padding], [width], and [duration] to customize the
/// location, appearance, and the duration for which the [SnackBar] is visible.
///
/// ** See code in examples/api/lib/material/snack_bar/snack_bar.1.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example demonstrates the various [SnackBar] widget components,
/// including an optional icon, in either floating or fixed format.
///
/// ** See code in examples/api/lib/material/snack_bar/snack_bar.2.dart **
/// {@end-tool}
///
/// See also:
///
///  * [ScaffoldMessenger.of], to obtain the current [ScaffoldMessengerState],
///    which manages the display and animation of snack bars.
///  * [ScaffoldMessengerState.showSnackBar], which displays a [SnackBar].
///  * [ScaffoldMessengerState.removeCurrentSnackBar], which abruptly hides the
///    currently displayed snack bar, if any, and allows the next to be displayed.
///  * [SnackBarAction], which is used to specify an [action] button to show
///    on the snack bar.
///  * [SnackBarThemeData], to configure the default property values for
///    [SnackBar] widgets.
///  * <https://material.io/design/components/snackbars.html>
class SnackBar extends StatefulWidget {
  /// Creates a snack bar.
  ///
  /// The [elevation] must be null or non-negative.
  const SnackBar({
    super.key,
    required this.content,
    this.backgroundColor,
    this.elevation,
    this.margin,
    this.padding,
    this.width,
    this.shape,
    this.hitTestBehavior,
    this.behavior,
    this.action,
    this.actionOverflowThreshold,
    this.showCloseIcon,
    this.closeIconColor,
    this.duration = _snackBarDisplayDuration,
    this.animation,
    this.onVisible,
    this.dismissDirection,
    this.clipBehavior = Clip.hardEdge,
  }) : assert(elevation == null || elevation >= 0.0),
       assert(width == null || margin == null, 'Width and margin can not be used together'),
       assert(
         actionOverflowThreshold == null ||
             (actionOverflowThreshold >= 0 && actionOverflowThreshold <= 1),
         'Action overflow threshold must be between 0 and 1 inclusive',
       );

  /// The primary content of the snack bar.
  ///
  /// Typically a [Text] widget.
  final Widget content;

  /// The snack bar's background color.
  ///
  /// If not specified, it will use [SnackBarThemeData.backgroundColor] of
  /// [ThemeData.snackBarTheme]. If that is not specified it will default to a
  /// dark variation of [ColorScheme.surface] for light themes, or
  /// [ColorScheme.onSurface] for dark themes.
  final Color? backgroundColor;

  /// The z-coordinate at which to place the snack bar. This controls the size
  /// of the shadow below the snack bar.
  ///
  /// Defines the card's [Material.elevation].
  ///
  /// If this property is null, then [SnackBarThemeData.elevation] of
  /// [ThemeData.snackBarTheme] is used, if that is also null, the default value
  /// is 6.0.
  final double? elevation;

  /// Empty space to surround the snack bar.
  ///
  /// This property is only used when [behavior] is [SnackBarBehavior.floating].
  /// It can not be used if [width] is specified.
  ///
  /// If this property is null, then [SnackBarThemeData.insetPadding] of
  /// [ThemeData.snackBarTheme] is used. If that is also null, then the default is
  /// `EdgeInsets.fromLTRB(15.0, 5.0, 15.0, 10.0)`.
  ///
  /// If this property is not null and [hitTestBehavior] is null, then [hitTestBehavior] default is [HitTestBehavior.deferToChild].
  final EdgeInsetsGeometry? margin;

  /// The amount of padding to apply to the snack bar's content and optional
  /// action.
  ///
  /// If this property is null, the default padding values are as follows:
  ///
  /// * [content]
  ///     * Top and bottom paddings are 14.
  ///     * Left padding is 24 if [behavior] is [SnackBarBehavior.fixed],
  ///       16 if [behavior] is [SnackBarBehavior.floating].
  ///     * Right padding is same as start padding if there is no [action],
  ///       otherwise 0.
  /// * [action]
  ///     * Top and bottom paddings are 14.
  ///     * Left and right paddings are half of [content]'s left padding.
  ///
  /// If this property is not null, the padding is as follows:
  ///
  /// * [content]
  ///     * Left, top and bottom paddings are assigned normally.
  ///     * Right padding is assigned normally if there is no [action],
  ///       otherwise 0.
  /// * [action]
  ///     * Left padding is replaced with half the right padding.
  ///     * Top and bottom paddings are assigned normally.
  ///     * Right padding is replaced with one and a half times the
  ///       right padding.
  final EdgeInsetsGeometry? padding;

  /// The width of the snack bar.
  ///
  /// If width is specified, the snack bar will be centered horizontally in the
  /// available space. This property is only used when [behavior] is
  /// [SnackBarBehavior.floating]. It can not be used if [margin] is specified.
  ///
  /// If this property is null, then [SnackBarThemeData.width] of
  /// [ThemeData.snackBarTheme] is used. If that is null, the snack bar will
  /// take up the full device width less the margin.
  final double? width;

  /// The shape of the snack bar's [Material].
  ///
  /// Defines the snack bar's [Material.shape].
  ///
  /// If this property is null then [SnackBarThemeData.shape] of
  /// [ThemeData.snackBarTheme] is used. If that's null then the shape will
  /// depend on the [SnackBarBehavior]. For [SnackBarBehavior.fixed], no
  /// overriding shape is specified, so the [SnackBar] is rectangular. For
  /// [SnackBarBehavior.floating], it uses a [RoundedRectangleBorder] with a
  /// circular corner radius of 4.0.
  final ShapeBorder? shape;

  /// Defines how the snack bar area, including margin, will behave during hit testing.
  ///
  /// If this property is null, and [margin] is not null or [SnackBarThemeData.insetPadding] of
  /// [ThemeData.snackBarTheme] is not null, then [HitTestBehavior.deferToChild] is used by default.
  ///
  /// Please refer to [HitTestBehavior] for a detailed explanation of every behavior.
  final HitTestBehavior? hitTestBehavior;

  /// This defines the behavior and location of the snack bar.
  ///
  /// Defines where a [SnackBar] should appear within a [Scaffold] and how its
  /// location should be adjusted when the scaffold also includes a
  /// [FloatingActionButton] or a [BottomNavigationBar]
  ///
  /// If this property is null, then [SnackBarThemeData.behavior] of
  /// [ThemeData.snackBarTheme] is used. If that is null, then the default is
  /// [SnackBarBehavior.fixed].
  ///
  /// If this value is [SnackBarBehavior.floating], the length of the bar
  /// is defined by either [width] or [margin].
  final SnackBarBehavior? behavior;

  /// (optional) An action that the user can take based on the snack bar.
  ///
  /// For example, the snack bar might let the user undo the operation that
  /// prompted the snackbar. Snack bars can have at most one action.
  ///
  /// The action should not be "dismiss" or "cancel".
  final SnackBarAction? action;

  /// (optional) The percentage threshold for action widget's width before it overflows
  /// to a new line.
  ///
  /// Must be between 0 and 1.
  /// If the width of the [action] divided by the total snackbar width
  /// is greater than this percentage, the [action] will appear below the [content].
  ///
  /// At a value of 0, the action will always overflow to a new line.
  ///
  /// Defaults to 0.25.
  final double? actionOverflowThreshold;

  /// (optional) Whether to include a "close" icon widget.
  ///
  /// Tapping the icon will close the snack bar.
  final bool? showCloseIcon;

  /// An optional color for the close icon, if [showCloseIcon] is
  /// true.
  ///
  /// If this property is null, then [SnackBarThemeData.closeIconColor] of
  /// [ThemeData.snackBarTheme] is used. If that is null, then the default is
  /// inverse surface.
  ///
  /// If [closeIconColor] is a [WidgetStateColor], then the icon color will be
  /// resolved against the set of [WidgetState]s that the action text
  /// is in, thus allowing for different colors for states such as pressed,
  /// hovered and others.
  final Color? closeIconColor;

  /// The amount of time the snack bar should be displayed.
  ///
  /// Defaults to 4.0s.
  ///
  /// See also:
  ///
  ///  * [ScaffoldMessengerState.removeCurrentSnackBar], which abruptly hides the
  ///    currently displayed snack bar, if any, and allows the next to be
  ///    displayed.
  ///  * <https://material.io/design/components/snackbars.html>
  final Duration duration;

  /// The animation driving the entrance and exit of the snack bar.
  final Animation<double>? animation;

  /// Called the first time that the snackbar is visible within a [Scaffold].
  ///
  /// When multiple [Scaffold]s are registered to the same [ScaffoldMessengerState],
  /// [onVisible] is called once for each scaffold.
  ///
  /// See also:
  ///
  ///  * [ScaffoldMessenger], which manages [SnackBar]s for [Scaffold] descendants.
  final VoidCallback? onVisible;

  /// The direction in which the SnackBar can be dismissed.
  ///
  /// If this property is null, then [SnackBarThemeData.dismissDirection] of
  /// [ThemeData.snackBarTheme] is used. If that is null, then the default is
  /// [DismissDirection.down].
  final DismissDirection? dismissDirection;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.hardEdge].
  final Clip clipBehavior;

  // API for ScaffoldMessengerState.showSnackBar():

  /// Creates an animation controller useful for driving a snack bar's entrance and exit animation.
  static AnimationController createAnimationController({
    required TickerProvider vsync,
    Duration? duration,
    Duration? reverseDuration,
  }) {
    return AnimationController(
      duration: duration ?? _snackBarTransitionDuration,
      reverseDuration: reverseDuration,
      debugLabel: 'SnackBar',
      vsync: vsync,
    );
  }

  /// Creates a copy of this snack bar but with the animation replaced with the given animation.
  ///
  /// If the original snack bar lacks a key, the newly created snack bar will
  /// use the given fallback key.
  SnackBar withAnimation(Animation<double> newAnimation, {Key? fallbackKey}) {
    return SnackBar(
      key: key ?? fallbackKey,
      content: content,
      backgroundColor: backgroundColor,
      elevation: elevation,
      margin: margin,
      padding: padding,
      width: width,
      shape: shape,
      hitTestBehavior: hitTestBehavior,
      behavior: behavior,
      action: action,
      actionOverflowThreshold: actionOverflowThreshold,
      showCloseIcon: showCloseIcon,
      closeIconColor: closeIconColor,
      duration: duration,
      animation: newAnimation,
      onVisible: onVisible,
      dismissDirection: dismissDirection,
      clipBehavior: clipBehavior,
    );
  }

  @override
  State<SnackBar> createState() => _SnackBarState();
}

class _SnackBarState extends State<SnackBar> {
  bool _wasVisible = false;

  CurvedAnimation? _heightAnimation;
  CurvedAnimation? _fadeInAnimation;
  CurvedAnimation? _fadeInM3Animation;
  CurvedAnimation? _fadeOutAnimation;
  CurvedAnimation? _heightM3Animation;

  @override
  void initState() {
    super.initState();
    widget.animation!.addStatusListener(_onAnimationStatusChanged);
    _setAnimations();
  }

  @override
  void didUpdateWidget(SnackBar oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.animation != oldWidget.animation) {
      oldWidget.animation!.removeStatusListener(_onAnimationStatusChanged);
      widget.animation!.addStatusListener(_onAnimationStatusChanged);
      _disposeAnimations();
      _setAnimations();
    }
  }

  void _setAnimations() {
    assert(widget.animation != null);
    _heightAnimation = CurvedAnimation(parent: widget.animation!, curve: _snackBarHeightCurve);
    _fadeInAnimation = CurvedAnimation(parent: widget.animation!, curve: _snackBarFadeInCurve);
    _fadeInM3Animation = CurvedAnimation(parent: widget.animation!, curve: _snackBarM3FadeInCurve);
    _fadeOutAnimation = CurvedAnimation(
      parent: widget.animation!,
      curve: _snackBarFadeOutCurve,
      reverseCurve: const Threshold(0.0),
    );
    // Material 3 Animation has a height animation on entry, but a direct fade out on exit.
    _heightM3Animation = CurvedAnimation(
      parent: widget.animation!,
      curve: _snackBarM3HeightCurve,
      reverseCurve: const Threshold(0.0),
    );
  }

  void _disposeAnimations() {
    _heightAnimation?.dispose();
    _fadeInAnimation?.dispose();
    _fadeInM3Animation?.dispose();
    _fadeOutAnimation?.dispose();
    _heightM3Animation?.dispose();
    _heightAnimation = null;
    _fadeInAnimation = null;
    _fadeInM3Animation = null;
    _fadeOutAnimation = null;
    _heightM3Animation = null;
  }

  @override
  void dispose() {
    widget.animation!.removeStatusListener(_onAnimationStatusChanged);
    _disposeAnimations();
    super.dispose();
  }

  void _onAnimationStatusChanged(AnimationStatus animationStatus) {
    if (animationStatus.isCompleted) {
      if (widget.onVisible != null && !_wasVisible) {
        widget.onVisible!();
      }
      _wasVisible = true;
    }
  }

  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasMediaQuery(context));
    final bool accessibleNavigation = MediaQuery.accessibleNavigationOf(context);
    assert(widget.animation != null);
    final ThemeData theme = Theme.of(context);
    final ColorScheme colorScheme = theme.colorScheme;
    final SnackBarThemeData snackBarTheme = theme.snackBarTheme;
    final bool isThemeDark = theme.brightness == Brightness.dark;
    final Color buttonColor = isThemeDark ? colorScheme.primary : colorScheme.secondary;
    final SnackBarThemeData defaults =
        theme.useMaterial3 ? _SnackbarDefaultsM3(context) : _SnackbarDefaultsM2(context);

    // SnackBar uses a theme that is the opposite brightness from
    // the surrounding theme.
    final Brightness brightness = isThemeDark ? Brightness.light : Brightness.dark;

    // Invert the theme values for Material 2. Material 3 values are tokenized to pre-inverted values.
    final ThemeData effectiveTheme =
        theme.useMaterial3
            ? theme
            : theme.copyWith(
              colorScheme: ColorScheme(
                primary: colorScheme.onPrimary,
                secondary: buttonColor,
                surface: colorScheme.onSurface,
                background: defaults.backgroundColor,
                error: colorScheme.onError,
                onPrimary: colorScheme.primary,
                onSecondary: colorScheme.secondary,
                onSurface: colorScheme.surface,
                onBackground: colorScheme.background,
                onError: colorScheme.error,
                brightness: brightness,
              ),
            );

    final TextStyle? contentTextStyle = snackBarTheme.contentTextStyle ?? defaults.contentTextStyle;
    final SnackBarBehavior snackBarBehavior =
        widget.behavior ?? snackBarTheme.behavior ?? defaults.behavior!;
    final double? width = widget.width ?? snackBarTheme.width;
    assert(() {
      // Whether the behavior is set through the constructor or the theme,
      // assert that other properties are configured properly.
      if (snackBarBehavior != SnackBarBehavior.floating) {
        String message(String parameter) {
          final String prefix = '$parameter can only be used with floating behavior.';
          if (widget.behavior != null) {
            return '$prefix SnackBarBehavior.fixed was set in the SnackBar constructor.';
          } else if (snackBarTheme.behavior != null) {
            return '$prefix SnackBarBehavior.fixed was set by the inherited SnackBarThemeData.';
          } else {
            return '$prefix SnackBarBehavior.fixed was set by default.';
          }
        }

        assert(widget.margin == null, message('Margin'));
        assert(width == null, message('Width'));
      }
      return true;
    }());

    final bool showCloseIcon =
        widget.showCloseIcon ?? snackBarTheme.showCloseIcon ?? defaults.showCloseIcon!;

    final bool isFloatingSnackBar = snackBarBehavior == SnackBarBehavior.floating;
    final double horizontalPadding = isFloatingSnackBar ? 16.0 : 24.0;
    final EdgeInsetsGeometry padding =
        widget.padding ??
        EdgeInsetsDirectional.only(
          start: horizontalPadding,
          end: widget.action != null || showCloseIcon ? 0 : horizontalPadding,
        );

    final double actionHorizontalMargin =
        (widget.padding?.resolve(TextDirection.ltr).right ?? horizontalPadding) / 2;
    final double iconHorizontalMargin =
        (widget.padding?.resolve(TextDirection.ltr).right ?? horizontalPadding) / 12.0;

    final IconButton? iconButton =
        showCloseIcon
            ? IconButton(
              key: StandardComponentType.closeButton.key,
              icon: const Icon(Icons.close),
              iconSize: 24.0,
              color:
                  widget.closeIconColor ?? snackBarTheme.closeIconColor ?? defaults.closeIconColor,
              onPressed:
                  () => ScaffoldMessenger.of(
                    context,
                  ).hideCurrentSnackBar(reason: SnackBarClosedReason.dismiss),
              tooltip: MaterialLocalizations.of(context).closeButtonTooltip,
            )
            : null;

    // Calculate combined width of Action, Icon, and their padding, if they are present.
    final TextPainter actionTextPainter = TextPainter(
      text: TextSpan(
        text: widget.action?.label ?? '',
        style: Theme.of(context).textTheme.labelLarge,
      ),
      maxLines: 1,
      textDirection: TextDirection.ltr,
    )..layout();
    final double actionAndIconWidth =
        actionTextPainter.size.width +
        (widget.action != null ? actionHorizontalMargin : 0) +
        (showCloseIcon ? (iconButton?.iconSize ?? 0 + iconHorizontalMargin) : 0);
    actionTextPainter.dispose();

    final EdgeInsets margin =
        widget.margin?.resolve(TextDirection.ltr) ??
        snackBarTheme.insetPadding ??
        defaults.insetPadding!;

    final double snackBarWidth =
        widget.width ?? MediaQuery.sizeOf(context).width - (margin.left + margin.right);
    final double actionOverflowThreshold =
        widget.actionOverflowThreshold ??
        snackBarTheme.actionOverflowThreshold ??
        defaults.actionOverflowThreshold!;

    final bool willOverflowAction = actionAndIconWidth / snackBarWidth > actionOverflowThreshold;

    final List<Widget> maybeActionAndIcon = <Widget>[
      if (widget.action != null)
        Padding(
          padding: EdgeInsets.symmetric(horizontal: actionHorizontalMargin),
          child: TextButtonTheme(
            data: TextButtonThemeData(
              style: TextButton.styleFrom(
                foregroundColor: buttonColor,
                padding: EdgeInsets.symmetric(horizontal: horizontalPadding),
              ),
            ),
            child: widget.action!,
          ),
        ),
      if (showCloseIcon)
        Padding(padding: EdgeInsets.symmetric(horizontal: iconHorizontalMargin), child: iconButton),
    ];

    Widget snackBar = Padding(
      padding: padding,
      child: Wrap(
        children: <Widget>[
          Row(
            children: <Widget>[
              Expanded(
                child: Padding(
                  padding:
                      widget.padding == null
                          ? const EdgeInsets.symmetric(vertical: _singleLineVerticalPadding)
                          : EdgeInsets.zero,
                  child: DefaultTextStyle(style: contentTextStyle!, child: widget.content),
                ),
              ),
              if (!willOverflowAction) ...maybeActionAndIcon,
              if (willOverflowAction) SizedBox(width: snackBarWidth * 0.4),
            ],
          ),
          if (willOverflowAction)
            Padding(
              padding: const EdgeInsets.only(bottom: _singleLineVerticalPadding),
              child: Row(mainAxisAlignment: MainAxisAlignment.end, children: maybeActionAndIcon),
            ),
        ],
      ),
    );

    if (!isFloatingSnackBar) {
      snackBar = SafeArea(top: false, child: snackBar);
    }

    final double elevation = widget.elevation ?? snackBarTheme.elevation ?? defaults.elevation!;
    final Color backgroundColor =
        widget.backgroundColor ?? snackBarTheme.backgroundColor ?? defaults.backgroundColor!;
    final ShapeBorder? shape =
        widget.shape ?? snackBarTheme.shape ?? (isFloatingSnackBar ? defaults.shape : null);
    final DismissDirection dismissDirection =
        widget.dismissDirection ?? snackBarTheme.dismissDirection ?? DismissDirection.down;

    snackBar = Material(
      shape: shape,
      elevation: elevation,
      color: backgroundColor,
      clipBehavior: widget.clipBehavior,
      child: Theme(
        data: effectiveTheme,
        child:
            accessibleNavigation || theme.useMaterial3
                ? snackBar
                : FadeTransition(opacity: _fadeOutAnimation!, child: snackBar),
      ),
    );

    if (isFloatingSnackBar) {
      // If width is provided, do not include horizontal margins.
      if (width != null) {
        snackBar = Padding(
          padding: EdgeInsets.only(top: margin.top, bottom: margin.bottom),
          child: SizedBox(width: width, child: snackBar),
        );
      } else {
        snackBar = Padding(padding: margin, child: snackBar);
      }
      snackBar = SafeArea(top: false, bottom: false, child: snackBar);
    }

    snackBar = Semantics(
      container: true,
      liveRegion: true,
      onDismiss: () {
        ScaffoldMessenger.of(context).removeCurrentSnackBar(reason: SnackBarClosedReason.dismiss);
      },
      child: Dismissible(
        key: const Key('dismissible'),
        direction: dismissDirection,
        resizeDuration: null,
        behavior:
            widget.hitTestBehavior ??
            (widget.margin != null || snackBarTheme.insetPadding != null
                ? HitTestBehavior.deferToChild
                : HitTestBehavior.opaque),
        onDismissed: (DismissDirection direction) {
          ScaffoldMessenger.of(context).removeCurrentSnackBar(reason: SnackBarClosedReason.swipe);
        },
        child: snackBar,
      ),
    );

    final Widget snackBarTransition;
    if (accessibleNavigation) {
      snackBarTransition = snackBar;
    } else if (isFloatingSnackBar && !theme.useMaterial3) {
      snackBarTransition = FadeTransition(opacity: _fadeInAnimation!, child: snackBar);
      // Is Material 3 Floating Snack Bar.
    } else if (isFloatingSnackBar && theme.useMaterial3) {
      snackBarTransition = FadeTransition(
        opacity: _fadeInM3Animation!,
        child: ValueListenableBuilder<double>(
          valueListenable: _heightM3Animation!,
          builder: (BuildContext context, double value, Widget? child) {
            return Align(alignment: Alignment.bottomLeft, heightFactor: value, child: child);
          },
          child: snackBar,
        ),
      );
    } else {
      snackBarTransition = ValueListenableBuilder<double>(
        valueListenable: _heightAnimation!,
        builder: (BuildContext context, double value, Widget? child) {
          return Align(alignment: AlignmentDirectional.topStart, heightFactor: value, child: child);
        },
        child: snackBar,
      );
    }

    return Hero(
      tag: '<SnackBar Hero tag - ${widget.content}>',
      transitionOnUserGestures: true,
      child: ClipRect(clipBehavior: widget.clipBehavior, child: snackBarTransition),
    );
  }
}

// Hand coded defaults based on Material Design 2.
class _SnackbarDefaultsM2 extends SnackBarThemeData {
  _SnackbarDefaultsM2(BuildContext context)
    : _theme = Theme.of(context),
      _colors = Theme.of(context).colorScheme,
      super(elevation: 6.0);

  late final ThemeData _theme;
  late final ColorScheme _colors;

  @override
  Color get backgroundColor =>
      _theme.brightness == Brightness.light
          ? Color.alphaBlend(_colors.onSurface.withOpacity(0.80), _colors.surface)
          : _colors.onSurface;

  @override
  TextStyle? get contentTextStyle =>
      ThemeData(
        useMaterial3: _theme.useMaterial3,
        brightness: _theme.brightness == Brightness.light ? Brightness.dark : Brightness.light,
      ).textTheme.titleMedium;

  @override
  SnackBarBehavior get behavior => SnackBarBehavior.fixed;

  @override
  Color get actionTextColor => _colors.secondary;

  @override
  Color get disabledActionTextColor =>
      _colors.onSurface.withOpacity(_theme.brightness == Brightness.light ? 0.38 : 0.3);

  @override
  ShapeBorder get shape =>
      const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(4.0)));

  @override
  EdgeInsets get insetPadding => const EdgeInsets.fromLTRB(15.0, 5.0, 15.0, 10.0);

  @override
  bool get showCloseIcon => false;

  @override
  Color get closeIconColor => _colors.onSurface;

  @override
  double get actionOverflowThreshold => 0.25;
}

// BEGIN GENERATED TOKEN PROPERTIES - Snackbar

// Do not edit by hand. The code between the "BEGIN GENERATED" and
// "END GENERATED" comments are generated from data in the Material
// Design token database by the script:
//   dev/tools/gen_defaults/bin/gen_defaults.dart.

// dart format off
class _SnackbarDefaultsM3 extends SnackBarThemeData {
    _SnackbarDefaultsM3(this.context);

  final BuildContext context;
  late final ThemeData _theme = Theme.of(context);
  late final ColorScheme _colors = _theme.colorScheme;

  @override
  Color get backgroundColor => _colors.inverseSurface;

  @override
  Color get actionTextColor =>  MaterialStateColor.resolveWith((Set<MaterialState> states) {
    if (states.contains(MaterialState.disabled)) {
      return _colors.inversePrimary;
    }
    if (states.contains(MaterialState.pressed)) {
      return _colors.inversePrimary;
    }
    if (states.contains(MaterialState.hovered)) {
      return _colors.inversePrimary;
    }
    if (states.contains(MaterialState.focused)) {
      return _colors.inversePrimary;
    }
    return _colors.inversePrimary;
  });

  @override
  Color get disabledActionTextColor =>
    _colors.inversePrimary;


  @override
  TextStyle get contentTextStyle =>
    Theme.of(context).textTheme.bodyMedium!.copyWith
      (color:  _colors.onInverseSurface,
    );

  @override
  double get elevation => 6.0;

  @override
  ShapeBorder get shape => const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(4.0)));

  @override
  SnackBarBehavior get behavior => SnackBarBehavior.fixed;

  @override
  EdgeInsets get insetPadding => const EdgeInsets.fromLTRB(15.0, 5.0, 15.0, 10.0);

  @override
  bool get showCloseIcon => false;

  @override
  Color? get closeIconColor => _colors.onInverseSurface;

  @override
  double get actionOverflowThreshold => 0.25;
}
// dart format on

// END GENERATED TOKEN PROPERTIES - Snackbar
