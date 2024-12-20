// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'nav_bar.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/semantics.dart';
import 'package:flutter/widgets.dart';

import 'colors.dart';
import 'constants.dart';
import 'text_theme.dart';
import 'theme.dart';

// Measured against iOS (17) [Human Interface Guidelines](https://developer.apple.com/design/human-interface-guidelines/buttons#iOS-iPadOS).

/// The size of a [CupertinoButton].
/// Based on the iOS (17) [Human Interface Guidelines](https://developer.apple.com/design/human-interface-guidelines/buttons#iOS-iPadOS).
enum CupertinoButtonSize {
  /// Displays a smaller button with round sides and smaller text (uses [CupertinoTextThemeData.actionSmallTextStyle]).
  small,

  /// Displays a medium sized button with round sides and regular-sized text.
  medium,

  /// Displays a (classic) large button with rounded edges and regular-sized text.
  large,
}

/// The style of a [CupertinoButton] that changes the style of the button's background.
///
/// Based on the iOS Human Interface Guidelines (https://developer.apple.com/design/human-interface-guidelines/buttons#iOS-iPadOS).
enum _CupertinoButtonStyle {
  /// No background or border, primary foreground color.
  plain,

  /// Translucent background, primary foreground color.
  tinted,

  /// Solid background, contrasting foreground color.
  filled,
}

/// An iOS-style button.
///
/// Takes in a text or an icon that fades out and in on touch. May optionally have a
/// background.
///
/// The [padding] defaults to 16.0 pixels. When using a [CupertinoButton] within
/// a fixed height parent, like a [CupertinoNavigationBar], a smaller, or even
/// [EdgeInsets.zero], should be used to prevent clipping larger [child]
/// widgets.
///
/// Preserves any parent [IconThemeData] but overwrites its [IconThemeData.color]
/// with the [CupertinoThemeData.primaryColor] (or
/// [CupertinoThemeData.primaryContrastingColor] if the button is disabled).
///
/// {@tool dartpad}
/// This sample shows produces an enabled and disabled [CupertinoButton] and
/// [CupertinoButton.filled].
///
/// ** See code in examples/api/lib/cupertino/button/cupertino_button.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * <https://developer.apple.com/design/human-interface-guidelines/buttons/>
class CupertinoButton extends StatefulWidget {
  /// Creates an iOS-style button.
  const CupertinoButton({
    super.key,
    required this.child,
    this.sizeStyle = CupertinoButtonSize.large,
    this.padding,
    this.color,
    this.disabledColor = CupertinoColors.quaternarySystemFill,
    this.minSize,
    this.pressedOpacity = 0.4,
    this.borderRadius,
    this.alignment = Alignment.center,
    this.focusColor,
    this.focusNode,
    this.onFocusChange,
    this.autofocus = false,
    this.onLongPress,
    required this.onPressed,
  }) : assert(pressedOpacity == null || (pressedOpacity >= 0.0 && pressedOpacity <= 1.0)),
       _style = _CupertinoButtonStyle.plain;

  /// Creates an iOS-style button with a tinted background.
  ///
  /// The background color is derived from the [CupertinoTheme]'s `primaryColor` + transparency.
  /// The foreground color is the [CupertinoTheme]'s `primaryColor`.
  ///
  /// To specify a custom background color, use the [color] argument of the
  /// default constructor.
  ///
  /// To match the iOS "grey" button style, set [color] to [CupertinoColors.systemGrey].
  const CupertinoButton.tinted({
    super.key,
    required this.child,
    this.sizeStyle = CupertinoButtonSize.large,
    this.padding,
    this.color,
    this.disabledColor = CupertinoColors.tertiarySystemFill,
    this.minSize,
    this.pressedOpacity = 0.4,
    this.borderRadius,
    this.alignment = Alignment.center,
    this.focusColor,
    this.focusNode,
    this.onFocusChange,
    this.autofocus = false,
    this.onLongPress,
    required this.onPressed,
  }) : _style = _CupertinoButtonStyle.tinted;

  /// Creates an iOS-style button with a filled background.
  ///
  /// The background color is derived from the [CupertinoTheme]'s `primaryColor`.
  ///
  /// To specify a custom background color, use the [color] argument of the
  /// default constructor.
  const CupertinoButton.filled({
    super.key,
    required this.child,
    this.sizeStyle = CupertinoButtonSize.large,
    this.padding,
    this.disabledColor = CupertinoColors.tertiarySystemFill,
    this.minSize,
    this.pressedOpacity = 0.4,
    this.borderRadius,
    this.alignment = Alignment.center,
    this.focusColor,
    this.focusNode,
    this.onFocusChange,
    this.autofocus = false,
    this.onLongPress,
    required this.onPressed,
  }) : assert(pressedOpacity == null || (pressedOpacity >= 0.0 && pressedOpacity <= 1.0)),
       color = null,
       _style = _CupertinoButtonStyle.filled;

  /// The widget below this widget in the tree.
  ///
  /// Typically a [Text] widget.
  final Widget child;

  /// The amount of space to surround the child inside the bounds of the button.
  ///
  /// Defaults to 16.0 pixels.
  final EdgeInsetsGeometry? padding;

  /// The color of the button's background.
  ///
  /// Defaults to null which produces a button with no background or border.
  ///
  /// Defaults to the [CupertinoTheme]'s `primaryColor` when the
  /// [CupertinoButton.filled] constructor is used.
  final Color? color;

  /// The color of the button's background when the button is disabled.
  ///
  /// Ignored if the [CupertinoButton] doesn't also have a [color].
  ///
  /// Defaults to [CupertinoColors.quaternarySystemFill] when [color] is
  /// specified.
  final Color disabledColor;

  /// The callback that is called when the button is tapped or otherwise activated.
  ///
  /// If [onPressed] and [onLongPress] callbacks are null, then the button will be disabled.
  final VoidCallback? onPressed;

  /// If [onPressed] and [onLongPress] callbacks are null, then the button will be disabled.
  final VoidCallback? onLongPress;

  /// Minimum size of the button.
  ///
  /// Defaults to kMinInteractiveDimensionCupertino which the iOS Human
  /// Interface Guidelines recommends as the minimum tappable area.
  final double? minSize;

  /// The opacity that the button will fade to when it is pressed.
  /// The button will have an opacity of 1.0 when it is not pressed.
  ///
  /// This defaults to 0.4. If null, opacity will not change on pressed if using
  /// your own custom effects is desired.
  final double? pressedOpacity;

  /// The radius of the button's corners when it has a background color.
  ///
  /// Defaults to [kCupertinoButtonSizeBorderRadius], based on [sizeStyle].
  final BorderRadius? borderRadius;

  /// The size of the button.
  ///
  /// Defaults to [CupertinoButtonSize.large].
  final CupertinoButtonSize sizeStyle;

  /// The alignment of the button's [child].
  ///
  /// Typically buttons are sized to be just big enough to contain the child and its
  /// [padding]. If the button's size is constrained to a fixed size, for example by
  /// enclosing it with a [SizedBox], this property defines how the child is aligned
  /// within the available space.
  ///
  /// Always defaults to [Alignment.center].
  final AlignmentGeometry alignment;

  /// The color to use for the focus highlight for keyboard interactions.
  ///
  /// Defaults to a slightly transparent [color]. If [color] is null, defaults
  /// to a slightly transparent [CupertinoColors.activeBlue]. Slightly
  /// transparent in this context means the color is used with an opacity of
  /// 0.80, a brightness of 0.69 and a saturation of 0.835.
  final Color? focusColor;

  /// {@macro flutter.widgets.Focus.focusNode}
  final FocusNode? focusNode;

  /// Handler called when the focus changes.
  ///
  /// Called with true if this widget's node gains focus, and false if it loses
  /// focus.
  final ValueChanged<bool>? onFocusChange;

  /// {@macro flutter.widgets.Focus.autofocus}
  final bool autofocus;

  final _CupertinoButtonStyle _style;

  /// Whether the button is enabled or disabled. Buttons are disabled by default. To
  /// enable a button, set [onPressed] or [onLongPress] to a non-null value.
  bool get enabled => onPressed != null || onLongPress != null;

  @override
  State<CupertinoButton> createState() => _CupertinoButtonState();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(FlagProperty('enabled', value: enabled, ifFalse: 'disabled'));
  }
}

class _CupertinoButtonState extends State<CupertinoButton> with SingleTickerProviderStateMixin {
  // Eyeballed values. Feel free to tweak.
  static const Duration kFadeOutDuration = Duration(milliseconds: 120);
  static const Duration kFadeInDuration = Duration(milliseconds: 180);
  final Tween<double> _opacityTween = Tween<double>(begin: 1.0);

  late AnimationController _animationController;
  late Animation<double> _opacityAnimation;

  late bool isFocused;

  @override
  void initState() {
    super.initState();
    isFocused = false;
    _animationController = AnimationController(
      duration: const Duration(milliseconds: 200),
      value: 0.0,
      vsync: this,
    );
    _opacityAnimation = _animationController
        .drive(CurveTween(curve: Curves.decelerate))
        .drive(_opacityTween);
    _setTween();
  }

  @override
  void didUpdateWidget(CupertinoButton old) {
    super.didUpdateWidget(old);
    _setTween();
  }

  void _setTween() {
    _opacityTween.end = widget.pressedOpacity ?? 1.0;
  }

  @override
  void dispose() {
    _animationController.dispose();
    super.dispose();
  }

  bool _buttonHeldDown = false;

  void _handleTapDown(TapDownDetails event) {
    if (!_buttonHeldDown) {
      _buttonHeldDown = true;
      _animate();
    }
  }

  void _handleTapUp(TapUpDetails event) {
    if (_buttonHeldDown) {
      _buttonHeldDown = false;
      _animate();
    }
  }

  void _handleTapCancel() {
    if (_buttonHeldDown) {
      _buttonHeldDown = false;
      _animate();
    }
  }

  void _handleTap([Intent? _]) {
    if (widget.onPressed != null) {
      widget.onPressed!();
      context.findRenderObject()!.sendSemanticsEvent(const TapSemanticEvent());
    }
  }

  void _animate() {
    if (_animationController.isAnimating) {
      return;
    }
    final bool wasHeldDown = _buttonHeldDown;
    final TickerFuture ticker =
        _buttonHeldDown
            ? _animationController.animateTo(
              1.0,
              duration: kFadeOutDuration,
              curve: Curves.easeInOutCubicEmphasized,
            )
            : _animationController.animateTo(
              0.0,
              duration: kFadeInDuration,
              curve: Curves.easeOutCubic,
            );
    ticker.then<void>((void value) {
      if (mounted && wasHeldDown != _buttonHeldDown) {
        _animate();
      }
    });
  }

  void _onShowFocusHighlight(bool showHighlight) {
    setState(() {
      isFocused = showHighlight;
    });
  }

  late final Map<Type, Action<Intent>> _actionMap = <Type, Action<Intent>>{
    ActivateIntent: CallbackAction<ActivateIntent>(onInvoke: _handleTap),
  };

  @override
  Widget build(BuildContext context) {
    final bool enabled = widget.enabled;
    final CupertinoThemeData themeData = CupertinoTheme.of(context);
    final Color primaryColor = themeData.primaryColor;
    final Color? backgroundColor = (widget.color == null
            ? widget._style != _CupertinoButtonStyle.plain
                ? primaryColor
                : null
            : CupertinoDynamicColor.maybeResolve(widget.color, context))
        ?.withOpacity(
          widget._style == _CupertinoButtonStyle.tinted
              ? CupertinoTheme.brightnessOf(context) == Brightness.light
                  ? kCupertinoButtonTintedOpacityLight
                  : kCupertinoButtonTintedOpacityDark
              : widget.color?.opacity ?? 1.0,
        );
    final Color foregroundColor =
        widget._style == _CupertinoButtonStyle.filled
            ? themeData.primaryContrastingColor
            : enabled
            ? primaryColor
            : CupertinoDynamicColor.resolve(CupertinoColors.tertiaryLabel, context);

    final Color effectiveFocusOutlineColor =
        widget.focusColor ??
        HSLColor.fromColor(
              (backgroundColor ?? CupertinoColors.activeBlue).withOpacity(
                kCupertinoFocusColorOpacity,
              ),
            )
            .withLightness(kCupertinoFocusColorBrightness)
            .withSaturation(kCupertinoFocusColorSaturation)
            .toColor();

    final TextStyle textStyle = (widget.sizeStyle == CupertinoButtonSize.small
            ? themeData.textTheme.actionSmallTextStyle
            : themeData.textTheme.actionTextStyle)
        .copyWith(color: foregroundColor);
    final IconThemeData iconTheme = IconTheme.of(context).copyWith(
      color: foregroundColor,
      size:
          textStyle.fontSize != null ? textStyle.fontSize! * 1.2 : kCupertinoButtonDefaultIconSize,
    );

    return MouseRegion(
      cursor: enabled && kIsWeb ? SystemMouseCursors.click : MouseCursor.defer,
      child: FocusableActionDetector(
        actions: _actionMap,
        focusNode: widget.focusNode,
        autofocus: widget.autofocus,
        onFocusChange: widget.onFocusChange,
        onShowFocusHighlight: _onShowFocusHighlight,
        enabled: enabled,
        child: GestureDetector(
          behavior: HitTestBehavior.opaque,
          onTapDown: enabled ? _handleTapDown : null,
          onTapUp: enabled ? _handleTapUp : null,
          onTapCancel: enabled ? _handleTapCancel : null,
          onTap: widget.onPressed,
          onLongPress: widget.onLongPress,
          child: Semantics(
            button: true,
            child: ConstrainedBox(
              constraints: BoxConstraints(
                minWidth:
                    widget.minSize ??
                    kCupertinoButtonMinSize[widget.sizeStyle] ??
                    kMinInteractiveDimensionCupertino,
                minHeight:
                    widget.minSize ??
                    kCupertinoButtonMinSize[widget.sizeStyle] ??
                    kMinInteractiveDimensionCupertino,
              ),
              child: FadeTransition(
                opacity: _opacityAnimation,
                child: DecoratedBox(
                  decoration: BoxDecoration(
                    border:
                        enabled && isFocused
                            ? Border.fromBorderSide(
                              BorderSide(
                                color: effectiveFocusOutlineColor,
                                width: 3.5,
                                strokeAlign: BorderSide.strokeAlignOutside,
                              ),
                            )
                            : null,
                    borderRadius:
                        widget.borderRadius ?? kCupertinoButtonSizeBorderRadius[widget.sizeStyle],
                    color:
                        backgroundColor != null && !enabled
                            ? CupertinoDynamicColor.resolve(widget.disabledColor, context)
                            : backgroundColor,
                  ),
                  child: Padding(
                    padding: widget.padding ?? kCupertinoButtonPadding[widget.sizeStyle]!,
                    child: Align(
                      alignment: widget.alignment,
                      widthFactor: 1.0,
                      heightFactor: 1.0,
                      child: DefaultTextStyle(
                        style: textStyle,
                        child: IconTheme(data: iconTheme, child: widget.child),
                      ),
                    ),
                  ),
                ),
              ),
            ),
          ),
        ),
      ),
    );
  }
}
