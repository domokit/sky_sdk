// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'button_style_button.dart';
/// @docImport 'ink_well.dart';
/// @docImport 'segmented_button.dart';
library;

import 'dart:math' as math;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

import 'button_style.dart';
import 'color_scheme.dart';
import 'constants.dart';
import 'ink_ripple.dart';
import 'material_state.dart';
import 'text_button.dart';
import 'theme.dart';
import 'theme_data.dart';
import 'toggle_buttons_theme.dart';

// Examples can assume:
// List<bool> isSelected = <bool>[];
// void setState(dynamic arg) { }

/// A set of toggle buttons.
///
/// The list of [children] are laid out along [direction]. The state of each button
/// is controlled by [isSelected], which is a list of bools that determine
/// if a button is in an unselected or selected state. They are both
/// correlated by their index in the list. The length of [isSelected] has to
/// match the length of the [children] list.
///
/// There is a Material 3 version of this component, [SegmentedButton],
/// that's preferred for applications that are configured for Material 3
/// (see [ThemeData.useMaterial3]).
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=kVEguaQWGAY}
///
/// ## Updating to [SegmentedButton]
///
/// There is a Material 3 version of this component, [SegmentedButton],
/// that's preferred for applications that are configured for Material 3
/// (see [ThemeData.useMaterial3]). The [SegmentedButton] widget's visuals
/// are a little bit different, see the Material 3 spec at
/// <https://m3.material.io/components/segmented-buttons/overview> for
/// more details. The [SegmentedButton] widget's API is also slightly different.
/// While the [ToggleButtons] widget can have list of widgets, the
/// [SegmentedButton] widget has a list of [ButtonSegment]s with
/// a type value. While the [ToggleButtons] uses a list of boolean values
/// to determine the selection state of each button, the [SegmentedButton]
/// uses a set of type values to determine the selection state of each segment.
/// The [SegmentedButton.style] is a [ButtonStyle] style field, which can be
/// used to customize the entire segmented button and the individual segments.
///
/// {@tool dartpad}
/// This sample shows how to migrate [ToggleButtons] that allows multiple
/// or no selection to [SegmentedButton] that allows multiple or no selection.
///
/// ** See code in examples/api/lib/material/toggle_buttons/toggle_buttons.1.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example showcase [ToggleButtons] in various configurations.
///
/// ** See code in examples/api/lib/material/toggle_buttons/toggle_buttons.0.dart **
/// {@end-tool}
///
/// ## Customizing toggle buttons
/// Each toggle's behavior can be configured by the [onPressed] callback, which
/// can update the [isSelected] list however it wants to.
///
/// {@animation 700 150 https://flutter.github.io/assets-for-api-docs/assets/material/toggle_buttons_simple.mp4}
///
/// Here is an implementation that allows for multiple buttons to be
/// simultaneously selected, while requiring none of the buttons to be
/// selected.
///
/// ```dart
/// ToggleButtons(
///   isSelected: isSelected,
///   onPressed: (int index) {
///     setState(() {
///       isSelected[index] = !isSelected[index];
///     });
///   },
///   children: const <Widget>[
///     Icon(Icons.ac_unit),
///     Icon(Icons.call),
///     Icon(Icons.cake),
///   ],
/// ),
/// ```
///
/// {@animation 700 150 https://flutter.github.io/assets-for-api-docs/assets/material/toggle_buttons_required_mutually_exclusive.mp4}
///
/// Here is an implementation that requires mutually exclusive selection while
/// requiring at least one selection. This assumes that [isSelected] was
/// properly initialized with one selection.
///
/// ```dart
/// ToggleButtons(
///   isSelected: isSelected,
///   onPressed: (int index) {
///     setState(() {
///       for (int buttonIndex = 0; buttonIndex < isSelected.length; buttonIndex++) {
///         if (buttonIndex == index) {
///           isSelected[buttonIndex] = true;
///         } else {
///           isSelected[buttonIndex] = false;
///         }
///       }
///     });
///   },
///   children: const <Widget>[
///     Icon(Icons.ac_unit),
///     Icon(Icons.call),
///     Icon(Icons.cake),
///   ],
/// ),
/// ```
///
/// {@animation 700 150 https://flutter.github.io/assets-for-api-docs/assets/material/toggle_buttons_mutually_exclusive.mp4}
///
/// Here is an implementation that requires mutually exclusive selection,
/// but allows for none of the buttons to be selected.
///
/// ```dart
/// ToggleButtons(
///   isSelected: isSelected,
///   onPressed: (int index) {
///     setState(() {
///       for (int buttonIndex = 0; buttonIndex < isSelected.length; buttonIndex++) {
///         if (buttonIndex == index) {
///           isSelected[buttonIndex] = !isSelected[buttonIndex];
///         } else {
///           isSelected[buttonIndex] = false;
///         }
///       }
///     });
///   },
///   children: const <Widget>[
///     Icon(Icons.ac_unit),
///     Icon(Icons.call),
///     Icon(Icons.cake),
///   ],
/// ),
/// ```
///
/// {@animation 700 150 https://flutter.github.io/assets-for-api-docs/assets/material/toggle_buttons_required.mp4}
///
/// Here is an implementation that allows for multiple buttons to be
/// simultaneously selected, while requiring at least one selection. This
/// assumes that [isSelected] was properly initialized with one selection.
///
/// ```dart
/// ToggleButtons(
///   isSelected: isSelected,
///   onPressed: (int index) {
///     int count = 0;
///     for (final bool value in isSelected) {
///       if (value) {
///         count += 1;
///       }
///     }
///     if (isSelected[index] && count < 2) {
///       return;
///     }
///     setState(() {
///       isSelected[index] = !isSelected[index];
///     });
///   },
///   children: const <Widget>[
///     Icon(Icons.ac_unit),
///     Icon(Icons.call),
///     Icon(Icons.cake),
///   ],
/// ),
/// ```
///
/// ## ToggleButton Borders
/// The toggle buttons, by default, have a solid, 1 logical pixel border
/// surrounding itself and separating each button. The toggle button borders'
/// color, width, and corner radii are configurable.
///
/// The [selectedBorderColor] determines the border's color when the button is
/// selected, while [disabledBorderColor] determines the border's color when
/// the button is disabled. [borderColor] is used when the button is enabled.
///
/// To remove the border, set [renderBorder] to false. Setting [borderWidth] to
/// 0.0 results in a hairline border. For more information on hairline borders,
/// see [BorderSide.width].
///
/// See also:
///
///  * <https://material.io/design/components/buttons.html#toggle-button>
class ToggleButtons extends StatelessWidget {
  /// Creates a set of toggle buttons.
  ///
  /// It displays its widgets provided in a [List] of [children] along [direction].
  /// The state of each button is controlled by [isSelected], which is a list
  /// of bools that determine if a button is in an active, disabled, or
  /// selected state. They are both correlated by their index in the list.
  /// The length of [isSelected] has to match the length of the [children]
  /// list.
  ///
  /// Both [children] and [isSelected] properties arguments are required.
  ///
  /// The [focusNodes] argument must be null or a list of nodes. If [direction]
  /// is [Axis.vertical], [verticalDirection] must not be null.
  const ToggleButtons({
    super.key,
    required this.children,
    required this.isSelected,
    this.onPressed,
    this.mouseCursor,
    this.tapTargetSize,
    this.textStyle,
    this.constraints,
    this.color,
    this.selectedColor,
    this.disabledColor,
    this.fillColor,
    this.focusColor,
    this.highlightColor,
    this.hoverColor,
    this.splashColor,
    this.focusNodes,
    this.renderBorder = true,
    this.borderColor,
    this.selectedBorderColor,
    this.disabledBorderColor,
    this.borderRadius,
    this.borderWidth,
    this.direction = Axis.horizontal,
    this.verticalDirection = VerticalDirection.down,
  }) : assert(children.length == isSelected.length);

  static const double _defaultBorderWidth = 1.0;

  /// The toggle button widgets.
  ///
  /// These are typically [Icon] or [Text] widgets. The boolean selection
  /// state of each widget is defined by the corresponding [isSelected]
  /// list item.
  ///
  /// The length of children has to match the length of [isSelected]. If
  /// [focusNodes] is not null, the length of children has to also match
  /// the length of [focusNodes].
  final List<Widget> children;

  /// The corresponding selection state of each toggle button.
  ///
  /// Each value in this list represents the selection state of the [children]
  /// widget at the same index.
  ///
  /// The length of [isSelected] has to match the length of [children].
  final List<bool> isSelected;

  /// The callback that is called when a button is tapped.
  ///
  /// The index parameter of the callback is the index of the button that is
  /// tapped or otherwise activated.
  ///
  /// When the callback is null, all toggle buttons will be disabled.
  final void Function(int index)? onPressed;

  /// {@macro flutter.material.RawMaterialButton.mouseCursor}
  ///
  /// If this property is null, [WidgetStateMouseCursor.clickable] will be used.
  final MouseCursor? mouseCursor;

  /// Configures the minimum size of the area within which the buttons may
  /// be pressed.
  ///
  /// If the [tapTargetSize] is larger than [constraints], the buttons will
  /// include a transparent margin that responds to taps.
  ///
  /// Defaults to [ThemeData.materialTapTargetSize].
  final MaterialTapTargetSize? tapTargetSize;

  /// The [TextStyle] to apply to any text in these toggle buttons.
  ///
  /// [TextStyle.color] will be ignored and substituted by [color],
  /// [selectedColor] or [disabledColor] depending on whether the buttons
  /// are active, selected, or disabled.
  final TextStyle? textStyle;

  /// Defines the button's size.
  ///
  /// Typically used to constrain the button's minimum size.
  ///
  /// If this property is null, then
  /// BoxConstraints(minWidth: 48.0, minHeight: 48.0) is be used.
  final BoxConstraints? constraints;

  /// The color for descendant [Text] and [Icon] widgets if the button is
  /// enabled and not selected.
  ///
  /// If [onPressed] is not null, this color will be used for values in
  /// [isSelected] that are false.
  ///
  /// If this property is null, then ToggleButtonTheme.of(context).color
  /// is used. If [ToggleButtonsThemeData.color] is also null, then
  /// Theme.of(context).colorScheme.onSurface is used.
  final Color? color;

  /// The color for descendant [Text] and [Icon] widgets if the button is
  /// selected.
  ///
  /// If [onPressed] is not null, this color will be used for values in
  /// [isSelected] that are true.
  ///
  /// If this property is null, then
  /// ToggleButtonTheme.of(context).selectedColor is used. If
  /// [ToggleButtonsThemeData.selectedColor] is also null, then
  /// Theme.of(context).colorScheme.primary is used.
  final Color? selectedColor;

  /// The color for descendant [Text] and [Icon] widgets if the button is
  /// disabled.
  ///
  /// If [onPressed] is null, this color will be used.
  ///
  /// If this property is null, then
  /// ToggleButtonTheme.of(context).disabledColor is used. If
  /// [ToggleButtonsThemeData.disabledColor] is also null, then
  /// Theme.of(context).colorScheme.onSurface.withOpacity(0.38) is used.
  final Color? disabledColor;

  /// The fill color for selected toggle buttons.
  ///
  /// If this property is null, then
  /// ToggleButtonTheme.of(context).fillColor is used. If
  /// [ToggleButtonsThemeData.fillColor] is also null, then
  /// the fill color is null.
  ///
  /// If fillColor is a [WidgetStateProperty<Color>], then [WidgetStateProperty.resolve]
  /// is used for the following [WidgetState]s:
  ///
  ///  * [WidgetState.disabled]
  ///  * [WidgetState.selected]
  ///
  final Color? fillColor;

  /// The color to use for filling the button when the button has input focus.
  ///
  /// If this property is null, then
  /// ToggleButtonTheme.of(context).focusColor is used. If
  /// [ToggleButtonsThemeData.focusColor] is also null, then
  /// Theme.of(context).focusColor is used.
  final Color? focusColor;

  /// The highlight color for the button's [InkWell].
  ///
  /// If this property is null, then
  /// ToggleButtonTheme.of(context).highlightColor is used. If
  /// [ToggleButtonsThemeData.highlightColor] is also null, then
  /// Theme.of(context).highlightColor is used.
  final Color? highlightColor;

  /// The splash color for the button's [InkWell].
  ///
  /// If this property is null, then
  /// ToggleButtonTheme.of(context).splashColor is used. If
  /// [ToggleButtonsThemeData.splashColor] is also null, then
  /// Theme.of(context).splashColor is used.
  final Color? splashColor;

  /// The color to use for filling the button when the button has a pointer
  /// hovering over it.
  ///
  /// If this property is null, then
  /// ToggleButtonTheme.of(context).hoverColor is used. If
  /// [ToggleButtonsThemeData.hoverColor] is also null, then
  /// Theme.of(context).hoverColor is used.
  final Color? hoverColor;

  /// The list of [FocusNode]s, corresponding to each toggle button.
  ///
  /// Focus is used to determine which widget should be affected by keyboard
  /// events. The focus tree keeps track of which widget is currently focused
  /// on by the user.
  ///
  /// If not null, the length of focusNodes has to match the length of
  /// [children].
  ///
  /// See [FocusNode] for more information about how focus nodes are used.
  final List<FocusNode>? focusNodes;

  /// Whether or not to render a border around each toggle button.
  ///
  /// When true, a border with [borderWidth], [borderRadius] and the
  /// appropriate border color will render. Otherwise, no border will be
  /// rendered.
  final bool renderBorder;

  /// The border color to display when the toggle button is enabled and not
  /// selected.
  ///
  /// If this property is null, then
  /// ToggleButtonTheme.of(context).borderColor is used. If
  /// [ToggleButtonsThemeData.borderColor] is also null, then
  /// Theme.of(context).colorScheme.onSurface is used.
  final Color? borderColor;

  /// The border color to display when the toggle button is selected.
  ///
  /// If this property is null, then
  /// ToggleButtonTheme.of(context).selectedBorderColor is used. If
  /// [ToggleButtonsThemeData.selectedBorderColor] is also null, then
  /// Theme.of(context).colorScheme.primary is used.
  final Color? selectedBorderColor;

  /// The border color to display when the toggle button is disabled.
  ///
  /// If this property is null, then
  /// ToggleButtonTheme.of(context).disabledBorderColor is used. If
  /// [ToggleButtonsThemeData.disabledBorderColor] is also null, then
  /// Theme.of(context).disabledBorderColor is used.
  final Color? disabledBorderColor;

  /// The width of the border surrounding each toggle button.
  ///
  /// This applies to both the greater surrounding border, as well as the
  /// borders rendered between toggle buttons.
  ///
  /// To render a hairline border (one physical pixel), set borderWidth to 0.0.
  /// See [BorderSide.width] for more details on hairline borders.
  ///
  /// To omit the border entirely, set [renderBorder] to false.
  ///
  /// If this property is null, then
  /// ToggleButtonTheme.of(context).borderWidth is used. If
  /// [ToggleButtonsThemeData.borderWidth] is also null, then
  /// a width of 1.0 is used.
  final double? borderWidth;

  /// The radii of the border's corners.
  ///
  /// If this property is null, then
  /// ToggleButtonTheme.of(context).borderRadius is used. If
  /// [ToggleButtonsThemeData.borderRadius] is also null, then
  /// the buttons default to non-rounded borders.
  final BorderRadius? borderRadius;

  /// The direction along which the buttons are rendered.
  ///
  /// Defaults to [Axis.horizontal].
  final Axis direction;

  /// If [direction] is [Axis.vertical], this parameter determines whether to lay out
  /// the buttons starting from the first or last child from top to bottom.
  final VerticalDirection verticalDirection;

  // Determines if this is the first child that is being laid out
  // by the render object, _not_ the order of the children in its list.
  bool _isFirstButton(int index, int length, TextDirection textDirection) {
    switch (direction) {
      case Axis.horizontal:
        return switch (textDirection) {
          TextDirection.rtl => index == length - 1,
          TextDirection.ltr => index == 0,
        };
      case Axis.vertical:
        return switch (verticalDirection) {
          VerticalDirection.up => index == length - 1,
          VerticalDirection.down => index == 0,
        };
    }
  }

  // Determines if this is the last child that is being laid out
  // by the render object, _not_ the order of the children in its list.
  bool _isLastButton(int index, int length, TextDirection textDirection) {
    switch (direction) {
      case Axis.horizontal:
        return switch (textDirection) {
          TextDirection.rtl => index == 0,
          TextDirection.ltr => index == length - 1,
        };
      case Axis.vertical:
        return switch (verticalDirection) {
          VerticalDirection.up => index == 0,
          VerticalDirection.down => index == length - 1,
        };
    }
  }

  BorderRadius _getEdgeBorderRadius(
    int index,
    int length,
    TextDirection textDirection,
    ToggleButtonsThemeData toggleButtonsTheme,
  ) {
    final BorderRadius resultingBorderRadius =
        borderRadius ?? toggleButtonsTheme.borderRadius ?? BorderRadius.zero;

    if (length == 1) {
      return resultingBorderRadius;
    } else if (direction == Axis.horizontal) {
      if (_isFirstButton(index, length, textDirection)) {
        return BorderRadius.only(
          topLeft: resultingBorderRadius.topLeft,
          bottomLeft: resultingBorderRadius.bottomLeft,
        );
      } else if (_isLastButton(index, length, textDirection)) {
        return BorderRadius.only(
          topRight: resultingBorderRadius.topRight,
          bottomRight: resultingBorderRadius.bottomRight,
        );
      }
    } else {
      if (_isFirstButton(index, length, textDirection)) {
        return BorderRadius.only(
          topLeft: resultingBorderRadius.topLeft,
          topRight: resultingBorderRadius.topRight,
        );
      } else if (_isLastButton(index, length, textDirection)) {
        return BorderRadius.only(
          bottomLeft: resultingBorderRadius.bottomLeft,
          bottomRight: resultingBorderRadius.bottomRight,
        );
      }
    }

    return BorderRadius.zero;
  }

  BorderRadius _getClipBorderRadius(
    int index,
    int length,
    TextDirection textDirection,
    ToggleButtonsThemeData toggleButtonsTheme,
  ) {
    final BorderRadius resultingBorderRadius =
        borderRadius ?? toggleButtonsTheme.borderRadius ?? BorderRadius.zero;
    final double resultingBorderWidth =
        borderWidth ?? toggleButtonsTheme.borderWidth ?? _defaultBorderWidth;

    if (length == 1) {
      return BorderRadius.only(
        topLeft: resultingBorderRadius.topLeft - Radius.circular(resultingBorderWidth / 2.0),
        bottomLeft: resultingBorderRadius.bottomLeft - Radius.circular(resultingBorderWidth / 2.0),
        topRight: resultingBorderRadius.topRight - Radius.circular(resultingBorderWidth / 2.0),
        bottomRight:
            resultingBorderRadius.bottomRight - Radius.circular(resultingBorderWidth / 2.0),
      );
    } else if (direction == Axis.horizontal) {
      if (_isFirstButton(index, length, textDirection)) {
        return BorderRadius.only(
          topLeft: resultingBorderRadius.topLeft - Radius.circular(resultingBorderWidth / 2.0),
          bottomLeft:
              resultingBorderRadius.bottomLeft - Radius.circular(resultingBorderWidth / 2.0),
        );
      } else if (_isLastButton(index, length, textDirection)) {
        return BorderRadius.only(
          topRight: resultingBorderRadius.topRight - Radius.circular(resultingBorderWidth / 2.0),
          bottomRight:
              resultingBorderRadius.bottomRight - Radius.circular(resultingBorderWidth / 2.0),
        );
      }
    } else {
      if (_isFirstButton(index, length, textDirection)) {
        return BorderRadius.only(
          topLeft: resultingBorderRadius.topLeft - Radius.circular(resultingBorderWidth / 2.0),
          topRight: resultingBorderRadius.topRight - Radius.circular(resultingBorderWidth / 2.0),
        );
      } else if (_isLastButton(index, length, textDirection)) {
        return BorderRadius.only(
          bottomLeft:
              resultingBorderRadius.bottomLeft - Radius.circular(resultingBorderWidth / 2.0),
          bottomRight:
              resultingBorderRadius.bottomRight - Radius.circular(resultingBorderWidth / 2.0),
        );
      }
    }
    return BorderRadius.zero;
  }

  BorderSide _getLeadingBorderSide(
    int index,
    ThemeData theme,
    ToggleButtonsThemeData toggleButtonsTheme,
  ) {
    if (!renderBorder) {
      return BorderSide.none;
    }

    final double resultingBorderWidth =
        borderWidth ?? toggleButtonsTheme.borderWidth ?? _defaultBorderWidth;
    if (onPressed != null && (isSelected[index] || (index != 0 && isSelected[index - 1]))) {
      return BorderSide(
        color:
            selectedBorderColor ??
            toggleButtonsTheme.selectedBorderColor ??
            theme.colorScheme.onSurface.withOpacity(0.12),
        width: resultingBorderWidth,
      );
    } else if (onPressed != null && !isSelected[index]) {
      return BorderSide(
        color:
            borderColor ??
            toggleButtonsTheme.borderColor ??
            theme.colorScheme.onSurface.withOpacity(0.12),
        width: resultingBorderWidth,
      );
    } else {
      return BorderSide(
        color:
            disabledBorderColor ??
            toggleButtonsTheme.disabledBorderColor ??
            theme.colorScheme.onSurface.withOpacity(0.12),
        width: resultingBorderWidth,
      );
    }
  }

  BorderSide _getBorderSide(int index, ThemeData theme, ToggleButtonsThemeData toggleButtonsTheme) {
    if (!renderBorder) {
      return BorderSide.none;
    }

    final double resultingBorderWidth =
        borderWidth ?? toggleButtonsTheme.borderWidth ?? _defaultBorderWidth;
    if (onPressed != null && isSelected[index]) {
      return BorderSide(
        color:
            selectedBorderColor ??
            toggleButtonsTheme.selectedBorderColor ??
            theme.colorScheme.onSurface.withOpacity(0.12),
        width: resultingBorderWidth,
      );
    } else if (onPressed != null && !isSelected[index]) {
      return BorderSide(
        color:
            borderColor ??
            toggleButtonsTheme.borderColor ??
            theme.colorScheme.onSurface.withOpacity(0.12),
        width: resultingBorderWidth,
      );
    } else {
      return BorderSide(
        color:
            disabledBorderColor ??
            toggleButtonsTheme.disabledBorderColor ??
            theme.colorScheme.onSurface.withOpacity(0.12),
        width: resultingBorderWidth,
      );
    }
  }

  BorderSide _getTrailingBorderSide(
    int index,
    ThemeData theme,
    ToggleButtonsThemeData toggleButtonsTheme,
  ) {
    if (!renderBorder) {
      return BorderSide.none;
    }

    if (index != children.length - 1) {
      return BorderSide.none;
    }

    final double resultingBorderWidth =
        borderWidth ?? toggleButtonsTheme.borderWidth ?? _defaultBorderWidth;
    if (onPressed != null && (isSelected[index])) {
      return BorderSide(
        color:
            selectedBorderColor ??
            toggleButtonsTheme.selectedBorderColor ??
            theme.colorScheme.onSurface.withOpacity(0.12),
        width: resultingBorderWidth,
      );
    } else if (onPressed != null && !isSelected[index]) {
      return BorderSide(
        color:
            borderColor ??
            toggleButtonsTheme.borderColor ??
            theme.colorScheme.onSurface.withOpacity(0.12),
        width: resultingBorderWidth,
      );
    } else {
      return BorderSide(
        color:
            disabledBorderColor ??
            toggleButtonsTheme.disabledBorderColor ??
            theme.colorScheme.onSurface.withOpacity(0.12),
        width: resultingBorderWidth,
      );
    }
  }

  @override
  Widget build(BuildContext context) {
    assert(
      () {
        if (focusNodes != null) {
          return focusNodes!.length == children.length;
        }
        return true;
      }(),
      'focusNodes.length must match children.length.\n'
      'There are ${focusNodes!.length} focus nodes, while '
      'there are ${children.length} children.',
    );
    final ThemeData theme = Theme.of(context);
    final ToggleButtonsThemeData toggleButtonsTheme = ToggleButtonsTheme.of(context);
    final TextDirection textDirection = Directionality.of(context);

    final List<Widget> buttons = List<Widget>.generate(children.length, (int index) {
      final BorderRadius edgeBorderRadius = _getEdgeBorderRadius(
        index,
        children.length,
        textDirection,
        toggleButtonsTheme,
      );
      final BorderRadius clipBorderRadius = _getClipBorderRadius(
        index,
        children.length,
        textDirection,
        toggleButtonsTheme,
      );

      final BorderSide leadingBorderSide = _getLeadingBorderSide(index, theme, toggleButtonsTheme);
      final BorderSide borderSide = _getBorderSide(index, theme, toggleButtonsTheme);
      final BorderSide trailingBorderSide = _getTrailingBorderSide(
        index,
        theme,
        toggleButtonsTheme,
      );

      final Set<MaterialState> states = <MaterialState>{
        if (isSelected[index] && onPressed != null) MaterialState.selected,
        if (onPressed == null) MaterialState.disabled,
      };
      final Color effectiveFillColor =
          _ResolveFillColor(fillColor ?? toggleButtonsTheme.fillColor).resolve(states) ??
          _DefaultFillColor(theme.colorScheme).resolve(states);
      final Color currentColor;
      if (onPressed != null && isSelected[index]) {
        currentColor =
            selectedColor ?? toggleButtonsTheme.selectedColor ?? theme.colorScheme.primary;
      } else if (onPressed != null && !isSelected[index]) {
        currentColor =
            color ?? toggleButtonsTheme.color ?? theme.colorScheme.onSurface.withOpacity(0.87);
      } else {
        currentColor =
            disabledColor ??
            toggleButtonsTheme.disabledColor ??
            theme.colorScheme.onSurface.withOpacity(0.38);
      }
      final TextStyle currentTextStyle =
          textStyle ?? toggleButtonsTheme.textStyle ?? theme.textTheme.bodyMedium!;
      final BoxConstraints? currentConstraints = constraints ?? toggleButtonsTheme.constraints;
      final Size minimumSize =
          currentConstraints?.smallest ?? const Size.square(kMinInteractiveDimension);
      final Size? maximumSize = currentConstraints?.biggest;
      final Size minPaddingSize;
      switch (tapTargetSize ?? theme.materialTapTargetSize) {
        case MaterialTapTargetSize.padded:
          minPaddingSize = switch (direction) {
            Axis.horizontal => const Size(0.0, kMinInteractiveDimension),
            Axis.vertical => const Size(kMinInteractiveDimension, 0.0),
          };
          assert(minPaddingSize.width >= 0.0);
          assert(minPaddingSize.height >= 0.0);
        case MaterialTapTargetSize.shrinkWrap:
          minPaddingSize = Size.zero;
      }

      Widget button = _SelectToggleButton(
        leadingBorderSide: leadingBorderSide,
        borderSide: borderSide,
        trailingBorderSide: trailingBorderSide,
        borderRadius: edgeBorderRadius,
        isFirstButton: index == 0,
        isLastButton: index == children.length - 1,
        direction: direction,
        verticalDirection: verticalDirection,
        child: ClipRRect(
          borderRadius: clipBorderRadius,
          child: TextButton(
            focusNode: focusNodes != null ? focusNodes![index] : null,
            style: ButtonStyle(
              backgroundColor: MaterialStatePropertyAll<Color?>(effectiveFillColor),
              foregroundColor: MaterialStatePropertyAll<Color?>(currentColor),
              iconSize: const MaterialStatePropertyAll<double>(24.0),
              iconColor: MaterialStatePropertyAll<Color?>(currentColor),
              overlayColor: _ToggleButtonDefaultOverlay(
                selected: onPressed != null && isSelected[index],
                unselected: onPressed != null && !isSelected[index],
                colorScheme: theme.colorScheme,
                disabledColor: disabledColor ?? toggleButtonsTheme.disabledColor,
                focusColor: focusColor ?? toggleButtonsTheme.focusColor,
                highlightColor: highlightColor ?? toggleButtonsTheme.highlightColor,
                hoverColor: hoverColor ?? toggleButtonsTheme.hoverColor,
                splashColor: splashColor ?? toggleButtonsTheme.splashColor,
              ),
              elevation: const MaterialStatePropertyAll<double>(0),
              textStyle: MaterialStatePropertyAll<TextStyle?>(
                currentTextStyle.copyWith(color: currentColor),
              ),
              padding: const MaterialStatePropertyAll<EdgeInsetsGeometry>(EdgeInsets.zero),
              minimumSize: MaterialStatePropertyAll<Size?>(minimumSize),
              maximumSize: MaterialStatePropertyAll<Size?>(maximumSize),
              shape: const MaterialStatePropertyAll<OutlinedBorder>(RoundedRectangleBorder()),
              mouseCursor: MaterialStatePropertyAll<MouseCursor?>(mouseCursor),
              visualDensity: VisualDensity.standard,
              tapTargetSize: MaterialTapTargetSize.shrinkWrap,
              animationDuration: kThemeChangeDuration,
              enableFeedback: true,
              alignment: Alignment.center,
              splashFactory: InkRipple.splashFactory,
            ),
            onPressed:
                onPressed != null
                    ? () {
                      onPressed!(index);
                    }
                    : null,
            child: children[index],
          ),
        ),
      );

      if (currentConstraints != null) {
        button = Center(child: button);
      }

      return MergeSemantics(
        child: Semantics(
          container: true,
          checked: isSelected[index],
          enabled: onPressed != null,
          child: _InputPadding(minSize: minPaddingSize, direction: direction, child: button),
        ),
      );
    });

    if (direction == Axis.vertical) {
      return IntrinsicWidth(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.stretch,
          verticalDirection: verticalDirection,
          children: buttons,
        ),
      );
    }

    return IntrinsicHeight(
      child: Row(
        mainAxisSize: MainAxisSize.min,
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: buttons,
      ),
    );
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      FlagProperty(
        'disabled',
        value: onPressed == null,
        ifTrue: 'Buttons are disabled',
        ifFalse: 'Buttons are enabled',
      ),
    );
    textStyle?.debugFillProperties(properties, prefix: 'textStyle.');
    properties.add(ColorProperty('color', color, defaultValue: null));
    properties.add(ColorProperty('selectedColor', selectedColor, defaultValue: null));
    properties.add(ColorProperty('disabledColor', disabledColor, defaultValue: null));
    properties.add(ColorProperty('fillColor', fillColor, defaultValue: null));
    properties.add(ColorProperty('focusColor', focusColor, defaultValue: null));
    properties.add(ColorProperty('highlightColor', highlightColor, defaultValue: null));
    properties.add(ColorProperty('hoverColor', hoverColor, defaultValue: null));
    properties.add(ColorProperty('splashColor', splashColor, defaultValue: null));
    properties.add(ColorProperty('borderColor', borderColor, defaultValue: null));
    properties.add(ColorProperty('selectedBorderColor', selectedBorderColor, defaultValue: null));
    properties.add(ColorProperty('disabledBorderColor', disabledBorderColor, defaultValue: null));
    properties.add(
      DiagnosticsProperty<BorderRadius>('borderRadius', borderRadius, defaultValue: null),
    );
    properties.add(DoubleProperty('borderWidth', borderWidth, defaultValue: null));
    properties.add(
      DiagnosticsProperty<Axis>('direction', direction, defaultValue: Axis.horizontal),
    );
    properties.add(
      DiagnosticsProperty<VerticalDirection>(
        'verticalDirection',
        verticalDirection,
        defaultValue: VerticalDirection.down,
      ),
    );
  }
}

@immutable
class _ResolveFillColor extends MaterialStateProperty<Color?> with Diagnosticable {
  _ResolveFillColor(this.primary);

  final Color? primary;

  @override
  Color? resolve(Set<MaterialState> states) {
    if (primary is MaterialStateProperty<Color>) {
      return MaterialStateProperty.resolveAs<Color?>(primary, states);
    }
    return states.contains(MaterialState.selected) ? primary : null;
  }
}

@immutable
class _DefaultFillColor extends MaterialStateProperty<Color> with Diagnosticable {
  _DefaultFillColor(this.colorScheme);

  final ColorScheme colorScheme;

  @override
  Color resolve(Set<MaterialState> states) {
    if (states.contains(MaterialState.selected)) {
      return colorScheme.primary.withOpacity(0.12);
    }
    return colorScheme.surface.withOpacity(0.0);
  }
}

@immutable
class _ToggleButtonDefaultOverlay extends MaterialStateProperty<Color?> {
  _ToggleButtonDefaultOverlay({
    required this.selected,
    required this.unselected,
    this.colorScheme,
    this.focusColor,
    this.highlightColor,
    this.hoverColor,
    this.splashColor,
    this.disabledColor,
  });

  final bool selected;
  final bool unselected;
  final ColorScheme? colorScheme;
  final Color? focusColor;
  final Color? highlightColor;
  final Color? hoverColor;
  final Color? splashColor;
  final Color? disabledColor;

  @override
  Color? resolve(Set<MaterialState> states) {
    if (selected) {
      if (states.contains(MaterialState.pressed)) {
        return splashColor ?? colorScheme?.primary.withOpacity(0.16);
      }
      if (states.contains(MaterialState.hovered)) {
        return hoverColor ?? colorScheme?.primary.withOpacity(0.04);
      }
      if (states.contains(MaterialState.focused)) {
        return focusColor ?? colorScheme?.primary.withOpacity(0.12);
      }
    } else if (unselected) {
      if (states.contains(MaterialState.pressed)) {
        return splashColor ?? highlightColor ?? colorScheme?.onSurface.withOpacity(0.16);
      }
      if (states.contains(MaterialState.hovered)) {
        return hoverColor ?? colorScheme?.onSurface.withOpacity(0.04);
      }
      if (states.contains(MaterialState.focused)) {
        return focusColor ?? colorScheme?.onSurface.withOpacity(0.12);
      }
    }
    return null;
  }

  @override
  String toString() {
    return '''
    {
      selected:
        hovered: $hoverColor, otherwise: ${colorScheme?.primary.withOpacity(0.04)},
        focused: $focusColor, otherwise: ${colorScheme?.primary.withOpacity(0.12)},
        pressed: $splashColor, otherwise: ${colorScheme?.primary.withOpacity(0.16)},
      unselected:
        hovered: $hoverColor, otherwise: ${colorScheme?.onSurface.withOpacity(0.04)},
        focused: $focusColor, otherwise: ${colorScheme?.onSurface.withOpacity(0.12)},
        pressed: $splashColor, otherwise: ${colorScheme?.onSurface.withOpacity(0.16)},
      otherwise: null,
    }
    ''';
  }
}

class _SelectToggleButton extends SingleChildRenderObjectWidget {
  const _SelectToggleButton({
    required Widget super.child,
    required this.leadingBorderSide,
    required this.borderSide,
    required this.trailingBorderSide,
    required this.borderRadius,
    required this.isFirstButton,
    required this.isLastButton,
    required this.direction,
    required this.verticalDirection,
  });

  // The width and color of the button's leading side border.
  final BorderSide leadingBorderSide;

  // The width and color of the side borders.
  //
  // If [direction] is [Axis.horizontal], this corresponds to the width and color
  // of the button's top and bottom side borders.
  //
  // If [direction] is [Axis.vertical], this corresponds to the width and color
  // of the button's left and right side borders.
  final BorderSide borderSide;

  // The width and color of the button's trailing side border.
  final BorderSide trailingBorderSide;

  // The border radii of each corner of the button.
  final BorderRadius borderRadius;

  // Whether or not this toggle button is the first button in the list.
  final bool isFirstButton;

  // Whether or not this toggle button is the last button in the list.
  final bool isLastButton;

  // The direction along which the buttons are rendered.
  final Axis direction;

  // If [direction] is [Axis.vertical], this property defines whether or not this button in its list
  // of buttons is laid out starting from top to bottom or from bottom to top.
  final VerticalDirection verticalDirection;

  @override
  _SelectToggleButtonRenderObject createRenderObject(BuildContext context) =>
      _SelectToggleButtonRenderObject(
        leadingBorderSide,
        borderSide,
        trailingBorderSide,
        borderRadius,
        isFirstButton,
        isLastButton,
        direction,
        verticalDirection,
        Directionality.of(context),
      );

  @override
  void updateRenderObject(BuildContext context, _SelectToggleButtonRenderObject renderObject) {
    renderObject
      ..leadingBorderSide = leadingBorderSide
      ..borderSide = borderSide
      ..trailingBorderSide = trailingBorderSide
      ..borderRadius = borderRadius
      ..isFirstButton = isFirstButton
      ..isLastButton = isLastButton
      ..direction = direction
      ..verticalDirection = verticalDirection
      ..textDirection = Directionality.of(context);
  }
}

class _SelectToggleButtonRenderObject extends RenderShiftedBox {
  _SelectToggleButtonRenderObject(
    this._leadingBorderSide,
    this._borderSide,
    this._trailingBorderSide,
    this._borderRadius,
    this._isFirstButton,
    this._isLastButton,
    this._direction,
    this._verticalDirection,
    this._textDirection, [
    RenderBox? child,
  ]) : super(child);

  Axis get direction => _direction;
  Axis _direction;
  set direction(Axis value) {
    if (_direction == value) {
      return;
    }
    _direction = value;
    markNeedsLayout();
  }

  VerticalDirection get verticalDirection => _verticalDirection;
  VerticalDirection _verticalDirection;
  set verticalDirection(VerticalDirection value) {
    if (_verticalDirection == value) {
      return;
    }
    _verticalDirection = value;
    markNeedsLayout();
  }

  // The width and color of the button's leading side border.
  BorderSide get leadingBorderSide => _leadingBorderSide;
  BorderSide _leadingBorderSide;
  set leadingBorderSide(BorderSide value) {
    if (_leadingBorderSide == value) {
      return;
    }
    _leadingBorderSide = value;
    markNeedsLayout();
  }

  // The width and color of the button's top and bottom side borders.
  BorderSide get borderSide => _borderSide;
  BorderSide _borderSide;
  set borderSide(BorderSide value) {
    if (_borderSide == value) {
      return;
    }
    _borderSide = value;
    markNeedsLayout();
  }

  // The width and color of the button's trailing side border.
  BorderSide get trailingBorderSide => _trailingBorderSide;
  BorderSide _trailingBorderSide;
  set trailingBorderSide(BorderSide value) {
    if (_trailingBorderSide == value) {
      return;
    }
    _trailingBorderSide = value;
    markNeedsLayout();
  }

  // The border radii of each corner of the button.
  BorderRadius get borderRadius => _borderRadius;
  BorderRadius _borderRadius;
  set borderRadius(BorderRadius value) {
    if (_borderRadius == value) {
      return;
    }
    _borderRadius = value;
    markNeedsLayout();
  }

  // Whether or not this toggle button is the first button in the list.
  bool get isFirstButton => _isFirstButton;
  bool _isFirstButton;
  set isFirstButton(bool value) {
    if (_isFirstButton == value) {
      return;
    }
    _isFirstButton = value;
    markNeedsLayout();
  }

  // Whether or not this toggle button is the last button in the list.
  bool get isLastButton => _isLastButton;
  bool _isLastButton;
  set isLastButton(bool value) {
    if (_isLastButton == value) {
      return;
    }
    _isLastButton = value;
    markNeedsLayout();
  }

  // The direction in which text flows for this application.
  TextDirection get textDirection => _textDirection;
  TextDirection _textDirection;
  set textDirection(TextDirection value) {
    if (_textDirection == value) {
      return;
    }
    _textDirection = value;
    markNeedsLayout();
  }

  static double _maxHeight(RenderBox? box, double width) {
    return box?.getMaxIntrinsicHeight(width) ?? 0.0;
  }

  static double _minHeight(RenderBox? box, double width) {
    return box?.getMinIntrinsicHeight(width) ?? 0.0;
  }

  static double _minWidth(RenderBox? box, double height) {
    return box?.getMinIntrinsicWidth(height) ?? 0.0;
  }

  static double _maxWidth(RenderBox? box, double height) {
    return box?.getMaxIntrinsicWidth(height) ?? 0.0;
  }

  @override
  double? computeDistanceToActualBaseline(TextBaseline baseline) {
    // The baseline of this widget is the baseline of its child
    final BaselineOffset childOffset = BaselineOffset(child?.getDistanceToActualBaseline(baseline));
    return switch (direction) {
      Axis.horizontal => childOffset + borderSide.width,
      Axis.vertical => childOffset + leadingBorderSide.width,
    }.offset;
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    return direction == Axis.horizontal
        ? borderSide.width * 2.0 + _maxHeight(child, width)
        : leadingBorderSide.width + _maxHeight(child, width) + trailingBorderSide.width;
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    return direction == Axis.horizontal
        ? borderSide.width * 2.0 + _minHeight(child, width)
        : leadingBorderSide.width + _maxHeight(child, width) + trailingBorderSide.width;
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    return direction == Axis.horizontal
        ? leadingBorderSide.width + _maxWidth(child, height) + trailingBorderSide.width
        : borderSide.width * 2.0 + _maxWidth(child, height);
  }

  @override
  double computeMinIntrinsicWidth(double height) {
    return direction == Axis.horizontal
        ? leadingBorderSide.width + _minWidth(child, height) + trailingBorderSide.width
        : borderSide.width * 2.0 + _minWidth(child, height);
  }

  @override
  Size computeDryLayout(BoxConstraints constraints) {
    return _computeSize(constraints: constraints, layoutChild: ChildLayoutHelper.dryLayoutChild);
  }

  EdgeInsetsDirectional get _childPadding {
    assert(child != null);
    // It does not matter what [textDirection] or [verticalDirection] is,
    // since deflating the size constraints horizontally/vertically
    // and the returned size accounts for the width of both sides.
    return switch (direction) {
      Axis.horizontal => EdgeInsetsDirectional.only(
        start: leadingBorderSide.width,
        end: trailingBorderSide.width,
        top: borderSide.width,
        bottom: borderSide.width,
      ),
      Axis.vertical => EdgeInsetsDirectional.only(
        start: borderSide.width,
        end: borderSide.width,
        top: leadingBorderSide.width,
        bottom: trailingBorderSide.width,
      ),
    };
  }

  @override
  double? computeDryBaseline(BoxConstraints constraints, TextBaseline baseline) {
    final double? childBaseline = child?.getDryBaseline(
      constraints.deflate(_childPadding),
      baseline,
    );
    if (childBaseline == null) {
      return null;
    }
    return childBaseline +
        switch (direction) {
          Axis.horizontal => borderSide.width,
          Axis.vertical => switch (verticalDirection) {
            VerticalDirection.down => leadingBorderSide.width,
            VerticalDirection.up => trailingBorderSide.width,
          },
        };
  }

  @override
  void performLayout() {
    size = _computeSize(constraints: constraints, layoutChild: ChildLayoutHelper.layoutChild);
    if (child == null) {
      return;
    }
    final BoxParentData childParentData = child!.parentData! as BoxParentData;
    if (direction == Axis.horizontal) {
      childParentData.offset = switch (textDirection) {
        TextDirection.ltr => Offset(leadingBorderSide.width, borderSide.width),
        TextDirection.rtl => Offset(trailingBorderSide.width, borderSide.width),
      };
    } else {
      childParentData.offset = switch (verticalDirection) {
        VerticalDirection.down => Offset(borderSide.width, leadingBorderSide.width),
        VerticalDirection.up => Offset(borderSide.width, trailingBorderSide.width),
      };
    }
  }

  Size _computeSize({required BoxConstraints constraints, required ChildLayouter layoutChild}) {
    final RenderBox? child = this.child;
    if (child == null) {
      final Size horizontalSize = Size(
        leadingBorderSide.width + trailingBorderSide.width,
        borderSide.width * 2.0,
      );
      return switch (direction) {
        Axis.horizontal => constraints.constrain(horizontalSize),
        Axis.vertical => constraints.constrain(horizontalSize.flipped),
      };
    }

    final EdgeInsetsDirectional childPadding = _childPadding;
    final BoxConstraints innerConstraints = constraints.deflate(childPadding);
    return constraints.constrain(childPadding.inflateSize(layoutChild(child, innerConstraints)));
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    super.paint(context, offset);
    final Offset bottomRight = size.bottomRight(offset);
    final Rect outer = Rect.fromLTRB(offset.dx, offset.dy, bottomRight.dx, bottomRight.dy);
    final Rect center = outer.deflate(borderSide.width / 2.0);
    const double sweepAngle = math.pi / 2.0;
    final RRect rrect =
        RRect.fromRectAndCorners(
          center,
          topLeft:
              (borderRadius.topLeft.x * borderRadius.topLeft.y != 0.0)
                  ? borderRadius.topLeft
                  : Radius.zero,
          topRight:
              (borderRadius.topRight.x * borderRadius.topRight.y != 0.0)
                  ? borderRadius.topRight
                  : Radius.zero,
          bottomLeft:
              (borderRadius.bottomLeft.x * borderRadius.bottomLeft.y != 0.0)
                  ? borderRadius.bottomLeft
                  : Radius.zero,
          bottomRight:
              (borderRadius.bottomRight.x * borderRadius.bottomRight.y != 0.0)
                  ? borderRadius.bottomRight
                  : Radius.zero,
        ).scaleRadii();

    final Rect tlCorner = Rect.fromLTWH(
      rrect.left,
      rrect.top,
      rrect.tlRadiusX * 2.0,
      rrect.tlRadiusY * 2.0,
    );
    final Rect blCorner = Rect.fromLTWH(
      rrect.left,
      rrect.bottom - (rrect.blRadiusY * 2.0),
      rrect.blRadiusX * 2.0,
      rrect.blRadiusY * 2.0,
    );
    final Rect trCorner = Rect.fromLTWH(
      rrect.right - (rrect.trRadiusX * 2),
      rrect.top,
      rrect.trRadiusX * 2,
      rrect.trRadiusY * 2,
    );
    final Rect brCorner = Rect.fromLTWH(
      rrect.right - (rrect.brRadiusX * 2),
      rrect.bottom - (rrect.brRadiusY * 2),
      rrect.brRadiusX * 2,
      rrect.brRadiusY * 2,
    );

    final Paint leadingPaint = leadingBorderSide.toPaint();
    // Only one button.
    if (isFirstButton && isLastButton) {
      final Path leadingPath = Path();
      final double startX = (rrect.brRadiusX == 0.0) ? outer.right : rrect.right - rrect.brRadiusX;
      leadingPath
        ..moveTo(startX, rrect.bottom)
        ..lineTo(rrect.left + rrect.blRadiusX, rrect.bottom)
        ..addArc(blCorner, math.pi / 2.0, sweepAngle)
        ..lineTo(rrect.left, rrect.top + rrect.tlRadiusY)
        ..addArc(tlCorner, math.pi, sweepAngle)
        ..lineTo(rrect.right - rrect.trRadiusX, rrect.top)
        ..addArc(trCorner, math.pi * 3.0 / 2.0, sweepAngle)
        ..lineTo(rrect.right, rrect.bottom - rrect.brRadiusY)
        ..addArc(brCorner, 0, sweepAngle);
      context.canvas.drawPath(leadingPath, leadingPaint);
      return;
    }

    if (direction == Axis.horizontal) {
      switch (textDirection) {
        case TextDirection.ltr:
          if (isLastButton) {
            final Path leftPath = Path();
            leftPath
              ..moveTo(rrect.left, rrect.bottom + leadingBorderSide.width / 2)
              ..lineTo(rrect.left, rrect.top - leadingBorderSide.width / 2);
            context.canvas.drawPath(leftPath, leadingPaint);

            final Paint endingPaint = trailingBorderSide.toPaint();
            final Path endingPath = Path();
            endingPath
              ..moveTo(rrect.left + borderSide.width / 2.0, rrect.top)
              ..lineTo(rrect.right - rrect.trRadiusX, rrect.top)
              ..addArc(trCorner, math.pi * 3.0 / 2.0, sweepAngle)
              ..lineTo(rrect.right, rrect.bottom - rrect.brRadiusY)
              ..addArc(brCorner, 0, sweepAngle)
              ..lineTo(rrect.left + borderSide.width / 2.0, rrect.bottom);
            context.canvas.drawPath(endingPath, endingPaint);
          } else if (isFirstButton) {
            final Path leadingPath = Path();
            leadingPath
              ..moveTo(outer.right, rrect.bottom)
              ..lineTo(rrect.left + rrect.blRadiusX, rrect.bottom)
              ..addArc(blCorner, math.pi / 2.0, sweepAngle)
              ..lineTo(rrect.left, rrect.top + rrect.tlRadiusY)
              ..addArc(tlCorner, math.pi, sweepAngle)
              ..lineTo(outer.right, rrect.top);
            context.canvas.drawPath(leadingPath, leadingPaint);
          } else {
            final Path leadingPath = Path();
            leadingPath
              ..moveTo(rrect.left, rrect.bottom + leadingBorderSide.width / 2)
              ..lineTo(rrect.left, rrect.top - leadingBorderSide.width / 2);
            context.canvas.drawPath(leadingPath, leadingPaint);

            final Paint horizontalPaint = borderSide.toPaint();
            final Path horizontalPaths = Path();
            horizontalPaths
              ..moveTo(rrect.left + borderSide.width / 2.0, rrect.top)
              ..lineTo(outer.right - rrect.trRadiusX, rrect.top)
              ..moveTo(rrect.left + borderSide.width / 2.0 + rrect.tlRadiusX, rrect.bottom)
              ..lineTo(outer.right - rrect.trRadiusX, rrect.bottom);
            context.canvas.drawPath(horizontalPaths, horizontalPaint);
          }
        case TextDirection.rtl:
          if (isLastButton) {
            final Path leadingPath = Path();
            leadingPath
              ..moveTo(rrect.right, rrect.bottom + leadingBorderSide.width / 2)
              ..lineTo(rrect.right, rrect.top - leadingBorderSide.width / 2);
            context.canvas.drawPath(leadingPath, leadingPaint);

            final Paint endingPaint = trailingBorderSide.toPaint();
            final Path endingPath = Path();
            endingPath
              ..moveTo(rrect.right - borderSide.width / 2.0, rrect.top)
              ..lineTo(rrect.left + rrect.tlRadiusX, rrect.top)
              ..addArc(tlCorner, math.pi * 3.0 / 2.0, -sweepAngle)
              ..lineTo(rrect.left, rrect.bottom - rrect.blRadiusY)
              ..addArc(blCorner, math.pi, -sweepAngle)
              ..lineTo(rrect.right - borderSide.width / 2.0, rrect.bottom);
            context.canvas.drawPath(endingPath, endingPaint);
          } else if (isFirstButton) {
            final Path leadingPath = Path();
            leadingPath
              ..moveTo(outer.left, rrect.bottom)
              ..lineTo(rrect.right - rrect.brRadiusX, rrect.bottom)
              ..addArc(brCorner, math.pi / 2.0, -sweepAngle)
              ..lineTo(rrect.right, rrect.top + rrect.trRadiusY)
              ..addArc(trCorner, 0, -sweepAngle)
              ..lineTo(outer.left, rrect.top);
            context.canvas.drawPath(leadingPath, leadingPaint);
          } else {
            final Path leadingPath = Path();
            leadingPath
              ..moveTo(rrect.right, rrect.bottom + leadingBorderSide.width / 2)
              ..lineTo(rrect.right, rrect.top - leadingBorderSide.width / 2);
            context.canvas.drawPath(leadingPath, leadingPaint);

            final Paint horizontalPaint = borderSide.toPaint();
            final Path horizontalPaths = Path();
            horizontalPaths
              ..moveTo(rrect.right - borderSide.width / 2.0, rrect.top)
              ..lineTo(outer.left - rrect.tlRadiusX, rrect.top)
              ..moveTo(rrect.right - borderSide.width / 2.0 + rrect.trRadiusX, rrect.bottom)
              ..lineTo(outer.left - rrect.tlRadiusX, rrect.bottom);
            context.canvas.drawPath(horizontalPaths, horizontalPaint);
          }
      }
    } else {
      switch (verticalDirection) {
        case VerticalDirection.down:
          if (isLastButton) {
            final Path topPath = Path();
            topPath
              ..moveTo(outer.left, outer.top + leadingBorderSide.width / 2)
              ..lineTo(outer.right, outer.top + leadingBorderSide.width / 2);
            context.canvas.drawPath(topPath, leadingPaint);

            final Paint endingPaint = trailingBorderSide.toPaint();
            final Path endingPath = Path();
            endingPath
              ..moveTo(rrect.left, rrect.top + leadingBorderSide.width / 2.0)
              ..lineTo(rrect.left, rrect.bottom - rrect.blRadiusY)
              ..addArc(blCorner, math.pi * 3.0, -sweepAngle)
              ..lineTo(rrect.right - rrect.blRadiusX, rrect.bottom)
              ..addArc(brCorner, math.pi / 2.0, -sweepAngle)
              ..lineTo(rrect.right, rrect.top + leadingBorderSide.width / 2.0);
            context.canvas.drawPath(endingPath, endingPaint);
          } else if (isFirstButton) {
            final Path leadingPath = Path();
            leadingPath
              ..moveTo(rrect.left, outer.bottom)
              ..lineTo(rrect.left, rrect.top + rrect.tlRadiusX)
              ..addArc(tlCorner, math.pi, sweepAngle)
              ..lineTo(rrect.right - rrect.trRadiusX, rrect.top)
              ..addArc(trCorner, math.pi * 3.0 / 2.0, sweepAngle)
              ..lineTo(rrect.right, outer.bottom);
            context.canvas.drawPath(leadingPath, leadingPaint);
          } else {
            final Path topPath = Path();
            topPath
              ..moveTo(outer.left, outer.top + leadingBorderSide.width / 2)
              ..lineTo(outer.right, outer.top + leadingBorderSide.width / 2);
            context.canvas.drawPath(topPath, leadingPaint);

            final Paint paint = borderSide.toPaint();
            final Path paths = Path(); // Left and right borders.
            paths
              ..moveTo(rrect.left, outer.top + leadingBorderSide.width)
              ..lineTo(rrect.left, outer.bottom)
              ..moveTo(rrect.right, outer.top + leadingBorderSide.width)
              ..lineTo(rrect.right, outer.bottom);
            context.canvas.drawPath(paths, paint);
          }
        case VerticalDirection.up:
          if (isLastButton) {
            final Path bottomPath = Path();
            bottomPath
              ..moveTo(outer.left, outer.bottom - leadingBorderSide.width / 2.0)
              ..lineTo(outer.right, outer.bottom - leadingBorderSide.width / 2.0);
            context.canvas.drawPath(bottomPath, leadingPaint);

            final Paint endingPaint = trailingBorderSide.toPaint();
            final Path endingPath = Path();
            endingPath
              ..moveTo(rrect.left, rrect.bottom - leadingBorderSide.width / 2.0)
              ..lineTo(rrect.left, rrect.top + rrect.tlRadiusY)
              ..addArc(tlCorner, math.pi, sweepAngle)
              ..lineTo(rrect.right - rrect.trRadiusX, rrect.top)
              ..addArc(trCorner, math.pi * 3.0 / 2.0, sweepAngle)
              ..lineTo(rrect.right, rrect.bottom - leadingBorderSide.width / 2.0);
            context.canvas.drawPath(endingPath, endingPaint);
          } else if (isFirstButton) {
            final Path leadingPath = Path();
            leadingPath
              ..moveTo(rrect.left, outer.top)
              ..lineTo(rrect.left, rrect.bottom - rrect.blRadiusY)
              ..addArc(blCorner, math.pi, -sweepAngle)
              ..lineTo(rrect.right - rrect.brRadiusX, rrect.bottom)
              ..addArc(brCorner, math.pi / 2.0, -sweepAngle)
              ..lineTo(rrect.right, outer.top);
            context.canvas.drawPath(leadingPath, leadingPaint);
          } else {
            final Path bottomPath = Path();
            bottomPath
              ..moveTo(outer.left, outer.bottom - leadingBorderSide.width / 2.0)
              ..lineTo(outer.right, outer.bottom - leadingBorderSide.width / 2.0);
            context.canvas.drawPath(bottomPath, leadingPaint);

            final Paint paint = borderSide.toPaint();
            final Path paths = Path(); // Left and right borders.
            paths
              ..moveTo(rrect.left, outer.top)
              ..lineTo(rrect.left, outer.bottom - leadingBorderSide.width)
              ..moveTo(rrect.right, outer.top)
              ..lineTo(rrect.right, outer.bottom - leadingBorderSide.width);
            context.canvas.drawPath(paths, paint);
          }
      }
    }
  }
}

/// A widget to pad the area around a [ToggleButtons]'s children.
///
/// This widget is based on a similar one used in [ButtonStyleButton] but it
/// only redirects taps along one axis to ensure the correct button is tapped
/// within the [ToggleButtons].
///
/// This ensures that a widget takes up at least as much space as the minSize
/// parameter to ensure adequate tap target size, while keeping the widget
/// visually smaller to the user.
class _InputPadding extends SingleChildRenderObjectWidget {
  const _InputPadding({super.child, required this.minSize, required this.direction});

  final Size minSize;
  final Axis direction;

  @override
  RenderObject createRenderObject(BuildContext context) {
    return _RenderInputPadding(minSize, direction);
  }

  @override
  void updateRenderObject(BuildContext context, covariant _RenderInputPadding renderObject) {
    renderObject.minSize = minSize;
    renderObject.direction = direction;
  }
}

class _RenderInputPadding extends RenderShiftedBox {
  _RenderInputPadding(this._minSize, this._direction, [RenderBox? child]) : super(child);

  Size get minSize => _minSize;
  Size _minSize;
  set minSize(Size value) {
    if (_minSize == value) {
      return;
    }
    _minSize = value;
    markNeedsLayout();
  }

  Axis get direction => _direction;
  Axis _direction;
  set direction(Axis value) {
    if (_direction == value) {
      return;
    }
    _direction = value;
    markNeedsLayout();
  }

  @override
  double computeMinIntrinsicWidth(double height) {
    if (child != null) {
      return math.max(child!.getMinIntrinsicWidth(height), minSize.width);
    }
    return 0.0;
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    if (child != null) {
      return math.max(child!.getMinIntrinsicHeight(width), minSize.height);
    }
    return 0.0;
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    if (child != null) {
      return math.max(child!.getMaxIntrinsicWidth(height), minSize.width);
    }
    return 0.0;
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    if (child != null) {
      return math.max(child!.getMaxIntrinsicHeight(width), minSize.height);
    }
    return 0.0;
  }

  Size _computeSize({required BoxConstraints constraints, required ChildLayouter layoutChild}) {
    if (child != null) {
      final Size childSize = layoutChild(child!, constraints);
      final double height = math.max(childSize.width, minSize.width);
      final double width = math.max(childSize.height, minSize.height);
      return constraints.constrain(Size(height, width));
    }
    return Size.zero;
  }

  @override
  Size computeDryLayout(BoxConstraints constraints) {
    return _computeSize(constraints: constraints, layoutChild: ChildLayoutHelper.dryLayoutChild);
  }

  @override
  double? computeDryBaseline(covariant BoxConstraints constraints, TextBaseline baseline) {
    final RenderBox? child = this.child;
    if (child == null) {
      return null;
    }
    final double? result = child.getDryBaseline(constraints, baseline);
    if (result == null) {
      return null;
    }
    final Size childSize = child.getDryLayout(constraints);
    return result +
        Alignment.center.alongOffset(getDryLayout(constraints) - childSize as Offset).dy;
  }

  @override
  void performLayout() {
    size = _computeSize(constraints: constraints, layoutChild: ChildLayoutHelper.layoutChild);
    if (child != null) {
      final BoxParentData childParentData = child!.parentData! as BoxParentData;
      childParentData.offset = Alignment.center.alongOffset(size - child!.size as Offset);
    }
  }

  @override
  bool hitTest(BoxHitTestResult result, {required Offset position}) {
    // The super.hitTest() method also checks hitTestChildren(). We don't
    // want that in this case because we've padded around the children per
    // tapTargetSize.
    if (!size.contains(position)) {
      return false;
    }

    // Only adjust one axis to ensure the correct button is tapped.
    final Offset center = switch (direction) {
      Axis.horizontal => Offset(position.dx, child!.size.height / 2),
      Axis.vertical => Offset(child!.size.width / 2, position.dy),
    };
    return result.addWithRawTransform(
      transform: MatrixUtils.forceToPoint(center),
      position: center,
      hitTest: (BoxHitTestResult result, Offset position) {
        assert(position == center);
        return child!.hitTest(result, position: center);
      },
    );
  }
}
