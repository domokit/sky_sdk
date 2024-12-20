// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'checkbox.dart';
/// @docImport 'list_tile.dart';
/// @docImport 'material.dart';
/// @docImport 'radio.dart';
/// @docImport 'slider.dart';
/// @docImport 'switch_list_tile.dart';
library;

import 'dart:ui';

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';

import 'color_scheme.dart';
import 'colors.dart';
import 'constants.dart';
import 'debug.dart';
import 'material_state.dart';
import 'shadows.dart';
import 'switch_theme.dart';
import 'theme.dart';
import 'theme_data.dart';

// Examples can assume:
// bool _giveVerse = true;
// late StateSetter setState;

enum _SwitchType { material, adaptive }

/// A Material Design switch.
///
/// Used to toggle the on/off state of a single setting.
///
/// The switch itself does not maintain any state. Instead, when the state of
/// the switch changes, the widget calls the [onChanged] callback. Most widgets
/// that use a switch will listen for the [onChanged] callback and rebuild the
/// switch with a new [value] to update the visual appearance of the switch.
///
/// If the [onChanged] callback is null, then the switch will be disabled (it
/// will not respond to input). A disabled switch's thumb and track are rendered
/// in shades of grey by default. The default appearance of a disabled switch
/// can be overridden with [inactiveThumbColor] and [inactiveTrackColor].
///
/// Requires one of its ancestors to be a [Material] widget.
///
/// Material Design 3 provides the option to add icons on the thumb of the [Switch].
/// If [ThemeData.useMaterial3] is set to true, users can use [Switch.thumbIcon]
/// to add optional Icons based on the different [WidgetState]s of the [Switch].
///
/// {@tool dartpad}
/// This example shows a toggleable [Switch]. When the thumb slides to the other
/// side of the track, the switch is toggled between on/off.
///
/// ** See code in examples/api/lib/material/switch/switch.0.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example shows how to customize [Switch] using [WidgetStateProperty]
/// switch properties.
///
/// ** See code in examples/api/lib/material/switch/switch.1.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example shows how to add icons on the thumb of the [Switch] using the
/// [Switch.thumbIcon] property.
///
/// ** See code in examples/api/lib/material/switch/switch.2.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example shows how to use the ambient [CupertinoThemeData] to style all
/// widgets which would otherwise use iOS defaults.
///
/// ** See code in examples/api/lib/material/switch/switch.3.dart **
/// {@end-tool}
///
/// See also:
///
///  * [SwitchListTile], which combines this widget with a [ListTile] so that
///    you can give the switch a label.
///  * [Checkbox], another widget with similar semantics.
///  * [Radio], for selecting among a set of explicit values.
///  * [Slider], for selecting a value in a range.
///  * [WidgetStateProperty], an interface for objects that "resolve" to
///    different values depending on a widget's material state.
///  * <https://material.io/design/components/selection-controls.html#switches>
class Switch extends StatelessWidget {
  /// Creates a Material Design switch.
  ///
  /// The switch itself does not maintain any state. Instead, when the state of
  /// the switch changes, the widget calls the [onChanged] callback. Most widgets
  /// that use a switch will listen for the [onChanged] callback and rebuild the
  /// switch with a new [value] to update the visual appearance of the switch.
  ///
  /// The following arguments are required:
  ///
  /// * [value] determines whether this switch is on or off.
  /// * [onChanged] is called when the user toggles the switch on or off.
  const Switch({
    super.key,
    required this.value,
    required this.onChanged,
    this.activeColor,
    this.activeTrackColor,
    this.inactiveThumbColor,
    this.inactiveTrackColor,
    this.activeThumbImage,
    this.onActiveThumbImageError,
    this.inactiveThumbImage,
    this.onInactiveThumbImageError,
    this.thumbColor,
    this.trackColor,
    this.trackOutlineColor,
    this.trackOutlineWidth,
    this.thumbIcon,
    this.materialTapTargetSize,
    this.dragStartBehavior = DragStartBehavior.start,
    this.mouseCursor,
    this.focusColor,
    this.hoverColor,
    this.overlayColor,
    this.splashRadius,
    this.focusNode,
    this.onFocusChange,
    this.autofocus = false,
    this.padding,
  }) : _switchType = _SwitchType.material,
       applyCupertinoTheme = false,
       assert(activeThumbImage != null || onActiveThumbImageError == null),
       assert(inactiveThumbImage != null || onInactiveThumbImageError == null);

  /// Creates an adaptive [Switch] based on whether the target platform is iOS
  /// or macOS, following Material design's
  /// [Cross-platform guidelines](https://material.io/design/platform-guidance/cross-platform-adaptation.html).
  ///
  /// Creates a switch that looks and feels native when the [ThemeData.platform]
  /// is iOS or macOS, otherwise a Material Design switch is created.
  ///
  /// To provide a custom switch theme that's only used by this factory
  /// constructor, pass a custom `Adaptation<SwitchThemeData>` class to the
  /// `adaptations` parameter of [ThemeData]. This can be useful in situations
  /// where you don't want the overall [ThemeData.switchTheme] to apply when
  /// this adaptive constructor is used.
  ///
  /// {@tool dartpad}
  /// This sample shows how to create and use subclasses of [Adaptation] that
  /// define adaptive [SwitchThemeData]s.
  ///
  /// ** See code in examples/api/lib/material/switch/switch.4.dart **
  /// {@end-tool}
  ///
  /// The target platform is based on the current [Theme]: [ThemeData.platform].
  const Switch.adaptive({
    super.key,
    required this.value,
    required this.onChanged,
    this.activeColor,
    this.activeTrackColor,
    this.inactiveThumbColor,
    this.inactiveTrackColor,
    this.activeThumbImage,
    this.onActiveThumbImageError,
    this.inactiveThumbImage,
    this.onInactiveThumbImageError,
    this.materialTapTargetSize,
    this.thumbColor,
    this.trackColor,
    this.trackOutlineColor,
    this.trackOutlineWidth,
    this.thumbIcon,
    this.dragStartBehavior = DragStartBehavior.start,
    this.mouseCursor,
    this.focusColor,
    this.hoverColor,
    this.overlayColor,
    this.splashRadius,
    this.focusNode,
    this.onFocusChange,
    this.autofocus = false,
    this.padding,
    this.applyCupertinoTheme,
  }) : assert(activeThumbImage != null || onActiveThumbImageError == null),
       assert(inactiveThumbImage != null || onInactiveThumbImageError == null),
       _switchType = _SwitchType.adaptive;

  /// Whether this switch is on or off.
  final bool value;

  /// Called when the user toggles the switch on or off.
  ///
  /// The switch passes the new value to the callback but does not actually
  /// change state until the parent widget rebuilds the switch with the new
  /// value.
  ///
  /// If null, the switch will be displayed as disabled.
  ///
  /// The callback provided to [onChanged] should update the state of the parent
  /// [StatefulWidget] using the [State.setState] method, so that the parent
  /// gets rebuilt; for example:
  ///
  /// ```dart
  /// Switch(
  ///   value: _giveVerse,
  ///   onChanged: (bool newValue) {
  ///     setState(() {
  ///       _giveVerse = newValue;
  ///     });
  ///   },
  /// )
  /// ```
  final ValueChanged<bool>? onChanged;

  /// {@template flutter.material.switch.activeColor}
  /// The color to use when this switch is on.
  /// {@endtemplate}
  ///
  /// Defaults to [ColorScheme.secondary].
  ///
  /// If [thumbColor] returns a non-null color in the [WidgetState.selected]
  /// state, it will be used instead of this color.
  final Color? activeColor;

  /// {@template flutter.material.switch.activeTrackColor}
  /// The color to use on the track when this switch is on.
  /// {@endtemplate}
  ///
  /// Defaults to [ColorScheme.secondary] with the opacity set at 50%.
  ///
  /// If [trackColor] returns a non-null color in the [WidgetState.selected]
  /// state, it will be used instead of this color.
  final Color? activeTrackColor;

  /// {@template flutter.material.switch.inactiveThumbColor}
  /// The color to use on the thumb when this switch is off.
  /// {@endtemplate}
  ///
  /// Defaults to the colors described in the Material design specification.
  ///
  /// If [thumbColor] returns a non-null color in the default state, it will be
  /// used instead of this color.
  final Color? inactiveThumbColor;

  /// {@template flutter.material.switch.inactiveTrackColor}
  /// The color to use on the track when this switch is off.
  /// {@endtemplate}
  ///
  /// Defaults to the colors described in the Material design specification.
  ///
  /// If [trackColor] returns a non-null color in the default state, it will be
  /// used instead of this color.
  final Color? inactiveTrackColor;

  /// {@template flutter.material.switch.activeThumbImage}
  /// An image to use on the thumb of this switch when the switch is on.
  /// {@endtemplate}
  final ImageProvider? activeThumbImage;

  /// {@template flutter.material.switch.onActiveThumbImageError}
  /// An optional error callback for errors emitted when loading
  /// [activeThumbImage].
  /// {@endtemplate}
  final ImageErrorListener? onActiveThumbImageError;

  /// {@template flutter.material.switch.inactiveThumbImage}
  /// An image to use on the thumb of this switch when the switch is off.
  /// {@endtemplate}
  final ImageProvider? inactiveThumbImage;

  /// {@template flutter.material.switch.onInactiveThumbImageError}
  /// An optional error callback for errors emitted when loading
  /// [inactiveThumbImage].
  /// {@endtemplate}
  final ImageErrorListener? onInactiveThumbImageError;

  /// {@template flutter.material.switch.thumbColor}
  /// The color of this [Switch]'s thumb.
  ///
  /// Resolved in the following states:
  ///  * [WidgetState.selected].
  ///  * [WidgetState.hovered].
  ///  * [WidgetState.focused].
  ///  * [WidgetState.disabled].
  ///
  /// {@tool snippet}
  /// This example resolves the [thumbColor] based on the current
  /// [WidgetState] of the [Switch], providing a different [Color] when it is
  /// [WidgetState.disabled].
  ///
  /// ```dart
  /// Switch(
  ///   value: true,
  ///   onChanged: (bool value) { },
  ///   thumbColor: WidgetStateProperty.resolveWith<Color>((Set<WidgetState> states) {
  ///     if (states.contains(WidgetState.disabled)) {
  ///       return Colors.orange.withOpacity(.48);
  ///     }
  ///     return Colors.orange;
  ///   }),
  /// )
  /// ```
  /// {@end-tool}
  /// {@endtemplate}
  ///
  /// If null, then the value of [activeColor] is used in the selected
  /// state and [inactiveThumbColor] in the default state. If that is also null,
  /// then the value of [SwitchThemeData.thumbColor] is used. If that is also
  /// null, then the following colors are used:
  ///
  /// | State    | Light theme                       | Dark theme                        |
  /// |----------|-----------------------------------|-----------------------------------|
  /// | Default  | `Colors.grey.shade50`             | `Colors.grey.shade400`            |
  /// | Selected | [ColorScheme.secondary] | [ColorScheme.secondary] |
  /// | Disabled | `Colors.grey.shade400`            | `Colors.grey.shade800`            |
  final MaterialStateProperty<Color?>? thumbColor;

  /// {@template flutter.material.switch.trackColor}
  /// The color of this [Switch]'s track.
  ///
  /// Resolved in the following states:
  ///  * [WidgetState.selected].
  ///  * [WidgetState.hovered].
  ///  * [WidgetState.focused].
  ///  * [WidgetState.disabled].
  ///
  /// {@tool snippet}
  /// This example resolves the [trackColor] based on the current
  /// [WidgetState] of the [Switch], providing a different [Color] when it is
  /// [WidgetState.disabled].
  ///
  /// ```dart
  /// Switch(
  ///   value: true,
  ///   onChanged: (bool value) { },
  ///   thumbColor: WidgetStateProperty.resolveWith<Color>((Set<WidgetState> states) {
  ///     if (states.contains(WidgetState.disabled)) {
  ///       return Colors.orange.withOpacity(.48);
  ///     }
  ///     return Colors.orange;
  ///   }),
  /// )
  /// ```
  /// {@end-tool}
  /// {@endtemplate}
  ///
  /// If null, then the value of [activeTrackColor] is used in the selected
  /// state and [inactiveTrackColor] in the default state. If that is also null,
  /// then the value of [SwitchThemeData.trackColor] is used. If that is also
  /// null, then the following colors are used:
  ///
  /// | State    | Light theme                     | Dark theme                      |
  /// |----------|---------------------------------|---------------------------------|
  /// | Default  | `Color(0x52000000)`             | `Colors.white30`                |
  /// | Selected | [activeColor] with alpha `0x80` | [activeColor] with alpha `0x80` |
  /// | Disabled | `Colors.black12`                | `Colors.white10`                |
  final MaterialStateProperty<Color?>? trackColor;

  /// {@template flutter.material.switch.trackOutlineColor}
  /// The outline color of this [Switch]'s track.
  ///
  /// Resolved in the following states:
  ///  * [WidgetState.selected].
  ///  * [WidgetState.hovered].
  ///  * [WidgetState.focused].
  ///  * [WidgetState.disabled].
  ///
  /// {@tool snippet}
  /// This example resolves the [trackOutlineColor] based on the current
  /// [WidgetState] of the [Switch], providing a different [Color] when it is
  /// [WidgetState.disabled].
  ///
  /// ```dart
  /// Switch(
  ///   value: true,
  ///   onChanged: (bool value) { },
  ///   trackOutlineColor: WidgetStateProperty.resolveWith<Color?>((Set<WidgetState> states) {
  ///     if (states.contains(WidgetState.disabled)) {
  ///       return Colors.orange.withOpacity(.48);
  ///     }
  ///     return null; // Use the default color.
  ///   }),
  /// )
  /// ```
  /// {@end-tool}
  /// {@endtemplate}
  ///
  /// In Material 3, the outline color defaults to transparent in the selected
  /// state and [ColorScheme.outline] in the unselected state. In Material 2,
  /// the [Switch] track has no outline by default.
  final MaterialStateProperty<Color?>? trackOutlineColor;

  /// {@template flutter.material.switch.trackOutlineWidth}
  /// The outline width of this [Switch]'s track.
  ///
  /// Resolved in the following states:
  ///  * [WidgetState.selected].
  ///  * [WidgetState.hovered].
  ///  * [WidgetState.focused].
  ///  * [WidgetState.disabled].
  ///
  /// {@tool snippet}
  /// This example resolves the [trackOutlineWidth] based on the current
  /// [WidgetState] of the [Switch], providing a different outline width when it is
  /// [WidgetState.disabled].
  ///
  /// ```dart
  /// Switch(
  ///   value: true,
  ///   onChanged: (bool value) { },
  ///   trackOutlineWidth: WidgetStateProperty.resolveWith<double?>((Set<WidgetState> states) {
  ///     if (states.contains(WidgetState.disabled)) {
  ///       return 5.0;
  ///     }
  ///     return null; // Use the default width.
  ///   }),
  /// )
  /// ```
  /// {@end-tool}
  /// {@endtemplate}
  ///
  /// Defaults to 2.0.
  final MaterialStateProperty<double?>? trackOutlineWidth;

  /// {@template flutter.material.switch.thumbIcon}
  /// The icon to use on the thumb of this switch
  ///
  /// Resolved in the following states:
  ///  * [WidgetState.selected].
  ///  * [WidgetState.hovered].
  ///  * [WidgetState.focused].
  ///  * [WidgetState.disabled].
  ///
  /// {@tool snippet}
  /// This example resolves the [thumbIcon] based on the current
  /// [WidgetState] of the [Switch], providing a different [Icon] when it is
  /// [WidgetState.disabled].
  ///
  /// ```dart
  /// Switch(
  ///   value: true,
  ///   onChanged: (bool value) { },
  ///   thumbIcon: WidgetStateProperty.resolveWith<Icon?>((Set<WidgetState> states) {
  ///     if (states.contains(WidgetState.disabled)) {
  ///       return const Icon(Icons.close);
  ///     }
  ///     return null; // All other states will use the default thumbIcon.
  ///   }),
  /// )
  /// ```
  /// {@end-tool}
  /// {@endtemplate}
  ///
  /// If null, then the value of [SwitchThemeData.thumbIcon] is used. If this is also null,
  /// then the [Switch] does not have any icons on the thumb.
  final MaterialStateProperty<Icon?>? thumbIcon;

  /// {@template flutter.material.switch.materialTapTargetSize}
  /// Configures the minimum size of the tap target.
  /// {@endtemplate}
  ///
  /// If null, then the value of [SwitchThemeData.materialTapTargetSize] is
  /// used. If that is also null, then the value of
  /// [ThemeData.materialTapTargetSize] is used.
  ///
  /// See also:
  ///
  ///  * [MaterialTapTargetSize], for a description of how this affects tap targets.
  final MaterialTapTargetSize? materialTapTargetSize;

  final _SwitchType _switchType;

  /// {@macro flutter.cupertino.CupertinoSwitch.applyTheme}
  final bool? applyCupertinoTheme;

  /// {@macro flutter.cupertino.CupertinoSwitch.dragStartBehavior}
  final DragStartBehavior dragStartBehavior;

  /// {@template flutter.material.switch.mouseCursor}
  /// The cursor for a mouse pointer when it enters or is hovering over the
  /// widget.
  ///
  /// If [mouseCursor] is a [WidgetStateMouseCursor],
  /// [WidgetStateProperty.resolve] is used for the following [WidgetState]s:
  ///
  ///  * [WidgetState.selected].
  ///  * [WidgetState.hovered].
  ///  * [WidgetState.focused].
  ///  * [WidgetState.disabled].
  /// {@endtemplate}
  ///
  /// If null, then the value of [SwitchThemeData.mouseCursor] is used. If that
  /// is also null, then [WidgetStateMouseCursor.clickable] is used.
  final MouseCursor? mouseCursor;

  /// The color for the button's [Material] when it has the input focus.
  ///
  /// If [overlayColor] returns a non-null color in the [WidgetState.focused]
  /// state, it will be used instead.
  ///
  /// If null, then the value of [SwitchThemeData.overlayColor] is used in the
  /// focused state. If that is also null, then the value of
  /// [ThemeData.focusColor] is used.
  final Color? focusColor;

  /// The color for the button's [Material] when a pointer is hovering over it.
  ///
  /// If [overlayColor] returns a non-null color in the [WidgetState.hovered]
  /// state, it will be used instead.
  ///
  /// If null, then the value of [SwitchThemeData.overlayColor] is used in the
  /// hovered state. If that is also null, then the value of
  /// [ThemeData.hoverColor] is used.
  final Color? hoverColor;

  /// {@template flutter.material.switch.overlayColor}
  /// The color for the switch's [Material].
  ///
  /// Resolves in the following states:
  ///  * [WidgetState.pressed].
  ///  * [WidgetState.selected].
  ///  * [WidgetState.hovered].
  ///  * [WidgetState.focused].
  /// {@endtemplate}
  ///
  /// If null, then the value of [activeColor] with alpha
  /// [kRadialReactionAlpha], [focusColor] and [hoverColor] is used in the
  /// pressed, focused and hovered state. If that is also null,
  /// the value of [SwitchThemeData.overlayColor] is used. If that is
  /// also null, then the value of [ColorScheme.secondary] with alpha
  /// [kRadialReactionAlpha], [ThemeData.focusColor] and [ThemeData.hoverColor]
  /// is used in the pressed, focused and hovered state.
  final MaterialStateProperty<Color?>? overlayColor;

  /// {@template flutter.material.switch.splashRadius}
  /// The splash radius of the circular [Material] ink response.
  /// {@endtemplate}
  ///
  /// If null, then the value of [SwitchThemeData.splashRadius] is used. If that
  /// is also null, then [kRadialReactionRadius] is used.
  final double? splashRadius;

  /// {@macro flutter.widgets.Focus.focusNode}
  final FocusNode? focusNode;

  /// {@macro flutter.material.inkwell.onFocusChange}
  final ValueChanged<bool>? onFocusChange;

  /// {@macro flutter.widgets.Focus.autofocus}
  final bool autofocus;

  /// The amount of space to surround the child inside the bounds of the [Switch].
  ///
  /// Defaults to horizontal padding of 4 pixels. If [ThemeData.useMaterial3] is false,
  /// then there is no padding by default.
  final EdgeInsetsGeometry? padding;

  Size _getSwitchSize(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    SwitchThemeData switchTheme = SwitchTheme.of(context);
    final SwitchThemeData defaults =
        theme.useMaterial3 ? _SwitchDefaultsM3(context) : _SwitchDefaultsM2(context);
    if (_switchType == _SwitchType.adaptive) {
      final Adaptation<SwitchThemeData> switchAdaptation =
          theme.getAdaptation<SwitchThemeData>() ?? const _SwitchThemeAdaptation();
      switchTheme = switchAdaptation.adapt(theme, switchTheme);
    }
    final _SwitchConfig switchConfig =
        theme.useMaterial3 ? _SwitchConfigM3(context) : _SwitchConfigM2();

    final MaterialTapTargetSize effectiveMaterialTapTargetSize =
        materialTapTargetSize ?? switchTheme.materialTapTargetSize ?? theme.materialTapTargetSize;
    final EdgeInsetsGeometry effectivePadding = padding ?? switchTheme.padding ?? defaults.padding!;
    return switch (effectiveMaterialTapTargetSize) {
      MaterialTapTargetSize.padded => Size(
        switchConfig.switchWidth + effectivePadding.horizontal,
        switchConfig.switchHeight + effectivePadding.vertical,
      ),
      MaterialTapTargetSize.shrinkWrap => Size(
        switchConfig.switchWidth + effectivePadding.horizontal,
        switchConfig.switchHeightCollapsed + effectivePadding.vertical,
      ),
    };
  }

  @override
  Widget build(BuildContext context) {
    Color? effectiveActiveThumbColor;
    Color? effectiveActiveTrackColor;

    switch (_switchType) {
      case _SwitchType.material:
        effectiveActiveThumbColor = activeColor;
      case _SwitchType.adaptive:
        switch (Theme.of(context).platform) {
          case TargetPlatform.android:
          case TargetPlatform.fuchsia:
          case TargetPlatform.linux:
          case TargetPlatform.windows:
            effectiveActiveThumbColor = activeColor;
          case TargetPlatform.iOS:
          case TargetPlatform.macOS:
            effectiveActiveTrackColor = activeColor;
        }
    }
    return _MaterialSwitch(
      value: value,
      onChanged: onChanged,
      size: _getSwitchSize(context),
      activeColor: effectiveActiveThumbColor,
      activeTrackColor: activeTrackColor ?? effectiveActiveTrackColor,
      inactiveThumbColor: inactiveThumbColor,
      inactiveTrackColor: inactiveTrackColor,
      activeThumbImage: activeThumbImage,
      onActiveThumbImageError: onActiveThumbImageError,
      inactiveThumbImage: inactiveThumbImage,
      onInactiveThumbImageError: onInactiveThumbImageError,
      thumbColor: thumbColor,
      trackColor: trackColor,
      trackOutlineColor: trackOutlineColor,
      trackOutlineWidth: trackOutlineWidth,
      thumbIcon: thumbIcon,
      materialTapTargetSize: materialTapTargetSize,
      dragStartBehavior: dragStartBehavior,
      mouseCursor: mouseCursor,
      focusColor: focusColor,
      hoverColor: hoverColor,
      overlayColor: overlayColor,
      splashRadius: splashRadius,
      focusNode: focusNode,
      onFocusChange: onFocusChange,
      autofocus: autofocus,
      applyCupertinoTheme: applyCupertinoTheme,
      switchType: _switchType,
    );
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      FlagProperty('value', value: value, ifTrue: 'on', ifFalse: 'off', showName: true),
    );
    properties.add(
      ObjectFlagProperty<ValueChanged<bool>>('onChanged', onChanged, ifNull: 'disabled'),
    );
  }
}

class _MaterialSwitch extends StatefulWidget {
  const _MaterialSwitch({
    required this.value,
    required this.onChanged,
    required this.size,
    required this.switchType,
    this.activeColor,
    this.activeTrackColor,
    this.inactiveThumbColor,
    this.inactiveTrackColor,
    this.activeThumbImage,
    this.onActiveThumbImageError,
    this.inactiveThumbImage,
    this.onInactiveThumbImageError,
    this.thumbColor,
    this.trackColor,
    this.trackOutlineColor,
    this.trackOutlineWidth,
    this.thumbIcon,
    this.materialTapTargetSize,
    this.dragStartBehavior = DragStartBehavior.start,
    this.mouseCursor,
    this.focusColor,
    this.hoverColor,
    this.overlayColor,
    this.splashRadius,
    this.focusNode,
    this.onFocusChange,
    this.autofocus = false,
    this.applyCupertinoTheme,
  }) : assert(activeThumbImage != null || onActiveThumbImageError == null),
       assert(inactiveThumbImage != null || onInactiveThumbImageError == null);

  final bool value;
  final ValueChanged<bool>? onChanged;
  final Color? activeColor;
  final Color? activeTrackColor;
  final Color? inactiveThumbColor;
  final Color? inactiveTrackColor;
  final ImageProvider? activeThumbImage;
  final ImageErrorListener? onActiveThumbImageError;
  final ImageProvider? inactiveThumbImage;
  final ImageErrorListener? onInactiveThumbImageError;
  final MaterialStateProperty<Color?>? thumbColor;
  final MaterialStateProperty<Color?>? trackColor;
  final MaterialStateProperty<Color?>? trackOutlineColor;
  final MaterialStateProperty<double?>? trackOutlineWidth;
  final MaterialStateProperty<Icon?>? thumbIcon;
  final MaterialTapTargetSize? materialTapTargetSize;
  final DragStartBehavior dragStartBehavior;
  final MouseCursor? mouseCursor;
  final Color? focusColor;
  final Color? hoverColor;
  final MaterialStateProperty<Color?>? overlayColor;
  final double? splashRadius;
  final FocusNode? focusNode;
  final ValueChanged<bool>? onFocusChange;
  final bool autofocus;
  final Size size;
  final bool? applyCupertinoTheme;
  final _SwitchType switchType;

  @override
  State<StatefulWidget> createState() => _MaterialSwitchState();
}

class _MaterialSwitchState extends State<_MaterialSwitch>
    with TickerProviderStateMixin, ToggleableStateMixin {
  final _SwitchPainter _painter = _SwitchPainter();

  @override
  void didUpdateWidget(_MaterialSwitch oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.value != widget.value) {
      // During a drag we may have modified the curve, reset it if its possible
      // to do without visual discontinuation.
      if (position.value == 0.0 || position.value == 1.0) {
        switch (widget.switchType) {
          case _SwitchType.adaptive:
            switch (Theme.of(context).platform) {
              case TargetPlatform.android:
              case TargetPlatform.fuchsia:
              case TargetPlatform.linux:
              case TargetPlatform.windows:
                updateCurve();
              case TargetPlatform.iOS:
              case TargetPlatform.macOS:
                position
                  ..curve = Curves.linear
                  ..reverseCurve = Curves.linear;
            }
          case _SwitchType.material:
            updateCurve();
        }
      }
      animateToValue();
    }
  }

  @override
  void dispose() {
    _painter.dispose();
    super.dispose();
  }

  @override
  ValueChanged<bool?>? get onChanged => widget.onChanged != null ? _handleChanged : null;

  @override
  bool get tristate => false;

  @override
  bool? get value => widget.value;

  @override
  Duration? get reactionAnimationDuration => kRadialReactionDuration;

  void updateCurve() {
    if (Theme.of(context).useMaterial3) {
      position
        ..curve = Curves.easeOutBack
        ..reverseCurve = Curves.easeOutBack.flipped;
    } else {
      position
        ..curve = Curves.easeIn
        ..reverseCurve = Curves.easeOut;
    }
  }

  MaterialStateProperty<Color?> get _widgetThumbColor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return widget.inactiveThumbColor;
      }
      if (states.contains(MaterialState.selected)) {
        return widget.activeColor;
      }
      return widget.inactiveThumbColor;
    });
  }

  MaterialStateProperty<Color?> get _widgetTrackColor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        return widget.activeTrackColor;
      }
      return widget.inactiveTrackColor;
    });
  }

  double get _trackInnerLength {
    switch (widget.switchType) {
      case _SwitchType.adaptive:
        switch (Theme.of(context).platform) {
          case TargetPlatform.android:
          case TargetPlatform.fuchsia:
          case TargetPlatform.linux:
          case TargetPlatform.windows:
            final _SwitchConfig config =
                Theme.of(context).useMaterial3 ? _SwitchConfigM3(context) : _SwitchConfigM2();
            final double trackInnerStart = config.trackHeight / 2.0;
            final double trackInnerEnd = config.trackWidth - trackInnerStart;
            final double trackInnerLength = trackInnerEnd - trackInnerStart;
            return trackInnerLength;
          case TargetPlatform.iOS:
          case TargetPlatform.macOS:
            final _SwitchConfig config = _SwitchConfigCupertino(context);
            final double trackInnerStart = config.trackHeight / 2.0;
            final double trackInnerEnd = config.trackWidth - trackInnerStart;
            final double trackInnerLength = trackInnerEnd - trackInnerStart;
            return trackInnerLength;
        }
      case _SwitchType.material:
        final _SwitchConfig config =
            Theme.of(context).useMaterial3 ? _SwitchConfigM3(context) : _SwitchConfigM2();
        final double trackInnerStart = config.trackHeight / 2.0;
        final double trackInnerEnd = config.trackWidth - trackInnerStart;
        final double trackInnerLength = trackInnerEnd - trackInnerStart;
        return trackInnerLength;
    }
  }

  void _handleDragStart(DragStartDetails details) {
    if (isInteractive) {
      reactionController.forward();
    }
  }

  void _handleDragUpdate(DragUpdateDetails details) {
    if (isInteractive) {
      position
        ..curve = Curves.linear
        ..reverseCurve = null;
      final double delta = details.primaryDelta! / _trackInnerLength;
      positionController.value += switch (Directionality.of(context)) {
        TextDirection.rtl => -delta,
        TextDirection.ltr => delta,
      };
    }
  }

  bool _needsPositionAnimation = false;

  void _handleDragEnd(DragEndDetails details) {
    if (position.value >= 0.5 != widget.value) {
      widget.onChanged?.call(!widget.value);
      // Wait with finishing the animation until widget.value has changed to
      // !widget.value as part of the widget.onChanged call above.
      setState(() {
        _needsPositionAnimation = true;
      });
    } else {
      animateToValue();
    }
    reactionController.reverse();
  }

  void _handleChanged(bool? value) {
    assert(value != null);
    assert(widget.onChanged != null);
    widget.onChanged?.call(value!);
  }

  bool isCupertino = false;

  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasMaterial(context));

    if (_needsPositionAnimation) {
      _needsPositionAnimation = false;
      animateToValue();
    }

    final ThemeData theme = Theme.of(context);
    SwitchThemeData switchTheme = SwitchTheme.of(context);
    final Color cupertinoPrimaryColor =
        theme.cupertinoOverrideTheme?.primaryColor ?? theme.colorScheme.primary;

    _SwitchConfig switchConfig;
    SwitchThemeData defaults;
    bool applyCupertinoTheme = false;
    double disabledOpacity = 1;
    switch (widget.switchType) {
      case _SwitchType.material:
        switchConfig = theme.useMaterial3 ? _SwitchConfigM3(context) : _SwitchConfigM2();
        defaults = theme.useMaterial3 ? _SwitchDefaultsM3(context) : _SwitchDefaultsM2(context);
      case _SwitchType.adaptive:
        final Adaptation<SwitchThemeData> switchAdaptation =
            theme.getAdaptation<SwitchThemeData>() ?? const _SwitchThemeAdaptation();
        switchTheme = switchAdaptation.adapt(theme, switchTheme);
        switch (theme.platform) {
          case TargetPlatform.android:
          case TargetPlatform.fuchsia:
          case TargetPlatform.linux:
          case TargetPlatform.windows:
            switchConfig = theme.useMaterial3 ? _SwitchConfigM3(context) : _SwitchConfigM2();
            defaults = theme.useMaterial3 ? _SwitchDefaultsM3(context) : _SwitchDefaultsM2(context);
          case TargetPlatform.iOS:
          case TargetPlatform.macOS:
            isCupertino = true;
            applyCupertinoTheme =
                widget.applyCupertinoTheme ??
                theme.cupertinoOverrideTheme?.applyThemeToAll ??
                false;
            disabledOpacity = 0.5;
            switchConfig = _SwitchConfigCupertino(context);
            defaults = _SwitchDefaultsCupertino(context);
            reactionController.duration = const Duration(milliseconds: 200);
        }
    }

    positionController.duration = Duration(milliseconds: switchConfig.toggleDuration);

    // Colors need to be resolved in selected and non selected states separately
    // so that they can be lerped between.
    final Set<MaterialState> activeStates = states..add(MaterialState.selected);
    final Set<MaterialState> inactiveStates = states..remove(MaterialState.selected);

    final Color? activeThumbColor =
        widget.thumbColor?.resolve(activeStates) ??
        _widgetThumbColor.resolve(activeStates) ??
        switchTheme.thumbColor?.resolve(activeStates);
    final Color effectiveActiveThumbColor =
        activeThumbColor ?? defaults.thumbColor!.resolve(activeStates)!;
    final Color? inactiveThumbColor =
        widget.thumbColor?.resolve(inactiveStates) ??
        _widgetThumbColor.resolve(inactiveStates) ??
        switchTheme.thumbColor?.resolve(inactiveStates);
    final Color effectiveInactiveThumbColor =
        inactiveThumbColor ?? defaults.thumbColor!.resolve(inactiveStates)!;
    final Color effectiveActiveTrackColor =
        widget.trackColor?.resolve(activeStates) ??
        _widgetTrackColor.resolve(activeStates) ??
        (applyCupertinoTheme
            ? cupertinoPrimaryColor
            : switchTheme.trackColor?.resolve(activeStates)) ??
        _widgetThumbColor.resolve(activeStates)?.withAlpha(0x80) ??
        defaults.trackColor!.resolve(activeStates)!;
    final Color? effectiveActiveTrackOutlineColor =
        widget.trackOutlineColor?.resolve(activeStates) ??
        switchTheme.trackOutlineColor?.resolve(activeStates) ??
        defaults.trackOutlineColor!.resolve(activeStates);
    final double? effectiveActiveTrackOutlineWidth =
        widget.trackOutlineWidth?.resolve(activeStates) ??
        switchTheme.trackOutlineWidth?.resolve(activeStates) ??
        defaults.trackOutlineWidth?.resolve(activeStates);

    final Color effectiveInactiveTrackColor =
        widget.trackColor?.resolve(inactiveStates) ??
        _widgetTrackColor.resolve(inactiveStates) ??
        switchTheme.trackColor?.resolve(inactiveStates) ??
        defaults.trackColor!.resolve(inactiveStates)!;
    final Color? effectiveInactiveTrackOutlineColor =
        widget.trackOutlineColor?.resolve(inactiveStates) ??
        switchTheme.trackOutlineColor?.resolve(inactiveStates) ??
        defaults.trackOutlineColor?.resolve(inactiveStates);
    final double? effectiveInactiveTrackOutlineWidth =
        widget.trackOutlineWidth?.resolve(inactiveStates) ??
        switchTheme.trackOutlineWidth?.resolve(inactiveStates) ??
        defaults.trackOutlineWidth?.resolve(inactiveStates);

    final Icon? effectiveActiveIcon =
        widget.thumbIcon?.resolve(activeStates) ?? switchTheme.thumbIcon?.resolve(activeStates);
    final Icon? effectiveInactiveIcon =
        widget.thumbIcon?.resolve(inactiveStates) ?? switchTheme.thumbIcon?.resolve(inactiveStates);

    final Color effectiveActiveIconColor =
        effectiveActiveIcon?.color ?? switchConfig.iconColor.resolve(activeStates);
    final Color effectiveInactiveIconColor =
        effectiveInactiveIcon?.color ?? switchConfig.iconColor.resolve(inactiveStates);

    final Set<MaterialState> focusedStates = states..add(MaterialState.focused);
    final Color effectiveFocusOverlayColor =
        widget.overlayColor?.resolve(focusedStates) ??
        widget.focusColor ??
        switchTheme.overlayColor?.resolve(focusedStates) ??
        (applyCupertinoTheme
            ? HSLColor.fromColor(
              cupertinoPrimaryColor.withOpacity(0.80),
            ).withLightness(0.69).withSaturation(0.835).toColor()
            : null) ??
        defaults.overlayColor!.resolve(focusedStates)!;

    final Set<MaterialState> hoveredStates = states..add(MaterialState.hovered);
    final Color effectiveHoverOverlayColor =
        widget.overlayColor?.resolve(hoveredStates) ??
        widget.hoverColor ??
        switchTheme.overlayColor?.resolve(hoveredStates) ??
        defaults.overlayColor!.resolve(hoveredStates)!;

    final Set<MaterialState> activePressedStates = activeStates..add(MaterialState.pressed);
    final Color effectiveActivePressedThumbColor =
        widget.thumbColor?.resolve(activePressedStates) ??
        _widgetThumbColor.resolve(activePressedStates) ??
        switchTheme.thumbColor?.resolve(activePressedStates) ??
        defaults.thumbColor!.resolve(activePressedStates)!;
    final Color effectiveActivePressedOverlayColor =
        widget.overlayColor?.resolve(activePressedStates) ??
        switchTheme.overlayColor?.resolve(activePressedStates) ??
        activeThumbColor?.withAlpha(kRadialReactionAlpha) ??
        defaults.overlayColor!.resolve(activePressedStates)!;

    final Set<MaterialState> inactivePressedStates = inactiveStates..add(MaterialState.pressed);
    final Color effectiveInactivePressedThumbColor =
        widget.thumbColor?.resolve(inactivePressedStates) ??
        _widgetThumbColor.resolve(inactivePressedStates) ??
        switchTheme.thumbColor?.resolve(inactivePressedStates) ??
        defaults.thumbColor!.resolve(inactivePressedStates)!;
    final Color effectiveInactivePressedOverlayColor =
        widget.overlayColor?.resolve(inactivePressedStates) ??
        switchTheme.overlayColor?.resolve(inactivePressedStates) ??
        inactiveThumbColor?.withAlpha(kRadialReactionAlpha) ??
        defaults.overlayColor!.resolve(inactivePressedStates)!;

    final MaterialStateProperty<MouseCursor> effectiveMouseCursor =
        MaterialStateProperty.resolveWith<MouseCursor>((Set<MaterialState> states) {
          return MaterialStateProperty.resolveAs<MouseCursor?>(widget.mouseCursor, states) ??
              switchTheme.mouseCursor?.resolve(states) ??
              defaults.mouseCursor!.resolve(states)!;
        });

    final double effectiveActiveThumbRadius =
        effectiveActiveIcon == null
            ? switchConfig.activeThumbRadius
            : switchConfig.thumbRadiusWithIcon;
    final double effectiveInactiveThumbRadius =
        effectiveInactiveIcon == null && widget.inactiveThumbImage == null
            ? switchConfig.inactiveThumbRadius
            : switchConfig.thumbRadiusWithIcon;
    final double effectiveSplashRadius =
        widget.splashRadius ?? switchTheme.splashRadius ?? defaults.splashRadius!;

    return Semantics(
      toggled: widget.value,
      child: GestureDetector(
        excludeFromSemantics: true,
        onHorizontalDragStart: _handleDragStart,
        onHorizontalDragUpdate: _handleDragUpdate,
        onHorizontalDragEnd: _handleDragEnd,
        dragStartBehavior: widget.dragStartBehavior,
        child: Opacity(
          opacity: onChanged == null ? disabledOpacity : 1,
          child: buildToggleable(
            mouseCursor: effectiveMouseCursor,
            focusNode: widget.focusNode,
            onFocusChange: widget.onFocusChange,
            autofocus: widget.autofocus,
            size: widget.size,
            painter:
                _painter
                  ..position = position
                  ..reaction = reaction
                  ..reactionFocusFade = reactionFocusFade
                  ..reactionHoverFade = reactionHoverFade
                  ..inactiveReactionColor = effectiveInactivePressedOverlayColor
                  ..reactionColor = effectiveActivePressedOverlayColor
                  ..hoverColor = effectiveHoverOverlayColor
                  ..focusColor = effectiveFocusOverlayColor
                  ..splashRadius = effectiveSplashRadius
                  ..downPosition = downPosition
                  ..isFocused = states.contains(MaterialState.focused)
                  ..isHovered = states.contains(MaterialState.hovered)
                  ..activeColor = effectiveActiveThumbColor
                  ..inactiveColor = effectiveInactiveThumbColor
                  ..activePressedColor = effectiveActivePressedThumbColor
                  ..inactivePressedColor = effectiveInactivePressedThumbColor
                  ..activeThumbImage = widget.activeThumbImage
                  ..onActiveThumbImageError = widget.onActiveThumbImageError
                  ..inactiveThumbImage = widget.inactiveThumbImage
                  ..onInactiveThumbImageError = widget.onInactiveThumbImageError
                  ..activeTrackColor = effectiveActiveTrackColor
                  ..activeTrackOutlineColor = effectiveActiveTrackOutlineColor
                  ..activeTrackOutlineWidth = effectiveActiveTrackOutlineWidth
                  ..inactiveTrackColor = effectiveInactiveTrackColor
                  ..inactiveTrackOutlineColor = effectiveInactiveTrackOutlineColor
                  ..inactiveTrackOutlineWidth = effectiveInactiveTrackOutlineWidth
                  ..configuration = createLocalImageConfiguration(context)
                  ..isInteractive = isInteractive
                  ..trackInnerLength = _trackInnerLength
                  ..textDirection = Directionality.of(context)
                  ..surfaceColor = theme.colorScheme.surface
                  ..inactiveThumbRadius = effectiveInactiveThumbRadius
                  ..activeThumbRadius = effectiveActiveThumbRadius
                  ..pressedThumbRadius = switchConfig.pressedThumbRadius
                  ..thumbOffset = switchConfig.thumbOffset
                  ..trackHeight = switchConfig.trackHeight
                  ..trackWidth = switchConfig.trackWidth
                  ..activeIconColor = effectiveActiveIconColor
                  ..inactiveIconColor = effectiveInactiveIconColor
                  ..activeIcon = effectiveActiveIcon
                  ..inactiveIcon = effectiveInactiveIcon
                  ..iconTheme = IconTheme.of(context)
                  ..thumbShadow = switchConfig.thumbShadow
                  ..transitionalThumbSize = switchConfig.transitionalThumbSize
                  ..positionController = positionController
                  ..isCupertino = isCupertino,
          ),
        ),
      ),
    );
  }
}

class _SwitchPainter extends ToggleablePainter {
  AnimationController get positionController => _positionController!;
  AnimationController? _positionController;
  set positionController(AnimationController? value) {
    assert(value != null);
    if (value == _positionController) {
      return;
    }
    _positionController = value;
    _colorAnimation?.dispose();
    _colorAnimation = CurvedAnimation(
      parent: positionController,
      curve: Curves.easeOut,
      reverseCurve: Curves.easeIn,
    );
    notifyListeners();
  }

  CurvedAnimation? _colorAnimation;

  Icon? get activeIcon => _activeIcon;
  Icon? _activeIcon;
  set activeIcon(Icon? value) {
    if (value == _activeIcon) {
      return;
    }
    _activeIcon = value;
    notifyListeners();
  }

  Icon? get inactiveIcon => _inactiveIcon;
  Icon? _inactiveIcon;
  set inactiveIcon(Icon? value) {
    if (value == _inactiveIcon) {
      return;
    }
    _inactiveIcon = value;
    notifyListeners();
  }

  IconThemeData? get iconTheme => _iconTheme;
  IconThemeData? _iconTheme;
  set iconTheme(IconThemeData? value) {
    if (value == _iconTheme) {
      return;
    }
    _iconTheme = value;
    notifyListeners();
  }

  Color get activeIconColor => _activeIconColor!;
  Color? _activeIconColor;
  set activeIconColor(Color value) {
    if (value == _activeIconColor) {
      return;
    }
    _activeIconColor = value;
    notifyListeners();
  }

  Color get inactiveIconColor => _inactiveIconColor!;
  Color? _inactiveIconColor;
  set inactiveIconColor(Color value) {
    if (value == _inactiveIconColor) {
      return;
    }
    _inactiveIconColor = value;
    notifyListeners();
  }

  Color get activePressedColor => _activePressedColor!;
  Color? _activePressedColor;
  set activePressedColor(Color? value) {
    assert(value != null);
    if (value == _activePressedColor) {
      return;
    }
    _activePressedColor = value;
    notifyListeners();
  }

  Color get inactivePressedColor => _inactivePressedColor!;
  Color? _inactivePressedColor;
  set inactivePressedColor(Color? value) {
    assert(value != null);
    if (value == _inactivePressedColor) {
      return;
    }
    _inactivePressedColor = value;
    notifyListeners();
  }

  double get activeThumbRadius => _activeThumbRadius!;
  double? _activeThumbRadius;
  set activeThumbRadius(double value) {
    if (value == _activeThumbRadius) {
      return;
    }
    _activeThumbRadius = value;
    notifyListeners();
  }

  double get inactiveThumbRadius => _inactiveThumbRadius!;
  double? _inactiveThumbRadius;
  set inactiveThumbRadius(double value) {
    if (value == _inactiveThumbRadius) {
      return;
    }
    _inactiveThumbRadius = value;
    notifyListeners();
  }

  double get pressedThumbRadius => _pressedThumbRadius!;
  double? _pressedThumbRadius;
  set pressedThumbRadius(double value) {
    if (value == _pressedThumbRadius) {
      return;
    }
    _pressedThumbRadius = value;
    notifyListeners();
  }

  double? get thumbOffset => _thumbOffset;
  double? _thumbOffset;
  set thumbOffset(double? value) {
    if (value == _thumbOffset) {
      return;
    }
    _thumbOffset = value;
    notifyListeners();
  }

  Size get transitionalThumbSize => _transitionalThumbSize!;
  Size? _transitionalThumbSize;
  set transitionalThumbSize(Size value) {
    if (value == _transitionalThumbSize) {
      return;
    }
    _transitionalThumbSize = value;
    notifyListeners();
  }

  double get trackHeight => _trackHeight!;
  double? _trackHeight;
  set trackHeight(double value) {
    if (value == _trackHeight) {
      return;
    }
    _trackHeight = value;
    notifyListeners();
  }

  double get trackWidth => _trackWidth!;
  double? _trackWidth;
  set trackWidth(double value) {
    if (value == _trackWidth) {
      return;
    }
    _trackWidth = value;
    notifyListeners();
  }

  ImageProvider? get activeThumbImage => _activeThumbImage;
  ImageProvider? _activeThumbImage;
  set activeThumbImage(ImageProvider? value) {
    if (value == _activeThumbImage) {
      return;
    }
    _activeThumbImage = value;
    notifyListeners();
  }

  ImageErrorListener? get onActiveThumbImageError => _onActiveThumbImageError;
  ImageErrorListener? _onActiveThumbImageError;
  set onActiveThumbImageError(ImageErrorListener? value) {
    if (value == _onActiveThumbImageError) {
      return;
    }
    _onActiveThumbImageError = value;
    notifyListeners();
  }

  ImageProvider? get inactiveThumbImage => _inactiveThumbImage;
  ImageProvider? _inactiveThumbImage;
  set inactiveThumbImage(ImageProvider? value) {
    if (value == _inactiveThumbImage) {
      return;
    }
    _inactiveThumbImage = value;
    notifyListeners();
  }

  ImageErrorListener? get onInactiveThumbImageError => _onInactiveThumbImageError;
  ImageErrorListener? _onInactiveThumbImageError;
  set onInactiveThumbImageError(ImageErrorListener? value) {
    if (value == _onInactiveThumbImageError) {
      return;
    }
    _onInactiveThumbImageError = value;
    notifyListeners();
  }

  Color get activeTrackColor => _activeTrackColor!;
  Color? _activeTrackColor;
  set activeTrackColor(Color value) {
    if (value == _activeTrackColor) {
      return;
    }
    _activeTrackColor = value;
    notifyListeners();
  }

  Color? get activeTrackOutlineColor => _activeTrackOutlineColor;
  Color? _activeTrackOutlineColor;
  set activeTrackOutlineColor(Color? value) {
    if (value == _activeTrackOutlineColor) {
      return;
    }
    _activeTrackOutlineColor = value;
    notifyListeners();
  }

  Color? get inactiveTrackOutlineColor => _inactiveTrackOutlineColor;
  Color? _inactiveTrackOutlineColor;
  set inactiveTrackOutlineColor(Color? value) {
    if (value == _inactiveTrackOutlineColor) {
      return;
    }
    _inactiveTrackOutlineColor = value;
    notifyListeners();
  }

  double? get activeTrackOutlineWidth => _activeTrackOutlineWidth;
  double? _activeTrackOutlineWidth;
  set activeTrackOutlineWidth(double? value) {
    if (value == _activeTrackOutlineWidth) {
      return;
    }
    _activeTrackOutlineWidth = value;
    notifyListeners();
  }

  double? get inactiveTrackOutlineWidth => _inactiveTrackOutlineWidth;
  double? _inactiveTrackOutlineWidth;
  set inactiveTrackOutlineWidth(double? value) {
    if (value == _inactiveTrackOutlineWidth) {
      return;
    }
    _inactiveTrackOutlineWidth = value;
    notifyListeners();
  }

  Color get inactiveTrackColor => _inactiveTrackColor!;
  Color? _inactiveTrackColor;
  set inactiveTrackColor(Color value) {
    if (value == _inactiveTrackColor) {
      return;
    }
    _inactiveTrackColor = value;
    notifyListeners();
  }

  ImageConfiguration get configuration => _configuration!;
  ImageConfiguration? _configuration;
  set configuration(ImageConfiguration value) {
    if (value == _configuration) {
      return;
    }
    _configuration = value;
    notifyListeners();
  }

  TextDirection get textDirection => _textDirection!;
  TextDirection? _textDirection;
  set textDirection(TextDirection value) {
    if (_textDirection == value) {
      return;
    }
    _textDirection = value;
    notifyListeners();
  }

  Color get surfaceColor => _surfaceColor!;
  Color? _surfaceColor;
  set surfaceColor(Color value) {
    if (value == _surfaceColor) {
      return;
    }
    _surfaceColor = value;
    notifyListeners();
  }

  bool get isInteractive => _isInteractive!;
  bool? _isInteractive;
  set isInteractive(bool value) {
    if (value == _isInteractive) {
      return;
    }
    _isInteractive = value;
    notifyListeners();
  }

  double get trackInnerLength => _trackInnerLength!;
  double? _trackInnerLength;
  set trackInnerLength(double value) {
    if (value == _trackInnerLength) {
      return;
    }
    _trackInnerLength = value;
    notifyListeners();
  }

  bool get isCupertino => _isCupertino!;
  bool? _isCupertino;
  set isCupertino(bool? value) {
    assert(value != null);
    if (value == _isCupertino) {
      return;
    }
    _isCupertino = value;
    notifyListeners();
  }

  List<BoxShadow>? get thumbShadow => _thumbShadow;
  List<BoxShadow>? _thumbShadow;
  set thumbShadow(List<BoxShadow>? value) {
    if (value == _thumbShadow) {
      return;
    }
    _thumbShadow = value;
    notifyListeners();
  }

  final TextPainter _textPainter = TextPainter();
  Color? _cachedThumbColor;
  ImageProvider? _cachedThumbImage;
  ImageErrorListener? _cachedThumbErrorListener;
  BoxPainter? _cachedThumbPainter;

  ShapeDecoration _createDefaultThumbDecoration(
    Color color,
    ImageProvider? image,
    ImageErrorListener? errorListener,
  ) {
    return ShapeDecoration(
      color: color,
      image: image == null ? null : DecorationImage(image: image, onError: errorListener),
      shape: const StadiumBorder(),
      shadows: isCupertino ? null : thumbShadow,
    );
  }

  bool _isPainting = false;

  void _handleDecorationChanged() {
    // If the image decoration is available synchronously, we'll get called here
    // during paint. There's no reason to mark ourselves as needing paint if we
    // are already in the middle of painting. (In fact, doing so would trigger
    // an assert).
    if (!_isPainting) {
      notifyListeners();
    }
  }

  bool _stopPressAnimation = false;
  double? _pressedInactiveThumbRadius;
  double? _pressedActiveThumbRadius;
  late double? _pressedThumbExtension;

  @override
  void paint(Canvas canvas, Size size) {
    final double currentValue = position.value;

    final double visualPosition = switch (textDirection) {
      TextDirection.rtl => 1.0 - currentValue,
      TextDirection.ltr => currentValue,
    };
    if (reaction.status == AnimationStatus.reverse && !_stopPressAnimation) {
      _stopPressAnimation = true;
    } else {
      _stopPressAnimation = false;
    }

    // To get the thumb radius when the press ends, the value can be any number
    // between activeThumbRadius/inactiveThumbRadius and pressedThumbRadius.
    if (!_stopPressAnimation) {
      _pressedThumbExtension = isCupertino ? reaction.value * 7 : 0;
      if (reaction.isCompleted) {
        // This happens when the thumb is dragged instead of being tapped.
        _pressedInactiveThumbRadius = lerpDouble(
          inactiveThumbRadius,
          pressedThumbRadius,
          reaction.value,
        );
        _pressedActiveThumbRadius = lerpDouble(
          activeThumbRadius,
          pressedThumbRadius,
          reaction.value,
        );
      }
      if (currentValue == 0) {
        _pressedInactiveThumbRadius = lerpDouble(
          inactiveThumbRadius,
          pressedThumbRadius,
          reaction.value,
        );
        _pressedActiveThumbRadius = activeThumbRadius;
      }
      if (currentValue == 1) {
        _pressedActiveThumbRadius = lerpDouble(
          activeThumbRadius,
          pressedThumbRadius,
          reaction.value,
        );
        _pressedInactiveThumbRadius = inactiveThumbRadius;
      }
    }
    final Size inactiveThumbSize =
        isCupertino
            ? Size(
              _pressedInactiveThumbRadius! * 2 + _pressedThumbExtension!,
              _pressedInactiveThumbRadius! * 2,
            )
            : Size.fromRadius(_pressedInactiveThumbRadius ?? inactiveThumbRadius);
    final Size activeThumbSize =
        isCupertino
            ? Size(
              _pressedActiveThumbRadius! * 2 + _pressedThumbExtension!,
              _pressedActiveThumbRadius! * 2,
            )
            : Size.fromRadius(_pressedActiveThumbRadius ?? activeThumbRadius);
    Animation<Size> thumbSizeAnimation(bool isForward) {
      List<TweenSequenceItem<Size>> thumbSizeSequence;
      if (isForward) {
        thumbSizeSequence = <TweenSequenceItem<Size>>[
          TweenSequenceItem<Size>(
            tween: Tween<Size>(
              begin: inactiveThumbSize,
              end: transitionalThumbSize,
            ).chain(CurveTween(curve: const Cubic(0.31, 0.00, 0.56, 1.00))),
            weight: 11,
          ),
          TweenSequenceItem<Size>(
            tween: Tween<Size>(
              begin: transitionalThumbSize,
              end: activeThumbSize,
            ).chain(CurveTween(curve: const Cubic(0.20, 0.00, 0.00, 1.00))),
            weight: 72,
          ),
          TweenSequenceItem<Size>(tween: ConstantTween<Size>(activeThumbSize), weight: 17),
        ];
      } else {
        thumbSizeSequence = <TweenSequenceItem<Size>>[
          TweenSequenceItem<Size>(tween: ConstantTween<Size>(inactiveThumbSize), weight: 17),
          TweenSequenceItem<Size>(
            tween: Tween<Size>(
              begin: inactiveThumbSize,
              end: transitionalThumbSize,
            ).chain(CurveTween(curve: const Cubic(0.20, 0.00, 0.00, 1.00).flipped)),
            weight: 72,
          ),
          TweenSequenceItem<Size>(
            tween: Tween<Size>(
              begin: transitionalThumbSize,
              end: activeThumbSize,
            ).chain(CurveTween(curve: const Cubic(0.31, 0.00, 0.56, 1.00).flipped)),
            weight: 11,
          ),
        ];
      }

      return TweenSequence<Size>(thumbSizeSequence).animate(positionController);
    }

    Size? thumbSize;
    if (isCupertino) {
      if (reaction.isCompleted) {
        thumbSize = Size(
          _pressedInactiveThumbRadius! * 2 + _pressedThumbExtension!,
          _pressedInactiveThumbRadius! * 2,
        );
      } else {
        if (position.isDismissed || position.status == AnimationStatus.forward) {
          thumbSize = Size.lerp(inactiveThumbSize, activeThumbSize, position.value);
        } else {
          thumbSize = Size.lerp(inactiveThumbSize, activeThumbSize, position.value);
        }
      }
    } else {
      if (reaction.isCompleted) {
        thumbSize = Size.fromRadius(pressedThumbRadius);
      } else {
        if (position.isDismissed || position.status == AnimationStatus.forward) {
          thumbSize = thumbSizeAnimation(true).value;
        } else {
          thumbSize = thumbSizeAnimation(false).value;
        }
      }
    }

    // The thumb contracts slightly during the animation in Material 2.
    final double inset = thumbOffset == null ? 0 : 1.0 - (currentValue - thumbOffset!).abs() * 2.0;
    thumbSize = Size(thumbSize!.width - inset, thumbSize.height - inset);

    final double colorValue = _colorAnimation!.value;
    final Color trackColor = Color.lerp(inactiveTrackColor, activeTrackColor, colorValue)!;
    final Color? trackOutlineColor =
        inactiveTrackOutlineColor == null || activeTrackOutlineColor == null
            ? null
            : Color.lerp(inactiveTrackOutlineColor, activeTrackOutlineColor, colorValue);
    final double? trackOutlineWidth = lerpDouble(
      inactiveTrackOutlineWidth,
      activeTrackOutlineWidth,
      colorValue,
    );
    Color lerpedThumbColor;
    if (!reaction.isDismissed) {
      lerpedThumbColor = Color.lerp(inactivePressedColor, activePressedColor, colorValue)!;
    } else if (positionController.status == AnimationStatus.forward) {
      lerpedThumbColor = Color.lerp(inactivePressedColor, activeColor, colorValue)!;
    } else if (positionController.status == AnimationStatus.reverse) {
      lerpedThumbColor = Color.lerp(inactiveColor, activePressedColor, colorValue)!;
    } else {
      lerpedThumbColor = Color.lerp(inactiveColor, activeColor, colorValue)!;
    }

    // Blend the thumb color against a `surfaceColor` background in case the
    // thumbColor is not opaque. This way we do not see through the thumb to the
    // track underneath.
    final Color thumbColor = Color.alphaBlend(lerpedThumbColor, surfaceColor);

    final Icon? thumbIcon = currentValue < 0.5 ? inactiveIcon : activeIcon;

    final ImageProvider? thumbImage = currentValue < 0.5 ? inactiveThumbImage : activeThumbImage;

    final ImageErrorListener? thumbErrorListener =
        currentValue < 0.5 ? onInactiveThumbImageError : onActiveThumbImageError;

    final Paint paint = Paint()..color = trackColor;

    final Offset trackPaintOffset = _computeTrackPaintOffset(size, trackWidth, trackHeight);
    final Offset thumbPaintOffset = _computeThumbPaintOffset(
      trackPaintOffset,
      thumbSize,
      visualPosition,
    );
    final Offset radialReactionOrigin = Offset(
      thumbPaintOffset.dx + thumbSize.height / 2,
      size.height / 2,
    );

    _paintTrackWith(canvas, paint, trackPaintOffset, trackOutlineColor, trackOutlineWidth);
    paintRadialReaction(canvas: canvas, origin: radialReactionOrigin);
    _paintThumbWith(
      thumbPaintOffset,
      canvas,
      colorValue,
      thumbColor,
      thumbImage,
      thumbErrorListener,
      thumbIcon,
      thumbSize,
      inset,
    );
  }

  /// Computes canvas offset for track's upper left corner
  Offset _computeTrackPaintOffset(Size canvasSize, double trackWidth, double trackHeight) {
    final double horizontalOffset = (canvasSize.width - trackWidth) / 2.0;
    final double verticalOffset = (canvasSize.height - trackHeight) / 2.0;

    return Offset(horizontalOffset, verticalOffset);
  }

  /// Computes canvas offset for thumb's upper left corner as if it were a
  /// square
  Offset _computeThumbPaintOffset(Offset trackPaintOffset, Size thumbSize, double visualPosition) {
    // How much thumb radius extends beyond the track
    final double trackRadius = trackHeight / 2;
    final double additionalThumbRadius = thumbSize.height / 2 - trackRadius;

    final double horizontalProgress = visualPosition * (trackInnerLength - _pressedThumbExtension!);
    final double thumbHorizontalOffset =
        trackPaintOffset.dx +
        trackRadius +
        (_pressedThumbExtension! / 2) -
        thumbSize.width / 2 +
        horizontalProgress;
    final double thumbVerticalOffset = trackPaintOffset.dy - additionalThumbRadius;
    return Offset(thumbHorizontalOffset, thumbVerticalOffset);
  }

  void _paintTrackWith(
    Canvas canvas,
    Paint paint,
    Offset trackPaintOffset,
    Color? trackOutlineColor,
    double? trackOutlineWidth,
  ) {
    final Rect trackRect = Rect.fromLTWH(
      trackPaintOffset.dx,
      trackPaintOffset.dy,
      trackWidth,
      trackHeight,
    );
    final double trackRadius = trackHeight / 2;
    final RRect trackRRect = RRect.fromRectAndRadius(trackRect, Radius.circular(trackRadius));

    canvas.drawRRect(trackRRect, paint);

    // paint track outline
    if (trackOutlineColor != null) {
      final Rect outlineTrackRect = Rect.fromLTWH(
        trackPaintOffset.dx + 1,
        trackPaintOffset.dy + 1,
        trackWidth - 2,
        trackHeight - 2,
      );
      final RRect outlineTrackRRect = RRect.fromRectAndRadius(
        outlineTrackRect,
        Radius.circular(trackRadius),
      );

      final Paint outlinePaint =
          Paint()
            ..style = PaintingStyle.stroke
            ..strokeWidth = trackOutlineWidth ?? 2.0
            ..color = trackOutlineColor;

      canvas.drawRRect(outlineTrackRRect, outlinePaint);
    }

    if (isCupertino) {
      if (isFocused) {
        final RRect focusedOutline = trackRRect.inflate(1.75);
        final Paint focusedPaint =
            Paint()
              ..style = PaintingStyle.stroke
              ..color = focusColor
              ..strokeWidth = _kCupertinoFocusTrackOutline;
        canvas.drawRRect(focusedOutline, focusedPaint);
      }
      canvas.clipRRect(trackRRect);
    }
  }

  void _paintThumbWith(
    Offset thumbPaintOffset,
    Canvas canvas,
    double currentValue,
    Color thumbColor,
    ImageProvider? thumbImage,
    ImageErrorListener? thumbErrorListener,
    Icon? thumbIcon,
    Size thumbSize,
    double inset,
  ) {
    try {
      _isPainting = true;
      if (_cachedThumbPainter == null ||
          thumbColor != _cachedThumbColor ||
          thumbImage != _cachedThumbImage ||
          thumbErrorListener != _cachedThumbErrorListener) {
        _cachedThumbColor = thumbColor;
        _cachedThumbImage = thumbImage;
        _cachedThumbErrorListener = thumbErrorListener;
        _cachedThumbPainter?.dispose();
        _cachedThumbPainter = _createDefaultThumbDecoration(
          thumbColor,
          thumbImage,
          thumbErrorListener,
        ).createBoxPainter(_handleDecorationChanged);
      }
      final BoxPainter thumbPainter = _cachedThumbPainter!;

      if (isCupertino) {
        _paintCupertinoThumbShadowAndBorder(canvas, thumbPaintOffset, thumbSize);
      }

      thumbPainter.paint(canvas, thumbPaintOffset, configuration.copyWith(size: thumbSize));

      if (thumbIcon != null && thumbIcon.icon != null) {
        final Color iconColor = Color.lerp(inactiveIconColor, activeIconColor, currentValue)!;
        final double iconSize = thumbIcon.size ?? _SwitchConfigM3.iconSize;
        final IconData iconData = thumbIcon.icon!;
        final double? iconWeight = thumbIcon.weight ?? iconTheme?.weight;
        final double? iconFill = thumbIcon.fill ?? iconTheme?.fill;
        final double? iconGrade = thumbIcon.grade ?? iconTheme?.grade;
        final double? iconOpticalSize = thumbIcon.opticalSize ?? iconTheme?.opticalSize;
        final List<Shadow>? iconShadows = thumbIcon.shadows ?? iconTheme?.shadows;

        final TextSpan textSpan = TextSpan(
          text: String.fromCharCode(iconData.codePoint),
          style: TextStyle(
            fontVariations: <FontVariation>[
              if (iconFill != null) FontVariation('FILL', iconFill),
              if (iconWeight != null) FontVariation('wght', iconWeight),
              if (iconGrade != null) FontVariation('GRAD', iconGrade),
              if (iconOpticalSize != null) FontVariation('opsz', iconOpticalSize),
            ],
            color: iconColor,
            fontSize: iconSize,
            inherit: false,
            fontFamily: iconData.fontFamily,
            package: iconData.fontPackage,
            shadows: iconShadows,
          ),
        );
        _textPainter
          ..textDirection = textDirection
          ..text = textSpan;
        _textPainter.layout();
        final double additionalHorizontalOffset = (thumbSize.width - iconSize) / 2;
        final double additionalVerticalOffset = (thumbSize.height - iconSize) / 2;
        final Offset offset =
            thumbPaintOffset + Offset(additionalHorizontalOffset, additionalVerticalOffset);

        _textPainter.paint(canvas, offset);
      }
    } finally {
      _isPainting = false;
    }
  }

  void _paintCupertinoThumbShadowAndBorder(Canvas canvas, Offset thumbPaintOffset, Size thumbSize) {
    final RRect thumbBounds = RRect.fromLTRBR(
      thumbPaintOffset.dx,
      thumbPaintOffset.dy,
      thumbPaintOffset.dx + thumbSize.width,
      thumbPaintOffset.dy + thumbSize.height,
      Radius.circular(thumbSize.height / 2.0),
    );
    if (thumbShadow != null) {
      for (final BoxShadow shadow in thumbShadow!) {
        canvas.drawRRect(thumbBounds.shift(shadow.offset), shadow.toPaint());
      }
    }

    canvas.drawRRect(thumbBounds.inflate(0.5), Paint()..color = const Color(0x0A000000));
  }

  @override
  void dispose() {
    _textPainter.dispose();
    _cachedThumbPainter?.dispose();
    _cachedThumbPainter = null;
    _cachedThumbColor = null;
    _cachedThumbImage = null;
    _cachedThumbErrorListener = null;
    _colorAnimation?.dispose();
    super.dispose();
  }
}

class _SwitchThemeAdaptation extends Adaptation<SwitchThemeData> {
  const _SwitchThemeAdaptation();

  @override
  SwitchThemeData adapt(ThemeData theme, SwitchThemeData defaultValue) {
    switch (theme.platform) {
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        return defaultValue;
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
        return const SwitchThemeData();
    }
  }
}

mixin _SwitchConfig {
  double get trackHeight;
  double get trackWidth;
  double get switchWidth;
  double get switchHeight;
  double get switchHeightCollapsed;
  double get activeThumbRadius;
  double get inactiveThumbRadius;
  double get pressedThumbRadius;
  double get thumbRadiusWithIcon;
  List<BoxShadow>? get thumbShadow;
  MaterialStateProperty<Color> get iconColor;
  double? get thumbOffset;
  Size get transitionalThumbSize;
  int get toggleDuration;
  Size get switchMinSize;
}

// Hand coded defaults for iOS/macOS Switch
class _SwitchDefaultsCupertino extends SwitchThemeData {
  const _SwitchDefaultsCupertino(this.context);

  final BuildContext context;

  @override
  MaterialStateProperty<MouseCursor?> get mouseCursor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return SystemMouseCursors.basic;
      }
      return kIsWeb ? SystemMouseCursors.click : SystemMouseCursors.basic;
    });
  }

  @override
  MaterialStateProperty<Color> get thumbColor =>
      const MaterialStatePropertyAll<Color>(Colors.white);

  @override
  MaterialStateProperty<Color> get trackColor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        return CupertinoDynamicColor.resolve(CupertinoColors.systemGreen, context);
      }
      return CupertinoDynamicColor.resolve(CupertinoColors.secondarySystemFill, context);
    });
  }

  @override
  MaterialStateProperty<Color?> get trackOutlineColor =>
      const MaterialStatePropertyAll<Color>(Colors.transparent);

  @override
  MaterialStateProperty<Color?> get overlayColor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.focused)) {
        return HSLColor.fromColor(
          CupertinoDynamicColor.resolve(CupertinoColors.systemGreen, context).withOpacity(0.80),
        ).withLightness(0.69).withSaturation(0.835).toColor();
      }
      return Colors.transparent;
    });
  }

  @override
  double get splashRadius => 0.0;
}

const double _kCupertinoFocusTrackOutline = 3.5;

class _SwitchConfigCupertino with _SwitchConfig {
  _SwitchConfigCupertino(this.context) : _colors = Theme.of(context).colorScheme;

  BuildContext context;
  final ColorScheme _colors;

  @override
  MaterialStateProperty<Color> get iconColor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return _colors.onSurface.withOpacity(0.38);
      }
      return _colors.onPrimaryContainer;
    });
  }

  @override
  double get activeThumbRadius => 14.0;

  @override
  double get inactiveThumbRadius => 14.0;

  @override
  double get pressedThumbRadius => 14.0;

  @override
  double get switchHeight => switchMinSize.height + 8.0;

  @override
  double get switchHeightCollapsed => switchMinSize.height;

  @override
  double get switchWidth => 60.0;

  @override
  double get thumbRadiusWithIcon => 14.0;

  @override
  List<BoxShadow>? get thumbShadow => const <BoxShadow>[
    BoxShadow(color: Color(0x26000000), offset: Offset(0, 3), blurRadius: 8.0),
    BoxShadow(color: Color(0x0F000000), offset: Offset(0, 3), blurRadius: 1.0),
  ];

  @override
  double get trackHeight => 31.0;

  @override
  double get trackWidth => 51.0;

  // The thumb size at the middle of the track. Hand coded default based on the animation specs.
  @override
  Size get transitionalThumbSize => const Size(28.0, 28.0);

  // Hand coded default by comparing with [CupertinoSwitch].
  @override
  int get toggleDuration => 140;

  // Hand coded default based on the animation specs.
  @override
  double? get thumbOffset => null;

  @override
  Size get switchMinSize => const Size.square(kMinInteractiveDimension - 8.0);
}

// Hand coded defaults based on Material Design 2.
class _SwitchConfigM2 with _SwitchConfig {
  _SwitchConfigM2();

  @override
  double get activeThumbRadius => 10.0;

  @override
  MaterialStateProperty<Color> get iconColor =>
      MaterialStateProperty.all<Color>(Colors.transparent);

  @override
  double get inactiveThumbRadius => 10.0;

  @override
  double get pressedThumbRadius => 10.0;

  @override
  double get switchHeight => switchMinSize.height + 8.0;

  @override
  double get switchHeightCollapsed => switchMinSize.height;

  @override
  double get switchWidth => trackWidth - 2 * (trackHeight / 2.0) + switchMinSize.width;

  @override
  double get thumbRadiusWithIcon => 10.0;

  @override
  List<BoxShadow>? get thumbShadow => kElevationToShadow[1];

  @override
  double get trackHeight => 14.0;

  @override
  double get trackWidth => 33.0;

  @override
  double get thumbOffset => 0.5;

  @override
  Size get transitionalThumbSize => const Size(20, 20);

  @override
  int get toggleDuration => 200;

  @override
  Size get switchMinSize => const Size.square(kMinInteractiveDimension - 8.0);
}

class _SwitchDefaultsM2 extends SwitchThemeData {
  _SwitchDefaultsM2(BuildContext context)
    : _theme = Theme.of(context),
      _colors = Theme.of(context).colorScheme;

  final ThemeData _theme;
  final ColorScheme _colors;

  @override
  MaterialStateProperty<Color> get thumbColor {
    final bool isDark = _theme.brightness == Brightness.dark;

    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return isDark ? Colors.grey.shade800 : Colors.grey.shade400;
      }
      if (states.contains(MaterialState.selected)) {
        return _colors.secondary;
      }
      return isDark ? Colors.grey.shade400 : Colors.grey.shade50;
    });
  }

  @override
  MaterialStateProperty<Color> get trackColor {
    final bool isDark = _theme.brightness == Brightness.dark;
    const Color black32 = Color(0x52000000); // Black with 32% opacity

    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return isDark ? Colors.white10 : Colors.black12;
      }
      if (states.contains(MaterialState.selected)) {
        final Color activeColor = _colors.secondary;
        return activeColor.withAlpha(0x80);
      }
      return isDark ? Colors.white30 : black32;
    });
  }

  @override
  MaterialStateProperty<Color?>? get trackOutlineColor =>
      const MaterialStatePropertyAll<Color>(Colors.transparent);

  @override
  MaterialTapTargetSize get materialTapTargetSize => _theme.materialTapTargetSize;

  @override
  MaterialStateProperty<MouseCursor> get mouseCursor => MaterialStateProperty.resolveWith(
    (Set<MaterialState> states) => MaterialStateMouseCursor.clickable.resolve(states),
  );

  @override
  MaterialStateProperty<Color?> get overlayColor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.pressed)) {
        return thumbColor.resolve(states).withAlpha(kRadialReactionAlpha);
      }
      if (states.contains(MaterialState.hovered)) {
        return _theme.hoverColor;
      }
      if (states.contains(MaterialState.focused)) {
        return _theme.focusColor;
      }
      return null;
    });
  }

  @override
  double get splashRadius => kRadialReactionRadius;

  @override
  EdgeInsetsGeometry? get padding => EdgeInsets.zero;
}

// BEGIN GENERATED TOKEN PROPERTIES - Switch

// Do not edit by hand. The code between the "BEGIN GENERATED" and
// "END GENERATED" comments are generated from data in the Material
// Design token database by the script:
//   dev/tools/gen_defaults/bin/gen_defaults.dart.

// dart format off
class _SwitchDefaultsM3 extends SwitchThemeData {
  _SwitchDefaultsM3(this.context);

  final BuildContext context;
  late final ColorScheme _colors = Theme.of(context).colorScheme;

  @override
  MaterialStateProperty<Color> get thumbColor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        if (states.contains(MaterialState.selected)) {
          return _colors.surface.withOpacity(1.0);
        }
        return _colors.onSurface.withOpacity(0.38);
      }
      if (states.contains(MaterialState.selected)) {
        if (states.contains(MaterialState.pressed)) {
          return _colors.primaryContainer;
        }
        if (states.contains(MaterialState.hovered)) {
          return _colors.primaryContainer;
        }
        if (states.contains(MaterialState.focused)) {
          return _colors.primaryContainer;
        }
        return _colors.onPrimary;
      }
      if (states.contains(MaterialState.pressed)) {
        return _colors.onSurfaceVariant;
      }
      if (states.contains(MaterialState.hovered)) {
        return _colors.onSurfaceVariant;
      }
      if (states.contains(MaterialState.focused)) {
        return _colors.onSurfaceVariant;
      }
      return _colors.outline;
    });
  }

  @override
  MaterialStateProperty<Color> get trackColor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        if (states.contains(MaterialState.selected)) {
          return _colors.onSurface.withOpacity(0.12);
        }
        return _colors.surfaceContainerHighest.withOpacity(0.12);
      }
      if (states.contains(MaterialState.selected)) {
        if (states.contains(MaterialState.pressed)) {
          return _colors.primary;
        }
        if (states.contains(MaterialState.hovered)) {
          return _colors.primary;
        }
        if (states.contains(MaterialState.focused)) {
          return _colors.primary;
        }
        return _colors.primary;
      }
      if (states.contains(MaterialState.pressed)) {
        return _colors.surfaceContainerHighest;
      }
      if (states.contains(MaterialState.hovered)) {
        return _colors.surfaceContainerHighest;
      }
      if (states.contains(MaterialState.focused)) {
        return _colors.surfaceContainerHighest;
      }
      return _colors.surfaceContainerHighest;
    });
  }

  @override
  MaterialStateProperty<Color?> get trackOutlineColor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        return Colors.transparent;
      }
      if (states.contains(MaterialState.disabled)) {
        return _colors.onSurface.withOpacity(0.12);
      }
      return _colors.outline;
    });
  }

  @override
  MaterialStateProperty<Color?> get overlayColor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        if (states.contains(MaterialState.pressed)) {
          return _colors.primary.withOpacity(0.1);
        }
        if (states.contains(MaterialState.hovered)) {
          return _colors.primary.withOpacity(0.08);
        }
        if (states.contains(MaterialState.focused)) {
          return _colors.primary.withOpacity(0.1);
        }
        return null;
      }
      if (states.contains(MaterialState.pressed)) {
        return _colors.onSurface.withOpacity(0.1);
      }
      if (states.contains(MaterialState.hovered)) {
        return _colors.onSurface.withOpacity(0.08);
      }
      if (states.contains(MaterialState.focused)) {
        return _colors.onSurface.withOpacity(0.1);
      }
      return null;
    });
  }

  @override
  MaterialStateProperty<MouseCursor> get mouseCursor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states)
      => MaterialStateMouseCursor.clickable.resolve(states));
  }

  @override
  MaterialStatePropertyAll<double> get trackOutlineWidth => const MaterialStatePropertyAll<double>(2.0);

  @override
  double get splashRadius => 40.0 / 2;

  @override
  EdgeInsetsGeometry? get padding => const EdgeInsets.symmetric(horizontal: 4);
}

class _SwitchConfigM3 with _SwitchConfig {
  _SwitchConfigM3(this.context)
    : _colors = Theme.of(context).colorScheme;

  BuildContext context;
  final ColorScheme _colors;

  static const double iconSize = 16.0;

  @override
  double get activeThumbRadius => 24.0 / 2;

  @override
  MaterialStateProperty<Color> get iconColor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        if (states.contains(MaterialState.selected)) {
          return _colors.onSurface.withOpacity(0.38);
        }
        return _colors.surfaceContainerHighest.withOpacity(0.38);
      }
      if (states.contains(MaterialState.selected)) {
        if (states.contains(MaterialState.pressed)) {
          return _colors.onPrimaryContainer;
        }
        if (states.contains(MaterialState.hovered)) {
          return _colors.onPrimaryContainer;
        }
        if (states.contains(MaterialState.focused)) {
          return _colors.onPrimaryContainer;
        }
        return _colors.onPrimaryContainer;
      }
      if (states.contains(MaterialState.pressed)) {
        return _colors.surfaceContainerHighest;
      }
      if (states.contains(MaterialState.hovered)) {
        return _colors.surfaceContainerHighest;
      }
      if (states.contains(MaterialState.focused)) {
        return _colors.surfaceContainerHighest;
      }
      return _colors.surfaceContainerHighest;
    });
  }

  @override
  double get inactiveThumbRadius => 16.0 / 2;

  @override
  double get pressedThumbRadius => 28.0 / 2;

  @override
  double get switchHeight => switchMinSize.height + 8.0;

  @override
  double get switchHeightCollapsed => switchMinSize.height;

  @override
  double get switchWidth => 52.0;

  @override
  double get thumbRadiusWithIcon => 24.0 / 2;

  @override
  List<BoxShadow>? get thumbShadow => kElevationToShadow[0];

  @override
  double get trackHeight => 32.0;

  @override
  double get trackWidth => 52.0;

  // The thumb size at the middle of the track. Hand coded default based on the animation specs.
  @override
  Size get transitionalThumbSize => const Size(34, 22);

  // Hand coded default based on the animation specs.
  @override
  int get toggleDuration => 300;

  // Hand coded default based on the animation specs.
  @override
  double? get thumbOffset => null;

  @override
  Size get switchMinSize => const Size(kMinInteractiveDimension, kMinInteractiveDimension - 8.0);
}
// dart format on

// END GENERATED TOKEN PROPERTIES - Switch
