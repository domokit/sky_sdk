// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'app.dart';
/// @docImport 'tooltip.dart';
/// @docImport 'tooltip_visibility.dart';
library;

import 'dart:ui' show lerpDouble;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'theme.dart';

// Examples can assume:
// late BuildContext context;

/// Defines the visual properties of [Tooltip] widgets, a tooltip theme.
///
/// Each property of [TooltipThemeData] corresponds to a property of [Tooltip],
/// and describes the value to use when the [Tooltip] property is
/// not given an explicit non-null value.
///
/// Use this class to configure a [TooltipTheme] widget, or to set the
/// [ThemeData.tooltipTheme] for a [Theme] widget or [MaterialApp.theme].
///
/// To obtain the current ambient tooltip theme, use [TooltipTheme.of].
///
/// See also:
///
///  * [TooltipTheme], a widget which overrides the tooltip theme for a subtree.
///  * [ThemeData.tooltipTheme], which specifies a tooltip theme as part of
///    an overall theme.
///  * [MaterialApp.theme], which specifies a theme for the whole application.
@immutable
class TooltipThemeData with Diagnosticable {
  /// Creates the set of properties used to configure [Tooltip]s.
  const TooltipThemeData({
    this.height,
    this.padding,
    this.margin,
    this.verticalOffset,
    this.preferBelow,
    this.excludeFromSemantics,
    this.decoration,
    this.textStyle,
    this.textAlign,
    this.waitDuration,
    this.showDuration,
    this.exitDuration,
    this.triggerMode,
    this.enableFeedback,
  });

  /// The height of [Tooltip.child].
  final double? height;

  /// If provided, the amount of space by which to inset [Tooltip.child].
  final EdgeInsetsGeometry? padding;

  /// If provided, the amount of empty space to surround the [Tooltip].
  final EdgeInsetsGeometry? margin;

  /// The vertical gap between the widget and the displayed tooltip.
  ///
  /// When [preferBelow] is set to true and tooltips have sufficient space to
  /// display themselves, this property defines how much vertical space
  /// tooltips will position themselves under their corresponding widgets.
  /// Otherwise, tooltips will position themselves above their corresponding
  /// widgets with the given offset.
  final double? verticalOffset;

  /// Whether the tooltip is displayed below its widget by default.
  ///
  /// If there is insufficient space to display the tooltip in the preferred
  /// direction, the tooltip will be displayed in the opposite direction.
  ///
  /// Applying `false` for the entire app is recommended
  /// to avoid having a finger or cursor hide a tooltip.
  final bool? preferBelow;

  /// Whether the [Tooltip.message] should be excluded from the semantics
  /// tree.
  ///
  /// By default, [Tooltip]s will add a [Semantics] label that is set to
  /// [Tooltip.message]. Set this property to true if the app is going to
  /// provide its own custom semantics label.
  final bool? excludeFromSemantics;

  /// The [Tooltip]'s shape and background color.
  final Decoration? decoration;

  /// The style to use for the message of [Tooltip]s.
  final TextStyle? textStyle;

  /// The [TextAlign] to use for the message of [Tooltip]s.
  final TextAlign? textAlign;

  /// The length of time that a pointer must hover over a tooltip's widget
  /// before the tooltip will be shown.
  final Duration? waitDuration;

  /// The length of time that the tooltip will be shown once it has appeared.
  final Duration? showDuration;

  /// The length of time that a pointer must have stopped hovering over a
  /// tooltip's widget before the tooltip will be hidden.
  final Duration? exitDuration;

  /// The [TooltipTriggerMode] that will show the tooltip.
  final TooltipTriggerMode? triggerMode;

  /// Whether the tooltip should provide acoustic and/or haptic feedback.
  ///
  /// For example, on Android a tap will produce a clicking sound and a
  /// long-press will produce a short vibration, when feedback is enabled.
  ///
  /// This value is used if [Tooltip.enableFeedback] is null.
  /// If this value is null, the default is true.
  ///
  /// See also:
  ///
  ///   * [Feedback], for providing platform-specific feedback to certain actions.
  final bool? enableFeedback;

  /// Creates a copy of this object but with the given fields replaced with the
  /// new values.
  TooltipThemeData copyWith({
    double? height,
    EdgeInsetsGeometry? padding,
    EdgeInsetsGeometry? margin,
    double? verticalOffset,
    bool? preferBelow,
    bool? excludeFromSemantics,
    Decoration? decoration,
    TextStyle? textStyle,
    TextAlign? textAlign,
    Duration? waitDuration,
    Duration? showDuration,
    Duration? exitDuration,
    TooltipTriggerMode? triggerMode,
    bool? enableFeedback,
  }) {
    return TooltipThemeData(
      height: height ?? this.height,
      padding: padding ?? this.padding,
      margin: margin ?? this.margin,
      verticalOffset: verticalOffset ?? this.verticalOffset,
      preferBelow: preferBelow ?? this.preferBelow,
      excludeFromSemantics: excludeFromSemantics ?? this.excludeFromSemantics,
      decoration: decoration ?? this.decoration,
      textStyle: textStyle ?? this.textStyle,
      textAlign: textAlign ?? this.textAlign,
      waitDuration: waitDuration ?? this.waitDuration,
      showDuration: showDuration ?? this.showDuration,
      triggerMode: triggerMode ?? this.triggerMode,
      enableFeedback: enableFeedback ?? this.enableFeedback,
    );
  }

  /// Linearly interpolate between two tooltip themes.
  ///
  /// If both arguments are null, then null is returned.
  ///
  /// {@macro dart.ui.shadow.lerp}
  static TooltipThemeData? lerp(TooltipThemeData? a, TooltipThemeData? b, double t) {
    if (identical(a, b)) {
      return a;
    }
    return TooltipThemeData(
      height: lerpDouble(a?.height, b?.height, t),
      padding: EdgeInsetsGeometry.lerp(a?.padding, b?.padding, t),
      margin: EdgeInsetsGeometry.lerp(a?.margin, b?.margin, t),
      verticalOffset: lerpDouble(a?.verticalOffset, b?.verticalOffset, t),
      preferBelow: t < 0.5 ? a?.preferBelow : b?.preferBelow,
      excludeFromSemantics: t < 0.5 ? a?.excludeFromSemantics : b?.excludeFromSemantics,
      decoration: Decoration.lerp(a?.decoration, b?.decoration, t),
      textStyle: TextStyle.lerp(a?.textStyle, b?.textStyle, t),
      textAlign: t < 0.5 ? a?.textAlign : b?.textAlign,
    );
  }

  @override
  int get hashCode => Object.hash(
    height,
    padding,
    margin,
    verticalOffset,
    preferBelow,
    excludeFromSemantics,
    decoration,
    textStyle,
    textAlign,
    waitDuration,
    showDuration,
    exitDuration,
    triggerMode,
    enableFeedback,
  );

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is TooltipThemeData &&
        other.height == height &&
        other.padding == padding &&
        other.margin == margin &&
        other.verticalOffset == verticalOffset &&
        other.preferBelow == preferBelow &&
        other.excludeFromSemantics == excludeFromSemantics &&
        other.decoration == decoration &&
        other.textStyle == textStyle &&
        other.textAlign == textAlign &&
        other.waitDuration == waitDuration &&
        other.showDuration == showDuration &&
        other.exitDuration == exitDuration &&
        other.triggerMode == triggerMode &&
        other.enableFeedback == enableFeedback;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DoubleProperty('height', height, defaultValue: null));
    properties.add(DiagnosticsProperty<EdgeInsetsGeometry>('padding', padding, defaultValue: null));
    properties.add(DiagnosticsProperty<EdgeInsetsGeometry>('margin', margin, defaultValue: null));
    properties.add(DoubleProperty('vertical offset', verticalOffset, defaultValue: null));
    properties.add(
      FlagProperty(
        'position',
        value: preferBelow,
        ifTrue: 'below',
        ifFalse: 'above',
        showName: true,
      ),
    );
    properties.add(
      FlagProperty('semantics', value: excludeFromSemantics, ifTrue: 'excluded', showName: true),
    );
    properties.add(DiagnosticsProperty<Decoration>('decoration', decoration, defaultValue: null));
    properties.add(DiagnosticsProperty<TextStyle>('textStyle', textStyle, defaultValue: null));
    properties.add(DiagnosticsProperty<TextAlign>('textAlign', textAlign, defaultValue: null));
    properties.add(
      DiagnosticsProperty<Duration>('wait duration', waitDuration, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<Duration>('show duration', showDuration, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<Duration>('exit duration', exitDuration, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<TooltipTriggerMode>('triggerMode', triggerMode, defaultValue: null),
    );
    properties.add(
      FlagProperty('enableFeedback', value: enableFeedback, ifTrue: 'true', showName: true),
    );
  }
}

/// Applies a tooltip theme to descendant [Tooltip] widgets.
///
/// A tooltip theme describes the values to use for [Tooltip] properties
/// that are not given an explicit non-null value.
///
/// Descendant widgets obtain the ambient tooltip theme, a [TooltipThemeData],
/// using [TooltipTheme.of].
///
/// {@tool snippet}
///
/// Here is an example of a tooltip theme that applies a blue foreground
/// with non-rounded corners.
///
/// ```dart
/// TooltipTheme(
///   data: TooltipThemeData(
///     decoration: BoxDecoration(
///       color: Colors.blue.withOpacity(0.9),
///       borderRadius: BorderRadius.zero,
///     ),
///   ),
///   child: Tooltip(
///     message: 'Example tooltip',
///     child: IconButton(
///       iconSize: 36.0,
///       icon: const Icon(Icons.touch_app),
///       onPressed: () {},
///     ),
///   ),
/// )
/// ```
/// {@end-tool}
///
/// See also:
///
///  * [TooltipThemeData], which describes the actual configuration of a
///    tooltip theme.
///  * [TooltipVisibility], which can be used to visually disable descendant [Tooltip]s.
class TooltipTheme extends InheritedTheme {
  /// Creates a tooltip theme that controls the configurations for
  /// [Tooltip].
  const TooltipTheme({super.key, required this.data, required super.child});

  /// The properties for descendant [Tooltip] widgets.
  final TooltipThemeData data;

  /// Returns the ambient tooltip theme.
  ///
  /// The result comes from the closest [TooltipTheme] ancestor if any,
  /// and otherwise from [Theme.of] and [ThemeData.tooltipTheme].
  ///
  /// When a widget uses this method, it is automatically rebuilt if the
  /// tooltip theme later changes, so that the changes can be applied.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// TooltipThemeData theme = TooltipTheme.of(context);
  /// ```
  static TooltipThemeData of(BuildContext context) {
    final TooltipTheme? tooltipTheme = context.dependOnInheritedWidgetOfExactType<TooltipTheme>();
    return tooltipTheme?.data ?? Theme.of(context).tooltipTheme;
  }

  @override
  Widget wrap(BuildContext context, Widget child) {
    return TooltipTheme(data: data, child: child);
  }

  @override
  bool updateShouldNotify(TooltipTheme oldWidget) => data != oldWidget.data;
}

/// The method of interaction that will trigger a tooltip.
/// Used in [Tooltip.triggerMode] and [TooltipThemeData.triggerMode].
///
/// On desktop, a tooltip will be shown as soon as a pointer hovers over
/// the widget, regardless of the value of [Tooltip.triggerMode].
///
/// See also:
///
///   * [Tooltip.waitDuration], which defines the length of time that
///     a pointer must hover over a tooltip's widget before the tooltip
///     will be shown.
enum TooltipTriggerMode {
  /// Tooltip will only be shown by calling `ensureTooltipVisible`.
  manual,

  /// Tooltip will be shown after a long press.
  ///
  /// See also:
  ///
  ///   * [GestureDetector.onLongPress], the event that is used for trigger.
  ///   * [Feedback.forLongPress], the feedback method called when feedback is enabled.
  longPress,

  /// Tooltip will be shown after a single tap.
  ///
  /// See also:
  ///
  ///   * [GestureDetector.onTap], the event that is used for trigger.
  ///   * [Feedback.forTap], the feedback method called when feedback is enabled.
  tap,
}
