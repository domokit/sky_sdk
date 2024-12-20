// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'color_scheme.dart';
/// @docImport 'dialog.dart';
/// @docImport 'icon_button.dart';
/// @docImport 'text_field.dart';
/// @docImport 'text_theme.dart';
/// @docImport 'time_picker.dart';
library;

import 'dart:ui';

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'button_style.dart';
import 'colors.dart';
import 'input_decorator.dart';
import 'material_state.dart';
import 'theme.dart';

// Examples can assume:
// late BuildContext context;

/// Defines the visual properties of the widget displayed with [showTimePicker].
///
/// Descendant widgets obtain the current [TimePickerThemeData] object using
/// `TimePickerTheme.of(context)`. Instances of [TimePickerThemeData]
/// can be customized with [TimePickerThemeData.copyWith].
///
/// Typically a [TimePickerThemeData] is specified as part of the overall
/// [Theme] with [ThemeData.timePickerTheme].
///
/// All [TimePickerThemeData] properties are `null` by default. When null,
/// [showTimePicker] will provide its own defaults.
///
/// See also:
///
///  * [ThemeData], which describes the overall theme information for the
///    application.
///  * [TimePickerTheme], which describes the actual configuration of a time
///    picker theme.
@immutable
class TimePickerThemeData with Diagnosticable {
  /// Creates a theme that can be used for [TimePickerTheme] or
  /// [ThemeData.timePickerTheme].
  const TimePickerThemeData({
    this.backgroundColor,
    this.cancelButtonStyle,
    this.confirmButtonStyle,
    this.dayPeriodBorderSide,
    Color? dayPeriodColor,
    this.dayPeriodShape,
    this.dayPeriodTextColor,
    this.dayPeriodTextStyle,
    this.dialBackgroundColor,
    this.dialHandColor,
    this.dialTextColor,
    this.dialTextStyle,
    this.elevation,
    this.entryModeIconColor,
    this.helpTextStyle,
    this.hourMinuteColor,
    this.hourMinuteShape,
    this.hourMinuteTextColor,
    this.hourMinuteTextStyle,
    this.inputDecorationTheme,
    this.padding,
    this.shape,
    this.timeSelectorSeparatorColor,
    this.timeSelectorSeparatorTextStyle,
  }) : _dayPeriodColor = dayPeriodColor;

  /// The background color of a time picker.
  ///
  /// If this is null, the time picker defaults to the overall theme's
  /// [ColorScheme.surfaceContainerHigh].
  final Color? backgroundColor;

  /// The style of the cancel button of a [TimePickerDialog].
  final ButtonStyle? cancelButtonStyle;

  /// The style of the confirm (OK) button of a [TimePickerDialog].
  final ButtonStyle? confirmButtonStyle;

  /// The color and weight of the day period's outline.
  ///
  /// If this is null, the time picker defaults to:
  ///
  /// ```dart
  /// BorderSide(
  ///   color: Color.alphaBlend(
  ///     Theme.of(context).colorScheme.onSurface.withOpacity(0.38),
  ///     Theme.of(context).colorScheme.surface,
  ///   ),
  /// ),
  /// ```
  final BorderSide? dayPeriodBorderSide;

  /// The background color of the AM/PM toggle.
  ///
  /// If [dayPeriodColor] is a [WidgetStateColor], then the effective
  /// background color can depend on the [WidgetState.selected] state, i.e.
  /// if the segment is selected or not.
  ///
  /// By default, if the segment is selected, the overall theme's
  /// `ColorScheme.primary.withOpacity(0.12)` is used when the overall theme's
  /// brightness is [Brightness.light] and
  /// `ColorScheme.primary.withOpacity(0.24)` is used when the overall theme's
  /// brightness is [Brightness.dark].
  /// If the segment is not selected, [Colors.transparent] is used to allow the
  /// [Dialog]'s color to be used.
  Color? get dayPeriodColor {
    if (_dayPeriodColor == null || _dayPeriodColor is MaterialStateColor) {
      return _dayPeriodColor;
    }
    return MaterialStateColor.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        return _dayPeriodColor;
      }
      // The unselected day period should match the overall picker dialog color.
      // Making it transparent enables that without being redundant and allows
      // the optional elevation overlay for dark mode to be visible.
      return Colors.transparent;
    });
  }

  final Color? _dayPeriodColor;

  /// The shape of the day period that the time picker uses.
  ///
  /// If this is null, the time picker defaults to:
  ///
  /// ```dart
  /// const RoundedRectangleBorder(
  ///   borderRadius: BorderRadius.all(Radius.circular(4.0)),
  ///   side: BorderSide(),
  /// )
  /// ```
  final OutlinedBorder? dayPeriodShape;

  /// The color of the day period text that represents AM/PM.
  ///
  /// If [dayPeriodTextColor] is a [WidgetStateColor], then the effective
  /// text color can depend on the [WidgetState.selected] state, i.e. if the
  /// text is selected or not.
  ///
  /// By default the overall theme's [ColorScheme.primary] color is used when
  /// the text is selected and `ColorScheme.onSurface.withOpacity(0.60)` when
  /// it's not selected.
  final Color? dayPeriodTextColor;

  /// Used to configure the [TextStyle]s for the day period control.
  ///
  /// If this is null, the time picker defaults to the overall theme's
  /// [TextTheme.titleMedium].
  final TextStyle? dayPeriodTextStyle;

  /// The background color of the time picker dial when the entry mode is
  /// [TimePickerEntryMode.dial] or [TimePickerEntryMode.dialOnly].
  ///
  /// If this is null and [ThemeData.useMaterial3] is true, the time picker
  /// dial background color defaults [ColorScheme.surfaceContainerHighest] color.
  ///
  /// If this is null and [ThemeData.useMaterial3] is false, the time picker
  /// dial background color defaults to [ColorScheme.onSurface] color with
  /// an opacity of 0.08 when the overall theme's brightness is [Brightness.light]
  /// and [ColorScheme.onSurface] color with an opacity of 0.12 when the overall
  /// theme's brightness is [Brightness.dark].
  final Color? dialBackgroundColor;

  /// The color of the time picker dial's hand when the entry mode is
  /// [TimePickerEntryMode.dial] or [TimePickerEntryMode.dialOnly].
  ///
  /// If this is null, the time picker defaults to the overall theme's
  /// [ColorScheme.primary].
  final Color? dialHandColor;

  /// The color of the dial text that represents specific hours and minutes.
  ///
  /// If [dialTextColor] is a [WidgetStateColor], then the effective
  /// text color can depend on the [WidgetState.selected] state, i.e. if the
  /// text is selected or not.
  ///
  /// If this color is null then the dial's text colors are based on the
  /// theme's [ThemeData.colorScheme].
  final Color? dialTextColor;

  /// The [TextStyle] for the numbers on the time selection dial.
  ///
  /// If [dialTextStyle]'s [TextStyle.color] is a [WidgetStateColor], then the
  /// effective text color can depend on the [WidgetState.selected] state,
  /// i.e. if the text is selected or not.
  ///
  /// If this style is null then the dial's text style is based on the theme's
  /// [ThemeData.textTheme].
  final TextStyle? dialTextStyle;

  /// The Material elevation for the time picker dialog.
  final double? elevation;

  /// The color of the entry mode [IconButton].
  ///
  /// If this is null, the time picker defaults to:
  ///
  ///
  /// ```dart
  /// Theme.of(context).colorScheme.onSurface.withOpacity(
  ///   Theme.of(context).colorScheme.brightness == Brightness.dark ? 1.0 : 0.6,
  /// )
  /// ```
  final Color? entryModeIconColor;

  /// Used to configure the [TextStyle]s for the helper text in the header.
  ///
  /// If this is null, the time picker defaults to the overall theme's
  /// [TextTheme.labelSmall].
  final TextStyle? helpTextStyle;

  /// The background color of the hour and minute header segments.
  ///
  /// If [hourMinuteColor] is a [WidgetStateColor], then the effective
  /// background color can depend on the [WidgetState.selected] state, i.e.
  /// if the segment is selected or not.
  ///
  /// By default, if the segment is selected, the overall theme's
  /// `ColorScheme.primary.withOpacity(0.12)` is used when the overall theme's
  /// brightness is [Brightness.light] and
  /// `ColorScheme.primary.withOpacity(0.24)` is used when the overall theme's
  /// brightness is [Brightness.dark].
  /// If the segment is not selected, the overall theme's
  /// `ColorScheme.onSurface.withOpacity(0.12)` is used.
  final Color? hourMinuteColor;

  /// The shape of the hour and minute controls that the time picker uses.
  ///
  /// If this is null, the time picker defaults to
  /// `RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(4.0)))`.
  final ShapeBorder? hourMinuteShape;

  /// The color of the header text that represents hours and minutes.
  ///
  /// If [hourMinuteTextColor] is a [WidgetStateColor], then the effective
  /// text color can depend on the [WidgetState.selected] state, i.e. if the
  /// text is selected or not.
  ///
  /// By default the overall theme's [ColorScheme.primary] color is used when
  /// the text is selected and [ColorScheme.onSurface] when it's not selected.
  final Color? hourMinuteTextColor;

  /// Used to configure the [TextStyle]s for the hour/minute controls.
  ///
  /// If this is null and entry mode is [TimePickerEntryMode.dial], the time
  /// picker defaults to the overall theme's [TextTheme.displayLarge] with
  /// the value of [hourMinuteTextColor].
  ///
  /// If this is null and entry mode is [TimePickerEntryMode.input], the time
  /// picker defaults to the overall theme's [TextTheme.displayMedium] with
  /// the value of [hourMinuteTextColor].
  ///
  /// If this is null and [ThemeData.useMaterial3] is false, the time picker
  /// defaults to the overall theme's [TextTheme.displayMedium].
  final TextStyle? hourMinuteTextStyle;

  /// The input decoration theme for the [TextField]s in the time picker.
  ///
  /// If this is null, the time picker provides its own defaults.
  final InputDecorationTheme? inputDecorationTheme;

  /// The padding around the time picker dialog when the entry mode is
  /// [TimePickerEntryMode.dial] or [TimePickerEntryMode.dialOnly].
  final EdgeInsetsGeometry? padding;

  /// The shape of the [Dialog] that the time picker is presented in.
  ///
  /// If this is null, the time picker defaults to
  /// `RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(4.0)))`.
  final ShapeBorder? shape;

  /// The color of the time selector separator between the hour and minute controls.
  ///
  /// if this is null, the time picker defaults to the overall theme's
  /// [ColorScheme.onSurface].
  ///
  /// If this is null and [ThemeData.useMaterial3] is false, then defaults to the value of
  /// [hourMinuteTextColor].
  final MaterialStateProperty<Color?>? timeSelectorSeparatorColor;

  /// Used to configure the text style for the time selector separator between the hour
  /// and minute controls.
  ///
  /// If this is null, the time picker defaults to the overall theme's
  /// [TextTheme.displayLarge].
  ///
  /// If this is null and [ThemeData.useMaterial3] is false, then defaults to the value of
  /// [hourMinuteTextStyle].
  final MaterialStateProperty<TextStyle?>? timeSelectorSeparatorTextStyle;

  /// Creates a copy of this object with the given fields replaced with the
  /// new values.
  TimePickerThemeData copyWith({
    Color? backgroundColor,
    ButtonStyle? cancelButtonStyle,
    ButtonStyle? confirmButtonStyle,
    ButtonStyle? dayPeriodButtonStyle,
    BorderSide? dayPeriodBorderSide,
    Color? dayPeriodColor,
    OutlinedBorder? dayPeriodShape,
    Color? dayPeriodTextColor,
    TextStyle? dayPeriodTextStyle,
    Color? dialBackgroundColor,
    Color? dialHandColor,
    Color? dialTextColor,
    TextStyle? dialTextStyle,
    double? elevation,
    Color? entryModeIconColor,
    TextStyle? helpTextStyle,
    Color? hourMinuteColor,
    ShapeBorder? hourMinuteShape,
    Color? hourMinuteTextColor,
    TextStyle? hourMinuteTextStyle,
    InputDecorationTheme? inputDecorationTheme,
    EdgeInsetsGeometry? padding,
    ShapeBorder? shape,
    MaterialStateProperty<Color?>? timeSelectorSeparatorColor,
    MaterialStateProperty<TextStyle?>? timeSelectorSeparatorTextStyle,
  }) {
    return TimePickerThemeData(
      backgroundColor: backgroundColor ?? this.backgroundColor,
      cancelButtonStyle: cancelButtonStyle ?? this.cancelButtonStyle,
      confirmButtonStyle: confirmButtonStyle ?? this.confirmButtonStyle,
      dayPeriodBorderSide: dayPeriodBorderSide ?? this.dayPeriodBorderSide,
      dayPeriodColor: dayPeriodColor ?? this.dayPeriodColor,
      dayPeriodShape: dayPeriodShape ?? this.dayPeriodShape,
      dayPeriodTextColor: dayPeriodTextColor ?? this.dayPeriodTextColor,
      dayPeriodTextStyle: dayPeriodTextStyle ?? this.dayPeriodTextStyle,
      dialBackgroundColor: dialBackgroundColor ?? this.dialBackgroundColor,
      dialHandColor: dialHandColor ?? this.dialHandColor,
      dialTextColor: dialTextColor ?? this.dialTextColor,
      dialTextStyle: dialTextStyle ?? this.dialTextStyle,
      elevation: elevation ?? this.elevation,
      entryModeIconColor: entryModeIconColor ?? this.entryModeIconColor,
      helpTextStyle: helpTextStyle ?? this.helpTextStyle,
      hourMinuteColor: hourMinuteColor ?? this.hourMinuteColor,
      hourMinuteShape: hourMinuteShape ?? this.hourMinuteShape,
      hourMinuteTextColor: hourMinuteTextColor ?? this.hourMinuteTextColor,
      hourMinuteTextStyle: hourMinuteTextStyle ?? this.hourMinuteTextStyle,
      inputDecorationTheme: inputDecorationTheme ?? this.inputDecorationTheme,
      padding: padding ?? this.padding,
      shape: shape ?? this.shape,
      timeSelectorSeparatorColor: timeSelectorSeparatorColor ?? this.timeSelectorSeparatorColor,
      timeSelectorSeparatorTextStyle:
          timeSelectorSeparatorTextStyle ?? this.timeSelectorSeparatorTextStyle,
    );
  }

  /// Linearly interpolate between two time picker themes.
  ///
  /// {@macro dart.ui.shadow.lerp}
  static TimePickerThemeData lerp(TimePickerThemeData? a, TimePickerThemeData? b, double t) {
    if (identical(a, b) && a != null) {
      return a;
    }
    // Workaround since BorderSide's lerp does not allow for null arguments.
    BorderSide? lerpedBorderSide;
    if (a?.dayPeriodBorderSide == null && b?.dayPeriodBorderSide == null) {
      lerpedBorderSide = null;
    } else if (a?.dayPeriodBorderSide == null) {
      lerpedBorderSide = b?.dayPeriodBorderSide;
    } else if (b?.dayPeriodBorderSide == null) {
      lerpedBorderSide = a?.dayPeriodBorderSide;
    } else {
      lerpedBorderSide = BorderSide.lerp(a!.dayPeriodBorderSide!, b!.dayPeriodBorderSide!, t);
    }
    return TimePickerThemeData(
      backgroundColor: Color.lerp(a?.backgroundColor, b?.backgroundColor, t),
      cancelButtonStyle: ButtonStyle.lerp(a?.cancelButtonStyle, b?.cancelButtonStyle, t),
      confirmButtonStyle: ButtonStyle.lerp(a?.confirmButtonStyle, b?.confirmButtonStyle, t),
      dayPeriodBorderSide: lerpedBorderSide,
      dayPeriodColor: Color.lerp(a?.dayPeriodColor, b?.dayPeriodColor, t),
      dayPeriodShape: ShapeBorder.lerp(a?.dayPeriodShape, b?.dayPeriodShape, t) as OutlinedBorder?,
      dayPeriodTextColor: Color.lerp(a?.dayPeriodTextColor, b?.dayPeriodTextColor, t),
      dayPeriodTextStyle: TextStyle.lerp(a?.dayPeriodTextStyle, b?.dayPeriodTextStyle, t),
      dialBackgroundColor: Color.lerp(a?.dialBackgroundColor, b?.dialBackgroundColor, t),
      dialHandColor: Color.lerp(a?.dialHandColor, b?.dialHandColor, t),
      dialTextColor: Color.lerp(a?.dialTextColor, b?.dialTextColor, t),
      dialTextStyle: TextStyle.lerp(a?.dialTextStyle, b?.dialTextStyle, t),
      elevation: lerpDouble(a?.elevation, b?.elevation, t),
      entryModeIconColor: Color.lerp(a?.entryModeIconColor, b?.entryModeIconColor, t),
      helpTextStyle: TextStyle.lerp(a?.helpTextStyle, b?.helpTextStyle, t),
      hourMinuteColor: Color.lerp(a?.hourMinuteColor, b?.hourMinuteColor, t),
      hourMinuteShape: ShapeBorder.lerp(a?.hourMinuteShape, b?.hourMinuteShape, t),
      hourMinuteTextColor: Color.lerp(a?.hourMinuteTextColor, b?.hourMinuteTextColor, t),
      hourMinuteTextStyle: TextStyle.lerp(a?.hourMinuteTextStyle, b?.hourMinuteTextStyle, t),
      inputDecorationTheme: t < 0.5 ? a?.inputDecorationTheme : b?.inputDecorationTheme,
      padding: EdgeInsetsGeometry.lerp(a?.padding, b?.padding, t),
      shape: ShapeBorder.lerp(a?.shape, b?.shape, t),
      timeSelectorSeparatorColor: MaterialStateProperty.lerp<Color?>(
        a?.timeSelectorSeparatorColor,
        b?.timeSelectorSeparatorColor,
        t,
        Color.lerp,
      ),
      timeSelectorSeparatorTextStyle: MaterialStateProperty.lerp<TextStyle?>(
        a?.timeSelectorSeparatorTextStyle,
        b?.timeSelectorSeparatorTextStyle,
        t,
        TextStyle.lerp,
      ),
    );
  }

  @override
  int get hashCode => Object.hashAll(<Object?>[
    backgroundColor,
    cancelButtonStyle,
    confirmButtonStyle,
    dayPeriodBorderSide,
    dayPeriodColor,
    dayPeriodShape,
    dayPeriodTextColor,
    dayPeriodTextStyle,
    dialBackgroundColor,
    dialHandColor,
    dialTextColor,
    dialTextStyle,
    elevation,
    entryModeIconColor,
    helpTextStyle,
    hourMinuteColor,
    hourMinuteShape,
    hourMinuteTextColor,
    hourMinuteTextStyle,
    inputDecorationTheme,
    padding,
    shape,
    timeSelectorSeparatorColor,
    timeSelectorSeparatorTextStyle,
  ]);

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is TimePickerThemeData &&
        other.backgroundColor == backgroundColor &&
        other.cancelButtonStyle == cancelButtonStyle &&
        other.confirmButtonStyle == confirmButtonStyle &&
        other.dayPeriodBorderSide == dayPeriodBorderSide &&
        other.dayPeriodColor == dayPeriodColor &&
        other.dayPeriodShape == dayPeriodShape &&
        other.dayPeriodTextColor == dayPeriodTextColor &&
        other.dayPeriodTextStyle == dayPeriodTextStyle &&
        other.dialBackgroundColor == dialBackgroundColor &&
        other.dialHandColor == dialHandColor &&
        other.dialTextColor == dialTextColor &&
        other.dialTextStyle == dialTextStyle &&
        other.elevation == elevation &&
        other.entryModeIconColor == entryModeIconColor &&
        other.helpTextStyle == helpTextStyle &&
        other.hourMinuteColor == hourMinuteColor &&
        other.hourMinuteShape == hourMinuteShape &&
        other.hourMinuteTextColor == hourMinuteTextColor &&
        other.hourMinuteTextStyle == hourMinuteTextStyle &&
        other.inputDecorationTheme == inputDecorationTheme &&
        other.padding == padding &&
        other.shape == shape &&
        other.timeSelectorSeparatorColor == timeSelectorSeparatorColor &&
        other.timeSelectorSeparatorTextStyle == timeSelectorSeparatorTextStyle;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(ColorProperty('backgroundColor', backgroundColor, defaultValue: null));
    properties.add(
      DiagnosticsProperty<ButtonStyle>('cancelButtonStyle', cancelButtonStyle, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<ButtonStyle>(
        'confirmButtonStyle',
        confirmButtonStyle,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<BorderSide>(
        'dayPeriodBorderSide',
        dayPeriodBorderSide,
        defaultValue: null,
      ),
    );
    properties.add(ColorProperty('dayPeriodColor', dayPeriodColor, defaultValue: null));
    properties.add(
      DiagnosticsProperty<ShapeBorder>('dayPeriodShape', dayPeriodShape, defaultValue: null),
    );
    properties.add(ColorProperty('dayPeriodTextColor', dayPeriodTextColor, defaultValue: null));
    properties.add(
      DiagnosticsProperty<TextStyle>('dayPeriodTextStyle', dayPeriodTextStyle, defaultValue: null),
    );
    properties.add(ColorProperty('dialBackgroundColor', dialBackgroundColor, defaultValue: null));
    properties.add(ColorProperty('dialHandColor', dialHandColor, defaultValue: null));
    properties.add(ColorProperty('dialTextColor', dialTextColor, defaultValue: null));
    properties.add(
      DiagnosticsProperty<TextStyle?>('dialTextStyle', dialTextStyle, defaultValue: null),
    );
    properties.add(DoubleProperty('elevation', elevation, defaultValue: null));
    properties.add(ColorProperty('entryModeIconColor', entryModeIconColor, defaultValue: null));
    properties.add(
      DiagnosticsProperty<TextStyle>('helpTextStyle', helpTextStyle, defaultValue: null),
    );
    properties.add(ColorProperty('hourMinuteColor', hourMinuteColor, defaultValue: null));
    properties.add(
      DiagnosticsProperty<ShapeBorder>('hourMinuteShape', hourMinuteShape, defaultValue: null),
    );
    properties.add(ColorProperty('hourMinuteTextColor', hourMinuteTextColor, defaultValue: null));
    properties.add(
      DiagnosticsProperty<TextStyle>(
        'hourMinuteTextStyle',
        hourMinuteTextStyle,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<InputDecorationTheme>(
        'inputDecorationTheme',
        inputDecorationTheme,
        defaultValue: null,
      ),
    );
    properties.add(DiagnosticsProperty<EdgeInsetsGeometry>('padding', padding, defaultValue: null));
    properties.add(DiagnosticsProperty<ShapeBorder>('shape', shape, defaultValue: null));
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<Color?>>(
        'timeSelectorSeparatorColor',
        timeSelectorSeparatorColor,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<TextStyle?>>(
        'timeSelectorSeparatorTextStyle',
        timeSelectorSeparatorTextStyle,
        defaultValue: null,
      ),
    );
  }
}

/// An inherited widget that defines the configuration for time pickers
/// displayed using [showTimePicker] in this widget's subtree.
///
/// Values specified here are used for time picker properties that are not
/// given an explicit non-null value.
class TimePickerTheme extends InheritedTheme {
  /// Creates a time picker theme that controls the configurations for
  /// time pickers displayed in its widget subtree.
  const TimePickerTheme({super.key, required this.data, required super.child});

  /// The properties for descendant time picker widgets.
  final TimePickerThemeData data;

  /// The [data] value of the closest [TimePickerTheme] ancestor.
  ///
  /// If there is no ancestor, it returns [ThemeData.timePickerTheme].
  /// Applications can assume that the returned value will not be null.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// TimePickerThemeData theme = TimePickerTheme.of(context);
  /// ```
  static TimePickerThemeData of(BuildContext context) {
    final TimePickerTheme? timePickerTheme =
        context.dependOnInheritedWidgetOfExactType<TimePickerTheme>();
    return timePickerTheme?.data ?? Theme.of(context).timePickerTheme;
  }

  @override
  Widget wrap(BuildContext context, Widget child) {
    return TimePickerTheme(data: data, child: child);
  }

  @override
  bool updateShouldNotify(TimePickerTheme oldWidget) => data != oldWidget.data;
}
