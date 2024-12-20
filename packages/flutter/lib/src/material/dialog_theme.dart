// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dialog.dart';
library;

import 'dart:ui' show lerpDouble;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'theme.dart';

/// Defines a theme for [Dialog] widgets.
///
/// Descendant widgets obtain the current [DialogTheme] object using
/// `DialogTheme.of(context)`. Instances of [DialogTheme] can be customized with
/// [DialogTheme.copyWith].
///
/// [titleTextStyle] and [contentTextStyle] are used in [AlertDialog]s and [SimpleDialog]s.
///
/// See also:
///
///  * [Dialog], a dialog that can be customized using this [DialogTheme].
///  * [AlertDialog], a dialog that can be customized using this [DialogTheme].
///  * [SimpleDialog], a dialog that can be customized using this [DialogTheme].
///  * [ThemeData], which describes the overall theme information for the
///    application.
@immutable
class DialogTheme extends InheritedTheme with Diagnosticable {
  /// Creates a dialog theme that can be used for [ThemeData.dialogTheme].
  const DialogTheme({
    super.key,
    Color? backgroundColor,
    double? elevation,
    Color? shadowColor,
    Color? surfaceTintColor,
    ShapeBorder? shape,
    AlignmentGeometry? alignment,
    Color? iconColor,
    TextStyle? titleTextStyle,
    TextStyle? contentTextStyle,
    EdgeInsetsGeometry? actionsPadding,
    Color? barrierColor,
    EdgeInsets? insetPadding,
    Clip? clipBehavior,
    DialogThemeData? data,
    Widget? child,
  }) : assert(
         data == null ||
             (backgroundColor ??
                     elevation ??
                     shadowColor ??
                     surfaceTintColor ??
                     shape ??
                     alignment ??
                     iconColor ??
                     titleTextStyle ??
                     contentTextStyle ??
                     actionsPadding ??
                     barrierColor ??
                     insetPadding ??
                     clipBehavior) ==
                 null,
       ),
       _data = data,
       _backgroundColor = backgroundColor,
       _elevation = elevation,
       _shadowColor = shadowColor,
       _surfaceTintColor = surfaceTintColor,
       _shape = shape,
       _alignment = alignment,
       _iconColor = iconColor,
       _titleTextStyle = titleTextStyle,
       _contentTextStyle = contentTextStyle,
       _actionsPadding = actionsPadding,
       _barrierColor = barrierColor,
       _insetPadding = insetPadding,
       _clipBehavior = clipBehavior,
       super(child: child ?? const SizedBox());

  final DialogThemeData? _data;
  final Color? _backgroundColor;
  final double? _elevation;
  final Color? _shadowColor;
  final Color? _surfaceTintColor;
  final ShapeBorder? _shape;
  final AlignmentGeometry? _alignment;
  final TextStyle? _titleTextStyle;
  final TextStyle? _contentTextStyle;
  final EdgeInsetsGeometry? _actionsPadding;
  final Color? _iconColor;
  final Color? _barrierColor;
  final EdgeInsets? _insetPadding;
  final Clip? _clipBehavior;

  /// Overrides the default value for [Dialog.backgroundColor].
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [DialogThemeData.backgroundColor] property in [data] instead.
  Color? get backgroundColor => _data != null ? _data.backgroundColor : _backgroundColor;

  /// Overrides the default value for [Dialog.elevation].
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [DialogThemeData.elevation] property in [data] instead.
  double? get elevation => _data != null ? _data.elevation : _elevation;

  /// Overrides the default value for [Dialog.shadowColor].
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [DialogThemeData.shadowColor] property in [data] instead.
  Color? get shadowColor => _data != null ? _data.shadowColor : _shadowColor;

  /// Overrides the default value for [Dialog.surfaceTintColor].
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [DialogThemeData.surfaceTintColor] property in [data] instead.
  Color? get surfaceTintColor => _data != null ? _data.surfaceTintColor : _surfaceTintColor;

  /// Overrides the default value for [Dialog.shape].
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [DialogThemeData.shape] property in [data] instead.
  ShapeBorder? get shape => _data != null ? _data.shape : _shape;

  /// Overrides the default value for [Dialog.alignment].
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [DialogThemeData.alignment] property in [data] instead.
  AlignmentGeometry? get alignment => _data != null ? _data.alignment : _alignment;

  /// Overrides the default value for [DefaultTextStyle] for [SimpleDialog.title] and
  /// [AlertDialog.title].
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [DialogThemeData.titleTextStyle] property in [data] instead.
  TextStyle? get titleTextStyle => _data != null ? _data.titleTextStyle : _titleTextStyle;

  /// Overrides the default value for [DefaultTextStyle] for [SimpleDialog.children] and
  /// [AlertDialog.content].
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [DialogThemeData.contentTextStyle] property in [data] instead.
  TextStyle? get contentTextStyle => _data != null ? _data.contentTextStyle : _contentTextStyle;

  /// Overrides the default value for [AlertDialog.actionsPadding].
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [DialogThemeData.actionsPadding] property in [data] instead.
  EdgeInsetsGeometry? get actionsPadding => _data != null ? _data.actionsPadding : _actionsPadding;

  /// Used to configure the [IconTheme] for the [AlertDialog.icon] widget.
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [DialogThemeData.iconColor] property in [data] instead.
  Color? get iconColor => _data != null ? _data.iconColor : _iconColor;

  /// Overrides the default value for [barrierColor] in [showDialog].
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [DialogThemeData.barrierColor] property in [data] instead.
  Color? get barrierColor => _data != null ? _data.barrierColor : _barrierColor;

  /// Overrides the default value for [Dialog.insetPadding].
  EdgeInsets? get insetPadding => _data != null ? _data.insetPadding : _insetPadding;

  /// Overrides the default value of [Dialog.clipBehavior].
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [DialogThemeData.clipBehavior] property in [data] instead.
  Clip? get clipBehavior => _data != null ? _data.clipBehavior : _clipBehavior;

  /// The properties used for all descendant [Dialog] widgets.
  DialogThemeData get data {
    return _data ??
        DialogThemeData(
          backgroundColor: _backgroundColor,
          elevation: _elevation,
          shadowColor: _shadowColor,
          surfaceTintColor: _surfaceTintColor,
          shape: _shape,
          alignment: _alignment,
          iconColor: _iconColor,
          titleTextStyle: _titleTextStyle,
          contentTextStyle: _contentTextStyle,
          actionsPadding: _actionsPadding,
          barrierColor: _barrierColor,
          insetPadding: _insetPadding,
          clipBehavior: _clipBehavior,
        );
  }

  /// The [ThemeData.dialogTheme] property of the ambient [Theme].
  static DialogThemeData of(BuildContext context) {
    final DialogTheme? dialogTheme = context.dependOnInheritedWidgetOfExactType<DialogTheme>();
    return dialogTheme?.data ?? Theme.of(context).dialogTheme;
  }

  @override
  Widget wrap(BuildContext context, Widget child) {
    return DialogTheme(data: data, child: child);
  }

  @override
  bool updateShouldNotify(DialogTheme oldWidget) => data != oldWidget.data;

  /// Creates a copy of this object but with the given fields replaced with the
  /// new values.
  ///
  /// This method is obsolete and will be deprecated in a future release:
  /// please use the [DialogThemeData.copyWith] instead.
  DialogTheme copyWith({
    Color? backgroundColor,
    double? elevation,
    Color? shadowColor,
    Color? surfaceTintColor,
    ShapeBorder? shape,
    AlignmentGeometry? alignment,
    Color? iconColor,
    TextStyle? titleTextStyle,
    TextStyle? contentTextStyle,
    EdgeInsetsGeometry? actionsPadding,
    Color? barrierColor,
    EdgeInsets? insetPadding,
    Clip? clipBehavior,
  }) {
    return DialogTheme(
      backgroundColor: backgroundColor ?? this.backgroundColor,
      elevation: elevation ?? this.elevation,
      shadowColor: shadowColor ?? this.shadowColor,
      surfaceTintColor: surfaceTintColor ?? this.surfaceTintColor,
      shape: shape ?? this.shape,
      alignment: alignment ?? this.alignment,
      iconColor: iconColor ?? this.iconColor,
      titleTextStyle: titleTextStyle ?? this.titleTextStyle,
      contentTextStyle: contentTextStyle ?? this.contentTextStyle,
      actionsPadding: actionsPadding ?? this.actionsPadding,
      barrierColor: barrierColor ?? this.barrierColor,
      insetPadding: insetPadding ?? this.insetPadding,
      clipBehavior: clipBehavior ?? this.clipBehavior,
    );
  }

  /// Linearly interpolate between two dialog themes.
  ///
  /// {@macro dart.ui.shadow.lerp}
  ///
  /// This method is obsolete and will be deprecated in a future release:
  /// please use the [DialogThemeData.lerp] instead.
  static DialogTheme lerp(DialogTheme? a, DialogTheme? b, double t) {
    if (identical(a, b) && a != null) {
      return a;
    }
    return DialogTheme(
      backgroundColor: Color.lerp(a?.backgroundColor, b?.backgroundColor, t),
      elevation: lerpDouble(a?.elevation, b?.elevation, t),
      shadowColor: Color.lerp(a?.shadowColor, b?.shadowColor, t),
      surfaceTintColor: Color.lerp(a?.surfaceTintColor, b?.surfaceTintColor, t),
      shape: ShapeBorder.lerp(a?.shape, b?.shape, t),
      alignment: AlignmentGeometry.lerp(a?.alignment, b?.alignment, t),
      iconColor: Color.lerp(a?.iconColor, b?.iconColor, t),
      titleTextStyle: TextStyle.lerp(a?.titleTextStyle, b?.titleTextStyle, t),
      contentTextStyle: TextStyle.lerp(a?.contentTextStyle, b?.contentTextStyle, t),
      actionsPadding: EdgeInsetsGeometry.lerp(a?.actionsPadding, b?.actionsPadding, t),
      barrierColor: Color.lerp(a?.barrierColor, b?.barrierColor, t),
      insetPadding: EdgeInsets.lerp(a?.insetPadding, b?.insetPadding, t),
      clipBehavior: t < 0.5 ? a?.clipBehavior : b?.clipBehavior,
    );
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(ColorProperty('backgroundColor', backgroundColor, defaultValue: null));
    properties.add(DoubleProperty('elevation', elevation, defaultValue: null));
    properties.add(ColorProperty('shadowColor', shadowColor, defaultValue: null));
    properties.add(ColorProperty('surfaceTintColor', surfaceTintColor, defaultValue: null));
    properties.add(DiagnosticsProperty<ShapeBorder>('shape', shape, defaultValue: null));
    properties.add(
      DiagnosticsProperty<AlignmentGeometry>('alignment', alignment, defaultValue: null),
    );
    properties.add(ColorProperty('iconColor', iconColor, defaultValue: null));
    properties.add(
      DiagnosticsProperty<TextStyle>('titleTextStyle', titleTextStyle, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<TextStyle>('contentTextStyle', contentTextStyle, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<EdgeInsetsGeometry>('actionsPadding', actionsPadding, defaultValue: null),
    );
    properties.add(ColorProperty('barrierColor', barrierColor, defaultValue: null));
    properties.add(
      DiagnosticsProperty<EdgeInsets>('insetPadding', insetPadding, defaultValue: null),
    );
    properties.add(DiagnosticsProperty<Clip>('clipBehavior', clipBehavior, defaultValue: null));
  }
}

/// Defines default property values for descendant [Dialog] widgets.
///
/// Descendant widgets obtain the current [DialogThemeData] object using
/// `CardTheme.of(context).data`. Instances of [DialogThemeData] can be
/// customized with [DialogThemeData.copyWith].
///
/// Typically a [DialogThemeData] is specified as part of the overall [Theme]
/// with [ThemeData.dialogTheme].
///
/// All [DialogThemeData] properties are `null` by default. When null, the [Dialog]
/// will use the values from [ThemeData] if they exist, otherwise it will
/// provide its own defaults. See the individual [Dialog] properties for details.
///
/// See also:
///
///  * [Dialog], a dialog that can be customized using this [DialogTheme].
///  * [AlertDialog], a dialog that can be customized using this [DialogTheme].
///  * [SimpleDialog], a dialog that can be customized using this [DialogTheme].
///  * [ThemeData], which describes the overall theme information for the
///    application.
@immutable
class DialogThemeData with Diagnosticable {
  /// Creates a dialog theme that can be used for [ThemeData.dialogTheme].
  const DialogThemeData({
    this.backgroundColor,
    this.elevation,
    this.shadowColor,
    this.surfaceTintColor,
    this.shape,
    this.alignment,
    this.iconColor,
    this.titleTextStyle,
    this.contentTextStyle,
    this.actionsPadding,
    this.barrierColor,
    this.insetPadding,
    this.clipBehavior,
  });

  /// Overrides the default value for [Dialog.backgroundColor].
  final Color? backgroundColor;

  /// Overrides the default value for [Dialog.elevation].
  final double? elevation;

  /// Overrides the default value for [Dialog.shadowColor].
  final Color? shadowColor;

  /// Overrides the default value for [Dialog.surfaceTintColor].
  final Color? surfaceTintColor;

  /// Overrides the default value for [Dialog.shape].
  final ShapeBorder? shape;

  /// Overrides the default value for [Dialog.alignment].
  final AlignmentGeometry? alignment;

  /// Overrides the default value for [DefaultTextStyle] for [SimpleDialog.title] and
  /// [AlertDialog.title].
  final TextStyle? titleTextStyle;

  /// Overrides the default value for [DefaultTextStyle] for [SimpleDialog.children] and
  /// [AlertDialog.content].
  final TextStyle? contentTextStyle;

  /// Overrides the default value for [AlertDialog.actionsPadding].
  final EdgeInsetsGeometry? actionsPadding;

  /// Used to configure the [IconTheme] for the [AlertDialog.icon] widget.
  final Color? iconColor;

  /// Overrides the default value for [barrierColor] in [showDialog].
  final Color? barrierColor;

  /// Overrides the default value for [Dialog.insetPadding].
  final EdgeInsets? insetPadding;

  /// Overrides the default value of [Dialog.clipBehavior].
  final Clip? clipBehavior;

  /// Creates a copy of this object but with the given fields replaced with the
  /// new values.
  DialogThemeData copyWith({
    Color? backgroundColor,
    double? elevation,
    Color? shadowColor,
    Color? surfaceTintColor,
    ShapeBorder? shape,
    AlignmentGeometry? alignment,
    Color? iconColor,
    TextStyle? titleTextStyle,
    TextStyle? contentTextStyle,
    EdgeInsetsGeometry? actionsPadding,
    Color? barrierColor,
    EdgeInsets? insetPadding,
    Clip? clipBehavior,
  }) {
    return DialogThemeData(
      backgroundColor: backgroundColor ?? this.backgroundColor,
      elevation: elevation ?? this.elevation,
      shadowColor: shadowColor ?? this.shadowColor,
      surfaceTintColor: surfaceTintColor ?? this.surfaceTintColor,
      shape: shape ?? this.shape,
      alignment: alignment ?? this.alignment,
      iconColor: iconColor ?? this.iconColor,
      titleTextStyle: titleTextStyle ?? this.titleTextStyle,
      contentTextStyle: contentTextStyle ?? this.contentTextStyle,
      actionsPadding: actionsPadding ?? this.actionsPadding,
      barrierColor: barrierColor ?? this.barrierColor,
      insetPadding: insetPadding ?? this.insetPadding,
      clipBehavior: clipBehavior ?? this.clipBehavior,
    );
  }

  /// Linearly interpolate between two [DialogThemeData].
  ///
  /// {@macro dart.ui.shadow.lerp}
  static DialogThemeData lerp(DialogThemeData? a, DialogThemeData? b, double t) {
    if (identical(a, b) && a != null) {
      return a;
    }
    return DialogThemeData(
      backgroundColor: Color.lerp(a?.backgroundColor, b?.backgroundColor, t),
      elevation: lerpDouble(a?.elevation, b?.elevation, t),
      shadowColor: Color.lerp(a?.shadowColor, b?.shadowColor, t),
      surfaceTintColor: Color.lerp(a?.surfaceTintColor, b?.surfaceTintColor, t),
      shape: ShapeBorder.lerp(a?.shape, b?.shape, t),
      alignment: AlignmentGeometry.lerp(a?.alignment, b?.alignment, t),
      iconColor: Color.lerp(a?.iconColor, b?.iconColor, t),
      titleTextStyle: TextStyle.lerp(a?.titleTextStyle, b?.titleTextStyle, t),
      contentTextStyle: TextStyle.lerp(a?.contentTextStyle, b?.contentTextStyle, t),
      actionsPadding: EdgeInsetsGeometry.lerp(a?.actionsPadding, b?.actionsPadding, t),
      barrierColor: Color.lerp(a?.barrierColor, b?.barrierColor, t),
      insetPadding: EdgeInsets.lerp(a?.insetPadding, b?.insetPadding, t),
      clipBehavior: t < 0.5 ? a?.clipBehavior : b?.clipBehavior,
    );
  }

  @override
  int get hashCode => Object.hashAll(<Object?>[
    backgroundColor,
    elevation,
    shadowColor,
    surfaceTintColor,
    shape,
    alignment,
    iconColor,
    titleTextStyle,
    contentTextStyle,
    actionsPadding,
    barrierColor,
    insetPadding,
    clipBehavior,
  ]);

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is DialogThemeData &&
        other.backgroundColor == backgroundColor &&
        other.elevation == elevation &&
        other.shadowColor == shadowColor &&
        other.surfaceTintColor == surfaceTintColor &&
        other.shape == shape &&
        other.alignment == alignment &&
        other.iconColor == iconColor &&
        other.titleTextStyle == titleTextStyle &&
        other.contentTextStyle == contentTextStyle &&
        other.actionsPadding == actionsPadding &&
        other.barrierColor == barrierColor &&
        other.insetPadding == insetPadding &&
        other.clipBehavior == clipBehavior;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(ColorProperty('backgroundColor', backgroundColor, defaultValue: null));
    properties.add(DoubleProperty('elevation', elevation, defaultValue: null));
    properties.add(ColorProperty('shadowColor', shadowColor, defaultValue: null));
    properties.add(ColorProperty('surfaceTintColor', surfaceTintColor, defaultValue: null));
    properties.add(DiagnosticsProperty<ShapeBorder>('shape', shape, defaultValue: null));
    properties.add(
      DiagnosticsProperty<AlignmentGeometry>('alignment', alignment, defaultValue: null),
    );
    properties.add(ColorProperty('iconColor', iconColor, defaultValue: null));
    properties.add(
      DiagnosticsProperty<TextStyle>('titleTextStyle', titleTextStyle, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<TextStyle>('contentTextStyle', contentTextStyle, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<EdgeInsetsGeometry>('actionsPadding', actionsPadding, defaultValue: null),
    );
    properties.add(ColorProperty('barrierColor', barrierColor, defaultValue: null));
    properties.add(
      DiagnosticsProperty<EdgeInsets>('insetPadding', insetPadding, defaultValue: null),
    );
    properties.add(DiagnosticsProperty<Clip>('clipBehavior', clipBehavior, defaultValue: null));
  }
}
