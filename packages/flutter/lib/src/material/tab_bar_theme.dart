// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'ink_well.dart';
import 'material_state.dart';
import 'tabs.dart';
import 'theme.dart';

/// Defines a theme for [TabBar] widgets.
///
/// Descendant widgets obtain the current [TabBarTheme] object using
/// `TabBarTheme.of(context)`.
///
/// See also:
///
///  * [TabBarThemeData], which describes the actual configuration of a switch
///    theme.
@immutable
class TabBarTheme extends InheritedTheme with Diagnosticable {
  /// Creates a tab bar theme that can be used with [ThemeData.tabBarTheme].
  const TabBarTheme({
    super.key,
    Decoration? indicator,
    Color? indicatorColor,
    TabBarIndicatorSize? indicatorSize,
    Color? dividerColor,
    double? dividerHeight,
    Color? labelColor,
    EdgeInsetsGeometry? labelPadding,
    TextStyle? labelStyle,
    Color? unselectedLabelColor,
    TextStyle? unselectedLabelStyle,
    WidgetStateProperty<Color?>? overlayColor,
    InteractiveInkFeatureFactory? splashFactory,
    WidgetStateProperty<MouseCursor?>? mouseCursor,
    TabAlignment? tabAlignment,
    TextScaler? textScaler,
    TabIndicatorAnimation? indicatorAnimation,
    TabBarThemeData? data,
    Widget? child,
  }) : assert(
         data == null ||
             (indicator ??
                     indicatorColor ??
                     indicatorSize ??
                     dividerColor ??
                     dividerHeight ??
                     labelColor ??
                     labelPadding ??
                     labelStyle ??
                     unselectedLabelColor ??
                     unselectedLabelStyle ??
                     overlayColor ??
                     splashFactory ??
                     mouseCursor ??
                     tabAlignment ??
                     textScaler ??
                     indicatorAnimation) ==
                 null,
       ),
       _indicator = indicator,
       _indicatorColor = indicatorColor,
       _indicatorSize = indicatorSize,
       _dividerColor = dividerColor,
       _dividerHeight = dividerHeight,
       _labelColor = labelColor,
       _labelPadding = labelPadding,
       _labelStyle = labelStyle,
       _unselectedLabelColor = unselectedLabelColor,
       _unselectedLabelStyle = unselectedLabelStyle,
       _overlayColor = overlayColor,
       _splashFactory = splashFactory,
       _mouseCursor = mouseCursor,
       _tabAlignment = tabAlignment,
       _textScaler = textScaler,
       _indicatorAnimation = indicatorAnimation,
       _data = data,
       super(child: child ?? const SizedBox());

  final TabBarThemeData? _data;
  final Decoration? _indicator;
  final Color? _indicatorColor;
  final TabBarIndicatorSize? _indicatorSize;
  final Color? _dividerColor;
  final double? _dividerHeight;
  final Color? _labelColor;
  final EdgeInsetsGeometry? _labelPadding;
  final TextStyle? _labelStyle;
  final Color? _unselectedLabelColor;
  final TextStyle? _unselectedLabelStyle;
  final MaterialStateProperty<Color?>? _overlayColor;
  final InteractiveInkFeatureFactory? _splashFactory;
  final MaterialStateProperty<MouseCursor?>? _mouseCursor;
  final TabAlignment? _tabAlignment;
  final TextScaler? _textScaler;
  final TabIndicatorAnimation? _indicatorAnimation;

  /// Overrides the default value for [TabBar.indicator].
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [TabBarThemeData.indicator] property in [data] instead.
  Decoration? get indicator => _data != null ? _data.indicator : _indicator;

  /// Overrides the default value for [TabBar.indicatorColor].
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [TabBarThemeData.indicatorColor] property in [data] instead.
  Color? get indicatorColor => _data != null ? _data.indicatorColor : _indicatorColor;

  /// Overrides the default value for [TabBar.indicatorSize].
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [TabBarThemeData.indicatorSize] property in [data] instead.
  TabBarIndicatorSize? get indicatorSize => _data != null ? _data.indicatorSize : _indicatorSize;

  /// Overrides the default value for [TabBar.dividerColor].
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [TabBarThemeData.dividerColor] property in [data] instead.
  Color? get dividerColor => _data != null ? _data.dividerColor : _dividerColor;

  /// Overrides the default value for [TabBar.dividerHeight].
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [TabBarThemeData.dividerHeight] property in [data] instead.
  double? get dividerHeight => _data != null ? _data.dividerHeight : _dividerHeight;

  /// Overrides the default value for [TabBar.labelColor].
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [TabBarThemeData.labelColor] property in [data] instead.
  Color? get labelColor => _data != null ? _data.labelColor : _labelColor;

  /// Overrides the default value for [TabBar.labelPadding].
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [TabBarThemeData.labelPadding] property in [data] instead.
  EdgeInsetsGeometry? get labelPadding => _data != null ? _data.labelPadding : _labelPadding;

  /// Overrides the default value for [TabBar.labelStyle].
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [TabBarThemeData.labelStyle] property in [data] instead.
  TextStyle? get labelStyle => _data != null ? _data.labelStyle : _labelStyle;

  /// Overrides the default value for [TabBar.unselectedLabelColor].
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [TabBarThemeData.unselectedLabelColor] property in [data] instead.
  Color? get unselectedLabelColor =>
      _data != null ? _data.unselectedLabelColor : _unselectedLabelColor;

  /// Overrides the default value for [TabBar.unselectedLabelStyle].
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [TabBarThemeData.unselectedLabelStyle] property in [data] instead.
  TextStyle? get unselectedLabelStyle =>
      _data != null ? _data.unselectedLabelStyle : _unselectedLabelStyle;

  /// Overrides the default value for [TabBar.overlayColor].
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [TabBarThemeData.overlayColor] property in [data] instead.
  MaterialStateProperty<Color?>? get overlayColor =>
      _data != null ? _data.overlayColor : _overlayColor;

  /// Overrides the default value for [TabBar.splashFactory].
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [TabBarThemeData.splashFactory] property in [data] instead.
  InteractiveInkFeatureFactory? get splashFactory =>
      _data != null ? _data.splashFactory : _splashFactory;

  /// Overrides the default value of [TabBar.mouseCursor].
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [TabBarThemeData.mouseCursor] property in [data] instead.
  MaterialStateProperty<MouseCursor?>? get mouseCursor =>
      _data != null ? _data.mouseCursor : _mouseCursor;

  /// Overrides the default value for [TabBar.tabAlignment].
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [TabBarThemeData.tabAlignment] property in [data] instead.
  TabAlignment? get tabAlignment => _data != null ? _data.tabAlignment : _tabAlignment;

  /// Overrides the default value for [TabBar.textScaler].
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [TabBarThemeData.textScaler] property in [data] instead.
  TextScaler? get textScaler => _data != null ? _data.textScaler : _textScaler;

  /// Overrides the default value for [TabBar.indicatorAnimation].
  ///
  /// This property is obsolete and will be deprecated in a future release:
  /// please use the [TabBarThemeData.indicatorAnimation] property in [data] instead.
  TabIndicatorAnimation? get indicatorAnimation =>
      _data != null ? _data.indicatorAnimation : _indicatorAnimation;

  /// The properties used for all descendant [TabBar] widgets.
  TabBarThemeData get data =>
      _data ??
      TabBarThemeData(
        indicator: _indicator,
        indicatorColor: _indicatorColor,
        indicatorSize: _indicatorSize,
        dividerColor: _dividerColor,
        dividerHeight: _dividerHeight,
        labelColor: _labelColor,
        labelPadding: _labelPadding,
        labelStyle: _labelStyle,
        unselectedLabelColor: _unselectedLabelColor,
        unselectedLabelStyle: _unselectedLabelStyle,
        overlayColor: _overlayColor,
        splashFactory: _splashFactory,
        mouseCursor: _mouseCursor,
        tabAlignment: _tabAlignment,
        textScaler: _textScaler,
        indicatorAnimation: _indicatorAnimation,
      );

  /// Creates a copy of this object but with the given fields replaced with the
  /// new values.
  ///
  /// This method is obsolete and will be deprecated in a future release:
  /// please use the [TabBarThemeData.copyWith] instead.
  TabBarTheme copyWith({
    Decoration? indicator,
    Color? indicatorColor,
    TabBarIndicatorSize? indicatorSize,
    Color? dividerColor,
    double? dividerHeight,
    Color? labelColor,
    EdgeInsetsGeometry? labelPadding,
    TextStyle? labelStyle,
    Color? unselectedLabelColor,
    TextStyle? unselectedLabelStyle,
    MaterialStateProperty<Color?>? overlayColor,
    InteractiveInkFeatureFactory? splashFactory,
    MaterialStateProperty<MouseCursor?>? mouseCursor,
    TabAlignment? tabAlignment,
    TextScaler? textScaler,
    TabIndicatorAnimation? indicatorAnimation,
  }) {
    return TabBarTheme(
      indicator: indicator ?? this.indicator,
      indicatorColor: indicatorColor ?? this.indicatorColor,
      indicatorSize: indicatorSize ?? this.indicatorSize,
      dividerColor: dividerColor ?? this.dividerColor,
      dividerHeight: dividerHeight ?? this.dividerHeight,
      labelColor: labelColor ?? this.labelColor,
      labelPadding: labelPadding ?? this.labelPadding,
      labelStyle: labelStyle ?? this.labelStyle,
      unselectedLabelColor: unselectedLabelColor ?? this.unselectedLabelColor,
      unselectedLabelStyle: unselectedLabelStyle ?? this.unselectedLabelStyle,
      overlayColor: overlayColor ?? this.overlayColor,
      splashFactory: splashFactory ?? this.splashFactory,
      mouseCursor: mouseCursor ?? this.mouseCursor,
      tabAlignment: tabAlignment ?? this.tabAlignment,
      textScaler: textScaler ?? this.textScaler,
      indicatorAnimation: indicatorAnimation ?? this.indicatorAnimation,
    );
  }

  /// Returns the closest [TabBarTheme] instance given the build context.
  static TabBarThemeData of(BuildContext context) {
    final TabBarTheme? tabBarTheme = context.dependOnInheritedWidgetOfExactType<TabBarTheme>();
    return tabBarTheme?.data ?? Theme.of(context).tabBarTheme;
  }

  /// Linearly interpolate between two tab bar themes.
  ///
  /// {@macro dart.ui.shadow.lerp}
  ///
  /// This method is obsolete and will be deprecated in a future release:
  /// please use the [TabBarThemeData.lerp] instead.
  static TabBarTheme lerp(TabBarTheme a, TabBarTheme b, double t) {
    if (identical(a, b)) {
      return a;
    }
    return TabBarTheme(
      indicator: Decoration.lerp(a.indicator, b.indicator, t),
      indicatorColor: Color.lerp(a.indicatorColor, b.indicatorColor, t),
      indicatorSize: t < 0.5 ? a.indicatorSize : b.indicatorSize,
      dividerColor: Color.lerp(a.dividerColor, b.dividerColor, t),
      dividerHeight: t < 0.5 ? a.dividerHeight : b.dividerHeight,
      labelColor: Color.lerp(a.labelColor, b.labelColor, t),
      labelPadding: EdgeInsetsGeometry.lerp(a.labelPadding, b.labelPadding, t),
      labelStyle: TextStyle.lerp(a.labelStyle, b.labelStyle, t),
      unselectedLabelColor: Color.lerp(a.unselectedLabelColor, b.unselectedLabelColor, t),
      unselectedLabelStyle: TextStyle.lerp(a.unselectedLabelStyle, b.unselectedLabelStyle, t),
      overlayColor: MaterialStateProperty.lerp<Color?>(
        a.overlayColor,
        b.overlayColor,
        t,
        Color.lerp,
      ),
      splashFactory: t < 0.5 ? a.splashFactory : b.splashFactory,
      mouseCursor: t < 0.5 ? a.mouseCursor : b.mouseCursor,
      tabAlignment: t < 0.5 ? a.tabAlignment : b.tabAlignment,
      textScaler: t < 0.5 ? a.textScaler : b.textScaler,
      indicatorAnimation: t < 0.5 ? a.indicatorAnimation : b.indicatorAnimation,
    );
  }

  @override
  bool updateShouldNotify(TabBarTheme oldWidget) => data != oldWidget.data;

  @override
  Widget wrap(BuildContext context, Widget child) {
    return TabBarTheme(data: data, child: child);
  }
}

/// Defines default property values for descendant [TabBar] widgets.
///
/// Descendant widgets obtain the current [TabBarThemeData] object using
/// `TabBarTheme.of(context).data`. Instances of [TabBarThemeData] can be
/// customized with [TabBarThemeData.copyWith].
///
/// Typically a [TabBarThemeData] is specified as part of the overall [Theme]
/// with [ThemeData.tabBarTheme].
///
/// All [TabBarThemeData] properties are `null` by default. When null, the [TabBar]
/// will use the values from [ThemeData] if they exist, otherwise it will
/// provide its own defaults. See the individual [TabBar] properties for details.
///
/// See also:
///
///  * [TabBar], which displays a row of tabs.
///  * [ThemeData], which describes the overall theme information for the
///    application.
@immutable
class TabBarThemeData with Diagnosticable {
  /// Creates a tab bar theme that can be used with [ThemeData.tabBarTheme].
  const TabBarThemeData({
    this.indicator,
    this.indicatorColor,
    this.indicatorSize,
    this.dividerColor,
    this.dividerHeight,
    this.labelColor,
    this.labelPadding,
    this.labelStyle,
    this.unselectedLabelColor,
    this.unselectedLabelStyle,
    this.overlayColor,
    this.splashFactory,
    this.mouseCursor,
    this.tabAlignment,
    this.textScaler,
    this.indicatorAnimation,
    this.splashBorderRadius,
  });

  /// Overrides the default value for [TabBar.indicator].
  final Decoration? indicator;

  /// Overrides the default value for [TabBar.indicatorColor].
  final Color? indicatorColor;

  /// Overrides the default value for [TabBar.indicatorSize].
  final TabBarIndicatorSize? indicatorSize;

  /// Overrides the default value for [TabBar.dividerColor].
  final Color? dividerColor;

  /// Overrides the default value for [TabBar.dividerHeight].
  final double? dividerHeight;

  /// Overrides the default value for [TabBar.labelColor].
  ///
  /// If [labelColor] is a [WidgetStateColor], then the effective color will
  /// depend on the [WidgetState.selected] state, i.e. if the [Tab] is
  /// selected or not. In case of unselected state, this [WidgetStateColor]'s
  /// resolved color will be used even if [TabBar.unselectedLabelColor] or
  /// [unselectedLabelColor] is non-null.
  final Color? labelColor;

  /// Overrides the default value for [TabBar.labelPadding].
  ///
  /// If there are few tabs with both icon and text and few
  /// tabs with only icon or text, this padding is vertically
  /// adjusted to provide uniform padding to all tabs.
  final EdgeInsetsGeometry? labelPadding;

  /// Overrides the default value for [TabBar.labelStyle].
  final TextStyle? labelStyle;

  /// Overrides the default value for [TabBar.unselectedLabelColor].
  final Color? unselectedLabelColor;

  /// Overrides the default value for [TabBar.unselectedLabelStyle].
  final TextStyle? unselectedLabelStyle;

  /// Overrides the default value for [TabBar.overlayColor].
  final MaterialStateProperty<Color?>? overlayColor;

  /// Overrides the default value for [TabBar.splashFactory].
  final InteractiveInkFeatureFactory? splashFactory;

  /// {@macro flutter.material.tabs.mouseCursor}
  ///
  /// If specified, overrides the default value of [TabBar.mouseCursor].
  final MaterialStateProperty<MouseCursor?>? mouseCursor;

  /// Overrides the default value for [TabBar.tabAlignment].
  final TabAlignment? tabAlignment;

  /// Overrides the default value for [TabBar.textScaler].
  final TextScaler? textScaler;

  /// Overrides the default value for [TabBar.indicatorAnimation].
  final TabIndicatorAnimation? indicatorAnimation;

  /// Defines the clipping radius of splashes that extend outside the bounds of the tab.
  final BorderRadius? splashBorderRadius;

  /// Creates a copy of this object but with the given fields replaced with the
  /// new values.
  TabBarThemeData copyWith({
    Decoration? indicator,
    Color? indicatorColor,
    TabBarIndicatorSize? indicatorSize,
    Color? dividerColor,
    double? dividerHeight,
    Color? labelColor,
    EdgeInsetsGeometry? labelPadding,
    TextStyle? labelStyle,
    Color? unselectedLabelColor,
    TextStyle? unselectedLabelStyle,
    MaterialStateProperty<Color?>? overlayColor,
    InteractiveInkFeatureFactory? splashFactory,
    MaterialStateProperty<MouseCursor?>? mouseCursor,
    TabAlignment? tabAlignment,
    TextScaler? textScaler,
    TabIndicatorAnimation? indicatorAnimation,
    BorderRadius? splashBorderRadius,
  }) {
    return TabBarThemeData(
      indicator: indicator ?? this.indicator,
      indicatorColor: indicatorColor ?? this.indicatorColor,
      indicatorSize: indicatorSize ?? this.indicatorSize,
      dividerColor: dividerColor ?? this.dividerColor,
      dividerHeight: dividerHeight ?? this.dividerHeight,
      labelColor: labelColor ?? this.labelColor,
      labelPadding: labelPadding ?? this.labelPadding,
      labelStyle: labelStyle ?? this.labelStyle,
      unselectedLabelColor: unselectedLabelColor ?? this.unselectedLabelColor,
      unselectedLabelStyle: unselectedLabelStyle ?? this.unselectedLabelStyle,
      overlayColor: overlayColor ?? this.overlayColor,
      splashFactory: splashFactory ?? this.splashFactory,
      mouseCursor: mouseCursor ?? this.mouseCursor,
      tabAlignment: tabAlignment ?? this.tabAlignment,
      textScaler: textScaler ?? this.textScaler,
      indicatorAnimation: indicatorAnimation ?? this.indicatorAnimation,
      splashBorderRadius: splashBorderRadius ?? this.splashBorderRadius,
    );
  }

  /// Linearly interpolate between two tab bar themes.
  ///
  /// {@macro dart.ui.shadow.lerp}
  static TabBarThemeData lerp(TabBarThemeData a, TabBarThemeData b, double t) {
    if (identical(a, b)) {
      return a;
    }
    return TabBarThemeData(
      indicator: Decoration.lerp(a.indicator, b.indicator, t),
      indicatorColor: Color.lerp(a.indicatorColor, b.indicatorColor, t),
      indicatorSize: t < 0.5 ? a.indicatorSize : b.indicatorSize,
      dividerColor: Color.lerp(a.dividerColor, b.dividerColor, t),
      dividerHeight: t < 0.5 ? a.dividerHeight : b.dividerHeight,
      labelColor: Color.lerp(a.labelColor, b.labelColor, t),
      labelPadding: EdgeInsetsGeometry.lerp(a.labelPadding, b.labelPadding, t),
      labelStyle: TextStyle.lerp(a.labelStyle, b.labelStyle, t),
      unselectedLabelColor: Color.lerp(a.unselectedLabelColor, b.unselectedLabelColor, t),
      unselectedLabelStyle: TextStyle.lerp(a.unselectedLabelStyle, b.unselectedLabelStyle, t),
      overlayColor: MaterialStateProperty.lerp<Color?>(
        a.overlayColor,
        b.overlayColor,
        t,
        Color.lerp,
      ),
      splashFactory: t < 0.5 ? a.splashFactory : b.splashFactory,
      mouseCursor: t < 0.5 ? a.mouseCursor : b.mouseCursor,
      tabAlignment: t < 0.5 ? a.tabAlignment : b.tabAlignment,
      textScaler: t < 0.5 ? a.textScaler : b.textScaler,
      indicatorAnimation: t < 0.5 ? a.indicatorAnimation : b.indicatorAnimation,
      splashBorderRadius: BorderRadius.lerp(a.splashBorderRadius, a.splashBorderRadius, t),
    );
  }

  @override
  int get hashCode => Object.hash(
    indicator,
    indicatorColor,
    indicatorSize,
    dividerColor,
    dividerHeight,
    labelColor,
    labelPadding,
    labelStyle,
    unselectedLabelColor,
    unselectedLabelStyle,
    overlayColor,
    splashFactory,
    mouseCursor,
    tabAlignment,
    textScaler,
    indicatorAnimation,
    splashBorderRadius,
  );

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is TabBarThemeData &&
        other.indicator == indicator &&
        other.indicatorColor == indicatorColor &&
        other.indicatorSize == indicatorSize &&
        other.dividerColor == dividerColor &&
        other.dividerHeight == dividerHeight &&
        other.labelColor == labelColor &&
        other.labelPadding == labelPadding &&
        other.labelStyle == labelStyle &&
        other.unselectedLabelColor == unselectedLabelColor &&
        other.unselectedLabelStyle == unselectedLabelStyle &&
        other.overlayColor == overlayColor &&
        other.splashFactory == splashFactory &&
        other.mouseCursor == mouseCursor &&
        other.tabAlignment == tabAlignment &&
        other.textScaler == textScaler &&
        other.indicatorAnimation == indicatorAnimation &&
        other.splashBorderRadius == splashBorderRadius;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<Decoration?>('indicator', indicator, defaultValue: null));
    properties.add(
      DiagnosticsProperty<Color?>('indicatorColor', indicatorColor, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<TabBarIndicatorSize?>('indicatorSize', indicatorSize, defaultValue: null),
    );
    properties.add(DiagnosticsProperty<Color?>('dividerColor', dividerColor, defaultValue: null));
    properties.add(
      DiagnosticsProperty<double?>('dividerHeight', dividerHeight, defaultValue: null),
    );
    properties.add(DiagnosticsProperty<Color?>('labelColor', labelColor, defaultValue: null));
    properties.add(
      DiagnosticsProperty<EdgeInsetsGeometry?>('labelPadding', labelPadding, defaultValue: null),
    );
    properties.add(DiagnosticsProperty<TextStyle?>('labelStyle', labelStyle, defaultValue: null));
    properties.add(
      DiagnosticsProperty<Color?>('unselectedLabelColor', unselectedLabelColor, defaultValue: null),
    );
    properties.add(
      DiagnosticsProperty<TextStyle?>(
        'unselectedLabelStyle',
        unselectedLabelStyle,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<Color?>?>(
        'overlayColor',
        overlayColor,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<InteractiveInkFeatureFactory?>(
        'splashFactory',
        splashFactory,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<MaterialStateProperty<MouseCursor?>?>(
        'mouseCursor',
        mouseCursor,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<TabAlignment?>('tabAlignment', tabAlignment, defaultValue: null),
    );
    properties.add(DiagnosticsProperty<TextScaler?>('textScaler', textScaler, defaultValue: null));
    properties.add(
      DiagnosticsProperty<TabIndicatorAnimation?>(
        'indicatorAnimation',
        indicatorAnimation,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<BorderRadius?>(
        'splashBorderRadius',
        splashBorderRadius,
        defaultValue: null,
      ),
    );
  }
}
