// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'app.dart';
/// @docImport 'color_scheme.dart';
/// @docImport 'text_theme.dart';
library;

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';

import 'material_localizations.dart';
import 'theme_data.dart';
import 'typography.dart';

export 'theme_data.dart' show Brightness, ThemeData;

/// The duration over which theme changes animate by default.
const Duration kThemeAnimationDuration = Duration(milliseconds: 200);

/// Applies a theme to descendant widgets.
///
/// A theme describes the colors and typographic choices of an application.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=oTvQDJOBXmM}
///
/// Descendant widgets obtain the current theme's [ThemeData] object using
/// [Theme.of]. When a widget uses [Theme.of], it is automatically rebuilt if
/// the theme later changes, so that the changes can be applied.
///
/// The [Theme] widget implies an [IconTheme] widget, set to the value of the
/// [ThemeData.iconTheme] of the [data] for the [Theme].
///
/// See also:
///
///  * [ThemeData], which describes the actual configuration of a theme.
///  * [AnimatedTheme], which animates the [ThemeData] when it changes rather
///    than changing the theme all at once.
///  * [MaterialApp], which includes an [AnimatedTheme] widget configured via
///    the [MaterialApp.theme] argument.
class Theme extends StatelessWidget {
  /// Applies the given theme [data] to [child].
  const Theme({super.key, required this.data, required this.child});

  /// Specifies the color and typography values for descendant widgets.
  final ThemeData data;

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  static final ThemeData _kFallbackTheme = ThemeData.fallback();

  /// The data from the closest [Theme] instance that encloses the given
  /// context.
  ///
  /// If the given context is enclosed in a [Localizations] widget providing
  /// [MaterialLocalizations], the returned data is localized according to the
  /// nearest available [MaterialLocalizations].
  ///
  /// Defaults to [ThemeData.fallback] if there is no [Theme] in the given
  /// build context.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// @override
  /// Widget build(BuildContext context) {
  ///   return Text(
  ///     'Example',
  ///     style: Theme.of(context).textTheme.titleLarge,
  ///   );
  /// }
  /// ```
  ///
  /// When the [Theme] is actually created in the same `build` function
  /// (possibly indirectly, e.g. as part of a [MaterialApp]), the `context`
  /// argument to the `build` function can't be used to find the [Theme] (since
  /// it's "above" the widget being returned). In such cases, the following
  /// technique with a [Builder] can be used to provide a new scope with a
  /// [BuildContext] that is "under" the [Theme]:
  ///
  /// ```dart
  /// @override
  /// Widget build(BuildContext context) {
  ///   return MaterialApp(
  ///     theme: ThemeData.light(),
  ///     home: Builder(
  ///       // Create an inner BuildContext so that we can refer to
  ///       // the Theme with Theme.of().
  ///       builder: (BuildContext context) {
  ///         return Center(
  ///           child: Text(
  ///             'Example',
  ///             style: Theme.of(context).textTheme.titleLarge,
  ///           ),
  ///         );
  ///       },
  ///     ),
  ///   );
  /// }
  /// ```
  ///
  /// See also:
  ///
  /// * [ColorScheme.of], a convenience method that returns [ThemeData.colorScheme]
  ///   from the closest [Theme] ancestor. (equivalent to `Theme.of(context).colorScheme`).
  /// * [TextTheme.of], a convenience method that returns [ThemeData.textTheme]
  ///   from the closest [Theme] ancestor. (equivalent to `Theme.of(context).textTheme`).
  /// * [IconTheme.of], that returns [ThemeData.iconTheme] from the closest [Theme] or
  ///   [IconThemeData.fallback] if there is no [IconTheme] ancestor.
  static ThemeData of(BuildContext context) {
    final _InheritedTheme? inheritedTheme =
        context.dependOnInheritedWidgetOfExactType<_InheritedTheme>();
    final MaterialLocalizations? localizations = Localizations.of<MaterialLocalizations>(
      context,
      MaterialLocalizations,
    );
    final ScriptCategory category = localizations?.scriptCategory ?? ScriptCategory.englishLike;
    final InheritedCupertinoTheme? inheritedCupertinoTheme =
        context.dependOnInheritedWidgetOfExactType<InheritedCupertinoTheme>();
    final ThemeData theme =
        inheritedTheme?.theme.data ??
        (inheritedCupertinoTheme != null
            ? CupertinoBasedMaterialThemeData(
              themeData: inheritedCupertinoTheme.theme.data,
            ).materialTheme
            : _kFallbackTheme);
    return ThemeData.localize(theme, theme.typography.geometryThemeFor(category));
  }

  // The inherited themes in widgets library can not infer their values from
  // Theme in material library. Wraps the child with these inherited themes to
  // overrides their values directly.
  Widget _wrapsWidgetThemes(BuildContext context, Widget child) {
    final DefaultSelectionStyle selectionStyle = DefaultSelectionStyle.of(context);
    return IconTheme(
      data: data.iconTheme,
      child: DefaultSelectionStyle(
        selectionColor: data.textSelectionTheme.selectionColor ?? selectionStyle.selectionColor,
        cursorColor: data.textSelectionTheme.cursorColor ?? selectionStyle.cursorColor,
        child: child,
      ),
    );
  }

  CupertinoThemeData _inheritedCupertinoThemeData(BuildContext context) {
    final InheritedCupertinoTheme? inheritedTheme =
        context.dependOnInheritedWidgetOfExactType<InheritedCupertinoTheme>();
    return (inheritedTheme?.theme.data ?? MaterialBasedCupertinoThemeData(materialTheme: data))
        .resolveFrom(context);
  }

  @override
  Widget build(BuildContext context) {
    return _InheritedTheme(
      theme: this,
      child: CupertinoTheme(
        // If a CupertinoThemeData doesn't exist, we're using a
        // MaterialBasedCupertinoThemeData here instead of a CupertinoThemeData
        // because it defers some properties to the Material ThemeData.
        data: _inheritedCupertinoThemeData(context),
        child: _wrapsWidgetThemes(context, child),
      ),
    );
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<ThemeData>('data', data, showName: false));
  }
}

class _InheritedTheme extends InheritedTheme {
  const _InheritedTheme({required this.theme, required super.child});

  final Theme theme;

  @override
  Widget wrap(BuildContext context, Widget child) {
    return Theme(data: theme.data, child: child);
  }

  @override
  bool updateShouldNotify(_InheritedTheme old) => theme.data != old.theme.data;
}

/// An interpolation between two [ThemeData]s.
///
/// This class specializes the interpolation of [Tween<ThemeData>] to call the
/// [ThemeData.lerp] method.
///
/// See [Tween] for a discussion on how to use interpolation objects.
class ThemeDataTween extends Tween<ThemeData> {
  /// Creates a [ThemeData] tween.
  ///
  /// The [begin] and [end] properties must be non-null before the tween is
  /// first used, but the arguments can be null if the values are going to be
  /// filled in later.
  ThemeDataTween({super.begin, super.end});

  @override
  ThemeData lerp(double t) => ThemeData.lerp(begin!, end!, t);
}

/// Animated version of [Theme] which automatically transitions the colors,
/// etc, over a given duration whenever the given theme changes.
///
/// Here's an illustration of what using this widget looks like, using a [curve]
/// of [Curves.elasticInOut].
/// {@animation 250 266 https://flutter.github.io/assets-for-api-docs/assets/widgets/animated_theme.mp4}
///
/// See also:
///
///  * [Theme], which [AnimatedTheme] uses to actually apply the interpolated
///    theme.
///  * [ThemeData], which describes the actual configuration of a theme.
///  * [MaterialApp], which includes an [AnimatedTheme] widget configured via
///    the [MaterialApp.theme] argument.
class AnimatedTheme extends ImplicitlyAnimatedWidget {
  /// Creates an animated theme.
  ///
  /// By default, the theme transition uses a linear curve.
  const AnimatedTheme({
    super.key,
    required this.data,
    super.curve,
    super.duration = kThemeAnimationDuration,
    super.onEnd,
    required this.child,
  });

  /// Specifies the color and typography values for descendant widgets.
  final ThemeData data;

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  @override
  AnimatedWidgetBaseState<AnimatedTheme> createState() => _AnimatedThemeState();
}

class _AnimatedThemeState extends AnimatedWidgetBaseState<AnimatedTheme> {
  ThemeDataTween? _data;

  @override
  void forEachTween(TweenVisitor<dynamic> visitor) {
    _data =
        visitor(_data, widget.data, (dynamic value) => ThemeDataTween(begin: value as ThemeData))!
            as ThemeDataTween;
  }

  @override
  Widget build(BuildContext context) {
    return Theme(data: _data!.evaluate(animation), child: widget.child);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder description) {
    super.debugFillProperties(description);
    description.add(
      DiagnosticsProperty<ThemeDataTween>('data', _data, showName: false, defaultValue: null),
    );
  }
}
