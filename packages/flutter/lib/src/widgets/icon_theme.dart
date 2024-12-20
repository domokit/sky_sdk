// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'icon.dart';
/// @docImport 'image_icon.dart';
library;

import 'package:flutter/foundation.dart';

import 'basic.dart';
import 'framework.dart';
import 'icon_theme_data.dart';
import 'inherited_theme.dart';

// Examples can assume:
// late BuildContext context;

/// Controls the default properties of icons in a widget subtree.
///
/// The icon theme is honored by [Icon] and [ImageIcon] widgets.
class IconTheme extends InheritedTheme {
  /// Creates an icon theme that controls properties of descendant widgets.
  const IconTheme({super.key, required this.data, required super.child});

  /// Creates an icon theme that controls the properties of
  /// descendant widgets, and merges in the current icon theme, if any.
  static Widget merge({Key? key, required IconThemeData data, required Widget child}) {
    return Builder(
      builder: (BuildContext context) {
        return IconTheme(
          key: key,
          data: _getInheritedIconThemeData(context).merge(data),
          child: child,
        );
      },
    );
  }

  /// The set of properties to use for icons in this subtree.
  final IconThemeData data;

  /// The data from the closest instance of this class that encloses the given
  /// context, if any.
  ///
  /// If there is no ambient icon theme, defaults to [IconThemeData.fallback].
  /// The returned [IconThemeData] is concrete (all values are non-null; see
  /// [IconThemeData.isConcrete]). Any properties on the ambient icon theme that
  /// are null get defaulted to the values specified on
  /// [IconThemeData.fallback].
  ///
  /// The [Theme] widget from the `material` library introduces an [IconTheme]
  /// widget set to the [ThemeData.iconTheme], so in a Material Design
  /// application, this will typically default to the icon theme from the
  /// ambient [Theme].
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// IconThemeData theme = IconTheme.of(context);
  /// ```
  static IconThemeData of(BuildContext context) {
    final IconThemeData iconThemeData = _getInheritedIconThemeData(context).resolve(context);
    return iconThemeData.isConcrete
        ? iconThemeData
        : iconThemeData.copyWith(
          size: iconThemeData.size ?? const IconThemeData.fallback().size,
          fill: iconThemeData.fill ?? const IconThemeData.fallback().fill,
          weight: iconThemeData.weight ?? const IconThemeData.fallback().weight,
          grade: iconThemeData.grade ?? const IconThemeData.fallback().grade,
          opticalSize: iconThemeData.opticalSize ?? const IconThemeData.fallback().opticalSize,
          color: iconThemeData.color ?? const IconThemeData.fallback().color,
          opacity: iconThemeData.opacity ?? const IconThemeData.fallback().opacity,
          shadows: iconThemeData.shadows ?? const IconThemeData.fallback().shadows,
          applyTextScaling:
              iconThemeData.applyTextScaling ?? const IconThemeData.fallback().applyTextScaling,
        );
  }

  static IconThemeData _getInheritedIconThemeData(BuildContext context) {
    final IconTheme? iconTheme = context.dependOnInheritedWidgetOfExactType<IconTheme>();
    return iconTheme?.data ?? const IconThemeData.fallback();
  }

  @override
  bool updateShouldNotify(IconTheme oldWidget) => data != oldWidget.data;

  @override
  Widget wrap(BuildContext context, Widget child) {
    return IconTheme(data: data, child: child);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    data.debugFillProperties(properties);
  }
}
