// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'action_buttons.dart';
import 'theme.dart';

// Examples can assume:
// late BuildContext context;

/// A [ActionIconThemeData] that overrides the default icons of
/// [BackButton], [CloseButton], [DrawerButton], and [EndDrawerButton] with
/// [ActionIconTheme.of] or the overall [Theme]'s [ThemeData.actionIconTheme].
@immutable
class ActionIconThemeData with Diagnosticable {
  /// Creates an [ActionIconThemeData].
  ///
  /// The builders [backButtonIconBuilder], [closeButtonIconBuilder],
  /// [drawerButtonIconBuilder], [endDrawerButtonIconBuilder] may be null.
  const ActionIconThemeData({
    this.backButtonIconBuilder,
    this.closeButtonIconBuilder,
    this.drawerButtonIconBuilder,
    this.endDrawerButtonIconBuilder,
  });

  /// Overrides [BackButtonIcon]'s icon.
  ///
  /// If [backButtonIconBuilder] is null, then [BackButtonIcon]
  /// fallbacks to the platform's default back button icon.
  final WidgetBuilder? backButtonIconBuilder;

  /// Overrides [CloseButtonIcon]'s icon.
  ///
  /// If [closeButtonIconBuilder] is null, then [CloseButtonIcon]
  /// fallbacks to the platform's default close button icon.
  final WidgetBuilder? closeButtonIconBuilder;

  /// Overrides [DrawerButtonIcon]'s icon.
  ///
  /// If [drawerButtonIconBuilder] is null, then [DrawerButtonIcon]
  /// fallbacks to the platform's default drawer button icon.
  final WidgetBuilder? drawerButtonIconBuilder;

  /// Overrides [EndDrawerButtonIcon]'s icon.
  ///
  /// If [endDrawerButtonIconBuilder] is null, then [EndDrawerButtonIcon]
  /// fallbacks to the platform's default end drawer button icon.
  final WidgetBuilder? endDrawerButtonIconBuilder;

  /// Creates a copy of this object but with the given fields replaced with the
  /// new values.
  ActionIconThemeData copyWith({
    WidgetBuilder? backButtonIconBuilder,
    WidgetBuilder? closeButtonIconBuilder,
    WidgetBuilder? drawerButtonIconBuilder,
    WidgetBuilder? endDrawerButtonIconBuilder,
  }) {
    return ActionIconThemeData(
      backButtonIconBuilder: backButtonIconBuilder ?? this.backButtonIconBuilder,
      closeButtonIconBuilder: closeButtonIconBuilder ?? this.closeButtonIconBuilder,
      drawerButtonIconBuilder: drawerButtonIconBuilder ?? this.drawerButtonIconBuilder,
      endDrawerButtonIconBuilder: endDrawerButtonIconBuilder ?? this.endDrawerButtonIconBuilder,
    );
  }

  /// Linearly interpolate between two action icon themes.
  static ActionIconThemeData? lerp(ActionIconThemeData? a, ActionIconThemeData? b, double t) {
    if (a == null && b == null) {
      return null;
    }
    return ActionIconThemeData(
      backButtonIconBuilder: t < 0.5 ? a?.backButtonIconBuilder : b?.backButtonIconBuilder,
      closeButtonIconBuilder: t < 0.5 ? a?.closeButtonIconBuilder : b?.closeButtonIconBuilder,
      drawerButtonIconBuilder: t < 0.5 ? a?.drawerButtonIconBuilder : b?.drawerButtonIconBuilder,
      endDrawerButtonIconBuilder:
          t < 0.5 ? a?.endDrawerButtonIconBuilder : b?.endDrawerButtonIconBuilder,
    );
  }

  @override
  int get hashCode {
    final List<Object?> values = <Object?>[
      backButtonIconBuilder,
      closeButtonIconBuilder,
      drawerButtonIconBuilder,
      endDrawerButtonIconBuilder,
    ];
    return Object.hashAll(values);
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is ActionIconThemeData &&
        other.backButtonIconBuilder == backButtonIconBuilder &&
        other.closeButtonIconBuilder == closeButtonIconBuilder &&
        other.drawerButtonIconBuilder == drawerButtonIconBuilder &&
        other.endDrawerButtonIconBuilder == endDrawerButtonIconBuilder;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      DiagnosticsProperty<WidgetBuilder>(
        'backButtonIconBuilder',
        backButtonIconBuilder,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<WidgetBuilder>(
        'closeButtonIconBuilder',
        closeButtonIconBuilder,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<WidgetBuilder>(
        'drawerButtonIconBuilder',
        drawerButtonIconBuilder,
        defaultValue: null,
      ),
    );
    properties.add(
      DiagnosticsProperty<WidgetBuilder>(
        'endDrawerButtonIconBuilder',
        endDrawerButtonIconBuilder,
        defaultValue: null,
      ),
    );
  }
}

/// An inherited widget that overrides the default icon of [BackButtonIcon],
/// [CloseButtonIcon], [DrawerButtonIcon], and [EndDrawerButtonIcon] in this
/// widget's subtree.
///
/// {@tool dartpad}
/// This example shows how to define custom builders for drawer and back
/// buttons.
///
/// ** See code in examples/api/lib/material/action_buttons/action_icon_theme.0.dart **
/// {@end-tool}
class ActionIconTheme extends InheritedTheme {
  /// Creates a theme that overrides the default icon of [BackButtonIcon],
  /// [CloseButtonIcon], [DrawerButtonIcon], and [EndDrawerButtonIcon] in this
  /// widget's subtree.
  const ActionIconTheme({super.key, required this.data, required super.child});

  /// Specifies the default icon overrides for descendant [BackButtonIcon],
  /// [CloseButtonIcon], [DrawerButtonIcon], and [EndDrawerButtonIcon] widgets.
  final ActionIconThemeData data;

  /// The closest instance of this class that encloses the given context.
  ///
  /// If there is no enclosing [ActionIconTheme] widget, then
  /// [ThemeData.actionIconTheme] is used.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// ActionIconThemeData? theme = ActionIconTheme.of(context);
  /// ```
  static ActionIconThemeData? of(BuildContext context) {
    final ActionIconTheme? actionIconTheme =
        context.dependOnInheritedWidgetOfExactType<ActionIconTheme>();
    return actionIconTheme?.data ?? Theme.of(context).actionIconTheme;
  }

  @override
  Widget wrap(BuildContext context, Widget child) {
    return ActionIconTheme(data: data, child: child);
  }

  @override
  bool updateShouldNotify(ActionIconTheme oldWidget) => data != oldWidget.data;
}
