// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'template.dart';

class AppBarTemplate extends TokenTemplate {
  const AppBarTemplate(super.blockName, super.fileName, super.tokens)
    : super(colorSchemePrefix: '_colors.', textThemePrefix: '_textTheme.');

  @override
  String generate() => '''
class _${blockName}DefaultsM3 extends AppBarTheme {
  _${blockName}DefaultsM3(this.context)
    : super(
      elevation: ${elevation('md.comp.top-app-bar.small.container')},
      scrolledUnderElevation: ${elevation('md.comp.top-app-bar.small.on-scroll.container')},
      titleSpacing: NavigationToolbar.kMiddleSpacing,
      toolbarHeight: ${getToken('md.comp.top-app-bar.small.container.height')},
    );

  final BuildContext context;
  late final ThemeData _theme = Theme.of(context);
  late final ColorScheme _colors = _theme.colorScheme;
  late final TextTheme _textTheme = _theme.textTheme;

  @override
  Color? get backgroundColor => ${componentColor('md.comp.top-app-bar.small.container')};

  @override
  Color? get foregroundColor => ${color('md.comp.top-app-bar.small.headline.color')};

  @override
  Color? get shadowColor => ${colorOrTransparent('md.comp.top-app-bar.small.container.shadow-color')};

  @override
  Color? get surfaceTintColor => ${colorOrTransparent('md.comp.top-app-bar.small.container.surface-tint-layer.color')};

  @override
  IconThemeData? get iconTheme => IconThemeData(
    color: ${componentColor('md.comp.top-app-bar.small.leading-icon')},
    size: ${getToken('md.comp.top-app-bar.small.leading-icon.size')},
  );

  @override
  IconThemeData? get actionsIconTheme => IconThemeData(
    color: ${componentColor('md.comp.top-app-bar.small.trailing-icon')},
    size: ${getToken('md.comp.top-app-bar.small.trailing-icon.size')},
  );

  @override
  TextStyle? get toolbarTextStyle => _textTheme.bodyMedium;

  @override
  TextStyle? get titleTextStyle => ${textStyle('md.comp.top-app-bar.small.headline')};

  // TODO(Craftplacer): Consider using EdgeInsets.only(right: 8.0) instead of
  // EdgeInsets.zero for Material 3 in the future,
  // https://github.com/flutter/flutter/issues/155747
  @override
  EdgeInsets? get actionsPadding => EdgeInsets.zero;
}

// Variant configuration
class _MediumScrollUnderFlexibleConfig with _ScrollUnderFlexibleConfig {
  _MediumScrollUnderFlexibleConfig(this.context);

  final BuildContext context;
  late final ThemeData _theme = Theme.of(context);
  late final ColorScheme _colors = _theme.colorScheme;
  late final TextTheme _textTheme = _theme.textTheme;

  static const double collapsedHeight = ${getToken('md.comp.top-app-bar.small.container.height')};
  static const double expandedHeight = ${getToken('md.comp.top-app-bar.medium.container.height')};

  @override
  TextStyle? get collapsedTextStyle =>
    ${textStyle('md.comp.top-app-bar.small.headline')}?.apply(color: ${color('md.comp.top-app-bar.small.headline.color')});

  @override
  TextStyle? get expandedTextStyle =>
    ${textStyle('md.comp.top-app-bar.medium.headline')}?.apply(color: ${color('md.comp.top-app-bar.medium.headline.color')});

  @override
  EdgeInsetsGeometry get expandedTitlePadding => const EdgeInsets.fromLTRB(16, 0, 16, 20);
}

class _LargeScrollUnderFlexibleConfig with _ScrollUnderFlexibleConfig {
  _LargeScrollUnderFlexibleConfig(this.context);

  final BuildContext context;
  late final ThemeData _theme = Theme.of(context);
  late final ColorScheme _colors = _theme.colorScheme;
  late final TextTheme _textTheme = _theme.textTheme;

  static const double collapsedHeight = ${getToken('md.comp.top-app-bar.small.container.height')};
  static const double expandedHeight = ${getToken('md.comp.top-app-bar.large.container.height')};

  @override
  TextStyle? get collapsedTextStyle =>
    ${textStyle('md.comp.top-app-bar.small.headline')}?.apply(color: ${color('md.comp.top-app-bar.small.headline.color')});

  @override
  TextStyle? get expandedTextStyle =>
    ${textStyle('md.comp.top-app-bar.large.headline')}?.apply(color: ${color('md.comp.top-app-bar.large.headline.color')});

  @override
  EdgeInsetsGeometry get expandedTitlePadding => const EdgeInsets.fromLTRB(16, 0, 16, 28);
}
''';
}
