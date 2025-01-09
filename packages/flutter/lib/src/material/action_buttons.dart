// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/services.dart';
///
/// @docImport 'app_bar.dart';
/// @docImport 'drawer.dart';
/// @docImport 'material.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'action_icons_theme.dart';
import 'button_style.dart';
import 'debug.dart';
import 'icon_button.dart';
import 'icons.dart';
import 'material_localizations.dart';
import 'scaffold.dart';
import 'theme.dart';

abstract class _ActionButton extends IconButton {
  /// Creates a Material Design icon button.
  const _ActionButton({
    super.key,
    super.color,
    super.style,
    super.onPressed,
    required super.icon,
    this.standardComponent,
  });

  /// An enum value to use to identify this button as a type of
  /// [StandardComponentType].
  final StandardComponentType? standardComponent;

  /// This returns the appropriate tooltip text for this action button.
  String _getTooltip(BuildContext context);

  /// This is the default function that is called when [onPressed] is set
  /// to null.
  void _onPressedCallback(BuildContext context);

  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasMaterialLocalizations(context));
    return IconButton(
      key: standardComponent?.key,
      icon: icon,
      style: style,
      color: color,
      tooltip: _getTooltip(context),
      onPressed: () {
        if (onPressed != null) {
          onPressed!();
        } else {
          _onPressedCallback(context);
        }
      },
    );
  }
}

typedef _ActionIconBuilderCallback = WidgetBuilder? Function(ActionIconThemeData? actionIconTheme);
typedef _ActionIconDataCallback = IconData Function(BuildContext context);
typedef _AndroidSemanticsLabelCallback =
    String Function(MaterialLocalizations materialLocalization);

class _ActionIcon extends StatelessWidget {
  const _ActionIcon({
    required this.iconBuilderCallback,
    required this.getIcon,
    required this.getAndroidSemanticsLabel,
  });

  final _ActionIconBuilderCallback iconBuilderCallback;
  final _ActionIconDataCallback getIcon;
  final _AndroidSemanticsLabelCallback getAndroidSemanticsLabel;

  @override
  Widget build(BuildContext context) {
    final ActionIconThemeData? actionIconTheme = ActionIconTheme.of(context);
    final WidgetBuilder? iconBuilder = iconBuilderCallback(actionIconTheme);
    if (iconBuilder != null) {
      return iconBuilder(context);
    }

    final IconData data = getIcon(context);
    final String? semanticsLabel;
    // This can't use the platform from Theme because it is the Android OS that
    // expects the duplicated tooltip and label.
    switch (defaultTargetPlatform) {
      case TargetPlatform.android:
        semanticsLabel = getAndroidSemanticsLabel(MaterialLocalizations.of(context));
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
        semanticsLabel = null;
    }

    return Icon(data, semanticLabel: semanticsLabel);
  }
}

/// A "back" icon that's appropriate for the current [TargetPlatform].
///
/// The current platform is determined by querying for the ambient [Theme].
///
/// See also:
///
///  * [BackButton], an [IconButton] with a [BackButtonIcon] that calls
///    [Navigator.maybePop] to return to the previous route.
///  * [IconButton], which is a more general widget for creating buttons
///    with icons.
///  * [Icon], a Material Design icon.
///  * [ThemeData.platform], which specifies the current platform.
class BackButtonIcon extends StatelessWidget {
  /// Creates an icon that shows the appropriate "back" image for
  /// the current platform (as obtained from the [Theme]).
  const BackButtonIcon({super.key});

  @override
  Widget build(BuildContext context) {
    return _ActionIcon(
      iconBuilderCallback: (ActionIconThemeData? actionIconTheme) {
        return actionIconTheme?.backButtonIconBuilder;
      },
      getIcon: (BuildContext context) {
        if (kIsWeb) {
          // Always use 'Icons.arrow_back' as a back_button icon in web.
          return Icons.arrow_back;
        }
        switch (Theme.of(context).platform) {
          case TargetPlatform.android:
          case TargetPlatform.fuchsia:
          case TargetPlatform.linux:
          case TargetPlatform.windows:
            return Icons.arrow_back;
          case TargetPlatform.iOS:
          case TargetPlatform.macOS:
            return Icons.arrow_back_ios_new_rounded;
        }
      },
      getAndroidSemanticsLabel: (MaterialLocalizations materialLocalization) {
        return materialLocalization.backButtonTooltip;
      },
    );
  }
}

/// A Material Design back icon button.
///
/// A [BackButton] is an [IconButton] with a "back" icon appropriate for the
/// current [TargetPlatform]. When pressed, the back button calls
/// [Navigator.maybePop] to return to the previous route unless a custom
/// [onPressed] callback is provided.
///
/// The [onPressed] callback can, for instance, be used to pop the platform's navigation stack
/// via [SystemNavigator] instead of Flutter's [Navigator] in add-to-app
/// situations.
///
/// In Material Design 3, both [style]'s [ButtonStyle.iconColor] and [color] are
/// used to override the default icon color of [BackButton]. If both exist, the [ButtonStyle.iconColor]
/// will override [color] for states where [ButtonStyle.foregroundColor] resolves to non-null.
///
/// When deciding to display a [BackButton], consider using
/// `ModalRoute.canPopOf(context)` to check whether the current route can be
/// popped. If that value is false (e.g., because the current route is the
/// initial route), the [BackButton] will not have any effect when pressed,
/// which could frustrate the user.
///
/// Requires one of its ancestors to be a [Material] widget.
///
/// See also:
///
///  * [AppBar], which automatically uses a [BackButton] in its
///    [AppBar.leading] slot when the [Scaffold] has no [Drawer] and the
///    current [Route] is not the [Navigator]'s first route.
///  * [BackButtonIcon], which is useful if you need to create a back button
///    that responds differently to being pressed.
///  * [IconButton], which is a more general widget for creating buttons with
///    icons.
///  * [CloseButton], an alternative which may be more appropriate for leaf
///    node pages in the navigation tree.
class BackButton extends _ActionButton {
  /// Creates an [IconButton] with the appropriate "back" icon for the current
  /// target platform.
  const BackButton({super.key, super.color, super.style, super.onPressed})
    : super(icon: const BackButtonIcon(), standardComponent: StandardComponentType.backButton);

  @override
  void _onPressedCallback(BuildContext context) => Navigator.maybePop(context);

  @override
  String _getTooltip(BuildContext context) {
    return MaterialLocalizations.of(context).backButtonTooltip;
  }
}

/// A "close" icon that's appropriate for the current [TargetPlatform].
///
/// The current platform is determined by querying for the ambient [Theme].
///
/// See also:
///
///  * [CloseButton], an [IconButton] with a [CloseButtonIcon] that calls
///    [Navigator.maybePop] to return to the previous route.
///  * [IconButton], which is a more general widget for creating buttons
///    with icons.
///  * [Icon], a Material Design icon.
///  * [ThemeData.platform], which specifies the current platform.
class CloseButtonIcon extends StatelessWidget {
  /// Creates an icon that shows the appropriate "close" image for
  /// the current platform (as obtained from the [Theme]).
  const CloseButtonIcon({super.key});

  @override
  Widget build(BuildContext context) {
    return _ActionIcon(
      iconBuilderCallback: (ActionIconThemeData? actionIconTheme) {
        return actionIconTheme?.closeButtonIconBuilder;
      },
      getIcon: (BuildContext context) => Icons.close,
      getAndroidSemanticsLabel: (MaterialLocalizations materialLocalization) {
        return materialLocalization.closeButtonTooltip;
      },
    );
  }
}

/// A Material Design close icon button.
///
/// A [CloseButton] is an [IconButton] with a "close" icon. When pressed, the
/// close button calls [Navigator.maybePop] to return to the previous route.
///
/// The [onPressed] callback can, for instance, be used to pop the platform's navigation stack
/// via [SystemNavigator] instead of Flutter's [Navigator] in add-to-app
/// situations.
///
/// In Material Design 3, both [style]'s [ButtonStyle.iconColor] and [color] are
/// used to override the default icon color of [CloseButton]. If both exist, the [ButtonStyle.iconColor]
/// will override [color] for states where [ButtonStyle.foregroundColor] resolves to non-null.
///
/// Use a [CloseButton] instead of a [BackButton] on fullscreen dialogs or
/// pages that may solicit additional actions to close.
///
/// See also:
///
///  * [AppBar], which automatically uses a [CloseButton] in its
///    [AppBar.leading] slot when appropriate.
///  * [BackButton], which is more appropriate for middle nodes in the
///    navigation tree or where pages can be popped instantaneously with
///    no user data consequence.
///  * [IconButton], to create other Material Design icon buttons.
class CloseButton extends _ActionButton {
  /// Creates a Material Design close icon button.
  const CloseButton({super.key, super.color, super.onPressed, super.style})
    : super(icon: const CloseButtonIcon(), standardComponent: StandardComponentType.closeButton);

  @override
  void _onPressedCallback(BuildContext context) => Navigator.maybePop(context);

  @override
  String _getTooltip(BuildContext context) {
    return MaterialLocalizations.of(context).closeButtonTooltip;
  }
}

/// A "drawer" icon that's appropriate for the current [TargetPlatform].
///
/// The current platform is determined by querying for the ambient [Theme].
///
/// See also:
///
///  * [DrawerButton], an [IconButton] with a [DrawerButtonIcon] that calls
///    [ScaffoldState.openDrawer] to open the [Scaffold.drawer].
///  * [EndDrawerButton], an [IconButton] with an [EndDrawerButtonIcon] that
///    calls [ScaffoldState.openEndDrawer] to open the [Scaffold.endDrawer].
///  * [IconButton], which is a more general widget for creating buttons
///    with icons.
///  * [Icon], a Material Design icon.
///  * [ThemeData.platform], which specifies the current platform.
class DrawerButtonIcon extends StatelessWidget {
  /// Creates an icon that shows the appropriate "close" image for
  /// the current platform (as obtained from the [Theme]).
  const DrawerButtonIcon({super.key});

  @override
  Widget build(BuildContext context) {
    return _ActionIcon(
      iconBuilderCallback: (ActionIconThemeData? actionIconTheme) {
        return actionIconTheme?.drawerButtonIconBuilder;
      },
      getIcon: (BuildContext context) => Icons.menu,
      getAndroidSemanticsLabel: (MaterialLocalizations materialLocalization) {
        return materialLocalization.openAppDrawerTooltip;
      },
    );
  }
}

/// A Material Design drawer icon button.
///
/// A [DrawerButton] is an [IconButton] with a "drawer" icon. When pressed, the
/// close button calls [ScaffoldState.openDrawer] to the [Scaffold.drawer].
///
/// The default behaviour on press can be overridden with [onPressed].
///
/// See also:
///
///  * [EndDrawerButton], an [IconButton] with an [EndDrawerButtonIcon] that
///    calls [ScaffoldState.openEndDrawer] to open the [Scaffold.endDrawer].
///  * [IconButton], which is a more general widget for creating buttons
///    with icons.
///  * [Icon], a Material Design icon.
///  * [ThemeData.platform], which specifies the current platform.
class DrawerButton extends _ActionButton {
  /// Creates a Material Design drawer icon button.
  const DrawerButton({super.key, super.color, super.style, super.onPressed})
    : super(icon: const DrawerButtonIcon(), standardComponent: StandardComponentType.drawerButton);

  @override
  void _onPressedCallback(BuildContext context) => Scaffold.of(context).openDrawer();

  @override
  String _getTooltip(BuildContext context) {
    return MaterialLocalizations.of(context).openAppDrawerTooltip;
  }
}

/// A "end drawer" icon that's appropriate for the current [TargetPlatform].
///
/// The current platform is determined by querying for the ambient [Theme].
///
/// See also:
///
///  * [DrawerButton], an [IconButton] with a [DrawerButtonIcon] that calls
///    [ScaffoldState.openDrawer] to open the [Scaffold.drawer].
///  * [EndDrawerButton], an [IconButton] with an [EndDrawerButtonIcon] that
///    calls [ScaffoldState.openEndDrawer] to open the [Scaffold.endDrawer]
///  * [IconButton], which is a more general widget for creating buttons
///    with icons.
///  * [Icon], a Material Design icon.
///  * [ThemeData.platform], which specifies the current platform.
class EndDrawerButtonIcon extends StatelessWidget {
  /// Creates an icon that shows the appropriate "end drawer" image for
  /// the current platform (as obtained from the [Theme]).
  const EndDrawerButtonIcon({super.key});

  @override
  Widget build(BuildContext context) {
    return _ActionIcon(
      iconBuilderCallback: (ActionIconThemeData? actionIconTheme) {
        return actionIconTheme?.endDrawerButtonIconBuilder;
      },
      getIcon: (BuildContext context) => Icons.menu,
      getAndroidSemanticsLabel: (MaterialLocalizations materialLocalization) {
        return materialLocalization.openAppDrawerTooltip;
      },
    );
  }
}

/// A Material Design end drawer icon button.
///
/// A [EndDrawerButton] is an [IconButton] with a "drawer" icon. When pressed, the
/// end drawer button calls [ScaffoldState.openEndDrawer] to open the [Scaffold.endDrawer].
///
/// The default behaviour on press can be overridden with [onPressed].
///
/// See also:
///
///  * [DrawerButton], an [IconButton] with a [DrawerButtonIcon] that calls
///    [ScaffoldState.openDrawer] to open a drawer.
///  * [IconButton], which is a more general widget for creating buttons
///    with icons.
///  * [Icon], a Material Design icon.
///  * [ThemeData.platform], which specifies the current platform.
class EndDrawerButton extends _ActionButton {
  /// Creates a Material Design end drawer icon button.
  const EndDrawerButton({super.key, super.color, super.style, super.onPressed})
    : super(icon: const EndDrawerButtonIcon());

  @override
  void _onPressedCallback(BuildContext context) => Scaffold.of(context).openEndDrawer();

  @override
  String _getTooltip(BuildContext context) {
    return MaterialLocalizations.of(context).openAppDrawerTooltip;
  }
}
