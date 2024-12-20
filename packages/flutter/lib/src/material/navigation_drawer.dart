// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'scaffold.dart';
library;

import 'package:flutter/widgets.dart';

import 'color_scheme.dart';
import 'colors.dart';
import 'drawer.dart';
import 'ink_decoration.dart';
import 'ink_well.dart';
import 'material.dart';
import 'material_localizations.dart';
import 'material_state.dart';
import 'navigation_bar.dart';
import 'navigation_drawer_theme.dart';
import 'text_theme.dart';
import 'theme.dart';

/// Material Design Navigation Drawer component.
///
/// On top of [Drawer]s, Navigation drawers offer a persistent and convenient way to switch
/// between primary destinations in an app.
///
/// The style for the icons and text are not affected by parent
/// [DefaultTextStyle]s or [IconTheme]s but rather controlled by parameters or
/// the [NavigationDrawerThemeData].
///
/// The [children] are a list of widgets to be displayed in the drawer. These can be a
/// mixture of any widgets, but there is special handling for [NavigationDrawerDestination]s.
/// They are treated as a group and when one is selected, the [onDestinationSelected]
/// is called with the index into the group that corresponds to the selected destination.
///
/// {@tool dartpad}
/// This example shows a [NavigationDrawer] used within a [Scaffold]
/// widget. The [NavigationDrawer] has headline widget, divider widget and three
/// [NavigationDrawerDestination] widgets. The initial [selectedIndex] is 0.
/// The [onDestinationSelected] callback changes the selected item's index and displays
/// a corresponding widget in the body of the [Scaffold].
///
/// ** See code in examples/api/lib/material/navigation_drawer/navigation_drawer.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [Scaffold.drawer], where one specifies a [Drawer] so that it can be
///    shown.
///  * [Scaffold.of], to obtain the current [ScaffoldState], which manages the
///    display and animation of the drawer.
///  * [ScaffoldState.openDrawer], which displays its [Drawer], if any.
///  * <https://material.io/design/components/navigation-drawer.html>
class NavigationDrawer extends StatelessWidget {
  /// Creates a Material Design Navigation Drawer component.
  const NavigationDrawer({
    super.key,
    required this.children,
    this.backgroundColor,
    this.shadowColor,
    this.surfaceTintColor,
    this.elevation,
    this.indicatorColor,
    this.indicatorShape,
    this.onDestinationSelected,
    this.selectedIndex = 0,
    this.tilePadding = const EdgeInsets.symmetric(horizontal: 12.0),
  });

  /// The background color of the [Material] that holds the [NavigationDrawer]'s
  /// contents.
  ///
  /// If this is null, then [NavigationDrawerThemeData.backgroundColor] is used.
  /// If that is also null, then it falls back to [ColorScheme.surfaceContainerLow].
  final Color? backgroundColor;

  /// The color used for the drop shadow to indicate elevation.
  ///
  /// If null, [NavigationDrawerThemeData.shadowColor] is used. If that
  /// is also null, the default value is [Colors.transparent] which
  /// indicates that no drop shadow will be displayed.
  ///
  /// See [Material.shadowColor] for more details on drop shadows.
  final Color? shadowColor;

  /// The surface tint of the [Material] that holds the [NavigationDrawer]'s
  /// contents.
  ///
  /// This is not recommended for use. [Material 3 spec](https://m3.material.io/styles/color/the-color-system/color-roles)
  /// introduced a set of tone-based surfaces and surface containers in its [ColorScheme],
  /// which provide more flexibility. The intention is to eventually remove surface tint color from
  /// the framework.
  ///
  /// If this is null, then [NavigationDrawerThemeData.surfaceTintColor] is used.
  /// If that is also null, the default value is [Colors.transparent].
  final Color? surfaceTintColor;

  /// The elevation of the [NavigationDrawer] itself.
  ///
  /// If null, [NavigationDrawerThemeData.elevation] is used. If that
  /// is also null, it will be 1.0.
  final double? elevation;

  /// The color of the [indicatorShape] when this destination is selected.
  ///
  /// If this is null, [NavigationDrawerThemeData.indicatorColor] is used.
  /// If that is also null, defaults to [ColorScheme.secondaryContainer].
  final Color? indicatorColor;

  /// The shape of the selected indicator.
  ///
  /// If this is null, [NavigationDrawerThemeData.indicatorShape] is used.
  /// If that is also null, defaults to [StadiumBorder].
  final ShapeBorder? indicatorShape;

  /// Defines the appearance of the items within the navigation drawer.
  ///
  /// The list contains [NavigationDrawerDestination] widgets and/or customized
  /// widgets like headlines and dividers.
  final List<Widget> children;

  /// The index into destinations for the current selected
  /// [NavigationDrawerDestination] or null if no destination is selected.
  ///
  /// A valid [selectedIndex] satisfies 0 <= [selectedIndex] < number of [NavigationDrawerDestination].
  /// For an invalid [selectedIndex] like `-1`, all destinations will appear unselected.
  final int? selectedIndex;

  /// Called when one of the [NavigationDrawerDestination] children is selected.
  ///
  /// This callback usually updates the int passed to [selectedIndex].
  ///
  /// Upon updating [selectedIndex], the [NavigationDrawer] will be rebuilt.
  final ValueChanged<int>? onDestinationSelected;

  /// Defines the padding for [NavigationDrawerDestination] widgets (Drawer items).
  ///
  /// Defaults to `EdgeInsets.symmetric(horizontal: 12.0)`.
  final EdgeInsetsGeometry tilePadding;

  @override
  Widget build(BuildContext context) {
    final int totalNumberOfDestinations =
        children.whereType<NavigationDrawerDestination>().toList().length;

    int destinationIndex = 0;
    Widget wrapChild(Widget child, int index) => _SelectableAnimatedBuilder(
      duration: const Duration(milliseconds: 500),
      isSelected: index == selectedIndex,
      builder: (BuildContext context, Animation<double> animation) {
        return _NavigationDrawerDestinationInfo(
          index: index,
          totalNumberOfDestinations: totalNumberOfDestinations,
          selectedAnimation: animation,
          indicatorColor: indicatorColor,
          indicatorShape: indicatorShape,
          tilePadding: tilePadding,
          onTap: () => onDestinationSelected?.call(index),
          child: child,
        );
      },
    );

    final List<Widget> wrappedChildren = <Widget>[
      for (final Widget child in children)
        if (child is! NavigationDrawerDestination) child else wrapChild(child, destinationIndex++),
    ];
    final NavigationDrawerThemeData navigationDrawerTheme = NavigationDrawerTheme.of(context);

    return Drawer(
      backgroundColor: backgroundColor ?? navigationDrawerTheme.backgroundColor,
      shadowColor: shadowColor ?? navigationDrawerTheme.shadowColor,
      surfaceTintColor: surfaceTintColor ?? navigationDrawerTheme.surfaceTintColor,
      elevation: elevation ?? navigationDrawerTheme.elevation,
      child: SafeArea(bottom: false, child: ListView(children: wrappedChildren)),
    );
  }
}

/// A Material Design [NavigationDrawer] destination.
///
/// Displays an icon with a label, for use in [NavigationDrawer.children].
class NavigationDrawerDestination extends StatelessWidget {
  /// Creates a navigation drawer destination.
  const NavigationDrawerDestination({
    super.key,
    this.backgroundColor,
    required this.icon,
    this.selectedIcon,
    required this.label,
    this.enabled = true,
  });

  /// The background color of the destination.
  ///
  /// If this is null, no background is set and [NavigationDrawer.backgroundColor] will be visible.
  ///
  /// This is the background color of the whole rectangular area behind the drawer destination.
  /// To customize only the indicator color consider using [NavigationDrawer.indicatorColor].
  final Color? backgroundColor;

  /// The [Widget] (usually an [Icon]) that's displayed for this
  /// [NavigationDestination].
  ///
  /// The icon will use [NavigationDrawerThemeData.iconTheme]. If this is
  /// null, the default [IconThemeData] would use a size of 24.0 and
  /// [ColorScheme.onSurfaceVariant].
  final Widget icon;

  /// The optional [Widget] (usually an [Icon]) that's displayed when this
  /// [NavigationDestination] is selected.
  ///
  /// If [selectedIcon] is non-null, the destination will fade from
  /// [icon] to [selectedIcon] when this destination goes from unselected to
  /// selected.
  ///
  /// The icon will use [NavigationDrawerThemeData.iconTheme] with
  /// [WidgetState.selected]. If this is null, the default [IconThemeData]
  /// would use a size of 24.0 and [ColorScheme.onSecondaryContainer].
  final Widget? selectedIcon;

  /// The text label that appears on the right of the icon
  ///
  /// The accompanying [Text] widget will use
  /// [NavigationDrawerThemeData.labelTextStyle]. If this are null, the default
  /// text style would use [TextTheme.labelLarge] with [ColorScheme.onSurfaceVariant].
  final Widget label;

  /// Indicates that this destination is selectable.
  ///
  /// Defaults to true.
  final bool enabled;

  @override
  Widget build(BuildContext context) {
    const Set<MaterialState> selectedState = <MaterialState>{MaterialState.selected};
    const Set<MaterialState> unselectedState = <MaterialState>{};
    const Set<MaterialState> disabledState = <MaterialState>{MaterialState.disabled};

    final NavigationDrawerThemeData navigationDrawerTheme = NavigationDrawerTheme.of(context);
    final NavigationDrawerThemeData defaults = _NavigationDrawerDefaultsM3(context);

    final Animation<double> animation =
        _NavigationDrawerDestinationInfo.of(context).selectedAnimation;

    return _NavigationDestinationBuilder(
      buildIcon: (BuildContext context) {
        final Widget selectedIconWidget = IconTheme.merge(
          data:
              navigationDrawerTheme.iconTheme?.resolve(enabled ? selectedState : disabledState) ??
              defaults.iconTheme!.resolve(enabled ? selectedState : disabledState)!,
          child: selectedIcon ?? icon,
        );
        final Widget unselectedIconWidget = IconTheme.merge(
          data:
              navigationDrawerTheme.iconTheme?.resolve(enabled ? unselectedState : disabledState) ??
              defaults.iconTheme!.resolve(enabled ? unselectedState : disabledState)!,
          child: icon,
        );

        return animation.isForwardOrCompleted ? selectedIconWidget : unselectedIconWidget;
      },
      buildLabel: (BuildContext context) {
        final TextStyle? effectiveSelectedLabelTextStyle =
            navigationDrawerTheme.labelTextStyle?.resolve(
              enabled ? selectedState : disabledState,
            ) ??
            defaults.labelTextStyle!.resolve(enabled ? selectedState : disabledState);
        final TextStyle? effectiveUnselectedLabelTextStyle =
            navigationDrawerTheme.labelTextStyle?.resolve(
              enabled ? unselectedState : disabledState,
            ) ??
            defaults.labelTextStyle!.resolve(enabled ? unselectedState : disabledState);

        return DefaultTextStyle(
          style:
              animation.isForwardOrCompleted
                  ? effectiveSelectedLabelTextStyle!
                  : effectiveUnselectedLabelTextStyle!,
          child: label,
        );
      },
      enabled: enabled,
      backgroundColor: backgroundColor,
    );
  }
}

/// Widget that handles the semantics and layout of a navigation drawer
/// destination.
///
/// Prefer [NavigationDestination] over this widget, as it is a simpler
/// (although less customizable) way to get navigation drawer destinations.
///
/// The icon and label of this destination are built with [buildIcon] and
/// [buildLabel]. They should build the unselected and selected icon and label
/// according to [_NavigationDrawerDestinationInfo.selectedAnimation], where an
/// animation value of 0 is unselected and 1 is selected.
///
/// See [NavigationDestination] for an example.
class _NavigationDestinationBuilder extends StatelessWidget {
  /// Builds a destination (icon + label) to use in a Material 3 [NavigationDrawer].
  const _NavigationDestinationBuilder({
    required this.buildIcon,
    required this.buildLabel,
    this.enabled = true,
    this.backgroundColor,
  });

  /// Builds the icon for a destination in a [NavigationDrawer].
  ///
  /// To animate between unselected and selected, build the icon based on
  /// [_NavigationDrawerDestinationInfo.selectedAnimation]. When the animation is 0,
  /// the destination is unselected, when the animation is 1, the destination is
  /// selected.
  ///
  /// The destination is considered selected as soon as the animation is
  /// increasing or completed, and it is considered unselected as soon as the
  /// animation is decreasing or dismissed.
  final WidgetBuilder buildIcon;

  /// Builds the label for a destination in a [NavigationDrawer].
  ///
  /// To animate between unselected and selected, build the icon based on
  /// [_NavigationDrawerDestinationInfo.selectedAnimation]. When the animation is
  /// 0, the destination is unselected, when the animation is 1, the destination
  /// is selected.
  ///
  /// The destination is considered selected as soon as the animation is
  /// increasing or completed, and it is considered unselected as soon as the
  /// animation is decreasing or dismissed.
  final WidgetBuilder buildLabel;

  /// Indicates that this destination is selectable.
  ///
  /// Defaults to true.
  final bool enabled;

  final Color? backgroundColor;

  @override
  Widget build(BuildContext context) {
    final _NavigationDrawerDestinationInfo info = _NavigationDrawerDestinationInfo.of(context);
    final NavigationDrawerThemeData navigationDrawerTheme = NavigationDrawerTheme.of(context);
    final NavigationDrawerThemeData defaults = _NavigationDrawerDefaultsM3(context);

    final InkWell inkWell = InkWell(
      highlightColor: Colors.transparent,
      onTap: enabled ? info.onTap : null,
      customBorder:
          info.indicatorShape ?? navigationDrawerTheme.indicatorShape ?? defaults.indicatorShape!,
      child: Stack(
        alignment: Alignment.center,
        children: <Widget>[
          NavigationIndicator(
            animation: info.selectedAnimation,
            color:
                info.indicatorColor ??
                navigationDrawerTheme.indicatorColor ??
                defaults.indicatorColor!,
            shape:
                info.indicatorShape ??
                navigationDrawerTheme.indicatorShape ??
                defaults.indicatorShape!,
            width: (navigationDrawerTheme.indicatorSize ?? defaults.indicatorSize!).width,
            height: (navigationDrawerTheme.indicatorSize ?? defaults.indicatorSize!).height,
          ),
          Row(
            children: <Widget>[
              const SizedBox(width: 16),
              buildIcon(context),
              const SizedBox(width: 12),
              buildLabel(context),
            ],
          ),
        ],
      ),
    );

    final Widget destination = Padding(
      padding: info.tilePadding,
      child: _NavigationDestinationSemantics(
        child: SizedBox(
          height: navigationDrawerTheme.tileHeight ?? defaults.tileHeight,
          child: inkWell,
        ),
      ),
    );

    if (backgroundColor != null) {
      return Ink(color: backgroundColor, child: destination);
    }
    return destination;
  }
}

/// Semantics widget for a navigation drawer destination.
///
/// Requires a [_NavigationDrawerDestinationInfo] parent (normally provided by the
/// [NavigationDrawer] by default).
///
/// Provides localized semantic labels to the destination, for example, it will
/// read "Home, Tab 1 of 3".
///
/// Used by [_NavigationDestinationBuilder].
class _NavigationDestinationSemantics extends StatelessWidget {
  /// Adds the appropriate semantics for navigation drawer destinations to the
  /// [child].
  const _NavigationDestinationSemantics({required this.child});

  /// The widget that should receive the destination semantics.
  final Widget child;

  @override
  Widget build(BuildContext context) {
    final MaterialLocalizations localizations = MaterialLocalizations.of(context);
    final _NavigationDrawerDestinationInfo destinationInfo = _NavigationDrawerDestinationInfo.of(
      context,
    );
    // The AnimationStatusBuilder will make sure that the semantics update to
    // "selected" when the animation status changes.
    return _StatusTransitionWidgetBuilder(
      animation: destinationInfo.selectedAnimation,
      builder: (BuildContext context, Widget? child) {
        return Semantics(
          selected: destinationInfo.selectedAnimation.isForwardOrCompleted,
          container: true,
          child: child,
        );
      },
      child: Stack(
        alignment: Alignment.center,
        children: <Widget>[
          child,
          Semantics(
            label: localizations.tabLabel(
              tabIndex: destinationInfo.index + 1,
              tabCount: destinationInfo.totalNumberOfDestinations,
            ),
          ),
        ],
      ),
    );
  }
}

/// Widget that listens to an animation, and rebuilds when the animation changes
/// [AnimationStatus].
///
/// This can be more efficient than just using an [AnimatedBuilder] when you
/// only need to rebuild when the [Animation.status] changes, since
/// [AnimatedBuilder] rebuilds every time the animation ticks.
class _StatusTransitionWidgetBuilder extends StatusTransitionWidget {
  /// Creates a widget that rebuilds when the given animation changes status.
  const _StatusTransitionWidgetBuilder({
    required super.animation,
    required this.builder,
    this.child,
  });

  /// Called every time the [animation] changes [AnimationStatus].
  final TransitionBuilder builder;

  /// The child widget to pass to the [builder].
  ///
  /// If a [builder] callback's return value contains a subtree that does not
  /// depend on the animation, it's more efficient to build that subtree once
  /// instead of rebuilding it on every animation status change.
  ///
  /// Using this pre-built child is entirely optional, but can improve
  /// performance in some cases and is therefore a good practice.
  ///
  /// See: [AnimatedBuilder.child]
  final Widget? child;

  @override
  Widget build(BuildContext context) => builder(context, child);
}

/// Inherited widget for passing data from the [NavigationDrawer] to the
/// [NavigationDrawerDestination] child widgets.
///
/// Useful for building navigation destinations using:
/// `_NavigationDrawerDestinationInfo.of(context)`.
class _NavigationDrawerDestinationInfo extends InheritedWidget {
  /// Adds the information needed to build a navigation destination to the
  /// [child] and descendants.
  const _NavigationDrawerDestinationInfo({
    required this.index,
    required this.totalNumberOfDestinations,
    required this.selectedAnimation,
    required this.indicatorColor,
    required this.indicatorShape,
    required this.onTap,
    required super.child,
    required this.tilePadding,
  });

  /// Which destination index is this in the navigation drawer.
  ///
  /// For example:
  ///
  /// ```dart
  /// const NavigationDrawer(
  ///   children: <Widget>[
  ///     Text('Headline'), // This doesn't have index.
  ///     NavigationDrawerDestination(
  ///       // This is destination index 0.
  ///       icon: Icon(Icons.surfing),
  ///       label: Text('Surfing'),
  ///     ),
  ///     NavigationDrawerDestination(
  ///       // This is destination index 1.
  ///       icon: Icon(Icons.support),
  ///       label: Text('Support'),
  ///     ),
  ///     NavigationDrawerDestination(
  ///       // This is destination index 2.
  ///       icon: Icon(Icons.local_hospital),
  ///       label: Text('Hospital'),
  ///     ),
  ///   ]
  /// )
  /// ```
  ///
  /// This is required for semantics, so that each destination can have a label
  /// "Tab 1 of 3", for example.
  final int index;

  /// How many total destinations are in this navigation drawer.
  ///
  /// This is required for semantics, so that each destination can have a label
  /// "Tab 1 of 4", for example.
  final int totalNumberOfDestinations;

  /// Indicates whether or not this destination is selected, from 0 (unselected)
  /// to 1 (selected).
  final Animation<double> selectedAnimation;

  /// The color of the indicator.
  ///
  /// This is used by destinations to override the indicator color.
  final Color? indicatorColor;

  /// The shape of the indicator.
  ///
  /// This is used by destinations to override the indicator shape.
  final ShapeBorder? indicatorShape;

  /// The callback that should be called when this destination is tapped.
  ///
  /// This is computed by calling [NavigationDrawer.onDestinationSelected]
  /// with [index] passed in.
  final VoidCallback onTap;

  /// Defines the padding for [NavigationDrawerDestination] widgets (Drawer items).
  ///
  /// Defaults to `EdgeInsets.symmetric(horizontal: 12.0)`.
  final EdgeInsetsGeometry tilePadding;

  /// Returns a non null [_NavigationDrawerDestinationInfo].
  ///
  /// This will return an error if called with no [_NavigationDrawerDestinationInfo]
  /// ancestor.
  ///
  /// Used by widgets that are implementing a navigation destination info to
  /// get information like the selected animation and destination number.
  static _NavigationDrawerDestinationInfo of(BuildContext context) {
    final _NavigationDrawerDestinationInfo? result =
        context.dependOnInheritedWidgetOfExactType<_NavigationDrawerDestinationInfo>();
    assert(
      result != null,
      'Navigation destinations need a _NavigationDrawerDestinationInfo parent, '
      'which is usually provided by NavigationDrawer.',
    );
    return result!;
  }

  @override
  bool updateShouldNotify(_NavigationDrawerDestinationInfo oldWidget) {
    return index != oldWidget.index ||
        totalNumberOfDestinations != oldWidget.totalNumberOfDestinations ||
        selectedAnimation != oldWidget.selectedAnimation ||
        onTap != oldWidget.onTap;
  }
}

// Builder widget for widgets that need to be animated from 0 (unselected) to
// 1.0 (selected).
//
// This widget creates and manages an [AnimationController] that it passes down
// to the child through the [builder] function.
//
// When [isSelected] is `true`, the animation controller will animate from
// 0 to 1 (for [duration] time).
//
// When [isSelected] is `false`, the animation controller will animate from
// 1 to 0 (for [duration] time).
//
// If [isSelected] is updated while the widget is animating, the animation will
// be reversed until it is either 0 or 1 again.
//
// Usage:
// ```dart
// _SelectableAnimatedBuilder(
//   isSelected: _isDrawerOpen,
//   builder: (context, animation) {
//     return AnimatedIcon(
//       icon: AnimatedIcons.menu_arrow,
//       progress: animation,
//       semanticLabel: 'Show menu',
//     );
//   }
// )
// ```
class _SelectableAnimatedBuilder extends StatefulWidget {
  /// Builds and maintains an [AnimationController] that will animate from 0 to
  /// 1 and back depending on when [isSelected] is true.
  const _SelectableAnimatedBuilder({
    required this.isSelected,
    this.duration = const Duration(milliseconds: 200),
    required this.builder,
  });

  /// When true, the widget will animate an animation controller from 0 to 1.
  ///
  /// The animation controller is passed to the child widget through [builder].
  final bool isSelected;

  /// How long the animation controller should animate for when [isSelected] is
  /// updated.
  ///
  /// If the animation is currently running and [isSelected] is updated, only
  /// the [duration] left to finish the animation will be run.
  final Duration duration;

  /// Builds the child widget based on the current animation status.
  ///
  /// When [isSelected] is updated to true, this builder will be called and the
  /// animation will animate up to 1. When [isSelected] is updated to
  /// `false`, this will be called and the animation will animate down to 0.
  final Widget Function(BuildContext, Animation<double>) builder;

  ///
  @override
  _SelectableAnimatedBuilderState createState() => _SelectableAnimatedBuilderState();
}

/// State that manages the [AnimationController] that is passed to
/// [_SelectableAnimatedBuilder.builder].
class _SelectableAnimatedBuilderState extends State<_SelectableAnimatedBuilder>
    with SingleTickerProviderStateMixin {
  late AnimationController _controller;

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(vsync: this);
    _controller.duration = widget.duration;
    _controller.value = widget.isSelected ? 1.0 : 0.0;
  }

  @override
  void didUpdateWidget(_SelectableAnimatedBuilder oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.duration != widget.duration) {
      _controller.duration = widget.duration;
    }
    if (oldWidget.isSelected != widget.isSelected) {
      if (widget.isSelected) {
        _controller.forward();
      } else {
        _controller.reverse();
      }
    }
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return widget.builder(context, _controller);
  }
}

// BEGIN GENERATED TOKEN PROPERTIES - NavigationDrawer

// Do not edit by hand. The code between the "BEGIN GENERATED" and
// "END GENERATED" comments are generated from data in the Material
// Design token database by the script:
//   dev/tools/gen_defaults/bin/gen_defaults.dart.

// dart format off
class _NavigationDrawerDefaultsM3 extends NavigationDrawerThemeData {
  _NavigationDrawerDefaultsM3(this.context)
    : super(
        elevation: 1.0,
        tileHeight: 56.0,
        indicatorShape: const StadiumBorder(),
        indicatorSize: const Size(336.0, 56.0),
      );

  final BuildContext context;
  late final ColorScheme _colors = Theme.of(context).colorScheme;
  late final TextTheme _textTheme = Theme.of(context).textTheme;

  @override
  Color? get backgroundColor => _colors.surfaceContainerLow;

  @override
  Color? get surfaceTintColor => Colors.transparent;

  @override
  Color? get shadowColor => Colors.transparent;

  @override
  Color? get indicatorColor => _colors.secondaryContainer;

  @override
  MaterialStateProperty<IconThemeData?>? get iconTheme {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      return IconThemeData(
        size: 24.0,
        color: states.contains(MaterialState.disabled)
          ? _colors.onSurfaceVariant.withOpacity(0.38)
          : states.contains(MaterialState.selected)
            ? _colors.onSecondaryContainer
            : _colors.onSurfaceVariant,
      );
    });
  }

  @override
  MaterialStateProperty<TextStyle?>? get labelTextStyle {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      final TextStyle style = _textTheme.labelLarge!;
      return style.apply(
        color: states.contains(MaterialState.disabled)
          ? _colors.onSurfaceVariant.withOpacity(0.38)
          : states.contains(MaterialState.selected)
            ? _colors.onSecondaryContainer
            : _colors.onSurfaceVariant,
      );
    });
  }
}
// dart format on

// END GENERATED TOKEN PROPERTIES - NavigationDrawer
