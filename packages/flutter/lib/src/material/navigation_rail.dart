// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'bottom_navigation_bar.dart';
/// @docImport 'floating_action_button.dart';
/// @docImport 'icons.dart';
/// @docImport 'scaffold.dart';
library;

import 'dart:ui';

import 'package:flutter/widgets.dart';

import 'color_scheme.dart';
import 'ink_well.dart';
import 'material.dart';
import 'material_localizations.dart';
import 'navigation_bar.dart';
import 'navigation_rail_theme.dart';
import 'text_theme.dart';
import 'theme.dart';

const double _kCircularIndicatorDiameter = 56;
const double _kIndicatorHeight = 32;

/// A Material Design widget that is meant to be displayed at the left or right of an
/// app to navigate between a small number of views, typically between three and
/// five.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=y9xchtVTtqQ}
///
/// The navigation rail is meant for layouts with wide viewports, such as a
/// desktop web or tablet landscape layout. For smaller layouts, like mobile
/// portrait, a [BottomNavigationBar] should be used instead.
///
/// A navigation rail is usually used as the first or last element of a [Row]
/// which defines the app's [Scaffold] body.
///
/// The appearance of all of the [NavigationRail]s within an app can be
/// specified with [NavigationRailTheme]. The default values for null theme
/// properties are based on the [Theme]'s [ThemeData.textTheme],
/// [ThemeData.iconTheme], and [ThemeData.colorScheme].
///
/// Adaptive layouts can build different instances of the [Scaffold] in order to
/// have a navigation rail for more horizontal layouts and a bottom navigation
/// bar for more vertical layouts. See
/// [the adaptive_scaffold.dart sample](https://github.com/flutter/samples/blob/main/experimental/web_dashboard/lib/src/widgets/third_party/adaptive_scaffold.dart)
/// for an example.
///
/// {@tool dartpad}
/// This example shows a [NavigationRail] used within a Scaffold with 3
/// [NavigationRailDestination]s. The main content is separated by a divider
/// (although elevation on the navigation rail can be used instead). The
/// `_selectedIndex` is updated by the `onDestinationSelected` callback.
///
/// ** See code in examples/api/lib/material/navigation_rail/navigation_rail.0.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This sample shows the creation of [NavigationRail] widget used within a Scaffold with 3
/// [NavigationRailDestination]s, as described in: https://m3.material.io/components/navigation-rail/overview
///
/// ** See code in examples/api/lib/material/navigation_rail/navigation_rail.1.dart **
/// {@end-tool}
///
/// See also:
///
///  * [Scaffold], which can display the navigation rail within a [Row] of the
///    [Scaffold.body] slot.
///  * [NavigationRailDestination], which is used as a model to create tappable
///    destinations in the navigation rail.
///  * [BottomNavigationBar], which is a similar navigation widget that's laid
///     out horizontally.
///  * <https://material.io/components/navigation-rail/>
///  * <https://m3.material.io/components/navigation-rail>
class NavigationRail extends StatefulWidget {
  /// Creates a Material Design navigation rail.
  ///
  /// The value of [destinations] must be a list of zero or more
  /// [NavigationRailDestination] values.
  ///
  /// If [elevation] is specified, it must be non-negative.
  ///
  /// If [minWidth] is specified, it must be non-negative, and if
  /// [minExtendedWidth] is specified, it must be non-negative and greater than
  /// [minWidth].
  ///
  /// The [extended] argument can only be set to true when the [labelType] is
  /// null or [NavigationRailLabelType.none].
  ///
  /// If [backgroundColor], [elevation], [groupAlignment], [labelType],
  /// [unselectedLabelTextStyle], [selectedLabelTextStyle],
  /// [unselectedIconTheme], or [selectedIconTheme] are null, then their
  /// [NavigationRailThemeData] values will be used. If the corresponding
  /// [NavigationRailThemeData] property is null, then the navigation rail
  /// defaults are used. See the individual properties for more information.
  ///
  /// Typically used within a [Row] that defines the [Scaffold.body] property.
  const NavigationRail({
    super.key,
    this.backgroundColor,
    this.extended = false,
    this.leading,
    this.trailing,
    required this.destinations,
    required this.selectedIndex,
    this.onDestinationSelected,
    this.elevation,
    this.groupAlignment,
    this.labelType,
    this.unselectedLabelTextStyle,
    this.selectedLabelTextStyle,
    this.unselectedIconTheme,
    this.selectedIconTheme,
    this.minWidth,
    this.minExtendedWidth,
    this.useIndicator,
    this.indicatorColor,
    this.indicatorShape,
  }) : assert(selectedIndex == null || (0 <= selectedIndex && selectedIndex < destinations.length)),
       assert(elevation == null || elevation > 0),
       assert(minWidth == null || minWidth > 0),
       assert(minExtendedWidth == null || minExtendedWidth > 0),
       assert((minWidth == null || minExtendedWidth == null) || minExtendedWidth >= minWidth),
       assert(!extended || (labelType == null || labelType == NavigationRailLabelType.none));

  /// Sets the color of the Container that holds all of the [NavigationRail]'s
  /// contents.
  ///
  /// The default value is [NavigationRailThemeData.backgroundColor]. If
  /// [NavigationRailThemeData.backgroundColor] is null, then the default value
  /// is based on [ColorScheme.surface] of [ThemeData.colorScheme].
  final Color? backgroundColor;

  /// Indicates that the [NavigationRail] should be in the extended state.
  ///
  /// The extended state has a wider rail container, and the labels are
  /// positioned next to the icons. [minExtendedWidth] can be used to set the
  /// minimum width of the rail when it is in this state.
  ///
  /// The rail will implicitly animate between the extended and normal state.
  ///
  /// If the rail is going to be in the extended state, then the [labelType]
  /// must be set to [NavigationRailLabelType.none].
  ///
  /// The default value is false.
  final bool extended;

  /// The leading widget in the rail that is placed above the destinations.
  ///
  /// It is placed at the top of the rail, above the [destinations]. Its
  /// location is not affected by [groupAlignment].
  ///
  /// This is commonly a [FloatingActionButton], but may also be a non-button,
  /// such as a logo.
  ///
  /// The default value is null.
  final Widget? leading;

  /// The trailing widget in the rail that is placed below the destinations.
  ///
  /// The trailing widget is placed below the last [NavigationRailDestination].
  /// It's location is affected by [groupAlignment].
  ///
  /// This is commonly a list of additional options or destinations that is
  /// usually only rendered when [extended] is true.
  ///
  /// The default value is null.
  final Widget? trailing;

  /// Defines the appearance of the button items that are arrayed within the
  /// navigation rail.
  ///
  /// The value must be a list of zero or more [NavigationRailDestination]
  /// values.
  final List<NavigationRailDestination> destinations;

  /// The index into [destinations] for the current selected
  /// [NavigationRailDestination] or null if no destination is selected.
  final int? selectedIndex;

  /// Called when one of the [destinations] is selected.
  ///
  /// The stateful widget that creates the navigation rail needs to keep
  /// track of the index of the selected [NavigationRailDestination] and call
  /// `setState` to rebuild the navigation rail with the new [selectedIndex].
  final ValueChanged<int>? onDestinationSelected;

  /// The rail's elevation or z-coordinate.
  ///
  /// If [Directionality] is [TextDirection.ltr], the inner side is the
  /// right side, and if [Directionality] is [TextDirection.rtl], it is
  /// the left side.
  ///
  /// The default value is 0.
  final double? elevation;

  /// The vertical alignment for the group of [destinations] within the rail.
  ///
  /// The [NavigationRailDestination]s are grouped together with the [trailing]
  /// widget, between the [leading] widget and the bottom of the rail.
  ///
  /// The value must be between -1.0 and 1.0.
  ///
  /// If [groupAlignment] is -1.0, then the items are aligned to the top. If
  /// [groupAlignment] is 0.0, then the items are aligned to the center. If
  /// [groupAlignment] is 1.0, then the items are aligned to the bottom.
  ///
  /// The default is -1.0.
  ///
  /// See also:
  ///   * [Alignment.y]
  ///
  final double? groupAlignment;

  /// Defines the layout and behavior of the labels for the default, unextended
  /// [NavigationRail].
  ///
  /// When a navigation rail is [extended], the labels are always shown.
  ///
  /// The default value is [NavigationRailThemeData.labelType]. If
  /// [NavigationRailThemeData.labelType] is null, then the default value is
  /// [NavigationRailLabelType.none].
  ///
  /// See also:
  ///
  ///   * [NavigationRailLabelType] for information on the meaning of different
  ///   types.
  final NavigationRailLabelType? labelType;

  /// The [TextStyle] of a destination's label when it is unselected.
  ///
  /// When one of the [destinations] is selected the [selectedLabelTextStyle]
  /// will be used instead.
  ///
  /// The default value is based on the [Theme]'s [TextTheme.bodyLarge]. The
  /// default color is based on the [Theme]'s [ColorScheme.onSurface].
  ///
  /// Properties from this text style, or
  /// [NavigationRailThemeData.unselectedLabelTextStyle] if this is null, are
  /// merged into the defaults.
  final TextStyle? unselectedLabelTextStyle;

  /// The [TextStyle] of a destination's label when it is selected.
  ///
  /// When a [NavigationRailDestination] is not selected,
  /// [unselectedLabelTextStyle] will be used.
  ///
  /// The default value is based on the [TextTheme.bodyLarge] of
  /// [ThemeData.textTheme]. The default color is based on the [Theme]'s
  /// [ColorScheme.primary].
  ///
  /// Properties from this text style,
  /// or [NavigationRailThemeData.selectedLabelTextStyle] if this is null, are
  /// merged into the defaults.
  final TextStyle? selectedLabelTextStyle;

  /// The visual properties of the icon in the unselected destination.
  ///
  /// If this field is not provided, or provided with any null properties, then
  /// a copy of the [IconThemeData.fallback] with a custom [NavigationRail]
  /// specific color will be used.
  ///
  /// The default value is the [Theme]'s [ThemeData.iconTheme] with a color
  /// of the [Theme]'s [ColorScheme.onSurface] with an opacity of 0.64.
  /// Properties from this icon theme, or
  /// [NavigationRailThemeData.unselectedIconTheme] if this is null, are
  /// merged into the defaults.
  final IconThemeData? unselectedIconTheme;

  /// The visual properties of the icon in the selected destination.
  ///
  /// When a [NavigationRailDestination] is not selected,
  /// [unselectedIconTheme] will be used.
  ///
  /// The default value is the [Theme]'s [ThemeData.iconTheme] with a color
  /// of the [Theme]'s [ColorScheme.primary]. Properties from this icon theme,
  /// or [NavigationRailThemeData.selectedIconTheme] if this is null, are
  /// merged into the defaults.
  final IconThemeData? selectedIconTheme;

  /// The smallest possible width for the rail regardless of the destination's
  /// icon or label size.
  ///
  /// The default is 72.
  ///
  /// This value also defines the min width and min height of the destinations.
  ///
  /// To make a compact rail, set this to 56 and use
  /// [NavigationRailLabelType.none].
  final double? minWidth;

  /// The final width when the animation is complete for setting [extended] to
  /// true.
  ///
  /// This is only used when [extended] is set to true.
  ///
  /// The default value is 256.
  final double? minExtendedWidth;

  /// If `true`, adds a rounded [NavigationIndicator] behind the selected
  /// destination's icon.
  ///
  /// The indicator's shape will be circular if [labelType] is
  /// [NavigationRailLabelType.none], or a [StadiumBorder] if [labelType] is
  /// [NavigationRailLabelType.all] or [NavigationRailLabelType.selected].
  ///
  /// If `null`, defaults to [NavigationRailThemeData.useIndicator]. If that is
  /// `null`, defaults to [ThemeData.useMaterial3].
  final bool? useIndicator;

  /// Overrides the default value of [NavigationRail]'s selection indicator color,
  /// when [useIndicator] is true.
  ///
  /// If this is null, [NavigationRailThemeData.indicatorColor] is used. If
  /// that is null, defaults to [ColorScheme.secondaryContainer].
  final Color? indicatorColor;

  /// Overrides the default value of [NavigationRail]'s selection indicator shape,
  /// when [useIndicator] is true.
  ///
  /// If this is null, [NavigationRailThemeData.indicatorShape] is used. If
  /// that is null, defaults to [StadiumBorder].
  final ShapeBorder? indicatorShape;

  /// Returns the animation that controls the [NavigationRail.extended] state.
  ///
  /// This can be used to synchronize animations in the [leading] or [trailing]
  /// widget, such as an animated menu or a [FloatingActionButton] animation.
  ///
  /// {@tool dartpad}
  /// This example shows how to use this animation to create a [FloatingActionButton]
  /// that animates itself between the normal and extended states of the
  /// [NavigationRail].
  ///
  /// An instance of `MyNavigationRailFab` is created for [NavigationRail.leading].
  /// Pressing the FAB button toggles the "extended" state of the [NavigationRail].
  ///
  /// ** See code in examples/api/lib/material/navigation_rail/navigation_rail.extended_animation.0.dart **
  /// {@end-tool}
  static Animation<double> extendedAnimation(BuildContext context) {
    return context
        .dependOnInheritedWidgetOfExactType<_ExtendedNavigationRailAnimation>()!
        .animation;
  }

  @override
  State<NavigationRail> createState() => _NavigationRailState();
}

class _NavigationRailState extends State<NavigationRail> with TickerProviderStateMixin {
  late List<AnimationController> _destinationControllers;
  late List<Animation<double>> _destinationAnimations;
  late AnimationController _extendedController;
  late CurvedAnimation _extendedAnimation;

  @override
  void initState() {
    super.initState();
    _initControllers();
  }

  @override
  void dispose() {
    _disposeControllers();
    super.dispose();
  }

  @override
  void didUpdateWidget(NavigationRail oldWidget) {
    super.didUpdateWidget(oldWidget);

    if (widget.extended != oldWidget.extended) {
      if (widget.extended) {
        _extendedController.forward();
      } else {
        _extendedController.reverse();
      }
    }

    // No animated segue if the length of the items list changes.
    if (widget.destinations.length != oldWidget.destinations.length) {
      _resetState();
      return;
    }

    if (widget.selectedIndex != oldWidget.selectedIndex) {
      if (oldWidget.selectedIndex != null) {
        _destinationControllers[oldWidget.selectedIndex!].reverse();
      }
      if (widget.selectedIndex != null) {
        _destinationControllers[widget.selectedIndex!].forward();
      }
      return;
    }
  }

  @override
  Widget build(BuildContext context) {
    final NavigationRailThemeData navigationRailTheme = NavigationRailTheme.of(context);
    final NavigationRailThemeData defaults =
        Theme.of(context).useMaterial3
            ? _NavigationRailDefaultsM3(context)
            : _NavigationRailDefaultsM2(context);
    final MaterialLocalizations localizations = MaterialLocalizations.of(context);

    final Color backgroundColor =
        widget.backgroundColor ?? navigationRailTheme.backgroundColor ?? defaults.backgroundColor!;
    final double elevation =
        widget.elevation ?? navigationRailTheme.elevation ?? defaults.elevation!;
    final double minWidth = widget.minWidth ?? navigationRailTheme.minWidth ?? defaults.minWidth!;
    final double minExtendedWidth =
        widget.minExtendedWidth ??
        navigationRailTheme.minExtendedWidth ??
        defaults.minExtendedWidth!;
    final TextStyle unselectedLabelTextStyle =
        widget.unselectedLabelTextStyle ??
        navigationRailTheme.unselectedLabelTextStyle ??
        defaults.unselectedLabelTextStyle!;
    final TextStyle selectedLabelTextStyle =
        widget.selectedLabelTextStyle ??
        navigationRailTheme.selectedLabelTextStyle ??
        defaults.selectedLabelTextStyle!;
    final IconThemeData unselectedIconTheme =
        widget.unselectedIconTheme ??
        navigationRailTheme.unselectedIconTheme ??
        defaults.unselectedIconTheme!;
    final IconThemeData selectedIconTheme =
        widget.selectedIconTheme ??
        navigationRailTheme.selectedIconTheme ??
        defaults.selectedIconTheme!;
    final double groupAlignment =
        widget.groupAlignment ?? navigationRailTheme.groupAlignment ?? defaults.groupAlignment!;
    final NavigationRailLabelType labelType =
        widget.labelType ?? navigationRailTheme.labelType ?? defaults.labelType!;
    final bool useIndicator =
        widget.useIndicator ?? navigationRailTheme.useIndicator ?? defaults.useIndicator!;
    final Color? indicatorColor =
        widget.indicatorColor ?? navigationRailTheme.indicatorColor ?? defaults.indicatorColor;
    final ShapeBorder? indicatorShape =
        widget.indicatorShape ?? navigationRailTheme.indicatorShape ?? defaults.indicatorShape;

    // For backwards compatibility, in M2 the opacity of the unselected icons needs
    // to be set to the default if it isn't in the given theme. This can be removed
    // when Material 3 is the default.
    final IconThemeData effectiveUnselectedIconTheme =
        Theme.of(context).useMaterial3
            ? unselectedIconTheme
            : unselectedIconTheme.copyWith(
              opacity: unselectedIconTheme.opacity ?? defaults.unselectedIconTheme!.opacity,
            );

    final bool isRTLDirection = Directionality.of(context) == TextDirection.rtl;

    return _ExtendedNavigationRailAnimation(
      animation: _extendedAnimation,
      child: Semantics(
        explicitChildNodes: true,
        child: Material(
          elevation: elevation,
          color: backgroundColor,
          child: SafeArea(
            right: isRTLDirection,
            left: !isRTLDirection,
            child: Column(
              children: <Widget>[
                _verticalSpacer,
                if (widget.leading != null) ...<Widget>[widget.leading!, _verticalSpacer],
                Expanded(
                  child: Align(
                    alignment: Alignment(0, groupAlignment),
                    child: Column(
                      mainAxisSize: MainAxisSize.min,
                      children: <Widget>[
                        for (int i = 0; i < widget.destinations.length; i += 1)
                          _RailDestination(
                            minWidth: minWidth,
                            minExtendedWidth: minExtendedWidth,
                            extendedTransitionAnimation: _extendedAnimation,
                            selected: widget.selectedIndex == i,
                            icon:
                                widget.selectedIndex == i
                                    ? widget.destinations[i].selectedIcon
                                    : widget.destinations[i].icon,
                            label: widget.destinations[i].label,
                            destinationAnimation: _destinationAnimations[i],
                            labelType: labelType,
                            iconTheme:
                                widget.selectedIndex == i
                                    ? selectedIconTheme
                                    : effectiveUnselectedIconTheme,
                            labelTextStyle:
                                widget.selectedIndex == i
                                    ? selectedLabelTextStyle
                                    : unselectedLabelTextStyle,
                            padding: widget.destinations[i].padding,
                            useIndicator: useIndicator,
                            indicatorColor: useIndicator ? indicatorColor : null,
                            indicatorShape: useIndicator ? indicatorShape : null,
                            onTap: () {
                              if (widget.onDestinationSelected != null) {
                                widget.onDestinationSelected!(i);
                              }
                            },
                            indexLabel: localizations.tabLabel(
                              tabIndex: i + 1,
                              tabCount: widget.destinations.length,
                            ),
                            disabled: widget.destinations[i].disabled,
                          ),
                        if (widget.trailing != null) widget.trailing!,
                      ],
                    ),
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }

  void _disposeControllers() {
    for (final AnimationController controller in _destinationControllers) {
      controller.dispose();
    }
    _extendedController.dispose();
    _extendedAnimation.dispose();
  }

  void _initControllers() {
    _destinationControllers = List<AnimationController>.generate(widget.destinations.length, (
      int index,
    ) {
      return AnimationController(duration: kThemeAnimationDuration, vsync: this)
        ..addListener(_rebuild);
    });
    _destinationAnimations =
        _destinationControllers.map((AnimationController controller) => controller.view).toList();
    if (widget.selectedIndex != null) {
      _destinationControllers[widget.selectedIndex!].value = 1.0;
    }
    _extendedController = AnimationController(
      duration: kThemeAnimationDuration,
      vsync: this,
      value: widget.extended ? 1.0 : 0.0,
    );
    _extendedAnimation = CurvedAnimation(parent: _extendedController, curve: Curves.easeInOut);
    _extendedController.addListener(() {
      _rebuild();
    });
  }

  void _resetState() {
    _disposeControllers();
    _initControllers();
  }

  void _rebuild() {
    setState(() {
      // Rebuilding when any of the controllers tick, i.e. when the items are
      // animating.
    });
  }
}

class _RailDestination extends StatefulWidget {
  const _RailDestination({
    required this.minWidth,
    required this.minExtendedWidth,
    required this.icon,
    required this.label,
    required this.destinationAnimation,
    required this.extendedTransitionAnimation,
    required this.labelType,
    required this.selected,
    required this.iconTheme,
    required this.labelTextStyle,
    required this.onTap,
    required this.indexLabel,
    this.padding,
    required this.useIndicator,
    this.indicatorColor,
    this.indicatorShape,
    this.disabled = false,
  });

  final double minWidth;
  final double minExtendedWidth;
  final Widget icon;
  final Widget label;
  final Animation<double> destinationAnimation;
  final NavigationRailLabelType labelType;
  final bool selected;
  final Animation<double> extendedTransitionAnimation;
  final IconThemeData iconTheme;
  final TextStyle labelTextStyle;
  final VoidCallback onTap;
  final String indexLabel;
  final EdgeInsetsGeometry? padding;
  final bool useIndicator;
  final Color? indicatorColor;
  final ShapeBorder? indicatorShape;
  final bool disabled;

  @override
  State<_RailDestination> createState() => _RailDestinationState();
}

class _RailDestinationState extends State<_RailDestination> {
  late CurvedAnimation _positionAnimation;

  @override
  void initState() {
    super.initState();
    _setPositionAnimation();
  }

  @override
  void didUpdateWidget(_RailDestination oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.destinationAnimation != oldWidget.destinationAnimation) {
      _positionAnimation.dispose();
      _setPositionAnimation();
    }
  }

  void _setPositionAnimation() {
    _positionAnimation = CurvedAnimation(
      parent: ReverseAnimation(widget.destinationAnimation),
      curve: Curves.easeInOut,
      reverseCurve: Curves.easeInOut.flipped,
    );
  }

  @override
  void dispose() {
    _positionAnimation.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    assert(
      widget.useIndicator || widget.indicatorColor == null,
      '[NavigationRail.indicatorColor] does not have an effect when [NavigationRail.useIndicator] is false',
    );

    final ThemeData theme = Theme.of(context);
    final TextDirection textDirection = Directionality.of(context);
    final bool material3 = theme.useMaterial3;
    final EdgeInsets destinationPadding = (widget.padding ?? EdgeInsets.zero).resolve(
      textDirection,
    );
    Offset indicatorOffset;
    bool applyXOffset = false;

    final Widget themedIcon = IconTheme(
      data:
          widget.disabled
              ? widget.iconTheme.copyWith(color: theme.colorScheme.onSurface.withOpacity(0.38))
              : widget.iconTheme,
      child: widget.icon,
    );
    final Widget styledLabel = DefaultTextStyle(
      style:
          widget.disabled
              ? widget.labelTextStyle.copyWith(color: theme.colorScheme.onSurface.withOpacity(0.38))
              : widget.labelTextStyle,
      child: widget.label,
    );

    Widget content;

    // The indicator height is fixed and equal to _kIndicatorHeight.
    // When the icon height is larger than the indicator height the indicator
    // vertical offset is used to vertically center the indicator.
    final bool isLargeIconSize =
        widget.iconTheme.size != null && widget.iconTheme.size! > _kIndicatorHeight;
    final double indicatorVerticalOffset =
        isLargeIconSize ? (widget.iconTheme.size! - _kIndicatorHeight) / 2 : 0;

    switch (widget.labelType) {
      case NavigationRailLabelType.none:
        // Split the destination spacing across the top and bottom to keep the icon centered.
        final Widget? spacing =
            material3 ? const SizedBox(height: _verticalDestinationSpacingM3 / 2) : null;
        indicatorOffset = Offset(
          widget.minWidth / 2 + destinationPadding.left,
          _verticalDestinationSpacingM3 / 2 + destinationPadding.top + indicatorVerticalOffset,
        );
        final Widget iconPart = Column(
          children: <Widget>[
            if (spacing != null) spacing,
            SizedBox(
              width: widget.minWidth,
              height: material3 ? null : widget.minWidth,
              child: Center(
                child: _AddIndicator(
                  addIndicator: widget.useIndicator,
                  indicatorColor: widget.indicatorColor,
                  indicatorShape: widget.indicatorShape,
                  isCircular: !material3,
                  indicatorAnimation: widget.destinationAnimation,
                  child: themedIcon,
                ),
              ),
            ),
            if (spacing != null) spacing,
          ],
        );
        if (widget.extendedTransitionAnimation.value == 0) {
          content = Padding(
            padding: widget.padding ?? EdgeInsets.zero,
            child: Stack(
              children: <Widget>[
                iconPart,
                // For semantics when label is not showing,
                SizedBox.shrink(child: Visibility.maintain(visible: false, child: widget.label)),
              ],
            ),
          );
        } else {
          final Animation<double> labelFadeAnimation = widget.extendedTransitionAnimation.drive(
            CurveTween(curve: const Interval(0.0, 0.25)),
          );
          applyXOffset = true;
          content = Padding(
            padding: widget.padding ?? EdgeInsets.zero,
            child: ConstrainedBox(
              constraints: BoxConstraints(
                minWidth:
                    lerpDouble(
                      widget.minWidth,
                      widget.minExtendedWidth,
                      widget.extendedTransitionAnimation.value,
                    )!,
              ),
              child: ClipRect(
                child: Row(
                  mainAxisSize: MainAxisSize.min,
                  children: <Widget>[
                    iconPart,
                    Flexible(
                      child: Align(
                        heightFactor: 1.0,
                        widthFactor: widget.extendedTransitionAnimation.value,
                        alignment: AlignmentDirectional.centerStart,
                        child: FadeTransition(
                          alwaysIncludeSemantics: true,
                          opacity: labelFadeAnimation,
                          child: styledLabel,
                        ),
                      ),
                    ),
                    SizedBox(
                      width:
                          _horizontalDestinationPadding * widget.extendedTransitionAnimation.value,
                    ),
                  ],
                ),
              ),
            ),
          );
        }
      case NavigationRailLabelType.selected:
        final double appearingAnimationValue = 1 - _positionAnimation.value;
        final double verticalPadding =
            lerpDouble(
              _verticalDestinationPaddingNoLabel,
              _verticalDestinationPaddingWithLabel,
              appearingAnimationValue,
            )!;
        final Interval interval =
            widget.selected ? const Interval(0.25, 0.75) : const Interval(0.75, 1.0);
        final Animation<double> labelFadeAnimation = widget.destinationAnimation.drive(
          CurveTween(curve: interval),
        );
        final double minHeight = material3 ? 0 : widget.minWidth;
        final Widget topSpacing = SizedBox(height: material3 ? 0 : verticalPadding);
        final Widget labelSpacing = SizedBox(
          height:
              material3 ? lerpDouble(0, _verticalIconLabelSpacingM3, appearingAnimationValue)! : 0,
        );
        final Widget bottomSpacing = SizedBox(
          height: material3 ? _verticalDestinationSpacingM3 : verticalPadding,
        );
        final double indicatorHorizontalPadding =
            (destinationPadding.left / 2) - (destinationPadding.right / 2);
        final double indicatorVerticalPadding = destinationPadding.top;
        indicatorOffset = Offset(
          widget.minWidth / 2 + indicatorHorizontalPadding,
          indicatorVerticalPadding + indicatorVerticalOffset,
        );
        if (widget.minWidth < _NavigationRailDefaultsM2(context).minWidth!) {
          indicatorOffset = Offset(
            widget.minWidth / 2 + _horizontalDestinationSpacingM3,
            indicatorVerticalPadding + indicatorVerticalOffset,
          );
        }
        content = ConstrainedBox(
          constraints: BoxConstraints(minWidth: widget.minWidth, minHeight: minHeight),
          child: Padding(
            padding:
                widget.padding ??
                const EdgeInsets.symmetric(horizontal: _horizontalDestinationPadding),
            child: ClipRect(
              child: Column(
                mainAxisSize: MainAxisSize.min,
                mainAxisAlignment: MainAxisAlignment.center,
                children: <Widget>[
                  topSpacing,
                  _AddIndicator(
                    addIndicator: widget.useIndicator,
                    indicatorColor: widget.indicatorColor,
                    indicatorShape: widget.indicatorShape,
                    isCircular: false,
                    indicatorAnimation: widget.destinationAnimation,
                    child: themedIcon,
                  ),
                  labelSpacing,
                  Align(
                    alignment: Alignment.topCenter,
                    heightFactor: appearingAnimationValue,
                    widthFactor: 1.0,
                    child: FadeTransition(
                      alwaysIncludeSemantics: true,
                      opacity: labelFadeAnimation,
                      child: styledLabel,
                    ),
                  ),
                  bottomSpacing,
                ],
              ),
            ),
          ),
        );
      case NavigationRailLabelType.all:
        final double minHeight = material3 ? 0 : widget.minWidth;
        final Widget topSpacing = SizedBox(
          height: material3 ? 0 : _verticalDestinationPaddingWithLabel,
        );
        final Widget labelSpacing = SizedBox(height: material3 ? _verticalIconLabelSpacingM3 : 0);
        final Widget bottomSpacing = SizedBox(
          height: material3 ? _verticalDestinationSpacingM3 : _verticalDestinationPaddingWithLabel,
        );
        final double indicatorHorizontalPadding =
            (destinationPadding.left / 2) - (destinationPadding.right / 2);
        final double indicatorVerticalPadding = destinationPadding.top;
        indicatorOffset = Offset(
          widget.minWidth / 2 + indicatorHorizontalPadding,
          indicatorVerticalPadding + indicatorVerticalOffset,
        );
        if (widget.minWidth < _NavigationRailDefaultsM2(context).minWidth!) {
          indicatorOffset = Offset(
            widget.minWidth / 2 + _horizontalDestinationSpacingM3,
            indicatorVerticalPadding + indicatorVerticalOffset,
          );
        }
        content = ConstrainedBox(
          constraints: BoxConstraints(minWidth: widget.minWidth, minHeight: minHeight),
          child: Padding(
            padding:
                widget.padding ??
                const EdgeInsets.symmetric(horizontal: _horizontalDestinationPadding),
            child: Column(
              children: <Widget>[
                topSpacing,
                _AddIndicator(
                  addIndicator: widget.useIndicator,
                  indicatorColor: widget.indicatorColor,
                  indicatorShape: widget.indicatorShape,
                  isCircular: false,
                  indicatorAnimation: widget.destinationAnimation,
                  child: themedIcon,
                ),
                labelSpacing,
                styledLabel,
                bottomSpacing,
              ],
            ),
          ),
        );
    }

    final ColorScheme colors = Theme.of(context).colorScheme;
    final bool primaryColorAlphaModified = colors.primary.alpha < 255.0;
    final Color effectiveSplashColor =
        primaryColorAlphaModified ? colors.primary : colors.primary.withOpacity(0.12);
    final Color effectiveHoverColor =
        primaryColorAlphaModified ? colors.primary : colors.primary.withOpacity(0.04);
    return Semantics(
      container: true,
      selected: widget.selected,
      child: Stack(
        children: <Widget>[
          Material(
            type: MaterialType.transparency,
            child: _IndicatorInkWell(
              onTap: widget.disabled ? null : widget.onTap,
              borderRadius: BorderRadius.all(Radius.circular(widget.minWidth / 2.0)),
              customBorder: widget.indicatorShape,
              splashColor: effectiveSplashColor,
              hoverColor: effectiveHoverColor,
              useMaterial3: material3,
              indicatorOffset: indicatorOffset,
              applyXOffset: applyXOffset,
              textDirection: textDirection,
              child: content,
            ),
          ),
          Semantics(label: widget.indexLabel),
        ],
      ),
    );
  }
}

class _IndicatorInkWell extends InkResponse {
  const _IndicatorInkWell({
    super.child,
    super.onTap,
    ShapeBorder? customBorder,
    BorderRadius? borderRadius,
    super.splashColor,
    super.hoverColor,
    required this.useMaterial3,
    required this.indicatorOffset,
    required this.applyXOffset,
    required this.textDirection,
  }) : super(
         containedInkWell: true,
         highlightShape: BoxShape.rectangle,
         borderRadius: useMaterial3 ? null : borderRadius,
         customBorder: useMaterial3 ? customBorder : null,
       );

  final bool useMaterial3;

  // The offset used to position Ink highlight.
  final Offset indicatorOffset;

  // Whether the horizontal offset from indicatorOffset should be used to position Ink highlight.
  // If true, Ink highlight uses the indicator horizontal offset. If false, Ink highlight is centered horizontally.
  final bool applyXOffset;

  // The text direction used to adjust the indicator horizontal offset.
  final TextDirection textDirection;

  @override
  RectCallback? getRectCallback(RenderBox referenceBox) {
    if (useMaterial3) {
      final double boxWidth = referenceBox.size.width;
      double indicatorHorizontalCenter = applyXOffset ? indicatorOffset.dx : boxWidth / 2;
      if (textDirection == TextDirection.rtl) {
        indicatorHorizontalCenter = boxWidth - indicatorHorizontalCenter;
      }
      return () {
        return Rect.fromLTWH(
          indicatorHorizontalCenter - (_kCircularIndicatorDiameter / 2),
          indicatorOffset.dy,
          _kCircularIndicatorDiameter,
          _kIndicatorHeight,
        );
      };
    }
    return null;
  }
}

/// When [addIndicator] is `true`, puts [child] center aligned in a [Stack] with
/// a [NavigationIndicator] behind it, otherwise returns [child].
///
/// When [isCircular] is true, the indicator will be a circle, otherwise the
/// indicator will be a stadium shape.
class _AddIndicator extends StatelessWidget {
  const _AddIndicator({
    required this.addIndicator,
    required this.isCircular,
    required this.indicatorColor,
    required this.indicatorShape,
    required this.indicatorAnimation,
    required this.child,
  });

  final bool addIndicator;
  final bool isCircular;
  final Color? indicatorColor;
  final ShapeBorder? indicatorShape;
  final Animation<double> indicatorAnimation;
  final Widget child;

  @override
  Widget build(BuildContext context) {
    if (!addIndicator) {
      return child;
    }
    late final Widget indicator;
    if (isCircular) {
      indicator = NavigationIndicator(
        animation: indicatorAnimation,
        height: _kCircularIndicatorDiameter,
        width: _kCircularIndicatorDiameter,
        borderRadius: BorderRadius.circular(_kCircularIndicatorDiameter / 2),
        color: indicatorColor,
      );
    } else {
      indicator = NavigationIndicator(
        animation: indicatorAnimation,
        width: _kCircularIndicatorDiameter,
        shape: indicatorShape,
        color: indicatorColor,
      );
    }

    return Stack(alignment: Alignment.center, children: <Widget>[indicator, child]);
  }
}

/// Defines the behavior of the labels of a [NavigationRail].
///
/// See also:
///
///   * [NavigationRail]
enum NavigationRailLabelType {
  /// Only the [NavigationRailDestination]s are shown.
  none,

  /// Only the selected [NavigationRailDestination] will show its label.
  ///
  /// The label will animate in and out as new [NavigationRailDestination]s are
  /// selected.
  selected,

  /// All [NavigationRailDestination]s will show their label.
  all,
}

/// Defines a [NavigationRail] button that represents one "destination" view.
///
/// See also:
///
///  * [NavigationRail]
class NavigationRailDestination {
  /// Creates a destination that is used with [NavigationRail.destinations].
  ///
  /// When the [NavigationRail.labelType] is [NavigationRailLabelType.none], the
  /// label is still used for semantics, and may still be used if
  /// [NavigationRail.extended] is true.
  const NavigationRailDestination({
    required this.icon,
    Widget? selectedIcon,
    this.indicatorColor,
    this.indicatorShape,
    required this.label,
    this.padding,
    this.disabled = false,
  }) : selectedIcon = selectedIcon ?? icon;

  /// The icon of the destination.
  ///
  /// Typically the icon is an [Icon] or an [ImageIcon] widget. If another type
  /// of widget is provided then it should configure itself to match the current
  /// [IconTheme] size and color.
  ///
  /// If [selectedIcon] is provided, this will only be displayed when the
  /// destination is not selected.
  ///
  /// To make the [NavigationRail] more accessible, consider choosing an
  /// icon with a stroked and filled version, such as [Icons.cloud] and
  /// [Icons.cloud_queue]. The [icon] should be set to the stroked version and
  /// [selectedIcon] to the filled version.
  final Widget icon;

  /// An alternative icon displayed when this destination is selected.
  ///
  /// If this icon is not provided, the [NavigationRail] will display [icon] in
  /// either state. The size, color, and opacity of the
  /// [NavigationRail.selectedIconTheme] will still apply.
  ///
  /// See also:
  ///
  ///  * [NavigationRailDestination.icon], for a description of how to pair
  ///    icons.
  final Widget selectedIcon;

  /// The color of the [indicatorShape] when this destination is selected.
  final Color? indicatorColor;

  /// The shape of the selection indicator.
  final ShapeBorder? indicatorShape;

  /// The label for the destination.
  ///
  /// The label must be provided when used with the [NavigationRail]. When the
  /// [NavigationRail.labelType] is [NavigationRailLabelType.none], the label is
  /// still used for semantics, and may still be used if
  /// [NavigationRail.extended] is true.
  final Widget label;

  /// The amount of space to inset the destination item.
  final EdgeInsetsGeometry? padding;

  /// Indicates that this destination is inaccessible.
  final bool disabled;
}

class _ExtendedNavigationRailAnimation extends InheritedWidget {
  const _ExtendedNavigationRailAnimation({required this.animation, required super.child});

  final Animation<double> animation;

  @override
  bool updateShouldNotify(_ExtendedNavigationRailAnimation old) => animation != old.animation;
}

// There don't appear to be tokens for these values, but they are
// shown in the spec.
const double _horizontalDestinationPadding = 8.0;
const double _verticalDestinationPaddingNoLabel = 24.0;
const double _verticalDestinationPaddingWithLabel = 16.0;
const Widget _verticalSpacer = SizedBox(height: 8.0);
const double _verticalIconLabelSpacingM3 = 4.0;
const double _verticalDestinationSpacingM3 = 12.0;
const double _horizontalDestinationSpacingM3 = 12.0;

// Hand coded defaults based on Material Design 2.
class _NavigationRailDefaultsM2 extends NavigationRailThemeData {
  _NavigationRailDefaultsM2(BuildContext context)
    : _theme = Theme.of(context),
      _colors = Theme.of(context).colorScheme,
      super(
        elevation: 0,
        groupAlignment: -1,
        labelType: NavigationRailLabelType.none,
        useIndicator: false,
        minWidth: 72.0,
        minExtendedWidth: 256,
      );

  final ThemeData _theme;
  final ColorScheme _colors;

  @override
  Color? get backgroundColor => _colors.surface;

  @override
  TextStyle? get unselectedLabelTextStyle {
    return _theme.textTheme.bodyLarge!.copyWith(color: _colors.onSurface.withOpacity(0.64));
  }

  @override
  TextStyle? get selectedLabelTextStyle {
    return _theme.textTheme.bodyLarge!.copyWith(color: _colors.primary);
  }

  @override
  IconThemeData? get unselectedIconTheme {
    return IconThemeData(size: 24.0, color: _colors.onSurface, opacity: 0.64);
  }

  @override
  IconThemeData? get selectedIconTheme {
    return IconThemeData(size: 24.0, color: _colors.primary, opacity: 1.0);
  }
}

// BEGIN GENERATED TOKEN PROPERTIES - NavigationRail

// Do not edit by hand. The code between the "BEGIN GENERATED" and
// "END GENERATED" comments are generated from data in the Material
// Design token database by the script:
//   dev/tools/gen_defaults/bin/gen_defaults.dart.

// dart format off
class _NavigationRailDefaultsM3 extends NavigationRailThemeData {
  _NavigationRailDefaultsM3(this.context)
    : super(
        elevation: 0.0,
        groupAlignment: -1,
        labelType: NavigationRailLabelType.none,
        useIndicator: true,
        minWidth: 80.0,
        minExtendedWidth: 256,
      );

  final BuildContext context;
  late final ColorScheme _colors = Theme.of(context).colorScheme;
  late final TextTheme _textTheme = Theme.of(context).textTheme;

  @override Color? get backgroundColor => _colors.surface;

  @override TextStyle? get unselectedLabelTextStyle {
    return _textTheme.labelMedium!.copyWith(color: _colors.onSurface);
  }

  @override TextStyle? get selectedLabelTextStyle {
    return _textTheme.labelMedium!.copyWith(color: _colors.onSurface);
  }

  @override IconThemeData? get unselectedIconTheme {
    return IconThemeData(
      size: 24.0,
      color: _colors.onSurfaceVariant,
    );
  }

  @override IconThemeData? get selectedIconTheme {
    return IconThemeData(
      size: 24.0,
      color: _colors.onSecondaryContainer,
    );
  }

  @override Color? get indicatorColor => _colors.secondaryContainer;

  @override ShapeBorder? get indicatorShape => const StadiumBorder();
}
// dart format on

// END GENERATED TOKEN PROPERTIES - NavigationRail
