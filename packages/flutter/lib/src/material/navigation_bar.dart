// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'bottom_navigation_bar.dart';
/// @docImport 'navigation_rail.dart';
/// @docImport 'scaffold.dart';
library;

import 'package:flutter/widgets.dart';

import 'color_scheme.dart';
import 'colors.dart';
import 'elevation_overlay.dart';
import 'ink_well.dart';
import 'material.dart';
import 'material_localizations.dart';
import 'material_state.dart';
import 'navigation_bar_theme.dart';
import 'text_theme.dart';
import 'theme.dart';
import 'tooltip.dart';

const double _kIndicatorHeight = 32;
const double _kIndicatorWidth = 64;
const double _kMaxLabelTextScaleFactor = 1.3;

// Examples can assume:
// late BuildContext context;
// late bool _isDrawerOpen;

/// Material 3 Navigation Bar component.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=DVGYddFaLv0}
///
/// Navigation bars offer a persistent and convenient way to switch between
/// primary destinations in an app.
///
/// This widget does not adjust its size with the [ThemeData.visualDensity].
///
/// The [MediaQueryData.textScaler] does not adjust the size of this widget but
/// rather the size of the [Tooltip]s displayed on long presses of the
/// destinations.
///
/// The style for the icons and text are not affected by parent
/// [DefaultTextStyle]s or [IconTheme]s but rather controlled by parameters or
/// the [NavigationBarThemeData].
///
/// This widget holds a collection of destinations (usually
/// [NavigationDestination]s).
///
/// {@tool dartpad}
/// This example shows a [NavigationBar] as it is used within a [Scaffold]
/// widget. The [NavigationBar] has three [NavigationDestination] widgets and
/// the initial [selectedIndex] is set to index 0. The [onDestinationSelected]
/// callback changes the selected item's index and displays a corresponding
/// widget in the body of the [Scaffold].
///
/// ** See code in examples/api/lib/material/navigation_bar/navigation_bar.0.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example showcases [NavigationBar] label behaviors. When tapping on one
/// of the label behavior options, the [labelBehavior] of the [NavigationBar]
/// will be updated.
///
/// ** See code in examples/api/lib/material/navigation_bar/navigation_bar.1.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example shows a [NavigationBar] within a main [Scaffold]
/// widget that's used to control the visibility of destination pages.
/// Each destination has its own scaffold and a nested navigator that
/// provides local navigation. The example's [NavigationBar] has four
/// [NavigationDestination] widgets with different color schemes. Its
/// [onDestinationSelected] callback changes the selected
/// destination's index and displays a corresponding page with its own
/// local navigator and scaffold - all within the body of the main
/// scaffold. The destination pages are organized in a [Stack] and
/// switching destinations fades out the current page and
/// fades in the new one. Destinations that aren't visible or animating
/// are kept [Offstage].
///
/// ** See code in examples/api/lib/material/navigation_bar/navigation_bar.2.dart **
/// {@end-tool}
/// See also:
///
///  * [NavigationDestination]
///  * [BottomNavigationBar]
///  * <https://api.flutter.dev/flutter/material/NavigationDestination-class.html>
///  * <https://m3.material.io/components/navigation-bar>
class NavigationBar extends StatelessWidget {
  /// Creates a Material 3 Navigation Bar component.
  ///
  /// The value of [destinations] must be a list of two or more
  /// [NavigationDestination] values.
  // TODO(goderbauer): This class cannot be const constructed, https://github.com/dart-lang/linter/issues/3366.
  // ignore: prefer_const_constructors_in_immutables
  NavigationBar({
    super.key,
    this.animationDuration,
    this.selectedIndex = 0,
    required this.destinations,
    this.onDestinationSelected,
    this.backgroundColor,
    this.elevation,
    this.shadowColor,
    this.surfaceTintColor,
    this.indicatorColor,
    this.indicatorShape,
    this.height,
    this.labelBehavior,
    this.overlayColor,
    this.labelTextStyle,
    this.labelPadding,
  }) : assert(destinations.length >= 2),
       assert(0 <= selectedIndex && selectedIndex < destinations.length);

  /// Determines the transition time for each destination as it goes between
  /// selected and unselected.
  final Duration? animationDuration;

  /// Determines which one of the [destinations] is currently selected.
  ///
  /// When this is updated, the destination (from [destinations]) at
  /// [selectedIndex] goes from unselected to selected.
  final int selectedIndex;

  /// The list of destinations (usually [NavigationDestination]s) in this
  /// [NavigationBar].
  ///
  /// When [selectedIndex] is updated, the destination from this list at
  /// [selectedIndex] will animate from 0 (unselected) to 1.0 (selected). When
  /// the animation is increasing or completed, the destination is considered
  /// selected, when the animation is decreasing or dismissed, the destination
  /// is considered unselected.
  final List<Widget> destinations;

  /// Called when one of the [destinations] is selected.
  ///
  /// This callback usually updates the int passed to [selectedIndex].
  ///
  /// Upon updating [selectedIndex], the [NavigationBar] will be rebuilt.
  final ValueChanged<int>? onDestinationSelected;

  /// The color of the [NavigationBar] itself.
  ///
  /// If null, [NavigationBarThemeData.backgroundColor] is used. If that
  /// is also null, then if [ThemeData.useMaterial3] is true, the value is
  /// [ColorScheme.surfaceContainer]. If that is false, the default blends [ColorScheme.surface]
  /// and [ColorScheme.onSurface] using an [ElevationOverlay].
  final Color? backgroundColor;

  /// The elevation of the [NavigationBar] itself.
  ///
  /// If null, [NavigationBarThemeData.elevation] is used. If that
  /// is also null, then if [ThemeData.useMaterial3] is true then it will
  /// be 3.0 otherwise 0.0.
  final double? elevation;

  /// The color used for the drop shadow to indicate elevation.
  ///
  /// If null, [NavigationBarThemeData.shadowColor] is used. If that
  /// is also null, the default value is [Colors.transparent] which
  /// indicates that no drop shadow will be displayed.
  ///
  /// See [Material.shadowColor] for more details on drop shadows.
  final Color? shadowColor;

  /// The color used as an overlay on [backgroundColor] to indicate elevation.
  ///
  /// This is not recommended for use. [Material 3 spec](https://m3.material.io/styles/color/the-color-system/color-roles)
  /// introduced a set of tone-based surfaces and surface containers in its [ColorScheme],
  /// which provide more flexibility. The intention is to eventually remove surface tint color from
  /// the framework.
  ///
  /// If null, [NavigationBarThemeData.surfaceTintColor] is used. If that
  /// is also null, the default value is [Colors.transparent].
  ///
  /// See [Material.surfaceTintColor] for more details on how this
  /// overlay is applied.
  final Color? surfaceTintColor;

  /// The color of the [indicatorShape] when this destination is selected.
  ///
  /// If null, [NavigationBarThemeData.indicatorColor] is used. If that
  /// is also null and [ThemeData.useMaterial3] is true, [ColorScheme.secondaryContainer]
  /// is used. Otherwise, [ColorScheme.secondary] with an opacity of 0.24 is used.
  final Color? indicatorColor;

  /// The shape of the selected indicator.
  ///
  /// If null, [NavigationBarThemeData.indicatorShape] is used. If that
  /// is also null and [ThemeData.useMaterial3] is true, [StadiumBorder] is used.
  /// Otherwise, [RoundedRectangleBorder] with a circular border radius of 16 is used.
  final ShapeBorder? indicatorShape;

  /// The height of the [NavigationBar] itself.
  ///
  /// If this is used in [Scaffold.bottomNavigationBar] and the scaffold is
  /// full-screen, the safe area padding is also added to the height
  /// automatically.
  ///
  /// The height does not adjust with [ThemeData.visualDensity] or
  /// [MediaQueryData.textScaler] as this component loses usability at
  /// larger and smaller sizes due to the truncating of labels or smaller tap
  /// targets.
  ///
  /// If null, [NavigationBarThemeData.height] is used. If that
  /// is also null, the default is 80.
  final double? height;

  /// Defines how the [destinations]' labels will be laid out and when they'll
  /// be displayed.
  ///
  /// Can be used to show all labels, show only the selected label, or hide all
  /// labels.
  ///
  /// If null, [NavigationBarThemeData.labelBehavior] is used. If that
  /// is also null, the default is
  /// [NavigationDestinationLabelBehavior.alwaysShow].
  final NavigationDestinationLabelBehavior? labelBehavior;

  /// The highlight color that's typically used to indicate that
  /// the [NavigationDestination] is focused, hovered, or pressed.
  final MaterialStateProperty<Color?>? overlayColor;

  //// The text style of the label.
  ///
  /// If null, [NavigationBarThemeData.labelTextStyle] is used. If that
  /// is also null, the default text style is [TextTheme.labelMedium] with
  /// [ColorScheme.onSurface] when the destination is selected, and
  /// [ColorScheme.onSurfaceVariant] when the destination is unselected, and
  /// [ColorScheme.onSurfaceVariant] with an opacity of 0.38 when the
  /// destination is disabled.
  ///
  /// If [ThemeData.useMaterial3] is false, then the default text style is
  /// [TextTheme.labelSmall] with [ColorScheme.onSurface].
  final MaterialStateProperty<TextStyle?>? labelTextStyle;

  /// The padding around the [NavigationDestination.label] widget.
  ///
  /// When [labelPadding] is null, [NavigationBarThemeData.labelPadding]
  /// is used. If that is also null, the default padding is 4 pixels on
  /// the top.
  final EdgeInsetsGeometry? labelPadding;

  VoidCallback _handleTap(int index) {
    return onDestinationSelected != null ? () => onDestinationSelected!(index) : () {};
  }

  @override
  Widget build(BuildContext context) {
    final NavigationBarThemeData defaults = _defaultsFor(context);

    final NavigationBarThemeData navigationBarTheme = NavigationBarTheme.of(context);
    final double effectiveHeight = height ?? navigationBarTheme.height ?? defaults.height!;
    final NavigationDestinationLabelBehavior effectiveLabelBehavior =
        labelBehavior ?? navigationBarTheme.labelBehavior ?? defaults.labelBehavior!;

    return Material(
      color: backgroundColor ?? navigationBarTheme.backgroundColor ?? defaults.backgroundColor!,
      elevation: elevation ?? navigationBarTheme.elevation ?? defaults.elevation!,
      shadowColor: shadowColor ?? navigationBarTheme.shadowColor ?? defaults.shadowColor,
      surfaceTintColor:
          surfaceTintColor ?? navigationBarTheme.surfaceTintColor ?? defaults.surfaceTintColor,
      child: SafeArea(
        child: SizedBox(
          height: effectiveHeight,
          child: Row(
            children: <Widget>[
              for (int i = 0; i < destinations.length; i++)
                Expanded(
                  child: _SelectableAnimatedBuilder(
                    duration: animationDuration ?? const Duration(milliseconds: 500),
                    isSelected: i == selectedIndex,
                    builder: (BuildContext context, Animation<double> animation) {
                      return _NavigationDestinationInfo(
                        index: i,
                        selectedIndex: selectedIndex,
                        totalNumberOfDestinations: destinations.length,
                        selectedAnimation: animation,
                        labelBehavior: effectiveLabelBehavior,
                        indicatorColor: indicatorColor,
                        indicatorShape: indicatorShape,
                        overlayColor: overlayColor,
                        onTap: _handleTap(i),
                        labelTextStyle: labelTextStyle,
                        labelPadding: labelPadding,
                        child: destinations[i],
                      );
                    },
                  ),
                ),
            ],
          ),
        ),
      ),
    );
  }
}

/// Specifies when each [NavigationDestination]'s label should appear.
///
/// This is used to determine the behavior of [NavigationBar]'s destinations.
enum NavigationDestinationLabelBehavior {
  /// Always shows all of the labels under each navigation bar destination,
  /// selected and unselected.
  alwaysShow,

  /// Never shows any of the labels under the navigation bar destinations,
  /// regardless of selected vs unselected.
  alwaysHide,

  /// Only shows the labels of the selected navigation bar destination.
  ///
  /// When a destination is unselected, the label will be faded out, and the
  /// icon will be centered.
  ///
  /// When a destination is selected, the label will fade in and the label and
  /// icon will slide up so that they are both centered.
  onlyShowSelected,
}

/// A Material 3 [NavigationBar] destination.
///
/// Displays a label below an icon. Use with [NavigationBar.destinations].
///
/// See also:
///
///  * [NavigationBar], for an interactive code sample.
class NavigationDestination extends StatelessWidget {
  /// Creates a navigation bar destination with an icon and a label, to be used
  /// in the [NavigationBar.destinations].
  const NavigationDestination({
    super.key,
    required this.icon,
    this.selectedIcon,
    required this.label,
    this.tooltip,
    this.enabled = true,
  });

  /// The [Widget] (usually an [Icon]) that's displayed for this
  /// [NavigationDestination].
  ///
  /// The icon will use [NavigationBarThemeData.iconTheme]. If this is
  /// null, the default [IconThemeData] would use a size of 24.0 and
  /// [ColorScheme.onSurface].
  final Widget icon;

  /// The optional [Widget] (usually an [Icon]) that's displayed when this
  /// [NavigationDestination] is selected.
  ///
  /// If [selectedIcon] is non-null, the destination will fade from
  /// [icon] to [selectedIcon] when this destination goes from unselected to
  /// selected.
  ///
  /// The icon will use [NavigationBarThemeData.iconTheme] with
  /// [WidgetState.selected]. If this is null, the default [IconThemeData]
  /// would use a size of 24.0 and [ColorScheme.onSurface].
  final Widget? selectedIcon;

  /// The text label that appears below the icon of this
  /// [NavigationDestination].
  ///
  /// The accompanying [Text] widget will use
  /// [NavigationBarThemeData.labelTextStyle]. If this are null, the default
  /// text style would use [TextTheme.labelSmall] with [ColorScheme.onSurface].
  final String label;

  /// The text to display in the tooltip for this [NavigationDestination], when
  /// the user long presses the destination.
  ///
  /// If [tooltip] is an empty string, no tooltip will be used.
  ///
  /// Defaults to null, in which case the [label] text will be used.
  final String? tooltip;

  /// Indicates that this destination is selectable.
  ///
  /// Defaults to true.
  final bool enabled;

  @override
  Widget build(BuildContext context) {
    final _NavigationDestinationInfo info = _NavigationDestinationInfo.of(context);
    const Set<MaterialState> selectedState = <MaterialState>{MaterialState.selected};
    const Set<MaterialState> unselectedState = <MaterialState>{};
    const Set<MaterialState> disabledState = <MaterialState>{MaterialState.disabled};

    final NavigationBarThemeData navigationBarTheme = NavigationBarTheme.of(context);
    final NavigationBarThemeData defaults = _defaultsFor(context);
    final Animation<double> animation = info.selectedAnimation;

    return _NavigationDestinationBuilder(
      label: label,
      tooltip: tooltip,
      enabled: enabled,
      buildIcon: (BuildContext context) {
        final IconThemeData selectedIconTheme =
            navigationBarTheme.iconTheme?.resolve(selectedState) ??
            defaults.iconTheme!.resolve(selectedState)!;
        final IconThemeData unselectedIconTheme =
            navigationBarTheme.iconTheme?.resolve(unselectedState) ??
            defaults.iconTheme!.resolve(unselectedState)!;
        final IconThemeData disabledIconTheme =
            navigationBarTheme.iconTheme?.resolve(disabledState) ??
            defaults.iconTheme!.resolve(disabledState)!;

        final Widget selectedIconWidget = IconTheme.merge(
          data: enabled ? selectedIconTheme : disabledIconTheme,
          child: selectedIcon ?? icon,
        );
        final Widget unselectedIconWidget = IconTheme.merge(
          data: enabled ? unselectedIconTheme : disabledIconTheme,
          child: icon,
        );

        return Stack(
          alignment: Alignment.center,
          children: <Widget>[
            NavigationIndicator(
              animation: animation,
              color:
                  info.indicatorColor ??
                  navigationBarTheme.indicatorColor ??
                  defaults.indicatorColor!,
              shape:
                  info.indicatorShape ??
                  navigationBarTheme.indicatorShape ??
                  defaults.indicatorShape!,
            ),
            _StatusTransitionWidgetBuilder(
              animation: animation,
              builder: (BuildContext context, Widget? child) {
                return animation.isForwardOrCompleted ? selectedIconWidget : unselectedIconWidget;
              },
            ),
          ],
        );
      },
      buildLabel: (BuildContext context) {
        final TextStyle? effectiveSelectedLabelTextStyle =
            info.labelTextStyle?.resolve(selectedState) ??
            navigationBarTheme.labelTextStyle?.resolve(selectedState) ??
            defaults.labelTextStyle!.resolve(selectedState);
        final TextStyle? effectiveUnselectedLabelTextStyle =
            info.labelTextStyle?.resolve(unselectedState) ??
            navigationBarTheme.labelTextStyle?.resolve(unselectedState) ??
            defaults.labelTextStyle!.resolve(unselectedState);
        final TextStyle? effectiveDisabledLabelTextStyle =
            info.labelTextStyle?.resolve(disabledState) ??
            navigationBarTheme.labelTextStyle?.resolve(disabledState) ??
            defaults.labelTextStyle!.resolve(disabledState);
        final EdgeInsetsGeometry labelPadding =
            info.labelPadding ?? navigationBarTheme.labelPadding ?? defaults.labelPadding!;

        final TextStyle? textStyle =
            enabled
                ? animation.isForwardOrCompleted
                    ? effectiveSelectedLabelTextStyle
                    : effectiveUnselectedLabelTextStyle
                : effectiveDisabledLabelTextStyle;

        return Padding(
          padding: labelPadding,
          child: MediaQuery.withClampedTextScaling(
            // Set maximum text scale factor to _kMaxLabelTextScaleFactor for the
            // label to keep the visual hierarchy the same even with larger font
            // sizes. To opt out, wrap the [label] widget in a [MediaQuery] widget
            // with a different `TextScaler`.
            maxScaleFactor: _kMaxLabelTextScaleFactor,
            child: Text(label, style: textStyle),
          ),
        );
      },
    );
  }
}

/// Widget that handles the semantics and layout of a navigation bar
/// destination.
///
/// Prefer [NavigationDestination] over this widget, as it is a simpler
/// (although less customizable) way to get navigation bar destinations.
///
/// The icon and label of this destination are built with [buildIcon] and
/// [buildLabel]. They should build the unselected and selected icon and label
/// according to [_NavigationDestinationInfo.selectedAnimation], where an
/// animation value of 0 is unselected and 1 is selected.
///
/// See [NavigationDestination] for an example.
class _NavigationDestinationBuilder extends StatefulWidget {
  /// Builds a destination (icon + label) to use in a Material 3 [NavigationBar].
  const _NavigationDestinationBuilder({
    required this.buildIcon,
    required this.buildLabel,
    required this.label,
    this.tooltip,
    this.enabled = true,
  });

  /// Builds the icon for a destination in a [NavigationBar].
  ///
  /// To animate between unselected and selected, build the icon based on
  /// [_NavigationDestinationInfo.selectedAnimation]. When the animation is 0,
  /// the destination is unselected, when the animation is 1, the destination is
  /// selected.
  ///
  /// The destination is considered selected as soon as the animation is
  /// increasing or completed, and it is considered unselected as soon as the
  /// animation is decreasing or dismissed.
  final WidgetBuilder buildIcon;

  /// Builds the label for a destination in a [NavigationBar].
  ///
  /// To animate between unselected and selected, build the icon based on
  /// [_NavigationDestinationInfo.selectedAnimation]. When the animation is
  /// 0, the destination is unselected, when the animation is 1, the destination
  /// is selected.
  ///
  /// The destination is considered selected as soon as the animation is
  /// increasing or completed, and it is considered unselected as soon as the
  /// animation is decreasing or dismissed.
  final WidgetBuilder buildLabel;

  /// The text value of what is in the label widget, this is required for
  /// semantics so that screen readers and tooltips can read the proper label.
  final String label;

  /// The text to display in the tooltip for this [NavigationDestination], when
  /// the user long presses the destination.
  ///
  /// If [tooltip] is an empty string, no tooltip will be used.
  ///
  /// Defaults to null, in which case the [label] text will be used.
  final String? tooltip;

  /// Indicates that this destination is selectable.
  ///
  /// Defaults to true.
  final bool enabled;

  @override
  State<_NavigationDestinationBuilder> createState() => _NavigationDestinationBuilderState();
}

class _NavigationDestinationBuilderState extends State<_NavigationDestinationBuilder> {
  final GlobalKey iconKey = GlobalKey();

  @override
  Widget build(BuildContext context) {
    final _NavigationDestinationInfo info = _NavigationDestinationInfo.of(context);
    final NavigationBarThemeData navigationBarTheme = NavigationBarTheme.of(context);
    final NavigationBarThemeData defaults = _defaultsFor(context);

    return _NavigationBarDestinationSemantics(
      child: _NavigationBarDestinationTooltip(
        message: widget.tooltip ?? widget.label,
        child: _IndicatorInkWell(
          iconKey: iconKey,
          labelBehavior: info.labelBehavior,
          customBorder:
              info.indicatorShape ?? navigationBarTheme.indicatorShape ?? defaults.indicatorShape,
          overlayColor: info.overlayColor ?? navigationBarTheme.overlayColor,
          onTap: widget.enabled ? info.onTap : null,
          child: Row(
            children: <Widget>[
              Expanded(
                child: _NavigationBarDestinationLayout(
                  icon: widget.buildIcon(context),
                  iconKey: iconKey,
                  label: widget.buildLabel(context),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}

class _IndicatorInkWell extends InkResponse {
  const _IndicatorInkWell({
    required this.iconKey,
    required this.labelBehavior,
    super.overlayColor,
    super.customBorder,
    super.onTap,
    super.child,
  }) : super(containedInkWell: true, highlightColor: Colors.transparent);

  final GlobalKey iconKey;
  final NavigationDestinationLabelBehavior labelBehavior;

  @override
  RectCallback? getRectCallback(RenderBox referenceBox) {
    return () {
      final RenderBox iconBox = iconKey.currentContext!.findRenderObject()! as RenderBox;
      final Rect iconRect = iconBox.localToGlobal(Offset.zero) & iconBox.size;
      return referenceBox.globalToLocal(iconRect.topLeft) & iconBox.size;
    };
  }
}

/// Inherited widget for passing data from the [NavigationBar] to the
/// [NavigationBar.destinations] children widgets.
///
/// Useful for building navigation destinations using:
/// `_NavigationDestinationInfo.of(context)`.
class _NavigationDestinationInfo extends InheritedWidget {
  /// Adds the information needed to build a navigation destination to the
  /// [child] and descendants.
  const _NavigationDestinationInfo({
    required this.index,
    required this.selectedIndex,
    required this.totalNumberOfDestinations,
    required this.selectedAnimation,
    required this.labelBehavior,
    required this.indicatorColor,
    required this.indicatorShape,
    required this.overlayColor,
    required this.onTap,
    this.labelTextStyle,
    this.labelPadding,
    required super.child,
  });

  /// Which destination index is this in the navigation bar.
  ///
  /// For example:
  ///
  /// ```dart
  /// NavigationBar(
  ///   destinations: const <Widget>[
  ///     NavigationDestination(
  ///       // This is destination index 0.
  ///       icon: Icon(Icons.surfing),
  ///       label: 'Surfing',
  ///     ),
  ///     NavigationDestination(
  ///       // This is destination index 1.
  ///       icon: Icon(Icons.support),
  ///       label: 'Support',
  ///     ),
  ///     NavigationDestination(
  ///       // This is destination index 2.
  ///       icon: Icon(Icons.local_hospital),
  ///       label: 'Hospital',
  ///     ),
  ///   ]
  /// )
  /// ```
  ///
  /// This is required for semantics, so that each destination can have a label
  /// "Tab 1 of 3", for example.
  final int index;

  /// This is the index of the currently selected destination.
  ///
  /// This is required for `_IndicatorInkWell` to apply label padding to ripple animations
  /// when label behavior is [NavigationDestinationLabelBehavior.onlyShowSelected].
  final int selectedIndex;

  /// How many total destinations are in this navigation bar.
  ///
  /// This is required for semantics, so that each destination can have a label
  /// "Tab 1 of 4", for example.
  final int totalNumberOfDestinations;

  /// Indicates whether or not this destination is selected, from 0 (unselected)
  /// to 1 (selected).
  final Animation<double> selectedAnimation;

  /// Determines the behavior for how the labels will layout.
  ///
  /// Can be used to show all labels (the default), show only the selected
  /// label, or hide all labels.
  final NavigationDestinationLabelBehavior labelBehavior;

  /// The color of the selection indicator.
  ///
  /// This is used by destinations to override the indicator color.
  final Color? indicatorColor;

  /// The shape of the selection indicator.
  ///
  /// This is used by destinations to override the indicator shape.
  final ShapeBorder? indicatorShape;

  /// The highlight color that's typically used to indicate that
  /// the [NavigationDestination] is focused, hovered, or pressed.
  ///
  /// This is used by destinations to override the overlay color.
  final MaterialStateProperty<Color?>? overlayColor;

  /// The callback that should be called when this destination is tapped.
  ///
  /// This is computed by calling [NavigationBar.onDestinationSelected]
  /// with [index] passed in.
  final VoidCallback onTap;

  /// The text style of the label.
  final MaterialStateProperty<TextStyle?>? labelTextStyle;

  /// The padding around the [label] widget.
  ///
  /// Defaults to a padding of 4 pixels on the top.
  final EdgeInsetsGeometry? labelPadding;

  /// Returns a non null [_NavigationDestinationInfo].
  ///
  /// This will return an error if called with no [_NavigationDestinationInfo]
  /// ancestor.
  ///
  /// Used by widgets that are implementing a navigation destination info to
  /// get information like the selected animation and destination number.
  static _NavigationDestinationInfo of(BuildContext context) {
    final _NavigationDestinationInfo? result =
        context.dependOnInheritedWidgetOfExactType<_NavigationDestinationInfo>();
    assert(
      result != null,
      'Navigation destinations need a _NavigationDestinationInfo parent, '
      'which is usually provided by NavigationBar.',
    );
    return result!;
  }

  @override
  bool updateShouldNotify(_NavigationDestinationInfo oldWidget) {
    return index != oldWidget.index ||
        totalNumberOfDestinations != oldWidget.totalNumberOfDestinations ||
        selectedAnimation != oldWidget.selectedAnimation ||
        labelBehavior != oldWidget.labelBehavior ||
        onTap != oldWidget.onTap;
  }
}

/// Selection Indicator for the Material 3 [NavigationBar] and [NavigationRail]
/// components.
///
/// When [animation] is 0, the indicator is not present. As [animation] grows
/// from 0 to 1, the indicator scales in on the x axis.
///
/// Used in a [Stack] widget behind the icons in the Material 3 Navigation Bar
/// to illuminate the selected destination.
class NavigationIndicator extends StatelessWidget {
  /// Builds an indicator, usually used in a stack behind the icon of a
  /// navigation bar destination.
  const NavigationIndicator({
    super.key,
    required this.animation,
    this.color,
    this.width = _kIndicatorWidth,
    this.height = _kIndicatorHeight,
    this.borderRadius = const BorderRadius.all(Radius.circular(16)),
    this.shape,
  });

  /// Determines the scale of the indicator.
  ///
  /// When [animation] is 0, the indicator is not present. The indicator scales
  /// in as [animation] grows from 0 to 1.
  final Animation<double> animation;

  /// The fill color of this indicator.
  ///
  /// If null, defaults to [ColorScheme.secondary].
  final Color? color;

  /// The width of this indicator.
  ///
  /// Defaults to `64`.
  final double width;

  /// The height of this indicator.
  ///
  /// Defaults to `32`.
  final double height;

  /// The border radius of the shape of the indicator.
  ///
  /// This is used to create a [RoundedRectangleBorder] shape for the indicator.
  /// This is ignored if [shape] is non-null.
  ///
  /// Defaults to `BorderRadius.circular(16)`.
  final BorderRadius borderRadius;

  /// The shape of the indicator.
  ///
  /// If non-null this is used as the shape used to draw the background
  /// of the indicator. If null then a [RoundedRectangleBorder] with the
  /// [borderRadius] is used.
  final ShapeBorder? shape;

  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: animation,
      builder: (BuildContext context, Widget? child) {
        // The scale should be 0 when the animation is unselected, as soon as
        // the animation starts, the scale jumps to 40%, and then animates to
        // 100% along a curve.
        final double scale =
            animation.isDismissed
                ? 0.0
                : Tween<double>(begin: .4, end: 1.0).transform(
                  CurveTween(curve: Curves.easeInOutCubicEmphasized).transform(animation.value),
                );

        return Transform(
          alignment: Alignment.center,
          // Scale in the X direction only.
          transform: Matrix4.diagonal3Values(scale, 1.0, 1.0),
          child: child,
        );
      },
      // Fade should be a 100ms animation whenever the parent animation changes
      // direction.
      child: _StatusTransitionWidgetBuilder(
        animation: animation,
        builder: (BuildContext context, Widget? child) {
          return _SelectableAnimatedBuilder(
            isSelected: animation.isForwardOrCompleted,
            duration: const Duration(milliseconds: 100),
            alwaysDoFullAnimation: true,
            builder: (BuildContext context, Animation<double> fadeAnimation) {
              return FadeTransition(
                opacity: fadeAnimation,
                child: Container(
                  width: width,
                  height: height,
                  decoration: ShapeDecoration(
                    shape: shape ?? RoundedRectangleBorder(borderRadius: borderRadius),
                    color: color ?? Theme.of(context).colorScheme.secondary,
                  ),
                ),
              );
            },
          );
        },
      ),
    );
  }
}

/// Widget that handles the layout of the icon + label in a navigation bar
/// destination, based on [_NavigationDestinationInfo.labelBehavior] and
/// [_NavigationDestinationInfo.selectedAnimation].
///
/// Depending on the [_NavigationDestinationInfo.labelBehavior], the labels
/// will shift and fade accordingly.
class _NavigationBarDestinationLayout extends StatelessWidget {
  /// Builds a widget to layout an icon + label for a destination in a Material
  /// 3 [NavigationBar].
  const _NavigationBarDestinationLayout({
    required this.icon,
    required this.iconKey,
    required this.label,
  });

  /// The icon widget that sits on top of the label.
  ///
  /// See [NavigationDestination.icon].
  final Widget icon;

  /// The global key for the icon of this destination.
  ///
  /// This is used to determine the position of the icon.
  final GlobalKey iconKey;

  /// The label widget that sits below the icon.
  ///
  /// This widget will sometimes be faded out, depending on
  /// [_NavigationDestinationInfo.selectedAnimation].
  ///
  /// See [NavigationDestination.label].
  final Widget label;

  static final Key _labelKey = UniqueKey();

  @override
  Widget build(BuildContext context) {
    return _DestinationLayoutAnimationBuilder(
      builder: (BuildContext context, Animation<double> animation) {
        return CustomMultiChildLayout(
          delegate: _NavigationDestinationLayoutDelegate(animation: animation),
          children: <Widget>[
            LayoutId(
              id: _NavigationDestinationLayoutDelegate.iconId,
              child: RepaintBoundary(key: iconKey, child: icon),
            ),
            LayoutId(
              id: _NavigationDestinationLayoutDelegate.labelId,
              child: FadeTransition(
                alwaysIncludeSemantics: true,
                opacity: animation,
                child: RepaintBoundary(key: _labelKey, child: label),
              ),
            ),
          ],
        );
      },
    );
  }
}

/// Determines the appropriate [Curve] and [Animation] to use for laying out the
/// [NavigationDestination], based on
/// [_NavigationDestinationInfo.labelBehavior].
///
/// The animation controlling the position and fade of the labels differs
/// from the selection animation, depending on the
/// [NavigationDestinationLabelBehavior]. This widget determines what
/// animation should be used for the position and fade of the labels.
class _DestinationLayoutAnimationBuilder extends StatelessWidget {
  /// Builds a child with the appropriate animation [Curve] based on the
  /// [_NavigationDestinationInfo.labelBehavior].
  const _DestinationLayoutAnimationBuilder({required this.builder});

  /// Builds the child of this widget.
  ///
  /// The [Animation] will be the appropriate [Animation] to use for the layout
  /// and fade of the [NavigationDestination], either a curve, always
  /// showing (1), or always hiding (0).
  final Widget Function(BuildContext, Animation<double>) builder;

  @override
  Widget build(BuildContext context) {
    final _NavigationDestinationInfo info = _NavigationDestinationInfo.of(context);
    switch (info.labelBehavior) {
      case NavigationDestinationLabelBehavior.alwaysShow:
        return builder(context, kAlwaysCompleteAnimation);
      case NavigationDestinationLabelBehavior.alwaysHide:
        return builder(context, kAlwaysDismissedAnimation);
      case NavigationDestinationLabelBehavior.onlyShowSelected:
        return _CurvedAnimationBuilder(
          animation: info.selectedAnimation,
          curve: Curves.easeInOutCubicEmphasized,
          reverseCurve: Curves.easeInOutCubicEmphasized.flipped,
          builder: builder,
        );
    }
  }
}

/// Semantics widget for a navigation bar destination.
///
/// Requires a [_NavigationDestinationInfo] parent (normally provided by the
/// [NavigationBar] by default).
///
/// Provides localized semantic labels to the destination, for example, it will
/// read "Home, Tab 1 of 3".
///
/// Used by [_NavigationDestinationBuilder].
class _NavigationBarDestinationSemantics extends StatelessWidget {
  /// Adds the appropriate semantics for navigation bar destinations to the
  /// [child].
  const _NavigationBarDestinationSemantics({required this.child});

  /// The widget that should receive the destination semantics.
  final Widget child;

  @override
  Widget build(BuildContext context) {
    final MaterialLocalizations localizations = MaterialLocalizations.of(context);
    final _NavigationDestinationInfo destinationInfo = _NavigationDestinationInfo.of(context);
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

/// Tooltip widget for use in a [NavigationBar].
///
/// It appears just above the navigation bar when one of the destinations is
/// long pressed.
class _NavigationBarDestinationTooltip extends StatelessWidget {
  /// Adds a tooltip to the [child] widget.
  const _NavigationBarDestinationTooltip({required this.message, required this.child});

  /// The text that is rendered in the tooltip when it appears.
  final String message;

  /// The widget that, when pressed, will show a tooltip.
  final Widget child;

  @override
  Widget build(BuildContext context) {
    return Tooltip(
      message: message,
      // TODO(johnsonmh): Make this value configurable/themable.
      verticalOffset: 42,
      excludeFromSemantics: true,
      preferBelow: false,
      child: child,
    );
  }
}

/// Custom layout delegate for shifting navigation bar destinations.
///
/// This will lay out the icon + label according to the [animation].
///
/// When the [animation] is 0, the icon will be centered, and the label will be
/// positioned directly below it.
///
/// When the [animation] is 1, the label will still be positioned directly below
/// the icon, but the icon + label combination will be centered.
///
/// Used in a [CustomMultiChildLayout] widget in the
/// [_NavigationDestinationBuilder].
class _NavigationDestinationLayoutDelegate extends MultiChildLayoutDelegate {
  _NavigationDestinationLayoutDelegate({required this.animation}) : super(relayout: animation);

  /// The selection animation that indicates whether or not this destination is
  /// selected.
  ///
  /// See [_NavigationDestinationInfo.selectedAnimation].
  final Animation<double> animation;

  /// ID for the icon widget child.
  ///
  /// This is used by the [LayoutId] when this delegate is used in a
  /// [CustomMultiChildLayout].
  ///
  /// See [_NavigationDestinationBuilder].
  static const int iconId = 1;

  /// ID for the label widget child.
  ///
  /// This is used by the [LayoutId] when this delegate is used in a
  /// [CustomMultiChildLayout].
  ///
  /// See [_NavigationDestinationBuilder].
  static const int labelId = 2;

  @override
  void performLayout(Size size) {
    double halfWidth(Size size) => size.width / 2;
    double halfHeight(Size size) => size.height / 2;

    final Size iconSize = layoutChild(iconId, BoxConstraints.loose(size));
    final Size labelSize = layoutChild(labelId, BoxConstraints.loose(size));

    final double yPositionOffset = Tween<double>(
      // When unselected, the icon is centered vertically.
      begin: halfHeight(iconSize),
      // When selected, the icon and label are centered vertically.
      end: halfHeight(iconSize) + halfHeight(labelSize),
    ).transform(animation.value);
    final double iconYPosition = halfHeight(size) - yPositionOffset;

    // Position the icon.
    positionChild(
      iconId,
      Offset(
        // Center the icon horizontally.
        halfWidth(size) - halfWidth(iconSize),
        iconYPosition,
      ),
    );

    // Position the label.
    positionChild(
      labelId,
      Offset(
        // Center the label horizontally.
        halfWidth(size) - halfWidth(labelSize),
        // Label always appears directly below the icon.
        iconYPosition + iconSize.height,
      ),
    );
  }

  @override
  bool shouldRelayout(_NavigationDestinationLayoutDelegate oldDelegate) {
    return oldDelegate.animation != animation;
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
// be reversed until it is either 0 or 1 again. If [alwaysDoFullAnimation] is
// true, the animation will reset to 0 or 1 before beginning the animation, so
// that the full animation is done.
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
    this.alwaysDoFullAnimation = false,
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

  /// If true, the animation will always go all the way from 0 to 1 when
  /// [isSelected] is true, and from 1 to 0 when [isSelected] is false, even
  /// when the status changes mid animation.
  ///
  /// If this is false and the status changes mid animation, the animation will
  /// reverse direction from it's current point.
  ///
  /// Defaults to false.
  final bool alwaysDoFullAnimation;

  /// Builds the child widget based on the current animation status.
  ///
  /// When [isSelected] is updated to true, this builder will be called and the
  /// animation will animate up to 1. When [isSelected] is updated to
  /// `false`, this will be called and the animation will animate down to 0.
  final Widget Function(BuildContext, Animation<double>) builder;

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
        _controller.forward(from: widget.alwaysDoFullAnimation ? 0 : null);
      } else {
        _controller.reverse(from: widget.alwaysDoFullAnimation ? 1 : null);
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

/// Watches [animation] and calls [builder] with the appropriate [Curve]
/// depending on the direction of the [animation] status.
///
/// If [Animation.status] is forward or complete, [curve] is used. If
/// [Animation.status] is reverse or dismissed, [reverseCurve] is used.
///
/// If the [animation] changes direction while it is already running, the curve
/// used will not change, this will keep the animations smooth until it
/// completes.
///
/// This is similar to [CurvedAnimation] except the animation status listeners
/// are removed when this widget is disposed.
class _CurvedAnimationBuilder extends StatefulWidget {
  const _CurvedAnimationBuilder({
    required this.animation,
    required this.curve,
    required this.reverseCurve,
    required this.builder,
  });

  final Animation<double> animation;
  final Curve curve;
  final Curve reverseCurve;
  final Widget Function(BuildContext, Animation<double>) builder;

  @override
  _CurvedAnimationBuilderState createState() => _CurvedAnimationBuilderState();
}

class _CurvedAnimationBuilderState extends State<_CurvedAnimationBuilder> {
  late AnimationStatus _animationDirection;
  AnimationStatus? _preservedDirection;

  @override
  void initState() {
    super.initState();
    _animationDirection = widget.animation.status;
    _updateStatus(widget.animation.status);
    widget.animation.addStatusListener(_updateStatus);
  }

  @override
  void dispose() {
    widget.animation.removeStatusListener(_updateStatus);
    super.dispose();
  }

  // Keeps track of the current animation status, as well as the "preserved
  // direction" when the animation changes direction mid animation.
  //
  // The preserved direction is reset when the animation finishes in either
  // direction.
  void _updateStatus(AnimationStatus status) {
    if (_animationDirection != status) {
      setState(() {
        _animationDirection = status;
      });
    }
    switch (status) {
      case AnimationStatus.forward || AnimationStatus.reverse when _preservedDirection != null:
        break;
      case AnimationStatus.forward || AnimationStatus.reverse:
        setState(() {
          _preservedDirection = status;
        });
      case AnimationStatus.completed || AnimationStatus.dismissed:
        setState(() {
          _preservedDirection = null;
        });
    }
  }

  @override
  Widget build(BuildContext context) {
    final bool shouldUseForwardCurve =
        (_preservedDirection ?? _animationDirection) != AnimationStatus.reverse;

    final Animation<double> curvedAnimation = CurveTween(
      curve: shouldUseForwardCurve ? widget.curve : widget.reverseCurve,
    ).animate(widget.animation);

    return widget.builder(context, curvedAnimation);
  }
}

NavigationBarThemeData _defaultsFor(BuildContext context) {
  return Theme.of(context).useMaterial3
      ? _NavigationBarDefaultsM3(context)
      : _NavigationBarDefaultsM2(context);
}

// Hand coded defaults based on Material Design 2.
class _NavigationBarDefaultsM2 extends NavigationBarThemeData {
  _NavigationBarDefaultsM2(BuildContext context)
    : _theme = Theme.of(context),
      _colors = Theme.of(context).colorScheme,
      super(
        height: 80.0,
        elevation: 0.0,
        indicatorShape: const RoundedRectangleBorder(
          borderRadius: BorderRadius.all(Radius.circular(16)),
        ),
        labelBehavior: NavigationDestinationLabelBehavior.alwaysShow,
      );

  final ThemeData _theme;
  final ColorScheme _colors;

  // With Material 2, the NavigationBar uses an overlay blend for the
  // default color regardless of light/dark mode.
  @override
  Color? get backgroundColor =>
      ElevationOverlay.colorWithOverlay(_colors.surface, _colors.onSurface, 3.0);

  @override
  MaterialStateProperty<IconThemeData?>? get iconTheme {
    return MaterialStatePropertyAll<IconThemeData>(
      IconThemeData(size: 24, color: _colors.onSurface),
    );
  }

  @override
  Color? get indicatorColor => _colors.secondary.withOpacity(0.24);

  @override
  MaterialStateProperty<TextStyle?>? get labelTextStyle => MaterialStatePropertyAll<TextStyle?>(
    _theme.textTheme.labelSmall!.copyWith(color: _colors.onSurface),
  );

  @override
  EdgeInsetsGeometry? get labelPadding => const EdgeInsets.only(top: 4);
}

// BEGIN GENERATED TOKEN PROPERTIES - NavigationBar

// Do not edit by hand. The code between the "BEGIN GENERATED" and
// "END GENERATED" comments are generated from data in the Material
// Design token database by the script:
//   dev/tools/gen_defaults/bin/gen_defaults.dart.

// dart format off
class _NavigationBarDefaultsM3 extends NavigationBarThemeData {
  _NavigationBarDefaultsM3(this.context)
    : super(
        height: 80.0,
        elevation: 3.0,
        labelBehavior: NavigationDestinationLabelBehavior.alwaysShow,
      );

  final BuildContext context;
  late final ColorScheme _colors = Theme.of(context).colorScheme;
  late final TextTheme _textTheme = Theme.of(context).textTheme;

  @override
  Color? get backgroundColor => _colors.surfaceContainer;

  @override
  Color? get shadowColor => Colors.transparent;

  @override
  Color? get surfaceTintColor => Colors.transparent;

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
  Color? get indicatorColor => _colors.secondaryContainer;

  @override
  ShapeBorder? get indicatorShape => const StadiumBorder();

  @override
  MaterialStateProperty<TextStyle?>? get labelTextStyle {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
    final TextStyle style = _textTheme.labelMedium!;
      return style.apply(
        color: states.contains(MaterialState.disabled)
          ? _colors.onSurfaceVariant.withOpacity(0.38)
          : states.contains(MaterialState.selected)
            ? _colors.onSurface
            : _colors.onSurfaceVariant
      );
    });
  }

  @override
  EdgeInsetsGeometry? get labelPadding => const EdgeInsets.only(top: 4);
}
// dart format on

// END GENERATED TOKEN PROPERTIES - NavigationBar
