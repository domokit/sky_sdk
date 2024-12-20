// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'app_bar.dart';
/// @docImport 'bottom_app_bar.dart';
/// @docImport 'bottom_navigation_bar.dart';
/// @docImport 'circle_avatar.dart';
/// @docImport 'floating_action_button.dart';
/// @docImport 'list_tile.dart';
library;

import 'dart:math' as math;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'scaffold.dart';

/// The margin that a [FloatingActionButton] should leave between it and the
/// edge of the screen.
///
/// [FloatingActionButtonLocation.endFloat] uses this to set the appropriate margin
/// between the [FloatingActionButton] and the end of the screen.
const double kFloatingActionButtonMargin = 16.0;

/// The amount of time the [FloatingActionButton] takes to transition in or out.
///
/// The [Scaffold] uses this to set the duration of [FloatingActionButton]
/// motion, entrance, and exit animations.
const Duration kFloatingActionButtonSegue = Duration(milliseconds: 200);

/// The fraction of a circle the [FloatingActionButton] should turn when it enters.
///
/// Its value corresponds to 0.125 of a full circle, equivalent to 45 degrees or pi/4 radians.
const double kFloatingActionButtonTurnInterval = 0.125;

/// If a [FloatingActionButton] is used on a [Scaffold] in certain positions,
/// it is moved [kMiniButtonOffsetAdjustment] pixels closer to the edge of the screen.
///
/// This is intended to be used with [FloatingActionButton.mini] set to true,
/// so that the floating action button appears to align with [CircleAvatar]s
/// in the [ListTile.leading] slot of a [ListTile] in a [ListView] in the
/// [Scaffold.body].
///
/// More specifically:
/// * In the following positions, the [FloatingActionButton] is moved *horizontally*
/// closer to the edge of the screen:
///   * [FloatingActionButtonLocation.miniStartTop]
///   * [FloatingActionButtonLocation.miniStartFloat]
///   * [FloatingActionButtonLocation.miniStartDocked]
///   * [FloatingActionButtonLocation.miniEndTop]
///   * [FloatingActionButtonLocation.miniEndFloat]
///   * [FloatingActionButtonLocation.miniEndDocked]
/// * In the following positions, the [FloatingActionButton] is moved *vertically*
/// closer to the bottom of the screen:
///   * [FloatingActionButtonLocation.miniStartFloat]
///   * [FloatingActionButtonLocation.miniCenterFloat]
///   * [FloatingActionButtonLocation.miniEndFloat]
const double kMiniButtonOffsetAdjustment = 4.0;

/// An object that defines a position for the [FloatingActionButton]
/// based on the [Scaffold]'s [ScaffoldPrelayoutGeometry].
///
/// Flutter provides [FloatingActionButtonLocation]s for the common
/// [FloatingActionButton] placements in Material Design applications. These
/// locations are available as static members of this class.
///
/// ## Floating Action Button placements
///
/// The following diagrams show the available placement locations for the FloatingActionButton.
///
/// * [FloatingActionButtonLocation.centerDocked]:
///
///   ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_center_docked.png)
///
///
/// * [FloatingActionButtonLocation.centerFloat]:
///
///   ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_center_float.png)
///
///
/// * [FloatingActionButtonLocation.centerTop]:
///
///   ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_center_top.png)
///
///
/// * [FloatingActionButtonLocation.endDocked]:
///
///   ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_end_docked.png)
///
///
/// * [FloatingActionButtonLocation.endFloat]:
///
///   ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_end_float.png)
///
///
/// * [FloatingActionButtonLocation.endTop]:
///
///   ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_end_top.png)
///
///
/// * [FloatingActionButtonLocation.startDocked]:
///
///   ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_start_docked.png)
///
///
/// * [FloatingActionButtonLocation.startFloat]:
///
///   ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_start_float.png)
///
///
/// * [FloatingActionButtonLocation.startTop]:
///
///   ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_start_top.png)
///
///
/// * [FloatingActionButtonLocation.miniCenterDocked]:
///
///   ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_mini_center_docked.png)
///
///
/// * [FloatingActionButtonLocation.miniCenterFloat]:
///
///   ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_mini_center_float.png)
///
///
/// * [FloatingActionButtonLocation.miniCenterTop]:
///
///   ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_mini_center_top.png)
///
///
/// * [FloatingActionButtonLocation.miniEndDocked]:
///
///   ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_mini_end_docked.png)
///
///
/// * [FloatingActionButtonLocation.miniEndFloat]:
///
///   ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_mini_end_float.png)
///
///
/// * [FloatingActionButtonLocation.miniEndTop]:
///
///   ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_mini_end_top.png)
///
///
/// * [FloatingActionButtonLocation.miniStartDocked]:
///
///   ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_mini_start_docked.png)
///
///
/// * [FloatingActionButtonLocation.miniStartFloat]:
///
///   ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_mini_start_float.png)
///
///
/// * [FloatingActionButtonLocation.miniStartTop]:
///
///   ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_mini_start_top.png)
///
///
/// See also:
///
///  * [FloatingActionButton], which is a circular button typically shown in the
///    bottom right corner of the app.
///  * [FloatingActionButtonAnimator], which is used to animate the
///    [Scaffold.floatingActionButton] from one [FloatingActionButtonLocation] to
///    another.
///  * [ScaffoldPrelayoutGeometry], the geometry that
///    [FloatingActionButtonLocation]s use to position the [FloatingActionButton].
abstract class FloatingActionButtonLocation {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const FloatingActionButtonLocation();

  /// Start-aligned [FloatingActionButton], floating over the transition between
  /// the [Scaffold.appBar] and the [Scaffold.body].
  ///
  /// To align a floating action button with [CircleAvatar]s in the
  /// [ListTile.leading] slots of [ListTile]s in a [ListView] in the [Scaffold.body],
  /// use [miniStartTop] and set [FloatingActionButton.mini] to true.
  ///
  /// This is unlikely to be a useful location for apps that lack a top [AppBar]
  /// or that use a [SliverAppBar] in the scaffold body itself.
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_start_top.png)
  static const FloatingActionButtonLocation startTop = _StartTopFabLocation();

  /// Start-aligned [FloatingActionButton], floating over the transition between
  /// the [Scaffold.appBar] and the [Scaffold.body], optimized for mini floating
  /// action buttons.
  ///
  /// This is intended to be used with [FloatingActionButton.mini] set to true,
  /// so that the floating action button appears to align with [CircleAvatar]s
  /// in the [ListTile.leading] slot of a [ListTile] in a [ListView] in the
  /// [Scaffold.body].
  ///
  /// This is unlikely to be a useful location for apps that lack a top [AppBar]
  /// or that use a [SliverAppBar] in the scaffold body itself.
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_mini_start_top.png)
  static const FloatingActionButtonLocation miniStartTop = _MiniStartTopFabLocation();

  /// Centered [FloatingActionButton], floating over the transition between
  /// the [Scaffold.appBar] and the [Scaffold.body].
  ///
  /// This is unlikely to be a useful location for apps that lack a top [AppBar]
  /// or that use a [SliverAppBar] in the scaffold body itself.
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_center_top.png)
  static const FloatingActionButtonLocation centerTop = _CenterTopFabLocation();

  /// Centered [FloatingActionButton], floating over the transition between
  /// the [Scaffold.appBar] and the [Scaffold.body], intended to be used with
  /// [FloatingActionButton.mini] set to true.
  ///
  /// This is unlikely to be a useful location for apps that lack a top [AppBar]
  /// or that use a [SliverAppBar] in the scaffold body itself.
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_mini_center_top.png)
  static const FloatingActionButtonLocation miniCenterTop = _MiniCenterTopFabLocation();

  /// End-aligned [FloatingActionButton], floating over the transition between
  /// the [Scaffold.appBar] and the [Scaffold.body].
  ///
  /// To align a floating action button with [CircleAvatar]s in the
  /// [ListTile.trailing] slots of [ListTile]s in a [ListView] in the [Scaffold.body],
  /// use [miniEndTop] and set [FloatingActionButton.mini] to true.
  ///
  /// This is unlikely to be a useful location for apps that lack a top [AppBar]
  /// or that use a [SliverAppBar] in the scaffold body itself.
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_end_top.png)
  static const FloatingActionButtonLocation endTop = _EndTopFabLocation();

  /// End-aligned [FloatingActionButton], floating over the transition between
  /// the [Scaffold.appBar] and the [Scaffold.body], optimized for mini floating
  /// action buttons.
  ///
  /// This is intended to be used with [FloatingActionButton.mini] set to true,
  /// so that the floating action button appears to align with [CircleAvatar]s
  /// in the [ListTile.trailing] slot of a [ListTile] in a [ListView] in the
  /// [Scaffold.body].
  ///
  /// This is unlikely to be a useful location for apps that lack a top [AppBar]
  /// or that use a [SliverAppBar] in the scaffold body itself.
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_mini_end_top.png)
  static const FloatingActionButtonLocation miniEndTop = _MiniEndTopFabLocation();

  /// Start-aligned [FloatingActionButton], floating at the bottom of the screen.
  ///
  /// To align a floating action button with [CircleAvatar]s in the
  /// [ListTile.leading] slots of [ListTile]s in a [ListView] in the [Scaffold.body],
  /// use [miniStartFloat] and set [FloatingActionButton.mini] to true.
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_start_float.png)
  static const FloatingActionButtonLocation startFloat = _StartFloatFabLocation();

  /// Start-aligned [FloatingActionButton], floating at the bottom of the screen,
  /// optimized for mini floating action buttons.
  ///
  /// This is intended to be used with [FloatingActionButton.mini] set to true,
  /// so that the floating action button appears to align with [CircleAvatar]s
  /// in the [ListTile.leading] slot of a [ListTile] in a [ListView] in the
  /// [Scaffold.body].
  ///
  /// Compared to [FloatingActionButtonLocation.startFloat], floating action
  /// buttons using this location will move horizontally _and_ vertically
  /// closer to the edges, by [kMiniButtonOffsetAdjustment] each.
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_mini_start_float.png)
  static const FloatingActionButtonLocation miniStartFloat = _MiniStartFloatFabLocation();

  /// Centered [FloatingActionButton], floating at the bottom of the screen.
  ///
  /// To position a mini floating action button, use [miniCenterFloat] and
  /// set [FloatingActionButton.mini] to true.
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_center_float.png)
  static const FloatingActionButtonLocation centerFloat = _CenterFloatFabLocation();

  /// Centered [FloatingActionButton], floating at the bottom of the screen,
  /// optimized for mini floating action buttons.
  ///
  /// This is intended to be used with [FloatingActionButton.mini] set to true,
  /// so that the floating action button appears to align horizontally with
  /// the locations [FloatingActionButtonLocation.miniStartFloat]
  /// and [FloatingActionButtonLocation.miniEndFloat].
  ///
  /// Compared to [FloatingActionButtonLocation.centerFloat], floating action
  /// buttons using this location will move vertically down
  /// by [kMiniButtonOffsetAdjustment].
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_mini_center_float.png)
  static const FloatingActionButtonLocation miniCenterFloat = _MiniCenterFloatFabLocation();

  /// End-aligned [FloatingActionButton], floating at the bottom of the screen.
  ///
  /// This is the default alignment of [FloatingActionButton]s in Material applications.
  ///
  /// To align a floating action button with [CircleAvatar]s in the
  /// [ListTile.trailing] slots of [ListTile]s in a [ListView] in the [Scaffold.body],
  /// use [miniEndFloat] and set [FloatingActionButton.mini] to true.
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_end_float.png)
  static const FloatingActionButtonLocation endFloat = _EndFloatFabLocation();

  /// End-aligned [FloatingActionButton], floating at the bottom of the screen,
  /// optimized for mini floating action buttons.
  ///
  /// This is intended to be used with [FloatingActionButton.mini] set to true,
  /// so that the floating action button appears to align with [CircleAvatar]s
  /// in the [ListTile.trailing] slot of a [ListTile] in a [ListView] in the
  /// [Scaffold.body].
  ///
  /// Compared to [FloatingActionButtonLocation.endFloat], floating action
  /// buttons using this location will move horizontally _and_ vertically
  /// closer to the edges, by [kMiniButtonOffsetAdjustment] each.
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_mini_end_float.png)
  static const FloatingActionButtonLocation miniEndFloat = _MiniEndFloatFabLocation();

  /// Start-aligned [FloatingActionButton], floating over the
  /// [Scaffold.bottomNavigationBar] so that the center of the floating
  /// action button lines up with the top of the bottom navigation bar.
  ///
  /// To align a floating action button with [CircleAvatar]s in the
  /// [ListTile.leading] slots of [ListTile]s in a [ListView] in the [Scaffold.body],
  /// use [miniStartDocked] and set [FloatingActionButton.mini] to true.
  ///
  /// If the value of [Scaffold.bottomNavigationBar] is a [BottomAppBar],
  /// the bottom app bar can include a "notch" in its shape that accommodates
  /// the overlapping floating action button.
  ///
  /// This is unlikely to be a useful location for apps that lack a bottom
  /// navigation bar.
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_start_docked.png)
  static const FloatingActionButtonLocation startDocked = _StartDockedFabLocation();

  /// Start-aligned [FloatingActionButton], floating over the
  /// [Scaffold.bottomNavigationBar] so that the center of the floating
  /// action button lines up with the top of the bottom navigation bar,
  /// optimized for mini floating action buttons.
  ///
  /// If the value of [Scaffold.bottomNavigationBar] is a [BottomAppBar],
  /// the bottom app bar can include a "notch" in its shape that accommodates
  /// the overlapping floating action button.
  ///
  /// This is intended to be used with [FloatingActionButton.mini] set to true,
  /// so that the floating action button appears to align with [CircleAvatar]s
  /// in the [ListTile.leading] slot of a [ListTile] in a [ListView] in the
  /// [Scaffold.body].
  ///
  /// This is unlikely to be a useful location for apps that lack a bottom
  /// navigation bar.
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_mini_start_docked.png)
  static const FloatingActionButtonLocation miniStartDocked = _MiniStartDockedFabLocation();

  /// Centered [FloatingActionButton], floating over the
  /// [Scaffold.bottomNavigationBar] so that the center of the floating
  /// action button lines up with the top of the bottom navigation bar.
  ///
  /// If the value of [Scaffold.bottomNavigationBar] is a [BottomAppBar],
  /// the bottom app bar can include a "notch" in its shape that accommodates
  /// the overlapping floating action button.
  ///
  /// This is unlikely to be a useful location for apps that lack a bottom
  /// navigation bar.
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_center_docked.png)
  static const FloatingActionButtonLocation centerDocked = _CenterDockedFabLocation();

  /// Centered [FloatingActionButton], floating over the
  /// [Scaffold.bottomNavigationBar] so that the center of the floating
  /// action button lines up with the top of the bottom navigation bar;
  /// intended to be used with [FloatingActionButton.mini] set to true.
  ///
  /// If the value of [Scaffold.bottomNavigationBar] is a [BottomAppBar],
  /// the bottom app bar can include a "notch" in its shape that accommodates
  /// the overlapping floating action button.
  ///
  /// This is unlikely to be a useful location for apps that lack a bottom
  /// navigation bar.
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_mini_center_docked.png)
  static const FloatingActionButtonLocation miniCenterDocked = _MiniCenterDockedFabLocation();

  /// End-aligned [FloatingActionButton], floating over the
  /// [Scaffold.bottomNavigationBar] so that the center of the floating
  /// action button lines up with the top of the bottom navigation bar.
  ///
  /// If the value of [Scaffold.bottomNavigationBar] is a [BottomAppBar],
  /// the bottom app bar can include a "notch" in its shape that accommodates
  /// the overlapping floating action button.
  ///
  /// This is unlikely to be a useful location for apps that lack a bottom
  /// navigation bar.
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_end_docked.png)
  static const FloatingActionButtonLocation endDocked = _EndDockedFabLocation();

  /// End-aligned [FloatingActionButton], floating over the
  /// [Scaffold.bottomNavigationBar] so that the center of the floating
  /// action button lines up with the top of the bottom navigation bar,
  /// optimized for mini floating action buttons.
  ///
  /// To align a floating action button with [CircleAvatar]s in the
  /// [ListTile.trailing] slots of [ListTile]s in a [ListView] in the [Scaffold.body],
  /// use [miniEndDocked] and set [FloatingActionButton.mini] to true.
  ///
  /// If the value of [Scaffold.bottomNavigationBar] is a [BottomAppBar],
  /// the bottom app bar can include a "notch" in its shape that accommodates
  /// the overlapping floating action button.
  ///
  /// This is intended to be used with [FloatingActionButton.mini] set to true,
  /// so that the floating action button appears to align with [CircleAvatar]s
  /// in the [ListTile.trailing] slot of a [ListTile] in a [ListView] in the
  /// [Scaffold.body].
  ///
  /// This is unlikely to be a useful location for apps that lack a bottom
  /// navigation bar.
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_mini_end_docked.png)
  static const FloatingActionButtonLocation miniEndDocked = _MiniEndDockedFabLocation();

  /// End-aligned [FloatingActionButton], floating over the
  /// [Scaffold.bottomNavigationBar] so that the floating
  /// action button lines up with the center of the bottom navigation bar.
  ///
  /// This is unlikely to be a useful location for apps which has a [BottomNavigationBar]
  /// or a non material 3 [BottomAppBar].
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/material/floating_action_button_location_end_contained.png)
  static const FloatingActionButtonLocation endContained = _EndContainedFabLocation();

  /// Places the [FloatingActionButton] based on the [Scaffold]'s layout.
  ///
  /// This uses a [ScaffoldPrelayoutGeometry], which the [Scaffold] constructs
  /// during its layout phase after it has laid out every widget it can lay out
  /// except the [FloatingActionButton]. The [Scaffold] uses the [Offset]
  /// returned from this method to position the [FloatingActionButton] and
  /// complete its layout.
  Offset getOffset(ScaffoldPrelayoutGeometry scaffoldGeometry);

  @override
  String toString() => objectRuntimeType(this, 'FloatingActionButtonLocation');
}

/// A base class that simplifies building [FloatingActionButtonLocation]s when
/// used with mixins [FabTopOffsetY], [FabFloatOffsetY], [FabDockedOffsetY],
/// [FabStartOffsetX], [FabCenterOffsetX], [FabEndOffsetX], and [FabMiniOffsetAdjustment].
///
/// A subclass of [FloatingActionButtonLocation] which implements its [getOffset] method
/// using three other methods: [getOffsetX], [getOffsetY], and [isMini].
///
/// Different mixins on this class override different methods, so that combining
/// a set of mixins creates a floating action button location.
///
/// For example: the location [FloatingActionButtonLocation.miniEndTop]
/// is based on a class that extends [StandardFabLocation]
/// with mixins [FabMiniOffsetAdjustment], [FabEndOffsetX], and [FabTopOffsetY].
///
/// You can create your own subclass of [StandardFabLocation]
/// to implement a custom [FloatingActionButtonLocation].
///
/// {@tool dartpad}
/// This is an example of a user-defined [FloatingActionButtonLocation].
///
/// The example shows a [Scaffold] with an [AppBar], a [BottomAppBar], and a
/// [FloatingActionButton] using a custom [FloatingActionButtonLocation].
///
/// The new [FloatingActionButtonLocation] is defined
/// by extending [StandardFabLocation] with two mixins,
/// [FabEndOffsetX] and [FabFloatOffsetY], and overriding the
/// [getOffsetX] method to adjust the FAB's x-coordinate, creating a
/// [FloatingActionButtonLocation] slightly different from
/// [FloatingActionButtonLocation.endFloat].
///
/// ** See code in examples/api/lib/material/floating_action_button_location/standard_fab_location.0.dart **
/// {@end-tool}
///
abstract class StandardFabLocation extends FloatingActionButtonLocation {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const StandardFabLocation();

  /// Obtains the x-offset to place the [FloatingActionButton] based on the
  /// [Scaffold]'s layout.
  ///
  /// Used by [getOffset] to compute its x-coordinate.
  double getOffsetX(ScaffoldPrelayoutGeometry scaffoldGeometry, double adjustment);

  /// Obtains the y-offset to place the [FloatingActionButton] based on the
  /// [Scaffold]'s layout.
  ///
  /// Used by [getOffset] to compute its y-coordinate.
  double getOffsetY(ScaffoldPrelayoutGeometry scaffoldGeometry, double adjustment);

  /// A function returning whether this [StandardFabLocation] is optimized for
  /// mini [FloatingActionButton]s.
  bool isMini() => false;

  @override
  Offset getOffset(ScaffoldPrelayoutGeometry scaffoldGeometry) {
    final double adjustment = isMini() ? kMiniButtonOffsetAdjustment : 0.0;
    return Offset(
      getOffsetX(scaffoldGeometry, adjustment),
      getOffsetY(scaffoldGeometry, adjustment),
    );
  }

  /// Calculates x-offset for left-aligned [FloatingActionButtonLocation]s.
  static double _leftOffsetX(ScaffoldPrelayoutGeometry scaffoldGeometry, double adjustment) {
    return kFloatingActionButtonMargin + scaffoldGeometry.minInsets.left - adjustment;
  }

  /// Calculates x-offset for right-aligned [FloatingActionButtonLocation]s.
  static double _rightOffsetX(ScaffoldPrelayoutGeometry scaffoldGeometry, double adjustment) {
    return scaffoldGeometry.scaffoldSize.width -
        kFloatingActionButtonMargin -
        scaffoldGeometry.minInsets.right -
        scaffoldGeometry.floatingActionButtonSize.width +
        adjustment;
  }
}

/// Mixin for a "top" floating action button location, such as
/// [FloatingActionButtonLocation.startTop].
///
/// The `adjustment`, typically [kMiniButtonOffsetAdjustment], is ignored in the
/// Y axis of "top" positions. For "top" positions, the X offset is adjusted to
/// move closer to the edge of the screen. This is so that a minified floating
/// action button appears to align with [CircleAvatar]s in the
/// [ListTile.leading] slot of a [ListTile] in a [ListView] in the
/// [Scaffold.body].
mixin FabTopOffsetY on StandardFabLocation {
  /// Calculates y-offset for [FloatingActionButtonLocation]s floating over
  /// the transition between the [Scaffold.appBar] and the [Scaffold.body].
  @override
  double getOffsetY(ScaffoldPrelayoutGeometry scaffoldGeometry, double adjustment) {
    if (scaffoldGeometry.contentTop > scaffoldGeometry.minViewPadding.top) {
      final double fabHalfHeight = scaffoldGeometry.floatingActionButtonSize.height / 2.0;
      return scaffoldGeometry.contentTop - fabHalfHeight;
    }
    // Otherwise, ensure we are placed within the bounds of a safe area.
    return scaffoldGeometry.minViewPadding.top;
  }
}

/// Mixin for a "float" floating action button location, such as [FloatingActionButtonLocation.centerFloat].
mixin FabFloatOffsetY on StandardFabLocation {
  /// Calculates y-offset for [FloatingActionButtonLocation]s floating at
  /// the bottom of the screen.
  @override
  double getOffsetY(ScaffoldPrelayoutGeometry scaffoldGeometry, double adjustment) {
    final double contentBottom = scaffoldGeometry.contentBottom;
    final double bottomContentHeight = scaffoldGeometry.scaffoldSize.height - contentBottom;
    final double bottomSheetHeight = scaffoldGeometry.bottomSheetSize.height;
    final double fabHeight = scaffoldGeometry.floatingActionButtonSize.height;
    final double snackBarHeight = scaffoldGeometry.snackBarSize.height;
    final double safeMargin = math.max(
      kFloatingActionButtonMargin,
      scaffoldGeometry.minViewPadding.bottom - bottomContentHeight + kFloatingActionButtonMargin,
    );

    double fabY = contentBottom - fabHeight - safeMargin;
    if (snackBarHeight > 0.0) {
      fabY = math.min(
        fabY,
        contentBottom - snackBarHeight - fabHeight - kFloatingActionButtonMargin,
      );
    }
    if (bottomSheetHeight > 0.0) {
      fabY = math.min(fabY, contentBottom - bottomSheetHeight - fabHeight / 2.0);
    }
    return fabY + adjustment;
  }
}

/// Mixin for a "docked" floating action button location, such as [FloatingActionButtonLocation.endDocked].
mixin FabDockedOffsetY on StandardFabLocation {
  /// Calculates y-offset for [FloatingActionButtonLocation]s floating over the
  /// [Scaffold.bottomNavigationBar] so that the center of the floating
  /// action button lines up with the top of the bottom navigation bar.
  @override
  double getOffsetY(ScaffoldPrelayoutGeometry scaffoldGeometry, double adjustment) {
    final double contentBottom = scaffoldGeometry.contentBottom;
    final double contentMargin = scaffoldGeometry.scaffoldSize.height - contentBottom;
    final double bottomViewPadding = scaffoldGeometry.minViewPadding.bottom;
    final double bottomSheetHeight = scaffoldGeometry.bottomSheetSize.height;
    final double fabHeight = scaffoldGeometry.floatingActionButtonSize.height;
    final double snackBarHeight = scaffoldGeometry.snackBarSize.height;
    final double bottomMinInset = scaffoldGeometry.minInsets.bottom;

    double safeMargin;

    if (contentMargin > bottomMinInset + fabHeight / 2.0) {
      // If contentMargin is higher than bottomMinInset enough to display the
      // FAB without clipping, don't provide a margin
      safeMargin = 0.0;
    } else if (bottomMinInset == 0.0) {
      // If bottomMinInset is zero(the software keyboard is not on the screen)
      // provide bottomViewPadding as margin
      safeMargin = bottomViewPadding;
    } else {
      // Provide a margin that would shift the FAB enough so that it stays away
      // from the keyboard
      safeMargin = fabHeight / 2.0 + kFloatingActionButtonMargin;
    }

    double fabY = contentBottom - fabHeight / 2.0 - safeMargin;
    // The FAB should sit with a margin between it and the snack bar.
    if (snackBarHeight > 0.0) {
      fabY = math.min(
        fabY,
        contentBottom - snackBarHeight - fabHeight - kFloatingActionButtonMargin,
      );
    }
    // The FAB should sit with its center in front of the top of the bottom sheet.
    if (bottomSheetHeight > 0.0) {
      fabY = math.min(fabY, contentBottom - bottomSheetHeight - fabHeight / 2.0);
    }
    final double maxFabY = scaffoldGeometry.scaffoldSize.height - fabHeight - safeMargin;
    return math.min(maxFabY, fabY);
  }
}

/// Mixin for a "contained" floating action button location, such as [FloatingActionButtonLocation.endContained].
mixin FabContainedOffsetY on StandardFabLocation {
  /// Calculates y-offset for [FloatingActionButtonLocation]s floating over the
  /// [Scaffold.bottomNavigationBar] so that the center of the floating
  /// action button lines up with the center of the bottom navigation bar.
  @override
  double getOffsetY(ScaffoldPrelayoutGeometry scaffoldGeometry, double adjustment) {
    final double contentBottom = scaffoldGeometry.contentBottom;
    final double contentMargin = scaffoldGeometry.scaffoldSize.height - contentBottom;
    final double bottomViewPadding = scaffoldGeometry.minViewPadding.bottom;
    final double fabHeight = scaffoldGeometry.floatingActionButtonSize.height;

    double safeMargin;
    if (contentMargin > bottomViewPadding + fabHeight) {
      // If contentMargin is higher than bottomViewPadding enough to display the
      // FAB without clipping, don't provide a margin
      safeMargin = 0.0;
    } else {
      safeMargin = bottomViewPadding;
    }

    // This is to compute the distance between the content bottom to the top edge
    // of the floating action button. This can be negative if content margin is
    // too small.
    final double contentBottomToFabTop = (contentMargin - bottomViewPadding - fabHeight) / 2.0;
    final double fabY = contentBottom + contentBottomToFabTop;
    final double maxFabY = scaffoldGeometry.scaffoldSize.height - fabHeight - safeMargin;

    return math.min(maxFabY, fabY);
  }
}

/// Mixin for a "start" floating action button location, such as [FloatingActionButtonLocation.startTop].
mixin FabStartOffsetX on StandardFabLocation {
  /// Calculates x-offset for start-aligned [FloatingActionButtonLocation]s.
  @override
  double getOffsetX(ScaffoldPrelayoutGeometry scaffoldGeometry, double adjustment) {
    return switch (scaffoldGeometry.textDirection) {
      TextDirection.rtl => StandardFabLocation._rightOffsetX(scaffoldGeometry, adjustment),
      TextDirection.ltr => StandardFabLocation._leftOffsetX(scaffoldGeometry, adjustment),
    };
  }
}

/// Mixin for a "center" floating action button location, such as [FloatingActionButtonLocation.centerFloat].
mixin FabCenterOffsetX on StandardFabLocation {
  /// Calculates x-offset for center-aligned [FloatingActionButtonLocation]s.
  @override
  double getOffsetX(ScaffoldPrelayoutGeometry scaffoldGeometry, double adjustment) {
    return (scaffoldGeometry.scaffoldSize.width - scaffoldGeometry.floatingActionButtonSize.width) /
        2.0;
  }
}

/// Mixin for an "end" floating action button location, such as [FloatingActionButtonLocation.endDocked].
mixin FabEndOffsetX on StandardFabLocation {
  /// Calculates x-offset for end-aligned [FloatingActionButtonLocation]s.
  @override
  double getOffsetX(ScaffoldPrelayoutGeometry scaffoldGeometry, double adjustment) {
    return switch (scaffoldGeometry.textDirection) {
      TextDirection.rtl => StandardFabLocation._leftOffsetX(scaffoldGeometry, adjustment),
      TextDirection.ltr => StandardFabLocation._rightOffsetX(scaffoldGeometry, adjustment),
    };
  }
}

/// Mixin for a "mini" floating action button location, such as [FloatingActionButtonLocation.miniStartTop].
mixin FabMiniOffsetAdjustment on StandardFabLocation {
  @override
  bool isMini() => true;
}

class _StartTopFabLocation extends StandardFabLocation with FabStartOffsetX, FabTopOffsetY {
  const _StartTopFabLocation();

  @override
  String toString() => 'FloatingActionButtonLocation.startTop';
}

class _MiniStartTopFabLocation extends StandardFabLocation
    with FabMiniOffsetAdjustment, FabStartOffsetX, FabTopOffsetY {
  const _MiniStartTopFabLocation();

  @override
  String toString() => 'FloatingActionButtonLocation.miniStartTop';
}

class _CenterTopFabLocation extends StandardFabLocation with FabCenterOffsetX, FabTopOffsetY {
  const _CenterTopFabLocation();

  @override
  String toString() => 'FloatingActionButtonLocation.centerTop';
}

class _MiniCenterTopFabLocation extends StandardFabLocation
    with FabMiniOffsetAdjustment, FabCenterOffsetX, FabTopOffsetY {
  const _MiniCenterTopFabLocation();

  @override
  String toString() => 'FloatingActionButtonLocation.miniCenterTop';
}

class _EndTopFabLocation extends StandardFabLocation with FabEndOffsetX, FabTopOffsetY {
  const _EndTopFabLocation();

  @override
  String toString() => 'FloatingActionButtonLocation.endTop';
}

class _MiniEndTopFabLocation extends StandardFabLocation
    with FabMiniOffsetAdjustment, FabEndOffsetX, FabTopOffsetY {
  const _MiniEndTopFabLocation();

  @override
  String toString() => 'FloatingActionButtonLocation.miniEndTop';
}

class _StartFloatFabLocation extends StandardFabLocation with FabStartOffsetX, FabFloatOffsetY {
  const _StartFloatFabLocation();

  @override
  String toString() => 'FloatingActionButtonLocation.startFloat';
}

class _MiniStartFloatFabLocation extends StandardFabLocation
    with FabMiniOffsetAdjustment, FabStartOffsetX, FabFloatOffsetY {
  const _MiniStartFloatFabLocation();

  @override
  String toString() => 'FloatingActionButtonLocation.miniStartFloat';
}

class _CenterFloatFabLocation extends StandardFabLocation with FabCenterOffsetX, FabFloatOffsetY {
  const _CenterFloatFabLocation();

  @override
  String toString() => 'FloatingActionButtonLocation.centerFloat';
}

class _MiniCenterFloatFabLocation extends StandardFabLocation
    with FabMiniOffsetAdjustment, FabCenterOffsetX, FabFloatOffsetY {
  const _MiniCenterFloatFabLocation();

  @override
  String toString() => 'FloatingActionButtonLocation.miniCenterFloat';
}

class _EndFloatFabLocation extends StandardFabLocation with FabEndOffsetX, FabFloatOffsetY {
  const _EndFloatFabLocation();

  @override
  String toString() => 'FloatingActionButtonLocation.endFloat';
}

class _MiniEndFloatFabLocation extends StandardFabLocation
    with FabMiniOffsetAdjustment, FabEndOffsetX, FabFloatOffsetY {
  const _MiniEndFloatFabLocation();

  @override
  String toString() => 'FloatingActionButtonLocation.miniEndFloat';
}

class _StartDockedFabLocation extends StandardFabLocation with FabStartOffsetX, FabDockedOffsetY {
  const _StartDockedFabLocation();

  @override
  String toString() => 'FloatingActionButtonLocation.startDocked';
}

class _MiniStartDockedFabLocation extends StandardFabLocation
    with FabMiniOffsetAdjustment, FabStartOffsetX, FabDockedOffsetY {
  const _MiniStartDockedFabLocation();

  @override
  String toString() => 'FloatingActionButtonLocation.miniStartDocked';
}

class _CenterDockedFabLocation extends StandardFabLocation with FabCenterOffsetX, FabDockedOffsetY {
  const _CenterDockedFabLocation();

  @override
  String toString() => 'FloatingActionButtonLocation.centerDocked';
}

class _MiniCenterDockedFabLocation extends StandardFabLocation
    with FabMiniOffsetAdjustment, FabCenterOffsetX, FabDockedOffsetY {
  const _MiniCenterDockedFabLocation();

  @override
  String toString() => 'FloatingActionButtonLocation.miniCenterDocked';
}

class _EndDockedFabLocation extends StandardFabLocation with FabEndOffsetX, FabDockedOffsetY {
  const _EndDockedFabLocation();

  @override
  String toString() => 'FloatingActionButtonLocation.endDocked';
}

class _MiniEndDockedFabLocation extends StandardFabLocation
    with FabMiniOffsetAdjustment, FabEndOffsetX, FabDockedOffsetY {
  const _MiniEndDockedFabLocation();

  @override
  String toString() => 'FloatingActionButtonLocation.miniEndDocked';
}

class _EndContainedFabLocation extends StandardFabLocation with FabEndOffsetX, FabContainedOffsetY {
  const _EndContainedFabLocation();

  @override
  String toString() => 'FloatingActionButtonLocation.endContained';
}

/// Provider of animations to move the [FloatingActionButton] between [FloatingActionButtonLocation]s.
///
/// The [Scaffold] uses [Scaffold.floatingActionButtonAnimator] to define:
///
///  * The [Offset] of the [FloatingActionButton] between the old and new
///    [FloatingActionButtonLocation]s as part of the transition animation.
///  * An [Animation] to scale the [FloatingActionButton] during the transition.
///  * An [Animation] to rotate the [FloatingActionButton] during the transition.
///  * Where to start a new animation from if an animation is interrupted.
///
/// See also:
///
///  * [FloatingActionButton], which is a circular button typically shown in the
///    bottom right corner of the app.
///  * [FloatingActionButtonLocation], which the [Scaffold] uses to place the
///    [Scaffold.floatingActionButton] within the [Scaffold]'s layout.
abstract class FloatingActionButtonAnimator {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const FloatingActionButtonAnimator();

  /// Moves the [FloatingActionButton] by scaling out and then in at a new
  /// [FloatingActionButtonLocation].
  ///
  /// This animator shrinks the [FloatingActionButton] down until it disappears, then
  /// grows it back to full size at its new [FloatingActionButtonLocation].
  ///
  /// This is the default [FloatingActionButton] motion animation.
  static const FloatingActionButtonAnimator scaling = _ScalingFabMotionAnimator();

  /// Gets the [FloatingActionButton]'s position relative to the origin of the
  /// [Scaffold] based on [progress].
  ///
  /// [begin] is the [Offset] provided by the previous
  /// [FloatingActionButtonLocation].
  ///
  /// [end] is the [Offset] provided by the new
  /// [FloatingActionButtonLocation].
  ///
  /// [progress] is the current progress of the transition animation.
  /// When [progress] is 0.0, the returned [Offset] should be equal to [begin].
  /// when [progress] is 1.0, the returned [Offset] should be equal to [end].
  Offset getOffset({required Offset begin, required Offset end, required double progress});

  /// Animates the scale of the [FloatingActionButton].
  ///
  /// The animation should both start and end with a value of 1.0.
  ///
  /// For example, to create an animation that linearly scales out and then back in,
  /// you could join animations that pass each other:
  ///
  /// ```dart
  ///   @override
  ///   Animation<double> getScaleAnimation({required Animation<double> parent}) {
  ///     // The animations will cross at value 0, and the train will return to 1.0.
  ///     return TrainHoppingAnimation(
  ///       Tween<double>(begin: 1.0, end: -1.0).animate(parent),
  ///       Tween<double>(begin: -1.0, end: 1.0).animate(parent),
  ///     );
  ///   }
  /// ```
  Animation<double> getScaleAnimation({required Animation<double> parent});

  /// Animates the rotation of [Scaffold.floatingActionButton].
  ///
  /// The animation should both start and end with a value of 0.0 or 1.0.
  ///
  /// The animation values are a fraction of a full circle, with 0.0 and 1.0
  /// corresponding to 0 and 360 degrees, while 0.5 corresponds to 180 degrees.
  ///
  /// For example, to create a rotation animation that rotates the
  /// [FloatingActionButton] through a full circle:
  ///
  /// ```dart
  /// @override
  /// Animation<double> getRotationAnimation({required Animation<double> parent}) {
  ///   return Tween<double>(begin: 0.0, end: 1.0).animate(parent);
  /// }
  /// ```
  Animation<double> getRotationAnimation({required Animation<double> parent});

  /// Gets the progress value to restart a motion animation from when the animation is interrupted.
  ///
  /// [previousValue] is the value of the animation before it was interrupted.
  ///
  /// The restart of the animation will affect all three parts of the motion animation:
  /// offset animation, scale animation, and rotation animation.
  ///
  /// An interruption triggers if the [Scaffold] is given a new [FloatingActionButtonLocation]
  /// while it is still animating a transition between two previous [FloatingActionButtonLocation]s.
  ///
  /// A sensible default is usually 0.0, which is the same as restarting
  /// the animation from the beginning, regardless of the original state of the animation.
  double getAnimationRestart(double previousValue) => 0.0;

  /// Creates an instance of [FloatingActionButtonAnimator] where the [FloatingActionButton]
  /// does not animate on entrance and exit when [FloatingActionButtonLocation] is shown
  /// or hidden and when transitioning between [FloatingActionButtonLocation]s.
  ///
  /// {@tool dartpad}
  /// This sample showcases how to override [FloatingActionButton] entrance and exit animations
  /// using [FloatingActionButtonAnimator.noAnimation] in [Scaffold.floatingActionButtonAnimator].
  ///
  /// ** See code in examples/api/lib/material/scaffold/scaffold.floating_action_button_animator.0.dart **
  /// {@end-tool}
  static const FloatingActionButtonAnimator noAnimation = _NoAnimationFabMotionAnimator();

  @override
  String toString() => objectRuntimeType(this, 'FloatingActionButtonAnimator');
}

class _ScalingFabMotionAnimator extends FloatingActionButtonAnimator {
  const _ScalingFabMotionAnimator();

  @override
  Offset getOffset({required Offset begin, required Offset end, required double progress}) {
    if (progress < 0.5) {
      return begin;
    } else {
      return end;
    }
  }

  @override
  Animation<double> getScaleAnimation({required Animation<double> parent}) {
    // Animate the scale down from 1 to 0 in the first half of the animation
    // then from 0 back to 1 in the second half.
    const Curve curve = Interval(0.5, 1.0, curve: Curves.ease);
    return _AnimationSwap<double>(
      ReverseAnimation(parent.drive(CurveTween(curve: curve.flipped))),
      parent.drive(CurveTween(curve: curve)),
      parent,
      0.5,
    );
  }

  // Because we only see the last half of the rotation tween,
  // it needs to go twice as far.
  static final Animatable<double> _rotationTween = Tween<double>(
    begin: 1.0 - kFloatingActionButtonTurnInterval * 2.0,
    end: 1.0,
  );

  static final Animatable<double> _thresholdCenterTween = CurveTween(curve: const Threshold(0.5));

  @override
  Animation<double> getRotationAnimation({required Animation<double> parent}) {
    // This rotation will turn on the way in, but not on the way out.
    return _AnimationSwap<double>(
      parent.drive(_rotationTween),
      ReverseAnimation(parent.drive(_thresholdCenterTween)),
      parent,
      0.5,
    );
  }

  // If the animation was just starting, we'll continue from where we left off.
  // If the animation was finishing, we'll treat it as if we were starting at that point in reverse.
  // This avoids a size jump during the animation.
  @override
  double getAnimationRestart(double previousValue) => math.min(1.0 - previousValue, previousValue);
}

class _NoAnimationFabMotionAnimator extends FloatingActionButtonAnimator {
  const _NoAnimationFabMotionAnimator();

  @override
  Offset getOffset({required Offset begin, required Offset end, required double progress}) {
    return end;
  }

  @override
  Animation<double> getRotationAnimation({required Animation<double> parent}) {
    return const AlwaysStoppedAnimation<double>(1.0);
  }

  @override
  Animation<double> getScaleAnimation({required Animation<double> parent}) {
    return const AlwaysStoppedAnimation<double>(1.0);
  }
}

/// An animation that swaps from one animation to the next when the [parent] passes [swapThreshold].
///
/// The [value] of this animation is the value of [first] when `parent.value` < [swapThreshold]
/// and the value of [next] otherwise.
class _AnimationSwap<T> extends CompoundAnimation<T> {
  /// Creates an [_AnimationSwap].
  ///
  /// Either argument can be an [_AnimationSwap] itself to combine multiple
  /// animations.
  _AnimationSwap(Animation<T> first, Animation<T> next, this.parent, this.swapThreshold)
    : super(first: first, next: next);

  final Animation<double> parent;
  final double swapThreshold;

  @override
  T get value => parent.value < swapThreshold ? first.value : next.value;
}
