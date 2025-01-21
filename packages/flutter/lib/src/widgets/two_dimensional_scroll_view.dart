// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'scroll_position.dart';
library;

import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';

import 'focus_manager.dart';
import 'focus_scope.dart';
import 'framework.dart';
import 'notification_listener.dart';
import 'primary_scroll_controller.dart';
import 'scroll_configuration.dart';
import 'scroll_controller.dart';
import 'scroll_delegate.dart';
import 'scroll_notification.dart';
import 'scroll_physics.dart';
import 'scroll_view.dart';
import 'scrollable.dart';
import 'scrollable_helpers.dart';
import 'two_dimensional_viewport.dart';

/// A widget that combines a [TwoDimensionalScrollable] and a
/// [TwoDimensionalViewport] to create an interactive scrolling pane of content
/// in both vertical and horizontal dimensions.
///
/// A two-way scrollable widget consist of three pieces:
///
///  1. A [TwoDimensionalScrollable] widget, which listens for various user
///     gestures and implements the interaction design for scrolling.
///  2. A [TwoDimensionalViewport] widget, which implements the visual design
///     for scrolling by displaying only a portion
///     of the widgets inside the scroll view.
///  3. A [TwoDimensionalChildDelegate], which provides the children visible in
///     the scroll view.
///
/// [TwoDimensionalScrollView] helps orchestrate these pieces by creating the
/// [TwoDimensionalScrollable] and deferring to its subclass to implement
/// [buildViewport], which builds a subclass of [TwoDimensionalViewport]. The
/// [TwoDimensionalChildDelegate] is provided by the [delegate] parameter.
///
/// A [TwoDimensionalScrollView] has two different [ScrollPosition]s, one for
/// each [Axis]. This means that there are also two unique [ScrollController]s
/// for these positions. To provide a ScrollController to access the
/// ScrollPosition, use the [ScrollableDetails.controller] property of the
/// associated axis that is provided to this scroll view.
abstract class TwoDimensionalScrollView extends StatelessWidget {
  /// Creates a widget that scrolls in both dimensions.
  ///
  /// The [primary] argument is associated with the [mainAxis]. The main axis
  /// [ScrollableDetails.controller] must be null if [primary] is configured for
  /// that axis. If [primary] is true, the nearest [PrimaryScrollController]
  /// surrounding the widget is attached to the scroll position of that axis.
  const TwoDimensionalScrollView({
    super.key,
    this.primary,
    this.mainAxis = Axis.vertical,
    this.verticalDetails = const ScrollableDetails.vertical(),
    this.horizontalDetails = const ScrollableDetails.horizontal(),
    required this.delegate,
    this.cacheExtent,
    this.diagonalDragBehavior = DiagonalDragBehavior.none,
    this.dragStartBehavior = DragStartBehavior.start,
    this.keyboardDismissBehavior,
    this.clipBehavior = Clip.hardEdge,
    this.hitTestBehavior = HitTestBehavior.opaque,
  });

  /// A delegate that provides the children for the [TwoDimensionalScrollView].
  final TwoDimensionalChildDelegate delegate;

  /// {@macro flutter.rendering.RenderViewportBase.cacheExtent}
  final double? cacheExtent;

  /// Whether scrolling gestures should lock to one axes, allow free movement
  /// in both axes, or be evaluated on a weighted scale.
  ///
  /// Defaults to [DiagonalDragBehavior.none], locking axes to receive input one
  /// at a time.
  final DiagonalDragBehavior diagonalDragBehavior;

  /// {@macro flutter.widgets.scroll_view.primary}
  final bool? primary;

  /// The main axis of the two.
  ///
  /// Used to determine how to apply [primary] when true.
  ///
  /// This value should also be provided to the subclass of
  /// [TwoDimensionalViewport], where it is used to determine paint order of
  /// children.
  final Axis mainAxis;

  /// The configuration of the vertical Scrollable.
  ///
  /// These [ScrollableDetails] can be used to set the [AxisDirection],
  /// [ScrollController], [ScrollPhysics] and more for the vertical axis.
  final ScrollableDetails verticalDetails;

  /// The configuration of the horizontal Scrollable.
  ///
  /// These [ScrollableDetails] can be used to set the [AxisDirection],
  /// [ScrollController], [ScrollPhysics] and more for the horizontal axis.
  final ScrollableDetails horizontalDetails;

  /// {@macro flutter.widgets.scrollable.dragStartBehavior}
  final DragStartBehavior dragStartBehavior;

  /// {@macro flutter.widgets.scroll_view.keyboardDismissBehavior}
  ///
  /// If [keyboardDismissBehavior] is null then it will fallback to the inherited
  /// [ScrollBehavior.getKeyboardDismissBehavior].
  final ScrollViewKeyboardDismissBehavior? keyboardDismissBehavior;

  /// {@macro flutter.widgets.scrollable.hitTestBehavior}
  ///
  /// This value applies to both axes.
  final HitTestBehavior hitTestBehavior;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.hardEdge].
  final Clip clipBehavior;

  /// Build the two dimensional viewport.
  ///
  /// Subclasses may override this method to change how the viewport is built,
  /// likely a subclass of [TwoDimensionalViewport].
  ///
  /// The `verticalOffset` and `horizontalOffset` arguments are the values
  /// obtained from [TwoDimensionalScrollable.viewportBuilder].
  Widget buildViewport(
    BuildContext context,
    ViewportOffset verticalOffset,
    ViewportOffset horizontalOffset,
  );

  @override
  Widget build(BuildContext context) {
    assert(
      axisDirectionToAxis(verticalDetails.direction) == Axis.vertical,
      'TwoDimensionalScrollView.verticalDetails are not Axis.vertical.',
    );
    assert(
      axisDirectionToAxis(horizontalDetails.direction) == Axis.horizontal,
      'TwoDimensionalScrollView.horizontalDetails are not Axis.horizontal.',
    );

    ScrollableDetails mainAxisDetails = switch (mainAxis) {
      Axis.vertical => verticalDetails,
      Axis.horizontal => horizontalDetails,
    };

    final bool effectivePrimary =
        primary ??
        mainAxisDetails.controller == null &&
            PrimaryScrollController.shouldInherit(context, mainAxis);

    if (effectivePrimary) {
      // Using PrimaryScrollController for mainAxis.
      assert(
        mainAxisDetails.controller == null,
        'TwoDimensionalScrollView.primary was explicitly set to true, but a '
        'ScrollController was provided in the ScrollableDetails of the '
        'TwoDimensionalScrollView.mainAxis.',
      );
      mainAxisDetails = mainAxisDetails.copyWith(controller: PrimaryScrollController.of(context));
    }

    final TwoDimensionalScrollable scrollable = TwoDimensionalScrollable(
      horizontalDetails: switch (mainAxis) {
        Axis.horizontal => mainAxisDetails,
        Axis.vertical => horizontalDetails,
      },
      verticalDetails: switch (mainAxis) {
        Axis.vertical => mainAxisDetails,
        Axis.horizontal => verticalDetails,
      },
      diagonalDragBehavior: diagonalDragBehavior,
      viewportBuilder: buildViewport,
      dragStartBehavior: dragStartBehavior,
      hitTestBehavior: hitTestBehavior,
    );

    final Widget scrollableResult =
        effectivePrimary
            // Further descendant ScrollViews will not inherit the same PrimaryScrollController
            ? PrimaryScrollController.none(child: scrollable)
            : scrollable;

    final ScrollViewKeyboardDismissBehavior effectiveKeyboardDismissBehavior =
        keyboardDismissBehavior ??
        ScrollConfiguration.of(context).getKeyboardDismissBehavior(context);

    if (effectiveKeyboardDismissBehavior == ScrollViewKeyboardDismissBehavior.onDrag) {
      return NotificationListener<ScrollUpdateNotification>(
        child: scrollableResult,
        onNotification: (ScrollUpdateNotification notification) {
          final FocusScopeNode currentScope = FocusScope.of(context);
          if (notification.dragDetails != null &&
              !currentScope.hasPrimaryFocus &&
              currentScope.hasFocus) {
            FocusManager.instance.primaryFocus?.unfocus();
          }
          return false;
        },
      );
    }
    return scrollableResult;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(EnumProperty<Axis>('mainAxis', mainAxis));
    properties.add(
      EnumProperty<DiagonalDragBehavior>('diagonalDragBehavior', diagonalDragBehavior),
    );
    properties.add(
      FlagProperty('primary', value: primary, ifTrue: 'using primary controller', showName: true),
    );
    properties.add(
      DiagnosticsProperty<ScrollableDetails>('verticalDetails', verticalDetails, showName: false),
    );
    properties.add(
      DiagnosticsProperty<ScrollableDetails>(
        'horizontalDetails',
        horizontalDetails,
        showName: false,
      ),
    );
  }
}
