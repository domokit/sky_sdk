// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:ui';
///
/// @docImport 'package:flutter/material.dart';
/// @docImport 'package:flutter/services.dart';
///
/// @docImport 'app.dart';
/// @docImport 'button.dart';
/// @docImport 'dialog.dart';
/// @docImport 'nav_bar.dart';
/// @docImport 'page_scaffold.dart';
/// @docImport 'tab_scaffold.dart';
library;

import 'dart:math';
import 'dart:ui' show ImageFilter;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/physics.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

import 'colors.dart';
import 'interface_level.dart';
import 'localizations.dart';

const double _kBackGestureWidth = 20.0;
const double _kMinFlingVelocity = 1.0; // Screen widths per second.

// The duration for a page to animate when the user releases it mid-swipe.
const Duration _kDroppedSwipePageAnimationDuration = Duration(milliseconds: 350);

/// Barrier color used for a barrier visible during transitions for Cupertino
/// page routes.
///
/// This barrier color is only used for full-screen page routes with
/// `fullscreenDialog: false`.
///
/// By default, `fullscreenDialog` Cupertino route transitions have no
/// `barrierColor`, and [CupertinoDialogRoute]s and [CupertinoModalPopupRoute]s
/// have a `barrierColor` defined by [kCupertinoModalBarrierColor].
///
/// A relatively rigorous eyeball estimation.
const Color _kCupertinoPageTransitionBarrierColor = Color(0x18000000);

/// Barrier color for a Cupertino modal barrier.
///
/// Extracted from https://developer.apple.com/design/resources/.
const Color kCupertinoModalBarrierColor = CupertinoDynamicColor.withBrightness(
  color: Color(0x33000000),
  darkColor: Color(0x7A000000),
);

// The duration of the transition used when a modal popup is shown.
const Duration _kModalPopupTransitionDuration = Duration(milliseconds: 335);

// Offset from offscreen to the right to fully on screen.
final Animatable<Offset> _kRightMiddleTween = Tween<Offset>(
  begin: const Offset(1.0, 0.0),
  end: Offset.zero,
);

// Offset from fully on screen to 1/3 offscreen to the left.
final Animatable<Offset> _kMiddleLeftTween = Tween<Offset>(
  begin: Offset.zero,
  end: const Offset(-1.0 / 3.0, 0.0),
);

// Offset from offscreen below to fully on screen.
final Animatable<Offset> _kBottomUpTween = Tween<Offset>(
  begin: const Offset(0.0, 1.0),
  end: Offset.zero,
);

/// A mixin that replaces the entire screen with an iOS transition for a
/// [PageRoute].
///
/// {@template flutter.cupertino.cupertinoRouteTransitionMixin}
/// The page slides in from the right and exits in reverse. The page also shifts
/// to the left in parallax when another page enters to cover it.
///
/// The page slides in from the bottom and exits in reverse with no parallax
/// effect for fullscreen dialogs.
/// {@endtemplate}
///
/// See also:
///
///  * [MaterialRouteTransitionMixin], which is a mixin that provides
///    platform-appropriate transitions for a [PageRoute].
///  * [CupertinoPageRoute], which is a [PageRoute] that leverages this mixin.
mixin CupertinoRouteTransitionMixin<T> on PageRoute<T> {
  /// Builds the primary contents of the route.
  @protected
  Widget buildContent(BuildContext context);

  /// {@template flutter.cupertino.CupertinoRouteTransitionMixin.title}
  /// A title string for this route.
  ///
  /// Used to auto-populate [CupertinoNavigationBar] and
  /// [CupertinoSliverNavigationBar]'s `middle`/`largeTitle` widgets when
  /// one is not manually supplied.
  /// {@endtemplate}
  String? get title;

  ValueNotifier<String?>? _previousTitle;

  /// The title string of the previous [CupertinoPageRoute].
  ///
  /// The [ValueListenable]'s value is readable after the route is installed
  /// onto a [Navigator]. The [ValueListenable] will also notify its listeners
  /// if the value changes (such as by replacing the previous route).
  ///
  /// The [ValueListenable] itself will be null before the route is installed.
  /// Its content value will be null if the previous route has no title or
  /// is not a [CupertinoPageRoute].
  ///
  /// See also:
  ///
  ///  * [ValueListenableBuilder], which can be used to listen and rebuild
  ///    widgets based on a ValueListenable.
  ValueListenable<String?> get previousTitle {
    assert(
      _previousTitle != null,
      'Cannot read the previousTitle for a route that has not yet been installed',
    );
    return _previousTitle!;
  }

  @override
  void dispose() {
    _previousTitle?.dispose();
    super.dispose();
  }

  @override
  void didChangePrevious(Route<dynamic>? previousRoute) {
    final String? previousTitleString =
        previousRoute is CupertinoRouteTransitionMixin ? previousRoute.title : null;
    if (_previousTitle == null) {
      _previousTitle = ValueNotifier<String?>(previousTitleString);
    } else {
      _previousTitle!.value = previousTitleString;
    }
    super.didChangePrevious(previousRoute);
  }

  @override
  // A relatively rigorous eyeball estimation.
  Duration get transitionDuration => const Duration(milliseconds: 500);

  @override
  Color? get barrierColor => fullscreenDialog ? null : _kCupertinoPageTransitionBarrierColor;

  @override
  String? get barrierLabel => null;

  @override
  bool canTransitionTo(TransitionRoute<dynamic> nextRoute) {
    // Don't perform outgoing animation if the next route is a fullscreen dialog.
    final bool nextRouteIsNotFullscreen =
        (nextRoute is! PageRoute<T>) || !nextRoute.fullscreenDialog;

    // If the next route has a delegated transition, then this route is able to
    // use that delegated transition to smoothly sync with the next route's
    // transition.
    final bool nextRouteHasDelegatedTransition =
        nextRoute is ModalRoute<T> && nextRoute.delegatedTransition != null;

    // Otherwise if the next route has the same route transition mixin as this
    // one, then this route will already be synced with its transition.
    return nextRouteIsNotFullscreen &&
        ((nextRoute is CupertinoRouteTransitionMixin) || nextRouteHasDelegatedTransition);
  }

  @override
  bool canTransitionFrom(TransitionRoute<dynamic> previousRoute) {
    // Supress previous route from transitioning if this is a fullscreenDialog route.
    return previousRoute is PageRoute && !fullscreenDialog;
  }

  @override
  Widget buildPage(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
  ) {
    final Widget child = buildContent(context);
    return Semantics(scopesRoute: true, explicitChildNodes: true, child: child);
  }

  // Called by _CupertinoBackGestureDetector when a pop ("back") drag start
  // gesture is detected. The returned controller handles all of the subsequent
  // drag events.
  static _CupertinoBackGestureController<T> _startPopGesture<T>(PageRoute<T> route) {
    assert(route.popGestureEnabled);

    return _CupertinoBackGestureController<T>(
      navigator: route.navigator!,
      getIsCurrent: () => route.isCurrent,
      getIsActive: () => route.isActive,
      controller: route.controller!, // protected access
    );
  }

  /// Returns a [CupertinoFullscreenDialogTransition] if [route] is a full
  /// screen dialog, otherwise a [CupertinoPageTransition] is returned.
  ///
  /// Used by [CupertinoPageRoute.buildTransitions].
  ///
  /// This method can be applied to any [PageRoute], not just
  /// [CupertinoPageRoute]. It's typically used to provide a Cupertino style
  /// horizontal transition for material widgets when the target platform
  /// is [TargetPlatform.iOS].
  ///
  /// See also:
  ///
  ///  * [CupertinoPageTransitionsBuilder], which uses this method to define a
  ///    [PageTransitionsBuilder] for the [PageTransitionsTheme].
  static Widget buildPageTransitions<T>(
    PageRoute<T> route,
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
    Widget child,
  ) {
    // Check if the route has an animation that's currently participating
    // in a back swipe gesture.
    //
    // In the middle of a back gesture drag, let the transition be linear to
    // match finger motions.
    final bool linearTransition = route.popGestureInProgress;
    if (route.fullscreenDialog) {
      return CupertinoFullscreenDialogTransition(
        primaryRouteAnimation: animation,
        secondaryRouteAnimation: secondaryAnimation,
        linearTransition: linearTransition,
        child: child,
      );
    } else {
      return CupertinoPageTransition(
        primaryRouteAnimation: animation,
        secondaryRouteAnimation: secondaryAnimation,
        linearTransition: linearTransition,
        child: _CupertinoBackGestureDetector<T>(
          enabledCallback: () => route.popGestureEnabled,
          onStartPopGesture: () => _startPopGesture<T>(route),
          child: child,
        ),
      );
    }
  }

  @override
  Widget buildTransitions(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
    Widget child,
  ) {
    return buildPageTransitions<T>(this, context, animation, secondaryAnimation, child);
  }
}

/// A modal route that replaces the entire screen with an iOS transition.
///
/// {@macro flutter.cupertino.cupertinoRouteTransitionMixin}
///
/// By default, when a modal route is replaced by another, the previous route
/// remains in memory. To free all the resources when this is not necessary, set
/// [maintainState] to false.
///
/// The type `T` specifies the return type of the route which can be supplied as
/// the route is popped from the stack via [Navigator.pop] when an optional
/// `result` can be provided.
///
/// If `barrierDismissible` is true, then pressing the escape key on the keyboard
/// will cause the current route to be popped with null as the value.
///
/// See also:
///
///  * [CupertinoRouteTransitionMixin], for a mixin that provides iOS transition
///    for this modal route.
///  * [MaterialPageRoute], for an adaptive [PageRoute] that uses a
///    platform-appropriate transition.
///  * [CupertinoPageScaffold], for applications that have one page with a fixed
///    navigation bar on top.
///  * [CupertinoTabScaffold], for applications that have a tab bar at the
///    bottom with multiple pages.
///  * [CupertinoPage], for a [Page] version of this class.
class CupertinoPageRoute<T> extends PageRoute<T> with CupertinoRouteTransitionMixin<T> {
  /// Creates a page route for use in an iOS designed app.
  ///
  /// The [builder], [maintainState], and [fullscreenDialog] arguments must not
  /// be null.
  CupertinoPageRoute({
    required this.builder,
    this.title,
    super.settings,
    super.requestFocus,
    this.maintainState = true,
    super.fullscreenDialog,
    super.allowSnapshotting = true,
    super.barrierDismissible = false,
  }) {
    assert(opaque);
  }

  @override
  DelegatedTransitionBuilder? get delegatedTransition =>
      CupertinoPageTransition.delegatedTransition;

  /// Builds the primary contents of the route.
  final WidgetBuilder builder;

  @override
  Widget buildContent(BuildContext context) => builder(context);

  @override
  final String? title;

  @override
  final bool maintainState;

  @override
  String get debugLabel => '${super.debugLabel}(${settings.name})';
}

// A page-based version of CupertinoPageRoute.
//
// This route uses the builder from the page to build its content. This ensures
// the content is up to date after page updates.
class _PageBasedCupertinoPageRoute<T> extends PageRoute<T> with CupertinoRouteTransitionMixin<T> {
  _PageBasedCupertinoPageRoute({required CupertinoPage<T> page, super.allowSnapshotting = true})
    : super(settings: page) {
    assert(opaque);
  }

  @override
  DelegatedTransitionBuilder? get delegatedTransition =>
      this.fullscreenDialog ? null : CupertinoPageTransition.delegatedTransition;

  CupertinoPage<T> get _page => settings as CupertinoPage<T>;

  @override
  Widget buildContent(BuildContext context) => _page.child;

  @override
  String? get title => _page.title;

  @override
  bool get maintainState => _page.maintainState;

  @override
  bool get fullscreenDialog => _page.fullscreenDialog;

  @override
  String get debugLabel => '${super.debugLabel}(${_page.name})';
}

/// A page that creates a cupertino style [PageRoute].
///
/// {@macro flutter.cupertino.cupertinoRouteTransitionMixin}
///
/// By default, when a created modal route is replaced by another, the previous
/// route remains in memory. To free all the resources when this is not
/// necessary, set [maintainState] to false.
///
/// The type `T` specifies the return type of the route which can be supplied as
/// the route is popped from the stack via [Navigator.transitionDelegate] by
/// providing the optional `result` argument to the
/// [RouteTransitionRecord.markForPop] in the [TransitionDelegate.resolve].
///
/// See also:
///
///  * [CupertinoPageRoute], for a [PageRoute] version of this class.
class CupertinoPage<T> extends Page<T> {
  /// Creates a cupertino page.
  const CupertinoPage({
    required this.child,
    this.maintainState = true,
    this.title,
    this.fullscreenDialog = false,
    this.allowSnapshotting = true,
    super.canPop,
    super.onPopInvoked,
    super.key,
    super.name,
    super.arguments,
    super.restorationId,
  });

  /// The content to be shown in the [Route] created by this page.
  final Widget child;

  /// {@macro flutter.cupertino.CupertinoRouteTransitionMixin.title}
  final String? title;

  /// {@macro flutter.widgets.ModalRoute.maintainState}
  final bool maintainState;

  /// {@macro flutter.widgets.PageRoute.fullscreenDialog}
  final bool fullscreenDialog;

  /// {@macro flutter.widgets.TransitionRoute.allowSnapshotting}
  final bool allowSnapshotting;

  @override
  Route<T> createRoute(BuildContext context) {
    return _PageBasedCupertinoPageRoute<T>(page: this, allowSnapshotting: allowSnapshotting);
  }
}

/// Provides an iOS-style page transition animation.
///
/// The page slides in from the right and exits in reverse. It also shifts to the left in
/// a parallax motion when another page enters to cover it.
class CupertinoPageTransition extends StatefulWidget {
  /// Creates an iOS-style page transition.
  ///
  const CupertinoPageTransition({
    super.key,
    required this.primaryRouteAnimation,
    required this.secondaryRouteAnimation,
    required this.child,
    required this.linearTransition,
  });

  /// The widget below this widget in the tree.
  final Widget child;

  ///  * `primaryRouteAnimation` is a linear route animation from 0.0 to 1.0
  ///    when this screen is being pushed.
  final Animation<double> primaryRouteAnimation;

  ///  * `secondaryRouteAnimation` is a linear route animation from 0.0 to 1.0
  ///    when another screen is being pushed on top of this one.
  final Animation<double> secondaryRouteAnimation;

  ///  * `linearTransition` is whether to perform the transitions linearly.
  ///    Used to precisely track back gesture drags.
  final bool linearTransition;

  /// The Cupertino styled [DelegatedTransitionBuilder] provided to the previous
  /// route.
  ///
  /// {@macro flutter.widgets.delegatedTransition}
  static Widget? delegatedTransition(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
    bool allowSnapshotting,
    Widget? child,
  ) {
    final CurvedAnimation animation = CurvedAnimation(
      parent: secondaryAnimation,
      curve: Curves.linearToEaseOut,
      reverseCurve: Curves.easeInToLinear,
    );
    final Animation<Offset> delegatedPositionAnimation = animation.drive(_kMiddleLeftTween);
    animation.dispose();

    assert(debugCheckHasDirectionality(context));
    final TextDirection textDirection = Directionality.of(context);
    return SlideTransition(
      position: delegatedPositionAnimation,
      textDirection: textDirection,
      transformHitTests: false,
      child: child,
    );
  }

  @override
  State<CupertinoPageTransition> createState() => _CupertinoPageTransitionState();
}

class _CupertinoPageTransitionState extends State<CupertinoPageTransition> {
  // When this page is coming in to cover another page.
  late Animation<Offset> _primaryPositionAnimation;
  // When this page is becoming covered by another page.
  late Animation<Offset> _secondaryPositionAnimation;
  // Shadow of page which is coming in to cover another page.
  late Animation<Decoration> _primaryShadowAnimation;
  // Curve of primary page which is coming in to cover another page.
  CurvedAnimation? _primaryPositionCurve;
  // Curve of secondary page which is becoming covered by another page.
  CurvedAnimation? _secondaryPositionCurve;
  // Curve of primary page's shadow.
  CurvedAnimation? _primaryShadowCurve;

  @override
  void initState() {
    super.initState();
    _setupAnimation();
  }

  @override
  void didUpdateWidget(covariant CupertinoPageTransition oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.primaryRouteAnimation != widget.primaryRouteAnimation ||
        oldWidget.secondaryRouteAnimation != widget.secondaryRouteAnimation ||
        oldWidget.linearTransition != widget.linearTransition) {
      _disposeCurve();
      _setupAnimation();
    }
  }

  @override
  void dispose() {
    _disposeCurve();
    super.dispose();
  }

  void _disposeCurve() {
    _primaryPositionCurve?.dispose();
    _secondaryPositionCurve?.dispose();
    _primaryShadowCurve?.dispose();
    _primaryPositionCurve = null;
    _secondaryPositionCurve = null;
    _primaryShadowCurve = null;
  }

  void _setupAnimation() {
    if (!widget.linearTransition) {
      _primaryPositionCurve = CurvedAnimation(
        parent: widget.primaryRouteAnimation,
        curve: Curves.fastEaseInToSlowEaseOut,
        reverseCurve: Curves.fastEaseInToSlowEaseOut.flipped,
      );
      _secondaryPositionCurve = CurvedAnimation(
        parent: widget.secondaryRouteAnimation,
        curve: Curves.linearToEaseOut,
        reverseCurve: Curves.easeInToLinear,
      );
      _primaryShadowCurve = CurvedAnimation(
        parent: widget.primaryRouteAnimation,
        curve: Curves.linearToEaseOut,
      );
    }
    _primaryPositionAnimation = (_primaryPositionCurve ?? widget.primaryRouteAnimation).drive(
      _kRightMiddleTween,
    );
    _secondaryPositionAnimation = (_secondaryPositionCurve ?? widget.secondaryRouteAnimation).drive(
      _kMiddleLeftTween,
    );
    _primaryShadowAnimation = (_primaryShadowCurve ?? widget.primaryRouteAnimation).drive(
      _CupertinoEdgeShadowDecoration.kTween,
    );
  }

  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasDirectionality(context));
    final TextDirection textDirection = Directionality.of(context);
    return SlideTransition(
      position: _secondaryPositionAnimation,
      textDirection: textDirection,
      transformHitTests: false,
      child: SlideTransition(
        position: _primaryPositionAnimation,
        textDirection: textDirection,
        child: DecoratedBoxTransition(decoration: _primaryShadowAnimation, child: widget.child),
      ),
    );
  }
}

/// An iOS-style transition used for summoning fullscreen dialogs.
///
/// For example, used when creating a new calendar event by bringing in the next
/// screen from the bottom.
class CupertinoFullscreenDialogTransition extends StatefulWidget {
  /// Creates an iOS-style transition used for summoning fullscreen dialogs.
  ///
  const CupertinoFullscreenDialogTransition({
    super.key,
    required this.primaryRouteAnimation,
    required this.secondaryRouteAnimation,
    required this.child,
    required this.linearTransition,
  });

  ///  * `primaryRouteAnimation` is a linear route animation from 0.0 to 1.0
  ///    when this screen is being pushed.
  final Animation<double> primaryRouteAnimation;

  ///  * `secondaryRouteAnimation` is a linear route animation from 0.0 to 1.0
  ///    when another screen is being pushed on top of this one.
  final Animation<double> secondaryRouteAnimation;

  ///  * `linearTransition` is whether to perform the transitions linearly.
  ///    Used to precisely track back gesture drags.
  final bool linearTransition;

  /// The widget below this widget in the tree.
  final Widget child;

  @override
  State<CupertinoFullscreenDialogTransition> createState() =>
      _CupertinoFullscreenDialogTransitionState();
}

class _CupertinoFullscreenDialogTransitionState extends State<CupertinoFullscreenDialogTransition> {
  /// When this page is coming in to cover another page.
  late Animation<Offset> _primaryPositionAnimation;

  /// When this page is becoming covered by another page.
  late Animation<Offset> _secondaryPositionAnimation;

  /// Curve of primary page which is coming in to cover another page.
  CurvedAnimation? _primaryPositionCurve;

  /// Curve of secondary page which is becoming covered by another page.
  CurvedAnimation? _secondaryPositionCurve;

  @override
  void initState() {
    super.initState();
    _setupAnimation();
  }

  @override
  void didUpdateWidget(covariant CupertinoFullscreenDialogTransition oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.primaryRouteAnimation != widget.primaryRouteAnimation ||
        oldWidget.secondaryRouteAnimation != widget.secondaryRouteAnimation ||
        oldWidget.linearTransition != widget.linearTransition) {
      _disposeCurve();
      _setupAnimation();
    }
  }

  @override
  void dispose() {
    _disposeCurve();
    super.dispose();
  }

  void _disposeCurve() {
    _primaryPositionCurve?.dispose();
    _secondaryPositionCurve?.dispose();
    _primaryPositionCurve = null;
    _secondaryPositionCurve = null;
  }

  void _setupAnimation() {
    _primaryPositionAnimation = (_primaryPositionCurve = CurvedAnimation(
          parent: widget.primaryRouteAnimation,
          curve: Curves.linearToEaseOut,
          // The curve must be flipped so that the reverse animation doesn't play
          // an ease-in curve, which iOS does not use.
          reverseCurve: Curves.linearToEaseOut.flipped,
        ))
        .drive(_kBottomUpTween);
    _secondaryPositionAnimation = (widget.linearTransition
            ? widget.secondaryRouteAnimation
            : _secondaryPositionCurve = CurvedAnimation(
              parent: widget.secondaryRouteAnimation,
              curve: Curves.linearToEaseOut,
              reverseCurve: Curves.easeInToLinear,
            ))
        .drive(_kMiddleLeftTween);
  }

  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasDirectionality(context));
    final TextDirection textDirection = Directionality.of(context);
    return SlideTransition(
      position: _secondaryPositionAnimation,
      textDirection: textDirection,
      transformHitTests: false,
      child: SlideTransition(position: _primaryPositionAnimation, child: widget.child),
    );
  }
}

/// This is the widget side of [_CupertinoBackGestureController].
///
/// This widget provides a gesture recognizer which, when it determines the
/// route can be closed with a back gesture, creates the controller and
/// feeds it the input from the gesture recognizer.
///
/// The gesture data is converted from absolute coordinates to logical
/// coordinates by this widget.
///
/// The type `T` specifies the return type of the route with which this gesture
/// detector is associated.
class _CupertinoBackGestureDetector<T> extends StatefulWidget {
  const _CupertinoBackGestureDetector({
    super.key,
    required this.enabledCallback,
    required this.onStartPopGesture,
    required this.child,
  });

  final Widget child;

  final ValueGetter<bool> enabledCallback;

  final ValueGetter<_CupertinoBackGestureController<T>> onStartPopGesture;

  @override
  _CupertinoBackGestureDetectorState<T> createState() => _CupertinoBackGestureDetectorState<T>();
}

class _CupertinoBackGestureDetectorState<T> extends State<_CupertinoBackGestureDetector<T>> {
  _CupertinoBackGestureController<T>? _backGestureController;

  late HorizontalDragGestureRecognizer _recognizer;

  @override
  void initState() {
    super.initState();
    _recognizer =
        HorizontalDragGestureRecognizer(debugOwner: this)
          ..onStart = _handleDragStart
          ..onUpdate = _handleDragUpdate
          ..onEnd = _handleDragEnd
          ..onCancel = _handleDragCancel;
  }

  @override
  void dispose() {
    _recognizer.dispose();

    // If this is disposed during a drag, call navigator.didStopUserGesture.
    if (_backGestureController != null) {
      WidgetsBinding.instance.addPostFrameCallback((_) {
        if (_backGestureController?.navigator.mounted ?? false) {
          _backGestureController?.navigator.didStopUserGesture();
        }
        _backGestureController = null;
      });
    }
    super.dispose();
  }

  void _handleDragStart(DragStartDetails details) {
    assert(mounted);
    assert(_backGestureController == null);
    _backGestureController = widget.onStartPopGesture();
  }

  void _handleDragUpdate(DragUpdateDetails details) {
    assert(mounted);
    assert(_backGestureController != null);
    _backGestureController!.dragUpdate(
      _convertToLogical(details.primaryDelta! / context.size!.width),
    );
  }

  void _handleDragEnd(DragEndDetails details) {
    assert(mounted);
    assert(_backGestureController != null);
    _backGestureController!.dragEnd(
      _convertToLogical(details.velocity.pixelsPerSecond.dx / context.size!.width),
    );
    _backGestureController = null;
  }

  void _handleDragCancel() {
    assert(mounted);
    // This can be called even if start is not called, paired with the "down" event
    // that we don't consider here.
    _backGestureController?.dragEnd(0.0);
    _backGestureController = null;
  }

  void _handlePointerDown(PointerDownEvent event) {
    if (widget.enabledCallback()) {
      _recognizer.addPointer(event);
    }
  }

  double _convertToLogical(double value) {
    return switch (Directionality.of(context)) {
      TextDirection.rtl => -value,
      TextDirection.ltr => value,
    };
  }

  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasDirectionality(context));
    // For devices with notches, the drag area needs to be larger on the side
    // that has the notch.
    final double dragAreaWidth = switch (Directionality.of(context)) {
      TextDirection.rtl => MediaQuery.paddingOf(context).right,
      TextDirection.ltr => MediaQuery.paddingOf(context).left,
    };
    return Stack(
      fit: StackFit.passthrough,
      children: <Widget>[
        widget.child,
        PositionedDirectional(
          start: 0.0,
          width: max(dragAreaWidth, _kBackGestureWidth),
          top: 0.0,
          bottom: 0.0,
          child: Listener(onPointerDown: _handlePointerDown, behavior: HitTestBehavior.translucent),
        ),
      ],
    );
  }
}

/// A controller for an iOS-style back gesture.
///
/// This is created by a [CupertinoPageRoute] in response from a gesture caught
/// by a [_CupertinoBackGestureDetector] widget, which then also feeds it input
/// from the gesture. It controls the animation controller owned by the route,
/// based on the input provided by the gesture detector.
///
/// This class works entirely in logical coordinates (0.0 is new page dismissed,
/// 1.0 is new page on top).
///
/// The type `T` specifies the return type of the route with which this gesture
/// detector controller is associated.
class _CupertinoBackGestureController<T> {
  /// Creates a controller for an iOS-style back gesture.
  _CupertinoBackGestureController({
    required this.navigator,
    required this.controller,
    required this.getIsActive,
    required this.getIsCurrent,
  }) {
    navigator.didStartUserGesture();
  }

  final AnimationController controller;
  final NavigatorState navigator;
  final ValueGetter<bool> getIsActive;
  final ValueGetter<bool> getIsCurrent;

  /// The drag gesture has changed by [delta]. The total range of the drag
  /// should be 0.0 to 1.0.
  void dragUpdate(double delta) {
    controller.value -= delta;
  }

  /// The drag gesture has ended with a horizontal motion of [velocity] as a
  /// fraction of screen width per second.
  void dragEnd(double velocity) {
    // Fling in the appropriate direction.
    //
    // This curve has been determined through rigorously eyeballing native iOS
    // animations.
    const Curve animationCurve = Curves.fastEaseInToSlowEaseOut;
    final bool isCurrent = getIsCurrent();
    final bool animateForward;

    if (!isCurrent) {
      // If the page has already been navigated away from, then the animation
      // direction depends on whether or not it's still in the navigation stack,
      // regardless of velocity or drag position. For example, if a route is
      // being slowly dragged back by just a few pixels, but then a programmatic
      // pop occurs, the route should still be animated off the screen.
      // See https://github.com/flutter/flutter/issues/141268.
      animateForward = getIsActive();
    } else if (velocity.abs() >= _kMinFlingVelocity) {
      // If the user releases the page before mid screen with sufficient velocity,
      // or after mid screen, we should animate the page out. Otherwise, the page
      // should be animated back in.
      animateForward = velocity <= 0;
    } else {
      animateForward = controller.value > 0.5;
    }

    if (animateForward) {
      controller.animateTo(
        1.0,
        duration: _kDroppedSwipePageAnimationDuration,
        curve: animationCurve,
      );
    } else {
      if (isCurrent) {
        // This route is destined to pop at this point. Reuse navigator's pop.
        navigator.pop();
      }

      // The popping may have finished inline if already at the target destination.
      if (controller.isAnimating) {
        controller.animateBack(
          0.0,
          duration: _kDroppedSwipePageAnimationDuration,
          curve: animationCurve,
        );
      }
    }

    if (controller.isAnimating) {
      // Keep the userGestureInProgress in true state so we don't change the
      // curve of the page transition mid-flight since CupertinoPageTransition
      // depends on userGestureInProgress.
      late AnimationStatusListener animationStatusCallback;
      animationStatusCallback = (AnimationStatus status) {
        navigator.didStopUserGesture();
        controller.removeStatusListener(animationStatusCallback);
      };
      controller.addStatusListener(animationStatusCallback);
    } else {
      navigator.didStopUserGesture();
    }
  }
}

// A custom [Decoration] used to paint an extra shadow on the start edge of the
// box it's decorating. It's like a [BoxDecoration] with only a gradient except
// it paints on the start side of the box instead of behind the box.
class _CupertinoEdgeShadowDecoration extends Decoration {
  const _CupertinoEdgeShadowDecoration._([this._colors]);

  static DecorationTween kTween = DecorationTween(
    begin: const _CupertinoEdgeShadowDecoration._(), // No decoration initially.
    end: const _CupertinoEdgeShadowDecoration._(
      // Eyeballed gradient used to mimic a drop shadow on the start side only.
      <Color>[Color(0x04000000), CupertinoColors.transparent],
    ),
  );

  // Colors used to paint a gradient at the start edge of the box it is
  // decorating.
  //
  // The first color in the list is used at the start of the gradient, which
  // is located at the start edge of the decorated box.
  //
  // If this is null, no shadow is drawn.
  //
  // The list must have at least two colors in it (otherwise it would not be a
  // gradient).
  final List<Color>? _colors;

  // Linearly interpolate between two edge shadow decorations decorations.
  //
  // The `t` argument represents position on the timeline, with 0.0 meaning
  // that the interpolation has not started, returning `a` (or something
  // equivalent to `a`), 1.0 meaning that the interpolation has finished,
  // returning `b` (or something equivalent to `b`), and values in between
  // meaning that the interpolation is at the relevant point on the timeline
  // between `a` and `b`. The interpolation can be extrapolated beyond 0.0 and
  // 1.0, so negative values and values greater than 1.0 are valid (and can
  // easily be generated by curves such as [Curves.elasticInOut]).
  //
  // Values for `t` are usually obtained from an [Animation<double>], such as
  // an [AnimationController].
  //
  // See also:
  //
  //  * [Decoration.lerp].
  static _CupertinoEdgeShadowDecoration? lerp(
    _CupertinoEdgeShadowDecoration? a,
    _CupertinoEdgeShadowDecoration? b,
    double t,
  ) {
    if (identical(a, b)) {
      return a;
    }
    if (a == null) {
      return b!._colors == null
          ? b
          : _CupertinoEdgeShadowDecoration._(
            b._colors!.map<Color>((Color color) => Color.lerp(null, color, t)!).toList(),
          );
    }
    if (b == null) {
      return a._colors == null
          ? a
          : _CupertinoEdgeShadowDecoration._(
            a._colors.map<Color>((Color color) => Color.lerp(null, color, 1.0 - t)!).toList(),
          );
    }
    assert(b._colors != null || a._colors != null);
    // If it ever becomes necessary, we could allow decorations with different
    // length' here, similarly to how it is handled in [LinearGradient.lerp].
    assert(b._colors == null || a._colors == null || a._colors.length == b._colors.length);
    return _CupertinoEdgeShadowDecoration._(<Color>[
      for (int i = 0; i < b._colors!.length; i += 1) Color.lerp(a._colors?[i], b._colors[i], t)!,
    ]);
  }

  @override
  _CupertinoEdgeShadowDecoration lerpFrom(Decoration? a, double t) {
    if (a is _CupertinoEdgeShadowDecoration) {
      return _CupertinoEdgeShadowDecoration.lerp(a, this, t)!;
    }
    return _CupertinoEdgeShadowDecoration.lerp(null, this, t)!;
  }

  @override
  _CupertinoEdgeShadowDecoration lerpTo(Decoration? b, double t) {
    if (b is _CupertinoEdgeShadowDecoration) {
      return _CupertinoEdgeShadowDecoration.lerp(this, b, t)!;
    }
    return _CupertinoEdgeShadowDecoration.lerp(this, null, t)!;
  }

  @override
  _CupertinoEdgeShadowPainter createBoxPainter([VoidCallback? onChanged]) {
    return _CupertinoEdgeShadowPainter(this, onChanged);
  }

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is _CupertinoEdgeShadowDecoration && other._colors == _colors;
  }

  @override
  int get hashCode => _colors.hashCode;

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(IterableProperty<Color>('colors', _colors));
  }
}

/// A [BoxPainter] used to draw the page transition shadow using gradients.
class _CupertinoEdgeShadowPainter extends BoxPainter {
  _CupertinoEdgeShadowPainter(this._decoration, super.onChanged)
    : assert(_decoration._colors == null || _decoration._colors.length > 1);

  final _CupertinoEdgeShadowDecoration _decoration;

  @override
  void paint(Canvas canvas, Offset offset, ImageConfiguration configuration) {
    final List<Color>? colors = _decoration._colors;
    if (colors == null) {
      return;
    }

    // The following code simulates drawing a [LinearGradient] configured as
    // follows:
    //
    // LinearGradient(
    //   begin: AlignmentDirectional(0.90, 0.0), // Spans 5% of the page.
    //   colors: _decoration._colors,
    // )
    //
    // A performance evaluation on Feb 8, 2021 showed, that drawing the gradient
    // manually as implemented below is more performant than relying on
    // [LinearGradient.createShader] because compiling that shader takes a long
    // time. On an iPhone XR, the implementation below reduced the worst frame
    // time for a cupertino page transition of a newly installed app from ~95ms
    // down to ~30ms, mainly because there's no longer a need to compile a
    // shader for the LinearGradient.
    //
    // The implementation below divides the width of the shadow into multiple
    // bands of equal width, one for each color interval defined by
    // `_decoration._colors`. Band x is filled with a gradient going from
    // `_decoration._colors[x]` to `_decoration._colors[x + 1]` by drawing a
    // bunch of 1px wide rects. The rects change their color by lerping between
    // the two colors that define the interval of the band.

    // Shadow spans 5% of the page.
    final double shadowWidth = 0.05 * configuration.size!.width;
    final double shadowHeight = configuration.size!.height;
    final double bandWidth = shadowWidth / (colors.length - 1);

    final TextDirection? textDirection = configuration.textDirection;
    assert(textDirection != null);
    final (double shadowDirection, double start) = switch (textDirection!) {
      TextDirection.rtl => (1, offset.dx + configuration.size!.width),
      TextDirection.ltr => (-1, offset.dx),
    };

    int bandColorIndex = 0;
    for (int dx = 0; dx < shadowWidth; dx += 1) {
      if (dx ~/ bandWidth != bandColorIndex) {
        bandColorIndex += 1;
      }
      final Paint paint =
          Paint()
            ..color =
                Color.lerp(
                  colors[bandColorIndex],
                  colors[bandColorIndex + 1],
                  (dx % bandWidth) / bandWidth,
                )!;
      final double x = start + shadowDirection * dx;
      canvas.drawRect(Rect.fromLTWH(x - 1.0, offset.dy, 1.0, shadowHeight), paint);
    }
  }
}

// The stiffness used by dialogs and action sheets.
//
// The stiffness value is obtained by examining the properties of
// `CASpringAnimation` in Xcode. The damping value is derived similarly, with
// additional precision calculated based on `_kStandardStiffness` to ensure a
// damping ratio of 1 (critically damped): damping = 2 * sqrt(stiffness)
const double _kStandardStiffness = 522.35;
const double _kStandardDamping = 45.7099552;
const SpringDescription _kStandardSpring = SpringDescription(
  mass: 1,
  stiffness: _kStandardStiffness,
  damping: _kStandardDamping,
);
// The iOS spring animation duration is 0.404 seconds, based on the properties
// of `CASpringAnimation` in Xcode. At this point, the spring's position
// `x(0.404)` is approximately 0.9990000, suggesting that iOS uses a position
// tolerance of 1e-3 (matching the default `_epsilonDefault` value).
//
// However, the spring's velocity `dx(0.404)` is about 0.02, indicating that iOS
// may not consider velocity when determining the animation's end condition. To
// account for this, a larger velocity tolerance is applied here for added
// safety.
const Tolerance _kStandardTolerance = Tolerance(velocity: 0.03);

/// A route that shows a modal iOS-style popup that slides up from the
/// bottom of the screen.
///
/// Such a popup is an alternative to a menu or a dialog and prevents the user
/// from interacting with the rest of the app.
///
/// It is used internally by [showCupertinoModalPopup] or can be directly pushed
/// onto the [Navigator] stack to enable state restoration. See
/// [showCupertinoModalPopup] for a state restoration app example.
///
/// The `barrierColor` argument determines the [Color] of the barrier underneath
/// the popup. When unspecified, the barrier color defaults to a light opacity
/// black scrim based on iOS's dialog screens. To correctly have iOS resolve
/// to the appropriate modal colors, pass in
/// `CupertinoDynamicColor.resolve(kCupertinoModalBarrierColor, context)`.
///
/// The `barrierDismissible` argument determines whether clicking outside the
/// popup results in dismissal. It is `true` by default.
///
/// The `semanticsDismissible` argument is used to determine whether the
/// semantics of the modal barrier are included in the semantics tree.
///
/// The `routeSettings` argument is used to provide [RouteSettings] to the
/// created Route.
///
/// {@macro flutter.widgets.RawDialogRoute}
///
/// See also:
///
///  * [DisplayFeatureSubScreen], which documents the specifics of how
///    [DisplayFeature]s can split the screen into sub-screens.
///  * [CupertinoActionSheet], which is the widget usually returned by the
///    `builder` argument.
///  * <https://developer.apple.com/design/human-interface-guidelines/ios/views/action-sheets/>
class CupertinoModalPopupRoute<T> extends PopupRoute<T> {
  /// A route that shows a modal iOS-style popup that slides up from the
  /// bottom of the screen.
  CupertinoModalPopupRoute({
    required this.builder,
    this.barrierLabel = 'Dismiss',
    this.barrierColor = kCupertinoModalBarrierColor,
    bool barrierDismissible = true,
    bool semanticsDismissible = false,
    super.filter,
    super.settings,
    super.requestFocus,
    this.anchorPoint,
  }) : _barrierDismissible = barrierDismissible,
       _semanticsDismissible = semanticsDismissible;

  /// A builder that builds the widget tree for the [CupertinoModalPopupRoute].
  ///
  /// The [builder] argument typically builds a [CupertinoActionSheet] widget.
  ///
  /// Content below the widget is dimmed with a [ModalBarrier]. The widget built
  /// by the [builder] does not share a context with the route it was originally
  /// built from. Use a [StatefulBuilder] or a custom [StatefulWidget] if the
  /// widget needs to update dynamically.
  final WidgetBuilder builder;

  final bool _barrierDismissible;

  final bool _semanticsDismissible;

  @override
  final String barrierLabel;

  @override
  final Color? barrierColor;

  @override
  bool get barrierDismissible => _barrierDismissible;

  @override
  bool get semanticsDismissible => _semanticsDismissible;

  @override
  Duration get transitionDuration => _kModalPopupTransitionDuration;

  /// {@macro flutter.widgets.DisplayFeatureSubScreen.anchorPoint}
  final Offset? anchorPoint;

  @override
  Simulation createSimulation({required bool forward}) {
    assert(!debugTransitionCompleted(), 'Cannot reuse a $runtimeType after disposing it.');
    final double end = forward ? 1.0 : 0.0;
    return SpringSimulation(
      _kStandardSpring,
      controller!.value,
      end,
      0,
      tolerance: _kStandardTolerance,
      snapToEnd: true,
    );
  }

  @override
  Widget buildPage(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
  ) {
    return CupertinoUserInterfaceLevel(
      data: CupertinoUserInterfaceLevelData.elevated,
      child: DisplayFeatureSubScreen(anchorPoint: anchorPoint, child: Builder(builder: builder)),
    );
  }

  @override
  Widget buildTransitions(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
    Widget child,
  ) {
    return Align(
      alignment: Alignment.bottomCenter,
      child: FractionalTranslation(translation: _offsetTween.evaluate(animation), child: child),
    );
  }

  static final Tween<Offset> _offsetTween = Tween<Offset>(
    begin: const Offset(0.0, 1.0),
    end: Offset.zero,
  );
}

/// Shows a modal iOS-style popup that slides up from the bottom of the screen.
///
/// Such a popup is an alternative to a menu or a dialog and prevents the user
/// from interacting with the rest of the app.
///
/// The `context` argument is used to look up the [Navigator] for the popup.
/// It is only used when the method is called. Its corresponding widget can be
/// safely removed from the tree before the popup is closed.
///
/// The `barrierColor` argument determines the [Color] of the barrier underneath
/// the popup. When unspecified, the barrier color defaults to a light opacity
/// black scrim based on iOS's dialog screens.
///
/// The `barrierDismissible` argument determines whether clicking outside the
/// popup results in dismissal. It is `true` by default.
///
/// The `useRootNavigator` argument is used to determine whether to push the
/// popup to the [Navigator] furthest from or nearest to the given `context`. It
/// is `true` by default.
///
/// The `semanticsDismissible` argument is used to determine whether the
/// semantics of the modal barrier are included in the semantics tree.
///
/// The `routeSettings` argument is used to provide [RouteSettings] to the
/// created Route.
///
/// The `builder` argument typically builds a [CupertinoActionSheet] widget.
/// Content below the widget is dimmed with a [ModalBarrier]. The widget built
/// by the `builder` does not share a context with the location that
/// [showCupertinoModalPopup] is originally called from. Use a
/// [StatefulBuilder] or a custom [StatefulWidget] if the widget needs to
/// update dynamically.
///
/// {@macro flutter.widgets.RawDialogRoute}
///
/// Returns a `Future` that resolves to the value that was passed to
/// [Navigator.pop] when the popup was closed.
///
/// ### State Restoration in Modals
///
/// Using this method will not enable state restoration for the modal. In order
/// to enable state restoration for a modal, use [Navigator.restorablePush]
/// or [Navigator.restorablePushNamed] with [CupertinoModalPopupRoute].
///
/// For more information about state restoration, see [RestorationManager].
///
/// {@tool dartpad}
/// This sample demonstrates how to create a restorable Cupertino modal route.
/// This is accomplished by enabling state restoration by specifying
/// [CupertinoApp.restorationScopeId] and using [Navigator.restorablePush] to
/// push [CupertinoModalPopupRoute] when the [CupertinoButton] is tapped.
///
/// {@macro flutter.widgets.RestorationManager}
///
/// ** See code in examples/api/lib/cupertino/route/show_cupertino_modal_popup.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [DisplayFeatureSubScreen], which documents the specifics of how
///    [DisplayFeature]s can split the screen into sub-screens.
///  * [CupertinoActionSheet], which is the widget usually returned by the
///    `builder` argument to [showCupertinoModalPopup].
///  * <https://developer.apple.com/design/human-interface-guidelines/ios/views/action-sheets/>
Future<T?> showCupertinoModalPopup<T>({
  required BuildContext context,
  required WidgetBuilder builder,
  ImageFilter? filter,
  Color barrierColor = kCupertinoModalBarrierColor,
  bool barrierDismissible = true,
  bool useRootNavigator = true,
  bool semanticsDismissible = false,
  RouteSettings? routeSettings,
  Offset? anchorPoint,
}) {
  return Navigator.of(context, rootNavigator: useRootNavigator).push(
    CupertinoModalPopupRoute<T>(
      builder: builder,
      filter: filter,
      barrierColor: CupertinoDynamicColor.resolve(barrierColor, context),
      barrierDismissible: barrierDismissible,
      semanticsDismissible: semanticsDismissible,
      settings: routeSettings,
      anchorPoint: anchorPoint,
    ),
  );
}

Widget _buildCupertinoDialogTransitions(
  BuildContext context,
  Animation<double> animation,
  Animation<double> secondaryAnimation,
  Widget child,
) {
  return child;
}

/// Displays an iOS-style dialog above the current contents of the app, with
/// iOS-style entrance and exit animations, modal barrier color, and modal
/// barrier behavior (by default, the dialog is not dismissible with a tap on
/// the barrier).
///
/// This function takes a `builder` which typically builds a [CupertinoAlertDialog]
/// widget. Content below the dialog is dimmed with a [ModalBarrier]. The widget
/// returned by the `builder` does not share a context with the location that
/// [showCupertinoDialog] is originally called from. Use a [StatefulBuilder] or
/// a custom [StatefulWidget] if the dialog needs to update dynamically.
///
/// The `context` argument is used to look up the [Navigator] for the dialog.
/// It is only used when the method is called. Its corresponding widget can
/// be safely removed from the tree before the dialog is closed.
///
/// The `useRootNavigator` argument is used to determine whether to push the
/// dialog to the [Navigator] furthest from or nearest to the given `context`.
/// By default, `useRootNavigator` is `true` and the dialog route created by
/// this method is pushed to the root navigator.
///
/// {@macro flutter.widgets.RawDialogRoute}
///
/// If the application has multiple [Navigator] objects, it may be necessary to
/// call `Navigator.of(context, rootNavigator: true).pop(result)` to close the
/// dialog rather than just `Navigator.pop(context, result)`.
///
/// Returns a [Future] that resolves to the value (if any) that was passed to
/// [Navigator.pop] when the dialog was closed.
///
/// ### State Restoration in Dialogs
///
/// Using this method will not enable state restoration for the dialog. In order
/// to enable state restoration for a dialog, use [Navigator.restorablePush]
/// or [Navigator.restorablePushNamed] with [CupertinoDialogRoute].
///
/// For more information about state restoration, see [RestorationManager].
///
/// {@tool dartpad}
/// This sample demonstrates how to create a restorable Cupertino dialog. This is
/// accomplished by enabling state restoration by specifying
/// [CupertinoApp.restorationScopeId] and using [Navigator.restorablePush] to
/// push [CupertinoDialogRoute] when the [CupertinoButton] is tapped.
///
/// {@macro flutter.widgets.RestorationManager}
///
/// ** See code in examples/api/lib/cupertino/route/show_cupertino_dialog.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [CupertinoAlertDialog], an iOS-style alert dialog.
///  * [showDialog], which displays a Material-style dialog.
///  * [showGeneralDialog], which allows for customization of the dialog popup.
///  * [DisplayFeatureSubScreen], which documents the specifics of how
///    [DisplayFeature]s can split the screen into sub-screens.
///  * <https://developer.apple.com/design/human-interface-guidelines/alerts/>
Future<T?> showCupertinoDialog<T>({
  required BuildContext context,
  required WidgetBuilder builder,
  String? barrierLabel,
  bool useRootNavigator = true,
  bool barrierDismissible = false,
  RouteSettings? routeSettings,
  Offset? anchorPoint,
}) {
  return Navigator.of(context, rootNavigator: useRootNavigator).push<T>(
    CupertinoDialogRoute<T>(
      builder: builder,
      context: context,
      barrierDismissible: barrierDismissible,
      barrierLabel: barrierLabel,
      barrierColor: CupertinoDynamicColor.resolve(kCupertinoModalBarrierColor, context),
      settings: routeSettings,
      anchorPoint: anchorPoint,
    ),
  );
}

/// A dialog route that shows an iOS-style dialog.
///
/// It is used internally by [showCupertinoDialog] or can be directly pushed
/// onto the [Navigator] stack to enable state restoration. See
/// [showCupertinoDialog] for a state restoration app example.
///
/// This function takes a `builder` which typically builds a [Dialog] widget.
/// Content below the dialog is dimmed with a [ModalBarrier]. The widget
/// returned by the `builder` does not share a context with the location that
/// `showDialog` is originally called from. Use a [StatefulBuilder] or a
/// custom [StatefulWidget] if the dialog needs to update dynamically.
///
/// The `context` argument is used to look up
/// [CupertinoLocalizations.modalBarrierDismissLabel], which provides the
/// modal with a localized accessibility label that will be used for the
/// modal's barrier. However, a custom `barrierLabel` can be passed in as well.
///
/// The `barrierDismissible` argument is used to indicate whether tapping on the
/// barrier will dismiss the dialog. It is `true` by default and cannot be `null`.
///
/// The `barrierColor` argument is used to specify the color of the modal
/// barrier that darkens everything below the dialog. If `null`, then
/// [CupertinoDynamicColor.resolve] is used to compute the modal color.
///
/// The `settings` argument define the settings for this route. See
/// [RouteSettings] for details.
///
/// {@macro flutter.widgets.RawDialogRoute}
///
/// See also:
///
///  * [showCupertinoDialog], which is a way to display
///     an iOS-style dialog.
///  * [showGeneralDialog], which allows for customization of the dialog popup.
///  * [showDialog], which displays a Material dialog.
///  * [DisplayFeatureSubScreen], which documents the specifics of how
///    [DisplayFeature]s can split the screen into sub-screens.
class CupertinoDialogRoute<T> extends RawDialogRoute<T> {
  /// A dialog route that shows an iOS-style dialog.
  CupertinoDialogRoute({
    required WidgetBuilder builder,
    required BuildContext context,
    super.barrierDismissible,
    Color? barrierColor,
    String? barrierLabel,
    // This transition duration was eyeballed comparing with iOS
    super.transitionDuration = const Duration(milliseconds: 250),
    this.transitionBuilder,
    super.settings,
    super.requestFocus,
    super.anchorPoint,
  }) : super(
         pageBuilder: (
           BuildContext context,
           Animation<double> animation,
           Animation<double> secondaryAnimation,
         ) {
           return builder(context);
         },
         transitionBuilder: transitionBuilder ?? _buildCupertinoDialogTransitions,
         barrierLabel: barrierLabel ?? CupertinoLocalizations.of(context).modalBarrierDismissLabel,
         barrierColor:
             barrierColor ?? CupertinoDynamicColor.resolve(kCupertinoModalBarrierColor, context),
       );

  /// Custom transition builder
  RouteTransitionsBuilder? transitionBuilder;

  CurvedAnimation? _fadeAnimation;

  @override
  Simulation createSimulation({required bool forward}) {
    assert(!debugTransitionCompleted(), 'Cannot reuse a $runtimeType after disposing it.');
    final double end = forward ? 1.0 : 0.0;
    return SpringSimulation(
      _kStandardSpring,
      controller!.value,
      end,
      0,
      tolerance: _kStandardTolerance,
      snapToEnd: true,
    );
  }

  @override
  Widget buildTransitions(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
    Widget child,
  ) {
    if (transitionBuilder != null) {
      return super.buildTransitions(context, animation, secondaryAnimation, child);
    }

    if (animation.status == AnimationStatus.reverse) {
      return FadeTransition(opacity: animation, child: child);
    }
    return FadeTransition(
      opacity: animation,
      child: ScaleTransition(scale: animation.drive(_dialogScaleTween), child: child),
    );
  }

  @override
  void dispose() {
    _fadeAnimation?.dispose();
    super.dispose();
  }

  // The curve and initial scale values were mostly eyeballed from iOS, however
  // they reuse the same animation curve that was modeled after native page
  // transitions.
  static final Tween<double> _dialogScaleTween = Tween<double>(begin: 1.3, end: 1.0);
}
