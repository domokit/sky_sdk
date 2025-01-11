// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'page.dart';
library;

import 'dart:ui' as ui;

import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';

import 'page_transitions_theme.dart';

/// Used by [PageTransitionsTheme] to define a [MaterialPageRoute] page
/// transition animation that looks like the default page transition used on
/// Android U and above when using predictive back.
///
/// Currently predictive back is only supported on Android U and above, and if
/// this [PageTransitionsBuilder] is used by any other platform, it will fall
/// back to [ZoomPageTransitionsBuilder].
///
/// When used on Android U and above, animates along with the back gesture to
/// reveal the destination route. Can be canceled by dragging back towards the
/// edge of the screen.
///
/// See also:
///
///  * [FadeUpwardsPageTransitionsBuilder], which defines a page transition
///    that's similar to the one provided by Android O.
///  * [OpenUpwardsPageTransitionsBuilder], which defines a page transition
///    that's similar to the one provided by Android P.
///  * [ZoomPageTransitionsBuilder], which defines the default page transition
///    that's similar to the one provided in Android Q.
///  * [CupertinoPageTransitionsBuilder], which defines a horizontal page
///    transition that matches native iOS page transitions.
class PredictiveBackPageTransitionsBuilder extends PageTransitionsBuilder {
  /// Creates an instance of a [PageTransitionsBuilder] that matches Android U's
  /// predictive back transition.
  const PredictiveBackPageTransitionsBuilder();

  @override
  Widget buildTransitions<T>(
      PageRoute<T> route,
      BuildContext context,
      Animation<double> animation,
      Animation<double> secondaryAnimation,
      Widget child,
      ) {
    return _PredictiveBackGestureDetector(
      route: route,
      builder: (BuildContext context, _, __, ___) {
        // Only do a predictive back transition when the user is performing a
        // pop gesture. Otherwise, for things like button presses or other
        // programmatic navigation, fall back to ZoomPageTransitionsBuilder.
        if (route.popGestureInProgress) {
          return _PredictiveBackPageFullScreenTransition(
            animation: animation,
            secondaryAnimation: secondaryAnimation,
            getIsCurrent: () => route.isCurrent,
            child: child,
          );
        }

        return const ZoomPageTransitionsBuilder().buildTransitions(
          route,
          context,
          animation,
          secondaryAnimation,
          child,
        );
      },
    );
  }
}

// TODO(maRci002): add docs
class PredictiveBackPageSharedElementTransitionsBuilder extends PageTransitionsBuilder {
  /// Creates an instance of a [PageTransitionsBuilder] that matches Android U's
  /// predictive back transition.
  const PredictiveBackPageSharedElementTransitionsBuilder();

  @override
  DelegatedTransitionBuilder? get delegatedTransition {
    return _delegatedTransition;
  }

  static Widget? _delegatedTransition(BuildContext context, Animation<double> animation, Animation<double> secondaryAnimation, bool allowSnapshotting, Widget? child) {
    final ModalRoute<Object?>? route = ModalRoute.of(context);
    if (child == null || route is! PageRoute) {
      return child;
    }

    return _PredictiveBackGestureListener(
      route: route,
      builder: (BuildContext context, _PredictiveBackPhase phase, PredictiveBackEvent? startBackEvent, PredictiveBackEvent? currentBackEvent) {
        // Only do a predictive back transition when the user is performing a
        // pop gesture. Otherwise, for things like button presses or other
        // programmatic navigation, fall back to ZoomPageTransitionsBuilder.
        if (route.popGestureInProgress) {
          return _PredictiveBackPageSharedElementTransition(
            isDelegatedTransition: true,
            animation: animation,
            phase: phase,
            secondaryAnimation: secondaryAnimation,
            startBackEvent: startBackEvent,
            currentBackEvent: currentBackEvent,
            child: child,
          );
        }

        return child;
      },
    );
  }

  @override
  Widget buildTransitions<T>(
    PageRoute<T> route,
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
    Widget child,
  ) {
    return _PredictiveBackGestureDetector(
      route: route,
      builder: (BuildContext context, _PredictiveBackPhase phase, PredictiveBackEvent? startBackEvent, PredictiveBackEvent? currentBackEvent) {
        // Only do a predictive back transition when the user is performing a
        // pop gesture. Otherwise, for things like button presses or other
        // programmatic navigation, fall back to ZoomPageTransitionsBuilder.
        if (route.popGestureInProgress) {
          return _PredictiveBackPageSharedElementTransition(
            isDelegatedTransition: false,
            animation: animation,
            phase: phase,
            secondaryAnimation: secondaryAnimation,
            startBackEvent: startBackEvent,
            currentBackEvent: currentBackEvent,
            child: child,
          );
        }

        return const ZoomPageTransitionsBuilder().buildTransitions(
          route,
          context,
          animation,
          secondaryAnimation,
          child,
        );
      },
    );
  }
}

typedef _PredictiveBackGestureDetectorWidgetBuilder = Widget Function(
  BuildContext context,
  _PredictiveBackPhase phase,
  PredictiveBackEvent? startBackEvent,
  PredictiveBackEvent? currentBackEvent,
);

enum _PredictiveBackPhase { idle, start, update, commit, cancel }

class _PredictiveBackGestureListener extends StatefulWidget {
  const _PredictiveBackGestureListener({
    required this.builder,
    required this.route,
  });

  final _PredictiveBackGestureDetectorWidgetBuilder builder;
  final PageRoute<dynamic> route;

  @override
  State<_PredictiveBackGestureListener> createState() => _PredictiveBackGestureListenerState();
}

class _PredictiveBackGestureListenerState extends State<_PredictiveBackGestureListener>
    with WidgetsBindingObserver{
  _PredictiveBackPhase get phase => _phase;
  _PredictiveBackPhase _phase = _PredictiveBackPhase.idle;
  set phase(_PredictiveBackPhase phase) {
    if (_phase != phase && mounted) {
      setState(() => _phase = phase);
    }
  }

  /// The back event when the gesture first started.
  PredictiveBackEvent? get startBackEvent => _startBackEvent;
  PredictiveBackEvent? _startBackEvent;
  set startBackEvent(PredictiveBackEvent? startBackEvent) {
    if (_startBackEvent != startBackEvent && mounted) {
      setState(() => _startBackEvent = startBackEvent);
    }
  }

  /// The most recent back event during the gesture.
  PredictiveBackEvent? get currentBackEvent => _currentBackEvent;
  PredictiveBackEvent? _currentBackEvent;
  set currentBackEvent(PredictiveBackEvent? currentBackEvent) {
    if (_currentBackEvent != currentBackEvent && mounted) {
      setState(() => _currentBackEvent = currentBackEvent);
    }
  }

  // Begin WidgetsBindingObserver.

  @override
  bool handleStartBackGesture(PredictiveBackEvent backEvent) {
    phase = _PredictiveBackPhase.start;

    final bool gestureInProgress = !backEvent.isButtonEvent && widget.route.popGestureEnabled;
    if (!gestureInProgress) {
      return false;
    }

    startBackEvent = currentBackEvent = backEvent;
    return true;
  }

  @override
  void handleUpdateBackGestureProgress(PredictiveBackEvent backEvent) {
    phase = _PredictiveBackPhase.update;
    currentBackEvent = backEvent;
  }

  @override
  void handleCancelBackGesture() {
    phase = _PredictiveBackPhase.cancel;
    startBackEvent = currentBackEvent = null;
  }

  @override
  void handleCommitBackGesture() {
    phase = _PredictiveBackPhase.commit;
    startBackEvent = currentBackEvent = null;
  }

  // End WidgetsBindingObserver.

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addObserver(this);
  }

  @override
  void dispose() {
    WidgetsBinding.instance.removeObserver(this);
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final _PredictiveBackPhase effectivePhase = widget.route.popGestureInProgress ? phase : _PredictiveBackPhase.idle;
    return widget.builder(context, effectivePhase, startBackEvent, currentBackEvent);
  }
}

class _PredictiveBackGestureDetector extends StatefulWidget {
  const _PredictiveBackGestureDetector({required this.route, required this.builder});

  final _PredictiveBackGestureDetectorWidgetBuilder builder;
  final PageRoute<dynamic> route;

  @override
  State<_PredictiveBackGestureDetector> createState() => _PredictiveBackGestureDetectorState();
}

class _PredictiveBackGestureDetectorState extends State<_PredictiveBackGestureDetector>
    with WidgetsBindingObserver {
  /// True when the predictive back gesture is enabled.
  bool get _isEnabled {
    return widget.route.isCurrent && widget.route.popGestureEnabled;
  }

  _PredictiveBackPhase get phase => _phase;
  _PredictiveBackPhase _phase = _PredictiveBackPhase.idle;
  set phase(_PredictiveBackPhase phase) {
    if (_phase != phase && mounted) {
      setState(() => _phase = phase);
    }
  }

  /// The back event when the gesture first started.
  PredictiveBackEvent? get startBackEvent => _startBackEvent;
  PredictiveBackEvent? _startBackEvent;
  set startBackEvent(PredictiveBackEvent? startBackEvent) {
    if (_startBackEvent != startBackEvent && mounted) {
      setState(() => _startBackEvent = startBackEvent);
    }
  }

  /// The most recent back event during the gesture.
  PredictiveBackEvent? get currentBackEvent => _currentBackEvent;
  PredictiveBackEvent? _currentBackEvent;
  set currentBackEvent(PredictiveBackEvent? currentBackEvent) {
    if (_currentBackEvent != currentBackEvent && mounted) {
      setState(() => _currentBackEvent = currentBackEvent);
    }
  }

  // Begin WidgetsBindingObserver.

  @override
  bool handleStartBackGesture(PredictiveBackEvent backEvent) {
    phase = _PredictiveBackPhase.start;

    final bool gestureInProgress = !backEvent.isButtonEvent && _isEnabled;
    if (!gestureInProgress) {
      return false;
    }

    widget.route.handleStartBackGesture(progress: 1 - backEvent.progress);
    startBackEvent = currentBackEvent = backEvent;
    return true;
  }

  @override
  void handleUpdateBackGestureProgress(PredictiveBackEvent backEvent) {
    phase = _PredictiveBackPhase.update;

    widget.route.handleUpdateBackGestureProgress(progress: 1 - backEvent.progress);
    currentBackEvent = backEvent;
  }

  @override
  void handleCancelBackGesture() {
    phase = _PredictiveBackPhase.cancel;

    widget.route.handleCancelBackGesture();
    startBackEvent = currentBackEvent = null;
  }

  @override
  void handleCommitBackGesture() {
    phase = _PredictiveBackPhase.commit;

    widget.route.handleCommitBackGesture();
    startBackEvent = currentBackEvent = null;
  }

  // End WidgetsBindingObserver.

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addObserver(this);
  }

  @override
  void dispose() {
    WidgetsBinding.instance.removeObserver(this);
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final _PredictiveBackPhase effectivePhase = widget.route.popGestureInProgress ? phase : _PredictiveBackPhase.idle;
    return widget.builder(context, effectivePhase, startBackEvent, currentBackEvent);
  }
}

/// Android's predictive back page transition for full screen surfaces.
/// https://developer.android.com/design/ui/mobile/guides/patterns/predictive-back#full-screen-surfaces
class _PredictiveBackPageFullScreenTransition extends StatelessWidget {
  const _PredictiveBackPageFullScreenTransition({
    required this.animation,
    required this.secondaryAnimation,
    required this.getIsCurrent,
    required this.child,
  });

  // These values were eyeballed to match the native predictive back animation
  // on a Pixel 2 running Android API 34.
  static const double _scaleFullyOpened = 1.0;
  static const double _scaleStartTransition = 0.95;
  static const double _opacityFullyOpened = 1.0;
  static const double _opacityStartTransition = 0.95;
  static const double _weightForStartState = 65.0;
  static const double _weightForEndState = 35.0;
  static const double _screenWidthDivisionFactor = 20.0;
  static const double _xShiftAdjustment = 8.0;

  final Animation<double> animation;
  final Animation<double> secondaryAnimation;
  final ValueGetter<bool> getIsCurrent;
  final Widget child;

  Widget _secondaryAnimatedBuilder(BuildContext context, Widget? child) {
    final Size size = MediaQuery.sizeOf(context);
    final double screenWidth = size.width;
    final double xShift = (screenWidth / _screenWidthDivisionFactor) - _xShiftAdjustment;

    final bool isCurrent = getIsCurrent();
    final Tween<double> xShiftTween =
        isCurrent ? ConstantTween<double>(0) : Tween<double>(begin: xShift, end: 0);
    final Animatable<double> scaleTween =
        isCurrent
            ? ConstantTween<double>(_scaleFullyOpened)
            : TweenSequence<double>(<TweenSequenceItem<double>>[
              TweenSequenceItem<double>(
                tween: Tween<double>(begin: _scaleStartTransition, end: _scaleFullyOpened),
                weight: _weightForStartState,
              ),
              TweenSequenceItem<double>(
                tween: Tween<double>(begin: _scaleFullyOpened, end: _scaleFullyOpened),
                weight: _weightForEndState,
              ),
            ]);
    final Animatable<double> fadeTween =
        isCurrent
            ? ConstantTween<double>(_opacityFullyOpened)
            : TweenSequence<double>(<TweenSequenceItem<double>>[
              TweenSequenceItem<double>(
                tween: Tween<double>(begin: _opacityFullyOpened, end: _opacityStartTransition),
                weight: _weightForStartState,
              ),
              TweenSequenceItem<double>(
                tween: Tween<double>(begin: _opacityFullyOpened, end: _opacityFullyOpened),
                weight: _weightForEndState,
              ),
            ]);

    return Transform.translate(
      offset: Offset(xShiftTween.animate(secondaryAnimation).value, 0),
      child: Transform.scale(
        scale: scaleTween.animate(secondaryAnimation).value,
        child: Opacity(opacity: fadeTween.animate(secondaryAnimation).value, child: child),
      ),
    );
  }

  Widget _primaryAnimatedBuilder(BuildContext context, Widget? child) {
    final Size size = MediaQuery.sizeOf(context);
    final double screenWidth = size.width;
    final double xShift = (screenWidth / _screenWidthDivisionFactor) - _xShiftAdjustment;

    final Animatable<double> xShiftTween = TweenSequence<double>(<TweenSequenceItem<double>>[
      TweenSequenceItem<double>(
        tween: Tween<double>(begin: 0.0, end: 0.0),
        weight: _weightForStartState,
      ),
      TweenSequenceItem<double>(
        tween: Tween<double>(begin: xShift, end: 0.0),
        weight: _weightForEndState,
      ),
    ]);
    final Animatable<double> scaleTween = TweenSequence<double>(<TweenSequenceItem<double>>[
      TweenSequenceItem<double>(
        tween: Tween<double>(begin: _scaleFullyOpened, end: _scaleFullyOpened),
        weight: _weightForStartState,
      ),
      TweenSequenceItem<double>(
        tween: Tween<double>(begin: _scaleStartTransition, end: _scaleFullyOpened),
        weight: _weightForEndState,
      ),
    ]);
    final Animatable<double> fadeTween = TweenSequence<double>(<TweenSequenceItem<double>>[
      TweenSequenceItem<double>(
        tween: Tween<double>(begin: 0.0, end: 0.0),
        weight: _weightForStartState,
      ),
      TweenSequenceItem<double>(
        tween: Tween<double>(begin: _opacityStartTransition, end: _opacityFullyOpened),
        weight: _weightForEndState,
      ),
    ]);

    return Transform.translate(
      offset: Offset(xShiftTween.animate(animation).value, 0),
      child: Transform.scale(
        scale: scaleTween.animate(animation).value,
        child: Opacity(opacity: fadeTween.animate(animation).value, child: child),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: secondaryAnimation,
      builder: _secondaryAnimatedBuilder,
      child: AnimatedBuilder(animation: animation, builder: _primaryAnimatedBuilder, child: child),
    );
  }
}

/// Android's predictive back page shared element transition.
/// https://developer.android.com/design/ui/mobile/guides/patterns/predictive-back#shared-element-transition
class _PredictiveBackPageSharedElementTransition extends StatefulWidget {
  const _PredictiveBackPageSharedElementTransition({
    required this.isDelegatedTransition,
    required this.animation,
    required this.secondaryAnimation,
    required this.phase,
    required this.startBackEvent,
    required this.currentBackEvent,
    required this.child,
  });

  final bool isDelegatedTransition;
  final Animation<double> animation;
  final Animation<double> secondaryAnimation;
  final _PredictiveBackPhase phase;
  final PredictiveBackEvent? startBackEvent;
  final PredictiveBackEvent? currentBackEvent;
  final Widget child;

  @override
  State<_PredictiveBackPageSharedElementTransition> createState() =>
      _PredictiveBackPageSharedElementTransitionState();
}

class _PredictiveBackPageSharedElementTransitionState
    extends State<_PredictiveBackPageSharedElementTransition>
    with SingleTickerProviderStateMixin {
  double xShift = 0;
  double yShift = 0;
  double scale = 1;
  late final AnimationController commitController;
  late final Listenable mergedAnimations;

  // Constants as per the motion specs
  // https://developer.android.com/design/ui/mobile/guides/patterns/predictive-back#motion-specs
  static const double scalePercentage = 0.90;
  static const double divisionFactor = 20.0;
  static const double margin = 8.0;
  static const double borderRadius = 32.0;
  static const double extraShiftDistance = 0.1;

  @override
  void initState() {
    super.initState();
    commitController = AnimationController(
      duration: const Duration(milliseconds: 500),
      vsync: this,
    );

    mergedAnimations =
        Listenable.merge(<Listenable>[widget.animation, commitController]);

    if (widget.phase == _PredictiveBackPhase.commit) {
      commitController.forward(from: 0.0);
    }
  }

  @override
  void didUpdateWidget(_PredictiveBackPageSharedElementTransition oldWidget) {
    super.didUpdateWidget(oldWidget);

    if (widget.phase != oldWidget.phase &&
        widget.phase == _PredictiveBackPhase.commit) {
      final int droppedPageBackAnimationTime =
      ui.lerpDouble(0, 800, widget.animation.value)!.floor();

      commitController.duration = Duration(
        milliseconds: droppedPageBackAnimationTime,
      );
      commitController.forward(from: 0.0);
    }
  }

  @override
  void dispose() {
    commitController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: mergedAnimations,
      builder: _animatedBuilder,
      child: widget.child,
    );
  }

  double calcXShift() {
    final double screenWidth = MediaQuery.of(context).size.width;
    final double xShift = (screenWidth / divisionFactor) - margin;

    return Tween<double>(
      begin: widget.currentBackEvent?.swipeEdge == SwipeEdge.right
          ? -xShift
          : xShift,
      end: 0.0,
    ).animate(widget.animation).value;
  }

  double calcCommitXShift() {
    final double screenWidth = MediaQuery.of(context).size.width;
    return Tween<double>(begin: 0.0, end: screenWidth * extraShiftDistance)
        .animate(
      CurvedAnimation(
        parent: commitController,
        curve: const Interval(0.3, 1.0, curve: Curves.easeOut),
      ),
    )
        .value;
  }

  double calcYShift() {
    final double screenHeight = MediaQuery.of(context).size.height;

    final double startTouchY = widget.startBackEvent?.touchOffset?.dy ?? 0;
    final double currentTouchY = widget.currentBackEvent?.touchOffset?.dy ?? 0;

    final double yShiftMax = (screenHeight / divisionFactor) - margin;

    final double rawYShift = currentTouchY - startTouchY;
    final double easedYShift = Curves.easeOut
        .transform((rawYShift.abs() / screenHeight).clamp(0.0, 1.0)) *
        rawYShift.sign *
        yShiftMax;

    return easedYShift.clamp(-yShiftMax, yShiftMax);
  }

  double calcScale() {
    return Tween<double>(begin: scalePercentage, end: 1.0)
        .animate(widget.animation)
        .value;
  }

  double calcCommitScale() {
    return Tween<double>(begin: 0.0, end: extraShiftDistance)
        .animate(
      CurvedAnimation(
        parent: commitController,
        curve: const Interval(0.0, 0.3, curve: Curves.easeOut),
      ),
    )
        .value;
  }

  double calcOpacity() {
    if (widget.isDelegatedTransition) {
      return 1;
    }

    return Tween<double>(begin: 1.0, end: 0.0)
        .animate(
      CurvedAnimation(parent: commitController, curve: Curves.easeOut),
    )
        .value;
  }

  Widget _animatedBuilder(BuildContext context, Widget? child) {
    final double xShift = widget.phase == _PredictiveBackPhase.commit
        ? this.xShift + calcCommitXShift()
        : this.xShift = calcXShift();
    final double yShift = widget.phase == _PredictiveBackPhase.commit
        ? this.yShift
        : this.yShift = calcYShift();
    final double scale = widget.phase == _PredictiveBackPhase.commit
        ? this.scale - calcCommitScale()
        : this.scale = calcScale();

    final double opacity = calcOpacity();

    final Tween<double> gapTween = Tween<double>(begin: margin, end: 0.0);
    final Tween<double> borderRadiusTween = Tween<double>(
      begin: borderRadius,
      end: 0.0,
    );

    return Transform.scale(
      scale: scale,
      child: Transform.translate(
        offset: Offset(xShift, yShift),
        child: Opacity(
          opacity: opacity,
          child: Padding(
            padding: EdgeInsets.symmetric(
              horizontal: gapTween.animate(widget.animation).value,
            ),
            child: ClipRRect(
              borderRadius: BorderRadius.circular(
                borderRadiusTween.animate(widget.animation).value,
              ),
              child: child,
            ),
          ),
        ),
      ),
    );
  }
}
