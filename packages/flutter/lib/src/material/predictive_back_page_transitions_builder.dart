// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'page.dart';
library;

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
      animation: animation,
      secondaryAnimation: secondaryAnimation,
      child: child,
    );
  }
}

class _PredictiveBackGestureDetector extends StatefulWidget {
  const _PredictiveBackGestureDetector({
    required this.route,
    required this.animation,
    required this.secondaryAnimation,
    required this.child,
  });

  final PageRoute route;
  final Animation<double> animation;
  final Animation<double> secondaryAnimation;
  final Widget child;

  @override
  State<_PredictiveBackGestureDetector> createState() =>
      _PredictiveBackGestureDetectorState();
}

enum Phase {
  preCommit,
  postCommit,
}

class _PredictiveBackGestureDetectorState extends State<_PredictiveBackGestureDetector>
    with WidgetsBindingObserver {
  /// True when the predictive back gesture is enabled.
  bool get _isEnabled {
    return widget.route.isCurrent
        && widget.route.popGestureEnabled;
  }

  Phase? _phase;
  PredictiveBackEvent? _startBackEvent;
  PredictiveBackEvent? _lastBackEvent;

  // Begin WidgetsBindingObserver.

  @override
  bool handleStartBackGesture(PredictiveBackEvent backEvent) {
    if (backEvent.isButtonEvent || !_isEnabled) {
      return false;
    }

    widget.route.handleStartBackGesture(progress: backEvent.progress);

    setState(() {
      _phase = Phase.preCommit;
      _startBackEvent = backEvent;
      _lastBackEvent = backEvent;
    });
    return true;
  }

  @override
  void handleUpdateBackGestureProgress(PredictiveBackEvent backEvent) {
    widget.route.handleUpdateBackGestureProgress(progress: backEvent.progress);

    setState(() {
      _lastBackEvent = backEvent;
    });
  }

  @override
  void handleCancelBackGesture() {
    widget.route.handleCancelBackGesture();
  }

  @override
  void handleCommitBackGesture() {
    widget.route.handleCommitBackGesture();
    setState(() {
      _phase = Phase.postCommit;
    });
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
    // Only do a predictive back transition when the user is performing a
    // pop gesture. Otherwise, for things like button presses or other
    // programmatic navigation, fall back to ZoomPageTransitionsBuilder.
    if (widget.route.popGestureInProgress) {
      final Size size = MediaQuery.sizeOf(context);
      const slideRatio = 0.06;
      const scaleRatio = slideRatio * 3;
      const commitOvershootFactor = 1.2;

      late Offset offset;
      late double scale;
      late double opacity;

      if (_phase != null) { // animation for the front page
        final progress = _lastBackEvent!.progress;
        final yOffset = (_lastBackEvent!.touchOffset!.dy - _startBackEvent!.touchOffset!.dy);
        final fromLeftEdge = _lastBackEvent!.swipeEdge == SwipeEdge.left;

        switch (_phase!) {
          case Phase.preCommit:
            offset = Offset(
              progress * slideRatio * size.width * (fromLeftEdge ? 1 : -1),
              progress * slideRatio * yOffset
            );
            scale = 1 - progress * scaleRatio;
            opacity = 1;
          case Phase.postCommit:
            final fixedAnimationValue = widget.animation.value / progress; 
            offset = Offset(
                progress * slideRatio * size.width * (fromLeftEdge ? 1 : -1) * fixedAnimationValue + (1 - fixedAnimationValue) * commitOvershootFactor * size.width,
                progress * slideRatio * yOffset                              * fixedAnimationValue,
            );
            scale = 1 - progress * scaleRatio; // stays the same
            opacity = fixedAnimationValue;
        }
      } else { // animation for the back page
        final progress = Tween(begin: 0.0, end: 1.0).animate(widget.secondaryAnimation).value;
        offset = Offset(-slideRatio * progress * size.width, 0.0);
        scale = 1 - progress * slideRatio;
        opacity = 1 - progress * 0.5;
      }

      return Transform.translate(
        offset: offset,
        child: Transform.scale(
          scale: scale,
          child: Opacity(
            opacity: opacity,
            child: widget.child
          )
        ),
      );
    }

     return const ZoomPageTransitionsBuilder().buildTransitions(
      widget.route,
      context,
      widget.animation,
      widget.secondaryAnimation,
      widget.child,
    );
  }
}