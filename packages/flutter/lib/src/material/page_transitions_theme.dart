// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'app.dart';
/// @docImport 'page.dart';
/// @docImport 'predictive_back_page_transitions_builder.dart';
library;

import 'dart:ui' as ui;

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';

import 'colors.dart';
import 'theme.dart';

// Slides the page upwards and fades it in, starting from 1/4 screen
// below the top. The transition is intended to match the default for
// Android O.
class _FadeUpwardsPageTransition extends StatelessWidget {
  _FadeUpwardsPageTransition({
    required Animation<double> routeAnimation, // The route's linear 0.0 - 1.0 animation.
    required this.child,
  }) : _positionAnimation = routeAnimation.drive(_bottomUpTween.chain(_fastOutSlowInTween)),
       _opacityAnimation = routeAnimation.drive(_easeInTween);

  // Fractional offset from 1/4 screen below the top to fully on screen.
  static final Tween<Offset> _bottomUpTween = Tween<Offset>(
    begin: const Offset(0.0, 0.25),
    end: Offset.zero,
  );
  static final Animatable<double> _fastOutSlowInTween = CurveTween(curve: Curves.fastOutSlowIn);
  static final Animatable<double> _easeInTween = CurveTween(curve: Curves.easeIn);

  final Animation<Offset> _positionAnimation;
  final Animation<double> _opacityAnimation;
  final Widget child;

  @override
  Widget build(BuildContext context) {
    return SlideTransition(
      position: _positionAnimation,
      // TODO(ianh): tell the transform to be un-transformed for hit testing
      child: FadeTransition(opacity: _opacityAnimation, child: child),
    );
  }
}

// This transition is intended to match the default for Android P.
class _OpenUpwardsPageTransition extends StatefulWidget {
  const _OpenUpwardsPageTransition({
    required this.animation,
    required this.secondaryAnimation,
    required this.child,
  });

  // The new page slides upwards just a little as its clip
  // rectangle exposes the page from bottom to top.
  static final Tween<Offset> _primaryTranslationTween = Tween<Offset>(
    begin: const Offset(0.0, 0.05),
    end: Offset.zero,
  );

  // The old page slides upwards a little as the new page appears.
  static final Tween<Offset> _secondaryTranslationTween = Tween<Offset>(
    begin: Offset.zero,
    end: const Offset(0.0, -0.025),
  );

  // The scrim obscures the old page by becoming increasingly opaque.
  static final Tween<double> _scrimOpacityTween = Tween<double>(begin: 0.0, end: 0.25);

  // Used by all of the transition animations.
  static const Curve _transitionCurve = Cubic(0.20, 0.00, 0.00, 1.00);

  final Animation<double> animation;
  final Animation<double> secondaryAnimation;
  final Widget child;

  @override
  State<_OpenUpwardsPageTransition> createState() => _OpenUpwardsPageTransitionState();
}

class _OpenUpwardsPageTransitionState extends State<_OpenUpwardsPageTransition> {
  late CurvedAnimation _primaryAnimation;
  late CurvedAnimation _secondaryTranslationCurvedAnimation;

  @override
  void initState() {
    super.initState();
    _setAnimations();
  }

  @override
  void didUpdateWidget(covariant _OpenUpwardsPageTransition oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.animation != widget.animation ||
        oldWidget.secondaryAnimation != widget.secondaryAnimation) {
      _disposeAnimations();
      _setAnimations();
    }
  }

  void _setAnimations() {
    _primaryAnimation = CurvedAnimation(
      parent: widget.animation,
      curve: _OpenUpwardsPageTransition._transitionCurve,
      reverseCurve: _OpenUpwardsPageTransition._transitionCurve.flipped,
    );
    _secondaryTranslationCurvedAnimation = CurvedAnimation(
      parent: widget.secondaryAnimation,
      curve: _OpenUpwardsPageTransition._transitionCurve,
      reverseCurve: _OpenUpwardsPageTransition._transitionCurve.flipped,
    );
  }

  void _disposeAnimations() {
    _primaryAnimation.dispose();
    _secondaryTranslationCurvedAnimation.dispose();
  }

  @override
  void dispose() {
    _disposeAnimations();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(
      builder: (BuildContext context, BoxConstraints constraints) {
        final Size size = constraints.biggest;

        // Gradually expose the new page from bottom to top.
        final Animation<double> clipAnimation = Tween<double>(
          begin: 0.0,
          end: size.height,
        ).animate(_primaryAnimation);

        final Animation<double> opacityAnimation = _OpenUpwardsPageTransition._scrimOpacityTween
            .animate(_primaryAnimation);
        final Animation<Offset> primaryTranslationAnimation = _OpenUpwardsPageTransition
            ._primaryTranslationTween
            .animate(_primaryAnimation);

        final Animation<Offset> secondaryTranslationAnimation = _OpenUpwardsPageTransition
            ._secondaryTranslationTween
            .animate(_secondaryTranslationCurvedAnimation);

        return AnimatedBuilder(
          animation: widget.animation,
          builder: (BuildContext context, Widget? child) {
            return ColoredBox(
              color: Colors.black.withOpacity(opacityAnimation.value),
              child: Align(
                alignment: Alignment.bottomLeft,
                child: ClipRect(
                  child: SizedBox(
                    height: clipAnimation.value,
                    child: OverflowBox(
                      alignment: Alignment.bottomLeft,
                      maxHeight: size.height,
                      child: child,
                    ),
                  ),
                ),
              ),
            );
          },
          child: AnimatedBuilder(
            animation: widget.secondaryAnimation,
            child: FractionalTranslation(
              translation: primaryTranslationAnimation.value,
              child: widget.child,
            ),
            builder: (BuildContext context, Widget? child) {
              return FractionalTranslation(
                translation: secondaryTranslationAnimation.value,
                child: child,
              );
            },
          ),
        );
      },
    );
  }
}

// Zooms and fades a new page in, zooming out the previous page. This transition
// is designed to match the Android Q activity transition.
class _ZoomPageTransition extends StatelessWidget {
  /// Creates a [_ZoomPageTransition].
  ///
  /// The [animation] and [secondaryAnimation] arguments are required and must
  /// not be null.
  const _ZoomPageTransition({
    required this.animation,
    required this.secondaryAnimation,
    required this.allowSnapshotting,
    required this.allowEnterRouteSnapshotting,
    this.backgroundColor,
    this.child,
  });

  // A curve sequence that is similar to the 'fastOutExtraSlowIn' curve used in
  // the native transition.
  static final List<TweenSequenceItem<double>> fastOutExtraSlowInTweenSequenceItems =
      <TweenSequenceItem<double>>[
        TweenSequenceItem<double>(
          tween: Tween<double>(
            begin: 0.0,
            end: 0.4,
          ).chain(CurveTween(curve: const Cubic(0.05, 0.0, 0.133333, 0.06))),
          weight: 0.166666,
        ),
        TweenSequenceItem<double>(
          tween: Tween<double>(
            begin: 0.4,
            end: 1.0,
          ).chain(CurveTween(curve: const Cubic(0.208333, 0.82, 0.25, 1.0))),
          weight: 1.0 - 0.166666,
        ),
      ];
  static final TweenSequence<double> _scaleCurveSequence = TweenSequence<double>(
    fastOutExtraSlowInTweenSequenceItems,
  );

  /// The animation that drives the [child]'s entrance and exit.
  ///
  /// See also:
  ///
  ///  * [TransitionRoute.animation], which is the value given to this property
  ///    when the [_ZoomPageTransition] is used as a page transition.
  final Animation<double> animation;

  /// The animation that transitions [child] when new content is pushed on top
  /// of it.
  ///
  /// See also:
  ///
  ///  * [TransitionRoute.secondaryAnimation], which is the value given to this
  ///    property when the [_ZoomPageTransition] is used as a page transition.
  final Animation<double> secondaryAnimation;

  /// Whether the [SnapshotWidget] will be used.
  ///
  /// When this value is true, performance is improved by disabling animations
  /// on both the outgoing and incoming route. This also implies that ink-splashes
  /// or similar animations will not animate during the transition.
  ///
  /// See also:
  ///
  ///  * [TransitionRoute.allowSnapshotting], which defines whether the route
  ///    transition will prefer to animate a snapshot of the entering and exiting
  ///    routes.
  final bool allowSnapshotting;

  /// The color of the scrim (background) that fades in and out during the transition.
  ///
  /// If not provided, defaults to current theme's [ColorScheme.surface] color.
  final Color? backgroundColor;

  /// The widget below this widget in the tree.
  ///
  /// This widget will transition in and out as driven by [animation] and
  /// [secondaryAnimation].
  final Widget? child;

  /// Whether to enable snapshotting on the entering route during the
  /// transition animation.
  ///
  /// If not specified, defaults to true.
  /// If false, the route snapshotting will not be applied to the route being
  /// animating into, e.g. when transitioning from route A to route B, B will
  /// not be snapshotted.
  final bool allowEnterRouteSnapshotting;

  @override
  Widget build(BuildContext context) {
    final Color enterTransitionBackgroundColor =
        backgroundColor ?? Theme.of(context).colorScheme.surface;
    return DualTransitionBuilder(
      animation: animation,
      forwardBuilder: (BuildContext context, Animation<double> animation, Widget? child) {
        return _ZoomEnterTransition(
          animation: animation,
          allowSnapshotting: allowSnapshotting && allowEnterRouteSnapshotting,
          backgroundColor: enterTransitionBackgroundColor,
          child: child,
        );
      },
      reverseBuilder: (BuildContext context, Animation<double> animation, Widget? child) {
        return _ZoomExitTransition(
          animation: animation,
          allowSnapshotting: allowSnapshotting,
          reverse: true,
          child: child,
        );
      },
      child: ZoomPageTransitionsBuilder._snapshotAwareDelegatedTransition(
        context,
        animation,
        secondaryAnimation,
        child,
        allowSnapshotting,
        allowEnterRouteSnapshotting,
        enterTransitionBackgroundColor,
      ),
    );
  }
}

class _ZoomEnterTransition extends StatefulWidget {
  const _ZoomEnterTransition({
    required this.animation,
    this.reverse = false,
    required this.allowSnapshotting,
    required this.backgroundColor,
    this.child,
  });

  final Animation<double> animation;
  final Widget? child;
  final bool allowSnapshotting;
  final bool reverse;
  final Color backgroundColor;

  @override
  State<_ZoomEnterTransition> createState() => _ZoomEnterTransitionState();
}

class _ZoomEnterTransitionState extends State<_ZoomEnterTransition>
    with _ZoomTransitionBase<_ZoomEnterTransition> {
  // See SnapshotWidget doc comment, this is disabled on web because the HTML backend doesn't
  // support this functionality and the canvaskit backend uses a single thread for UI and raster
  // work which diminishes the impact of this performance improvement.
  @override
  bool get useSnapshot => !kIsWeb && widget.allowSnapshotting;

  late _ZoomEnterTransitionPainter delegate;

  static final Animatable<double> _fadeInTransition = Tween<double>(
    begin: 0.0,
    end: 1.00,
  ).chain(CurveTween(curve: const Interval(0.125, 0.250)));

  static final Animatable<double> _scaleDownTransition = Tween<double>(
    begin: 1.10,
    end: 1.00,
  ).chain(_ZoomPageTransition._scaleCurveSequence);

  static final Animatable<double> _scaleUpTransition = Tween<double>(
    begin: 0.85,
    end: 1.00,
  ).chain(_ZoomPageTransition._scaleCurveSequence);

  static final Animatable<double?> _scrimOpacityTween = Tween<double?>(
    begin: 0.0,
    end: 0.60,
  ).chain(CurveTween(curve: const Interval(0.2075, 0.4175)));

  void _updateAnimations() {
    fadeTransition =
        widget.reverse ? kAlwaysCompleteAnimation : _fadeInTransition.animate(widget.animation);

    scaleTransition = (widget.reverse ? _scaleDownTransition : _scaleUpTransition).animate(
      widget.animation,
    );

    widget.animation.addListener(onAnimationValueChange);
    widget.animation.addStatusListener(onAnimationStatusChange);
  }

  @override
  void initState() {
    _updateAnimations();
    delegate = _ZoomEnterTransitionPainter(
      reverse: widget.reverse,
      fade: fadeTransition,
      scale: scaleTransition,
      animation: widget.animation,
      backgroundColor: widget.backgroundColor,
    );
    super.initState();
  }

  @override
  void didUpdateWidget(covariant _ZoomEnterTransition oldWidget) {
    if (oldWidget.reverse != widget.reverse || oldWidget.animation != widget.animation) {
      oldWidget.animation.removeListener(onAnimationValueChange);
      oldWidget.animation.removeStatusListener(onAnimationStatusChange);
      _updateAnimations();
      delegate.dispose();
      delegate = _ZoomEnterTransitionPainter(
        reverse: widget.reverse,
        fade: fadeTransition,
        scale: scaleTransition,
        animation: widget.animation,
        backgroundColor: widget.backgroundColor,
      );
    }
    super.didUpdateWidget(oldWidget);
  }

  @override
  void dispose() {
    widget.animation.removeListener(onAnimationValueChange);
    widget.animation.removeStatusListener(onAnimationStatusChange);
    delegate.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return SnapshotWidget(
      painter: delegate,
      controller: controller,
      mode: SnapshotMode.permissive,
      autoresize: true,
      child: widget.child,
    );
  }
}

class _ZoomExitTransition extends StatefulWidget {
  const _ZoomExitTransition({
    required this.animation,
    this.reverse = false,
    required this.allowSnapshotting,
    this.child,
  });

  final Animation<double> animation;
  final bool allowSnapshotting;
  final bool reverse;
  final Widget? child;

  @override
  State<_ZoomExitTransition> createState() => _ZoomExitTransitionState();
}

class _ZoomExitTransitionState extends State<_ZoomExitTransition>
    with _ZoomTransitionBase<_ZoomExitTransition> {
  late _ZoomExitTransitionPainter delegate;

  // See SnapshotWidget doc comment, this is disabled on web because the HTML backend doesn't
  // support this functionality and the canvaskit backend uses a single thread for UI and raster
  // work which diminishes the impact of this performance improvement.
  @override
  bool get useSnapshot => !kIsWeb && widget.allowSnapshotting;

  static final Animatable<double> _fadeOutTransition = Tween<double>(
    begin: 1.0,
    end: 0.0,
  ).chain(CurveTween(curve: const Interval(0.0825, 0.2075)));

  static final Animatable<double> _scaleUpTransition = Tween<double>(
    begin: 1.00,
    end: 1.05,
  ).chain(_ZoomPageTransition._scaleCurveSequence);

  static final Animatable<double> _scaleDownTransition = Tween<double>(
    begin: 1.00,
    end: 0.90,
  ).chain(_ZoomPageTransition._scaleCurveSequence);

  void _updateAnimations() {
    fadeTransition =
        widget.reverse ? _fadeOutTransition.animate(widget.animation) : kAlwaysCompleteAnimation;
    scaleTransition = (widget.reverse ? _scaleDownTransition : _scaleUpTransition).animate(
      widget.animation,
    );

    widget.animation.addListener(onAnimationValueChange);
    widget.animation.addStatusListener(onAnimationStatusChange);
  }

  @override
  void initState() {
    _updateAnimations();
    delegate = _ZoomExitTransitionPainter(
      reverse: widget.reverse,
      fade: fadeTransition,
      scale: scaleTransition,
      animation: widget.animation,
    );
    super.initState();
  }

  @override
  void didUpdateWidget(covariant _ZoomExitTransition oldWidget) {
    if (oldWidget.reverse != widget.reverse || oldWidget.animation != widget.animation) {
      oldWidget.animation.removeListener(onAnimationValueChange);
      oldWidget.animation.removeStatusListener(onAnimationStatusChange);
      _updateAnimations();
      delegate.dispose();
      delegate = _ZoomExitTransitionPainter(
        reverse: widget.reverse,
        fade: fadeTransition,
        scale: scaleTransition,
        animation: widget.animation,
      );
    }
    super.didUpdateWidget(oldWidget);
  }

  @override
  void dispose() {
    widget.animation.removeListener(onAnimationValueChange);
    widget.animation.removeStatusListener(onAnimationStatusChange);
    delegate.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return SnapshotWidget(
      painter: delegate,
      controller: controller,
      mode: SnapshotMode.permissive,
      autoresize: true,
      child: widget.child,
    );
  }
}

// This transition slides a new page in from right to left while fading it in,
// and simultaneously slides the previous page out to the left while fading it out.
// This transition is designed to match the Android U activity transition.
class _FadeForwardsPageTransition extends StatelessWidget {
  const _FadeForwardsPageTransition({
    required this.animation,
    required this.secondaryAnimation,
    this.backgroundColor,
    this.child,
  });

  final Animation<double> animation;

  final Animation<double> secondaryAnimation;

  final Color? backgroundColor;

  final Widget? child;

  // The new page slides in from right to left.
  static final Animatable<Offset> _forwardTranslationTween = Tween<Offset>(
    begin: const Offset(0.25, 0.0),
    end: Offset.zero,
  ).chain(CurveTween(curve: FadeForwardsPageTransitionsBuilder._transitionCurve));

  // The old page slides back from left to right.
  static final Animatable<Offset> _backwardTranslationTween = Tween<Offset>(
    begin: Offset.zero,
    end: const Offset(0.25, 0.0),
  ).chain(CurveTween(curve: FadeForwardsPageTransitionsBuilder._transitionCurve));

  @override
  Widget build(BuildContext context) {
    return DualTransitionBuilder(
      animation: animation,
      forwardBuilder: (BuildContext context, Animation<double> animation, Widget? child) {
        return FadeTransition(
          opacity: FadeForwardsPageTransitionsBuilder._fadeInTransition.animate(animation),
          child: SlideTransition(
            position: _forwardTranslationTween.animate(animation),
            child: child,
          ),
        );
      },
      reverseBuilder: (BuildContext context, Animation<double> animation, Widget? child) {
        return FadeTransition(
          opacity: FadeForwardsPageTransitionsBuilder._fadeOutTransition.animate(animation),
          child: SlideTransition(
            position: _backwardTranslationTween.animate(animation),
            child: child,
          ),
        );
      },
      child: FadeForwardsPageTransitionsBuilder._delegatedTransition(
        context,
        secondaryAnimation,
        backgroundColor,
        child,
      ),
    );
  }
}

/// Used by [PageTransitionsTheme] to define a [MaterialPageRoute] page
/// transition animation.
///
/// Apps can configure the map of builders for [ThemeData.pageTransitionsTheme]
/// to customize the default [MaterialPageRoute] page transition animation
/// for different platforms.
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
///  * [FadeForwardsPageTransitionsBuilder], which defines a page transition
///    that's similar to the one provided by Android U.
abstract class PageTransitionsBuilder {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const PageTransitionsBuilder();

  /// Provides a secondary transition to the previous route.
  ///
  /// {@macro flutter.widgets.delegatedTransition}
  DelegatedTransitionBuilder? get delegatedTransition => null;

  /// {@macro flutter.widgets.TransitionRoute.transitionDuration}
  ///
  /// Defaults to 300 milliseconds.
  Duration get transitionDuration => const Duration(milliseconds: 300);

  /// {@macro flutter.widgets.TransitionRoute.reverseTransitionDuration}
  ///
  /// Defaults to 300 milliseconds.
  Duration get reverseTransitionDuration => transitionDuration;

  /// Wraps the child with one or more transition widgets which define how [route]
  /// arrives on and leaves the screen.
  ///
  /// The [MaterialPageRoute.buildTransitions] method looks up the
  /// current [PageTransitionsTheme] with `Theme.of(context).pageTransitionsTheme`
  /// and delegates to this method with a [PageTransitionsBuilder] based
  /// on the theme's [ThemeData.platform].
  Widget buildTransitions<T>(
    PageRoute<T> route,
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
    Widget child,
  );
}

/// Used by [PageTransitionsTheme] to define a vertically fading
/// [MaterialPageRoute] page transition animation that looks like
/// the default page transition used on Android O.
///
/// The animation fades the new page in while translating it upwards,
/// starting from about 25% below the top of the screen.
///
/// See also:
///
///  * [OpenUpwardsPageTransitionsBuilder], which defines a page transition
///    that's similar to the one provided by Android P.
///  * [ZoomPageTransitionsBuilder], which defines the default page transition
///    that's similar to the one provided in Android Q.
///  * [CupertinoPageTransitionsBuilder], which defines a horizontal page
///    transition that matches native iOS page transitions.
///  * [PredictiveBackPageTransitionsBuilder], which defines a page
///    transition that allows peeking behind the current route on Android U and
///    above.
class FadeUpwardsPageTransitionsBuilder extends PageTransitionsBuilder {
  /// Constructs a page transition animation that slides the page up.
  const FadeUpwardsPageTransitionsBuilder();

  @override
  Widget buildTransitions<T>(
    PageRoute<T>? route,
    BuildContext? context,
    Animation<double> animation,
    Animation<double>? secondaryAnimation,
    Widget child,
  ) {
    return _FadeUpwardsPageTransition(routeAnimation: animation, child: child);
  }
}

/// Used by [PageTransitionsTheme] to define a vertical [MaterialPageRoute] page
/// transition animation that looks like the default page transition
/// used on Android P.
///
/// See also:
///
///  * [FadeUpwardsPageTransitionsBuilder], which defines a page transition
///    that's similar to the one provided by Android O.
///  * [ZoomPageTransitionsBuilder], which defines the default page transition
///    that's similar to the one provided in Android Q.
///  * [CupertinoPageTransitionsBuilder], which defines a horizontal page
///    transition that matches native iOS page transitions.
///  * [PredictiveBackPageTransitionsBuilder], which defines a page
///    transition that allows peeking behind the current route on Android.
///  * [FadeForwardsPageTransitionsBuilder], which defines a page transition
///    that's similar to the one provided by Android U.
class OpenUpwardsPageTransitionsBuilder extends PageTransitionsBuilder {
  /// Constructs a page transition animation that matches the transition used on
  /// Android P.
  const OpenUpwardsPageTransitionsBuilder();

  @override
  Widget buildTransitions<T>(
    PageRoute<T>? route,
    BuildContext? context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
    Widget child,
  ) {
    return _OpenUpwardsPageTransition(
      animation: animation,
      secondaryAnimation: secondaryAnimation,
      child: child,
    );
  }
}

/// Used by [PageTransitionsTheme] to define a horizontal [MaterialPageRoute] page
/// transition animation that looks like the default page transition
/// used on Android U.
///
/// {@tool dartpad}
/// This example shows the default page transition on Android.
///
/// ** See code in examples/api/lib/material/page_transitions_theme/page_transitions_theme.3.dart **
/// {@end-tool}
///
/// See also:
///
///  * [FadeUpwardsPageTransitionsBuilder], which defines a page transition
///    that's similar to the one provided by Android O.
///  * [OpenUpwardsPageTransitionsBuilder], which defines a page transition
///    that's similar to the one provided by Andoird P.
///  * [ZoomPageTransitionsBuilder], which defines the default page transition
///    that's similar to the one provided in Android Q.
///  * [CupertinoPageTransitionsBuilder], which defines a horizontal page
///    transition that matches native iOS page transitions.
///  * [PredictiveBackPageTransitionsBuilder], which defines a page
///    transition that allows peeking behind the current route on Android.
///  * [FadeForwardsPageTransitionsBuilder], which defines a page transition
///    that's similar to the one provided by Android U.
class FadeForwardsPageTransitionsBuilder extends PageTransitionsBuilder {
  /// Constructs a page transition animation that matches the transition used on
  /// Android U.
  const FadeForwardsPageTransitionsBuilder({this.backgroundColor});

  /// The background color during transition between two routes.
  ///
  /// When a new page fades in and the old page fades out, this background color
  /// helps avoid a black background between two page.
  ///
  /// Defaults to [ColorScheme.surface]
  final Color? backgroundColor;

  @override
  Duration get transitionDuration => const Duration(milliseconds: 800);

  @override
  DelegatedTransitionBuilder? get delegatedTransition =>
      (
        BuildContext context,
        Animation<double> animation,
        Animation<double> secondaryAnimation,
        bool allowSnapshotting,
        Widget? child,
      ) => _delegatedTransition(context, animation, backgroundColor, child);

  // Used by all of the sliding transition animations.
  static const Curve _transitionCurve = Curves.easeInOutCubicEmphasized;

  // The previous page slides from right to left as the current page appears.
  static final Animatable<Offset> _secondaryBackwardTranslationTween = Tween<Offset>(
    begin: Offset.zero,
    end: const Offset(-0.25, 0.0),
  ).chain(CurveTween(curve: _transitionCurve));

  // The previous page slides from left to right as the current page disappears.
  static final Animatable<Offset> _secondaryForwardTranslationTween = Tween<Offset>(
    begin: const Offset(-0.25, 0.0),
    end: Offset.zero,
  ).chain(CurveTween(curve: _transitionCurve));

  // The fade in transition when the new page appears.
  static final Animatable<double> _fadeInTransition = Tween<double>(
    begin: 0.0,
    end: 1.0,
  ).chain(CurveTween(curve: const Interval(0.0, 0.75)));

  // The fade out transition of the old page when the new page appears.
  static final Animatable<double> _fadeOutTransition = Tween<double>(
    begin: 1.0,
    end: 0.0,
  ).chain(CurveTween(curve: const Interval(0.0, 0.25)));

  static Widget _delegatedTransition(
    BuildContext context,
    Animation<double> secondaryAnimation,
    Color? backgroundColor,
    Widget? child,
  ) => DualTransitionBuilder(
    animation: ReverseAnimation(secondaryAnimation),
    forwardBuilder: (BuildContext context, Animation<double> animation, Widget? child) {
      return ColoredBox(
        color:
            animation.isAnimating
                ? backgroundColor ?? Theme.of(context).colorScheme.surface
                : Colors.transparent,
        child: FadeTransition(
          opacity: _fadeInTransition.animate(animation),
          child: SlideTransition(
            position: _secondaryForwardTranslationTween.animate(animation),
            child: child,
          ),
        ),
      );
    },
    reverseBuilder: (BuildContext context, Animation<double> animation, Widget? child) {
      return ColoredBox(
        color:
            animation.isAnimating
                ? backgroundColor ?? Theme.of(context).colorScheme.surface
                : Colors.transparent,
        child: FadeTransition(
          opacity: _fadeOutTransition.animate(animation),
          child: SlideTransition(
            position: _secondaryBackwardTranslationTween.animate(animation),
            child: child,
          ),
        ),
      );
    },
    child: child,
  );

  @override
  Widget buildTransitions<T>(
    PageRoute<T>? route,
    BuildContext? context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
    Widget child,
  ) {
    return _FadeForwardsPageTransition(
      animation: animation,
      secondaryAnimation: secondaryAnimation,
      backgroundColor: backgroundColor,
      child: child,
    );
  }
}

/// Used by [PageTransitionsTheme] to define a zooming [MaterialPageRoute] page
/// transition animation that looks like the default page transition used on
/// Android Q.
///
/// See also:
///
///  * [FadeUpwardsPageTransitionsBuilder], which defines a page transition
///    that's similar to the one provided by Android O.
///  * [OpenUpwardsPageTransitionsBuilder], which defines a page transition
///    that's similar to the one provided by Android P.
///  * [CupertinoPageTransitionsBuilder], which defines a horizontal page
///    transition that matches native iOS page transitions.
///  * [PredictiveBackPageTransitionsBuilder], which defines a page
///    transition that allows peeking behind the current route on Android.
///  * [FadeForwardsPageTransitionsBuilder], which defines a page transition
///    that's similar to the one provided by Android U.
class ZoomPageTransitionsBuilder extends PageTransitionsBuilder {
  /// Constructs a page transition animation that matches the transition used on
  /// Android Q.
  const ZoomPageTransitionsBuilder({
    this.allowSnapshotting = true,
    this.allowEnterRouteSnapshotting = true,
    this.backgroundColor,
  });

  /// Whether zoom page transitions will prefer to animate a snapshot of the entering
  /// and exiting routes.
  ///
  /// If not specified, defaults to true.
  ///
  /// When this value is true, zoom page transitions will snapshot the entering and
  /// exiting routes. These snapshots are then animated in place of the underlying
  /// widgets to improve performance of the transition.
  ///
  /// Generally this means that animations that occur on the entering/exiting route
  /// while the route animation plays may appear frozen - unless they are a hero
  /// animation or something that is drawn in a separate overlay.
  ///
  /// {@tool dartpad}
  /// This example shows a [MaterialApp] that disables snapshotting for the zoom
  /// transitions on Android.
  ///
  /// ** See code in examples/api/lib/material/page_transitions_theme/page_transitions_theme.1.dart **
  /// {@end-tool}
  ///
  /// See also:
  ///
  ///  * [PageRoute.allowSnapshotting], which enables or disables snapshotting
  ///    on a per route basis.
  final bool allowSnapshotting;

  /// Whether to enable snapshotting on the entering route during the
  /// transition animation.
  ///
  /// If not specified, defaults to true.
  /// If false, the route snapshotting will not be applied to the route being
  /// animating into, e.g. when transitioning from route A to route B, B will
  /// not be snapshotted.
  final bool allowEnterRouteSnapshotting;

  /// The color of the scrim (background) that fades in and out during the transition.
  ///
  /// If not provided, defaults to current theme's [ColorScheme.surface] color.
  final Color? backgroundColor;

  // Allows devicelab benchmarks to force disable the snapshotting. This is
  // intended to allow us to profile and fix the underlying performance issues
  // for the Impeller backend.
  static const bool _kProfileForceDisableSnapshotting = bool.fromEnvironment(
    'flutter.benchmarks.force_disable_snapshot',
  );

  @override
  DelegatedTransitionBuilder? get delegatedTransition =>
      (
        BuildContext context,
        Animation<double> animation,
        Animation<double> secondaryAnimation,
        bool allowSnapshotting,
        Widget? child,
      ) => _snapshotAwareDelegatedTransition(
        context,
        animation,
        secondaryAnimation,
        child,
        allowSnapshotting && this.allowSnapshotting,
        allowEnterRouteSnapshotting,
        backgroundColor,
      );

  // A transition builder that takes into account the snapshotting properties of
  // ZoomPageTransitionsBuilder.
  static Widget _snapshotAwareDelegatedTransition(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
    Widget? child,
    bool allowSnapshotting,
    bool allowEnterRouteSnapshotting,
    Color? backgroundColor,
  ) {
    final Color enterTransitionBackgroundColor =
        backgroundColor ?? Theme.of(context).colorScheme.surface;
    return DualTransitionBuilder(
      animation: ReverseAnimation(secondaryAnimation),
      forwardBuilder: (BuildContext context, Animation<double> animation, Widget? child) {
        return _ZoomEnterTransition(
          animation: animation,
          allowSnapshotting: allowSnapshotting && allowEnterRouteSnapshotting,
          reverse: true,
          backgroundColor: enterTransitionBackgroundColor,
          child: child,
        );
      },
      reverseBuilder: (BuildContext context, Animation<double> animation, Widget? child) {
        return _ZoomExitTransition(
          animation: animation,
          allowSnapshotting: allowSnapshotting,
          child: child,
        );
      },
      child: child,
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
    if (_kProfileForceDisableSnapshotting) {
      return _ZoomPageTransitionNoCache(
        animation: animation,
        secondaryAnimation: secondaryAnimation,
        child: child,
      );
    }
    return _ZoomPageTransition(
      animation: animation,
      secondaryAnimation: secondaryAnimation,
      allowSnapshotting: allowSnapshotting && route.allowSnapshotting,
      allowEnterRouteSnapshotting: allowEnterRouteSnapshotting,
      backgroundColor: backgroundColor,
      child: child,
    );
  }
}

/// Used by [PageTransitionsTheme] to define a horizontal [MaterialPageRoute]
/// page transition animation that matches native iOS page transitions.
///
/// See also:
///
///  * [FadeUpwardsPageTransitionsBuilder], which defines a page transition
///    that's similar to the one provided by Android O.
///  * [OpenUpwardsPageTransitionsBuilder], which defines a page transition
///    that's similar to the one provided by Android P.
///  * [ZoomPageTransitionsBuilder], which defines the default page transition
///    that's similar to the one provided in Android Q.
///  * [PredictiveBackPageTransitionsBuilder], which defines a page
///    transition that allows peeking behind the current route on Android.
class CupertinoPageTransitionsBuilder extends PageTransitionsBuilder {
  /// Constructs a page transition animation that matches the iOS transition.
  const CupertinoPageTransitionsBuilder();

  @override
  DelegatedTransitionBuilder? get delegatedTransition =>
      CupertinoPageTransition.delegatedTransition;

  @override
  Widget buildTransitions<T>(
    PageRoute<T> route,
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
    Widget child,
  ) {
    return CupertinoRouteTransitionMixin.buildPageTransitions<T>(
      route,
      context,
      animation,
      secondaryAnimation,
      child,
    );
  }
}

/// Defines the page transition animations used by [MaterialPageRoute]
/// for different [TargetPlatform]s.
///
/// The [MaterialPageRoute.buildTransitions] method looks up the
/// current [PageTransitionsTheme] with `Theme.of(context).pageTransitionsTheme`
/// and delegates to [buildTransitions].
///
/// If a builder with a matching platform is not found, then the
/// [ZoomPageTransitionsBuilder] is used.
///
/// {@tool dartpad}
/// This example shows a [MaterialApp] that defines a custom [PageTransitionsTheme].
///
/// ** See code in examples/api/lib/material/page_transitions_theme/page_transitions_theme.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [ThemeData.pageTransitionsTheme], which defines the default page
///    transitions for the overall theme.
///  * [FadeUpwardsPageTransitionsBuilder], which defines a page transition
///    that's similar to the one provided by Android O.
///  * [OpenUpwardsPageTransitionsBuilder], which defines a page transition
///    that's similar to the one provided by Android P.
///  * [ZoomPageTransitionsBuilder], which defines the default page transition
///    that's similar to the one provided by Android Q.
///  * [FadeForwardsPageTransitionsBuilder], which defines the default page transition
///    that's similar to the one provided by Android U.
///  * [CupertinoPageTransitionsBuilder], which defines a horizontal page
///    transition that matches native iOS page transitions.
@immutable
class PageTransitionsTheme with Diagnosticable {
  /// Constructs an object that selects a transition based on the platform.
  ///
  /// By default the list of builders is: [ZoomPageTransitionsBuilder]
  /// for [TargetPlatform.android], and [CupertinoPageTransitionsBuilder] for
  /// [TargetPlatform.iOS] and [TargetPlatform.macOS].
  const PageTransitionsTheme({
    Map<TargetPlatform, PageTransitionsBuilder> builders = _defaultBuilders,
  }) : _builders = builders;

  static const Map<TargetPlatform, PageTransitionsBuilder> _defaultBuilders =
      <TargetPlatform, PageTransitionsBuilder>{
        TargetPlatform.android: ZoomPageTransitionsBuilder(),
        TargetPlatform.iOS: CupertinoPageTransitionsBuilder(),
        TargetPlatform.macOS: CupertinoPageTransitionsBuilder(),
      };

  /// The [PageTransitionsBuilder]s supported by this theme.
  Map<TargetPlatform, PageTransitionsBuilder> get builders => _builders;
  final Map<TargetPlatform, PageTransitionsBuilder> _builders;

  /// Delegates to the builder for the current [ThemeData.platform].
  /// If a builder for the current platform is not found, then the
  /// [ZoomPageTransitionsBuilder] is used.
  ///
  /// [MaterialPageRoute.buildTransitions] delegates to this method.
  Widget buildTransitions<T>(
    PageRoute<T> route,
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
    Widget child,
  ) {
    return _PageTransitionsThemeTransitions<T>(
      builders: builders,
      route: route,
      animation: animation,
      secondaryAnimation: secondaryAnimation,
      child: child,
    );
  }

  /// Provides the delegate transition for the target platform.
  ///
  /// {@macro flutter.widgets.delegatedTransition}
  DelegatedTransitionBuilder? delegatedTransition(TargetPlatform platform) {
    final PageTransitionsBuilder matchingBuilder =
        builders[platform] ?? const ZoomPageTransitionsBuilder();

    return matchingBuilder.delegatedTransition;
  }

  // Map the builders to a list with one PageTransitionsBuilder per platform for
  // the operator == overload.
  List<PageTransitionsBuilder?> _all(Map<TargetPlatform, PageTransitionsBuilder> builders) {
    return TargetPlatform.values.map((TargetPlatform platform) => builders[platform]).toList();
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    if (other is PageTransitionsTheme && identical(builders, other.builders)) {
      return true;
    }
    return other is PageTransitionsTheme &&
        listEquals<PageTransitionsBuilder?>(_all(other.builders), _all(builders));
  }

  @override
  int get hashCode => Object.hashAll(_all(builders));

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      DiagnosticsProperty<Map<TargetPlatform, PageTransitionsBuilder>>(
        'builders',
        builders,
        defaultValue: PageTransitionsTheme._defaultBuilders,
      ),
    );
  }
}

class _PageTransitionsThemeTransitions<T> extends StatefulWidget {
  const _PageTransitionsThemeTransitions({
    required this.builders,
    required this.route,
    required this.animation,
    required this.secondaryAnimation,
    required this.child,
  });

  final Map<TargetPlatform, PageTransitionsBuilder> builders;
  final PageRoute<T> route;
  final Animation<double> animation;
  final Animation<double> secondaryAnimation;
  final Widget child;

  @override
  State<_PageTransitionsThemeTransitions<T>> createState() =>
      _PageTransitionsThemeTransitionsState<T>();
}

class _PageTransitionsThemeTransitionsState<T> extends State<_PageTransitionsThemeTransitions<T>> {
  TargetPlatform? _transitionPlatform;

  @override
  Widget build(BuildContext context) {
    TargetPlatform platform = Theme.of(context).platform;

    // If the theme platform is changed in the middle of a pop gesture, keep the
    // transition that the gesture began with until the gesture is finished.
    if (widget.route.popGestureInProgress) {
      _transitionPlatform ??= platform;
      platform = _transitionPlatform!;
    } else {
      _transitionPlatform = null;
    }

    final PageTransitionsBuilder matchingBuilder =
        widget.builders[platform] ??
        switch (platform) {
          TargetPlatform.iOS => const CupertinoPageTransitionsBuilder(),
          TargetPlatform.android ||
          TargetPlatform.fuchsia ||
          TargetPlatform.windows ||
          TargetPlatform.macOS ||
          TargetPlatform.linux =>
            const ZoomPageTransitionsBuilder(),
        };
    return matchingBuilder.buildTransitions<T>(
      widget.route,
      context,
      widget.animation,
      widget.secondaryAnimation,
      widget.child,
    );
  }
}

// Take an image and draw it centered and scaled. The image is already scaled by the [pixelRatio].
void _drawImageScaledAndCentered(
  PaintingContext context,
  ui.Image image,
  double scale,
  double opacity,
  double pixelRatio,
) {
  if (scale <= 0.0 || opacity <= 0.0) {
    return;
  }
  final Paint paint =
      Paint()
        ..filterQuality = ui.FilterQuality.medium
        ..color = Color.fromRGBO(0, 0, 0, opacity);
  final double logicalWidth = image.width / pixelRatio;
  final double logicalHeight = image.height / pixelRatio;
  final double scaledLogicalWidth = logicalWidth * scale;
  final double scaledLogicalHeight = logicalHeight * scale;
  final double left = (logicalWidth - scaledLogicalWidth) / 2;
  final double top = (logicalHeight - scaledLogicalHeight) / 2;
  final Rect dst = Rect.fromLTWH(left, top, scaledLogicalWidth, scaledLogicalHeight);
  context.canvas.drawImageRect(
    image,
    Rect.fromLTWH(0, 0, image.width.toDouble(), image.height.toDouble()),
    dst,
    paint,
  );
}

void _updateScaledTransform(Matrix4 transform, double scale, Size size) {
  transform.setIdentity();
  if (scale == 1.0) {
    return;
  }
  transform.scale(scale, scale);
  final double dx = ((size.width * scale) - size.width) / 2;
  final double dy = ((size.height * scale) - size.height) / 2;
  transform.translate(-dx, -dy);
}

mixin _ZoomTransitionBase<S extends StatefulWidget> on State<S> {
  bool get useSnapshot;

  // Don't rasterize if:
  // 1. Rasterization is disabled by the platform.
  // 2. The animation is paused/stopped.
  // 3. The values of the scale/fade transition do not
  //    benefit from rasterization.
  final SnapshotController controller = SnapshotController();

  late Animation<double> fadeTransition;
  late Animation<double> scaleTransition;

  void onAnimationValueChange() {
    if ((scaleTransition.value == 1.0) &&
        (fadeTransition.value == 0.0 || fadeTransition.value == 1.0)) {
      controller.allowSnapshotting = false;
    } else {
      controller.allowSnapshotting = useSnapshot;
    }
  }

  void onAnimationStatusChange(AnimationStatus status) {
    controller.allowSnapshotting = status.isAnimating && useSnapshot;
  }

  @override
  void dispose() {
    controller.dispose();
    super.dispose();
  }
}

class _ZoomEnterTransitionPainter extends SnapshotPainter {
  _ZoomEnterTransitionPainter({
    required this.reverse,
    required this.scale,
    required this.fade,
    required this.animation,
    required this.backgroundColor,
  }) {
    animation.addListener(notifyListeners);
    animation.addStatusListener(_onStatusChange);
    scale.addListener(notifyListeners);
    fade.addListener(notifyListeners);
  }

  void _onStatusChange(_) {
    notifyListeners();
  }

  final bool reverse;
  final Animation<double> animation;
  final Animation<double> scale;
  final Animation<double> fade;
  final Color backgroundColor;

  final Matrix4 _transform = Matrix4.zero();
  final LayerHandle<OpacityLayer> _opacityHandle = LayerHandle<OpacityLayer>();
  final LayerHandle<TransformLayer> _transformHandler = LayerHandle<TransformLayer>();

  void _drawScrim(PaintingContext context, Offset offset, Size size) {
    double scrimOpacity = 0.0;
    // The transition's scrim opacity only increases on the forward transition.
    // In the reverse transition, the opacity should always be 0.0.
    //
    // Therefore, we need to only apply the scrim opacity animation when
    // the transition is running forwards.
    //
    // The reason that we check that the animation's status is not `completed`
    // instead of checking that it is `forward` is that this allows
    // the interrupted reversal of the forward transition to smoothly fade
    // the scrim away. This prevents a disjointed removal of the scrim.
    if (!reverse && !animation.isCompleted) {
      scrimOpacity = _ZoomEnterTransitionState._scrimOpacityTween.evaluate(animation)!;
    }
    assert(!reverse || scrimOpacity == 0.0);
    if (scrimOpacity > 0.0) {
      context.canvas.drawRect(
        offset & size,
        Paint()..color = backgroundColor.withOpacity(scrimOpacity),
      );
    }
  }

  @override
  void paint(
    PaintingContext context,
    ui.Offset offset,
    Size size,
    PaintingContextCallback painter,
  ) {
    if (!animation.isAnimating) {
      return painter(context, offset);
    }

    _drawScrim(context, offset, size);
    _updateScaledTransform(_transform, scale.value, size);
    _transformHandler.layer = context.pushTransform(true, offset, _transform, (
      PaintingContext context,
      Offset offset,
    ) {
      _opacityHandle.layer = context.pushOpacity(
        offset,
        (fade.value * 255).round(),
        painter,
        oldLayer: _opacityHandle.layer,
      );
    }, oldLayer: _transformHandler.layer);
  }

  @override
  void paintSnapshot(
    PaintingContext context,
    Offset offset,
    Size size,
    ui.Image image,
    Size sourceSize,
    double pixelRatio,
  ) {
    _drawScrim(context, offset, size);
    _drawImageScaledAndCentered(context, image, scale.value, fade.value, pixelRatio);
  }

  @override
  void dispose() {
    animation.removeListener(notifyListeners);
    animation.removeStatusListener(_onStatusChange);
    scale.removeListener(notifyListeners);
    fade.removeListener(notifyListeners);
    _opacityHandle.layer = null;
    _transformHandler.layer = null;
    super.dispose();
  }

  @override
  bool shouldRepaint(covariant _ZoomEnterTransitionPainter oldDelegate) {
    return oldDelegate.reverse != reverse ||
        oldDelegate.animation.value != animation.value ||
        oldDelegate.scale.value != scale.value ||
        oldDelegate.fade.value != fade.value;
  }
}

class _ZoomExitTransitionPainter extends SnapshotPainter {
  _ZoomExitTransitionPainter({
    required this.reverse,
    required this.scale,
    required this.fade,
    required this.animation,
  }) {
    scale.addListener(notifyListeners);
    fade.addListener(notifyListeners);
    animation.addStatusListener(_onStatusChange);
  }

  void _onStatusChange(_) {
    notifyListeners();
  }

  final bool reverse;
  final Animation<double> scale;
  final Animation<double> fade;
  final Animation<double> animation;
  final Matrix4 _transform = Matrix4.zero();
  final LayerHandle<OpacityLayer> _opacityHandle = LayerHandle<OpacityLayer>();
  final LayerHandle<TransformLayer> _transformHandler = LayerHandle<TransformLayer>();

  @override
  void paintSnapshot(
    PaintingContext context,
    Offset offset,
    Size size,
    ui.Image image,
    Size sourceSize,
    double pixelRatio,
  ) {
    _drawImageScaledAndCentered(context, image, scale.value, fade.value, pixelRatio);
  }

  @override
  void paint(
    PaintingContext context,
    ui.Offset offset,
    Size size,
    PaintingContextCallback painter,
  ) {
    if (!animation.isAnimating) {
      return painter(context, offset);
    }

    _updateScaledTransform(_transform, scale.value, size);
    _transformHandler.layer = context.pushTransform(true, offset, _transform, (
      PaintingContext context,
      Offset offset,
    ) {
      _opacityHandle.layer = context.pushOpacity(
        offset,
        (fade.value * 255).round(),
        painter,
        oldLayer: _opacityHandle.layer,
      );
    }, oldLayer: _transformHandler.layer);
  }

  @override
  bool shouldRepaint(covariant _ZoomExitTransitionPainter oldDelegate) {
    return oldDelegate.reverse != reverse ||
        oldDelegate.fade.value != fade.value ||
        oldDelegate.scale.value != scale.value;
  }

  @override
  void dispose() {
    _opacityHandle.layer = null;
    _transformHandler.layer = null;
    scale.removeListener(notifyListeners);
    fade.removeListener(notifyListeners);
    animation.removeStatusListener(_onStatusChange);
    super.dispose();
  }
}

// Zooms and fades a new page in, zooming out the previous page. This transition
// is designed to match the Android Q activity transition.
//
// This was the historical implementation of the cacheless zoom page transition
// that was too slow to run on the Skia backend. This is being benchmarked on
// the Impeller backend so that we can improve performance enough to restore
// the default behavior.
class _ZoomPageTransitionNoCache extends StatelessWidget {
  /// Creates a [_ZoomPageTransitionNoCache].
  ///
  /// The [animation] and [secondaryAnimation] argument are required and must
  /// not be null.
  const _ZoomPageTransitionNoCache({
    required this.animation,
    required this.secondaryAnimation,
    this.child,
  });

  /// The animation that drives the [child]'s entrance and exit.
  ///
  /// See also:
  ///
  ///  * [TransitionRoute.animation], which is the value given to this property
  ///    when the [_ZoomPageTransition] is used as a page transition.
  final Animation<double> animation;

  /// The animation that transitions [child] when new content is pushed on top
  /// of it.
  ///
  /// See also:
  ///
  ///  * [TransitionRoute.secondaryAnimation], which is the value given to this
  ///    property when the [_ZoomPageTransition] is used as a page transition.
  final Animation<double> secondaryAnimation;

  /// The widget below this widget in the tree.
  ///
  /// This widget will transition in and out as driven by [animation] and
  /// [secondaryAnimation].
  final Widget? child;

  @override
  Widget build(BuildContext context) {
    return DualTransitionBuilder(
      animation: animation,
      forwardBuilder: (BuildContext context, Animation<double> animation, Widget? child) {
        return _ZoomEnterTransitionNoCache(animation: animation, child: child);
      },
      reverseBuilder: (BuildContext context, Animation<double> animation, Widget? child) {
        return _ZoomExitTransitionNoCache(animation: animation, reverse: true, child: child);
      },
      child: DualTransitionBuilder(
        animation: ReverseAnimation(secondaryAnimation),
        forwardBuilder: (BuildContext context, Animation<double> animation, Widget? child) {
          return _ZoomEnterTransitionNoCache(animation: animation, reverse: true, child: child);
        },
        reverseBuilder: (BuildContext context, Animation<double> animation, Widget? child) {
          return _ZoomExitTransitionNoCache(animation: animation, child: child);
        },
        child: child,
      ),
    );
  }
}

class _ZoomEnterTransitionNoCache extends StatelessWidget {
  const _ZoomEnterTransitionNoCache({required this.animation, this.reverse = false, this.child});

  final Animation<double> animation;
  final Widget? child;
  final bool reverse;

  @override
  Widget build(BuildContext context) {
    double opacity = 0;
    // The transition's scrim opacity only increases on the forward transition.
    // In the reverse transition, the opacity should always be 0.0.
    //
    // Therefore, we need to only apply the scrim opacity animation when
    // the transition is running forwards.
    //
    // The reason that we check that the animation's status is not `completed`
    // instead of checking that it is `forward` is that this allows
    // the interrupted reversal of the forward transition to smoothly fade
    // the scrim away. This prevents a disjointed removal of the scrim.
    if (!reverse && !animation.isCompleted) {
      opacity = _ZoomEnterTransitionState._scrimOpacityTween.evaluate(animation)!;
    }

    final Animation<double> fadeTransition =
        reverse
            ? kAlwaysCompleteAnimation
            : _ZoomEnterTransitionState._fadeInTransition.animate(animation);

    final Animation<double> scaleTransition = (reverse
            ? _ZoomEnterTransitionState._scaleDownTransition
            : _ZoomEnterTransitionState._scaleUpTransition)
        .animate(animation);

    return AnimatedBuilder(
      animation: animation,
      builder: (BuildContext context, Widget? child) {
        return ColoredBox(color: Colors.black.withOpacity(opacity), child: child);
      },
      child: FadeTransition(
        opacity: fadeTransition,
        child: ScaleTransition(
          scale: scaleTransition,
          filterQuality: FilterQuality.medium,
          child: child,
        ),
      ),
    );
  }
}

class _ZoomExitTransitionNoCache extends StatelessWidget {
  const _ZoomExitTransitionNoCache({required this.animation, this.reverse = false, this.child});

  final Animation<double> animation;
  final bool reverse;
  final Widget? child;

  @override
  Widget build(BuildContext context) {
    final Animation<double> fadeTransition =
        reverse
            ? _ZoomExitTransitionState._fadeOutTransition.animate(animation)
            : kAlwaysCompleteAnimation;
    final Animation<double> scaleTransition = (reverse
            ? _ZoomExitTransitionState._scaleDownTransition
            : _ZoomExitTransitionState._scaleUpTransition)
        .animate(animation);

    return FadeTransition(
      opacity: fadeTransition,
      child: ScaleTransition(
        scale: scaleTransition,
        filterQuality: FilterQuality.medium,
        child: child,
      ),
    );
  }
}
