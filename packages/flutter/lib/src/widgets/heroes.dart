// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
library;

import 'package:flutter/foundation.dart';
import 'basic.dart';
import 'binding.dart';
import 'framework.dart';
import 'implicit_animations.dart';
import 'media_query.dart';
import 'navigator.dart';
import 'overlay.dart';
import 'pages.dart';
import 'routes.dart';
import 'ticker_provider.dart' show TickerMode;
import 'transitions.dart';

/// Signature for a function that takes two [Rect] instances and returns a
/// [RectTween] that transitions between them.
///
/// This is typically used with a [HeroController] to provide an animation for
/// [Hero] positions that looks nicer than a linear movement. For example, see
/// [MaterialRectArcTween].
typedef CreateRectTween = Tween<Rect?> Function(Rect? begin, Rect? end);

/// Signature for a function that builds a [Hero] placeholder widget given a
/// child and a [Size].
///
/// The child can optionally be part of the returned widget tree. The returned
/// widget should typically be constrained to [heroSize], if it doesn't do so
/// implicitly.
///
/// See also:
///
///  * [TransitionBuilder], which is similar but only takes a [BuildContext]
///    and a child widget.
typedef HeroPlaceholderBuilder = Widget Function(BuildContext context, Size heroSize, Widget child);

/// A function that lets [Hero]es self supply a [Widget] that is shown during the
/// hero's flight from one route to another instead of default (which is to
/// show the destination route's instance of the Hero).
typedef HeroFlightShuttleBuilder =
    Widget Function(
      BuildContext flightContext,
      Animation<double> animation,
      HeroFlightDirection flightDirection,
      BuildContext fromHeroContext,
      BuildContext toHeroContext,
    );

typedef _OnFlightEnded = void Function(_HeroFlight flight);

/// Direction of the hero's flight based on the navigation operation.
enum HeroFlightDirection {
  /// A flight triggered by a route push.
  ///
  /// The animation goes from 0 to 1.
  ///
  /// If no custom [HeroFlightShuttleBuilder] is supplied, the top route's
  /// [Hero] child is shown in flight.
  push,

  /// A flight triggered by a route pop.
  ///
  /// The animation goes from 1 to 0.
  ///
  /// If no custom [HeroFlightShuttleBuilder] is supplied, the bottom route's
  /// [Hero] child is shown in flight.
  pop,
}

/// A widget that marks its child as being a candidate for
/// [hero animations](https://docs.flutter.dev/ui/animations/hero-animations).
///
/// When a [PageRoute] is pushed or popped with the [Navigator], the entire
/// screen's content is replaced. An old route disappears and a new route
/// appears. If there's a common visual feature on both routes then it can
/// be helpful for orienting the user for the feature to physically move from
/// one page to the other during the routes' transition. Such an animation
/// is called a *hero animation*. The hero widgets "fly" in the Navigator's
/// overlay during the transition and while they're in-flight they're, by
/// default, not shown in their original locations in the old and new routes.
///
/// To label a widget as such a feature, wrap it in a [Hero] widget. When
/// navigation happens, the [Hero] widgets on each route are identified
/// by the [HeroController]. For each pair of [Hero] widgets that have the
/// same tag, a hero animation is triggered.
///
/// If a [Hero] is already in flight when navigation occurs, its
/// flight animation will be redirected to its new destination. The
/// widget shown in-flight during the transition is, by default, the
/// destination route's [Hero]'s child.
///
/// For a Hero animation to trigger, the Hero has to exist on the very first
/// frame of the new page's animation.
///
/// Routes must not contain more than one [Hero] for each [tag].
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=Be9UH1kXFDw}
///
/// {@tool dartpad}
/// This sample shows a [Hero] used within a [ListTile].
///
/// Tapping on the Hero-wrapped rectangle triggers a hero
/// animation as a new [MaterialPageRoute] is pushed. Both the size
/// and location of the rectangle animates.
///
/// Both widgets use the same [Hero.tag].
///
/// The Hero widget uses the matching tags to identify and execute this
/// animation.
///
/// ** See code in examples/api/lib/widgets/heroes/hero.0.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This sample shows [Hero] flight animations using default tween
/// and custom rect tween.
///
/// ** See code in examples/api/lib/widgets/heroes/hero.1.dart **
/// {@end-tool}
///
/// ## Discussion
///
/// Heroes and the [Navigator]'s [Overlay] [Stack] must be axis-aligned for
/// all this to work. The top left and bottom right coordinates of each animated
/// Hero will be converted to global coordinates and then from there converted
/// to that [Stack]'s coordinate space, and the entire Hero subtree will, for
/// the duration of the animation, be lifted out of its original place, and
/// positioned on that stack. If the [Hero] isn't axis aligned, this is going to
/// fail in a rather ugly fashion. Don't rotate your heroes!
///
/// To make the animations look good, it's critical that the widget tree for the
/// hero in both locations be essentially identical. The widget of the *target*
/// is, by default, used to do the transition: when going from route A to route
/// B, route B's hero's widget is placed over route A's hero's widget. Additionally,
/// if the [Hero] subtree changes appearance based on an [InheritedWidget] (such
/// as [MediaQuery] or [Theme]), then the hero animation may have discontinuity
/// at the start or the end of the animation because route A and route B provides
/// different such [InheritedWidget]s. Consider providing a custom [flightShuttleBuilder]
/// to ensure smooth transitions. The default [flightShuttleBuilder] interpolates
/// [MediaQuery]'s paddings. If your [Hero] widget uses custom [InheritedWidget]s
/// and displays a discontinuity in the animation, try to provide custom in-flight
/// transition using [flightShuttleBuilder].
///
/// By default, both route A and route B's heroes are hidden while the
/// transitioning widget is animating in-flight above the 2 routes.
/// [placeholderBuilder] can be used to show a custom widget in their place
/// instead once the transition has taken flight.
///
/// During the transition, the transition widget is animated to route B's hero's
/// position, and then the widget is inserted into route B. When going back from
/// B to A, route A's hero's widget is, by default, placed over where route B's
/// hero's widget was, and then the animation goes the other way.
///
/// ### Nested Navigators
///
/// If either or both routes contain nested [Navigator]s, only [Hero]es
/// contained in the top-most routes (as defined by [Route.isCurrent]) *of those
/// nested [Navigator]s* are considered for animation. Just like in the
/// non-nested case the top-most routes containing these [Hero]es in the nested
/// [Navigator]s have to be [PageRoute]s.
///
/// ## Parts of a Hero Transition
///
/// ![Diagrams with parts of the Hero transition.](https://flutter.github.io/assets-for-api-docs/assets/interaction/heroes.png)
class Hero extends StatefulWidget {
  /// Create a hero.
  ///
  /// The [child] parameter and all of the its descendants must not be [Hero]es.
  const Hero({
    super.key,
    required this.tag,
    this.createRectTween,
    this.flightShuttleBuilder,
    this.placeholderBuilder,
    this.transitionOnUserGestures = false,
    required this.child,
  });

  /// The identifier for this particular hero. If the tag of this hero matches
  /// the tag of a hero on a [PageRoute] that we're navigating to or from, then
  /// a hero animation will be triggered.
  final Object tag;

  /// Defines how the destination hero's bounds change as it flies from the starting
  /// route to the destination route.
  ///
  /// A hero flight begins with the destination hero's [child] aligned with the
  /// starting hero's child. The [Tween<Rect>] returned by this callback is used
  /// to compute the hero's bounds as the flight animation's value goes from 0.0
  /// to 1.0.
  ///
  /// If this property is null, the default, then the value of
  /// [HeroController.createRectTween] is used. The [HeroController] created by
  /// [MaterialApp] creates a [MaterialRectArcTween].
  final CreateRectTween? createRectTween;

  /// The widget subtree that will "fly" from one route to another during a
  /// [Navigator] push or pop transition.
  ///
  /// The appearance of this subtree should be similar to the appearance of
  /// the subtrees of any other heroes in the application with the same [tag].
  /// Changes in scale and aspect ratio work well in hero animations, changes
  /// in layout or composition do not.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  /// Optional override to supply a widget that's shown during the hero's flight.
  ///
  /// This in-flight widget can depend on the route transition's animation as
  /// well as the incoming and outgoing routes' [Hero] descendants' widgets and
  /// layout.
  ///
  /// When both the source and destination [Hero]es provide a [flightShuttleBuilder],
  /// the destination's [flightShuttleBuilder] takes precedence.
  ///
  /// If none is provided, the destination route's Hero child is shown in-flight
  /// by default.
  ///
  /// ## Limitations
  ///
  /// If a widget built by [flightShuttleBuilder] takes part in a [Navigator]
  /// push transition, that widget or its descendants must not have any
  /// [GlobalKey] that is used in the source Hero's descendant widgets. That is
  /// because both subtrees will be included in the widget tree during the Hero
  /// flight animation, and [GlobalKey]s must be unique across the entire widget
  /// tree.
  ///
  /// If the said [GlobalKey] is essential to your application, consider providing
  /// a custom [placeholderBuilder] for the source Hero, to avoid the [GlobalKey]
  /// collision, such as a builder that builds an empty [SizedBox], keeping the
  /// Hero [child]'s original size.
  final HeroFlightShuttleBuilder? flightShuttleBuilder;

  /// Placeholder widget left in place as the Hero's [child] once the flight takes
  /// off.
  ///
  /// By default the placeholder widget is an empty [SizedBox] keeping the Hero
  /// child's original size, unless this Hero is a source Hero of a [Navigator]
  /// push transition, in which case [child] will be a descendant of the placeholder
  /// and will be kept [Offstage] during the Hero's flight.
  final HeroPlaceholderBuilder? placeholderBuilder;

  /// Whether to perform the hero transition if the [PageRoute] transition was
  /// triggered by a user gesture, such as a back swipe on iOS.
  ///
  /// If [Hero]es with the same [tag] on both the from and the to routes have
  /// [transitionOnUserGestures] set to true, a back swipe gesture will
  /// trigger the same hero animation as a programmatically triggered push or
  /// pop.
  ///
  /// The route being popped to or the bottom route must also have
  /// [PageRoute.maintainState] set to true for a gesture triggered hero
  /// transition to work.
  ///
  /// Defaults to false.
  final bool transitionOnUserGestures;

  // Returns a map of all of the heroes in `context` indexed by hero tag that
  // should be considered for animation when `navigator` transitions from one
  // PageRoute to another.
  static Map<Object, _HeroState> _allHeroesFor(
    BuildContext context,
    bool isUserGestureTransition,
    NavigatorState navigator,
  ) {
    final Map<Object, _HeroState> result = <Object, _HeroState>{};

    void inviteHero(StatefulElement hero, Object tag) {
      assert(() {
        if (result.containsKey(tag)) {
          throw FlutterError.fromParts(<DiagnosticsNode>[
            ErrorSummary('There are multiple heroes that share the same tag within a subtree.'),
            ErrorDescription(
              'Within each subtree for which heroes are to be animated (i.e. a PageRoute subtree), '
              'each Hero must have a unique non-null tag.\n'
              'In this case, multiple heroes had the following tag: $tag',
            ),
            DiagnosticsProperty<StatefulElement>(
              'Here is the subtree for one of the offending heroes',
              hero,
              linePrefix: '# ',
              style: DiagnosticsTreeStyle.dense,
            ),
          ]);
        }
        return true;
      }());
      final Hero heroWidget = hero.widget as Hero;
      final _HeroState heroState = hero.state as _HeroState;
      if (!isUserGestureTransition || heroWidget.transitionOnUserGestures) {
        result[tag] = heroState;
      } else {
        // If transition is not allowed, we need to make sure hero is not hidden.
        // A hero can be hidden previously due to hero transition.
        heroState.endFlight();
      }
    }

    void visitor(Element element) {
      final Widget widget = element.widget;
      if (widget is Hero) {
        final StatefulElement hero = element as StatefulElement;
        final Object tag = widget.tag;
        if (Navigator.of(hero) == navigator) {
          inviteHero(hero, tag);
        } else {
          // The nearest navigator to the Hero is not the Navigator that is
          // currently transitioning from one route to another. This means
          // the Hero is inside a nested Navigator and should only be
          // considered for animation if it is part of the top-most route in
          // that nested Navigator and if that route is also a PageRoute.
          final ModalRoute<Object?>? heroRoute = ModalRoute.of(hero);
          if (heroRoute != null && heroRoute is PageRoute && heroRoute.isCurrent) {
            inviteHero(hero, tag);
          }
        }
      } else if (widget is HeroMode && !widget.enabled) {
        return;
      }
      element.visitChildren(visitor);
    }

    context.visitChildElements(visitor);
    return result;
  }

  @override
  State<Hero> createState() => _HeroState();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<Object>('tag', tag));
  }
}

/// The [Hero] widget displays different content based on whether it is in an
/// animated transition ("flight"), from/to another [Hero] with the same tag:
///   * When [startFlight] is called, the real content of this [Hero] will be
///     replaced by a "placeholder" widget.
///   * When the flight ends, the "toHero"'s [endFlight] method must be called
///     by the hero controller, so the real content of that [Hero] becomes
///     visible again when the animation completes.
class _HeroState extends State<Hero> {
  final GlobalKey _key = GlobalKey();
  Size? _placeholderSize;
  // Whether the placeholder widget should wrap the hero's child widget as its
  // own child, when `_placeholderSize` is non-null (i.e. the hero is currently
  // in its flight animation). See `startFlight`.
  bool _shouldIncludeChild = true;

  // The `shouldIncludeChildInPlaceholder` flag dictates if the child widget of
  // this hero should be included in the placeholder widget as a descendant.
  //
  // When a new hero flight animation takes place, a placeholder widget
  // needs to be built to replace the original hero widget. When
  // `shouldIncludeChildInPlaceholder` is set to true and `widget.placeholderBuilder`
  // is null, the placeholder widget will include the original hero's child
  // widget as a descendant, allowing the original element tree to be preserved.
  //
  // It is typically set to true for the *from* hero in a push transition,
  // and false otherwise.
  void startFlight({bool shouldIncludedChildInPlaceholder = false}) {
    _shouldIncludeChild = shouldIncludedChildInPlaceholder;
    assert(mounted);
    final RenderBox box = context.findRenderObject()! as RenderBox;
    assert(box.hasSize);
    setState(() {
      _placeholderSize = box.size;
    });
  }

  // When `keepPlaceholder` is true, the placeholder will continue to be shown
  // after the flight ends. Otherwise the child of the Hero will become visible
  // and its TickerMode will be re-enabled.
  //
  // This method can be safely called even when this [Hero] is currently not in
  // a flight.
  void endFlight({bool keepPlaceholder = false}) {
    if (keepPlaceholder || _placeholderSize == null) {
      return;
    }

    _placeholderSize = null;
    if (mounted) {
      // Tell the widget to rebuild if it's mounted. _placeholderSize has already
      // been updated.
      setState(() {});
    }
  }

  @override
  Widget build(BuildContext context) {
    assert(
      context.findAncestorWidgetOfExactType<Hero>() == null,
      'A Hero widget cannot be the descendant of another Hero widget.',
    );

    final bool showPlaceholder = _placeholderSize != null;

    if (showPlaceholder && widget.placeholderBuilder != null) {
      return widget.placeholderBuilder!(context, _placeholderSize!, widget.child);
    }

    if (showPlaceholder && !_shouldIncludeChild) {
      return SizedBox(width: _placeholderSize!.width, height: _placeholderSize!.height);
    }

    return SizedBox(
      width: _placeholderSize?.width,
      height: _placeholderSize?.height,
      child: Offstage(
        offstage: showPlaceholder,
        child: TickerMode(
          enabled: !showPlaceholder,
          child: KeyedSubtree(key: _key, child: widget.child),
        ),
      ),
    );
  }
}

// Everything known about a hero flight that's to be started or diverted.
class _HeroFlightManifest {
  _HeroFlightManifest({
    required this.type,
    required this.overlay,
    required this.navigatorSize,
    required this.fromRoute,
    required this.toRoute,
    required this.fromHero,
    required this.toHero,
    required this.createRectTween,
    required this.shuttleBuilder,
    required this.isUserGestureTransition,
    required this.isDiverted,
  }) : assert(fromHero.widget.tag == toHero.widget.tag);

  final HeroFlightDirection type;
  final OverlayState overlay;
  final Size navigatorSize;
  final PageRoute<dynamic> fromRoute;
  final PageRoute<dynamic> toRoute;
  final _HeroState fromHero;
  final _HeroState toHero;
  final CreateRectTween? createRectTween;
  final HeroFlightShuttleBuilder shuttleBuilder;
  final bool isUserGestureTransition;
  final bool isDiverted;

  Object get tag => fromHero.widget.tag;

  CurvedAnimation? _animation;

  Animation<double> get animation {
    return _animation ??= CurvedAnimation(
      parent: (type == HeroFlightDirection.push) ? toRoute.animation! : fromRoute.animation!,
      curve: Curves.fastOutSlowIn,
      reverseCurve: isDiverted ? null : Curves.fastOutSlowIn.flipped,
    );
  }

  Tween<Rect?> createHeroRectTween({required Rect? begin, required Rect? end}) {
    final CreateRectTween? createRectTween = toHero.widget.createRectTween ?? this.createRectTween;
    return createRectTween?.call(begin, end) ?? RectTween(begin: begin, end: end);
  }

  // The bounding box for `context`'s render object,  in `ancestorContext`'s
  // render object's coordinate space.
  static Rect _boundingBoxFor(BuildContext context, BuildContext? ancestorContext) {
    assert(ancestorContext != null);
    final RenderBox box = context.findRenderObject()! as RenderBox;
    assert(box.hasSize && box.size.isFinite);
    return MatrixUtils.transformRect(
      box.getTransformTo(ancestorContext?.findRenderObject()),
      Offset.zero & box.size,
    );
  }

  /// The bounding box of [fromHero], in [fromRoute]'s coordinate space.
  ///
  /// This property should only be accessed in [_HeroFlight.start].
  late final Rect fromHeroLocation = _boundingBoxFor(fromHero.context, fromRoute.subtreeContext);

  /// The bounding box of [toHero], in [toRoute]'s coordinate space.
  ///
  /// This property should only be accessed in [_HeroFlight.start] or
  /// [_HeroFlight.divert].
  late final Rect toHeroLocation = _boundingBoxFor(toHero.context, toRoute.subtreeContext);

  /// Whether this [_HeroFlightManifest] is valid and can be used to start or
  /// divert a [_HeroFlight].
  ///
  /// When starting or diverting a [_HeroFlight] with a brand new
  /// [_HeroFlightManifest], this flag must be checked to ensure the [RectTween]
  /// the [_HeroFlightManifest] produces does not contain coordinates that have
  /// [double.infinity] or [double.nan].
  late final bool isValid = toHeroLocation.isFinite && (isDiverted || fromHeroLocation.isFinite);

  @override
  String toString() {
    return '_HeroFlightManifest($type tag: $tag from route: ${fromRoute.settings} '
        'to route: ${toRoute.settings} with hero: $fromHero to $toHero)${isValid ? '' : ', INVALID'}';
  }

  @mustCallSuper
  void dispose() {
    _animation?.dispose();
  }
}

// Builds the in-flight hero widget.
class _HeroFlight {
  _HeroFlight(this.onFlightEnded) {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectCreated(
        library: 'package:flutter/widgets.dart',
        className: '$_HeroFlight',
        object: this,
      );
    }
    _proxyAnimation = ProxyAnimation()..addStatusListener(_handleAnimationUpdate);
  }

  final _OnFlightEnded onFlightEnded;

  late Tween<Rect?> heroRectTween;
  Widget? shuttle;

  Animation<double> _heroOpacity = kAlwaysCompleteAnimation;
  late ProxyAnimation _proxyAnimation;
  // The manifest will be available once `start` is called, throughout the
  // flight's lifecycle.
  _HeroFlightManifest? _manifest;
  _HeroFlightManifest get manifest => _manifest!;
  set manifest(_HeroFlightManifest value) {
    _manifest?.dispose();
    _manifest = value;
  }

  OverlayEntry? overlayEntry;
  bool _aborted = false;

  static final Animatable<double> _reverseTween = Tween<double>(begin: 1.0, end: 0.0);

  // The OverlayEntry WidgetBuilder callback for the hero's overlay.
  Widget _buildOverlay(BuildContext context) {
    shuttle ??= manifest.shuttleBuilder(
      context,
      manifest.animation,
      manifest.type,
      manifest.fromHero.context,
      manifest.toHero.context,
    );
    assert(shuttle != null);

    return AnimatedBuilder(
      animation: _proxyAnimation,
      child: shuttle,
      builder: (BuildContext context, Widget? child) {
        final Rect rect = heroRectTween.evaluate(_proxyAnimation)!;
        final RelativeRect offsets = RelativeRect.fromSize(rect, manifest.navigatorSize);
        return Positioned(
          top: offsets.top,
          right: offsets.right,
          bottom: offsets.bottom,
          left: offsets.left,
          child: IgnorePointer(child: FadeTransition(opacity: _heroOpacity, child: child)),
        );
      },
    );
  }

  void _performAnimationUpdate(AnimationStatus status) {
    if (!status.isAnimating) {
      _proxyAnimation.parent = null;

      assert(overlayEntry != null);
      overlayEntry!.remove();
      overlayEntry!.dispose();
      overlayEntry = null;
      // We want to keep the hero underneath the current page hidden. If
      // [AnimationStatus.completed], toHero will be the one on top and we keep
      // fromHero hidden. If [AnimationStatus.dismissed], the animation is
      // triggered but canceled before it finishes. In this case, we keep toHero
      // hidden instead.
      manifest.fromHero.endFlight(keepPlaceholder: status.isCompleted);
      manifest.toHero.endFlight(keepPlaceholder: status.isDismissed);
      onFlightEnded(this);
      _proxyAnimation.removeListener(onTick);
    }
  }

  bool _scheduledPerformAnimationUpdate = false;
  void _handleAnimationUpdate(AnimationStatus status) {
    // The animation will not finish until the user lifts their finger, so we
    // should suppress the status update if the gesture is in progress, and
    // delay it until the finger is lifted.
    if (manifest.fromRoute.navigator?.userGestureInProgress != true) {
      _performAnimationUpdate(status);
      return;
    }

    if (_scheduledPerformAnimationUpdate) {
      return;
    }

    // The `navigator` must be non-null here, or the first if clause above would
    // have returned from this method.
    final NavigatorState navigator = manifest.fromRoute.navigator!;

    void delayedPerformAnimationUpdate() {
      assert(!navigator.userGestureInProgress);
      assert(_scheduledPerformAnimationUpdate);
      _scheduledPerformAnimationUpdate = false;
      navigator.userGestureInProgressNotifier.removeListener(delayedPerformAnimationUpdate);
      _performAnimationUpdate(_proxyAnimation.status);
    }

    assert(navigator.userGestureInProgress);
    _scheduledPerformAnimationUpdate = true;
    navigator.userGestureInProgressNotifier.addListener(delayedPerformAnimationUpdate);
  }

  /// Releases resources.
  @mustCallSuper
  void dispose() {
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectDisposed(object: this);
    }
    if (overlayEntry != null) {
      overlayEntry!.remove();
      overlayEntry!.dispose();
      overlayEntry = null;
      _proxyAnimation.parent = null;
      _proxyAnimation.removeListener(onTick);
      _proxyAnimation.removeStatusListener(_handleAnimationUpdate);
    }
    _manifest?.dispose();
  }

  void onTick() {
    final RenderBox? toHeroBox =
        (!_aborted && manifest.toHero.mounted)
            ? manifest.toHero.context.findRenderObject() as RenderBox?
            : null;
    // Try to find the new origin of the toHero, if the flight isn't aborted.
    final Offset? toHeroOrigin =
        toHeroBox != null && toHeroBox.attached && toHeroBox.hasSize
            ? toHeroBox.localToGlobal(
              Offset.zero,
              ancestor: manifest.toRoute.subtreeContext?.findRenderObject() as RenderBox?,
            )
            : null;

    if (toHeroOrigin != null && toHeroOrigin.isFinite) {
      // If the new origin of toHero is available and also paintable, try to
      // update heroRectTween with it.
      if (toHeroOrigin != heroRectTween.end!.topLeft) {
        final Rect heroRectEnd = toHeroOrigin & heroRectTween.end!.size;
        heroRectTween = manifest.createHeroRectTween(begin: heroRectTween.begin, end: heroRectEnd);
      }
    } else if (_heroOpacity.isCompleted) {
      // The toHero no longer exists or it's no longer the flight's destination.
      // Continue flying while fading out.
      _heroOpacity = _proxyAnimation.drive(
        _reverseTween.chain(CurveTween(curve: Interval(_proxyAnimation.value, 1.0))),
      );
    }
    // Update _aborted for the next animation tick.
    _aborted = toHeroOrigin == null || !toHeroOrigin.isFinite;
  }

  // The simple case: we're either starting a push or a pop animation.
  void start(_HeroFlightManifest initialManifest) {
    assert(!_aborted);
    assert(() {
      final Animation<double> initial = initialManifest.animation;
      final HeroFlightDirection type = initialManifest.type;
      switch (type) {
        case HeroFlightDirection.pop:
          return initial.value == 1.0 && initialManifest.isUserGestureTransition
              // During user gesture transitions, the animation controller isn't
              // driving the reverse transition, but should still be in a previously
              // completed stage with the initial value at 1.0.
              ? initial.status == AnimationStatus.completed
              : initial.status == AnimationStatus.reverse;
        case HeroFlightDirection.push:
          return initial.value == 0.0 && initial.status == AnimationStatus.forward;
      }
    }());

    manifest = initialManifest;

    final bool shouldIncludeChildInPlaceholder;
    switch (manifest.type) {
      case HeroFlightDirection.pop:
        _proxyAnimation.parent = ReverseAnimation(manifest.animation);
        shouldIncludeChildInPlaceholder = false;
      case HeroFlightDirection.push:
        _proxyAnimation.parent = manifest.animation;
        shouldIncludeChildInPlaceholder = true;
    }

    heroRectTween = manifest.createHeroRectTween(
      begin: manifest.fromHeroLocation,
      end: manifest.toHeroLocation,
    );
    manifest.fromHero.startFlight(
      shouldIncludedChildInPlaceholder: shouldIncludeChildInPlaceholder,
    );
    manifest.toHero.startFlight();
    manifest.overlay.insert(overlayEntry = OverlayEntry(builder: _buildOverlay));
    _proxyAnimation.addListener(onTick);
  }

  // While this flight's hero was in transition a push or a pop occurred for
  // routes with the same hero. Redirect the in-flight hero to the new toRoute.
  void divert(_HeroFlightManifest newManifest) {
    assert(manifest.tag == newManifest.tag);
    if (manifest.type == HeroFlightDirection.push && newManifest.type == HeroFlightDirection.pop) {
      // A push flight was interrupted by a pop.
      assert(newManifest.animation.status == AnimationStatus.reverse);
      assert(manifest.fromHero == newManifest.toHero);
      assert(manifest.toHero == newManifest.fromHero);
      assert(manifest.fromRoute == newManifest.toRoute);
      assert(manifest.toRoute == newManifest.fromRoute);

      // The same heroRect tween is used in reverse, rather than creating
      // a new heroRect with _doCreateRectTween(heroRect.end, heroRect.begin).
      // That's because tweens like MaterialRectArcTween may create a different
      // path for swapped begin and end parameters. We want the pop flight
      // path to be the same (in reverse) as the push flight path.
      _proxyAnimation.parent = ReverseAnimation(newManifest.animation);
      heroRectTween = ReverseTween<Rect?>(heroRectTween);
    } else if (manifest.type == HeroFlightDirection.pop &&
        newManifest.type == HeroFlightDirection.push) {
      // A pop flight was interrupted by a push.
      assert(newManifest.animation.status == AnimationStatus.forward);
      assert(manifest.toHero == newManifest.fromHero);
      assert(manifest.toRoute == newManifest.fromRoute);

      _proxyAnimation.parent = newManifest.animation.drive(
        Tween<double>(begin: manifest.animation.value, end: 1.0),
      );
      if (manifest.fromHero != newManifest.toHero) {
        manifest.fromHero.endFlight(keepPlaceholder: true);
        newManifest.toHero.startFlight();
        heroRectTween = manifest.createHeroRectTween(
          begin: heroRectTween.end,
          end: newManifest.toHeroLocation,
        );
      } else {
        // TODO(hansmuller): Use ReverseTween here per github.com/flutter/flutter/pull/12203.
        heroRectTween = manifest.createHeroRectTween(
          begin: heroRectTween.end,
          end: heroRectTween.begin,
        );
      }
    } else {
      // A push or a pop flight is heading to a new route, i.e.
      // manifest.type == _HeroFlightType.push && newManifest.type == _HeroFlightType.push ||
      // manifest.type == _HeroFlightType.pop && newManifest.type == _HeroFlightType.pop
      assert(manifest.fromHero != newManifest.fromHero);
      assert(manifest.toHero != newManifest.toHero);

      heroRectTween = manifest.createHeroRectTween(
        begin: heroRectTween.evaluate(_proxyAnimation),
        end: newManifest.toHeroLocation,
      );
      shuttle = null;

      if (newManifest.type == HeroFlightDirection.pop) {
        _proxyAnimation.parent = ReverseAnimation(newManifest.animation);
      } else {
        _proxyAnimation.parent = newManifest.animation;
      }

      manifest.fromHero.endFlight(keepPlaceholder: true);
      manifest.toHero.endFlight(keepPlaceholder: true);

      // Let the heroes in each of the routes rebuild with their placeholders.
      newManifest.fromHero.startFlight(
        shouldIncludedChildInPlaceholder: newManifest.type == HeroFlightDirection.push,
      );
      newManifest.toHero.startFlight();

      // Let the transition overlay on top of the routes also rebuild since
      // we cleared the old shuttle.
      overlayEntry!.markNeedsBuild();
    }

    manifest = newManifest;
  }

  void abort() {
    _aborted = true;
  }

  @override
  String toString() {
    final RouteSettings from = manifest.fromRoute.settings;
    final RouteSettings to = manifest.toRoute.settings;
    final Object tag = manifest.tag;
    return 'HeroFlight(for: $tag, from: $from, to: $to ${_proxyAnimation.parent})';
  }
}

/// A [Navigator] observer that manages [Hero] transitions.
///
/// An instance of [HeroController] should be used in [Navigator.observers].
/// This is done automatically by [MaterialApp].
class HeroController extends NavigatorObserver {
  /// Creates a hero controller with the given [RectTween] constructor if any.
  ///
  /// The [createRectTween] argument is optional. If null, the controller uses a
  /// linear [Tween<Rect>].
  HeroController({this.createRectTween}) {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectCreated(
        library: 'package:flutter/widgets.dart',
        className: '$HeroController',
        object: this,
      );
    }
  }

  /// Used to create [RectTween]s that interpolate the position of heroes in flight.
  ///
  /// If null, the controller uses a linear [RectTween].
  final CreateRectTween? createRectTween;

  // All of the heroes that are currently in the overlay and in motion.
  // Indexed by the hero tag.
  final Map<Object, _HeroFlight> _flights = <Object, _HeroFlight>{};

  @override
  void didChangeTop(Route<dynamic> topRoute, Route<dynamic>? previousTopRoute) {
    assert(topRoute.isCurrent);
    assert(navigator != null);
    if (previousTopRoute == null) {
      return;
    }
    // Don't trigger another flight when a pop is committed as a user gesture
    // back swipe is snapped.
    if (!navigator!.userGestureInProgress) {
      _maybeStartHeroTransition(
        fromRoute: previousTopRoute,
        toRoute: topRoute,
        isUserGestureTransition: false,
      );
    }
  }

  @override
  void didStartUserGesture(Route<dynamic> route, Route<dynamic>? previousRoute) {
    assert(navigator != null);
    _maybeStartHeroTransition(
      fromRoute: route,
      toRoute: previousRoute,
      isUserGestureTransition: true,
    );
  }

  @override
  void didStopUserGesture() {
    if (navigator!.userGestureInProgress) {
      return;
    }

    // When the user gesture ends, if the user horizontal drag gesture initiated
    // the flight (i.e. the back swipe) didn't move towards the pop direction at
    // all, the animation will not play and thus the status update callback
    // _handleAnimationUpdate will never be called when the gesture finishes. In
    // this case the initiated flight needs to be manually invalidated.
    bool isInvalidFlight(_HeroFlight flight) {
      return flight.manifest.isUserGestureTransition &&
          flight.manifest.type == HeroFlightDirection.pop &&
          flight._proxyAnimation.isDismissed;
    }

    final List<_HeroFlight> invalidFlights = _flights.values
        .where(isInvalidFlight)
        .toList(growable: false);

    // Treat these invalidated flights as dismissed. Calling _handleAnimationUpdate
    // will also remove the flight from _flights.
    for (final _HeroFlight flight in invalidFlights) {
      flight._handleAnimationUpdate(AnimationStatus.dismissed);
    }
  }

  // If we're transitioning between different page routes, start a hero transition
  // after the toRoute has been laid out with its animation's value at 1.0.
  void _maybeStartHeroTransition({
    required Route<dynamic>? fromRoute,
    required Route<dynamic>? toRoute,
    required bool isUserGestureTransition,
  }) {
    if (toRoute == fromRoute ||
        toRoute is! PageRoute<dynamic> ||
        fromRoute is! PageRoute<dynamic>) {
      return;
    }
    final Animation<double> newRouteAnimation = toRoute.animation!;
    final Animation<double> oldRouteAnimation = fromRoute.animation!;
    final HeroFlightDirection flightType;
    switch ((isUserGestureTransition, oldRouteAnimation.status, newRouteAnimation.status)) {
      case (true, _, _):
      case (_, AnimationStatus.reverse, _):
        flightType = HeroFlightDirection.pop;
      case (_, _, AnimationStatus.forward):
        flightType = HeroFlightDirection.push;
      default:
        return;
    }

    // A user gesture may have already completed the pop, or we might be the initial route
    switch (flightType) {
      case HeroFlightDirection.pop:
        if (fromRoute.animation!.value == 0.0) {
          return;
        }
      case HeroFlightDirection.push:
        if (toRoute.animation!.value == 1.0) {
          return;
        }
    }

    // For pop transitions driven by a user gesture: if the "to" page has
    // maintainState = true, then the hero's final dimensions can be measured
    // immediately because their page's layout is still valid.
    if (isUserGestureTransition && flightType == HeroFlightDirection.pop && toRoute.maintainState) {
      _startHeroTransition(fromRoute, toRoute, flightType, isUserGestureTransition);
    } else {
      // Otherwise, delay measuring until the end of the next frame to allow
      // the 'to' route to build and layout.

      // Putting a route offstage changes its animation value to 1.0. Once this
      // frame completes, we'll know where the heroes in the `to` route are
      // going to end up, and the `to` route will go back onstage.
      toRoute.offstage = toRoute.animation!.value == 0.0;

      WidgetsBinding.instance.addPostFrameCallback((Duration value) {
        if (fromRoute.navigator == null || toRoute.navigator == null) {
          return;
        }
        _startHeroTransition(fromRoute, toRoute, flightType, isUserGestureTransition);
      }, debugLabel: 'HeroController.startTransition');
    }
  }

  // Find the matching pairs of heroes in from and to and either start or a new
  // hero flight, or divert an existing one.
  void _startHeroTransition(
    PageRoute<dynamic> from,
    PageRoute<dynamic> to,
    HeroFlightDirection flightType,
    bool isUserGestureTransition,
  ) {
    // If the `to` route was offstage, then we're implicitly restoring its
    // animation value back to what it was before it was "moved" offstage.
    to.offstage = false;

    final NavigatorState? navigator = this.navigator;
    final OverlayState? overlay = navigator?.overlay;
    // If the navigator or the overlay was removed before this end-of-frame
    // callback was called, then don't actually start a transition, and we don't
    // have to worry about any Hero widget we might have hidden in a previous
    // flight, or ongoing flights.
    if (navigator == null || overlay == null) {
      return;
    }

    final RenderObject? navigatorRenderObject = navigator.context.findRenderObject();

    if (navigatorRenderObject is! RenderBox) {
      assert(
        false,
        'Navigator $navigator has an invalid RenderObject type ${navigatorRenderObject.runtimeType}.',
      );
      return;
    }
    assert(navigatorRenderObject.hasSize);

    // At this point, the toHeroes may have been built and laid out for the first time.
    //
    // If `fromSubtreeContext` is null, call endFlight on all toHeroes, for good measure.
    // If `toSubtreeContext` is null abort existingFlights.
    final BuildContext? fromSubtreeContext = from.subtreeContext;
    final Map<Object, _HeroState> fromHeroes =
        fromSubtreeContext != null
            ? Hero._allHeroesFor(fromSubtreeContext, isUserGestureTransition, navigator)
            : const <Object, _HeroState>{};
    final BuildContext? toSubtreeContext = to.subtreeContext;
    final Map<Object, _HeroState> toHeroes =
        toSubtreeContext != null
            ? Hero._allHeroesFor(toSubtreeContext, isUserGestureTransition, navigator)
            : const <Object, _HeroState>{};

    for (final MapEntry<Object, _HeroState> fromHeroEntry in fromHeroes.entries) {
      final Object tag = fromHeroEntry.key;
      final _HeroState fromHero = fromHeroEntry.value;
      final _HeroState? toHero = toHeroes[tag];
      final _HeroFlight? existingFlight = _flights[tag];
      final _HeroFlightManifest? manifest =
          toHero == null
              ? null
              : _HeroFlightManifest(
                type: flightType,
                overlay: overlay,
                navigatorSize: navigatorRenderObject.size,
                fromRoute: from,
                toRoute: to,
                fromHero: fromHero,
                toHero: toHero,
                createRectTween: createRectTween,
                shuttleBuilder:
                    toHero.widget.flightShuttleBuilder ??
                    fromHero.widget.flightShuttleBuilder ??
                    _defaultHeroFlightShuttleBuilder,
                isUserGestureTransition: isUserGestureTransition,
                isDiverted: existingFlight != null,
              );

      // Only proceed with a valid manifest. Otherwise abort the existing
      // flight, and call endFlight when this for loop finishes.
      if (manifest != null && manifest.isValid) {
        toHeroes.remove(tag);
        if (existingFlight != null) {
          existingFlight.divert(manifest);
        } else {
          _flights[tag] = _HeroFlight(_handleFlightEnded)..start(manifest);
        }
      } else {
        existingFlight?.abort();
      }
    }

    // The remaining entries in toHeroes are those failed to participate in a
    // new flight (for not having a valid manifest).
    //
    // This can happen in a route pop transition when a fromHero is no longer
    // mounted, or kept alive by the [KeepAlive] mechanism but no longer visible.
    // TODO(LongCatIsLooong): resume aborted flights: https://github.com/flutter/flutter/issues/72947
    for (final _HeroState toHero in toHeroes.values) {
      toHero.endFlight();
    }
  }

  void _handleFlightEnded(_HeroFlight flight) {
    _flights.remove(flight.manifest.tag)?.dispose();
  }

  Widget _defaultHeroFlightShuttleBuilder(
    BuildContext flightContext,
    Animation<double> animation,
    HeroFlightDirection flightDirection,
    BuildContext fromHeroContext,
    BuildContext toHeroContext,
  ) {
    final Hero toHero = toHeroContext.widget as Hero;

    final MediaQueryData? toMediaQueryData = MediaQuery.maybeOf(toHeroContext);
    final MediaQueryData? fromMediaQueryData = MediaQuery.maybeOf(fromHeroContext);

    if (toMediaQueryData == null || fromMediaQueryData == null) {
      return toHero.child;
    }

    final EdgeInsets fromHeroPadding = fromMediaQueryData.padding;
    final EdgeInsets toHeroPadding = toMediaQueryData.padding;

    return AnimatedBuilder(
      animation: animation,
      builder: (BuildContext context, Widget? child) {
        return MediaQuery(
          data: toMediaQueryData.copyWith(
            padding:
                (flightDirection == HeroFlightDirection.push)
                    ? EdgeInsetsTween(
                      begin: fromHeroPadding,
                      end: toHeroPadding,
                    ).evaluate(animation)
                    : EdgeInsetsTween(
                      begin: toHeroPadding,
                      end: fromHeroPadding,
                    ).evaluate(animation),
          ),
          child: toHero.child,
        );
      },
    );
  }

  /// Releases resources.
  @mustCallSuper
  void dispose() {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectDisposed(object: this);
    }

    for (final _HeroFlight flight in _flights.values) {
      flight.dispose();
    }
  }
}

/// Enables or disables [Hero]es in the widget subtree.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=AaIASk2u1C0}
///
/// When [enabled] is false, all [Hero] widgets in this subtree will not be
/// involved in hero animations.
///
/// When [enabled] is true (the default), [Hero] widgets may be involved in
/// hero animations, as usual.
class HeroMode extends StatelessWidget {
  /// Creates a widget that enables or disables [Hero]es.
  const HeroMode({super.key, required this.child, this.enabled = true});

  /// The subtree to place inside the [HeroMode].
  final Widget child;

  /// Whether or not [Hero]es are enabled in this subtree.
  ///
  /// If this property is false, the [Hero]es in this subtree will not animate
  /// on route changes. Otherwise, they will animate as usual.
  ///
  /// Defaults to true.
  final bool enabled;

  @override
  Widget build(BuildContext context) => child;

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      FlagProperty('mode', value: enabled, ifTrue: 'enabled', ifFalse: 'disabled', showName: true),
    );
  }
}
