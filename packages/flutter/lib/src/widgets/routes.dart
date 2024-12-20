// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:ui';
/// @docImport 'package:flutter/cupertino.dart';
/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'app.dart';
/// @docImport 'form.dart';
/// @docImport 'heroes.dart';
/// @docImport 'pages.dart';
/// @docImport 'pop_scope.dart';
/// @docImport 'will_pop_scope.dart';
library;

import 'dart:async';
import 'dart:math';
import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';

import 'actions.dart';
import 'basic.dart';
import 'display_feature_sub_screen.dart';
import 'focus_manager.dart';
import 'focus_scope.dart';
import 'focus_traversal.dart';
import 'framework.dart';
import 'inherited_model.dart';
import 'modal_barrier.dart';
import 'navigator.dart';
import 'overlay.dart';
import 'page_storage.dart';
import 'primary_scroll_controller.dart';
import 'restoration.dart';
import 'scroll_controller.dart';
import 'transitions.dart';

// Examples can assume:
// late NavigatorState navigator;
// late BuildContext context;
// Future<bool> askTheUserIfTheyAreSure() async { return true; }
// abstract class MyWidget extends StatefulWidget { const MyWidget({super.key}); }
// late dynamic _myState, newValue;
// late StateSetter setState;

/// A route that displays widgets in the [Navigator]'s [Overlay].
///
/// See also:
///
///  * [Route], which documents the meaning of the `T` generic type argument.
abstract class OverlayRoute<T> extends Route<T> {
  /// Creates a route that knows how to interact with an [Overlay].
  OverlayRoute({super.settings, super.requestFocus});

  /// Subclasses should override this getter to return the builders for the overlay.
  @factory
  Iterable<OverlayEntry> createOverlayEntries();

  @override
  List<OverlayEntry> get overlayEntries => _overlayEntries;
  final List<OverlayEntry> _overlayEntries = <OverlayEntry>[];

  @override
  void install() {
    assert(_overlayEntries.isEmpty);
    _overlayEntries.addAll(createOverlayEntries());
    super.install();
  }

  /// Controls whether [didPop] calls [NavigatorState.finalizeRoute].
  ///
  /// If true, this route removes its overlay entries during [didPop].
  /// Subclasses can override this getter if they want to delay finalization
  /// (for example to animate the route's exit before removing it from the
  /// overlay).
  ///
  /// Subclasses that return false from [finishedWhenPopped] are responsible for
  /// calling [NavigatorState.finalizeRoute] themselves.
  @protected
  bool get finishedWhenPopped => true;

  @override
  bool didPop(T? result) {
    final bool returnValue = super.didPop(result);
    assert(returnValue);
    if (finishedWhenPopped) {
      navigator!.finalizeRoute(this);
    }
    return returnValue;
  }

  @override
  void dispose() {
    for (final OverlayEntry entry in _overlayEntries) {
      entry.dispose();
    }
    _overlayEntries.clear();
    super.dispose();
  }
}

/// A route with entrance and exit transitions.
///
/// See also:
///
///  * [Route], which documents the meaning of the `T` generic type argument.
abstract class TransitionRoute<T> extends OverlayRoute<T> implements PredictiveBackRoute {
  /// Creates a route that animates itself when it is pushed or popped.
  TransitionRoute({super.settings, super.requestFocus});

  /// This future completes only once the transition itself has finished, after
  /// the overlay entries have been removed from the navigator's overlay.
  ///
  /// This future completes once the animation has been dismissed. That will be
  /// after [popped], because [popped] typically completes before the animation
  /// even starts, as soon as the route is popped.
  Future<T?> get completed => _transitionCompleter.future;
  final Completer<T?> _transitionCompleter = Completer<T?>();

  /// Handle to the performance mode request.
  ///
  /// When the route is animating, the performance mode is requested. It is then
  /// disposed when the animation ends. Requesting [DartPerformanceMode.latency]
  /// indicates to the engine that the transition is latency sensitive and to delay
  /// non-essential work while this handle is active.
  PerformanceModeRequestHandle? _performanceModeRequestHandle;

  /// {@template flutter.widgets.TransitionRoute.transitionDuration}
  /// The duration the transition going forwards.
  ///
  /// See also:
  ///
  /// * [reverseTransitionDuration], which controls the duration of the
  /// transition when it is in reverse.
  /// {@endtemplate}
  Duration get transitionDuration;

  /// {@template flutter.widgets.TransitionRoute.reverseTransitionDuration}
  /// The duration the transition going in reverse.
  ///
  /// By default, the reverse transition duration is set to the value of
  /// the forwards [transitionDuration].
  /// {@endtemplate}
  Duration get reverseTransitionDuration => transitionDuration;

  /// {@template flutter.widgets.TransitionRoute.opaque}
  /// Whether the route obscures previous routes when the transition is complete.
  ///
  /// When an opaque route's entrance transition is complete, the routes behind
  /// the opaque route will not be built to save resources.
  /// {@endtemplate}
  bool get opaque;

  /// {@template flutter.widgets.TransitionRoute.allowSnapshotting}
  /// Whether the route transition will prefer to animate a snapshot of the
  /// entering/exiting routes.
  ///
  /// When this value is true, certain route transitions (such as the Android
  /// zoom page transition) will snapshot the entering and exiting routes.
  /// These snapshots are then animated in place of the underlying widgets to
  /// improve performance of the transition.
  ///
  /// Generally this means that animations that occur on the entering/exiting
  /// route while the route animation plays may appear frozen - unless they
  /// are a hero animation or something that is drawn in a separate overlay.
  /// {@endtemplate}
  bool get allowSnapshotting => true;

  // This ensures that if we got to the dismissed state while still current,
  // we will still be disposed when we are eventually popped.
  //
  // This situation arises when dealing with the Cupertino dismiss gesture.
  @override
  bool get finishedWhenPopped => _controller!.isDismissed && !_popFinalized;

  bool _popFinalized = false;

  /// The animation that drives the route's transition and the previous route's
  /// forward transition.
  Animation<double>? get animation => _animation;
  Animation<double>? _animation;

  /// The animation controller that the route uses to drive the transitions.
  ///
  /// The animation itself is exposed by the [animation] property.
  @protected
  AnimationController? get controller => _controller;
  AnimationController? _controller;

  /// The animation for the route being pushed on top of this route. This
  /// animation lets this route coordinate with the entrance and exit transition
  /// of route pushed on top of this route.
  Animation<double>? get secondaryAnimation => _secondaryAnimation;
  final ProxyAnimation _secondaryAnimation = ProxyAnimation(kAlwaysDismissedAnimation);

  /// Whether to takeover the [controller] created by [createAnimationController].
  ///
  /// If true, this route will call [AnimationController.dispose] when the
  /// controller is no longer needed.
  /// If false, the controller should be disposed by whoever owned it.
  ///
  /// It defaults to `true`.
  bool willDisposeAnimationController = true;

  /// Returns true if the transition has completed.
  ///
  /// It is equivalent to whether the future returned by [completed] has
  /// completed.
  ///
  /// This method only works if assert is enabled. Otherwise it always returns
  /// false.
  @protected
  bool debugTransitionCompleted() {
    bool disposed = false;
    assert(() {
      disposed = _transitionCompleter.isCompleted;
      return true;
    }());
    return disposed;
  }

  /// Called to create the animation controller that will drive the transitions to
  /// this route from the previous one, and back to the previous route from this
  /// one.
  ///
  /// The returned controller will be disposed by [AnimationController.dispose]
  /// if the [willDisposeAnimationController] is `true`.
  AnimationController createAnimationController() {
    assert(!debugTransitionCompleted(), 'Cannot reuse a $runtimeType after disposing it.');
    final Duration duration = transitionDuration;
    final Duration reverseDuration = reverseTransitionDuration;
    return AnimationController(
      duration: duration,
      reverseDuration: reverseDuration,
      debugLabel: debugLabel,
      vsync: navigator!,
    );
  }

  /// Called to create the animation that exposes the current progress of
  /// the transition controlled by the animation controller created by
  /// [createAnimationController()].
  Animation<double> createAnimation() {
    assert(!debugTransitionCompleted(), 'Cannot reuse a $runtimeType after disposing it.');
    assert(_controller != null);
    return _controller!.view;
  }

  Simulation? _simulation;

  /// Creates the simulation that drives the transition animation for this route.
  ///
  /// By default, this method returns null, indicating that the route doesn't
  /// use simulations, but initiates the transition by calling either
  /// [AnimationController.forward] or [AnimationController.reverse] with
  /// [transitionDuration] and the controller's curve.
  ///
  /// Subclasses can override this method to return a non-null [Simulation]. In
  /// this case, the [controller] will instead use the provided simulation to
  /// animate the transition using [AnimationController.animateWith] or
  /// [AnimationController.animateBackWith], and the [Simulation.x] is forwarded
  /// to the value of [animation]. The [controller]'s curve and
  /// [transitionDuration] are ignored.
  ///
  /// This method is invoked each time the navigator pushes or pops this route.
  /// The `forward` parameter indicates the direction of the transition: true when
  /// the route is pushed, and false when it is popped.
  Simulation? createSimulation({required bool forward}) {
    assert(
      transitionDuration >= Duration.zero,
      'The `duration` must be positive for a non-simulation animation. Received $transitionDuration.',
    );
    return null;
  }

  Simulation? _createSimulationAndVerify({required bool forward}) {
    final Simulation? simulation = createSimulation(forward: forward);
    assert(
      transitionDuration >= Duration.zero,
      "The `duration` must be positive for an animation that doesn't use simulation. "
      'Either set `transitionDuration` or set `createSimulation`. '
      'Received $transitionDuration.',
    );
    return simulation;
  }

  T? _result;

  void _handleStatusChanged(AnimationStatus status) {
    switch (status) {
      case AnimationStatus.completed:
        if (overlayEntries.isNotEmpty) {
          overlayEntries.first.opaque = opaque;
        }
        _performanceModeRequestHandle?.dispose();
        _performanceModeRequestHandle = null;
      case AnimationStatus.forward:
      case AnimationStatus.reverse:
        if (overlayEntries.isNotEmpty) {
          overlayEntries.first.opaque = false;
        }
        _performanceModeRequestHandle ??= SchedulerBinding.instance.requestPerformanceMode(
          ui.DartPerformanceMode.latency,
        );
      case AnimationStatus.dismissed:
        // We might still be an active route if a subclass is controlling the
        // transition and hits the dismissed status. For example, the iOS
        // back gesture drives this animation to the dismissed status before
        // removing the route and disposing it.
        if (!isActive) {
          navigator!.finalizeRoute(this);
          _popFinalized = true;
          _performanceModeRequestHandle?.dispose();
          _performanceModeRequestHandle = null;
        }
    }
  }

  @override
  void install() {
    assert(!debugTransitionCompleted(), 'Cannot install a $runtimeType after disposing it.');
    _controller = createAnimationController();
    assert(_controller != null, '$runtimeType.createAnimationController() returned null.');
    _animation = createAnimation()..addStatusListener(_handleStatusChanged);
    assert(_animation != null, '$runtimeType.createAnimation() returned null.');
    super.install();
    if (_animation!.isCompleted && overlayEntries.isNotEmpty) {
      overlayEntries.first.opaque = opaque;
    }
  }

  @override
  TickerFuture didPush() {
    assert(
      _controller != null,
      '$runtimeType.didPush called before calling install() or after calling dispose().',
    );
    assert(!debugTransitionCompleted(), 'Cannot reuse a $runtimeType after disposing it.');
    super.didPush();
    _simulation = _createSimulationAndVerify(forward: true);
    if (_simulation == null) {
      return _controller!.forward();
    } else {
      return _controller!.animateWith(_simulation!);
    }
  }

  @override
  void didAdd() {
    assert(
      _controller != null,
      '$runtimeType.didPush called before calling install() or after calling dispose().',
    );
    assert(!debugTransitionCompleted(), 'Cannot reuse a $runtimeType after disposing it.');
    super.didAdd();
    _controller!.value = _controller!.upperBound;
  }

  @override
  void didReplace(Route<dynamic>? oldRoute) {
    assert(
      _controller != null,
      '$runtimeType.didReplace called before calling install() or after calling dispose().',
    );
    assert(!debugTransitionCompleted(), 'Cannot reuse a $runtimeType after disposing it.');
    if (oldRoute is TransitionRoute) {
      _controller!.value = oldRoute._controller!.value;
    }
    super.didReplace(oldRoute);
  }

  @override
  bool didPop(T? result) {
    assert(
      _controller != null,
      '$runtimeType.didPop called before calling install() or after calling dispose().',
    );
    assert(!_transitionCompleter.isCompleted, 'Cannot reuse a $runtimeType after disposing it.');
    _result = result;
    _simulation = _createSimulationAndVerify(forward: false);
    if (_simulation == null) {
      _controller!.reverse();
    } else {
      _controller!.animateBackWith(_simulation!);
    }
    return super.didPop(result);
  }

  @override
  void didPopNext(Route<dynamic> nextRoute) {
    assert(
      _controller != null,
      '$runtimeType.didPopNext called before calling install() or after calling dispose().',
    );
    assert(!debugTransitionCompleted(), 'Cannot reuse a $runtimeType after disposing it.');
    _updateSecondaryAnimation(nextRoute);
    super.didPopNext(nextRoute);
  }

  @override
  void didChangeNext(Route<dynamic>? nextRoute) {
    assert(
      _controller != null,
      '$runtimeType.didChangeNext called before calling install() or after calling dispose().',
    );
    assert(!debugTransitionCompleted(), 'Cannot reuse a $runtimeType after disposing it.');
    _updateSecondaryAnimation(nextRoute);
    super.didChangeNext(nextRoute);
  }

  // A callback method that disposes existing train hopping animation and
  // removes its listener.
  //
  // This property is non-null if there is a train hopping in progress, and the
  // caller must reset this property to null after it is called.
  VoidCallback? _trainHoppingListenerRemover;

  void _updateSecondaryAnimation(Route<dynamic>? nextRoute) {
    // There is an existing train hopping in progress. Unfortunately, we cannot
    // dispose current train hopping animation until we replace it with a new
    // animation.
    final VoidCallback? previousTrainHoppingListenerRemover = _trainHoppingListenerRemover;
    _trainHoppingListenerRemover = null;

    if (nextRoute is TransitionRoute<dynamic> &&
        canTransitionTo(nextRoute) &&
        nextRoute.canTransitionFrom(this)) {
      final Animation<double>? current = _secondaryAnimation.parent;
      if (current != null) {
        final Animation<double> currentTrain =
            (current is TrainHoppingAnimation ? current.currentTrain : current)!;
        final Animation<double> nextTrain = nextRoute._animation!;
        if (currentTrain.value == nextTrain.value || !nextTrain.isAnimating) {
          _setSecondaryAnimation(nextTrain, nextRoute.completed);
        } else {
          // Two trains animate at different values. We have to do train hopping.
          // There are three possibilities of train hopping:
          //  1. We hop on the nextTrain when two trains meet in the middle using
          //     TrainHoppingAnimation.
          //  2. There is no chance to hop on nextTrain because two trains never
          //     cross each other. We have to directly set the animation to
          //     nextTrain once the nextTrain stops animating.
          //  3. A new _updateSecondaryAnimation is called before train hopping
          //     finishes. We leave a listener remover for the next call to
          //     properly clean up the existing train hopping.
          TrainHoppingAnimation? newAnimation;
          void jumpOnAnimationEnd(AnimationStatus status) {
            if (!status.isAnimating) {
              // The nextTrain has stopped animating without train hopping.
              // Directly sets the secondary animation and disposes the
              // TrainHoppingAnimation.
              _setSecondaryAnimation(nextTrain, nextRoute.completed);
              if (_trainHoppingListenerRemover != null) {
                _trainHoppingListenerRemover!();
                _trainHoppingListenerRemover = null;
              }
            }
          }

          _trainHoppingListenerRemover = () {
            nextTrain.removeStatusListener(jumpOnAnimationEnd);
            newAnimation?.dispose();
          };
          nextTrain.addStatusListener(jumpOnAnimationEnd);
          newAnimation = TrainHoppingAnimation(
            currentTrain,
            nextTrain,
            onSwitchedTrain: () {
              assert(_secondaryAnimation.parent == newAnimation);
              assert(newAnimation!.currentTrain == nextRoute._animation);
              // We can hop on the nextTrain, so we don't need to listen to
              // whether the nextTrain has stopped.
              _setSecondaryAnimation(newAnimation!.currentTrain, nextRoute.completed);
              if (_trainHoppingListenerRemover != null) {
                _trainHoppingListenerRemover!();
                _trainHoppingListenerRemover = null;
              }
            },
          );
          _setSecondaryAnimation(newAnimation, nextRoute.completed);
        }
      } else {
        _setSecondaryAnimation(nextRoute._animation, nextRoute.completed);
      }
    } else {
      _setSecondaryAnimation(kAlwaysDismissedAnimation);
    }
    // Finally, we dispose any previous train hopping animation because it
    // has been successfully updated at this point.
    previousTrainHoppingListenerRemover?.call();
  }

  void _setSecondaryAnimation(Animation<double>? animation, [Future<dynamic>? disposed]) {
    _secondaryAnimation.parent = animation;
    // Releases the reference to the next route's animation when that route
    // is disposed.
    disposed?.then((dynamic _) {
      if (_secondaryAnimation.parent == animation) {
        _secondaryAnimation.parent = kAlwaysDismissedAnimation;
        if (animation is TrainHoppingAnimation) {
          animation.dispose();
        }
      }
    });
  }

  /// Returns true if this route supports a transition animation that runs
  /// when [nextRoute] is pushed on top of it or when [nextRoute] is popped
  /// off of it.
  ///
  /// Subclasses can override this method to restrict the set of routes they
  /// need to coordinate transitions with.
  ///
  /// If true, and `nextRoute.canTransitionFrom()` is true, then the
  /// [ModalRoute.buildTransitions] `secondaryAnimation` will run from 0.0 - 1.0
  /// when [nextRoute] is pushed on top of this one. Similarly, if
  /// the [nextRoute] is popped off of this route, the
  /// `secondaryAnimation` will run from 1.0 - 0.0.
  ///
  /// If false, this route's [ModalRoute.buildTransitions] `secondaryAnimation` parameter
  /// value will be [kAlwaysDismissedAnimation]. In other words, this route
  /// will not animate when [nextRoute] is pushed on top of it or when
  /// [nextRoute] is popped off of it.
  ///
  /// Returns true by default.
  ///
  /// See also:
  ///
  ///  * [canTransitionFrom], which must be true for [nextRoute] for the
  ///    [ModalRoute.buildTransitions] `secondaryAnimation` to run.
  bool canTransitionTo(TransitionRoute<dynamic> nextRoute) => true;

  /// Returns true if [previousRoute] should animate when this route
  /// is pushed on top of it or when then this route is popped off of it.
  ///
  /// Subclasses can override this method to restrict the set of routes they
  /// need to coordinate transitions with.
  ///
  /// If true, and `previousRoute.canTransitionTo()` is true, then the
  /// previous route's [ModalRoute.buildTransitions] `secondaryAnimation` will
  /// run from 0.0 - 1.0 when this route is pushed on top of
  /// it. Similarly, if this route is popped off of [previousRoute]
  /// the previous route's `secondaryAnimation` will run from 1.0 - 0.0.
  ///
  /// If false, then the previous route's [ModalRoute.buildTransitions]
  /// `secondaryAnimation` value will be kAlwaysDismissedAnimation. In
  /// other words [previousRoute] will not animate when this route is
  /// pushed on top of it or when then this route is popped off of it.
  ///
  /// Returns true by default.
  ///
  /// See also:
  ///
  ///  * [canTransitionTo], which must be true for [previousRoute] for its
  ///    [ModalRoute.buildTransitions] `secondaryAnimation` to run.
  bool canTransitionFrom(TransitionRoute<dynamic> previousRoute) => true;

  // Begin PredictiveBackRoute.

  @override
  void handleStartBackGesture({double progress = 0.0}) {
    assert(isCurrent);
    _controller?.value = progress;
    navigator?.didStartUserGesture();
  }

  @override
  void handleUpdateBackGestureProgress({required double progress}) {
    // If some other navigation happened during this gesture, don't mess with
    // the transition anymore.
    if (!isCurrent) {
      return;
    }
    _controller?.value = progress;
  }

  @override
  void handleCancelBackGesture() {
    _handleDragEnd(animateForward: true);
  }

  @override
  void handleCommitBackGesture() {
    _handleDragEnd(animateForward: false);
  }

  void _handleDragEnd({required bool animateForward}) {
    if (isCurrent) {
      if (animateForward) {
        // The closer the panel is to dismissing, the shorter the animation is.
        // We want to cap the animation time, but we want to use a linear curve
        // to determine it.
        // These values were eyeballed to match the native predictive back
        // animation on a Pixel 2 running Android API 34.
        final int droppedPageForwardAnimationTime = min(
          ui.lerpDouble(800, 0, _controller!.value)!.floor(),
          300,
        );
        _controller?.animateTo(
          1.0,
          duration: Duration(milliseconds: droppedPageForwardAnimationTime),
          curve: Curves.fastLinearToSlowEaseIn,
        );
      } else {
        // This route is destined to pop at this point. Reuse navigator's pop.
        navigator?.pop();

        // The popping may have finished inline if already at the target destination.
        if (_controller?.isAnimating ?? false) {
          // Otherwise, use a custom popping animation duration and curve.
          final int droppedPageBackAnimationTime =
              ui.lerpDouble(0, 800, _controller!.value)!.floor();
          _controller!.animateBack(
            0.0,
            duration: Duration(milliseconds: droppedPageBackAnimationTime),
            curve: Curves.fastLinearToSlowEaseIn,
          );
        }
      }
    }

    if (_controller?.isAnimating ?? false) {
      // Keep the userGestureInProgress in true state since AndroidBackGesturePageTransitionsBuilder
      // depends on userGestureInProgress.
      late final AnimationStatusListener animationStatusCallback;
      animationStatusCallback = (AnimationStatus status) {
        navigator?.didStopUserGesture();
        _controller!.removeStatusListener(animationStatusCallback);
      };
      _controller!.addStatusListener(animationStatusCallback);
    } else {
      navigator?.didStopUserGesture();
    }
  }

  // End PredictiveBackRoute.

  @override
  void dispose() {
    assert(!_transitionCompleter.isCompleted, 'Cannot dispose a $runtimeType twice.');
    assert(!debugTransitionCompleted(), 'Cannot dispose a $runtimeType twice.');
    _animation?.removeStatusListener(_handleStatusChanged);
    _performanceModeRequestHandle?.dispose();
    _performanceModeRequestHandle = null;
    if (willDisposeAnimationController) {
      _controller?.dispose();
    }
    _transitionCompleter.complete(_result);
    super.dispose();
  }

  /// A short description of this route useful for debugging.
  String get debugLabel => objectRuntimeType(this, 'TransitionRoute');

  @override
  String toString() => '${objectRuntimeType(this, 'TransitionRoute')}(animation: $_controller)';
}

/// An interface for a route that supports predictive back gestures.
///
/// See also:
///
///  * [PredictiveBackPageTransitionsBuilder], which builds page transitions for
///    predictive back.
abstract interface class PredictiveBackRoute {
  /// Whether this route is the top-most route on the navigator.
  bool get isCurrent;

  /// Whether a pop gesture can be started by the user for this route.
  bool get popGestureEnabled;

  /// Handles a predictive back gesture starting.
  ///
  /// The `progress` parameter indicates the progress of the gesture from 0.0 to
  /// 1.0, as in [PredictiveBackEvent.progress].
  void handleStartBackGesture({double progress = 0.0});

  /// Handles a predictive back gesture updating as the user drags across the
  /// screen.
  ///
  /// The `progress` parameter indicates the progress of the gesture from 0.0 to
  /// 1.0, as in [PredictiveBackEvent.progress].
  void handleUpdateBackGestureProgress({required double progress});

  /// Handles a predictive back gesture ending successfully.
  void handleCommitBackGesture();

  /// Handles a predictive back gesture ending in cancellation.
  void handleCancelBackGesture();
}

/// An entry in the history of a [LocalHistoryRoute].
class LocalHistoryEntry {
  /// Creates an entry in the history of a [LocalHistoryRoute].
  ///
  /// The [impliesAppBarDismissal] defaults to true if not provided.
  LocalHistoryEntry({this.onRemove, this.impliesAppBarDismissal = true});

  /// Called when this entry is removed from the history of its associated [LocalHistoryRoute].
  final VoidCallback? onRemove;

  LocalHistoryRoute<dynamic>? _owner;

  /// Whether an [AppBar] in the route this entry belongs to should
  /// automatically add a back button or close button.
  ///
  /// Defaults to true.
  final bool impliesAppBarDismissal;

  /// Remove this entry from the history of its associated [LocalHistoryRoute].
  void remove() {
    _owner?.removeLocalHistoryEntry(this);
    assert(_owner == null);
  }

  void _notifyRemoved() {
    onRemove?.call();
  }
}

/// A mixin used by routes to handle back navigations internally by popping a list.
///
/// When a [Navigator] is instructed to pop, the current route is given an
/// opportunity to handle the pop internally. A [LocalHistoryRoute] handles the
/// pop internally if its list of local history entries is non-empty. Rather
/// than being removed as the current route, the most recent [LocalHistoryEntry]
/// is removed from the list and its [LocalHistoryEntry.onRemove] is called.
///
/// See also:
///
///  * [Route], which documents the meaning of the `T` generic type argument.
mixin LocalHistoryRoute<T> on Route<T> {
  List<LocalHistoryEntry>? _localHistory;
  int _entriesImpliesAppBarDismissal = 0;

  /// Adds a local history entry to this route.
  ///
  /// When asked to pop, if this route has any local history entries, this route
  /// will handle the pop internally by removing the most recently added local
  /// history entry.
  ///
  /// The given local history entry must not already be part of another local
  /// history route.
  ///
  /// {@tool snippet}
  ///
  /// The following example is an app with 2 pages: `HomePage` and `SecondPage`.
  /// The `HomePage` can navigate to the `SecondPage`.
  ///
  /// The `SecondPage` uses a [LocalHistoryEntry] to implement local navigation
  /// within that page. Pressing 'show rectangle' displays a red rectangle and
  /// adds a local history entry. At that point, pressing the '< back' button
  /// pops the latest route, which is the local history entry, and the red
  /// rectangle disappears. Pressing the '< back' button a second time
  /// once again pops the latest route, which is the `SecondPage`, itself.
  /// Therefore, the second press navigates back to the `HomePage`.
  ///
  /// ```dart
  /// class App extends StatelessWidget {
  ///   const App({super.key});
  ///
  ///   @override
  ///   Widget build(BuildContext context) {
  ///     return MaterialApp(
  ///       initialRoute: '/',
  ///       routes: <String, WidgetBuilder>{
  ///         '/': (BuildContext context) => const HomePage(),
  ///         '/second_page': (BuildContext context) => const SecondPage(),
  ///       },
  ///     );
  ///   }
  /// }
  ///
  /// class HomePage extends StatefulWidget {
  ///   const HomePage({super.key});
  ///
  ///   @override
  ///   State<HomePage> createState() => _HomePageState();
  /// }
  ///
  /// class _HomePageState extends State<HomePage> {
  ///   @override
  ///   Widget build(BuildContext context) {
  ///     return Scaffold(
  ///       body: Center(
  ///         child: Column(
  ///           mainAxisSize: MainAxisSize.min,
  ///           children: <Widget>[
  ///             const Text('HomePage'),
  ///             // Press this button to open the SecondPage.
  ///             ElevatedButton(
  ///               child: const Text('Second Page >'),
  ///               onPressed: () {
  ///                 Navigator.pushNamed(context, '/second_page');
  ///               },
  ///             ),
  ///           ],
  ///         ),
  ///       ),
  ///     );
  ///   }
  /// }
  ///
  /// class SecondPage extends StatefulWidget {
  ///   const SecondPage({super.key});
  ///
  ///   @override
  ///   State<SecondPage> createState() => _SecondPageState();
  /// }
  ///
  /// class _SecondPageState extends State<SecondPage> {
  ///
  ///   bool _showRectangle = false;
  ///
  ///   Future<void> _navigateLocallyToShowRectangle() async {
  ///     // This local history entry essentially represents the display of the red
  ///     // rectangle. When this local history entry is removed, we hide the red
  ///     // rectangle.
  ///     setState(() => _showRectangle = true);
  ///     ModalRoute.of(context)?.addLocalHistoryEntry(
  ///         LocalHistoryEntry(
  ///             onRemove: () {
  ///               // Hide the red rectangle.
  ///               setState(() => _showRectangle = false);
  ///             }
  ///         )
  ///     );
  ///   }
  ///
  ///   @override
  ///   Widget build(BuildContext context) {
  ///     final Widget localNavContent = _showRectangle
  ///       ? Container(
  ///           width: 100.0,
  ///           height: 100.0,
  ///           color: Colors.red,
  ///         )
  ///       : ElevatedButton(
  ///           onPressed: _navigateLocallyToShowRectangle,
  ///           child: const Text('Show Rectangle'),
  ///         );
  ///
  ///     return Scaffold(
  ///       body: Center(
  ///         child: Column(
  ///           mainAxisAlignment: MainAxisAlignment.center,
  ///           children: <Widget>[
  ///             localNavContent,
  ///             ElevatedButton(
  ///               child: const Text('< Back'),
  ///               onPressed: () {
  ///                 // Pop a route. If this is pressed while the red rectangle is
  ///                 // visible then it will pop our local history entry, which
  ///                 // will hide the red rectangle. Otherwise, the SecondPage will
  ///                 // navigate back to the HomePage.
  ///                 Navigator.of(context).pop();
  ///               },
  ///             ),
  ///           ],
  ///         ),
  ///       ),
  ///     );
  ///   }
  /// }
  /// ```
  /// {@end-tool}
  void addLocalHistoryEntry(LocalHistoryEntry entry) {
    assert(entry._owner == null);
    entry._owner = this;
    _localHistory ??= <LocalHistoryEntry>[];
    final bool wasEmpty = _localHistory!.isEmpty;
    _localHistory!.add(entry);
    bool internalStateChanged = false;
    if (entry.impliesAppBarDismissal) {
      internalStateChanged = _entriesImpliesAppBarDismissal == 0;
      _entriesImpliesAppBarDismissal += 1;
    }
    if (wasEmpty || internalStateChanged) {
      changedInternalState();
    }
  }

  /// Remove a local history entry from this route.
  ///
  /// The entry's [LocalHistoryEntry.onRemove] callback, if any, will be called
  /// synchronously.
  void removeLocalHistoryEntry(LocalHistoryEntry entry) {
    assert(entry._owner == this);
    assert(_localHistory!.contains(entry));
    bool internalStateChanged = false;
    if (_localHistory!.remove(entry) && entry.impliesAppBarDismissal) {
      _entriesImpliesAppBarDismissal -= 1;
      internalStateChanged = _entriesImpliesAppBarDismissal == 0;
    }
    entry._owner = null;
    entry._notifyRemoved();
    if (_localHistory!.isEmpty || internalStateChanged) {
      assert(_entriesImpliesAppBarDismissal == 0);
      if (SchedulerBinding.instance.schedulerPhase == SchedulerPhase.persistentCallbacks) {
        // The local history might be removed as a result of disposing inactive
        // elements during finalizeTree. The state is locked at this moment, and
        // we can only notify state has changed in the next frame.
        SchedulerBinding.instance.addPostFrameCallback((Duration duration) {
          if (isActive) {
            changedInternalState();
          }
        }, debugLabel: 'LocalHistoryRoute.changedInternalState');
      } else {
        changedInternalState();
      }
    }
  }

  @Deprecated(
    'Use popDisposition instead. '
    'This feature was deprecated after v3.12.0-1.0.pre.',
  )
  @override
  Future<RoutePopDisposition> willPop() async {
    if (willHandlePopInternally) {
      return RoutePopDisposition.pop;
    }
    return super.willPop();
  }

  @override
  RoutePopDisposition get popDisposition {
    if (willHandlePopInternally) {
      return RoutePopDisposition.pop;
    }
    return super.popDisposition;
  }

  @override
  bool didPop(T? result) {
    if (_localHistory != null && _localHistory!.isNotEmpty) {
      final LocalHistoryEntry entry = _localHistory!.removeLast();
      assert(entry._owner == this);
      entry._owner = null;
      entry._notifyRemoved();
      bool internalStateChanged = false;
      if (entry.impliesAppBarDismissal) {
        _entriesImpliesAppBarDismissal -= 1;
        internalStateChanged = _entriesImpliesAppBarDismissal == 0;
      }
      if (_localHistory!.isEmpty || internalStateChanged) {
        changedInternalState();
      }
      return false;
    }
    return super.didPop(result);
  }

  @override
  bool get willHandlePopInternally {
    return _localHistory != null && _localHistory!.isNotEmpty;
  }
}

class _DismissModalAction extends DismissAction {
  _DismissModalAction(this.context);

  final BuildContext context;

  @override
  bool isEnabled(DismissIntent intent) {
    final ModalRoute<dynamic> route = ModalRoute.of<dynamic>(context)!;
    return route.barrierDismissible;
  }

  @override
  Object invoke(DismissIntent intent) {
    return Navigator.of(context).maybePop();
  }
}

enum _ModalRouteAspect {
  /// Specifies the aspect corresponding to [ModalRoute.isCurrent].
  isCurrent,

  /// Specifies the aspect corresponding to [ModalRoute.canPop].
  canPop,

  /// Specifies the aspect corresponding to [ModalRoute.settings].
  settings,
}

class _ModalScopeStatus extends InheritedModel<_ModalRouteAspect> {
  const _ModalScopeStatus({
    required this.isCurrent,
    required this.canPop,
    required this.impliesAppBarDismissal,
    required this.route,
    required super.child,
  });

  final bool isCurrent;
  final bool canPop;
  final bool impliesAppBarDismissal;
  final Route<dynamic> route;

  @override
  bool updateShouldNotify(_ModalScopeStatus old) {
    return isCurrent != old.isCurrent ||
        canPop != old.canPop ||
        impliesAppBarDismissal != old.impliesAppBarDismissal ||
        route != old.route;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder description) {
    super.debugFillProperties(description);
    description.add(
      FlagProperty('isCurrent', value: isCurrent, ifTrue: 'active', ifFalse: 'inactive'),
    );
    description.add(FlagProperty('canPop', value: canPop, ifTrue: 'can pop'));
    description.add(
      FlagProperty(
        'impliesAppBarDismissal',
        value: impliesAppBarDismissal,
        ifTrue: 'implies app bar dismissal',
      ),
    );
  }

  @override
  bool updateShouldNotifyDependent(
    covariant _ModalScopeStatus oldWidget,
    Set<_ModalRouteAspect> dependencies,
  ) {
    return dependencies.any(
      (_ModalRouteAspect dependency) => switch (dependency) {
        _ModalRouteAspect.isCurrent => isCurrent != oldWidget.isCurrent,
        _ModalRouteAspect.canPop => canPop != oldWidget.canPop,
        _ModalRouteAspect.settings => route.settings != oldWidget.route.settings,
      },
    );
  }
}

class _ModalScope<T> extends StatefulWidget {
  const _ModalScope({super.key, required this.route});

  final ModalRoute<T> route;

  @override
  _ModalScopeState<T> createState() => _ModalScopeState<T>();
}

class _ModalScopeState<T> extends State<_ModalScope<T>> {
  // We cache the result of calling the route's buildPage, and clear the cache
  // whenever the dependencies change. This implements the contract described in
  // the documentation for buildPage, namely that it gets called once, unless
  // something like a ModalRoute.of() dependency triggers an update.
  Widget? _page;

  // This is the combination of the two animations for the route.
  late Listenable _listenable;

  /// The node this scope will use for its root [FocusScope] widget.
  final FocusScopeNode focusScopeNode = FocusScopeNode(debugLabel: '$_ModalScopeState Focus Scope');
  final ScrollController primaryScrollController = ScrollController();

  @override
  void initState() {
    super.initState();
    final List<Listenable> animations = <Listenable>[
      if (widget.route.animation != null) widget.route.animation!,
      if (widget.route.secondaryAnimation != null) widget.route.secondaryAnimation!,
    ];
    _listenable = Listenable.merge(animations);
  }

  @override
  void didUpdateWidget(_ModalScope<T> oldWidget) {
    super.didUpdateWidget(oldWidget);
    assert(widget.route == oldWidget.route);
    _updateFocusScopeNode();
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    _page = null;
    _updateFocusScopeNode();
  }

  void _updateFocusScopeNode() {
    final TraversalEdgeBehavior traversalEdgeBehavior;
    final ModalRoute<T> route = widget.route;
    if (route.traversalEdgeBehavior != null) {
      traversalEdgeBehavior = route.traversalEdgeBehavior!;
    } else {
      traversalEdgeBehavior = route.navigator!.widget.routeTraversalEdgeBehavior;
    }
    focusScopeNode.traversalEdgeBehavior = traversalEdgeBehavior;
    if (route.isCurrent && _shouldRequestFocus) {
      route.navigator!.focusNode.enclosingScope?.setFirstFocus(focusScopeNode);
    }
  }

  void _forceRebuildPage() {
    setState(() {
      _page = null;
    });
  }

  @override
  void dispose() {
    focusScopeNode.dispose();
    primaryScrollController.dispose();
    super.dispose();
  }

  bool get _shouldIgnoreFocusRequest {
    return widget.route.animation?.status == AnimationStatus.reverse ||
        (widget.route.navigator?.userGestureInProgress ?? false);
  }

  bool get _shouldRequestFocus {
    return widget.route.requestFocus;
  }

  // This should be called to wrap any changes to route.isCurrent, route.canPop,
  // and route.offstage.
  void _routeSetState(VoidCallback fn) {
    if (widget.route.isCurrent && !_shouldIgnoreFocusRequest && _shouldRequestFocus) {
      widget.route.navigator!.focusNode.enclosingScope?.setFirstFocus(focusScopeNode);
    }
    setState(fn);
  }

  @override
  Widget build(BuildContext context) {
    // Only top most route can participate in focus traversal.
    focusScopeNode.skipTraversal = !widget.route.isCurrent;
    return AnimatedBuilder(
      animation: widget.route.restorationScopeId,
      builder: (BuildContext context, Widget? child) {
        assert(child != null);
        return RestorationScope(
          restorationId: widget.route.restorationScopeId.value,
          child: child!,
        );
      },
      child: _ModalScopeStatus(
        route: widget.route,
        isCurrent: widget.route.isCurrent, // _routeSetState is called if this updates
        canPop: widget.route.canPop, // _routeSetState is called if this updates
        impliesAppBarDismissal: widget.route.impliesAppBarDismissal,
        child: Offstage(
          offstage: widget.route.offstage, // _routeSetState is called if this updates
          child: PageStorage(
            bucket: widget.route._storageBucket, // immutable
            child: Builder(
              builder: (BuildContext context) {
                return Actions(
                  actions: <Type, Action<Intent>>{DismissIntent: _DismissModalAction(context)},
                  child: PrimaryScrollController(
                    controller: primaryScrollController,
                    child: FocusScope.withExternalFocusNode(
                      focusScopeNode: focusScopeNode, // immutable
                      child: RepaintBoundary(
                        child: ListenableBuilder(
                          listenable: _listenable, // immutable
                          builder: (BuildContext context, Widget? child) {
                            return widget.route._buildFlexibleTransitions(
                              context,
                              widget.route.animation!,
                              widget.route.secondaryAnimation!,
                              // This additional ListenableBuilder is include because if the
                              // value of the userGestureInProgressNotifier changes, it's
                              // only necessary to rebuild the IgnorePointer widget and set
                              // the focus node's ability to focus.
                              ListenableBuilder(
                                listenable:
                                    widget.route.navigator?.userGestureInProgressNotifier ??
                                    ValueNotifier<bool>(false),
                                builder: (BuildContext context, Widget? child) {
                                  final bool ignoreEvents = _shouldIgnoreFocusRequest;
                                  focusScopeNode.canRequestFocus = !ignoreEvents;
                                  return IgnorePointer(ignoring: ignoreEvents, child: child);
                                },
                                child: child,
                              ),
                            );
                          },
                          child:
                              _page ??= RepaintBoundary(
                                key: widget.route._subtreeKey, // immutable
                                child: Builder(
                                  builder: (BuildContext context) {
                                    return widget.route.buildPage(
                                      context,
                                      widget.route.animation!,
                                      widget.route.secondaryAnimation!,
                                    );
                                  },
                                ),
                              ),
                        ),
                      ),
                    ),
                  ),
                );
              },
            ),
          ),
        ),
      ),
    );
  }
}

/// A route that blocks interaction with previous routes.
///
/// [ModalRoute]s cover the entire [Navigator]. They are not necessarily
/// [opaque], however; for example, a pop-up menu uses a [ModalRoute] but only
/// shows the menu in a small box overlapping the previous route.
///
/// The `T` type argument is the return value of the route. If there is no
/// return value, consider using `void` as the return value.
///
/// See also:
///
///  * [Route], which further documents the meaning of the `T` generic type argument.
abstract class ModalRoute<T> extends TransitionRoute<T> with LocalHistoryRoute<T> {
  /// Creates a route that blocks interaction with previous routes.
  ModalRoute({super.settings, super.requestFocus, this.filter, this.traversalEdgeBehavior});

  /// The filter to add to the barrier.
  ///
  /// If given, this filter will be applied to the modal barrier using
  /// [BackdropFilter]. This allows blur effects, for example.
  final ui.ImageFilter? filter;

  /// Controls the transfer of focus beyond the first and the last items of a
  /// [FocusScopeNode].
  ///
  /// If set to null, [Navigator.routeTraversalEdgeBehavior] is used.
  final TraversalEdgeBehavior? traversalEdgeBehavior;

  // The API for general users of this class

  /// Returns the modal route most closely associated with the given context.
  ///
  /// Returns null if the given context is not associated with a modal route.
  ///
  /// {@tool snippet}
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// ModalRoute<int>? route = ModalRoute.of<int>(context);
  /// ```
  /// {@end-tool}
  ///
  /// The given [BuildContext] will be rebuilt if the state of the route changes
  /// while it is visible (specifically, if [isCurrent] or [canPop] change value).
  @optionalTypeArgs
  static ModalRoute<T>? of<T extends Object?>(BuildContext context) {
    return _of<T>(context);
  }

  static ModalRoute<T>? _of<T extends Object?>(BuildContext context, [_ModalRouteAspect? aspect]) {
    return InheritedModel.inheritFrom<_ModalScopeStatus>(context, aspect: aspect)?.route
        as ModalRoute<T>?;
  }

  /// Returns [ModalRoute.isCurrent] for the modal route most closely associated
  /// with the given context.
  ///
  /// Returns null if the given context is not associated with a modal route.
  ///
  /// Use of this method will cause the given [context] to rebuild any time that
  /// the [ModalRoute.isCurrent] property of the ancestor [_ModalScopeStatus] changes.
  static bool? isCurrentOf(BuildContext context) =>
      _of(context, _ModalRouteAspect.isCurrent)?.isCurrent;

  /// Returns [ModalRoute.canPop] for the modal route most closely associated
  /// with the given context.
  ///
  /// Returns null if the given context is not associated with a modal route.
  ///
  /// Use of this method will cause the given [context] to rebuild any time that
  /// the [ModalRoute.canPop] property of the ancestor [_ModalScopeStatus] changes.
  static bool? canPopOf(BuildContext context) => _of(context, _ModalRouteAspect.canPop)?.canPop;

  /// Returns [ModalRoute.settings] for the modal route most closely associated
  /// with the given context.
  ///
  /// Returns null if the given context is not associated with a modal route.
  ///
  /// Use of this method will cause the given [context] to rebuild any time that
  /// the [ModalRoute.settings] property of the ancestor [_ModalScopeStatus] changes.
  static RouteSettings? settingsOf(BuildContext context) =>
      _of(context, _ModalRouteAspect.settings)?.settings;

  /// Schedule a call to [buildTransitions].
  ///
  /// Whenever you need to change internal state for a [ModalRoute] object, make
  /// the change in a function that you pass to [setState], as in:
  ///
  /// ```dart
  /// setState(() { _myState = newValue; });
  /// ```
  ///
  /// If you just change the state directly without calling [setState], then the
  /// route will not be scheduled for rebuilding, meaning that its rendering
  /// will not be updated.
  @protected
  void setState(VoidCallback fn) {
    if (_scopeKey.currentState != null) {
      _scopeKey.currentState!._routeSetState(fn);
    } else {
      // The route isn't currently visible, so we don't have to call its setState
      // method, but we do still need to call the fn callback, otherwise the state
      // in the route won't be updated!
      fn();
    }
  }

  /// Returns a predicate that's true if the route has the specified name and if
  /// popping the route will not yield the same route, i.e. if the route's
  /// [willHandlePopInternally] property is false.
  ///
  /// This function is typically used with [Navigator.popUntil()].
  static RoutePredicate withName(String name) {
    return (Route<dynamic> route) {
      return !route.willHandlePopInternally && route is ModalRoute && route.settings.name == name;
    };
  }

  // The API for subclasses to override - used by _ModalScope

  /// Override this method to build the primary content of this route.
  ///
  /// The arguments have the following meanings:
  ///
  ///  * `context`: The context in which the route is being built.
  ///  * [animation]: The animation for this route's transition. When entering,
  ///    the animation runs forward from 0.0 to 1.0. When exiting, this animation
  ///    runs backwards from 1.0 to 0.0.
  ///  * [secondaryAnimation]: The animation for the route being pushed on top of
  ///    this route. This animation lets this route coordinate with the entrance
  ///    and exit transition of routes pushed on top of this route.
  ///
  /// This method is only called when the route is first built, and rarely
  /// thereafter. In particular, it is not automatically called again when the
  /// route's state changes unless it uses [ModalRoute.of]. For a builder that
  /// is called every time the route's state changes, consider
  /// [buildTransitions]. For widgets that change their behavior when the
  /// route's state changes, consider [ModalRoute.of] to obtain a reference to
  /// the route; this will cause the widget to be rebuilt each time the route
  /// changes state.
  ///
  /// In general, [buildPage] should be used to build the page contents, and
  /// [buildTransitions] for the widgets that change as the page is brought in
  /// and out of view. Avoid using [buildTransitions] for content that never
  /// changes; building such content once from [buildPage] is more efficient.
  Widget buildPage(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
  );

  /// Override this method to wrap the [child] with one or more transition
  /// widgets that define how the route arrives on and leaves the screen.
  ///
  /// By default, the child (which contains the widget returned by [buildPage])
  /// is not wrapped in any transition widgets.
  ///
  /// The [buildTransitions] method, in contrast to [buildPage], is called each
  /// time the [Route]'s state changes while it is visible (e.g. if the value of
  /// [canPop] changes on the active route).
  ///
  /// The [buildTransitions] method is typically used to define transitions
  /// that animate the new topmost route's comings and goings. When the
  /// [Navigator] pushes a route on the top of its stack, the new route's
  /// primary [animation] runs from 0.0 to 1.0. When the Navigator pops the
  /// topmost route, e.g. because the use pressed the back button, the
  /// primary animation runs from 1.0 to 0.0.
  ///
  /// {@tool snippet}
  /// The following example uses the primary animation to drive a
  /// [SlideTransition] that translates the top of the new route vertically
  /// from the bottom of the screen when it is pushed on the Navigator's
  /// stack. When the route is popped the SlideTransition translates the
  /// route from the top of the screen back to the bottom.
  ///
  /// We've used [PageRouteBuilder] to demonstrate the [buildTransitions] method
  /// here. The body of an override of the [buildTransitions] method would be
  /// defined in the same way.
  ///
  /// ```dart
  /// PageRouteBuilder<void>(
  ///   pageBuilder: (BuildContext context,
  ///       Animation<double> animation,
  ///       Animation<double> secondaryAnimation,
  ///   ) {
  ///     return Scaffold(
  ///       appBar: AppBar(title: const Text('Hello')),
  ///       body: const Center(
  ///         child: Text('Hello World'),
  ///       ),
  ///     );
  ///   },
  ///   transitionsBuilder: (
  ///       BuildContext context,
  ///       Animation<double> animation,
  ///       Animation<double> secondaryAnimation,
  ///       Widget child,
  ///    ) {
  ///     return SlideTransition(
  ///       position: Tween<Offset>(
  ///         begin: const Offset(0.0, 1.0),
  ///         end: Offset.zero,
  ///       ).animate(animation),
  ///       child: child, // child is the value returned by pageBuilder
  ///     );
  ///   },
  /// )
  /// ```
  /// {@end-tool}
  ///
  /// When the [Navigator] pushes a route on the top of its stack, the
  /// [secondaryAnimation] can be used to define how the route that was on
  /// the top of the stack leaves the screen. Similarly when the topmost route
  /// is popped, the secondaryAnimation can be used to define how the route
  /// below it reappears on the screen. When the Navigator pushes a new route
  /// on the top of its stack, the old topmost route's secondaryAnimation
  /// runs from 0.0 to 1.0. When the Navigator pops the topmost route, the
  /// secondaryAnimation for the route below it runs from 1.0 to 0.0.
  ///
  /// {@tool snippet}
  /// The example below adds a transition that's driven by the
  /// [secondaryAnimation]. When this route disappears because a new route has
  /// been pushed on top of it, it translates in the opposite direction of
  /// the new route. Likewise when the route is exposed because the topmost
  /// route has been popped off.
  ///
  /// ```dart
  /// PageRouteBuilder<void>(
  ///   pageBuilder: (BuildContext context,
  ///       Animation<double> animation,
  ///       Animation<double> secondaryAnimation,
  ///   ) {
  ///     return Scaffold(
  ///       appBar: AppBar(title: const Text('Hello')),
  ///       body: const Center(
  ///         child: Text('Hello World'),
  ///       ),
  ///     );
  ///   },
  ///   transitionsBuilder: (
  ///       BuildContext context,
  ///       Animation<double> animation,
  ///       Animation<double> secondaryAnimation,
  ///       Widget child,
  ///   ) {
  ///     return SlideTransition(
  ///       position: Tween<Offset>(
  ///         begin: const Offset(0.0, 1.0),
  ///         end: Offset.zero,
  ///       ).animate(animation),
  ///       child: SlideTransition(
  ///         position: Tween<Offset>(
  ///           begin: Offset.zero,
  ///           end: const Offset(0.0, 1.0),
  ///         ).animate(secondaryAnimation),
  ///         child: child,
  ///       ),
  ///      );
  ///   },
  /// )
  /// ```
  /// {@end-tool}
  ///
  /// In practice the `secondaryAnimation` is used pretty rarely.
  ///
  /// The arguments to this method are as follows:
  ///
  ///  * `context`: The context in which the route is being built.
  ///  * [animation]: When the [Navigator] pushes a route on the top of its stack,
  ///    the new route's primary [animation] runs from 0.0 to 1.0. When the [Navigator]
  ///    pops the topmost route this animation runs from 1.0 to 0.0.
  ///  * [secondaryAnimation]: When the Navigator pushes a new route
  ///    on the top of its stack, the old topmost route's [secondaryAnimation]
  ///    runs from 0.0 to 1.0. When the [Navigator] pops the topmost route, the
  ///    [secondaryAnimation] for the route below it runs from 1.0 to 0.0.
  ///  * `child`, the page contents, as returned by [buildPage].
  ///
  /// See also:
  ///
  ///  * [buildPage], which is used to describe the actual contents of the page,
  ///    and whose result is passed to the `child` argument of this method.
  Widget buildTransitions(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
    Widget child,
  ) {
    return child;
  }

  /// The [DelegatedTransitionBuilder] provided to the route below this one in the
  /// navigation stack.
  ///
  /// {@template flutter.widgets.delegatedTransition}
  /// Used for the purposes of coordinating transitions between two routes with
  /// different route transitions. When a route is added to the stack, the original
  /// topmost route will look for this transition, and if available, it will use
  /// the `delegatedTransition` from the incoming transition to animate off the
  /// screen.
  ///
  /// If the return of the [DelegatedTransitionBuilder] is null, then by default
  /// the original transition of the routes will be used. This is useful if a
  /// route can conditionally provide a transition based on the [BuildContext].
  /// {@endtemplate}
  ///
  /// The [ModalRoute] receiving this transition will set it to their
  /// [receivedTransition] property.
  ///
  /// {@tool dartpad}
  /// This sample shows an app that uses three different page transitions, a
  /// Material Zoom transition, the standard Cupertino sliding transition, and a
  /// custom vertical transition. All of the page routes are able to inform the
  /// previous page how to transition off the screen to sync with the new page.
  ///
  /// ** See code in examples/api/lib/widgets/routes/flexible_route_transitions.0.dart **
  /// {@end-tool}
  ///
  /// {@tool dartpad}
  /// This sample shows an app that uses the same transitions as the previous
  /// sample, this time in a [MaterialApp.router].
  ///
  /// ** See code in examples/api/lib/widgets/routes/flexible_route_transitions.1.dart **
  /// {@end-tool}
  DelegatedTransitionBuilder? get delegatedTransition => null;

  /// The [DelegatedTransitionBuilder] received from the route above this one in
  /// the navigation stack.
  ///
  /// {@macro flutter.widgets.delegatedTransition}
  ///
  /// The `receivedTransition` will use the above route's [delegatedTransition] in
  /// order to show the right route transition when the above route either enters
  /// or leaves the navigation stack. If not null, the `receivedTransition` will
  /// wrap the route content.
  @visibleForTesting
  DelegatedTransitionBuilder? receivedTransition;

  // Wraps the transitions of this route with a DelegatedTransitionBuilder, when
  // _receivedTransition is not null.
  Widget _buildFlexibleTransitions(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
    Widget child,
  ) {
    if (receivedTransition == null) {
      return buildTransitions(context, animation, secondaryAnimation, child);
    }

    // Create a static proxy animation to suppress the original secondary transition.
    final ProxyAnimation proxyAnimation = ProxyAnimation();

    final Widget proxiedOriginalTransitions = buildTransitions(
      context,
      animation,
      proxyAnimation,
      child,
    );

    // If receivedTransitions return null, then we want to return the original transitions,
    // but with the secondary animation still proxied. This keeps a desynched
    // animation from playing.
    return receivedTransition!(
          context,
          animation,
          secondaryAnimation,
          allowSnapshotting,
          proxiedOriginalTransitions,
        ) ??
        proxiedOriginalTransitions;
  }

  @override
  void install() {
    super.install();
    _animationProxy = ProxyAnimation(super.animation);
    _secondaryAnimationProxy = ProxyAnimation(super.secondaryAnimation);
  }

  @override
  TickerFuture didPush() {
    if (_scopeKey.currentState != null && navigator!.widget.requestFocus) {
      navigator!.focusNode.enclosingScope?.setFirstFocus(_scopeKey.currentState!.focusScopeNode);
    }
    return super.didPush();
  }

  @override
  void didAdd() {
    if (_scopeKey.currentState != null && navigator!.widget.requestFocus) {
      navigator!.focusNode.enclosingScope?.setFirstFocus(_scopeKey.currentState!.focusScopeNode);
    }
    super.didAdd();
  }

  // The API for subclasses to override - used by this class

  /// {@template flutter.widgets.ModalRoute.barrierDismissible}
  /// Whether you can dismiss this route by tapping the modal barrier.
  ///
  /// The modal barrier is the scrim that is rendered behind each route, which
  /// generally prevents the user from interacting with the route below the
  /// current route, and normally partially obscures such routes.
  ///
  /// For example, when a dialog is on the screen, the page below the dialog is
  /// usually darkened by the modal barrier.
  ///
  /// If [barrierDismissible] is true, then tapping this barrier, pressing
  /// the escape key on the keyboard, or calling route popping functions
  /// such as [Navigator.pop] will cause the current route to be popped
  /// with null as the value.
  ///
  /// If [barrierDismissible] is false, then tapping the barrier has no effect.
  ///
  /// If this getter would ever start returning a different value,
  /// either [changedInternalState] or [changedExternalState] should
  /// be invoked so that the change can take effect.
  ///
  /// It is safe to use `navigator.context` to look up inherited
  /// widgets here, because the [Navigator] calls
  /// [changedExternalState] whenever its dependencies change, and
  /// [changedExternalState] causes the modal barrier to rebuild.
  ///
  /// See also:
  ///
  ///  * [Navigator.pop], which is used to dismiss the route.
  ///  * [barrierColor], which controls the color of the scrim for this route.
  ///  * [ModalBarrier], the widget that implements this feature.
  /// {@endtemplate}
  bool get barrierDismissible;

  /// Whether the semantics of the modal barrier are included in the
  /// semantics tree.
  ///
  /// The modal barrier is the scrim that is rendered behind each route, which
  /// generally prevents the user from interacting with the route below the
  /// current route, and normally partially obscures such routes.
  ///
  /// If [semanticsDismissible] is true, then modal barrier semantics are
  /// included in the semantics tree.
  ///
  /// If [semanticsDismissible] is false, then modal barrier semantics are
  /// excluded from the semantics tree and tapping on the modal barrier
  /// has no effect.
  ///
  /// If this getter would ever start returning a different value,
  /// either [changedInternalState] or [changedExternalState] should
  /// be invoked so that the change can take effect.
  ///
  /// It is safe to use `navigator.context` to look up inherited
  /// widgets here, because the [Navigator] calls
  /// [changedExternalState] whenever its dependencies change, and
  /// [changedExternalState] causes the modal barrier to rebuild.
  bool get semanticsDismissible => true;

  /// {@template flutter.widgets.ModalRoute.barrierColor}
  /// The color to use for the modal barrier. If this is null, the barrier will
  /// be transparent.
  ///
  /// The modal barrier is the scrim that is rendered behind each route, which
  /// generally prevents the user from interacting with the route below the
  /// current route, and normally partially obscures such routes.
  ///
  /// For example, when a dialog is on the screen, the page below the dialog is
  /// usually darkened by the modal barrier.
  ///
  /// The color is ignored, and the barrier made invisible, when
  /// [ModalRoute.offstage] is true.
  ///
  /// While the route is animating into position, the color is animated from
  /// transparent to the specified color.
  /// {@endtemplate}
  ///
  /// If this getter would ever start returning a different color, one
  /// of the [changedInternalState] or [changedExternalState] methods
  /// should be invoked so that the change can take effect.
  ///
  /// It is safe to use `navigator.context` to look up inherited
  /// widgets here, because the [Navigator] calls
  /// [changedExternalState] whenever its dependencies change, and
  /// [changedExternalState] causes the modal barrier to rebuild.
  ///
  /// {@tool snippet}
  ///
  /// For example, to make the barrier color use the theme's
  /// background color, one could say:
  ///
  /// ```dart
  /// Color get barrierColor => Theme.of(navigator.context).colorScheme.surface;
  /// ```
  ///
  /// {@end-tool}
  ///
  /// See also:
  ///
  ///  * [barrierDismissible], which controls the behavior of the barrier when
  ///    tapped.
  ///  * [ModalBarrier], the widget that implements this feature.
  Color? get barrierColor;

  /// {@template flutter.widgets.ModalRoute.barrierLabel}
  /// The semantic label used for a dismissible barrier.
  ///
  /// If the barrier is dismissible, this label will be read out if
  /// accessibility tools (like VoiceOver on iOS) focus on the barrier.
  ///
  /// The modal barrier is the scrim that is rendered behind each route, which
  /// generally prevents the user from interacting with the route below the
  /// current route, and normally partially obscures such routes.
  ///
  /// For example, when a dialog is on the screen, the page below the dialog is
  /// usually darkened by the modal barrier.
  /// {@endtemplate}
  ///
  /// If this getter would ever start returning a different label,
  /// either [changedInternalState] or [changedExternalState] should
  /// be invoked so that the change can take effect.
  ///
  /// It is safe to use `navigator.context` to look up inherited
  /// widgets here, because the [Navigator] calls
  /// [changedExternalState] whenever its dependencies change, and
  /// [changedExternalState] causes the modal barrier to rebuild.
  ///
  /// See also:
  ///
  ///  * [barrierDismissible], which controls the behavior of the barrier when
  ///    tapped.
  ///  * [ModalBarrier], the widget that implements this feature.
  String? get barrierLabel;

  /// The curve that is used for animating the modal barrier in and out.
  ///
  /// The modal barrier is the scrim that is rendered behind each route, which
  /// generally prevents the user from interacting with the route below the
  /// current route, and normally partially obscures such routes.
  ///
  /// For example, when a dialog is on the screen, the page below the dialog is
  /// usually darkened by the modal barrier.
  ///
  /// While the route is animating into position, the color is animated from
  /// transparent to the specified [barrierColor].
  ///
  /// If this getter would ever start returning a different curve,
  /// either [changedInternalState] or [changedExternalState] should
  /// be invoked so that the change can take effect.
  ///
  /// It is safe to use `navigator.context` to look up inherited
  /// widgets here, because the [Navigator] calls
  /// [changedExternalState] whenever its dependencies change, and
  /// [changedExternalState] causes the modal barrier to rebuild.
  ///
  /// It defaults to [Curves.ease].
  ///
  /// See also:
  ///
  ///  * [barrierColor], which determines the color that the modal transitions
  ///    to.
  ///  * [Curves] for a collection of common curves.
  ///  * [AnimatedModalBarrier], the widget that implements this feature.
  Curve get barrierCurve => Curves.ease;

  /// {@template flutter.widgets.ModalRoute.maintainState}
  /// Whether the route should remain in memory when it is inactive.
  ///
  /// If this is true, then the route is maintained, so that any futures it is
  /// holding from the next route will properly resolve when the next route
  /// pops. If this is not necessary, this can be set to false to allow the
  /// framework to entirely discard the route's widget hierarchy when it is not
  /// visible.
  ///
  /// Setting [maintainState] to false does not guarantee that the route will be
  /// discarded. For instance, it will not be discarded if it is still visible
  /// because the next above it is not opaque (e.g. it is a popup dialog).
  /// {@endtemplate}
  ///
  /// If this getter would ever start returning a different value, the
  /// [changedInternalState] should be invoked so that the change can take
  /// effect.
  ///
  /// See also:
  ///
  ///  * [OverlayEntry.maintainState], which is the underlying implementation
  ///    of this property.
  bool get maintainState;

  /// True if a back gesture (iOS-style back swipe or Android predictive back)
  /// is currently underway for this route.
  ///
  /// See also:
  ///
  ///  * [popGestureEnabled], which returns true if a user-triggered pop gesture
  ///    would be allowed.
  bool get popGestureInProgress => navigator!.userGestureInProgress;

  /// Whether a pop gesture can be started by the user for this route.
  ///
  /// Returns true if the user can edge-swipe to a previous route.
  ///
  /// This should only be used between frames, not during build.
  @override
  bool get popGestureEnabled {
    // If there's nothing to go back to, then obviously we don't support
    // the back gesture.
    if (isFirst) {
      return false;
    }
    // If the route wouldn't actually pop if we popped it, then the gesture
    // would be really confusing (or would skip internal routes), so disallow it.
    if (willHandlePopInternally) {
      return false;
    }
    // If attempts to dismiss this route might be vetoed such as in a page
    // with forms, then do not allow the user to dismiss the route with a swipe.
    if (hasScopedWillPopCallback || popDisposition == RoutePopDisposition.doNotPop) {
      return false;
    }
    // If we're in an animation already, we cannot be manually swiped.
    if (!animation!.isCompleted) {
      return false;
    }
    // If we're being popped into, we also cannot be swiped until the pop above
    // it completes. This translates to our secondary animation being
    // dismissed.
    if (!secondaryAnimation!.isDismissed) {
      return false;
    }
    // If we're in a gesture already, we cannot start another.
    if (popGestureInProgress) {
      return false;
    }

    // Looks like a back gesture would be welcome!
    return true;
  }

  // The API for _ModalScope and HeroController

  /// Whether this route is currently offstage.
  ///
  /// On the first frame of a route's entrance transition, the route is built
  /// [Offstage] using an animation progress of 1.0. The route is invisible and
  /// non-interactive, but each widget has its final size and position. This
  /// mechanism lets the [HeroController] determine the final local of any hero
  /// widgets being animated as part of the transition.
  ///
  /// The modal barrier, if any, is not rendered if [offstage] is true (see
  /// [barrierColor]).
  ///
  /// Whenever this changes value, [changedInternalState] is called.
  bool get offstage => _offstage;
  bool _offstage = false;
  set offstage(bool value) {
    if (_offstage == value) {
      return;
    }
    setState(() {
      _offstage = value;
    });
    _animationProxy!.parent = _offstage ? kAlwaysCompleteAnimation : super.animation;
    _secondaryAnimationProxy!.parent =
        _offstage ? kAlwaysDismissedAnimation : super.secondaryAnimation;
    changedInternalState();
  }

  /// The build context for the subtree containing the primary content of this route.
  BuildContext? get subtreeContext => _subtreeKey.currentContext;

  @override
  Animation<double>? get animation => _animationProxy;
  ProxyAnimation? _animationProxy;

  @override
  Animation<double>? get secondaryAnimation => _secondaryAnimationProxy;
  ProxyAnimation? _secondaryAnimationProxy;

  final List<WillPopCallback> _willPopCallbacks = <WillPopCallback>[];

  // Holding as Object? instead of T so that PopScope in this route can be
  // declared with any supertype of T.
  final Set<PopEntry<Object?>> _popEntries = <PopEntry<Object?>>{};

  /// Returns [RoutePopDisposition.doNotPop] if any of callbacks added with
  /// [addScopedWillPopCallback] returns either false or null. If they all
  /// return true, the base [Route.willPop]'s result will be returned. The
  /// callbacks will be called in the order they were added, and will only be
  /// called if all previous callbacks returned true.
  ///
  /// Typically this method is not overridden because applications usually
  /// don't create modal routes directly, they use higher level primitives
  /// like [showDialog]. The scoped [WillPopCallback] list makes it possible
  /// for ModalRoute descendants to collectively define the value of [willPop].
  ///
  /// See also:
  ///
  ///  * [Form], which provides an `onWillPop` callback that uses this mechanism.
  ///  * [addScopedWillPopCallback], which adds a callback to the list this
  ///    method checks.
  ///  * [removeScopedWillPopCallback], which removes a callback from the list
  ///    this method checks.
  @Deprecated(
    'Use popDisposition instead. '
    'This feature was deprecated after v3.12.0-1.0.pre.',
  )
  @override
  Future<RoutePopDisposition> willPop() async {
    final _ModalScopeState<T>? scope = _scopeKey.currentState;
    assert(scope != null);
    for (final WillPopCallback callback in List<WillPopCallback>.of(_willPopCallbacks)) {
      if (!await callback()) {
        return RoutePopDisposition.doNotPop;
      }
    }
    return super.willPop();
  }

  /// Returns [RoutePopDisposition.doNotPop] if any of the [PopEntry] instances
  /// registered with [registerPopEntry] have [PopEntry.canPopNotifier] set to
  /// false.
  ///
  /// Typically this method is not overridden because applications usually
  /// don't create modal routes directly, they use higher level primitives
  /// like [showDialog]. The scoped [PopEntry] list makes it possible for
  /// ModalRoute descendants to collectively define the value of
  /// [popDisposition].
  ///
  /// See also:
  ///
  ///  * [Form], which provides an `onPopInvokedWithResult` callback that is similar.
  ///  * [registerPopEntry], which adds a [PopEntry] to the list this method
  ///    checks.
  ///  * [unregisterPopEntry], which removes a [PopEntry] from the list this
  ///    method checks.
  @override
  RoutePopDisposition get popDisposition {
    for (final PopEntry<Object?> popEntry in _popEntries) {
      if (!popEntry.canPopNotifier.value) {
        return RoutePopDisposition.doNotPop;
      }
    }

    return super.popDisposition;
  }

  @override
  void onPopInvokedWithResult(bool didPop, T? result) {
    for (final PopEntry<Object?> popEntry in _popEntries) {
      popEntry.onPopInvokedWithResult(didPop, result);
    }
    super.onPopInvokedWithResult(didPop, result);
  }

  /// Enables this route to veto attempts by the user to dismiss it.
  ///
  /// This callback runs asynchronously and it's possible that it will be called
  /// after its route has been disposed. The callback should check [State.mounted]
  /// before doing anything.
  ///
  /// A typical application of this callback would be to warn the user about
  /// unsaved [Form] data if the user attempts to back out of the form. In that
  /// case, use the [Form.onWillPop] property to register the callback.
  ///
  /// See also:
  ///
  ///  * [WillPopScope], which manages the registration and unregistration
  ///    process automatically.
  ///  * [Form], which provides an `onWillPop` callback that uses this mechanism.
  ///  * [willPop], which runs the callbacks added with this method.
  ///  * [removeScopedWillPopCallback], which removes a callback from the list
  ///    that [willPop] checks.
  @Deprecated(
    'Use registerPopEntry or PopScope instead. '
    'This feature was deprecated after v3.12.0-1.0.pre.',
  )
  void addScopedWillPopCallback(WillPopCallback callback) {
    assert(
      _scopeKey.currentState != null,
      'Tried to add a willPop callback to a route that is not currently in the tree.',
    );
    _willPopCallbacks.add(callback);
    if (_willPopCallbacks.length == 1) {
      _maybeDispatchNavigationNotification();
    }
  }

  /// Remove one of the callbacks run by [willPop].
  ///
  /// See also:
  ///
  ///  * [Form], which provides an `onWillPop` callback that uses this mechanism.
  ///  * [addScopedWillPopCallback], which adds callback to the list
  ///    checked by [willPop].
  @Deprecated(
    'Use unregisterPopEntry or PopScope instead. '
    'This feature was deprecated after v3.12.0-1.0.pre.',
  )
  void removeScopedWillPopCallback(WillPopCallback callback) {
    assert(
      _scopeKey.currentState != null,
      'Tried to remove a willPop callback from a route that is not currently in the tree.',
    );
    _willPopCallbacks.remove(callback);
    if (_willPopCallbacks.isEmpty) {
      _maybeDispatchNavigationNotification();
    }
  }

  /// Registers the existence of a [PopEntry] in the route.
  ///
  /// [PopEntry] instances registered in this way will have their
  /// [PopEntry.onPopInvokedWithResult] callbacks called when a route is popped or a pop
  /// is attempted. They will also be able to block pop operations with
  /// [PopEntry.canPopNotifier] through this route's [popDisposition] method.
  ///
  /// See also:
  ///
  ///  * [unregisterPopEntry], which performs the opposite operation.
  void registerPopEntry(PopEntry<Object?> popEntry) {
    _popEntries.add(popEntry);
    popEntry.canPopNotifier.addListener(_maybeDispatchNavigationNotification);
    _maybeDispatchNavigationNotification();
  }

  /// Unregisters a [PopEntry] in the route's widget subtree.
  ///
  /// See also:
  ///
  ///  * [registerPopEntry], which performs the opposite operation.
  void unregisterPopEntry(PopEntry<Object?> popEntry) {
    _popEntries.remove(popEntry);
    popEntry.canPopNotifier.removeListener(_maybeDispatchNavigationNotification);
    _maybeDispatchNavigationNotification();
  }

  void _maybeDispatchNavigationNotification() {
    if (!isCurrent) {
      return;
    }
    final NavigationNotification notification = NavigationNotification(
      // canPop indicates that the originator of the Notification can handle a
      // pop. In the case of PopScope, it handles pops when canPop is
      // false. Hence the seemingly backward logic here.
      canHandlePop: popDisposition == RoutePopDisposition.doNotPop || _willPopCallbacks.isNotEmpty,
    );
    // Avoid dispatching a notification in the middle of a build.
    switch (SchedulerBinding.instance.schedulerPhase) {
      case SchedulerPhase.postFrameCallbacks:
        notification.dispatch(subtreeContext);
      case SchedulerPhase.idle:
      case SchedulerPhase.midFrameMicrotasks:
      case SchedulerPhase.persistentCallbacks:
      case SchedulerPhase.transientCallbacks:
        SchedulerBinding.instance.addPostFrameCallback((Duration timeStamp) {
          if (!(subtreeContext?.mounted ?? false)) {
            return;
          }
          notification.dispatch(subtreeContext);
        }, debugLabel: 'ModalRoute.dispatchNotification');
    }
  }

  /// True if one or more [WillPopCallback] callbacks exist.
  ///
  /// This method is used to disable the horizontal swipe pop gesture supported
  /// by [MaterialPageRoute] for [TargetPlatform.iOS] and
  /// [TargetPlatform.macOS]. If a pop might be vetoed, then the back gesture is
  /// disabled.
  ///
  /// The [buildTransitions] method will not be called again if this changes,
  /// since it can change during the build as descendants of the route add or
  /// remove callbacks.
  ///
  /// See also:
  ///
  ///  * [addScopedWillPopCallback], which adds a callback.
  ///  * [removeScopedWillPopCallback], which removes a callback.
  ///  * [willHandlePopInternally], which reports on another reason why
  ///    a pop might be vetoed.
  @Deprecated(
    'Use popDisposition instead. '
    'This feature was deprecated after v3.12.0-1.0.pre.',
  )
  @protected
  bool get hasScopedWillPopCallback {
    return _willPopCallbacks.isNotEmpty;
  }

  @override
  void didChangePrevious(Route<dynamic>? previousRoute) {
    super.didChangePrevious(previousRoute);
    changedInternalState();
  }

  @override
  void didChangeNext(Route<dynamic>? nextRoute) {
    if (nextRoute is ModalRoute<T> &&
        canTransitionTo(nextRoute) &&
        nextRoute.delegatedTransition != this.delegatedTransition) {
      receivedTransition = nextRoute.delegatedTransition;
    } else {
      receivedTransition = null;
    }
    super.didChangeNext(nextRoute);
    changedInternalState();
  }

  @override
  void didPopNext(Route<dynamic> nextRoute) {
    if (nextRoute is ModalRoute<T> &&
        canTransitionTo(nextRoute) &&
        nextRoute.delegatedTransition != this.delegatedTransition) {
      receivedTransition = nextRoute.delegatedTransition;
    } else {
      receivedTransition = null;
    }
    super.didPopNext(nextRoute);
    changedInternalState();
    _maybeDispatchNavigationNotification();
  }

  @override
  void changedInternalState() {
    super.changedInternalState();
    // No need to mark dirty if this method is called during build phase.
    if (SchedulerBinding.instance.schedulerPhase != SchedulerPhase.persistentCallbacks) {
      setState(() {
        /* internal state already changed */
      });
      _modalBarrier.markNeedsBuild();
    }
    _modalScope.maintainState = maintainState;
  }

  @override
  void changedExternalState() {
    super.changedExternalState();
    _modalBarrier.markNeedsBuild();
    if (_scopeKey.currentState != null) {
      _scopeKey.currentState!._forceRebuildPage();
    }
  }

  /// Whether this route can be popped.
  ///
  /// A route can be popped if there is at least one active route below it, or
  /// if [willHandlePopInternally] returns true.
  ///
  /// When this changes, if the route is visible, the route will
  /// rebuild, and any widgets that used [ModalRoute.of] will be
  /// notified.
  bool get canPop => hasActiveRouteBelow || willHandlePopInternally;

  /// Whether an [AppBar] in the route should automatically add a back button or
  /// close button.
  ///
  /// This getter returns true if there is at least one active route below it,
  /// or there is at least one [LocalHistoryEntry] with [impliesAppBarDismissal]
  /// set to true
  bool get impliesAppBarDismissal => hasActiveRouteBelow || _entriesImpliesAppBarDismissal > 0;

  // Internals

  final GlobalKey<_ModalScopeState<T>> _scopeKey = GlobalKey<_ModalScopeState<T>>();
  final GlobalKey _subtreeKey = GlobalKey();
  final PageStorageBucket _storageBucket = PageStorageBucket();

  // one of the builders
  late OverlayEntry _modalBarrier;
  Widget _buildModalBarrier(BuildContext context) {
    Widget barrier = buildModalBarrier();
    if (filter != null) {
      barrier = BackdropFilter(filter: filter!, child: barrier);
    }
    barrier = IgnorePointer(
      ignoring:
          !animation!
              .isForwardOrCompleted, // changedInternalState is called when animation.status updates
      child: barrier, // dismissed is possible when doing a manual pop gesture
    );
    if (semanticsDismissible && barrierDismissible) {
      // To be sorted after the _modalScope.
      barrier = Semantics(sortKey: const OrdinalSortKey(1.0), child: barrier);
    }
    return barrier;
  }

  /// Build the barrier for this [ModalRoute], subclasses can override
  /// this method to create their own barrier with customized features such as
  /// color or accessibility focus size.
  ///
  /// See also:
  /// * [ModalBarrier], which is typically used to build a barrier.
  /// * [ModalBottomSheetRoute], which overrides this method to build a
  ///   customized barrier.
  Widget buildModalBarrier() {
    Widget barrier;
    if (barrierColor != null && barrierColor!.alpha != 0 && !offstage) {
      // changedInternalState is called if barrierColor or offstage updates
      assert(barrierColor != barrierColor!.withOpacity(0.0));
      final Animation<Color?> color = animation!.drive(
        ColorTween(
          begin: barrierColor!.withOpacity(0.0),
          end: barrierColor, // changedInternalState is called if barrierColor updates
        ).chain(
          CurveTween(curve: barrierCurve),
        ), // changedInternalState is called if barrierCurve updates
      );
      barrier = AnimatedModalBarrier(
        color: color,
        dismissible:
            barrierDismissible, // changedInternalState is called if barrierDismissible updates
        semanticsLabel: barrierLabel, // changedInternalState is called if barrierLabel updates
        barrierSemanticsDismissible: semanticsDismissible,
      );
    } else {
      barrier = ModalBarrier(
        dismissible:
            barrierDismissible, // changedInternalState is called if barrierDismissible updates
        semanticsLabel: barrierLabel, // changedInternalState is called if barrierLabel updates
        barrierSemanticsDismissible: semanticsDismissible,
      );
    }

    return barrier;
  }

  // We cache the part of the modal scope that doesn't change from frame to
  // frame so that we minimize the amount of building that happens.
  Widget? _modalScopeCache;

  // one of the builders
  Widget _buildModalScope(BuildContext context) {
    // To be sorted before the _modalBarrier.
    return _modalScopeCache ??= Semantics(
      sortKey: const OrdinalSortKey(0.0),
      child: _ModalScope<T>(
        key: _scopeKey,
        route: this,
        // _ModalScope calls buildTransitions() and buildChild(), defined above
      ),
    );
  }

  late OverlayEntry _modalScope;

  @override
  Iterable<OverlayEntry> createOverlayEntries() {
    return <OverlayEntry>[
      _modalBarrier = OverlayEntry(builder: _buildModalBarrier),
      _modalScope = OverlayEntry(
        builder: _buildModalScope,
        maintainState: maintainState,
        canSizeOverlay: opaque,
      ),
    ];
  }

  @override
  String toString() =>
      '${objectRuntimeType(this, 'ModalRoute')}($settings, animation: $_animation)';
}

/// A modal route that overlays a widget over the current route.
///
/// {@macro flutter.widgets.ModalRoute.barrierDismissible}
///
/// {@tool dartpad}
/// This example shows how to create a dialog box that is dismissible.
///
/// ** See code in examples/api/lib/widgets/routes/popup_route.0.dart **
/// {@end-tool}
///
/// See also:
///
///   * [ModalRoute], which is the base class for this class.
///   * [Navigator.pop], which is used to dismiss the route.
abstract class PopupRoute<T> extends ModalRoute<T> {
  /// Initializes the [PopupRoute].
  PopupRoute({super.settings, super.requestFocus, super.filter, super.traversalEdgeBehavior});

  @override
  bool get opaque => false;

  @override
  bool get maintainState => true;

  @override
  bool get allowSnapshotting => false;
}

/// A [Navigator] observer that notifies [RouteAware]s of changes to the
/// state of their [Route].
///
/// [RouteObserver] informs subscribers whenever a route of type `R` is pushed
/// on top of their own route of type `R` or popped from it. This is for example
/// useful to keep track of page transitions, e.g. a `RouteObserver<PageRoute>`
/// will inform subscribed [RouteAware]s whenever the user navigates away from
/// the current page route to another page route.
///
/// To be informed about route changes of any type, consider instantiating a
/// `RouteObserver<Route>`.
///
/// ## Type arguments
///
/// When using more aggressive [lints](https://dart.dev/lints),
/// in particular lints such as `always_specify_types`,
/// the Dart analyzer will require that certain types
/// be given with their type arguments. Since the [Route] class and its
/// subclasses have a type argument, this includes the arguments passed to this
/// class. Consider using `dynamic` to specify the entire class of routes rather
/// than only specific subtypes. For example, to watch for all [ModalRoute]
/// variants, the `RouteObserver<ModalRoute<dynamic>>` type may be used.
///
/// {@tool dartpad}
/// This example demonstrates how to implement a [RouteObserver] that notifies
/// [RouteAware] widget of changes to the state of their [Route].
///
/// ** See code in examples/api/lib/widgets/routes/route_observer.0.dart **
/// {@end-tool}
///
/// See also:
///  * [RouteAware], this is used with [RouteObserver] to make a widget aware
///   of changes to the [Navigator]'s session history.
class RouteObserver<R extends Route<dynamic>> extends NavigatorObserver {
  final Map<R, Set<RouteAware>> _listeners = <R, Set<RouteAware>>{};

  /// Whether this observer is managing changes for the specified route.
  ///
  /// If asserts are disabled, this method will throw an exception.
  @visibleForTesting
  bool debugObservingRoute(R route) {
    late bool contained;
    assert(() {
      contained = _listeners.containsKey(route);
      return true;
    }());
    return contained;
  }

  /// Subscribe [routeAware] to be informed about changes to [route].
  ///
  /// Going forward, [routeAware] will be informed about qualifying changes
  /// to [route], e.g. when [route] is covered by another route or when [route]
  /// is popped off the [Navigator] stack.
  void subscribe(RouteAware routeAware, R route) {
    final Set<RouteAware> subscribers = _listeners.putIfAbsent(route, () => <RouteAware>{});
    if (subscribers.add(routeAware)) {
      routeAware.didPush();
    }
  }

  /// Unsubscribe [routeAware].
  ///
  /// [routeAware] is no longer informed about changes to its route. If the given argument was
  /// subscribed to multiple types, this will unregister it (once) from each type.
  void unsubscribe(RouteAware routeAware) {
    final List<R> routes = _listeners.keys.toList();
    for (final R route in routes) {
      final Set<RouteAware>? subscribers = _listeners[route];
      if (subscribers != null) {
        subscribers.remove(routeAware);
        if (subscribers.isEmpty) {
          _listeners.remove(route);
        }
      }
    }
  }

  @override
  void didPop(Route<dynamic> route, Route<dynamic>? previousRoute) {
    if (route is R && previousRoute is R) {
      final List<RouteAware>? previousSubscribers = _listeners[previousRoute]?.toList();

      if (previousSubscribers != null) {
        for (final RouteAware routeAware in previousSubscribers) {
          routeAware.didPopNext();
        }
      }

      final List<RouteAware>? subscribers = _listeners[route]?.toList();

      if (subscribers != null) {
        for (final RouteAware routeAware in subscribers) {
          routeAware.didPop();
        }
      }
    }
  }

  @override
  void didPush(Route<dynamic> route, Route<dynamic>? previousRoute) {
    if (route is R && previousRoute is R) {
      final Set<RouteAware>? previousSubscribers = _listeners[previousRoute];

      if (previousSubscribers != null) {
        for (final RouteAware routeAware in previousSubscribers) {
          routeAware.didPushNext();
        }
      }
    }
  }
}

/// An interface for objects that are aware of their current [Route].
///
/// This is used with [RouteObserver] to make a widget aware of changes to the
/// [Navigator]'s session history.
abstract mixin class RouteAware {
  /// Called when the top route has been popped off, and the current route
  /// shows up.
  void didPopNext() {}

  /// Called when the current route has been pushed.
  void didPush() {}

  /// Called when the current route has been popped off.
  void didPop() {}

  /// Called when a new route has been pushed, and the current route is no
  /// longer visible.
  void didPushNext() {}
}

/// A general dialog route which allows for customization of the dialog popup.
///
/// It is used internally by [showGeneralDialog] or can be directly pushed
/// onto the [Navigator] stack to enable state restoration. See
/// [showGeneralDialog] for a state restoration app example.
///
/// This function takes a `pageBuilder`, which typically builds a dialog.
/// Content below the dialog is dimmed with a [ModalBarrier]. The widget
/// returned by the `builder` does not share a context with the location that
/// `showDialog` is originally called from. Use a [StatefulBuilder] or a
/// custom [StatefulWidget] if the dialog needs to update dynamically.
///
/// The `barrierDismissible` argument is used to indicate whether tapping on the
/// barrier will dismiss the dialog. It is `true` by default and cannot be `null`.
///
/// The `barrierColor` argument is used to specify the color of the modal
/// barrier that darkens everything below the dialog. If `null`, the default
/// color `Colors.black54` is used.
///
/// The `settings` argument define the settings for this route. See
/// [RouteSettings] for details.
///
/// {@template flutter.widgets.RawDialogRoute}
/// A [DisplayFeature] can split the screen into sub-screens. The closest one to
/// [anchorPoint] is used to render the content.
///
/// If no [anchorPoint] is provided, then [Directionality] is used:
///
///   * for [TextDirection.ltr], [anchorPoint] is `Offset.zero`, which will
///     cause the content to appear in the top-left sub-screen.
///   * for [TextDirection.rtl], [anchorPoint] is `Offset(double.maxFinite, 0)`,
///     which will cause the content to appear in the top-right sub-screen.
///
/// If no [anchorPoint] is provided, and there is no [Directionality] ancestor
/// widget in the tree, then the widget asserts during build in debug mode.
/// {@endtemplate}
///
/// See also:
///
///  * [DisplayFeatureSubScreen], which documents the specifics of how
///    [DisplayFeature]s can split the screen into sub-screens.
///  * [showGeneralDialog], which is a way to display a RawDialogRoute.
///  * [showDialog], which is a way to display a DialogRoute.
///  * [showCupertinoDialog], which displays an iOS-style dialog.
class RawDialogRoute<T> extends PopupRoute<T> {
  /// A general dialog route which allows for customization of the dialog popup.
  RawDialogRoute({
    required RoutePageBuilder pageBuilder,
    bool barrierDismissible = true,
    Color? barrierColor = const Color(0x80000000),
    String? barrierLabel,
    Duration transitionDuration = const Duration(milliseconds: 200),
    RouteTransitionsBuilder? transitionBuilder,
    super.settings,
    super.requestFocus,
    this.anchorPoint,
    super.traversalEdgeBehavior,
  }) : _pageBuilder = pageBuilder,
       _barrierDismissible = barrierDismissible,
       _barrierLabel = barrierLabel,
       _barrierColor = barrierColor,
       _transitionDuration = transitionDuration,
       _transitionBuilder = transitionBuilder;

  final RoutePageBuilder _pageBuilder;

  @override
  bool get barrierDismissible => _barrierDismissible;
  final bool _barrierDismissible;

  @override
  String? get barrierLabel => _barrierLabel;
  final String? _barrierLabel;

  @override
  Color? get barrierColor => _barrierColor;
  final Color? _barrierColor;

  @override
  Duration get transitionDuration => _transitionDuration;
  final Duration _transitionDuration;

  final RouteTransitionsBuilder? _transitionBuilder;

  /// {@macro flutter.widgets.DisplayFeatureSubScreen.anchorPoint}
  final Offset? anchorPoint;

  @override
  Widget buildPage(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
  ) {
    return Semantics(
      scopesRoute: true,
      explicitChildNodes: true,
      child: DisplayFeatureSubScreen(
        anchorPoint: anchorPoint,
        child: _pageBuilder(context, animation, secondaryAnimation),
      ),
    );
  }

  @override
  Widget buildTransitions(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
    Widget child,
  ) {
    if (_transitionBuilder == null) {
      // Some default transition.
      return FadeTransition(opacity: animation, child: child);
    }
    return _transitionBuilder(context, animation, secondaryAnimation, child);
  }
}

/// Displays a dialog above the current contents of the app.
///
/// This function allows for customization of aspects of the dialog popup.
///
/// This function takes a `pageBuilder` which is used to build the primary
/// content of the route (typically a dialog widget). Content below the dialog
/// is dimmed with a [ModalBarrier]. The widget returned by the `pageBuilder`
/// does not share a context with the location that [showGeneralDialog] is
/// originally called from. Use a [StatefulBuilder] or a custom
/// [StatefulWidget] if the dialog needs to update dynamically.
///
/// The `context` argument is used to look up the [Navigator] for the
/// dialog. It is only used when the method is called. Its corresponding widget
/// can be safely removed from the tree before the dialog is closed.
///
/// The `useRootNavigator` argument is used to determine whether to push the
/// dialog to the [Navigator] furthest from or nearest to the given `context`.
/// By default, `useRootNavigator` is `true` and the dialog route created by
/// this method is pushed to the root navigator.
///
/// If the application has multiple [Navigator] objects, it may be necessary to
/// call `Navigator.of(context, rootNavigator: true).pop(result)` to close the
/// dialog rather than just `Navigator.pop(context, result)`.
///
/// The `barrierDismissible` argument is used to determine whether this route
/// can be dismissed by tapping the modal barrier. This argument defaults
/// to false. If `barrierDismissible` is true, a non-null `barrierLabel` must be
/// provided.
///
/// The `barrierLabel` argument is the semantic label used for a dismissible
/// barrier. This argument defaults to `null`.
///
/// The `barrierColor` argument is the color used for the modal barrier. This
/// argument defaults to `Color(0x80000000)`.
///
/// The `transitionDuration` argument is used to determine how long it takes
/// for the route to arrive on or leave off the screen. This argument defaults
/// to 200 milliseconds.
///
/// The `transitionBuilder` argument is used to define how the route arrives on
/// and leaves off the screen. By default, the transition is a linear fade of
/// the page's contents.
///
/// The `routeSettings` will be used in the construction of the dialog's route.
/// See [RouteSettings] for more details.
///
/// {@macro flutter.widgets.RawDialogRoute}
///
/// Returns a [Future] that resolves to the value (if any) that was passed to
/// [Navigator.pop] when the dialog was closed.
///
/// ### State Restoration in Dialogs
///
/// Using this method will not enable state restoration for the dialog. In order
/// to enable state restoration for a dialog, use [Navigator.restorablePush]
/// or [Navigator.restorablePushNamed] with [RawDialogRoute].
///
/// For more information about state restoration, see [RestorationManager].
///
/// {@tool sample}
/// This sample demonstrates how to create a restorable dialog. This is
/// accomplished by enabling state restoration by specifying
/// [WidgetsApp.restorationScopeId] and using [Navigator.restorablePush] to
/// push [RawDialogRoute] when the button is tapped.
///
/// {@macro flutter.widgets.RestorationManager}
///
/// ** See code in examples/api/lib/widgets/routes/show_general_dialog.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [DisplayFeatureSubScreen], which documents the specifics of how
///    [DisplayFeature]s can split the screen into sub-screens.
///  * [showDialog], which displays a Material-style dialog.
///  * [showCupertinoDialog], which displays an iOS-style dialog.
Future<T?> showGeneralDialog<T extends Object?>({
  required BuildContext context,
  required RoutePageBuilder pageBuilder,
  bool barrierDismissible = false,
  String? barrierLabel,
  Color barrierColor = const Color(0x80000000),
  Duration transitionDuration = const Duration(milliseconds: 200),
  RouteTransitionsBuilder? transitionBuilder,
  bool useRootNavigator = true,
  RouteSettings? routeSettings,
  Offset? anchorPoint,
}) {
  assert(!barrierDismissible || barrierLabel != null);
  return Navigator.of(context, rootNavigator: useRootNavigator).push<T>(
    RawDialogRoute<T>(
      pageBuilder: pageBuilder,
      barrierDismissible: barrierDismissible,
      barrierLabel: barrierLabel,
      barrierColor: barrierColor,
      transitionDuration: transitionDuration,
      transitionBuilder: transitionBuilder,
      settings: routeSettings,
      anchorPoint: anchorPoint,
    ),
  );
}

/// Signature for the function that builds a route's primary contents.
/// Used in [PageRouteBuilder] and [showGeneralDialog].
///
/// See [ModalRoute.buildPage] for complete definition of the parameters.
typedef RoutePageBuilder =
    Widget Function(
      BuildContext context,
      Animation<double> animation,
      Animation<double> secondaryAnimation,
    );

/// Signature for the function that builds a route's transitions.
/// Used in [PageRouteBuilder] and [showGeneralDialog].
///
/// See [ModalRoute.buildTransitions] for complete definition of the parameters.
typedef RouteTransitionsBuilder =
    Widget Function(
      BuildContext context,
      Animation<double> animation,
      Animation<double> secondaryAnimation,
      Widget child,
    );

/// A callback type for informing that a navigation pop has been invoked,
/// whether or not it was handled successfully.
///
/// Accepts a didPop boolean indicating whether or not back navigation
/// succeeded.
///
/// The `result` contains the pop result.
typedef PopInvokedWithResultCallback<T> = void Function(bool didPop, T? result);

/// Allows listening to and preventing pops.
///
/// Can be registered in [ModalRoute] to listen to pops with [onPopInvokedWithResult] or
/// to enable/disable them with [canPopNotifier].
///
/// See also:
///
///  * [PopScope], which provides similar functionality in a widget.
///  * [ModalRoute.registerPopEntry], which unregisters instances of this.
///  * [ModalRoute.unregisterPopEntry], which unregisters instances of this.
abstract class PopEntry<T> {
  /// {@macro flutter.widgets.PopScope.onPopInvokedWithResult}
  @Deprecated(
    'Use onPopInvokedWithResult instead. '
    'This feature was deprecated after v3.22.0-12.0.pre.',
  )
  void onPopInvoked(bool didPop) {}

  /// {@macro flutter.widgets.PopScope.onPopInvokedWithResult}
  void onPopInvokedWithResult(bool didPop, T? result) => onPopInvoked(didPop);

  /// {@macro flutter.widgets.PopScope.canPop}
  ValueListenable<bool> get canPopNotifier;

  @override
  String toString() {
    return 'PopEntry canPop: ${canPopNotifier.value}, onPopInvoked: $onPopInvokedWithResult';
  }
}
