// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/scheduler.dart';
/// @docImport 'package:flutter/widgets.dart';
library;

import 'package:flutter/foundation.dart';

import 'tween.dart';

export 'dart:ui' show VoidCallback;

export 'tween.dart' show Animatable;

// Examples can assume:
// late AnimationController _controller;
// late ValueNotifier<double> _scrollPosition;

/// The status of an animation.
enum AnimationStatus {
  /// The animation is stopped at the beginning.
  dismissed,

  /// The animation is running from beginning to end.
  forward,

  /// The animation is running backwards, from end to beginning.
  reverse,

  /// The animation is stopped at the end.
  completed;

  /// Whether the animation is stopped at the beginning.
  bool get isDismissed => this == dismissed;

  /// Whether the animation is stopped at the end.
  bool get isCompleted => this == completed;

  /// Whether the animation is running in either direction.
  bool get isAnimating => switch (this) {
    forward || reverse => true,
    completed || dismissed => false,
  };

  /// {@template flutter.animation.AnimationStatus.isForwardOrCompleted}
  /// Whether the current aim of the animation is toward completion.
  ///
  /// Specifically, returns `true` for [AnimationStatus.forward] or
  /// [AnimationStatus.completed], and `false` for
  /// [AnimationStatus.reverse] or [AnimationStatus.dismissed].
  /// {@endtemplate}
  bool get isForwardOrCompleted => switch (this) {
    forward || completed => true,
    reverse || dismissed => false,
  };
}

/// Signature for listeners attached using [Animation.addStatusListener].
typedef AnimationStatusListener = void Function(AnimationStatus status);

/// Signature for method used to transform values in [Animation.fromValueListenable].
typedef ValueListenableTransformer<T> = T Function(T);

/// A value which might change over time, moving forward or backward.
///
/// An animation has a [value] (of type [T]) and a [status].
/// The value conceptually lies on some path, and
/// the status indicates how the value is currently moving along the path:
/// forward, backward, or stopped at the end or the beginning.
/// The path may double back on itself
/// (e.g., if the animation uses a curve that bounces),
/// so even when the animation is conceptually moving forward
/// the value might not change monotonically.
///
/// Consumers of the animation can listen for changes to either the value
/// or the status, with [addListener] and [addStatusListener].
/// The listener callbacks are called during the "animation" phase of
/// the pipeline, just prior to rebuilding widgets.
///
/// An animation might move forward or backward on its own as time passes
/// (like the opacity of a button that fades over a fixed duration
/// once the user touches it),
/// or it might be driven by the user
/// (like the position of a slider that the user can drag back and forth),
/// or it might do both
/// (like a switch that snaps into place when released,
/// or a [Dismissible] that responds to drag and fling gestures, etc.).
/// The behavior is normally controlled by method calls on
/// some underlying [AnimationController].
/// When an animation is actively animating, it typically updates on
/// each frame, driven by a [Ticker].
///
/// ## Using animations
///
/// For simple animation effects, consider using one of the
/// [ImplicitlyAnimatedWidget] subclasses,
/// like [AnimatedScale], [AnimatedOpacity], and many others.
/// When an [ImplicitlyAnimatedWidget] suffices, there is
/// no need to work with [Animation] or the rest of the classes
/// discussed in this section.
///
/// Otherwise, typically an animation originates with an [AnimationController]
/// (which is itself an [Animation<double>])
/// created by a [State] that implements [TickerProvider].
/// Further animations might be derived from that animation
/// by using e.g. [Tween] or [CurvedAnimation].
/// The animations might be used to configure an [AnimatedWidget]
/// (using one of its many subclasses like [FadeTransition]),
/// or their values might be used directly.
///
/// For example, the [AnimationController] may represent
/// the abstract progress of the animation from 0.0 to 1.0;
/// then a [CurvedAnimation] might apply an easing curve;
/// and a [SizeTween] and [ColorTween] might each be applied to that
/// to produce an [Animation<Size>] and an [Animation<Color>] that control
/// a widget shrinking and changing color as the animation proceeds.
///
/// ## Performance considerations
///
/// Because the [Animation] keeps the same identity as the animation proceeds,
/// it provides a convenient way for a [StatefulWidget] that orchestrates
/// a complex animation to communicate the animation's progress to its
/// various child widgets.  Consider having higher-level widgets in the tree
/// pass lower-level widgets the [Animation] itself, rather than its value,
/// in order to avoid rebuilding the higher-level widgets on each frame
/// even while the animation is active.
/// If the leaf widgets also ignore [value] and pass the whole
/// [Animation] object to a render object (like [FadeTransition] does),
/// they too might be able to avoid rebuild and even relayout, so that the
/// only work needed on each frame of the animation is to repaint.
///
/// See also:
///
///  * [ImplicitlyAnimatedWidget] and its subclasses, which provide
///    animation effects without the need to manually work with [Animation],
///    [AnimationController], or even [State].
///  * [AnimationController], an animation you can run forward and backward,
///    stop, or set to a specific value.
///  * [Tween], which can be used to convert [Animation<double>]s into
///    other kinds of [Animation]s.
///  * [AnimatedWidget] and its subclasses, which provide animation effects
///    that can be controlled by an [Animation].
abstract class Animation<T> extends Listenable implements ValueListenable<T> {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const Animation();

  /// Create a new animation from a [ValueListenable].
  ///
  /// The returned animation will always have an animation status of
  /// [AnimationStatus.forward]. The value of the provided listenable can
  /// be optionally transformed using the [transformer] function.
  ///
  /// {@tool snippet}
  ///
  /// This constructor can be used to replace instances of [ValueListenableBuilder]
  /// widgets with a corresponding animated widget, like a [FadeTransition].
  ///
  /// Before:
  ///
  /// ```dart
  /// Widget build(BuildContext context) {
  ///   return ValueListenableBuilder<double>(
  ///     valueListenable: _scrollPosition,
  ///     builder: (BuildContext context, double value, Widget? child) {
  ///       final double opacity = (value / 1000).clamp(0, 1);
  ///       return Opacity(opacity: opacity, child: child);
  ///     },
  ///     child: const ColoredBox(
  ///       color: Colors.red,
  ///       child: Text('Hello, Animation'),
  ///     ),
  ///   );
  /// }
  /// ```
  ///
  /// {@end-tool}
  /// {@tool snippet}
  ///
  /// After:
  ///
  /// ```dart
  /// Widget build2(BuildContext context) {
  ///   return FadeTransition(
  ///     opacity: Animation<double>.fromValueListenable(_scrollPosition, transformer: (double value) {
  ///       return (value / 1000).clamp(0, 1);
  ///     }),
  ///     child: const ColoredBox(
  ///       color: Colors.red,
  ///       child: Text('Hello, Animation'),
  ///     ),
  ///   );
  /// }
  /// ```
  /// {@end-tool}
  factory Animation.fromValueListenable(
    ValueListenable<T> listenable, {
    ValueListenableTransformer<T>? transformer,
  }) = _ValueListenableDelegateAnimation<T>;

  // keep these next five dartdocs in sync with the dartdocs in AnimationWithParentMixin<T>

  /// Calls the listener every time the value of the animation changes.
  ///
  /// Listeners can be removed with [removeListener].
  @override
  void addListener(VoidCallback listener);

  /// Stop calling the listener every time the value of the animation changes.
  ///
  /// If `listener` is not currently registered as a listener, this method does
  /// nothing.
  ///
  /// Listeners can be added with [addListener].
  @override
  void removeListener(VoidCallback listener);

  /// Calls listener every time the status of the animation changes.
  ///
  /// Listeners can be removed with [removeStatusListener].
  void addStatusListener(AnimationStatusListener listener);

  /// Stops calling the listener every time the status of the animation changes.
  ///
  /// If `listener` is not currently registered as a status listener, this
  /// method does nothing.
  ///
  /// Listeners can be added with [addStatusListener].
  void removeStatusListener(AnimationStatusListener listener);

  /// The current status of this animation.
  AnimationStatus get status;

  /// The current value of the animation.
  @override
  T get value;

  /// Whether this animation is stopped at the beginning.
  bool get isDismissed => status.isDismissed;

  /// Whether this animation is stopped at the end.
  bool get isCompleted => status.isCompleted;

  /// Whether this animation is running in either direction.
  ///
  /// By default, this value is equal to `status.isAnimating`, but
  /// [AnimationController] overrides this method so that its output
  /// depends on whether the controller is actively ticking.
  bool get isAnimating => status.isAnimating;

  /// {@macro flutter.animation.AnimationStatus.isForwardOrCompleted}
  bool get isForwardOrCompleted => status.isForwardOrCompleted;

  /// Chains a [Tween] (or [CurveTween]) to this [Animation].
  ///
  /// This method is only valid for `Animation<double>` instances (i.e. when `T`
  /// is `double`). This means, for instance, that it can be called on
  /// [AnimationController] objects, as well as [CurvedAnimation]s,
  /// [ProxyAnimation]s, [ReverseAnimation]s, [TrainHoppingAnimation]s, etc.
  ///
  /// It returns an [Animation] specialized to the same type, `U`, as the
  /// argument to the method (`child`), whose value is derived by applying the
  /// given [Tween] to the value of this [Animation].
  ///
  /// {@tool snippet}
  ///
  /// Given an [AnimationController] `_controller`, the following code creates
  /// an `Animation<Alignment>` that swings from top left to top right as the
  /// controller goes from 0.0 to 1.0:
  ///
  /// ```dart
  /// Animation<Alignment> alignment1 = _controller.drive(
  ///   AlignmentTween(
  ///     begin: Alignment.topLeft,
  ///     end: Alignment.topRight,
  ///   ),
  /// );
  /// ```
  /// {@end-tool}
  /// {@tool snippet}
  ///
  /// The `alignment1.value` could then be used in a widget's build method, for
  /// instance, to position a child using an [Align] widget such that the
  /// position of the child shifts over time from the top left to the top right.
  ///
  /// It is common to ease this kind of curve, e.g. making the transition slower
  /// at the start and faster at the end. The following snippet shows one way to
  /// chain the alignment tween in the previous example to an easing curve (in
  /// this case, [Curves.easeIn]). In this example, the tween is created
  /// elsewhere as a variable that can be reused, since none of its arguments
  /// vary.
  ///
  /// ```dart
  /// final Animatable<Alignment> tween = AlignmentTween(begin: Alignment.topLeft, end: Alignment.topRight)
  ///   .chain(CurveTween(curve: Curves.easeIn));
  /// // ...
  /// final Animation<Alignment> alignment2 = _controller.drive(tween);
  /// ```
  /// {@end-tool}
  /// {@tool snippet}
  ///
  /// The following code is exactly equivalent, and is typically clearer when
  /// the tweens are created inline, as might be preferred when the tweens have
  /// values that depend on other variables:
  ///
  /// ```dart
  /// Animation<Alignment> alignment3 = _controller
  ///   .drive(CurveTween(curve: Curves.easeIn))
  ///   .drive(AlignmentTween(
  ///     begin: Alignment.topLeft,
  ///     end: Alignment.topRight,
  ///   ));
  /// ```
  /// {@end-tool}
  /// {@tool snippet}
  ///
  /// This method can be paired with an [Animatable] created via
  /// [Animatable.fromCallback] in order to transform an animation with a
  /// callback function. This can be useful for performing animations that
  /// do not have well defined start or end points. This example transforms
  /// the current scroll position into a color that cycles through values
  /// of red.
  ///
  /// ```dart
  /// Animation<Color> color = Animation<double>.fromValueListenable(_scrollPosition)
  ///   .drive(Animatable<Color>.fromCallback((double value) {
  ///     return Color.fromRGBO(value.round() % 255, 0, 0, 1);
  ///   }));
  /// ```
  ///
  /// {@end-tool}
  ///
  /// See also:
  ///
  ///  * [Animatable.animate], which does the same thing.
  ///  * [AnimationController], which is usually used to drive animations.
  ///  * [CurvedAnimation], an alternative to [CurveTween] for applying easing
  ///    curves, which supports distinct curves in the forward direction and the
  ///    reverse direction.
  ///  * [Animatable.fromCallback], which allows creating an [Animatable] from an
  ///    arbitrary transformation.
  @optionalTypeArgs
  Animation<U> drive<U>(Animatable<U> child) {
    assert(this is Animation<double>);
    return child.animate(this as Animation<double>);
  }

  @override
  String toString() {
    return '${describeIdentity(this)}(${toStringDetails()})';
  }

  /// Provides a string describing the status of this object, but not including
  /// information about the object itself.
  ///
  /// This function is used by [Animation.toString] so that [Animation]
  /// subclasses can provide additional details while ensuring all [Animation]
  /// subclasses have a consistent [toString] style.
  ///
  /// The result of this function includes an icon describing the status of this
  /// [Animation] object:
  ///
  /// * "&#x25B6;": [AnimationStatus.forward] ([value] increasing)
  /// * "&#x25C0;": [AnimationStatus.reverse] ([value] decreasing)
  /// * "&#x23ED;": [AnimationStatus.completed] ([value] == 1.0)
  /// * "&#x23EE;": [AnimationStatus.dismissed] ([value] == 0.0)
  String toStringDetails() {
    return switch (status) {
      AnimationStatus.forward => '\u25B6', // >
      AnimationStatus.reverse => '\u25C0', // <
      AnimationStatus.completed => '\u23ED', // >>|
      AnimationStatus.dismissed => '\u23EE', // |<<
    };
  }
}

// An implementation of an animation that delegates to a value listenable with a fixed direction.
class _ValueListenableDelegateAnimation<T> extends Animation<T> {
  _ValueListenableDelegateAnimation(this._listenable, {ValueListenableTransformer<T>? transformer})
    : _transformer = transformer;

  final ValueListenable<T> _listenable;
  final ValueListenableTransformer<T>? _transformer;

  @override
  void addListener(VoidCallback listener) {
    _listenable.addListener(listener);
  }

  @override
  void addStatusListener(AnimationStatusListener listener) {
    // status will never change.
  }

  @override
  void removeListener(VoidCallback listener) {
    _listenable.removeListener(listener);
  }

  @override
  void removeStatusListener(AnimationStatusListener listener) {
    // status will never change.
  }

  @override
  AnimationStatus get status => AnimationStatus.forward;

  @override
  T get value => _transformer?.call(_listenable.value) ?? _listenable.value;
}
