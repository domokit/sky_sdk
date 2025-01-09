// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
library;

import 'dart:math' as math;

import 'package:flutter/foundation.dart';

import 'animation.dart';
import 'curves.dart';
import 'listener_helpers.dart';

export 'dart:ui' show VoidCallback;

export 'animation.dart' show Animation, AnimationStatus, AnimationStatusListener;
export 'curves.dart' show Curve;

// Examples can assume:
// late AnimationController controller;

class _AlwaysCompleteAnimation extends Animation<double> {
  const _AlwaysCompleteAnimation();

  @override
  void addListener(VoidCallback listener) {}

  @override
  void removeListener(VoidCallback listener) {}

  @override
  void addStatusListener(AnimationStatusListener listener) {}

  @override
  void removeStatusListener(AnimationStatusListener listener) {}

  @override
  AnimationStatus get status => AnimationStatus.completed;

  @override
  double get value => 1.0;

  @override
  String toString() => 'kAlwaysCompleteAnimation';
}

/// An animation that is always complete.
///
/// Using this constant involves less overhead than building an
/// [AnimationController] with an initial value of 1.0. This is useful when an
/// API expects an animation but you don't actually want to animate anything.
const Animation<double> kAlwaysCompleteAnimation = _AlwaysCompleteAnimation();

class _AlwaysDismissedAnimation extends Animation<double> {
  const _AlwaysDismissedAnimation();

  @override
  void addListener(VoidCallback listener) {}

  @override
  void removeListener(VoidCallback listener) {}

  @override
  void addStatusListener(AnimationStatusListener listener) {}

  @override
  void removeStatusListener(AnimationStatusListener listener) {}

  @override
  AnimationStatus get status => AnimationStatus.dismissed;

  @override
  double get value => 0.0;

  @override
  String toString() => 'kAlwaysDismissedAnimation';
}

/// An animation that is always dismissed.
///
/// Using this constant involves less overhead than building an
/// [AnimationController] with an initial value of 0.0. This is useful when an
/// API expects an animation but you don't actually want to animate anything.
const Animation<double> kAlwaysDismissedAnimation = _AlwaysDismissedAnimation();

/// An animation that is always stopped at a given value.
///
/// The [status] is always [AnimationStatus.forward].
class AlwaysStoppedAnimation<T> extends Animation<T> {
  /// Creates an [AlwaysStoppedAnimation] with the given value.
  ///
  /// Since the [value] and [status] of an [AlwaysStoppedAnimation] can never
  /// change, the listeners can never be called. It is therefore safe to reuse
  /// an [AlwaysStoppedAnimation] instance in multiple places. If the [value] to
  /// be used is known at compile time, the constructor should be called as a
  /// `const` constructor.
  const AlwaysStoppedAnimation(this.value);

  @override
  final T value;

  @override
  void addListener(VoidCallback listener) {}

  @override
  void removeListener(VoidCallback listener) {}

  @override
  void addStatusListener(AnimationStatusListener listener) {}

  @override
  void removeStatusListener(AnimationStatusListener listener) {}

  @override
  AnimationStatus get status => AnimationStatus.forward;

  @override
  String toStringDetails() {
    return '${super.toStringDetails()} $value; paused';
  }
}

/// Implements most of the [Animation] interface by deferring its behavior to a
/// given [parent] Animation.
///
/// To implement an [Animation] that is driven by a parent, it is only necessary
/// to mix in this class, implement [parent], and implement `T get value`.
///
/// To define a mapping from values in the range 0..1, consider subclassing
/// [Tween] instead.
mixin AnimationWithParentMixin<T> {
  /// The animation whose value this animation will proxy.
  ///
  /// This animation must remain the same for the lifetime of this object. If
  /// you wish to proxy a different animation at different times, consider using
  /// [ProxyAnimation].
  Animation<T> get parent;

  // keep these next five dartdocs in sync with the dartdocs in Animation<T>

  /// Calls the listener every time the value of the animation changes.
  ///
  /// Listeners can be removed with [removeListener].
  void addListener(VoidCallback listener) => parent.addListener(listener);

  /// Stop calling the listener every time the value of the animation changes.
  ///
  /// Listeners can be added with [addListener].
  void removeListener(VoidCallback listener) => parent.removeListener(listener);

  /// Calls listener every time the status of the animation changes.
  ///
  /// Listeners can be removed with [removeStatusListener].
  void addStatusListener(AnimationStatusListener listener) => parent.addStatusListener(listener);

  /// Stops calling the listener every time the status of the animation changes.
  ///
  /// Listeners can be added with [addStatusListener].
  void removeStatusListener(AnimationStatusListener listener) =>
      parent.removeStatusListener(listener);

  /// The current status of this animation.
  AnimationStatus get status => parent.status;
}

/// An animation that is a proxy for another animation.
///
/// A proxy animation is useful because the parent animation can be mutated. For
/// example, one object can create a proxy animation, hand the proxy to another
/// object, and then later change the animation from which the proxy receives
/// its value.
class ProxyAnimation extends Animation<double>
    with
        AnimationLazyListenerMixin,
        AnimationLocalListenersMixin,
        AnimationLocalStatusListenersMixin {
  /// Creates a proxy animation.
  ///
  /// If the animation argument is omitted, the proxy animation will have the
  /// status [AnimationStatus.dismissed] and a value of 0.0.
  ProxyAnimation([Animation<double>? animation]) {
    _parent = animation;
    if (_parent == null) {
      _status = AnimationStatus.dismissed;
      _value = 0.0;
    }
  }

  AnimationStatus? _status;
  double? _value;

  /// The animation whose value this animation will proxy.
  ///
  /// This value is mutable. When mutated, the listeners on the proxy animation
  /// will be transparently updated to be listening to the new parent animation.
  Animation<double>? get parent => _parent;
  Animation<double>? _parent;
  set parent(Animation<double>? value) {
    if (value == _parent) {
      return;
    }
    if (_parent != null) {
      _status = _parent!.status;
      _value = _parent!.value;
      if (isListening) {
        didStopListening();
      }
    }
    _parent = value;
    if (_parent != null) {
      if (isListening) {
        didStartListening();
      }
      if (_value != _parent!.value) {
        notifyListeners();
      }
      if (_status != _parent!.status) {
        notifyStatusListeners(_parent!.status);
      }
      _status = null;
      _value = null;
    }
  }

  @override
  void didStartListening() {
    if (_parent != null) {
      _parent!.addListener(notifyListeners);
      _parent!.addStatusListener(notifyStatusListeners);
    }
  }

  @override
  void didStopListening() {
    if (_parent != null) {
      _parent!.removeListener(notifyListeners);
      _parent!.removeStatusListener(notifyStatusListeners);
    }
  }

  @override
  AnimationStatus get status => _parent != null ? _parent!.status : _status!;

  @override
  double get value => _parent != null ? _parent!.value : _value!;

  @override
  String toString() {
    if (parent == null) {
      return '${objectRuntimeType(this, 'ProxyAnimation')}(null; ${super.toStringDetails()} ${value.toStringAsFixed(3)})';
    }
    return '$parent\u27A9${objectRuntimeType(this, 'ProxyAnimation')}';
  }
}

/// An animation that is the reverse of another animation.
///
/// If the parent animation is running forward from 0.0 to 1.0, this animation
/// is running in reverse from 1.0 to 0.0.
///
/// Using a [ReverseAnimation] is different from using a [Tween] with a
/// `begin` of 1.0 and an `end` of 0.0 because the tween does not change the status
/// or direction of the animation.
///
/// See also:
///
///  * [Curve.flipped] and [FlippedCurve], which provide a similar effect but on
///    [Curve]s.
///  * [CurvedAnimation], which can take separate curves for when the animation
///    is going forward than for when it is going in reverse.
class ReverseAnimation extends Animation<double>
    with AnimationLazyListenerMixin, AnimationLocalStatusListenersMixin {
  /// Creates a reverse animation.
  ReverseAnimation(this.parent);

  /// The animation whose value and direction this animation is reversing.
  final Animation<double> parent;

  @override
  void addListener(VoidCallback listener) {
    didRegisterListener();
    parent.addListener(listener);
  }

  @override
  void removeListener(VoidCallback listener) {
    parent.removeListener(listener);
    didUnregisterListener();
  }

  @override
  void didStartListening() {
    parent.addStatusListener(_statusChangeHandler);
  }

  @override
  void didStopListening() {
    parent.removeStatusListener(_statusChangeHandler);
  }

  void _statusChangeHandler(AnimationStatus status) {
    notifyStatusListeners(_reverseStatus(status));
  }

  @override
  AnimationStatus get status => _reverseStatus(parent.status);

  @override
  double get value => 1.0 - parent.value;

  AnimationStatus _reverseStatus(AnimationStatus status) {
    return switch (status) {
      AnimationStatus.forward => AnimationStatus.reverse,
      AnimationStatus.reverse => AnimationStatus.forward,
      AnimationStatus.completed => AnimationStatus.dismissed,
      AnimationStatus.dismissed => AnimationStatus.completed,
    };
  }

  @override
  String toString() {
    return '$parent\u27AA${objectRuntimeType(this, 'ReverseAnimation')}';
  }
}

/// An animation that applies a curve to another animation.
///
/// [CurvedAnimation] is useful when you want to apply a non-linear [Curve] to
/// an animation object, especially if you want different curves when the
/// animation is going forward vs when it is going backward.
///
/// Depending on the given curve, the output of the [CurvedAnimation] could have
/// a wider range than its input. For example, elastic curves such as
/// [Curves.elasticIn] will significantly overshoot or undershoot the default
/// range of 0.0 to 1.0.
///
/// If you want to apply a [Curve] to a [Tween], consider using [CurveTween].
///
/// {@tool snippet}
///
/// The following code snippet shows how you can apply a curve to a linear
/// animation produced by an [AnimationController] `controller`.
///
/// ```dart
/// final Animation<double> animation = CurvedAnimation(
///   parent: controller,
///   curve: Curves.ease,
/// );
/// ```
/// {@end-tool}
/// {@tool snippet}
///
/// This second code snippet shows how to apply a different curve in the forward
/// direction than in the reverse direction. This can't be done using a
/// [CurveTween] (since [Tween]s are not aware of the animation direction when
/// they are applied).
///
/// ```dart
/// final Animation<double> animation = CurvedAnimation(
///   parent: controller,
///   curve: Curves.easeIn,
///   reverseCurve: Curves.easeOut,
/// );
/// ```
/// {@end-tool}
///
/// By default, the [reverseCurve] matches the forward [curve].
///
/// See also:
///
///  * [CurveTween], for an alternative way of expressing the first sample
///    above.
///  * [AnimationController], for examples of creating and disposing of an
///    [AnimationController].
///  * [Curve.flipped] and [FlippedCurve], which provide the reverse of a
///    [Curve].
class CurvedAnimation extends Animation<double> with AnimationWithParentMixin<double> {
  /// Creates a curved animation.
  CurvedAnimation({required this.parent, required this.curve, this.reverseCurve}) {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectCreated(
        library: 'package:flutter/animation.dart',
        className: '$CurvedAnimation',
        object: this,
      );
    }
    _updateCurveDirection(parent.status);
    parent.addStatusListener(_updateCurveDirection);
  }

  /// The animation to which this animation applies a curve.
  @override
  final Animation<double> parent;

  /// The curve to use in the forward direction.
  Curve curve;

  /// The curve to use in the reverse direction.
  ///
  /// If the parent animation changes direction without first reaching the
  /// [AnimationStatus.completed] or [AnimationStatus.dismissed] status, the
  /// [CurvedAnimation] stays on the same curve (albeit in the opposite
  /// direction) to avoid visual discontinuities.
  ///
  /// If you use a non-null [reverseCurve], you might want to hold this object
  /// in a [State] object rather than recreating it each time your widget builds
  /// in order to take advantage of the state in this object that avoids visual
  /// discontinuities.
  ///
  /// If this field is null, uses [curve] in both directions.
  Curve? reverseCurve;

  /// The direction used to select the current curve.
  ///
  /// The curve direction is only reset when we hit the beginning or the end of
  /// the timeline to avoid discontinuities in the value of any variables this
  /// animation is used to animate.
  AnimationStatus? _curveDirection;

  /// True if this CurvedAnimation has been disposed.
  bool isDisposed = false;

  void _updateCurveDirection(AnimationStatus status) {
    _curveDirection = status.isAnimating ? _curveDirection ?? status : null;
  }

  bool get _useForwardCurve {
    return reverseCurve == null || (_curveDirection ?? parent.status) != AnimationStatus.reverse;
  }

  /// Cleans up any listeners added by this CurvedAnimation.
  void dispose() {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectDisposed(object: this);
    }
    isDisposed = true;
    parent.removeStatusListener(_updateCurveDirection);
  }

  @override
  double get value {
    final Curve? activeCurve = _useForwardCurve ? curve : reverseCurve;

    final double t = parent.value;
    if (activeCurve == null) {
      return t;
    }
    if (t == 0.0 || t == 1.0) {
      assert(() {
        final double transformedValue = activeCurve.transform(t);
        final double roundedTransformedValue = transformedValue.round().toDouble();
        if (roundedTransformedValue != t) {
          throw FlutterError(
            'Invalid curve endpoint at $t.\n'
            'Curves must map 0.0 to near zero and 1.0 to near one but '
            '${activeCurve.runtimeType} mapped $t to $transformedValue, which '
            'is near $roundedTransformedValue.',
          );
        }
        return true;
      }());
      return t;
    }
    return activeCurve.transform(t);
  }

  @override
  String toString() {
    if (reverseCurve == null) {
      return '$parent\u27A9$curve';
    }
    if (_useForwardCurve) {
      return '$parent\u27A9$curve\u2092\u2099/$reverseCurve';
    }
    return '$parent\u27A9$curve/$reverseCurve\u2092\u2099';
  }
}

enum _TrainHoppingMode { minimize, maximize }

/// This animation starts by proxying one animation, but when the value of that
/// animation crosses the value of the second (either because the second is
/// going in the opposite direction, or because the one overtakes the other),
/// the animation hops over to proxying the second animation.
///
/// When the [TrainHoppingAnimation] starts proxying the second animation
/// instead of the first, the [onSwitchedTrain] callback is called.
///
/// If the two animations start at the same value, then the
/// [TrainHoppingAnimation] immediately hops to the second animation, and the
/// [onSwitchedTrain] callback is not called. If only one animation is provided
/// (i.e. if the second is null), then the [TrainHoppingAnimation] just proxies
/// the first animation.
///
/// Since this object must track the two animations even when it has no
/// listeners of its own, instead of shutting down when all its listeners are
/// removed, it exposes a [dispose()] method. Call this method to shut this
/// object down.
class TrainHoppingAnimation extends Animation<double>
    with
        AnimationEagerListenerMixin,
        AnimationLocalListenersMixin,
        AnimationLocalStatusListenersMixin {
  /// Creates a train-hopping animation.
  ///
  /// The current train argument must not be null but the next train argument
  /// can be null. If the next train is null, then this object will just proxy
  /// the first animation and never hop.
  TrainHoppingAnimation(
    Animation<double> this._currentTrain,
    this._nextTrain, {
    this.onSwitchedTrain,
  }) {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectCreated(
        library: 'package:flutter/animation.dart',
        className: '$TrainHoppingAnimation',
        object: this,
      );
    }
    if (_nextTrain != null) {
      if (_currentTrain!.value == _nextTrain!.value) {
        _currentTrain = _nextTrain;
        _nextTrain = null;
      } else if (_currentTrain!.value > _nextTrain!.value) {
        _mode = _TrainHoppingMode.maximize;
      } else {
        assert(_currentTrain!.value < _nextTrain!.value);
        _mode = _TrainHoppingMode.minimize;
      }
    }
    _currentTrain!.addStatusListener(_statusChangeHandler);
    _currentTrain!.addListener(_valueChangeHandler);
    _nextTrain?.addListener(_valueChangeHandler);
    assert(_mode != null || _nextTrain == null);
  }

  /// The animation that is currently driving this animation.
  ///
  /// The identity of this object will change from the first animation to the
  /// second animation when [onSwitchedTrain] is called.
  Animation<double>? get currentTrain => _currentTrain;
  Animation<double>? _currentTrain;
  Animation<double>? _nextTrain;
  _TrainHoppingMode? _mode;

  /// Called when this animation switches to be driven by the second animation.
  ///
  /// This is not called if the two animations provided to the constructor have
  /// the same value at the time of the call to the constructor. In that case,
  /// the second animation is used from the start, and the first is ignored.
  VoidCallback? onSwitchedTrain;

  AnimationStatus? _lastStatus;
  void _statusChangeHandler(AnimationStatus status) {
    assert(_currentTrain != null);
    if (status != _lastStatus) {
      notifyListeners();
      _lastStatus = status;
    }
    assert(_lastStatus != null);
  }

  @override
  AnimationStatus get status => _currentTrain!.status;

  double? _lastValue;
  void _valueChangeHandler() {
    assert(_currentTrain != null);
    bool hop = false;
    if (_nextTrain != null) {
      assert(_mode != null);
      hop = switch (_mode!) {
        _TrainHoppingMode.minimize => _nextTrain!.value <= _currentTrain!.value,
        _TrainHoppingMode.maximize => _nextTrain!.value >= _currentTrain!.value,
      };
      if (hop) {
        _currentTrain!
          ..removeStatusListener(_statusChangeHandler)
          ..removeListener(_valueChangeHandler);
        _currentTrain = _nextTrain;
        _nextTrain = null;
        _currentTrain!.addStatusListener(_statusChangeHandler);
        _statusChangeHandler(_currentTrain!.status);
      }
    }
    final double newValue = value;
    if (newValue != _lastValue) {
      notifyListeners();
      _lastValue = newValue;
    }
    assert(_lastValue != null);
    if (hop && onSwitchedTrain != null) {
      onSwitchedTrain!();
    }
  }

  @override
  double get value => _currentTrain!.value;

  /// Frees all the resources used by this performance.
  /// After this is called, this object is no longer usable.
  @override
  void dispose() {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectDisposed(object: this);
    }
    assert(_currentTrain != null);
    _currentTrain!.removeStatusListener(_statusChangeHandler);
    _currentTrain!.removeListener(_valueChangeHandler);
    _currentTrain = null;
    _nextTrain?.removeListener(_valueChangeHandler);
    _nextTrain = null;
    clearListeners();
    clearStatusListeners();
    super.dispose();
  }

  @override
  String toString() {
    if (_nextTrain != null) {
      return '$currentTrain\u27A9${objectRuntimeType(this, 'TrainHoppingAnimation')}(next: $_nextTrain)';
    }
    return '$currentTrain\u27A9${objectRuntimeType(this, 'TrainHoppingAnimation')}(no next)';
  }
}

/// An interface for combining multiple Animations. Subclasses need only
/// implement the `value` getter to control how the child animations are
/// combined. Can be chained to combine more than 2 animations.
///
/// For example, to create an animation that is the sum of two others, subclass
/// this class and define `T get value = first.value + second.value;`
///
/// By default, the [status] of a [CompoundAnimation] is the status of the
/// [next] animation if [next] is moving, and the status of the [first]
/// animation otherwise.
abstract class CompoundAnimation<T> extends Animation<T>
    with
        AnimationLazyListenerMixin,
        AnimationLocalListenersMixin,
        AnimationLocalStatusListenersMixin {
  /// Creates a [CompoundAnimation].
  ///
  /// Either argument can be a [CompoundAnimation] itself to combine multiple animations.
  CompoundAnimation({required this.first, required this.next});

  /// The first sub-animation. Its status takes precedence if neither are
  /// animating.
  final Animation<T> first;

  /// The second sub-animation.
  final Animation<T> next;

  @override
  void didStartListening() {
    first.addListener(_maybeNotifyListeners);
    first.addStatusListener(_maybeNotifyStatusListeners);
    next.addListener(_maybeNotifyListeners);
    next.addStatusListener(_maybeNotifyStatusListeners);
  }

  @override
  void didStopListening() {
    first.removeListener(_maybeNotifyListeners);
    first.removeStatusListener(_maybeNotifyStatusListeners);
    next.removeListener(_maybeNotifyListeners);
    next.removeStatusListener(_maybeNotifyStatusListeners);
  }

  /// Gets the status of this animation based on the [first] and [next] status.
  ///
  /// The default is that if the [next] animation is moving, use its status.
  /// Otherwise, default to [first].
  @override
  AnimationStatus get status => next.status.isAnimating ? next.status : first.status;

  @override
  String toString() {
    return '${objectRuntimeType(this, 'CompoundAnimation')}($first, $next)';
  }

  AnimationStatus? _lastStatus;
  void _maybeNotifyStatusListeners(AnimationStatus _) {
    if (status != _lastStatus) {
      _lastStatus = status;
      notifyStatusListeners(status);
    }
  }

  T? _lastValue;
  void _maybeNotifyListeners() {
    if (value != _lastValue) {
      _lastValue = value;
      notifyListeners();
    }
  }
}

/// An animation of [double]s that tracks the mean of two other animations.
///
/// The [status] of this animation is the status of the `right` animation if it is
/// moving, and the `left` animation otherwise.
///
/// The [value] of this animation is the [double] that represents the mean value
/// of the values of the `left` and `right` animations.
class AnimationMean extends CompoundAnimation<double> {
  /// Creates an animation that tracks the mean of two other animations.
  AnimationMean({required Animation<double> left, required Animation<double> right})
    : super(first: left, next: right);

  @override
  double get value => (first.value + next.value) / 2.0;
}

/// An animation that tracks the maximum of two other animations.
///
/// The [value] of this animation is the maximum of the values of
/// [first] and [next].
class AnimationMax<T extends num> extends CompoundAnimation<T> {
  /// Creates an [AnimationMax].
  ///
  /// Either argument can be an [AnimationMax] itself to combine multiple
  /// animations.
  AnimationMax(Animation<T> first, Animation<T> next) : super(first: first, next: next);

  @override
  T get value => math.max(first.value, next.value);
}

/// An animation that tracks the minimum of two other animations.
///
/// The [value] of this animation is the minimum of the values of
/// [first] and [next].
class AnimationMin<T extends num> extends CompoundAnimation<T> {
  /// Creates an [AnimationMin].
  ///
  /// Either argument can be an [AnimationMin] itself to combine multiple
  /// animations.
  AnimationMin(Animation<T> first, Animation<T> next) : super(first: first, next: next);

  @override
  T get value => math.min(first.value, next.value);
}
