// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'tween.dart';

export 'tween.dart' show Animatable;

// Examples can assume:
// late AnimationController myAnimationController;

/// Enables creating an [Animation] whose value is defined by a sequence of
/// [Tween]s.
///
/// Each [TweenSequenceItem] has a weight that defines its percentage of the
/// animation's duration. Each tween defines the animation's value during the
/// interval indicated by its weight.
///
/// {@tool snippet}
/// This example defines an animation that uses an easing curve to interpolate
/// between 5.0 and 10.0 during the first 40% of the animation, remains at 10.0
/// for the next 20%, and then returns to 5.0 for the final 40%.
///
/// ```dart
/// final Animation<double> animation = TweenSequence<double>(
///   <TweenSequenceItem<double>>[
///     TweenSequenceItem<double>(
///       tween: Tween<double>(begin: 5.0, end: 10.0)
///         .chain(CurveTween(curve: Curves.ease)),
///       weight: 40.0,
///     ),
///     TweenSequenceItem<double>(
///       tween: ConstantTween<double>(10.0),
///       weight: 20.0,
///     ),
///     TweenSequenceItem<double>(
///       tween: Tween<double>(begin: 10.0, end: 5.0)
///         .chain(CurveTween(curve: Curves.ease)),
///       weight: 40.0,
///     ),
///   ],
/// ).animate(myAnimationController);
/// ```
/// {@end-tool}
class TweenSequence<T> extends Animatable<T> {
  /// Construct a TweenSequence.
  ///
  /// The [items] parameter must be a list of one or more [TweenSequenceItem]s.
  ///
  /// There's a small cost associated with building a [TweenSequence] so it's
  /// best to reuse one, rather than rebuilding it on every frame, when that's
  /// possible.
  TweenSequence(List<TweenSequenceItem<T>> items) : assert(items.isNotEmpty) {
    _items.addAll(items);

    double totalWeight = 0.0;
    for (final TweenSequenceItem<T> item in _items) {
      totalWeight += item.weight;
    }
    assert(totalWeight > 0.0);

    double start = 0.0;
    for (int i = 0; i < _items.length; i += 1) {
      final double end = i == _items.length - 1 ? 1.0 : start + _items[i].weight / totalWeight;
      _intervals.add(_Interval(start, end));
      start = end;
    }
  }

  final List<TweenSequenceItem<T>> _items = <TweenSequenceItem<T>>[];
  final List<_Interval> _intervals = <_Interval>[];

  T _evaluateAt(double t, int index) {
    final TweenSequenceItem<T> element = _items[index];
    final double tInterval = _intervals[index].value(t);
    return element.tween.transform(tInterval);
  }

  @override
  T transform(double t) {
    assert(t >= 0.0 && t <= 1.0);
    if (t == 1.0) {
      return _evaluateAt(t, _items.length - 1);
    }
    for (int index = 0; index < _items.length; index++) {
      if (_intervals[index].contains(t)) {
        return _evaluateAt(t, index);
      }
    }
    // Should be unreachable.
    throw StateError('TweenSequence.evaluate() could not find an interval for $t');
  }

  @override
  String toString() => 'TweenSequence(${_items.length} items)';
}

/// Enables creating a flipped [Animation] whose value is defined by a sequence
/// of [Tween]s.
///
/// This creates a [TweenSequence] that evaluates to a result that flips the
/// tween both horizontally and vertically.
///
/// This tween sequence assumes that the evaluated result has to be a double
/// between 0.0 and 1.0.
class FlippedTweenSequence extends TweenSequence<double> {
  /// Creates a flipped [TweenSequence].
  ///
  /// The [items] parameter must be a list of one or more [TweenSequenceItem]s.
  ///
  /// There's a small cost associated with building a `TweenSequence` so it's
  /// best to reuse one, rather than rebuilding it on every frame, when that's
  /// possible.
  FlippedTweenSequence(super.items);

  @override
  double transform(double t) => 1 - super.transform(1 - t);
}

/// A simple holder for one element of a [TweenSequence].
class TweenSequenceItem<T> {
  /// Construct a TweenSequenceItem.
  ///
  /// The [weight] must be greater than 0.0.
  const TweenSequenceItem({required this.tween, required this.weight}) : assert(weight > 0.0);

  /// Defines the value of the [TweenSequence] for the interval within the
  /// animation's duration indicated by [weight] and this item's position
  /// in the list of items.
  ///
  /// {@tool snippet}
  ///
  /// The value of this item can be "curved" by chaining it to a [CurveTween].
  /// For example to create a tween that eases from 0.0 to 10.0:
  ///
  /// ```dart
  /// Tween<double>(begin: 0.0, end: 10.0)
  ///   .chain(CurveTween(curve: Curves.ease))
  /// ```
  /// {@end-tool}
  final Animatable<T> tween;

  /// An arbitrary value that indicates the relative percentage of a
  /// [TweenSequence] animation's duration when [tween] will be used.
  ///
  /// The percentage for an individual item is the item's weight divided by the
  /// sum of all of the items' weights.
  final double weight;
}

class _Interval {
  const _Interval(this.start, this.end) : assert(end > start);

  final double start;
  final double end;

  bool contains(double t) => t >= start && t < end;

  double value(double t) => (t - start) / (end - start);

  @override
  String toString() => '<$start, $end>';
}
