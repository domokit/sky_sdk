// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';

import 'binding.dart';
import 'events.dart';
import 'lsq_solver.dart';

export 'dart:ui' show Offset, PointerDeviceKind;

/// A velocity in two dimensions.
@immutable
class Velocity {
  /// Creates a [Velocity].
  const Velocity({required this.pixelsPerSecond});

  /// A velocity that isn't moving at all.
  static const Velocity zero = Velocity(pixelsPerSecond: Offset.zero);

  /// The number of pixels per second of velocity in the x and y directions.
  final Offset pixelsPerSecond;

  /// Return the negation of a velocity.
  Velocity operator -() => Velocity(pixelsPerSecond: -pixelsPerSecond);

  /// Return the difference of two velocities.
  Velocity operator -(Velocity other) {
    return Velocity(pixelsPerSecond: pixelsPerSecond - other.pixelsPerSecond);
  }

  /// Return the sum of two velocities.
  Velocity operator +(Velocity other) {
    return Velocity(pixelsPerSecond: pixelsPerSecond + other.pixelsPerSecond);
  }

  /// Return a velocity whose magnitude has been clamped to [minValue]
  /// and [maxValue].
  ///
  /// If the magnitude of this Velocity is less than minValue then return a new
  /// Velocity with the same direction and with magnitude [minValue]. Similarly,
  /// if the magnitude of this Velocity is greater than maxValue then return a
  /// new Velocity with the same direction and magnitude [maxValue].
  ///
  /// If the magnitude of this Velocity is within the specified bounds then
  /// just return this.
  Velocity clampMagnitude(double minValue, double maxValue) {
    assert(minValue >= 0.0);
    assert(maxValue >= 0.0 && maxValue >= minValue);
    final double valueSquared = pixelsPerSecond.distanceSquared;
    if (valueSquared > maxValue * maxValue) {
      return Velocity(pixelsPerSecond: (pixelsPerSecond / pixelsPerSecond.distance) * maxValue);
    }
    if (valueSquared < minValue * minValue) {
      return Velocity(pixelsPerSecond: (pixelsPerSecond / pixelsPerSecond.distance) * minValue);
    }
    return this;
  }

  @override
  bool operator ==(Object other) {
    return other is Velocity && other.pixelsPerSecond == pixelsPerSecond;
  }

  @override
  int get hashCode => pixelsPerSecond.hashCode;

  @override
  String toString() =>
      'Velocity(${pixelsPerSecond.dx.toStringAsFixed(1)}, ${pixelsPerSecond.dy.toStringAsFixed(1)})';
}

/// A two dimensional velocity estimate.
///
/// VelocityEstimates are computed by [VelocityTracker.getVelocityEstimate]. An
/// estimate's [confidence] measures how well the velocity tracker's position
/// data fit a straight line, [duration] is the time that elapsed between the
/// first and last position sample used to compute the velocity, and [offset]
/// is similarly the difference between the first and last positions.
///
/// See also:
///
///  * [VelocityTracker], which computes [VelocityEstimate]s.
///  * [Velocity], which encapsulates (just) a velocity vector and provides some
///    useful velocity operations.
class VelocityEstimate {
  /// Creates a dimensional velocity estimate.
  const VelocityEstimate({
    required this.pixelsPerSecond,
    required this.confidence,
    required this.duration,
    required this.offset,
  });

  /// The number of pixels per second of velocity in the x and y directions.
  final Offset pixelsPerSecond;

  /// A value between 0.0 and 1.0 that indicates how well [VelocityTracker]
  /// was able to fit a straight line to its position data.
  ///
  /// The value of this property is 1.0 for a perfect fit, 0.0 for a poor fit.
  final double confidence;

  /// The time that elapsed between the first and last position sample used
  /// to compute [pixelsPerSecond].
  final Duration duration;

  /// The difference between the first and last position sample used
  /// to compute [pixelsPerSecond].
  final Offset offset;

  @override
  String toString() =>
      'VelocityEstimate(${pixelsPerSecond.dx.toStringAsFixed(1)}, ${pixelsPerSecond.dy.toStringAsFixed(1)}; offset: $offset, duration: $duration, confidence: ${confidence.toStringAsFixed(1)})';
}

class _PointAtTime {
  const _PointAtTime(this.point, this.time);

  final Duration time;
  final Offset point;

  @override
  String toString() => '_PointAtTime($point at $time)';
}

/// Computes a pointer's velocity based on data from [PointerMoveEvent]s.
///
/// The input data is provided by calling [addPosition]. Adding data is cheap.
///
/// To obtain a velocity, call [getVelocity] or [getVelocityEstimate]. This will
/// compute the velocity based on the data added so far. Only call these when
/// you need to use the velocity, as they are comparatively expensive.
///
/// The quality of the velocity estimation will be better if more data points
/// have been received.
class VelocityTracker {
  /// Create a new velocity tracker for a pointer [kind].
  VelocityTracker.withKind(this.kind);

  static const int _assumePointerMoveStoppedMilliseconds = 40;
  static const int _historySize = 20;
  static const int _horizonMilliseconds = 100;
  static const int _minSampleSize = 3;

  /// The kind of pointer this tracker is for.
  final PointerDeviceKind kind;

  // Time difference since the last sample was added
  Stopwatch get _sinceLastSample {
    _stopwatch ??= GestureBinding.instance.samplingClock.stopwatch();
    return _stopwatch!;
  }

  Stopwatch? _stopwatch;

  // Circular buffer; current sample at _index.
  final List<_PointAtTime?> _samples = List<_PointAtTime?>.filled(_historySize, null);
  int _index = 0;

  /// Adds a position as the given time to the tracker.
  void addPosition(Duration time, Offset position) {
    _sinceLastSample.start();
    _sinceLastSample.reset();
    _index += 1;
    if (_index == _historySize) {
      _index = 0;
    }
    _samples[_index] = _PointAtTime(position, time);
  }

  /// Returns an estimate of the velocity of the object being tracked by the
  /// tracker given the current information available to the tracker.
  ///
  /// Information is added using [addPosition].
  ///
  /// Returns null if there is no data on which to base an estimate.
  VelocityEstimate? getVelocityEstimate() {
    // Has user recently moved since last sample?
    if (_sinceLastSample.elapsedMilliseconds > _assumePointerMoveStoppedMilliseconds) {
      return const VelocityEstimate(
        pixelsPerSecond: Offset.zero,
        confidence: 1.0,
        duration: Duration.zero,
        offset: Offset.zero,
      );
    }

    final List<double> x = <double>[];
    final List<double> y = <double>[];
    final List<double> w = <double>[];
    final List<double> time = <double>[];
    int sampleCount = 0;
    int index = _index;

    final _PointAtTime? newestSample = _samples[index];
    if (newestSample == null) {
      return null;
    }

    _PointAtTime previousSample = newestSample;
    _PointAtTime oldestSample = newestSample;

    // Starting with the most recent PointAtTime sample, iterate backwards while
    // the samples represent continuous motion.
    do {
      final _PointAtTime? sample = _samples[index];
      if (sample == null) {
        break;
      }

      final double age = (newestSample.time - sample.time).inMicroseconds.toDouble() / 1000;
      final double delta =
          (sample.time - previousSample.time).inMicroseconds.abs().toDouble() / 1000;
      previousSample = sample;
      if (age > _horizonMilliseconds || delta > _assumePointerMoveStoppedMilliseconds) {
        break;
      }

      oldestSample = sample;
      final Offset position = sample.point;
      x.add(position.dx);
      y.add(position.dy);
      w.add(1.0);
      time.add(-age);
      index = (index == 0 ? _historySize : index) - 1;

      sampleCount += 1;
    } while (sampleCount < _historySize);

    if (sampleCount >= _minSampleSize) {
      // Marking as "late" ensures that yFit isn't evaluated unless it's needed.
      late final PolynomialFit? xFit = LeastSquaresSolver(time, x, w).solve(2);
      late final PolynomialFit? yFit = LeastSquaresSolver(time, y, w).solve(2);

      if (xFit != null && yFit != null) {
        return VelocityEstimate(
          // convert from pixels/ms to pixels/s
          pixelsPerSecond: Offset(xFit.coefficients[1] * 1000, yFit.coefficients[1] * 1000),
          confidence: xFit.confidence * yFit.confidence,
          duration: newestSample.time - oldestSample.time,
          offset: newestSample.point - oldestSample.point,
        );
      }
    }

    // We're unable to make a velocity estimate but we did have at least one
    // valid pointer position.
    return VelocityEstimate(
      pixelsPerSecond: Offset.zero,
      confidence: 1.0,
      duration: newestSample.time - oldestSample.time,
      offset: newestSample.point - oldestSample.point,
    );
  }

  /// Computes the velocity of the pointer at the time of the last
  /// provided data point.
  ///
  /// This can be expensive. Only call this when you need the velocity.
  ///
  /// Returns [Velocity.zero] if there is no data from which to compute an
  /// estimate or if the estimated velocity is zero.
  Velocity getVelocity() {
    final VelocityEstimate? estimate = getVelocityEstimate();
    if (estimate == null || estimate.pixelsPerSecond == Offset.zero) {
      return Velocity.zero;
    }
    return Velocity(pixelsPerSecond: estimate.pixelsPerSecond);
  }
}

/// A [VelocityTracker] subclass that provides a close approximation of iOS
/// scroll view's velocity estimation strategy.
///
/// The estimated velocity reported by this class is a close approximation of
/// the velocity an iOS scroll view would report with the same
/// [PointerMoveEvent]s, when the touch that initiates a fling is released.
///
/// This class differs from the [VelocityTracker] class in that it uses weighted
/// average of the latest few velocity samples of the tracked pointer, instead
/// of doing a linear regression on a relatively large amount of data points, to
/// estimate the velocity of the tracked pointer. Adding data points and
/// estimating the velocity are both cheap.
///
/// To obtain a velocity, call [getVelocity] or [getVelocityEstimate]. The
/// estimated velocity is typically used as the initial flinging velocity of a
/// `Scrollable`, when its drag gesture ends.
///
/// See also:
///
/// * [scrollViewWillEndDragging(_:withVelocity:targetContentOffset:)](https://developer.apple.com/documentation/uikit/uiscrollviewdelegate/1619385-scrollviewwillenddragging),
///   the iOS method that reports the fling velocity when the touch is released.
class IOSScrollViewFlingVelocityTracker extends VelocityTracker {
  /// Create a new IOSScrollViewFlingVelocityTracker.
  IOSScrollViewFlingVelocityTracker(super.kind) : super.withKind();

  /// The velocity estimation uses at most 4 `_PointAtTime` samples. The extra
  /// samples are there to make the `VelocityEstimate.offset` sufficiently large
  /// to be recognized as a fling. See
  /// `VerticalDragGestureRecognizer.isFlingGesture`.
  static const int _sampleSize = 20;

  final List<_PointAtTime?> _touchSamples = List<_PointAtTime?>.filled(_sampleSize, null);

  @override
  void addPosition(Duration time, Offset position) {
    _sinceLastSample.start();
    _sinceLastSample.reset();
    assert(() {
      final _PointAtTime? previousPoint = _touchSamples[_index];
      if (previousPoint == null || previousPoint.time <= time) {
        return true;
      }
      throw FlutterError(
        'The position being added ($position) has a smaller timestamp ($time) '
        'than its predecessor: $previousPoint.',
      );
    }());
    _index = (_index + 1) % _sampleSize;
    _touchSamples[_index] = _PointAtTime(position, time);
  }

  // Computes the velocity using 2 adjacent points in history. When index = 0,
  // it uses the latest point recorded and the point recorded immediately before
  // it. The smaller index is, the earlier in history the points used are.
  Offset _previousVelocityAt(int index) {
    final int endIndex = (_index + index) % _sampleSize;
    final int startIndex = (_index + index - 1) % _sampleSize;
    final _PointAtTime? end = _touchSamples[endIndex];
    final _PointAtTime? start = _touchSamples[startIndex];

    if (end == null || start == null) {
      return Offset.zero;
    }

    final int dt = (end.time - start.time).inMicroseconds;
    assert(dt >= 0);

    return dt > 0
        // Convert dt to milliseconds to preserve floating point precision.
        ? (end.point - start.point) * 1000 / (dt.toDouble() / 1000)
        : Offset.zero;
  }

  @override
  VelocityEstimate getVelocityEstimate() {
    // Has user recently moved since last sample?
    if (_sinceLastSample.elapsedMilliseconds >
        VelocityTracker._assumePointerMoveStoppedMilliseconds) {
      return const VelocityEstimate(
        pixelsPerSecond: Offset.zero,
        confidence: 1.0,
        duration: Duration.zero,
        offset: Offset.zero,
      );
    }

    // The velocity estimated using this expression is an approximation of the
    // scroll velocity of an iOS scroll view at the moment the user touch was
    // released, not the final velocity of the iOS pan gesture recognizer
    // installed on the scroll view would report. Typically in an iOS scroll
    // view the velocity values are different between the two, because the
    // scroll view usually slows down when the touch is released.
    final Offset estimatedVelocity =
        _previousVelocityAt(-2) * 0.6 +
        _previousVelocityAt(-1) * 0.35 +
        _previousVelocityAt(0) * 0.05;

    final _PointAtTime? newestSample = _touchSamples[_index];
    _PointAtTime? oldestNonNullSample;

    for (int i = 1; i <= _sampleSize; i += 1) {
      oldestNonNullSample = _touchSamples[(_index + i) % _sampleSize];
      if (oldestNonNullSample != null) {
        break;
      }
    }

    if (oldestNonNullSample == null || newestSample == null) {
      assert(false, 'There must be at least 1 point in _touchSamples: $_touchSamples');
      return const VelocityEstimate(
        pixelsPerSecond: Offset.zero,
        confidence: 0.0,
        duration: Duration.zero,
        offset: Offset.zero,
      );
    } else {
      return VelocityEstimate(
        pixelsPerSecond: estimatedVelocity,
        confidence: 1.0,
        duration: newestSample.time - oldestNonNullSample.time,
        offset: newestSample.point - oldestNonNullSample.point,
      );
    }
  }
}

/// A [VelocityTracker] subclass that provides a close approximation of macOS
/// scroll view's velocity estimation strategy.
///
/// The estimated velocity reported by this class is a close approximation of
/// the velocity a macOS scroll view would report with the same
/// [PointerMoveEvent]s, when the touch that initiates a fling is released.
///
/// This class differs from the [VelocityTracker] class in that it uses weighted
/// average of the latest few velocity samples of the tracked pointer, instead
/// of doing a linear regression on a relatively large amount of data points, to
/// estimate the velocity of the tracked pointer. Adding data points and
/// estimating the velocity are both cheap.
///
/// To obtain a velocity, call [getVelocity] or [getVelocityEstimate]. The
/// estimated velocity is typically used as the initial flinging velocity of a
/// `Scrollable`, when its drag gesture ends.
class MacOSScrollViewFlingVelocityTracker extends IOSScrollViewFlingVelocityTracker {
  /// Create a new MacOSScrollViewFlingVelocityTracker.
  MacOSScrollViewFlingVelocityTracker(super.kind);

  @override
  VelocityEstimate getVelocityEstimate() {
    // Has user recently moved since last sample?
    if (_sinceLastSample.elapsedMilliseconds >
        VelocityTracker._assumePointerMoveStoppedMilliseconds) {
      return const VelocityEstimate(
        pixelsPerSecond: Offset.zero,
        confidence: 1.0,
        duration: Duration.zero,
        offset: Offset.zero,
      );
    }

    // The velocity estimated using this expression is an approximation of the
    // scroll velocity of a macOS scroll view at the moment the user touch was
    // released.
    final Offset estimatedVelocity =
        _previousVelocityAt(-2) * 0.15 +
        _previousVelocityAt(-1) * 0.65 +
        _previousVelocityAt(0) * 0.2;

    final _PointAtTime? newestSample = _touchSamples[_index];
    _PointAtTime? oldestNonNullSample;

    for (int i = 1; i <= IOSScrollViewFlingVelocityTracker._sampleSize; i += 1) {
      oldestNonNullSample =
          _touchSamples[(_index + i) % IOSScrollViewFlingVelocityTracker._sampleSize];
      if (oldestNonNullSample != null) {
        break;
      }
    }

    if (oldestNonNullSample == null || newestSample == null) {
      assert(false, 'There must be at least 1 point in _touchSamples: $_touchSamples');
      return const VelocityEstimate(
        pixelsPerSecond: Offset.zero,
        confidence: 0.0,
        duration: Duration.zero,
        offset: Offset.zero,
      );
    } else {
      return VelocityEstimate(
        pixelsPerSecond: estimatedVelocity,
        confidence: 1.0,
        duration: newestSample.time - oldestNonNullSample.time,
        offset: newestSample.point - oldestNonNullSample.point,
      );
    }
  }
}
