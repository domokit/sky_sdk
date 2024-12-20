// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
/// @docImport 'package:flutter_test/flutter_test.dart';
library;

import 'dart:async';

import 'package:flutter/foundation.dart';

import 'binding.dart';

export 'dart:ui' show VoidCallback;

export 'package:flutter/foundation.dart' show DiagnosticsNode;

/// Signature for the callback passed to the [Ticker] class's constructor.
///
/// The argument is the time elapsed from
/// the frame timestamp when the ticker was last started
/// to the current frame timestamp.
typedef TickerCallback = void Function(Duration elapsed);

/// An interface implemented by classes that can vend [Ticker] objects.
///
/// To obtain a [TickerProvider], consider mixing in either
/// [TickerProviderStateMixin] (which always works)
/// or [SingleTickerProviderStateMixin] (which is more efficient when it works)
/// to make a [State] subclass implement [TickerProvider].
/// That [State] can then be passed to lower-level widgets
/// or other related objects.
/// This ensures the resulting [Ticker]s will only tick when that [State]'s
/// subtree is enabled, as defined by [TickerMode].
///
/// In widget tests, the [WidgetTester] object is also a [TickerProvider].
///
/// Tickers can be used by any object that wants to be notified whenever a frame
/// triggers, but are most commonly used indirectly via an
/// [AnimationController]. [AnimationController]s need a [TickerProvider] to
/// obtain their [Ticker].
abstract class TickerProvider {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const TickerProvider();

  /// Creates a ticker with the given callback.
  ///
  /// The kind of ticker provided depends on the kind of ticker provider.
  @factory
  Ticker createTicker(TickerCallback onTick);
}

/// Calls its callback once per animation frame, when enabled.
///
/// To obtain a ticker, consider [TickerProvider].
///
/// When created, a ticker is initially disabled. Call [start] to
/// enable the ticker.
///
/// A [Ticker] can be silenced by setting [muted] to true. While silenced, time
/// still elapses, and [start] and [stop] can still be called, but no callbacks
/// are called.
///
/// By convention, the [start] and [stop] methods are used by the ticker's
/// consumer (for example, an [AnimationController]), and the [muted] property
/// is controlled by the [TickerProvider] that created the ticker (for example,
/// a [State] that uses [TickerProviderStateMixin] to silence the ticker when
/// the state's subtree is disabled as defined by [TickerMode]).
///
/// See also:
///
/// * [TickerProvider], for obtaining a ticker.
/// * [SchedulerBinding.scheduleFrameCallback], which drives tickers.
// TODO(jacobr): make Ticker use Diagnosticable to simplify reporting errors
// related to a ticker.
class Ticker {
  /// Creates a ticker that will call the provided callback once per frame while
  /// running.
  ///
  /// An optional label can be provided for debugging purposes. That label
  /// will appear in the [toString] output in debug builds.
  Ticker(this._onTick, {this.debugLabel}) {
    assert(() {
      _debugCreationStack = StackTrace.current;
      return true;
    }());
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectCreated(
        library: 'package:flutter/scheduler.dart',
        className: '$Ticker',
        object: this,
      );
    }
  }

  TickerFuture? _future;

  /// Whether this ticker has been silenced.
  ///
  /// While silenced, a ticker's clock can still run, but the callback will not
  /// be called.
  bool get muted => _muted;
  bool _muted = false;

  /// When set to true, silences the ticker, so that it is no longer ticking. If
  /// a tick is already scheduled, it will unschedule it. This will not
  /// unschedule the next frame, though.
  ///
  /// When set to false, unsilences the ticker, potentially scheduling a frame
  /// to handle the next tick.
  ///
  /// By convention, the [muted] property is controlled by the object that
  /// created the [Ticker] (typically a [TickerProvider]), not the object that
  /// listens to the ticker's ticks.
  set muted(bool value) {
    if (value == muted) {
      return;
    }
    _muted = value;
    if (value) {
      unscheduleTick();
    } else if (shouldScheduleTick) {
      scheduleTick();
    }
  }

  /// Whether this [Ticker] has scheduled a call to call its callback
  /// on the next frame.
  ///
  /// A ticker that is [muted] can be active (see [isActive]) yet not be
  /// ticking. In that case, the ticker will not call its callback, and
  /// [isTicking] will be false, but time will still be progressing.
  ///
  /// This will return false if the [SchedulerBinding.lifecycleState] is one
  /// that indicates the application is not currently visible (e.g. if the
  /// device's screen is turned off).
  bool get isTicking {
    if (_future == null) {
      return false;
    }
    if (muted) {
      return false;
    }
    if (SchedulerBinding.instance.framesEnabled) {
      return true;
    }
    if (SchedulerBinding.instance.schedulerPhase != SchedulerPhase.idle) {
      return true;
    } // for example, we might be in a warm-up frame or forced frame
    return false;
  }

  /// Whether time is elapsing for this [Ticker]. Becomes true when [start] is
  /// called and false when [stop] is called.
  ///
  /// A ticker can be active yet not be actually ticking (i.e. not be calling
  /// the callback). To determine if a ticker is actually ticking, use
  /// [isTicking].
  bool get isActive => _future != null;

  /// The frame timestamp when the ticker was last started,
  /// as reported by [SchedulerBinding.currentFrameTimestamp].
  Duration? _startTime;

  /// Starts the clock for this [Ticker]. If the ticker is not [muted], then this
  /// also starts calling the ticker's callback once per animation frame.
  ///
  /// The returned future resolves once the ticker [stop]s ticking. If the
  /// ticker is disposed, the future does not resolve. A derivative future is
  /// available from the returned [TickerFuture] object that resolves with an
  /// error in that case, via [TickerFuture.orCancel].
  ///
  /// Calling this sets [isActive] to true.
  ///
  /// This method cannot be called while the ticker is active. To restart the
  /// ticker, first [stop] it.
  ///
  /// By convention, this method is used by the object that receives the ticks
  /// (as opposed to the [TickerProvider] which created the ticker).
  TickerFuture start() {
    assert(() {
      if (isActive) {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary('A ticker was started twice.'),
          ErrorDescription(
            'A ticker that is already active cannot be started again without first stopping it.',
          ),
          describeForError('The affected ticker was'),
        ]);
      }
      return true;
    }());
    assert(_startTime == null);
    _future = TickerFuture._();
    if (shouldScheduleTick) {
      scheduleTick();
    }
    if (SchedulerBinding.instance.schedulerPhase.index > SchedulerPhase.idle.index &&
        SchedulerBinding.instance.schedulerPhase.index < SchedulerPhase.postFrameCallbacks.index) {
      _startTime = SchedulerBinding.instance.currentFrameTimeStamp;
    }
    return _future!;
  }

  /// Adds a debug representation of a [Ticker] optimized for including in error
  /// messages.
  DiagnosticsNode describeForError(String name) {
    // TODO(jacobr): make this more structured.
    return DiagnosticsProperty<Ticker>(name, this, description: toString(debugIncludeStack: true));
  }

  /// Stops calling this [Ticker]'s callback.
  ///
  /// If called with the `canceled` argument set to false (the default), causes
  /// the future returned by [start] to resolve. If called with the `canceled`
  /// argument set to true, the future does not resolve, and the future obtained
  /// from [TickerFuture.orCancel], if any, resolves with a [TickerCanceled]
  /// error.
  ///
  /// Calling this sets [isActive] to false.
  ///
  /// This method does nothing if called when the ticker is inactive.
  ///
  /// By convention, this method is used by the object that receives the ticks
  /// (as opposed to the [TickerProvider] which created the ticker).
  void stop({bool canceled = false}) {
    if (!isActive) {
      return;
    }

    // We take the _future into a local variable so that isTicking is false
    // when we actually complete the future (isTicking uses _future to
    // determine its state).
    final TickerFuture localFuture = _future!;
    _future = null;
    _startTime = null;
    assert(!isActive);

    unscheduleTick();
    if (canceled) {
      localFuture._cancel(this);
    } else {
      localFuture._complete();
    }
  }

  final TickerCallback _onTick;

  int? _animationId;

  /// Whether this [Ticker] has already scheduled a frame callback.
  @protected
  bool get scheduled => _animationId != null;

  /// Whether a tick should be scheduled.
  ///
  /// If this is true, then calling [scheduleTick] should succeed.
  ///
  /// Reasons why a tick should not be scheduled include:
  ///
  /// * A tick has already been scheduled for the coming frame.
  /// * The ticker is not active ([start] has not been called).
  /// * The ticker is not ticking, e.g. because it is [muted] (see [isTicking]).
  @protected
  bool get shouldScheduleTick => !muted && isActive && !scheduled;

  void _tick(Duration timeStamp) {
    assert(isTicking);
    assert(scheduled);
    _animationId = null;

    _startTime ??= timeStamp;
    _onTick(timeStamp - _startTime!);

    // The onTick callback may have scheduled another tick already, for
    // example by calling stop then start again.
    if (shouldScheduleTick) {
      scheduleTick(rescheduling: true);
    }
  }

  /// Schedules a tick for the next frame.
  ///
  /// This should only be called if [shouldScheduleTick] is true.
  @protected
  void scheduleTick({bool rescheduling = false}) {
    assert(!scheduled);
    assert(shouldScheduleTick);
    _animationId = SchedulerBinding.instance.scheduleFrameCallback(
      _tick,
      rescheduling: rescheduling,
    );
  }

  /// Cancels the frame callback that was requested by [scheduleTick], if any.
  ///
  /// Calling this method when no tick is [scheduled] is harmless.
  ///
  /// This method should not be called when [shouldScheduleTick] would return
  /// true if no tick was scheduled.
  @protected
  void unscheduleTick() {
    if (scheduled) {
      SchedulerBinding.instance.cancelFrameCallbackWithId(_animationId!);
      _animationId = null;
    }
    assert(!shouldScheduleTick);
  }

  /// Makes this [Ticker] take the state of another ticker, and disposes the
  /// other ticker.
  ///
  /// This is useful if an object with a [Ticker] is given a new
  /// [TickerProvider] but needs to maintain continuity. In particular, this
  /// maintains the identity of the [TickerFuture] returned by the [start]
  /// function of the original [Ticker] if the original ticker is active.
  ///
  /// This ticker must not be active when this method is called.
  void absorbTicker(Ticker originalTicker) {
    assert(!isActive);
    assert(_future == null);
    assert(_startTime == null);
    assert(_animationId == null);
    assert(
      (originalTicker._future == null) == (originalTicker._startTime == null),
      'Cannot absorb Ticker after it has been disposed.',
    );
    if (originalTicker._future != null) {
      _future = originalTicker._future;
      _startTime = originalTicker._startTime;
      if (shouldScheduleTick) {
        scheduleTick();
      }
      originalTicker._future =
          null; // so that it doesn't get disposed when we dispose of originalTicker
      originalTicker.unscheduleTick();
    }
    originalTicker.dispose();
  }

  /// Release the resources used by this object. The object is no longer usable
  /// after this method is called.
  ///
  /// It is legal to call this method while [isActive] is true, in which case:
  ///
  ///  * The frame callback that was requested by [scheduleTick], if any, is
  ///    canceled.
  ///  * The future that was returned by [start] does not resolve.
  ///  * The future obtained from [TickerFuture.orCancel], if any, resolves
  ///    with a [TickerCanceled] error.
  @mustCallSuper
  void dispose() {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectDisposed(object: this);
    }

    if (_future != null) {
      final TickerFuture localFuture = _future!;
      _future = null;
      assert(!isActive);
      unscheduleTick();
      localFuture._cancel(this);
    }
    assert(() {
      // We intentionally don't null out _startTime. This means that if start()
      // was ever called, the object is now in a bogus state. This weakly helps
      // catch cases of use-after-dispose.
      _startTime = Duration.zero;
      return true;
    }());
  }

  /// An optional label can be provided for debugging purposes.
  ///
  /// This label will appear in the [toString] output in debug builds.
  final String? debugLabel;
  late StackTrace _debugCreationStack;

  @override
  String toString({bool debugIncludeStack = false}) {
    final StringBuffer buffer = StringBuffer();
    buffer.write('${objectRuntimeType(this, 'Ticker')}(');
    assert(() {
      buffer.write(debugLabel ?? '');
      return true;
    }());
    buffer.write(')');
    assert(() {
      if (debugIncludeStack) {
        buffer.writeln();
        buffer.writeln('The stack trace when the $runtimeType was actually created was:');
        FlutterError.defaultStackFilter(
          _debugCreationStack.toString().trimRight().split('\n'),
        ).forEach(buffer.writeln);
      }
      return true;
    }());
    return buffer.toString();
  }
}

/// An object representing an ongoing [Ticker] sequence.
///
/// The [Ticker.start] method returns a [TickerFuture]. The [TickerFuture] will
/// complete successfully if the [Ticker] is stopped using [Ticker.stop] with
/// the `canceled` argument set to false (the default).
///
/// If the [Ticker] is disposed without being stopped, or if it is stopped with
/// `canceled` set to true, then this Future will never complete.
///
/// This class works like a normal [Future], but has an additional property,
/// [orCancel], which returns a derivative [Future] that completes with an error
/// if the [Ticker] that returned the [TickerFuture] was stopped with `canceled`
/// set to true, or if it was disposed without being stopped.
///
/// To run a callback when either this future resolves or when the ticker is
/// canceled, use [whenCompleteOrCancel].
class TickerFuture implements Future<void> {
  TickerFuture._();

  /// Creates a [TickerFuture] instance that represents an already-complete
  /// [Ticker] sequence.
  ///
  /// This is useful for implementing objects that normally defer to a [Ticker]
  /// but sometimes can skip the ticker because the animation is of zero
  /// duration, but which still need to represent the completed animation in the
  /// form of a [TickerFuture].
  TickerFuture.complete() {
    _complete();
  }

  final Completer<void> _primaryCompleter = Completer<void>();
  Completer<void>? _secondaryCompleter;
  bool? _completed; // null means unresolved, true means complete, false means canceled

  void _complete() {
    assert(_completed == null);
    _completed = true;
    _primaryCompleter.complete();
    _secondaryCompleter?.complete();
  }

  void _cancel(Ticker ticker) {
    assert(_completed == null);
    _completed = false;
    _secondaryCompleter?.completeError(TickerCanceled(ticker));
  }

  /// Calls `callback` either when this future resolves or when the ticker is
  /// canceled.
  ///
  /// Calling this method registers an exception handler for the [orCancel]
  /// future, so even if the [orCancel] property is accessed, canceling the
  /// ticker will not cause an uncaught exception in the current zone.
  void whenCompleteOrCancel(VoidCallback callback) {
    void thunk(dynamic value) {
      callback();
    }

    orCancel.then<void>(thunk, onError: thunk);
  }

  /// A future that resolves when this future resolves or throws when the ticker
  /// is canceled.
  ///
  /// If this property is never accessed, then canceling the ticker does not
  /// throw any exceptions. Once this property is accessed, though, if the
  /// corresponding ticker is canceled, then the [Future] returned by this
  /// getter will complete with an error, and if that error is not caught, there
  /// will be an uncaught exception in the current zone.
  Future<void> get orCancel {
    if (_secondaryCompleter == null) {
      _secondaryCompleter = Completer<void>();
      if (_completed != null) {
        if (_completed!) {
          _secondaryCompleter!.complete();
        } else {
          _secondaryCompleter!.completeError(const TickerCanceled());
        }
      }
    }
    return _secondaryCompleter!.future;
  }

  @override
  Stream<void> asStream() {
    return _primaryCompleter.future.asStream();
  }

  @override
  Future<void> catchError(Function onError, {bool Function(Object)? test}) {
    return _primaryCompleter.future.catchError(onError, test: test);
  }

  @override
  Future<R> then<R>(FutureOr<R> Function(void value) onValue, {Function? onError}) {
    return _primaryCompleter.future.then<R>(onValue, onError: onError);
  }

  @override
  Future<void> timeout(Duration timeLimit, {FutureOr<void> Function()? onTimeout}) {
    return _primaryCompleter.future.timeout(timeLimit, onTimeout: onTimeout);
  }

  @override
  Future<void> whenComplete(dynamic Function() action) {
    return _primaryCompleter.future.whenComplete(action);
  }

  @override
  String toString() =>
      '${describeIdentity(this)}(${_completed == null
          ? "active"
          : _completed!
          ? "complete"
          : "canceled"})';
}

/// Exception thrown by [Ticker] objects on the [TickerFuture.orCancel] future
/// when the ticker is canceled.
class TickerCanceled implements Exception {
  /// Creates a canceled-ticker exception.
  const TickerCanceled([this.ticker]);

  /// Reference to the [Ticker] object that was canceled.
  ///
  /// This may be null in the case that the [Future] created for
  /// [TickerFuture.orCancel] was created after the ticker was canceled.
  final Ticker? ticker;

  @override
  String toString() {
    if (ticker != null) {
      return 'This ticker was canceled: $ticker';
    }
    return 'The ticker was canceled before the "orCancel" property was first used.';
  }
}
