// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';

import 'animation.dart';

export 'dart:ui' show VoidCallback;

export 'animation.dart' show AnimationStatus, AnimationStatusListener;

/// A mixin that helps listen to another object only when this object has registered listeners.
///
/// This mixin provides implementations of [didRegisterListener] and [didUnregisterListener],
/// and therefore can be used in conjunction with mixins that require these methods,
/// [AnimationLocalListenersMixin] and [AnimationLocalStatusListenersMixin].
mixin AnimationLazyListenerMixin {
  int _listenerCounter = 0;

  /// Calls [didStartListening] every time a registration of a listener causes
  /// an empty list of listeners to become non-empty.
  ///
  /// See also:
  ///
  ///  * [didUnregisterListener], which may cause the listener list to
  ///    become empty again, and in turn cause this method to call
  ///    [didStartListening] again.
  @protected
  void didRegisterListener() {
    assert(_listenerCounter >= 0);
    if (_listenerCounter == 0) {
      didStartListening();
    }
    _listenerCounter += 1;
  }

  /// Calls [didStopListening] when an only remaining listener is unregistered,
  /// thus making the list empty.
  ///
  /// See also:
  ///
  ///  * [didRegisterListener], which causes the listener list to become non-empty.
  @protected
  void didUnregisterListener() {
    assert(_listenerCounter >= 1);
    _listenerCounter -= 1;
    if (_listenerCounter == 0) {
      didStopListening();
    }
  }

  /// Called when the number of listeners changes from zero to one.
  @protected
  void didStartListening();

  /// Called when the number of listeners changes from one to zero.
  @protected
  void didStopListening();

  /// Whether there are any listeners.
  bool get isListening => _listenerCounter > 0;
}

/// A mixin that replaces the [didRegisterListener]/[didUnregisterListener] contract
/// with a dispose contract.
///
/// This mixin provides implementations of [didRegisterListener] and [didUnregisterListener],
/// and therefore can be used in conjunction with mixins that require these methods,
/// [AnimationLocalListenersMixin] and [AnimationLocalStatusListenersMixin].
mixin AnimationEagerListenerMixin {
  /// This implementation ignores listener registrations.
  @protected
  void didRegisterListener() {}

  /// This implementation ignores listener registrations.
  @protected
  void didUnregisterListener() {}

  /// Release the resources used by this object. The object is no longer usable
  /// after this method is called.
  @mustCallSuper
  void dispose() {}
}

/// A mixin that implements the [addListener]/[removeListener] protocol and notifies
/// all the registered listeners when [notifyListeners] is called.
///
/// This mixin requires that the mixing class provide methods [didRegisterListener]
/// and [didUnregisterListener]. Implementations of these methods can be obtained
/// by mixing in another mixin from this library, such as [AnimationLazyListenerMixin].
mixin AnimationLocalListenersMixin {
  final HashedObserverList<VoidCallback> _listeners = HashedObserverList<VoidCallback>();

  /// Called immediately before a listener is added via [addListener].
  ///
  /// At the time this method is called the registered listener is not yet
  /// notified by [notifyListeners].
  @protected
  void didRegisterListener();

  /// Called immediately after a listener is removed via [removeListener].
  ///
  /// At the time this method is called the removed listener is no longer
  /// notified by [notifyListeners].
  @protected
  void didUnregisterListener();

  /// Calls the listener every time the value of the animation changes.
  ///
  /// Listeners can be removed with [removeListener].
  void addListener(VoidCallback listener) {
    didRegisterListener();
    _listeners.add(listener);
  }

  /// Stop calling the listener every time the value of the animation changes.
  ///
  /// Listeners can be added with [addListener].
  void removeListener(VoidCallback listener) {
    final bool removed = _listeners.remove(listener);
    if (removed) {
      didUnregisterListener();
    }
  }

  /// Removes all listeners added with [addListener].
  ///
  /// This method is typically called from the `dispose` method of the class
  /// using this mixin if the class also uses the [AnimationEagerListenerMixin].
  ///
  /// Calling this method will not trigger [didUnregisterListener].
  @protected
  void clearListeners() {
    _listeners.clear();
  }

  /// Calls all the listeners.
  ///
  /// If listeners are added or removed during this function, the modifications
  /// will not change which listeners are called during this iteration.
  @protected
  @pragma('vm:notify-debugger-on-exception')
  void notifyListeners() {
    final List<VoidCallback> localListeners = _listeners.toList(growable: false);
    for (final VoidCallback listener in localListeners) {
      InformationCollector? collector;
      assert(() {
        collector =
            () => <DiagnosticsNode>[
              DiagnosticsProperty<AnimationLocalListenersMixin>(
                'The $runtimeType notifying listeners was',
                this,
                style: DiagnosticsTreeStyle.errorProperty,
              ),
            ];
        return true;
      }());
      try {
        if (_listeners.contains(listener)) {
          listener();
        }
      } catch (exception, stack) {
        FlutterError.reportError(
          FlutterErrorDetails(
            exception: exception,
            stack: stack,
            library: 'animation library',
            context: ErrorDescription('while notifying listeners for $runtimeType'),
            informationCollector: collector,
          ),
        );
      }
    }
  }
}

/// A mixin that implements the addStatusListener/removeStatusListener protocol
/// and notifies all the registered listeners when notifyStatusListeners is
/// called.
///
/// This mixin requires that the mixing class provide methods [didRegisterListener]
/// and [didUnregisterListener]. Implementations of these methods can be obtained
/// by mixing in another mixin from this library, such as [AnimationLazyListenerMixin].
mixin AnimationLocalStatusListenersMixin {
  final ObserverList<AnimationStatusListener> _statusListeners =
      ObserverList<AnimationStatusListener>();

  /// Called immediately before a status listener is added via [addStatusListener].
  ///
  /// At the time this method is called the registered listener is not yet
  /// notified by [notifyStatusListeners].
  @protected
  void didRegisterListener();

  /// Called immediately after a status listener is removed via [removeStatusListener].
  ///
  /// At the time this method is called the removed listener is no longer
  /// notified by [notifyStatusListeners].
  @protected
  void didUnregisterListener();

  /// Calls listener every time the status of the animation changes.
  ///
  /// Listeners can be removed with [removeStatusListener].
  void addStatusListener(AnimationStatusListener listener) {
    didRegisterListener();
    _statusListeners.add(listener);
  }

  /// Stops calling the listener every time the status of the animation changes.
  ///
  /// Listeners can be added with [addStatusListener].
  void removeStatusListener(AnimationStatusListener listener) {
    final bool removed = _statusListeners.remove(listener);
    if (removed) {
      didUnregisterListener();
    }
  }

  /// Removes all listeners added with [addStatusListener].
  ///
  /// This method is typically called from the `dispose` method of the class
  /// using this mixin if the class also uses the [AnimationEagerListenerMixin].
  ///
  /// Calling this method will not trigger [didUnregisterListener].
  @protected
  void clearStatusListeners() {
    _statusListeners.clear();
  }

  /// Calls all the status listeners.
  ///
  /// If listeners are added or removed during this function, the modifications
  /// will not change which listeners are called during this iteration.
  @protected
  @pragma('vm:notify-debugger-on-exception')
  void notifyStatusListeners(AnimationStatus status) {
    final List<AnimationStatusListener> localListeners = _statusListeners.toList(growable: false);
    for (final AnimationStatusListener listener in localListeners) {
      try {
        if (_statusListeners.contains(listener)) {
          listener(status);
        }
      } catch (exception, stack) {
        InformationCollector? collector;
        assert(() {
          collector =
              () => <DiagnosticsNode>[
                DiagnosticsProperty<AnimationLocalStatusListenersMixin>(
                  'The $runtimeType notifying status listeners was',
                  this,
                  style: DiagnosticsTreeStyle.errorProperty,
                ),
              ];
          return true;
        }());
        FlutterError.reportError(
          FlutterErrorDetails(
            exception: exception,
            stack: stack,
            library: 'animation library',
            context: ErrorDescription('while notifying status listeners for $runtimeType'),
            informationCollector: collector,
          ),
        );
      }
    }
  }
}
