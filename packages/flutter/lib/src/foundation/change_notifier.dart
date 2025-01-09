// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/animation.dart';
/// @docImport 'package:flutter/widgets.dart';
library;

import 'dart:ui' show VoidCallback;

import 'package:meta/meta.dart';

import 'assertions.dart';
import 'diagnostics.dart';
import 'memory_allocations.dart';

export 'dart:ui' show VoidCallback;

/// An object that maintains a list of listeners.
///
/// The listeners are typically used to notify clients that the object has been
/// updated.
///
/// There are two variants of this interface:
///
///  * [ValueListenable], an interface that augments the [Listenable] interface
///    with the concept of a _current value_.
///
///  * [Animation], an interface that augments the [ValueListenable] interface
///    to add the concept of direction (forward or reverse).
///
/// Many classes in the Flutter API use or implement these interfaces. The
/// following subclasses are especially relevant:
///
///  * [ChangeNotifier], which can be subclassed or mixed in to create objects
///    that implement the [Listenable] interface.
///
///  * [ValueNotifier], which implements the [ValueListenable] interface with
///    a mutable value that triggers the notifications when modified.
///
/// The terms "notify clients", "send notifications", "trigger notifications",
/// and "fire notifications" are used interchangeably.
///
/// See also:
///
///  * [AnimatedBuilder], a widget that uses a builder callback to rebuild
///    whenever a given [Listenable] triggers its notifications. This widget is
///    commonly used with [Animation] subclasses, hence its name, but is by no
///    means limited to animations, as it can be used with any [Listenable]. It
///    is a subclass of [AnimatedWidget], which can be used to create widgets
///    that are driven from a [Listenable].
///  * [ValueListenableBuilder], a widget that uses a builder callback to
///    rebuild whenever a [ValueListenable] object triggers its notifications,
///    providing the builder with the value of the object.
///  * [InheritedNotifier], an abstract superclass for widgets that use a
///    [Listenable]'s notifications to trigger rebuilds in descendant widgets
///    that declare a dependency on them, using the [InheritedWidget] mechanism.
///  * [Listenable.merge], which creates a [Listenable] that triggers
///    notifications whenever any of a list of other [Listenable]s trigger their
///    notifications.
abstract class Listenable {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const Listenable();

  /// Return a [Listenable] that triggers when any of the given [Listenable]s
  /// themselves trigger.
  ///
  /// Once the factory is called, items must not be added or removed from the iterable.
  /// Doing so will lead to memory leaks or exceptions.
  ///
  /// The iterable may contain nulls; they are ignored.
  factory Listenable.merge(Iterable<Listenable?> listenables) = _MergingListenable;

  /// Register a closure to be called when the object notifies its listeners.
  void addListener(VoidCallback listener);

  /// Remove a previously registered closure from the list of closures that the
  /// object notifies.
  void removeListener(VoidCallback listener);
}

/// An interface for subclasses of [Listenable] that expose a [value].
///
/// This interface is implemented by [ValueNotifier<T>] and [Animation<T>], and
/// allows other APIs to accept either of those implementations interchangeably.
///
/// See also:
///
///  * [ValueListenableBuilder], a widget that uses a builder callback to
///    rebuild whenever a [ValueListenable] object triggers its notifications,
///    providing the builder with the value of the object.
abstract class ValueListenable<T> extends Listenable {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const ValueListenable();

  /// The current value of the object. When the value changes, the callbacks
  /// registered with [addListener] will be invoked.
  T get value;
}

const String _flutterFoundationLibrary = 'package:flutter/foundation.dart';

/// A class that can be extended or mixed in that provides a change notification
/// API using [VoidCallback] for notifications.
///
/// It is O(1) for adding listeners and O(N) for removing listeners and dispatching
/// notifications (where N is the number of listeners).
///
/// ## Using ChangeNotifier subclasses for data models
///
/// A data structure can extend or mix in [ChangeNotifier] to implement the
/// [Listenable] interface and thus become usable with widgets that listen for
/// changes to [Listenable]s, such as [ListenableBuilder].
///
/// {@tool dartpad}
/// The following example implements a simple counter that utilizes a
/// [ListenableBuilder] to limit rebuilds to only the [Text] widget containing
/// the count. The current count is stored in a [ChangeNotifier] subclass, which
/// rebuilds the [ListenableBuilder]'s contents when its value is changed.
///
/// ** See code in examples/api/lib/widgets/transitions/listenable_builder.2.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// In this case, the [ChangeNotifier] subclass encapsulates a list, and notifies
/// the clients any time an item is added to the list. This example only supports
/// adding items; as an exercise, consider adding buttons to remove items from
/// the list as well.
///
/// ** See code in examples/api/lib/widgets/transitions/listenable_builder.3.dart **
/// {@end-tool}
///
/// See also:
///
///  * [ValueNotifier], which is a [ChangeNotifier] that wraps a single value.
mixin class ChangeNotifier implements Listenable {
  int _count = 0;
  // The _listeners is intentionally set to a fixed-length _GrowableList instead
  // of const [].
  //
  // The const [] creates an instance of _ImmutableList which would be
  // different from fixed-length _GrowableList used elsewhere in this class.
  // keeping runtime type the same during the lifetime of this class lets the
  // compiler to infer concrete type for this property, and thus improves
  // performance.
  static final List<VoidCallback?> _emptyListeners = List<VoidCallback?>.filled(0, null);
  List<VoidCallback?> _listeners = _emptyListeners;
  int _notificationCallStackDepth = 0;
  int _reentrantlyRemovedListeners = 0;
  bool _debugDisposed = false;

  /// If true, the event [ObjectCreated] for this instance was dispatched to
  /// [FlutterMemoryAllocations].
  ///
  /// As [ChangeNotifier] is used as mixin, it does not have constructor,
  /// so we use [addListener] to dispatch the event.
  bool _creationDispatched = false;

  /// Used by subclasses to assert that the [ChangeNotifier] has not yet been
  /// disposed.
  ///
  /// {@tool snippet}
  /// The [debugAssertNotDisposed] function should only be called inside of an
  /// assert, as in this example.
  ///
  /// ```dart
  /// class MyNotifier with ChangeNotifier {
  ///   void doUpdate() {
  ///     assert(ChangeNotifier.debugAssertNotDisposed(this));
  ///     // ...
  ///   }
  /// }
  /// ```
  /// {@end-tool}
  // This is static and not an instance method because too many people try to
  // implement ChangeNotifier instead of extending it (and so it is too breaking
  // to add a method, especially for debug).
  static bool debugAssertNotDisposed(ChangeNotifier notifier) {
    assert(() {
      if (notifier._debugDisposed) {
        throw FlutterError(
          'A ${notifier.runtimeType} was used after being disposed.\n'
          'Once you have called dispose() on a ${notifier.runtimeType}, it '
          'can no longer be used.',
        );
      }
      return true;
    }());
    return true;
  }

  /// Whether any listeners are currently registered.
  ///
  /// Clients should not depend on this value for their behavior, because having
  /// one listener's logic change when another listener happens to start or stop
  /// listening will lead to extremely hard-to-track bugs. Subclasses might use
  /// this information to determine whether to do any work when there are no
  /// listeners, however; for example, resuming a [Stream] when a listener is
  /// added and pausing it when a listener is removed.
  ///
  /// Typically this is used by overriding [addListener], checking if
  /// [hasListeners] is false before calling `super.addListener()`, and if so,
  /// starting whatever work is needed to determine when to call
  /// [notifyListeners]; and similarly, by overriding [removeListener], checking
  /// if [hasListeners] is false after calling `super.removeListener()`, and if
  /// so, stopping that same work.
  ///
  /// This method returns false if [dispose] has been called.
  @protected
  bool get hasListeners => _count > 0;

  /// Dispatches event of the [object] creation to [FlutterMemoryAllocations.instance].
  ///
  /// If the event was already dispatched or [kFlutterMemoryAllocationsEnabled]
  /// is false, the method is noop.
  ///
  /// Tools like leak_tracker use the event of object creation to help
  /// developers identify the owner of the object, for troubleshooting purposes,
  /// by taking stack trace at the moment of the event.
  ///
  /// But, as [ChangeNotifier] is mixin, it does not have its own constructor. So, it
  /// communicates object creation in first `addListener`, that results
  /// in the stack trace pointing to `addListener`, not to constructor.
  ///
  /// To make debugging easier, invoke [ChangeNotifier.maybeDispatchObjectCreation]
  /// in constructor of the class. It will help
  /// to identify the owner.
  ///
  /// Make sure to invoke it with condition `if (kFlutterMemoryAllocationsEnabled) ...`
  /// so that the method is tree-shaken away when the flag is false.
  @protected
  static void maybeDispatchObjectCreation(ChangeNotifier object) {
    // Tree shaker does not include this method and the class MemoryAllocations
    // if kFlutterMemoryAllocationsEnabled is false.
    if (kFlutterMemoryAllocationsEnabled && !object._creationDispatched) {
      FlutterMemoryAllocations.instance.dispatchObjectCreated(
        library: _flutterFoundationLibrary,
        className: '$ChangeNotifier',
        object: object,
      );
      object._creationDispatched = true;
    }
  }

  /// Register a closure to be called when the object changes.
  ///
  /// If the given closure is already registered, an additional instance is
  /// added, and must be removed the same number of times it is added before it
  /// will stop being called.
  ///
  /// This method must not be called after [dispose] has been called.
  ///
  /// {@template flutter.foundation.ChangeNotifier.addListener}
  /// If a listener is added twice, and is removed once during an iteration
  /// (e.g. in response to a notification), it will still be called again. If,
  /// on the other hand, it is removed as many times as it was registered, then
  /// it will no longer be called. This odd behavior is the result of the
  /// [ChangeNotifier] not being able to determine which listener is being
  /// removed, since they are identical, therefore it will conservatively still
  /// call all the listeners when it knows that any are still registered.
  ///
  /// This surprising behavior can be unexpectedly observed when registering a
  /// listener on two separate objects which are both forwarding all
  /// registrations to a common upstream object.
  /// {@endtemplate}
  ///
  /// See also:
  ///
  ///  * [removeListener], which removes a previously registered closure from
  ///    the list of closures that are notified when the object changes.
  @override
  void addListener(VoidCallback listener) {
    assert(ChangeNotifier.debugAssertNotDisposed(this));

    if (kFlutterMemoryAllocationsEnabled) {
      maybeDispatchObjectCreation(this);
    }

    if (_count == _listeners.length) {
      if (_count == 0) {
        _listeners = List<VoidCallback?>.filled(1, null);
      } else {
        final List<VoidCallback?> newListeners = List<VoidCallback?>.filled(
          _listeners.length * 2,
          null,
        );
        for (int i = 0; i < _count; i++) {
          newListeners[i] = _listeners[i];
        }
        _listeners = newListeners;
      }
    }
    _listeners[_count++] = listener;
  }

  void _removeAt(int index) {
    // The list holding the listeners is not growable for performances reasons.
    // We still want to shrink this list if a lot of listeners have been added
    // and then removed outside a notifyListeners iteration.
    // We do this only when the real number of listeners is half the length
    // of our list.
    _count -= 1;
    if (_count * 2 <= _listeners.length) {
      final List<VoidCallback?> newListeners = List<VoidCallback?>.filled(_count, null);

      // Listeners before the index are at the same place.
      for (int i = 0; i < index; i++) {
        newListeners[i] = _listeners[i];
      }

      // Listeners after the index move towards the start of the list.
      for (int i = index; i < _count; i++) {
        newListeners[i] = _listeners[i + 1];
      }

      _listeners = newListeners;
    } else {
      // When there are more listeners than half the length of the list, we only
      // shift our listeners, so that we avoid to reallocate memory for the
      // whole list.
      for (int i = index; i < _count; i++) {
        _listeners[i] = _listeners[i + 1];
      }
      _listeners[_count] = null;
    }
  }

  /// Remove a previously registered closure from the list of closures that are
  /// notified when the object changes.
  ///
  /// If the given listener is not registered, the call is ignored.
  ///
  /// This method returns immediately if [dispose] has been called.
  ///
  /// {@macro flutter.foundation.ChangeNotifier.addListener}
  ///
  /// See also:
  ///
  ///  * [addListener], which registers a closure to be called when the object
  ///    changes.
  @override
  void removeListener(VoidCallback listener) {
    // This method is allowed to be called on disposed instances for usability
    // reasons. Due to how our frame scheduling logic between render objects and
    // overlays, it is common that the owner of this instance would be disposed a
    // frame earlier than the listeners. Allowing calls to this method after it
    // is disposed makes it easier for listeners to properly clean up.
    for (int i = 0; i < _count; i++) {
      final VoidCallback? listenerAtIndex = _listeners[i];
      if (listenerAtIndex == listener) {
        if (_notificationCallStackDepth > 0) {
          // We don't resize the list during notifyListeners iterations
          // but we set to null, the listeners we want to remove. We will
          // effectively resize the list at the end of all notifyListeners
          // iterations.
          _listeners[i] = null;
          _reentrantlyRemovedListeners++;
        } else {
          // When we are outside the notifyListeners iterations we can
          // effectively shrink the list.
          _removeAt(i);
        }
        break;
      }
    }
  }

  /// Discards any resources used by the object. After this is called, the
  /// object is not in a usable state and should be discarded (calls to
  /// [addListener] will throw after the object is disposed).
  ///
  /// This method should only be called by the object's owner.
  ///
  /// This method does not notify listeners, and clears the listener list once
  /// it is called. Consumers of this class must decide on whether to notify
  /// listeners or not immediately before disposal.
  @mustCallSuper
  void dispose() {
    assert(ChangeNotifier.debugAssertNotDisposed(this));
    assert(
      _notificationCallStackDepth == 0,
      'The "dispose()" method on $this was called during the call to '
      '"notifyListeners()". This is likely to cause errors since it modifies '
      'the list of listeners while the list is being used.',
    );
    assert(() {
      _debugDisposed = true;
      return true;
    }());
    if (kFlutterMemoryAllocationsEnabled && _creationDispatched) {
      FlutterMemoryAllocations.instance.dispatchObjectDisposed(object: this);
    }
    _listeners = _emptyListeners;
    _count = 0;
  }

  /// Call all the registered listeners.
  ///
  /// Call this method whenever the object changes, to notify any clients the
  /// object may have changed. Listeners that are added during this iteration
  /// will not be visited. Listeners that are removed during this iteration will
  /// not be visited after they are removed.
  ///
  /// Exceptions thrown by listeners will be caught and reported using
  /// [FlutterError.reportError].
  ///
  /// This method must not be called after [dispose] has been called.
  ///
  /// Surprising behavior can result when reentrantly removing a listener (e.g.
  /// in response to a notification) that has been registered multiple times.
  /// See the discussion at [removeListener].
  @protected
  @visibleForTesting
  @pragma('vm:notify-debugger-on-exception')
  void notifyListeners() {
    assert(ChangeNotifier.debugAssertNotDisposed(this));
    if (_count == 0) {
      return;
    }

    // To make sure that listeners removed during this iteration are not called,
    // we set them to null, but we don't shrink the list right away.
    // By doing this, we can continue to iterate on our list until it reaches
    // the last listener added before the call to this method.

    // To allow potential listeners to recursively call notifyListener, we track
    // the number of times this method is called in _notificationCallStackDepth.
    // Once every recursive iteration is finished (i.e. when _notificationCallStackDepth == 0),
    // we can safely shrink our list so that it will only contain not null
    // listeners.

    _notificationCallStackDepth++;

    final int end = _count;
    for (int i = 0; i < end; i++) {
      try {
        _listeners[i]?.call();
      } catch (exception, stack) {
        FlutterError.reportError(
          FlutterErrorDetails(
            exception: exception,
            stack: stack,
            library: 'foundation library',
            context: ErrorDescription('while dispatching notifications for $runtimeType'),
            informationCollector:
                () => <DiagnosticsNode>[
                  DiagnosticsProperty<ChangeNotifier>(
                    'The $runtimeType sending notification was',
                    this,
                    style: DiagnosticsTreeStyle.errorProperty,
                  ),
                ],
          ),
        );
      }
    }

    _notificationCallStackDepth--;

    if (_notificationCallStackDepth == 0 && _reentrantlyRemovedListeners > 0) {
      // We really remove the listeners when all notifications are done.
      final int newLength = _count - _reentrantlyRemovedListeners;
      if (newLength * 2 <= _listeners.length) {
        // As in _removeAt, we only shrink the list when the real number of
        // listeners is half the length of our list.
        final List<VoidCallback?> newListeners = List<VoidCallback?>.filled(newLength, null);

        int newIndex = 0;
        for (int i = 0; i < _count; i++) {
          final VoidCallback? listener = _listeners[i];
          if (listener != null) {
            newListeners[newIndex++] = listener;
          }
        }

        _listeners = newListeners;
      } else {
        // Otherwise we put all the null references at the end.
        for (int i = 0; i < newLength; i += 1) {
          if (_listeners[i] == null) {
            // We swap this item with the next not null item.
            int swapIndex = i + 1;
            while (_listeners[swapIndex] == null) {
              swapIndex += 1;
            }
            _listeners[i] = _listeners[swapIndex];
            _listeners[swapIndex] = null;
          }
        }
      }

      _reentrantlyRemovedListeners = 0;
      _count = newLength;
    }
  }
}

class _MergingListenable extends Listenable {
  _MergingListenable(this._children);

  final Iterable<Listenable?> _children;

  @override
  void addListener(VoidCallback listener) {
    for (final Listenable? child in _children) {
      child?.addListener(listener);
    }
  }

  @override
  void removeListener(VoidCallback listener) {
    for (final Listenable? child in _children) {
      child?.removeListener(listener);
    }
  }

  @override
  String toString() {
    return 'Listenable.merge([${_children.join(", ")}])';
  }
}

/// A [ChangeNotifier] that holds a single value.
///
/// When [value] is replaced with something that is not equal to the old
/// value as evaluated by the equality operator ==, this class notifies its
/// listeners.
///
/// ## Limitations
///
/// Because this class only notifies listeners when the [value]'s _identity_
/// changes, listeners will not be notified when mutable state within the
/// value itself changes.
///
/// For example, a `ValueNotifier<List<int>>` will not notify its listeners
/// when the _contents_ of the list are changed.
///
/// As a result, this class is best used with only immutable data types.
///
/// For mutable data types, consider extending [ChangeNotifier] directly.
class ValueNotifier<T> extends ChangeNotifier implements ValueListenable<T> {
  /// Creates a [ChangeNotifier] that wraps this value.
  ValueNotifier(this._value) {
    if (kFlutterMemoryAllocationsEnabled) {
      ChangeNotifier.maybeDispatchObjectCreation(this);
    }
  }

  /// The current value stored in this notifier.
  ///
  /// When the value is replaced with something that is not equal to the old
  /// value as evaluated by the equality operator ==, this class notifies its
  /// listeners.
  @override
  T get value => _value;
  T _value;
  set value(T newValue) {
    if (_value == newValue) {
      return;
    }
    _value = newValue;
    notifyListeners();
  }

  @override
  String toString() => '${describeIdentity(this)}($value)';
}
