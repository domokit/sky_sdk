// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'scroll_view.dart';
library;

import 'dart:collection';

import 'package:flutter/foundation.dart';

import 'framework.dart';
import 'notification_listener.dart';
import 'scroll_notification.dart';
import 'scroll_position.dart';

// Examples can assume:
// void _listener(ScrollNotification notification) { }
// late BuildContext context;

/// A [ScrollNotification] listener for [ScrollNotificationObserver].
///
/// [ScrollNotificationObserver] is similar to
/// [NotificationListener]. It supports a listener list instead of
/// just a single listener and its listeners run unconditionally, they
/// do not require a gating boolean return value.
typedef ScrollNotificationCallback = void Function(ScrollNotification notification);

class _ScrollNotificationObserverScope extends InheritedWidget {
  const _ScrollNotificationObserverScope({
    required super.child,
    required ScrollNotificationObserverState scrollNotificationObserverState,
  }) : _scrollNotificationObserverState = scrollNotificationObserverState;

  final ScrollNotificationObserverState _scrollNotificationObserverState;

  @override
  bool updateShouldNotify(_ScrollNotificationObserverScope old) =>
      _scrollNotificationObserverState != old._scrollNotificationObserverState;
}

final class _ListenerEntry extends LinkedListEntry<_ListenerEntry> {
  _ListenerEntry(this.listener);
  final ScrollNotificationCallback listener;
}

/// Notifies its listeners when a descendant scrolls.
///
/// To add a listener to a [ScrollNotificationObserver] ancestor:
///
/// ```dart
/// ScrollNotificationObserver.of(context).addListener(_listener);
/// ```
///
/// To remove the listener from a [ScrollNotificationObserver] ancestor:
///
/// ```dart
/// ScrollNotificationObserver.of(context).removeListener(_listener);
/// ```
///
/// Stateful widgets that share an ancestor [ScrollNotificationObserver] typically
/// add a listener in [State.didChangeDependencies] (removing the old one
/// if necessary) and remove the listener in their [State.dispose] method.
///
/// Any function with the [ScrollNotificationCallback] signature can act as a
/// listener:
///
/// ```dart
/// // (e.g. in a stateful widget)
/// void _listener(ScrollNotification notification) {
///   // Do something, maybe setState()
/// }
/// ```
///
/// This widget is similar to [NotificationListener]. It supports a listener
/// list instead of just a single listener and its listeners run
/// unconditionally, they do not require a gating boolean return value.
///
/// {@tool dartpad}
/// This sample shows a "Scroll to top" button that uses [ScrollNotificationObserver]
/// to listen for scroll notifications from [ListView]. The button is only visible
/// when the user has scrolled down. When pressed, the button animates the scroll
/// position of the [ListView] back to the top.
///
/// ** See code in examples/api/lib/widgets/scroll_notification_observer/scroll_notification_observer.0.dart **
/// {@end-tool}
class ScrollNotificationObserver extends StatefulWidget {
  /// Create a [ScrollNotificationObserver].
  const ScrollNotificationObserver({super.key, required this.child});

  /// The subtree below this widget.
  final Widget child;

  /// The closest instance of this class that encloses the given context.
  ///
  /// If there is no enclosing [ScrollNotificationObserver] widget, then null is
  /// returned.
  ///
  /// Calling this method will create a dependency on the closest
  /// [ScrollNotificationObserver] in the [context], if there is one.
  ///
  /// See also:
  ///
  /// * [ScrollNotificationObserver.of], which is similar to this method, but
  ///   asserts if no [ScrollNotificationObserver] ancestor is found.
  static ScrollNotificationObserverState? maybeOf(BuildContext context) {
    return context
        .dependOnInheritedWidgetOfExactType<_ScrollNotificationObserverScope>()
        ?._scrollNotificationObserverState;
  }

  /// The closest instance of this class that encloses the given context.
  ///
  /// If no ancestor is found, this method will assert in debug mode, and throw
  /// an exception in release mode.
  ///
  /// Calling this method will create a dependency on the closest
  /// [ScrollNotificationObserver] in the [context].
  ///
  /// See also:
  ///
  /// * [ScrollNotificationObserver.maybeOf], which is similar to this method,
  ///   but returns null if no [ScrollNotificationObserver] ancestor is found.
  static ScrollNotificationObserverState of(BuildContext context) {
    final ScrollNotificationObserverState? observerState = maybeOf(context);
    assert(() {
      if (observerState == null) {
        throw FlutterError(
          'ScrollNotificationObserver.of() was called with a context that does not contain a '
          'ScrollNotificationObserver widget.\n'
          'No ScrollNotificationObserver widget ancestor could be found starting from the '
          'context that was passed to ScrollNotificationObserver.of(). This can happen '
          'because you are using a widget that looks for a ScrollNotificationObserver '
          'ancestor, but no such ancestor exists.\n'
          'The context used was:\n'
          '  $context',
        );
      }
      return true;
    }());
    return observerState!;
  }

  @override
  ScrollNotificationObserverState createState() => ScrollNotificationObserverState();
}

/// The listener list state for a [ScrollNotificationObserver] returned by
/// [ScrollNotificationObserver.of].
///
/// [ScrollNotificationObserver] is similar to
/// [NotificationListener]. It supports a listener list instead of
/// just a single listener and its listeners run unconditionally, they
/// do not require a gating boolean return value.
class ScrollNotificationObserverState extends State<ScrollNotificationObserver> {
  LinkedList<_ListenerEntry>? _listeners = LinkedList<_ListenerEntry>();

  bool _debugAssertNotDisposed() {
    assert(() {
      if (_listeners == null) {
        throw FlutterError(
          'A $runtimeType was used after being disposed.\n'
          'Once you have called dispose() on a $runtimeType, it can no longer be used.',
        );
      }
      return true;
    }());
    return true;
  }

  /// Add a [ScrollNotificationCallback] that will be called each time
  /// a descendant scrolls.
  void addListener(ScrollNotificationCallback listener) {
    assert(_debugAssertNotDisposed());
    _listeners!.add(_ListenerEntry(listener));
  }

  /// Remove the specified [ScrollNotificationCallback].
  void removeListener(ScrollNotificationCallback listener) {
    assert(_debugAssertNotDisposed());
    for (final _ListenerEntry entry in _listeners!) {
      if (entry.listener == listener) {
        entry.unlink();
        return;
      }
    }
  }

  void _notifyListeners(ScrollNotification notification) {
    assert(_debugAssertNotDisposed());
    if (_listeners!.isEmpty) {
      return;
    }

    final List<_ListenerEntry> localListeners = List<_ListenerEntry>.of(_listeners!);
    for (final _ListenerEntry entry in localListeners) {
      try {
        if (entry.list != null) {
          entry.listener(notification);
        }
      } catch (exception, stack) {
        FlutterError.reportError(
          FlutterErrorDetails(
            exception: exception,
            stack: stack,
            library: 'widget library',
            context: ErrorDescription('while dispatching notifications for $runtimeType'),
            informationCollector:
                () => <DiagnosticsNode>[
                  DiagnosticsProperty<ScrollNotificationObserverState>(
                    'The $runtimeType sending notification was',
                    this,
                    style: DiagnosticsTreeStyle.errorProperty,
                  ),
                ],
          ),
        );
      }
    }
  }

  @protected
  @override
  Widget build(BuildContext context) {
    return NotificationListener<ScrollMetricsNotification>(
      onNotification: (ScrollMetricsNotification notification) {
        // A ScrollMetricsNotification allows listeners to be notified for an
        // initial state, as well as if the content dimensions change without
        // scrolling.
        _notifyListeners(notification.asScrollUpdate());
        return false;
      },
      child: NotificationListener<ScrollNotification>(
        onNotification: (ScrollNotification notification) {
          _notifyListeners(notification);
          return false;
        },
        child: _ScrollNotificationObserverScope(
          scrollNotificationObserverState: this,
          child: widget.child,
        ),
      ),
    );
  }

  @protected
  @override
  void dispose() {
    assert(_debugAssertNotDisposed());
    _listeners = null;
    super.dispose();
  }
}
