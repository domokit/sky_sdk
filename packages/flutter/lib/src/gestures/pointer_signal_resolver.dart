// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
///
/// @docImport 'binding.dart';
library;

import 'package:flutter/foundation.dart';

import 'events.dart';

export 'events.dart' show PointerSignalEvent;

/// The callback to register with a [PointerSignalResolver] to express
/// interest in a pointer signal event.
typedef PointerSignalResolvedCallback = void Function(PointerSignalEvent event);

bool _isSameEvent(PointerSignalEvent event1, PointerSignalEvent event2) {
  return (event1.original ?? event1) == (event2.original ?? event2);
}

/// Mediates disputes over which listener should handle pointer signal events
/// when multiple listeners wish to handle those events.
///
/// Pointer signals (such as [PointerScrollEvent]) are immediate, so unlike
/// events that participate in the gesture arena, pointer signals always
/// resolve at the end of event dispatch. Yet if objects interested in handling
/// these signal events were to handle them directly, it would cause issues
/// such as multiple [Scrollable] widgets in the widget hierarchy responding
/// to the same mouse wheel event. Using this class, these events will only
/// be dispatched to the first registered handler, which will in turn
/// correspond to the widget that's deepest in the widget hierarchy.
///
/// To use this class, objects should register their event handler like so:
///
/// ```dart
/// void handleSignalEvent(PointerSignalEvent event) {
///   GestureBinding.instance.pointerSignalResolver.register(event, (PointerSignalEvent event) {
///     // handle the event...
///   });
/// }
/// ```
///
/// {@tool dartpad}
/// Here is an example that demonstrates the effect of not using the resolver
/// versus using it.
///
/// When this example is set to _not_ use the resolver, then triggering the
/// mouse wheel over the outer box will cause only the outer box to change
/// color, but triggering the mouse wheel over the inner box will cause _both_
/// the outer and the inner boxes to change color (because they're both
/// receiving the event).
///
/// When this example is set to _use_ the resolver, then only the box located
/// directly under the cursor will change color when the mouse wheel is
/// triggered.
///
/// ** See code in examples/api/lib/gestures/pointer_signal_resolver/pointer_signal_resolver.0.dart **
/// {@end-tool}
class PointerSignalResolver {
  PointerSignalResolvedCallback? _firstRegisteredCallback;

  PointerSignalEvent? _currentEvent;

  /// Registers interest in handling [event].
  ///
  /// This method may be called multiple times (typically from different parts
  /// of the widget hierarchy) for the same `event`, with different `callback`s,
  /// as the event is being dispatched across the tree. Once the dispatching is
  /// complete, the [GestureBinding] calls [resolve], and the first registered
  /// callback is called.
  ///
  /// The `callback` is invoked with one argument, the `event`.
  ///
  /// Once the [register] method has been called with a particular `event`, it
  /// must not be called for other `event`s until after [resolve] has been
  /// called. Only one event disambiguation can be in flight at a time. In
  /// normal use this is achieved by only registering callbacks for an event as
  /// it is actively being dispatched (for example, in
  /// [Listener.onPointerSignal]).
  ///
  /// See the documentation for the [PointerSignalResolver] class for an example
  /// of using this method.
  void register(PointerSignalEvent event, PointerSignalResolvedCallback callback) {
    assert(_currentEvent == null || _isSameEvent(_currentEvent!, event));
    if (_firstRegisteredCallback != null) {
      return;
    }
    _currentEvent = event;
    _firstRegisteredCallback = callback;
  }

  /// Resolves the event, calling the first registered callback if there was
  /// one.
  ///
  /// This is called by the [GestureBinding] after the framework has finished
  /// dispatching the pointer signal event.
  @pragma('vm:notify-debugger-on-exception')
  void resolve(PointerSignalEvent event) {
    if (_firstRegisteredCallback == null) {
      assert(_currentEvent == null);
      // Nothing in the framework/app wants to handle the `event`. Allow the
      // platform to trigger any default native actions.
      event.respond(allowPlatformDefault: true);
      return;
    }
    assert(_isSameEvent(_currentEvent!, event));
    try {
      _firstRegisteredCallback!(_currentEvent!);
    } catch (exception, stack) {
      InformationCollector? collector;
      assert(() {
        collector =
            () => <DiagnosticsNode>[
              DiagnosticsProperty<PointerSignalEvent>(
                'Event',
                event,
                style: DiagnosticsTreeStyle.errorProperty,
              ),
            ];
        return true;
      }());
      FlutterError.reportError(
        FlutterErrorDetails(
          exception: exception,
          stack: stack,
          library: 'gesture library',
          context: ErrorDescription('while resolving a PointerSignalEvent'),
          informationCollector: collector,
        ),
      );
    }
    _firstRegisteredCallback = null;
    _currentEvent = null;
  }
}
