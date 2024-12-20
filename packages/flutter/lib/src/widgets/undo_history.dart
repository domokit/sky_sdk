// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'editable_text.dart';
/// @docImport 'focus_scope.dart';
library;

import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart';

import 'actions.dart';
import 'focus_manager.dart';
import 'framework.dart';
import 'text_editing_intents.dart';

/// Provides undo/redo capabilities for a [ValueNotifier].
///
/// Listens to [value] and saves relevant values for undoing/redoing. The
/// cadence at which values are saved is a best approximation of the native
/// behaviors of a number of hardware keyboard on Flutter's desktop
/// platforms, as there are subtle differences between each of the platforms.
///
/// Listens to keyboard undo/redo shortcuts and calls [onTriggered] when a
/// shortcut is triggered that would affect the state of the [value].
///
/// The [child] must manage focus on the [focusNode]. For example, using a
/// [TextField] or [Focus] widget.
class UndoHistory<T> extends StatefulWidget {
  /// Creates an instance of [UndoHistory].
  const UndoHistory({
    super.key,
    this.shouldChangeUndoStack,
    required this.value,
    required this.onTriggered,
    required this.focusNode,
    this.undoStackModifier,
    this.controller,
    required this.child,
  });

  /// The value to track over time.
  final ValueNotifier<T> value;

  /// Called when checking whether a value change should be pushed onto
  /// the undo stack.
  final bool Function(T? oldValue, T newValue)? shouldChangeUndoStack;

  /// Called right before a new entry is pushed to the undo stack.
  ///
  /// The value returned from this method will be pushed to the stack instead
  /// of the original value.
  ///
  /// If null then the original value will always be pushed to the stack.
  final T Function(T value)? undoStackModifier;

  /// Called when an undo or redo causes a state change.
  ///
  /// If the state would still be the same before and after the undo/redo, this
  /// will not be called. For example, receiving a redo when there is nothing
  /// to redo will not call this method.
  ///
  /// Changes to the [value] while this method is running will not be recorded
  /// on the undo stack. For example, a [TextInputFormatter] may change the value
  /// from what was on the undo stack, but this new value will not be recorded,
  /// as that would wipe out the redo history.
  final void Function(T value) onTriggered;

  /// The [FocusNode] that will be used to listen for focus to set the initial
  /// undo state for the element.
  final FocusNode focusNode;

  /// {@template flutter.widgets.undoHistory.controller}
  /// Controls the undo state.
  ///
  /// If null, this widget will create its own [UndoHistoryController].
  /// {@endtemplate}
  final UndoHistoryController? controller;

  /// The child widget of [UndoHistory].
  final Widget child;

  @override
  State<UndoHistory<T>> createState() => UndoHistoryState<T>();
}

/// State for a [UndoHistory].
///
/// Provides [undo], [redo], [canUndo], and [canRedo] for programmatic access
/// to the undo state for custom undo and redo UI implementations.
@visibleForTesting
class UndoHistoryState<T> extends State<UndoHistory<T>> with UndoManagerClient {
  final _UndoStack<T> _stack = _UndoStack<T>();
  late final _Throttled<T> _throttledPush;
  Timer? _throttleTimer;
  bool _duringTrigger = false;

  // This duration was chosen as a best fit for the behavior of Mac, Linux,
  // and Windows undo/redo state save durations, but it is not perfect for any
  // of them.
  static const Duration _kThrottleDuration = Duration(milliseconds: 500);

  // Record the last value to prevent pushing multiple
  // of the same value in a row onto the undo stack. For example, _push gets
  // called both in initState and when the EditableText receives focus.
  T? _lastValue;

  UndoHistoryController? _controller;

  UndoHistoryController get _effectiveController =>
      widget.controller ?? (_controller ??= UndoHistoryController());

  @override
  void undo() {
    if (_stack.currentValue == null) {
      // Returns early if there is not a first value registered in the history.
      // This is important because, if an undo is received while the initial
      // value is being pushed (a.k.a when the field gets the focus but the
      // throttling delay is pending), the initial push should not be canceled.
      return;
    }
    if (_throttleTimer?.isActive ?? false) {
      _throttleTimer?.cancel(); // Cancel ongoing push, if any.
      _update(_stack.currentValue);
    } else {
      _update(_stack.undo());
    }
    _updateState();
  }

  @override
  void redo() {
    _update(_stack.redo());
    _updateState();
  }

  @override
  bool get canUndo => _stack.canUndo;

  @override
  bool get canRedo => _stack.canRedo;

  void _updateState() {
    _effectiveController.value = UndoHistoryValue(canUndo: canUndo, canRedo: canRedo);

    if (defaultTargetPlatform != TargetPlatform.iOS) {
      return;
    }

    if (UndoManager.client == this) {
      UndoManager.setUndoState(canUndo: canUndo, canRedo: canRedo);
    }
  }

  void _undoFromIntent(UndoTextIntent intent) {
    undo();
  }

  void _redoFromIntent(RedoTextIntent intent) {
    redo();
  }

  void _update(T? nextValue) {
    if (nextValue == null) {
      return;
    }
    if (nextValue == _lastValue) {
      return;
    }
    _lastValue = nextValue;
    _duringTrigger = true;
    try {
      widget.onTriggered(nextValue);
      assert(widget.value.value == nextValue);
    } finally {
      _duringTrigger = false;
    }
  }

  void _push() {
    if (widget.value.value == _lastValue) {
      return;
    }

    if (_duringTrigger) {
      return;
    }

    if (!(widget.shouldChangeUndoStack?.call(_lastValue, widget.value.value) ?? true)) {
      return;
    }

    final T nextValue = widget.undoStackModifier?.call(widget.value.value) ?? widget.value.value;
    if (nextValue == _lastValue) {
      return;
    }

    _lastValue = nextValue;

    _throttleTimer = _throttledPush(nextValue);
  }

  void _handleFocus() {
    if (!widget.focusNode.hasFocus) {
      if (UndoManager.client == this) {
        UndoManager.client = null;
      }

      return;
    }
    UndoManager.client = this;
    _updateState();
  }

  @override
  void handlePlatformUndo(UndoDirection direction) {
    switch (direction) {
      case UndoDirection.undo:
        undo();
      case UndoDirection.redo:
        redo();
    }
  }

  @protected
  @override
  void initState() {
    super.initState();
    _throttledPush = _throttle<T>(
      duration: _kThrottleDuration,
      function: (T currentValue) {
        _stack.push(currentValue);
        _updateState();
      },
    );
    _push();
    widget.value.addListener(_push);
    _handleFocus();
    widget.focusNode.addListener(_handleFocus);
    _effectiveController.onUndo.addListener(undo);
    _effectiveController.onRedo.addListener(redo);
  }

  @protected
  @override
  void didUpdateWidget(UndoHistory<T> oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.value != oldWidget.value) {
      _stack.clear();
      oldWidget.value.removeListener(_push);
      widget.value.addListener(_push);
    }
    if (widget.focusNode != oldWidget.focusNode) {
      oldWidget.focusNode.removeListener(_handleFocus);
      widget.focusNode.addListener(_handleFocus);
    }
    if (widget.controller != oldWidget.controller) {
      _effectiveController.onUndo.removeListener(undo);
      _effectiveController.onRedo.removeListener(redo);
      _controller?.dispose();
      _controller = null;
      _effectiveController.onUndo.addListener(undo);
      _effectiveController.onRedo.addListener(redo);
    }
  }

  @protected
  @override
  void dispose() {
    if (UndoManager.client == this) {
      UndoManager.client = null;
    }

    widget.value.removeListener(_push);
    widget.focusNode.removeListener(_handleFocus);
    _effectiveController.onUndo.removeListener(undo);
    _effectiveController.onRedo.removeListener(redo);
    _controller?.dispose();
    _throttleTimer?.cancel();
    super.dispose();
  }

  @protected
  @override
  Widget build(BuildContext context) {
    return Actions(
      actions: <Type, Action<Intent>>{
        UndoTextIntent: Action<UndoTextIntent>.overridable(
          context: context,
          defaultAction: CallbackAction<UndoTextIntent>(onInvoke: _undoFromIntent),
        ),
        RedoTextIntent: Action<RedoTextIntent>.overridable(
          context: context,
          defaultAction: CallbackAction<RedoTextIntent>(onInvoke: _redoFromIntent),
        ),
      },
      child: widget.child,
    );
  }
}

/// Represents whether the current undo stack can undo or redo.
@immutable
class UndoHistoryValue {
  /// Creates a value for whether the current undo stack can undo or redo.
  ///
  /// The [canUndo] and [canRedo] arguments must have a value, but default to
  /// false.
  const UndoHistoryValue({this.canUndo = false, this.canRedo = false});

  /// A value corresponding to an undo stack that can neither undo nor redo.
  static const UndoHistoryValue empty = UndoHistoryValue();

  /// Whether the current undo stack can perform an undo operation.
  final bool canUndo;

  /// Whether the current undo stack can perform a redo operation.
  final bool canRedo;

  @override
  String toString() =>
      '${objectRuntimeType(this, 'UndoHistoryValue')}(canUndo: $canUndo, canRedo: $canRedo)';

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    return other is UndoHistoryValue && other.canUndo == canUndo && other.canRedo == canRedo;
  }

  @override
  int get hashCode => Object.hash(canUndo.hashCode, canRedo.hashCode);
}

/// A controller for the undo history, for example for an editable text field.
///
/// Whenever a change happens to the underlying value that the [UndoHistory]
/// widget tracks, that widget updates the [value] and the controller notifies
/// it's listeners. Listeners can then read the canUndo and canRedo
/// properties of the value to discover whether [undo] or [redo] are possible.
///
/// The controller also has [undo] and [redo] methods to modify the undo
/// history.
///
/// {@tool dartpad}
/// This example creates a [TextField] with an [UndoHistoryController]
/// which provides undo and redo buttons.
///
/// ** See code in examples/api/lib/widgets/undo_history/undo_history_controller.0.dart **
/// {@end-tool}
///
/// See also:
///
/// * [EditableText], which uses the [UndoHistory] widget and allows
///   control of the underlying history using an [UndoHistoryController].
class UndoHistoryController extends ValueNotifier<UndoHistoryValue> {
  /// Creates a controller for an [UndoHistory] widget.
  UndoHistoryController({UndoHistoryValue? value}) : super(value ?? UndoHistoryValue.empty);

  /// Notifies listeners that [undo] has been called.
  final ChangeNotifier onUndo = ChangeNotifier();

  /// Notifies listeners that [redo] has been called.
  final ChangeNotifier onRedo = ChangeNotifier();

  /// Reverts the value on the stack to the previous value.
  void undo() {
    if (!value.canUndo) {
      return;
    }

    onUndo.notifyListeners();
  }

  /// Updates the value on the stack to the next value.
  void redo() {
    if (!value.canRedo) {
      return;
    }

    onRedo.notifyListeners();
  }

  @override
  void dispose() {
    onUndo.dispose();
    onRedo.dispose();
    super.dispose();
  }
}

/// A data structure representing a chronological list of states that can be
/// undone and redone.
class _UndoStack<T> {
  /// Creates an instance of [_UndoStack].
  _UndoStack();

  final List<T> _list = <T>[];

  // The index of the current value, or -1 if the list is empty.
  int _index = -1;

  /// Returns the current value of the stack.
  T? get currentValue => _list.isEmpty ? null : _list[_index];

  bool get canUndo => _list.isNotEmpty && _index > 0;

  bool get canRedo => _list.isNotEmpty && _index < _list.length - 1;

  /// Add a new state change to the stack.
  ///
  /// Pushing identical objects will not create multiple entries.
  void push(T value) {
    if (_list.isEmpty) {
      _index = 0;
      _list.add(value);
      return;
    }

    assert(_index < _list.length && _index >= 0);

    if (value == currentValue) {
      return;
    }

    // If anything has been undone in this stack, remove those irrelevant states
    // before adding the new one.
    if (_index != _list.length - 1) {
      _list.removeRange(_index + 1, _list.length);
    }
    _list.add(value);
    _index = _list.length - 1;
  }

  /// Returns the current value after an undo operation.
  ///
  /// An undo operation moves the current value to the previously pushed value,
  /// if any.
  ///
  /// Iff the stack is completely empty, then returns null.
  T? undo() {
    if (_list.isEmpty) {
      return null;
    }

    assert(_index < _list.length && _index >= 0);

    if (_index != 0) {
      _index = _index - 1;
    }

    return currentValue;
  }

  /// Returns the current value after a redo operation.
  ///
  /// A redo operation moves the current value to the value that was last
  /// undone, if any.
  ///
  /// Iff the stack is completely empty, then returns null.
  T? redo() {
    if (_list.isEmpty) {
      return null;
    }

    assert(_index < _list.length && _index >= 0);

    if (_index < _list.length - 1) {
      _index = _index + 1;
    }

    return currentValue;
  }

  /// Remove everything from the stack.
  void clear() {
    _list.clear();
    _index = -1;
  }

  @override
  String toString() {
    return '_UndoStack $_list';
  }
}

/// A function that can be throttled with the throttle function.
typedef _Throttleable<T> = void Function(T currentArg);

/// A function that has been throttled by [_throttle].
typedef _Throttled<T> = Timer Function(T currentArg);

/// Returns a _Throttled that will call through to the given function only a
/// maximum of once per duration.
///
/// Only works for functions that take exactly one argument and return void.
_Throttled<T> _throttle<T>({required Duration duration, required _Throttleable<T> function}) {
  Timer? timer;
  late T arg;

  return (T currentArg) {
    arg = currentArg;
    if (timer != null && timer!.isActive) {
      return timer!;
    }
    timer = Timer(duration, () {
      function(arg);
      timer = null;
    });
    return timer!;
  };
}
