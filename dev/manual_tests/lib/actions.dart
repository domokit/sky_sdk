// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:collection';
import 'dart:io';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

void main() {
  runApp(const MaterialApp(title: 'Actions Demo', home: FocusDemo()));
}

/// A class that can hold invocation information that an [UndoableAction] can
/// use to undo/redo itself.
///
/// Instances of this class are returned from [UndoableAction]s and placed on
/// the undo stack when they are invoked.
class Memento extends Object with Diagnosticable {
  const Memento({required this.name, required this.undo, required this.redo});

  /// Returns true if this Memento can be used to undo.
  ///
  /// Subclasses could override to provide their own conditions when a command is
  /// undoable.
  bool get canUndo => true;

  /// Returns true if this Memento can be used to redo.
  ///
  /// Subclasses could override to provide their own conditions when a command is
  /// redoable.
  bool get canRedo => true;

  final String name;
  final VoidCallback undo;
  final ValueGetter<Memento> redo;

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(StringProperty('name', name));
  }
}

/// Undoable Actions

/// An [ActionDispatcher] subclass that manages the invocation of undoable
/// actions.
class UndoableActionDispatcher extends ActionDispatcher implements Listenable {
  // A stack of actions that have been performed. The most recent action
  // performed is at the end of the list.
  final DoubleLinkedQueue<Memento> _completedActions = DoubleLinkedQueue<Memento>();
  // A stack of actions that can be redone. The most recent action performed is
  // at the end of the list.
  final List<Memento> _undoneActions = <Memento>[];

  /// The maximum number of undo levels allowed.
  ///
  /// If this value is set to a value smaller than the number of completed
  /// actions, then the stack of completed actions is truncated to only include
  /// the last [maxUndoLevels] actions.
  int get maxUndoLevels => 1000;

  final Set<VoidCallback> _listeners = <VoidCallback>{};

  @override
  void addListener(VoidCallback listener) {
    _listeners.add(listener);
  }

  @override
  void removeListener(VoidCallback listener) {
    _listeners.remove(listener);
  }

  /// Notifies listeners that the [ActionDispatcher] has changed state.
  ///
  /// May only be called by subclasses.
  @protected
  void notifyListeners() {
    for (final VoidCallback callback in _listeners) {
      callback();
    }
  }

  @override
  Object? invokeAction(Action<Intent> action, Intent intent, [BuildContext? context]) {
    final Object? result = super.invokeAction(action, intent, context);
    print('Invoking ${action is UndoableAction ? 'undoable ' : ''}$intent as $action: $this ');
    if (action is UndoableAction) {
      _completedActions.addLast(result! as Memento);
      _undoneActions.clear();
      _pruneActions();
      notifyListeners();
    }
    return result;
  }

  // Enforces undo level limit.
  void _pruneActions() {
    while (_completedActions.length > maxUndoLevels) {
      _completedActions.removeFirst();
    }
  }

  /// Returns true if there is an action on the stack that can be undone.
  bool get canUndo {
    if (_completedActions.isNotEmpty) {
      return _completedActions.first.canUndo;
    }
    return false;
  }

  /// Returns true if an action that has been undone can be re-invoked.
  bool get canRedo {
    if (_undoneActions.isNotEmpty) {
      return _undoneActions.first.canRedo;
    }
    return false;
  }

  /// Undoes the last action executed if possible.
  ///
  /// Returns true if the action was successfully undone.
  bool undo() {
    print('Undoing. $this');
    if (!canUndo) {
      return false;
    }
    final Memento memento = _completedActions.removeLast();
    memento.undo();
    _undoneActions.add(memento);
    notifyListeners();
    return true;
  }

  /// Re-invokes a previously undone action, if possible.
  ///
  /// Returns true if the action was successfully invoked.
  bool redo() {
    print('Redoing. $this');
    if (!canRedo) {
      return false;
    }
    final Memento memento = _undoneActions.removeLast();
    final Memento replacement = memento.redo();
    _completedActions.add(replacement);
    _pruneActions();
    notifyListeners();
    return true;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(IntProperty('undoable items', _completedActions.length));
    properties.add(IntProperty('redoable items', _undoneActions.length));
    properties.add(IterableProperty<Memento>('undo stack', _completedActions));
    properties.add(IterableProperty<Memento>('redo stack', _undoneActions));
  }
}

class UndoIntent extends Intent {
  const UndoIntent();
}

class UndoAction extends Action<UndoIntent> {
  @override
  bool isEnabled(UndoIntent intent) {
    final BuildContext? buildContext = primaryFocus?.context ?? FocusDemo.appKey.currentContext;
    if (buildContext == null) {
      return false;
    }
    final UndoableActionDispatcher manager = Actions.of(buildContext) as UndoableActionDispatcher;
    return manager.canUndo;
  }

  @override
  void invoke(UndoIntent intent) {
    final BuildContext? buildContext = primaryFocus?.context ?? FocusDemo.appKey.currentContext;
    if (buildContext == null) {
      return;
    }
    final UndoableActionDispatcher manager =
        Actions.of(primaryFocus?.context ?? FocusDemo.appKey.currentContext!)
            as UndoableActionDispatcher;
    manager.undo();
  }
}

class RedoIntent extends Intent {
  const RedoIntent();
}

class RedoAction extends Action<RedoIntent> {
  @override
  bool isEnabled(RedoIntent intent) {
    final BuildContext? buildContext = primaryFocus?.context ?? FocusDemo.appKey.currentContext;
    if (buildContext == null) {
      return false;
    }
    final UndoableActionDispatcher manager = Actions.of(buildContext) as UndoableActionDispatcher;
    return manager.canRedo;
  }

  @override
  RedoAction invoke(RedoIntent intent) {
    final BuildContext? buildContext = primaryFocus?.context ?? FocusDemo.appKey.currentContext;
    if (buildContext == null) {
      return this;
    }
    final UndoableActionDispatcher manager = Actions.of(buildContext) as UndoableActionDispatcher;
    manager.redo();
    return this;
  }
}

/// An action that can be undone.
abstract class UndoableAction<T extends Intent> extends Action<T> {}

class UndoableFocusActionBase<T extends Intent> extends UndoableAction<T> {
  @override
  @mustCallSuper
  Memento invoke(T intent) {
    final FocusNode? previousFocus = primaryFocus;
    return Memento(
      name: previousFocus!.debugLabel!,
      undo: () {
        previousFocus.requestFocus();
      },
      redo: () {
        return invoke(intent);
      },
    );
  }
}

class UndoableRequestFocusAction extends UndoableFocusActionBase<RequestFocusIntent> {
  @override
  Memento invoke(RequestFocusIntent intent) {
    final Memento memento = super.invoke(intent);
    intent.focusNode.requestFocus();
    return memento;
  }
}

/// Actions for manipulating focus.
class UndoableNextFocusAction extends UndoableFocusActionBase<NextFocusIntent> {
  @override
  Memento invoke(NextFocusIntent intent) {
    final Memento memento = super.invoke(intent);
    primaryFocus?.nextFocus();
    return memento;
  }
}

class UndoablePreviousFocusAction extends UndoableFocusActionBase<PreviousFocusIntent> {
  @override
  Memento invoke(PreviousFocusIntent intent) {
    final Memento memento = super.invoke(intent);
    primaryFocus?.previousFocus();
    return memento;
  }
}

class UndoableDirectionalFocusAction extends UndoableFocusActionBase<DirectionalFocusIntent> {
  @override
  Memento invoke(DirectionalFocusIntent intent) {
    final Memento memento = super.invoke(intent);
    primaryFocus?.focusInDirection(intent.direction);
    return memento;
  }
}

/// A button class that takes focus when clicked.
class DemoButton extends StatefulWidget {
  const DemoButton({super.key, required this.name});

  final String name;

  @override
  State<DemoButton> createState() => _DemoButtonState();
}

class _DemoButtonState extends State<DemoButton> {
  late final FocusNode _focusNode = FocusNode(debugLabel: widget.name);
  final GlobalKey _nameKey = GlobalKey();

  void _handleOnPressed() {
    print('Button ${widget.name} pressed.');
    setState(() {
      Actions.invoke(_nameKey.currentContext!, RequestFocusIntent(_focusNode));
    });
  }

  @override
  void dispose() {
    super.dispose();
    _focusNode.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return TextButton(
      focusNode: _focusNode,
      style: ButtonStyle(
        foregroundColor: const MaterialStatePropertyAll<Color>(Colors.black),
        overlayColor: MaterialStateProperty.resolveWith<Color>((Set<MaterialState> states) {
          if (states.contains(MaterialState.focused)) {
            return Colors.red;
          }
          if (states.contains(MaterialState.hovered)) {
            return Colors.blue;
          }
          return Colors.transparent;
        }),
      ),
      onPressed: () => _handleOnPressed(),
      child: Text(widget.name, key: _nameKey),
    );
  }
}

class FocusDemo extends StatefulWidget {
  const FocusDemo({super.key});

  static GlobalKey appKey = GlobalKey();

  @override
  State<FocusDemo> createState() => _FocusDemoState();
}

class _FocusDemoState extends State<FocusDemo> {
  final FocusNode outlineFocus = FocusNode(debugLabel: 'Demo Focus Node');
  late final UndoableActionDispatcher dispatcher = UndoableActionDispatcher();
  bool canUndo = false;
  bool canRedo = false;

  @override
  void initState() {
    super.initState();
    canUndo = dispatcher.canUndo;
    canRedo = dispatcher.canRedo;
    dispatcher.addListener(_handleUndoStateChange);
  }

  void _handleUndoStateChange() {
    if (dispatcher.canUndo != canUndo) {
      setState(() {
        canUndo = dispatcher.canUndo;
      });
    }
    if (dispatcher.canRedo != canRedo) {
      setState(() {
        canRedo = dispatcher.canRedo;
      });
    }
  }

  @override
  void dispose() {
    dispatcher.removeListener(_handleUndoStateChange);
    outlineFocus.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final TextTheme textTheme = Theme.of(context).textTheme;
    return Actions(
      dispatcher: dispatcher,
      actions: <Type, Action<Intent>>{
        RequestFocusIntent: UndoableRequestFocusAction(),
        NextFocusIntent: UndoableNextFocusAction(),
        PreviousFocusIntent: UndoablePreviousFocusAction(),
        DirectionalFocusIntent: UndoableDirectionalFocusAction(),
        UndoIntent: UndoAction(),
        RedoIntent: RedoAction(),
      },
      child: FocusTraversalGroup(
        policy: ReadingOrderTraversalPolicy(),
        child: Shortcuts(
          shortcuts: <ShortcutActivator, Intent>{
            SingleActivator(
                  LogicalKeyboardKey.keyZ,
                  meta: Platform.isMacOS,
                  control: !Platform.isMacOS,
                  shift: true,
                ):
                const RedoIntent(),
            SingleActivator(
                  LogicalKeyboardKey.keyZ,
                  meta: Platform.isMacOS,
                  control: !Platform.isMacOS,
                ):
                const UndoIntent(),
          },
          child: FocusScope(
            key: FocusDemo.appKey,
            debugLabel: 'Scope',
            autofocus: true,
            child: DefaultTextStyle(
              style: textTheme.headlineMedium!,
              child: Scaffold(
                appBar: AppBar(title: const Text('Actions Demo')),
                body: Center(
                  child: Builder(
                    builder: (BuildContext context) {
                      return Column(
                        mainAxisAlignment: MainAxisAlignment.center,
                        children: <Widget>[
                          const Row(
                            mainAxisAlignment: MainAxisAlignment.center,
                            children: <Widget>[
                              DemoButton(name: 'One'),
                              DemoButton(name: 'Two'),
                              DemoButton(name: 'Three'),
                            ],
                          ),
                          const Row(
                            mainAxisAlignment: MainAxisAlignment.center,
                            children: <Widget>[
                              DemoButton(name: 'Four'),
                              DemoButton(name: 'Five'),
                              DemoButton(name: 'Six'),
                            ],
                          ),
                          const Row(
                            mainAxisAlignment: MainAxisAlignment.center,
                            children: <Widget>[
                              DemoButton(name: 'Seven'),
                              DemoButton(name: 'Eight'),
                              DemoButton(name: 'Nine'),
                            ],
                          ),
                          Row(
                            mainAxisAlignment: MainAxisAlignment.center,
                            children: <Widget>[
                              Padding(
                                padding: const EdgeInsets.all(8.0),
                                child: ElevatedButton(
                                  onPressed:
                                      canUndo
                                          ? () {
                                            Actions.invoke(context, const UndoIntent());
                                          }
                                          : null,
                                  child: const Text('UNDO'),
                                ),
                              ),
                              Padding(
                                padding: const EdgeInsets.all(8.0),
                                child: ElevatedButton(
                                  onPressed:
                                      canRedo
                                          ? () {
                                            Actions.invoke(context, const RedoIntent());
                                          }
                                          : null,
                                  child: const Text('REDO'),
                                ),
                              ),
                            ],
                          ),
                        ],
                      );
                    },
                  ),
                ),
              ),
            ),
          ),
        ),
      ),
    );
  }
}
