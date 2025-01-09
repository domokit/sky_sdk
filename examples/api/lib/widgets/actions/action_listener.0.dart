// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [ActionListener].

void main() => runApp(const ActionListenerExampleApp());

class ActionListenerExampleApp extends StatelessWidget {
  const ActionListenerExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('ActionListener Sample')),
        body: const Center(child: ActionListenerExample()),
      ),
    );
  }
}

class ActionListenerExample extends StatefulWidget {
  const ActionListenerExample({super.key});

  @override
  State<ActionListenerExample> createState() => _ActionListenerExampleState();
}

class _ActionListenerExampleState extends State<ActionListenerExample> {
  bool _on = false;
  late final MyAction _myAction;

  @override
  void initState() {
    super.initState();
    _myAction = MyAction();
  }

  void _toggleState() {
    setState(() {
      _on = !_on;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Row(
      mainAxisAlignment: MainAxisAlignment.center,
      children: <Widget>[
        Padding(
          padding: const EdgeInsets.all(8.0),
          child: OutlinedButton(onPressed: _toggleState, child: Text(_on ? 'Disable' : 'Enable')),
        ),
        if (_on)
          Padding(
            padding: const EdgeInsets.all(8.0),
            child: ActionListener(
              listener: (Action<Intent> action) {
                if (action.intentType == MyIntent) {
                  ScaffoldMessenger.of(
                    context,
                  ).showSnackBar(const SnackBar(content: Text('Action Listener Called')));
                }
              },
              action: _myAction,
              child: ElevatedButton(
                onPressed: () => const ActionDispatcher().invokeAction(_myAction, const MyIntent()),
                child: const Text('Call Action Listener'),
              ),
            ),
          ),
        if (!_on) Container(),
      ],
    );
  }
}

class MyAction extends Action<MyIntent> {
  @override
  void addActionListener(ActionListenerCallback listener) {
    super.addActionListener(listener);
    debugPrint('Action Listener was added');
  }

  @override
  void removeActionListener(ActionListenerCallback listener) {
    super.removeActionListener(listener);
    debugPrint('Action Listener was removed');
  }

  @override
  void invoke(covariant MyIntent intent) {
    notifyActionListeners();
  }
}

class MyIntent extends Intent {
  const MyIntent();
}
