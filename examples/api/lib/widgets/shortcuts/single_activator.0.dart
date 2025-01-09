// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

/// Flutter code sample for [SingleActivator].

void main() => runApp(const SingleActivatorExampleApp());

class SingleActivatorExampleApp extends StatelessWidget {
  const SingleActivatorExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('SingleActivator Sample')),
        body: const Center(child: SingleActivatorExample()),
      ),
    );
  }
}

class IncrementIntent extends Intent {
  const IncrementIntent();
}

class SingleActivatorExample extends StatefulWidget {
  const SingleActivatorExample({super.key});

  @override
  State<SingleActivatorExample> createState() => _SingleActivatorExampleState();
}

class _SingleActivatorExampleState extends State<SingleActivatorExample> {
  int count = 0;

  @override
  Widget build(BuildContext context) {
    return Shortcuts(
      shortcuts: const <ShortcutActivator, Intent>{
        SingleActivator(LogicalKeyboardKey.keyC, control: true): IncrementIntent(),
      },
      child: Actions(
        actions: <Type, Action<Intent>>{
          IncrementIntent: CallbackAction<IncrementIntent>(
            onInvoke:
                (IncrementIntent intent) => setState(() {
                  count = count + 1;
                }),
          ),
        },
        child: Focus(
          autofocus: true,
          child: Column(
            children: <Widget>[
              const Text('Add to the counter by pressing Ctrl+C'),
              Text('count: $count'),
            ],
          ),
        ),
      ),
    );
  }
}
