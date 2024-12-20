// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

/// Flutter code sample for [LogicalKeySet].

void main() => runApp(const LogicalKeySetExampleApp());

class LogicalKeySetExampleApp extends StatelessWidget {
  const LogicalKeySetExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('LogicalKeySet Sample')),
        body: const Center(child: LogicalKeySetExample()),
      ),
    );
  }
}

class IncrementIntent extends Intent {
  const IncrementIntent();
}

class LogicalKeySetExample extends StatefulWidget {
  const LogicalKeySetExample({super.key});

  @override
  State<LogicalKeySetExample> createState() => _LogicalKeySetExampleState();
}

class _LogicalKeySetExampleState extends State<LogicalKeySetExample> {
  int count = 0;

  @override
  Widget build(BuildContext context) {
    return Shortcuts(
      shortcuts: <ShortcutActivator, Intent>{
        LogicalKeySet(LogicalKeyboardKey.keyC, LogicalKeyboardKey.controlLeft):
            const IncrementIntent(),
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
