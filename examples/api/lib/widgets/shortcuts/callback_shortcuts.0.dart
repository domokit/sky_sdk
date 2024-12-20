// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

/// Flutter code sample for [CallbackShortcuts].

void main() => runApp(const CallbackShortcutsApp());

class CallbackShortcutsApp extends StatelessWidget {
  const CallbackShortcutsApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('CallbackShortcuts Sample')),
        body: const Center(child: CallbackShortcutsExample()),
      ),
    );
  }
}

class CallbackShortcutsExample extends StatefulWidget {
  const CallbackShortcutsExample({super.key});

  @override
  State<CallbackShortcutsExample> createState() => _CallbackShortcutsExampleState();
}

class _CallbackShortcutsExampleState extends State<CallbackShortcutsExample> {
  int count = 0;

  @override
  Widget build(BuildContext context) {
    return CallbackShortcuts(
      bindings: <ShortcutActivator, VoidCallback>{
        const SingleActivator(LogicalKeyboardKey.arrowUp): () {
          setState(() => count = count + 1);
        },
        const SingleActivator(LogicalKeyboardKey.arrowDown): () {
          setState(() => count = count - 1);
        },
      },
      child: Focus(
        autofocus: true,
        child: Column(
          children: <Widget>[
            const Text('Press the up arrow key to add to the counter'),
            const Text('Press the down arrow key to subtract from the counter'),
            Text('count: $count'),
          ],
        ),
      ),
    );
  }
}
