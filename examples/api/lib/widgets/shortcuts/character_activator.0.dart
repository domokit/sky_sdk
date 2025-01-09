// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [CharacterActivator].

void main() => runApp(const CharacterActivatorExampleApp());

class CharacterActivatorExampleApp extends StatelessWidget {
  const CharacterActivatorExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('CharacterActivator Sample')),
        body: const Center(child: CharacterActivatorExample()),
      ),
    );
  }
}

class HelpMenuIntent extends Intent {
  const HelpMenuIntent();
}

class CharacterActivatorExample extends StatefulWidget {
  const CharacterActivatorExample({super.key});

  @override
  State<CharacterActivatorExample> createState() => _CharacterActivatorExampleState();
}

class _CharacterActivatorExampleState extends State<CharacterActivatorExample> {
  @override
  Widget build(BuildContext context) {
    return Shortcuts(
      shortcuts: const <ShortcutActivator, Intent>{CharacterActivator('?'): HelpMenuIntent()},
      child: Actions(
        actions: <Type, Action<Intent>>{
          HelpMenuIntent: CallbackAction<HelpMenuIntent>(
            onInvoke: (HelpMenuIntent intent) {
              ScaffoldMessenger.of(
                context,
              ).showSnackBar(const SnackBar(content: Text('Keep calm and carry on!')));
              return null;
            },
          ),
        },
        child: const Focus(
          autofocus: true,
          child: Column(children: <Widget>[Text('Press question mark for help')]),
        ),
      ),
    );
  }
}
