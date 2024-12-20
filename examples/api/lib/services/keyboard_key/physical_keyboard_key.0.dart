// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

/// Flutter code sample for [PhysicalKeyboardKey].

void main() => runApp(const KeyExampleApp());

class KeyExampleApp extends StatelessWidget {
  const KeyExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('PhysicalKeyboardKey Example')),
        body: const MyPhysicalKeyExample(),
      ),
    );
  }
}

class MyPhysicalKeyExample extends StatefulWidget {
  const MyPhysicalKeyExample({super.key});

  @override
  State<MyPhysicalKeyExample> createState() => _MyPhysicalKeyExampleState();
}

class _MyPhysicalKeyExampleState extends State<MyPhysicalKeyExample> {
  // The node used to request the keyboard focus.
  final FocusNode _focusNode = FocusNode();
  // The message to display.
  String? _message;

  // Focus nodes need to be disposed.
  @override
  void dispose() {
    _focusNode.dispose();
    super.dispose();
  }

  // Handles the key events from the RawKeyboardListener and update the
  // _message.
  KeyEventResult _handleKeyEvent(FocusNode node, KeyEvent event) {
    setState(() {
      if (event.physicalKey == PhysicalKeyboardKey.keyA) {
        _message = 'Pressed the key next to CAPS LOCK!';
      } else {
        if (kReleaseMode) {
          _message =
              'Not the key next to CAPS LOCK: Pressed 0x${event.physicalKey.usbHidUsage.toRadixString(16)}';
        } else {
          // As the name implies, the debugName will only print useful
          // information in debug mode.
          _message = 'Not the key next to CAPS LOCK: Pressed ${event.physicalKey.debugName}';
        }
      }
    });
    return event.physicalKey == PhysicalKeyboardKey.keyA
        ? KeyEventResult.handled
        : KeyEventResult.ignored;
  }

  @override
  Widget build(BuildContext context) {
    final TextTheme textTheme = Theme.of(context).textTheme;
    return Container(
      color: Colors.white,
      alignment: Alignment.center,
      child: DefaultTextStyle(
        style: textTheme.headlineMedium!,
        child: Focus(
          focusNode: _focusNode,
          onKeyEvent: _handleKeyEvent,
          child: ListenableBuilder(
            listenable: _focusNode,
            builder: (BuildContext context, Widget? child) {
              if (!_focusNode.hasFocus) {
                return GestureDetector(
                  onTap: () {
                    FocusScope.of(context).requestFocus(_focusNode);
                  },
                  child: const Text('Click to focus'),
                );
              }
              return Text(_message ?? 'Press a key');
            },
          ),
        ),
      ),
    );
  }
}
