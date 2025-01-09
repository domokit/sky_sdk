// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

/// Flutter code sample for [Focus].

void main() => runApp(const FocusExampleApp());

class FocusExampleApp extends StatelessWidget {
  const FocusExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(appBar: AppBar(title: const Text('Focus Sample')), body: const FocusExample()),
    );
  }
}

class FocusExample extends StatefulWidget {
  const FocusExample({super.key});

  @override
  State<FocusExample> createState() => _FocusExampleState();
}

class _FocusExampleState extends State<FocusExample> {
  Color _color = Colors.white;

  KeyEventResult _handleKeyPress(FocusNode node, KeyEvent event) {
    if (event is KeyDownEvent) {
      debugPrint('Focus node ${node.debugLabel} got key event: ${event.logicalKey}');
      switch (event.logicalKey) {
        case LogicalKeyboardKey.keyR:
          debugPrint('Changing color to red.');
          setState(() {
            _color = Colors.red;
          });
          return KeyEventResult.handled;
        case LogicalKeyboardKey.keyG:
          debugPrint('Changing color to green.');
          setState(() {
            _color = Colors.green;
          });
          return KeyEventResult.handled;
        case LogicalKeyboardKey.keyB:
          debugPrint('Changing color to blue.');
          setState(() {
            _color = Colors.blue;
          });
          return KeyEventResult.handled;
      }
    }
    return KeyEventResult.ignored;
  }

  @override
  Widget build(BuildContext context) {
    final TextTheme textTheme = Theme.of(context).textTheme;
    return FocusScope(
      debugLabel: 'Scope',
      autofocus: true,
      child: DefaultTextStyle(
        style: textTheme.headlineMedium!,
        child: Focus(
          onKeyEvent: _handleKeyPress,
          debugLabel: 'Button',
          child: Builder(
            builder: (BuildContext context) {
              final FocusNode focusNode = Focus.of(context);
              final bool hasFocus = focusNode.hasFocus;
              return GestureDetector(
                onTap: () {
                  if (hasFocus) {
                    focusNode.unfocus();
                  } else {
                    focusNode.requestFocus();
                  }
                },
                child: Center(
                  child: Container(
                    width: 400,
                    height: 100,
                    alignment: Alignment.center,
                    color: hasFocus ? _color : Colors.white,
                    child: Text(hasFocus ? "I'm in color! Press R,G,B!" : 'Press to focus'),
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );
  }
}
