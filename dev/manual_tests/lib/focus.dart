// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

void main() {
  runApp(const MaterialApp(title: 'Focus Demo', home: FocusDemo()));
}

class DemoButton extends StatefulWidget {
  const DemoButton({
    super.key,
    required this.name,
    this.canRequestFocus = true,
    this.autofocus = false,
  });

  final String name;
  final bool canRequestFocus;
  final bool autofocus;

  @override
  State<DemoButton> createState() => _DemoButtonState();
}

class _DemoButtonState extends State<DemoButton> {
  late final FocusNode focusNode = FocusNode(
    debugLabel: widget.name,
    canRequestFocus: widget.canRequestFocus,
  );

  @override
  void dispose() {
    focusNode.dispose();
    super.dispose();
  }

  @override
  void didUpdateWidget(DemoButton oldWidget) {
    super.didUpdateWidget(oldWidget);
    focusNode.canRequestFocus = widget.canRequestFocus;
  }

  void _handleOnPressed() {
    focusNode.requestFocus();
    print('Button ${widget.name} pressed.');
    debugDumpFocusTree();
  }

  @override
  Widget build(BuildContext context) {
    return TextButton(
      focusNode: focusNode,
      autofocus: widget.autofocus,
      style: ButtonStyle(
        overlayColor: MaterialStateProperty.resolveWith<Color>((Set<MaterialState> states) {
          if (states.contains(MaterialState.focused)) {
            return Colors.red.withOpacity(0.25);
          }
          if (states.contains(MaterialState.hovered)) {
            return Colors.blue.withOpacity(0.25);
          }
          return Colors.transparent;
        }),
      ),
      onPressed: () => _handleOnPressed(),
      child: Text(widget.name),
    );
  }
}

class FocusDemo extends StatefulWidget {
  const FocusDemo({super.key});

  @override
  State<FocusDemo> createState() => _FocusDemoState();
}

class _FocusDemoState extends State<FocusDemo> {
  FocusNode? outlineFocus;

  @override
  void initState() {
    super.initState();
    outlineFocus = FocusNode(debugLabel: 'Demo Focus Node');
  }

  @override
  void dispose() {
    outlineFocus?.dispose();
    super.dispose();
  }

  KeyEventResult _handleKeyPress(FocusNode node, KeyEvent event) {
    if (event is KeyDownEvent) {
      print('Scope got key event: ${event.logicalKey}, $node');
      print('Keys down: ${HardwareKeyboard.instance.logicalKeysPressed}');
      if (event.logicalKey == LogicalKeyboardKey.tab) {
        debugDumpFocusTree();
        if (HardwareKeyboard.instance.logicalKeysPressed.contains(LogicalKeyboardKey.shiftLeft) ||
            HardwareKeyboard.instance.logicalKeysPressed.contains(LogicalKeyboardKey.shiftRight)) {
          print('Moving to previous.');
          node.previousFocus();
          return KeyEventResult.handled;
        } else {
          print('Moving to next.');
          node.nextFocus();
          return KeyEventResult.handled;
        }
      }
      final TraversalDirection? direction = switch (event.logicalKey) {
        LogicalKeyboardKey.arrowLeft => TraversalDirection.left,
        LogicalKeyboardKey.arrowRight => TraversalDirection.right,
        LogicalKeyboardKey.arrowUp => TraversalDirection.up,
        LogicalKeyboardKey.arrowDown => TraversalDirection.down,
        _ => null,
      };
      if (direction != null) {
        node.focusInDirection(direction);
        return KeyEventResult.handled;
      }
    }
    return KeyEventResult.ignored;
  }

  @override
  Widget build(BuildContext context) {
    final TextTheme textTheme = Theme.of(context).textTheme;

    return FocusTraversalGroup(
      policy: ReadingOrderTraversalPolicy(),
      child: FocusScope(
        debugLabel: 'Scope',
        onKeyEvent: _handleKeyPress,
        autofocus: true,
        child: DefaultTextStyle(
          style: textTheme.headlineMedium!,
          child: Scaffold(
            appBar: AppBar(title: const Text('Focus Demo')),
            floatingActionButton: FloatingActionButton(child: const Text('+'), onPressed: () {}),
            body: Center(
              child: Builder(
                builder: (BuildContext context) {
                  return Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: <Widget>[
                      const Row(
                        mainAxisAlignment: MainAxisAlignment.center,
                        children: <Widget>[DemoButton(name: 'One', autofocus: true)],
                      ),
                      const Row(
                        mainAxisAlignment: MainAxisAlignment.center,
                        children: <Widget>[
                          DemoButton(name: 'Two'),
                          DemoButton(name: 'Three', canRequestFocus: false),
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
                      OutlinedButton(
                        onPressed: () => print('pressed'),
                        child: const Text('PRESS ME'),
                      ),
                      const Padding(
                        padding: EdgeInsets.all(8.0),
                        child: TextField(
                          decoration: InputDecoration(labelText: 'Enter Text', filled: true),
                        ),
                      ),
                      const Padding(
                        padding: EdgeInsets.all(8.0),
                        child: TextField(
                          decoration: InputDecoration(
                            border: OutlineInputBorder(),
                            labelText: 'Enter Text',
                            filled: false,
                          ),
                        ),
                      ),
                    ],
                  );
                },
              ),
            ),
          ),
        ),
      ),
    );
  }
}
