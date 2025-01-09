// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

void main() {
  runApp(
    MaterialApp(
      title: 'Hardware Key Demo',
      home: Scaffold(
        appBar: AppBar(title: const Text('Hardware Key Demo')),
        body: const Center(child: HardwareKeyboardDemo()),
      ),
    ),
  );
}

class HardwareKeyboardDemo extends StatefulWidget {
  const HardwareKeyboardDemo({super.key});

  @override
  State<HardwareKeyboardDemo> createState() => _HardwareKeyboardDemoState();
}

class _HardwareKeyboardDemoState extends State<HardwareKeyboardDemo> {
  final FocusNode _focusNode = FocusNode();
  KeyEvent? _event;

  @override
  void dispose() {
    _focusNode.dispose();
    super.dispose();
  }

  KeyEventResult _handleKeyEvent(FocusNode node, KeyEvent event) {
    setState(() {
      _event = event;
    });
    return KeyEventResult.ignored;
  }

  @override
  Widget build(BuildContext context) {
    final TextTheme textTheme = Theme.of(context).textTheme;
    return Focus(
      focusNode: _focusNode,
      onKeyEvent: _handleKeyEvent,
      autofocus: true,
      child: AnimatedBuilder(
        animation: _focusNode,
        builder: (BuildContext context, Widget? child) {
          if (!_focusNode.hasFocus) {
            return GestureDetector(
              onTap: () {
                _focusNode.requestFocus();
              },
              child: Text('Tap to focus', style: textTheme.headlineMedium),
            );
          }

          if (_event == null) {
            return Text('Press a key', style: textTheme.headlineMedium);
          }

          final List<Widget> dataText = <Widget>[
            Text('${_event.runtimeType}'),
            if (_event?.character?.isNotEmpty ?? false)
              Text('character produced: "${_event?.character}"'),
          ];
          dataText.add(Text('logical: ${_event?.logicalKey}'));
          dataText.add(Text('physical: ${_event?.physicalKey}'));
          if (_event?.character != null) {
            dataText.add(Text('character: ${_event?.character}'));
          }
          final List<String> pressed = <String>['Pressed:'];
          for (final LogicalKeyboardKey key in HardwareKeyboard.instance.logicalKeysPressed) {
            pressed.add(key.debugName!);
          }
          dataText.add(Text(pressed.join(' ')));
          return DefaultTextStyle(
            style: textTheme.titleMedium!,
            child: Column(mainAxisAlignment: MainAxisAlignment.center, children: dataText),
          );
        },
      ),
    );
  }
}
