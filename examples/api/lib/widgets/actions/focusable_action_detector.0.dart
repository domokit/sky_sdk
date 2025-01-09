// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

/// Flutter code sample for [FocusableActionDetector].

void main() => runApp(const FocusableActionDetectorExampleApp());

class FocusableActionDetectorExampleApp extends StatelessWidget {
  const FocusableActionDetectorExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: FocusableActionDetectorExample());
  }
}

class FadButton extends StatefulWidget {
  const FadButton({super.key, required this.onPressed, required this.child});

  final VoidCallback onPressed;
  final Widget child;

  @override
  State<FadButton> createState() => _FadButtonState();
}

class _FadButtonState extends State<FadButton> {
  bool _focused = false;
  bool _hovering = false;
  bool _on = false;
  late final Map<Type, Action<Intent>> _actionMap;
  final Map<ShortcutActivator, Intent> _shortcutMap = const <ShortcutActivator, Intent>{
    SingleActivator(LogicalKeyboardKey.keyX): ActivateIntent(),
  };

  @override
  void initState() {
    super.initState();
    _actionMap = <Type, Action<Intent>>{
      ActivateIntent: CallbackAction<Intent>(onInvoke: (Intent intent) => _toggleState()),
    };
  }

  Color get color {
    Color baseColor = Colors.lightBlue;
    if (_focused) {
      baseColor = Color.alphaBlend(Colors.black.withOpacity(0.25), baseColor);
    }
    if (_hovering) {
      baseColor = Color.alphaBlend(Colors.black.withOpacity(0.1), baseColor);
    }
    return baseColor;
  }

  void _toggleState() {
    setState(() {
      _on = !_on;
    });
  }

  void _handleFocusHighlight(bool value) {
    setState(() {
      _focused = value;
    });
  }

  void _handleHoveHighlight(bool value) {
    setState(() {
      _hovering = value;
    });
  }

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTap: _toggleState,
      child: FocusableActionDetector(
        actions: _actionMap,
        shortcuts: _shortcutMap,
        onShowFocusHighlight: _handleFocusHighlight,
        onShowHoverHighlight: _handleHoveHighlight,
        child: Row(
          children: <Widget>[
            Container(padding: const EdgeInsets.all(10.0), color: color, child: widget.child),
            Container(
              width: 30,
              height: 30,
              margin: const EdgeInsets.all(10.0),
              color: _on ? Colors.red : Colors.transparent,
            ),
          ],
        ),
      ),
    );
  }
}

class FocusableActionDetectorExample extends StatefulWidget {
  const FocusableActionDetectorExample({super.key});

  @override
  State<FocusableActionDetectorExample> createState() => _FocusableActionDetectorExampleState();
}

class _FocusableActionDetectorExampleState extends State<FocusableActionDetectorExample> {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('FocusableActionDetector Example')),
      body: Center(
        child: Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            Padding(
              padding: const EdgeInsets.all(8.0),
              child: TextButton(onPressed: () {}, child: const Text('Press Me')),
            ),
            Padding(
              padding: const EdgeInsets.all(8.0),
              child: FadButton(onPressed: () {}, child: const Text('And Me')),
            ),
          ],
        ),
      ),
    );
  }
}
