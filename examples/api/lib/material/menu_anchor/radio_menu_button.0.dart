// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

/// Flutter code sample for [RadioMenuButton].

void main() => runApp(const MenuApp());

class MyRadioMenu extends StatefulWidget {
  const MyRadioMenu({super.key});

  @override
  State<MyRadioMenu> createState() => _MyRadioMenuState();
}

class _MyRadioMenuState extends State<MyRadioMenu> {
  final FocusNode _buttonFocusNode = FocusNode(debugLabel: 'Menu Button');
  Color _backgroundColor = Colors.red;
  late ShortcutRegistryEntry _entry;

  static const SingleActivator _redShortcut = SingleActivator(
    LogicalKeyboardKey.keyR,
    control: true,
  );
  static const SingleActivator _greenShortcut = SingleActivator(
    LogicalKeyboardKey.keyG,
    control: true,
  );
  static const SingleActivator _blueShortcut = SingleActivator(
    LogicalKeyboardKey.keyB,
    control: true,
  );

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    _entry = ShortcutRegistry.of(context).addAll(<ShortcutActivator, VoidCallbackIntent>{
      _redShortcut: VoidCallbackIntent(() => _setBackgroundColor(Colors.red)),
      _greenShortcut: VoidCallbackIntent(() => _setBackgroundColor(Colors.green)),
      _blueShortcut: VoidCallbackIntent(() => _setBackgroundColor(Colors.blue)),
    });
  }

  @override
  void dispose() {
    _buttonFocusNode.dispose();
    _entry.dispose();
    super.dispose();
  }

  void _setBackgroundColor(Color? color) {
    setState(() {
      _backgroundColor = color!;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: <Widget>[
        MenuAnchor(
          childFocusNode: _buttonFocusNode,
          menuChildren: <Widget>[
            RadioMenuButton<Color>(
              value: Colors.red,
              shortcut: _redShortcut,
              groupValue: _backgroundColor,
              onChanged: _setBackgroundColor,
              child: const Text('Red Background'),
            ),
            RadioMenuButton<Color>(
              value: Colors.green,
              shortcut: _greenShortcut,
              groupValue: _backgroundColor,
              onChanged: _setBackgroundColor,
              child: const Text('Green Background'),
            ),
            RadioMenuButton<Color>(
              value: Colors.blue,
              shortcut: _blueShortcut,
              groupValue: _backgroundColor,
              onChanged: _setBackgroundColor,
              child: const Text('Blue Background'),
            ),
          ],
          builder: (BuildContext context, MenuController controller, Widget? child) {
            return TextButton(
              focusNode: _buttonFocusNode,
              onPressed: () {
                if (controller.isOpen) {
                  controller.close();
                } else {
                  controller.open();
                }
              },
              child: const Text('OPEN MENU'),
            );
          },
        ),
        Expanded(child: Container(color: _backgroundColor)),
      ],
    );
  }
}

class MenuApp extends StatelessWidget {
  const MenuApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData(useMaterial3: true),
      home: const Scaffold(body: SafeArea(child: MyRadioMenu())),
    );
  }
}
