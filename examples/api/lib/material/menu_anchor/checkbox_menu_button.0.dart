// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

/// Flutter code sample for [CheckboxMenuButton].

void main() => runApp(const MenuApp());

class MyCheckboxMenu extends StatefulWidget {
  const MyCheckboxMenu({super.key, required this.message});

  final String message;

  @override
  State<MyCheckboxMenu> createState() => _MyCheckboxMenuState();
}

class _MyCheckboxMenuState extends State<MyCheckboxMenu> {
  final FocusNode _buttonFocusNode = FocusNode(debugLabel: 'Menu Button');
  static const SingleActivator _showShortcut = SingleActivator(
    LogicalKeyboardKey.keyS,
    control: true,
  );
  bool _showingMessage = false;

  @override
  void dispose() {
    _buttonFocusNode.dispose();
    super.dispose();
  }

  void _setMessageVisibility(bool visible) {
    setState(() {
      _showingMessage = visible;
    });
  }

  @override
  Widget build(BuildContext context) {
    return CallbackShortcuts(
      bindings: <ShortcutActivator, VoidCallback>{
        _showShortcut: () {
          _setMessageVisibility(!_showingMessage);
        },
      },
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: <Widget>[
          MenuAnchor(
            childFocusNode: _buttonFocusNode,
            menuChildren: <Widget>[
              CheckboxMenuButton(
                value: _showingMessage,
                onChanged: (bool? value) {
                  _setMessageVisibility(value!);
                },
                child: const Text('Show Message'),
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
          Expanded(
            child: Container(
              alignment: Alignment.center,
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: <Widget>[
                  Padding(
                    padding: const EdgeInsets.all(12.0),
                    child: Text(
                      _showingMessage ? widget.message : '',
                      style: Theme.of(context).textTheme.headlineSmall,
                    ),
                  ),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class MenuApp extends StatelessWidget {
  const MenuApp({super.key});

  static const String kMessage = '"Talk less. Smile more." - A. Burr';

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(
      home: Scaffold(body: SafeArea(child: MyCheckboxMenu(message: kMessage))),
    );
  }
}
