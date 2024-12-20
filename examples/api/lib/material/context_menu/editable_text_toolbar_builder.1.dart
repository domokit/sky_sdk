// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This example demonstrates showing a custom context menu only when some
// narrowly defined text is selected.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

void main() => runApp(const EditableTextToolbarBuilderExampleApp());

const String emailAddress = 'me@example.com';
const String text = 'Select the email address and open the menu: $emailAddress';

class EditableTextToolbarBuilderExampleApp extends StatefulWidget {
  const EditableTextToolbarBuilderExampleApp({super.key});

  @override
  State<EditableTextToolbarBuilderExampleApp> createState() =>
      _EditableTextToolbarBuilderExampleAppState();
}

class _EditableTextToolbarBuilderExampleAppState
    extends State<EditableTextToolbarBuilderExampleApp> {
  final TextEditingController _controller = TextEditingController(text: text);

  void _showDialog(BuildContext context) {
    Navigator.of(context).push(
      DialogRoute<void>(
        context: context,
        builder:
            (BuildContext context) => const AlertDialog(title: Text('You clicked send email!')),
      ),
    );
  }

  @override
  void initState() {
    super.initState();
    // On web, disable the browser's context menu since this example uses a custom
    // Flutter-rendered context menu.
    if (kIsWeb) {
      BrowserContextMenu.disableContextMenu();
    }
  }

  @override
  void dispose() {
    if (kIsWeb) {
      BrowserContextMenu.enableContextMenu();
    }
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('Custom button for emails')),
        body: Center(
          child: Column(
            children: <Widget>[
              Container(height: 20.0),
              TextField(
                controller: _controller,
                contextMenuBuilder: (BuildContext context, EditableTextState editableTextState) {
                  final List<ContextMenuButtonItem> buttonItems =
                      editableTextState.contextMenuButtonItems;
                  // Here we add an "Email" button to the default TextField
                  // context menu for the current platform, but only if an email
                  // address is currently selected.
                  final TextEditingValue value = _controller.value;
                  if (_isValidEmail(value.selection.textInside(value.text))) {
                    buttonItems.insert(
                      0,
                      ContextMenuButtonItem(
                        label: 'Send email',
                        onPressed: () {
                          ContextMenuController.removeAny();
                          _showDialog(context);
                        },
                      ),
                    );
                  }
                  return AdaptiveTextSelectionToolbar.buttonItems(
                    anchors: editableTextState.contextMenuAnchors,
                    buttonItems: buttonItems,
                  );
                },
              ),
            ],
          ),
        ),
      ),
    );
  }
}

bool _isValidEmail(String text) {
  return RegExp(
    r'(?<name>[a-zA-Z0-9]+)'
    r'@'
    r'(?<domain>[a-zA-Z0-9]+)'
    r'\.'
    r'(?<topLevelDomain>[a-zA-Z0-9]+)',
  ).hasMatch(text);
}
