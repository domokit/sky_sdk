// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This example demonstrates how to create a custom toolbar that retains the
// look of the default buttons for the current platform.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

void main() => runApp(const EditableTextToolbarBuilderExampleApp());

class EditableTextToolbarBuilderExampleApp extends StatefulWidget {
  const EditableTextToolbarBuilderExampleApp({super.key});

  @override
  State<EditableTextToolbarBuilderExampleApp> createState() =>
      _EditableTextToolbarBuilderExampleAppState();
}

class _EditableTextToolbarBuilderExampleAppState
    extends State<EditableTextToolbarBuilderExampleApp> {
  final TextEditingController _controller = TextEditingController(
    text: 'Right click (desktop) or long press (mobile) to see the menu with a custom toolbar.',
  );

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
        appBar: AppBar(title: const Text('Custom toolbar, default-looking buttons')),
        body: Center(
          child: Column(
            children: <Widget>[
              const SizedBox(height: 20.0),
              TextField(
                controller: _controller,
                contextMenuBuilder: (BuildContext context, EditableTextState editableTextState) {
                  return _MyTextSelectionToolbar(
                    anchor: editableTextState.contextMenuAnchors.primaryAnchor,
                    // getAdaptiveButtons creates the default button widgets for
                    // the current platform.
                    children:
                        AdaptiveTextSelectionToolbar.getAdaptiveButtons(
                          context,
                          // These buttons just close the menu when clicked.
                          <ContextMenuButtonItem>[
                            ContextMenuButtonItem(
                              label: 'One',
                              onPressed: () => ContextMenuController.removeAny(),
                            ),
                            ContextMenuButtonItem(
                              label: 'Two',
                              onPressed: () => ContextMenuController.removeAny(),
                            ),
                            ContextMenuButtonItem(
                              label: 'Three',
                              onPressed: () => ContextMenuController.removeAny(),
                            ),
                            ContextMenuButtonItem(
                              label: 'Four',
                              onPressed: () => ContextMenuController.removeAny(),
                            ),
                            ContextMenuButtonItem(
                              label: 'Five',
                              onPressed: () => ContextMenuController.removeAny(),
                            ),
                          ],
                        ).toList(),
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

/// A simple, yet totally custom, text selection toolbar.
///
/// Displays its children in a scrollable grid.
class _MyTextSelectionToolbar extends StatelessWidget {
  const _MyTextSelectionToolbar({required this.anchor, required this.children});

  final Offset anchor;
  final List<Widget> children;

  @override
  Widget build(BuildContext context) {
    return Stack(
      children: <Widget>[
        Positioned(
          top: anchor.dy,
          left: anchor.dx,
          child: Container(
            width: 200.0,
            height: 200.0,
            color: Colors.cyanAccent.withOpacity(0.5),
            child: GridView.count(
              padding: const EdgeInsets.all(12.0),
              crossAxisCount: 2,
              children: children,
            ),
          ),
        ),
      ],
    );
  }
}
