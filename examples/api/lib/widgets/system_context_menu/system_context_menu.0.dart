// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [SystemContextMenu].

void main() => runApp(const SystemContextMenuExampleApp());

class SystemContextMenuExampleApp extends StatelessWidget {
  const SystemContextMenuExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('SystemContextMenu Basic Example')),
        body: Center(
          child: TextField(
            contextMenuBuilder: (BuildContext context, EditableTextState editableTextState) {
              // If supported, show the system context menu.
              if (SystemContextMenu.isSupported(context)) {
                return SystemContextMenu.editableText(editableTextState: editableTextState);
              }
              // Otherwise, show the flutter-rendered context menu for the current
              // platform.
              return AdaptiveTextSelectionToolbar.editableText(
                editableTextState: editableTextState,
              );
            },
          ),
        ),
      ),
    );
  }
}
