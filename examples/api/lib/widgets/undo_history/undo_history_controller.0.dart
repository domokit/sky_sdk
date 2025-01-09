// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [UndoHistoryController].

void main() {
  runApp(const UndoHistoryControllerExampleApp());
}

class UndoHistoryControllerExampleApp extends StatelessWidget {
  const UndoHistoryControllerExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: MyHomePage());
  }
}

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key});

  @override
  State<MyHomePage> createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  final TextEditingController _controller = TextEditingController();
  final FocusNode _focusNode = FocusNode();
  final UndoHistoryController _undoController = UndoHistoryController();

  TextStyle? get enabledStyle => Theme.of(context).textTheme.bodyMedium;
  TextStyle? get disabledStyle =>
      Theme.of(context).textTheme.bodyMedium?.copyWith(color: Colors.grey);

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            TextField(
              maxLines: 4,
              controller: _controller,
              focusNode: _focusNode,
              undoController: _undoController,
            ),
            ValueListenableBuilder<UndoHistoryValue>(
              valueListenable: _undoController,
              builder: (BuildContext context, UndoHistoryValue value, Widget? child) {
                return Row(
                  children: <Widget>[
                    TextButton(
                      child: Text('Undo', style: value.canUndo ? enabledStyle : disabledStyle),
                      onPressed: () {
                        _undoController.undo();
                      },
                    ),
                    TextButton(
                      child: Text('Redo', style: value.canRedo ? enabledStyle : disabledStyle),
                      onPressed: () {
                        _undoController.redo();
                      },
                    ),
                  ],
                );
              },
            ),
          ],
        ),
      ),
    );
  }
}
