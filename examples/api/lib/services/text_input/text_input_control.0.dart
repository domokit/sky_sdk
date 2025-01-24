// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

/// Flutter code sample for [TextInputControl].

void main() => runApp(const TextInputControlExampleApp());

class TextInputControlExampleApp extends StatelessWidget {
  const TextInputControlExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: TextInputControlExample());
  }
}

class TextInputControlExample extends StatefulWidget {
  const TextInputControlExample({super.key});

  @override
  MyStatefulWidgetState createState() => MyStatefulWidgetState();
}

class MyStatefulWidgetState extends State<TextInputControlExample> {
  final TextEditingController _controller = TextEditingController();
  final FocusNode _focusNode = FocusNode();

  @override
  void dispose() {
    super.dispose();
    _controller.dispose();
    _focusNode.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(
        child: TextField(
          autofocus: true,
          controller: _controller,
          focusNode: _focusNode,
          decoration: InputDecoration(
            suffix: IconButton(
              icon: const Icon(Icons.clear),
              tooltip: 'Clear and unfocus',
              onPressed: () {
                _controller.clear();
                _focusNode.unfocus();
              },
            ),
          ),
        ),
      ),
      bottomSheet: const MyVirtualKeyboard(),
    );
  }
}

class MyVirtualKeyboard extends StatefulWidget {
  const MyVirtualKeyboard({super.key});

  @override
  MyVirtualKeyboardState createState() => MyVirtualKeyboardState();
}

class MyVirtualKeyboardState extends State<MyVirtualKeyboard> {
  final MyTextInputControl _inputControl = MyTextInputControl();

  @override
  void initState() {
    super.initState();
    _inputControl.register();
  }

  @override
  void dispose() {
    super.dispose();
    _inputControl.unregister();
  }

  void _handleKeyPress(String key) {
    _inputControl.processUserInput(key);
  }

  @override
  Widget build(BuildContext context) {
    return ValueListenableBuilder<bool>(
      valueListenable: _inputControl.visible,
      builder: (_, bool visible, _) {
        return Visibility(
          visible: visible,
          child: FocusScope(
            canRequestFocus: false,
            child: TextFieldTapRegion(
              child: Row(
                mainAxisAlignment: MainAxisAlignment.center,
                children: <Widget>[
                  for (final String key in <String>['A', 'B', 'C'])
                    ElevatedButton(child: Text(key), onPressed: () => _handleKeyPress(key)),
                ],
              ),
            ),
          ),
        );
      },
    );
  }
}

class MyTextInputControl with TextInputControl {
  TextEditingValue _editingState = TextEditingValue.empty;
  final ValueNotifier<bool> _visible = ValueNotifier<bool>(false);

  /// The input control's visibility state for updating the visual presentation.
  ValueListenable<bool> get visible => _visible;

  /// Register the input control.
  void register() => TextInput.setInputControl(this);

  /// Restore the original platform input control.
  void unregister() => TextInput.restorePlatformInputControl();

  @override
  void show() => _visible.value = true;

  @override
  void hide() => _visible.value = false;

  @override
  void setEditingState(TextEditingValue value) => _editingState = value;

  /// Process user input.
  ///
  /// Updates the internal editing state by inserting the input text,
  /// and by replacing the current selection if any.
  void processUserInput(String input) {
    _editingState = _editingState.copyWith(
      text: _insertText(input),
      selection: _replaceSelection(input),
    );

    // Request the attached client to update accordingly.
    TextInput.updateEditingValue(_editingState);
  }

  String _insertText(String input) {
    final String text = _editingState.text;
    final TextSelection selection = _editingState.selection;
    return text.replaceRange(selection.start, selection.end, input);
  }

  TextSelection _replaceSelection(String input) {
    final TextSelection selection = _editingState.selection;
    return TextSelection.collapsed(offset: selection.start + input.length);
  }
}
