// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

/// Flutter code sample for [TextFieldTapRegion].

void main() => runApp(const TapRegionApp());

class TapRegionApp extends StatelessWidget {
  const TapRegionApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('TextFieldTapRegion Example')),
        body: const TextFieldTapRegionExample(),
      ),
    );
  }
}

class TextFieldTapRegionExample extends StatefulWidget {
  const TextFieldTapRegionExample({super.key});

  @override
  State<TextFieldTapRegionExample> createState() => _TextFieldTapRegionExampleState();
}

class _TextFieldTapRegionExampleState extends State<TextFieldTapRegionExample> {
  int value = 0;

  @override
  Widget build(BuildContext context) {
    return ListView(
      children: <Widget>[
        Center(
          child: Padding(
            padding: const EdgeInsets.all(20.0),
            child: SizedBox(
              width: 150,
              height: 80,
              child: IntegerSpinnerField(
                value: value,
                autofocus: true,
                onChanged: (int newValue) {
                  if (value == newValue) {
                    // Avoid unnecessary redraws.
                    return;
                  }
                  setState(() {
                    // Update the value and redraw.
                    value = newValue;
                  });
                },
              ),
            ),
          ),
        ),
      ],
    );
  }
}

/// An integer example of the generic [SpinnerField] that validates input and
/// increments by a delta.
class IntegerSpinnerField extends StatelessWidget {
  const IntegerSpinnerField({
    super.key,
    required this.value,
    this.autofocus = false,
    this.delta = 1,
    this.onChanged,
  });

  final int value;
  final bool autofocus;
  final int delta;
  final ValueChanged<int>? onChanged;

  @override
  Widget build(BuildContext context) {
    return SpinnerField<int>(
      value: value,
      onChanged: onChanged,
      autofocus: autofocus,
      fromString: (String stringValue) => int.tryParse(stringValue) ?? value,
      increment: (int i) => i + delta,
      decrement: (int i) => i - delta,
      // Add a text formatter that only allows integer values and a leading
      // minus sign.
      inputFormatters: <TextInputFormatter>[
        TextInputFormatter.withFunction((TextEditingValue oldValue, TextEditingValue newValue) {
          String newString;
          if (newValue.text.startsWith('-')) {
            newString = '-${newValue.text.replaceAll(RegExp(r'\D'), '')}';
          } else {
            newString = newValue.text.replaceAll(RegExp(r'\D'), '');
          }
          return newValue.copyWith(
            text: newString,
            selection: newValue.selection.copyWith(
              baseOffset: newValue.selection.baseOffset.clamp(0, newString.length),
              extentOffset: newValue.selection.extentOffset.clamp(0, newString.length),
            ),
          );
        }),
      ],
    );
  }
}

/// A generic "spinner" field example which adds extra buttons next to a
/// [TextField] to increment and decrement the value.
///
/// This widget uses [TextFieldTapRegion] to indicate that tapping on the
/// spinner buttons should not cause the text field to lose focus.
class SpinnerField<T> extends StatefulWidget {
  SpinnerField({
    super.key,
    required this.value,
    required this.fromString,
    this.autofocus = false,
    String Function(T value)? asString,
    this.increment,
    this.decrement,
    this.onChanged,
    this.inputFormatters = const <TextInputFormatter>[],
  }) : asString = asString ?? ((T value) => value.toString());

  final T value;
  final T Function(T value)? increment;
  final T Function(T value)? decrement;
  final String Function(T value) asString;
  final T Function(String value) fromString;
  final ValueChanged<T>? onChanged;
  final List<TextInputFormatter> inputFormatters;
  final bool autofocus;

  @override
  State<SpinnerField<T>> createState() => _SpinnerFieldState<T>();
}

class _SpinnerFieldState<T> extends State<SpinnerField<T>> {
  TextEditingController controller = TextEditingController();

  @override
  void initState() {
    super.initState();
    _updateText(widget.asString(widget.value));
  }

  @override
  void dispose() {
    controller.dispose();
    super.dispose();
  }

  @override
  void didUpdateWidget(covariant SpinnerField<T> oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.asString != widget.asString || oldWidget.value != widget.value) {
      final String newText = widget.asString(widget.value);
      _updateText(newText);
    }
  }

  void _updateText(String text, {bool collapsed = true}) {
    if (text != controller.text) {
      controller.value = TextEditingValue(
        text: text,
        selection:
            collapsed
                ? TextSelection.collapsed(offset: text.length)
                : TextSelection(baseOffset: 0, extentOffset: text.length),
      );
    }
  }

  void _spin(T Function(T value)? spinFunction) {
    if (spinFunction == null) {
      return;
    }
    final T newValue = spinFunction(widget.value);
    widget.onChanged?.call(newValue);
    _updateText(widget.asString(newValue), collapsed: false);
  }

  void _increment() {
    _spin(widget.increment);
  }

  void _decrement() {
    _spin(widget.decrement);
  }

  @override
  Widget build(BuildContext context) {
    return CallbackShortcuts(
      bindings: <ShortcutActivator, VoidCallback>{
        const SingleActivator(LogicalKeyboardKey.arrowUp): _increment,
        const SingleActivator(LogicalKeyboardKey.arrowDown): _decrement,
      },
      child: Row(
        children: <Widget>[
          Expanded(
            child: TextField(
              autofocus: widget.autofocus,
              inputFormatters: widget.inputFormatters,
              decoration: const InputDecoration(border: OutlineInputBorder()),
              onChanged: (String value) => widget.onChanged?.call(widget.fromString(value)),
              controller: controller,
              textAlign: TextAlign.center,
            ),
          ),
          const SizedBox(width: 12),
          // Without this TextFieldTapRegion, tapping on the buttons below would
          // increment the value, but it would cause the text field to be
          // unfocused, since tapping outside of a text field should unfocus it
          // on non-mobile platforms.
          TextFieldTapRegion(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: <Widget>[
                Expanded(
                  child: OutlinedButton(onPressed: _increment, child: const Icon(Icons.add)),
                ),
                Expanded(
                  child: OutlinedButton(onPressed: _decrement, child: const Icon(Icons.remove)),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}
