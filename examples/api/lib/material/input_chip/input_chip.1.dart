// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/material.dart';

const List<String> _pizzaToppings = <String>[
  'Olives',
  'Tomato',
  'Cheese',
  'Pepperoni',
  'Bacon',
  'Onion',
  'Jalapeno',
  'Mushrooms',
  'Pineapple',
];

void main() => runApp(const EditableChipFieldApp());

class EditableChipFieldApp extends StatelessWidget {
  const EditableChipFieldApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData(useMaterial3: true),
      home: const EditableChipFieldExample(),
    );
  }
}

class EditableChipFieldExample extends StatefulWidget {
  const EditableChipFieldExample({super.key});

  @override
  EditableChipFieldExampleState createState() {
    return EditableChipFieldExampleState();
  }
}

class EditableChipFieldExampleState extends State<EditableChipFieldExample> {
  final FocusNode _chipFocusNode = FocusNode();
  List<String> _toppings = <String>[_pizzaToppings.first];
  List<String> _suggestions = <String>[];

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Editable Chip Field Sample')),
      body: Column(
        children: <Widget>[
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 16),
            child: ChipsInput<String>(
              values: _toppings,
              decoration: const InputDecoration(
                prefixIcon: Icon(Icons.local_pizza_rounded),
                hintText: 'Search for toppings',
              ),
              strutStyle: const StrutStyle(fontSize: 15),
              onChanged: _onChanged,
              onSubmitted: _onSubmitted,
              chipBuilder: _chipBuilder,
              onTextChanged: _onSearchChanged,
            ),
          ),
          if (_suggestions.isNotEmpty)
            Expanded(
              child: ListView.builder(
                itemCount: _suggestions.length,
                itemBuilder: (BuildContext context, int index) {
                  return ToppingSuggestion(_suggestions[index], onTap: _selectSuggestion);
                },
              ),
            ),
        ],
      ),
    );
  }

  Future<void> _onSearchChanged(String value) async {
    final List<String> results = await _suggestionCallback(value);
    setState(() {
      _suggestions = results.where((String topping) => !_toppings.contains(topping)).toList();
    });
  }

  Widget _chipBuilder(BuildContext context, String topping) {
    return ToppingInputChip(topping: topping, onDeleted: _onChipDeleted, onSelected: _onChipTapped);
  }

  void _selectSuggestion(String topping) {
    setState(() {
      _toppings.add(topping);
      _suggestions = <String>[];
    });
  }

  void _onChipTapped(String topping) {}

  void _onChipDeleted(String topping) {
    setState(() {
      _toppings.remove(topping);
      _suggestions = <String>[];
    });
  }

  void _onSubmitted(String text) {
    if (text.trim().isNotEmpty) {
      setState(() {
        _toppings = <String>[..._toppings, text.trim()];
      });
    } else {
      _chipFocusNode.unfocus();
      setState(() {
        _toppings = <String>[];
      });
    }
  }

  void _onChanged(List<String> data) {
    setState(() {
      _toppings = data;
    });
  }

  FutureOr<List<String>> _suggestionCallback(String text) {
    if (text.isNotEmpty) {
      return _pizzaToppings.where((String topping) {
        return topping.toLowerCase().contains(text.toLowerCase());
      }).toList();
    }
    return const <String>[];
  }
}

class ChipsInput<T> extends StatefulWidget {
  const ChipsInput({
    super.key,
    required this.values,
    this.decoration = const InputDecoration(),
    this.style,
    this.strutStyle,
    required this.chipBuilder,
    required this.onChanged,
    this.onChipTapped,
    this.onSubmitted,
    this.onTextChanged,
  });

  final List<T> values;
  final InputDecoration decoration;
  final TextStyle? style;
  final StrutStyle? strutStyle;

  final ValueChanged<List<T>> onChanged;
  final ValueChanged<T>? onChipTapped;
  final ValueChanged<String>? onSubmitted;
  final ValueChanged<String>? onTextChanged;

  final Widget Function(BuildContext context, T data) chipBuilder;

  @override
  ChipsInputState<T> createState() => ChipsInputState<T>();
}

class ChipsInputState<T> extends State<ChipsInput<T>> {
  @visibleForTesting
  late final ChipsInputEditingController<T> controller;

  String _previousText = '';
  TextSelection? _previousSelection;

  @override
  void initState() {
    super.initState();

    controller = ChipsInputEditingController<T>(<T>[...widget.values], widget.chipBuilder);
    controller.addListener(_textListener);
  }

  @override
  void dispose() {
    controller.removeListener(_textListener);
    controller.dispose();

    super.dispose();
  }

  void _textListener() {
    final String currentText = controller.text;

    if (_previousSelection != null) {
      final int currentNumber = countReplacements(currentText);
      final int previousNumber = countReplacements(_previousText);

      final int cursorEnd = _previousSelection!.extentOffset;
      final int cursorStart = _previousSelection!.baseOffset;

      final List<T> values = <T>[...widget.values];

      // If the current number and the previous number of replacements are different, then
      // the user has deleted the InputChip using the keyboard. In this case, we trigger
      // the onChanged callback. We need to be sure also that the current number of
      // replacements is different from the input chip to avoid double-deletion.
      if (currentNumber < previousNumber && currentNumber != values.length) {
        if (cursorStart == cursorEnd) {
          values.removeRange(cursorStart - 1, cursorEnd);
        } else {
          if (cursorStart > cursorEnd) {
            values.removeRange(cursorEnd, cursorStart);
          } else {
            values.removeRange(cursorStart, cursorEnd);
          }
        }
        widget.onChanged(values);
      }
    }

    _previousText = currentText;
    _previousSelection = controller.selection;
  }

  static int countReplacements(String text) {
    return text.codeUnits
        .where((int u) => u == ChipsInputEditingController.kObjectReplacementChar)
        .length;
  }

  @override
  Widget build(BuildContext context) {
    controller.updateValues(<T>[...widget.values]);

    return TextField(
      minLines: 1,
      maxLines: 3,
      textInputAction: TextInputAction.done,
      style: widget.style,
      strutStyle: widget.strutStyle,
      controller: controller,
      onChanged: (String value) => widget.onTextChanged?.call(controller.textWithoutReplacements),
      onSubmitted: (String value) => widget.onSubmitted?.call(controller.textWithoutReplacements),
    );
  }
}

class ChipsInputEditingController<T> extends TextEditingController {
  ChipsInputEditingController(this.values, this.chipBuilder)
    : super(text: String.fromCharCode(kObjectReplacementChar) * values.length);

  // This constant character acts as a placeholder in the TextField text value.
  // There will be one character for each of the InputChip displayed.
  static const int kObjectReplacementChar = 0xFFFE;

  List<T> values;

  final Widget Function(BuildContext context, T data) chipBuilder;

  /// Called whenever chip is either added or removed
  /// from the outside the context of the text field.
  void updateValues(List<T> values) {
    if (values.length != this.values.length) {
      final String char = String.fromCharCode(kObjectReplacementChar);
      final int length = values.length;
      value = TextEditingValue(
        text: char * length,
        selection: TextSelection.collapsed(offset: length),
      );
      this.values = values;
    }
  }

  String get textWithoutReplacements {
    final String char = String.fromCharCode(kObjectReplacementChar);
    return text.replaceAll(RegExp(char), '');
  }

  String get textWithReplacements => text;

  @override
  TextSpan buildTextSpan({
    required BuildContext context,
    TextStyle? style,
    required bool withComposing,
  }) {
    final Iterable<WidgetSpan> chipWidgets = values.map(
      (T v) => WidgetSpan(child: chipBuilder(context, v)),
    );

    return TextSpan(
      style: style,
      children: <InlineSpan>[
        ...chipWidgets,
        if (textWithoutReplacements.isNotEmpty) TextSpan(text: textWithoutReplacements),
      ],
    );
  }
}

class ToppingSuggestion extends StatelessWidget {
  const ToppingSuggestion(this.topping, {super.key, this.onTap});

  final String topping;
  final ValueChanged<String>? onTap;

  @override
  Widget build(BuildContext context) {
    return ListTile(
      key: ObjectKey(topping),
      leading: CircleAvatar(child: Text(topping[0].toUpperCase())),
      title: Text(topping),
      onTap: () => onTap?.call(topping),
    );
  }
}

class ToppingInputChip extends StatelessWidget {
  const ToppingInputChip({
    super.key,
    required this.topping,
    required this.onDeleted,
    required this.onSelected,
  });

  final String topping;
  final ValueChanged<String> onDeleted;
  final ValueChanged<String> onSelected;

  @override
  Widget build(BuildContext context) {
    return Container(
      margin: const EdgeInsets.only(right: 3),
      child: InputChip(
        key: ObjectKey(topping),
        label: Text(topping),
        avatar: CircleAvatar(child: Text(topping[0].toUpperCase())),
        onDeleted: () => onDeleted(topping),
        onSelected: (bool value) => onSelected(topping),
        materialTapTargetSize: MaterialTapTargetSize.shrinkWrap,
        padding: const EdgeInsets.all(2),
      ),
    );
  }
}
