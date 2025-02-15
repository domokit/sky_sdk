// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for migrating from [ToggleButtons] to [SegmentedButton].

void main() {
  runApp(const ToggleButtonsApp());
}

class ToggleButtonsApp extends StatelessWidget {
  const ToggleButtonsApp({super.key});
  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: Scaffold(body: ToggleButtonsExample()));
  }
}

enum ShirtSize { extraSmall, small, medium, large, extraLarge }

const List<(ShirtSize, String)> shirtSizeOptions = <(ShirtSize, String)>[
  (ShirtSize.extraSmall, 'XS'),
  (ShirtSize.small, 'S'),
  (ShirtSize.medium, 'M'),
  (ShirtSize.large, 'L'),
  (ShirtSize.extraLarge, 'XL'),
];

class ToggleButtonsExample extends StatefulWidget {
  const ToggleButtonsExample({super.key});

  @override
  State<ToggleButtonsExample> createState() => _ToggleButtonsExampleState();
}

class _ToggleButtonsExampleState extends State<ToggleButtonsExample> {
  final List<bool> _toggleButtonsSelection =
      ShirtSize.values.map((ShirtSize e) => e == ShirtSize.medium).toList();
  Set<ShirtSize> _segmentedButtonSelection = <ShirtSize>{ShirtSize.medium};

  @override
  Widget build(BuildContext context) {
    return Center(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: <Widget>[
          const Text('ToggleButtons'),
          const SizedBox(height: 10),
          // This ToggleButtons allows multiple or no selection.
          ToggleButtons(
            // ToggleButtons uses a List<bool> to track its selection state.
            isSelected: _toggleButtonsSelection,
            // This callback return the index of the child that was pressed.
            onPressed: (int index) {
              setState(() {
                _toggleButtonsSelection[index] = !_toggleButtonsSelection[index];
              });
            },
            // Constraints are used to determine the size of each child widget.
            constraints: const BoxConstraints(minHeight: 32.0, minWidth: 56.0),
            // ToggleButtons uses a List<Widget> to build its children.
            children: shirtSizeOptions.map(((ShirtSize, String) shirt) => Text(shirt.$2)).toList(),
          ),
          const SizedBox(height: 20),
          const Text('SegmentedButton'),
          const SizedBox(height: 10),
          SegmentedButton<ShirtSize>(
            // ToggleButtons above allows multiple or no selection.
            // Set `multiSelectionEnabled` and `emptySelectionAllowed` to true
            // to match the behavior of ToggleButtons.
            multiSelectionEnabled: true,
            emptySelectionAllowed: true,
            // Hide the selected icon to match the behavior of ToggleButtons.
            showSelectedIcon: false,
            // SegmentedButton uses a Set<T> to track its selection state.
            selected: _segmentedButtonSelection,
            // This callback updates the set of selected segment values.
            onSelectionChanged: (Set<ShirtSize> newSelection) {
              setState(() {
                _segmentedButtonSelection = newSelection;
              });
            },
            // SegmentedButton uses a List<ButtonSegment<T>> to build its children
            // instead of a List<Widget> like ToggleButtons.
            segments:
                shirtSizeOptions.map<ButtonSegment<ShirtSize>>(((ShirtSize, String) shirt) {
                  return ButtonSegment<ShirtSize>(value: shirt.$1, label: Text(shirt.$2));
                }).toList(),
          ),
        ],
      ),
    );
  }
}
