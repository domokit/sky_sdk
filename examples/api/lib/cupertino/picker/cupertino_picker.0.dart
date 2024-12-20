// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

/// Flutter code sample for [CupertinoPicker].

const double _kItemExtent = 32.0;
const List<String> _fruitNames = <String>[
  'Apple',
  'Mango',
  'Banana',
  'Orange',
  'Pineapple',
  'Strawberry',
];

void main() => runApp(const CupertinoPickerApp());

class CupertinoPickerApp extends StatelessWidget {
  const CupertinoPickerApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const CupertinoApp(
      theme: CupertinoThemeData(brightness: Brightness.light),
      home: CupertinoPickerExample(),
    );
  }
}

class CupertinoPickerExample extends StatefulWidget {
  const CupertinoPickerExample({super.key});

  @override
  State<CupertinoPickerExample> createState() => _CupertinoPickerExampleState();
}

class _CupertinoPickerExampleState extends State<CupertinoPickerExample> {
  int _selectedFruit = 0;

  // This shows a CupertinoModalPopup with a reasonable fixed height which hosts CupertinoPicker.
  void _showDialog(Widget child) {
    showCupertinoModalPopup<void>(
      context: context,
      builder:
          (BuildContext context) => Container(
            height: 216,
            padding: const EdgeInsets.only(top: 6.0),
            // The Bottom margin is provided to align the popup above the system navigation bar.
            margin: EdgeInsets.only(bottom: MediaQuery.of(context).viewInsets.bottom),
            // Provide a background color for the popup.
            color: CupertinoColors.systemBackground.resolveFrom(context),
            // Use a SafeArea widget to avoid system overlaps.
            child: SafeArea(top: false, child: child),
          ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: const CupertinoNavigationBar(middle: Text('CupertinoPicker Sample')),
      child: DefaultTextStyle(
        style: TextStyle(color: CupertinoColors.label.resolveFrom(context), fontSize: 22.0),
        child: Center(
          child: Row(
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[
              const Text('Selected fruit: '),
              CupertinoButton(
                padding: EdgeInsets.zero,
                // Display a CupertinoPicker with list of fruits.
                onPressed:
                    () => _showDialog(
                      CupertinoPicker(
                        magnification: 1.22,
                        squeeze: 1.2,
                        useMagnifier: true,
                        itemExtent: _kItemExtent,
                        // This sets the initial item.
                        scrollController: FixedExtentScrollController(initialItem: _selectedFruit),
                        // This is called when selected item is changed.
                        onSelectedItemChanged: (int selectedItem) {
                          setState(() {
                            _selectedFruit = selectedItem;
                          });
                        },
                        children: List<Widget>.generate(_fruitNames.length, (int index) {
                          return Center(child: Text(_fruitNames[index]));
                        }),
                      ),
                    ),
                // This displays the selected fruit name.
                child: Text(_fruitNames[_selectedFruit], style: const TextStyle(fontSize: 22.0)),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
