// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../../gallery/demo.dart';

class MenuDemo extends StatefulWidget {
  const MenuDemo({super.key});

  static const String routeName = '/material/menu';

  @override
  MenuDemoState createState() => MenuDemoState();
}

class MenuDemoState extends State<MenuDemo> {
  final String _simpleValue1 = 'Menu item value one';
  final String _simpleValue2 = 'Menu item value two';
  final String _simpleValue3 = 'Menu item value three';
  String? _simpleValue;

  final String _checkedValue1 = 'One';
  final String _checkedValue2 = 'Two';
  final String _checkedValue3 = 'Free';
  final String _checkedValue4 = 'Four';
  late List<String> _checkedValues;

  @override
  void initState() {
    super.initState();
    _simpleValue = _simpleValue2;
    _checkedValues = <String>[_checkedValue3];
  }

  void showInSnackBar(String value) {
    ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(value)));
  }

  void showMenuSelection(String value) {
    if (<String>[_simpleValue1, _simpleValue2, _simpleValue3].contains(value)) {
      setState(() => _simpleValue = value);
    }
    showInSnackBar('You selected: $value');
  }

  void showCheckedMenuSelections(String value) {
    if (_checkedValues.contains(value)) {
      _checkedValues.remove(value);
    } else {
      _checkedValues.add(value);
    }

    showInSnackBar('Checked $_checkedValues');
  }

  bool isChecked(String value) => _checkedValues.contains(value);

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Menus'),
        actions: <Widget>[
          MaterialDemoDocumentationButton(MenuDemo.routeName),
          PopupMenuButton<String>(
            onSelected: showMenuSelection,
            itemBuilder:
                (BuildContext context) => <PopupMenuItem<String>>[
                  const PopupMenuItem<String>(value: 'Toolbar menu', child: Text('Toolbar menu')),
                  const PopupMenuItem<String>(value: 'Right here', child: Text('Right here')),
                  const PopupMenuItem<String>(value: 'Hooray!', child: Text('Hooray!')),
                ],
          ),
        ],
      ),
      body: ListTileTheme(
        iconColor:
            Theme.brightnessOf(context) == Brightness.light ? Colors.grey[600] : Colors.grey[500],
        child: ListView(
          padding: kMaterialListPadding,
          children: <Widget>[
            // Pressing the PopupMenuButton on the right of this item shows
            // a simple menu with one disabled item. Typically the contents
            // of this "contextual menu" would reflect the app's state.
            ListTile(
              title: const Text('An item with a context menu button'),
              trailing: PopupMenuButton<String>(
                padding: EdgeInsets.zero,
                onSelected: showMenuSelection,
                itemBuilder:
                    (BuildContext context) => <PopupMenuItem<String>>[
                      PopupMenuItem<String>(
                        value: _simpleValue1,
                        child: const Text('Context menu item one'),
                      ),
                      const PopupMenuItem<String>(
                        enabled: false,
                        child: Text('A disabled menu item'),
                      ),
                      PopupMenuItem<String>(
                        value: _simpleValue3,
                        child: const Text('Context menu item three'),
                      ),
                    ],
              ),
            ),
            // Pressing the PopupMenuButton on the right of this item shows
            // a menu whose items have text labels and icons and a divider
            // That separates the first three items from the last one.
            ListTile(
              title: const Text('An item with a sectioned menu'),
              trailing: PopupMenuButton<String>(
                padding: EdgeInsets.zero,
                onSelected: showMenuSelection,
                itemBuilder:
                    (BuildContext context) => <PopupMenuEntry<String>>[
                      const PopupMenuItem<String>(
                        value: 'Preview',
                        child: ListTile(leading: Icon(Icons.visibility), title: Text('Preview')),
                      ),
                      const PopupMenuItem<String>(
                        value: 'Share',
                        child: ListTile(leading: Icon(Icons.person_add), title: Text('Share')),
                      ),
                      const PopupMenuItem<String>(
                        value: 'Get Link',
                        child: ListTile(leading: Icon(Icons.link), title: Text('Get link')),
                      ),
                      const PopupMenuDivider(),
                      const PopupMenuItem<String>(
                        value: 'Remove',
                        child: ListTile(leading: Icon(Icons.delete), title: Text('Remove')),
                      ),
                    ],
              ),
            ),
            // This entire list item is a PopupMenuButton. Tapping anywhere shows
            // a menu whose current value is highlighted and aligned over the
            // list item's center line.
            PopupMenuButton<String>(
              padding: EdgeInsets.zero,
              initialValue: _simpleValue,
              onSelected: showMenuSelection,
              child: ListTile(
                title: const Text('An item with a simple menu'),
                subtitle: Text(_simpleValue!),
              ),
              itemBuilder:
                  (BuildContext context) => <PopupMenuItem<String>>[
                    PopupMenuItem<String>(value: _simpleValue1, child: Text(_simpleValue1)),
                    PopupMenuItem<String>(value: _simpleValue2, child: Text(_simpleValue2)),
                    PopupMenuItem<String>(value: _simpleValue3, child: Text(_simpleValue3)),
                  ],
            ),
            // Pressing the PopupMenuButton on the right of this item shows a menu
            // whose items have checked icons that reflect this app's state.
            ListTile(
              title: const Text('An item with a checklist menu'),
              trailing: PopupMenuButton<String>(
                padding: EdgeInsets.zero,
                onSelected: showCheckedMenuSelections,
                itemBuilder:
                    (BuildContext context) => <PopupMenuItem<String>>[
                      CheckedPopupMenuItem<String>(
                        value: _checkedValue1,
                        checked: isChecked(_checkedValue1),
                        child: Text(_checkedValue1),
                      ),
                      CheckedPopupMenuItem<String>(
                        value: _checkedValue2,
                        enabled: false,
                        checked: isChecked(_checkedValue2),
                        child: Text(_checkedValue2),
                      ),
                      CheckedPopupMenuItem<String>(
                        value: _checkedValue3,
                        checked: isChecked(_checkedValue3),
                        child: Text(_checkedValue3),
                      ),
                      CheckedPopupMenuItem<String>(
                        value: _checkedValue4,
                        checked: isChecked(_checkedValue4),
                        child: Text(_checkedValue4),
                      ),
                    ],
              ),
            ),
          ],
        ),
      ),
    );
  }
}
