// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [ExpansionPanelList.radio].

void main() => runApp(const ExpansionPanelListRadioExampleApp());

class ExpansionPanelListRadioExampleApp extends StatelessWidget {
  const ExpansionPanelListRadioExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('ExpansionPanelList.radio Sample')),
        body: const ExpansionPanelListRadioExample(),
      ),
    );
  }
}

// stores ExpansionPanel state information
class Item {
  Item({required this.id, required this.expandedValue, required this.headerValue});

  int id;
  String expandedValue;
  String headerValue;
}

List<Item> generateItems(int numberOfItems) {
  return List<Item>.generate(numberOfItems, (int index) {
    return Item(
      id: index,
      headerValue: 'Panel $index',
      expandedValue: 'This is item number $index',
    );
  });
}

class ExpansionPanelListRadioExample extends StatefulWidget {
  const ExpansionPanelListRadioExample({super.key});

  @override
  State<ExpansionPanelListRadioExample> createState() => _ExpansionPanelListRadioExampleState();
}

class _ExpansionPanelListRadioExampleState extends State<ExpansionPanelListRadioExample> {
  final List<Item> _data = generateItems(8);

  @override
  Widget build(BuildContext context) {
    return SingleChildScrollView(child: Container(child: _buildPanel()));
  }

  Widget _buildPanel() {
    return ExpansionPanelList.radio(
      initialOpenPanelValue: 2,
      children:
          _data.map<ExpansionPanelRadio>((Item item) {
            return ExpansionPanelRadio(
              value: item.id,
              headerBuilder: (BuildContext context, bool isExpanded) {
                return ListTile(title: Text(item.headerValue));
              },
              body: ListTile(
                title: Text(item.expandedValue),
                subtitle: const Text('To delete this panel, tap the trash can icon'),
                trailing: const Icon(Icons.delete),
                onTap: () {
                  setState(() {
                    _data.removeWhere((Item currentItem) => item == currentItem);
                  });
                },
              ),
            );
          }).toList(),
    );
  }
}
