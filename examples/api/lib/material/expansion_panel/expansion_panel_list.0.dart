// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [ExpansionPanelList].

void main() => runApp(const ExpansionPanelListExampleApp());

class ExpansionPanelListExampleApp extends StatelessWidget {
  const ExpansionPanelListExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('ExpansionPanelList Sample')),
        body: const ExpansionPanelListExample(),
      ),
    );
  }
}

// stores ExpansionPanel state information
class Item {
  Item({required this.expandedValue, required this.headerValue, this.isExpanded = false});

  String expandedValue;
  String headerValue;
  bool isExpanded;
}

List<Item> generateItems(int numberOfItems) {
  return List<Item>.generate(numberOfItems, (int index) {
    return Item(headerValue: 'Panel $index', expandedValue: 'This is item number $index');
  });
}

class ExpansionPanelListExample extends StatefulWidget {
  const ExpansionPanelListExample({super.key});

  @override
  State<ExpansionPanelListExample> createState() => _ExpansionPanelListExampleState();
}

class _ExpansionPanelListExampleState extends State<ExpansionPanelListExample> {
  final List<Item> _data = generateItems(8);

  @override
  Widget build(BuildContext context) {
    return SingleChildScrollView(child: Container(child: _buildPanel()));
  }

  Widget _buildPanel() {
    return ExpansionPanelList(
      expansionCallback: (int index, bool isExpanded) {
        setState(() {
          _data[index].isExpanded = isExpanded;
        });
      },
      children:
          _data.map<ExpansionPanel>((Item item) {
            return ExpansionPanel(
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
              isExpanded: item.isExpanded,
            );
          }).toList(),
    );
  }
}
