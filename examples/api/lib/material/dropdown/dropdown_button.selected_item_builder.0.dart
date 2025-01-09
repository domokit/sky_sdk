// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [DropdownButton.selectedItemBuilder].

Map<String, String> cities = <String, String>{
  'New York': 'NYC',
  'Los Angeles': 'LA',
  'San Francisco': 'SF',
  'Chicago': 'CH',
  'Miami': 'MI',
};

void main() => runApp(const DropdownButtonApp());

class DropdownButtonApp extends StatelessWidget {
  const DropdownButtonApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('DropdownButton Sample')),
        body: const Center(child: DropdownButtonExample()),
      ),
    );
  }
}

class DropdownButtonExample extends StatefulWidget {
  const DropdownButtonExample({super.key});

  @override
  State<DropdownButtonExample> createState() => _DropdownButtonExampleState();
}

class _DropdownButtonExampleState extends State<DropdownButtonExample> {
  String selectedItem = cities.keys.first;

  @override
  Widget build(BuildContext context) {
    return Center(
      child: Row(
        mainAxisAlignment: MainAxisAlignment.center,
        children: <Widget>[
          Text('Select a city:', style: Theme.of(context).textTheme.bodyLarge),
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 8.0),
            child: DropdownButton<String>(
              value: selectedItem,
              onChanged: (String? value) {
                // This is called when the user selects an item.
                setState(() => selectedItem = value!);
              },
              selectedItemBuilder: (BuildContext context) {
                return cities.values.map<Widget>((String item) {
                  // This is the widget that will be shown when you select an item.
                  // Here custom text style, alignment and layout size can be applied
                  // to selected item string.
                  return Container(
                    alignment: Alignment.centerLeft,
                    constraints: const BoxConstraints(minWidth: 100),
                    child: Text(
                      item,
                      style: const TextStyle(color: Colors.blue, fontWeight: FontWeight.w600),
                    ),
                  );
                }).toList();
              },
              items:
                  cities.keys.map<DropdownMenuItem<String>>((String item) {
                    return DropdownMenuItem<String>(value: item, child: Text(item));
                  }).toList(),
            ),
          ),
        ],
      ),
    );
  }
}
