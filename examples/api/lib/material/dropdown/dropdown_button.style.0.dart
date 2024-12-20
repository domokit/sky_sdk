// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [DropdownButton.style].

void main() => runApp(const DropdownButtonApp());

class DropdownButtonApp extends StatelessWidget {
  const DropdownButtonApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('DropdownButton Sample')),
        body: const DropdownButtonExample(),
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
  List<String> options = <String>['One', 'Two', 'Three', 'Four'];
  String dropdownValue = 'One';

  @override
  Widget build(BuildContext context) {
    return Container(
      alignment: Alignment.center,
      color: Colors.blue,
      child: DropdownButton<String>(
        value: dropdownValue,
        onChanged: (String? value) {
          // This is called when the user selects an item.
          setState(() {
            dropdownValue = value!;
          });
        },
        style: const TextStyle(color: Colors.blue),
        selectedItemBuilder: (BuildContext context) {
          // This is the widget that will be shown when you select an item.
          // Here custom text style, alignment and layout size can be applied
          // to selected item string.
          return options.map((String value) {
            return Align(
              alignment: Alignment.centerLeft,
              child: Text(dropdownValue, style: const TextStyle(color: Colors.white)),
            );
          }).toList();
        },
        items:
            options.map<DropdownMenuItem<String>>((String value) {
              return DropdownMenuItem<String>(value: value, child: Text(value));
            }).toList(),
      ),
    );
  }
}
