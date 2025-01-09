// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [ListTile].

void main() => runApp(const ListTileApp());

class ListTileApp extends StatelessWidget {
  const ListTileApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(theme: ThemeData(useMaterial3: true), home: const ListTileExample());
  }
}

class ListTileExample extends StatefulWidget {
  const ListTileExample({super.key});

  @override
  State<ListTileExample> createState() => _ListTileExampleState();
}

class _ListTileExampleState extends State<ListTileExample> {
  bool _selected = false;
  bool _enabled = true;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('ListTile Sample')),
      body: Center(
        child: ListTile(
          enabled: _enabled,
          selected: _selected,
          onTap: () {
            setState(() {
              // This is called when the user toggles the switch.
              _selected = !_selected;
            });
          },
          iconColor: const WidgetStateColor.fromMap(<WidgetStatesConstraint, Color>{
            WidgetState.disabled: Colors.red,
            WidgetState.selected: Colors.green,
            WidgetState.any: Colors.black,
          }),
          // The same can be achieved using the .resolveWith() constructor.
          // The text color will be identical to the icon color above.
          textColor: WidgetStateColor.resolveWith((Set<WidgetState> states) {
            if (states.contains(WidgetState.disabled)) {
              return Colors.red;
            }
            if (states.contains(WidgetState.selected)) {
              return Colors.green;
            }
            return Colors.black;
          }),
          leading: const Icon(Icons.person),
          title: const Text('Headline'),
          subtitle: Text('Enabled: $_enabled, Selected: $_selected'),
          trailing: Switch(
            onChanged: (bool? value) {
              // This is called when the user toggles the switch.
              setState(() {
                _enabled = value!;
              });
            },
            value: _enabled,
          ),
        ),
      ),
    );
  }
}
