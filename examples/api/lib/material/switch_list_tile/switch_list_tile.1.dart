// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [SwitchListTile].

void main() => runApp(const SwitchListTileApp());

class SwitchListTileApp extends StatelessWidget {
  const SwitchListTileApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData(useMaterial3: true),
      home: Scaffold(
        appBar: AppBar(title: const Text('SwitchListTile Sample')),
        body: const SwitchListTileExample(),
      ),
    );
  }
}

class SwitchListTileExample extends StatefulWidget {
  const SwitchListTileExample({super.key});

  @override
  State<SwitchListTileExample> createState() => _SwitchListTileExampleState();
}

class _SwitchListTileExampleState extends State<SwitchListTileExample> {
  bool switchValue1 = true;
  bool switchValue2 = true;
  bool switchValue3 = true;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Column(
        children: <Widget>[
          SwitchListTile(
            value: switchValue1,
            onChanged: (bool? value) {
              setState(() {
                switchValue1 = value!;
              });
            },
            title: const Text('Headline'),
            subtitle: const Text('Supporting text'),
          ),
          const Divider(height: 0),
          SwitchListTile(
            value: switchValue2,
            onChanged: (bool? value) {
              setState(() {
                switchValue2 = value!;
              });
            },
            title: const Text('Headline'),
            subtitle: const Text(
              'Longer supporting text to demonstrate how the text wraps and the switch is centered vertically with the text.',
            ),
          ),
          const Divider(height: 0),
          SwitchListTile(
            value: switchValue3,
            onChanged: (bool? value) {
              setState(() {
                switchValue3 = value!;
              });
            },
            title: const Text('Headline'),
            subtitle: const Text(
              "Longer supporting text to demonstrate how the text wraps and how setting 'SwitchListTile.isThreeLine = true' aligns the switch to the top vertically with the text.",
            ),
            isThreeLine: true,
          ),
          const Divider(height: 0),
        ],
      ),
    );
  }
}
