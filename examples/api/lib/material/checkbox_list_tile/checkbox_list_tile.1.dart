// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [CheckboxListTile].

void main() => runApp(const CheckboxListTileApp());

class CheckboxListTileApp extends StatelessWidget {
  const CheckboxListTileApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(theme: ThemeData(useMaterial3: true), home: const CheckboxListTileExample());
  }
}

class CheckboxListTileExample extends StatefulWidget {
  const CheckboxListTileExample({super.key});

  @override
  State<CheckboxListTileExample> createState() => _CheckboxListTileExampleState();
}

class _CheckboxListTileExampleState extends State<CheckboxListTileExample> {
  bool checkboxValue1 = true;
  bool checkboxValue2 = true;
  bool checkboxValue3 = true;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('CheckboxListTile Sample')),
      body: Column(
        children: <Widget>[
          CheckboxListTile(
            value: checkboxValue1,
            onChanged: (bool? value) {
              setState(() {
                checkboxValue1 = value!;
              });
            },
            title: const Text('Headline'),
            subtitle: const Text('Supporting text'),
          ),
          const Divider(height: 0),
          CheckboxListTile(
            value: checkboxValue2,
            onChanged: (bool? value) {
              setState(() {
                checkboxValue2 = value!;
              });
            },
            title: const Text('Headline'),
            subtitle: const Text(
              'Longer supporting text to demonstrate how the text wraps and the checkbox is centered vertically with the text.',
            ),
          ),
          const Divider(height: 0),
          CheckboxListTile(
            value: checkboxValue3,
            onChanged: (bool? value) {
              setState(() {
                checkboxValue3 = value!;
              });
            },
            title: const Text('Headline'),
            subtitle: const Text(
              "Longer supporting text to demonstrate how the text wraps and how setting 'CheckboxListTile.isThreeLine = true' aligns the checkbox to the top vertically with the text.",
            ),
            isThreeLine: true,
          ),
          const Divider(height: 0),
        ],
      ),
    );
  }
}
