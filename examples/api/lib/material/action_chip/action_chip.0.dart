// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Flutter code sample [ActionChip].

import 'package:flutter/material.dart';

void main() => runApp(const ChipApp());

class ChipApp extends StatelessWidget {
  const ChipApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData(colorSchemeSeed: const Color(0xff6750a4), useMaterial3: true),
      home: const ActionChipExample(),
    );
  }
}

class ActionChipExample extends StatefulWidget {
  const ActionChipExample({super.key});

  @override
  State<ActionChipExample> createState() => _ActionChipExampleState();
}

class _ActionChipExampleState extends State<ActionChipExample> {
  bool favorite = false;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('ActionChip Sample')),
      body: Center(
        child: ActionChip(
          avatar: Icon(favorite ? Icons.favorite : Icons.favorite_border),
          label: const Text('Save to favorites'),
          onPressed: () {
            setState(() {
              favorite = !favorite;
            });
          },
        ),
      ),
    );
  }
}
