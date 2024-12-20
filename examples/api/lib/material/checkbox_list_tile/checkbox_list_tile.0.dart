// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart' show timeDilation;

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
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('CheckboxListTile Sample')),
      body: Center(
        child: CheckboxListTile(
          title: const Text('Animate Slowly'),
          value: timeDilation != 1.0,
          onChanged: (bool? value) {
            setState(() {
              timeDilation = value! ? 10.0 : 1.0;
            });
          },
          secondary: const Icon(Icons.hourglass_empty),
        ),
      ),
    );
  }
}
