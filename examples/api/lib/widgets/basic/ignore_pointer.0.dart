// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [IgnorePointer].

void main() => runApp(const IgnorePointerApp());

class IgnorePointerApp extends StatelessWidget {
  const IgnorePointerApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(centerTitle: true, title: const Text('IgnorePointer Sample')),
        body: const Center(child: IgnorePointerExample()),
      ),
    );
  }
}

class IgnorePointerExample extends StatefulWidget {
  const IgnorePointerExample({super.key});

  @override
  State<IgnorePointerExample> createState() => _IgnorePointerExampleState();
}

class _IgnorePointerExampleState extends State<IgnorePointerExample> {
  bool ignoring = false;
  void setIgnoring(bool newValue) {
    setState(() {
      ignoring = newValue;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      mainAxisAlignment: MainAxisAlignment.spaceEvenly,
      children: <Widget>[
        Text('Ignoring: $ignoring'),
        IgnorePointer(
          ignoring: ignoring,
          child: ElevatedButton(
            style: ElevatedButton.styleFrom(padding: const EdgeInsets.all(24.0)),
            onPressed: () {},
            child: const Text('Click me!'),
          ),
        ),
        FilledButton(
          onPressed: () {
            setIgnoring(!ignoring);
          },
          child: Text(ignoring ? 'Set ignoring to false' : 'Set ignoring to true'),
        ),
      ],
    );
  }
}
