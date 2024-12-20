// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

void main() {
  runApp(const MaterialApp(home: Test()));
}

class Test extends StatefulWidget {
  const Test({super.key});

  @override
  State<Test> createState() => _TestState();
}

class _TestState extends State<Test> {
  bool _triggered = false;

  @override
  void reassemble() {
    _triggered = true;
    super.reassemble();
  }

  @override
  Widget build(BuildContext context) {
    if (!_triggered) {
      return const SizedBox.shrink();
    }
    return const Row(
      children: <Widget>[
        SizedBox(width: 10000.0),
        SizedBox(width: 10000.0),
        SizedBox(width: 10000.0),
        SizedBox(width: 10000.0),
        SizedBox(width: 10000.0),
        SizedBox(width: 10000.0),
        SizedBox(width: 10000.0),
      ],
    );
  }
}
