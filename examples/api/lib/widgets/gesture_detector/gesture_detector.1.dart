// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [GestureDetector].

void main() => runApp(const GestureDetectorExampleApp());

class GestureDetectorExampleApp extends StatelessWidget {
  const GestureDetectorExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: GestureDetectorExample());
  }
}

class GestureDetectorExample extends StatefulWidget {
  const GestureDetectorExample({super.key});

  @override
  State<GestureDetectorExample> createState() => _GestureDetectorExampleState();
}

class _GestureDetectorExampleState extends State<GestureDetectorExample> {
  Color _color = Colors.white;

  @override
  Widget build(BuildContext context) {
    return Container(
      color: _color,
      height: 200.0,
      width: 200.0,
      child: GestureDetector(
        onTap: () {
          setState(() {
            _color == Colors.yellow ? _color = Colors.white : _color = Colors.yellow;
          });
        },
      ),
    );
  }
}
