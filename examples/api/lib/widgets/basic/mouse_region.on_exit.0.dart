// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [MouseRegion.onExit].

void main() => runApp(const MouseRegionApp());

class MouseRegionApp extends StatelessWidget {
  const MouseRegionApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('MouseRegion.onExit Sample')),
        body: const Center(child: MouseRegionExample()),
      ),
    );
  }
}

class MouseRegionExample extends StatefulWidget {
  const MouseRegionExample({super.key});

  @override
  State<MouseRegionExample> createState() => _MouseRegionExampleState();
}

class _MouseRegionExampleState extends State<MouseRegionExample> {
  bool hovered = false;

  @override
  Widget build(BuildContext context) {
    return Container(
      height: 100,
      width: 100,
      decoration: BoxDecoration(color: hovered ? Colors.yellow : Colors.blue),
      child: MouseRegion(
        onEnter: (_) {
          setState(() {
            hovered = true;
          });
        },
        onExit: (_) {
          setState(() {
            hovered = false;
          });
        },
      ),
    );
  }
}
