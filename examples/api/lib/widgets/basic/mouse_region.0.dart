// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [MouseRegion].

void main() => runApp(const MouseRegionApp());

class MouseRegionApp extends StatelessWidget {
  const MouseRegionApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('MouseRegion Sample')),
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
  int _enterCounter = 0;
  int _exitCounter = 0;
  double x = 0.0;
  double y = 0.0;

  void _incrementEnter(PointerEvent details) {
    setState(() {
      _enterCounter++;
    });
  }

  void _incrementExit(PointerEvent details) {
    setState(() {
      _exitCounter++;
    });
  }

  void _updateLocation(PointerEvent details) {
    setState(() {
      x = details.position.dx;
      y = details.position.dy;
    });
  }

  @override
  Widget build(BuildContext context) {
    return ConstrainedBox(
      constraints: BoxConstraints.tight(const Size(300.0, 200.0)),
      child: MouseRegion(
        onEnter: _incrementEnter,
        onHover: _updateLocation,
        onExit: _incrementExit,
        child: ColoredBox(
          color: Colors.lightBlueAccent,
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[
              const Text('You have entered or exited this box this many times:'),
              Text(
                '$_enterCounter Entries\n$_exitCounter Exits',
                style: Theme.of(context).textTheme.headlineMedium,
              ),
              Text('The cursor is here: (${x.toStringAsFixed(2)}, ${y.toStringAsFixed(2)})'),
            ],
          ),
        ),
      ),
    );
  }
}
