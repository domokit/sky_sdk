// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_driver/driver_extension.dart';

/// This sample application creates a hard to render frame, causing the
/// driver script to race the raster thread. If the driver script wins the
/// race, it will screenshot the previous frame. If the raster thread wins
/// it, it will screenshot the latest frame.
void main() {
  enableFlutterDriverExtension();

  runApp(const Toggler());
}

class Toggler extends StatefulWidget {
  const Toggler({super.key});

  @override
  State<Toggler> createState() => TogglerState();
}

class TogglerState extends State<Toggler> {
  bool _visible = false;

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('FlutterDriver test')),
        body: Material(
          child: Column(
            children: <Widget>[
              TextButton(
                key: const ValueKey<String>('toggle'),
                child: const Text('Toggle visibility'),
                onPressed: () {
                  setState(() {
                    _visible = !_visible;
                  });
                },
              ),
              Expanded(child: ListView(children: _buildRows(_visible ? 10 : 0))),
            ],
          ),
        ),
      ),
    );
  }
}

List<Widget> _buildRows(int count) {
  return List<Widget>.generate(count, (int i) {
    return Row(children: _buildCells(i / count));
  });
}

/// Builds cells that are known to take time to render causing a delay on the
/// raster thread.
List<Widget> _buildCells(double epsilon) {
  return List<Widget>.generate(15, (int i) {
    return Expanded(
      child: Material(
        // A magic color that the test will be looking for on the screenshot.
        color: const Color(0xffff0102),
        borderRadius: BorderRadius.all(Radius.circular(i.toDouble() + epsilon)),
        elevation: 5.0,
        child: const SizedBox(height: 10.0, width: 10.0),
      ),
    );
  });
}
