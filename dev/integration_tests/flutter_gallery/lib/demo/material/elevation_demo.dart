// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../../gallery/demo.dart';

class ElevationDemo extends StatefulWidget {
  const ElevationDemo({super.key});

  static const String routeName = '/material/elevation';

  @override
  State<StatefulWidget> createState() => _ElevationDemoState();
}

class _ElevationDemoState extends State<ElevationDemo> {
  bool _showElevation = true;

  List<Widget> buildCards() {
    const List<double> elevations = <double>[0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 8.0, 16.0, 24.0];

    return elevations.map<Widget>((double elevation) {
      return Center(
        child: Card(
          margin: const EdgeInsets.all(20.0),
          elevation: _showElevation ? elevation : 0.0,
          child: SizedBox(
            height: 100.0,
            width: 100.0,
            child: Center(child: Text('${elevation.toStringAsFixed(0)} pt')),
          ),
        ),
      );
    }).toList();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Elevation'),
        actions: <Widget>[
          MaterialDemoDocumentationButton(ElevationDemo.routeName),
          IconButton(
            tooltip: 'Toggle elevation',
            icon: const Icon(Icons.sentiment_very_satisfied),
            onPressed: () {
              setState(() => _showElevation = !_showElevation);
            },
          ),
        ],
      ),
      body: Scrollbar(child: ListView(primary: true, children: buildCards())),
    );
  }
}
