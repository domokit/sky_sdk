// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [StandardFabLocation].

void main() => runApp(const StandardFabLocationExampleApp());

class StandardFabLocationExampleApp extends StatelessWidget {
  const StandardFabLocationExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: StandardFabLocationExample());
  }
}

class AlmostEndFloatFabLocation extends StandardFabLocation with FabEndOffsetX, FabFloatOffsetY {
  @override
  double getOffsetX(ScaffoldPrelayoutGeometry scaffoldGeometry, double adjustment) {
    final double directionalAdjustment =
        scaffoldGeometry.textDirection == TextDirection.ltr ? -50.0 : 50.0;
    return super.getOffsetX(scaffoldGeometry, adjustment) + directionalAdjustment;
  }
}

class StandardFabLocationExample extends StatelessWidget {
  const StandardFabLocationExample({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Home page')),
      floatingActionButton: FloatingActionButton(
        onPressed: () {
          debugPrint('FAB pressed.');
        },
        tooltip: 'Increment',
        child: const Icon(Icons.add),
      ),
      floatingActionButtonLocation: AlmostEndFloatFabLocation(),
    );
  }
}
