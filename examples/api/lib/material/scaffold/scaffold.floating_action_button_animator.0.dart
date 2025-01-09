// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [Scaffold.floatingActionButtonAnimator].

void main() => runApp(const ScaffoldFloatingActionButtonAnimatorApp());

class ScaffoldFloatingActionButtonAnimatorApp extends StatelessWidget {
  const ScaffoldFloatingActionButtonAnimatorApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: ScaffoldFloatingActionButtonAnimatorExample());
  }
}

enum FabAnimator { defaultStyle, none }

const List<(FabAnimator, String)> fabAnimatoregments = <(FabAnimator, String)>[
  (FabAnimator.defaultStyle, 'Default'),
  (FabAnimator.none, 'None'),
];

enum FabLocation { centerFloat, endFloat, endTop }

const List<(FabLocation, String)> fabLocationegments = <(FabLocation, String)>[
  (FabLocation.centerFloat, 'centerFloat'),
  (FabLocation.endFloat, 'endFloat'),
  (FabLocation.endTop, 'endTop'),
];

class ScaffoldFloatingActionButtonAnimatorExample extends StatefulWidget {
  const ScaffoldFloatingActionButtonAnimatorExample({super.key});

  @override
  State<ScaffoldFloatingActionButtonAnimatorExample> createState() =>
      _ScaffoldFloatingActionButtonAnimatorExampleState();
}

class _ScaffoldFloatingActionButtonAnimatorExampleState
    extends State<ScaffoldFloatingActionButtonAnimatorExample> {
  Set<FabAnimator> _selectedFabAnimator = <FabAnimator>{FabAnimator.defaultStyle};
  Set<FabLocation> _selectedFabLocation = <FabLocation>{FabLocation.endFloat};
  FloatingActionButtonAnimator? _floatingActionButtonAnimator;
  FloatingActionButtonLocation? _floatingActionButtonLocation;
  bool _showFab = false;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      floatingActionButtonLocation: _floatingActionButtonLocation,
      floatingActionButtonAnimator: _floatingActionButtonAnimator,
      appBar: AppBar(title: const Text('FloatingActionButtonAnimator Sample')),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            SegmentedButton<FabAnimator>(
              selected: _selectedFabAnimator,
              onSelectionChanged: (Set<FabAnimator> styles) {
                setState(() {
                  _floatingActionButtonAnimator = switch (styles.first) {
                    FabAnimator.defaultStyle => null,
                    FabAnimator.none => FloatingActionButtonAnimator.noAnimation,
                  };
                  _selectedFabAnimator = styles;
                });
              },
              segments:
                  fabAnimatoregments.map<ButtonSegment<FabAnimator>>((
                    (FabAnimator, String) fabAnimator,
                  ) {
                    final FabAnimator animator = fabAnimator.$1;
                    final String label = fabAnimator.$2;
                    return ButtonSegment<FabAnimator>(value: animator, label: Text(label));
                  }).toList(),
            ),
            const SizedBox(height: 10),
            SegmentedButton<FabLocation>(
              selected: _selectedFabLocation,
              onSelectionChanged: (Set<FabLocation> styles) {
                setState(() {
                  _floatingActionButtonLocation = switch (styles.first) {
                    FabLocation.centerFloat => FloatingActionButtonLocation.centerFloat,
                    FabLocation.endFloat => FloatingActionButtonLocation.endFloat,
                    FabLocation.endTop => FloatingActionButtonLocation.endTop,
                  };
                  _selectedFabLocation = styles;
                });
              },
              segments:
                  fabLocationegments.map<ButtonSegment<FabLocation>>((
                    (FabLocation, String) fabLocation,
                  ) {
                    final FabLocation location = fabLocation.$1;
                    final String label = fabLocation.$2;
                    return ButtonSegment<FabLocation>(value: location, label: Text(label));
                  }).toList(),
            ),
            const SizedBox(height: 10),
            FilledButton.icon(
              onPressed: () {
                setState(() {
                  _showFab = !_showFab;
                });
              },
              icon: Icon(_showFab ? Icons.visibility_off : Icons.visibility),
              label: const Text('Toggle FAB'),
            ),
          ],
        ),
      ),
      floatingActionButton:
          !_showFab ? null : FloatingActionButton(onPressed: () {}, child: const Icon(Icons.add)),
    );
  }
}
