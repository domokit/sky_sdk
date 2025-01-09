// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [ToggleButtons].

const List<Widget> fruits = <Widget>[Text('Apple'), Text('Banana'), Text('Orange')];

const List<Widget> vegetables = <Widget>[Text('Tomatoes'), Text('Potatoes'), Text('Carrots')];

const List<Widget> icons = <Widget>[Icon(Icons.sunny), Icon(Icons.cloud), Icon(Icons.ac_unit)];

void main() => runApp(const ToggleButtonsExampleApp());

class ToggleButtonsExampleApp extends StatelessWidget {
  const ToggleButtonsExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData(useMaterial3: true),
      home: const ToggleButtonsSample(title: 'ToggleButtons Sample'),
    );
  }
}

class ToggleButtonsSample extends StatefulWidget {
  const ToggleButtonsSample({super.key, required this.title});

  final String title;

  @override
  State<ToggleButtonsSample> createState() => _ToggleButtonsSampleState();
}

class _ToggleButtonsSampleState extends State<ToggleButtonsSample> {
  final List<bool> _selectedFruits = <bool>[true, false, false];
  final List<bool> _selectedVegetables = <bool>[false, true, false];
  final List<bool> _selectedWeather = <bool>[false, false, true];
  bool vertical = false;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);

    return Scaffold(
      appBar: AppBar(title: Text(widget.title)),
      body: Center(
        child: SingleChildScrollView(
          child: Column(
            mainAxisSize: MainAxisSize.min,
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[
              // ToggleButtons with a single selection.
              Text('Single-select', style: theme.textTheme.titleSmall),
              const SizedBox(height: 5),
              ToggleButtons(
                direction: vertical ? Axis.vertical : Axis.horizontal,
                onPressed: (int index) {
                  setState(() {
                    // The button that is tapped is set to true, and the others to false.
                    for (int i = 0; i < _selectedFruits.length; i++) {
                      _selectedFruits[i] = i == index;
                    }
                  });
                },
                borderRadius: const BorderRadius.all(Radius.circular(8)),
                selectedBorderColor: Colors.red[700],
                selectedColor: Colors.white,
                fillColor: Colors.red[200],
                color: Colors.red[400],
                constraints: const BoxConstraints(minHeight: 40.0, minWidth: 80.0),
                isSelected: _selectedFruits,
                children: fruits,
              ),
              const SizedBox(height: 20),
              // ToggleButtons with a multiple selection.
              Text('Multi-select', style: theme.textTheme.titleSmall),
              const SizedBox(height: 5),
              ToggleButtons(
                direction: vertical ? Axis.vertical : Axis.horizontal,
                onPressed: (int index) {
                  // All buttons are selectable.
                  setState(() {
                    _selectedVegetables[index] = !_selectedVegetables[index];
                  });
                },
                borderRadius: const BorderRadius.all(Radius.circular(8)),
                selectedBorderColor: Colors.green[700],
                selectedColor: Colors.white,
                fillColor: Colors.green[200],
                color: Colors.green[400],
                constraints: const BoxConstraints(minHeight: 40.0, minWidth: 80.0),
                isSelected: _selectedVegetables,
                children: vegetables,
              ),
              const SizedBox(height: 20),
              // ToggleButtons with icons only.
              Text('Icon-only', style: theme.textTheme.titleSmall),
              const SizedBox(height: 5),
              ToggleButtons(
                direction: vertical ? Axis.vertical : Axis.horizontal,
                onPressed: (int index) {
                  setState(() {
                    // The button that is tapped is set to true, and the others to false.
                    for (int i = 0; i < _selectedWeather.length; i++) {
                      _selectedWeather[i] = i == index;
                    }
                  });
                },
                borderRadius: const BorderRadius.all(Radius.circular(8)),
                selectedBorderColor: Colors.blue[700],
                selectedColor: Colors.white,
                fillColor: Colors.blue[200],
                color: Colors.blue[400],
                isSelected: _selectedWeather,
                children: icons,
              ),
            ],
          ),
        ),
      ),
      floatingActionButton: FloatingActionButton.extended(
        onPressed: () {
          setState(() {
            // When the button is pressed, ToggleButtons direction is changed.
            vertical = !vertical;
          });
        },
        icon: const Icon(Icons.screen_rotation_outlined),
        label: Text(vertical ? 'Horizontal' : 'Vertical'),
      ),
    );
  }
}
