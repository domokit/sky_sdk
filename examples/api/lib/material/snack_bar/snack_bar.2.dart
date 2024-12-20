// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [SnackBar].

void main() => runApp(const SnackBarExampleApp());

/// A Material 3 [SnackBar] demonstrating an optional icon, in either floating
/// or fixed format.
class SnackBarExampleApp extends StatelessWidget {
  const SnackBarExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(theme: ThemeData(useMaterial3: true), home: const SnackBarExample());
  }
}

class SnackBarExample extends StatefulWidget {
  const SnackBarExample({super.key});

  @override
  State<SnackBarExample> createState() => _SnackBarExampleState();
}

class _SnackBarExampleState extends State<SnackBarExample> {
  SnackBarBehavior? _snackBarBehavior = SnackBarBehavior.floating;
  bool _withIcon = true;
  bool _withAction = true;
  bool _multiLine = false;
  bool _longActionLabel = false;
  double _sliderValue = 0.25;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('SnackBar Sample')),
      floatingActionButton: FloatingActionButton.extended(
        onPressed: () {
          ScaffoldMessenger.of(context).showSnackBar(_snackBar());
        },
        icon: const Icon(Icons.play_arrow),
        label: const Text('Show Snackbar'),
      ),
      body: ListView(
        children: <Widget>[
          ExpansionTile(
            title: const Text('Behavior'),
            initiallyExpanded: true,
            children: <Widget>[
              RadioListTile<SnackBarBehavior>(
                title: const Text('Fixed'),
                value: SnackBarBehavior.fixed,
                groupValue: _snackBarBehavior,
                onChanged: (SnackBarBehavior? value) {
                  setState(() {
                    _snackBarBehavior = value;
                  });
                },
              ),
              RadioListTile<SnackBarBehavior>(
                title: const Text('Floating'),
                value: SnackBarBehavior.floating,
                groupValue: _snackBarBehavior,
                onChanged: (SnackBarBehavior? value) {
                  setState(() {
                    _snackBarBehavior = value;
                  });
                },
              ),
            ],
          ),
          ExpansionTile(
            title: const Text('Content'),
            initiallyExpanded: true,
            children: <Widget>[
              SwitchListTile(
                title: const Text('Include close Icon'),
                value: _withIcon,
                onChanged: (bool value) {
                  setState(() {
                    _withIcon = value;
                  });
                },
              ),
              SwitchListTile(
                title: const Text('Multi Line Text'),
                value: _multiLine,
                onChanged: (bool value) {
                  setState(() {
                    _multiLine = value;
                  });
                },
              ),
              SwitchListTile(
                title: const Text('Include Action'),
                value: _withAction,
                onChanged: (bool value) {
                  setState(() {
                    _withAction = value;
                  });
                },
              ),
              SwitchListTile(
                title: const Text('Long Action Label'),
                value: _longActionLabel,
                onChanged:
                    !_withAction
                        ? null
                        : (bool value) => setState(() {
                          _longActionLabel = value;
                        }),
              ),
            ],
          ),
          ExpansionTile(
            title: const Text('Action new-line overflow threshold'),
            initiallyExpanded: true,
            children: <Widget>[
              Slider(
                value: _sliderValue,
                divisions: 20,
                label: _sliderValue.toStringAsFixed(2),
                onChanged:
                    (double value) => setState(() {
                      _sliderValue = value;
                    }),
              ),
            ],
          ),
          // Avoid hiding content behind the floating action button
          const SizedBox(height: 100),
        ],
      ),
    );
  }

  SnackBar _snackBar() {
    final SnackBarAction? action =
        _withAction
            ? SnackBarAction(
              label: _longActionLabel ? 'Long Action Text' : 'Action',
              onPressed: () {
                // Code to execute.
              },
            )
            : null;
    final double? width = _snackBarBehavior == SnackBarBehavior.floating ? 400.0 : null;
    final String label =
        _multiLine
            ? 'A Snack Bar with quite a lot of text which spans across multiple '
                'lines. You can look at how the Action Label moves around when trying '
                'to layout this text.'
            : 'Single Line Snack Bar';
    return SnackBar(
      content: Text(label),
      showCloseIcon: _withIcon,
      width: width,
      behavior: _snackBarBehavior,
      action: action,
      duration: const Duration(seconds: 3),
      actionOverflowThreshold: _sliderValue,
    );
  }
}
