// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';

import 'package:flutter/material.dart';

/// Flutter code sample for custom labeled switch.

void main() => runApp(const LabeledSwitchApp());

class LabeledSwitchApp extends StatelessWidget {
  const LabeledSwitchApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData(useMaterial3: true),
      home: Scaffold(
        appBar: AppBar(title: const Text('Custom Labeled Switch Sample')),
        body: const Center(child: LabeledSwitchExample()),
      ),
    );
  }
}

class LinkedLabelSwitch extends StatelessWidget {
  const LinkedLabelSwitch({
    super.key,
    required this.label,
    required this.padding,
    required this.value,
    required this.onChanged,
  });

  final String label;
  final EdgeInsets padding;
  final bool value;
  final ValueChanged<bool> onChanged;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: padding,
      child: Row(
        children: <Widget>[
          Expanded(
            child: RichText(
              text: TextSpan(
                text: label,
                style: TextStyle(
                  color: Theme.of(context).colorScheme.primary,
                  decoration: TextDecoration.underline,
                ),
                recognizer:
                    TapGestureRecognizer()
                      ..onTap = () {
                        debugPrint('Label has been tapped.');
                      },
              ),
            ),
          ),
          Switch(
            value: value,
            onChanged: (bool newValue) {
              onChanged(newValue);
            },
          ),
        ],
      ),
    );
  }
}

class LabeledSwitchExample extends StatefulWidget {
  const LabeledSwitchExample({super.key});

  @override
  State<LabeledSwitchExample> createState() => _LabeledSwitchExampleState();
}

class _LabeledSwitchExampleState extends State<LabeledSwitchExample> {
  bool _isSelected = false;

  @override
  Widget build(BuildContext context) {
    return LinkedLabelSwitch(
      label: 'Linked, tappable label text',
      padding: const EdgeInsets.symmetric(horizontal: 20.0),
      value: _isSelected,
      onChanged: (bool newValue) {
        setState(() {
          _isSelected = newValue;
        });
      },
    );
  }
}
