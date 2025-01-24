// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [ChipAttributes.chipAnimationStyle].

void main() => runApp(const ChipAnimationStyleExampleApp());

class ChipAnimationStyleExampleApp extends StatelessWidget {
  const ChipAnimationStyleExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: Scaffold(body: Center(child: ChipAnimationStyleExample())));
  }
}

class ChipAnimationStyleExample extends StatefulWidget {
  const ChipAnimationStyleExample({super.key});

  @override
  State<ChipAnimationStyleExample> createState() => _ChipAnimationStyleExampleState();
}

class _ChipAnimationStyleExampleState extends State<ChipAnimationStyleExample> {
  bool enabled = true;
  bool selected = false;
  bool showCheckmark = true;
  bool showDeleteIcon = true;

  @override
  Widget build(BuildContext context) {
    return Column(
      mainAxisAlignment: MainAxisAlignment.spaceEvenly,
      children: <Widget>[
        Row(
          mainAxisAlignment: MainAxisAlignment.spaceEvenly,
          children: <Widget>[
            Column(
              mainAxisSize: MainAxisSize.min,
              children: <Widget>[
                FilterChip.elevated(
                  chipAnimationStyle: ChipAnimationStyle(
                    enableAnimation: const AnimationStyle(
                      duration: Duration(seconds: 3),
                      reverseDuration: Duration(seconds: 1),
                    ),
                  ),
                  onSelected: !enabled ? null : (bool value) {},
                  disabledColor: Colors.red.withOpacity(0.12),
                  backgroundColor: Colors.amber,
                  label: Text(enabled ? 'Enabled' : 'Disabled'),
                ),
                const SizedBox(height: 16),
                ElevatedButton(
                  onPressed: () {
                    setState(() {
                      enabled = !enabled;
                    });
                  },
                  child: Text(enabled ? 'Disable' : 'Enable'),
                ),
              ],
            ),
            Column(
              mainAxisSize: MainAxisSize.min,
              children: <Widget>[
                FilterChip.elevated(
                  chipAnimationStyle: ChipAnimationStyle(
                    selectAnimation: const AnimationStyle(
                      duration: Duration(seconds: 3),
                      reverseDuration: Duration(seconds: 1),
                    ),
                  ),
                  backgroundColor: Colors.amber,
                  selectedColor: Colors.blue,
                  selected: selected,
                  showCheckmark: false,
                  onSelected: (bool value) {},
                  label: Text(selected ? 'Selected' : 'Unselected'),
                ),
                const SizedBox(height: 16),
                ElevatedButton(
                  onPressed: () {
                    setState(() {
                      selected = !selected;
                    });
                  },
                  child: Text(selected ? 'Unselect' : 'Select'),
                ),
              ],
            ),
          ],
        ),
        Row(
          mainAxisAlignment: MainAxisAlignment.spaceEvenly,
          children: <Widget>[
            Column(
              mainAxisSize: MainAxisSize.min,
              children: <Widget>[
                FilterChip.elevated(
                  chipAnimationStyle: ChipAnimationStyle(
                    avatarDrawerAnimation: const AnimationStyle(
                      duration: Duration(seconds: 2),
                      reverseDuration: Duration(seconds: 1),
                    ),
                  ),
                  selected: showCheckmark,
                  onSelected: (bool value) {},
                  label: Text(showCheckmark ? 'Checked' : 'Unchecked'),
                ),
                const SizedBox(height: 16),
                ElevatedButton(
                  onPressed: () {
                    setState(() {
                      showCheckmark = !showCheckmark;
                    });
                  },
                  child: Text(showCheckmark ? 'Hide checkmark' : 'Show checkmark'),
                ),
              ],
            ),
            Column(
              mainAxisSize: MainAxisSize.min,
              children: <Widget>[
                FilterChip.elevated(
                  chipAnimationStyle: ChipAnimationStyle(
                    deleteDrawerAnimation: const AnimationStyle(
                      duration: Duration(seconds: 2),
                      reverseDuration: Duration(seconds: 1),
                    ),
                  ),
                  onDeleted: showDeleteIcon ? () {} : null,
                  onSelected: (bool value) {},
                  label: Text(showDeleteIcon ? 'Deletable' : 'Undeletable'),
                ),
                const SizedBox(height: 16),
                ElevatedButton(
                  onPressed: () {
                    setState(() {
                      showDeleteIcon = !showDeleteIcon;
                    });
                  },
                  child: Text(showDeleteIcon ? 'Hide delete icon' : 'Show delete icon'),
                ),
              ],
            ),
          ],
        ),
      ],
    );
  }
}
