// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [SnackBar].

void main() => runApp(const SnackBarApp());

class SnackBarApp extends StatelessWidget {
  const SnackBarApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: SnackBarExample());
  }
}

enum AnimationStyles { defaultStyle, custom, none }

const List<(AnimationStyles, String)> animationStyleSegments = <(AnimationStyles, String)>[
  (AnimationStyles.defaultStyle, 'Default'),
  (AnimationStyles.custom, 'Custom'),
  (AnimationStyles.none, 'None'),
];

class SnackBarExample extends StatefulWidget {
  const SnackBarExample({super.key});

  @override
  State<SnackBarExample> createState() => _SnackBarExampleState();
}

class _SnackBarExampleState extends State<SnackBarExample> {
  Set<AnimationStyles> _animationStyleSelection = <AnimationStyles>{AnimationStyles.defaultStyle};
  AnimationStyle? _animationStyle;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('SnackBar Sample')),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            SegmentedButton<AnimationStyles>(
              selected: _animationStyleSelection,
              onSelectionChanged: (Set<AnimationStyles> styles) {
                setState(() {
                  _animationStyle = switch (styles.first) {
                    AnimationStyles.defaultStyle => null,
                    AnimationStyles.custom => AnimationStyle(
                      duration: const Duration(seconds: 3),
                      reverseDuration: const Duration(seconds: 1),
                    ),
                    AnimationStyles.none => AnimationStyle.noAnimation,
                  };
                  _animationStyleSelection = styles;
                });
              },
              segments:
                  animationStyleSegments.map<ButtonSegment<AnimationStyles>>((
                    (AnimationStyles, String) shirt,
                  ) {
                    return ButtonSegment<AnimationStyles>(value: shirt.$1, label: Text(shirt.$2));
                  }).toList(),
            ),
            const SizedBox(height: 10),
            Builder(
              builder: (BuildContext context) {
                return ElevatedButton(
                  onPressed: () {
                    ScaffoldMessenger.of(context).showSnackBar(
                      const SnackBar(content: Text('I am a snack bar.'), showCloseIcon: true),
                      snackBarAnimationStyle: _animationStyle,
                    );
                  },
                  child: const Text('Show SnackBar'),
                );
              },
            ),
          ],
        ),
      ),
    );
  }
}
