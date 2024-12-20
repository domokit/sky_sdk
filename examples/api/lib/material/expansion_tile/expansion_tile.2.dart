// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [ExpansionTile] and [AnimationStyle].

void main() {
  runApp(const ExpansionTileAnimationStyleApp());
}

enum AnimationStyles { defaultStyle, custom, none }

const List<(AnimationStyles, String)> animationStyleSegments = <(AnimationStyles, String)>[
  (AnimationStyles.defaultStyle, 'Default'),
  (AnimationStyles.custom, 'Custom'),
  (AnimationStyles.none, 'None'),
];

class ExpansionTileAnimationStyleApp extends StatefulWidget {
  const ExpansionTileAnimationStyleApp({super.key});

  @override
  State<ExpansionTileAnimationStyleApp> createState() => _ExpansionTileAnimationStyleAppState();
}

class _ExpansionTileAnimationStyleAppState extends State<ExpansionTileAnimationStyleApp> {
  Set<AnimationStyles> _animationStyleSelection = <AnimationStyles>{AnimationStyles.defaultStyle};
  AnimationStyle? _animationStyle;

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        body: SafeArea(
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[
              SegmentedButton<AnimationStyles>(
                selected: _animationStyleSelection,
                onSelectionChanged: (Set<AnimationStyles> styles) {
                  setState(() {
                    _animationStyleSelection = styles;
                    switch (styles.first) {
                      case AnimationStyles.defaultStyle:
                        _animationStyle = null;
                      case AnimationStyles.custom:
                        _animationStyle = AnimationStyle(
                          curve: Easing.emphasizedAccelerate,
                          duration: Durations.extralong1,
                        );
                      case AnimationStyles.none:
                        _animationStyle = AnimationStyle.noAnimation;
                    }
                  });
                },
                segments:
                    animationStyleSegments.map<ButtonSegment<AnimationStyles>>((
                      (AnimationStyles, String) shirt,
                    ) {
                      return ButtonSegment<AnimationStyles>(value: shirt.$1, label: Text(shirt.$2));
                    }).toList(),
              ),
              const SizedBox(height: 20),
              ExpansionTile(
                expansionAnimationStyle: _animationStyle,
                title: const Text('ExpansionTile'),
                children: const <Widget>[
                  ListTile(title: Text('Expanded Item 1')),
                  ListTile(title: Text('Expanded Item 2')),
                ],
              ),
            ],
          ),
        ),
      ),
    );
  }
}
