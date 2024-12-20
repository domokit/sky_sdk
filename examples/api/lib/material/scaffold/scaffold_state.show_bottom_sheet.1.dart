// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [ScaffoldState.showBottomSheet].

void main() => runApp(const ShowBottomSheetExampleApp());

class ShowBottomSheetExampleApp extends StatelessWidget {
  const ShowBottomSheetExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('ScaffoldState BottomSheet Sample')),
        body: const ShowBottomSheetExample(),
      ),
    );
  }
}

enum AnimationStyles { defaultStyle, custom, none }

const List<(AnimationStyles, String)> animationStyleSegments = <(AnimationStyles, String)>[
  (AnimationStyles.defaultStyle, 'Default'),
  (AnimationStyles.custom, 'Custom'),
  (AnimationStyles.none, 'None'),
];

class ShowBottomSheetExample extends StatefulWidget {
  const ShowBottomSheetExample({super.key});

  @override
  State<ShowBottomSheetExample> createState() => _ShowBottomSheetExampleState();
}

class _ShowBottomSheetExampleState extends State<ShowBottomSheetExample> {
  Set<AnimationStyles> _animationStyleSelection = <AnimationStyles>{AnimationStyles.defaultStyle};
  AnimationStyle? _animationStyle;

  @override
  Widget build(BuildContext context) {
    return Center(
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
          ElevatedButton(
            child: const Text('showBottomSheet'),
            onPressed: () {
              Scaffold.of(context).showBottomSheet(sheetAnimationStyle: _animationStyle, (
                BuildContext context,
              ) {
                return SizedBox(
                  height: 200,
                  child: Center(
                    child: Column(
                      mainAxisAlignment: MainAxisAlignment.center,
                      mainAxisSize: MainAxisSize.min,
                      children: <Widget>[
                        const Text('BottomSheet'),
                        ElevatedButton(
                          child: const Text('Close'),
                          onPressed: () {
                            Navigator.pop(context);
                          },
                        ),
                      ],
                    ),
                  ),
                );
              });
            },
          ),
        ],
      ),
    );
  }
}
