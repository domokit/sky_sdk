// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [MaterialApp].

void main() {
  runApp(const MaterialAppExample());
}

enum AnimationStyles { defaultStyle, custom, none }

const List<(AnimationStyles, String)> animationStyleSegments = <(AnimationStyles, String)>[
  (AnimationStyles.defaultStyle, 'Default'),
  (AnimationStyles.custom, 'Custom'),
  (AnimationStyles.none, 'None'),
];

class MaterialAppExample extends StatefulWidget {
  const MaterialAppExample({super.key});

  @override
  State<MaterialAppExample> createState() => _MaterialAppExampleState();
}

class _MaterialAppExampleState extends State<MaterialAppExample> {
  Set<AnimationStyles> _animationStyleSelection = <AnimationStyles>{AnimationStyles.defaultStyle};
  AnimationStyle? _animationStyle;
  bool isDarkTheme = false;

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      themeAnimationStyle: _animationStyle,
      themeMode: isDarkTheme ? ThemeMode.dark : ThemeMode.light,
      theme: ThemeData(colorSchemeSeed: Colors.green),
      darkTheme: ThemeData(colorSchemeSeed: Colors.green, brightness: Brightness.dark),
      home: Scaffold(
        body: Center(
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
                        _animationStyle = const AnimationStyle(
                          curve: Easing.emphasizedAccelerate,
                          duration: Duration(seconds: 1),
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
              const SizedBox(height: 10),
              OutlinedButton.icon(
                onPressed: () {
                  setState(() {
                    isDarkTheme = !isDarkTheme;
                  });
                },
                icon: Icon(isDarkTheme ? Icons.wb_sunny : Icons.nightlight_round),
                label: const Text('Switch Theme Mode'),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
