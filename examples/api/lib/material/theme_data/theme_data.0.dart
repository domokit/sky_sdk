// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

void main() {
  runApp(const ThemeDataExampleApp());
}

// This app's theme specifies an overall ColorScheme as well as overrides
// for the default configuration of FloatingActionButtons. To customize
// the appearance of other components, add additional component specific
// themes, rather than tweaking the color scheme.
//
// Creating an entire color scheme from a single seed color is a good
// way to ensure a visually appealing color palette where the default
// component colors have sufficient contrast for accessibility. Another
// good way to create an app's color scheme is to use
// ColorScheme.fromImageProvider.
//
// The color scheme reflects the platform's light or dark setting
// which is retrieved with `MediaQuery.platformBrightnessOf`. The color
// scheme's colors will be different for light and dark settings although
// they'll all be related to the seed color in both cases.
//
// Color scheme colors have been used where component defaults have
// been overridden so that the app will look good and remain accessible
// in both light and dark modes.
//
// Text styles are derived from the theme's textTheme (not the obsolete
// primaryTextTheme property) and then customized using copyWith.
// Using the _on_ version of a color scheme color as the foreground,
// as in `tertiary` and `onTertiary`, guarantees sufficient contrast
// for readability/accessibility.

class ThemeDataExampleApp extends StatelessWidget {
  const ThemeDataExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    final ColorScheme colorScheme = ColorScheme.fromSeed(
      brightness: MediaQuery.platformBrightnessOf(context),
      seedColor: Colors.indigo,
    );
    return MaterialApp(
      title: 'ThemeData Demo',
      theme: ThemeData(
        colorScheme: colorScheme,
        floatingActionButtonTheme: FloatingActionButtonThemeData(
          backgroundColor: colorScheme.tertiary,
          foregroundColor: colorScheme.onTertiary,
        ),
      ),
      home: const Home(),
    );
  }
}

class Home extends StatefulWidget {
  const Home({super.key});

  @override
  State<Home> createState() => _HomeState();
}

class _HomeState extends State<Home> {
  int buttonPressCount = 0;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final ColorScheme colorScheme = theme.colorScheme;
    final double pointCount = 8 + (buttonPressCount % 6);

    return Scaffold(
      appBar: AppBar(title: const Text('Press the + Button')),
      // An AnimatedContainer makes the decoration changes entertaining.
      body: AnimatedContainer(
        duration: const Duration(milliseconds: 500),
        margin: const EdgeInsets.all(32),
        alignment: Alignment.center,
        decoration: ShapeDecoration(
          color: colorScheme.tertiaryContainer,
          shape: StarBorder(
            points: pointCount,
            pointRounding: 0.4,
            valleyRounding: 0.6,
            side: BorderSide(width: 9, color: colorScheme.tertiary),
          ),
        ),
        child: Text(
          '${pointCount.toInt()} Points',
          style: theme.textTheme.headlineMedium!.copyWith(color: colorScheme.onPrimaryContainer),
        ),
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: () {
          setState(() {
            buttonPressCount += 1;
          });
        },
        tooltip: "Change the shape's point count",
        child: const Icon(Icons.add),
      ),
    );
  }
}
