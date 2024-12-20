// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [FontFeature].

void main() => runApp(const ExampleApp());

class ExampleApp extends StatelessWidget {
  const ExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: ExampleWidget());
  }
}

final TextStyle titleStyle = TextStyle(
  fontSize: 18,
  fontFeatures: const <FontFeature>[FontFeature.enable('smcp')],
  color: Colors.blueGrey[600],
);

class ExampleWidget extends StatelessWidget {
  const ExampleWidget({super.key});

  @override
  Widget build(BuildContext context) {
    // The Cardo, Milonga and Raleway Dots fonts can be downloaded from Google
    // Fonts (https://www.google.com/fonts).
    return Scaffold(
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            const Spacer(flex: 5),
            Text('regular numbers have their place:', style: titleStyle),
            const Text(
              'The 1972 cup final was a 1-1 draw.',
              style: TextStyle(fontFamily: 'Cardo', fontSize: 24),
            ),
            const Spacer(),
            Text('but old-style figures blend well with lower case:', style: titleStyle),
            const Text(
              'The 1972 cup final was a 1-1 draw.',
              style: TextStyle(
                fontFamily: 'Cardo',
                fontSize: 24,
                fontFeatures: <FontFeature>[FontFeature.oldstyleFigures()],
              ),
            ),
            const Spacer(),
            const Divider(),
            const Spacer(),
            Text('fractions look better with a custom ligature:', style: titleStyle),
            const Text(
              'Add 1/2 tsp of flour and stir.',
              style: TextStyle(
                fontFamily: 'Milonga',
                fontSize: 24,
                fontFeatures: <FontFeature>[FontFeature.alternativeFractions()],
              ),
            ),
            const Spacer(),
            const Divider(),
            const Spacer(),
            Text('multiple stylistic sets in one font:', style: titleStyle),
            const Text('Raleway Dots', style: TextStyle(fontFamily: 'Raleway Dots', fontSize: 48)),
            Text(
              'Raleway Dots',
              style: TextStyle(
                fontFeatures: <FontFeature>[FontFeature.stylisticSet(1)],
                fontFamily: 'Raleway Dots',
                fontSize: 48,
              ),
            ),
            const Spacer(flex: 5),
          ],
        ),
      ),
    );
  }
}
