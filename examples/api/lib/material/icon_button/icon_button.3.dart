// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [IconButton] with toggle feature.

void main() {
  runApp(const IconButtonToggleApp());
}

class IconButtonToggleApp extends StatelessWidget {
  const IconButtonToggleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData(colorSchemeSeed: const Color(0xff6750a4), useMaterial3: true),
      title: 'Icon Button Types',
      home: const Scaffold(body: DemoIconToggleButtons()),
    );
  }
}

class DemoIconToggleButtons extends StatefulWidget {
  const DemoIconToggleButtons({super.key});

  @override
  State<DemoIconToggleButtons> createState() => _DemoIconToggleButtonsState();
}

class _DemoIconToggleButtonsState extends State<DemoIconToggleButtons> {
  bool standardSelected = false;
  bool filledSelected = false;
  bool tonalSelected = false;
  bool outlinedSelected = false;

  @override
  Widget build(BuildContext context) {
    return Column(
      mainAxisAlignment: MainAxisAlignment.spaceEvenly,
      children: <Widget>[
        Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            IconButton(
              isSelected: standardSelected,
              icon: const Icon(Icons.settings_outlined),
              selectedIcon: const Icon(Icons.settings),
              onPressed: () {
                setState(() {
                  standardSelected = !standardSelected;
                });
              },
            ),
            const SizedBox(width: 10),
            IconButton(
              isSelected: standardSelected,
              icon: const Icon(Icons.settings_outlined),
              selectedIcon: const Icon(Icons.settings),
              onPressed: null,
            ),
          ],
        ),
        Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            IconButton.filled(
              isSelected: filledSelected,
              icon: const Icon(Icons.settings_outlined),
              selectedIcon: const Icon(Icons.settings),
              onPressed: () {
                setState(() {
                  filledSelected = !filledSelected;
                });
              },
            ),
            const SizedBox(width: 10),
            IconButton.filled(
              isSelected: filledSelected,
              icon: const Icon(Icons.settings_outlined),
              selectedIcon: const Icon(Icons.settings),
              onPressed: null,
            ),
          ],
        ),
        Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            IconButton.filledTonal(
              isSelected: tonalSelected,
              icon: const Icon(Icons.settings_outlined),
              selectedIcon: const Icon(Icons.settings),
              onPressed: () {
                setState(() {
                  tonalSelected = !tonalSelected;
                });
              },
            ),
            const SizedBox(width: 10),
            IconButton.filledTonal(
              isSelected: tonalSelected,
              icon: const Icon(Icons.settings_outlined),
              selectedIcon: const Icon(Icons.settings),
              onPressed: null,
            ),
          ],
        ),
        Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            IconButton.outlined(
              isSelected: outlinedSelected,
              icon: const Icon(Icons.settings_outlined),
              selectedIcon: const Icon(Icons.settings),
              onPressed: () {
                setState(() {
                  outlinedSelected = !outlinedSelected;
                });
              },
            ),
            const SizedBox(width: 10),
            IconButton.outlined(
              isSelected: outlinedSelected,
              icon: const Icon(Icons.settings_outlined),
              selectedIcon: const Icon(Icons.settings),
              onPressed: null,
            ),
          ],
        ),
      ],
    );
  }
}
