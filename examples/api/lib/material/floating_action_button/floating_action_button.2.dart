// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [FloatingActionButton].

void main() => runApp(const FloatingActionButtonExampleApp());

class FloatingActionButtonExampleApp extends StatelessWidget {
  const FloatingActionButtonExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData(useMaterial3: true),
      home: const FloatingActionButtonExample(),
    );
  }
}

class FloatingActionButtonExample extends StatelessWidget {
  const FloatingActionButtonExample({super.key});

  @override
  Widget build(BuildContext context) {
    final ColorScheme colorScheme = Theme.of(context).colorScheme;

    Widget titleBox(String title) {
      return DecoratedBox(
        decoration: BoxDecoration(
          color: colorScheme.inverseSurface,
          borderRadius: BorderRadius.circular(4),
        ),
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
          child: Text(title, style: TextStyle(color: colorScheme.onInverseSurface)),
        ),
      );
    }

    return Scaffold(
      appBar: AppBar(title: const Text('FAB Additional Color Mappings')),
      body: Center(
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceEvenly,
          children: <Widget>[
            // Surface color mapping.
            Column(
              mainAxisSize: MainAxisSize.min,
              children: <Widget>[
                FloatingActionButton.large(
                  foregroundColor: colorScheme.primary,
                  backgroundColor: colorScheme.surface,
                  onPressed: () {
                    // Add your onPressed code here!
                  },
                  child: const Icon(Icons.edit_outlined),
                ),
                const SizedBox(height: 20),
                titleBox('Surface'),
              ],
            ),
            // Secondary color mapping.
            Column(
              mainAxisSize: MainAxisSize.min,
              children: <Widget>[
                FloatingActionButton.large(
                  foregroundColor: colorScheme.onSecondaryContainer,
                  backgroundColor: colorScheme.secondaryContainer,
                  onPressed: () {
                    // Add your onPressed code here!
                  },
                  child: const Icon(Icons.edit_outlined),
                ),
                const SizedBox(height: 20),
                titleBox('Secondary'),
              ],
            ),
            // Tertiary color mapping.
            Column(
              mainAxisSize: MainAxisSize.min,
              children: <Widget>[
                FloatingActionButton.large(
                  foregroundColor: colorScheme.onTertiaryContainer,
                  backgroundColor: colorScheme.tertiaryContainer,
                  onPressed: () {
                    // Add your onPressed code here!
                  },
                  child: const Icon(Icons.edit_outlined),
                ),
                const SizedBox(height: 20),
                titleBox('Tertiary'),
              ],
            ),
          ],
        ),
      ),
    );
  }
}
