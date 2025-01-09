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
    return Scaffold(
      appBar: AppBar(title: const Text('FloatingActionButton Sample')),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.spaceEvenly,
          children: <Widget>[
            Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: <Widget>[
                const Text('Small'),
                const SizedBox(width: 16),
                // An example of the small floating action button.
                //
                // https://m3.material.io/components/floating-action-button/specs#669a1be8-7271-48cb-a74d-dd502d73bda4
                FloatingActionButton.small(
                  onPressed: () {
                    // Add your onPressed code here!
                  },
                  child: const Icon(Icons.add),
                ),
              ],
            ),
            Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: <Widget>[
                const Text('Regular'),
                const SizedBox(width: 16),
                // An example of the regular floating action button.
                //
                // https://m3.material.io/components/floating-action-button/specs#71504201-7bd1-423d-8bb7-07e0291743e5
                FloatingActionButton(
                  onPressed: () {
                    // Add your onPressed code here!
                  },
                  child: const Icon(Icons.add),
                ),
              ],
            ),
            Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: <Widget>[
                const Text('Large'),
                const SizedBox(width: 16),
                // An example of the large floating action button.
                //
                // https://m3.material.io/components/floating-action-button/specs#9d7d3d6a-bab7-47cb-be32-5596fbd660fe
                FloatingActionButton.large(
                  onPressed: () {
                    // Add your onPressed code here!
                  },
                  child: const Icon(Icons.add),
                ),
              ],
            ),
            Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: <Widget>[
                const Text('Extended'),
                const SizedBox(width: 16),
                // An example of the extended floating action button.
                //
                // https://m3.material.io/components/extended-fab/specs#686cb8af-87c9-48e8-a3e1-db9da6f6c69b
                FloatingActionButton.extended(
                  onPressed: () {
                    // Add your onPressed code here!
                  },
                  label: const Text('Add'),
                  icon: const Icon(Icons.add),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }
}
