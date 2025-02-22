// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [ExpansionTile] and [ExpansionTileController].

void main() {
  runApp(const ExpansionTileControllerApp());
}

class ExpansionTileControllerApp extends StatefulWidget {
  const ExpansionTileControllerApp({super.key});

  @override
  State<ExpansionTileControllerApp> createState() => _ExpansionTileControllerAppState();
}

class _ExpansionTileControllerAppState extends State<ExpansionTileControllerApp> {
  final ExpansionTileController controller = ExpansionTileController();

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('ExpansionTileController Sample')),
        body: Column(
          children: <Widget>[
            // A controller has been provided to the ExpansionTile because it's
            // going to be accessed from a component that is not within the
            // tile's BuildContext.
            ExpansionTile(
              controller: controller,
              title: const Text('ExpansionTile with explicit controller.'),
              children: <Widget>[
                Container(
                  alignment: Alignment.center,
                  padding: const EdgeInsets.all(24),
                  child: const Text('ExpansionTile Contents'),
                ),
              ],
            ),
            const SizedBox(height: 8),
            ElevatedButton(
              child: const Text('Expand/Collapse the Tile Above'),
              onPressed: () {
                if (controller.isExpanded) {
                  controller.collapse();
                } else {
                  controller.expand();
                }
              },
            ),
            const SizedBox(height: 48),
            // A controller has not been provided to the ExpansionTile because
            // the automatically created one can be retrieved via the tile's BuildContext.
            ExpansionTile(
              title: const Text('ExpansionTile with implicit controller.'),
              children: <Widget>[
                Builder(
                  builder: (BuildContext context) {
                    return Container(
                      padding: const EdgeInsets.all(24),
                      alignment: Alignment.center,
                      child: ElevatedButton(
                        child: const Text('Collapse This Tile'),
                        onPressed: () {
                          return ExpansionTileController.of(context).collapse();
                        },
                      ),
                    );
                  },
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }
}
