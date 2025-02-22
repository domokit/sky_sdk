// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [SliverAppBar.medium].

void main() {
  runApp(const AppBarMediumApp());
}

class AppBarMediumApp extends StatelessWidget {
  const AppBarMediumApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData(colorSchemeSeed: const Color(0xff6750A4)),
      home: Material(
        child: CustomScrollView(
          slivers: <Widget>[
            SliverAppBar.medium(
              leading: IconButton(icon: const Icon(Icons.menu), onPressed: () {}),
              title: const Text('Medium App Bar'),
              actions: <Widget>[IconButton(icon: const Icon(Icons.more_vert), onPressed: () {})],
            ),
            // Just some content big enough to have something to scroll.
            SliverToBoxAdapter(
              child: Card(
                child: SizedBox(
                  height: 1200,
                  child: Padding(
                    padding: const EdgeInsets.fromLTRB(8, 100, 8, 100),
                    child: Text(
                      'Here be scrolling content...',
                      style: Theme.of(context).textTheme.headlineSmall,
                    ),
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
