// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [SliverAppBar.large].

void main() {
  runApp(const AppBarLargeApp());
}

class AppBarLargeApp extends StatelessWidget {
  const AppBarLargeApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData(useMaterial3: true, colorSchemeSeed: const Color(0xff6750A4)),
      home: Material(
        child: CustomScrollView(
          slivers: <Widget>[
            SliverAppBar.large(
              leading: IconButton(icon: const Icon(Icons.menu), onPressed: () {}),
              title: const Text('Large App Bar'),
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
