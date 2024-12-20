// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [NestedScrollView].

void main() => runApp(const NestedScrollViewExampleApp());

class NestedScrollViewExampleApp extends StatelessWidget {
  const NestedScrollViewExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: NestedScrollViewExample());
  }
}

class NestedScrollViewExample extends StatelessWidget {
  const NestedScrollViewExample({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: NestedScrollView(
        headerSliverBuilder: (BuildContext context, bool innerBoxIsScrolled) {
          return <Widget>[
            SliverOverlapAbsorber(
              handle: NestedScrollView.sliverOverlapAbsorberHandleFor(context),
              sliver: SliverAppBar(
                title: const Text('Snapping Nested SliverAppBar'),
                floating: true,
                snap: true,
                expandedHeight: 200.0,
                forceElevated: innerBoxIsScrolled,
              ),
            ),
          ];
        },
        body: Builder(
          builder: (BuildContext context) {
            return CustomScrollView(
              // The "controller" and "primary" members should be left unset, so that
              // the NestedScrollView can control this inner scroll view.
              // If the "controller" property is set, then this scroll view will not
              // be associated with the NestedScrollView.
              slivers: <Widget>[
                SliverOverlapInjector(
                  handle: NestedScrollView.sliverOverlapAbsorberHandleFor(context),
                ),
                SliverFixedExtentList(
                  itemExtent: 48.0,
                  delegate: SliverChildBuilderDelegate(
                    (BuildContext context, int index) => ListTile(title: Text('Item $index')),
                    childCount: 30,
                  ),
                ),
              ],
            );
          },
        ),
      ),
    );
  }
}
