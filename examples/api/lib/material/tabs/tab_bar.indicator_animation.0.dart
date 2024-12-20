// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [TabBar.indicatorAnimation].

void main() => runApp(const IndicatorAnimationExampleApp());

class IndicatorAnimationExampleApp extends StatelessWidget {
  const IndicatorAnimationExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: IndicatorAnimationExample());
  }
}

const List<(TabIndicatorAnimation, String)> indicatorAnimationSegments =
    <(TabIndicatorAnimation, String)>[
      (TabIndicatorAnimation.linear, 'Linear'),
      (TabIndicatorAnimation.elastic, 'Elastic'),
    ];

class IndicatorAnimationExample extends StatefulWidget {
  const IndicatorAnimationExample({super.key});

  @override
  State<IndicatorAnimationExample> createState() => _IndicatorAnimationExampleState();
}

class _IndicatorAnimationExampleState extends State<IndicatorAnimationExample> {
  Set<TabIndicatorAnimation> _animationStyleSelection = <TabIndicatorAnimation>{
    TabIndicatorAnimation.linear,
  };
  TabIndicatorAnimation _tabIndicatorAnimation = TabIndicatorAnimation.linear;

  @override
  Widget build(BuildContext context) {
    return DefaultTabController(
      length: 6,
      child: Scaffold(
        appBar: AppBar(
          title: const Text('Indicator Animation Example'),
          bottom: TabBar(
            indicatorAnimation: _tabIndicatorAnimation,
            isScrollable: true,
            tabAlignment: TabAlignment.start,
            tabs: const <Widget>[
              Tab(text: 'Short Tab'),
              Tab(text: 'Very Very Very Long Tab'),
              Tab(text: 'Short Tab'),
              Tab(text: 'Very Very Very Long Tab'),
              Tab(text: 'Short Tab'),
              Tab(text: 'Very Very Very Long Tab'),
            ],
          ),
        ),
        body: Column(
          children: <Widget>[
            const SizedBox(height: 16),
            SegmentedButton<TabIndicatorAnimation>(
              selected: _animationStyleSelection,
              onSelectionChanged: (Set<TabIndicatorAnimation> styles) {
                setState(() {
                  _animationStyleSelection = styles;
                  _tabIndicatorAnimation = styles.first;
                });
              },
              segments:
                  indicatorAnimationSegments.map<ButtonSegment<TabIndicatorAnimation>>((
                    (TabIndicatorAnimation, String) shirt,
                  ) {
                    return ButtonSegment<TabIndicatorAnimation>(
                      value: shirt.$1,
                      label: Text(shirt.$2),
                    );
                  }).toList(),
            ),
            const SizedBox(height: 16),
            const Expanded(
              child: TabBarView(
                children: <Widget>[
                  Center(child: Text('Short Tab Page')),
                  Center(child: Text('Very Very Very Long Tab Page')),
                  Center(child: Text('Short Tab Page')),
                  Center(child: Text('Very Very Very Long Tab Page')),
                  Center(child: Text('Short Tab Page')),
                  Center(child: Text('Very Very Very Long Tab Page')),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }
}
