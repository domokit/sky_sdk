// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

class PictureCacheComplexityScoringPage extends StatelessWidget {
  const PictureCacheComplexityScoringPage({super.key});

  static const List<String> kTabNames = <String>['1', '2'];

  @override
  Widget build(BuildContext context) {
    return DefaultTabController(
      length: kTabNames.length, // This is the number of tabs.
      child: Scaffold(
        appBar: AppBar(
          title: const Text('Picture Cache Complexity Scoring'),
          // pinned: true,
          // expandedHeight: 50.0,
          // forceElevated: innerBoxIsScrolled,
          bottom: TabBar(tabs: kTabNames.map((String name) => Tab(text: name)).toList()),
        ),
        body: TabBarView(
          key: const Key('tabbar_view_complexity'), // this key is used by the driver test
          children:
              kTabNames.map((String name) {
                return _buildComplexityScoringWidgets(name);
              }).toList(),
        ),
      ),
    );
  }

  // For now we just test a single case where the widget being cached is actually
  // relatively cheap to rasterize, and so should not be in the cache.
  //
  // Eventually we can extend this to add new test cases based on the tab name.
  Widget _buildComplexityScoringWidgets(String name) {
    return Column(
      children: <Widget>[
        Slider(value: 50, label: 'Slider 1', onChanged: (double _) {}, max: 100, divisions: 10),
        Slider(value: 50, label: 'Slider 2', onChanged: (double _) {}, max: 100, divisions: 10),
        Slider(value: 50, label: 'Slider 3', onChanged: (double _) {}, max: 100, divisions: 10),
      ],
    );
  }
}
