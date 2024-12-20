// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

/// Flutter code sample for [CupertinoSliverRefreshControl].

void main() => runApp(const RefreshControlApp());

class RefreshControlApp extends StatelessWidget {
  const RefreshControlApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const CupertinoApp(
      theme: CupertinoThemeData(brightness: Brightness.light),
      home: RefreshControlExample(),
    );
  }
}

class RefreshControlExample extends StatefulWidget {
  const RefreshControlExample({super.key});

  @override
  State<RefreshControlExample> createState() => _RefreshControlExampleState();
}

class _RefreshControlExampleState extends State<RefreshControlExample> {
  List<Color> colors = <Color>[
    CupertinoColors.systemYellow,
    CupertinoColors.systemOrange,
    CupertinoColors.systemPink,
  ];
  List<Widget> items = <Widget>[
    Container(color: CupertinoColors.systemPink, height: 100.0),
    Container(color: CupertinoColors.systemOrange, height: 100.0),
    Container(color: CupertinoColors.systemYellow, height: 100.0),
  ];

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: const CupertinoNavigationBar(
        middle: Text('CupertinoSliverRefreshControl Sample'),
      ),
      child: CustomScrollView(
        physics: const BouncingScrollPhysics(parent: AlwaysScrollableScrollPhysics()),
        slivers: <Widget>[
          const CupertinoSliverNavigationBar(largeTitle: Text('Scroll down')),
          CupertinoSliverRefreshControl(
            onRefresh: () async {
              await Future<void>.delayed(const Duration(milliseconds: 1000));
              setState(() {
                items.insert(0, Container(color: colors[items.length % 3], height: 100.0));
              });
            },
          ),
          SliverList(
            delegate: SliverChildBuilderDelegate(
              (BuildContext context, int index) => items[index],
              childCount: items.length,
            ),
          ),
        ],
      ),
    );
  }
}
