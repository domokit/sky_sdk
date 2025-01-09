// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

/// Flutter code sample for [CupertinoTabController].

void main() => runApp(const TabControllerApp());

class TabControllerApp extends StatelessWidget {
  const TabControllerApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const CupertinoApp(
      theme: CupertinoThemeData(brightness: Brightness.light),
      home: TabControllerExample(),
    );
  }
}

class TabControllerExample extends StatefulWidget {
  const TabControllerExample({super.key});

  @override
  State<TabControllerExample> createState() => _TabControllerExampleState();
}

class _TabControllerExampleState extends State<TabControllerExample> {
  final CupertinoTabController controller = CupertinoTabController();

  @override
  void dispose() {
    controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return CupertinoTabScaffold(
      controller: controller,
      tabBar: CupertinoTabBar(
        items: const <BottomNavigationBarItem>[
          BottomNavigationBarItem(icon: Icon(CupertinoIcons.square_grid_2x2_fill), label: 'Browse'),
          BottomNavigationBarItem(icon: Icon(CupertinoIcons.star_circle_fill), label: 'Starred'),
        ],
      ),
      tabBuilder: (BuildContext context, int index) {
        return Center(
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[
              Text('Content of tab $index'),
              const SizedBox(height: 10),
              CupertinoButton(
                onPressed: () => controller.index = 0,
                child: const Text('Go to first tab'),
              ),
            ],
          ),
        );
      },
    );
  }
}
