// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

/// Flutter code sample for [CupertinoTabBar].

void main() => runApp(const CupertinoTabBarApp());

class CupertinoTabBarApp extends StatelessWidget {
  const CupertinoTabBarApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const CupertinoApp(
      theme: CupertinoThemeData(brightness: Brightness.light),
      home: CupertinoTabBarExample(),
    );
  }
}

class CupertinoTabBarExample extends StatelessWidget {
  const CupertinoTabBarExample({super.key});

  @override
  Widget build(BuildContext context) {
    return CupertinoTabScaffold(
      tabBar: CupertinoTabBar(
        items: const <BottomNavigationBarItem>[
          BottomNavigationBarItem(icon: Icon(CupertinoIcons.star_fill), label: 'Favorites'),
          BottomNavigationBarItem(icon: Icon(CupertinoIcons.clock_solid), label: 'Recents'),
          BottomNavigationBarItem(
            icon: Icon(CupertinoIcons.person_alt_circle_fill),
            label: 'Contacts',
          ),
          BottomNavigationBarItem(icon: Icon(CupertinoIcons.circle_grid_3x3_fill), label: 'Keypad'),
        ],
      ),
      tabBuilder: (BuildContext context, int index) {
        return CupertinoTabView(
          builder: (BuildContext context) {
            return Center(child: Text('Content of tab $index'));
          },
        );
      },
    );
  }
}
