// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

/// Flutter code sample for [CupertinoNavigationBar] showing a
/// [CupertinoSearchTextField] with padding at the bottom of the navigation bar.

void main() => runApp(const NavBarApp());

class NavBarApp extends StatelessWidget {
  const NavBarApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const CupertinoApp(
      theme: CupertinoThemeData(brightness: Brightness.light),
      home: NavBarExample(),
    );
  }
}

class NavBarExample extends StatefulWidget {
  const NavBarExample({super.key});

  @override
  State<NavBarExample> createState() => _NavBarExampleState();
}

class _NavBarExampleState extends State<NavBarExample> {
  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: const CupertinoNavigationBar(
        middle: Text('CupertinoNavigationBar Sample'),
        bottom: _NavigationBarSearchField(),
        automaticBackgroundVisibility: false,
      ),
      child: Column(
        children: <Widget>[
          Container(height: 50, color: CupertinoColors.systemRed),
          Container(height: 50, color: CupertinoColors.systemGreen),
          Container(height: 50, color: CupertinoColors.systemBlue),
          Container(height: 50, color: CupertinoColors.systemYellow),
        ],
      ),
    );
  }
}

class _NavigationBarSearchField extends StatelessWidget implements PreferredSizeWidget {
  const _NavigationBarSearchField();

  static const double padding = 8.0;
  static const double searchFieldHeight = 35.0;

  @override
  Widget build(BuildContext context) {
    return const Padding(
      padding: EdgeInsets.symmetric(horizontal: padding, vertical: padding),
      child: SizedBox(height: searchFieldHeight, child: CupertinoSearchTextField()),
    );
  }

  @override
  Size get preferredSize => const Size.fromHeight(searchFieldHeight + padding * 2);
}
