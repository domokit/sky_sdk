// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

/// Flutter code sample for [CupertinoNavigationBar.large].

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
  int _count = 0;

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: const CupertinoNavigationBar.large(largeTitle: Text('Large Sample')),
      child: SafeArea(
        child: Center(
          child: Column(
            children: <Widget>[
              const Spacer(),
              const Text('You have pushed the button this many times:'),
              Text('$_count', style: CupertinoTheme.of(context).textTheme.navLargeTitleTextStyle),
              const Spacer(),
              Padding(
                padding: const EdgeInsets.all(15.0),
                child: CupertinoButton.filled(
                  onPressed: () => setState(() => _count++),
                  child: const Text('Increment'),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
