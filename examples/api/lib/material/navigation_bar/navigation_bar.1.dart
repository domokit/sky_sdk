// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [NavigationBar].

void main() => runApp(const NavigationBarApp());

class NavigationBarApp extends StatelessWidget {
  const NavigationBarApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: NavigationExample());
  }
}

class NavigationExample extends StatefulWidget {
  const NavigationExample({super.key});

  @override
  State<NavigationExample> createState() => _NavigationExampleState();
}

class _NavigationExampleState extends State<NavigationExample> {
  int currentPageIndex = 0;
  NavigationDestinationLabelBehavior labelBehavior = NavigationDestinationLabelBehavior.alwaysShow;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      bottomNavigationBar: NavigationBar(
        labelBehavior: labelBehavior,
        selectedIndex: currentPageIndex,
        onDestinationSelected: (int index) {
          setState(() {
            currentPageIndex = index;
          });
        },
        destinations: const <Widget>[
          NavigationDestination(icon: Icon(Icons.explore), label: 'Explore'),
          NavigationDestination(icon: Icon(Icons.commute), label: 'Commute'),
          NavigationDestination(
            selectedIcon: Icon(Icons.bookmark),
            icon: Icon(Icons.bookmark_border),
            label: 'Saved',
          ),
        ],
      ),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            Text('Label behavior: ${labelBehavior.name}'),
            const SizedBox(height: 10),
            OverflowBar(
              spacing: 10.0,
              overflowAlignment: OverflowBarAlignment.center,
              overflowSpacing: 10.0,
              children: <Widget>[
                ElevatedButton(
                  onPressed: () {
                    setState(() {
                      labelBehavior = NavigationDestinationLabelBehavior.alwaysShow;
                    });
                  },
                  child: const Text('alwaysShow'),
                ),
                ElevatedButton(
                  onPressed: () {
                    setState(() {
                      labelBehavior = NavigationDestinationLabelBehavior.onlyShowSelected;
                    });
                  },
                  child: const Text('onlyShowSelected'),
                ),
                ElevatedButton(
                  onPressed: () {
                    setState(() {
                      labelBehavior = NavigationDestinationLabelBehavior.alwaysHide;
                    });
                  },
                  child: const Text('alwaysHide'),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }
}
