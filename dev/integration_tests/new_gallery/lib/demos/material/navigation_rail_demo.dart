// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import '../../gallery_localizations.dart';

// BEGIN navRailDemo

class NavRailDemo extends StatefulWidget {
  const NavRailDemo({super.key});

  @override
  State<NavRailDemo> createState() => _NavRailDemoState();
}

class _NavRailDemoState extends State<NavRailDemo> with RestorationMixin {
  final RestorableInt _selectedIndex = RestorableInt(0);

  @override
  String get restorationId => 'nav_rail_demo';

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_selectedIndex, 'selected_index');
  }

  @override
  void dispose() {
    _selectedIndex.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localization = GalleryLocalizations.of(context)!;
    final String destinationFirst = localization.demoNavigationRailFirst;
    final String destinationSecond = localization.demoNavigationRailSecond;
    final String destinationThird = localization.demoNavigationRailThird;
    final List<String> selectedItem = <String>[
      destinationFirst,
      destinationSecond,
      destinationThird,
    ];
    return Scaffold(
      appBar: AppBar(title: Text(localization.demoNavigationRailTitle)),
      body: Row(
        children: <Widget>[
          NavigationRail(
            leading: FloatingActionButton(
              onPressed: () {},
              tooltip: localization.buttonTextCreate,
              child: const Icon(Icons.add),
            ),
            selectedIndex: _selectedIndex.value,
            onDestinationSelected: (int index) {
              setState(() {
                _selectedIndex.value = index;
              });
            },
            labelType: NavigationRailLabelType.selected,
            destinations: <NavigationRailDestination>[
              NavigationRailDestination(
                icon: const Icon(Icons.favorite_border),
                selectedIcon: const Icon(Icons.favorite),
                label: Text(destinationFirst),
              ),
              NavigationRailDestination(
                icon: const Icon(Icons.bookmark_border),
                selectedIcon: const Icon(Icons.book),
                label: Text(destinationSecond),
              ),
              NavigationRailDestination(
                icon: const Icon(Icons.star_border),
                selectedIcon: const Icon(Icons.star),
                label: Text(destinationThird),
              ),
            ],
          ),
          const VerticalDivider(thickness: 1, width: 1),
          Expanded(child: Center(child: Text(selectedItem[_selectedIndex.value]))),
        ],
      ),
    );
  }
}

// END
