// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import '../../gallery_localizations.dart';
import 'material_demo_types.dart';

class TabsDemo extends StatelessWidget {
  const TabsDemo({super.key, required this.type});

  final TabsDemoType type;

  @override
  Widget build(BuildContext context) {
    return switch (type) {
      TabsDemoType.scrollable => _TabsScrollableDemo(),
      TabsDemoType.nonScrollable => _TabsNonScrollableDemo(),
    };
  }
}

// BEGIN tabsScrollableDemo

class _TabsScrollableDemo extends StatefulWidget {
  @override
  __TabsScrollableDemoState createState() => __TabsScrollableDemoState();
}

class __TabsScrollableDemoState extends State<_TabsScrollableDemo>
    with SingleTickerProviderStateMixin, RestorationMixin {
  TabController? _tabController;

  final RestorableInt tabIndex = RestorableInt(0);

  @override
  String get restorationId => 'tab_scrollable_demo';

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(tabIndex, 'tab_index');
    _tabController!.index = tabIndex.value;
  }

  @override
  void initState() {
    _tabController = TabController(length: 12, vsync: this);
    _tabController!.addListener(() {
      // When the tab controller's value is updated, make sure to update the
      // tab index value, which is state restorable.
      setState(() {
        tabIndex.value = _tabController!.index;
      });
    });
    super.initState();
  }

  @override
  void dispose() {
    _tabController!.dispose();
    tabIndex.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    final List<String> tabs = <String>[
      localizations.colorsRed,
      localizations.colorsOrange,
      localizations.colorsGreen,
      localizations.colorsBlue,
      localizations.colorsIndigo,
      localizations.colorsPurple,
      localizations.colorsRed,
      localizations.colorsOrange,
      localizations.colorsGreen,
      localizations.colorsBlue,
      localizations.colorsIndigo,
      localizations.colorsPurple,
    ];

    return Scaffold(
      appBar: AppBar(
        automaticallyImplyLeading: false,
        title: Text(localizations.demoTabsScrollingTitle),
        bottom: TabBar(
          controller: _tabController,
          isScrollable: true,
          tabs: <Widget>[for (final String tab in tabs) Tab(text: tab)],
        ),
      ),
      body: TabBarView(
        controller: _tabController,
        children: <Widget>[for (final String tab in tabs) Center(child: Text(tab))],
      ),
    );
  }
}

// END

// BEGIN tabsNonScrollableDemo

class _TabsNonScrollableDemo extends StatefulWidget {
  @override
  __TabsNonScrollableDemoState createState() => __TabsNonScrollableDemoState();
}

class __TabsNonScrollableDemoState extends State<_TabsNonScrollableDemo>
    with SingleTickerProviderStateMixin, RestorationMixin {
  late TabController _tabController;

  final RestorableInt tabIndex = RestorableInt(0);

  @override
  String get restorationId => 'tab_non_scrollable_demo';

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(tabIndex, 'tab_index');
    _tabController.index = tabIndex.value;
  }

  @override
  void initState() {
    super.initState();
    _tabController = TabController(length: 3, vsync: this);
    _tabController.addListener(() {
      // When the tab controller's value is updated, make sure to update the
      // tab index value, which is state restorable.
      setState(() {
        tabIndex.value = _tabController.index;
      });
    });
  }

  @override
  void dispose() {
    _tabController.dispose();
    tabIndex.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    final List<String> tabs = <String>[
      localizations.colorsRed,
      localizations.colorsOrange,
      localizations.colorsGreen,
    ];

    return Scaffold(
      appBar: AppBar(
        automaticallyImplyLeading: false,
        title: Text(localizations.demoTabsNonScrollingTitle),
        bottom: TabBar(
          controller: _tabController,
          tabs: <Widget>[for (final String tab in tabs) Tab(text: tab)],
        ),
      ),
      body: TabBarView(
        controller: _tabController,
        children: <Widget>[for (final String tab in tabs) Center(child: Text(tab))],
      ),
    );
  }
}

// END
