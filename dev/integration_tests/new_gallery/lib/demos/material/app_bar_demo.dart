// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import '../../gallery_localizations.dart';

// BEGIN appbarDemo

class AppBarDemo extends StatelessWidget {
  const AppBarDemo({super.key});

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localization = GalleryLocalizations.of(context)!;
    return Scaffold(
      appBar: AppBar(
        leading: IconButton(
          tooltip: MaterialLocalizations.of(context).openAppDrawerTooltip,
          icon: const Icon(Icons.menu),
          onPressed: () {},
        ),
        title: Text(localization.demoAppBarTitle),
        actions: <Widget>[
          IconButton(
            tooltip: localization.starterAppTooltipFavorite,
            icon: const Icon(Icons.favorite),
            onPressed: () {},
          ),
          IconButton(
            tooltip: localization.starterAppTooltipSearch,
            icon: const Icon(Icons.search),
            onPressed: () {},
          ),
          PopupMenuButton<Text>(
            itemBuilder: (BuildContext context) {
              return <PopupMenuEntry<Text>>[
                PopupMenuItem<Text>(child: Text(localization.demoNavigationRailFirst)),
                PopupMenuItem<Text>(child: Text(localization.demoNavigationRailSecond)),
                PopupMenuItem<Text>(child: Text(localization.demoNavigationRailThird)),
              ];
            },
          ),
        ],
      ),
      body: Center(child: Text(localization.cupertinoTabBarHomeTab)),
    );
  }
}

// END
