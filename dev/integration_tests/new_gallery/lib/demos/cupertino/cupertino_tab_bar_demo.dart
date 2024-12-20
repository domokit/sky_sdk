// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

import '../../gallery_localizations.dart';

// BEGIN cupertinoNavigationDemo

class _TabInfo {
  const _TabInfo(this.title, this.icon);

  final String title;
  final IconData icon;
}

class CupertinoTabBarDemo extends StatelessWidget {
  const CupertinoTabBarDemo({super.key});

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    final List<_TabInfo> tabInfo = <_TabInfo>[
      _TabInfo(localizations.cupertinoTabBarHomeTab, CupertinoIcons.home),
      _TabInfo(localizations.cupertinoTabBarChatTab, CupertinoIcons.conversation_bubble),
      _TabInfo(localizations.cupertinoTabBarProfileTab, CupertinoIcons.profile_circled),
    ];

    return DefaultTextStyle(
      style: CupertinoTheme.of(context).textTheme.textStyle,
      child: CupertinoTabScaffold(
        restorationId: 'cupertino_tab_scaffold',
        tabBar: CupertinoTabBar(
          items: <BottomNavigationBarItem>[
            for (final _TabInfo tabInfo in tabInfo)
              BottomNavigationBarItem(label: tabInfo.title, icon: Icon(tabInfo.icon)),
          ],
        ),
        tabBuilder: (BuildContext context, int index) {
          return CupertinoTabView(
            restorationScopeId: 'cupertino_tab_view_$index',
            builder:
                (BuildContext context) =>
                    _CupertinoDemoTab(title: tabInfo[index].title, icon: tabInfo[index].icon),
            defaultTitle: tabInfo[index].title,
          );
        },
      ),
    );
  }
}

class _CupertinoDemoTab extends StatelessWidget {
  const _CupertinoDemoTab({required this.title, required this.icon});

  final String title;
  final IconData icon;

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: const CupertinoNavigationBar(),
      backgroundColor: CupertinoColors.systemBackground,
      child: Center(child: Icon(icon, semanticLabel: title, size: 100)),
    );
  }
}

// END
