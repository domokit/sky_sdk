// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import '../../gallery_localizations.dart';

// BEGIN navDrawerDemo

// Press the Navigation Drawer button to the left of AppBar to show
// a simple Drawer with two items.
class NavDrawerDemo extends StatelessWidget {
  const NavDrawerDemo({super.key});

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localization = GalleryLocalizations.of(context)!;
    final UserAccountsDrawerHeader drawerHeader = UserAccountsDrawerHeader(
      accountName: Text(localization.demoNavigationDrawerUserName),
      accountEmail: Text(localization.demoNavigationDrawerUserEmail),
      currentAccountPicture: const CircleAvatar(child: FlutterLogo(size: 42.0)),
    );
    final ListView drawerItems = ListView(
      children: <Widget>[
        drawerHeader,
        ListTile(
          title: Text(localization.demoNavigationDrawerToPageOne),
          leading: const Icon(Icons.favorite),
          onTap: () {
            Navigator.pop(context);
          },
        ),
        ListTile(
          title: Text(localization.demoNavigationDrawerToPageTwo),
          leading: const Icon(Icons.comment),
          onTap: () {
            Navigator.pop(context);
          },
        ),
      ],
    );
    return Scaffold(
      appBar: AppBar(title: Text(localization.demoNavigationDrawerTitle)),
      body: Semantics(
        container: true,
        child: Center(
          child: Padding(
            padding: const EdgeInsets.all(50.0),
            child: Text(localization.demoNavigationDrawerText),
          ),
        ),
      ),
      drawer: Drawer(child: drawerItems),
    );
  }
}

// END
