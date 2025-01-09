// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

/// Flutter code sample for [CupertinoSliverNavigationBar].

void main() => runApp(const SliverNavBarApp());

class SliverNavBarApp extends StatelessWidget {
  const SliverNavBarApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const CupertinoApp(
      theme: CupertinoThemeData(brightness: Brightness.light),
      home: SliverNavBarExample(),
    );
  }
}

class SliverNavBarExample extends StatelessWidget {
  const SliverNavBarExample({super.key});

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      child: CustomScrollView(
        slivers: <Widget>[
          const CupertinoSliverNavigationBar(
            leading: Icon(CupertinoIcons.person_2),
            largeTitle: Text('Contacts'),
            trailing: Icon(CupertinoIcons.add_circled),
          ),
          SliverFillRemaining(
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 10.0),
              child: Column(
                mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                children: <Widget>[
                  const Text('Drag me up', textAlign: TextAlign.center),
                  CupertinoButton.filled(
                    onPressed: () {
                      Navigator.push(
                        context,
                        CupertinoPageRoute<Widget>(
                          builder: (BuildContext context) {
                            return const NextPage();
                          },
                        ),
                      );
                    },
                    child: const Text('Bottom Automatic mode'),
                  ),
                  CupertinoButton.filled(
                    onPressed: () {
                      Navigator.push(
                        context,
                        CupertinoPageRoute<Widget>(
                          builder: (BuildContext context) {
                            return const NextPage(bottomMode: NavigationBarBottomMode.always);
                          },
                        ),
                      );
                    },
                    child: const Text('Bottom Always mode'),
                  ),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class NextPage extends StatelessWidget {
  const NextPage({super.key, this.bottomMode = NavigationBarBottomMode.automatic});

  final NavigationBarBottomMode bottomMode;

  @override
  Widget build(BuildContext context) {
    final Brightness brightness = CupertinoTheme.brightnessOf(context);
    return CupertinoPageScaffold(
      child: CustomScrollView(
        slivers: <Widget>[
          CupertinoSliverNavigationBar.search(
            backgroundColor: CupertinoColors.systemYellow,
            border: Border(
              bottom: BorderSide(
                color:
                    brightness == Brightness.light ? CupertinoColors.black : CupertinoColors.white,
              ),
            ),
            middle: const Text('Contacts Group'),
            largeTitle: const Text('Family'),
            bottomMode: bottomMode,
          ),
          const SliverFillRemaining(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
              children: <Widget>[
                Text('Drag me up', textAlign: TextAlign.center),
                Text('Tap on the leading button to navigate back', textAlign: TextAlign.center),
              ],
            ),
          ),
        ],
      ),
    );
  }
}
