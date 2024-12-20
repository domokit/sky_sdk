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
      // A ScrollView that creates custom scroll effects using slivers.
      child: CustomScrollView(
        // A list of sliver widgets.
        slivers: <Widget>[
          const CupertinoSliverNavigationBar(
            leading: Icon(CupertinoIcons.person_2),
            // This title is visible in both collapsed and expanded states.
            // When the "middle" parameter is omitted, the widget provided
            // in the "largeTitle" parameter is used instead in the collapsed state.
            largeTitle: Text('Contacts'),
            trailing: Icon(CupertinoIcons.add_circled),
          ),
          // This widget fills the remaining space in the viewport.
          // Drag the scrollable area to collapse the CupertinoSliverNavigationBar.
          SliverFillRemaining(
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
                  child: const Text('Go to Next Page'),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}

class NextPage extends StatelessWidget {
  const NextPage({super.key});

  @override
  Widget build(BuildContext context) {
    final Brightness brightness = CupertinoTheme.brightnessOf(context);
    return CupertinoPageScaffold(
      child: CustomScrollView(
        slivers: <Widget>[
          CupertinoSliverNavigationBar(
            backgroundColor: CupertinoColors.systemYellow,
            border: Border(
              bottom: BorderSide(
                color:
                    brightness == Brightness.light ? CupertinoColors.black : CupertinoColors.white,
              ),
            ),
            // The middle widget is visible in both collapsed and expanded states.
            middle: const Text('Contacts Group'),
            // When the "middle" parameter is implemented, the largest title is only visible
            // when the CupertinoSliverNavigationBar is fully expanded.
            largeTitle: const Text('Family'),
          ),
          const SliverFillRemaining(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
              children: <Widget>[
                Text('Drag me up', textAlign: TextAlign.center),
                // When the "leading" parameter is omitted on a route that has a previous page,
                // the back button is automatically added to the leading position.
                Text('Tap on the leading button to navigate back', textAlign: TextAlign.center),
              ],
            ),
          ),
        ],
      ),
    );
  }
}
