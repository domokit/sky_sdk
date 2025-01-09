// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

/// Flutter code sample for base [CupertinoListSection] and [CupertinoListTile].

void main() => runApp(const CupertinoListSectionBaseApp());

class CupertinoListSectionBaseApp extends StatelessWidget {
  const CupertinoListSectionBaseApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const CupertinoApp(home: ListSectionBaseExample());
  }
}

class ListSectionBaseExample extends StatelessWidget {
  const ListSectionBaseExample({super.key});

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      child: CupertinoListSection(
        header: const Text('My Reminders'),
        children: <CupertinoListTile>[
          CupertinoListTile(
            title: const Text('Open pull request'),
            leading: Container(
              width: double.infinity,
              height: double.infinity,
              color: CupertinoColors.activeGreen,
            ),
            trailing: const CupertinoListTileChevron(),
            onTap:
                () => Navigator.of(context).push(
                  CupertinoPageRoute<void>(
                    builder: (BuildContext context) {
                      return const _SecondPage(text: 'Open pull request');
                    },
                  ),
                ),
          ),
          CupertinoListTile(
            title: const Text('Push to master'),
            leading: Container(
              width: double.infinity,
              height: double.infinity,
              color: CupertinoColors.systemRed,
            ),
            additionalInfo: const Text('Not available'),
          ),
          CupertinoListTile(
            title: const Text('View last commit'),
            leading: Container(
              width: double.infinity,
              height: double.infinity,
              color: CupertinoColors.activeOrange,
            ),
            additionalInfo: const Text('12 days ago'),
            trailing: const CupertinoListTileChevron(),
            onTap:
                () => Navigator.of(context).push(
                  CupertinoPageRoute<void>(
                    builder: (BuildContext context) {
                      return const _SecondPage(text: 'Last commit');
                    },
                  ),
                ),
          ),
        ],
      ),
    );
  }
}

class _SecondPage extends StatelessWidget {
  const _SecondPage({required this.text});

  final String text;

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(child: Center(child: Text(text)));
  }
}
