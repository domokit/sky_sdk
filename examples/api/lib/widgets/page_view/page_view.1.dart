// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for a [PageView] using the `findChildIndexCallback` argument.
void main() => runApp(const PageViewExampleApp());

class PageViewExampleApp extends StatelessWidget {
  const PageViewExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: PageViewExample());
  }
}

class PageViewExample extends StatefulWidget {
  const PageViewExample({super.key});

  @override
  State<PageViewExample> createState() => _PageViewExampleState();
}

class _PageViewExampleState extends State<PageViewExample> {
  List<String> items = <String>['1', '2', '3', '4', '5'];

  void _reverse() {
    setState(() {
      items = items.reversed.toList();
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('PageView Sample')),
      body: SafeArea(
        child: PageView.custom(
          childrenDelegate: SliverChildBuilderDelegate(
            (BuildContext context, int index) {
              return KeepAliveItem(data: items[index], key: ValueKey<String>(items[index]));
            },
            childCount: items.length,
            findChildIndexCallback: (Key key) {
              final ValueKey<String> valueKey = key as ValueKey<String>;
              final String data = valueKey.value;
              final int index = items.indexOf(data);
              if (index >= 0) {
                return index;
              }
              return null;
            },
          ),
        ),
      ),
      bottomNavigationBar: BottomAppBar(
        child: Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            TextButton(onPressed: () => _reverse(), child: const Text('Reverse items')),
          ],
        ),
      ),
    );
  }
}

class KeepAliveItem extends StatefulWidget {
  const KeepAliveItem({super.key, required this.data});

  final String data;

  @override
  State<KeepAliveItem> createState() => _KeepAliveItemState();
}

class _KeepAliveItemState extends State<KeepAliveItem> with AutomaticKeepAliveClientMixin {
  @override
  bool get wantKeepAlive => true;

  @override
  Widget build(BuildContext context) {
    super.build(context);
    return Text(widget.data);
  }
}
