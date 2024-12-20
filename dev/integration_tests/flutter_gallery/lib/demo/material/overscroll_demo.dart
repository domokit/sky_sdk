// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/material.dart';

import '../../gallery/demo.dart';

enum IndicatorType { overscroll, refresh }

class OverscrollDemo extends StatefulWidget {
  const OverscrollDemo({super.key});

  static const String routeName = '/material/overscroll';

  @override
  OverscrollDemoState createState() => OverscrollDemoState();
}

class OverscrollDemoState extends State<OverscrollDemo> {
  final GlobalKey<RefreshIndicatorState> _refreshIndicatorKey = GlobalKey<RefreshIndicatorState>();
  static final List<String> _items = <String>[
    'A',
    'B',
    'C',
    'D',
    'E',
    'F',
    'G',
    'H',
    'I',
    'J',
    'K',
    'L',
    'M',
    'N',
  ];

  Future<void> _handleRefresh() {
    final Completer<void> completer = Completer<void>();
    Timer(const Duration(seconds: 3), () => completer.complete());
    return completer.future.then((_) {
      if (!mounted) {
        return;
      }
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: const Text('Refresh complete'),
          action: SnackBarAction(
            label: 'RETRY',
            onPressed: () {
              _refreshIndicatorKey.currentState!.show();
            },
          ),
        ),
      );
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Pull to refresh'),
        actions: <Widget>[
          MaterialDemoDocumentationButton(OverscrollDemo.routeName),
          IconButton(
            icon: const Icon(Icons.refresh),
            tooltip: 'Refresh',
            onPressed: () {
              _refreshIndicatorKey.currentState!.show();
            },
          ),
        ],
      ),
      body: RefreshIndicator(
        key: _refreshIndicatorKey,
        onRefresh: _handleRefresh,
        child: Scrollbar(
          child: ListView.builder(
            primary: true,
            padding: kMaterialListPadding,
            itemCount: _items.length,
            itemBuilder: (BuildContext context, int index) {
              final String item = _items[index];
              return ListTile(
                isThreeLine: true,
                leading: CircleAvatar(child: Text(item)),
                title: Text('This item represents $item.'),
                subtitle: const Text(
                  'Even more additional list item information appears on line three.',
                ),
              );
            },
          ),
        ),
      ),
    );
  }
}
