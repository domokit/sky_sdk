// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

/// Flutter code sample for [CupertinoScrollbar].

void main() => runApp(const ScrollbarApp());

class ScrollbarApp extends StatelessWidget {
  const ScrollbarApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const CupertinoApp(
      theme: CupertinoThemeData(brightness: Brightness.light),
      home: ScrollbarExample(),
    );
  }
}

class ScrollbarExample extends StatefulWidget {
  const ScrollbarExample({super.key});

  @override
  State<ScrollbarExample> createState() => _ScrollbarExampleState();
}

class _ScrollbarExampleState extends State<ScrollbarExample> {
  final ScrollController _controllerOne = ScrollController();

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: const CupertinoNavigationBar(middle: Text('CupertinoScrollbar Sample')),
      child: CupertinoScrollbar(
        thickness: 6.0,
        thicknessWhileDragging: 10.0,
        radius: const Radius.circular(34.0),
        radiusWhileDragging: Radius.zero,
        controller: _controllerOne,
        thumbVisibility: true,
        child: ListView.builder(
          controller: _controllerOne,
          itemCount: 120,
          itemBuilder: (BuildContext context, int index) {
            return Center(
              child: Padding(padding: const EdgeInsets.all(8.0), child: Text('Item $index')),
            );
          },
        ),
      ),
    );
  }
}
