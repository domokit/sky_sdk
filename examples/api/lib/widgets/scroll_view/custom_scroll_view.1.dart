// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [CustomScrollView].

void main() => runApp(const CustomScrollViewExampleApp());

class CustomScrollViewExampleApp extends StatelessWidget {
  const CustomScrollViewExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: CustomScrollViewExample());
  }
}

class CustomScrollViewExample extends StatefulWidget {
  const CustomScrollViewExample({super.key});

  @override
  State<CustomScrollViewExample> createState() => _CustomScrollViewExampleState();
}

class _CustomScrollViewExampleState extends State<CustomScrollViewExample> {
  List<int> top = <int>[];
  List<int> bottom = <int>[0];

  @override
  Widget build(BuildContext context) {
    const Key centerKey = ValueKey<String>('bottom-sliver-list');
    return Scaffold(
      appBar: AppBar(
        title: const Text('Press on the plus to add items above and below'),
        leading: IconButton(
          icon: const Icon(Icons.add),
          onPressed: () {
            setState(() {
              top.add(-top.length - 1);
              bottom.add(bottom.length);
            });
          },
        ),
      ),
      body: CustomScrollView(
        center: centerKey,
        slivers: <Widget>[
          SliverList(
            delegate: SliverChildBuilderDelegate((BuildContext context, int index) {
              return Container(
                alignment: Alignment.center,
                color: Colors.blue[200 + top[index] % 4 * 100],
                height: 100 + top[index] % 4 * 20.0,
                child: Text('Item: ${top[index]}'),
              );
            }, childCount: top.length),
          ),
          SliverList(
            key: centerKey,
            delegate: SliverChildBuilderDelegate((BuildContext context, int index) {
              return Container(
                alignment: Alignment.center,
                color: Colors.blue[200 + bottom[index] % 4 * 100],
                height: 100 + bottom[index] % 4 * 20.0,
                child: Text('Item: ${bottom[index]}'),
              );
            }, childCount: bottom.length),
          ),
        ],
      ),
    );
  }
}
