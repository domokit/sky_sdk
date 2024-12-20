// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

void main() => runApp(const SliverMainAxisGroupExampleApp());

class SliverMainAxisGroupExampleApp extends StatelessWidget {
  const SliverMainAxisGroupExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('SliverMainAxisGroup Sample')),
        body: const SliverMainAxisGroupExample(),
      ),
    );
  }
}

class SliverMainAxisGroupExample extends StatelessWidget {
  const SliverMainAxisGroupExample({super.key});

  @override
  Widget build(BuildContext context) {
    return CustomScrollView(
      slivers: <Widget>[
        SliverMainAxisGroup(
          slivers: <Widget>[
            const SliverAppBar(title: Text('Section Title'), expandedHeight: 70.0, pinned: true),
            SliverList.builder(
              itemBuilder: (BuildContext context, int index) {
                return Container(
                  color: index.isEven ? Colors.amber[300] : Colors.blue[300],
                  height: 100.0,
                  child: Center(child: Text('Item $index', style: const TextStyle(fontSize: 24))),
                );
              },
              itemCount: 5,
            ),
            SliverToBoxAdapter(
              child: Container(
                color: Colors.cyan,
                height: 100,
                child: const Center(
                  child: Text('Another sliver child', style: TextStyle(fontSize: 24)),
                ),
              ),
            ),
          ],
        ),
        SliverToBoxAdapter(
          child: Container(
            height: 1000,
            decoration: const BoxDecoration(color: Colors.greenAccent),
            child: const Center(child: Text('Hello World!', style: TextStyle(fontSize: 24))),
          ),
        ),
      ],
    );
  }
}
