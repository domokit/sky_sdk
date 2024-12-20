// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

void main() => runApp(const SliverCrossAxisGroupExampleApp());

class SliverCrossAxisGroupExampleApp extends StatelessWidget {
  const SliverCrossAxisGroupExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('SliverCrossAxisGroup Sample')),
        body: const SliverCrossAxisGroupExample(),
      ),
    );
  }
}

class SliverCrossAxisGroupExample extends StatelessWidget {
  const SliverCrossAxisGroupExample({super.key});

  @override
  Widget build(BuildContext context) {
    return CustomScrollView(
      slivers: <Widget>[
        SliverCrossAxisGroup(
          slivers: <Widget>[
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
            SliverConstrainedCrossAxis(
              maxExtent: 200,
              sliver: SliverList.builder(
                itemBuilder: (BuildContext context, int index) {
                  return Container(
                    color: index.isEven ? Colors.green[300] : Colors.red[300],
                    height: 100.0,
                    child: Center(
                      child: Text('Item ${index + 5}', style: const TextStyle(fontSize: 24)),
                    ),
                  );
                },
                itemCount: 5,
              ),
            ),
            SliverCrossAxisExpanded(
              flex: 2,
              sliver: SliverList.builder(
                itemBuilder: (BuildContext context, int index) {
                  return Container(
                    color: index.isEven ? Colors.purple[300] : Colors.orange[300],
                    height: 100.0,
                    child: Center(
                      child: Text('Item ${index + 10}', style: const TextStyle(fontSize: 24)),
                    ),
                  );
                },
                itemCount: 5,
              ),
            ),
          ],
        ),
      ],
    );
  }
}
