// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

void main() => runApp(const SliverConstrainedCrossAxisExampleApp());

class SliverConstrainedCrossAxisExampleApp extends StatelessWidget {
  const SliverConstrainedCrossAxisExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('SliverConstrainedCrossAxis Sample')),
        body: const SliverConstrainedCrossAxisExample(),
      ),
    );
  }
}

class SliverConstrainedCrossAxisExample extends StatelessWidget {
  const SliverConstrainedCrossAxisExample({super.key});

  @override
  Widget build(BuildContext context) {
    return CustomScrollView(
      slivers: <Widget>[
        SliverConstrainedCrossAxis(
          maxExtent: 200,
          sliver: SliverList.builder(
            itemBuilder: (BuildContext context, int index) {
              return Container(
                color: index.isEven ? Colors.amber[300] : Colors.blue[300],
                height: 100.0,
                child: Center(child: Text('Item $index', style: const TextStyle(fontSize: 24))),
              );
            },
            itemCount: 10,
          ),
        ),
      ],
    );
  }
}
