// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [SliverFillRemaining].

void main() => runApp(const SliverFillRemainingExampleApp());

class SliverFillRemainingExampleApp extends StatelessWidget {
  const SliverFillRemainingExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('SliverFillRemaining Sample')),
        body: const SliverFillRemainingExample(),
      ),
    );
  }
}

class SliverFillRemainingExample extends StatelessWidget {
  const SliverFillRemainingExample({super.key});

  @override
  Widget build(BuildContext context) {
    return CustomScrollView(
      slivers: <Widget>[
        SliverFixedExtentList(
          itemExtent: 100.0,
          delegate: SliverChildBuilderDelegate((BuildContext context, int index) {
            return Container(color: index.isEven ? Colors.amber[200] : Colors.blue[200]);
          }, childCount: 3),
        ),
        SliverFillRemaining(
          hasScrollBody: false,
          child: Container(
            color: Colors.orange[300],
            child: const Padding(padding: EdgeInsets.all(50.0), child: FlutterLogo(size: 100)),
          ),
        ),
      ],
    );
  }
}
