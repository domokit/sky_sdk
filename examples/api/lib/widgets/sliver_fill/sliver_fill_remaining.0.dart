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
        SliverToBoxAdapter(child: Container(color: Colors.amber[300], height: 150.0)),
        SliverFillRemaining(
          hasScrollBody: false,
          child: Container(
            color: Colors.blue[100],
            child: Icon(Icons.sentiment_very_satisfied, size: 75, color: Colors.blue[900]),
          ),
        ),
      ],
    );
  }
}
