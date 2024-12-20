// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [Expanded].

void main() => runApp(const ExpandedApp());

class ExpandedApp extends StatelessWidget {
  const ExpandedApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('Expanded Column Sample')),
        body: const ExpandedExample(),
      ),
    );
  }
}

class ExpandedExample extends StatelessWidget {
  const ExpandedExample({super.key});

  @override
  Widget build(BuildContext context) {
    return Center(
      child: Column(
        children: <Widget>[
          Container(color: Colors.blue, height: 100, width: 100),
          Expanded(child: Container(color: Colors.amber, width: 100)),
          Container(color: Colors.blue, height: 100, width: 100),
        ],
      ),
    );
  }
}
