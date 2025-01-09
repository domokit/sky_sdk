// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [Badge].

void main() => runApp(const BadgeExampleApp());

class BadgeExampleApp extends StatelessWidget {
  const BadgeExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(appBar: AppBar(title: const Text('Badge Sample')), body: const BadgeExample()),
    );
  }
}

class BadgeExample extends StatelessWidget {
  const BadgeExample({super.key});

  @override
  Widget build(BuildContext context) {
    return Center(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: <Widget>[
          IconButton(
            icon: const Badge(
              label: Text('Your label'),
              backgroundColor: Colors.blueAccent,
              child: Icon(Icons.receipt),
            ),
            onPressed: () {},
          ),
          const SizedBox(height: 20),
          IconButton(
            icon: Badge.count(count: 9999, child: const Icon(Icons.notifications)),
            onPressed: () {},
          ),
        ],
      ),
    );
  }
}
