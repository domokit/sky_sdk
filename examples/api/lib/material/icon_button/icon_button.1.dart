// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [IconButton].

void main() => runApp(const IconButtonExampleApp());

class IconButtonExampleApp extends StatelessWidget {
  const IconButtonExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('IconButton Sample')),
        body: const IconButtonExample(),
      ),
    );
  }
}

class IconButtonExample extends StatelessWidget {
  const IconButtonExample({super.key});

  @override
  @override
  Widget build(BuildContext context) {
    return Material(
      color: Colors.white,
      child: Center(
        child: Ink(
          decoration: const ShapeDecoration(color: Colors.lightBlue, shape: CircleBorder()),
          child: IconButton(icon: const Icon(Icons.android), color: Colors.white, onPressed: () {}),
        ),
      ),
    );
  }
}
