// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [DefaultTextStyle].

void main() => runApp(const DefaultTextStyleApp());

class DefaultTextStyleApp extends StatelessWidget {
  const DefaultTextStyleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData(
        useMaterial3: true,
        brightness: Brightness.light,
        colorSchemeSeed: Colors.purple,
      ),
      home: const DefaultTextStyleExample(),
    );
  }
}

class DefaultTextStyleExample extends StatelessWidget {
  const DefaultTextStyleExample({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('DefaultTextStyle.merge Sample')),
      // Inherit MaterialApp text theme and override font size and font weight.
      body: DefaultTextStyle.merge(
        style: const TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
        child: const Center(child: Text('Flutter')),
      ),
    );
  }
}
