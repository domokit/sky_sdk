// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [RawScrollbar].

void main() => runApp(const RawScrollbarExampleApp());

class RawScrollbarExampleApp extends StatelessWidget {
  const RawScrollbarExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('RawScrollbar Sample')),
        body: const RawScrollbarExample(),
      ),
    );
  }
}

class RawScrollbarExample extends StatelessWidget {
  const RawScrollbarExample({super.key});

  @override
  Widget build(BuildContext context) {
    return RawScrollbar(
      child: GridView.builder(
        itemCount: 120,
        gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 3),
        itemBuilder: (BuildContext context, int index) {
          return Center(child: Text('item $index'));
        },
      ),
    );
  }
}
