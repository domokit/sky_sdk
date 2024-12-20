// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [OutlinedButton].

void main() => runApp(const OutlinedButtonExampleApp());

class OutlinedButtonExampleApp extends StatelessWidget {
  const OutlinedButtonExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('OutlinedButton Sample')),
        body: const Center(child: OutlinedButtonExample()),
      ),
    );
  }
}

class OutlinedButtonExample extends StatelessWidget {
  const OutlinedButtonExample({super.key});

  @override
  Widget build(BuildContext context) {
    return OutlinedButton(
      onPressed: () {
        debugPrint('Received click');
      },
      child: const Text('Click Me'),
    );
  }
}
