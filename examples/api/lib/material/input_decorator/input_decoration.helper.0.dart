// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [InputDecoration.helper].

void main() => runApp(const HelperExampleApp());

class HelperExampleApp extends StatelessWidget {
  const HelperExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('InputDecoration.helper Sample')),
        body: const HelperExample(),
      ),
    );
  }
}

class HelperExample extends StatelessWidget {
  const HelperExample({super.key});

  @override
  Widget build(BuildContext context) {
    return const Center(
      child: TextField(
        decoration: InputDecoration(
          helper: Text.rich(
            TextSpan(
              children: <InlineSpan>[
                WidgetSpan(child: Text('Helper Text ')),
                WidgetSpan(child: Icon(Icons.help_outline, color: Colors.blue, size: 20.0)),
              ],
            ),
          ),
        ),
      ),
    );
  }
}
