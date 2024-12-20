// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [MouseCursor].

void main() => runApp(const MouseCursorExampleApp());

class MouseCursorExampleApp extends StatelessWidget {
  const MouseCursorExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('MouseCursor Code Sample')),
        body: const MouseCursorExample(),
      ),
    );
  }
}

class MouseCursorExample extends StatelessWidget {
  const MouseCursorExample({super.key});

  @override
  Widget build(BuildContext context) {
    return Center(
      child: MouseRegion(
        cursor: SystemMouseCursors.text,
        child: Container(
          width: 200,
          height: 100,
          decoration: BoxDecoration(color: Colors.blue, border: Border.all(color: Colors.yellow)),
        ),
      ),
    );
  }
}
