// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [MaterialStateMouseCursor].

void main() => runApp(const MaterialStateMouseCursorExampleApp());

class MaterialStateMouseCursorExampleApp extends StatelessWidget {
  const MaterialStateMouseCursorExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('MaterialStateMouseCursor Sample')),
        body: const Center(
          child: MaterialStateMouseCursorExample(
            // TRY THIS: Switch to get a different mouse cursor while hovering ListTile.
            enabled: false,
          ),
        ),
      ),
    );
  }
}

class ListTileCursor extends MaterialStateMouseCursor {
  const ListTileCursor();

  @override
  MouseCursor resolve(Set<MaterialState> states) {
    if (states.contains(MaterialState.disabled)) {
      return SystemMouseCursors.forbidden;
    }
    return SystemMouseCursors.click;
  }

  @override
  String get debugDescription => 'ListTileCursor()';
}

class MaterialStateMouseCursorExample extends StatelessWidget {
  const MaterialStateMouseCursorExample({required this.enabled, super.key});

  final bool enabled;

  @override
  Widget build(BuildContext context) {
    return ListTile(
      title: const Text('ListTile'),
      enabled: enabled,
      onTap: () {},
      mouseCursor: const ListTileCursor(),
    );
  }
}
