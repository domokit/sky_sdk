// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [WidgetStateMouseCursor].

void main() {
  runApp(const WidgetStateMouseCursorExampleApp());
}

class WidgetStateMouseCursorExampleApp extends StatelessWidget {
  const WidgetStateMouseCursorExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('WidgetStateMouseCursor Sample')),
        body: const WidgetStateMouseCursorExample(),
      ),
    );
  }
}

class ListTileCursor extends WidgetStateMouseCursor {
  const ListTileCursor();

  @override
  MouseCursor resolve(Set<WidgetState> states) {
    if (states.contains(WidgetState.disabled)) {
      return SystemMouseCursors.forbidden;
    }

    return SystemMouseCursors.click;
  }

  @override
  String get debugDescription => 'ListTileCursor()';
}

class WidgetStateMouseCursorExample extends StatefulWidget {
  const WidgetStateMouseCursorExample({super.key});

  @override
  State<WidgetStateMouseCursorExample> createState() => _WidgetStateMouseCursorExampleState();
}

class _WidgetStateMouseCursorExampleState extends State<WidgetStateMouseCursorExample> {
  bool _enabled = false;

  @override
  Widget build(BuildContext context) {
    return Column(
      mainAxisAlignment: MainAxisAlignment.center,
      children: <Widget>[
        ListTile(
          title: const Text('ListTile'),
          enabled: _enabled,
          onTap: () {},
          mouseCursor: const ListTileCursor(),
        ),
        const SizedBox(height: 10),
        Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            Text('Enabled: ', style: Theme.of(context).textTheme.titleSmall),
            Switch(
              value: _enabled,
              onChanged: (_) {
                setState(() {
                  _enabled = !_enabled;
                });
              },
            ),
          ],
        ),
      ],
    );
  }
}
