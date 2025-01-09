// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';

/// Flutter code sample for [Switch].

void main() => runApp(const SwitchApp());

class SwitchApp extends StatelessWidget {
  const SwitchApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData.light(useMaterial3: true).copyWith(
        // Use the ambient CupertinoThemeData to style all widgets which would
        // otherwise use iOS defaults.
        cupertinoOverrideTheme: const CupertinoThemeData(applyThemeToAll: true),
      ),
      home: Scaffold(
        appBar: AppBar(title: const Text('Switch Sample')),
        body: const Center(child: SwitchExample()),
      ),
    );
  }
}

class SwitchExample extends StatefulWidget {
  const SwitchExample({super.key});

  @override
  State<SwitchExample> createState() => _SwitchExampleState();
}

class _SwitchExampleState extends State<SwitchExample> {
  bool light = true;

  @override
  Widget build(BuildContext context) {
    return Column(
      mainAxisAlignment: MainAxisAlignment.center,
      children: <Widget>[
        Switch.adaptive(
          value: light,
          onChanged: (bool value) {
            setState(() {
              light = value;
            });
          },
        ),
        Switch.adaptive(
          // Don't use the ambient CupertinoThemeData to style this switch.
          applyCupertinoTheme: false,
          value: light,
          onChanged: (bool value) {
            setState(() {
              light = value;
            });
          },
        ),
      ],
    );
  }
}
