// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [Tooltip].

void main() => runApp(const TooltipExampleApp());

class TooltipExampleApp extends StatelessWidget {
  const TooltipExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData(tooltipTheme: const TooltipThemeData(preferBelow: false)),
      home: Scaffold(
        appBar: AppBar(title: const Text('Tooltip Sample')),
        body: const Center(child: TooltipSample()),
      ),
    );
  }
}

class TooltipSample extends StatelessWidget {
  const TooltipSample({super.key});

  @override
  Widget build(BuildContext context) {
    return const Tooltip(
      richMessage: TextSpan(
        text: 'I am a rich tooltip. ',
        style: TextStyle(color: Colors.red),
        children: <InlineSpan>[
          TextSpan(
            text: 'I am another span of this rich tooltip',
            style: TextStyle(fontWeight: FontWeight.bold),
          ),
        ],
      ),
      child: Text('Tap this text and hold down to show a tooltip.'),
    );
  }
}
