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
    return Tooltip(
      message: 'I am a Tooltip',
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(25),
        gradient: const LinearGradient(colors: <Color>[Colors.amber, Colors.red]),
      ),
      height: 50,
      padding: const EdgeInsets.all(8.0),
      preferBelow: true,
      textStyle: const TextStyle(fontSize: 24),
      showDuration: const Duration(seconds: 2),
      waitDuration: const Duration(seconds: 1),
      child: const Text('Tap this text and hold down to show a tooltip.'),
    );
  }
}
