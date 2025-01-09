// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [TextEditingController].

void main() {
  runApp(const TextEditingControllerExampleApp());
}

class TextEditingControllerExampleApp extends StatelessWidget {
  const TextEditingControllerExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: TextEditingControllerExample());
  }
}

class TextEditingControllerExample extends StatefulWidget {
  const TextEditingControllerExample({super.key});

  @override
  State<TextEditingControllerExample> createState() => _TextEditingControllerExampleState();
}

class _TextEditingControllerExampleState extends State<TextEditingControllerExample> {
  // Create a controller whose initial selection is empty (collapsed) and positioned
  // before the text (offset is 0).
  final TextEditingController _controller = TextEditingController.fromValue(
    const TextEditingValue(text: 'Flutter', selection: TextSelection.collapsed(offset: 0)),
  );

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Container(
        alignment: Alignment.center,
        padding: const EdgeInsets.all(16.0),
        child: TextField(
          controller: _controller,
          autofocus: true,
          decoration: const InputDecoration(filled: true),
        ),
      ),
    );
  }
}
