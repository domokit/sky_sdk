// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [TextEditingController].

void main() => runApp(const TextEditingControllerExampleApp());

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
  final TextEditingController _controller = TextEditingController();

  @override
  void initState() {
    super.initState();
    _controller.addListener(() {
      final String text = _controller.text.toLowerCase();
      _controller.value = _controller.value.copyWith(
        text: text,
        selection: TextSelection(baseOffset: text.length, extentOffset: text.length),
        composing: TextRange.empty,
      );
    });
  }

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
        padding: const EdgeInsets.all(6),
        child: TextFormField(
          controller: _controller,
          decoration: const InputDecoration(border: OutlineInputBorder()),
        ),
      ),
    );
  }
}
