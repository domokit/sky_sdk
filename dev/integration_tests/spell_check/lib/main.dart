// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Spellcheck Demo',
      theme: ThemeData(primarySwatch: Colors.blue),
      home: const MyHomePage(title: 'Spellcheck Demo'),
    );
  }
}

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key, required this.title});

  final String title;

  @override
  State<MyHomePage> createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text(widget.title)),
      body: Center(
        child: EditableText(
          controller: TextEditingController(),
          backgroundCursorColor: Colors.grey,
          focusNode: FocusNode(),
          style: const TextStyle(),
          cursorColor: Colors.red,
          spellCheckConfiguration: const SpellCheckConfiguration(
            misspelledTextStyle: TextField.materialMisspelledTextStyle,
          ),
        ),
      ),
    );
  }
}
