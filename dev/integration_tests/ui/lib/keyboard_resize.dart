// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_driver/driver_extension.dart';

import 'keys.dart' as keys;

void main() {
  enableFlutterDriverExtension(
    enableTextEntryEmulation: false,
    handler: (String? message) async {
      // TODO(cbernaschina): remove when test flakiness is resolved
      return 'keyboard_resize';
    },
  );
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Text Editing',
      theme: ThemeData(primarySwatch: Colors.blue),
      home: const MyHomePage(),
    );
  }
}

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key});

  @override
  State<MyHomePage> createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  final TextEditingController _controller = TextEditingController();

  @override
  Widget build(BuildContext context) {
    final TextField textField = TextField(
      key: const Key(keys.kDefaultTextField),
      controller: _controller,
      focusNode: FocusNode(),
      decoration: const InputDecoration(border: OutlineInputBorder()),
    );
    return Scaffold(
      body: SafeArea(
        child: Stack(
          fit: StackFit.expand,
          alignment: Alignment.bottomCenter,
          children: <Widget>[
            LayoutBuilder(
              builder: (BuildContext context, BoxConstraints constraints) {
                return Center(
                  child: Text('${constraints.biggest.height}', key: const Key(keys.kHeightText)),
                );
              },
            ),
            textField,
          ],
        ),
      ),
      floatingActionButton: FloatingActionButton(
        key: const Key(keys.kUnfocusButton),
        onPressed: () {
          textField.focusNode!.unfocus();
        },
        tooltip: 'Unfocus',
        child: const Icon(Icons.done),
      ),
    );
  }
}
