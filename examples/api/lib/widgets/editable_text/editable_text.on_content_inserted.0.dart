// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:flutter/material.dart';

/// Flutter code sample for [EditableText.contentInsertionConfiguration].

void main() => runApp(const KeyboardInsertedContentApp());

class KeyboardInsertedContentApp extends StatelessWidget {
  const KeyboardInsertedContentApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: KeyboardInsertedContentDemo());
  }
}

class KeyboardInsertedContentDemo extends StatefulWidget {
  const KeyboardInsertedContentDemo({super.key});

  @override
  State<KeyboardInsertedContentDemo> createState() => _KeyboardInsertedContentDemoState();
}

class _KeyboardInsertedContentDemoState extends State<KeyboardInsertedContentDemo> {
  final TextEditingController _controller = TextEditingController();
  Uint8List? bytes;

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Keyboard Inserted Content Sample')),
      body: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: <Widget>[
          const Text("Here's a text field that supports inserting only png or gif content:"),
          TextField(
            controller: _controller,
            contentInsertionConfiguration: ContentInsertionConfiguration(
              allowedMimeTypes: const <String>['image/png', 'image/gif'],
              onContentInserted: (KeyboardInsertedContent data) async {
                if (data.data != null) {
                  setState(() {
                    bytes = data.data;
                  });
                }
              },
            ),
          ),
          if (bytes != null) const Text("Here's the most recently inserted content:"),
          if (bytes != null) Image.memory(bytes!),
        ],
      ),
    );
  }
}
