// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

void main() => runApp(const MyApp());

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(
      key: Key('mainapp'),
      title: 'Integration Test App',
      home: MyHomePage(title: 'Integration Test App'),
    );
  }
}

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key, this.title});

  final String? title;

  @override
  State<MyHomePage> createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  String infoText = 'no-enter';

  // Controller with no initial value;
  final TextEditingController _emptyController = TextEditingController();

  final TextEditingController _controller = TextEditingController(text: 'Text1');

  final TextEditingController _controller2 = TextEditingController(text: 'Text2');

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text(widget.title ?? '')),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            const Text('Text Editing Test 1'),
            TextFormField(
              key: const Key('empty-input'),
              enabled: true,
              controller: _emptyController,
              decoration: const InputDecoration(
                contentPadding: EdgeInsets.all(10.0),
                labelText: 'Empty Input Field:',
              ),
            ),
            const Text('Text Editing Test 2'),
            TextFormField(
              key: const Key('input'),
              enabled: true,
              controller: _controller,
              decoration: const InputDecoration(
                contentPadding: EdgeInsets.all(10.0),
                labelText: 'Text Input Field:',
              ),
            ),
            const Text('Text Editing Test 3'),
            TextFormField(
              key: const Key('input2'),
              enabled: true,
              controller: _controller2,
              decoration: const InputDecoration(
                contentPadding: EdgeInsets.all(10.0),
                labelText: 'Text Input Field 2:',
              ),
              onFieldSubmitted: (String str) {
                print('event received');
                setState(() => infoText = 'enter pressed');
              },
            ),
            Text(infoText, key: const Key('text')),
            const Padding(
              padding: EdgeInsets.all(12.0),
              child: SelectableText(
                'Lorem ipsum dolor sit amet',
                key: Key('selectable'),
                style: TextStyle(fontFamily: 'Roboto', fontSize: 20.0),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
