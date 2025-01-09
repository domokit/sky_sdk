// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [EditableText.onChanged].

void main() => runApp(const OnChangedExampleApp());

class OnChangedExampleApp extends StatelessWidget {
  const OnChangedExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: OnChangedExample());
  }
}

class OnChangedExample extends StatefulWidget {
  const OnChangedExample({super.key});

  @override
  State<OnChangedExample> createState() => _OnChangedExampleState();
}

class _OnChangedExampleState extends State<OnChangedExample> {
  final TextEditingController _controller = TextEditingController();

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: <Widget>[
          const Text('What number comes next in the sequence?'),
          const Text('1, 1, 2, 3, 5, 8...?'),
          TextField(
            controller: _controller,
            onChanged: (String value) async {
              if (value != '13') {
                return;
              }
              await showDialog<void>(
                context: context,
                builder: (BuildContext context) {
                  return AlertDialog(
                    title: const Text('That is correct!'),
                    content: const Text('13 is the right answer.'),
                    actions: <Widget>[
                      TextButton(
                        onPressed: () {
                          Navigator.pop(context);
                        },
                        child: const Text('OK'),
                      ),
                    ],
                  );
                },
              );
            },
          ),
        ],
      ),
    );
  }
}
