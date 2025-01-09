// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

class ButtonMatrixApp extends StatefulWidget {
  const ButtonMatrixApp({super.key});

  @override
  ButtonMatrixAppState createState() => ButtonMatrixAppState();
}

class ButtonMatrixAppState extends State<ButtonMatrixApp> {
  int count = 1;
  int increment = 1;

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(
          title: Text('Count: $count'),
          actions: <Widget>[
            TextButton(
              onPressed:
                  () => setState(() {
                    count += increment;
                  }),
              child: Text('Add $increment'),
            ),
          ],
        ),
        body: Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: List<Widget>.filled(
            3,
            Column(
              children: List<Widget>.filled(
                10,
                TextButton(
                  child: const Text('Faster'),
                  onPressed:
                      () => setState(() {
                        increment += 1;
                      }),
                ),
              ),
            ),
          ),
        ),
      ),
    );
  }
}

void main() {
  runApp(const ButtonMatrixApp());
}
