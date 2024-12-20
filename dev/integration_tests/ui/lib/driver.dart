// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_driver/driver_extension.dart';

void main() {
  enableFlutterDriverExtension();
  runApp(const DriverTestApp());
}

class DriverTestApp extends StatefulWidget {
  const DriverTestApp({super.key});

  @override
  State<StatefulWidget> createState() {
    return DriverTestAppState();
  }
}

class DriverTestAppState extends State<DriverTestApp> {
  bool present = true;
  Letter _selectedValue = Letter.a;

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('FlutterDriver test')),
        body: ListView(
          padding: const EdgeInsets.all(5.0),
          children: <Widget>[
            Row(
              children: <Widget>[
                Expanded(child: Text(present ? 'present' : 'absent')),
                ElevatedButton(
                  child: const Text('toggle', key: ValueKey<String>('togglePresent')),
                  onPressed: () {
                    setState(() {
                      present = !present;
                    });
                  },
                ),
              ],
            ),
            Row(
              children: <Widget>[
                const Expanded(child: Text('hit testability')),
                DropdownButton<Letter>(
                  key: const ValueKey<String>('dropdown'),
                  value: _selectedValue,
                  onChanged: (Letter? newValue) {
                    setState(() {
                      _selectedValue = newValue!;
                    });
                  },
                  items: const <DropdownMenuItem<Letter>>[
                    DropdownMenuItem<Letter>(
                      value: Letter.a,
                      child: Text('Aaa', key: ValueKey<String>('a')),
                    ),
                    DropdownMenuItem<Letter>(
                      value: Letter.b,
                      child: Text('Bbb', key: ValueKey<String>('b')),
                    ),
                    DropdownMenuItem<Letter>(
                      value: Letter.c,
                      child: Text('Ccc', key: ValueKey<String>('c')),
                    ),
                  ],
                ),
              ],
            ),
            const TextField(key: ValueKey<String>('enter-text-field')),
          ],
        ),
      ),
    );
  }
}

enum Letter { a, b, c }
