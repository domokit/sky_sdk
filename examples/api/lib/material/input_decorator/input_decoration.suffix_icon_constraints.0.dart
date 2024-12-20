// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [InputDecoration.suffixIconConstraints].

void main() => runApp(const SuffixIconConstraintsExampleApp());

class SuffixIconConstraintsExampleApp extends StatelessWidget {
  const SuffixIconConstraintsExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData(useMaterial3: true),
      home: Scaffold(
        appBar: AppBar(title: const Text('InputDecoration Sample')),
        body: const SuffixIconConstraintsExample(),
      ),
    );
  }
}

class SuffixIconConstraintsExample extends StatelessWidget {
  const SuffixIconConstraintsExample({super.key});

  @override
  Widget build(BuildContext context) {
    return const Padding(
      padding: EdgeInsets.symmetric(horizontal: 8.0),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: <Widget>[
          TextField(
            decoration: InputDecoration(
              hintText: 'Normal Icon Constraints',
              suffixIcon: Icon(Icons.search),
            ),
          ),
          SizedBox(height: 10),
          TextField(
            decoration: InputDecoration(
              isDense: true,
              hintText: 'Smaller Icon Constraints',
              suffixIcon: Icon(Icons.search),
              suffixIconConstraints: BoxConstraints(minHeight: 32, minWidth: 32),
            ),
          ),
        ],
      ),
    );
  }
}
