// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [InputDecorator].

void main() => runApp(const PrefixIconExampleApp());

class PrefixIconExampleApp extends StatelessWidget {
  const PrefixIconExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: Scaffold(body: InputDecoratorExample()));
  }
}

class InputDecoratorExample extends StatelessWidget {
  const InputDecoratorExample({super.key});

  @override
  Widget build(BuildContext context) {
    return const TextField(
      decoration: InputDecoration(
        border: OutlineInputBorder(),
        labelText: 'Enter name',
        prefixIcon: Align(widthFactor: 1.0, heightFactor: 1.0, child: Icon(Icons.person)),
      ),
    );
  }
}
