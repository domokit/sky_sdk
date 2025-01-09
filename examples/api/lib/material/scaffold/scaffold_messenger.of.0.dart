// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [ScaffoldMessenger.of].

void main() => runApp(const OfExampleApp());

class OfExampleApp extends StatelessWidget {
  const OfExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('ScaffoldMessenger.of Sample')),
        body: const Center(child: OfExample()),
      ),
    );
  }
}

class OfExample extends StatelessWidget {
  const OfExample({super.key});

  @override
  Widget build(BuildContext context) {
    return ElevatedButton(
      child: const Text('SHOW A SNACKBAR'),
      onPressed: () {
        ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('Have a snack!')));
      },
    );
  }
}
