// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [ColorFiltered].

void main() => runApp(const ColorFilteredExampleApp());

class ColorFilteredExampleApp extends StatelessWidget {
  const ColorFilteredExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('ColorFiltered Sample')),
        body: const ColorFilteredExample(),
      ),
    );
  }
}

class ColorFilteredExample extends StatelessWidget {
  const ColorFilteredExample({super.key});

  @override
  Widget build(BuildContext context) {
    return SingleChildScrollView(
      child: Column(
        children: <Widget>[
          ColorFiltered(
            colorFilter: const ColorFilter.mode(Colors.red, BlendMode.modulate),
            child: Image.network(
              'https://flutter.github.io/assets-for-api-docs/assets/widgets/owl-2.jpg',
            ),
          ),
          ColorFiltered(
            colorFilter: const ColorFilter.mode(Colors.grey, BlendMode.saturation),
            child: Image.network(
              'https://flutter.github.io/assets-for-api-docs/assets/widgets/owl.jpg',
            ),
          ),
        ],
      ),
    );
  }
}
