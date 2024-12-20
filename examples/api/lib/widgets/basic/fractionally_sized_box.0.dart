// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [FractionallySizedBox].

void main() => runApp(const FractionallySizedBoxApp());

class FractionallySizedBoxApp extends StatelessWidget {
  const FractionallySizedBoxApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('FractionallySizedBox Sample')),
        body: const FractionallySizedBoxExample(),
      ),
    );
  }
}

class FractionallySizedBoxExample extends StatelessWidget {
  const FractionallySizedBoxExample({super.key});

  @override
  Widget build(BuildContext context) {
    return SizedBox.expand(
      child: FractionallySizedBox(
        widthFactor: 0.5,
        heightFactor: 0.5,
        alignment: FractionalOffset.center,
        child: DecoratedBox(
          decoration: BoxDecoration(border: Border.all(color: Colors.blue, width: 4)),
        ),
      ),
    );
  }
}
