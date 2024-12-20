// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [InkWell].

void main() => runApp(const InkWellExampleApp());

class InkWellExampleApp extends StatelessWidget {
  const InkWellExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('InkWell Sample')),
        body: const Center(child: InkWellExample()),
      ),
    );
  }
}

class InkWellExample extends StatefulWidget {
  const InkWellExample({super.key});

  @override
  State<InkWellExample> createState() => _InkWellExampleState();
}

class _InkWellExampleState extends State<InkWellExample> {
  double sideLength = 50;

  @override
  Widget build(BuildContext context) {
    return AnimatedContainer(
      height: sideLength,
      width: sideLength,
      duration: const Duration(seconds: 2),
      curve: Curves.easeIn,
      child: Material(
        color: Colors.yellow,
        child: InkWell(
          onTap: () {
            setState(() {
              sideLength == 50 ? sideLength = 100 : sideLength = 50;
            });
          },
        ),
      ),
    );
  }
}
