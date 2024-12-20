// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [PhysicalShape].

void main() => runApp(const PhysicalShapeApp());

class PhysicalShapeApp extends StatelessWidget {
  const PhysicalShapeApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('PhysicalShape Sample')),
        body: const Center(child: PhysicalShapeExample()),
      ),
    );
  }
}

class PhysicalShapeExample extends StatelessWidget {
  const PhysicalShapeExample({super.key});

  @override
  Widget build(BuildContext context) {
    return PhysicalShape(
      elevation: 5.0,
      clipper: ShapeBorderClipper(
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10.0)),
      ),
      color: Colors.orange,
      child: const SizedBox(
        height: 200.0,
        width: 200.0,
        child: Center(
          child: Text('Hello, World!', style: TextStyle(color: Colors.white, fontSize: 20.0)),
        ),
      ),
    );
  }
}
