// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [LinearGradient].

void main() => runApp(const LinearGradientExampleApp());

class LinearGradientExampleApp extends StatelessWidget {
  const LinearGradientExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: MoodyGradient());
  }
}

class MoodyGradient extends StatelessWidget {
  const MoodyGradient({super.key});

  @override
  Widget build(BuildContext context) {
    return Material(
      child: Container(
        decoration: const BoxDecoration(
          gradient: LinearGradient(
            begin: Alignment.topLeft,
            end: Alignment(0.8, 1),
            colors: <Color>[
              Color(0xff1f005c),
              Color(0xff5b0060),
              Color(0xff870160),
              Color(0xffac255e),
              Color(0xffca485c),
              Color(0xffe16b5c),
              Color(0xfff39060),
              Color(0xffffb56b),
            ], // Gradient from https://learnui.design/tools/gradient-generator.html
            tileMode: TileMode.mirror,
          ),
        ),
        child: const Center(
          child: Text('From Night to Day', style: TextStyle(fontSize: 24, color: Colors.white)),
        ),
      ),
    );
  }
}
