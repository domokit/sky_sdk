// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/material.dart';

void main() {
  runApp(const MyApp());
}

final class MyApp extends StatefulWidget {
  const MyApp({super.key});

  @override
  State<MyApp> createState() => _MyAppState();
}

class _MyAppState extends State<MyApp> {
  @override
  Widget build(BuildContext context) {
    final List<DisplayFeature> displayFeatures = MediaQuery.of(context).displayFeatures;
    displayFeatures.retainWhere(
      (DisplayFeature feature) => feature.type == DisplayFeatureType.cutout,
    );
    String text;
    // None of this complexity is required for the test but it helps when
    // visually debugging or watching a video of a remote device.
    if (displayFeatures.isEmpty) {
      text = 'CutoutNone';
    } else if (displayFeatures.length > 1) {
      text = 'CutoutMany';
    } else {
      final Rect cutout = displayFeatures[0].bounds;
      if (cutout.top == 0) {
        text = 'CutoutTop';
      } else if (cutout.left == 0) {
        text = 'CutoutLeft';
      } else {
        text = 'CutoutNeither';
      }
    }
    // Tests assume there is some text element displayed.
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      home: Text('Cutout status: $text', key: Key(text)),
    );
  }
}
