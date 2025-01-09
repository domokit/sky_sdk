// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [Image.frameBuilder].

void main() => runApp(const FrameBuilderExampleApp());

class FrameBuilderExampleApp extends StatelessWidget {
  const FrameBuilderExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: FrameBuilderExample());
  }
}

class FrameBuilderExample extends StatelessWidget {
  const FrameBuilderExample({super.key});

  @override
  @override
  Widget build(BuildContext context) {
    return DecoratedBox(
      decoration: BoxDecoration(
        color: Colors.white,
        border: Border.all(),
        borderRadius: BorderRadius.circular(20),
      ),
      child: Image.network(
        'https://flutter.github.io/assets-for-api-docs/assets/widgets/puffin.jpg',
        frameBuilder: (
          BuildContext context,
          Widget child,
          int? frame,
          bool wasSynchronouslyLoaded,
        ) {
          if (wasSynchronouslyLoaded) {
            return child;
          }
          return AnimatedOpacity(
            opacity: frame == null ? 0 : 1,
            duration: const Duration(seconds: 1),
            curve: Curves.easeOut,
            child: child,
          );
        },
      ),
    );
  }
}
