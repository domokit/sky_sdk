// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [OverflowBox].

void main() => runApp(const OverflowBoxApp());

class OverflowBoxApp extends StatelessWidget {
  const OverflowBoxApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('OverflowBox Sample')),
        body: const Center(child: OverflowBoxExample()),
      ),
    );
  }
}

class OverflowBoxExample extends StatelessWidget {
  const OverflowBoxExample({super.key});

  @override
  Widget build(BuildContext context) {
    return Column(
      mainAxisAlignment: MainAxisAlignment.center,
      children: <Widget>[
        const Text('Cover Me'),
        // This parent container has fixed width and
        // height of 100 pixels.
        Container(
          width: 100,
          height: 100,
          color: Theme.of(context).colorScheme.secondaryContainer,
          // This OverflowBox imposes its own constraints of maxWidth
          // and maxHeight of 200 pixels on its child which allows the
          // child to overflow the parent container.
          child: const OverflowBox(
            maxWidth: 200,
            maxHeight: 200,
            // Without the OverflowBox, the child widget would be
            // constrained to the size of the parent container
            // and would not overflow the parent container.
            child: FlutterLogo(size: 200),
          ),
        ),
      ],
    );
  }
}
