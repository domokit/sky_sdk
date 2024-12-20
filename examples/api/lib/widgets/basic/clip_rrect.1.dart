// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [ClipRRect].

void main() => runApp(const ClipRRectApp());

class ClipRRectApp extends StatelessWidget {
  const ClipRRectApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('ClipRRect Sample')),
        body: const ClipRRectExample(),
      ),
    );
  }
}

class ClipRRectExample extends StatelessWidget {
  const ClipRRectExample({super.key});

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(40.0),
      constraints: const BoxConstraints.expand(),
      // Add a FittedBox to make ClipRRect sized accordingly to the image it contains
      child: FittedBox(
        child: ClipRRect(borderRadius: BorderRadius.circular(40.0), child: const _FakedImage()),
      ),
    );
  }
}

// A widget exposing the FlutterLogo as a 400x400 image.
//
// It can be replaced by a NetworkImage if internet connection is available, e.g. :
// const Image(
//   image: NetworkImage(
//       'https://flutter.github.io/assets-for-api-docs/assets/widgets/owl.jpg'),
// );
class _FakedImage extends StatelessWidget {
  const _FakedImage();

  @override
  Widget build(BuildContext context) {
    return Container(
      // Set constraints as if it were a 400x400 image
      constraints: BoxConstraints.tight(const Size(400, 400)),
      color: Colors.blueGrey,
      child: const FlutterLogo(),
    );
  }
}
