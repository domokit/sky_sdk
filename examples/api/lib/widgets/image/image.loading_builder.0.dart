// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [Image.loadingBuilder].

void main() => runApp(const LoadingBuilderExampleApp());

class LoadingBuilderExampleApp extends StatelessWidget {
  const LoadingBuilderExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: LoadingBuilderExample());
  }
}

class LoadingBuilderExample extends StatelessWidget {
  const LoadingBuilderExample({super.key});

  @override
  Widget build(BuildContext context) {
    return DecoratedBox(
      decoration: BoxDecoration(
        color: Colors.white,
        border: Border.all(),
        borderRadius: BorderRadius.circular(20),
      ),
      child: Image.network(
        'https://flutter.github.io/assets-for-api-docs/assets/widgets/falcon.jpg',
        loadingBuilder: (BuildContext context, Widget child, ImageChunkEvent? loadingProgress) {
          if (loadingProgress == null) {
            return child;
          }
          return Center(
            child: CircularProgressIndicator(
              value:
                  loadingProgress.expectedTotalBytes != null
                      ? loadingProgress.cumulativeBytesLoaded / loadingProgress.expectedTotalBytes!
                      : null,
            ),
          );
        },
      ),
    );
  }
}
