// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

void main() async {
  runApp(const ResizeApp());
}

class ResizeApp extends StatefulWidget {
  const ResizeApp({super.key});

  static const double resizeBy = 10.0;
  static const Key heightLabel = Key('height label');
  static const Key widthLabel = Key('width label');
  static const Key extendedFab = Key('extended FAB');

  static const MethodChannel platform = MethodChannel('samples.flutter.dev/resize');

  static Future<void> resize(Size size) async {
    await ResizeApp.platform.invokeMethod<void>('resize', <String, dynamic>{
      'width': size.width,
      'height': size.height,
    });
  }

  @override
  State<ResizeApp> createState() => _ResizeAppState();
}

class _ResizeAppState extends State<ResizeApp> {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Builder(
        builder: (BuildContext context) {
          final Size currentSize = MediaQuery.of(context).size;
          return Scaffold(
            floatingActionButton: FloatingActionButton.extended(
              key: ResizeApp.extendedFab,
              label: const Text('Resize'),
              onPressed: () {
                final Size nextSize = Size(
                  currentSize.width + ResizeApp.resizeBy,
                  currentSize.height + ResizeApp.resizeBy,
                );
                ResizeApp.resize(nextSize);
              },
            ),
            body: Center(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: <Widget>[
                  Text(key: ResizeApp.widthLabel, 'width: ${currentSize.width}'),
                  Text(key: ResizeApp.heightLabel, 'height: ${currentSize.height}'),
                ],
              ),
            ),
          );
        },
      ),
    );
  }
}
