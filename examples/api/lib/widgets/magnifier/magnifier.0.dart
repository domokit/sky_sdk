// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

void main() => runApp(const MagnifierExampleApp());

class MagnifierExampleApp extends StatefulWidget {
  const MagnifierExampleApp({super.key});

  @override
  State<MagnifierExampleApp> createState() => _MagnifierExampleAppState();
}

class _MagnifierExampleAppState extends State<MagnifierExampleApp> {
  static const double magnifierRadius = 50.0;
  Offset dragGesturePosition = const Offset(100, 100);

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        body: Center(
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[
              const Text('Drag on the logo!'),
              RepaintBoundary(
                child: Stack(
                  children: <Widget>[
                    GestureDetector(
                      onPanUpdate:
                          (DragUpdateDetails details) => setState(() {
                            dragGesturePosition = details.localPosition;
                          }),
                      onPanDown:
                          (DragDownDetails details) => setState(() {
                            dragGesturePosition = details.localPosition;
                          }),
                      child: const FlutterLogo(size: 200),
                    ),
                    Positioned(
                      left: dragGesturePosition.dx - magnifierRadius,
                      top: dragGesturePosition.dy - magnifierRadius,
                      child: const RawMagnifier(
                        decoration: MagnifierDecoration(
                          shape: CircleBorder(side: BorderSide(color: Colors.pink, width: 3)),
                        ),
                        size: Size(magnifierRadius * 2, magnifierRadius * 2),
                        magnificationScale: 2,
                      ),
                    ),
                  ],
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
