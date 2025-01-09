// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

void main() {
  runApp(const DragBoundaryExampleApp());
}

class DragBoundaryExampleApp extends StatefulWidget {
  const DragBoundaryExampleApp({super.key});

  @override
  State<StatefulWidget> createState() => DragBoundaryExampleAppState();
}

class DragBoundaryExampleAppState extends State<DragBoundaryExampleApp> {
  Offset _currentPosition = Offset.zero;
  Offset _initialPosition = Offset.zero;
  final Size _boxSize = const Size(100, 100);
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        body: Padding(
          padding: const EdgeInsets.all(100),
          child: DragBoundary(
            child: Builder(
              builder: (BuildContext context) {
                return Stack(
                  children: <Widget>[
                    Container(color: Colors.green),
                    Positioned(
                      top: _currentPosition.dy,
                      left: _currentPosition.dx,
                      child: GestureDetector(
                        behavior: HitTestBehavior.translucent,
                        onPanStart: (DragStartDetails details) {
                          _initialPosition = details.localPosition - _currentPosition;
                        },
                        onPanUpdate: (DragUpdateDetails details) {
                          _currentPosition = details.localPosition - _initialPosition;
                          final Rect withinBoundary = DragBoundary.forRectOf(
                            context,
                            useGlobalPosition: false,
                          ).nearestPositionWithinBoundary(_currentPosition & _boxSize);
                          setState(() {
                            _currentPosition = withinBoundary.topLeft;
                          });
                        },
                        child: Container(
                          width: _boxSize.width,
                          height: _boxSize.height,
                          color: Colors.red,
                        ),
                      ),
                    ),
                  ],
                );
              },
            ),
          ),
        ),
      ),
    );
  }
}
