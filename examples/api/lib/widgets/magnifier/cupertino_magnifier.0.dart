// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

/// Flutter code sample for [CupertinoMagnifier].

void main() => runApp(const CupertinoMagnifierApp());

class CupertinoMagnifierApp extends StatelessWidget {
  const CupertinoMagnifierApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const CupertinoApp(
      theme: CupertinoThemeData(brightness: Brightness.light),
      home: CupertinoMagnifierExample(),
    );
  }
}

class CupertinoMagnifierExample extends StatefulWidget {
  const CupertinoMagnifierExample({super.key});

  @override
  State<CupertinoMagnifierExample> createState() => _CupertinoMagnifierExampleState();
}

class _CupertinoMagnifierExampleState extends State<CupertinoMagnifierExample> {
  static const double magnifierRadius = 50.0;
  Offset dragGesturePosition = Offset.zero;

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: const CupertinoNavigationBar(middle: Text('CupertinoMagnifier Sample')),
      child: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            const Text('Drag on the logo!'),
            RepaintBoundary(
              child: Stack(
                children: <Widget>[
                  GestureDetector(
                    onPanUpdate: (DragUpdateDetails details) {
                      setState(() {
                        dragGesturePosition = details.localPosition;
                      });
                    },
                    onPanDown: (DragDownDetails details) {
                      setState(() {
                        dragGesturePosition = details.localPosition;
                      });
                    },
                    child: const FlutterLogo(size: 200),
                  ),
                  Positioned(
                    left: dragGesturePosition.dx - magnifierRadius,
                    top: dragGesturePosition.dy - magnifierRadius,
                    child: const CupertinoMagnifier(
                      magnificationScale: 1.5,
                      borderRadius: BorderRadius.all(Radius.circular(magnifierRadius)),
                      additionalFocalPointOffset: Offset(0, -magnifierRadius),
                    ),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }
}
