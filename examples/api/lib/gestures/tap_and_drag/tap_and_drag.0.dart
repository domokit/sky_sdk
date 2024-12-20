// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';

/// Flutter code sample for [TapAndPanGestureRecognizer].

void main() {
  runApp(const TapAndDragToZoomApp());
}

class TapAndDragToZoomApp extends StatelessWidget {
  const TapAndDragToZoomApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(
      home: Scaffold(body: Center(child: TapAndDragToZoomWidget(child: MyBoxWidget()))),
    );
  }
}

class MyBoxWidget extends StatelessWidget {
  const MyBoxWidget({super.key});

  @override
  Widget build(BuildContext context) {
    return Container(color: Colors.blueAccent, height: 100.0, width: 100.0);
  }
}

// This widget will scale its child up when it detects a drag up, after a
// double tap/click. It will scale the widget down when it detects a drag down,
// after a double tap. Dragging down and then up after a double tap/click will
// zoom the child in/out. The scale of the child will be reset when the drag ends.
class TapAndDragToZoomWidget extends StatefulWidget {
  const TapAndDragToZoomWidget({super.key, required this.child});

  final Widget child;

  @override
  State<TapAndDragToZoomWidget> createState() => _TapAndDragToZoomWidgetState();
}

class _TapAndDragToZoomWidgetState extends State<TapAndDragToZoomWidget> {
  final double scaleMultiplier = -0.0001;
  double _currentScale = 1.0;
  Offset? _previousDragPosition;

  static double _keepScaleWithinBounds(double scale) {
    const double minScale = 0.1;
    const double maxScale = 30;
    if (scale <= 0) {
      return minScale;
    }
    if (scale >= 30) {
      return maxScale;
    }
    return scale;
  }

  void _zoomLogic(Offset currentDragPosition) {
    final double dx = (_previousDragPosition!.dx - currentDragPosition.dx).abs();
    final double dy = (_previousDragPosition!.dy - currentDragPosition.dy).abs();

    if (dx > dy) {
      // Ignore horizontal drags.
      _previousDragPosition = currentDragPosition;
      return;
    }

    if (currentDragPosition.dy < _previousDragPosition!.dy) {
      // Zoom out on drag up.
      setState(() {
        _currentScale += currentDragPosition.dy * scaleMultiplier;
        _currentScale = _keepScaleWithinBounds(_currentScale);
      });
    } else {
      // Zoom in on drag down.
      setState(() {
        _currentScale -= currentDragPosition.dy * scaleMultiplier;
        _currentScale = _keepScaleWithinBounds(_currentScale);
      });
    }
    _previousDragPosition = currentDragPosition;
  }

  @override
  Widget build(BuildContext context) {
    return RawGestureDetector(
      gestures: <Type, GestureRecognizerFactory>{
        TapAndPanGestureRecognizer:
            GestureRecognizerFactoryWithHandlers<TapAndPanGestureRecognizer>(
              () => TapAndPanGestureRecognizer(),
              (TapAndPanGestureRecognizer instance) {
                instance
                  ..onTapDown = (TapDragDownDetails details) {
                    _previousDragPosition = details.globalPosition;
                  }
                  ..onDragStart = (TapDragStartDetails details) {
                    if (details.consecutiveTapCount == 2) {
                      _zoomLogic(details.globalPosition);
                    }
                  }
                  ..onDragUpdate = (TapDragUpdateDetails details) {
                    if (details.consecutiveTapCount == 2) {
                      _zoomLogic(details.globalPosition);
                    }
                  }
                  ..onDragEnd = (TapDragEndDetails details) {
                    if (details.consecutiveTapCount == 2) {
                      setState(() {
                        _currentScale = 1.0;
                      });
                      _previousDragPosition = null;
                    }
                  };
              },
            ),
      },
      child: Transform.scale(scale: _currentScale, child: widget.child),
    );
  }
}
