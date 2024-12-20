// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

class _GesturePainter extends CustomPainter {
  const _GesturePainter({
    required this.zoom,
    required this.offset,
    required this.swatch,
    required this.forward,
    required this.scaleEnabled,
    required this.tapEnabled,
    required this.doubleTapEnabled,
    required this.longPressEnabled,
  });

  final double zoom;
  final Offset offset;
  final MaterialColor swatch;
  final bool forward;
  final bool scaleEnabled;
  final bool tapEnabled;
  final bool doubleTapEnabled;
  final bool longPressEnabled;

  @override
  void paint(Canvas canvas, Size size) {
    final Offset center = size.center(Offset.zero) * zoom + offset;
    final double radius = size.width / 2.0 * zoom;
    final Gradient gradient = RadialGradient(
      colors:
          forward
              ? <Color>[swatch.shade50, swatch.shade900]
              : <Color>[swatch.shade900, swatch.shade50],
    );
    final Paint paint =
        Paint()..shader = gradient.createShader(Rect.fromCircle(center: center, radius: radius));
    canvas.drawCircle(center, radius, paint);
  }

  @override
  bool shouldRepaint(_GesturePainter oldPainter) {
    return oldPainter.zoom != zoom ||
        oldPainter.offset != offset ||
        oldPainter.swatch != swatch ||
        oldPainter.forward != forward ||
        oldPainter.scaleEnabled != scaleEnabled ||
        oldPainter.tapEnabled != tapEnabled ||
        oldPainter.doubleTapEnabled != doubleTapEnabled ||
        oldPainter.longPressEnabled != longPressEnabled;
  }
}

class GestureDemo extends StatefulWidget {
  const GestureDemo({super.key});

  @override
  GestureDemoState createState() => GestureDemoState();
}

class GestureDemoState extends State<GestureDemo> {
  late Offset _startingFocalPoint;

  late Offset _previousOffset;
  Offset _offset = Offset.zero;

  late double _previousZoom;
  double _zoom = 1.0;

  static const List<MaterialColor> kSwatches = <MaterialColor>[
    Colors.red,
    Colors.pink,
    Colors.purple,
    Colors.deepPurple,
    Colors.indigo,
    Colors.blue,
    Colors.lightBlue,
    Colors.cyan,
    Colors.green,
    Colors.lightGreen,
    Colors.lime,
    Colors.yellow,
    Colors.amber,
    Colors.orange,
    Colors.deepOrange,
    Colors.brown,
    Colors.grey,
    Colors.blueGrey,
  ];
  int _swatchIndex = 0;
  MaterialColor _swatch = kSwatches.first;
  MaterialColor get swatch => _swatch;

  bool _forward = true;
  bool _scaleEnabled = true;
  bool _tapEnabled = true;
  bool _doubleTapEnabled = true;
  bool _longPressEnabled = true;

  void _handleScaleStart(ScaleStartDetails details) {
    setState(() {
      _startingFocalPoint = details.focalPoint;
      _previousOffset = _offset;
      _previousZoom = _zoom;
    });
  }

  void _handleScaleUpdate(ScaleUpdateDetails details) {
    setState(() {
      _zoom = _previousZoom * details.scale;

      // Ensure that item under the focal point stays in the same place despite zooming
      final Offset normalizedOffset = (_startingFocalPoint - _previousOffset) / _previousZoom;
      _offset = details.focalPoint - normalizedOffset * _zoom;
    });
  }

  void _handleScaleReset() {
    setState(() {
      _zoom = 1.0;
      _offset = Offset.zero;
    });
  }

  void _handleColorChange() {
    setState(() {
      _swatchIndex += 1;
      if (_swatchIndex == kSwatches.length) {
        _swatchIndex = 0;
      }
      _swatch = kSwatches[_swatchIndex];
    });
  }

  void _handleDirectionChange() {
    setState(() {
      _forward = !_forward;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Stack(
      fit: StackFit.expand,
      children: <Widget>[
        GestureDetector(
          onScaleStart: _scaleEnabled ? _handleScaleStart : null,
          onScaleUpdate: _scaleEnabled ? _handleScaleUpdate : null,
          onTap: _tapEnabled ? _handleColorChange : null,
          onDoubleTap: _doubleTapEnabled ? _handleScaleReset : null,
          onLongPress: _longPressEnabled ? _handleDirectionChange : null,
          child: CustomPaint(
            painter: _GesturePainter(
              zoom: _zoom,
              offset: _offset,
              swatch: swatch,
              forward: _forward,
              scaleEnabled: _scaleEnabled,
              tapEnabled: _tapEnabled,
              doubleTapEnabled: _doubleTapEnabled,
              longPressEnabled: _longPressEnabled,
            ),
          ),
        ),
        Positioned(
          bottom: 0.0,
          left: 0.0,
          child: Card(
            child: Container(
              padding: const EdgeInsets.all(4.0),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: <Widget>[
                  Row(
                    children: <Widget>[
                      Checkbox(
                        value: _scaleEnabled,
                        onChanged: (bool? value) {
                          setState(() {
                            _scaleEnabled = value!;
                          });
                        },
                      ),
                      const Text('Scale'),
                    ],
                  ),
                  Row(
                    children: <Widget>[
                      Checkbox(
                        value: _tapEnabled,
                        onChanged: (bool? value) {
                          setState(() {
                            _tapEnabled = value!;
                          });
                        },
                      ),
                      const Text('Tap'),
                    ],
                  ),
                  Row(
                    children: <Widget>[
                      Checkbox(
                        value: _doubleTapEnabled,
                        onChanged: (bool? value) {
                          setState(() {
                            _doubleTapEnabled = value!;
                          });
                        },
                      ),
                      const Text('Double Tap'),
                    ],
                  ),
                  Row(
                    children: <Widget>[
                      Checkbox(
                        value: _longPressEnabled,
                        onChanged: (bool? value) {
                          setState(() {
                            _longPressEnabled = value!;
                          });
                        },
                      ),
                      const Text('Long Press'),
                    ],
                  ),
                ],
              ),
            ),
          ),
        ),
      ],
    );
  }
}

void main() {
  runApp(
    MaterialApp(
      theme: ThemeData.dark(),
      home: Scaffold(appBar: AppBar(title: const Text('Gestures Demo')), body: const GestureDemo()),
    ),
  );
}
