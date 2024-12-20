// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart';

/// Contains method name that was called on [MockEngineCanvas] and arguments
/// that were passed.
class MockCanvasCall {
  MockCanvasCall._({required this.methodName, this.arguments});

  final String methodName;
  final dynamic arguments;

  @override
  String toString() {
    return '$MockCanvasCall($methodName, $arguments)';
  }
}

/// A fake implementation of [EngineCanvas] that logs calls to its methods but
/// doesn't actually paint anything.
///
/// Useful for testing interactions between upper layers of the system with
/// canvases.
class MockEngineCanvas implements EngineCanvas {
  final List<MockCanvasCall> methodCallLog = <MockCanvasCall>[];

  @override
  DomElement get rootElement => _rootElement;
  final DomElement _rootElement = createDomHTMLDivElement();

  void _called(String methodName, {dynamic arguments}) {
    methodCallLog.add(MockCanvasCall._(methodName: methodName, arguments: arguments));
  }

  @override
  void dispose() {
    _called('dispose');
  }

  @override
  void clear() {
    _called('clear');
  }

  @override
  void save() {
    _called('save');
  }

  @override
  void restore() {
    _called('restore');
  }

  @override
  void translate(double dx, double dy) {
    _called('translate', arguments: <String, double>{'dx': dx, 'dy': dy});
  }

  @override
  void scale(double sx, double sy) {
    _called('scale', arguments: <String, double>{'sx': sx, 'sy': sy});
  }

  @override
  void rotate(double radians) {
    _called('rotate', arguments: radians);
  }

  @override
  void skew(double sx, double sy) {
    _called('skew', arguments: <String, double>{'sx': sx, 'sy': sy});
  }

  @override
  void transform(Float32List matrix4) {
    _called('transform', arguments: matrix4);
  }

  @override
  void clipRect(Rect rect, ClipOp op) {
    _called('clipRect', arguments: rect);
  }

  @override
  void clipRRect(RRect rrect) {
    _called('clipRRect', arguments: rrect);
  }

  @override
  void clipPath(Path path) {
    _called('clipPath', arguments: path);
  }

  @override
  void drawColor(Color color, BlendMode blendMode) {
    _called('drawColor', arguments: <String, dynamic>{'color': color, 'blendMode': blendMode});
  }

  @override
  void drawLine(Offset p1, Offset p2, SurfacePaintData paint) {
    _called('drawLine', arguments: <String, dynamic>{'p1': p1, 'p2': p2, 'paint': paint});
  }

  @override
  void drawPaint(SurfacePaintData paint) {
    _called('drawPaint', arguments: paint);
  }

  @override
  void drawRect(Rect rect, SurfacePaintData paint) {
    _called('drawRect', arguments: <String, dynamic>{'rect': rect, 'paint': paint});
  }

  @override
  void drawRRect(RRect rrect, SurfacePaintData paint) {
    _called('drawRRect', arguments: <String, dynamic>{'rrect': rrect, 'paint': paint});
  }

  @override
  void drawDRRect(RRect outer, RRect inner, SurfacePaintData paint) {
    _called(
      'drawDRRect',
      arguments: <String, dynamic>{'outer': outer, 'inner': inner, 'paint': paint},
    );
  }

  @override
  void drawOval(Rect rect, SurfacePaintData paint) {
    _called('drawOval', arguments: <String, dynamic>{'rect': rect, 'paint': paint});
  }

  @override
  void drawCircle(Offset c, double radius, SurfacePaintData paint) {
    _called('drawCircle', arguments: <String, dynamic>{'c': c, 'radius': radius, 'paint': paint});
  }

  @override
  void drawPath(Path path, SurfacePaintData paint) {
    _called('drawPath', arguments: <String, dynamic>{'path': path, 'paint': paint});
  }

  @override
  void drawShadow(Path path, Color color, double elevation, bool transparentOccluder) {
    _called(
      'drawShadow',
      arguments: <String, dynamic>{
        'path': path,
        'color': color,
        'elevation': elevation,
        'transparentOccluder': transparentOccluder,
      },
    );
  }

  @override
  void drawImage(Image image, Offset p, SurfacePaintData paint) {
    _called('drawImage', arguments: <String, dynamic>{'image': image, 'p': p, 'paint': paint});
  }

  @override
  void drawImageRect(Image image, Rect src, Rect dst, SurfacePaintData paint) {
    _called(
      'drawImageRect',
      arguments: <String, dynamic>{'image': image, 'src': src, 'dst': dst, 'paint': paint},
    );
  }

  @override
  void drawParagraph(Paragraph paragraph, Offset offset) {
    _called(
      'drawParagraph',
      arguments: <String, dynamic>{'paragraph': paragraph, 'offset': offset},
    );
  }

  @override
  void drawVertices(Vertices vertices, BlendMode blendMode, SurfacePaintData paint) {
    _called(
      'drawVertices',
      arguments: <String, dynamic>{'vertices': vertices, 'blendMode': blendMode, 'paint': paint},
    );
  }

  @override
  void drawPoints(PointMode pointMode, Float32List points, SurfacePaintData paint) {
    _called(
      'drawPoints',
      arguments: <String, dynamic>{'pointMode': pointMode, 'points': points, 'paint': paint},
    );
  }

  @override
  void endOfPaint() {
    _called('endOfPaint', arguments: <String, dynamic>{});
  }
}
