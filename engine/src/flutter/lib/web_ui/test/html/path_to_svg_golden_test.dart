// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart';

import 'package:web_engine_tester/golden_tester.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

enum PaintMode { kStrokeAndFill, kStroke, kFill, kStrokeWidthOnly }

Future<void> testMain() async {
  const Rect region = Rect.fromLTWH(8, 8, 600, 400); // Compensate for old golden tester padding

  Future<void> testPath(
    Path path,
    String goldenFileName, {
    SurfacePaint? paint,
    PaintMode mode = PaintMode.kStrokeAndFill,
  }) async {
    const Rect canvasBounds = Rect.fromLTWH(0, 0, 600, 400);
    final BitmapCanvas bitmapCanvas = BitmapCanvas(canvasBounds, RenderStrategy());
    final RecordingCanvas canvas = RecordingCanvas(canvasBounds);

    final bool enableFill = mode == PaintMode.kStrokeAndFill || mode == PaintMode.kFill;
    if (enableFill) {
      paint ??=
          SurfacePaint()
            ..color = const Color(0x807F7F7F)
            ..style = PaintingStyle.fill;
      canvas.drawPath(path, paint);
    }

    if (mode == PaintMode.kStrokeAndFill || mode == PaintMode.kStroke) {
      paint =
          SurfacePaint()
            ..strokeWidth = 2
            ..color = enableFill ? const Color(0xFFFF0000) : const Color(0xFF000000)
            ..style = PaintingStyle.stroke;
    }

    if (mode == PaintMode.kStrokeWidthOnly) {
      paint =
          SurfacePaint()
            ..color = const Color(0xFF4060E0)
            ..strokeWidth = 10;
    }

    canvas.drawPath(path, paint!);

    final DomElement svgElement = pathToSvgElement(path, paint, enableFill);

    canvas.endRecording();
    canvas.apply(bitmapCanvas, canvasBounds);

    final DomElement sceneElement = createDomElement('flt-scene');
    domDocument.body!.append(sceneElement);
    if (isIosSafari) {
      // Shrink to fit on the iPhone screen.
      sceneElement.style.position = 'absolute';
      sceneElement.style.transformOrigin = '0 0 0';
      sceneElement.style.transform = 'scale(0.3)';
    }
    sceneElement.append(bitmapCanvas.rootElement);
    sceneElement.append(svgElement);

    await matchGoldenFile('$goldenFileName.png', region: region);

    bitmapCanvas.rootElement.remove();
    svgElement.remove();
  }

  tearDown(() {
    domDocument.body!.clearChildren();
  });

  test('render line strokes', () async {
    final Path path = Path();
    path.moveTo(50, 60);
    path.lineTo(200, 300);
    await testPath(
      path,
      'svg_stroke_line',
      paint:
          SurfacePaint()
            ..color = const Color(0xFFFF0000)
            ..strokeWidth = 2.0
            ..style = PaintingStyle.stroke,
    );
  });

  test('render quad bezier curve', () async {
    final Path path = Path();
    path.moveTo(50, 60);
    path.quadraticBezierTo(200, 60, 50, 200);
    await testPath(path, 'svg_quad_bezier');
  });

  test('render cubic curve', () async {
    final Path path = Path();
    path.moveTo(50, 60);
    path.cubicTo(200, 60, -100, -50, 150, 200);
    await testPath(path, 'svg_cubic_bezier');
  });

  test('render arcs', () async {
    final List<ArcSample> arcs = <ArcSample>[
      ArcSample(Offset.zero, distance: 20),
      ArcSample(const Offset(200, 0), largeArc: true, distance: 20),
      ArcSample(Offset.zero, clockwise: true, distance: 20),
      ArcSample(const Offset(200, 0), largeArc: true, clockwise: true, distance: 20),
      ArcSample(Offset.zero, distance: -20),
      ArcSample(const Offset(200, 0), largeArc: true, distance: -20),
      ArcSample(Offset.zero, clockwise: true, distance: -20),
      ArcSample(const Offset(200, 0), largeArc: true, clockwise: true, distance: -20),
    ];
    int sampleIndex = 0;
    for (final ArcSample sample in arcs) {
      ++sampleIndex;
      final Path path = sample.createPath();
      await testPath(path, 'svg_arc_$sampleIndex');
    }
  });

  test('render rect', () async {
    final Path path = Path();
    path.addRect(const Rect.fromLTRB(15, 15, 60, 20));
    path.addRect(const Rect.fromLTRB(35, 160, 15, 100));
    await testPath(path, 'svg_rect');
  });

  test('render notch', () async {
    final Path path = Path();
    path.moveTo(0, 0);
    path.lineTo(83, 0);
    path.quadraticBezierTo(98, 0, 99.97, 7.8);
    path.arcToPoint(const Offset(162, 7.8), radius: const Radius.circular(32), clockwise: false);
    path.lineTo(200, 7.8);
    path.lineTo(200, 80);
    path.lineTo(0, 80);
    path.lineTo(0, 10);
    await testPath(path, 'svg_notch');
  });

  /// Regression test for https://github.com/flutter/flutter/issues/70980
  test('render notch', () async {
    const double w = 0.7;
    final Path path = Path();
    path.moveTo(0.5, 14);
    path.conicTo(0.5, 10.5, 4, 10.5, w);
    path.moveTo(4, 10.5);
    path.lineTo(6.5, 10.5);
    path.moveTo(36.0, 10.5);
    path.lineTo(158, 10.5);
    path.conicTo(161.5, 10.5, 161.5, 14, w);
    path.moveTo(161.5, 14);
    path.lineTo(161.5, 48);
    path.conicTo(161.5, 51.5, 158, 51.5, w);
    path.lineTo(4, 51.5);
    path.conicTo(0.5, 51.5, 0.5, 48, w);
    path.lineTo(0.5, 14);
    await testPath(path, 'svg_editoutline', mode: PaintMode.kStroke);
  });

  /// Regression test for https://github.com/flutter/flutter/issues/74416
  test('render stroke', () async {
    final Path path = Path();
    path.moveTo(20, 20);
    path.lineTo(200, 200);
    await testPath(path, 'svg_stroke_width', mode: PaintMode.kStrokeWidthOnly);
  });
}

DomElement pathToSvgElement(Path path, Paint paint, bool enableFill) {
  final Rect bounds = path.getBounds();
  final SVGSVGElement root = createSVGSVGElement();
  root.style.transform = 'translate(200px, 0px)';
  root.setAttribute('viewBox', '0 0 ${bounds.right} ${bounds.bottom}');
  root.width!.baseVal!.newValueSpecifiedUnits(svgLengthTypeNumber, bounds.right);
  root.height!.baseVal!.newValueSpecifiedUnits(svgLengthTypeNumber, bounds.bottom);

  final SVGPathElement pathElement = createSVGPathElement();
  root.append(pathElement);
  if (paint.style == PaintingStyle.stroke || paint.strokeWidth != 0.0) {
    pathElement.setAttribute('stroke', paint.color.toCssString());
    pathElement.setAttribute('stroke-width', paint.strokeWidth);
    if (!enableFill) {
      pathElement.setAttribute('fill', 'none');
    }
  }
  if (paint.style == PaintingStyle.fill) {
    pathElement.setAttribute('fill', paint.color.toCssString());
  }
  pathElement.setAttribute(
    'd',
    pathToSvg((path as SurfacePath).pathRef),
  ); // This is what we're testing!
  return root;
}

class ArcSample {
  ArcSample(this.offset, {this.largeArc = false, this.clockwise = false, this.distance = 0});

  final Offset offset;
  final bool largeArc;
  final bool clockwise;
  final double distance;

  Path createPath() {
    final Offset startP = Offset(75 - distance + offset.dx, 75 - distance + offset.dy);
    final Offset endP = Offset(75.0 + distance + offset.dx, 75.0 + distance + offset.dy);
    final Path path = Path();
    path.moveTo(startP.dx, startP.dy);
    path.arcToPoint(
      endP,
      rotation: 60,
      radius: const Radius.elliptical(40, 60),
      largeArc: largeArc,
      clockwise: clockwise,
    );
    return path;
  }
}
