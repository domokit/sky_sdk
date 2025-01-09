// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' hide TextStyle;

import 'package:web_engine_tester/golden_tester.dart';

import '../common/test_initialization.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  const double screenWidth = 600.0;
  const double screenHeight = 800.0;
  const Rect screenRect = Rect.fromLTWH(0, 0, screenWidth, screenHeight);
  final SurfacePaint testPaint =
      SurfacePaint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2.0
        ..color = const Color(0xFFFF00FF);

  setUpUnitTests();

  // Regression test for https://github.com/flutter/flutter/issues/51514
  test("Canvas is reused and z-index doesn't leak across paints", () async {
    final EngineCanvas engineCanvas = BitmapCanvas(screenRect, RenderStrategy());
    const Rect region = Rect.fromLTWH(0, 0, 500, 500);

    // Draw first frame into engine canvas.
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTWH(1, 2, 300, 400));
    final Path path =
        Path()
          ..moveTo(3, 0)
          ..lineTo(100, 97);
    rc.drawPath(path, testPaint);
    rc.endRecording();
    rc.apply(engineCanvas, screenRect);
    engineCanvas.endOfPaint();

    DomElement sceneElement = createDomElement('flt-scene');
    if (isIosSafari) {
      // Shrink to fit on the iPhone screen.
      sceneElement.style.position = 'absolute';
      sceneElement.style.transformOrigin = '0 0 0';
      sceneElement.style.transform = 'scale(0.3)';
    }
    sceneElement.append(engineCanvas.rootElement);
    domDocument.body!.append(sceneElement);

    final DomCanvasElement canvas = domDocument.querySelector('canvas')! as DomCanvasElement;
    // ! Since canvas is first element, it should have zIndex = -1 for correct
    // paint order.
    expect(canvas.style.zIndex, '-1');

    // Add id to canvas element to test for reuse.
    const String kTestId = 'test-id-5698';
    canvas.id = kTestId;

    sceneElement.remove();
    // Clear so resources are marked for reuse.

    engineCanvas.clear();

    // Now paint a second scene to same [BitmapCanvas] but paint an image
    // before the path to move canvas element into second position.
    final RecordingCanvas rc2 = RecordingCanvas(const Rect.fromLTWH(1, 2, 300, 400));
    final Path path2 =
        Path()
          ..moveTo(3, 0)
          ..quadraticBezierTo(100, 0, 100, 100);
    rc2.drawImage(_createRealTestImage(), Offset.zero, SurfacePaint());
    rc2.drawPath(path2, testPaint);
    rc2.endRecording();
    rc2.apply(engineCanvas, screenRect);

    sceneElement = createDomElement('flt-scene');
    if (isIosSafari) {
      // Shrink to fit on the iPhone screen.
      sceneElement.style.position = 'absolute';
      sceneElement.style.transformOrigin = '0 0 0';
      sceneElement.style.transform = 'scale(0.3)';
    }
    sceneElement.append(engineCanvas.rootElement);
    domDocument.body!.append(sceneElement);

    final DomCanvasElement canvas2 = domDocument.querySelector('canvas')! as DomCanvasElement;
    // ZIndex should have been cleared since we have image element preceding
    // canvas.
    expect(canvas.style.zIndex != '-1', isTrue);
    expect(canvas2.id, kTestId);
    await matchGoldenFile('bitmap_canvas_reuse_zindex.png', region: region);
  });
}

const String _base64Encoded20x20TestImage =
    'iVBORw0KGgoAAAANSUhEUgAAABQAAAAUCAIAAAAC64paAAAACXBIWXMAAC4jAAAuIwF4pT92AAAA'
    'B3RJTUUH5AMFFBksg4i3gQAAABl0RVh0Q29tbWVudABDcmVhdGVkIHdpdGggR0lNUFeBDhcAAAAj'
    'SURBVDjLY2TAC/7jlWVioACMah4ZmhnxpyHG0QAb1UyZZgBjWAIm/clP0AAAAABJRU5ErkJggg==';

HtmlImage _createRealTestImage() {
  return HtmlImage(
    createDomHTMLImageElement()..src = 'data:text/plain;base64,$_base64Encoded20x20TestImage',
    20,
    20,
  );
}
