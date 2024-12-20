// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart';

import '../common/mock_engine_canvas.dart';
import '../common/test_initialization.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  setUpUnitTests(withImplicitView: true, setUpTestViewDimensions: false);

  late RecordingCanvas underTest;
  late MockEngineCanvas mockCanvas;
  const Rect screenRect = Rect.largest;

  setUp(() {
    underTest = RecordingCanvas(screenRect);
    mockCanvas = MockEngineCanvas();
  });

  group('paragraph bounds', () {
    Paragraph paragraphForBoundsTest(TextAlign alignment) {
      final ParagraphBuilder builder = ParagraphBuilder(
        ParagraphStyle(fontFamily: 'Ahem', fontSize: 20, textAlign: alignment),
      );
      builder.addText('A AAAAA AAA');
      return builder.build();
    }

    test('not laid out', () {
      final Paragraph paragraph = paragraphForBoundsTest(TextAlign.start);
      underTest.drawParagraph(paragraph, Offset.zero);
      underTest.endRecording();
      expect(underTest.pictureBounds, Rect.zero);
    });

    test('finite width', () {
      final Paragraph paragraph = paragraphForBoundsTest(TextAlign.start);
      paragraph.layout(const ParagraphConstraints(width: 110));
      underTest.drawParagraph(paragraph, Offset.zero);
      underTest.endRecording();
      expect(paragraph.width, 110);
      expect(paragraph.height, 60);
      expect(underTest.pictureBounds, const Rect.fromLTRB(0, 0, 100, 60));
    });

    test('finite width center-aligned', () {
      final Paragraph paragraph = paragraphForBoundsTest(TextAlign.center);
      paragraph.layout(const ParagraphConstraints(width: 110));
      underTest.drawParagraph(paragraph, Offset.zero);
      underTest.endRecording();
      expect(paragraph.width, 110);
      expect(paragraph.height, 60);
      expect(underTest.pictureBounds, const Rect.fromLTRB(5, 0, 105, 60));
    });

    test('infinite width', () {
      final Paragraph paragraph = paragraphForBoundsTest(TextAlign.start);
      paragraph.layout(const ParagraphConstraints(width: double.infinity));
      underTest.drawParagraph(paragraph, Offset.zero);
      underTest.endRecording();
      expect(paragraph.width, double.infinity);
      expect(paragraph.height, 20);
      expect(underTest.pictureBounds, const Rect.fromLTRB(0, 0, 220, 20));
    });
  });

  group('drawDRRect', () {
    final RRect rrect = RRect.fromLTRBR(10, 10, 50, 50, const Radius.circular(3));
    final SurfacePaint somePaint = SurfacePaint()..color = const Color(0xFFFF0000);

    test('Happy case', () {
      underTest.drawDRRect(rrect, rrect.deflate(1), somePaint);
      underTest.endRecording();
      underTest.apply(mockCanvas, screenRect);

      _expectDrawDRRectCall(mockCanvas, <String, dynamic>{
        'path':
            'Path('
            'MoveTo(${10.0}, ${47.0}) '
            'LineTo(${10.0}, ${13.0}) '
            'Conic(${10.0}, ${10.0}, ${10.0}, ${13.0}, w = ${0.7071067690849304}) '
            'LineTo(${47.0}, ${10.0}) '
            'Conic(${50.0}, ${10.0}, ${10.0}, ${50.0}, w = ${0.7071067690849304}) '
            'LineTo(${50.0}, ${47.0}) '
            'Conic(${50.0}, ${50.0}, ${50.0}, ${47.0}, w = ${0.7071067690849304}) '
            'LineTo(${13.0}, ${50.0}) '
            'Conic(${10.0}, ${50.0}, ${50.0}, ${10.0}, w = ${0.7071067690849304}) '
            'Close() '
            'MoveTo(${11.0}, ${47.0}) '
            'LineTo(${11.0}, ${13.0}) '
            'Conic(${11.0}, ${11.0}, ${11.0}, ${13.0}, w = ${0.7071067690849304}) '
            'LineTo(${47.0}, ${11.0}) '
            'Conic(${49.0}, ${11.0}, ${11.0}, ${49.0}, w = ${0.7071067690849304}) '
            'LineTo(${49.0}, ${47.0}) '
            'Conic(${49.0}, ${49.0}, ${49.0}, ${47.0}, w = ${0.7071067690849304}) '
            'LineTo(${13.0}, ${49.0}) '
            'Conic(${11.0}, ${49.0}, ${49.0}, ${11.0}, w = ${0.7071067690849304}) '
            'Close()'
            ')',
        'paint': somePaint.paintData,
      });
    });

    test('Inner RRect > Outer RRect', () {
      underTest.drawDRRect(rrect, rrect.inflate(1), somePaint);
      underTest.endRecording();
      underTest.apply(mockCanvas, screenRect);
      // Expect nothing to be called
      expect(mockCanvas.methodCallLog.length, equals(1));
      expect(mockCanvas.methodCallLog.single.methodName, 'endOfPaint');
    });

    test('Inner RRect not completely inside Outer RRect', () {
      underTest.drawDRRect(rrect, rrect.deflate(1).shift(const Offset(0.0, 10)), somePaint);
      underTest.endRecording();
      underTest.apply(mockCanvas, screenRect);
      // Expect nothing to be called
      expect(mockCanvas.methodCallLog.length, equals(1));
      expect(mockCanvas.methodCallLog.single.methodName, 'endOfPaint');
    });

    test('Inner RRect same as Outer RRect', () {
      underTest.drawDRRect(rrect, rrect, somePaint);
      underTest.endRecording();
      underTest.apply(mockCanvas, screenRect);
      // Expect nothing to be called
      expect(mockCanvas.methodCallLog.length, equals(1));
      expect(mockCanvas.methodCallLog.single.methodName, 'endOfPaint');
    });

    test('deflated corners in inner RRect get passed through to draw', () {
      // This comes from github issue #40728
      final RRect outer = RRect.fromRectAndCorners(
        const Rect.fromLTWH(0, 0, 88, 48),
        topLeft: const Radius.circular(6),
        bottomLeft: const Radius.circular(6),
      );
      final RRect inner = outer.deflate(1);

      expect(inner.brRadius, equals(Radius.zero));
      expect(inner.trRadius, equals(Radius.zero));

      underTest.drawDRRect(outer, inner, somePaint);
      underTest.endRecording();
      underTest.apply(mockCanvas, screenRect);

      // Expect to draw, even when inner has negative radii (which get ignored by canvas)
      _expectDrawDRRectCall(mockCanvas, <String, dynamic>{
        'path':
            'Path('
            'MoveTo(${0.0}, ${42.0}) '
            'LineTo(${0.0}, ${6.0}) '
            'Conic(${0.0}, ${0.0}, ${0.0}, ${6.0}, w = ${0.7071067690849304}) '
            'LineTo(${88.0}, ${0.0}) '
            'Conic(${88.0}, ${0.0}, ${0.0}, ${88.0}, w = ${0.7071067690849304}) '
            'LineTo(${88.0}, ${48.0}) '
            'Conic(${88.0}, ${48.0}, ${48.0}, ${88.0}, w = ${0.7071067690849304}) '
            'LineTo(${6.0}, ${48.0}) '
            'Conic(${0.0}, ${48.0}, ${48.0}, ${0.0}, w = ${0.7071067690849304}) '
            'Close() '
            'MoveTo(${1.0}, ${42.0}) '
            'LineTo(${1.0}, ${6.0}) '
            'Conic(${1.0}, ${1.0}, ${1.0}, ${6.0}, w = ${0.7071067690849304}) '
            'LineTo(${87.0}, ${1.0}) '
            'Conic(${87.0}, ${1.0}, ${1.0}, ${87.0}, w = ${0.7071067690849304}) '
            'LineTo(${87.0}, ${47.0}) '
            'Conic(${87.0}, ${47.0}, ${47.0}, ${87.0}, w = ${0.7071067690849304}) '
            'LineTo(${6.0}, ${47.0}) '
            'Conic(${1.0}, ${47.0}, ${47.0}, ${1.0}, w = ${0.7071067690849304}) '
            'Close()'
            ')',
        'paint': somePaint.paintData,
      });
    });

    test('preserve old golden test behavior', () {
      final RRect outer = RRect.fromRectAndCorners(const Rect.fromLTRB(10, 20, 30, 40));
      final RRect inner = RRect.fromRectAndCorners(const Rect.fromLTRB(12, 22, 28, 38));

      underTest.drawDRRect(outer, inner, somePaint);
      underTest.endRecording();
      underTest.apply(mockCanvas, screenRect);

      _expectDrawDRRectCall(mockCanvas, <String, dynamic>{
        'path':
            'Path('
            'MoveTo(${10.0}, ${20.0}) '
            'LineTo(${30.0}, ${20.0}) '
            'LineTo(${30.0}, ${40.0}) '
            'LineTo(${10.0}, ${40.0}) '
            'Close() '
            'MoveTo(${12.0}, ${22.0}) '
            'LineTo(${28.0}, ${22.0}) '
            'LineTo(${28.0}, ${38.0}) '
            'LineTo(${12.0}, ${38.0}) '
            'Close()'
            ')',
        'paint': somePaint.paintData,
      });
    });
  });

  test('Filters out paint commands outside the clip rect', () {
    // Outside to the left
    underTest.drawRect(const Rect.fromLTWH(0.0, 20.0, 10.0, 10.0), SurfacePaint());

    // Outside above
    underTest.drawRect(const Rect.fromLTWH(20.0, 0.0, 10.0, 10.0), SurfacePaint());

    // Visible
    underTest.drawRect(const Rect.fromLTWH(20.0, 20.0, 10.0, 10.0), SurfacePaint());

    // Inside the layer clip rect but zero-size
    underTest.drawRect(const Rect.fromLTRB(20.0, 20.0, 30.0, 20.0), SurfacePaint());

    // Inside the layer clip but clipped out by a canvas clip
    underTest.save();
    underTest.clipRect(const Rect.fromLTWH(0, 0, 10, 10), ClipOp.intersect);
    underTest.drawRect(const Rect.fromLTWH(20.0, 20.0, 10.0, 10.0), SurfacePaint());
    underTest.restore();

    // Outside to the right
    underTest.drawRect(const Rect.fromLTWH(40.0, 20.0, 10.0, 10.0), SurfacePaint());

    // Outside below
    underTest.drawRect(const Rect.fromLTWH(20.0, 40.0, 10.0, 10.0), SurfacePaint());

    underTest.endRecording();

    expect(underTest.debugPaintCommands, hasLength(10));
    final PaintDrawRect outsideLeft = underTest.debugPaintCommands[0] as PaintDrawRect;
    expect(outsideLeft.isClippedOut, isFalse);
    expect(outsideLeft.leftBound, 0);
    expect(outsideLeft.topBound, 20);
    expect(outsideLeft.rightBound, 10);
    expect(outsideLeft.bottomBound, 30);

    final PaintDrawRect outsideAbove = underTest.debugPaintCommands[1] as PaintDrawRect;
    expect(outsideAbove.isClippedOut, isFalse);

    final PaintDrawRect visible = underTest.debugPaintCommands[2] as PaintDrawRect;
    expect(visible.isClippedOut, isFalse);

    final PaintDrawRect zeroSize = underTest.debugPaintCommands[3] as PaintDrawRect;
    expect(zeroSize.isClippedOut, isTrue);

    expect(underTest.debugPaintCommands[4], isA<PaintSave>());

    final PaintClipRect clip = underTest.debugPaintCommands[5] as PaintClipRect;
    expect(clip.isClippedOut, isFalse);

    final PaintDrawRect clippedOut = underTest.debugPaintCommands[6] as PaintDrawRect;
    expect(clippedOut.isClippedOut, isTrue);

    expect(underTest.debugPaintCommands[7], isA<PaintRestore>());

    final PaintDrawRect outsideRight = underTest.debugPaintCommands[8] as PaintDrawRect;
    expect(outsideRight.isClippedOut, isFalse);

    final PaintDrawRect outsideBelow = underTest.debugPaintCommands[9] as PaintDrawRect;
    expect(outsideBelow.isClippedOut, isFalse);

    // Give it the entire screen so everything paints.
    underTest.apply(mockCanvas, screenRect);
    expect(mockCanvas.methodCallLog, hasLength(11));
    expect(mockCanvas.methodCallLog[0].methodName, 'drawRect');
    expect(mockCanvas.methodCallLog[1].methodName, 'drawRect');
    expect(mockCanvas.methodCallLog[2].methodName, 'drawRect');
    expect(mockCanvas.methodCallLog[3].methodName, 'drawRect');
    expect(mockCanvas.methodCallLog[4].methodName, 'save');
    expect(mockCanvas.methodCallLog[5].methodName, 'clipRect');
    expect(mockCanvas.methodCallLog[6].methodName, 'drawRect');
    expect(mockCanvas.methodCallLog[7].methodName, 'restore');
    expect(mockCanvas.methodCallLog[8].methodName, 'drawRect');
    expect(mockCanvas.methodCallLog[9].methodName, 'drawRect');
    expect(mockCanvas.methodCallLog[10].methodName, 'endOfPaint');

    // Clip out a middle region that only contains 'drawRect'
    mockCanvas.methodCallLog.clear();
    underTest.apply(mockCanvas, const Rect.fromLTRB(15, 15, 35, 35));
    expect(mockCanvas.methodCallLog, hasLength(4));
    expect(mockCanvas.methodCallLog[0].methodName, 'drawRect');
    expect(mockCanvas.methodCallLog[1].methodName, 'save');
    expect(mockCanvas.methodCallLog[2].methodName, 'restore');
    expect(mockCanvas.methodCallLog[3].methodName, 'endOfPaint');
  });

  // Regression test for https://github.com/flutter/flutter/issues/61697.
  test('Allows restore calls after recording has ended', () {
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 200, 400));
    rc.endRecording();
    // Should not throw exception on restore.
    expect(() => rc.restore(), returnsNormally);
  });

  // Regression test for https://github.com/flutter/flutter/issues/61697.
  test('Allows restore calls even if recording is not ended', () {
    final RecordingCanvas rc = RecordingCanvas(const Rect.fromLTRB(0, 0, 200, 400));
    // Should not throw exception on restore.
    expect(() => rc.restore(), returnsNormally);
  });
}

// Expect a drawDRRect call to be registered in the mock call log, with the expectedArguments
void _expectDrawDRRectCall(MockEngineCanvas mock, Map<String, dynamic> expectedArguments) {
  expect(mock.methodCallLog.length, equals(2));
  final MockCanvasCall mockCall = mock.methodCallLog[0];
  expect(mockCall.methodName, equals('drawPath'));
  final Map<String, dynamic> argMap = mockCall.arguments as Map<String, dynamic>;
  final Map<String, dynamic> argContents = <String, dynamic>{};
  argMap.forEach((String key, dynamic value) {
    argContents[key] = value is SurfacePath ? value.toString() : value;
  });
  expect(argContents, equals(expectedArguments));
}
