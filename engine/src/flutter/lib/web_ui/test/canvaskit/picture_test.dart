// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';

import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

import '../common/matchers.dart';
import 'common.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('CkPicture', () {
    setUpCanvasKitTest();

    group('lifecycle', () {
      test('can be disposed of manually', () {
        final ui.PictureRecorder recorder = ui.PictureRecorder();
        final ui.Canvas canvas = ui.Canvas(recorder);
        canvas.drawPaint(ui.Paint());
        final CkPicture picture = recorder.endRecording() as CkPicture;
        expect(picture.skiaObject, isNotNull);
        expect(picture.debugDisposed, isFalse);
        picture.debugCheckNotDisposed('Test.'); // must not throw
        picture.dispose();
        expect(() => picture.skiaObject, throwsA(isAssertionError));
        expect(picture.debugDisposed, isTrue);

        StateError? actualError;
        try {
          picture.debugCheckNotDisposed('Test.');
        } on StateError catch (error) {
          actualError = error;
        }

        expect(actualError, isNotNull);

        // TODO(yjbanov): cannot test precise message due to https://github.com/flutter/flutter/issues/96298
        expect(
          '$actualError',
          startsWith(
            'Bad state: Test.\n'
            'The picture has been disposed. '
            'When the picture was disposed the stack trace was:\n',
          ),
        );
      });
    });

    test('toImageSync', () async {
      const ui.Color color = ui.Color(0xFFAAAAAA);
      final ui.PictureRecorder recorder = ui.PictureRecorder();
      final ui.Canvas canvas = ui.Canvas(recorder);
      canvas.drawPaint(ui.Paint()..color = color);
      final ui.Picture picture = recorder.endRecording();
      final ui.Image image = picture.toImageSync(10, 15);

      expect(image.width, 10);
      expect(image.height, 15);

      final ByteData? data = await image.toByteData();
      expect(data, isNotNull);
      expect(data!.lengthInBytes, 10 * 15 * 4);
      expect(data.buffer.asUint32List().first, color.value);
    });

    test('cullRect bounds are tight', () async {
      const ui.Color red = ui.Color.fromRGBO(255, 0, 0, 1);
      const ui.Color green = ui.Color.fromRGBO(0, 255, 0, 1);
      const ui.Color blue = ui.Color.fromRGBO(0, 0, 255, 1);

      final ui.PictureRecorder recorder = ui.PictureRecorder();
      final ui.Canvas canvas = ui.Canvas(recorder);
      canvas.drawRRect(
        ui.RRect.fromRectXY(const ui.Rect.fromLTRB(20, 20, 150, 300), 15, 15),
        ui.Paint()..color = red,
      );
      canvas.drawCircle(const ui.Offset(200, 200), 100, ui.Paint()..color = green);
      canvas.drawOval(const ui.Rect.fromLTRB(210, 40, 268, 199), ui.Paint()..color = blue);

      final CkPicture picture = recorder.endRecording() as CkPicture;
      final ui.Rect bounds = picture.cullRect;
      // Top left bounded by the red rrect, right bounded by right edge
      // of red rrect, bottom bounded by bottom of green circle.
      expect(bounds, equals(const ui.Rect.fromLTRB(20, 20, 300, 300)));
    });

    test('cullRect bounds with infinite size draw', () async {
      const ui.Color red = ui.Color.fromRGBO(255, 0, 0, 1);

      final ui.PictureRecorder recorder = ui.PictureRecorder();
      final ui.Canvas canvas = ui.Canvas(recorder);
      canvas.drawColor(red, ui.BlendMode.src);

      final CkPicture picture = recorder.endRecording() as CkPicture;
      final ui.Rect bounds = picture.cullRect;
      // Since the drawColor command fills the entire canvas, the computed
      // bounds default to the cullRect that is passed in when the
      // PictureRecorder is created, ie ui.Rect.largest.
      expect(bounds, equals(ui.Rect.largest));
    });

    test('approximateBytesUsed', () async {
      const ui.Color red = ui.Color.fromRGBO(255, 0, 0, 1);
      const ui.Color green = ui.Color.fromRGBO(0, 255, 0, 1);
      const ui.Color blue = ui.Color.fromRGBO(0, 0, 255, 1);

      final ui.PictureRecorder recorder = ui.PictureRecorder();
      final ui.Canvas canvas = ui.Canvas(recorder);
      canvas.drawRRect(
        ui.RRect.fromRectXY(const ui.Rect.fromLTRB(20, 20, 150, 300), 15, 15),
        ui.Paint()..color = red,
      );
      canvas.drawCircle(const ui.Offset(200, 200), 100, ui.Paint()..color = green);
      canvas.drawOval(const ui.Rect.fromLTRB(210, 40, 268, 199), ui.Paint()..color = blue);

      final CkPicture picture = recorder.endRecording() as CkPicture;
      final int bytesUsed = picture.approximateBytesUsed;
      // Sanity check: the picture should use more than 20 bytes of memory.
      expect(bytesUsed, greaterThan(20));
    });
    // TODO(hterkelsen): https://github.com/flutter/flutter/issues/60040
  }, skip: isIosSafari);
}
