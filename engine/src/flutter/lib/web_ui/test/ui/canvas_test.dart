// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math';
import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

import '../common/matchers.dart';
import '../common/test_initialization.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests(setUpTestViewDimensions: false);

  final bool deviceClipRoundsOut = renderer is! HtmlRenderer;
  runCanvasTests(deviceClipRoundsOut: deviceClipRoundsOut);
}

void runCanvasTests({required bool deviceClipRoundsOut}) {
  setUp(() {
    EngineSemantics.debugResetSemantics();
  });

  group('ui.Canvas transform tests', () {
    void transformsClose(Float64List value, Float64List expected) {
      expect(expected.length, equals(16));
      expect(value.length, equals(16));
      for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
          expect(value[r * 4 + c], within(from: expected[r * 4 + c]));
        }
      }
    }

    void transformsNotClose(Float64List value, Float64List expected) {
      // We check the lengths here even though [transformsClose] will
      // check them so that the [TestFailure] we catch below can only
      // be due to a difference in matrix values.
      expect(expected.length, equals(16));
      expect(value.length, equals(16));
      try {
        transformsClose(value, expected);
      } on TestFailure {
        return;
      }
      throw TestFailure('transforms were too close to equal'); // ignore: only_throw_errors
    }

    test('ui.Canvas.translate affects canvas.getTransform', () {
      final ui.PictureRecorder recorder = ui.PictureRecorder();
      final ui.Canvas canvas = ui.Canvas(recorder);
      canvas.translate(12, 14.5);
      final Float64List matrix = Matrix4.translationValues(12, 14.5, 0).toFloat64();
      final Float64List curMatrix = canvas.getTransform();
      transformsClose(curMatrix, matrix);
      canvas.translate(10, 10);
      final Float64List newCurMatrix = canvas.getTransform();
      transformsNotClose(newCurMatrix, matrix);
      transformsClose(curMatrix, matrix);
    });

    test('ui.Canvas.scale affects canvas.getTransform', () {
      final ui.PictureRecorder recorder = ui.PictureRecorder();
      final ui.Canvas canvas = ui.Canvas(recorder);
      canvas.scale(12, 14.5);
      final Float64List matrix = Matrix4.diagonal3Values(12, 14.5, 1).toFloat64();
      final Float64List curMatrix = canvas.getTransform();
      transformsClose(curMatrix, matrix);
      canvas.scale(10, 10);
      final Float64List newCurMatrix = canvas.getTransform();
      transformsNotClose(newCurMatrix, matrix);
      transformsClose(curMatrix, matrix);
    });

    test('Canvas.rotate affects canvas.getTransform', () async {
      final ui.PictureRecorder recorder = ui.PictureRecorder();
      final ui.Canvas canvas = ui.Canvas(recorder);
      canvas.rotate(pi);
      final Float64List matrix = Matrix4.rotationZ(pi).toFloat64();
      final Float64List curMatrix = canvas.getTransform();
      transformsClose(curMatrix, matrix);
      canvas.rotate(pi / 2);
      final Float64List newCurMatrix = canvas.getTransform();
      transformsNotClose(newCurMatrix, matrix);
      transformsClose(curMatrix, matrix);
    });

    test('Canvas.skew affects canvas.getTransform', () async {
      final ui.PictureRecorder recorder = ui.PictureRecorder();
      final ui.Canvas canvas = ui.Canvas(recorder);
      canvas.skew(12, 14.5);
      final Float64List matrix =
          (Matrix4.identity()
                ..setEntry(0, 1, 12)
                ..setEntry(1, 0, 14.5))
              .toFloat64();
      final Float64List curMatrix = canvas.getTransform();
      transformsClose(curMatrix, matrix);
      canvas.skew(10, 10);
      final Float64List newCurMatrix = canvas.getTransform();
      transformsNotClose(newCurMatrix, matrix);
      transformsClose(curMatrix, matrix);
    });

    test('Canvas.transform affects canvas.getTransform', () async {
      final ui.PictureRecorder recorder = ui.PictureRecorder();
      final ui.Canvas canvas = ui.Canvas(recorder);
      final Float64List matrix =
          (Matrix4.identity()
                ..translate(12.0, 14.5)
                ..scale(12.0, 14.5))
              .toFloat64();
      canvas.transform(matrix);
      final Float64List curMatrix = canvas.getTransform();
      transformsClose(curMatrix, matrix);
      canvas.translate(10, 10);
      final Float64List newCurMatrix = canvas.getTransform();
      transformsNotClose(newCurMatrix, matrix);
      transformsClose(curMatrix, matrix);
    });
  });

  void rectsClose(ui.Rect value, ui.Rect expected) {
    expect(value.left, closeTo(expected.left, 1e-6));
    expect(value.top, closeTo(expected.top, 1e-6));
    expect(value.right, closeTo(expected.right, 1e-6));
    expect(value.bottom, closeTo(expected.bottom, 1e-6));
  }

  void rectsNotClose(ui.Rect value, ui.Rect expected) {
    try {
      rectsClose(value, expected);
    } on TestFailure {
      return;
    }
    throw TestFailure('transforms were too close to equal'); // ignore: only_throw_errors
  }

  group('ui.Canvas clip tests', () {
    test('Canvas.clipRect affects canvas.getClipBounds', () async {
      final ui.PictureRecorder recorder = ui.PictureRecorder();
      final ui.Canvas canvas = ui.Canvas(recorder, const ui.Rect.fromLTRB(0, 0, 100, 100));
      const ui.Rect clipRawBounds = ui.Rect.fromLTRB(10.2, 11.3, 20.4, 25.7);
      const ui.Rect clipExpandedBounds = ui.Rect.fromLTRB(10, 11, 21, 26);
      final ui.Rect clipDestBounds = deviceClipRoundsOut ? clipExpandedBounds : clipRawBounds;
      canvas.clipRect(clipRawBounds);

      // Save initial return values for testing restored values
      final ui.Rect initialLocalBounds = canvas.getLocalClipBounds();
      final ui.Rect initialDestinationBounds = canvas.getDestinationClipBounds();
      rectsClose(initialLocalBounds, clipExpandedBounds);
      rectsClose(initialDestinationBounds, clipDestBounds);

      canvas.save();
      canvas.clipRect(const ui.Rect.fromLTRB(0, 0, 15, 15));
      // Both clip bounds have changed
      rectsNotClose(canvas.getLocalClipBounds(), clipExpandedBounds);
      rectsNotClose(canvas.getDestinationClipBounds(), clipDestBounds);
      // Previous return values have not changed
      rectsClose(initialLocalBounds, clipExpandedBounds);
      rectsClose(initialDestinationBounds, clipDestBounds);
      canvas.restore();

      // save/restore returned the values to their original values
      expect(canvas.getLocalClipBounds(), initialLocalBounds);
      expect(canvas.getDestinationClipBounds(), initialDestinationBounds);

      canvas.save();
      canvas.scale(2, 2);
      const ui.Rect scaledExpandedBounds = ui.Rect.fromLTRB(5, 5.5, 10.5, 13);
      rectsClose(canvas.getLocalClipBounds(), scaledExpandedBounds);
      // Destination bounds are unaffected by transform
      rectsClose(canvas.getDestinationClipBounds(), clipDestBounds);
      canvas.restore();

      // save/restore returned the values to their original values
      expect(canvas.getLocalClipBounds(), initialLocalBounds);
      expect(canvas.getDestinationClipBounds(), initialDestinationBounds);
    });

    test('Canvas.clipRRect affects canvas.getClipBounds', () async {
      final ui.PictureRecorder recorder = ui.PictureRecorder();
      final ui.Canvas canvas = ui.Canvas(recorder, const ui.Rect.fromLTRB(0, 0, 100, 100));
      const ui.Rect clipRawBounds = ui.Rect.fromLTRB(10.2, 11.3, 20.4, 25.7);
      const ui.Rect clipExpandedBounds = ui.Rect.fromLTRB(10, 11, 21, 26);
      final ui.Rect clipDestBounds = deviceClipRoundsOut ? clipExpandedBounds : clipRawBounds;
      final ui.RRect clip = ui.RRect.fromRectAndRadius(clipRawBounds, const ui.Radius.circular(3));
      canvas.clipRRect(clip);

      // Save initial return values for testing restored values
      final ui.Rect initialLocalBounds = canvas.getLocalClipBounds();
      final ui.Rect initialDestinationBounds = canvas.getDestinationClipBounds();
      rectsClose(initialLocalBounds, clipExpandedBounds);
      rectsClose(initialDestinationBounds, clipDestBounds);

      canvas.save();
      canvas.clipRect(const ui.Rect.fromLTRB(0, 0, 15, 15));
      // Both clip bounds have changed
      rectsNotClose(canvas.getLocalClipBounds(), clipExpandedBounds);
      rectsNotClose(canvas.getDestinationClipBounds(), clipDestBounds);
      // Previous return values have not changed
      rectsClose(initialLocalBounds, clipExpandedBounds);
      rectsClose(initialDestinationBounds, clipDestBounds);
      canvas.restore();

      // save/restore returned the values to their original values
      expect(canvas.getLocalClipBounds(), initialLocalBounds);
      expect(canvas.getDestinationClipBounds(), initialDestinationBounds);

      canvas.save();
      canvas.scale(2, 2);
      const ui.Rect scaledExpandedBounds = ui.Rect.fromLTRB(5, 5.5, 10.5, 13);
      rectsClose(canvas.getLocalClipBounds(), scaledExpandedBounds);
      // Destination bounds are unaffected by transform
      rectsClose(canvas.getDestinationClipBounds(), clipDestBounds);
      canvas.restore();

      // save/restore returned the values to their original values
      expect(canvas.getLocalClipBounds(), initialLocalBounds);
      expect(canvas.getDestinationClipBounds(), initialDestinationBounds);
    });

    test('Canvas.clipPath affects canvas.getClipBounds', () async {
      final ui.PictureRecorder recorder = ui.PictureRecorder();
      final ui.Canvas canvas = ui.Canvas(recorder, const ui.Rect.fromLTRB(0, 0, 100, 100));
      const ui.Rect clipRawBounds = ui.Rect.fromLTRB(10.2, 11.3, 20.4, 25.7);
      const ui.Rect clipExpandedBounds = ui.Rect.fromLTRB(10, 11, 21, 26);
      final ui.Rect clipDestBounds = deviceClipRoundsOut ? clipExpandedBounds : clipRawBounds;
      final ui.Path clip =
          ui.Path()
            ..addRect(clipRawBounds)
            ..addOval(clipRawBounds);
      canvas.clipPath(clip);

      // Save initial return values for testing restored values
      final ui.Rect initialLocalBounds = canvas.getLocalClipBounds();
      final ui.Rect initialDestinationBounds = canvas.getDestinationClipBounds();
      rectsClose(initialLocalBounds, clipExpandedBounds);
      rectsClose(initialDestinationBounds, clipDestBounds);

      canvas.save();
      canvas.clipRect(const ui.Rect.fromLTRB(0, 0, 15, 15));
      // Both clip bounds have changed
      rectsNotClose(canvas.getLocalClipBounds(), clipExpandedBounds);
      rectsNotClose(canvas.getDestinationClipBounds(), clipDestBounds);
      // Previous return values have not changed
      rectsClose(initialLocalBounds, clipExpandedBounds);
      rectsClose(initialDestinationBounds, clipDestBounds);
      canvas.restore();

      // save/restore returned the values to their original values
      expect(canvas.getLocalClipBounds(), initialLocalBounds);
      expect(canvas.getDestinationClipBounds(), initialDestinationBounds);

      canvas.save();
      canvas.scale(2, 2);
      const ui.Rect scaledExpandedBounds = ui.Rect.fromLTRB(5, 5.5, 10.5, 13);
      rectsClose(canvas.getLocalClipBounds(), scaledExpandedBounds);
      // Destination bounds are unaffected by transform
      rectsClose(canvas.getDestinationClipBounds(), clipDestBounds);
      canvas.restore();

      // save/restore returned the values to their original values
      expect(canvas.getLocalClipBounds(), initialLocalBounds);
      expect(canvas.getDestinationClipBounds(), initialDestinationBounds);
    });

    test('Canvas.clipRect(diff) does not affect canvas.getClipBounds', () async {
      final ui.PictureRecorder recorder = ui.PictureRecorder();
      final ui.Canvas canvas = ui.Canvas(recorder, const ui.Rect.fromLTRB(0, 0, 100, 100));
      const ui.Rect clipRawBounds = ui.Rect.fromLTRB(10.2, 11.3, 20.4, 25.7);
      const ui.Rect clipExpandedBounds = ui.Rect.fromLTRB(10, 11, 21, 26);
      final ui.Rect clipDestBounds = deviceClipRoundsOut ? clipExpandedBounds : clipRawBounds;
      canvas.clipRect(clipRawBounds);

      // Save initial return values for testing restored values
      final ui.Rect initialLocalBounds = canvas.getLocalClipBounds();
      final ui.Rect initialDestinationBounds = canvas.getDestinationClipBounds();
      rectsClose(initialLocalBounds, clipExpandedBounds);
      rectsClose(initialDestinationBounds, clipDestBounds);

      canvas.clipRect(const ui.Rect.fromLTRB(0, 0, 15, 15), clipOp: ui.ClipOp.difference);
      expect(canvas.getLocalClipBounds(), initialLocalBounds);
      expect(canvas.getDestinationClipBounds(), initialDestinationBounds);
    });
  });

  group('RestoreToCount function tests', () {
    test('RestoreToCount can work', () async {
      final ui.PictureRecorder recorder = ui.PictureRecorder();
      final ui.Canvas canvas = ui.Canvas(recorder);
      canvas.save();
      canvas.save();
      canvas.save();
      canvas.save();
      canvas.save();
      expect(canvas.getSaveCount(), 6);
      canvas.restoreToCount(2);
      expect(canvas.getSaveCount(), 2);
      canvas.restore();
      expect(canvas.getSaveCount(), 1);
    });

    test('RestoreToCount count less than 1, the stack should be reset', () async {
      final ui.PictureRecorder recorder = ui.PictureRecorder();
      final ui.Canvas canvas = ui.Canvas(recorder);
      canvas.save();
      canvas.save();
      canvas.save();
      canvas.save();
      canvas.save();
      expect(canvas.getSaveCount(), equals(6));
      canvas.restoreToCount(0);
      expect(canvas.getSaveCount(), equals(1));
    });

    test('RestoreToCount count greater than current [getSaveCount]', () async {
      final ui.PictureRecorder recorder = ui.PictureRecorder();
      final ui.Canvas canvas = ui.Canvas(recorder);
      canvas.save();
      canvas.save();
      canvas.save();
      canvas.save();
      canvas.save();
      expect(canvas.getSaveCount(), equals(6));
      canvas.restoreToCount(canvas.getSaveCount() + 1);
      expect(canvas.getSaveCount(), equals(6));
    });
  });
}
