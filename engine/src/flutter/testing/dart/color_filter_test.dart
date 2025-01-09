// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';
import 'dart:ui';

import 'package:test/test.dart';

import 'impeller_enabled.dart';

const Color transparent = Color(0x00000000);
const Color red = Color(0xFFAA0000);
const Color green = Color(0xFF00AA00);

const int greenRedColorBlend = 0xFF3131DB;
const int greenRedColorBlendInverted = 0xFFCECE24;
const int greenGreyscaled = 0xFF7A7A7A;
const int greenInvertedGreyscaled = 0xFF858585;

const int greenLinearToSrgbGamma = 0xFF00D500;
const int greenLinearToSrgbGammaInverted = 0xFFFF2AFF;

const int greenSrgbToLinearGamma = 0xFF006700;
const int greenSrgbToLinearGammaInverted = 0xFFFF98FF;

const List<double> greyscaleColorMatrix = <double>[
  0.2126, 0.7152, 0.0722, 0, 0, //
  0.2126, 0.7152, 0.0722, 0, 0, //
  0.2126, 0.7152, 0.0722, 0, 0, //
  0, 0, 0, 1, 0, //
];
const List<double> identityColorMatrix = <double>[
  1,
  0,
  0,
  0,
  0,
  0,
  1,
  0,
  0,
  0,
  0,
  0,
  1,
  0,
  0,
  0,
  0,
  0,
  1,
  0,
];

void main() {
  Future<Uint32List> getBytesForPaint(Paint paint, {int width = 1, int height = 1}) async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas recorderCanvas = Canvas(recorder);
    recorderCanvas.drawPaint(paint);
    final Picture picture = recorder.endRecording();
    final Image image = await picture.toImage(width, height);
    final ByteData bytes = (await image.toByteData())!;

    expect(bytes.lengthInBytes, width * height * 4);
    return bytes.buffer.asUint32List();
  }

  test('ColorFilter - mode', () async {
    final Paint paint =
        Paint()
          ..color = green
          ..colorFilter = const ColorFilter.mode(red, BlendMode.color);

    Uint32List bytes = await getBytesForPaint(paint);
    expect(bytes[0], greenRedColorBlend);

    // TODO(135699): enable this
    if (impellerEnabled) {
      return;
    }

    paint.invertColors = true;
    bytes = await getBytesForPaint(paint);
    expect(bytes[0], greenRedColorBlendInverted);
  });

  test('ColorFilter - NOP mode does not crash', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    final Paint paint =
        Paint()
          ..color = green
          ..colorFilter = const ColorFilter.mode(transparent, BlendMode.srcOver);
    canvas.saveLayer(const Rect.fromLTRB(-100, -100, 200, 200), paint);
    canvas.drawRect(const Rect.fromLTRB(0, 0, 100, 100), Paint());
    canvas.restore();
    final Picture picture = recorder.endRecording();

    final SceneBuilder builder = SceneBuilder();
    builder.addPicture(Offset.zero, picture);

    final Scene scene = builder.build();
    await scene.toImage(100, 100);
  });

  test('ColorFilter - matrix', () async {
    final Paint paint =
        Paint()
          ..color = green
          ..colorFilter = const ColorFilter.matrix(greyscaleColorMatrix);

    Uint32List bytes = await getBytesForPaint(paint);
    expect(bytes[0], greenGreyscaled);

    // TODO(135699): enable this
    if (impellerEnabled) {
      return;
    }

    paint.invertColors = true;
    bytes = await getBytesForPaint(paint);
    expect(bytes[0], greenInvertedGreyscaled);
  });

  test('ColorFilter - NOP matrix does not crash', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    final Paint paint =
        Paint()
          ..color = const Color(0xff00AA00)
          ..colorFilter = const ColorFilter.matrix(identityColorMatrix);
    canvas.saveLayer(const Rect.fromLTRB(-100, -100, 200, 200), paint);
    canvas.drawRect(const Rect.fromLTRB(0, 0, 100, 100), Paint());
    canvas.restore();
    final Picture picture = recorder.endRecording();

    final SceneBuilder builder = SceneBuilder();
    builder.addPicture(Offset.zero, picture);

    final Scene scene = builder.build();
    await scene.toImage(100, 100);
  });

  test('ColorFilter - linearToSrgbGamma', () async {
    final Paint paint =
        Paint()
          ..color = green
          ..colorFilter = const ColorFilter.linearToSrgbGamma();

    Uint32List bytes = await getBytesForPaint(paint);
    expect(bytes[0], greenLinearToSrgbGamma);

    // TODO(135699): enable this
    if (impellerEnabled) {
      return;
    }
    paint.invertColors = true;
    bytes = await getBytesForPaint(paint);
    expect(bytes[0], greenLinearToSrgbGammaInverted);
  });

  test('ColorFilter - srgbToLinearGamma', () async {
    final Paint paint =
        Paint()
          ..color = green
          ..colorFilter = const ColorFilter.srgbToLinearGamma();

    Uint32List bytes = await getBytesForPaint(paint);
    expect(bytes[0], greenSrgbToLinearGamma);

    // TODO(135699): enable this
    if (impellerEnabled) {
      return;
    }

    paint.invertColors = true;
    bytes = await getBytesForPaint(paint);
    expect(bytes[0], greenSrgbToLinearGammaInverted);
  });
}
