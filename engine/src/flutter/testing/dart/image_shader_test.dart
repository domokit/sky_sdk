// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';
import 'dart:ui';

import 'package:test/test.dart';

import 'canvas_test.dart' show createImage, testCanvas;

void main() {
  test('Construct an ImageShader', () async {
    final Image image = await createImage(50, 50);
    final ImageShader shader = ImageShader(image, TileMode.clamp, TileMode.clamp, Float64List(16));
    final Paint paint = Paint()..shader = shader;
    const Rect rect = Rect.fromLTRB(0, 0, 100, 100);
    testCanvas((Canvas canvas) => canvas.drawRect(rect, paint));

    expect(shader.debugDisposed, false);
    shader.dispose();
    expect(shader.debugDisposed, true);

    image.dispose();
  });

  test('ImageShader with disposed image', () async {
    final Image image = await createImage(50, 50);
    image.dispose();

    expect(
      () => ImageShader(image, TileMode.clamp, TileMode.clamp, Float64List(16)),
      throwsA(isA<AssertionError>()),
    );
  });

  test('Disposed image shader in a paint', () async {
    final Image image = await createImage(50, 50);
    final ImageShader shader = ImageShader(image, TileMode.clamp, TileMode.clamp, Float64List(16));
    shader.dispose();

    expect(() => Paint()..shader = shader, throwsA(isA<AssertionError>()));
  });

  test('Construct an ImageShader - GPU image', () async {
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);
    canvas.drawPaint(Paint()..color = const Color(0xFFABCDEF));
    final Picture picture = recorder.endRecording();
    final Image image = picture.toImageSync(50, 50);
    picture.dispose();

    final ImageShader shader = ImageShader(image, TileMode.clamp, TileMode.clamp, Float64List(16));
    final Paint paint = Paint()..shader = shader;
    const Rect rect = Rect.fromLTRB(0, 0, 100, 100);
    testCanvas((Canvas canvas) => canvas.drawRect(rect, paint));

    expect(shader.debugDisposed, false);
    shader.dispose();
    expect(shader.debugDisposed, true);

    image.dispose();
  });
}
