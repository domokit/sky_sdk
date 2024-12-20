// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:ui';

import 'package:test/test.dart';

void main() {
  test('toImage succeeds', () async {
    final Image image = await _createImage();
    expect(image.runtimeType.toString(), equals('Image'));
    image.dispose();
  });
}

Future<Image> _createImage() => _createPicture().toImage(10, 10);

Picture _createPicture() {
  final PictureRecorder recorder = PictureRecorder();
  final Canvas canvas = Canvas(recorder);
  const Rect rect = Rect.fromLTWH(0.0, 0.0, 100.0, 100.0);
  canvas.clipRect(rect);
  return recorder.endRecording();
}
