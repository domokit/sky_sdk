// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:ui';

import 'package:test/test.dart';

void main() {
  test('Image constructor and dispose invokes onCreate once', () async {
    int onCreateInvokedCount = 0;
    Image? createdImage;
    int onDisposeInvokedCount = 0;
    Image? disposedImage;
    Image.onCreate = (Image image) {
      onCreateInvokedCount++;
      createdImage = image;
    };
    Image.onDispose = (Image image) {
      onDisposeInvokedCount++;
      disposedImage = image;
    };

    final Image image1 =
        await _createImage()
          ..dispose();

    expect(onCreateInvokedCount, 1);
    expect(createdImage, image1);
    expect(onDisposeInvokedCount, 1);
    expect(disposedImage, image1);

    final Image image2 =
        await _createImage()
          ..dispose();

    expect(onCreateInvokedCount, 2);
    expect(createdImage, image2);
    expect(onDisposeInvokedCount, 2);
    expect(disposedImage, image2);

    Image.onCreate = null;
    Image.onDispose = null;
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
