// Copyright 2022 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:test/test.dart';

void main() {
  test('Picture construction invokes onCreate once', () async {
    int onCreateInvokedCount = 0;
    Picture? createdPicture;
    Picture.onCreate = (Picture picture) {
      onCreateInvokedCount++;
      createdPicture = picture;
    };

    final Picture picture1 = _createPicture();

    expect(onCreateInvokedCount, 1);
    expect(createdPicture, picture1);

    final Picture picture2 = _createPicture();

    expect(onCreateInvokedCount, 2);
    expect(createdPicture, picture2);
    Picture.onCreate = null;
  });

  test('approximateBytesUsed is available for onCreate', () async {
    int pictureSize = -1;

    Picture.onCreate = (Picture picture) => pictureSize = picture.approximateBytesUsed;

    _createPicture();

    expect(pictureSize >= 0, true);
    Picture.onCreate = null;
  });

  test('dispose() invokes onDispose once', () async {
    int onDisposeInvokedCount = 0;
    Picture? disposedPicture;
    Picture.onDispose = (Picture picture) {
      onDisposeInvokedCount++;
      disposedPicture = picture;
    };

    final Picture picture1 = _createPicture()..dispose();

    expect(onDisposeInvokedCount, 1);
    expect(disposedPicture, picture1);

    final Picture picture2 = _createPicture()..dispose();

    expect(onDisposeInvokedCount, 2);
    expect(disposedPicture, picture2);

    Picture.onDispose = null;
  });
}

Picture _createPicture() {
  final PictureRecorder recorder = PictureRecorder();
  final Canvas canvas = Canvas(recorder);
  const Rect rect = Rect.fromLTWH(0.0, 0.0, 100.0, 100.0);
  canvas.clipRect(rect);
  return recorder.endRecording();
}
