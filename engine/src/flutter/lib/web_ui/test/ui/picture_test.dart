// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/ui.dart' as ui;

import '../common/test_initialization.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests();

  test('Picture construction invokes onCreate once', () async {
    int onCreateInvokedCount = 0;
    ui.Picture? createdPicture;
    ui.Picture.onCreate = (ui.Picture picture) {
      onCreateInvokedCount++;
      createdPicture = picture;
    };

    final ui.Picture picture1 = _createPicture();

    expect(onCreateInvokedCount, 1);
    expect(createdPicture, picture1);

    final ui.Picture picture2 = _createPicture();

    expect(onCreateInvokedCount, 2);
    expect(createdPicture, picture2);
    ui.Picture.onCreate = null;
  });

  test('approximateBytesUsed is available for onCreate', () async {
    int pictureSize = -1;

    ui.Picture.onCreate = (ui.Picture picture) => pictureSize = picture.approximateBytesUsed;

    _createPicture();

    expect(pictureSize >= 0, true);
    ui.Picture.onCreate = null;
  });

  test('dispose() invokes onDispose once', () async {
    int onDisposeInvokedCount = 0;
    ui.Picture? disposedPicture;
    ui.Picture.onDispose = (ui.Picture picture) {
      onDisposeInvokedCount++;
      disposedPicture = picture;
    };

    final ui.Picture picture1 = _createPicture()..dispose();

    expect(onDisposeInvokedCount, 1);
    expect(disposedPicture, picture1);

    final ui.Picture picture2 = _createPicture()..dispose();

    expect(onDisposeInvokedCount, 2);
    expect(disposedPicture, picture2);

    ui.Picture.onDispose = null;
  });
}

ui.Picture _createPicture() {
  final ui.PictureRecorder recorder = ui.PictureRecorder();
  final ui.Canvas canvas = ui.Canvas(recorder);
  const ui.Rect rect = ui.Rect.fromLTWH(0.0, 0.0, 100.0, 100.0);
  canvas.clipRect(rect);
  return recorder.endRecording();
}
