// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

import 'common.dart';
import 'test_data.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  setUpCanvasKitTest();

  tearDown(() {
    ui.Image.onCreate = null;
    ui.Image.onDispose = null;
  });

  test('toImage succeeds', () async {
    final ui.Image image = await _createImage();
    expect(image.runtimeType.toString(), equals('CkImage'));
    image.dispose();
  });

  test('Image constructor invokes onCreate once', () async {
    int onCreateInvokedCount = 0;
    ui.Image? createdImage;
    ui.Image.onCreate = (ui.Image image) {
      onCreateInvokedCount++;
      createdImage = image;
    };

    final ui.Image image1 = await _createImage();

    expect(onCreateInvokedCount, 1);
    expect(createdImage, image1);

    final ui.Image image2 = await _createImage();

    expect(onCreateInvokedCount, 2);
    expect(createdImage, image2);
  });

  test('dispose() invokes onDispose once', () async {
    int onDisposeInvokedCount = 0;
    ui.Image? disposedImage;
    ui.Image.onDispose = (ui.Image image) {
      onDisposeInvokedCount++;
      disposedImage = image;
    };

    final ui.Image image1 =
        await _createImage()
          ..dispose();

    expect(onDisposeInvokedCount, 1);
    expect(disposedImage, image1);

    final ui.Image image2 =
        await _createImage()
          ..dispose();

    expect(onDisposeInvokedCount, 2);
    expect(disposedImage, image2);
  });

  test('fetchImage fetches image in chunks', () async {
    final List<int> cumulativeBytesLoadedInvocations = <int>[];
    final List<int> expectedTotalBytesInvocations = <int>[];
    final Uint8List result = await fetchImage('/long_test_payload?length=100000&chunk=1000', (
      int cumulativeBytesLoaded,
      int expectedTotalBytes,
    ) {
      cumulativeBytesLoadedInvocations.add(cumulativeBytesLoaded);
      expectedTotalBytesInvocations.add(expectedTotalBytes);
    });

    // Check that image payload was chunked.
    expect(cumulativeBytesLoadedInvocations, hasLength(greaterThan(1)));

    // Check that reported total byte count is the same across all invocations.
    for (final int expectedTotalBytes in expectedTotalBytesInvocations) {
      expect(expectedTotalBytes, 100000);
    }

    // Check that cumulative byte count grows with each invocation.
    cumulativeBytesLoadedInvocations.reduce((int previous, int next) {
      expect(next, greaterThan(previous));
      return next;
    });

    // Check that the last cumulative byte count matches the total byte count.
    expect(cumulativeBytesLoadedInvocations.last, 100000);

    // Check the contents of the returned data.
    expect(result, List<int>.generate(100000, (int i) => i & 0xFF));
  });

  test('scaledImageSize scales to a target width with no target height', () {
    final BitmapSize? size = scaledImageSize(200, 100, 600, null);
    expect(size?.width, 600);
    expect(size?.height, 300);
  });

  test('instantiateImageCodecWithSize disposes temporary image', () async {
    final Set<ui.Image> activeImages = <ui.Image>{};
    ui.Image.onCreate = activeImages.add;
    ui.Image.onDispose = activeImages.remove;

    final ui.Image image = await _createImage();
    final ByteData? imageData = await image.toByteData(format: ui.ImageByteFormat.png);
    final ui.ImmutableBuffer imageBuffer = await ui.ImmutableBuffer.fromUint8List(
      imageData!.buffer.asUint8List(),
    );
    image.dispose();

    final ui.Codec codec = await ui.instantiateImageCodecWithSize(
      imageBuffer,
      getTargetSize: (w, h) => ui.TargetImageSize(width: w ~/ 2, height: h ~/ 2),
    );
    final ui.FrameInfo frameInfo = await codec.getNextFrame();

    expect(activeImages.length, 1);

    frameInfo.image.dispose();
    codec.dispose();

    expect(activeImages.length, 0);
  });

  test('CkImage does not close image source too early', () async {
    final ImageSource imageSource = ImageBitmapImageSource(
      await domWindow.createImageBitmap(createBlankDomImageData(4, 4)),
    );

    final SkImage skImage1 =
        canvasKit.MakeAnimatedImageFromEncoded(k4x4PngImage)!.makeImageAtCurrentFrame();
    final CkImage image1 = CkImage(skImage1, imageSource: imageSource);

    final SkImage skImage2 =
        canvasKit.MakeAnimatedImageFromEncoded(k4x4PngImage)!.makeImageAtCurrentFrame();
    final CkImage image2 = CkImage(skImage2, imageSource: imageSource);

    final CkImage image3 = image1.clone();

    expect(imageSource.debugIsClosed, isFalse);

    image1.dispose();
    expect(imageSource.debugIsClosed, isFalse);

    image2.dispose();
    expect(imageSource.debugIsClosed, isFalse);

    image3.dispose();
    expect(imageSource.debugIsClosed, isTrue);
  });
}

Future<ui.Image> _createImage() => _createPicture().toImage(10, 10);

ui.Picture _createPicture() {
  final ui.PictureRecorder recorder = ui.PictureRecorder();
  final ui.Canvas canvas = ui.Canvas(recorder);
  const ui.Rect rect = ui.Rect.fromLTWH(0.0, 0.0, 100.0, 100.0);
  canvas.clipRect(rect);
  return recorder.endRecording();
}
