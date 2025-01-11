// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'dart:async';
import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Image at default filterQuality', (WidgetTester tester) async {
    await testImageQuality(tester, null);
  });

  testWidgets('Image at high filterQuality', (WidgetTester tester) async {
    await testImageQuality(tester, ui.FilterQuality.high);
  });

  testWidgets('Image at none filterQuality', (WidgetTester tester) async {
    await testImageQuality(tester, ui.FilterQuality.none);
  });
}

Future<void> testImageQuality(WidgetTester tester, ui.FilterQuality? quality) async {
  await tester.binding.setSurfaceSize(const ui.Size(3, 3));
  // A 3x3 image encoded as PNG with white background and black pixels on the diagonal:
  // ┌──────┐
  // │▓▓    │
  // │  ▓▓  │
  // │    ▓▓│
  // └──────┘
  // At different levels of quality these pixels are blurred differently.
  final Uint8List test3x3Image = Uint8List.fromList(<int>[
    0x89,
    0x50,
    0x4e,
    0x47,
    0x0d,
    0x0a,
    0x1a,
    0x0a,
    0x00,
    0x00,
    0x00,
    0x0d,
    0x49,
    0x48,
    0x44,
    0x52,
    0x00,
    0x00,
    0x00,
    0x03,
    0x00,
    0x00,
    0x00,
    0x03,
    0x08,
    0x02,
    0x00,
    0x00,
    0x00,
    0xd9,
    0x4a,
    0x22,
    0xe8,
    0x00,
    0x00,
    0x00,
    0x1b,
    0x49,
    0x44,
    0x41,
    0x54,
    0x08,
    0xd7,
    0x63,
    0x64,
    0x60,
    0x60,
    0xf8,
    0xff,
    0xff,
    0x3f,
    0x03,
    0x9c,
    0xfa,
    0xff,
    0xff,
    0x3f,
    0xc3,
    0xff,
    0xff,
    0xff,
    0x21,
    0x1c,
    0x00,
    0xcb,
    0x70,
    0x0e,
    0xf3,
    0x5d,
    0x11,
    0xc2,
    0xf8,
    0x00,
    0x00,
    0x00,
    0x00,
    0x49,
    0x45,
    0x4e,
    0x44,
    0xae,
    0x42,
    0x60,
    0x82,
  ]);
  final ui.Image image =
      (await tester.runAsync(() async {
        final ui.Codec codec = await ui.instantiateImageCodec(test3x3Image);
        addTearDown(codec.dispose);
        return (await codec.getNextFrame()).image;
      }))!;
  addTearDown(image.dispose);
  expect(image.width, 3);
  expect(image.height, 3);
  final _TestImageStreamCompleter streamCompleter = _TestImageStreamCompleter();
  streamCompleter.setData(imageInfo: ImageInfo(image: image));
  final _TestImageProvider imageProvider = _TestImageProvider(streamCompleter: streamCompleter);

  await tester.pumpWidget(
    quality == null
        ? Image(image: imageProvider)
        : Image(image: imageProvider, filterQuality: quality),
  );

  await expectLater(
    find.byType(Image),
    matchesGoldenFile('image_quality_${quality ?? 'default'}.png'),
  );
}

class _TestImageStreamCompleter extends ImageStreamCompleter {
  ImageInfo? _currentImage;
  final Set<ImageStreamListener> listeners = <ImageStreamListener>{};

  @override
  void addListener(ImageStreamListener listener) {
    listeners.add(listener);
    if (_currentImage != null) {
      listener.onImage(_currentImage!.clone(), true);
    }
  }

  @override
  void removeListener(ImageStreamListener listener) {
    listeners.remove(listener);
  }

  void setData({ImageInfo? imageInfo, ImageChunkEvent? chunkEvent}) {
    if (imageInfo != null) {
      _currentImage?.dispose();
      _currentImage = imageInfo;
    }
    final List<ImageStreamListener> localListeners = listeners.toList();
    for (final ImageStreamListener listener in localListeners) {
      if (imageInfo != null) {
        listener.onImage(imageInfo.clone(), false);
      }
      if (chunkEvent != null && listener.onChunk != null) {
        listener.onChunk!(chunkEvent);
      }
    }
  }

  void setError({required Object exception, StackTrace? stackTrace}) {
    final List<ImageStreamListener> localListeners = listeners.toList();
    for (final ImageStreamListener listener in localListeners) {
      listener.onError?.call(exception, stackTrace);
    }
  }
}

class _TestImageProvider extends ImageProvider<Object> {
  _TestImageProvider({ImageStreamCompleter? streamCompleter}) {
    _streamCompleter = streamCompleter ?? OneFrameImageStreamCompleter(_completer.future);
  }

  final Completer<ImageInfo> _completer = Completer<ImageInfo>();
  late ImageStreamCompleter _streamCompleter;

  bool get loadCalled => _loadCallCount > 0;
  int get loadCallCount => _loadCallCount;
  int _loadCallCount = 0;

  @override
  Future<Object> obtainKey(ImageConfiguration configuration) {
    return SynchronousFuture<_TestImageProvider>(this);
  }

  @override
  ImageStreamCompleter loadImage(Object key, ImageDecoderCallback decode) {
    _loadCallCount += 1;
    return _streamCompleter;
  }

  void complete(ui.Image image) {
    _completer.complete(ImageInfo(image: image));
  }

  void fail(Object exception, StackTrace? stackTrace) {
    _completer.completeError(exception, stackTrace);
  }

  @override
  String toString() => '${describeIdentity(this)}()';
}
