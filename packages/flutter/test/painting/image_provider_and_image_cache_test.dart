// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/painting.dart';
import 'package:flutter_test/flutter_test.dart';

import '../image_data.dart';
import '../rendering/rendering_tester.dart';
import 'mocks_for_image_cache.dart';

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  Future<ui.Codec> basicDecoder(
    ui.ImmutableBuffer bytes, {
    int? cacheWidth,
    int? cacheHeight,
    bool? allowUpscaling,
  }) {
    return PaintingBinding.instance.instantiateImageCodecFromBuffer(
      bytes,
      cacheWidth: cacheWidth,
      cacheHeight: cacheHeight,
      allowUpscaling: allowUpscaling ?? false,
    );
  }

  FlutterExceptionHandler? oldError;
  setUp(() {
    oldError = FlutterError.onError;
  });

  tearDown(() {
    FlutterError.onError = oldError;
    PaintingBinding.instance.imageCache.clear();
    PaintingBinding.instance.imageCache.clearLiveImages();
  });

  tearDown(() {
    imageCache.clear();
  });

  test('AssetImageProvider - evicts on failure to load', () async {
    final Completer<FlutterError> error = Completer<FlutterError>();
    FlutterError.onError = (FlutterErrorDetails details) {
      error.complete(details.exception as FlutterError);
    };

    const ImageProvider provider = ExactAssetImage('does-not-exist');
    final Object key = await provider.obtainKey(ImageConfiguration.empty);
    expect(imageCache.statusForKey(provider).untracked, true);
    expect(imageCache.pendingImageCount, 0);

    provider.resolve(ImageConfiguration.empty);

    expect(imageCache.statusForKey(key).pending, true);
    expect(imageCache.pendingImageCount, 1);

    await error.future;

    expect(imageCache.statusForKey(provider).untracked, true);
    expect(imageCache.pendingImageCount, 0);
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/56314

  test('ImageProvider can evict images', () async {
    final Uint8List bytes = Uint8List.fromList(kTransparentImage);
    final MemoryImage imageProvider = MemoryImage(bytes);
    final ImageStream stream = imageProvider.resolve(ImageConfiguration.empty);
    final Completer<void> completer = Completer<void>();
    stream.addListener(
      ImageStreamListener((ImageInfo info, bool syncCall) => completer.complete()),
    );
    await completer.future;

    expect(imageCache.currentSize, 1);
    expect(await MemoryImage(bytes).evict(), true);
    expect(imageCache.currentSize, 0);
  });

  test('ImageProvider.evict respects the provided ImageCache', () async {
    final ImageCache otherCache = ImageCache();
    final Uint8List bytes = Uint8List.fromList(kTransparentImage);
    final MemoryImage imageProvider = MemoryImage(bytes);
    final ImageStreamCompleter cacheStream =
        otherCache.putIfAbsent(
          imageProvider,
          () => imageProvider.loadBuffer(imageProvider, basicDecoder),
        )!;
    final ImageStream stream = imageProvider.resolve(ImageConfiguration.empty);
    final Completer<void> completer = Completer<void>();
    final Completer<void> cacheCompleter = Completer<void>();
    stream.addListener(
      ImageStreamListener((ImageInfo info, bool syncCall) {
        completer.complete();
      }),
    );
    cacheStream.addListener(
      ImageStreamListener((ImageInfo info, bool syncCall) {
        cacheCompleter.complete();
      }),
    );
    await Future.wait(<Future<void>>[completer.future, cacheCompleter.future]);

    expect(otherCache.currentSize, 1);
    expect(imageCache.currentSize, 1);
    expect(await imageProvider.evict(cache: otherCache), true);
    expect(otherCache.currentSize, 0);
    expect(imageCache.currentSize, 1);
  });

  test('ImageProvider errors can always be caught', () async {
    final ErrorImageProvider imageProvider = ErrorImageProvider();
    final Completer<bool> caughtError = Completer<bool>();
    FlutterError.onError = (FlutterErrorDetails details) {
      caughtError.complete(false);
    };
    final ImageStream stream = imageProvider.resolve(ImageConfiguration.empty);
    stream.addListener(
      ImageStreamListener(
        (ImageInfo info, bool syncCall) {
          caughtError.complete(false);
        },
        onError: (dynamic error, StackTrace? stackTrace) {
          caughtError.complete(true);
        },
      ),
    );
    expect(await caughtError.future, true);
  });
}
