// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';
import 'dart:ui' as ui;

import 'package:flutter/painting.dart';
import 'package:flutter_test/flutter_test.dart';

import '../image_data.dart';
import '../rendering/rendering_tester.dart';

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  tearDown(() {
    PaintingBinding.instance.imageCache.clear();
    PaintingBinding.instance.imageCache.clearLiveImages();
  });

  group('ResizeImage', () {
    group('resizing', () {
      test('upscales to the correct dimensions', () async {
        final Uint8List bytes = Uint8List.fromList(kTransparentImage);
        final MemoryImage imageProvider = MemoryImage(bytes);
        final Size rawImageSize = await _resolveAndGetSize(imageProvider);
        expect(rawImageSize, const Size(1, 1));

        const Size resizeDims = Size(14, 7);
        final ResizeImage resizedImage = ResizeImage(
          MemoryImage(bytes),
          width: resizeDims.width.round(),
          height: resizeDims.height.round(),
          allowUpscaling: true,
        );
        const ImageConfiguration resizeConfig = ImageConfiguration(size: resizeDims);
        final Size resizedImageSize = await _resolveAndGetSize(
          resizedImage,
          configuration: resizeConfig,
        );
        expect(resizedImageSize, resizeDims);
      });

      test('downscales to the correct dimensions', () async {
        final Uint8List bytes = Uint8List.fromList(kBlueSquarePng);
        final MemoryImage imageProvider = MemoryImage(bytes);
        final Size rawImageSize = await _resolveAndGetSize(imageProvider);
        expect(rawImageSize, const Size(50, 50));

        const Size resizeDims = Size(25, 25);
        final ResizeImage resizedImage = ResizeImage(
          MemoryImage(bytes),
          width: resizeDims.width.round(),
          height: resizeDims.height.round(),
          allowUpscaling: true,
        );
        const ImageConfiguration resizeConfig = ImageConfiguration(size: resizeDims);
        final Size resizedImageSize = await _resolveAndGetSize(
          resizedImage,
          configuration: resizeConfig,
        );
        expect(resizedImageSize, resizeDims);
      });

      test('refuses upscaling when allowUpscaling=false', () async {
        final Uint8List bytes = Uint8List.fromList(kTransparentImage);
        final MemoryImage imageProvider = MemoryImage(bytes);
        final Size rawImageSize = await _resolveAndGetSize(imageProvider);
        expect(rawImageSize, const Size(1, 1));

        const Size resizeDims = Size(50, 50);
        final ResizeImage resizedImage = ResizeImage(
          MemoryImage(bytes),
          width: resizeDims.width.round(),
          height: resizeDims.height.round(),
        );
        const ImageConfiguration resizeConfig = ImageConfiguration(size: resizeDims);
        final Size resizedImageSize = await _resolveAndGetSize(
          resizedImage,
          configuration: resizeConfig,
        );
        expect(resizedImageSize, const Size(1, 1));
      });

      group('with policy=fit and allowResize=false', () {
        test('constrains square image to bounded portrait rect', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBlueSquarePng));
          await _expectImageSize(rawImage, const Size(50, 50));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            width: 25,
            height: 50,
            policy: ResizeImagePolicy.fit,
          );
          await _expectImageSize(resizedImage, const Size(25, 25));
        });

        test('constrains square image to bounded landscape rect', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBlueSquarePng));
          await _expectImageSize(rawImage, const Size(50, 50));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            width: 50,
            height: 25,
            policy: ResizeImagePolicy.fit,
          );
          await _expectImageSize(resizedImage, const Size(25, 25));
        });

        test('constrains square image to bounded square', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBlueSquarePng));
          await _expectImageSize(rawImage, const Size(50, 50));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            width: 25,
            height: 25,
            policy: ResizeImagePolicy.fit,
          );
          await _expectImageSize(resizedImage, const Size(25, 25));
        });

        test('constrains square image to bounded width', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBlueSquarePng));
          await _expectImageSize(rawImage, const Size(50, 50));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            width: 25,
            policy: ResizeImagePolicy.fit,
          );
          await _expectImageSize(resizedImage, const Size(25, 25));
        });

        test('constrains square image to bounded height', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBlueSquarePng));
          await _expectImageSize(rawImage, const Size(50, 50));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            height: 25,
            policy: ResizeImagePolicy.fit,
          );
          await _expectImageSize(resizedImage, const Size(25, 25));
        });

        test('constrains portrait image to bounded portrait rect', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBluePortraitPng));
          await _expectImageSize(rawImage, const Size(50, 100));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            width: 25,
            height: 60,
            policy: ResizeImagePolicy.fit,
          );
          await _expectImageSize(resizedImage, const Size(25, 50));
        });

        test('constrains portrait image to bounded landscape rect', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBluePortraitPng));
          await _expectImageSize(rawImage, const Size(50, 100));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            width: 60,
            height: 25,
            policy: ResizeImagePolicy.fit,
          );
          await _expectImageSize(resizedImage, const Size(12, 25));
        });

        test('constrains portrait image to bounded square', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBluePortraitPng));
          await _expectImageSize(rawImage, const Size(50, 100));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            width: 25,
            height: 25,
            policy: ResizeImagePolicy.fit,
          );
          await _expectImageSize(resizedImage, const Size(12, 25));
        });

        test('constrains portrait image to bounded width', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBluePortraitPng));
          await _expectImageSize(rawImage, const Size(50, 100));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            width: 25,
            policy: ResizeImagePolicy.fit,
          );
          await _expectImageSize(resizedImage, const Size(25, 50));
        });

        test('constrains portrait image to bounded height', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBluePortraitPng));
          await _expectImageSize(rawImage, const Size(50, 100));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            height: 25,
            policy: ResizeImagePolicy.fit,
          );
          await _expectImageSize(resizedImage, const Size(12, 25));
        });

        test('constrains landscape image to bounded portrait rect', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBlueLandscapePng));
          await _expectImageSize(rawImage, const Size(100, 50));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            width: 25,
            height: 60,
            policy: ResizeImagePolicy.fit,
          );
          await _expectImageSize(resizedImage, const Size(25, 12));
        });

        test('constrains landscape image to bounded landscape rect', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBlueLandscapePng));
          await _expectImageSize(rawImage, const Size(100, 50));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            width: 60,
            height: 25,
            policy: ResizeImagePolicy.fit,
          );
          await _expectImageSize(resizedImage, const Size(50, 25));
        });

        test('constrains landscape image to bounded square', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBlueLandscapePng));
          await _expectImageSize(rawImage, const Size(100, 50));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            width: 25,
            height: 25,
            policy: ResizeImagePolicy.fit,
          );
          await _expectImageSize(resizedImage, const Size(25, 12));
        });

        test('constrains landscape image to bounded width', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBlueLandscapePng));
          await _expectImageSize(rawImage, const Size(100, 50));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            width: 25,
            policy: ResizeImagePolicy.fit,
          );
          await _expectImageSize(resizedImage, const Size(25, 12));
        });

        test('constrains landscape image to bounded height', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBlueLandscapePng));
          await _expectImageSize(rawImage, const Size(100, 50));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            height: 25,
            policy: ResizeImagePolicy.fit,
          );
          await _expectImageSize(resizedImage, const Size(50, 25));
        });

        test('leaves image as-is if constraints are bigger than image', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBlueSquarePng));
          await _expectImageSize(rawImage, const Size(50, 50));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            width: 120,
            height: 100,
            policy: ResizeImagePolicy.fit,
          );
          await _expectImageSize(resizedImage, const Size(50, 50));
        });
      });

      group('with policy=fit and allowResize=true', () {
        test('constrains square image to bounded portrait rect', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBlueSquarePng));
          await _expectImageSize(rawImage, const Size(50, 50));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            width: 100,
            height: 200,
            policy: ResizeImagePolicy.fit,
            allowUpscaling: true,
          );
          await _expectImageSize(resizedImage, const Size(100, 100));
        });

        test('constrains square image to bounded landscape rect', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBlueSquarePng));
          await _expectImageSize(rawImage, const Size(50, 50));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            width: 200,
            height: 100,
            policy: ResizeImagePolicy.fit,
            allowUpscaling: true,
          );
          await _expectImageSize(resizedImage, const Size(100, 100));
        });

        test('constrains square image to bounded square', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBlueSquarePng));
          await _expectImageSize(rawImage, const Size(50, 50));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            width: 100,
            height: 100,
            policy: ResizeImagePolicy.fit,
            allowUpscaling: true,
          );
          await _expectImageSize(resizedImage, const Size(100, 100));
        });

        test('constrains square image to bounded width', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBlueSquarePng));
          await _expectImageSize(rawImage, const Size(50, 50));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            width: 100,
            policy: ResizeImagePolicy.fit,
            allowUpscaling: true,
          );
          await _expectImageSize(resizedImage, const Size(100, 100));
        });

        test('constrains square image to bounded height', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBlueSquarePng));
          await _expectImageSize(rawImage, const Size(50, 50));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            height: 100,
            policy: ResizeImagePolicy.fit,
            allowUpscaling: true,
          );
          await _expectImageSize(resizedImage, const Size(100, 100));
        });

        test('constrains portrait image to bounded portrait rect', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBluePortraitPng));
          await _expectImageSize(rawImage, const Size(50, 100));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            width: 100,
            height: 250,
            policy: ResizeImagePolicy.fit,
            allowUpscaling: true,
          );
          await _expectImageSize(resizedImage, const Size(100, 200));
        });

        test('constrains portrait image to bounded landscape rect', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBluePortraitPng));
          await _expectImageSize(rawImage, const Size(50, 100));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            width: 400,
            height: 200,
            policy: ResizeImagePolicy.fit,
            allowUpscaling: true,
          );
          await _expectImageSize(resizedImage, const Size(100, 200));
        });

        test('constrains portrait image to bounded square', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBluePortraitPng));
          await _expectImageSize(rawImage, const Size(50, 100));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            width: 200,
            height: 200,
            policy: ResizeImagePolicy.fit,
            allowUpscaling: true,
          );
          await _expectImageSize(resizedImage, const Size(100, 200));
        });

        test('constrains portrait image to bounded width', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBluePortraitPng));
          await _expectImageSize(rawImage, const Size(50, 100));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            width: 100,
            policy: ResizeImagePolicy.fit,
            allowUpscaling: true,
          );
          await _expectImageSize(resizedImage, const Size(100, 200));
        });

        test('constrains portrait image to bounded height', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBluePortraitPng));
          await _expectImageSize(rawImage, const Size(50, 100));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            height: 200,
            policy: ResizeImagePolicy.fit,
            allowUpscaling: true,
          );
          await _expectImageSize(resizedImage, const Size(100, 200));
        });

        test('constrains landscape image to bounded portrait rect', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBlueLandscapePng));
          await _expectImageSize(rawImage, const Size(100, 50));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            width: 200,
            height: 400,
            policy: ResizeImagePolicy.fit,
            allowUpscaling: true,
          );
          await _expectImageSize(resizedImage, const Size(200, 100));
        });

        test('constrains landscape image to bounded landscape rect', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBlueLandscapePng));
          await _expectImageSize(rawImage, const Size(100, 50));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            width: 250,
            height: 100,
            policy: ResizeImagePolicy.fit,
            allowUpscaling: true,
          );
          await _expectImageSize(resizedImage, const Size(200, 100));
        });

        test('constrains landscape image to bounded square', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBlueLandscapePng));
          await _expectImageSize(rawImage, const Size(100, 50));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            width: 200,
            height: 200,
            policy: ResizeImagePolicy.fit,
            allowUpscaling: true,
          );
          await _expectImageSize(resizedImage, const Size(200, 100));
        });

        test('constrains landscape image to bounded width', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBlueLandscapePng));
          await _expectImageSize(rawImage, const Size(100, 50));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            width: 200,
            policy: ResizeImagePolicy.fit,
            allowUpscaling: true,
          );
          await _expectImageSize(resizedImage, const Size(200, 100));
        });

        test('constrains landscape image to bounded height', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBlueLandscapePng));
          await _expectImageSize(rawImage, const Size(100, 50));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            height: 100,
            policy: ResizeImagePolicy.fit,
            allowUpscaling: true,
          );
          await _expectImageSize(resizedImage, const Size(200, 100));
        });

        test('shrinks image if constraints are smaller than image', () async {
          final MemoryImage rawImage = MemoryImage(Uint8List.fromList(kBlueSquarePng));
          await _expectImageSize(rawImage, const Size(50, 50));
          final ResizeImage resizedImage = ResizeImage(
            rawImage,
            width: 25,
            height: 30,
            policy: ResizeImagePolicy.fit,
            allowUpscaling: true,
          );
          await _expectImageSize(resizedImage, const Size(25, 25));
        });
      });
    }, skip: isBrowser); // https://github.com/flutter/flutter/issues/73120);

    test('does not resize when no size is passed', () async {
      final Uint8List bytes = Uint8List.fromList(kTransparentImage);
      final MemoryImage imageProvider = MemoryImage(bytes);
      final Size rawImageSize = await _resolveAndGetSize(imageProvider);
      expect(rawImageSize, const Size(1, 1));

      final ImageProvider<Object> resizedImage = ResizeImage.resizeIfNeeded(
        null,
        null,
        imageProvider,
      );
      final Size resizedImageSize = await _resolveAndGetSize(resizedImage);
      expect(resizedImageSize, const Size(1, 1));
    });

    test('stores values', () async {
      final Uint8List bytes = Uint8List.fromList(kTransparentImage);
      final MemoryImage memoryImage = MemoryImage(bytes);
      memoryImage.resolve(ImageConfiguration.empty);
      final ResizeImage resizeImage = ResizeImage(memoryImage, width: 10, height: 20);
      expect(resizeImage.width, 10);
      expect(resizeImage.height, 20);
      expect(resizeImage.imageProvider, memoryImage);
      expect(
        memoryImage.resolve(ImageConfiguration.empty) !=
            resizeImage.resolve(ImageConfiguration.empty),
        true,
      );
    });

    test('takes one dim', () async {
      final Uint8List bytes = Uint8List.fromList(kTransparentImage);
      final MemoryImage memoryImage = MemoryImage(bytes);
      final ResizeImage resizeImage = ResizeImage(memoryImage, width: 10);
      expect(resizeImage.width, 10);
      expect(resizeImage.height, null);
      expect(resizeImage.imageProvider, memoryImage);
      expect(
        memoryImage.resolve(ImageConfiguration.empty) !=
            resizeImage.resolve(ImageConfiguration.empty),
        true,
      );
    });

    test('forms closure', () async {
      final Uint8List bytes = Uint8List.fromList(kTransparentImage);
      final MemoryImage memoryImage = MemoryImage(bytes);
      final ResizeImage resizeImage = ResizeImage(
        memoryImage,
        width: 123,
        height: 321,
        allowUpscaling: true,
      );

      Future<ui.Codec> decode(
        ui.ImmutableBuffer buffer, {
        ui.TargetImageSizeCallback? getTargetSize,
      }) {
        return PaintingBinding.instance.instantiateImageCodecWithSize(
          buffer,
          getTargetSize: (int intrinsicWidth, int intrinsicHeight) {
            expect(getTargetSize, isNotNull);
            final ui.TargetImageSize targetSize = getTargetSize!(intrinsicWidth, intrinsicHeight);
            expect(targetSize.width, 123);
            expect(targetSize.height, 321);
            return targetSize;
          },
        );
      }

      resizeImage.loadImage(await resizeImage.obtainKey(ImageConfiguration.empty), decode);
    });

    test('handles sync obtainKey', () async {
      final Uint8List bytes = Uint8List.fromList(kTransparentImage);
      final MemoryImage memoryImage = MemoryImage(bytes);
      final ResizeImage resizeImage = ResizeImage(memoryImage, width: 123, height: 321);

      bool isAsync = false;
      bool keyObtained = false;
      resizeImage.obtainKey(ImageConfiguration.empty).then((Object key) {
        keyObtained = true;
        expect(isAsync, false);
      });
      isAsync = true;
      expect(isAsync, true);
      expect(keyObtained, true);
    });

    test('handles async obtainKey', () async {
      final Uint8List bytes = Uint8List.fromList(kTransparentImage);
      final _AsyncKeyMemoryImage memoryImage = _AsyncKeyMemoryImage(bytes);
      final ResizeImage resizeImage = ResizeImage(memoryImage, width: 123, height: 321);

      bool isAsync = false;
      final Completer<void> completer = Completer<void>();
      resizeImage.obtainKey(ImageConfiguration.empty).then((Object key) {
        try {
          expect(isAsync, true);
        } finally {
          completer.complete();
        }
      });
      isAsync = true;
      await completer.future;
      expect(isAsync, true);
    });
  });
}

Future<void> _expectImageSize(ImageProvider<Object> imageProvider, Size size) async {
  final Size actualSize = await _resolveAndGetSize(imageProvider);
  expect(actualSize, size);
}

Future<Size> _resolveAndGetSize(
  ImageProvider imageProvider, {
  ImageConfiguration configuration = ImageConfiguration.empty,
}) async {
  final ImageStream stream = imageProvider.resolve(configuration);
  final Completer<Size> completer = Completer<Size>();
  final ImageStreamListener listener = ImageStreamListener((ImageInfo image, bool synchronousCall) {
    final int height = image.image.height;
    final int width = image.image.width;
    completer.complete(Size(width.toDouble(), height.toDouble()));
  });
  stream.addListener(listener);
  return completer.future;
}

// This version of MemoryImage guarantees obtainKey returns a future that has not been
// completed synchronously.
class _AsyncKeyMemoryImage extends MemoryImage {
  const _AsyncKeyMemoryImage(super.bytes);

  @override
  Future<MemoryImage> obtainKey(ImageConfiguration configuration) {
    return Future<MemoryImage>(() => this);
  }
}
