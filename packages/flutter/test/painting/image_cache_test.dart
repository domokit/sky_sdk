// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/painting.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter_test/flutter_test.dart';

import '../rendering/rendering_tester.dart';
import 'mocks_for_image_cache.dart';

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  tearDown(() {
    imageCache
      ..clear()
      ..clearLiveImages()
      ..maximumSize = 1000
      ..maximumSizeBytes = 10485760;
  });

  test('maintains cache size', () async {
    imageCache.maximumSize = 3;

    final TestImageInfo a =
        await extractOneFrame(
              TestImageProvider(
                1,
                1,
                image: await createTestImage(),
              ).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    expect(a.value, equals(1));
    final TestImageInfo b =
        await extractOneFrame(
              TestImageProvider(
                1,
                2,
                image: await createTestImage(),
              ).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    expect(b.value, equals(1));
    final TestImageInfo c =
        await extractOneFrame(
              TestImageProvider(
                1,
                3,
                image: await createTestImage(),
              ).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    expect(c.value, equals(1));
    final TestImageInfo d =
        await extractOneFrame(
              TestImageProvider(
                1,
                4,
                image: await createTestImage(),
              ).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    expect(d.value, equals(1));
    final TestImageInfo e =
        await extractOneFrame(
              TestImageProvider(
                1,
                5,
                image: await createTestImage(),
              ).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    expect(e.value, equals(1));
    final TestImageInfo f =
        await extractOneFrame(
              TestImageProvider(
                1,
                6,
                image: await createTestImage(),
              ).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    expect(f.value, equals(1));

    expect(f, equals(a));

    // cache still only has one entry in it: 1(1)

    final TestImageInfo g =
        await extractOneFrame(
              TestImageProvider(
                2,
                7,
                image: await createTestImage(),
              ).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    expect(g.value, equals(7));

    // cache has two entries in it: 1(1), 2(7)

    final TestImageInfo h =
        await extractOneFrame(
              TestImageProvider(
                1,
                8,
                image: await createTestImage(),
              ).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    expect(h.value, equals(1));

    // cache still has two entries in it: 2(7), 1(1)

    final TestImageInfo i =
        await extractOneFrame(
              TestImageProvider(
                3,
                9,
                image: await createTestImage(),
              ).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    expect(i.value, equals(9));

    // cache has three entries in it: 2(7), 1(1), 3(9)

    final TestImageInfo j =
        await extractOneFrame(
              TestImageProvider(
                1,
                10,
                image: await createTestImage(),
              ).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    expect(j.value, equals(1));

    // cache still has three entries in it: 2(7), 3(9), 1(1)

    final TestImageInfo k =
        await extractOneFrame(
              TestImageProvider(
                4,
                11,
                image: await createTestImage(),
              ).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    expect(k.value, equals(11));

    // cache has three entries: 3(9), 1(1), 4(11)

    final TestImageInfo l =
        await extractOneFrame(
              TestImageProvider(
                1,
                12,
                image: await createTestImage(),
              ).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    expect(l.value, equals(1));

    // cache has three entries: 3(9), 4(11), 1(1)

    final TestImageInfo m =
        await extractOneFrame(
              TestImageProvider(
                2,
                13,
                image: await createTestImage(),
              ).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    expect(m.value, equals(13));

    // cache has three entries: 4(11), 1(1), 2(13)

    final TestImageInfo n =
        await extractOneFrame(
              TestImageProvider(
                3,
                14,
                image: await createTestImage(),
              ).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    expect(n.value, equals(14));

    // cache has three entries: 1(1), 2(13), 3(14)

    final TestImageInfo o =
        await extractOneFrame(
              TestImageProvider(
                4,
                15,
                image: await createTestImage(),
              ).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    expect(o.value, equals(15));

    // cache has three entries: 2(13), 3(14), 4(15)

    final TestImageInfo p =
        await extractOneFrame(
              TestImageProvider(
                1,
                16,
                image: await createTestImage(),
              ).resolve(ImageConfiguration.empty),
            )
            as TestImageInfo;
    expect(p.value, equals(16));

    // cache has three entries: 3(14), 4(15), 1(16)
  });

  test('clear removes all images and resets cache size', () async {
    final ui.Image testImage = await createTestImage(width: 8, height: 8);

    expect(imageCache.currentSize, 0);
    expect(imageCache.currentSizeBytes, 0);

    await extractOneFrame(
      TestImageProvider(1, 1, image: testImage).resolve(ImageConfiguration.empty),
    );
    await extractOneFrame(
      TestImageProvider(2, 2, image: testImage).resolve(ImageConfiguration.empty),
    );

    expect(imageCache.currentSize, 2);
    expect(imageCache.currentSizeBytes, 256 * 2);

    imageCache.clear();

    expect(imageCache.currentSize, 0);
    expect(imageCache.currentSizeBytes, 0);
  });

  test('evicts individual images', () async {
    final ui.Image testImage = await createTestImage(width: 8, height: 8);
    await extractOneFrame(
      TestImageProvider(1, 1, image: testImage).resolve(ImageConfiguration.empty),
    );
    await extractOneFrame(
      TestImageProvider(2, 2, image: testImage).resolve(ImageConfiguration.empty),
    );

    expect(imageCache.currentSize, 2);
    expect(imageCache.currentSizeBytes, 256 * 2);
    expect(imageCache.evict(1), true);
    expect(imageCache.currentSize, 1);
    expect(imageCache.currentSizeBytes, 256);
  });

  test('Do not cache large images', () async {
    final ui.Image testImage = await createTestImage(width: 8, height: 8);

    imageCache.maximumSizeBytes = 1;
    await extractOneFrame(
      TestImageProvider(1, 1, image: testImage).resolve(ImageConfiguration.empty),
    );
    expect(imageCache.currentSize, 0);
    expect(imageCache.currentSizeBytes, 0);
    expect(imageCache.maximumSizeBytes, 1);
  });

  test('Returns null if an error is caught resolving an image', () {
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

    final ErrorImageProvider errorImage = ErrorImageProvider();
    expect(
      () =>
          imageCache.putIfAbsent(errorImage, () => errorImage.loadBuffer(errorImage, basicDecoder)),
      throwsA(isA<Error>()),
    );
    bool caughtError = false;
    final ImageStreamCompleter? result = imageCache.putIfAbsent(
      errorImage,
      () => errorImage.loadBuffer(errorImage, basicDecoder),
      onError: (dynamic error, StackTrace? stackTrace) {
        caughtError = true;
      },
    );
    expect(result, null);
    expect(caughtError, true);
  });

  test('already pending image is returned when it is put into the cache again', () async {
    final ui.Image testImage = await createTestImage(width: 8, height: 8);

    final TestImageStreamCompleter completer1 = TestImageStreamCompleter();
    final TestImageStreamCompleter completer2 = TestImageStreamCompleter();

    final TestImageStreamCompleter resultingCompleter1 =
        imageCache.putIfAbsent(testImage, () {
              return completer1;
            })!
            as TestImageStreamCompleter;
    final TestImageStreamCompleter resultingCompleter2 =
        imageCache.putIfAbsent(testImage, () {
              return completer2;
            })!
            as TestImageStreamCompleter;

    expect(resultingCompleter1, completer1);
    expect(resultingCompleter2, completer1);
  });

  test('pending image is removed when cache is cleared', () async {
    final ui.Image testImage = await createTestImage(width: 8, height: 8);

    final TestImageStreamCompleter completer1 = TestImageStreamCompleter();
    final TestImageStreamCompleter completer2 = TestImageStreamCompleter();

    final TestImageStreamCompleter resultingCompleter1 =
        imageCache.putIfAbsent(testImage, () {
              return completer1;
            })!
            as TestImageStreamCompleter;

    // Make the image seem live.
    final ImageStreamListener listener = ImageStreamListener((_, __) {});
    completer1.addListener(listener);

    expect(imageCache.statusForKey(testImage).pending, true);
    expect(imageCache.statusForKey(testImage).live, true);
    imageCache.clear();
    expect(imageCache.statusForKey(testImage).pending, false);
    expect(imageCache.statusForKey(testImage).live, true);
    imageCache.clearLiveImages();
    expect(imageCache.statusForKey(testImage).pending, false);
    expect(imageCache.statusForKey(testImage).live, false);

    final TestImageStreamCompleter resultingCompleter2 =
        imageCache.putIfAbsent(testImage, () {
              return completer2;
            })!
            as TestImageStreamCompleter;

    expect(resultingCompleter1, completer1);
    expect(resultingCompleter2, completer2);
  });

  test('pending image is removed when image is evicted', () async {
    final ui.Image testImage = await createTestImage(width: 8, height: 8);

    final TestImageStreamCompleter completer1 = TestImageStreamCompleter();
    final TestImageStreamCompleter completer2 = TestImageStreamCompleter();

    final TestImageStreamCompleter resultingCompleter1 =
        imageCache.putIfAbsent(testImage, () {
              return completer1;
            })!
            as TestImageStreamCompleter;

    imageCache.evict(testImage);

    final TestImageStreamCompleter resultingCompleter2 =
        imageCache.putIfAbsent(testImage, () {
              return completer2;
            })!
            as TestImageStreamCompleter;

    expect(resultingCompleter1, completer1);
    expect(resultingCompleter2, completer2);
  });

  test("failed image can successfully be removed from the cache's pending images", () async {
    final ui.Image testImage = await createTestImage(width: 8, height: 8);

    FailingTestImageProvider(1, 1, image: testImage)
        .resolve(ImageConfiguration.empty)
        .addListener(
          ImageStreamListener(
            (ImageInfo image, bool synchronousCall) {
              fail('Image should not complete successfully');
            },
            onError: (dynamic exception, StackTrace? stackTrace) {
              final bool evictionResult = imageCache.evict(1);
              expect(evictionResult, isTrue);
            },
          ),
        );
    // yield an event turn so that async work can complete.
    await null;
  });

  test('containsKey - pending', () async {
    final ui.Image testImage = await createTestImage(width: 8, height: 8);

    final TestImageStreamCompleter completer1 = TestImageStreamCompleter();

    final TestImageStreamCompleter resultingCompleter1 =
        imageCache.putIfAbsent(testImage, () {
              return completer1;
            })!
            as TestImageStreamCompleter;

    expect(resultingCompleter1, completer1);
    expect(imageCache.containsKey(testImage), true);
  });

  test('containsKey - completed', () async {
    final ui.Image testImage = await createTestImage(width: 8, height: 8);

    final TestImageStreamCompleter completer1 = TestImageStreamCompleter();

    final TestImageStreamCompleter resultingCompleter1 =
        imageCache.putIfAbsent(testImage, () {
              return completer1;
            })!
            as TestImageStreamCompleter;

    // Mark as complete
    completer1.testSetImage(testImage);

    expect(resultingCompleter1, completer1);
    expect(imageCache.containsKey(testImage), true);
  });

  test('putIfAbsent updates LRU properties of a live image', () async {
    imageCache.maximumSize = 1;
    final ui.Image testImage = await createTestImage(width: 8, height: 8);
    final ui.Image testImage2 = await createTestImage(width: 10, height: 10);

    final TestImageStreamCompleter completer1 = TestImageStreamCompleter()..testSetImage(testImage);
    final TestImageStreamCompleter completer2 =
        TestImageStreamCompleter()..testSetImage(testImage2);

    completer1.addListener(ImageStreamListener((ImageInfo info, bool syncCall) {}));

    final TestImageStreamCompleter resultingCompleter1 =
        imageCache.putIfAbsent(testImage, () {
              return completer1;
            })!
            as TestImageStreamCompleter;

    expect(imageCache.statusForKey(testImage).pending, false);
    expect(imageCache.statusForKey(testImage).keepAlive, true);
    expect(imageCache.statusForKey(testImage).live, true);
    expect(imageCache.statusForKey(testImage2).untracked, true);
    final TestImageStreamCompleter resultingCompleter2 =
        imageCache.putIfAbsent(testImage2, () {
              return completer2;
            })!
            as TestImageStreamCompleter;

    expect(imageCache.statusForKey(testImage).pending, false);
    expect(imageCache.statusForKey(testImage).keepAlive, false); // evicted
    expect(imageCache.statusForKey(testImage).live, true);
    expect(imageCache.statusForKey(testImage2).pending, false);
    expect(imageCache.statusForKey(testImage2).keepAlive, true); // took the LRU spot.
    expect(imageCache.statusForKey(testImage2).live, false); // no listeners

    expect(resultingCompleter1, completer1);
    expect(resultingCompleter2, completer2);
  });

  test('Live image cache avoids leaks of unlistened streams', () async {
    imageCache.maximumSize = 3;

    TestImageProvider(1, 1, image: await createTestImage()).resolve(ImageConfiguration.empty);
    TestImageProvider(2, 2, image: await createTestImage()).resolve(ImageConfiguration.empty);
    TestImageProvider(3, 3, image: await createTestImage()).resolve(ImageConfiguration.empty);
    TestImageProvider(4, 4, image: await createTestImage()).resolve(ImageConfiguration.empty);
    TestImageProvider(5, 5, image: await createTestImage()).resolve(ImageConfiguration.empty);
    TestImageProvider(6, 6, image: await createTestImage()).resolve(ImageConfiguration.empty);

    // wait an event loop to let image resolution process.
    await null;

    expect(imageCache.currentSize, 3);
    expect(imageCache.liveImageCount, 0);
  });

  test('Disabled image cache does not leak live images', () async {
    imageCache.maximumSize = 0;

    TestImageProvider(1, 1, image: await createTestImage()).resolve(ImageConfiguration.empty);
    TestImageProvider(2, 2, image: await createTestImage()).resolve(ImageConfiguration.empty);
    TestImageProvider(3, 3, image: await createTestImage()).resolve(ImageConfiguration.empty);
    TestImageProvider(4, 4, image: await createTestImage()).resolve(ImageConfiguration.empty);
    TestImageProvider(5, 5, image: await createTestImage()).resolve(ImageConfiguration.empty);
    TestImageProvider(6, 6, image: await createTestImage()).resolve(ImageConfiguration.empty);

    // wait an event loop to let image resolution process.
    await null;

    expect(imageCache.currentSize, 0);
    expect(imageCache.liveImageCount, 0);
  });

  test('Clearing image cache does not leak live images', () async {
    imageCache.maximumSize = 1;

    final ui.Image testImage1 = await createTestImage(width: 8, height: 8);
    final ui.Image testImage2 = await createTestImage(width: 10, height: 10);

    final TestImageStreamCompleter completer1 = TestImageStreamCompleter();
    final TestImageStreamCompleter completer2 =
        TestImageStreamCompleter()..testSetImage(testImage2);

    imageCache.putIfAbsent(testImage1, () => completer1);
    expect(imageCache.statusForKey(testImage1).pending, true);
    expect(imageCache.statusForKey(testImage1).live, true);

    imageCache.clear();
    expect(imageCache.statusForKey(testImage1).pending, false);
    expect(imageCache.statusForKey(testImage1).live, false);

    completer1.testSetImage(testImage1);
    expect(imageCache.statusForKey(testImage1).keepAlive, false);
    expect(imageCache.statusForKey(testImage1).live, false);

    imageCache.putIfAbsent(testImage2, () => completer2);
    expect(imageCache.statusForKey(testImage1).tracked, false); // evicted
    expect(imageCache.statusForKey(testImage2).tracked, true);
  });

  test('Evicting a pending image clears the live image by default', () async {
    final ui.Image testImage = await createTestImage(width: 8, height: 8);

    final TestImageStreamCompleter completer1 = TestImageStreamCompleter();

    imageCache.putIfAbsent(testImage, () => completer1);
    expect(imageCache.statusForKey(testImage).pending, true);
    expect(imageCache.statusForKey(testImage).live, true);
    expect(imageCache.statusForKey(testImage).keepAlive, false);

    imageCache.evict(testImage);
    expect(imageCache.statusForKey(testImage).untracked, true);
  });

  test(
    'Evicting a pending image does clear the live image when includeLive is false and only cache listening',
    () async {
      final ui.Image testImage = await createTestImage(width: 8, height: 8);

      final TestImageStreamCompleter completer1 = TestImageStreamCompleter();

      imageCache.putIfAbsent(testImage, () => completer1);
      expect(imageCache.statusForKey(testImage).pending, true);
      expect(imageCache.statusForKey(testImage).live, true);
      expect(imageCache.statusForKey(testImage).keepAlive, false);

      imageCache.evict(testImage, includeLive: false);
      expect(imageCache.statusForKey(testImage).pending, false);
      expect(imageCache.statusForKey(testImage).live, false);
      expect(imageCache.statusForKey(testImage).keepAlive, false);
    },
  );

  test(
    'Evicting a pending image does clear the live image when includeLive is false and some other listener',
    () async {
      final ui.Image testImage = await createTestImage(width: 8, height: 8);

      final TestImageStreamCompleter completer1 = TestImageStreamCompleter();

      imageCache.putIfAbsent(testImage, () => completer1);
      expect(imageCache.statusForKey(testImage).pending, true);
      expect(imageCache.statusForKey(testImage).live, true);
      expect(imageCache.statusForKey(testImage).keepAlive, false);

      completer1.addListener(ImageStreamListener((_, __) {}));
      imageCache.evict(testImage, includeLive: false);
      expect(imageCache.statusForKey(testImage).pending, false);
      expect(imageCache.statusForKey(testImage).live, true);
      expect(imageCache.statusForKey(testImage).keepAlive, false);
    },
  );

  test('Evicting a completed image does clear the live image by default', () async {
    final ui.Image testImage = await createTestImage(width: 8, height: 8);

    final TestImageStreamCompleter completer1 =
        TestImageStreamCompleter()
          ..testSetImage(testImage)
          ..addListener(ImageStreamListener((ImageInfo info, bool syncCall) {}));

    imageCache.putIfAbsent(testImage, () => completer1);
    expect(imageCache.statusForKey(testImage).pending, false);
    expect(imageCache.statusForKey(testImage).live, true);
    expect(imageCache.statusForKey(testImage).keepAlive, true);

    imageCache.evict(testImage);
    expect(imageCache.statusForKey(testImage).untracked, true);
  });

  test(
    'Evicting a completed image does not clear the live image when includeLive is set to false',
    () async {
      final ui.Image testImage = await createTestImage(width: 8, height: 8);

      final TestImageStreamCompleter completer1 =
          TestImageStreamCompleter()
            ..testSetImage(testImage)
            ..addListener(ImageStreamListener((ImageInfo info, bool syncCall) {}));

      imageCache.putIfAbsent(testImage, () => completer1);
      expect(imageCache.statusForKey(testImage).pending, false);
      expect(imageCache.statusForKey(testImage).live, true);
      expect(imageCache.statusForKey(testImage).keepAlive, true);

      imageCache.evict(testImage, includeLive: false);
      expect(imageCache.statusForKey(testImage).pending, false);
      expect(imageCache.statusForKey(testImage).live, true);
      expect(imageCache.statusForKey(testImage).keepAlive, false);
    },
  );

  test('Clearing liveImages removes callbacks', () async {
    final ui.Image testImage = await createTestImage(width: 8, height: 8);

    final ImageStreamListener listener = ImageStreamListener((ImageInfo info, bool syncCall) {});

    final TestImageStreamCompleter completer1 =
        TestImageStreamCompleter()
          ..testSetImage(testImage)
          ..addListener(listener);

    final TestImageStreamCompleter completer2 =
        TestImageStreamCompleter()
          ..testSetImage(testImage)
          ..addListener(listener);

    imageCache.putIfAbsent(testImage, () => completer1);
    expect(imageCache.statusForKey(testImage).pending, false);
    expect(imageCache.statusForKey(testImage).live, true);
    expect(imageCache.statusForKey(testImage).keepAlive, true);

    imageCache.clear();
    imageCache.clearLiveImages();
    expect(imageCache.statusForKey(testImage).pending, false);
    expect(imageCache.statusForKey(testImage).live, false);
    expect(imageCache.statusForKey(testImage).keepAlive, false);

    imageCache.putIfAbsent(testImage, () => completer2);
    expect(imageCache.statusForKey(testImage).pending, false);
    expect(imageCache.statusForKey(testImage).live, true);
    expect(imageCache.statusForKey(testImage).keepAlive, true);

    completer1.removeListener(listener);

    expect(imageCache.statusForKey(testImage).pending, false);
    expect(imageCache.statusForKey(testImage).live, true);
    expect(imageCache.statusForKey(testImage).keepAlive, true);
  });

  test('Live image gets size updated', () async {
    // Add an image to the cache in pending state
    // Complete it once it is in there as live
    // Evict it but leave the live one.
    // Add it again.
    // If the live image did not track the size properly, the last line of
    // this test will fail.

    final ui.Image testImage = await createTestImage(width: 8, height: 8);
    const int testImageSize = 8 * 8 * 4;

    final ImageStreamListener listener = ImageStreamListener((ImageInfo info, bool syncCall) {});

    final TestImageStreamCompleter completer1 = TestImageStreamCompleter()..addListener(listener);

    imageCache.putIfAbsent(testImage, () => completer1);
    expect(imageCache.statusForKey(testImage).pending, true);
    expect(imageCache.statusForKey(testImage).live, true);
    expect(imageCache.statusForKey(testImage).keepAlive, false);
    expect(imageCache.currentSizeBytes, 0);

    completer1.testSetImage(testImage);

    expect(imageCache.statusForKey(testImage).pending, false);
    expect(imageCache.statusForKey(testImage).live, true);
    expect(imageCache.statusForKey(testImage).keepAlive, true);
    expect(imageCache.currentSizeBytes, testImageSize);

    imageCache.evict(testImage, includeLive: false);

    expect(imageCache.statusForKey(testImage).pending, false);
    expect(imageCache.statusForKey(testImage).live, true);
    expect(imageCache.statusForKey(testImage).keepAlive, false);
    expect(imageCache.currentSizeBytes, 0);

    imageCache.putIfAbsent(testImage, () => completer1);

    expect(imageCache.statusForKey(testImage).pending, false);
    expect(imageCache.statusForKey(testImage).live, true);
    expect(imageCache.statusForKey(testImage).keepAlive, true);
    expect(imageCache.currentSizeBytes, testImageSize);
  });

  test('Image is obtained and disposed of properly for cache', () async {
    const int key = 1;
    final ui.Image testImage = await createTestImage(width: 8, height: 8, cache: false);
    expect(testImage.debugGetOpenHandleStackTraces()!.length, 1);

    late ImageInfo imageInfo;
    final ImageStreamListener listener = ImageStreamListener((ImageInfo info, bool syncCall) {
      imageInfo = info;
    });

    final TestImageStreamCompleter completer = TestImageStreamCompleter();

    completer.addListener(listener);
    imageCache.putIfAbsent(key, () => completer);

    expect(testImage.debugGetOpenHandleStackTraces()!.length, 1);

    // This should cause keepAlive to be set to true.
    completer.testSetImage(testImage);
    expect(imageInfo, isNotNull);
    // +1 ImageStreamCompleter
    expect(testImage.debugGetOpenHandleStackTraces()!.length, 2);

    completer.removeListener(listener);

    // Force us to the end of the frame.
    SchedulerBinding.instance.scheduleFrame();
    await SchedulerBinding.instance.endOfFrame;

    expect(testImage.debugGetOpenHandleStackTraces()!.length, 2);

    expect(imageCache.evict(key), true);

    // Force us to the end of the frame.
    SchedulerBinding.instance.scheduleFrame();
    await SchedulerBinding.instance.endOfFrame;

    // -1 _CachedImage
    // -1 ImageStreamCompleter
    expect(testImage.debugGetOpenHandleStackTraces()!.length, 1);

    imageInfo.dispose();
    expect(testImage.debugGetOpenHandleStackTraces()!.length, 0);
  }, skip: kIsWeb); // https://github.com/flutter/flutter/issues/87442

  test(
    'Image is obtained and disposed of properly for cache when listener is still active',
    () async {
      const int key = 1;
      final ui.Image testImage = await createTestImage(width: 8, height: 8, cache: false);
      expect(testImage.debugGetOpenHandleStackTraces()!.length, 1);

      late ImageInfo imageInfo;
      final ImageStreamListener listener = ImageStreamListener((ImageInfo info, bool syncCall) {
        imageInfo = info;
      });

      final TestImageStreamCompleter completer = TestImageStreamCompleter();

      completer.addListener(listener);
      imageCache.putIfAbsent(key, () => completer);

      expect(testImage.debugGetOpenHandleStackTraces()!.length, 1);

      // This should cause keepAlive to be set to true.
      completer.testSetImage(testImage);
      expect(imageInfo, isNotNull);
      // Just our imageInfo and the completer.
      expect(testImage.debugGetOpenHandleStackTraces()!.length, 2);

      expect(imageCache.evict(key), true);

      // Force us to the end of the frame.
      SchedulerBinding.instance.scheduleFrame();
      await SchedulerBinding.instance.endOfFrame;

      // Live image still around since there's still a listener, and the listener
      // should be holding a handle.
      expect(testImage.debugGetOpenHandleStackTraces()!.length, 2);
      completer.removeListener(listener);

      expect(testImage.debugGetOpenHandleStackTraces()!.length, 1);
      imageInfo.dispose();
      expect(testImage.debugGetOpenHandleStackTraces()!.length, 0);
    },
    skip: kIsWeb, // https://github.com/flutter/flutter/issues/87442
  );

  test('clear does not leave pending images stuck', () async {
    final ui.Image testImage = await createTestImage(width: 8, height: 8);

    final TestImageStreamCompleter completer1 = TestImageStreamCompleter();

    imageCache.putIfAbsent(testImage, () {
      return completer1;
    });

    expect(imageCache.statusForKey(testImage).pending, true);
    expect(imageCache.statusForKey(testImage).live, true);
    expect(imageCache.statusForKey(testImage).keepAlive, false);

    imageCache.clear();

    // No one else is listening to the completer. It should not be considered
    // live anymore.

    expect(imageCache.statusForKey(testImage).pending, false);
    expect(imageCache.statusForKey(testImage).live, false);
    expect(imageCache.statusForKey(testImage).keepAlive, false);
  });
}
