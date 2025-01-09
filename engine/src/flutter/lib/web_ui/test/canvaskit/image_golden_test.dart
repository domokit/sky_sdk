// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:js_interop';
import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

import 'common.dart';
import 'test_data.dart';

List<TestCodec>? testCodecs;

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

abstract class TestCodec {
  TestCodec({required this.description});
  final String description;

  ui.Codec? _cachedCodec;

  Future<ui.Codec> getCodec() async => _cachedCodec ??= await createCodec();

  Future<ui.Codec> createCodec();
}

abstract class TestFileCodec extends TestCodec {
  TestFileCodec.fromTestFile(this.testFile, {required super.description});

  final String testFile;

  Future<ui.Codec> createCodecFromTestFile(String testFile);

  @override
  Future<ui.Codec> createCodec() {
    return createCodecFromTestFile(testFile);
  }
}

class UrlTestCodec extends TestFileCodec {
  UrlTestCodec(super.testFile, this.codecFactory, String function)
    : super.fromTestFile(description: 'created with $function("$testFile")');

  final Future<ui.Codec> Function(String) codecFactory;

  @override
  Future<ui.Codec> createCodecFromTestFile(String testFile) {
    return codecFactory(testFile);
  }
}

class FetchTestCodec extends TestFileCodec {
  FetchTestCodec(super.testFile, this.codecFactory, String function)
    : super.fromTestFile(
        description:
            'created with $function from bytes '
            'fetch()\'ed from "$testFile"',
      );

  final Future<ui.Codec> Function(Uint8List) codecFactory;

  @override
  Future<ui.Codec> createCodecFromTestFile(String testFile) async {
    final HttpFetchResponse response = await httpFetch(testFile);

    if (!response.hasPayload) {
      throw Exception('Unable to fetch() image test file "$testFile"');
    }

    final Uint8List responseBytes = await response.asUint8List();
    return codecFactory(responseBytes);
  }
}

class BitmapTestCodec extends TestFileCodec {
  BitmapTestCodec(super.testFile, this.codecFactory, String function)
    : super.fromTestFile(
        description:
            'created with $function from ImageBitmap'
            ' created from "$testFile"',
      );

  final Future<ui.Image> Function(DomImageBitmap) codecFactory;

  @override
  Future<ui.Codec> createCodecFromTestFile(String testFile) async {
    final DomHTMLImageElement imageElement = createDomHTMLImageElement();
    imageElement.src = testFile;
    setJsProperty<String>(imageElement, 'decoding', 'async');

    await imageElement.decode();

    final DomImageBitmap bitmap = await createImageBitmap(imageElement as JSObject, (
      x: 0,
      y: 0,
      width: imageElement.naturalWidth.toInt(),
      height: imageElement.naturalHeight.toInt(),
    ));

    final ui.Image image = await codecFactory(bitmap);
    return BitmapSingleFrameCodec(bitmap, image);
  }
}

class BitmapSingleFrameCodec implements ui.Codec {
  BitmapSingleFrameCodec(this.bitmap, this.image);

  final DomImageBitmap bitmap;
  final ui.Image image;

  @override
  void dispose() {
    image.dispose();
    bitmap.close();
  }

  @override
  int get frameCount => 1;

  @override
  Future<ui.FrameInfo> getNextFrame() async {
    return SingleFrameInfo(image);
  }

  @override
  int get repetitionCount => 0;
}

Future<void> testMain() async {
  Future<List<TestCodec>> createTestCodecs({
    int testTargetWidth = 300,
    int testTargetHeight = 300,
  }) async {
    final HttpFetchResponse listingResponse = await httpFetch('/test_images/');
    final List<String> testFiles = (await listingResponse.json() as List<dynamic>).cast<String>();

    // Sanity-check the test file list. If suddenly test files are moved or
    // deleted, and the test server returns an empty list, or is missing some
    // important test files, we want to know.
    assert(testFiles.isNotEmpty);
    assert(testFiles.any((String testFile) => testFile.endsWith('.jpg')));
    assert(testFiles.any((String testFile) => testFile.endsWith('.png')));
    assert(testFiles.any((String testFile) => testFile.endsWith('.gif')));
    assert(testFiles.any((String testFile) => testFile.endsWith('.webp')));
    assert(testFiles.any((String testFile) => testFile.endsWith('.bmp')));

    final List<TestCodec> testCodecs = <TestCodec>[];
    for (final String testFile in testFiles) {
      testCodecs.add(
        UrlTestCodec(
          testFile,
          (String file) =>
              renderer.instantiateImageCodecFromUrl(Uri.tryParse('/test_images/$file')!),
          'renderer.instantiateImageFromUrl',
        ),
      );
      testCodecs.add(
        FetchTestCodec(
          '/test_images/$testFile',
          (Uint8List bytes) => renderer.instantiateImageCodec(bytes),
          'renderer.instantiateImageCodec',
        ),
      );
      testCodecs.add(
        FetchTestCodec(
          '/test_images/$testFile',
          (Uint8List bytes) => renderer.instantiateImageCodec(
            bytes,
            targetWidth: testTargetWidth,
            targetHeight: testTargetHeight,
          ),
          'renderer.instantiateImageCodec '
              '($testTargetWidth x $testTargetHeight)',
        ),
      );
      testCodecs.add(
        BitmapTestCodec(
          'test_images/$testFile',
          (DomImageBitmap bitmap) async => renderer.createImageFromImageBitmap(bitmap),
          'renderer.createImageFromImageBitmap',
        ),
      );
    }

    return testCodecs;
  }

  testCodecs = await createTestCodecs();

  group('CanvasKit Images', () {
    setUpCanvasKitTest(withImplicitView: true);

    tearDown(() {
      mockHttpFetchResponseFactory = null;
    });

    group('Codecs', () {
      for (final TestCodec testCodec in testCodecs!) {
        test('${testCodec.description} can create an image', () async {
          try {
            final ui.Codec codec = await testCodec.getCodec();
            final ui.FrameInfo frameInfo = await codec.getNextFrame();
            final ui.Image image = frameInfo.image;
            expect(image, isNotNull);
            expect(image.width, isNonZero);
            expect(image.height, isNonZero);
            expect(image.colorSpace, isNotNull);
          } catch (e) {
            throw TestFailure('Failed to get image for ${testCodec.description}: $e');
          }
        });

        test('${testCodec.description} can be decoded with toByteData', () async {
          ui.Image image;
          try {
            final ui.Codec codec = await testCodec.getCodec();
            final ui.FrameInfo frameInfo = await codec.getNextFrame();
            image = frameInfo.image;
          } catch (e) {
            throw TestFailure('Failed to get image for ${testCodec.description}: $e');
          }

          final ByteData? byteData = await image.toByteData();
          expect(
            byteData,
            isNotNull,
            reason: '${testCodec.description} toByteData() should not be null',
          );
          expect(
            byteData!.lengthInBytes,
            isNonZero,
            reason: '${testCodec.description} toByteData() should not be empty',
          );
          expect(
            byteData.buffer.asUint8List().any((int byte) => byte > 0),
            isTrue,
            reason:
                '${testCodec.description} toByteData() should '
                'contain nonzero value',
          );
        });
      }
    });

    test('crossOrigin requests cause an error', () async {
      final String otherOrigin = domWindow.location.origin.replaceAll('localhost', '127.0.0.1');
      bool gotError = false;
      try {
        final ui.Codec _ = await renderer.instantiateImageCodecFromUrl(
          Uri.parse('$otherOrigin/test_images/1x1.png'),
        );
      } catch (e) {
        gotError = true;
      }
      expect(gotError, isTrue, reason: 'Should have got CORS error');
    });

    _testCkAnimatedImage();

    test('isAvif', () {
      expect(isAvif(Uint8List.fromList(<int>[])), isFalse);
      expect(isAvif(Uint8List.fromList(<int>[1, 2, 3])), isFalse);
      expect(
        isAvif(
          Uint8List.fromList(<int>[
            0x00,
            0x00,
            0x00,
            0x1c,
            0x66,
            0x74,
            0x79,
            0x70,
            0x61,
            0x76,
            0x69,
            0x66,
            0x00,
            0x00,
            0x00,
            0x00,
          ]),
        ),
        isTrue,
      );
      expect(
        isAvif(
          Uint8List.fromList(<int>[
            0x00,
            0x00,
            0x00,
            0x20,
            0x66,
            0x74,
            0x79,
            0x70,
            0x61,
            0x76,
            0x69,
            0x66,
            0x00,
            0x00,
            0x00,
            0x00,
          ]),
        ),
        isTrue,
      );
    });
  }, skip: isSafari);
}

/// Tests specific to WASM codecs bundled with CanvasKit.
void _testCkAnimatedImage() {
  test('ImageDecoder toByteData(PNG)', () async {
    final CkAnimatedImage image = CkAnimatedImage.decodeFromBytes(kAnimatedGif, 'test');
    final ui.FrameInfo frame = await image.getNextFrame();
    final ByteData? png = await frame.image.toByteData(format: ui.ImageByteFormat.png);
    expect(png, isNotNull);

    // The precise PNG encoding is browser-specific, but we can check the file
    // signature.
    expect(detectImageType(png!.buffer.asUint8List()), ImageType.png);
  });

  test('CkAnimatedImage toByteData(RGBA)', () async {
    final CkAnimatedImage image = CkAnimatedImage.decodeFromBytes(kAnimatedGif, 'test');
    const List<List<int>> expectedColors = <List<int>>[
      <int>[255, 0, 0, 255],
      <int>[0, 255, 0, 255],
      <int>[0, 0, 255, 255],
    ];
    for (int i = 0; i < image.frameCount; i++) {
      final ui.FrameInfo frame = await image.getNextFrame();
      final ByteData? rgba = await frame.image.toByteData();
      expect(rgba, isNotNull);
      expect(rgba!.buffer.asUint8List(), expectedColors[i]);
    }
  });
}
