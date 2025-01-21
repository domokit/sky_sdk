// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';

import '../common/test_initialization.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

List<String>? testFiles;

Future<void> testMain() async {
  setUpImplicitView();

  Future<List<String>> createTestFiles() async {
    final HttpFetchResponse listingResponse = await httpFetch('/test_images/');
    List<String> testFiles = (await listingResponse.json() as List<dynamic>).cast<String>();
    testFiles = testFiles.map((String baseName) => '/test_images/$baseName').toList();

    // Sanity-check the test file list. If suddenly test files are moved or
    // deleted, and the test server returns an empty list, or is missing some
    // important test files, we want to know.
    assert(testFiles.isNotEmpty);
    assert(testFiles.any((String testFile) => testFile.endsWith('.jpg')));
    assert(testFiles.any((String testFile) => testFile.endsWith('.png')));
    assert(testFiles.any((String testFile) => testFile.endsWith('.gif')));
    assert(testFiles.any((String testFile) => testFile.endsWith('.webp')));
    assert(testFiles.any((String testFile) => testFile.endsWith('.bmp')));

    return testFiles;
  }

  testFiles = await createTestFiles();

  for (final String testFile in testFiles!) {
    test('can detect image type of $testFile', () async {
      final HttpFetchResponse response = await httpFetch(testFile);

      if (!response.hasPayload) {
        throw Exception('Unable to fetch() image test file "$testFile"');
      }

      final Uint8List responseBytes = await response.asUint8List();

      // WebP files which are known to be animated.
      const List<String> animatedWebpFiles = <String>[
        '/test_images/blendBG.webp',
        '/test_images/required.webp',
        '/test_images/stoplight_h.webp',
        '/test_images/stoplight.webp',
      ];

      // GIF files which are known to be animated.
      const List<String> animatedGifFiles = <String>[
        '/test_images/alphabetAnim.gif',
        '/test_images/colorTables.gif',
        '/test_images/flightAnim.gif',
        '/test_images/gif-transparent-index.gif',
        '/test_images/randPixelsAnim.gif',
        '/test_images/randPixelsAnim2.gif',
        '/test_images/required.gif',
        '/test_images/test640x479.gif',
        '/test_images/xOffsetTooBig.gif',
      ];

      final String testFileExtension = testFile.substring(testFile.lastIndexOf('.') + 1);
      final ImageType? expectedImageType = switch (testFileExtension) {
        'jpg' => ImageType.jpeg,
        'jpeg' => ImageType.jpeg,
        'gif' => animatedGifFiles.contains(testFile) ? ImageType.animatedGif : ImageType.gif,
        'webp' => animatedWebpFiles.contains(testFile) ? ImageType.animatedWebp : ImageType.webp,
        'avif' => ImageType.avif,
        'bmp' => ImageType.bmp,
        'png' => ImageType.png,
        _ => null,
      };

      expect(detectImageType(responseBytes), expectedImageType);
    });
  }

  test('can decode GIF with many nonstandard Special Purpose Blocks', () async {
    expect(detectImageType(_createTestGif()), ImageType.animatedGif);
  });
}

/// Generates a blank GIF to be used in tests.
Uint8List _createTestGif({
  int width = 1,
  int height = 1,
  int numFrames = 2,
  bool includeManyCommentBlocks = true,
}) {
  final List<int> bytes = <int>[];
  // Generate header.
  bytes.addAll('GIF'.codeUnits);
  bytes.addAll('89a'.codeUnits);

  // Generate logical screen.
  List<int> padInt(int x) {
    assert(x >= 0 && x.bitLength <= 16);
    if (x.bitLength > 8) {
      return <int>[x >> 8, x & 0xff];
    }
    return <int>[0, x];
  }

  bytes.addAll(padInt(width));
  bytes.addAll(padInt(height));
  // Indicate there is no Global Color Table.
  bytes.add(0x70);
  bytes.add(0);
  bytes.add(0);

  // Generate data.
  List<int> generateCommentBlock() {
    final List<int> comment = <int>[];
    comment.add(0x21);
    comment.add(0xfe);
    const String commentString = 'This is a comment';
    comment.add(commentString.codeUnits.length);
    comment.addAll(commentString.codeUnits);
    comment.add(0);
    return comment;
  }

  for (int i = 0; i < numFrames; i++) {
    if (includeManyCommentBlocks) {
      bytes.addAll(generateCommentBlock());
    }
    // Add a Graphic Control Extension block.
    bytes.add(0x21);
    bytes.add(0xf9);
    bytes.add(4);
    bytes.add(0);
    // Indicate a delay of 1/10 of a second between frames.
    bytes.add(0);
    bytes.add(10);
    bytes.add(0);
    bytes.add(0);

    if (includeManyCommentBlocks) {
      bytes.addAll(generateCommentBlock());
    }

    // Add a Table-Based Image.
    bytes.add(0x2c);
    bytes.add(0);
    bytes.add(0);
    bytes.add(0);
    bytes.add(0);
    bytes.addAll(padInt(width));
    bytes.addAll(padInt(height));
    bytes.add(0);

    bytes.add(0);
    const String fakeImageData = 'This is an image';
    bytes.add(fakeImageData.codeUnits.length);
    bytes.addAll(fakeImageData.codeUnits);
    bytes.add(0);
  }

  if (includeManyCommentBlocks) {
    bytes.addAll(generateCommentBlock());
  }

  // Generate trailer.
  bytes.add(0x3b);

  return Uint8List.fromList(bytes);
}
