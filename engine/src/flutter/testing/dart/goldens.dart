// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';
import 'dart:typed_data';
import 'dart:ui';

import 'package:path/path.dart' as path;
import 'package:skia_gold_client/skia_gold_client.dart';

import 'impeller_enabled.dart';

const String _kSkiaGoldWorkDirectoryKey = 'kSkiaGoldWorkDirectory';

/// A helper for doing image comparison (golden) tests.
///
/// Contains utilities for comparing two images in memory that are expected to
/// be identical, or for adding images to Skia gold for comparison.
class ImageComparer {
  ImageComparer._({required SkiaGoldClient client}) : _client = client;

  // Avoid talking to Skia gold for the force-multithreading variants.
  static bool get _useSkiaGold => !Platform.executableArguments.contains('--force-multithreading');

  /// Creates an image comparer and authorizes.
  static Future<ImageComparer> create({bool verbose = false}) async {
    const String workDirectoryPath = String.fromEnvironment(_kSkiaGoldWorkDirectoryKey);
    if (workDirectoryPath.isEmpty) {
      throw UnsupportedError('Using ImageComparer requries defining kSkiaGoldWorkDirectoryKey.');
    }

    final Directory workDirectory = Directory(
      impellerEnabled ? '${workDirectoryPath}_iplr' : workDirectoryPath,
    )..createSync();
    final Map<String, String> dimensions = <String, String>{
      'impeller_enabled': impellerEnabled.toString(),
    };
    final SkiaGoldClient client =
        SkiaGoldClient.isAvailable() && _useSkiaGold
            ? SkiaGoldClient(workDirectory, dimensions: dimensions, verbose: verbose)
            : _FakeSkiaGoldClient(workDirectory, dimensions, verbose: verbose);

    await client.auth();
    return ImageComparer._(client: client);
  }

  final SkiaGoldClient _client;

  /// Adds an [Image] to Skia Gold for comparison.
  ///
  /// The [fileName] must be unique.
  Future<void> addGoldenImage(Image image, String fileName) async {
    final ByteData data = (await image.toByteData(format: ImageByteFormat.png))!;

    final File file = File(path.join(_client.workDirectory.path, fileName))
      ..writeAsBytesSync(data.buffer.asUint8List());
    await _client.addImg(fileName, file, screenshotSize: image.width * image.height).catchError((
      dynamic error,
    ) {
      print('Skia gold comparison failed: $error');
      throw Exception('Failed comparison: $fileName');
    });
  }

  Future<bool> fuzzyCompareImages(Image golden, Image testImage) async {
    if (golden.width != testImage.width || golden.height != testImage.height) {
      return false;
    }
    int getPixel(ByteData data, int x, int y) => data.getUint32((x + y * golden.width) * 4);
    final ByteData goldenData = (await golden.toByteData())!;
    final ByteData testImageData = (await testImage.toByteData())!;
    for (int y = 0; y < golden.height; y++) {
      for (int x = 0; x < golden.width; x++) {
        if (getPixel(goldenData, x, y) != getPixel(testImageData, x, y)) {
          return false;
        }
      }
    }
    return true;
  }
}

// TODO(dnfield): add local comparison against baseline,
// https://github.com/flutter/flutter/issues/136831
class _FakeSkiaGoldClient implements SkiaGoldClient {
  _FakeSkiaGoldClient(this.workDirectory, this.dimensions, {this.verbose = false});

  @override
  final Directory workDirectory;

  @override
  final Map<String, String> dimensions;

  @override
  final bool verbose;

  @override
  Future<void> auth() async {}

  @override
  Future<void> addImg(
    String testName,
    File goldenFile, {
    double differentPixelsRate = 0.01,
    int pixelColorDelta = 0,
    required int screenshotSize,
  }) async {}

  @override
  dynamic noSuchMethod(Invocation invocation) {
    throw UnimplementedError(invocation.memberName.toString().split('"')[1]);
  }
}
