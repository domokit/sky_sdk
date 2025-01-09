// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'matchers.dart';
library;

import 'dart:async';
import 'dart:io';
import 'dart:math' as math;
import 'dart:ui';

import 'package:flutter/foundation.dart';
import 'package:matcher/expect.dart' show fail;
import 'package:path/path.dart' as path;

import 'goldens.dart';
import 'test_async_utils.dart';

/// The default [GoldenFileComparator] implementation for `flutter test`.
///
/// The term __golden file__ refers to a master image that is considered the
/// true rendering of a given widget, state, application, or other visual
/// representation you have chosen to capture. This comparator loads golden
/// files from the local file system, treating the golden key as a relative
/// path from the test file's directory.
///
/// This comparator performs a pixel-for-pixel comparison of the decoded PNGs,
/// returning true only if there's an exact match. In cases where the captured
/// test image does not match the golden file, this comparator will provide
/// output to illustrate the difference, described in further detail below.
///
/// When using `flutter test --update-goldens`, [LocalFileComparator]
/// updates the golden files on disk to match the rendering.
///
/// ## Local Output from Golden File Testing
///
/// The [LocalFileComparator] will output test feedback when a golden file test
/// fails. This output takes the form of differential images contained within a
/// `failures` directory that will be generated in the same location specified
/// by the golden key. The differential images include the master and test
/// images that were compared, as well as an isolated diff of detected pixels,
/// and a masked diff that overlays these detected pixels over the master image.
///
/// The following images are examples of a test failure output:
///
/// |  File Name                 |  Image Output |
/// |----------------------------|---------------|
/// |  testName_masterImage.png  | ![A golden master image](https://flutter.github.io/assets-for-api-docs/assets/flutter-test/goldens/widget_masterImage.png)  |
/// |  testName_testImage.png    | ![Test image](https://flutter.github.io/assets-for-api-docs/assets/flutter-test/goldens/widget_testImage.png)  |
/// |  testName_isolatedDiff.png | ![An isolated pixel difference.](https://flutter.github.io/assets-for-api-docs/assets/flutter-test/goldens/widget_isolatedDiff.png) |
/// |  testName_maskedDiff.png   | ![A masked pixel difference](https://flutter.github.io/assets-for-api-docs/assets/flutter-test/goldens/widget_maskedDiff.png) |
///
/// {@macro flutter.flutter_test.matchesGoldenFile.custom_fonts}
///
/// See also:
///
///   * [GoldenFileComparator], the abstract class that [LocalFileComparator]
///   implements.
///   * [matchesGoldenFile], the function that invokes the comparator.
class LocalFileComparator extends GoldenFileComparator with LocalComparisonOutput {
  /// Creates a new [LocalFileComparator] for the specified [testFile].
  ///
  /// Golden file keys will be interpreted as file paths relative to the
  /// directory in which [testFile] resides.
  ///
  /// The [testFile] URL must represent a file.
  LocalFileComparator(Uri testFile, {path.Style? pathStyle})
    : basedir = _getBasedir(testFile, pathStyle),
      _path = _getPath(pathStyle);

  static path.Context _getPath(path.Style? style) {
    return path.Context(style: style ?? path.Style.platform);
  }

  static Uri _getBasedir(Uri testFile, path.Style? pathStyle) {
    final path.Context context = _getPath(pathStyle);
    final String testFilePath = context.fromUri(testFile);
    final String testDirectoryPath = context.dirname(testFilePath);
    return context.toUri(testDirectoryPath + context.separator);
  }

  /// The directory in which the test was loaded.
  ///
  /// Golden file keys will be interpreted as file paths relative to this
  /// directory.
  final Uri basedir;

  /// Path context exists as an instance variable rather than just using the
  /// system path context in order to support testing, where we can spoof the
  /// platform to test behaviors with arbitrary path styles.
  final path.Context _path;

  @override
  Future<bool> compare(Uint8List imageBytes, Uri golden) async {
    final ComparisonResult result = await GoldenFileComparator.compareLists(
      imageBytes,
      await getGoldenBytes(golden),
    );

    if (result.passed) {
      result.dispose();
      return true;
    }

    final String error = await generateFailureOutput(result, golden, basedir);
    result.dispose();
    throw FlutterError(error);
  }

  @override
  Future<void> update(Uri golden, Uint8List imageBytes) async {
    final File goldenFile = _getGoldenFile(golden);
    await goldenFile.parent.create(recursive: true);
    await goldenFile.writeAsBytes(imageBytes, flush: true);
  }

  /// Returns the bytes of the given [golden] file.
  ///
  /// If the file cannot be found, an error will be thrown.
  @protected
  Future<List<int>> getGoldenBytes(Uri golden) async {
    final File goldenFile = _getGoldenFile(golden);
    if (!goldenFile.existsSync()) {
      fail('Could not be compared against non-existent file: "$golden"');
    }
    final List<int> goldenBytes = await goldenFile.readAsBytes();
    return goldenBytes;
  }

  File _getGoldenFile(Uri golden) =>
      File(_path.join(_path.fromUri(basedir), _path.fromUri(golden.path)));
}

/// A mixin for use in golden file comparators that run locally and provide
/// output.
mixin LocalComparisonOutput {
  /// Writes out diffs from the [ComparisonResult] of a golden file test.
  ///
  /// Will throw an error if a null result is provided.
  Future<String> generateFailureOutput(
    ComparisonResult result,
    Uri golden,
    Uri basedir, {
    String key = '',
  }) async => TestAsyncUtils.guard<String>(() async {
    String additionalFeedback = '';
    if (result.diffs != null) {
      additionalFeedback =
          '\nFailure feedback can be found at ${path.join(basedir.path, 'failures')}';
      final Map<String, Image> diffs = result.diffs!;
      for (final MapEntry<String, Image> entry in diffs.entries) {
        final File output = getFailureFile(
          key.isEmpty ? entry.key : '${entry.key}_$key',
          golden,
          basedir,
        );
        output.parent.createSync(recursive: true);
        final ByteData? pngBytes = await entry.value.toByteData(format: ImageByteFormat.png);
        output.writeAsBytesSync(pngBytes!.buffer.asUint8List());
      }
    }
    return 'Golden "$golden": ${result.error}$additionalFeedback';
  });

  /// Returns the appropriate file for a given diff from a [ComparisonResult].
  File getFailureFile(String failure, Uri golden, Uri basedir) {
    final String fileName = golden.pathSegments.last;
    final String testName = '${fileName.split(path.extension(fileName))[0]}_$failure.png';
    return File(path.join(path.fromUri(basedir), path.fromUri(Uri.parse('failures/$testName'))));
  }
}

/// Returns a [ComparisonResult] to describe the pixel differential of the
/// [test] and [master] image bytes provided.
Future<ComparisonResult> compareLists(List<int>? test, List<int>? master) async {
  if (test == null || master == null || test.isEmpty || master.isEmpty) {
    return ComparisonResult(
      passed: false,
      diffPercent: 1.0,
      error: 'Pixel test failed, null image provided.',
    );
  }

  if (listEquals(test, master)) {
    return ComparisonResult(passed: true, diffPercent: 0.0);
  }

  final Codec testImageCodec = await instantiateImageCodec(Uint8List.fromList(test));
  final Image testImage = (await testImageCodec.getNextFrame()).image;
  final ByteData? testImageRgba = await testImage.toByteData();

  final Codec masterImageCodec = await instantiateImageCodec(Uint8List.fromList(master));
  final Image masterImage = (await masterImageCodec.getNextFrame()).image;
  final ByteData? masterImageRgba = await masterImage.toByteData();

  final int width = testImage.width;
  final int height = testImage.height;

  if (width != masterImage.width || height != masterImage.height) {
    final ComparisonResult result = ComparisonResult(
      passed: false,
      diffPercent: 1.0,
      error:
          'Pixel test failed, image sizes do not match.\n'
          'Master Image: ${masterImage.width} X ${masterImage.height}\n'
          'Test Image: ${testImage.width} X ${testImage.height}',
      diffs: <String, Image>{'masterImage': masterImage, 'testImage': testImage},
    );
    return result;
  }

  int pixelDiffCount = 0;
  final int totalPixels = width * height;
  final ByteData invertedMasterRgba = _invert(masterImageRgba!);
  final ByteData invertedTestRgba = _invert(testImageRgba!);

  final Uint8List testImageBytes = (await testImage.toByteData())!.buffer.asUint8List();
  final ByteData maskedDiffRgba = ByteData(testImageBytes.length);
  maskedDiffRgba.buffer.asUint8List().setRange(0, testImageBytes.length, testImageBytes);
  final ByteData isolatedDiffRgba = ByteData(width * height * 4);

  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      final int byteOffset = (width * y + x) * 4;
      final int testPixel = testImageRgba.getUint32(byteOffset);
      final int masterPixel = masterImageRgba.getUint32(byteOffset);

      final int diffPixel =
          (_readRed(testPixel) - _readRed(masterPixel)).abs() +
          (_readGreen(testPixel) - _readGreen(masterPixel)).abs() +
          (_readBlue(testPixel) - _readBlue(masterPixel)).abs() +
          (_readAlpha(testPixel) - _readAlpha(masterPixel)).abs();

      if (diffPixel != 0) {
        final int invertedMasterPixel = invertedMasterRgba.getUint32(byteOffset);
        final int invertedTestPixel = invertedTestRgba.getUint32(byteOffset);
        // We grab the max of the 0xAABBGGRR encoded bytes, and then convert
        // back to 0xRRGGBBAA for the actual pixel value, since this is how it
        // was historically done.
        final int maskPixel = _toRGBA(
          math.max(_toABGR(invertedMasterPixel), _toABGR(invertedTestPixel)),
        );
        maskedDiffRgba.setUint32(byteOffset, maskPixel);
        isolatedDiffRgba.setUint32(byteOffset, maskPixel);
        pixelDiffCount++;
      }
    }
  }

  if (pixelDiffCount > 0) {
    final double diffPercent = pixelDiffCount / totalPixels;
    return ComparisonResult(
      passed: false,
      diffPercent: diffPercent,
      error:
          'Pixel test failed, '
          '${(diffPercent * 100).toStringAsFixed(2)}%, ${pixelDiffCount}px '
          'diff detected.',
      diffs: <String, Image>{
        'masterImage': masterImage,
        'testImage': testImage,
        'maskedDiff': await _createImage(maskedDiffRgba, width, height),
        'isolatedDiff': await _createImage(isolatedDiffRgba, width, height),
      },
    );
  }
  masterImage.dispose();
  testImage.dispose();
  return ComparisonResult(passed: true, diffPercent: 0.0);
}

/// Inverts [imageBytes], returning a new [ByteData] object.
ByteData _invert(ByteData imageBytes) {
  final ByteData bytes = ByteData(imageBytes.lengthInBytes);
  // Invert the RGB data (but not A).
  for (int i = 0; i < imageBytes.lengthInBytes; i += 4) {
    bytes.setUint8(i, 255 - imageBytes.getUint8(i));
    bytes.setUint8(i + 1, 255 - imageBytes.getUint8(i + 1));
    bytes.setUint8(i + 2, 255 - imageBytes.getUint8(i + 2));
    bytes.setUint8(i + 3, imageBytes.getUint8(i + 3));
  }
  return bytes;
}

/// An unsupported [WebGoldenComparator] that exists for API compatibility.
class DefaultWebGoldenComparator extends WebGoldenComparator {
  /// This is provided to prevent warnings from the analyzer.
  DefaultWebGoldenComparator(Uri _) {
    throw UnsupportedError('DefaultWebGoldenComparator is only supported on the web.');
  }

  @override
  Future<bool> compare(double width, double height, Uri golden) {
    throw UnsupportedError('DefaultWebGoldenComparator is only supported on the web.');
  }

  @override
  Future<void> update(double width, double height, Uri golden) {
    throw UnsupportedError('DefaultWebGoldenComparator is only supported on the web.');
  }

  @override
  Future<bool> compareBytes(Uint8List bytes, Uri golden) {
    throw UnsupportedError('DefaultWebGoldenComparator is only supported on the web.');
  }

  @override
  Future<void> updateBytes(Uint8List bytes, Uri golden) {
    throw UnsupportedError('DefaultWebGoldenComparator is only supported on the web.');
  }
}

/// Reads the red value out of a 32 bit rgba pixel.
int _readRed(int pixel) => (pixel >> 24) & 0xff;

/// Reads the green value out of a 32 bit rgba pixel.
int _readGreen(int pixel) => (pixel >> 16) & 0xff;

/// Reads the blue value out of a 32 bit rgba pixel.
int _readBlue(int pixel) => (pixel >> 8) & 0xff;

/// Reads the alpha value out of a 32 bit rgba pixel.
int _readAlpha(int pixel) => pixel & 0xff;

/// Convenience wrapper around [decodeImageFromPixels].
Future<Image> _createImage(ByteData bytes, int width, int height) {
  final Completer<Image> completer = Completer<Image>();
  decodeImageFromPixels(
    bytes.buffer.asUint8List(),
    width,
    height,
    PixelFormat.rgba8888,
    completer.complete,
  );
  return completer.future;
}

// Converts a 32 bit rgba pixel to a 32 bit abgr pixel
int _toABGR(int rgba) =>
    (_readAlpha(rgba) << 24) | (_readBlue(rgba) << 16) | (_readGreen(rgba) << 8) | _readRed(rgba);

// Converts a 32 bit abgr pixel to a 32 bit rgba pixel
int _toRGBA(int abgr) =>
// This is just a mirror of the other conversion.
_toABGR(abgr);
