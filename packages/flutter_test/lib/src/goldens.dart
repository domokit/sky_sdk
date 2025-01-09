// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
///
/// @docImport '_goldens_io.dart';
/// @docImport 'binding.dart';
/// @docImport 'matchers.dart';
/// @docImport 'widget_tester.dart';
library;

import 'dart:typed_data';
import 'dart:ui';

import 'package:meta/meta.dart';
import 'package:path/path.dart' as path;

import '_goldens_io.dart' if (dart.library.js_interop) '_goldens_web.dart' as goldens;

/// Compares image pixels against a golden image file.
///
/// Instances of this comparator will be used as the backend for
/// [matchesGoldenFile].
///
/// Instances of this comparator will be invoked by the test framework in the
/// [TestWidgetsFlutterBinding.runAsync] zone and are thus not subject to the
/// fake async constraints that are normally imposed on widget tests (i.e. the
/// need or the ability to call [WidgetTester.pump] to advance the microtask
/// queue).
///
/// ## What is Golden File Testing?
///
/// The term __golden file__ refers to a master image that is considered the true
/// rendering of a given widget, state, application, or other visual
/// representation you have chosen to capture.
///
/// By keeping a master reference of visual aspects of your application, you can
/// prevent unintended changes as you develop by testing against them.
///
/// Here, a minor code change has altered the appearance of a widget. A golden
/// file test has compared the image generated at the time of the test to the
/// golden master file that was generated earlier. The test has identified the
/// change, preventing unintended modifications.
///
/// |  Sample                        |  Image |
/// |--------------------------------|--------|
/// |  Golden Master Image           | ![A golden master image](https://flutter.github.io/assets-for-api-docs/assets/flutter-test/goldens/widget_masterImage.png)  |
/// |  Difference                    | ![The pixel difference](https://flutter.github.io/assets-for-api-docs/assets/flutter-test/goldens/widget_isolatedDiff.png)  |
/// |  Test image after modification | ![Test image](https://flutter.github.io/assets-for-api-docs/assets/flutter-test/goldens/widget_testImage.png) |
///
/// {@macro flutter.flutter_test.matchesGoldenFile.custom_fonts}
///
/// See also:
///
///  * [LocalFileComparator] for the default [GoldenFileComparator]
///    implementation for `flutter test`.
///  * [matchesGoldenFile], the function that invokes the comparator.
abstract class GoldenFileComparator {
  /// Compares the pixels of decoded png [imageBytes] against the golden file
  /// identified by [golden].
  ///
  /// The returned future completes with a boolean value that indicates whether
  /// the pixels decoded from [imageBytes] match the golden file's pixels.
  ///
  /// In the case of comparison mismatch, the comparator may choose to throw a
  /// [TestFailure] if it wants to control the failure message, often in the
  /// form of a [ComparisonResult] that provides detailed information about the
  /// mismatch.
  ///
  /// The method by which [golden] is located and by which its bytes are loaded
  /// is left up to the implementation class. For instance, some implementations
  /// may load files from the local file system, whereas others may load files
  /// over the network or from a remote repository.
  Future<bool> compare(Uint8List imageBytes, Uri golden);

  /// Updates the golden file identified by [golden] with [imageBytes].
  ///
  /// This will be invoked in lieu of [compare] when [autoUpdateGoldenFiles]
  /// is `true` (which gets set automatically by the test framework when the
  /// user runs `flutter test --update-goldens`).
  ///
  /// The method by which [golden] is located and by which its bytes are written
  /// is left up to the implementation class.
  Future<void> update(Uri golden, Uint8List imageBytes);

  /// Returns a new golden file [Uri] to incorporate any [version] number with
  /// the [key].
  ///
  /// The [version] is an optional int that can be used to differentiate
  /// historical golden files.
  ///
  /// Version numbers are used in golden file tests for package:flutter. You can
  /// learn more about these tests [here](https://github.com/flutter/flutter/blob/main/docs/contributing/testing/Writing-a-golden-file-test-for-package-flutter.md).
  Uri getTestUri(Uri key, int? version) {
    if (version == null) {
      return key;
    }
    final String keyString = key.toString();
    final String extension = path.extension(keyString);
    return Uri.parse('${keyString.split(extension).join()}.$version$extension');
  }

  /// Returns a [ComparisonResult] to describe the pixel differential of the
  /// [test] and [master] image bytes provided.
  static Future<ComparisonResult> compareLists(List<int> test, List<int> master) {
    return goldens.compareLists(test, master);
  }
}

/// Compares pixels against those of a golden image file.
///
/// This comparator is used as the backend for [matchesGoldenFile].
///
/// When using `flutter test`, a comparator implemented by [LocalFileComparator]
/// is used if no other comparator is specified. It treats the golden key as
/// a relative path from the test file's directory. It will then load the
/// golden file's bytes from disk and perform a pixel-for-pixel comparison of
/// the decoded PNGs, returning true only if there's an exact match.
///
/// When using `flutter test --update-goldens`, the [LocalFileComparator]
/// updates the files on disk to match the rendering.
///
/// When using `flutter run`, the default comparator ([TrivialComparator])
/// is used. It prints a message to the console but otherwise does nothing. This
/// allows tests to be developed visually on a real device.
///
/// Callers may choose to override the default comparator by setting this to a
/// custom comparator during test set-up (or using directory-level test
/// configuration).
///
/// {@tool snippet}
/// For example, some projects may wish to install a comparator with tolerance
/// levels for allowable differences:
///
/// ```dart
/// void main() {
///   testWidgets('matches golden file with a 0.01 tolerance', (WidgetTester tester) async {
///     final GoldenFileComparator previousGoldenFileComparator = goldenFileComparator;
///     goldenFileComparator = _TolerantGoldenFileComparator(
///       Uri.parse('test/my_widget_test.dart'),
///       precisionTolerance: 0.01,
///     );
///     addTearDown(() => goldenFileComparator = previousGoldenFileComparator);
///
///     await tester.pumpWidget(const ColoredBox(color: Color(0xff00ff00)));
///
///     await expectLater(
///       find.byType(ColoredBox),
///       matchesGoldenFile('my_golden.png'),
///     );
///   });
/// }
///
/// class _TolerantGoldenFileComparator extends LocalFileComparator {
///   _TolerantGoldenFileComparator(
///     super.testFile, {
///     required double precisionTolerance,
///   })  : assert(
///         0 <= precisionTolerance && precisionTolerance <= 1,
///         'precisionTolerance must be between 0 and 1',
///         ),
///         _precisionTolerance = precisionTolerance;
///
///   /// How much the golden image can differ from the test image.
///   ///
///   /// It is expected to be between 0 and 1. Where 0 is no difference (the same image)
///   /// and 1 is the maximum difference (completely different images).
///   final double _precisionTolerance;
///
///   @override
///   Future<bool> compare(Uint8List imageBytes, Uri golden) async {
///     final ComparisonResult result = await GoldenFileComparator.compareLists(
///       imageBytes,
///       await getGoldenBytes(golden),
///     );
///
///     final bool passed = result.passed || result.diffPercent <= _precisionTolerance;
///     if (passed) {
///       result.dispose();
///       return true;
///     }
///
///     final String error = await generateFailureOutput(result, golden, basedir);
///     result.dispose();
///     throw FlutterError(error);
///   }
/// }
/// ```
/// {@end-tool}
///
/// See also:
///
///  * [flutter_test] for more information about how to configure tests at the
///    directory-level.
GoldenFileComparator goldenFileComparator = const TrivialComparator._();

/// Compares image pixels against a golden image file.
///
/// Instances of this comparator will be used as the backend for
/// [matchesGoldenFile] when tests are running on Flutter Web, and will usually
/// implemented by deferring the screenshot taking and image comparison to a
/// test server.
///
/// Instances of this comparator will be invoked by the test framework in the
/// [TestWidgetsFlutterBinding.runAsync] zone and are thus not subject to the
/// fake async constraints that are normally imposed on widget tests (i.e. the
/// need or the ability to call [WidgetTester.pump] to advance the microtask
/// queue). Prior to the invocation, the test framework will render only the
/// [Element] to be compared on the screen.
///
/// See also:
///
///  * [GoldenFileComparator] for the comparator to be used when the test is
///    not running in a web browser.
///  * [DefaultWebGoldenComparator] for the default [WebGoldenComparator]
///    implementation for `flutter test`.
///  * [matchesGoldenFile], the function that invokes the comparator.
@Deprecated(
  'Use GoldenFileComparator instead. '
  'This feature was deprecated after v3.28.0-0.1.pre.',
)
abstract class WebGoldenComparator {
  /// Compares the rendered pixels of size [width]x[height] that is being
  /// rendered on the top left of the screen against the golden file identified
  /// by [golden].
  ///
  /// The returned future completes with a boolean value that indicates whether
  /// the pixels rendered on screen match the golden file's pixels.
  ///
  /// In the case of comparison mismatch, the comparator may choose to throw a
  /// [TestFailure] if it wants to control the failure message, often in the
  /// form of a [ComparisonResult] that provides detailed information about the
  /// mismatch.
  ///
  /// The method by which [golden] is located and by which its bytes are loaded
  /// is left up to the implementation class. For instance, some implementations
  /// may load files from the local file system, whereas others may load files
  /// over the network or from a remote repository.
  Future<bool> compare(double width, double height, Uri golden);

  /// Updates the golden file identified by [golden] with rendered pixels of
  /// [width]x[height].
  ///
  /// This will be invoked in lieu of [compare] when [autoUpdateGoldenFiles]
  /// is `true` (which gets set automatically by the test framework when the
  /// user runs `flutter test --update-goldens --platform=chrome`).
  ///
  /// The method by which [golden] is located and by which its bytes are written
  /// is left up to the implementation class.
  Future<void> update(double width, double height, Uri golden);

  /// Compares the pixels of decoded png [bytes] against the golden file
  /// identified by [golden].
  ///
  /// The returned future completes with a boolean value that indicates whether
  /// the pixels rendered on screen match the golden file's pixels.
  ///
  /// In the case of comparison mismatch, the comparator may choose to throw a
  /// [TestFailure] if it wants to control the failure message, often in the
  /// form of a [ComparisonResult] that provides detailed information about the
  /// mismatch.
  ///
  /// The method by which [golden] is located and by which its bytes are loaded
  /// is left up to the implementation class. For instance, some implementations
  /// may load files from the local file system, whereas others may load files
  /// over the network or from a remote repository.
  Future<bool> compareBytes(Uint8List bytes, Uri golden);

  /// Compares the pixels of decoded png [bytes] against the golden file
  /// identified by [golden].
  ///
  /// This will be invoked in lieu of [compareBytes] when [autoUpdateGoldenFiles]
  /// is `true` (which gets set automatically by the test framework when the
  /// user runs `flutter test --update-goldens --platform=chrome`).
  ///
  /// The method by which [golden] is located and by which its bytes are written
  /// is left up to the implementation class.
  Future<void> updateBytes(Uint8List bytes, Uri golden);

  /// Returns a new golden file [Uri] to incorporate any [version] number with
  /// the [key].
  ///
  /// The [version] is an optional int that can be used to differentiate
  /// historical golden files.
  ///
  /// Version numbers are used in golden file tests for package:flutter. You can
  /// learn more about these tests [here](https://github.com/flutter/flutter/blob/main/docs/contributing/testing/Writing-a-golden-file-test-for-package-flutter.md).
  Uri getTestUri(Uri key, int? version) {
    if (version == null) {
      return key;
    }
    final String keyString = key.toString();
    final String extension = path.extension(keyString);
    return Uri.parse('${keyString.split(extension).join()}.$version$extension');
  }
}

/// Compares pixels against those of a golden image file.
///
/// This comparator is used as the backend for [matchesGoldenFile] when tests
/// are running in a web browser.
///
/// When using `flutter test --platform=chrome`, a comparator implemented by
/// [DefaultWebGoldenComparator] is used if no other comparator is specified. It
/// will send a request to the test server, which uses [goldenFileComparator]
/// for golden file comparison.
///
/// When using `flutter test --update-goldens`, the [DefaultWebGoldenComparator]
/// updates the files on disk to match the rendering.
///
/// When using `flutter run`, the default comparator
/// (`_TrivialWebGoldenComparator`) is used. It prints a message to the console
/// but otherwise does nothing. This allows tests to be developed visually on a
/// web browser.
///
/// Callers may choose to override the default comparator by setting this to a
/// custom comparator during test set-up (or using directory-level test
/// configuration). For example, some projects may wish to install a comparator
/// with tolerance levels for allowable differences.
///
/// See also:
///
///  * [flutter_test] for more information about how to configure tests at the
///    directory-level.
///  * [goldenFileComparator], the comparator used when tests are not running on
///    a web browser.
WebGoldenComparator get webGoldenComparator => _webGoldenComparator;
WebGoldenComparator _webGoldenComparator = const _TrivialWebGoldenComparator._();
set webGoldenComparator(WebGoldenComparator value) {
  _webGoldenComparator = value;
}

/// Whether golden files should be automatically updated during tests rather
/// than compared to the image bytes recorded by the tests.
///
/// When this is `true`, [matchesGoldenFile] will always report a successful
/// match, because the bytes being tested implicitly become the new golden.
///
/// The Flutter tool will automatically set this to `true` when the user runs
/// `flutter test --update-goldens`, so callers should generally never have to
/// explicitly modify this value.
///
/// See also:
///
///   * [goldenFileComparator]
bool autoUpdateGoldenFiles = false;

/// Placeholder comparator that is set as the value of [goldenFileComparator]
/// when the initialization that happens in the test bootstrap either has not
/// yet happened or has been bypassed.
///
/// The test bootstrap file that gets generated by the Flutter tool when the
/// user runs `flutter test` is expected to set [goldenFileComparator] to
/// a comparator that resolves golden file references relative to the test
/// directory. From there, the caller may choose to override the comparator by
/// setting it to another value during test initialization. The only case
/// where we expect it to remain uninitialized is when the user runs a test
/// via `flutter run`. In this case, the [compare] method will just print a
/// message that it would have otherwise run a real comparison, and it will
/// return trivial success.
///
/// This class can't be constructed. It represents the default value of
/// [goldenFileComparator].
class TrivialComparator implements GoldenFileComparator {
  const TrivialComparator._();

  @override
  Future<bool> compare(Uint8List imageBytes, Uri golden) {
    // Ideally we would use markTestSkipped here but in some situations,
    // comparators are called outside of tests.
    // See also: https://github.com/flutter/flutter/issues/91285
    // ignore: avoid_print
    print('Golden file comparison requested for "$golden"; skipping...');
    return Future<bool>.value(true);
  }

  @override
  Future<void> update(Uri golden, Uint8List imageBytes) {
    throw StateError('goldenFileComparator has not been initialized');
  }

  @override
  Uri getTestUri(Uri key, int? version) {
    return key;
  }
}

class _TrivialWebGoldenComparator implements WebGoldenComparator {
  const _TrivialWebGoldenComparator._();

  @override
  Future<bool> compare(double width, double height, Uri golden) {
    return _warnAboutSkipping(golden);
  }

  @override
  Future<void> update(double width, double height, Uri golden) {
    throw StateError('webGoldenComparator has not been initialized');
  }

  @override
  Uri getTestUri(Uri key, int? version) {
    return key;
  }

  @override
  Future<bool> compareBytes(Uint8List bytes, Uri golden) {
    return _warnAboutSkipping(golden);
  }

  @override
  Future<void> updateBytes(Uint8List bytes, Uri golden) {
    throw StateError('webGoldenComparator has not been initialized');
  }

  Future<bool> _warnAboutSkipping(Uri golden) {
    // Ideally we would use markTestSkipped here but in some situations,
    // comparators are called outside of tests.
    // See also: https://github.com/flutter/flutter/issues/91285
    // ignore: avoid_print
    print('Golden comparison requested for "$golden"; skipping...');
    return Future<bool>.value(true);
  }
}

/// The result of a pixel comparison test.
///
/// The [ComparisonResult] will always indicate if a test has [passed]. The
/// optional [error] and [diffs] parameters provide further information about
/// the result of a failing test.
class ComparisonResult {
  /// Creates a new [ComparisonResult] for the current test.
  ComparisonResult({required this.passed, required this.diffPercent, this.error, this.diffs});

  /// Indicates whether or not a pixel comparison test has failed.
  final bool passed;

  /// Error message used to describe the cause of the pixel comparison failure.
  final String? error;

  /// Map containing differential images to illustrate found variants in pixel
  /// values in the execution of the pixel test.
  final Map<String, Image>? diffs;

  /// The calculated percentage of pixel difference between two images.
  final double diffPercent;

  /// Disposes the images held by this [ComparisonResult].
  @mustCallSuper
  void dispose() {
    if (diffs == null) {
      return;
    }

    for (final MapEntry<String, Image> entry in diffs!.entries) {
      entry.value.dispose();
    }
  }
}
