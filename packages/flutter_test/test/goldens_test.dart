// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io' as io;
import 'dart:typed_data';
import 'dart:ui' as ui;

import 'package:file/memory.dart';
import 'package:flutter/foundation.dart'
    show DiagnosticLevel, DiagnosticPropertiesBuilder, DiagnosticsNode, FlutterError;
import 'package:flutter_test/flutter_test.dart' as test_package;
import 'package:flutter_test/flutter_test.dart' hide test;

// 1x1 transparent pixel
const List<int> _kExpectedPngBytes = <int>[
  137,
  80,
  78,
  71,
  13,
  10,
  26,
  10,
  0,
  0,
  0,
  13,
  73,
  72,
  68,
  82,
  0,
  0,
  0,
  1,
  0,
  0,
  0,
  1,
  8,
  6,
  0,
  0,
  0,
  31,
  21,
  196,
  137,
  0,
  0,
  0,
  11,
  73,
  68,
  65,
  84,
  120,
  1,
  99,
  97,
  0,
  2,
  0,
  0,
  25,
  0,
  5,
  144,
  240,
  54,
  245,
  0,
  0,
  0,
  0,
  73,
  69,
  78,
  68,
  174,
  66,
  96,
  130,
];

// 1x1 colored pixel
const List<int> _kColorFailurePngBytes = <int>[
  137,
  80,
  78,
  71,
  13,
  10,
  26,
  10,
  0,
  0,
  0,
  13,
  73,
  72,
  68,
  82,
  0,
  0,
  0,
  1,
  0,
  0,
  0,
  1,
  8,
  6,
  0,
  0,
  0,
  31,
  21,
  196,
  137,
  0,
  0,
  0,
  13,
  73,
  68,
  65,
  84,
  120,
  1,
  99,
  249,
  207,
  240,
  255,
  63,
  0,
  7,
  18,
  3,
  2,
  164,
  147,
  160,
  197,
  0,
  0,
  0,
  0,
  73,
  69,
  78,
  68,
  174,
  66,
  96,
  130,
];

// 1x2 transparent pixel
const List<int> _kSizeFailurePngBytes = <int>[
  137,
  80,
  78,
  71,
  13,
  10,
  26,
  10,
  0,
  0,
  0,
  13,
  73,
  72,
  68,
  82,
  0,
  0,
  0,
  1,
  0,
  0,
  0,
  2,
  8,
  6,
  0,
  0,
  0,
  153,
  129,
  182,
  39,
  0,
  0,
  0,
  14,
  73,
  68,
  65,
  84,
  120,
  1,
  99,
  97,
  0,
  2,
  22,
  16,
  1,
  0,
  0,
  70,
  0,
  9,
  112,
  117,
  150,
  160,
  0,
  0,
  0,
  0,
  73,
  69,
  78,
  68,
  174,
  66,
  96,
  130,
];

void main() {
  late MemoryFileSystem fs;

  setUp(() {
    final FileSystemStyle style =
        io.Platform.isWindows ? FileSystemStyle.windows : FileSystemStyle.posix;
    fs = MemoryFileSystem(style: style);
  });

  /// Converts posix-style paths to the style associated with [fs].
  ///
  /// This allows us to deal in posix-style paths in the tests.
  String fix(String path) {
    if (path.startsWith('/')) {
      path = '${fs.style.drive}$path';
    }
    return path.replaceAll('/', fs.path.separator);
  }

  void test(String description, FutureOr<void> Function() body) {
    test_package.test(description, () async {
      await io.IOOverrides.runZoned<FutureOr<void>>(
        body,
        createDirectory: (String path) => fs.directory(path),
        createFile: (String path) => fs.file(path),
        createLink: (String path) => fs.link(path),
        getCurrentDirectory: () => fs.currentDirectory,
        setCurrentDirectory: (String path) => fs.currentDirectory = path,
        getSystemTempDirectory: () => fs.systemTempDirectory,
        stat: (String path) => fs.stat(path),
        statSync: (String path) => fs.statSync(path),
        fseIdentical: (String p1, String p2) => fs.identical(p1, p2),
        fseIdenticalSync: (String p1, String p2) => fs.identicalSync(p1, p2),
        fseGetType: (String path, bool followLinks) => fs.type(path, followLinks: followLinks),
        fseGetTypeSync:
            (String path, bool followLinks) => fs.typeSync(path, followLinks: followLinks),
        fsWatch: (String a, int b, bool c) => throw UnsupportedError('unsupported'),
        fsWatchIsSupported: () => fs.isWatchSupported,
      );
    });
  }

  group('goldenFileComparator', () {
    test('is initialized by test framework', () {
      expect(goldenFileComparator, isNotNull);
      expect(goldenFileComparator, isA<LocalFileComparator>());
      final LocalFileComparator comparator = goldenFileComparator as LocalFileComparator;
      expect(comparator.basedir.path, contains('flutter_test'));
    });

    test('image comparison should not loop over all pixels when the data is the same', () async {
      final List<int> invalidImageData1 = Uint8List.fromList(<int>[127]);
      final List<int> invalidImageData2 = Uint8List.fromList(<int>[127]);
      // This will fail if the comparison algorithm tries to generate the images
      // to loop over every pixel which is not necessary when test and master
      // is exactly the same (for performance reasons).
      await GoldenFileComparator.compareLists(invalidImageData1, invalidImageData2);
    });
  });

  group('LocalFileComparator', () {
    late LocalFileComparator comparator;

    setUp(() {
      comparator = LocalFileComparator(
        fs.file(fix('/golden_test.dart')).uri,
        pathStyle: fs.path.style,
      );
    });

    test('calculates basedir correctly', () {
      expect(comparator.basedir, fs.file(fix('/')).uri);
      comparator = LocalFileComparator(
        fs.file(fix('/foo/bar/golden_test.dart')).uri,
        pathStyle: fs.path.style,
      );
      expect(comparator.basedir, fs.directory(fix('/foo/bar/')).uri);
    });

    test('can be instantiated with uri that represents file in same folder', () {
      comparator = LocalFileComparator(Uri.parse('foo_test.dart'), pathStyle: fs.path.style);
      expect(comparator.basedir, Uri.parse('./'));
    });

    test('throws if local output is not awaited', () {
      try {
        comparator.generateFailureOutput(
          ComparisonResult(passed: false, diffPercent: 1.0),
          Uri.parse('foo_test.dart'),
          Uri.parse('/foo/bar/'),
        );
        TestAsyncUtils.verifyAllScopesClosed();
        fail('unexpectedly did not throw');
      } on FlutterError catch (e) {
        final List<String> lines = e.message.split('\n');
        expectSync(lines[0], 'Asynchronous call to guarded function leaked.');
        expectSync(lines[1], 'You must use "await" with all Future-returning test APIs.');
        expectSync(
          lines[2],
          matches(
            r'^The guarded method "generateFailureOutput" from class '
            r'LocalComparisonOutput was called from .*goldens_test.dart on line '
            r'[0-9]+, but never completed before its parent scope closed\.$',
          ),
        );
        expectSync(lines.length, 3);
        final DiagnosticPropertiesBuilder propertiesBuilder = DiagnosticPropertiesBuilder();
        e.debugFillProperties(propertiesBuilder);
        final List<DiagnosticsNode> information = propertiesBuilder.properties;
        expectSync(information.length, 3);
        expectSync(information[0].level, DiagnosticLevel.summary);
        expectSync(information[1].level, DiagnosticLevel.hint);
        expectSync(information[2].level, DiagnosticLevel.info);
      }
    });

    group('compare', () {
      Future<bool> doComparison([String golden = 'golden.png']) {
        final Uri uri = fs.file(fix(golden)).uri;
        return comparator.compare(Uint8List.fromList(_kExpectedPngBytes), uri);
      }

      group('succeeds', () {
        test('when golden file is in same folder as test', () async {
          fs.file(fix('/golden.png')).writeAsBytesSync(_kExpectedPngBytes);
          final bool success = await doComparison();
          expect(success, isTrue);
        });

        test('when golden file is in subfolder of test', () async {
          fs.file(fix('/sub/foo.png'))
            ..createSync(recursive: true)
            ..writeAsBytesSync(_kExpectedPngBytes);
          final bool success = await doComparison('sub/foo.png');
          expect(success, isTrue);
        });

        group('when comparator instantiated with uri that represents file in same folder', () {
          test('and golden file is in same folder as test', () async {
            fs.file(fix('/foo/bar/golden.png'))
              ..createSync(recursive: true)
              ..writeAsBytesSync(_kExpectedPngBytes);
            fs.currentDirectory = fix('/foo/bar');
            comparator = LocalFileComparator(
              Uri.parse('local_test.dart'),
              pathStyle: fs.path.style,
            );
            final bool success = await doComparison();
            expect(success, isTrue);
          });

          test('and golden file is in subfolder of test', () async {
            fs.file(fix('/foo/bar/baz/golden.png'))
              ..createSync(recursive: true)
              ..writeAsBytesSync(_kExpectedPngBytes);
            fs.currentDirectory = fix('/foo/bar');
            comparator = LocalFileComparator(
              Uri.parse('local_test.dart'),
              pathStyle: fs.path.style,
            );
            final bool success = await doComparison('baz/golden.png');
            expect(success, isTrue);
          });
        });
      });

      group('fails', () {
        test('and generates correct output in the correct base location', () async {
          comparator = LocalFileComparator(Uri.parse('local_test.dart'), pathStyle: fs.path.style);
          await fs.file(fix('/golden.png')).writeAsBytes(_kColorFailurePngBytes);
          await expectLater(
            () => doComparison(),
            throwsA(
              isFlutterError.having(
                (FlutterError error) => error.message,
                'message',
                contains('100.00%, 1px diff detected'),
              ),
            ),
          );
          final io.File master = fs.file(fix('/failures/golden_masterImage.png'));
          final io.File test = fs.file(fix('/failures/golden_testImage.png'));
          final io.File isolated = fs.file(fix('/failures/golden_isolatedDiff.png'));
          final io.File masked = fs.file(fix('/failures/golden_maskedDiff.png'));
          expect(master.existsSync(), isTrue);
          expect(test.existsSync(), isTrue);
          expect(isolated.existsSync(), isTrue);
          expect(masked.existsSync(), isTrue);
        });

        test('and generates correct output when files are in a subdirectory', () async {
          comparator = LocalFileComparator(Uri.parse('local_test.dart'), pathStyle: fs.path.style);
          fs.file(fix('subdir/golden.png'))
            ..createSync(recursive: true)
            ..writeAsBytesSync(_kColorFailurePngBytes);
          await expectLater(
            () => doComparison('subdir/golden.png'),
            throwsA(
              isFlutterError.having(
                (FlutterError error) => error.message,
                'message',
                contains('100.00%, 1px diff detected'),
              ),
            ),
          );
          final io.File master = fs.file(fix('/failures/golden_masterImage.png'));
          final io.File test = fs.file(fix('/failures/golden_testImage.png'));
          final io.File isolated = fs.file(fix('/failures/golden_isolatedDiff.png'));
          final io.File masked = fs.file(fix('/failures/golden_maskedDiff.png'));
          expect(master.existsSync(), isTrue);
          expect(test.existsSync(), isTrue);
          expect(isolated.existsSync(), isTrue);
          expect(masked.existsSync(), isTrue);
        });

        test('and generates correct output when images are not the same size', () async {
          await fs.file(fix('/golden.png')).writeAsBytes(_kSizeFailurePngBytes);
          await expectLater(
            () => doComparison(),
            throwsA(
              isFlutterError.having(
                (FlutterError error) => error.message,
                'message',
                contains('image sizes do not match'),
              ),
            ),
          );
          final io.File master = fs.file(fix('/failures/golden_masterImage.png'));
          final io.File test = fs.file(fix('/failures/golden_testImage.png'));
          final io.File isolated = fs.file(fix('/failures/golden_isolatedDiff.png'));
          final io.File masked = fs.file(fix('/failures/golden_maskedDiff.png'));
          expect(master.existsSync(), isTrue);
          expect(test.existsSync(), isTrue);
          expect(isolated.existsSync(), isFalse);
          expect(masked.existsSync(), isFalse);
        });

        test('when golden file does not exist', () async {
          await expectLater(
            () => doComparison(),
            throwsA(
              isA<TestFailure>().having(
                (TestFailure error) => error.message,
                'message',
                contains('Could not be compared against non-existent file'),
              ),
            ),
          );
        });

        test('when images are not the same size', () async {
          await fs.file(fix('/golden.png')).writeAsBytes(_kSizeFailurePngBytes);
          await expectLater(
            () => doComparison(),
            throwsA(
              isFlutterError.having(
                (FlutterError error) => error.message,
                'message',
                contains('image sizes do not match'),
              ),
            ),
          );
        });

        test('when pixels do not match', () async {
          await fs.file(fix('/golden.png')).writeAsBytes(_kColorFailurePngBytes);
          await expectLater(
            () => doComparison(),
            throwsA(
              isFlutterError.having(
                (FlutterError error) => error.message,
                'message',
                contains('100.00%, 1px diff detected'),
              ),
            ),
          );
        });

        test('when golden bytes are empty', () async {
          await fs.file(fix('/golden.png')).writeAsBytes(<int>[]);
          await expectLater(
            () => doComparison(),
            throwsA(
              isFlutterError.having(
                (FlutterError error) => error.message,
                'message',
                contains('null image provided'),
              ),
            ),
          );
        });
      });
    });

    group('update', () {
      test('updates existing file', () async {
        fs.file(fix('/golden.png')).writeAsBytesSync(_kExpectedPngBytes);
        const List<int> newBytes = <int>[11, 12, 13];
        await comparator.update(fs.file('golden.png').uri, Uint8List.fromList(newBytes));
        expect(fs.file(fix('/golden.png')).readAsBytesSync(), newBytes);
      });

      test('creates non-existent file', () async {
        expect(fs.file(fix('/foo.png')).existsSync(), isFalse);
        const List<int> newBytes = <int>[11, 12, 13];
        await comparator.update(fs.file('foo.png').uri, Uint8List.fromList(newBytes));
        expect(fs.file(fix('/foo.png')).existsSync(), isTrue);
        expect(fs.file(fix('/foo.png')).readAsBytesSync(), newBytes);
      });
    });

    group('getTestUri', () {
      test('updates file name with version number', () {
        final Uri key = Uri.parse('foo.png');
        final Uri key1 = comparator.getTestUri(key, 1);
        expect(key1, Uri.parse('foo.1.png'));
      });
      test('does nothing for null version number', () {
        final Uri key = Uri.parse('foo.png');
        final Uri keyNull = comparator.getTestUri(key, null);
        expect(keyNull, Uri.parse('foo.png'));
      });
    });
  });

  group('ComparisonResult', () {
    group('dispose', () {
      test('disposes diffs images', () async {
        final ui.Image image1 = await createTestImage(width: 10, height: 10, cache: false);
        final ui.Image image2 = await createTestImage(width: 15, height: 5, cache: false);
        final ui.Image image3 = await createTestImage(width: 5, height: 10, cache: false);

        final ComparisonResult result = ComparisonResult(
          passed: false,
          diffPercent: 1.0,
          diffs: <String, ui.Image>{'image1': image1, 'image2': image2, 'image3': image3},
        );

        expect(image1.debugDisposed, isFalse);
        expect(image2.debugDisposed, isFalse);
        expect(image3.debugDisposed, isFalse);

        result.dispose();

        expect(image1.debugDisposed, isTrue);
        expect(image2.debugDisposed, isTrue);
        expect(image3.debugDisposed, isTrue);
      });
    });
  });
}
