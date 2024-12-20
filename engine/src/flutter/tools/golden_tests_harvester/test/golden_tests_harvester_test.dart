// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io' as io;

import 'package:golden_tests_harvester/golden_tests_harvester.dart';
import 'package:path/path.dart' as p;
import 'package:test/test.dart';

void main() async {
  Future<void> withTempDirectory(FutureOr<void> Function(io.Directory) callback) async {
    final io.Directory tempDirectory = await io.Directory.systemTemp.createTemp(
      'golden_tests_harvester_test.',
    );
    try {
      await callback(tempDirectory);
    } finally {
      await tempDirectory.delete(recursive: true);
    }
  }

  test('should fail on a missing directory', () async {
    await withTempDirectory((io.Directory tempDirectory) async {
      final StringSink stderr = StringBuffer();
      expect(
        () async {
          await Harvester.create(
            io.Directory(p.join(tempDirectory.path, 'non_existent')),
            stderr,
            addImageToSkiaGold: _alwaysThrowsAddImg,
          );
        },
        throwsA(
          isA<ArgumentError>().having(
            (error) => error.message,
            'message',
            contains('non_existent'),
          ),
        ),
      );
      expect(stderr.toString(), isEmpty);
    });
  });

  test('should require a file named "digest.json" in the working directory', () async {
    await withTempDirectory((io.Directory tempDirectory) async {
      final StringSink stderr = StringBuffer();
      await expectLater(
        () async {
          await Harvester.create(tempDirectory, stderr, addImageToSkiaGold: _alwaysThrowsAddImg);
        },
        throwsA(
          isA<StateError>().having(
            (error) => error.toString(),
            'toString()',
            contains('digest.json'),
          ),
        ),
      );
      expect(stderr.toString(), isEmpty);
    });
  });

  test('should throw if "digest.json" is in an unexpected format', () async {
    await withTempDirectory((io.Directory tempDirectory) async {
      final StringSink stderr = StringBuffer();
      final io.File digestsFile = io.File(p.join(tempDirectory.path, 'digest.json'));
      await digestsFile.writeAsString('{"dimensions": "not a map", "entries": []}');
      await expectLater(
        () async {
          await Harvester.create(tempDirectory, stderr, addImageToSkiaGold: _alwaysThrowsAddImg);
        },
        throwsA(
          isA<FormatException>().having(
            (error) => error.message,
            'message',
            contains('dimensions'),
          ),
        ),
      );
      expect(stderr.toString(), isEmpty);
    });
  });

  test('should fail eagerly if addImg fails', () async {
    await withTempDirectory((io.Directory tempDirectory) async {
      final io.File digestsFile = io.File(p.join(tempDirectory.path, 'digest.json'));
      final StringSink stderr = StringBuffer();
      await digestsFile.writeAsString('''
      {
        "dimensions": {},
        "entries": [
        {
          "filename": "test_name_1.png",
          "width": 100,
          "height": 100,
          "maxDiffPixelsPercent": 0.01,
          "maxColorDelta": 0
        }
        ]
      }
      ''');

      final Harvester harvester = await Harvester.create(
        tempDirectory,
        stderr,
        addImageToSkiaGold: _alwaysThrowsAddImg,
      );
      expect(
        () => harvest(harvester),
        throwsA(
          isA<FailedComparisonException>()
              .having((e) => e.testName, 'testName', 'test_name_1.png')
              .having(
                (e) => e.toString(),
                'toString()',
                contains('Failed comparison: test_name_1.png'),
              ),
        ),
      );
    });
  });

  test('should invoke addImg per test', () async {
    await withTempDirectory((io.Directory tempDirectory) async {
      final io.File digestsFile = io.File(p.join(tempDirectory.path, 'digest.json'));
      await digestsFile.writeAsString('''
        {
          "dimensions": {},
          "entries": [
            {
              "filename": "test_name_1.png",
              "width": 100,
              "height": 100,
              "maxDiffPixelsPercent": 0.01,
              "maxColorDelta": 0
            },
            {
              "filename": "test_name_2.png",
              "width": 200,
              "height": 200,
              "maxDiffPixelsPercent": 0.02,
              "maxColorDelta": 1
            }
          ]
        }
      ''');
      final List<String> addImgCalls = <String>[];
      final StringSink stderr = StringBuffer();

      final Harvester harvester = await Harvester.create(
        tempDirectory,
        stderr,
        addImageToSkiaGold: (
          String testName,
          io.File goldenFile, {
          required int screenshotSize,
          double differentPixelsRate = 0.01,
          int pixelColorDelta = 0,
        }) async {
          addImgCalls.add('$testName $screenshotSize $differentPixelsRate $pixelColorDelta');
        },
      );
      await harvest(harvester);
      expect(addImgCalls, <String>['test_name_1.png 10000 0.01 0', 'test_name_2.png 40000 0.02 1']);
    });
  });

  test('client has dimensions', () async {
    await withTempDirectory((io.Directory tempDirectory) async {
      final StringSink stderr = StringBuffer();
      final io.File digestsFile = io.File(p.join(tempDirectory.path, 'digest.json'));
      await digestsFile.writeAsString('{"dimensions": {"key":"value"}, "entries": []}');
      final Harvester harvester = await Harvester.create(tempDirectory, stderr);
      expect(harvester is SkiaGoldHarvester, true);
      final SkiaGoldHarvester skiaGoldHarvester = harvester as SkiaGoldHarvester;
      expect(skiaGoldHarvester.client.dimensions, <String, String>{'key': 'value'});
    });
  });

  test('throws without GOLDCTL', () async {
    await withTempDirectory((io.Directory tempDirectory) async {
      final StringSink stderr = StringBuffer();
      final io.File digestsFile = io.File(p.join(tempDirectory.path, 'digest.json'));
      await digestsFile.writeAsString('''
{
  "dimensions": {"key":"value"},
  "entries": [
    {
      "filename": "foo.png",
      "width": 100,
      "height": 100,
      "maxDiffPixelsPercent": 0.01,
      "maxColorDelta": 0
    }
  ]}
''');
      final Harvester harvester = await Harvester.create(tempDirectory, stderr);
      expect(
        () => harvest(harvester),
        throwsA(isA<StateError>().having((t) => t.message, 'message', contains('GOLDCTL'))),
      );
      expect(stderr.toString(), isEmpty);
    });
  });
}

final class _IntentionalError extends Error {}

Future<void> _alwaysThrowsAddImg(
  String testName,
  io.File goldenFile, {
  required int screenshotSize,
  double differentPixelsRate = 0.01,
  int pixelColorDelta = 0,
}) async {
  throw _IntentionalError();
}
