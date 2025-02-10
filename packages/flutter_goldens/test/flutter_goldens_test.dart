// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// See also dev/automated_tests/flutter_test/flutter_gold_test.dart

import 'dart:convert';
import 'dart:io' hide Directory;

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter_goldens/flutter_goldens.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:path/path.dart' as path;
import 'package:platform/platform.dart';
import 'package:process/process.dart';

import 'json_templates.dart';

// TODO(ianh): make sure all constructors order their arguments in a manner consistent with the defined parameter order

const String _kFlutterRoot = '/flutter';

// 1x1 transparent pixel
const List<int> _kTestPngBytes = <int>[
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

void main() {
  group('SkiaGoldClient', () {
    test('web CanvasKit test', () async {
      final MemoryFileSystem fs = MemoryFileSystem();
      final FakePlatform platform = FakePlatform(
        environment: <String, String>{
          'GOLDCTL': 'goldctl',
          'FLUTTER_ROOT': _kFlutterRoot,
          'FLUTTER_TEST_BROWSER': 'Chrome',
          'FLUTTER_WEB_RENDERER': 'canvaskit',
        },
        operatingSystem: 'macos',
      );
      final FakeProcessManager process = FakeProcessManager();
      final FakeHttpClient fakeHttpClient = FakeHttpClient();
      fs.directory(_kFlutterRoot).createSync(recursive: true);
      final Directory workDirectory = fs.directory('/workDirectory')..createSync(recursive: true);
      final SkiaGoldClient skiaClient = SkiaGoldClient(
        workDirectory,
        fs: fs,
        process: process,
        platform: platform,
        httpClient: fakeHttpClient,
        log: (String message) => fail('skia gold client printed unexpected output: "$message"'),
      );

      final File goldenFile = fs.file('/workDirectory/temp/golden_file_test.png')
        ..createSync(recursive: true);

      const RunInvocation goldctlInvocation = RunInvocation(<String>[
        'goldctl',
        'imgtest',
        'add',
        '--work-dir',
        '/workDirectory/temp',
        '--test-name',
        'golden_file_test',
        '--png-file',
        '/workDirectory/temp/golden_file_test.png',
        '--passfail',
      ], null);
      process.processResults[goldctlInvocation] = ProcessResult(123, 0, '', '');

      expect(await skiaClient.imgtestAdd('golden_file_test.png', goldenFile), isTrue);
    });

    test('auth performs minimal work if already authorized', () async {
      final MemoryFileSystem fs = MemoryFileSystem();
      final FakePlatform platform = FakePlatform(
        environment: <String, String>{'FLUTTER_ROOT': _kFlutterRoot},
        operatingSystem: 'macos',
      );
      final FakeProcessManager process = FakeProcessManager();
      final FakeHttpClient fakeHttpClient = FakeHttpClient();
      fs.directory(_kFlutterRoot).createSync(recursive: true);
      final Directory workDirectory = fs.directory('/workDirectory')..createSync(recursive: true);
      final SkiaGoldClient skiaClient = SkiaGoldClient(
        workDirectory,
        fs: fs,
        process: process,
        platform: platform,
        httpClient: fakeHttpClient,
        log: (String message) => fail('skia gold client printed unexpected output: "$message"'),
      );
      final File authFile = fs.file('/workDirectory/temp/auth_opt.json')
        ..createSync(recursive: true);
      authFile.writeAsStringSync(authTemplate());
      process.fallbackProcessResult = ProcessResult(123, 0, '', '');
      await skiaClient.auth();

      expect(process.workingDirectories, isEmpty);
    });

    test('gsutil is checked when authorization file is present', () async {
      final MemoryFileSystem fs = MemoryFileSystem();
      final FakePlatform platform = FakePlatform(
        environment: <String, String>{'FLUTTER_ROOT': _kFlutterRoot},
        operatingSystem: 'macos',
      );
      final FakeProcessManager process = FakeProcessManager();
      final FakeHttpClient fakeHttpClient = FakeHttpClient();
      fs.directory(_kFlutterRoot).createSync(recursive: true);
      final Directory workDirectory = fs.directory('/workDirectory')..createSync(recursive: true);
      final SkiaGoldClient skiaClient = SkiaGoldClient(
        workDirectory,
        fs: fs,
        process: process,
        platform: platform,
        httpClient: fakeHttpClient,
        log: (String message) => fail('skia gold client printed unexpected output: "$message"'),
      );
      final File authFile = fs.file('/workDirectory/temp/auth_opt.json')
        ..createSync(recursive: true);
      authFile.writeAsStringSync(authTemplate(gsutil: true));
      expect(await skiaClient.clientIsAuthorized(), isFalse);
    });

    test('throws for error state from auth', () async {
      final MemoryFileSystem fs = MemoryFileSystem();
      final FakePlatform platform = FakePlatform(
        environment: <String, String>{
          'FLUTTER_ROOT': _kFlutterRoot,
          'GOLD_SERVICE_ACCOUNT': 'Service Account',
          'GOLDCTL': 'goldctl',
        },
        operatingSystem: 'macos',
      );
      final FakeProcessManager process = FakeProcessManager();
      final FakeHttpClient fakeHttpClient = FakeHttpClient();
      fs.directory(_kFlutterRoot).createSync(recursive: true);
      final Directory workDirectory = fs.directory('/workDirectory')..createSync(recursive: true);
      final SkiaGoldClient skiaClient = SkiaGoldClient(
        workDirectory,
        fs: fs,
        process: process,
        platform: platform,
        httpClient: fakeHttpClient,
        log: (String message) => fail('skia gold client printed unexpected output: "$message"'),
      );

      process.fallbackProcessResult = ProcessResult(123, 1, 'Fallback failure', 'Fallback failure');

      expect(skiaClient.auth(), throwsException);
    });

    test('throws for error state from init', () {
      final MemoryFileSystem fs = MemoryFileSystem();
      final FakePlatform platform = FakePlatform(
        environment: <String, String>{'FLUTTER_ROOT': _kFlutterRoot, 'GOLDCTL': 'goldctl'},
        operatingSystem: 'macos',
      );
      final FakeProcessManager process = FakeProcessManager();
      final FakeHttpClient fakeHttpClient = FakeHttpClient();
      fs.directory(_kFlutterRoot).createSync(recursive: true);
      final Directory workDirectory = fs.directory('/workDirectory')..createSync(recursive: true);
      final SkiaGoldClient skiaClient = SkiaGoldClient(
        workDirectory,
        fs: fs,
        process: process,
        platform: platform,
        httpClient: fakeHttpClient,
        log: (String message) => fail('skia gold client printed unexpected output: "$message"'),
      );

      const RunInvocation gitInvocation = RunInvocation(<String>[
        'git',
        'rev-parse',
        'HEAD',
      ], '/flutter');
      const RunInvocation goldctlInvocation = RunInvocation(<String>[
        'goldctl',
        'imgtest',
        'init',
        '--instance',
        'flutter',
        '--work-dir',
        '/workDirectory/temp',
        '--commit',
        '12345678',
        '--keys-file',
        '/workDirectory/keys.json',
        '--failure-file',
        '/workDirectory/failures.json',
        '--passfail',
      ], null);
      process.processResults[gitInvocation] = ProcessResult(12345678, 0, '12345678', '');
      process.processResults[goldctlInvocation] = ProcessResult(
        123,
        1,
        'Expected failure',
        'Expected failure',
      );
      process.fallbackProcessResult = ProcessResult(123, 1, 'Fallback failure', 'Fallback failure');

      expect(skiaClient.imgtestInit(), throwsException);
    });

    test('Only calls init once', () async {
      final MemoryFileSystem fs = MemoryFileSystem();
      final FakePlatform platform = FakePlatform(
        environment: <String, String>{'FLUTTER_ROOT': _kFlutterRoot, 'GOLDCTL': 'goldctl'},
        operatingSystem: 'macos',
      );
      final FakeProcessManager process = FakeProcessManager();
      final FakeHttpClient fakeHttpClient = FakeHttpClient();
      fs.directory(_kFlutterRoot).createSync(recursive: true);
      final Directory workDirectory = fs.directory('/workDirectory')..createSync(recursive: true);
      final SkiaGoldClient skiaClient = SkiaGoldClient(
        workDirectory,
        fs: fs,
        process: process,
        platform: platform,
        httpClient: fakeHttpClient,
        log: (String message) => fail('skia gold client printed unexpected output: "$message"'),
      );

      const RunInvocation gitInvocation = RunInvocation(<String>[
        'git',
        'rev-parse',
        'HEAD',
      ], '/flutter');
      const RunInvocation goldctlInvocation = RunInvocation(<String>[
        'goldctl',
        'imgtest',
        'init',
        '--instance',
        'flutter',
        '--work-dir',
        '/workDirectory/temp',
        '--commit',
        '1234',
        '--keys-file',
        '/workDirectory/keys.json',
        '--failure-file',
        '/workDirectory/failures.json',
        '--passfail',
      ], null);
      process.processResults[gitInvocation] = ProcessResult(1234, 0, '1234', '');
      process.processResults[goldctlInvocation] = ProcessResult(5678, 0, '5678', '');
      process.fallbackProcessResult = ProcessResult(123, 1, 'Fallback failure', 'Fallback failure');

      // First call
      await skiaClient.imgtestInit();

      // Remove fake process result.
      // If the init call is executed again, the fallback process will throw.
      process.processResults.remove(goldctlInvocation);

      // Second call
      await skiaClient.imgtestInit();
    });

    test('Only calls tryjob init once', () async {
      final MemoryFileSystem fs = MemoryFileSystem();
      final FakePlatform platform = FakePlatform(
        environment: <String, String>{
          'FLUTTER_ROOT': _kFlutterRoot,
          'GOLDCTL': 'goldctl',
          'SWARMING_TASK_ID': '4ae997b50dfd4d11',
          'LOGDOG_STREAM_PREFIX': 'buildbucket/cr-buildbucket.appspot.com/8885996262141582672',
          'GOLD_TRYJOB': 'refs/pull/49815/head',
        },
        operatingSystem: 'macos',
      );
      final FakeProcessManager process = FakeProcessManager();
      final FakeHttpClient fakeHttpClient = FakeHttpClient();
      fs.directory(_kFlutterRoot).createSync(recursive: true);
      final Directory workDirectory = fs.directory('/workDirectory')..createSync(recursive: true);
      final SkiaGoldClient skiaClient = SkiaGoldClient(
        workDirectory,
        fs: fs,
        process: process,
        platform: platform,
        httpClient: fakeHttpClient,
        log: (String message) => fail('skia gold client printed unexpected output: "$message"'),
      );

      const RunInvocation gitInvocation = RunInvocation(<String>[
        'git',
        'rev-parse',
        'HEAD',
      ], '/flutter');
      const RunInvocation goldctlInvocation = RunInvocation(<String>[
        'goldctl',
        'imgtest',
        'init',
        '--instance',
        'flutter',
        '--work-dir',
        '/workDirectory/temp',
        '--commit',
        '1234',
        '--keys-file',
        '/workDirectory/keys.json',
        '--failure-file',
        '/workDirectory/failures.json',
        '--passfail',
        '--crs',
        'github',
        '--patchset_id',
        '1234',
        '--changelist',
        '49815',
        '--cis',
        'buildbucket',
        '--jobid',
        '8885996262141582672',
      ], null);
      process.processResults[gitInvocation] = ProcessResult(1234, 0, '1234', '');
      process.processResults[goldctlInvocation] = ProcessResult(5678, 0, '5678', '');
      process.fallbackProcessResult = ProcessResult(123, 1, 'Fallback failure', 'Fallback failure');

      // First call
      await skiaClient.tryjobInit();

      // Remove fake process result.
      // If the init call is executed again, the fallback process will throw.
      process.processResults.remove(goldctlInvocation);

      // Second call
      await skiaClient.tryjobInit();
    });

    test('throws for error state from imgtestAdd', () {
      final MemoryFileSystem fs = MemoryFileSystem();
      final File goldenFile = fs.file('/workDirectory/temp/golden_file_test.png')
        ..createSync(recursive: true);
      final FakePlatform platform = FakePlatform(
        environment: <String, String>{'FLUTTER_ROOT': _kFlutterRoot, 'GOLDCTL': 'goldctl'},
        operatingSystem: 'macos',
      );
      final FakeProcessManager process = FakeProcessManager();
      final FakeHttpClient fakeHttpClient = FakeHttpClient();
      fs.directory(_kFlutterRoot).createSync(recursive: true);
      final Directory workDirectory = fs.directory('/workDirectory')..createSync(recursive: true);
      final SkiaGoldClient skiaClient = SkiaGoldClient(
        workDirectory,
        fs: fs,
        process: process,
        platform: platform,
        httpClient: fakeHttpClient,
        log: (String message) => fail('skia gold client printed unexpected output: "$message"'),
      );
      const RunInvocation goldctlInvocation = RunInvocation(<String>[
        'goldctl',
        'imgtest',
        'add',
        '--work-dir',
        '/workDirectory/temp',
        '--test-name',
        'golden_file_test',
        '--png-file',
        '/workDirectory/temp/golden_file_test.png',
        '--passfail',
      ], null);
      process.processResults[goldctlInvocation] = ProcessResult(
        123,
        1,
        'Expected failure',
        'Expected failure',
      );
      process.fallbackProcessResult = ProcessResult(123, 1, 'Fallback failure', 'Fallback failure');

      expect(skiaClient.imgtestAdd('golden_file_test', goldenFile), throwsException);
    });

    test('correctly inits tryjob for luci', () async {
      final MemoryFileSystem fs = MemoryFileSystem();
      final FakePlatform platform = FakePlatform(
        environment: <String, String>{
          'FLUTTER_ROOT': _kFlutterRoot,
          'GOLDCTL': 'goldctl',
          'SWARMING_TASK_ID': '4ae997b50dfd4d11',
          'LOGDOG_STREAM_PREFIX': 'buildbucket/cr-buildbucket.appspot.com/8885996262141582672',
          'GOLD_TRYJOB': 'refs/pull/49815/head',
        },
        operatingSystem: 'macos',
      );
      final FakeProcessManager process = FakeProcessManager();
      final FakeHttpClient fakeHttpClient = FakeHttpClient();
      fs.directory(_kFlutterRoot).createSync(recursive: true);
      final Directory workDirectory = fs.directory('/workDirectory')..createSync(recursive: true);
      final SkiaGoldClient skiaClient = SkiaGoldClient(
        workDirectory,
        fs: fs,
        process: process,
        platform: platform,
        httpClient: fakeHttpClient,
        log: (String message) => fail('skia gold client printed unexpected output: "$message"'),
      );

      final List<String> ciArguments = skiaClient.getCIArguments();

      expect(
        ciArguments,
        equals(<String>[
          '--changelist',
          '49815',
          '--cis',
          'buildbucket',
          '--jobid',
          '8885996262141582672',
        ]),
      );
    });

    test('Creates traceID correctly', () async {
      final MemoryFileSystem fs = MemoryFileSystem();
      final FakePlatform platform = FakePlatform(
        environment: <String, String>{
          'FLUTTER_ROOT': _kFlutterRoot,
          'GOLDCTL': 'goldctl',
          'SWARMING_TASK_ID': '4ae997b50dfd4d11',
          'LOGDOG_STREAM_PREFIX': 'buildbucket/cr-buildbucket.appspot.com/8885996262141582672',
          'GOLD_TRYJOB': 'refs/pull/49815/head',
        },
        operatingSystem: 'linux',
      );
      final FakeProcessManager process = FakeProcessManager();
      final FakeHttpClient fakeHttpClient = FakeHttpClient();
      fs.directory(_kFlutterRoot).createSync(recursive: true);
      final Directory workDirectory = fs.directory('/workDirectory')..createSync(recursive: true);
      final SkiaGoldClient skiaClient = SkiaGoldClient(
        workDirectory,
        fs: fs,
        process: process,
        platform: platform,
        httpClient: fakeHttpClient,
        log: (String message) => fail('skia gold client printed unexpected output: "$message"'),
      );

      expect(skiaClient.getTraceID('flutter.golden.1'), equals('ae18c7a6aa48e0685525dfe8fdf79003'));
    });

    test('Creates traceID correctly - Browser', () async {
      final MemoryFileSystem fs = MemoryFileSystem();
      final FakePlatform platform = FakePlatform(
        environment: <String, String>{
          'FLUTTER_ROOT': _kFlutterRoot,
          'GOLDCTL': 'goldctl',
          'SWARMING_TASK_ID': '4ae997b50dfd4d11',
          'LOGDOG_STREAM_PREFIX': 'buildbucket/cr-buildbucket.appspot.com/8885996262141582672',
          'GOLD_TRYJOB': 'refs/pull/49815/head',
          'FLUTTER_TEST_BROWSER': 'chrome',
        },
        operatingSystem: 'linux',
      );
      final FakeProcessManager process = FakeProcessManager();
      final FakeHttpClient fakeHttpClient = FakeHttpClient();
      fs.directory(_kFlutterRoot).createSync(recursive: true);
      final Directory workDirectory = fs.directory('/workDirectory')..createSync(recursive: true);
      final SkiaGoldClient skiaClient = SkiaGoldClient(
        workDirectory,
        fs: fs,
        process: process,
        platform: platform,
        httpClient: fakeHttpClient,
        log: (String message) => fail('skia gold client printed unexpected output: "$message"'),
      );

      expect(skiaClient.getTraceID('flutter.golden.1'), equals('e9d5c296c48e7126808520e9cc191243'));
    });

    test('Creates traceID correctly - locally - should defer to luci traceID', () async {
      final MemoryFileSystem fs = MemoryFileSystem();
      final FakePlatform platform = FakePlatform(
        environment: <String, String>{'FLUTTER_ROOT': _kFlutterRoot},
        operatingSystem: 'macos',
      );
      final FakeProcessManager process = FakeProcessManager();
      final FakeHttpClient fakeHttpClient = FakeHttpClient();
      fs.directory(_kFlutterRoot).createSync(recursive: true);
      final Directory workDirectory = fs.directory('/workDirectory')..createSync(recursive: true);
      final SkiaGoldClient skiaClient = SkiaGoldClient(
        workDirectory,
        fs: fs,
        process: process,
        platform: platform,
        httpClient: fakeHttpClient,
        log: (String message) => fail('skia gold client printed unexpected output: "$message"'),
      );
      expect(skiaClient.getTraceID('flutter.golden.1'), equals('9968695b9ae78cdb77cbb2be621ca2d6'));
    });

    test('throws for error state from imgtestAdd', () {
      final MemoryFileSystem fs = MemoryFileSystem();
      final File goldenFile = fs.file('/workDirectory/temp/golden_file_test.png')
        ..createSync(recursive: true);
      final FakePlatform platform = FakePlatform(
        environment: <String, String>{'FLUTTER_ROOT': _kFlutterRoot, 'GOLDCTL': 'goldctl'},
        operatingSystem: 'macos',
      );
      final FakeProcessManager process = FakeProcessManager();
      final FakeHttpClient fakeHttpClient = FakeHttpClient();
      fs.directory(_kFlutterRoot).createSync(recursive: true);
      final Directory workDirectory = fs.directory('/workDirectory')..createSync(recursive: true);
      final SkiaGoldClient skiaClient = SkiaGoldClient(
        workDirectory,
        fs: fs,
        process: process,
        platform: platform,
        httpClient: fakeHttpClient,
        log: (String message) => fail('skia gold client printed unexpected output: "$message"'),
      );
      const RunInvocation goldctlInvocation = RunInvocation(<String>[
        'goldctl',
        'imgtest',
        'add',
        '--work-dir',
        '/workDirectory/temp',
        '--test-name',
        'golden_file_test',
        '--png-file',
        '/workDirectory/temp/golden_file_test.png',
        '--passfail',
      ], null);
      process.processResults[goldctlInvocation] = ProcessResult(
        123,
        1,
        'Expected failure',
        'Expected failure',
      );
      process.fallbackProcessResult = ProcessResult(123, 1, 'Fallback failure', 'Fallback failure');

      expect(
        skiaClient.imgtestAdd('golden_file_test', goldenFile),
        throwsA(
          isA<SkiaException>().having(
            (SkiaException error) => error.message,
            'message',
            contains('result-state.json'),
          ),
        ),
      );
    });

    test('throws for error state from tryjobAdd', () {
      final MemoryFileSystem fs = MemoryFileSystem();
      final File goldenFile = fs.file('/workDirectory/temp/golden_file_test.png')
        ..createSync(recursive: true);
      final FakePlatform platform = FakePlatform(
        environment: <String, String>{'FLUTTER_ROOT': _kFlutterRoot, 'GOLDCTL': 'goldctl'},
        operatingSystem: 'macos',
      );
      final FakeProcessManager process = FakeProcessManager();
      final FakeHttpClient fakeHttpClient = FakeHttpClient();
      fs.directory(_kFlutterRoot).createSync(recursive: true);
      final Directory workDirectory = fs.directory('/workDirectory')..createSync(recursive: true);
      final SkiaGoldClient skiaClient = SkiaGoldClient(
        workDirectory,
        fs: fs,
        process: process,
        platform: platform,
        httpClient: fakeHttpClient,
        log: (String message) => fail('skia gold client printed unexpected output: "$message"'),
      );
      const RunInvocation goldctlInvocation = RunInvocation(<String>[
        'goldctl',
        'imgtest',
        'add',
        '--work-dir',
        '/workDirectory/temp',
        '--test-name',
        'golden_file_test',
        '--png-file',
        '/workDirectory/temp/golden_file_test.png',
        '--passfail',
      ], null);
      process.processResults[goldctlInvocation] = ProcessResult(
        123,
        1,
        'Expected failure',
        'Expected failure',
      );
      process.fallbackProcessResult = ProcessResult(123, 1, 'Fallback failure', 'Fallback failure');
      expect(
        skiaClient.tryjobAdd('golden_file_test', goldenFile),
        throwsA(
          isA<SkiaException>().having(
            (SkiaException error) => error.message,
            'message',
            contains('result-state.json'),
          ),
        ),
      );
    });

    group('Request Handling', () {
      test('image bytes are processed properly', () async {
        const String expectation = '55109a4bed52acc780530f7a9aeff6c0';
        final MemoryFileSystem fs = MemoryFileSystem();
        final FakePlatform platform = FakePlatform(
          environment: <String, String>{'FLUTTER_ROOT': _kFlutterRoot},
          operatingSystem: 'macos',
        );
        final FakeProcessManager process = FakeProcessManager();
        final FakeHttpClient fakeHttpClient = FakeHttpClient();
        fs.directory(_kFlutterRoot).createSync(recursive: true);
        final Directory workDirectory = fs.directory('/workDirectory')..createSync(recursive: true);
        final SkiaGoldClient skiaClient = SkiaGoldClient(
          workDirectory,
          fs: fs,
          process: process,
          platform: platform,
          httpClient: fakeHttpClient,
          log: (String message) => fail('skia gold client printed unexpected output: "$message"'),
        );
        final Uri imageUrl = Uri.parse('https://flutter-gold.skia.org/img/images/$expectation.png');
        final FakeHttpClientRequest fakeImageRequest = FakeHttpClientRequest();
        final FakeHttpImageResponse fakeImageResponse = FakeHttpImageResponse(
          imageResponseTemplate(),
        );

        fakeHttpClient.request = fakeImageRequest;
        fakeImageRequest.response = fakeImageResponse;

        final List<int> masterBytes = await skiaClient.getImageBytes(expectation);

        expect(fakeHttpClient.lastUri, imageUrl);
        expect(masterBytes, equals(_kTestPngBytes));
      });
    });
  });

  group('FlutterGoldenFileComparator', () {
    test('calculates the basedir correctly from defaultComparator for local testing', () async {
      final MemoryFileSystem fs = MemoryFileSystem();
      final FakePlatform platform = FakePlatform(
        environment: <String, String>{'FLUTTER_ROOT': _kFlutterRoot},
        operatingSystem: 'macos',
      );
      fs.directory(_kFlutterRoot).createSync(recursive: true);
      final FakeLocalFileComparator defaultComparator = FakeLocalFileComparator();
      final Directory flutterRoot = fs.directory(platform.environment['FLUTTER_ROOT'])
        ..createSync(recursive: true);
      defaultComparator.basedir = flutterRoot.childDirectory('baz').uri;
      final Directory basedir = FlutterGoldenFileComparator.getBaseDirectory(
        defaultComparator,
        platform: platform,
        fs: fs,
      );
      expect(basedir.uri, fs.directory('/flutter/bin/cache/pkg/skia_goldens/baz').uri);
    });

    test('ignores version number', () {
      final List<String> log = <String>[];
      final MemoryFileSystem fs = MemoryFileSystem();
      final FakePlatform platform = FakePlatform(
        environment: <String, String>{'FLUTTER_ROOT': _kFlutterRoot},
        operatingSystem: 'macos',
      );
      fs.directory(_kFlutterRoot).createSync(recursive: true);
      final Directory basedir = fs.directory('flutter/test/library/')..createSync(recursive: true);
      final FlutterGoldenFileComparator comparator = FlutterPostSubmitFileComparator(
        basedir.uri,
        FakeSkiaGoldClient(),
        fs: fs,
        platform: platform,
        log: log.add,
      );
      final Uri key = comparator.getTestUri(Uri.parse('foo.png'), 1);
      expect(key, Uri.parse('foo.png'));
      expect(log, isEmpty);
    });

    test('adds namePrefix', () async {
      final List<String> log = <String>[];
      final MemoryFileSystem fs = MemoryFileSystem();
      final FakePlatform platform = FakePlatform(
        environment: <String, String>{'FLUTTER_ROOT': _kFlutterRoot},
        operatingSystem: 'macos',
      );
      fs.directory(_kFlutterRoot).createSync(recursive: true);
      const String libraryName = 'sidedishes';
      const String namePrefix = 'tomatosalad';
      const String fileName = 'lettuce.png';
      final FakeSkiaGoldClient fakeSkiaClient = FakeSkiaGoldClient();
      final Directory basedir = fs.directory('flutter/test/$libraryName/')
        ..createSync(recursive: true);
      final FlutterGoldenFileComparator comparator = FlutterPostSubmitFileComparator(
        basedir.uri,
        fakeSkiaClient,
        fs: fs,
        platform: platform,
        namePrefix: namePrefix,
        log: log.add,
      );
      await comparator.compare(Uint8List.fromList(_kTestPngBytes), Uri.parse(fileName));
      expect(fakeSkiaClient.testNames.single, '$namePrefix.$libraryName.$fileName');
      expect(log, isEmpty);
    });

    group('Post-Submit', () {
      test('asserts .png format', () async {
        final List<String> log = <String>[];
        final MemoryFileSystem fs = MemoryFileSystem();
        final FakePlatform platform = FakePlatform(
          environment: <String, String>{'FLUTTER_ROOT': _kFlutterRoot},
          operatingSystem: 'macos',
        );
        fs.directory(_kFlutterRoot).createSync(recursive: true);
        final Directory basedir = fs.directory('flutter/test/library/')
          ..createSync(recursive: true);
        final FakeSkiaGoldClient fakeSkiaClient = FakeSkiaGoldClient();
        final FlutterGoldenFileComparator comparator = FlutterPostSubmitFileComparator(
          basedir.uri,
          fakeSkiaClient,
          fs: fs,
          platform: platform,
          log: log.add,
        );
        await expectLater(
          () async {
            return comparator.compare(
              Uint8List.fromList(_kTestPngBytes),
              Uri.parse('flutter.golden_test.1'),
            );
          },
          throwsA(
            isA<AssertionError>().having(
              (AssertionError error) => error.toString(),
              'description',
              contains(
                'Golden files in the Flutter framework must end with the file '
                'extension .png.',
              ),
            ),
          ),
        );
        expect(log, isEmpty);
      });

      test('calls init during compare', () {
        final List<String> log = <String>[];
        final MemoryFileSystem fs = MemoryFileSystem();
        final FakePlatform platform = FakePlatform(
          environment: <String, String>{'FLUTTER_ROOT': _kFlutterRoot},
          operatingSystem: 'macos',
        );
        fs.directory(_kFlutterRoot).createSync(recursive: true);
        final Directory basedir = fs.directory('flutter/test/library/')
          ..createSync(recursive: true);
        final FakeSkiaGoldClient fakeSkiaClient = FakeSkiaGoldClient();
        final FlutterGoldenFileComparator comparator = FlutterPostSubmitFileComparator(
          basedir.uri,
          fakeSkiaClient,
          fs: fs,
          platform: platform,
          log: log.add,
        );
        expect(fakeSkiaClient.initCalls, 0);
        comparator.compare(
          Uint8List.fromList(_kTestPngBytes),
          Uri.parse('flutter.golden_test.1.png'),
        );
        expect(fakeSkiaClient.initCalls, 1);
        expect(log, isEmpty);
      });

      test('does not call init in during construction', () {
        final MemoryFileSystem fs = MemoryFileSystem();
        final FakePlatform platform = FakePlatform(
          environment: <String, String>{'FLUTTER_ROOT': _kFlutterRoot},
          operatingSystem: 'macos',
        );
        fs.directory(_kFlutterRoot).createSync(recursive: true);
        final FakeSkiaGoldClient fakeSkiaClient = FakeSkiaGoldClient();
        expect(fakeSkiaClient.initCalls, 0);
        FlutterPostSubmitFileComparator.fromLocalFileComparator(
          localFileComparator: LocalFileComparator(Uri.parse('/test'), pathStyle: path.Style.posix),
          platform: platform,
          goldens: fakeSkiaClient,
          log: (String message) => fail('skia gold client printed unexpected output: "$message"'),
          fs: fs,
          process: FakeProcessManager(),
          httpClient: FakeHttpClient(),
        );
        expect(fakeSkiaClient.initCalls, 0);
      });

      test('reports a failure as a TestFailure', () async {
        final List<String> log = <String>[];
        final MemoryFileSystem fs = MemoryFileSystem();
        final FakePlatform platform = FakePlatform(
          environment: <String, String>{'FLUTTER_ROOT': _kFlutterRoot},
          operatingSystem: 'macos',
        );
        fs.directory(_kFlutterRoot).createSync(recursive: true);
        final Directory basedir = fs.directory('flutter/test/library/')
          ..createSync(recursive: true);
        final FlutterGoldenFileComparator comparator = FlutterPostSubmitFileComparator(
          basedir.uri,
          ThrowsOnImgTestAddSkiaClient(
            message: 'Skia Gold received an unapproved image in post-submit',
          ),
          fs: fs,
          platform: platform,
          log: log.add,
        );
        await expectLater(
          () async {
            return comparator.compare(
              Uint8List.fromList(_kTestPngBytes),
              Uri.parse('flutter.golden_test.1.png'),
            );
          },
          throwsA(
            isA<TestFailure>().having(
              (TestFailure error) => error.toString(),
              'description',
              contains('Skia Gold received an unapproved image in post-submit'),
            ),
          ),
        );
        expect(log, isEmpty);
      });
    });

    group('Pre-Submit', () {
      test('asserts .png format', () async {
        final List<String> log = <String>[];
        final MemoryFileSystem fs = MemoryFileSystem();
        final FakePlatform platform = FakePlatform(
          environment: <String, String>{'FLUTTER_ROOT': _kFlutterRoot},
          operatingSystem: 'macos',
        );
        fs.directory(_kFlutterRoot).createSync(recursive: true);
        final Directory basedir = fs.directory('flutter/test/library/')
          ..createSync(recursive: true);
        final FakeSkiaGoldClient fakeSkiaClient = FakeSkiaGoldClient();
        final FlutterGoldenFileComparator comparator = FlutterPreSubmitFileComparator(
          basedir.uri,
          fakeSkiaClient,
          fs: fs,
          platform: platform,
          log: log.add,
        );
        await expectLater(
          () async {
            return comparator.compare(
              Uint8List.fromList(_kTestPngBytes),
              Uri.parse('flutter.golden_test.1'),
            );
          },
          throwsA(
            isA<AssertionError>().having(
              (AssertionError error) => error.toString(),
              'description',
              contains(
                'Golden files in the Flutter framework must end with the file '
                'extension .png.',
              ),
            ),
          ),
        );
        expect(log, isEmpty);
      });

      test('calls init during compare', () {
        final List<String> log = <String>[];
        final MemoryFileSystem fs = MemoryFileSystem();
        final FakePlatform platform = FakePlatform(
          environment: <String, String>{'FLUTTER_ROOT': _kFlutterRoot},
          operatingSystem: 'macos',
        );
        fs.directory(_kFlutterRoot).createSync(recursive: true);
        final Directory basedir = fs.directory('flutter/test/library/')
          ..createSync(recursive: true);
        final FakeSkiaGoldClient fakeSkiaClient = FakeSkiaGoldClient();
        final FlutterGoldenFileComparator comparator = FlutterPreSubmitFileComparator(
          basedir.uri,
          fakeSkiaClient,
          fs: fs,
          platform: platform,
          log: log.add,
        );
        expect(fakeSkiaClient.tryInitCalls, 0);
        comparator.compare(
          Uint8List.fromList(_kTestPngBytes),
          Uri.parse('flutter.golden_test.1.png'),
        );
        expect(fakeSkiaClient.tryInitCalls, 1);
        expect(log, isEmpty);
      });

      test('does not call init in during construction', () {
        final MemoryFileSystem fs = MemoryFileSystem();
        final FakePlatform platform = FakePlatform(
          environment: <String, String>{'FLUTTER_ROOT': _kFlutterRoot},
          operatingSystem: 'macos',
        );
        fs.directory(_kFlutterRoot).createSync(recursive: true);
        final FakeSkiaGoldClient fakeSkiaClient = FakeSkiaGoldClient();
        expect(fakeSkiaClient.tryInitCalls, 0);
        FlutterPostSubmitFileComparator.fromLocalFileComparator(
          localFileComparator: LocalFileComparator(Uri.parse('/test'), pathStyle: path.Style.posix),
          platform: platform,
          goldens: fakeSkiaClient,
          log: (String message) => fail('skia gold client printed unexpected output: "$message"'),
          fs: fs,
          process: FakeProcessManager(),
          httpClient: FakeHttpClient(),
        );
        expect(fakeSkiaClient.tryInitCalls, 0);
      });
    });

    group('Local', () {
      test('asserts .png format', () async {
        final List<String> log = <String>[];
        final MemoryFileSystem fs = MemoryFileSystem();
        fs.directory(_kFlutterRoot).createSync(recursive: true);
        final Directory basedir = fs.directory('flutter/test/library/')
          ..createSync(recursive: true);
        final FakeSkiaGoldClient fakeSkiaClient = FakeSkiaGoldClient();
        final FlutterGoldenFileComparator comparator = FlutterLocalFileComparator(
          basedir.uri,
          fakeSkiaClient,
          fs: fs,
          platform: FakePlatform(
            environment: <String, String>{'FLUTTER_ROOT': _kFlutterRoot},
            operatingSystem: 'macos',
          ),
          log: log.add,
        );
        const String hash = '55109a4bed52acc780530f7a9aeff6c0';
        fakeSkiaClient.expectationForTestValues['flutter.golden_test.1'] = hash;
        fakeSkiaClient.imageBytesValues[hash] = _kTestPngBytes;
        fakeSkiaClient.cleanTestNameValues['library.flutter.golden_test.1.png'] =
            'flutter.golden_test.1';
        await expectLater(
          () async {
            return comparator.compare(
              Uint8List.fromList(_kTestPngBytes),
              Uri.parse('flutter.golden_test.1'),
            );
          },
          throwsA(
            isA<AssertionError>().having(
              (AssertionError error) => error.toString(),
              'description',
              contains(
                'Golden files in the Flutter framework must end with the file '
                'extension .png.',
              ),
            ),
          ),
        );
        expect(log, isEmpty);
      });

      test('passes when bytes match', () async {
        final List<String> log = <String>[];
        final MemoryFileSystem fs = MemoryFileSystem();
        fs.directory(_kFlutterRoot).createSync(recursive: true);
        final Directory basedir = fs.directory('flutter/test/library/')
          ..createSync(recursive: true);
        final FakeSkiaGoldClient fakeSkiaClient = FakeSkiaGoldClient();
        final FlutterGoldenFileComparator comparator = FlutterLocalFileComparator(
          basedir.uri,
          fakeSkiaClient,
          fs: fs,
          platform: FakePlatform(
            environment: <String, String>{'FLUTTER_ROOT': _kFlutterRoot},
            operatingSystem: 'macos',
          ),
          log: log.add,
        );
        const String hash = '55109a4bed52acc780530f7a9aeff6c0';
        fakeSkiaClient.expectationForTestValues['flutter.golden_test.1'] = hash;
        fakeSkiaClient.imageBytesValues[hash] = _kTestPngBytes;
        fakeSkiaClient.cleanTestNameValues['library.flutter.golden_test.1.png'] =
            'flutter.golden_test.1';
        expect(
          await comparator.compare(
            Uint8List.fromList(_kTestPngBytes),
            Uri.parse('flutter.golden_test.1.png'),
          ),
          isTrue,
        );
        expect(log, isEmpty);
      });

      test(
        'returns FlutterSkippingGoldenFileComparator when network connection is unavailable',
        () async {
          final MemoryFileSystem fs = MemoryFileSystem();
          final FakePlatform platform = FakePlatform(
            environment: <String, String>{'FLUTTER_ROOT': _kFlutterRoot},
            operatingSystem: 'macos',
          );
          fs.directory(_kFlutterRoot).createSync(recursive: true);
          final FakeSkiaGoldClient fakeSkiaClient = FakeSkiaGoldClient();

          const String hash = '55109a4bed52acc780530f7a9aeff6c0';
          fakeSkiaClient.expectationForTestValues['flutter.golden_test.1'] = hash;
          fakeSkiaClient.imageBytesValues[hash] = _kTestPngBytes;
          fakeSkiaClient.cleanTestNameValues['library.flutter.golden_test.1.png'] =
              'flutter.golden_test.1';
          final FakeDirectory fakeDirectory = FakeDirectory();
          fakeDirectory.existsSyncValue = true;
          fakeDirectory.uri = Uri.parse('/flutter');

          fakeSkiaClient.getExpectationForTestThrowable = const OSError("Can't reach Gold");
          final FlutterGoldenFileComparator
          comparator1 = await FlutterLocalFileComparator.fromLocalFileComparator(
            localFileComparator: LocalFileComparator(
              Uri.parse('/test'),
              pathStyle: path.Style.posix,
            ),
            platform: platform,
            goldens: fakeSkiaClient,
            baseDirectory: fakeDirectory,
            log: (String message) => fail('skia gold client printed unexpected output: "$message"'),
            fs: fs,
            process: FakeProcessManager(),
            httpClient: FakeHttpClient(),
          );
          expect(comparator1.runtimeType, FlutterSkippingFileComparator);

          fakeSkiaClient.getExpectationForTestThrowable = const SocketException("Can't reach Gold");
          final FlutterGoldenFileComparator
          comparator2 = await FlutterLocalFileComparator.fromLocalFileComparator(
            localFileComparator: LocalFileComparator(
              Uri.parse('/test'),
              pathStyle: path.Style.posix,
            ),
            platform: platform,
            goldens: fakeSkiaClient,
            baseDirectory: fakeDirectory,
            log: (String message) => fail('skia gold client printed unexpected output: "$message"'),
            fs: fs,
            process: FakeProcessManager(),
            httpClient: FakeHttpClient(),
          );
          expect(comparator2.runtimeType, FlutterSkippingFileComparator);

          fakeSkiaClient.getExpectationForTestThrowable = const FormatException("Can't reach Gold");
          final FlutterGoldenFileComparator
          comparator3 = await FlutterLocalFileComparator.fromLocalFileComparator(
            localFileComparator: LocalFileComparator(
              Uri.parse('/test'),
              pathStyle: path.Style.posix,
            ),
            platform: platform,
            goldens: fakeSkiaClient,
            baseDirectory: fakeDirectory,
            log: (String message) => fail('skia gold client printed unexpected output: "$message"'),
            fs: fs,
            process: FakeProcessManager(),
            httpClient: FakeHttpClient(),
          );
          expect(comparator3.runtimeType, FlutterSkippingFileComparator);

          // reset property or it will carry on to other tests
          fakeSkiaClient.getExpectationForTestThrowable = null;
        },
      );
    });
  });
}

@immutable
class RunInvocation {
  const RunInvocation(this.command, this.workingDirectory);

  final List<String> command;
  final String? workingDirectory;

  @override
  int get hashCode => Object.hash(Object.hashAll(command), workingDirectory);

  bool _commandEquals(List<String> other) {
    if (other == command) {
      return true;
    }
    if (other.length != command.length) {
      return false;
    }
    for (int index = 0; index < other.length; index += 1) {
      if (other[index] != command[index]) {
        return false;
      }
    }
    return true;
  }

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is RunInvocation &&
        _commandEquals(other.command) &&
        other.workingDirectory == workingDirectory;
  }

  @override
  String toString() => '$command ($workingDirectory)';
}

class FakeProcessManager extends Fake implements ProcessManager {
  Map<RunInvocation, ProcessResult> processResults = <RunInvocation, ProcessResult>{};

  /// Used if [processResults] does not contain a matching invocation.
  ProcessResult? fallbackProcessResult;

  final List<String?> workingDirectories = <String?>[];

  @override
  Future<ProcessResult> run(
    List<Object> command, {
    String? workingDirectory,
    Map<String, String>? environment,
    bool includeParentEnvironment = true,
    bool runInShell = false,
    Encoding? stdoutEncoding = systemEncoding,
    Encoding? stderrEncoding = systemEncoding,
  }) async {
    workingDirectories.add(workingDirectory);
    final ProcessResult? result =
        processResults[RunInvocation(command.cast<String>(), workingDirectory)];
    if (result == null && fallbackProcessResult == null) {
      printOnFailure(
        'ProcessManager.run was called with $command ($workingDirectory) unexpectedly - $processResults.',
      );
      fail('See above.');
    }
    return result ?? fallbackProcessResult!;
  }
}

// See also dev/automated_tests/flutter_test/flutter_gold_test.dart
class FakeSkiaGoldClient extends Fake implements SkiaGoldClient {
  Map<String, String> expectationForTestValues = <String, String>{};
  Exception? getExpectationForTestThrowable;
  @override
  Future<String> getExpectationForTest(String testName) async {
    if (getExpectationForTestThrowable != null) {
      throw getExpectationForTestThrowable!;
    }
    return expectationForTestValues[testName] ?? '';
  }

  @override
  Future<void> auth() async {}

  final List<String> testNames = <String>[];

  int initCalls = 0;
  @override
  Future<void> imgtestInit() async => initCalls += 1;
  @override
  Future<bool> imgtestAdd(String testName, File goldenFile) async {
    testNames.add(testName);
    return true;
  }

  int tryInitCalls = 0;
  @override
  Future<void> tryjobInit() async => tryInitCalls += 1;
  @override
  Future<String?> tryjobAdd(String testName, File goldenFile) async => null;

  Map<String, List<int>> imageBytesValues = <String, List<int>>{};
  @override
  Future<List<int>> getImageBytes(String imageHash) async => imageBytesValues[imageHash]!;

  Map<String, String> cleanTestNameValues = <String, String>{};
  @override
  String cleanTestName(String fileName) => cleanTestNameValues[fileName] ?? '';
}

class ThrowsOnImgTestAddSkiaClient extends Fake implements SkiaGoldClient {
  ThrowsOnImgTestAddSkiaClient({required this.message});
  final String message;

  @override
  Future<void> imgtestInit() async {
    // Assume this function works.
  }

  @override
  Future<bool> imgtestAdd(String testName, File goldenFile) {
    throw SkiaException(message);
  }
}

class FakeLocalFileComparator extends Fake implements LocalFileComparator {
  @override
  late Uri basedir;
}

class FakeDirectory extends Fake implements Directory {
  late bool existsSyncValue;
  @override
  bool existsSync() => existsSyncValue;

  @override
  late Uri uri;
}

class FakeHttpClient extends Fake implements HttpClient {
  late Uri lastUri;
  late FakeHttpClientRequest request;

  @override
  Future<HttpClientRequest> getUrl(Uri url) async {
    lastUri = url;
    return request;
  }
}

class FakeHttpClientRequest extends Fake implements HttpClientRequest {
  late FakeHttpImageResponse response;

  @override
  Future<HttpClientResponse> close() async {
    return response;
  }
}

class FakeHttpImageResponse extends Fake implements HttpClientResponse {
  FakeHttpImageResponse(this.response);

  final List<List<int>> response;

  @override
  Future<void> forEach(void Function(List<int> element) action) async {
    response.forEach(action);
  }
}
