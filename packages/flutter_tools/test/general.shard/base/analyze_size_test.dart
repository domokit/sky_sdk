// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:archive/archive.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/analyze_size.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../../src/common.dart';

const String aotSizeOutput = '''
[
    {
        "l": "dart:_internal",
        "c": "SubListIterable",
        "n": "[Optimized] skip",
        "s": 2400
    },
    {
        "l": "dart:_internal",
        "c": "SubListIterable",
        "n": "[Optimized] new SubListIterable.",
        "s": 3560
    },
    {
        "l": "dart:core",
        "c": "RangeError",
        "n": "[Optimized] new RangeError.range",
        "s": 3920
    },
    {
        "l": "dart:core",
        "c": "ArgumentError",
        "n": "[Stub] Allocate ArgumentError",
        "s": 4650
    }
]
''';

void main() {
  late MemoryFileSystem fileSystem;
  late BufferLogger logger;

  setUp(() {
    fileSystem = MemoryFileSystem.test();
    logger = BufferLogger.test();
  });

  testWithoutContext('matchesPattern matches only entire strings', () {
    expect(matchesPattern('', pattern: ''), isNotNull);
    expect(matchesPattern('', pattern: 'foo'), null);
    expect(matchesPattern('foo', pattern: ''), null);
    expect(matchesPattern('foo', pattern: 'foo'), isNotNull);
    expect(matchesPattern('foo', pattern: 'foobar'), null);
    expect(matchesPattern('foobar', pattern: 'foo'), null);
    expect(matchesPattern('foobar', pattern: RegExp(r'.*b.*')), isNotNull);
    expect(matchesPattern('foobar', pattern: RegExp(r'.*b')), null);
  });

  testWithoutContext('builds APK analysis correctly', () async {
    final SizeAnalyzer sizeAnalyzer = SizeAnalyzer(
      fileSystem: fileSystem,
      logger: logger,
      appFilenamePattern: RegExp(r'lib.*app\.so'),
      analytics: const NoOpAnalytics(),
    );

    final Archive archive =
        Archive()
          ..addFile(ArchiveFile('AndroidManifest.xml', 100, List<int>.filled(100, 0)))
          ..addFile(ArchiveFile('META-INF/CERT.RSA', 10, List<int>.filled(10, 0)))
          ..addFile(ArchiveFile('META-INF/CERT.SF', 10, List<int>.filled(10, 0)))
          ..addFile(ArchiveFile('lib/arm64-v8a/libxyzzyapp.so', 50, List<int>.filled(50, 0)))
          ..addFile(ArchiveFile('lib/arm64-v8a/libflutter.so', 50, List<int>.filled(50, 0)));

    final File apk = fileSystem.file('test.apk')..writeAsBytesSync(ZipEncoder().encode(archive)!);
    final File aotSizeJson =
        fileSystem.file('test.json')
          ..createSync()
          ..writeAsStringSync(aotSizeOutput);
    final File precompilerTrace = fileSystem.file('trace.json')..writeAsStringSync('{}');
    final Map<String, dynamic> result = await sizeAnalyzer.analyzeZipSizeAndAotSnapshot(
      zipFile: apk,
      aotSnapshot: aotSizeJson,
      precompilerTrace: precompilerTrace,
      kind: 'apk',
    );

    expect(result['type'], 'apk');
    final List<dynamic> resultChildren = result['children'] as List<dynamic>;
    final Map<String, dynamic> androidManifestMap = resultChildren[0] as Map<String, dynamic>;
    expect(androidManifestMap['n'], 'AndroidManifest.xml');
    expect(androidManifestMap['value'], 6);

    final Map<String, Object?> metaInfMap = resultChildren[1] as Map<String, Object?>;
    final List<Map<String, Object?>> metaInfMapChildren =
        metaInfMap['children']! as List<Map<String, Object?>>;
    expect(metaInfMap['n'], 'META-INF');
    expect(metaInfMap['value'], 10);
    final Map<String, dynamic> certRsaMap = metaInfMapChildren[0];
    expect(certRsaMap['n'], 'CERT.RSA');
    expect(certRsaMap['value'], 5);
    final Map<String, dynamic> certSfMap = metaInfMapChildren[1];
    expect(certSfMap['n'], 'CERT.SF');
    expect(certSfMap['value'], 5);

    final Map<String, Object?> libMap = resultChildren[2] as Map<String, Object?>;
    final List<Map<String, Object?>> libMapChildren =
        libMap['children']! as List<Map<String, Object?>>;
    expect(libMap['n'], 'lib');
    expect(libMap['value'], 12);
    final Map<String, Object?> arm64Map = libMapChildren[0];
    final List<Map<String, Object?>> arn64MapChildren =
        arm64Map['children']! as List<Map<String, Object?>>;
    expect(arm64Map['n'], 'arm64-v8a');
    expect(arm64Map['value'], 12);
    final Map<String, Object?> libAppMap = arn64MapChildren[0];
    final List<dynamic> libAppMapChildren = libAppMap['children']! as List<dynamic>;
    expect(libAppMap['n'], 'libxyzzyapp.so (Dart AOT)');
    expect(libAppMap['value'], 6);
    expect(libAppMapChildren.length, 3);
    final Map<String, Object?> internalMap = libAppMapChildren[0] as Map<String, Object?>;
    final List<dynamic> internalMapChildren = internalMap['children']! as List<dynamic>;
    final Map<String, Object?> skipMap = internalMapChildren[0] as Map<String, Object?>;
    expect(skipMap['n'], 'skip');
    expect(skipMap['value'], 2400);
    final Map<String, Object?> subListIterableMap = internalMapChildren[1] as Map<String, Object?>;
    expect(subListIterableMap['n'], 'new SubListIterable.');
    expect(subListIterableMap['value'], 3560);
    final Map<String, Object?> coreMap = libAppMapChildren[1] as Map<String, Object?>;
    final List<dynamic> coreMapChildren = coreMap['children']! as List<dynamic>;
    final Map<String, Object?> rangeErrorMap = coreMapChildren[0] as Map<String, Object?>;
    expect(rangeErrorMap['n'], 'new RangeError.range');
    expect(rangeErrorMap['value'], 3920);
    final Map<String, Object?> stubsMap = libAppMapChildren[2] as Map<String, Object?>;
    final List<dynamic> stubsMapChildren = stubsMap['children']! as List<dynamic>;
    final Map<String, Object?> allocateMap = stubsMapChildren[0] as Map<String, Object?>;
    expect(allocateMap['n'], 'Allocate ArgumentError');
    expect(allocateMap['value'], 4650);
    final Map<String, Object?> libFlutterMap = arn64MapChildren[1];
    expect(libFlutterMap['n'], 'libflutter.so (Flutter Engine)');
    expect(libFlutterMap['value'], 6);

    expect(result['precompiler-trace'], <String, Object>{});
  });

  testWithoutContext('outputs summary to command line correctly', () async {
    final SizeAnalyzer sizeAnalyzer = SizeAnalyzer(
      fileSystem: fileSystem,
      logger: logger,
      appFilenamePattern: RegExp(r'lib.*app\.so'),
      analytics: const NoOpAnalytics(),
    );

    final Archive archive =
        Archive()
          ..addFile(ArchiveFile('AndroidManifest.xml', 100, List<int>.filled(100, 0)))
          ..addFile(ArchiveFile('META-INF/CERT.RSA', 10, List<int>.filled(10, 0)))
          ..addFile(ArchiveFile('META-INF/CERT.SF', 10, List<int>.filled(10, 0)))
          ..addFile(ArchiveFile('lib/arm64-v8a/libxyzzyapp.so', 50, List<int>.filled(50, 0)))
          ..addFile(ArchiveFile('lib/arm64-v8a/libflutter.so', 50, List<int>.filled(50, 0)));

    final File apk = fileSystem.file('test.apk')..writeAsBytesSync(ZipEncoder().encode(archive)!);
    final File aotSizeJson = fileSystem.file('test.json')..writeAsStringSync(aotSizeOutput);
    final File precompilerTrace = fileSystem.file('trace.json')..writeAsStringSync('{}');
    await sizeAnalyzer.analyzeZipSizeAndAotSnapshot(
      zipFile: apk,
      aotSnapshot: aotSizeJson,
      precompilerTrace: precompilerTrace,
      kind: 'apk',
    );

    final List<String> stdout = logger.statusText.split('\n');
    expect(
      stdout,
      containsAll(<String>[
        'test.apk (total compressed)                                                644 B',
        '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━',
        '  lib                                                                       12 B',
        '  Dart AOT symbols accounted decompressed size                             14 KB',
        '    dart:core/',
        '      RangeError                                                            4 KB',
      ]),
    );
  });

  testWithoutContext('can analyze contents of output directory', () async {
    final SizeAnalyzer sizeAnalyzer = SizeAnalyzer(
      fileSystem: fileSystem,
      logger: logger,
      appFilenamePattern: RegExp(r'lib.*app\.so'),
      analytics: const NoOpAnalytics(),
    );

    final Directory outputDirectory = fileSystem.directory('example/out/foo.app')
      ..createSync(recursive: true);
    outputDirectory.childFile('a.txt')
      ..createSync()
      ..writeAsStringSync('hello');
    outputDirectory.childFile('libapp.so')
      ..createSync()
      ..writeAsStringSync('goodbye');
    final File aotSizeJson = fileSystem.file('test.json')..writeAsStringSync(aotSizeOutput);
    final File precompilerTrace = fileSystem.file('trace.json')..writeAsStringSync('{}');

    final Map<String, Object?> result = await sizeAnalyzer.analyzeAotSnapshot(
      outputDirectory: outputDirectory,
      aotSnapshot: aotSizeJson,
      precompilerTrace: precompilerTrace,
      type: 'linux',
    );

    final List<String> stdout = logger.statusText.split('\n');
    expect(
      stdout,
      containsAll(<String>[
        '  foo.app                                                                   12 B',
        '  foo.app                                                                   12 B',
        '  Dart AOT symbols accounted decompressed size                             14 KB',
        '    dart:core/',
        '      RangeError                                                            4 KB',
      ]),
    );
    expect(result['type'], 'linux');
    expect(result['precompiler-trace'], <String, Object>{});
  });

  testWithoutContext('handles null AOT snapshot json', () async {
    final SizeAnalyzer sizeAnalyzer = SizeAnalyzer(
      fileSystem: fileSystem,
      logger: logger,
      appFilenamePattern: RegExp(r'lib.*app\.so'),
      analytics: const NoOpAnalytics(),
    );

    final Directory outputDirectory = fileSystem.directory('example/out/foo.app')
      ..createSync(recursive: true);
    final File invalidAotSizeJson = fileSystem.file('test.json')..writeAsStringSync('null');
    final File precompilerTrace = fileSystem.file('trace.json');

    await expectLater(
      () => sizeAnalyzer.analyzeAotSnapshot(
        outputDirectory: outputDirectory,
        aotSnapshot: invalidAotSizeJson,
        precompilerTrace: precompilerTrace,
        type: 'linux',
      ),
      throwsToolExit(),
    );

    final File apk = fileSystem.file('test.apk')..writeAsBytesSync(ZipEncoder().encode(Archive())!);
    await expectLater(
      () => sizeAnalyzer.analyzeZipSizeAndAotSnapshot(
        zipFile: apk,
        aotSnapshot: invalidAotSizeJson,
        precompilerTrace: precompilerTrace,
        kind: 'apk',
      ),
      throwsToolExit(),
    );
  });
}
