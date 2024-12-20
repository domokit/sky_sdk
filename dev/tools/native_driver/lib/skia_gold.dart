// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// A fork of `package:flutter_goldens/flutter_goldens.dart` without the
/// dependency on `package:flutter_test` or `package:flutter`; this allows
/// the library to be used in a standalone Dart VM context.
library;

import 'dart:async';
import 'dart:io' as io;
import 'dart:typed_data';

import 'package:file/local.dart';
import 'package:flutter_goldens/skia_client.dart';
import 'package:path/path.dart' as path;
import 'package:platform/platform.dart';
import 'package:process/process.dart';

import 'native_driver.dart';

const LocalFileSystem _localFs = LocalFileSystem();
const String _kGoldctlKey = 'GOLDCTL';
const String _kGoldctlPresubmitKey = 'GOLD_TRYJOB';

/// Configures [goldenFileComparator] to use Skia Gold (i.e. on CI).
///
/// Requires that the `GOLDCTL` environment variable is set.
///
/// If the `GOLD_TRYJOB` environment variable is set, the test will be run in
/// presubmit mode; that is, the test will not fail if the comparison fails.
/// See <https://github.com/flutter/flutter/blob/main/docs/contributing/testing/Writing-a-golden-file-test-for-package-flutter.md>
/// for more information.
///
/// May optionally provide a [namePrefix] to be used when uploading images.
Future<void> enableSkiaGoldComparator({String? namePrefix}) async {
  assert(
    goldenFileComparator is NaiveLocalFileComparator,
    'The flutter_goldens_fork library should be used from a *_test.dart file '
    'where the "goldenFileComparator" has not yet been set. This is to ensure '
    'that the correct comparator is used for the current test environment.',
  );
  if (!io.Platform.environment.containsKey(_kGoldctlKey)) {
    throw StateError(
      'Environment variable $_kGoldctlKey is not set. '
      'Set it to use Skia Gold.',
    );
  }
  final io.Directory tmpDir = io.Directory.systemTemp.createTempSync('android_driver_test');
  final bool isPresubmit = io.Platform.environment.containsKey(_kGoldctlPresubmitKey);
  io.stderr.writeln(
    '=== Using Skia Gold ===\n'
    'Environment variable $_kGoldctlKey is set, using Skia Gold: \n'
    '  - tmpDir:      ${tmpDir.path}\n'
    '  - namePrefix:  $namePrefix\n'
    '  - isPresubmit: $isPresubmit\n',
  );
  final SkiaGoldClient skiaGoldClient = SkiaGoldClient(
    _localFs.directory(tmpDir.path),
    fs: _localFs,
    process: const LocalProcessManager(),
    platform: const LocalPlatform(),
    httpClient: io.HttpClient(),
    log: io.stderr.writeln,
  );
  await skiaGoldClient.auth();
  goldenFileComparator = _GoldenFileComparator(
    skiaGoldClient,
    namePrefix: namePrefix,
    isPresubmit: isPresubmit,
  );
}

final class _GoldenFileComparator extends GoldenFileComparator {
  _GoldenFileComparator(this.skiaClient, {required this.isPresubmit, this.namePrefix, Uri? baseDir})
    : baseDir = baseDir ?? Uri.parse(path.dirname(io.Platform.script.path));

  final Uri baseDir;
  final SkiaGoldClient skiaClient;
  final String? namePrefix;
  final bool isPresubmit;

  @override
  Future<bool> compare(Uint8List imageBytes, Uri golden) async {
    if (isPresubmit) {
      await skiaClient.tryjobInit();
    } else {
      await skiaClient.imgtestInit();
    }

    golden = _addPrefix(golden);
    final io.File goldenFile = await update(golden, imageBytes);
    if (isPresubmit) {
      final String? result = await skiaClient.tryjobAdd(
        golden.path,
        _localFs.file(goldenFile.path),
      );
      if (result != null) {
        io.stderr.writeln('Skia Gold detected an error when comparing "$golden":\n\n$result');
        io.stderr.writeln('Still succeeding, will be triaged in Flutter Gold');
      } else {
        io.stderr.writeln('Skia Gold comparison succeeded comparing "$golden".');
      }
      return true;
    } else {
      return skiaClient.imgtestAdd(golden.path, _localFs.file(goldenFile.path));
    }
  }

  @override
  Future<io.File> update(Uri golden, Uint8List imageBytes) async {
    io.stderr.writeln('Updating golden file: $golden (${imageBytes.length} bytes)...');
    final io.File goldenFile = _getGoldenFile(golden);
    await goldenFile.parent.create(recursive: true);
    await goldenFile.writeAsBytes(imageBytes, flush: true);
    return goldenFile;
  }

  io.File _getGoldenFile(Uri uri) {
    return io.File.fromUri(baseDir.resolveUri(uri));
  }

  Uri _addPrefix(Uri golden) {
    assert(
      golden.toString().split('.').last == 'png',
      'Golden files in the Flutter framework must end with the file extension '
      '.png.',
    );
    return Uri.parse(
      <String>[
        if (namePrefix != null) namePrefix!,
        baseDir.pathSegments[baseDir.pathSegments.length - 2],
        golden.toString(),
      ].join('.'),
    );
  }
}
