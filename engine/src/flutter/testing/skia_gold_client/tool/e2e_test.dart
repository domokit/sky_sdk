// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:path/path.dart' as path;
import 'package:skia_gold_client/skia_gold_client.dart';

/// An E2E test for the Skia Gold client.
///
/// Attempts to answer the question: "Does the Skia Gold client, and our custom
/// integration with it (GitHub, LUCI, etc), work as expected?" in an automated
/// way.
///
/// In a sibling directory, `e2e_fixtures`, there are static (checked-in) files
/// that represent (fake) generated output to be uploaded and verified by the
/// Skia Gold client. This test, when run on CI, will use these fixtures to
/// simulate the process of uploading and verifying real output.
///
/// For example, try changing the contents of a fixture file, and then uploading
/// a PR with this change. The CI will run this test, and it should fail (on
/// pre-submit), since the fixture file no longer matches the expected output.
///
/// Next, after the PR is merged, the CI will run this test again, and it should
/// pass (on post-submit), since the fixture file now matches the expected
/// output.
///
/// There are also tests for the "dimensions" feature of the Skia Gold client,
/// which live in `e2e_fixtures/dimensions`. These tests are similar to the
/// regular tests, but experiment with different fake dimensions to ensure that
/// our CI environment is correctly handling this feature.
void main() async {
  // If the client is not available, we can't run the test.
  if (!SkiaGoldClient.isAvailable()) {
    stderr.writeln('Skia gold is unavailable in this environment.');
    exitCode = 1;
    return;
  }

  // If we're not in an engine repo, we can't run the test.
  final Engine? engine = Engine.tryFindWithin();
  if (engine == null) {
    stderr.writeln('Must run within the engine repo.');
    exitCode = 1;
    return;
  }

  // Create a client.
  final SkiaGoldClient skiaGoldClient = SkiaGoldClient(engine.flutterDir);

  // Authenticate the client.
  await skiaGoldClient.auth();

  const String prefix = 'SkiaGoldClientE2ETest';
  const List<_Digest> digests = <_Digest>[
    _Digest(
      name: '${prefix}_SolidBlueSquare',
      source: 'e2e_fixtures/solid_blue_square.png',
      pixelCount: 512 * 512,
    ),
    _Digest(
      name: '${prefix}_SolidRedSquare',
      source: 'e2e_fixtures/solid_red_square.png',
      pixelCount: 768 * 768,
    ),
    _Digest(
      name: '${prefix}_SolidGreenSquare',
      source: 'e2e_fixtures/solid_green_square.png',
      pixelCount: 1200 * 1200,
    ),
  ];

  // Upload the digests to Skia Gold.
  final Set<_Digest> comparisonsFailed = <_Digest>{};
  for (final _Digest digest in digests) {
    final String digestPath = digest.source;
    final String digestName = digest.name;
    final File digestFile = File(
      path.join(engine.flutterDir.path, 'testing', 'skia_gold_client', 'tool', digestPath),
    );
    if (!digestFile.existsSync()) {
      stderr.writeln('The digest file "$digestPath" does not exist.');
      exitCode = 1;
      return;
    }

    print('Uploading digest: $digestName ($digestPath): ${digest.pixelCount} pixels...');

    try {
      await skiaGoldClient.addImg(digestName, digestFile, screenshotSize: 0);
      stderr.writeln('Comparison success: $digestName');
    } on Exception catch (e) {
      stderr.writeln('Comparison failure: $digestName: $e');
      comparisonsFailed.add(digest);
    }
  }

  if (comparisonsFailed.isNotEmpty) {
    stdout.writeln('${comparisonsFailed.length} digest(s) failed.');
    for (final _Digest digest in comparisonsFailed) {
      stderr.writeln('  ${digest.name} (${digest.source})');
    }
    exitCode = 1;
    return;
  }
}

final class _Digest {
  const _Digest({required this.name, required this.source, required this.pixelCount});

  /// The name of the digest/test.
  final String name;

  /// The source of the digest (e.g. the path to the image file).
  final String source;

  /// The number of pixels in the image.
  final int pixelCount;

  @override
  int get hashCode => source.hashCode;

  @override
  bool operator ==(Object other) {
    return other is _Digest && other.source == source;
  }
}
