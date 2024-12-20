// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:path/path.dart' as path;

import '../analyze.dart';
import '../custom_rules/analyze.dart';
import '../custom_rules/no_double_clamp.dart';
import '../custom_rules/no_stop_watches.dart';
import '../custom_rules/render_box_intrinsics.dart';
import '../utils.dart';
import 'common.dart';

typedef AsyncVoidCallback = Future<void> Function();

Future<String> capture(AsyncVoidCallback callback, {bool shouldHaveErrors = false}) async {
  final StringBuffer buffer = StringBuffer();
  final PrintCallback oldPrint = print;
  try {
    print = (Object? line) {
      buffer.writeln(line);
    };
    await callback();
    expect(
      hasError,
      shouldHaveErrors,
      reason:
          buffer.isEmpty
              ? '(No output to report.)'
              : hasError
              ? 'Unexpected errors:\n$buffer'
              : 'Unexpected success:\n$buffer',
    );
  } finally {
    print = oldPrint;
    resetErrorStatus();
  }
  if (stdout.supportsAnsiEscapes) {
    // Remove ANSI escapes when this test is running on a terminal.
    return buffer.toString().replaceAll(RegExp(r'(\x9B|\x1B\[)[0-?]{1,3}[ -/]*[@-~]'), '');
  } else {
    return buffer.toString();
  }
}

void main() {
  final String testRootPath = path.join('test', 'analyze-test-input', 'root');
  final String dartName = Platform.isWindows ? 'dart.exe' : 'dart';
  final String dartPath = path.canonicalize(
    path.join('..', '..', 'bin', 'cache', 'dart-sdk', 'bin', dartName),
  );
  final String testGenDefaultsPath = path.join('test', 'analyze-gen-defaults');

  test(
    'analyze.dart - verifyDeprecations',
    () async {
      final String result = await capture(
        () => verifyDeprecations(testRootPath, minimumMatches: 2),
        shouldHaveErrors: true,
      );
      final String lines = <String>[
            '║ test/analyze-test-input/root/packages/foo/deprecation.dart:14: Deprecation notice does not match required pattern. There might be a missing space character at the end of the line.',
            '║ test/analyze-test-input/root/packages/foo/deprecation.dart:20: Deprecation notice should be a grammatically correct sentence and start with a capital letter; see style guide: STYLE_GUIDE_URL',
            '║ test/analyze-test-input/root/packages/foo/deprecation.dart:27: Deprecation notice should be a grammatically correct sentence and end with a period; notice appears to be "Also bad grammar".',
            '║ test/analyze-test-input/root/packages/foo/deprecation.dart:31: Deprecation notice does not match required pattern.',
            '║ test/analyze-test-input/root/packages/foo/deprecation.dart:34: Deprecation notice does not match required pattern.',
            '║ test/analyze-test-input/root/packages/foo/deprecation.dart:39: Deprecation notice does not match required pattern. It might be missing the line saying "This feature was deprecated after...".',
            '║ test/analyze-test-input/root/packages/foo/deprecation.dart:43: Deprecation notice does not match required pattern. There might not be an explanatory message.',
            '║ test/analyze-test-input/root/packages/foo/deprecation.dart:50: End of deprecation notice does not match required pattern.',
            '║ test/analyze-test-input/root/packages/foo/deprecation.dart:53: Unexpected deprecation notice indent.',
            '║ test/analyze-test-input/root/packages/foo/deprecation.dart:72: Deprecation notice does not accurately indicate a beta branch version number; please see RELEASES_URL to find the latest beta build version number.',
            '║ test/analyze-test-input/root/packages/foo/deprecation.dart:78: Deprecation notice does not accurately indicate a beta branch version number; please see RELEASES_URL to find the latest beta build version number.',
            '║ test/analyze-test-input/root/packages/foo/deprecation.dart:101: Deprecation notice does not match required pattern. You might have used double quotes (") for the string instead of single quotes (\').',
          ]
          .map((String line) {
            return line
                .replaceAll('/', Platform.isWindows ? r'\' : '/')
                .replaceAll(
                  'STYLE_GUIDE_URL',
                  'https://github.com/flutter/flutter/blob/main/docs/contributing/Style-guide-for-Flutter-repo.md',
                )
                .replaceAll(
                  'RELEASES_URL',
                  'https://flutter.dev/docs/development/tools/sdk/releases',
                );
          })
          .join('\n');
      expect(
        result,
        '╔═╡ERROR #1╞════════════════════════════════════════════════════════════════════\n'
        '$lines\n'
        '║ See: https://github.com/flutter/flutter/blob/main/docs/contributing/Tree-hygiene.md#handling-breaking-changes\n'
        '╚═══════════════════════════════════════════════════════════════════════════════\n',
      );
    },
    // TODO(goderbauer): Update and re-enable this after formatting changes have landed.
    skip: true,
  );

  test('analyze.dart - verifyGoldenTags', () async {
    final List<String> result = (await capture(
      () => verifyGoldenTags(testRootPath, minimumMatches: 6),
      shouldHaveErrors: true,
    )).split('\n');
    const String noTag =
        "Files containing golden tests must be tagged using @Tags(<String>['reduced-test-set']) "
        'at the top of the file before import statements.';
    const String missingTag =
        "Files containing golden tests must be tagged with 'reduced-test-set'.";
    final List<String> lines =
        <String>[
          '║ test/analyze-test-input/root/packages/foo/golden_missing_tag.dart: $missingTag',
          '║ test/analyze-test-input/root/packages/foo/golden_no_tag.dart: $noTag',
        ].map((String line) => line.replaceAll('/', Platform.isWindows ? r'\' : '/')).toList();
    expect(
      result.length,
      4 + lines.length,
      reason: 'output had unexpected number of lines:\n${result.join('\n')}',
    );
    expect(
      result[0],
      '╔═╡ERROR #1╞════════════════════════════════════════════════════════════════════',
    );
    expect(result.getRange(1, result.length - 3).toSet(), lines.toSet());
    expect(
      result[result.length - 3],
      '║ See: https://github.com/flutter/flutter/blob/main/docs/contributing/testing/Writing-a-golden-file-test-for-package-flutter.md',
    );
    expect(
      result[result.length - 2],
      '╚═══════════════════════════════════════════════════════════════════════════════',
    );
    expect(result[result.length - 1], ''); // trailing newline
  });

  test('analyze.dart - verifyNoMissingLicense', () async {
    final String result = await capture(
      () => verifyNoMissingLicense(testRootPath, checkMinimums: false),
      shouldHaveErrors: true,
    );
    final String file = 'test/analyze-test-input/root/packages/foo/foo.dart'.replaceAll(
      '/',
      Platform.isWindows ? r'\' : '/',
    );
    expect(
      result,
      '╔═╡ERROR #1╞════════════════════════════════════════════════════════════════════\n'
      '║ The following file does not have the right license header for dart files:\n'
      '║   $file\n'
      '║ The expected license header is:\n'
      '║ // Copyright 2014 The Flutter Authors. All rights reserved.\n'
      '║ // Use of this source code is governed by a BSD-style license that can be\n'
      '║ // found in the LICENSE file.\n'
      '║ ...followed by a blank line.\n'
      '╚═══════════════════════════════════════════════════════════════════════════════\n',
    );
  });

  test('analyze.dart - verifyNoTrailingSpaces', () async {
    final String result = await capture(
      () => verifyNoTrailingSpaces(testRootPath, minimumMatches: 2),
      shouldHaveErrors: true,
    );
    final String lines = <String>[
      '║ test/analyze-test-input/root/packages/foo/spaces.txt:5: trailing U+0020 space character',
      '║ test/analyze-test-input/root/packages/foo/spaces.txt:9: trailing blank line',
    ].map((String line) => line.replaceAll('/', Platform.isWindows ? r'\' : '/')).join('\n');
    expect(
      result,
      '╔═╡ERROR #1╞════════════════════════════════════════════════════════════════════\n'
      '$lines\n'
      '╚═══════════════════════════════════════════════════════════════════════════════\n',
    );
  });

  test('analyze.dart - verifyRepositoryLinks', () async {
    final String result = await capture(
      () => verifyRepositoryLinks(testRootPath),
      shouldHaveErrors: true,
    );
    const String bannedBranch = 'master';
    final String file =
        Platform.isWindows
            ? r'test\analyze-test-input\root\packages\foo\bad_repository_links.dart'
            : 'test/analyze-test-input/root/packages/foo/bad_repository_links.dart';
    final String lines = <String>[
      '║ $file contains https://android.googlesource.com/+/$bannedBranch/file1, which uses the banned "master" branch.',
      '║ $file contains https://chromium.googlesource.com/+/$bannedBranch/file1, which uses the banned "master" branch.',
      '║ $file contains https://cs.opensource.google.com/+/$bannedBranch/file1, which uses the banned "master" branch.',
      '║ $file contains https://dart.googlesource.com/+/$bannedBranch/file1, which uses the banned "master" branch.',
      '║ $file contains https://flutter.googlesource.com/+/$bannedBranch/file1, which uses the banned "master" branch.',
      '║ $file contains https://source.chromium.org/+/$bannedBranch/file1, which uses the banned "master" branch.',
      '║ $file contains https://github.com/flutter/flutter/tree/$bannedBranch/file1, which uses the banned "master" branch.',
      '║ $file contains https://raw.githubusercontent.com/flutter/flutter/blob/$bannedBranch/file1, which uses the banned "master" branch.',
      '║ Change the URLs above to the expected pattern by using the "main" branch if it exists, otherwise adding the repository to the list of exceptions in analyze.dart.',
    ].join('\n');
    expect(
      result,
      '╔═╡ERROR #1╞════════════════════════════════════════════════════════════════════\n'
      '$lines\n'
      '╚═══════════════════════════════════════════════════════════════════════════════\n',
    );
  });

  test('analyze.dart - verifyNoBinaries - positive', () async {
    final String result = await capture(
      () => verifyNoBinaries(
        testRootPath,
        legacyBinaries: <Hash256>{const Hash256(0x39A050CD69434936, 0, 0, 0)},
      ),
      shouldHaveErrors: !Platform.isWindows,
    );
    if (!Platform.isWindows) {
      expect(
        result,
        '╔═╡ERROR #1╞════════════════════════════════════════════════════════════════════\n'
        '║ test/analyze-test-input/root/packages/foo/serviceaccount.enc:0: file is not valid UTF-8\n'
        '║ All files in this repository must be UTF-8. In particular, images and other binaries\n'
        '║ must not be checked into this repository. This is because we are very sensitive to the\n'
        '║ size of the repository as it is distributed to all our developers. If you have a binary\n'
        '║ to which you need access, you should consider how to fetch it from another repository;\n'
        '║ for example, the "assets-for-api-docs" repository is used for images in API docs.\n'
        '║ To add assets to flutter_tools templates, see the instructions in the wiki:\n'
        '║ https://github.com/flutter/flutter/blob/main/docs/tool/Managing-template-image-assets.md\n'
        '╚═══════════════════════════════════════════════════════════════════════════════\n',
      );
    }
  });

  test('analyze.dart - verifyInternationalizations - comparison fails', () async {
    final String result = await capture(
      () => verifyInternationalizations(testRootPath, dartPath),
      shouldHaveErrors: true,
    );
    final String genLocalizationsScript = path.join(
      'dev',
      'tools',
      'localization',
      'bin',
      'gen_localizations.dart',
    );
    expect(result, contains('$dartName $genLocalizationsScript --cupertino'));
    expect(result, contains('$dartName $genLocalizationsScript --material'));
    final String generatedFile = path.join(
      testRootPath,
      'packages',
      'flutter_localizations',
      'lib',
      'src',
      'l10n',
      'generated_material_localizations.dart',
    );
    expect(
      result,
      contains(
        'The contents of $generatedFile are different from that produced by gen_localizations.',
      ),
    );
    expect(
      result,
      contains(r'Did you forget to run gen_localizations.dart after updating a .arb file?'),
    );
  });

  test('analyze.dart - verifyNoBinaries - negative', () async {
    await capture(
      () => verifyNoBinaries(
        testRootPath,
        legacyBinaries: <Hash256>{
          const Hash256(
            0xA8100AE6AA1940D0,
            0xB663BB31CD466142,
            0xEBBDBD5187131B92,
            0xD93818987832EB89,
          ), // sha256("\xff")
          const Hash256(0x155644D3F13D98BF, 0, 0, 0),
        },
      ),
    );
  });

  test('analyze.dart - verifyNullInitializedDebugExpensiveFields', () async {
    final String result = await capture(
      () => verifyNullInitializedDebugExpensiveFields(testRootPath, minimumMatches: 1),
      shouldHaveErrors: true,
    );

    expect(result, contains(':16'));
    expect(result, isNot(contains(':13')));
  });

  test('analyze.dart - verifyTabooDocumentation', () async {
    final String result = await capture(
      () => verifyTabooDocumentation(testRootPath, minimumMatches: 1),
      shouldHaveErrors: true,
    );

    expect(result, isNot(contains(':19')));
    expect(result, contains(':20'));
    expect(result, contains(':21'));
  });

  test('analyze.dart - clampDouble', () async {
    final String result = await capture(
      () => analyzeWithRules(
        testRootPath,
        <AnalyzeRule>[noDoubleClamp],
        includePaths: <String>['packages/flutter/lib'],
      ),
      shouldHaveErrors: true,
    );
    final String lines = <String>[
      '║ packages/flutter/lib/bar.dart:37: input.clamp(0.0, 2)',
      '║ packages/flutter/lib/bar.dart:38: input.toDouble().clamp(0, 2)',
      '║ packages/flutter/lib/bar.dart:42: nullableInt?.clamp(0, 2.0)',
      '║ packages/flutter/lib/bar.dart:43: nullableDouble?.clamp(0, 2)',
      '║ packages/flutter/lib/bar.dart:48: nullableInt?.clamp',
      '║ packages/flutter/lib/bar.dart:50: nullableDouble?.clamp',
    ].map((String line) => line.replaceAll('/', Platform.isWindows ? r'\' : '/')).join('\n');
    expect(
      result,
      '╔═╡ERROR #1╞════════════════════════════════════════════════════════════════════\n'
      '$lines\n'
      '║ \n'
      '║ For performance reasons, we use a custom "clampDouble" function instead of using "double.clamp".\n'
      '╚═══════════════════════════════════════════════════════════════════════════════\n',
    );
  });

  test('analyze.dart - stopwatch', () async {
    final String result = await capture(
      () => analyzeWithRules(
        testRootPath,
        <AnalyzeRule>[noStopwatches],
        includePaths: <String>['packages/flutter/lib'],
      ),
      shouldHaveErrors: true,
    );
    final String lines = <String>[
      '║ packages/flutter/lib/stopwatch.dart:20: Stopwatch()',
      '║ packages/flutter/lib/stopwatch.dart:22: Stopwatch()',
      '║ packages/flutter/lib/stopwatch.dart:28: StopwatchAtHome()',
      '║ packages/flutter/lib/stopwatch.dart:33: StopwatchAtHome.new',
      '║ packages/flutter/lib/stopwatch.dart:37: StopwatchAtHome.create',
      '║ packages/flutter/lib/stopwatch.dart:44: externallib.MyStopwatch.create()',
      '║ packages/flutter/lib/stopwatch.dart:49: externallib.MyStopwatch.new',
      '║ packages/flutter/lib/stopwatch.dart:55: externallib.stopwatch',
      '║ packages/flutter/lib/stopwatch.dart:57: externallib.createMyStopwatch()',
      '║ packages/flutter/lib/stopwatch.dart:59: externallib.createStopwatch()',
      '║ packages/flutter/lib/stopwatch.dart:61: externallib.createMyStopwatch',
    ].map((String line) => line.replaceAll('/', Platform.isWindows ? r'\' : '/')).join('\n');
    expect(
      result,
      '╔═╡ERROR #1╞════════════════════════════════════════════════════════════════════\n'
      '$lines\n'
      '║ \n'
      '║ Stopwatches introduce flakes by falling out of sync with the FakeAsync used in testing.\n'
      '║ A Stopwatch that stays in sync with FakeAsync is available through the Gesture or Test bindings, through samplingClock.\n'
      '╚═══════════════════════════════════════════════════════════════════════════════\n',
    );
  });

  test('analyze.dart - RenderBox intrinsics', () async {
    final String result = await capture(
      () => analyzeWithRules(
        testRootPath,
        <AnalyzeRule>[renderBoxIntrinsicCalculation],
        includePaths: <String>['packages/flutter/lib'],
      ),
      shouldHaveErrors: true,
    );
    final String lines = <String>[
      '║ packages/flutter/lib/renderbox_intrinsics.dart:12: computeMaxIntrinsicWidth(). Consider calling getMaxIntrinsicWidth instead.',
      '║ packages/flutter/lib/renderbox_intrinsics.dart:16: f = computeMaxIntrinsicWidth. Consider calling getMaxIntrinsicWidth instead.',
      '║ packages/flutter/lib/renderbox_intrinsics.dart:23: computeDryBaseline(). Consider calling getDryBaseline instead.',
      '║ packages/flutter/lib/renderbox_intrinsics.dart:24: computeDryLayout(). Consider calling getDryLayout instead.',
      '║ packages/flutter/lib/renderbox_intrinsics.dart:31: computeDistanceToActualBaseline(). Consider calling getDistanceToBaseline, or getDistanceToActualBaseline instead.',
      '║ packages/flutter/lib/renderbox_intrinsics.dart:36: computeMaxIntrinsicHeight(). Consider calling getMaxIntrinsicHeight instead.',
    ].map((String line) => line.replaceAll('/', Platform.isWindows ? r'\' : '/')).join('\n');
    expect(
      result,
      '╔═╡ERROR #1╞════════════════════════════════════════════════════════════════════\n'
      '$lines\n'
      '║ \n'
      '║ Typically the get* methods should be used to obtain the intrinsics of a RenderBox.\n'
      '╚═══════════════════════════════════════════════════════════════════════════════\n',
    );
  });

  test('analyze.dart - verifyMaterialFilesAreUpToDateWithTemplateFiles', () async {
    String result = await capture(
      () => verifyMaterialFilesAreUpToDateWithTemplateFiles(testGenDefaultsPath, dartPath),
      shouldHaveErrors: true,
    );
    final String lines = <String>[
      '║ chip.dart is not up-to-date with the token template file.',
    ].map((String line) => line.replaceAll('/', Platform.isWindows ? r'\' : '/')).join('\n');
    const String errorStart = '╔═';
    result = result.substring(result.indexOf(errorStart));
    expect(
      result,
      '╔═╡ERROR #1╞════════════════════════════════════════════════════════════════════\n'
      '$lines\n'
      '║ See: https://github.com/flutter/flutter/blob/main/dev/tools/gen_defaults to update the token template files.\n'
      '╚═══════════════════════════════════════════════════════════════════════════════\n',
    );
  });
}
