// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:core' hide print;
import 'dart:io' hide exit;
import 'dart:typed_data';

import 'package:analyzer/dart/analysis/features.dart';
import 'package:analyzer/dart/analysis/results.dart';
import 'package:analyzer/dart/analysis/utilities.dart';
import 'package:analyzer/dart/ast/ast.dart';
import 'package:analyzer/dart/ast/visitor.dart';
import 'package:collection/equality.dart';
import 'package:crypto/crypto.dart';
import 'package:dart_style/dart_style.dart';
import 'package:meta/meta.dart';
import 'package:path/path.dart' as path;

import 'allowlist.dart';
import 'custom_rules/analyze.dart';
import 'custom_rules/avoid_future_catcherror.dart';
import 'custom_rules/no_double_clamp.dart';
import 'custom_rules/no_stop_watches.dart';
import 'custom_rules/protect_public_state_subtypes.dart';
import 'custom_rules/render_box_intrinsics.dart';
import 'run_command.dart';
import 'utils.dart';

final String flutterPackages = path.join(flutterRoot, 'packages');
final String flutterExamples = path.join(flutterRoot, 'examples');

/// The path to the `dart` executable; set at the top of `main`
late final String dart;

/// The path to the `pub` executable; set at the top of `main`
late final String pub;

/// When you call this, you can pass additional arguments to pass custom
/// arguments to flutter analyze. For example, you might want to call this
/// script with the parameter --dart-sdk to use custom dart sdk.
///
/// For example:
/// bin/cache/dart-sdk/bin/dart dev/bots/analyze.dart --dart-sdk=/tmp/dart-sdk
Future<void> main(List<String> arguments) async {
  final String dartSdk = path.join(
    Directory.current.absolute.path,
    _getDartSdkFromArguments(arguments) ?? path.join(flutterRoot, 'bin', 'cache', 'dart-sdk'),
  );
  dart = path.join(dartSdk, 'bin', Platform.isWindows ? 'dart.exe' : 'dart');
  pub = path.join(dartSdk, 'bin', Platform.isWindows ? 'pub.bat' : 'pub');
  printProgress('STARTING ANALYSIS');
  await run(arguments);
  if (hasError) {
    reportErrorsAndExit('${bold}Analysis failed.$reset');
  }
  reportSuccessAndExit('${bold}Analysis successful.$reset');
}

/// Scans [arguments] for an argument of the form `--dart-sdk` or
/// `--dart-sdk=...` and returns the configured SDK, if any.
String? _getDartSdkFromArguments(List<String> arguments) {
  String? result;
  for (int i = 0; i < arguments.length; i += 1) {
    if (arguments[i] == '--dart-sdk') {
      if (result != null) {
        foundError(<String>['The --dart-sdk argument must not be used more than once.']);
        return null;
      }
      if (i + 1 < arguments.length) {
        result = arguments[i + 1];
      } else {
        foundError(<String>['--dart-sdk must be followed by a path.']);
        return null;
      }
    }
    if (arguments[i].startsWith('--dart-sdk=')) {
      if (result != null) {
        foundError(<String>['The --dart-sdk argument must not be used more than once.']);
        return null;
      }
      result = arguments[i].substring('--dart-sdk='.length);
    }
  }
  return result;
}

Future<void> run(List<String> arguments) async {
  bool assertsEnabled = false;
  assert(() { assertsEnabled = true; return true; }());
  if (!assertsEnabled) {
    foundError(<String>['The analyze.dart script must be run with --enable-asserts.']);
  }

  printProgress('TargetPlatform tool/framework consistency');
  await verifyTargetPlatform(flutterRoot);

  printProgress('All tool test files end in _test.dart...');
  await verifyToolTestsEndInTestDart(flutterRoot);

  printProgress('No sync*/async*');
  await verifyNoSyncAsyncStar(flutterPackages);
  await verifyNoSyncAsyncStar(flutterExamples, minimumMatches: 200);

  printProgress('No runtimeType in toString...');
  await verifyNoRuntimeTypeInToString(flutterRoot);

  printProgress('Debug mode instead of checked mode...');
  await verifyNoCheckedMode(flutterRoot);

  printProgress('Links for creating GitHub issues...');
  await verifyIssueLinks(flutterRoot);

  printProgress('Links to repositories...');
  await verifyRepositoryLinks(flutterRoot);

  printProgress('Unexpected binaries...');
  await verifyNoBinaries(flutterRoot);

  printProgress('Trailing spaces...');
  await verifyNoTrailingSpaces(flutterRoot); // assumes no unexpected binaries, so should be after verifyNoBinaries

  printProgress('Spaces after flow control statements...');
  await verifySpacesAfterFlowControlStatements(flutterRoot);

  printProgress('Deprecations...');
  await verifyDeprecations(flutterRoot);

  printProgress('Goldens...');
  await verifyGoldenTags(flutterPackages);

  printProgress('Skip test comments...');
  await verifySkipTestComments(flutterRoot);

  printProgress('Licenses...');
  await verifyNoMissingLicense(flutterRoot);

  printProgress('Test imports...');
  await verifyNoTestImports(flutterRoot);

  printProgress('Bad imports (framework)...');
  await verifyNoBadImportsInFlutter(flutterRoot);

  printProgress('Bad imports (tools)...');
  await verifyNoBadImportsInFlutterTools(flutterRoot);

  printProgress('Internationalization...');
  await verifyInternationalizations(flutterRoot, dart);

  printProgress('Integration test timeouts...');
  await verifyIntegrationTestTimeouts(flutterRoot);

  printProgress('null initialized debug fields...');
  await verifyNullInitializedDebugExpensiveFields(flutterRoot);

  printProgress('Taboo words...');
  await verifyTabooDocumentation(flutterRoot);

  printProgress('Lint Kotlin files...');
  await lintKotlinFiles(flutterRoot);

  // Ensure that all package dependencies are in sync.
  printProgress('Package dependencies...');
  await runCommand(flutter, <String>['update-packages', '--verify-only'],
    workingDirectory: flutterRoot,
  );

  /// Ensure that no new dependencies have been accidentally
  /// added to core packages.
  printProgress('Package Allowlist...');
  await _checkConsumerDependencies();

  // Analyze all the Dart code in the repo.
  printProgress('Dart analysis...');
  final CommandResult dartAnalyzeResult = await _runFlutterAnalyze(flutterRoot, options: <String>[
    '--flutter-repo',
    ...arguments,
  ]);

  if (dartAnalyzeResult.exitCode == 0) {
    // Only run the private lints when the code is free of type errors. The
    // lints are easier to write when they can assume, for example, there is no
    // inheritance cycles.
    final List<AnalyzeRule> rules = <AnalyzeRule>[
      noDoubleClamp,
      noStopwatches,
      renderBoxIntrinsicCalculation,
      protectPublicStateSubtypes,
    ];
    final String ruleNames = rules.map((AnalyzeRule rule) => '\n * $rule').join();
    printProgress('Analyzing code in the framework with the following rules:$ruleNames');
    await analyzeWithRules(flutterRoot, rules,
      includePaths: const <String>['packages/flutter/lib'],
      excludePaths: const <String>['packages/flutter/lib/fix_data'],
    );
    final List<AnalyzeRule> testRules = <AnalyzeRule>[noStopwatches];
    final String testRuleNames = testRules.map((AnalyzeRule rule) => '\n * $rule').join();
    printProgress('Analyzing code in the test folder with the following rules:$testRuleNames');
    await analyzeWithRules(flutterRoot, testRules,
      includePaths: <String>['packages/flutter/test'],
    );
    final List<AnalyzeRule> toolRules = <AnalyzeRule>[AvoidFutureCatchError()];
    final String toolRuleNames = toolRules.map((AnalyzeRule rule) => '\n * $rule').join();
    printProgress('Analyzing code in the tool with the following rules:$toolRuleNames');
    await analyzeWithRules(
      flutterRoot,
      toolRules,
      includePaths: const <String>[
        'packages/flutter_tools/lib',
        'packages/flutter_tools/test',
      ],
    );
  } else {
    printProgress('Skipped performing further analysis in the framework because "flutter analyze" finished with a non-zero exit code.');
  }

  printProgress('Executable allowlist...');
  await _checkForNewExecutables();

  // Try with the --watch analyzer, to make sure it returns success also.
  // The --benchmark argument exits after one run.
  // We specify a failureMessage so that the actual output is muted in the case where _runFlutterAnalyze above already failed.
  printProgress('Dart analysis (with --watch)...');
  await _runFlutterAnalyze(flutterRoot, failureMessage: 'Dart analyzer failed when --watch was used.', options: <String>[
    '--flutter-repo',
    '--watch',
    '--benchmark',
    ...arguments,
  ]);

  // Analyze the code in `{@tool snippet}` sections in the repo.
  printProgress('Snippet code...');
  await runCommand(dart,
    <String>['--enable-asserts', path.join(flutterRoot, 'dev', 'bots', 'analyze_snippet_code.dart'), '--verbose'],
    workingDirectory: flutterRoot,
  );

  // Make sure that all of the existing samples are linked from at least one API doc comment.
  printProgress('Code sample link validation...');
  await runCommand(dart,
    <String>['--enable-asserts', path.join(flutterRoot, 'dev', 'bots', 'check_code_samples.dart')],
    workingDirectory: flutterRoot,
  );

  // Try analysis against a big version of the gallery; generate into a temporary directory.
  printProgress('Dart analysis (mega gallery)...');
  final Directory outDir = Directory.systemTemp.createTempSync('flutter_mega_gallery.');
  try {
    await runCommand(dart,
      <String>[
        path.join(flutterRoot, 'dev', 'tools', 'mega_gallery.dart'),
        '--out',
        outDir.path,
      ],
      workingDirectory: flutterRoot,
    );
    await _runFlutterAnalyze(outDir.path, failureMessage: 'Dart analyzer failed on mega_gallery benchmark.', options: <String>[
      '--watch',
      '--benchmark',
      ...arguments,
    ]);
  } finally {
    outDir.deleteSync(recursive: true);
  }

  // Ensure gen_default links the correct files
  printProgress('Correct file names in gen_defaults.dart...');
  await verifyTokenTemplatesUpdateCorrectFiles(flutterRoot);

  // Ensure material library files are up-to-date with the token template files.
  printProgress('Material library files are up-to-date with token template files...');
  await verifyMaterialFilesAreUpToDateWithTemplateFiles(flutterRoot, dart);

  // Ensure integration test files are up-to-date with the app template.
  printProgress('Up to date integration test template files...');
  await verifyIntegrationTestTemplateFiles(flutterRoot);
}


// TESTS

FeatureSet _parsingFeatureSet() => FeatureSet.latestLanguageVersion();

_Line _getLine(ParseStringResult parseResult, int offset) {
  final int lineNumber = parseResult.lineInfo.getLocation(offset).lineNumber;
  final String content = parseResult.content.substring(
    parseResult.lineInfo.getOffsetOfLine(lineNumber - 1),
    parseResult.lineInfo.getOffsetOfLine(lineNumber) - 1,
  );
  return _Line(lineNumber, content);
}

Future<void> verifyTargetPlatform(String workingDirectory) async {
  final File framework = File('$workingDirectory/packages/flutter/lib/src/foundation/platform.dart');
  final Set<String> frameworkPlatforms = <String>{};
  List<String> lines = framework.readAsLinesSync();
  int index = 0;
  while (true) {
    if (index >= lines.length) {
      foundError(<String>['${framework.path}: Can no longer find TargetPlatform enum.']);
      return;
    }
    if (lines[index].startsWith('enum TargetPlatform {')) {
      index += 1;
      break;
    }
    index += 1;
  }
  while (true) {
    if (index >= lines.length) {
      foundError(<String>['${framework.path}: Could not find end of TargetPlatform enum.']);
      return;
    }
    String line = lines[index].trim();
    final int comment = line.indexOf('//');
    if (comment >= 0) {
      line = line.substring(0, comment);
    }
    if (line == '}') {
      break;
    }
    if (line.isNotEmpty) {
      if (line.endsWith(',')) {
        frameworkPlatforms.add(line.substring(0, line.length - 1));
      } else {
        foundError(<String>['${framework.path}:$index: unparseable line when looking for TargetPlatform values']);
      }
    }
    index += 1;
  }
  final File tool = File('$workingDirectory/packages/flutter_tools/lib/src/resident_runner.dart');
  final Set<String> toolPlatforms = <String>{};
  lines = tool.readAsLinesSync();
  index = 0;
  while (true) {
    if (index >= lines.length) {
      foundError(<String>['${tool.path}: Can no longer find nextPlatform logic.']);
      return;
    }
    if (lines[index].trim().startsWith('const List<String> platforms = <String>[')) {
      index += 1;
      break;
    }
    index += 1;
  }
  while (true) {
    if (index >= lines.length) {
      foundError(<String>['${tool.path}: Could not find end of nextPlatform logic.']);
      return;
    }
    final String line = lines[index].trim();
    if (line.startsWith("'") && line.endsWith("',")) {
      toolPlatforms.add(line.substring(1, line.length - 2));
    } else if (line == '];') {
      break;
    } else {
      foundError(<String>['${tool.path}:$index: unparseable line when looking for nextPlatform values']);
    }
    index += 1;
  }
  final Set<String> frameworkExtra = frameworkPlatforms.difference(toolPlatforms);
  if (frameworkExtra.isNotEmpty) {
    foundError(<String>['TargetPlatform has some extra values not found in the tool: ${frameworkExtra.join(", ")}']);
  }
  final Set<String> toolExtra = toolPlatforms.difference(frameworkPlatforms);
  if (toolExtra.isNotEmpty) {
    foundError(<String>['The nextPlatform logic in the tool has some extra values not found in TargetPlatform: ${toolExtra.join(", ")}']);
  }
}

/// Verify Token Templates are mapped to correct file names while generating
/// M3 defaults in /dev/tools/gen_defaults/bin/gen_defaults.dart.
Future<void> verifyTokenTemplatesUpdateCorrectFiles(String workingDirectory) async {
  final List<String> errors = <String>[];

  String getMaterialDirPath(List<String> lines) {
    final String line = lines.firstWhere((String line) => line.contains('String materialLib'));
    final String relativePath = line.substring(line.indexOf("'") + 1, line.lastIndexOf("'"));
    return path.join(workingDirectory, relativePath);
  }

  String getFileName(String line) {
    const String materialLibString = r"'$materialLib/";
    final String leftClamp = line.substring(line.indexOf(materialLibString) + materialLibString.length);
    return leftClamp.substring(0, leftClamp.indexOf("'"));
  }

  final String genDefaultsBinDir = '$workingDirectory/dev/tools/gen_defaults/bin';
  final File file = File(path.join(genDefaultsBinDir, 'gen_defaults.dart'));
  final List<String> lines = file.readAsLinesSync();
  final String materialDirPath = getMaterialDirPath(lines);
  bool atLeastOneTargetLineExists = false;

  for (final String line in lines) {
    if (line.contains('updateFile();')) {
      atLeastOneTargetLineExists = true;
      final String fileName = getFileName(line);
      final String filePath = path.join(materialDirPath, fileName);
      final File file = File(filePath);

      if (!file.existsSync()) {
        errors.add('file $filePath does not exist.');
      }
    }
  }

  assert(atLeastOneTargetLineExists, 'No lines exist that this test expects to '
      'verify. Check if the target file is correct or remove this test');

  // Fail if any errors
  if (errors.isNotEmpty) {
    final String s = errors.length > 1 ? 's' : '';
    final String itThem = errors.length > 1 ? 'them' : 'it';
    foundError(<String>[
      ...errors,
      '${bold}Please correct the file name$s or remove $itThem from /dev/tools/gen_defaults/bin/gen_defaults.dart$reset',
    ]);
  }
}

/// Verify Material library files are up-to-date with the token template files
/// when running /dev/tools/gen_defaults/bin/gen_defaults.dart.
Future<void> verifyMaterialFilesAreUpToDateWithTemplateFiles(String workingDirectory, String dartExecutable) async {
  final List<String> errors = <String>[];
  const String beginGeneratedComment = '// BEGIN GENERATED TOKEN PROPERTIES';

  String getMaterialDirPath(List<String> lines) {
    final String line = lines.firstWhere((String line) => line.contains('String materialLib'));
    final String relativePath = line.substring(line.indexOf("'") + 1, line.lastIndexOf("'"));
    return path.join(workingDirectory, relativePath);
  }

  String getFileName(String line) {
    const String materialLibString = r"'$materialLib/";
    final String leftClamp = line.substring(line.indexOf(materialLibString) + materialLibString.length);
    return leftClamp.substring(0, leftClamp.indexOf("'"));
  }

  // Get the template generated code from the file.
  List<String> getGeneratedCode(List<String> lines) {
    return lines.skipWhile((String line) => !line.contains(beginGeneratedComment)).toList();
  }

  final String genDefaultsBinDir = '$workingDirectory/dev/tools/gen_defaults/bin';
  final File file = File(path.join(genDefaultsBinDir, 'gen_defaults.dart'));
  final List<String> lines = file.readAsLinesSync();
  final String materialDirPath = getMaterialDirPath(lines);
  final Map<String, List<String>> beforeGeneratedCode = <String, List<String>>{};
  final Map<String, List<String>> afterGeneratedCode = <String, List<String>>{};

  for (final String line in lines) {
    if (line.contains('updateFile();')) {
      final String fileName = getFileName(line);
      final String filePath = path.join(materialDirPath, fileName);
      final File file = File(filePath);
      beforeGeneratedCode[fileName] = getGeneratedCode(file.readAsLinesSync());
    }
  }

  // Run gen_defaults.dart to generate the token template files.
  await runCommand(dartExecutable,
    <String>['--enable-asserts', path.join('dev', 'tools', 'gen_defaults', 'bin', 'gen_defaults.dart')],
    workingDirectory: workingDirectory,
  );

  for (final String line in lines) {
    if (line.contains('updateFile();')) {
      final String fileName = getFileName(line);
      final String filePath = path.join(materialDirPath, fileName);
      final File file = File(filePath);
      afterGeneratedCode[fileName] = getGeneratedCode(file.readAsLinesSync());
    }
  }

  // Compare the generated code before and after running gen_defaults.dart.
  for (final String fileName in beforeGeneratedCode.keys) {
    final List<String> before = beforeGeneratedCode[fileName]!;
    final List<String> after = afterGeneratedCode[fileName]!;
    if (!const IterableEquality<String>().equals(before, after)) {
      errors.add('$fileName is not up-to-date with the token template file.');
    }
  }

  // Fail if any errors.
  if (errors.isNotEmpty) {
    foundError(<String>[
      ...errors,
      '${bold}See: https://github.com/flutter/flutter/blob/main/dev/tools/gen_defaults to update the token template files.$reset',
    ]);
  }
}

/// Verify tool test files end in `_test.dart`.
///
/// The test runner will only recognize files ending in `_test.dart` as tests to
/// be run: https://github.com/dart-lang/test/tree/master/pkgs/test#running-tests
Future<void> verifyToolTestsEndInTestDart(String workingDirectory) async {
  final String toolsTestPath = path.join(
    workingDirectory,
    'packages',
    'flutter_tools',
    'test',
  );
  final List<String> violations = <String>[];

  // detect files that contains calls to test(), testUsingContext(), and testWithoutContext()
  final RegExp callsTestFunctionPattern = RegExp(r'(test\(.*\)|testUsingContext\(.*\)|testWithoutContext\(.*\))');

  await for (final File file in _allFiles(toolsTestPath, 'dart', minimumMatches: 300)) {
    final bool isValidTestFile = file.path.endsWith('_test.dart');
    if (isValidTestFile) {
      continue;
    }

    final bool isTestData = file.path.contains(r'test_data');
    if (isTestData) {
      continue;
    }

    final bool isInTestShard = file.path.contains(r'.shard/');
    if (!isInTestShard) {
      continue;
    }

    final bool callsTestFunction = file.readAsStringSync().contains(callsTestFunctionPattern);
    if (!callsTestFunction) {
      continue;
    }

    violations.add(file.path);
  }
  if (violations.isNotEmpty) {
    foundError(<String>[
      '${bold}Found flutter_tools tests that do not end in `_test.dart`; these will not be run by the test runner$reset',
      ...violations,
    ]);
  }
}

Future<void> verifyNoSyncAsyncStar(String workingDirectory, {int minimumMatches = 2000 }) async {
  final RegExp syncPattern = RegExp(r'\s*?a?sync\*\s*?{');
  final RegExp ignorePattern = RegExp(r'^\s*?// The following uses a?sync\* because:? ');
  final RegExp commentPattern = RegExp(r'^\s*?//');
  final List<String> errors = <String>[];
  await for (final File file in _allFiles(workingDirectory, 'dart', minimumMatches: minimumMatches)) {
    if (file.path.contains('test')) {
      continue;
    }
    final List<String> lines = file.readAsLinesSync();
    for (int index = 0; index < lines.length; index += 1) {
      final String line = lines[index];
      if (line.startsWith(commentPattern)) {
        continue;
      }
      if (line.contains(syncPattern)) {
        int lookBehindIndex = index - 1;
        bool hasExplanation = false;
        while (lookBehindIndex >= 0 && lines[lookBehindIndex].startsWith(commentPattern)) {
          if (lines[lookBehindIndex].startsWith(ignorePattern)) {
            hasExplanation = true;
            break;
          }
          lookBehindIndex -= 1;
        }
        if (!hasExplanation) {
          errors.add('${file.path}:$index: sync*/async* without an explanation.');
        }
      }
    }
  }
  if (errors.isNotEmpty) {
    foundError(<String>[
      '${bold}Do not use sync*/async* methods. See https://github.com/flutter/flutter/blob/main/docs/contributing/Style-guide-for-Flutter-repo.md#avoid-syncasync for details.$reset',
      ...errors,
    ]);
  }
}

final RegExp _findGoldenTestPattern = RegExp(r'matchesGoldenFile\(');
final RegExp _findGoldenDefinitionPattern = RegExp(r'matchesGoldenFile\(Object');
final RegExp _leadingComment = RegExp(r'//');
final RegExp _goldenTagPattern1 = RegExp(r'@Tags\(');
final RegExp _goldenTagPattern2 = RegExp(r"'reduced-test-set'");

/// Only golden file tests in the flutter package are subject to reduced testing,
/// for example, invocations in flutter_test to validate comparator
/// functionality do not require tagging.
const String _ignoreGoldenTag = '// flutter_ignore: golden_tag (see analyze.dart)';
const String _ignoreGoldenTagForFile = '// flutter_ignore_for_file: golden_tag (see analyze.dart)';

Future<void> verifyGoldenTags(String workingDirectory, { int minimumMatches = 2000 }) async {
  final List<String> errors = <String>[];
  await for (final File file in _allFiles(workingDirectory, 'dart', minimumMatches: minimumMatches)) {
    bool needsTag = false;
    bool hasTagNotation = false;
    bool hasReducedTag = false;
    bool ignoreForFile = false;
    final List<String> lines = file.readAsLinesSync();
    for (final String line in lines) {
      if (line.contains(_goldenTagPattern1)) {
        hasTagNotation = true;
      }
      if (line.contains(_goldenTagPattern2)) {
        hasReducedTag = true;
      }
      if (line.contains(_findGoldenTestPattern)
          && !line.contains(_findGoldenDefinitionPattern)
          && !line.contains(_leadingComment)
          && !line.contains(_ignoreGoldenTag)) {
        needsTag = true;
      }
      if (line.contains(_ignoreGoldenTagForFile)) {
        ignoreForFile = true;
      }
      // If the file is being ignored or a reduced test tag is already accounted
      // for, skip parsing the rest of the lines for golden file tests.
      if (ignoreForFile || (hasTagNotation && hasReducedTag)) {
        break;
      }
    }
    // If a reduced test tag is already accounted for, move on to the next file.
    if (ignoreForFile || (hasTagNotation && hasReducedTag)) {
      continue;
    }
    // If there are golden file tests, ensure they are tagged for all reduced
    // test environments.
    if (needsTag) {
      if (!hasTagNotation) {
        errors.add('${file.path}: Files containing golden tests must be tagged using '
            "@Tags(<String>['reduced-test-set']) at the top of the file before import statements.");
      } else if (!hasReducedTag) {
        errors.add('${file.path}: Files containing golden tests must be tagged with '
            "'reduced-test-set'.");
      }
    }
  }
  if (errors.isNotEmpty) {
    foundError(<String>[
      ...errors,
      '${bold}See: https://github.com/flutter/flutter/blob/main/docs/contributing/testing/Writing-a-golden-file-test-for-package-flutter.md$reset',
    ]);
  }
}

final RegExp _findDeprecationPattern = RegExp(r'@[Dd]eprecated');
final RegExp _deprecationStartPattern = RegExp(r'^(?<indent> *)@Deprecated\($'); // flutter_ignore: deprecation_syntax (see analyze.dart)
final RegExp _deprecationMessagePattern = RegExp(r"^ *'(?<message>.+) '$");
final RegExp _deprecationVersionPattern = RegExp(r"^ *'This feature was deprecated after v(?<major>\d+)\.(?<minor>\d+)\.(?<patch>\d+)(?<build>-\d+\.\d+\.pre)?\.',?$");
final RegExp _deprecationEndPattern = RegExp(r'^ *\)$');

/// Some deprecation notices are special, for example they're used to annotate members that
/// will never go away and were never allowed but which we are trying to show messages for.
/// (One example would be a library that intentionally conflicts with a member in another
/// library to indicate that it is incompatible with that other library. Another would be
/// the regexp just above...)
const String _ignoreDeprecation = ' // flutter_ignore: deprecation_syntax (see analyze.dart)';

/// Some deprecation notices are exempt for historical reasons. They must have an issue listed.
final RegExp _legacyDeprecation = RegExp(r' // flutter_ignore: deprecation_syntax, https://github.com/flutter/flutter/issues/\d+$');

Future<void> verifyDeprecations(String workingDirectory, { int minimumMatches = 2000 }) async {
  final List<String> errors = <String>[];
  await for (final File file in _allFiles(workingDirectory, 'dart', minimumMatches: minimumMatches)) {
    int lineNumber = 0;
    final List<String> lines = file.readAsLinesSync();
    final List<int> linesWithDeprecations = <int>[];
    for (final String line in lines) {
      if (line.contains(_findDeprecationPattern) &&
          !line.endsWith(_ignoreDeprecation) &&
          !line.contains(_legacyDeprecation)) {
        linesWithDeprecations.add(lineNumber);
      }
      lineNumber += 1;
    }
    for (int lineNumber in linesWithDeprecations) {
      try {
        final RegExpMatch? startMatch = _deprecationStartPattern.firstMatch(lines[lineNumber]);
        if (startMatch == null) {
          throw 'Deprecation notice does not match required pattern.';
        }
        final String indent = startMatch.namedGroup('indent')!;
        lineNumber += 1;
        if (lineNumber >= lines.length) {
          throw 'Incomplete deprecation notice.';
        }
        RegExpMatch? versionMatch;
        String? message;
        do {
          final RegExpMatch? messageMatch = _deprecationMessagePattern.firstMatch(lines[lineNumber]);
          if (messageMatch == null) {
            String possibleReason = '';
            if (lines[lineNumber].trimLeft().startsWith('"')) {
              possibleReason = ' You might have used double quotes (") for the string instead of single quotes (\').';
            } else if (!lines[lineNumber].contains("'")) {
              possibleReason = ' It might be missing the line saying "This feature was deprecated after...".';
            } else if (!lines[lineNumber].trimRight().endsWith(" '")) {
              if (lines[lineNumber].contains('This feature was deprecated')) {
                possibleReason = ' There might not be an explanatory message.';
              } else {
                possibleReason = ' There might be a missing space character at the end of the line.';
              }
            }
            throw 'Deprecation notice does not match required pattern.$possibleReason';
          }
          if (!lines[lineNumber].startsWith("$indent  '")) {
            throw 'Unexpected deprecation notice indent.';
          }
          if (message == null) {
            message = messageMatch.namedGroup('message');
            final String firstChar = String.fromCharCode(message!.runes.first);
            if (firstChar.toUpperCase() != firstChar) {
              throw 'Deprecation notice should be a grammatically correct sentence and start with a capital letter; see style guide: https://github.com/flutter/flutter/blob/main/docs/contributing/Style-guide-for-Flutter-repo.md';
            }
          } else {
            message += messageMatch.namedGroup('message')!;
          }
          lineNumber += 1;
          if (lineNumber >= lines.length) {
            throw 'Incomplete deprecation notice.';
          }
          versionMatch = _deprecationVersionPattern.firstMatch(lines[lineNumber]);
        } while (versionMatch == null);
        final int major = int.parse(versionMatch.namedGroup('major')!);
        final int minor = int.parse(versionMatch.namedGroup('minor')!);
        final int patch = int.parse(versionMatch.namedGroup('patch')!);
        final bool hasBuild = versionMatch.namedGroup('build') != null;
        // There was a beta release that was mistakenly labeled 3.1.0 without a build.
        final bool specialBeta = major == 3 && minor == 1 && patch == 0;
        if (!specialBeta && (major > 1 || (major == 1 && minor >= 20))) {
          if (!hasBuild) {
            throw 'Deprecation notice does not accurately indicate a beta branch version number; please see https://flutter.dev/docs/development/tools/sdk/releases to find the latest beta build version number.';
          }
        }
        if (!message.endsWith('.') && !message.endsWith('!') && !message.endsWith('?')) {
          throw 'Deprecation notice should be a grammatically correct sentence and end with a period; notice appears to be "$message".';
        }
        if (!lines[lineNumber].startsWith("$indent  '")) {
          throw 'Unexpected deprecation notice indent.';
        }
        lineNumber += 1;
        if (lineNumber >= lines.length) {
          throw 'Incomplete deprecation notice.';
        }
        if (!lines[lineNumber].contains(_deprecationEndPattern)) {
          throw 'End of deprecation notice does not match required pattern.';
        }
        if (!lines[lineNumber].startsWith('$indent)')) {
          throw 'Unexpected deprecation notice indent.';
        }
      } catch (error) {
        errors.add('${file.path}:${lineNumber + 1}: $error');
      }
    }
  }
  // Fail if any errors
  if (errors.isNotEmpty) {
    foundError(<String>[
      ...errors,
      '${bold}See: https://github.com/flutter/flutter/blob/main/docs/contributing/Tree-hygiene.md#handling-breaking-changes$reset',
    ]);
  }
}

String _generateLicense(String prefix) {
  return '${prefix}Copyright 2014 The Flutter Authors. All rights reserved.\n'
         '${prefix}Use of this source code is governed by a BSD-style license that can be\n'
         '${prefix}found in the LICENSE file.';
}

Future<void> verifyNoMissingLicense(String workingDirectory, { bool checkMinimums = true }) async {
  final int? overrideMinimumMatches = checkMinimums ? null : 0;
  await _verifyNoMissingLicenseForExtension(workingDirectory, 'dart', overrideMinimumMatches ?? 2000, _generateLicense('// '));
  await _verifyNoMissingLicenseForExtension(workingDirectory, 'java', overrideMinimumMatches ?? 39, _generateLicense('// '));
  await _verifyNoMissingLicenseForExtension(workingDirectory, 'h', overrideMinimumMatches ?? 30, _generateLicense('// '));
  await _verifyNoMissingLicenseForExtension(workingDirectory, 'm', overrideMinimumMatches ?? 30, _generateLicense('// '));
  await _verifyNoMissingLicenseForExtension(workingDirectory, 'cc', overrideMinimumMatches ?? 10, _generateLicense('// '));
  await _verifyNoMissingLicenseForExtension(workingDirectory, 'cpp', overrideMinimumMatches ?? 0, _generateLicense('// '));
  await _verifyNoMissingLicenseForExtension(workingDirectory, 'swift', overrideMinimumMatches ?? 10, _generateLicense('// '));
  await _verifyNoMissingLicenseForExtension(workingDirectory, 'gradle', overrideMinimumMatches ?? 80, _generateLicense('// '));
  await _verifyNoMissingLicenseForExtension(workingDirectory, 'gn', overrideMinimumMatches ?? 0, _generateLicense('# '));
  await _verifyNoMissingLicenseForExtension(workingDirectory, 'sh', overrideMinimumMatches ?? 1, _generateLicense('# '), header: r'#!/usr/bin/env bash\n',);
  await _verifyNoMissingLicenseForExtension(workingDirectory, 'bat', overrideMinimumMatches ?? 1, _generateLicense('REM '), header: r'@ECHO off\n');
  await _verifyNoMissingLicenseForExtension(workingDirectory, 'ps1', overrideMinimumMatches ?? 1, _generateLicense('# '));
  await _verifyNoMissingLicenseForExtension(workingDirectory, 'html', overrideMinimumMatches ?? 1, '<!-- ${_generateLicense('')} -->', trailingBlank: false, header: r'<!DOCTYPE HTML>\n');
  await _verifyNoMissingLicenseForExtension(workingDirectory, 'xml', overrideMinimumMatches ?? 1, '<!-- ${_generateLicense('')} -->', header: r'(<\?xml version="1.0" encoding="utf-8"\?>\n)?');
  await _verifyNoMissingLicenseForExtension(workingDirectory, 'frag', overrideMinimumMatches ?? 1, _generateLicense('// '), header: r'#version 320 es(\n)+');
}

Future<void> _verifyNoMissingLicenseForExtension(
  String workingDirectory,
  String extension,
  int minimumMatches,
  String license, {
  bool trailingBlank = true,
  // The "header" is a regular expression matching the header that comes before
  // the license in some files.
  String header = '',
}) async {
  assert(!license.endsWith('\n'));
  final String licensePattern = RegExp.escape('$license\n${trailingBlank ? '\n' : ''}');
  final List<String> errors = <String>[];
  await for (final File file in _allFiles(workingDirectory, extension, minimumMatches: minimumMatches)) {
    final String contents = file.readAsStringSync().replaceAll('\r\n', '\n');
    if (contents.isEmpty) {
      continue; // let's not go down the /bin/true rabbit hole
    }
    if (path.basename(file.path) == 'Package.swift') {
      continue;
    }
    if (!contents.startsWith(RegExp(header + licensePattern))) {
      errors.add(file.path);
    }
  }
  // Fail if any errors
  if (errors.isNotEmpty) {
    final String fileDoes = errors.length == 1 ? 'file does' : '${errors.length} files do';
    foundError(<String>[
      '${bold}The following $fileDoes not have the right license header for $extension files:$reset',
      ...errors.map<String>((String error) => '  $error'),
      'The expected license header is:',
      if (header.isNotEmpty) 'A header matching the regular expression "$header",',
      if (header.isNotEmpty) 'followed by the following license text:',
      license,
      if (trailingBlank) '...followed by a blank line.',
    ]);
  }
}

class _Line {
  _Line(this.line, this.content);

  final int line;
  final String content;
}

Iterable<_Line> _getTestSkips(File file) {
  final ParseStringResult parseResult = parseFile(
    featureSet: _parsingFeatureSet(),
    path: file.absolute.path,
  );
  final _TestSkipLinesVisitor<CompilationUnit> visitor = _TestSkipLinesVisitor<CompilationUnit>(parseResult);
  visitor.visitCompilationUnit(parseResult.unit);
  return visitor.skips;
}

class _TestSkipLinesVisitor<T> extends RecursiveAstVisitor<T> {
  _TestSkipLinesVisitor(this.parseResult) : skips = <_Line>{};

  final ParseStringResult parseResult;
  final Set<_Line> skips;

  static bool isTestMethod(String name) {
    return name.startsWith('test') || name == 'group' || name == 'expect';
  }

  @override
  T? visitMethodInvocation(MethodInvocation node) {
    if (isTestMethod(node.methodName.toString())) {
      for (final Expression argument in node.argumentList.arguments) {
        if (argument is NamedExpression && argument.name.label.name == 'skip') {
          skips.add(_getLine(parseResult, argument.beginToken.charOffset));
        }
      }
    }
    return super.visitMethodInvocation(node);
  }
}

final RegExp _skipTestCommentPattern = RegExp(r'//(.*)$');
const Pattern _skipTestIntentionalPattern = '[intended]';
final Pattern _skipTestTrackingBugPattern = RegExp(r'https+?://github.com/.*/issues/\d+');

Future<void> verifySkipTestComments(String workingDirectory) async {
  final List<String> errors = <String>[];
  final Stream<File> testFiles =_allFiles(workingDirectory, 'dart', minimumMatches: 1500)
    .where((File f) => f.path.endsWith('_test.dart'));

  await for (final File file in testFiles) {
    for (final _Line skip in _getTestSkips(file)) {
      final Match? match = _skipTestCommentPattern.firstMatch(skip.content);
      final String? skipComment = match?.group(1);
      if (skipComment == null ||
          !(skipComment.contains(_skipTestIntentionalPattern) ||
            skipComment.contains(_skipTestTrackingBugPattern))) {
        errors.add('${file.path}:${skip.line}: skip test without a justification comment.');
      }
    }
  }

  // Fail if any errors
  if (errors.isNotEmpty) {
    foundError(<String>[
      ...errors,
      '\n${bold}See: https://github.com/flutter/flutter/blob/main/docs/contributing/Tree-hygiene.md#skipped-tests$reset',
    ]);
  }
}

final RegExp _testImportPattern = RegExp(r'''import (['"])([^'"]+_test\.dart)\1''');
const Set<String> _exemptTestImports = <String>{
  'package:flutter_test/flutter_test.dart',
  'hit_test.dart',
  'package:test_api/src/backend/live_test.dart',
  'package:integration_test/integration_test.dart',
};

Future<void> verifyNoTestImports(String workingDirectory) async {
  final List<String> errors = <String>[];
  assert("// foo\nimport 'binding_test.dart' as binding;\n'".contains(_testImportPattern));
  final List<File> dartFiles = await _allFiles(path.join(workingDirectory, 'packages'), 'dart', minimumMatches: 1500).toList();
  for (final File file in dartFiles) {
    for (final String line in file.readAsLinesSync()) {
      final Match? match = _testImportPattern.firstMatch(line);
      if (match != null && !_exemptTestImports.contains(match.group(2))) {
        errors.add(file.path);
      }
    }
  }
  // Fail if any errors
  if (errors.isNotEmpty) {
    foundError(<String>[
      '${bold}The following file(s) import a test directly. Test utilities should be in their own file.$reset',
      ...errors,
    ]);
  }
}

Future<void> verifyNoBadImportsInFlutter(String workingDirectory) async {
  final List<String> errors = <String>[];
  final String libPath = path.join(workingDirectory, 'packages', 'flutter', 'lib');
  final String srcPath = path.join(workingDirectory, 'packages', 'flutter', 'lib', 'src');
  // Verify there's one libPath/*.dart for each srcPath/*/.
  final List<String> packages = Directory(libPath).listSync()
    .where((FileSystemEntity entity) => entity is File && path.extension(entity.path) == '.dart')
    .map<String>((FileSystemEntity entity) => path.basenameWithoutExtension(entity.path))
    .toList()..sort();
  final List<String> directories = Directory(srcPath).listSync()
    .whereType<Directory>()
    .map<String>((Directory entity) => path.basename(entity.path))
    .toList()..sort();
  if (!_listEquals<String>(packages, directories)) {
    errors.add(<String>[
      'flutter/lib/*.dart does not match flutter/lib/src/*/:',
      'These are the exported packages:',
      ...packages.map<String>((String path) => '  lib/$path.dart'),
      'These are the directories:',
      ...directories.map<String>((String path) => '  lib/src/$path/'),
    ].join('\n'));
  }
  // Verify that the imports are well-ordered.
  final Map<String, Set<String>> dependencyMap = <String, Set<String>>{};
  for (final String directory in directories) {
    dependencyMap[directory] = await _findFlutterDependencies(path.join(srcPath, directory), errors, checkForMeta: directory != 'foundation');
  }
  assert(dependencyMap['material']!.contains('widgets') &&
         dependencyMap['widgets']!.contains('rendering') &&
         dependencyMap['rendering']!.contains('painting')); // to make sure we're convinced _findFlutterDependencies is finding some
  for (final String package in dependencyMap.keys) {
    if (dependencyMap[package]!.contains(package)) {
      errors.add(
        'One of the files in the $yellow$package$reset package imports that package recursively.'
      );
    }
  }

  for (final String key in dependencyMap.keys) {
    for (final String dependency in dependencyMap[key]!) {
      if (dependencyMap[dependency] != null) {
        continue;
      }
      // Sanity check before performing _deepSearch, to ensure there's no rogue
      // dependencies.
      final String validFilenames = dependencyMap.keys.map((String name) => '$name.dart').join(', ');
      errors.add(
        '$key imported package:flutter/$dependency.dart '
        'which is not one of the valid exports { $validFilenames }.\n'
        'Consider changing $dependency.dart to one of them.'
      );
    }
  }

  for (final String package in dependencyMap.keys) {
    final List<String>? loop = _deepSearch<String>(dependencyMap, package);
    if (loop != null) {
      errors.add('${yellow}Dependency loop:$reset ${loop.join(' depends on ')}');
    }
  }
  // Fail if any errors
  if (errors.isNotEmpty) {
    foundError(<String>[
      if (errors.length == 1)
        '${bold}An error was detected when looking at import dependencies within the Flutter package:$reset'
      else
        '${bold}Multiple errors were detected when looking at import dependencies within the Flutter package:$reset',
      ...errors,
    ]);
  }
}

Future<void> verifyNoBadImportsInFlutterTools(String workingDirectory) async {
  final List<String> errors = <String>[];
  final List<File> files = await _allFiles(path.join(workingDirectory, 'packages', 'flutter_tools', 'lib'), 'dart', minimumMatches: 200).toList();
  for (final File file in files) {
    if (file.readAsStringSync().contains('package:flutter_tools/')) {
      errors.add('$yellow${file.path}$reset imports flutter_tools.');
    }
  }
  // Fail if any errors
  if (errors.isNotEmpty) {
    foundError(<String>[
      if (errors.length == 1)
        '${bold}An error was detected when looking at import dependencies within the flutter_tools package:$reset'
      else
        '${bold}Multiple errors were detected when looking at import dependencies within the flutter_tools package:$reset',
      ...errors.map((String paragraph) => '$paragraph\n'),
    ]);
  }
}

Future<void> verifyIntegrationTestTimeouts(String workingDirectory) async {
  final List<String> errors = <String>[];
  final String dev = path.join(workingDirectory, 'dev');
  final List<File> files = await _allFiles(dev, 'dart', minimumMatches: 1)
      .where((File file) => file.path.contains('test_driver') && (file.path.endsWith('_test.dart') || file.path.endsWith('util.dart')))
      .toList();
  for (final File file in files) {
    final String contents = file.readAsStringSync();
    final int testCount = ' test('.allMatches(contents).length;
    final int timeoutNoneCount = 'timeout: Timeout.none'.allMatches(contents).length;
    if (testCount != timeoutNoneCount) {
      errors.add('$yellow${file.path}$reset has at least $testCount test(s) but only $timeoutNoneCount `Timeout.none`(s).');
    }
  }
  if (errors.isNotEmpty) {
    foundError(<String>[
      if (errors.length == 1)
        '${bold}An error was detected when looking at integration test timeouts:$reset'
      else
        '${bold}Multiple errors were detected when looking at integration test timeouts:$reset',
      ...errors.map((String paragraph) => '$paragraph\n'),
    ]);
  }
}

final DartFormatter _formatter = DartFormatter(languageVersion: DartFormatter.latestLanguageVersion);

Future<void> verifyInternationalizations(String workingDirectory, String dartExecutable) async {
  await _verifyGenLocalizations(workingDirectory, dartExecutable);
  await _verifyStocksAppLocalizations(workingDirectory, dartExecutable);
}

Future<void> _verifyStocksAppLocalizations(String workingDirectory, String dartExecutable) async {
  final Directory appRoot = Directory(
    path.join(workingDirectory, 'dev', 'benchmarks', 'test_apps', 'stocks'),
  );

  if (!appRoot.existsSync()) {}

  // We have already run `flutter update-packages`, so let's just check if
  // there's a dirty checkout.
  final EvalResult result = await _evalCommand('git', const <String>['diff-files', '--name-only']);
  final List<String> dirtyFiles = result.stdout.trim().split('\n');
}

Future<void> _verifyGenLocalizations(String workingDirectory, String dartExecutable) async {
  final EvalResult materialGenResult = await _evalCommand(
    dartExecutable,
    <String>[
      path.join('dev', 'tools', 'localization', 'bin', 'gen_localizations.dart'),
      '--material',
      '--remove-undefined',
    ],
    workingDirectory: workingDirectory,
  );
  final EvalResult cupertinoGenResult = await _evalCommand(
    dartExecutable,
    <String>[
      path.join('dev', 'tools', 'localization', 'bin', 'gen_localizations.dart'),
      '--cupertino',
      '--remove-undefined',
    ],
    workingDirectory: workingDirectory,
  );

  final String materialLocalizationsFile = path.join(workingDirectory, 'packages', 'flutter_localizations', 'lib', 'src', 'l10n', 'generated_material_localizations.dart');
  final String cupertinoLocalizationsFile = path.join(workingDirectory, 'packages', 'flutter_localizations', 'lib', 'src', 'l10n', 'generated_cupertino_localizations.dart');
  final String expectedMaterialResult = await File(materialLocalizationsFile).readAsString();
  final String expectedCupertinoResult = await File(cupertinoLocalizationsFile).readAsString();

  // Normalize both with the Dart formatter.
  final String formattedMaterialGenResult = _formatter.format(materialGenResult.stdout.trim());
  final String formattedExpectedMaterialResult = _formatter.format(expectedMaterialResult.trim());

  if (formattedMaterialGenResult != formattedExpectedMaterialResult) {
    foundError(<String>[
      '<<<<<<< $materialLocalizationsFile',
      formattedExpectedMaterialResult,
      '=======',
      formattedMaterialGenResult,
      '>>>>>>> gen_localizations',
      'The contents of $materialLocalizationsFile are different from that produced by gen_localizations.',
      '',
      'Did you forget to run gen_localizations.dart after updating a .arb file?',
    ]);
  }
  final String formattedCupertinoGenResult = _formatter.format(cupertinoGenResult.stdout.trim());
  final String formattedExpectedCupertinoResult = _formatter.format(expectedCupertinoResult.trim());
  if (formattedCupertinoGenResult != formattedExpectedCupertinoResult) {
    foundError(<String>[
      '<<<<<<< $cupertinoLocalizationsFile',
      formattedExpectedCupertinoResult,
      '=======',
      formattedCupertinoGenResult,
      '>>>>>>> gen_localizations',
      'The contents of $cupertinoLocalizationsFile are different from that produced by gen_localizations.',
      '',
      'Did you forget to run gen_localizations.dart after updating a .arb file?',
    ]);
  }
}


/// Verifies that all instances of "checked mode" have been migrated to "debug mode".
Future<void> verifyNoCheckedMode(String workingDirectory) async {
  final String flutterPackages = path.join(workingDirectory, 'packages');
  final List<File> files = await _allFiles(flutterPackages, 'dart', minimumMatches: 400)
      .where((File file) => path.extension(file.path) == '.dart')
      .toList();
  final List<String> problems = <String>[];
  for (final File file in files) {
    int lineCount = 0;
    for (final String line in file.readAsLinesSync()) {
      if (line.toLowerCase().contains('checked mode')) {
        problems.add('${file.path}:$lineCount uses deprecated "checked mode" instead of "debug mode".');
      }
      lineCount += 1;
    }
  }
  if (problems.isNotEmpty) {
    foundError(problems);
  }
}


Future<void> verifyNoRuntimeTypeInToString(String workingDirectory) async {
  final String flutterLib = path.join(workingDirectory, 'packages', 'flutter', 'lib');
  final Set<String> excludedFiles = <String>{
    path.join(flutterLib, 'src', 'foundation', 'object.dart'), // Calls this from within an assert.
  };
  final List<File> files = await _allFiles(flutterLib, 'dart', minimumMatches: 400)
      .where((File file) => !excludedFiles.contains(file.path))
      .toList();
  final RegExp toStringRegExp = RegExp(r'^\s+String\s+to(.+?)?String(.+?)?\(\)\s+(\{|=>)');
  final List<String> problems = <String>[];
  for (final File file in files) {
    final List<String> lines = file.readAsLinesSync();
    for (int index = 0; index < lines.length; index++) {
      if (toStringRegExp.hasMatch(lines[index])) {
        final int sourceLine = index + 1;
        bool checkForRuntimeType(String line) {
          if (line.contains(r'$runtimeType') || line.contains('runtimeType.toString()')) {
            problems.add('${file.path}:$sourceLine}: toString calls runtimeType.toString');
            return true;
          }
          return false;
        }
        if (checkForRuntimeType(lines[index])) {
          continue;
        }
        if (lines[index].contains('=>')) {
          while (!lines[index].contains(';')) {
            index++;
            assert(index < lines.length, 'Source file $file has unterminated toString method.');
            if (checkForRuntimeType(lines[index])) {
              break;
            }
          }
        } else {
          int openBraceCount = '{'.allMatches(lines[index]).length - '}'.allMatches(lines[index]).length;
          while (!lines[index].contains('}') && openBraceCount > 0) {
            index++;
            assert(index < lines.length, 'Source file $file has unbalanced braces in a toString method.');
            if (checkForRuntimeType(lines[index])) {
              break;
            }
            openBraceCount += '{'.allMatches(lines[index]).length;
            openBraceCount -= '}'.allMatches(lines[index]).length;
          }
        }
      }
    }
  }
  if (problems.isNotEmpty) {
    foundError(problems);
  }
}

Future<void> verifyNoTrailingSpaces(String workingDirectory, { int minimumMatches = 4000 }) async {
  final List<File> files = await _allFiles(workingDirectory, null, minimumMatches: minimumMatches)
    .where((File file) => path.basename(file.path) != 'serviceaccount.enc')
    .where((File file) => path.basename(file.path) != 'Ahem.ttf')
    .where((File file) => path.extension(file.path) != '.snapshot')
    .where((File file) => path.extension(file.path) != '.png')
    .where((File file) => path.extension(file.path) != '.jpg')
    .where((File file) => path.extension(file.path) != '.ico')
    .where((File file) => path.extension(file.path) != '.jar')
    .where((File file) => path.extension(file.path) != '.swp')
    .toList();
  final List<String> problems = <String>[];
  for (final File file in files) {
    final List<String> lines = file.readAsLinesSync();
    for (int index = 0; index < lines.length; index += 1) {
      if (lines[index].endsWith(' ')) {
        problems.add('${file.path}:${index + 1}: trailing U+0020 space character');
      } else if (lines[index].endsWith('\t')) {
        problems.add('${file.path}:${index + 1}: trailing U+0009 tab character');
      }
    }
    if (lines.isNotEmpty && lines.last == '') {
      problems.add('${file.path}:${lines.length}: trailing blank line');
    }
  }
  if (problems.isNotEmpty) {
    foundError(problems);
  }
}

final RegExp _flowControlStatementWithoutSpace = RegExp(r'(^|[ \t])(if|switch|for|do|while|catch)\(', multiLine: true);

Future<void> verifySpacesAfterFlowControlStatements(String workingDirectory, { int minimumMatches = 4000 }) async {
  const Set<String> extensions = <String>{
    '.dart',
    '.java',
    '.js',
    '.kt',
    '.swift',
    '.c',
    '.cc',
    '.cpp',
    '.h',
    '.m',
  };
  final List<File> files = await _allFiles(workingDirectory, null, minimumMatches: minimumMatches)
    .where((File file) => extensions.contains(path.extension(file.path)))
    .toList();
  final List<String> problems = <String>[];
  for (final File file in files) {
    final List<String> lines = file.readAsLinesSync();
    for (int index = 0; index < lines.length; index += 1) {
      if (lines[index].contains(_flowControlStatementWithoutSpace)) {
        problems.add('${file.path}:${index + 1}: no space after flow control statement');
      }
    }
  }
  if (problems.isNotEmpty) {
    foundError(problems);
  }
}

String _bullets(String value) => ' * $value';

Future<void> verifyIssueLinks(String workingDirectory) async {
  const String issueLinkPrefix = 'https://github.com/flutter/flutter/issues/new';
  const Set<String> stops = <String>{ '\n', ' ', "'", '"', r'\', ')', '>' };
  assert(!stops.contains('.')); // instead of "visit https://foo." say "visit: https://foo", it copy-pastes better
  const String kGiveTemplates =
    'Prefer to provide a link either to $issueLinkPrefix/choose (the list of issue '
    'templates) or to a specific template directly ($issueLinkPrefix?template=...).\n';
  final Set<String> templateNames =
    Directory(path.join(workingDirectory, '.github', 'ISSUE_TEMPLATE'))
      .listSync()
      .whereType<File>()
      .where((File file) => path.extension(file.path) == '.md' || path.extension(file.path) == '.yml')
      .map<String>((File file) => path.basename(file.path))
      .toSet();
  final String kTemplates = 'The available templates are:\n${templateNames.map(_bullets).join("\n")}';
  final List<String> problems = <String>[];
  final Set<String> suggestions = <String>{};
  final List<File> files = await _gitFiles(workingDirectory);
  for (final File file in files) {
    if (path.basename(file.path).endsWith('_test.dart') || path.basename(file.path) == 'analyze.dart') {
      continue; // Skip tests, they're not public-facing.
    }
    final Uint8List bytes = file.readAsBytesSync();
    // We allow invalid UTF-8 here so that binaries don't trip us up.
    // There's a separate test in this file that verifies that all text
    // files are actually valid UTF-8 (see verifyNoBinaries below).
    final String contents = utf8.decode(bytes, allowMalformed: true);
    int start = 0;
    while ((start = contents.indexOf(issueLinkPrefix, start)) >= 0) {
      int end = start + issueLinkPrefix.length;
      while (end < contents.length && !stops.contains(contents[end])) {
        end += 1;
      }
      final String url = contents.substring(start, end);
      if (url == issueLinkPrefix) {
        if (file.path != path.join(workingDirectory, 'dev', 'bots', 'analyze.dart')) {
          problems.add('${file.path} contains a direct link to $issueLinkPrefix.');
          suggestions.add(kGiveTemplates);
          suggestions.add(kTemplates);
        }
      } else if (url.startsWith('$issueLinkPrefix?')) {
        final Uri parsedUrl = Uri.parse(url);
        final List<String>? templates = parsedUrl.queryParametersAll['template'];
        if (templates == null) {
          problems.add('${file.path} contains $url, which has no "template" argument specified.');
          suggestions.add(kGiveTemplates);
          suggestions.add(kTemplates);
        } else if (templates.length != 1) {
          problems.add('${file.path} contains $url, which has ${templates.length} templates specified.');
          suggestions.add(kGiveTemplates);
          suggestions.add(kTemplates);
        } else if (!templateNames.contains(templates.single)) {
          problems.add('${file.path} contains $url, which specifies a non-existent template ("${templates.single}").');
          suggestions.add(kTemplates);
        } else if (parsedUrl.queryParametersAll.keys.length > 1) {
          problems.add('${file.path} contains $url, which the analyze.dart script is not sure how to handle.');
          suggestions.add('Update analyze.dart to handle the URLs above, or change them to the expected pattern.');
        }
      } else if (url != '$issueLinkPrefix/choose') {
        problems.add('${file.path} contains $url, which the analyze.dart script is not sure how to handle.');
        suggestions.add('Update analyze.dart to handle the URLs above, or change them to the expected pattern.');
      }
      start = end;
    }
  }
  assert(problems.isEmpty == suggestions.isEmpty);
  if (problems.isNotEmpty) {
    foundError(<String>[
      ...problems,
      ...suggestions,
    ]);
  }
}

Future<void> verifyRepositoryLinks(String workingDirectory) async {
  const Set<String> stops = <String>{ '\n', ' ', "'", '"', r'\', ')', '>' };
  assert(!stops.contains('.')); // instead of "visit https://foo." say "visit: https://foo", it copy-pastes better

  // Repos whose default branch is still 'master'
  const Set<String> repoExceptions = <String>{
    'bdero/flutter-gpu-examples',
    'chromium/chromium',
    'clojure/clojure',
    'dart-lang/test', // TODO(guidezpl): remove when https://github.com/dart-lang/test/issues/2209 is closed
    'dart-lang/webdev',
    'eseidelGoogle/bezier_perf',
    'flutter/devtools', // TODO(guidezpl): remove when https://github.com/flutter/devtools/issues/7551 is closed
    'flutter/flutter_gallery_assets', // TODO(guidezpl): remove when subtask in https://github.com/flutter/flutter/issues/121564 is complete
    'flutter/flutter-intellij', // TODO(guidezpl): remove when https://github.com/flutter/flutter-intellij/issues/7342 is closed
    'flutter/platform_tests', // TODO(guidezpl): remove when subtask in https://github.com/flutter/flutter/issues/121564 is complete
    'flutter/web_installers',
    'glfw/glfw',
    'GoogleCloudPlatform/artifact-registry-maven-tools',
    'material-components/material-components-android', // TODO(guidezpl): remove when https://github.com/material-components/material-components-android/issues/4144 is closed
    'torvalds/linux',
    'tpn/winsdk-10',
  };

  // See dev/bots/test/analyze-test-input/root/packages/foo/bad_repository_links.dart
  // for examples of repository links that are not allowed.
  final RegExp pattern = RegExp(r'^(https:\/\/(?:cs\.opensource\.google|github|raw\.githubusercontent|source\.chromium|([a-z0-9\-]+)\.googlesource)\.)');

  final List<String> problems = <String>[];
  final Set<String> suggestions = <String>{};
  final List<File> files = await _allFiles(workingDirectory, null, minimumMatches: 10).toList();
  for (final File file in files) {
    final Uint8List bytes = file.readAsBytesSync();
    // We allow invalid UTF-8 here so that binaries don't trip us up.
    // There's a separate test in this file that verifies that all text
    // files are actually valid UTF-8 (see verifyNoBinaries below).
    final String contents = utf8.decode(bytes, allowMalformed: true);
    int start = 0;
    while ((start = contents.indexOf('https://', start)) >= 0) { // Find all 'https://' links
      int end = start + 8; // Length of 'https://'
      while (end < contents.length && !stops.contains(contents[end])) {
        end += 1;
      }
      final String url = contents.substring(start, end).replaceAll('\r', '');

      if (pattern.hasMatch(url) && !repoExceptions.any(url.contains)) {
        if (url.contains('master')) {
          problems.add('${file.path} contains $url, which uses the banned "master" branch.');
          suggestions.add('Change the URLs above to the expected pattern by '
              'using the "main" branch if it exists, otherwise adding the '
              'repository to the list of exceptions in analyze.dart.');
        }
      }
      start = end;
    }
  }
  assert(problems.isEmpty == suggestions.isEmpty);
  if (problems.isNotEmpty) {
    foundError(<String>[
      ...problems,
      ...suggestions,
    ]);
  }
}

@immutable
class Hash256 {
  const Hash256(this.a, this.b, this.c, this.d);

  factory Hash256.fromDigest(Digest digest) {
    assert(digest.bytes.length == 32);
    return Hash256(
      digest.bytes[ 0] << 56 |
      digest.bytes[ 1] << 48 |
      digest.bytes[ 2] << 40 |
      digest.bytes[ 3] << 32 |
      digest.bytes[ 4] << 24 |
      digest.bytes[ 5] << 16 |
      digest.bytes[ 6] <<  8 |
      digest.bytes[ 7] <<  0,
      digest.bytes[ 8] << 56 |
      digest.bytes[ 9] << 48 |
      digest.bytes[10] << 40 |
      digest.bytes[11] << 32 |
      digest.bytes[12] << 24 |
      digest.bytes[13] << 16 |
      digest.bytes[14] <<  8 |
      digest.bytes[15] <<  0,
      digest.bytes[16] << 56 |
      digest.bytes[17] << 48 |
      digest.bytes[18] << 40 |
      digest.bytes[19] << 32 |
      digest.bytes[20] << 24 |
      digest.bytes[21] << 16 |
      digest.bytes[22] <<  8 |
      digest.bytes[23] <<  0,
      digest.bytes[24] << 56 |
      digest.bytes[25] << 48 |
      digest.bytes[26] << 40 |
      digest.bytes[27] << 32 |
      digest.bytes[28] << 24 |
      digest.bytes[29] << 16 |
      digest.bytes[30] <<  8 |
      digest.bytes[31] <<  0,
    );
  }

  final int a;
  final int b;
  final int c;
  final int d;

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is Hash256
        && other.a == a
        && other.b == b
        && other.c == c
        && other.d == d;
  }

  @override
  int get hashCode => Object.hash(a, b, c, d);
}

// DO NOT ADD ANY ENTRIES TO THIS LIST.
// We have a policy of not checking in binaries into this repository.
// If you are adding/changing template images, use the flutter_template_images
// package and a .img.tmpl placeholder instead.
// If you have other binaries to add, please consult Hixie for advice.
final Set<Hash256> _legacyBinaries = <Hash256>{
  // DEFAULT ICON IMAGES

  // packages/flutter_tools/templates/app/android.tmpl/app/src/main/res/mipmap-hdpi/ic_launcher.png
  // packages/flutter_tools/templates/module/android/host_app_common/app.tmpl/src/main/res/mipmap-hdpi/ic_launcher.png
  // (also used by many examples)
  const Hash256(0x6A7C8F0D703E3682, 0x108F9662F8133022, 0x36240D3F8F638BB3, 0x91E32BFB96055FEF),

  // packages/flutter_tools/templates/app/android.tmpl/app/src/main/res/mipmap-mdpi/ic_launcher.png
  // (also used by many examples)
  const Hash256(0xC7C0C0189145E4E3, 0x2A401C61C9BDC615, 0x754B0264E7AFAE24, 0xE834BB81049EAF81),

  // packages/flutter_tools/templates/app/android.tmpl/app/src/main/res/mipmap-xhdpi/ic_launcher.png
  // (also used by many examples)
  const Hash256(0xE14AA40904929BF3, 0x13FDED22CF7E7FFC, 0xBF1D1AAC4263B5EF, 0x1BE8BFCE650397AA),

  // packages/flutter_tools/templates/app/android.tmpl/app/src/main/res/mipmap-xxhdpi/ic_launcher.png
  // (also used by many examples)
  const Hash256(0x4D470BF22D5C17D8, 0x4EDC5F82516D1BA8, 0xA1C09559CD761CEF, 0xB792F86D9F52B540),

  // packages/flutter_tools/templates/app/android.tmpl/app/src/main/res/mipmap-xxxhdpi/ic_launcher.png
  // (also used by many examples)
  const Hash256(0x3C34E1F298D0C9EA, 0x3455D46DB6B7759C, 0x8211A49E9EC6E44B, 0x635FC5C87DFB4180),

  // packages/flutter_tools/templates/app/ios.tmpl/Runner/Assets.xcassets/AppIcon.appiconset/Icon-App-1024x1024@1x.png
  // packages/flutter_tools/templates/module/ios/host_app_ephemeral/Runner.tmpl/Assets.xcassets/AppIcon.appiconset/Icon-App-1024x1024@1x.png
  // (also used by a few examples)
  const Hash256(0x7770183009E91411, 0x2DE7D8EF1D235A6A, 0x30C5834424858E0D, 0x2F8253F6B8D31926),

  // packages/flutter_tools/templates/app/ios.tmpl/Runner/Assets.xcassets/AppIcon.appiconset/Icon-App-20x20@1x.png
  // packages/flutter_tools/templates/module/ios/host_app_ephemeral/Runner.tmpl/Assets.xcassets/AppIcon.appiconset/Icon-App-20x20@1x.png
  // (also used by many examples)
  const Hash256(0x5925DAB509451F9E, 0xCBB12CE8A625F9D4, 0xC104718EE20CAFF8, 0xB1B51032D1CD8946),

  // packages/flutter_tools/templates/app/ios.tmpl/Runner/Assets.xcassets/AppIcon.appiconset/Icon-App-20x20@2x.png
  // packages/flutter_tools/templates/app/ios.tmpl/Runner/Assets.xcassets/AppIcon.appiconset/Icon-App-40x40@1x.png
  // packages/flutter_tools/templates/module/ios/host_app_ephemeral/Runner.tmpl/Assets.xcassets/AppIcon.appiconset/Icon-App-20x20@2x.png
  // packages/flutter_tools/templates/module/ios/host_app_ephemeral/Runner.tmpl/Assets.xcassets/AppIcon.appiconset/Icon-App-40x40@1x.png
  // (also used by many examples)
  const Hash256(0xC4D9A284C12301D0, 0xF50E248EC53ED51A, 0x19A10147B774B233, 0x08399250B0D44C55),

  // packages/flutter_tools/templates/app/ios.tmpl/Runner/Assets.xcassets/AppIcon.appiconset/Icon-App-20x20@3x.png
  // packages/flutter_tools/templates/module/ios/host_app_ephemeral/Runner.tmpl/Assets.xcassets/AppIcon.appiconset/Icon-App-20x20@3x.png
  // (also used by many examples)
  const Hash256(0xBF97F9D3233F33E1, 0x389B09F7B8ADD537, 0x41300CB834D6C7A5, 0xCA32CBED363A4FB2),

  // packages/flutter_tools/templates/app/ios.tmpl/Runner/Assets.xcassets/AppIcon.appiconset/Icon-App-29x29@1x.png
  // packages/flutter_tools/templates/module/ios/host_app_ephemeral/Runner.tmpl/Assets.xcassets/AppIcon.appiconset/Icon-App-29x29@1x.png
  // (also used by many examples)
  const Hash256(0x285442F69A06B45D, 0x9D79DF80321815B5, 0x46473548A37B7881, 0x9B68959C7B8ED237),

  // packages/flutter_tools/templates/app/ios.tmpl/Runner/Assets.xcassets/AppIcon.appiconset/Icon-App-29x29@2x.png
  // packages/flutter_tools/templates/module/ios/host_app_ephemeral/Runner.tmpl/Assets.xcassets/AppIcon.appiconset/Icon-App-29x29@2x.png
  // (also used by many examples)
  const Hash256(0x2AB64AF8AC727EA9, 0x9C6AB9EAFF847F46, 0xFBF2A9A0A78A0ABC, 0xBF3180F3851645B4),

  // packages/flutter_tools/templates/app/ios.tmpl/Runner/Assets.xcassets/AppIcon.appiconset/Icon-App-29x29@3x.png
  // packages/flutter_tools/templates/module/ios/host_app_ephemeral/Runner.tmpl/Assets.xcassets/AppIcon.appiconset/Icon-App-29x29@3x.png
  // (also used by many examples)
  const Hash256(0x9DCA09F4E5ED5684, 0xD3C4DFF41F4E8B7C, 0xB864B438172D72BE, 0x069315FA362930F9),

  // packages/flutter_tools/templates/app/ios.tmpl/Runner/Assets.xcassets/AppIcon.appiconset/Icon-App-40x40@2x.png
  // packages/flutter_tools/templates/module/ios/host_app_ephemeral/Runner.tmpl/Assets.xcassets/AppIcon.appiconset/Icon-App-40x40@2x.png
  // (also used by many examples)
  const Hash256(0xD5AD04DE321EF37C, 0xACC5A7B960AFCCE7, 0x1BDCB96FA020C482, 0x49C1545DD1A0F497),

  // packages/flutter_tools/templates/app/ios.tmpl/Runner/Assets.xcassets/AppIcon.appiconset/Icon-App-40x40@3x.png
  // packages/flutter_tools/templates/app/ios.tmpl/Runner/Assets.xcassets/AppIcon.appiconset/Icon-App-60x60@2x.png
  // packages/flutter_tools/templates/module/ios/host_app_ephemeral/Runner.tmpl/Assets.xcassets/AppIcon.appiconset/Icon-App-40x40@3x.png
  // packages/flutter_tools/templates/module/ios/host_app_ephemeral/Runner.tmpl/Assets.xcassets/AppIcon.appiconset/Icon-App-60x60@2x.png
  // (also used by many examples)
  const Hash256(0x809ABFE75C440770, 0xC13C4E2E46D09603, 0xC22053E9D4E0E227, 0x5DCB9C1DCFBB2C75),

  // packages/flutter_tools/templates/app/ios.tmpl/Runner/Assets.xcassets/AppIcon.appiconset/Icon-App-60x60@3x.png
  // packages/flutter_tools/templates/module/ios/host_app_ephemeral/Runner.tmpl/Assets.xcassets/AppIcon.appiconset/Icon-App-60x60@3x.png
  // (also used by many examples)
  const Hash256(0x3DB08CB79E7B01B9, 0xE81F956E3A0AE101, 0x48D0FAFDE3EA7AA7, 0x0048DF905AA52CFD),

  // packages/flutter_tools/templates/app/ios.tmpl/Runner/Assets.xcassets/AppIcon.appiconset/Icon-App-76x76@1x.png
  // packages/flutter_tools/templates/module/ios/host_app_ephemeral/Runner.tmpl/Assets.xcassets/AppIcon.appiconset/Icon-App-76x76@1x.png
  // (also used by many examples)
  const Hash256(0x23C13D463F5DCA5C, 0x1F14A14934003601, 0xC29F1218FD461016, 0xD8A22CEF579A665F),

  // packages/flutter_tools/templates/app/ios.tmpl/Runner/Assets.xcassets/AppIcon.appiconset/Icon-App-76x76@2x.png
  // packages/flutter_tools/templates/module/ios/host_app_ephemeral/Runner.tmpl/Assets.xcassets/AppIcon.appiconset/Icon-App-76x76@2x.png
  // (also used by many examples)
  const Hash256(0x6DB7726530D71D3F, 0x52CB59793EB69131, 0x3BAA04796E129E1E, 0x043C0A58A1BFFD2F),

  // packages/flutter_tools/templates/app/ios.tmpl/Runner/Assets.xcassets/AppIcon.appiconset/Icon-App-83.5x83.5@2x.png
  // packages/flutter_tools/templates/module/ios/host_app_ephemeral/Runner.tmpl/Assets.xcassets/AppIcon.appiconset/Icon-App-83.5x83.5@2x.png
  // (also used by many examples)
  const Hash256(0xCEE565F5E6211656, 0x9B64980B209FD5CA, 0x4B3D3739011F5343, 0x250B33A1A2C6EB65),

  // packages/flutter_tools/templates/app/ios.tmpl/Runner/Assets.xcassets/LaunchImage.imageset/LaunchImage.png
  // packages/flutter_tools/templates/app/ios.tmpl/Runner/Assets.xcassets/LaunchImage.imageset/LaunchImage@2x.png
  // packages/flutter_tools/templates/app/ios.tmpl/Runner/Assets.xcassets/LaunchImage.imageset/LaunchImage@3x.png
  // packages/flutter_tools/templates/module/ios/host_app_ephemeral/Runner.tmpl/Assets.xcassets/LaunchImage.imageset/LaunchImage.png
  // packages/flutter_tools/templates/module/ios/host_app_ephemeral/Runner.tmpl/Assets.xcassets/LaunchImage.imageset/LaunchImage@2x.png
  // packages/flutter_tools/templates/module/ios/host_app_ephemeral/Runner.tmpl/Assets.xcassets/LaunchImage.imageset/LaunchImage@3x.png
  // (also used by many examples)
  const Hash256(0x93AE7D494FAD0FB3, 0x0CBF3AE746A39C4B, 0xC7A0F8BBF87FBB58, 0x7A3F3C01F3C5CE20),

  // packages/flutter_tools/templates/app/macos.tmpl/Runner/Assets.xcassets/AppIcon.appiconset/app_icon_1024.png
  // (also used by a few examples)
  const Hash256(0xB18BEBAAD1AD6724, 0xE48BCDF699BA3927, 0xDF3F258FEBE646A3, 0xAB5C62767C6BAB40),

  // packages/flutter_tools/templates/app/macos.tmpl/Runner/Assets.xcassets/AppIcon.appiconset/app_icon_128.png
  // (also used by a few examples)
  const Hash256(0xF90D839A289ECADB, 0xF2B0B3400DA43EB8, 0x08B84908335AE4A0, 0x07457C4D5A56A57C),

  // packages/flutter_tools/templates/app/macos.tmpl/Runner/Assets.xcassets/AppIcon.appiconset/app_icon_16.png
  // (also used by a few examples)
  const Hash256(0x592C2ABF84ADB2D3, 0x91AED8B634D3233E, 0x2C65369F06018DCD, 0x8A4B27BA755EDCBE),

  // packages/flutter_tools/templates/app/macos.tmpl/Runner/Assets.xcassets/AppIcon.appiconset/app_icon_256.png
  // (also used by a few examples)
  const Hash256(0x75D9A0C034113CA8, 0xA1EC11C24B81F208, 0x6630A5A5C65C7D26, 0xA5DC03A1C0A4478C),

  // packages/flutter_tools/templates/app/macos.tmpl/Runner/Assets.xcassets/AppIcon.appiconset/app_icon_32.png
  // (also used by a few examples)
  const Hash256(0xA896E65745557732, 0xC72BD4EE3A10782F, 0xE2AA95590B5AF659, 0x869E5808DB9C01C1),

  // packages/flutter_tools/templates/app/macos.tmpl/Runner/Assets.xcassets/AppIcon.appiconset/app_icon_512.png
  // (also used by a few examples)
  const Hash256(0x3A69A8A1AAC5D9A8, 0x374492AF4B6D07A4, 0xCE637659EB24A784, 0x9C4DFB261D75C6A3),

  // packages/flutter_tools/templates/app/macos.tmpl/Runner/Assets.xcassets/AppIcon.appiconset/app_icon_64.png
  // (also used by a few examples)
  const Hash256(0xD29D4E0AF9256DC9, 0x2D0A8F8810608A5E, 0x64A132AD8B397CA2, 0xC4DDC0B1C26A68C3),

  // packages/flutter_tools/templates/app/web/icons/Icon-192.png.copy.tmpl
  // dev/integration_tests/flutter_gallery/web/icons/Icon-192.png
  const Hash256(0x3DCE99077602F704, 0x21C1C6B2A240BC9B, 0x83D64D86681D45F2, 0x154143310C980BE3),

  // packages/flutter_tools/templates/app/web/icons/Icon-512.png.copy.tmpl
  // dev/integration_tests/flutter_gallery/web/icons/Icon-512.png
  const Hash256(0xBACCB205AE45f0B4, 0x21BE1657259B4943, 0xAC40C95094AB877F, 0x3BCBE12CD544DCBE),

  // packages/flutter_tools/templates/app/web/favicon.png.copy.tmpl
  // dev/integration_tests/flutter_gallery/web/favicon.png
  const Hash256(0x7AB2525F4B86B65D, 0x3E4C70358A17E5A1, 0xAAF6F437f99CBCC0, 0x46DAD73d59BB9015),

  // GALLERY ICONS

  // dev/integration_tests/flutter_gallery/android/app/src/main/res/mipmap-hdpi/ic_background.png
  const Hash256(0x03CFDE53C249475C, 0x277E8B8E90AC8A13, 0xE5FC13C358A94CCB, 0x67CA866C9862A0DD),

  // dev/integration_tests/flutter_gallery/android/app/src/main/res/mipmap-hdpi/ic_foreground.png
  const Hash256(0x86A83E23A505EFCC, 0x39C358B699EDE12F, 0xC088EE516A1D0C73, 0xF3B5D74DDAD164B1),

  // dev/integration_tests/flutter_gallery/android/app/src/main/res/mipmap-hdpi/ic_launcher.png
  const Hash256(0xD813B1A77320355E, 0xB68C485CD47D0F0F, 0x3C7E1910DCD46F08, 0x60A6401B8DC13647),

  // dev/integration_tests/flutter_gallery/android/app/src/main/res/mipmap-xhdpi/ic_background.png
  const Hash256(0x35AFA76BD5D6053F, 0xEE927436C78A8794, 0xA8BA5F5D9FC9653B, 0xE5B96567BB7215ED),

  // dev/integration_tests/flutter_gallery/android/app/src/main/res/mipmap-xhdpi/ic_foreground.png
  const Hash256(0x263CE9B4F1F69B43, 0xEBB08AE9FE8F80E7, 0x95647A59EF2C040B, 0xA8AEB246861A7DFF),

  // dev/integration_tests/flutter_gallery/android/app/src/main/res/mipmap-xhdpi/ic_launcher.png
  const Hash256(0x5E1A93C3653BAAFF, 0x1AAC6BCEB8DCBC2F, 0x2AE7D68ECB07E507, 0xCB1FA8354B28313A),

  // dev/integration_tests/flutter_gallery/android/app/src/main/res/mipmap-xxhdpi/ic_background.png
  const Hash256(0xA5C77499151DDEC6, 0xDB40D0AC7321FD74, 0x0646C0C0F786743F, 0x8F3C3C408CAC5E8C),

  // dev/integration_tests/flutter_gallery/android/app/src/main/res/mipmap-xxhdpi/ic_foreground.png
  const Hash256(0x33DE450980A2A16B, 0x1982AC7CDC1E7B01, 0x919E07E0289C2139, 0x65F85BCED8895FEF),

  // dev/integration_tests/flutter_gallery/android/app/src/main/res/mipmap-xxhdpi/ic_launcher.png
  const Hash256(0xC3B8577F4A89BA03, 0x830944FB06C3566B, 0x4C99140A2CA52958, 0x089BFDC3079C59B7),

  // dev/integration_tests/flutter_gallery/android/app/src/main/res/mipmap-xxxhdpi/ic_background.png
  const Hash256(0xDEBC241D6F9C5767, 0x8980FDD46FA7ED0C, 0x5B8ACD26BCC5E1BC, 0x473C89B432D467AD),

  // dev/integration_tests/flutter_gallery/android/app/src/main/res/mipmap-xxxhdpi/ic_foreground.png
  const Hash256(0xBEFE5F7E82BF8B64, 0x148D869E3742004B, 0xF821A9F5A1BCDC00, 0x357D246DCC659DC2),

  // dev/integration_tests/flutter_gallery/android/app/src/main/res/mipmap-xxxhdpi/ic_launcher.png
  const Hash256(0xC385404341FF9EDD, 0x30FBE76F0EC99155, 0x8EA4F4AFE8CC0C60, 0x1CA3EDEF177E1DA8),

  // dev/integration_tests/flutter_gallery/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-1024.png
  const Hash256(0x6BE5751A29F57A80, 0x36A4B31CC542C749, 0x984E49B22BD65CAA, 0x75AE8B2440848719),

  // dev/integration_tests/flutter_gallery/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-120.png
  const Hash256(0x9972A2264BFA8F8D, 0x964AFE799EADC1FA, 0x2247FB31097F994A, 0x1495DC32DF071793),

  // dev/integration_tests/flutter_gallery/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-152.png
  const Hash256(0x4C7CC9B09BEEDA24, 0x45F57D6967753910, 0x57D68E1A6B883D2C, 0x8C52701A74F1400F),

  // dev/integration_tests/flutter_gallery/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-167.png
  const Hash256(0x66DACAC1CFE4D349, 0xDBE994CB9125FFD7, 0x2D795CFC9CF9F739, 0xEDBB06CE25082E9C),

  // dev/integration_tests/flutter_gallery/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-180.png
  const Hash256(0x5188621015EBC327, 0xC9EF63AD76E60ECE, 0xE82BDC3E4ABF09E2, 0xEE0139FA7C0A2BE5),

  // dev/integration_tests/flutter_gallery/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-20.png
  const Hash256(0x27D2752D04EE9A6B, 0x78410E208F74A6CD, 0xC90D9E03B73B8C60, 0xD05F7D623E790487),

  // dev/integration_tests/flutter_gallery/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-29.png
  const Hash256(0xBB20556B2826CF85, 0xD5BAC73AA69C2AC3, 0x8E71DAD64F15B855, 0xB30CB73E0AF89307),

  // dev/integration_tests/flutter_gallery/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-40.png
  const Hash256(0x623820FA45CDB0AC, 0x808403E34AD6A53E, 0xA3E9FDAE83EE0931, 0xB020A3A4EF2CDDE7),

  // dev/integration_tests/flutter_gallery/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-58.png
  const Hash256(0xC6D631D1E107215E, 0xD4A58FEC5F3AA4B5, 0x0AE9724E07114C0C, 0x453E5D87C2CAD3B3),

  // dev/integration_tests/flutter_gallery/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-60.png
  const Hash256(0x4B6F58D1EB8723C6, 0xE717A0D09FEC8806, 0x90C6D1EF4F71836E, 0x618672827979B1A2),

  // dev/integration_tests/flutter_gallery/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-76.png
  const Hash256(0x0A1744CC7634D508, 0xE85DD793331F0C8A, 0x0B7C6DDFE0975D8F, 0x29E91C905BBB1BED),

  // dev/integration_tests/flutter_gallery/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-80.png
  const Hash256(0x24032FBD1E6519D6, 0x0BA93C0D5C189554, 0xF50EAE23756518A2, 0x3FABACF4BD5DAF08),

  // dev/integration_tests/flutter_gallery/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-87.png
  const Hash256(0xC17BAE6DF6BB234A, 0xE0AF4BEB0B805F12, 0x14E74EB7AA9A30F1, 0x5763689165DA7DDF),


  // STOCKS ICONS

  // dev/benchmarks/test_apps/stocks/android/app/src/main/res/mipmap-hdpi/ic_launcher.png
  const Hash256(0x74052AB5241D4418, 0x7085180608BC3114, 0xD12493C50CD8BBC7, 0x56DED186C37ACE84),

  // dev/benchmarks/test_apps/stocks/android/app/src/main/res/mipmap-mdpi/ic_launcher.png
  const Hash256(0xE37947332E3491CB, 0x82920EE86A086FEA, 0xE1E0A70B3700A7DA, 0xDCAFBDD8F40E2E19),

  // dev/benchmarks/test_apps/stocks/android/app/src/main/res/mipmap-xhdpi/ic_launcher.png
  const Hash256(0xE608CDFC0C8579FB, 0xE38873BAAF7BC944, 0x9C9D2EE3685A4FAE, 0x671EF0C8BC41D17C),

  // dev/benchmarks/test_apps/stocks/android/app/src/main/res/mipmap-xxhdpi/ic_launcher.png
  const Hash256(0xBD53D86977DF9C54, 0xF605743C5ABA114C, 0x9D51D1A8BB917E1A, 0x14CAA26C335CAEBD),

  // dev/benchmarks/test_apps/stocks/android/app/src/main/res/mipmap-xxxhdpi/ic_launcher.png
  const Hash256(0x64E4D02262C4F3D0, 0xBB4FDC21CD0A816C, 0x4CD2A0194E00FB0F, 0x1C3AE4142FAC0D15),

  // dev/benchmarks/test_apps/stocks/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-60@2x.png
  // dev/benchmarks/test_apps/stocks/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-Small-40@3x.png
  const Hash256(0x5BA3283A76918FC0, 0xEE127D0F22D7A0B6, 0xDF03DAED61669427, 0x93D89DDD87A08117),

  // dev/benchmarks/test_apps/stocks/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-60@3x.png
  const Hash256(0xCD7F26ED31DEA42A, 0x535D155EC6261499, 0x34E6738255FDB2C4, 0xBD8D4BDDE9A99B05),

  // dev/benchmarks/test_apps/stocks/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-76.png
  const Hash256(0x3FA1225FC9A96A7E, 0xCD071BC42881AB0E, 0x7747EB72FFB72459, 0xA37971BBAD27EE24),

  // dev/benchmarks/test_apps/stocks/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-76@2x.png
  const Hash256(0xCD867001ACD7BBDB, 0x25CDFD452AE89FA2, 0x8C2DC980CAF55F48, 0x0B16C246CFB389BC),

  // dev/benchmarks/test_apps/stocks/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-83.5@2x.png
  const Hash256(0x848E9736E5C4915A, 0x7945BCF6B32FD56B, 0x1F1E7CDDD914352E, 0xC9681D38EF2A70DA),

  // dev/benchmarks/test_apps/stocks/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-Notification.png
  const Hash256(0x654BA7D6C4E05CA0, 0x7799878884EF8F11, 0xA383E1F24CEF5568, 0x3C47604A966983C8),

  // dev/benchmarks/test_apps/stocks/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-Notification@2x.png
  // dev/benchmarks/test_apps/stocks/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-Small-40.png
  const Hash256(0x743056FE7D83FE42, 0xA2990825B6AD0415, 0x1AF73D0D43B227AA, 0x07EBEA9B767381D9),

  // dev/benchmarks/test_apps/stocks/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-Notification@3x.png
  const Hash256(0xA7E1570812D119CF, 0xEF4B602EF28DD0A4, 0x100D066E66F5B9B9, 0x881765DC9303343B),

  // dev/benchmarks/test_apps/stocks/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-Small-40@2x.png
  const Hash256(0xB4102839A1E41671, 0x62DACBDEFA471953, 0xB1EE89A0AB7594BE, 0x1D9AC1E67DC2B2CE),

  // dev/benchmarks/test_apps/stocks/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-Small.png
  const Hash256(0x70AC6571B593A967, 0xF1CBAEC9BC02D02D, 0x93AD766D8290ADE6, 0x840139BF9F219019),

  // dev/benchmarks/test_apps/stocks/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-Small@2x.png
  const Hash256(0x5D87A78386DA2C43, 0xDDA8FEF2CA51438C, 0xE5A276FE28C6CF0A, 0xEBE89085B56665B6),

  // dev/benchmarks/test_apps/stocks/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-Small@3x.png
  const Hash256(0x4D9F5E81F668DA44, 0xB20A77F8BF7BA2E1, 0xF384533B5AD58F07, 0xB3A2F93F8635CD96),


  // LEGACY ICONS

  // dev/benchmarks/complex_layout/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-App-76x76@3x.png
  // dev/benchmarks/microbenchmarks/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-App-76x76@3x.png
  // examples/flutter_view/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-App-76x76@3x.png
  // (not really sure where this came from, or why neither the template nor most examples use them)
  const Hash256(0x6E645DC9ED913AAD, 0xB50ED29EEB16830D, 0xB32CA12F39121DB9, 0xB7BC1449DDDBF8B8),

  // dev/benchmarks/macrobenchmarks/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-App-1024x1024@1x.png
  // dev/integration_tests/codegen/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-App-1024x1024@1x.png
  // dev/integration_tests/ios_add2app/ios_add2app/Assets.xcassets/AppIcon.appiconset/Icon-App-1024x1024@1x.png
  // dev/integration_tests/release_smoke_test/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-App-1024x1024@1x.png
  const Hash256(0xDEFAC77E08EC71EC, 0xA04CCA3C95D1FC33, 0xB9F26E1CB15CB051, 0x47DEFC79CDD7C158),

  // examples/flutter_view/ios/Runner/ic_add.png
  // examples/platform_view/ios/Runner/ic_add.png
  const Hash256(0x3CCE7450334675E2, 0xE3AABCA20B028993, 0x127BE82FE0EB3DFF, 0x8B027B3BAF052F2F),

  // examples/image_list/images/coast.jpg
  const Hash256(0xDA957FD30C51B8D2, 0x7D74C2C918692DC4, 0xD3C5C99BB00F0D6B, 0x5EBB30395A6EDE82),

  // examples/image_list/ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-App-1024x1024@1x.png
  const Hash256(0xB5792CA06F48A431, 0xD4379ABA2160BD5D, 0xE92339FC64C6A0D3, 0x417AA359634CD905),


  // TEST ASSETS

  // dev/benchmarks/macrobenchmarks/assets/999x1000.png
  const Hash256(0x553E9C36DFF3E610, 0x6A608BDE822A0019, 0xDE4F1769B6FBDB97, 0xBC3C20E26B839F59),

  // dev/bots/test/analyze-test-input/root/packages/foo/serviceaccount.enc
  const Hash256(0xA8100AE6AA1940D0, 0xB663BB31CD466142, 0xEBBDBD5187131B92, 0xD93818987832EB89),

  // dev/automated_tests/icon/test.png
  const Hash256(0xE214B4A0FEEEC6FA, 0x8E7AA8CC9BFBEC40, 0xBCDAC2F2DEBC950F, 0x75AF8EBF02BCE459),

  // dev/integration_tests/android_splash_screens/splash_screen_kitchen_sink/android/app/src/main/res/drawable-land-xxhdpi/flutter_splash_screen.png
  // dev/integration_tests/android_splash_screens/splash_screen_kitchen_sink/android/app/src/main/res/mipmap-land-xxhdpi/flutter_splash_screen.png
  const Hash256(0x2D4F8D7A3DFEF9D3, 0xA0C66938E169AB58, 0x8C6BBBBD1973E34E, 0x03C428416D010182),

  // dev/integration_tests/android_splash_screens/splash_screen_kitchen_sink/android/app/src/main/res/drawable-xxhdpi/flutter_splash_screen.png
  // dev/integration_tests/android_splash_screens/splash_screen_kitchen_sink/android/app/src/main/res/mipmap-xxhdpi/flutter_splash_screen.png
  const Hash256(0xCD46C01BAFA3B243, 0xA6AA1645EEDDE481, 0x143AC8ABAB1A0996, 0x22CAA9D41F74649A),

  // dev/integration_tests/flutter_driver_screenshot_test/assets/red_square.png
  const Hash256(0x40054377E1E084F4, 0x4F4410CE8F44C210, 0xABA945DFC55ED0EF, 0x23BDF9469E32F8D3),

  // dev/integration_tests/flutter_driver_screenshot_test/test_driver/goldens/red_square_image/iPhone7,2.png
  const Hash256(0x7F9D27C7BC418284, 0x01214E21CA886B2F, 0x40D9DA2B31AE7754, 0x71D68375F9C8A824),

  // examples/flutter_view/assets/flutter-mark-square-64.png
  // examples/platform_view/assets/flutter-mark-square-64.png
  const Hash256(0xF416B0D8AC552EC8, 0x819D1F492D1AB5E6, 0xD4F20CF45DB47C22, 0x7BB431FEFB5B67B2),

  // packages/flutter_tools/test/data/intellij/plugins/Dart/lib/Dart.jar
  const Hash256(0x576E489D788A13DB, 0xBF40E4A39A3DAB37, 0x15CCF0002032E79C, 0xD260C69B29E06646),

  // packages/flutter_tools/test/data/intellij/plugins/flutter-intellij.jar
  const Hash256(0x4C67221E25626CB2, 0x3F94E1F49D34E4CF, 0x3A9787A514924FC5, 0x9EF1E143E5BC5690),


  // MISCELLANEOUS

  // dev/bots/serviceaccount.enc
  const Hash256(0x1F19ADB4D80AFE8C, 0xE61899BA776B1A8D, 0xCA398C75F5F7050D, 0xFB0E72D7FBBBA69B),

  // dev/docs/favicon.ico
  const Hash256(0x67368CA1733E933A, 0xCA3BC56EF0695012, 0xE862C371AD4412F0, 0x3EC396039C609965),

  // dev/snippets/assets/code_sample.png
  const Hash256(0xAB2211A47BDA001D, 0x173A52FD9C75EBC7, 0xE158942FFA8243AD, 0x2A148871990D4297),

  // dev/snippets/assets/code_snippet.png
  const Hash256(0xDEC70574DA46DFBB, 0xFA657A771F3E1FBD, 0xB265CFC6B2AA5FE3, 0x93BA4F325D1520BA),

  // packages/flutter_tools/static/Ahem.ttf
  const Hash256(0x63D2ABD0041C3E3B, 0x4B52AD8D382353B5, 0x3C51C6785E76CE56, 0xED9DACAD2D2E31C4),
};

Future<void> verifyNoBinaries(String workingDirectory, { Set<Hash256>? legacyBinaries }) async {
  // Please do not add anything to the _legacyBinaries set above.
  // We have a policy of not checking in binaries into this repository.
  // If you are adding/changing template images, use the flutter_template_images
  // package and a .img.tmpl placeholder instead.
  // If you have other binaries to add, please consult Hixie for advice.
  assert(
    _legacyBinaries
      .expand<int>((Hash256 hash) => <int>[hash.a, hash.b, hash.c, hash.d])
      .reduce((int value, int element) => value ^ element) == 0x606B51C908B40BFA // Please do not modify this line.
  );
  legacyBinaries ??= _legacyBinaries;
  if (!Platform.isWindows) { // TODO(ianh): Port this to Windows
    final List<File> files = await _gitFiles(workingDirectory);
    final List<String> problems = <String>[];
    for (final File file in files) {
      final Uint8List bytes = file.readAsBytesSync();
      try {
        utf8.decode(bytes);
      } on FormatException catch (error) {
        final Digest digest = sha256.convert(bytes);
        if (!legacyBinaries.contains(Hash256.fromDigest(digest))) {
          problems.add('${file.path}:${error.offset}: file is not valid UTF-8');
        }
      }
    }
    if (problems.isNotEmpty) {
      foundError(<String>[
        ...problems,
        'All files in this repository must be UTF-8. In particular, images and other binaries',
        'must not be checked into this repository. This is because we are very sensitive to the',
        'size of the repository as it is distributed to all our developers. If you have a binary',
        'to which you need access, you should consider how to fetch it from another repository;',
        'for example, the "assets-for-api-docs" repository is used for images in API docs.',
        'To add assets to flutter_tools templates, see the instructions in the wiki:',
        'https://github.com/flutter/flutter/blob/main/docs/tool/Managing-template-image-assets.md',
      ]);
    }
  }
}


// UTILITY FUNCTIONS

bool _listEquals<T>(List<T> a, List<T> b) {
  if (a.length != b.length) {
    return false;
  }
  for (int index = 0; index < a.length; index += 1) {
    if (a[index] != b[index]) {
      return false;
    }
  }
  return true;
}

Future<List<File>> _gitFiles(String workingDirectory, {bool runSilently = true}) async {
  final EvalResult evalResult = await _evalCommand(
    'git', <String>['ls-files', '-z'],
    workingDirectory: workingDirectory,
    runSilently: runSilently,
  );
  if (evalResult.exitCode != 0) {
    foundError(<String>[
      'git ls-files failed with exit code ${evalResult.exitCode}',
      '${bold}stdout:$reset',
      evalResult.stdout,
      '${bold}stderr:$reset',
      evalResult.stderr,
    ]);
  }
  final List<String> filenames = evalResult
      .stdout
      .split('\x00');
  assert(filenames.last.isEmpty); // git ls-files gives a trailing blank 0x00
  filenames.removeLast();
  return filenames
      .where((String filename) => !filename.startsWith('engine/'))
      .map<File>((String filename) => File(path.join(workingDirectory, filename)))
      .toList();
}

Stream<File> _allFiles(String workingDirectory, String? extension, { required int minimumMatches }) async* {
  final Set<String> gitFileNamesSet = <String>{};
  gitFileNamesSet.addAll((await _gitFiles(workingDirectory)).map((File f) => path.canonicalize(f.absolute.path)));

  assert(extension == null || !extension.startsWith('.'), 'Extension argument should not start with a period.');
  final Set<FileSystemEntity> pending = <FileSystemEntity>{ Directory(workingDirectory) };
  int matches = 0;
  while (pending.isNotEmpty) {
    final FileSystemEntity entity = pending.first;
    pending.remove(entity);
    if (path.extension(entity.path) == '.tmpl') {
      continue;
    }
    if (entity is File) {
      if (!gitFileNamesSet.contains(path.canonicalize(entity.absolute.path))) {
        continue;
      }
      if (_isGeneratedPluginRegistrant(entity)) {
        continue;
      }
      switch (path.basename(entity.path)) {
        case 'flutter_export_environment.sh' || 'gradlew.bat' || '.DS_Store':
          continue;
      }
      if (extension == null || path.extension(entity.path) == '.$extension') {
        matches += 1;
        yield entity;
      }
    } else if (entity is Directory) {
      if (File(path.join(entity.path, '.dartignore')).existsSync()) {
        continue;
      }
      switch (path.basename(entity.path)) {
        case '.git' || '.idea' || '.gradle' || '.dart_tool' || 'build':
          continue;
      }
      pending.addAll(entity.listSync());
    }
  }
  assert(matches >= minimumMatches, 'Expected to find at least $minimumMatches files with extension ".$extension" in "$workingDirectory", but only found $matches.');
}

class EvalResult {
  EvalResult({
    required this.stdout,
    required this.stderr,
    this.exitCode = 0,
  });

  final String stdout;
  final String stderr;
  final int exitCode;
}

// TODO(ianh): Refactor this to reuse the code in run_command.dart
Future<EvalResult> _evalCommand(String executable, List<String> arguments, {
  required String workingDirectory,
  Map<String, String>? environment,
  bool allowNonZeroExit = false,
  bool runSilently = false,
}) async {
  final String commandDescription = '${path.relative(executable, from: workingDirectory)} ${arguments.join(' ')}';
  final String relativeWorkingDir = path.relative(workingDirectory);

  if (!runSilently) {
    print('RUNNING: cd $cyan$relativeWorkingDir$reset; $green$commandDescription$reset');
  }

  final Stopwatch time = Stopwatch()..start();
  final Process process = await Process.start(executable, arguments,
    workingDirectory: workingDirectory,
    environment: environment,
  );

  final Future<List<List<int>>> savedStdout = process.stdout.toList();
  final Future<List<List<int>>> savedStderr = process.stderr.toList();
  final int exitCode = await process.exitCode;
  final EvalResult result = EvalResult(
    stdout: utf8.decode((await savedStdout).expand<int>((List<int> ints) => ints).toList()),
    stderr: utf8.decode((await savedStderr).expand<int>((List<int> ints) => ints).toList()),
    exitCode: exitCode,
  );

  if (!runSilently) {
    print('ELAPSED TIME: $bold${prettyPrintDuration(time.elapsed)}$reset for $commandDescription in $relativeWorkingDir');
  }

  if (exitCode != 0 && !allowNonZeroExit) {
    foundError(<String>[
      result.stderr,
      '${bold}ERROR:$red Last command exited with $exitCode.$reset',
      '${bold}Command:$red $commandDescription$reset',
      '${bold}Relative working directory:$red $relativeWorkingDir$reset',
    ]);
  }

  return result;
}

Future<void> _checkConsumerDependencies() async {
  const List<String> kCorePackages = <String>[
    'flutter',
    'flutter_test',
    'flutter_driver',
    'flutter_localizations',
    'integration_test',
    'fuchsia_remote_debug_protocol',
  ];
  final Set<String> dependencies = <String>{};

  // Parse the output of pub deps --json to determine all of the
  // current packages used by the core set of flutter packages.
  for (final String package in kCorePackages) {
    final ProcessResult result = await Process.run(flutter, <String>[
      'pub',
      'deps',
      '--json',
      '--directory=${path.join(flutterRoot, 'packages', package)}',
    ]);
    if (result.exitCode != 0) {
      foundError(<String>[
        result.stdout.toString(),
        result.stderr.toString(),
      ]);
      return;
    }
    final Map<String, Object?> rawJson = json.decode(result.stdout as String) as Map<String, Object?>;
    final Map<String, Map<String, Object?>> dependencyTree = <String, Map<String, Object?>>{
      for (final Map<String, Object?> package in (rawJson['packages']! as List<Object?>).cast<Map<String, Object?>>())
        package['name']! as String : package,
    };
    final List<Map<String, Object?>> workset = <Map<String, Object?>>[];
    workset.add(dependencyTree[package]!);

    while (workset.isNotEmpty) {
      final Map<String, Object?> currentPackage = workset.removeLast();
      if (currentPackage['kind'] == 'dev') {
        continue;
      }
      dependencies.add(currentPackage['name']! as String);

      final List<String> currentDependencies = (currentPackage['dependencies']! as List<Object?>).cast<String>();
      for (final String dependency in currentDependencies) {
        // Don't add dependencies we've already seen or we will get stuck
        // forever if there are any circular references.
        // TODO(dantup): Consider failing gracefully with the names of the
        //  packages once the cycle between test_api and matcher is resolved.
        //  https://github.com/dart-lang/test/issues/1979
        if (!dependencies.contains(dependency)) {
          workset.add(dependencyTree[dependency]!);
        }
      }
    }
  }

  final Set<String> removed = kCorePackageAllowList.difference(dependencies);
  final Set<String> added = dependencies.difference(kCorePackageAllowList);

  String plural(int n, String s, String p) => n == 1 ? s : p;

  if (added.isNotEmpty) {
    foundError(<String>[
      'The transitive closure of package dependencies contains ${plural(added.length, "a non-allowlisted package", "non-allowlisted packages")}:',
      '  ${added.join(', ')}',
      'We strongly desire to keep the number of dependencies to a minimum and',
      'therefore would much prefer not to add new dependencies.',
      'See dev/bots/allowlist.dart for instructions on how to update the package',
      'allowlist if you nonetheless believe this is a necessary addition.',
    ]);
  }

  if (removed.isNotEmpty) {
    foundError(<String>[
      'Excellent news! ${plural(removed.length, "A package dependency has been removed!", "Multiple package dependencies have been removed!")}',
      '  ${removed.join(', ')}',
      'To make sure we do not accidentally add ${plural(removed.length, "this dependency", "these dependencies")} back in the future,',
      'please remove ${plural(removed.length, "this", "these")} packages from the allow-list in dev/bots/allowlist.dart.',
      'Thanks!',
    ]);
  }
}

const String _kDebugOnlyAnnotation = '@_debugOnly';
final RegExp _nullInitializedField = RegExp(r'kDebugMode \? [\w<> ,{}()]+ : null;');

Future<void> verifyNullInitializedDebugExpensiveFields(String workingDirectory, {int minimumMatches = 400}) async {
  final String flutterLib = path.join(workingDirectory, 'packages', 'flutter', 'lib');
  final List<File> files = await _allFiles(flutterLib, 'dart', minimumMatches: minimumMatches)
    .toList();
  final List<String> errors = <String>[];
  for (final File file in files) {
    final List<String> lines = file.readAsLinesSync();
    for (int index = 0; index < lines.length; index += 1) {
      final String line = lines[index];
      if (!line.contains(_kDebugOnlyAnnotation)) {
        continue;
      }
      final String nextLine = lines[index + 1];
      if (_nullInitializedField.firstMatch(nextLine) == null) {
        errors.add('${file.path}:$index');
      }
    }
  }
  if (errors.isNotEmpty) {
    foundError(<String>[
     '${bold}ERROR: ${red}fields annotated with @_debugOnly must null initialize.$reset',
     'to ensure both the field and initializer are removed from profile/release mode.',
     'These fields should be written as:\n',
     'field = kDebugMode ? <DebugValue> : null;\n',
     'Errors were found in the following files:',
      ...errors,
    ]);
  }
}

final RegExp tabooPattern = RegExp(r'^ *///.*\b(simply|note:|note that)\b', caseSensitive: false);

Future<void> verifyTabooDocumentation(String workingDirectory, { int minimumMatches = 100 }) async {
  final List<String> errors = <String>[];
  await for (final File file in _allFiles(workingDirectory, 'dart', minimumMatches: minimumMatches)) {
    final List<String> lines = file.readAsLinesSync();
    for (int index = 0; index < lines.length; index += 1) {
      final String line = lines[index];
      final Match? match = tabooPattern.firstMatch(line);
      if (match != null) {
        errors.add('${file.path}:${index + 1}: Found use of the taboo word "${match.group(1)}" in documentation string.');
      }
    }
  }
  if (errors.isNotEmpty) {
    foundError(<String>[
      '${bold}Avoid the word "simply" in documentation. See https://github.com/flutter/flutter/blob/main/docs/contributing/Style-guide-for-Flutter-repo.md#use-the-passive-voice-recommend-do-not-require-never-say-things-are-simple for details.$reset',
      '${bold}In many cases these words can be omitted without loss of generality; in other cases it may require a bit of rewording to avoid implying that the task is simple.$reset',
      '${bold}Similarly, avoid using "note:" or the phrase "note that". See https://github.com/flutter/flutter/blob/main/docs/contributing/Style-guide-for-Flutter-repo.md#avoid-empty-prose for details.$reset',
      ...errors,
    ]);
  }
}

Future<void> lintKotlinFiles(String workingDirectory) async {
  const String baselineRelativePath = 'dev/bots/test/analyze-test-input/ktlint-baseline.xml';
  const String editorConfigRelativePath = 'dev/bots/test/analyze-test-input/.editorconfig';
  final EvalResult lintResult = await _evalCommand('ktlint',
      <String>['--baseline=$flutterRoot/$baselineRelativePath', '--editorconfig=$flutterRoot/$editorConfigRelativePath'],
      workingDirectory: workingDirectory);
  if (lintResult.exitCode != 0) {
    final String errorMessage = 'Found lint violations in Kotlin files:\n ${lintResult.stdout}\n\n'
        'To reproduce this lint locally:\n'
        '1. Identify the CIPD version tag used to resolve this particular version of ktlint (check the dependencies section of this shard in the ci.yaml). \n'
        '2. Download that version from https://chrome-infra-packages.appspot.com/p/flutter/ktlint/linux-amd64/+/<version_tag>\n'
        '3. From the repository root, run `<path_to_ktlint>/ktlint --editorconfig=$editorConfigRelativePath --baseline=$baselineRelativePath`\n'
        'Alternatively, if you use Android Studio, follow the docs at docs/platforms/android/Kotlin-android-studio-formatting.md to enable auto formatting.';
    foundError(<String>[errorMessage]);
  }
}

const List<String> _kIgnoreList = <String>[
  'Runner.rc.tmpl',
  'flutter_window.cpp',
];
final String _kIntegrationTestsRelativePath = path.join('dev', 'integration_tests');
final String _kTemplateRelativePath = path.join('packages', 'flutter_tools', 'templates', 'app_shared', 'windows.tmpl', 'runner');
final String _kWindowsRunnerSubPath = path.join('windows', 'runner');
const String _kProjectNameKey = '{{projectName}}';
const String _kTmplExt = '.tmpl';
final String _kLicensePath = path.join('dev', 'conductor', 'core', 'lib', 'src', 'proto', 'license_header.txt');

String _getFlutterLicense(String flutterRoot) {
  return '${File(path.join(flutterRoot, _kLicensePath)).readAsLinesSync().join("\n")}\n\n';
}

String _removeLicenseIfPresent(String fileContents, String license) {
  if (fileContents.startsWith(license)) {
    return fileContents.substring(license.length);
  }
  return fileContents;
}

Future<void> verifyIntegrationTestTemplateFiles(String flutterRoot) async {
  final List<String> errors = <String>[];
  final String license = _getFlutterLicense(flutterRoot);
  final String integrationTestsPath = path.join(flutterRoot, _kIntegrationTestsRelativePath);
  final String templatePath = path.join(flutterRoot, _kTemplateRelativePath);
  final Iterable<Directory>subDirs = Directory(integrationTestsPath).listSync().toList().whereType<Directory>();
  for (final Directory testPath in subDirs) {
    final String projectName = path.basename(testPath.path);
    final String runnerPath = path.join(testPath.path, _kWindowsRunnerSubPath);
    final Directory runner = Directory(runnerPath);
    if (!runner.existsSync()) {
      continue;
    }
    final Iterable<File> files = Directory(templatePath).listSync().toList().whereType<File>();
    for (final File templateFile in files) {
      final String fileName = path.basename(templateFile.path);
      if (_kIgnoreList.contains(fileName)) {
        continue;
      }
      String templateFileContents = templateFile.readAsLinesSync().join('\n');
      String appFilePath = path.join(runnerPath, fileName);
      if (fileName.endsWith(_kTmplExt)) {
        appFilePath = appFilePath.substring(0, appFilePath.length - _kTmplExt.length); // Remove '.tmpl' from app file path
        templateFileContents = templateFileContents.replaceAll(_kProjectNameKey, projectName); // Substitute template project name
      }
      String appFileContents = File(appFilePath).readAsLinesSync().join('\n');
      appFileContents = _removeLicenseIfPresent(appFileContents, license);
      if (appFileContents != templateFileContents) {
        int indexOfDifference;
        for (indexOfDifference = 0; indexOfDifference < appFileContents.length; indexOfDifference++) {
          if (indexOfDifference >= templateFileContents.length || templateFileContents.codeUnitAt(indexOfDifference) != appFileContents.codeUnitAt(indexOfDifference)) {
            break;
          }
        }
        final String error = '''
Error: file $fileName mismatched for integration test $testPath
Verify the integration test has been migrated to the latest app template.
=====$appFilePath======
$appFileContents
=====${templateFile.path}======
$templateFileContents
==========
Diff at character #$indexOfDifference
        ''';
        errors.add(error);
      }
    }
  }
  if (errors.isNotEmpty) {
    foundError(errors);
  }
}

Future<CommandResult> _runFlutterAnalyze(String workingDirectory, {
  List<String> options = const <String>[],
  String? failureMessage,
}) async {
  return runCommand(
    flutter,
    <String>['analyze', ...options],
    workingDirectory: workingDirectory,
    failureMessage: failureMessage,
  );
}

// These files legitimately require executable permissions
const Set<String> kExecutableAllowlist = <String>{
  'bin/dart',
  'bin/flutter',
  'bin/flutter-dev',
  'bin/internal/update_dart_sdk.sh',

  'dev/bots/accept_android_sdk_licenses.sh',
  'dev/bots/codelabs_build_test.sh',
  'dev/bots/docs.sh',

  'dev/conductor/bin/conductor',
  'dev/conductor/bin/packages_autoroller',
  'dev/conductor/core/lib/src/proto/compile_proto.sh',

  'dev/customer_testing/ci.sh',

  'dev/integration_tests/flutter_gallery/tool/run_instrumentation_test.sh',

  'dev/integration_tests/ios_add2app_life_cycle/build_and_test.sh',

  'dev/integration_tests/deferred_components_test/download_assets.sh',
  'dev/integration_tests/deferred_components_test/run_release_test.sh',

  'dev/tools/gen_keycodes/bin/gen_keycodes',
  'dev/tools/repackage_gradle_wrapper.sh',
  'dev/tools/bin/engine_hash.sh',
  'dev/tools/format.sh',

  'packages/flutter_tools/bin/macos_assemble.sh',
  'packages/flutter_tools/bin/tool_backend.sh',
  'packages/flutter_tools/bin/xcode_backend.sh',
};

Future<void> _checkForNewExecutables() async {
  // 0b001001001
  const int executableBitMask = 0x49;
  final List<File> files = await _gitFiles(flutterRoot);
  final List<String> errors = <String>[];
  for (final File file in files) {
    final String relativePath = path.relative(
      file.path,
      from: flutterRoot,
    );
    final FileStat stat = file.statSync();
    final bool isExecutable = stat.mode & executableBitMask != 0x0;
    if (isExecutable && !kExecutableAllowlist.contains(relativePath)) {
      errors.add('$relativePath is executable: ${(stat.mode & 0x1FF).toRadixString(2)}');
    }
  }
  if (errors.isNotEmpty) {
    throw Exception(
      '${errors.join('\n')}\n'
      'found ${errors.length} unexpected executable file'
      '${errors.length == 1 ? '' : 's'}! If this was intended, you '
      'must add this file to kExecutableAllowlist in dev/bots/analyze.dart',
    );
  }
}

final RegExp _importPattern = RegExp(r'''^\s*import (['"])package:flutter/([^.]+)\.dart\1''');
final RegExp _importMetaPattern = RegExp(r'''^\s*import (['"])package:meta/meta\.dart\1''');

Future<Set<String>> _findFlutterDependencies(String srcPath, List<String> errors, { bool checkForMeta = false }) async {
  return _allFiles(srcPath, 'dart', minimumMatches: 1)
    .map<Set<String>>((File file) {
      final Set<String> result = <String>{};
      for (final String line in file.readAsLinesSync()) {
        Match? match = _importPattern.firstMatch(line);
        if (match != null) {
          result.add(match.group(2)!);
        }
        if (checkForMeta) {
          match = _importMetaPattern.firstMatch(line);
          if (match != null) {
            errors.add(
              '${file.path}\nThis package imports the ${yellow}meta$reset package.\n'
              'You should instead import the "foundation.dart" library.'
            );
          }
        }
      }
      return result;
    })
    .reduce((Set<String>? value, Set<String> element) {
      value ??= <String>{};
      value.addAll(element);
      return value;
    });
}

List<T>? _deepSearch<T>(Map<T, Set<T>> map, T start, [ Set<T>? seen ]) {
  if (map[start] == null) {
    return null; // We catch these separately.
  }

  for (final T key in map[start]!) {
    if (key == start) {
      continue; // we catch these separately
    }
    if (seen != null && seen.contains(key)) {
      return <T>[start, key];
    }
    final List<T>? result = _deepSearch<T>(
      map,
      key,
      <T>{
        if (seen == null) start else ...seen,
        key,
      },
    );
    if (result != null) {
      result.insert(0, start);
      // Only report the shortest chains.
      // For example a->b->a, rather than c->a->b->a.
      // Since we visit every node, we know the shortest chains are those
      // that start and end on the loop.
      if (result.first == result.last) {
        return result;
      }
    }
  }
  return null;
}

bool _isGeneratedPluginRegistrant(File file) {
  final String filename = path.basename(file.path);
  return !file.path.contains('.pub-cache')
      && (filename == 'GeneratedPluginRegistrant.java' ||
          filename == 'GeneratedPluginRegistrant.swift' ||
          filename == 'GeneratedPluginRegistrant.h' ||
          filename == 'GeneratedPluginRegistrant.m' ||
          filename == 'generated_plugin_registrant.dart' ||
          filename == 'generated_plugin_registrant.h' ||
          filename == 'generated_plugin_registrant.cc');
}
