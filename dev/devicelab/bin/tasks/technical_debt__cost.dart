// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io';

import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/task_result.dart';
import 'package:flutter_devicelab/framework/utils.dart';
import 'package:path/path.dart' as path;

// the numbers below are prime, so that the totals don't seem round. :-)
const double todoCost = 1009.0; // about two average SWE days, in dollars
const double ignoreCost = 2003.0; // four average SWE days, in dollars
const double pythonCost = 3001.0; // six average SWE days, in dollars
const double skipCost =
    2473.0; // 20 hours: 5 to fix the issue we're ignoring, 15 to fix the bugs we missed because the test was off
const double ignoreForFileCost = 2477.0; // similar thinking as skipCost
const double asDynamicCost = 2011.0; // a few days to refactor the code.
const double deprecationCost = 233.0; // a few hours to remove the old code.
const double legacyDeprecationCost = 9973.0; // a couple of weeks.

final RegExp todoPattern = RegExp(r'(?://|#) *TODO');
final RegExp ignorePattern = RegExp(r'// *ignore:');
final RegExp ignoreForFilePattern = RegExp(r'// *ignore_for_file:');
final RegExp asDynamicPattern = RegExp(r'\bas dynamic\b');
final RegExp deprecationPattern = RegExp(r'^ *@[dD]eprecated');
const Pattern globalsPattern = 'globals.';
const String legacyDeprecationPattern = '// flutter_ignore: deprecation_syntax, https';

Future<double> findCostsForFile(File file) async {
  if (path.extension(file.path) == '.py') {
    return pythonCost;
  }
  if (path.extension(file.path) != '.dart' &&
      path.extension(file.path) != '.yaml' &&
      path.extension(file.path) != '.sh') {
    return 0.0;
  }
  final bool isTest = file.path.endsWith('_test.dart');
  double total = 0.0;
  for (final String line in await file.readAsLines()) {
    if (line.contains(todoPattern)) {
      total += todoCost;
    }
    if (line.contains(ignorePattern)) {
      total += ignoreCost;
    }
    if (line.contains(ignoreForFilePattern)) {
      total += ignoreForFileCost;
    }
    if (!isTest && line.contains(asDynamicPattern)) {
      total += asDynamicCost;
    }
    if (line.contains(deprecationPattern)) {
      total += deprecationCost;
    }
    if (line.contains(legacyDeprecationPattern)) {
      total += legacyDeprecationCost;
    }
    if (isTest && line.contains('skip:') && !line.contains('[intended]')) {
      total += skipCost;
    }
  }
  return total;
}

Future<int> findGlobalsForFile(File file) async {
  if (path.extension(file.path) != '.dart') {
    return 0;
  }
  int total = 0;
  for (final String line in await file.readAsLines()) {
    if (line.contains(globalsPattern)) {
      total += 1;
    }
  }
  return total;
}

Future<double> findCostsForRepo() async {
  final Process git = await startProcess('git', <String>[
    'ls-files',
    '--exclude',
    'engine',
    '--full-name',
    flutterDirectory.path,
  ], workingDirectory: flutterDirectory.path);
  double total = 0.0;
  await for (final String entry in git.stdout
      .transform<String>(utf8.decoder)
      .transform<String>(const LineSplitter())) {
    total += await findCostsForFile(File(path.join(flutterDirectory.path, entry)));
  }
  final int gitExitCode = await git.exitCode;
  if (gitExitCode != 0) {
    throw Exception('git exit with unexpected error code $gitExitCode');
  }
  return total;
}

Future<int> findGlobalsForTool() async {
  final Process git = await startProcess('git', <String>[
    'ls-files',
    '--full-name',
    path.join(flutterDirectory.path, 'packages', 'flutter_tools'),
  ], workingDirectory: flutterDirectory.path);
  int total = 0;
  await for (final String entry in git.stdout
      .transform<String>(utf8.decoder)
      .transform<String>(const LineSplitter())) {
    total += await findGlobalsForFile(File(path.join(flutterDirectory.path, entry)));
  }
  final int gitExitCode = await git.exitCode;
  if (gitExitCode != 0) {
    throw Exception('git exit with unexpected error code $gitExitCode');
  }
  return total;
}

Future<int> countDependencies() async {
  final List<String> lines = (await evalFlutter(
    'update-packages',
    options: <String>['--transitive-closure'],
  )).split('\n');
  final int count = lines.where((String line) => line.contains('->')).length;
  if (count < 2) {
    throw Exception(
      '"flutter update-packages --transitive-closure" returned bogus output:\n${lines.join("\n")}',
    );
  }
  return count;
}

Future<int> countConsumerDependencies() async {
  final List<String> lines = (await evalFlutter(
    'update-packages',
    options: <String>['--transitive-closure', '--consumer-only'],
  )).split('\n');
  final int count = lines.where((String line) => line.contains('->')).length;
  if (count < 2) {
    throw Exception(
      '"flutter update-packages --transitive-closure" returned bogus output:\n${lines.join("\n")}',
    );
  }
  return count;
}

const String _kCostBenchmarkKey = 'technical_debt_in_dollars';
const String _kNumberOfDependenciesKey = 'dependencies_count';
const String _kNumberOfConsumerDependenciesKey = 'consumer_dependencies_count';
const String _kNumberOfFlutterToolGlobals = 'flutter_tool_globals_count';

Future<void> main() async {
  await task(() async {
    return TaskResult.success(
      <String, dynamic>{
        _kCostBenchmarkKey: await findCostsForRepo(),
        _kNumberOfDependenciesKey: await countDependencies(),
        _kNumberOfConsumerDependenciesKey: await countConsumerDependencies(),
        _kNumberOfFlutterToolGlobals: await findGlobalsForTool(),
      },
      benchmarkScoreKeys: <String>[
        _kCostBenchmarkKey,
        _kNumberOfDependenciesKey,
        _kNumberOfConsumerDependenciesKey,
        _kNumberOfFlutterToolGlobals,
      ],
    );
  });
}
