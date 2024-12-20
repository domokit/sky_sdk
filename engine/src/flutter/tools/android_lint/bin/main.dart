// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:args/args.dart';
import 'package:path/path.dart' as path;
import 'package:process/process.dart';

const LocalProcessManager processManager = LocalProcessManager();

/// Runs the Android SDK Lint tool on flutter/shell/platform/android.
///
/// This script scans the flutter/shell/platform/android directory for Java
/// files to build a `project.xml` file.  This file is then passed to the lint
/// tool. If an `--html` flag is also passed in, HTML output is reqeusted in the
/// directory for the optional `--out` parameter, which defaults to
/// `lint_report`. Otherwise the output is printed to STDOUT.
///
/// The `--in` parameter may be specified to force this script to scan a
/// specific location for the engine repository, and expects to be given the
/// `src` directory that contains both `third_party` and `flutter`.
///
/// At the time of this writing, the Android Lint tool doesn't work well with
/// Java > 1.8.  This script will print a warning if you are not running
/// Java 1.8.
Future<void> main(List<String> args) async {
  final ArgParser argParser = setupOptions();
  final int exitCode = await runLint(argParser, argParser.parse(args));
  exit(exitCode);
}

Future<int> runLint(ArgParser argParser, ArgResults argResults) async {
  final String inArgument = argResults['in'] as String;
  final Directory androidDir = Directory(
    path.join(inArgument, 'flutter', 'shell', 'platform', 'android'),
  );
  if (!androidDir.existsSync()) {
    print(
      'This command must be run from the engine/src directory, '
      'or be passed that directory as the --in parameter.\n',
    );
    print(argParser.usage);
    return -1;
  }

  final Directory androidSdkDir = Directory(
    path.join(inArgument, 'flutter', 'third_party', 'android_tools', 'sdk'),
  );

  if (!androidSdkDir.existsSync()) {
    print(
      'The Android SDK for this engine is missing from the '
      'flutter/third_party/android_tools directory. Have you run gclient '
      'sync?\n',
    );
    print(argParser.usage);
    return -1;
  }

  final bool rebaseline = argResults['rebaseline'] as bool;
  if (rebaseline) {
    print('Removing previous baseline.xml...');
    final File baselineXml = File(baselineXmlPath);
    if (baselineXml.existsSync()) {
      await baselineXml.delete();
    }
  }
  print('Preparing project.xml...');
  final IOSink projectXml = File(projectXmlPath).openWrite();
  projectXml.write('''
<!-- THIS FILE IS GENERATED. PLEASE USE THE INCLUDED DART PROGRAM  WHICH -->
<!-- WILL AUTOMATICALLY FIND ALL .java FILES AND INCLUDE THEM HERE       -->
<project>
  <sdk dir="${androidSdkDir.path}" />
  <module name="FlutterEngine" android="true" library="true" compile-sdk-version="android-U">
  <manifest file="${path.join(androidDir.path, 'AndroidManifest.xml')}" />
''');
  for (final FileSystemEntity entity in androidDir.listSync(recursive: true)) {
    if (!entity.path.endsWith('.java')) {
      continue;
    }
    if (entity.path.endsWith('Test.java')) {
      continue;
    }
    projectXml.writeln('    <src file="${entity.path}" />');
  }

  projectXml.write('''
  </module>
</project>
''');
  await projectXml.close();
  print('Wrote project.xml, starting lint...');
  final List<String> lintArgs = <String>[
    path.join(androidSdkDir.path, 'cmdline-tools', 'latest', 'bin', 'lint'),
    '--project', projectXmlPath,
    '--compile-sdk-version', '34',
    '--showall',
    '--exitcode', // Set non-zero exit code on errors
    '-Wall',
    '-Werror',
    '--baseline',
    baselineXmlPath,
  ];
  final bool html = argResults['html'] as bool;
  if (html) {
    lintArgs.addAll(<String>['--html', argResults['out'] as String]);
  }
  final String javahome = getJavaHome(inArgument);
  print('Using JAVA_HOME=$javahome');
  final Process lintProcess = await processManager.start(
    lintArgs,
    environment: <String, String>{'JAVA_HOME': javahome},
  );
  lintProcess.stdout.pipe(stdout);
  lintProcess.stderr.pipe(stderr);
  return lintProcess.exitCode;
}

/// Prepares an [ArgParser] for this script.
ArgParser setupOptions() {
  final ArgParser argParser = ArgParser();
  argParser
    ..addOption(
      'in',
      help: 'The path to `engine/src`.',
      defaultsTo: path.relative(path.join(projectDir, '..', '..', '..')),
    )
    ..addFlag('help', help: 'Print usage of the command.', negatable: false)
    ..addFlag(
      'rebaseline',
      help:
          'Recalculates the baseline for errors and warnings '
          'in this project.',
      negatable: false,
    )
    ..addFlag(
      'html',
      help:
          'Creates an HTML output for this report instead of printing '
          'command line output.',
      negatable: false,
    )
    ..addOption(
      'out',
      help:
          'The path to write the generated HTML report. Ignored if '
          '--html is not also true.',
      defaultsTo: path.join(projectDir, 'lint_report'),
    );

  return argParser;
}

String getJavaHome(String src) {
  if (Platform.isMacOS) {
    return path.normalize('$src/flutter/third_party/java/openjdk/Contents/Home/');
  }
  return path.normalize('$src/flutter/third_party/java/openjdk/');
}

/// The root directory of this project.
String get projectDir => path.dirname(path.dirname(path.fromUri(Platform.script)));

/// The path to use for project.xml, which tells the linter where to find source
/// files.
String get projectXmlPath => path.join(projectDir, 'project.xml');

/// The path to use for baseline.xml, which tells the linter what errors or
/// warnings to ignore.
String get baselineXmlPath => path.join(projectDir, 'baseline.xml');
