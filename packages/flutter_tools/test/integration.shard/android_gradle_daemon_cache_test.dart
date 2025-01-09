// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:flutter_tools/src/base/io.dart';

import '../src/common.dart';
import 'test_utils.dart';

void main() {
  late Directory tempDir;

  setUp(() async {
    tempDir = createResolvedTempDirectorySync('gradle_daemon_test.');
  });

  tearDown(() async {
    tryToDelete(tempDir);
  });

  testWithoutContext(
    'gradle task succeeds when adding plugins with gradle daemon enabled',
    () async {
      final Directory appDir = tempDir.childDirectory('testapp');
      final Directory androidDir = appDir.childDirectory('android');

      // Create dummy plugins
      processManager.runSync(<String>[
        flutterBin,
        ...getLocalEngineArguments(),
        'create',
        '--template=plugin',
        '--platforms=android',
        'test_plugin_one',
      ], workingDirectory: tempDir.path);
      processManager.runSync(<String>[
        flutterBin,
        ...getLocalEngineArguments(),
        'create',
        '--template=plugin',
        '--platforms=android',
        'test_plugin_two',
      ], workingDirectory: tempDir.path);

      // Create a new flutter project.
      ProcessResult result = await processManager.run(<String>[
        flutterBin,
        'create',
        appDir.path,
        '--project-name=testapp',
      ], workingDirectory: tempDir.path);
      expect(result, const ProcessResultMatcher());

      // Enable gradle daemon for this project
      final File gradleProperties = androidDir.childFile('gradle.properties');
      gradleProperties.writeAsStringSync(r'''
org.gradle.daemon=true
''', mode: FileMode.append);

      // TODO(gustl22): Override with in 'gradle.properties' has no effect, set GRADLE_OPTS instead,
      //  see https://github.com/gradle/gradle/issues/19501
      final Map<String, String> envVars = <String, String>{
        'GRADLE_OPTS': '-Dorg.gradle.daemon=true',
      };

      // Stop gradle daemon
      result = await processManager.run(<String>[
        androidDir.childFile('gradlew').path,
        '--stop',
      ], workingDirectory: androidDir.path);
      expect(result, const ProcessResultMatcher());

      result = await processManager.run(
        <String>[flutterBin, 'pub', 'add', 'test_plugin_one', '--path', '../test_plugin_one'],
        workingDirectory: appDir.path,
        environment: envVars,
      );
      expect(result, const ProcessResultMatcher());

      // Build with gradle daemon
      result = await processManager.run(
        <String>[flutterBin, 'build', 'apk', '--debug'],
        workingDirectory: appDir.path,
        environment: envVars,
      );
      expect(result, const ProcessResultMatcher());

      // Add second plugin
      result = await processManager.run(
        <String>[flutterBin, 'pub', 'add', 'test_plugin_two', '--path', '../test_plugin_two'],
        workingDirectory: appDir.path,
        environment: envVars,
      );
      expect(result, const ProcessResultMatcher());

      // Build again with cached plugin through daemon
      result = await processManager.run(
        <String>[flutterBin, 'build', 'apk', '--debug'],
        workingDirectory: appDir.path,
        environment: envVars,
      );
      expect(result, const ProcessResultMatcher());
    },
  );
}
