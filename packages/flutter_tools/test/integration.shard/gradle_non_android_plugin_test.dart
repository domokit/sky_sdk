// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/io.dart';

import '../src/common.dart';
import 'test_utils.dart';

void main() {
  late Directory tempDir;

  setUp(() {
    tempDir = createResolvedTempDirectorySync('flutter_plugin_test.');
  });

  tearDown(() {
    tryToDelete(tempDir);
  });

  testWithoutContext(
    'Flutter app that depends on a non-Android plugin can still build for Android',
    () {
      processManager.runSync(<String>[
        flutterBin,
        ...getLocalEngineArguments(),
        'create',
        '-t',
        'plugin',
        '--platforms=ios,android',
        'ios_only',
      ], workingDirectory: tempDir.path);

      // Delete plugin's Android folder
      final Directory projectRoot = tempDir.childDirectory('ios_only');
      projectRoot.childDirectory('android').deleteSync(recursive: true);

      // Update pubspec.yaml to iOS only plugin
      final File pubspecFile = projectRoot.childFile('pubspec.yaml');
      final String pubspecString = pubspecFile.readAsStringSync();

      final StringBuffer iosOnlyPubspec = StringBuffer();
      bool inAndroidSection = false;
      const String pluginPlatformIndentation = '      ';
      for (final String line in pubspecString.split('\n')) {
        // Skip everything in the Android section of the plugin platforms list.
        if (line.startsWith('${pluginPlatformIndentation}android:')) {
          inAndroidSection = true;
          continue;
        }
        if (inAndroidSection) {
          if (line.startsWith('$pluginPlatformIndentation  ')) {
            continue;
          } else {
            inAndroidSection = false;
          }
        }
        iosOnlyPubspec.write('$line\n');
      }

      pubspecFile.writeAsStringSync(iosOnlyPubspec.toString());

      // Build example APK
      final Directory exampleDir = projectRoot.childDirectory('example');

      final ProcessResult result = processManager.runSync(<String>[
        flutterBin,
        ...getLocalEngineArguments(),
        'build',
        'apk',
        '--target-platform',
        'android-arm',
        '--verbose',
      ], workingDirectory: exampleDir.path);

      expect(result, const ProcessResultMatcher());

      final String exampleAppApk = fileSystem.path.join(
        exampleDir.path,
        'build',
        'app',
        'outputs',
        'apk',
        'release',
        'app-release.apk',
      );
      expect(fileSystem.file(exampleAppApk), exists);
    },
  );
}
