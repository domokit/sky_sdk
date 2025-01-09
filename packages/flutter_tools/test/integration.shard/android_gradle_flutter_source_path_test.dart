// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/cache.dart';

import '../src/common.dart';
import 'test_data/deferred_components_project.dart';
import 'test_data/project.dart';
import 'test_utils.dart';

void main() {
  late Directory tempDir;

  setUp(() {
    Cache.flutterRoot = getFlutterRoot();
    tempDir = createResolvedTempDirectorySync('flutter_gradle_source_path_test.');
  });

  tearDown(() async {
    tryToDelete(tempDir);
  });

  test('gradle task builds without setting a source path in app/build.gradle', () async {
    final Project project = DeferredComponentsProject(
      MissingFlutterSourcePathDeferredComponentsConfig(),
    );

    final Directory exampleAppDir = tempDir.childDirectory('example');
    await project.setUpIn(exampleAppDir);

    // Run flutter build apk to build example project.
    final ProcessResult buildApkResult = processManager.runSync(<String>[
      flutterBin,
      ...getLocalEngineArguments(),
      'build',
      'apk',
      '--debug',
    ], workingDirectory: exampleAppDir.path);

    expect(buildApkResult, const ProcessResultMatcher());
  });
}

class MissingFlutterSourcePathDeferredComponentsConfig extends BasicDeferredComponentsConfig {
  final String _flutterSourcePath = '''
flutter {
    source '../..'
}
''';

  @override
  String get appBuild {
    if (!super.appBuild.contains(_flutterSourcePath)) {
      throw Exception('Flutter source path not found in original configuration!');
    }
    return super.appBuild.replaceAll(_flutterSourcePath, '');
  }
}
