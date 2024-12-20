// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io';

import 'package:flutter_devicelab/framework/apk_utils.dart';
import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/task_result.dart';
import 'package:flutter_devicelab/framework/utils.dart';
import 'package:path/path.dart' as path;

final String gradlew = Platform.isWindows ? 'gradlew.bat' : 'gradlew';
final String gradlewExecutable = Platform.isWindows ? '.\\$gradlew' : './$gradlew';

/// Tests that we respect storage proxy URLs in gradle dependencies.
Future<void> main() async {
  await task(() async {
    section('Find Java');

    final String? javaHome = await findJavaHome();
    if (javaHome == null) {
      return TaskResult.failure('Could not find Java');
    }
    print('\nUsing JAVA_HOME=$javaHome');

    section('Create project');
    await runProjectTest((FlutterProject flutterProject) async {
      await inDirectory(path.join(flutterProject.rootPath, 'android'), () async {
        section('Insert gradle testing script');
        final File buildFile = getAndroidBuildFile(
          path.join(flutterProject.rootPath, 'android', 'app'),
        );
        buildFile.writeAsStringSync(
          '''
tasks.register("printEngineMavenUrl") {
    doLast {
        project.repositories.forEach { repo ->
            if (repo.name == "maven") {
                repo as MavenArtifactRepository
                logger.quiet(repo.url.toString())
            }
        }
    }
}
          ''',
          mode: FileMode.append,
          flush: true,
        );

        section('Checking default maven URL');

        String gradleOutput = await eval(gradlewExecutable, <String>['printEngineMavenUrl', '-q']);
        const LineSplitter splitter = LineSplitter();
        List<String> outputLines = splitter.convert(gradleOutput);
        String mavenUrl = outputLines.last;
        print('Returned maven url: $mavenUrl');

        String realm =
            File(
              path.join(flutterDirectory.path, 'bin', 'internal', 'engine.realm'),
            ).readAsStringSync().trim();
        if (realm.isNotEmpty) {
          realm = '$realm/';
        }

        if (mavenUrl != 'https://storage.googleapis.com/${realm}download.flutter.io') {
          throw TaskResult.failure(
            'Expected Android engine maven dependency URL to '
            'resolve to https://storage.googleapis.com/${realm}download.flutter.io. Got '
            '$mavenUrl instead',
          );
        }

        section('Checking overridden maven URL');
        gradleOutput = await eval(
          gradlewExecutable,
          <String>['printEngineMavenUrl', '-q'],
          environment: <String, String>{'FLUTTER_STORAGE_BASE_URL': 'https://my.special.proxy'},
        );
        outputLines = splitter.convert(gradleOutput);
        mavenUrl = outputLines.last;

        if (mavenUrl != 'https://my.special.proxy/${realm}download.flutter.io') {
          throw TaskResult.failure(
            'Expected overridden Android engine maven '
            'dependency URL to resolve to proxy location '
            'https://my.special.proxy/${realm}download.flutter.io. Got '
            '$mavenUrl instead',
          );
        }
      });
    });
    return TaskResult.success(null);
  });
}
