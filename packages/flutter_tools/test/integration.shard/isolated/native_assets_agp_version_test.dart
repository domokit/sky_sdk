// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:file/file.dart';
import 'package:file_testing/file_testing.dart';

import '../../src/common.dart';
import '../test_utils.dart' show flutterBin, platform;
import '../transition_test_utils.dart';
import 'native_assets_test_utils.dart';

const String packageName = 'package_with_native_assets';

/// The AGP versions to run these tests against.
final List<String> agpVersions = <String>['8.4.0'];

/// The build modes to target for each flutter command that supports passing
/// a build mode.
///
/// The flow of compiling kernel as well as bundling dylibs can differ based on
/// build mode, so we should cover this.
const List<String> buildModes = <String>['debug', 'profile', 'release'];

void main() {
  if (!platform.isMacOS && !platform.isLinux && !platform.isWindows) {
    // TODO(dacoharkes): Implement Fuchsia. https://github.com/flutter/flutter/issues/129757
    return;
  }

  setUpAll(() {
    processManager.runSync(<String>[flutterBin, 'config', '--enable-native-assets']);
  });

  for (final String agpVersion in agpVersions) {
    for (final String buildMode in buildModes) {
      testWithoutContext(
        'flutter build apk with native assets with build mode $buildMode and multiple flavors on AGP $agpVersion',
        () async {
          await inTempDir((Directory tempDirectory) async {
            final Directory packageDirectory = await createTestProject(packageName, tempDirectory);
            final Directory exampleDirectory = packageDirectory.childDirectory('example');

            File appBuildGradleFile = exampleDirectory
                .childDirectory('android')
                .childDirectory('app')
                .childFile('build.gradle');
            if (!appBuildGradleFile.existsSync()) {
              appBuildGradleFile = exampleDirectory
                  .childDirectory('android')
                  .childDirectory('app')
                  .childFile('build.gradle.kts');
            }

            final File settingsGradleFile = exampleDirectory
                .childDirectory('android')
                .childFile('settings.gradle.kts');

            expect(appBuildGradleFile, exists);
            expect(settingsGradleFile, exists);

            // Use expected AGP version.
            final String settingsGradle = settingsGradleFile.readAsStringSync();

            final RegExp androidPluginRegExp = RegExp(
              r'id\("com\.android\.application"\)\s+version\s+"([^"]+)"\s+apply\s+false',
            );
            expect(androidPluginRegExp.firstMatch(settingsGradle), isNotNull);

            final String newSettingsGradle = settingsGradle.replaceAll(
              androidPluginRegExp,
              'id("com.android.application") version "$agpVersion" apply false',
            );
            settingsGradleFile.writeAsStringSync(newSettingsGradle);

            // Use Android app with multiple flavors.
            final String appBuildGradle = appBuildGradleFile.readAsStringSync().replaceAll(
              '\r\n',
              '\n',
            );
            final RegExp buildTypesBlockRegExp = RegExp(
              r'buildTypes {\n[ \t]+release {((.|\n)*)\n[ \t]+}\n[ \t]+}',
            );
            final String buildTypesBlock = buildTypesBlockRegExp.firstMatch(appBuildGradle)![0]!;
            final String appBuildGradleSegmentDefiningFlavors = '''
    $buildTypesBlock

    flavorDimensions += "mode"

    productFlavors {
        create("flavorOne") {
            dimension = "mode"
        }
        create("flavorTwo") {
            dimension = "mode"
        }
        create("flavorThree") {
            dimension = "mode"
        }
    }
''';
            final String newAppBuildGradle = appBuildGradle.replaceFirst(
              buildTypesBlockRegExp,
              appBuildGradleSegmentDefiningFlavors,
            );
            appBuildGradleFile.writeAsStringSync(newAppBuildGradle);

            final ProcessResult result = processManager.runSync(<String>[
              flutterBin,
              'build',
              'apk',
              '--flavor',
              'flavorOne',
              '--$buildMode',
            ], workingDirectory: exampleDirectory.path);
            if (result.exitCode != 0) {
              throw Exception(
                'flutter build failed: ${result.exitCode}\n${result.stderr}\n${result.stdout}',
              );
            }

            // Test that the native libraries are included as expected.
            final Directory nativeAssetsDir = exampleDirectory
                .childDirectory('build')
                .childDirectory('native_assets')
                .childDirectory('android')
                .childDirectory('jniLibs')
                .childDirectory('lib');
            expect(nativeAssetsDir, exists);

            // We expect one subdirectory for each Android architecture.
            expect(nativeAssetsDir.listSync().length, 4);
            expect(nativeAssetsDir..childDirectory('armeabi-v7a'), exists);
            expect(nativeAssetsDir..childDirectory('x86'), exists);
            expect(nativeAssetsDir..childDirectory('arm64-v8a'), exists);
            expect(nativeAssetsDir..childDirectory('x86_64'), exists);
          });
        },
      );
    }
  }
}
