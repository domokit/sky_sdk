// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_devicelab/framework/apk_utils.dart';
import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/task_result.dart';
import 'package:flutter_devicelab/framework/utils.dart';
import 'package:path/path.dart' as path;

Future<void> main() async {
  final Iterable<String> baseAabFiles = <String>[
    'base/dex/classes.dex',
    'base/manifest/AndroidManifest.xml',
  ];
  final Iterable<String> flutterAabAssets = flutterAssets.map((String file) => 'base/$file');
  await task(() async {
    try {
      await runProjectTest((FlutterProject project) async {
        section('App bundle content for task bundleRelease without explicit target platform');

        await inDirectory(project.rootPath, () {
          return flutter('build', options: <String>['appbundle']);
        });

        final String releaseBundle = path.join(
          project.rootPath,
          'build',
          'app',
          'outputs',
          'bundle',
          'release',
          'app-release.aab',
        );
        checkCollectionContains<String>(<String>[
          ...baseAabFiles,
          ...flutterAabAssets,
          'base/lib/arm64-v8a/libapp.so',
          'base/lib/arm64-v8a/libflutter.so',
          'base/lib/armeabi-v7a/libapp.so',
          'base/lib/armeabi-v7a/libflutter.so',
        ], await getFilesInAppBundle(releaseBundle));
      });

      await runProjectTest((FlutterProject project) async {
        section('App bundle content using flavors without explicit target platform');
        // Add a few flavors.
        await project.addProductFlavors(<String>[
          'production',
          'staging',
          'development',
          'flavor_underscore', // https://github.com/flutter/flutter/issues/36067
        ]);
        // Build the production flavor in release mode.
        await inDirectory(project.rootPath, () {
          return flutter('build', options: <String>['appbundle', '--flavor', 'production']);
        });

        final String bundleFromGradlePath = path.join(
          project.rootPath,
          'build',
          'app',
          'outputs',
          'bundle',
          'productionRelease',
          'app-production-release.aab',
        );
        checkCollectionContains<String>(<String>[
          ...baseAabFiles,
          ...flutterAabAssets,
          'base/lib/arm64-v8a/libapp.so',
          'base/lib/arm64-v8a/libflutter.so',
          'base/lib/armeabi-v7a/libapp.so',
          'base/lib/armeabi-v7a/libflutter.so',
        ], await getFilesInAppBundle(bundleFromGradlePath));

        section('Build app bundle using the flutter tool - flavor: flavor_underscore');

        int exitCode = await inDirectory(project.rootPath, () {
          return flutter(
            'build',
            options: <String>['appbundle', '--flavor=flavor_underscore', '--verbose'],
          );
        });

        if (exitCode != 0) {
          throw TaskResult.failure('flutter build appbundle command exited with code: $exitCode');
        }

        final String flavorUnderscoreBundlePath = path.join(
          project.rootPath,
          'build',
          'app',
          'outputs',
          'bundle',
          'flavor_underscoreRelease',
          'app-flavor_underscore-release.aab',
        );
        checkCollectionContains<String>(<String>[
          ...baseAabFiles,
          ...flutterAabAssets,
          'base/lib/arm64-v8a/libapp.so',
          'base/lib/arm64-v8a/libflutter.so',
          'base/lib/armeabi-v7a/libapp.so',
          'base/lib/armeabi-v7a/libflutter.so',
        ], await getFilesInAppBundle(flavorUnderscoreBundlePath));

        section('Build app bundle using the flutter tool - flavor: production');

        exitCode = await inDirectory(project.rootPath, () {
          return flutter(
            'build',
            options: <String>['appbundle', '--flavor=production', '--verbose'],
          );
        });

        if (exitCode != 0) {
          throw TaskResult.failure('flutter build appbundle command exited with code: $exitCode');
        }

        final String productionBundlePath = path.join(
          project.rootPath,
          'build',
          'app',
          'outputs',
          'bundle',
          'productionRelease',
          'app-production-release.aab',
        );
        checkCollectionContains<String>(<String>[
          ...baseAabFiles,
          ...flutterAabAssets,
          'base/lib/arm64-v8a/libapp.so',
          'base/lib/arm64-v8a/libflutter.so',
          'base/lib/armeabi-v7a/libapp.so',
          'base/lib/armeabi-v7a/libflutter.so',
        ], await getFilesInAppBundle(productionBundlePath));
      });

      await runProjectTest((FlutterProject project) async {
        section('App bundle content for task bundleRelease with target platform = android-arm');

        await inDirectory(project.rootPath, () {
          return flutter('build', options: <String>['appbundle', '--target-platform=android-arm']);
        });

        final String releaseBundle = path.join(
          project.rootPath,
          'build',
          'app',
          'outputs',
          'bundle',
          'release',
          'app-release.aab',
        );

        final Iterable<String> bundleFiles = await getFilesInAppBundle(releaseBundle);
        checkCollectionContains<String>(<String>[
          ...baseAabFiles,
          ...flutterAabAssets,
          'base/lib/armeabi-v7a/libapp.so',
          'base/lib/armeabi-v7a/libflutter.so',
        ], bundleFiles);

        checkCollectionDoesNotContain<String>(<String>[
          'base/lib/arm64-v8a/libapp.so',
          'base/lib/arm64-v8a/libflutter.so',
        ], bundleFiles);
      });
      return TaskResult.success(null);
    } on TaskResult catch (taskResult) {
      return taskResult;
    } catch (e) {
      return TaskResult.failure(e.toString());
    }
  });
}
