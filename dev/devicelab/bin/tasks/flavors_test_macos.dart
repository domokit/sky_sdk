// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';
import 'dart:typed_data';

import 'package:flutter_devicelab/framework/devices.dart';
import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/ios.dart';
import 'package:flutter_devicelab/framework/task_result.dart';
import 'package:flutter_devicelab/framework/utils.dart';
import 'package:flutter_devicelab/tasks/integration_tests.dart';
import 'package:path/path.dart' as path;
import 'package:standard_message_codec/standard_message_codec.dart';

Future<void> main() async {
  deviceOperatingSystem = DeviceOperatingSystem.macos;
  await task(() async {
    await createFlavorsTest().call();
    await createIntegrationTestFlavorsTest().call();

    final String projectDir = '${flutterDirectory.path}/dev/integration_tests/flavors';
    final TaskResult installTestsResult = await inDirectory(projectDir, () async {
      await flutter('install', options: <String>['--flavor', 'paid', '-d', 'macos']);
      await flutter(
        'install',
        options: <String>['--flavor', 'paid', '--uninstall-only', '-d', 'macos'],
      );
      final StringBuffer stderr = StringBuffer();
      await evalFlutter(
        'build',
        canFail: true,
        stderr: stderr,
        options: <String>['macos', '--flavor', 'bogus'],
      );

      final Uint8List assetManifestFileData =
          File(
            path.join(
              projectDir,
              'build',
              'macos',
              'Build',
              'Products',
              'Debug-paid',
              'Debug Paid.app',
              'Contents',
              'Frameworks',
              'App.framework',
              'Resources',
              'flutter_assets',
              'AssetManifest.bin',
            ),
          ).readAsBytesSync();

      final Map<Object?, Object?> assetManifest =
          const StandardMessageCodec().decodeMessage(ByteData.sublistView(assetManifestFileData))
              as Map<Object?, Object?>;

      if (assetManifest.containsKey('assets/free/free.txt')) {
        return TaskResult.failure(
          'Expected the asset "assets/free/free.txt", which '
          ' was declared with a flavor of "free" to not be included in the asset bundle '
          ' because the --flavor was set to "paid".',
        );
      }

      if (!assetManifest.containsKey('assets/paid/paid.txt')) {
        return TaskResult.failure(
          'Expected the asset "assets/paid/paid.txt", which '
          ' was declared with a flavor of "paid" to be included in the asset bundle '
          ' because the --flavor was set to "paid".',
        );
      }

      final String stderrString = stderr.toString();
      print(stderrString);
      if (!stderrString.contains('The Xcode project defines schemes:')) {
        print(stderrString);
        return TaskResult.failure('Should not succeed with bogus flavor');
      }

      return TaskResult.success(null);
    });

    await _testFlavorWhenBuiltFromXcode(projectDir);

    return installTestsResult;
  });
}

Future<TaskResult> _testFlavorWhenBuiltFromXcode(String projectDir) async {
  await inDirectory(projectDir, () async {
    // This will put FLAVOR=free in the Flutter/ephemeral/Flutter-Generated.xcconfig file
    await flutter(
      'build',
      options: <String>['macos', '--config-only', '--debug', '--flavor', 'free'],
    );
  });

  final File generatedXcconfig = File(
    path.join(projectDir, 'macos/Flutter/ephemeral/Flutter-Generated.xcconfig'),
  );
  if (!generatedXcconfig.existsSync()) {
    throw TaskResult.failure('Unable to find Generated.xcconfig');
  }
  if (!generatedXcconfig.readAsStringSync().contains('FLAVOR=free')) {
    throw TaskResult.failure('Generated.xcconfig does not contain FLAVOR=free');
  }

  const String configuration = 'Debug-paid';
  const String productName = 'Debug Paid';
  const String buildDir = 'build/macos';
  final String appPath = '$projectDir/$buildDir/$configuration/$productName.app';

  // Delete app bundle before build to ensure checks below do not use previously
  // built bundle.
  final Directory appBundle = Directory(appPath);
  if (appBundle.existsSync()) {
    appBundle.deleteSync(recursive: true);
  }

  if (!await runXcodeBuild(
    platformDirectory: path.join(projectDir, 'macos'),
    destination: 'platform=macOS',
    testName: 'flavors_test_macos',
    configuration: configuration,
    scheme: 'paid',
    actions: <String>['clean', 'build'],
    extraOptions: <String>['BUILD_DIR=${path.join(projectDir, buildDir)}'],
    skipCodesign: true,
  )) {
    throw TaskResult.failure('Build failed');
  }

  if (!appBundle.existsSync()) {
    throw TaskResult.failure('App not found at $appPath');
  }

  if (!generatedXcconfig.readAsStringSync().contains('FLAVOR=free')) {
    throw TaskResult.failure('Generated.xcconfig does not contain FLAVOR=free');
  }

  // Despite FLAVOR=free being in the Generated.xcconfig, the flavor found in
  // the test should be "paid" because it was built with the "Debug-paid" configuration.
  return createFlavorsTest(
    extraOptions: <String>['--flavor', 'paid', '--use-application-binary=$appPath'],
  ).call();
}
