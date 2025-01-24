// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This test exercises the embedding of the native assets mapping in dill files.
// An initial dill file is created by `flutter assemble` and used for running
// the application. This dill must contain the mapping.
// When doing hot reload, this mapping must stay in place.
// When doing a hot restart, a new dill file is pushed. This dill file must also
// contain the native assets mapping.
// When doing a hot reload, this mapping must stay in place.

@Timeout(Duration(minutes: 10))
library;

import 'dart:convert';
import 'dart:io';

import 'package:file/file.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/os.dart';
import 'package:native_assets_cli/code_assets_builder.dart';

import '../../src/common.dart';
import '../test_utils.dart' show fileSystem, flutterBin, platform;
import '../transition_test_utils.dart';
import 'native_assets_test_utils.dart';

final String hostOs = platform.operatingSystem;

final List<String> devices = <String>['flutter-tester', hostOs];

final List<String> buildSubcommands = <String>[hostOs, if (hostOs == 'macos') 'ios', 'apk'];

final List<String> add2appBuildSubcommands = <String>[
  if (hostOs == 'macos') ...<String>['macos-framework', 'ios-framework'],
];

/// The build modes to target for each flutter command that supports passing
/// a build mode.
///
/// The flow of compiling kernel as well as bundling dylibs can differ based on
/// build mode, so we should cover this.
const List<String> buildModes = <String>['debug', 'profile', 'release'];

const String packageName = 'package_with_native_assets';

const String exampleAppName = '${packageName}_example';

void main() {
  if (!platform.isMacOS && !platform.isLinux && !platform.isWindows) {
    // TODO(dacoharkes): Implement Fuchsia. https://github.com/flutter/flutter/issues/129757
    return;
  }

  setUpAll(() {
    processManager.runSync(<String>[flutterBin, 'config', '--enable-native-assets']);
  });

  for (final String device in devices) {
    for (final String buildMode in buildModes) {
      if (device == 'flutter-tester' && buildMode != 'debug') {
        continue;
      }
      final String hotReload = buildMode == 'debug' ? ' hot reload and hot restart' : '';
      testWithoutContext('flutter run$hotReload with native assets $device $buildMode', () async {
        await inTempDir((Directory tempDirectory) async {
          final Directory packageDirectory = await createTestProject(packageName, tempDirectory);
          final Directory exampleDirectory = packageDirectory.childDirectory('example');

          final ProcessTestResult result = await runFlutter(
            <String>['run', '-d$device', '--$buildMode'],
            exampleDirectory.path,
            <Transition>[
              Multiple.contains(
                <Pattern>['Flutter run key commands.'],
                handler: (String line) {
                  if (buildMode == 'debug') {
                    // Do a hot reload diff on the initial dill file.
                    return 'r';
                  } else {
                    // No hot reload and hot restart in release mode.
                    return 'q';
                  }
                },
              ),
              if (buildMode == 'debug') ...<Transition>[
                Barrier.contains('Performing hot reload...', logging: true),
                Multiple(
                  <Pattern>[RegExp('Reloaded .*')],
                  handler: (String line) {
                    // Do a hot restart, pushing a new complete dill file.
                    return 'R';
                  },
                ),
                Barrier.contains('Performing hot restart...'),
                Multiple(
                  <Pattern>[RegExp('Restarted application .*')],
                  handler: (String line) {
                    // Do another hot reload, pushing a diff to the second dill file.
                    return 'r';
                  },
                ),
                Barrier.contains('Performing hot reload...', logging: true),
                Multiple(
                  <Pattern>[RegExp('Reloaded .*')],
                  handler: (String line) {
                    return 'q';
                  },
                ),
              ],
              Barrier.contains('Application finished.'),
            ],
            logging: false,
          );
          if (result.exitCode != 0) {
            throw Exception(
              'flutter run failed: ${result.exitCode}\n${result.stderr}\n${result.stdout}',
            );
          }
          final String stdout = result.stdout.join('\n');
          // Check that we did not fail to resolve the native function in the
          // dynamic library.
          expect(
            stdout,
            isNot(contains("Invalid argument(s): Couldn't resolve native function 'sum'")),
          );
          // And also check that we did not have any other exceptions that might
          // shadow the exception we would have gotten.
          expect(stdout, isNot(contains('EXCEPTION CAUGHT BY WIDGETS LIBRARY')));

          switch (device) {
            case 'macos':
              expectDylibIsBundledMacOS(exampleDirectory, buildMode);
            case 'linux':
              expectDylibIsBundledLinux(exampleDirectory, buildMode);
            case 'windows':
              expectDylibIsBundledWindows(exampleDirectory, buildMode);
          }
          if (device == hostOs) {
            expectCCompilerIsConfigured(exampleDirectory);
          }
        });
      });
    }
  }

  testWithoutContext('flutter test with native assets', () async {
    await inTempDir((Directory tempDirectory) async {
      final Directory packageDirectory = await createTestProject(packageName, tempDirectory);

      final ProcessTestResult result = await runFlutter(
        <String>['test'],
        packageDirectory.path,
        <Transition>[Barrier(RegExp('.* All tests passed!'))],
        logging: false,
      );
      if (result.exitCode != 0) {
        throw Exception(
          'flutter test failed: ${result.exitCode}\n${result.stderr}\n${result.stdout}',
        );
      }
    });
  });

  for (final String buildSubcommand in buildSubcommands) {
    for (final String buildMode in buildModes) {
      testWithoutContext(
        'flutter build $buildSubcommand with native assets $buildMode',
        () async {
          await inTempDir((Directory tempDirectory) async {
            final Directory packageDirectory = await createTestProject(packageName, tempDirectory);
            final Directory exampleDirectory = packageDirectory.childDirectory('example');

            final ProcessResult result = processManager.runSync(<String>[
              flutterBin,
              'build',
              buildSubcommand,
              '--$buildMode',
              if (buildSubcommand == 'ios') '--no-codesign',
            ], workingDirectory: exampleDirectory.path);
            if (result.exitCode != 0) {
              throw Exception(
                'flutter build failed: ${result.exitCode}\n${result.stderr}\n${result.stdout}',
              );
            }

            switch (buildSubcommand) {
              case 'macos':
                expectDylibIsBundledMacOS(exampleDirectory, buildMode);
                expectDylibIsCodeSignedMacOS(exampleDirectory, buildMode);
              case 'ios':
                expectDylibIsBundledIos(exampleDirectory, buildMode);
              case 'linux':
                expectDylibIsBundledLinux(exampleDirectory, buildMode);
              case 'windows':
                expectDylibIsBundledWindows(exampleDirectory, buildMode);
              case 'apk':
                expectDylibIsBundledAndroid(exampleDirectory, buildMode);
            }
            expectCCompilerIsConfigured(exampleDirectory);
          });
        },
        tags: <String>['flutter-build-apk'],
      );
    }
  }

  for (final String add2appBuildSubcommand in add2appBuildSubcommands) {
    testWithoutContext('flutter build $add2appBuildSubcommand with native assets', () async {
      await inTempDir((Directory tempDirectory) async {
        final Directory packageDirectory = await createTestProject(packageName, tempDirectory);
        final Directory exampleDirectory = packageDirectory.childDirectory('example');

        final ProcessResult result = processManager.runSync(<String>[
          flutterBin,
          'build',
          add2appBuildSubcommand,
        ], workingDirectory: exampleDirectory.path);
        if (result.exitCode != 0) {
          throw Exception(
            'flutter build failed: ${result.exitCode}\n${result.stderr}\n${result.stdout}',
          );
        }

        for (final String buildMode in buildModes) {
          expectDylibIsBundledWithFrameworks(
            exampleDirectory,
            buildMode,
            add2appBuildSubcommand.replaceAll('-framework', ''),
          );
        }
        expectCCompilerIsConfigured(exampleDirectory);
      });
    });
  }
}

void expectDylibIsCodeSignedMacOS(Directory appDirectory, String buildMode) {
  final Directory appBundle = appDirectory.childDirectory(
    'build/$hostOs/Build/Products/${buildMode.upperCaseFirst()}/$exampleAppName.app',
  );
  final Directory frameworksFolder = appBundle.childDirectory('Contents/Frameworks');
  expect(frameworksFolder, exists);
  const String frameworkName = packageName;
  final Directory frameworkDir = frameworksFolder.childDirectory('$frameworkName.framework');
  final ProcessResult codesign = processManager.runSync(<String>[
    'codesign',
    '-dv',
    frameworkDir.absolute.path,
  ]);
  expect(codesign.exitCode, 0);

  // Expect adhoc signature, but not linker-signed (which would mean no code-signing happened after linking).
  final List<String> lines = codesign.stderr.toString().split('\n');
  final bool isLinkerSigned = lines.any((String line) => line.contains('linker-signed'));
  final bool isAdhoc = lines.any((String line) => line.contains('Signature=adhoc'));
  expect(isAdhoc, isTrue);
  expect(isLinkerSigned, isFalse);
}

/// For `flutter build` we can't easily test whether running the app works.
/// Check that we have the dylibs in the app.
void expectDylibIsBundledMacOS(Directory appDirectory, String buildMode) {
  final Directory appBundle = appDirectory.childDirectory(
    'build/$hostOs/Build/Products/${buildMode.upperCaseFirst()}/$exampleAppName.app',
  );
  expect(appBundle, exists);
  final Directory frameworksFolder = appBundle.childDirectory('Contents/Frameworks');
  expect(frameworksFolder, exists);

  // MyFramework.framework/
  //   MyFramework  -> Versions/Current/MyFramework
  //   Resources    -> Versions/Current/Resources
  //   Versions/
  //     A/
  //       MyFramework
  //       Resources/
  //         Info.plist
  //     Current  -> A
  const String frameworkName = packageName;
  final Directory frameworkDir = frameworksFolder.childDirectory('$frameworkName.framework');
  final Directory versionsDir = frameworkDir.childDirectory('Versions');
  final Directory versionADir = versionsDir.childDirectory('A');
  final Directory resourcesDir = versionADir.childDirectory('Resources');
  expect(resourcesDir, exists);
  final File dylibFile = versionADir.childFile(frameworkName);
  expect(dylibFile, exists);
  final Link currentLink = versionsDir.childLink('Current');
  expect(currentLink, exists);
  expect(currentLink.resolveSymbolicLinksSync(), versionADir.path);
  final Link resourcesLink = frameworkDir.childLink('Resources');
  expect(resourcesLink, exists);
  expect(resourcesLink.resolveSymbolicLinksSync(), resourcesDir.path);
  final Link dylibLink = frameworkDir.childLink(frameworkName);
  expect(dylibLink, exists);
  expect(dylibLink.resolveSymbolicLinksSync(), dylibFile.path);
  final String infoPlist = resourcesDir.childFile('Info.plist').readAsStringSync();
  expect(infoPlist, '''
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
	<key>CFBundleDevelopmentRegion</key>
	<string>en</string>
	<key>CFBundleExecutable</key>
	<string>package_with_native_assets</string>
	<key>CFBundleIdentifier</key>
	<string>io.flutter.flutter.native-assets.package-with-native-assets</string>
	<key>CFBundleInfoDictionaryVersion</key>
	<string>6.0</string>
	<key>CFBundleName</key>
	<string>package_with_native_assets</string>
	<key>CFBundlePackageType</key>
	<string>FMWK</string>
	<key>CFBundleShortVersionString</key>
	<string>1.0</string>
	<key>CFBundleSignature</key>
	<string>????</string>
	<key>CFBundleVersion</key>
	<string>1.0</string>
</dict>
</plist>''');
}

void expectDylibIsBundledIos(Directory appDirectory, String buildMode) {
  final Directory appBundle = appDirectory.childDirectory(
    'build/ios/${buildMode.upperCaseFirst()}-iphoneos/Runner.app',
  );
  expect(appBundle, exists);
  final Directory frameworksFolder = appBundle.childDirectory('Frameworks');
  expect(frameworksFolder, exists);
  const String frameworkName = packageName;
  final File dylib = frameworksFolder
      .childDirectory('$frameworkName.framework')
      .childFile(frameworkName);
  expect(dylib, exists);
  final String infoPlist =
      frameworksFolder
          .childDirectory('$frameworkName.framework')
          .childFile('Info.plist')
          .readAsStringSync();
  expect(infoPlist, '''
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
	<key>CFBundleDevelopmentRegion</key>
	<string>en</string>
	<key>CFBundleExecutable</key>
	<string>package_with_native_assets</string>
	<key>CFBundleIdentifier</key>
	<string>io.flutter.flutter.native-assets.package-with-native-assets</string>
	<key>CFBundleInfoDictionaryVersion</key>
	<string>6.0</string>
	<key>CFBundleName</key>
	<string>package_with_native_assets</string>
	<key>CFBundlePackageType</key>
	<string>FMWK</string>
	<key>CFBundleShortVersionString</key>
	<string>1.0</string>
	<key>CFBundleSignature</key>
	<string>????</string>
	<key>CFBundleVersion</key>
	<string>1.0</string>
	<key>MinimumOSVersion</key>
	<string>12.0</string>
</dict>
</plist>''');
}

/// Checks that dylibs are bundled.
///
/// Sample path: build/linux/x64/release/bundle/lib/libmy_package.so
void expectDylibIsBundledLinux(Directory appDirectory, String buildMode) {
  // Linux does not support cross compilation, so always only check current architecture.
  final String architecture = Architecture.current.name;
  final Directory appBundle = appDirectory
      .childDirectory('build')
      .childDirectory(hostOs)
      .childDirectory(architecture)
      .childDirectory(buildMode)
      .childDirectory('bundle');
  expect(appBundle, exists);
  final Directory dylibsFolder = appBundle.childDirectory('lib');
  expect(dylibsFolder, exists);
  final File dylib = dylibsFolder.childFile(OS.linux.dylibFileName(packageName));
  expect(dylib, exists);
}

/// Checks that dylibs are bundled.
///
/// Sample path: build\windows\x64\runner\Debug\my_package_example.exe
void expectDylibIsBundledWindows(Directory appDirectory, String buildMode) {
  // Linux does not support cross compilation, so always only check current architecture.
  final String architecture = Architecture.current.name;
  final Directory appBundle = appDirectory
      .childDirectory('build')
      .childDirectory(hostOs)
      .childDirectory(architecture)
      .childDirectory('runner')
      .childDirectory(buildMode.upperCaseFirst());
  expect(appBundle, exists);
  final File dylib = appBundle.childFile(OS.windows.dylibFileName(packageName));
  expect(dylib, exists);
}

void expectDylibIsBundledAndroid(Directory appDirectory, String buildMode) {
  final File apk = appDirectory
      .childDirectory('build')
      .childDirectory('app')
      .childDirectory('outputs')
      .childDirectory('flutter-apk')
      .childFile('app-$buildMode.apk');
  expect(apk, exists);
  final OperatingSystemUtils osUtils = OperatingSystemUtils(
    fileSystem: fileSystem,
    logger: BufferLogger.test(),
    platform: platform,
    processManager: processManager,
  );
  final Directory apkUnzipped = appDirectory.childDirectory('apk-unzipped');
  apkUnzipped.createSync();
  osUtils.unzip(apk, apkUnzipped);
  final Directory lib = apkUnzipped.childDirectory('lib');
  for (final String arch in <String>['arm64-v8a', 'armeabi-v7a', 'x86_64']) {
    final Directory archDir = lib.childDirectory(arch);
    expect(archDir, exists);
    // The dylibs should be next to the flutter and app so.
    expect(archDir.childFile('libflutter.so'), exists);
    if (buildMode != 'debug') {
      expect(archDir.childFile('libapp.so'), exists);
    }
    final File dylib = archDir.childFile(OS.android.dylibFileName(packageName));
    expect(dylib, exists);
  }
}

/// For `flutter build` we can't easily test whether running the app works.
/// Check that we have the dylibs in the app.
void expectDylibIsBundledWithFrameworks(Directory appDirectory, String buildMode, String os) {
  final Directory frameworksFolder = appDirectory.childDirectory(
    'build/$os/framework/${buildMode.upperCaseFirst()}',
  );
  expect(frameworksFolder, exists);
  const String frameworkName = packageName;
  final File dylib = frameworksFolder
      .childDirectory('$frameworkName.framework')
      .childFile(frameworkName);
  expect(dylib, exists);
}

/// Check that the native assets are built with the C Compiler that Flutter uses.
///
/// This inspects the build configuration to see if the C compiler was configured.
void expectCCompilerIsConfigured(Directory appDirectory) {
  final Directory nativeAssetsBuilderDir = appDirectory.childDirectory(
    '.dart_tool/native_assets_builder/$packageName/',
  );
  for (final Directory subDir in nativeAssetsBuilderDir.listSync().whereType<Directory>()) {
    // We only want to look at build/link hook invocation directories. The
    // `/shared/*` directory allows the individual hooks to store data that is
    // reusable across different build/link configurations.
    if (subDir.path.endsWith('shared')) {
      continue;
    }

    final File inputFile = subDir.childFile('input.json');
    expect(inputFile, exists);
    final Map<String, Object?> inputContents =
        json.decode(inputFile.readAsStringSync()) as Map<String, Object?>;
    final BuildInput input = BuildInput(inputContents);
    final BuildConfig config = input.config;
    if (!config.buildCodeAssets) {
      continue;
    }
    expect(config.code.cCompiler?.compiler, isNot(isNull));
  }
}

extension on String {
  String upperCaseFirst() {
    return replaceFirst(this[0], this[0].toUpperCase());
  }
}
