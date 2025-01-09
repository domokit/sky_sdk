// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/utils.dart';
import 'package:flutter_tools/src/build_info.dart';

import '../integration.shard/test_utils.dart';
import '../src/common.dart';

void main() {
  group(
    'iOS app validation',
    () {
      late String flutterRoot;
      late Directory pluginRoot;
      late String projectRoot;
      late String flutterBin;
      late Directory tempDir;
      late File hiddenFile;

      setUpAll(() {
        flutterRoot = getFlutterRoot();
        tempDir = createResolvedTempDirectorySync('ios_content_validation.');
        flutterBin = fileSystem.path.join(flutterRoot, 'bin', 'flutter');

        final Directory xcframeworkArtifact = fileSystem.directory(
          fileSystem.path.join(
            flutterRoot,
            'bin',
            'cache',
            'artifacts',
            'engine',
            'ios',
            'Flutter.xcframework',
          ),
        );

        // Pre-cache iOS engine Flutter.xcframework artifacts.
        ProcessResult result = processManager.runSync(<String>[
          flutterBin,
          ...getLocalEngineArguments(),
          'precache',
          '--ios',
        ], workingDirectory: tempDir.path);
        expect(result, const ProcessResultMatcher());

        // Pretend the SDK was on an external drive with stray "._" files in the xcframework
        hiddenFile = xcframeworkArtifact.childFile('._Info.plist')..createSync();

        // Test a plugin example app to allow plugins validation.
        result = processManager.runSync(<String>[
          flutterBin,
          ...getLocalEngineArguments(),
          'create',
          '--verbose',
          '--platforms=ios',
          '-t',
          'plugin',
          'hello',
        ], workingDirectory: tempDir.path);
        expect(result, const ProcessResultMatcher());

        pluginRoot = tempDir.childDirectory('hello');
        projectRoot = pluginRoot.childDirectory('example').path;
      });

      tearDownAll(() {
        tryToDelete(hiddenFile);
        tryToDelete(tempDir);
      });

      for (final BuildMode buildMode in <BuildMode>[BuildMode.debug, BuildMode.release]) {
        group('build in ${buildMode.cliName} mode', () {
          late Directory outputPath;
          late Directory outputApp;
          late Directory frameworkDirectory;
          late Directory outputFlutterFramework;
          late File outputFlutterFrameworkBinary;
          late Directory outputAppFramework;
          late File outputAppFrameworkBinary;
          late File outputRunnerBinary;
          late File outputPluginFrameworkBinary;
          late Directory buildPath;
          late Directory buildAppFrameworkDsym;
          late File buildAppFrameworkDsymBinary;
          late ProcessResult buildResult;

          setUpAll(() {
            buildResult = processManager.runSync(<String>[
              flutterBin,
              ...getLocalEngineArguments(),
              'build',
              'ios',
              '--verbose',
              '--no-codesign',
              '--${buildMode.cliName}',
              '--obfuscate',
              '--split-debug-info=foo debug info/',
            ], workingDirectory: projectRoot);

            outputPath = fileSystem.directory(
              fileSystem.path.join(projectRoot, 'build', 'ios', 'iphoneos'),
            );

            outputApp = outputPath.childDirectory('Runner.app');

            frameworkDirectory = outputApp.childDirectory('Frameworks');
            outputFlutterFramework = frameworkDirectory.childDirectory('Flutter.framework');
            outputFlutterFrameworkBinary = outputFlutterFramework.childFile('Flutter');

            outputAppFramework = frameworkDirectory.childDirectory('App.framework');
            outputAppFrameworkBinary = outputAppFramework.childFile('App');

            outputRunnerBinary = outputApp.childFile('Runner');

            // Exists only if the plugin is built as a dynamic framework.
            // This is is the default for CocoaPods but not Swift Package Manager.
            outputPluginFrameworkBinary = frameworkDirectory
                .childDirectory('hello.framework')
                .childFile('hello');

            buildPath = fileSystem.directory(
              fileSystem.path.join(
                projectRoot,
                'build',
                'ios',
                '${sentenceCase(buildMode.cliName)}-iphoneos',
              ),
            );

            buildAppFrameworkDsym = buildPath.childDirectory('App.framework.dSYM');
            buildAppFrameworkDsymBinary = buildAppFrameworkDsym.childFile(
              'Contents/Resources/DWARF/App',
            );
          });

          testWithoutContext('flutter build ios builds a valid app', () {
            printOnFailure('Output of flutter build ios:');
            printOnFailure(buildResult.stdout.toString());
            printOnFailure(buildResult.stderr.toString());
            expect(buildResult.exitCode, 0);

            // Plugins are built either as a static library (SwiftPM's default)
            // or as a dynamic library (CocoaPods's default).
            // If built as a dynamic library, the plugin will have a .framework.
            // If built as static library, the plugin's symbols will be in the
            // Runner binary.
            final bool helloDynamic = outputPluginFrameworkBinary.existsSync();
            final bool helloStatic = AppleTestUtils.getExportedSymbols(
              outputRunnerBinary.path,
            ).any((String symbol) => symbol.contains('HelloPlugin') && symbol.contains('handle'));

            // Plugin is a dynamic xor static framework.
            expect(helloDynamic != helloStatic, isTrue);

            expect(outputFlutterFramework.childFile('PrivacyInfo.xcprivacy'), exists);

            expect(outputAppFrameworkBinary, exists);
            expect(outputAppFramework.childFile('Info.plist'), exists);

            expect(buildAppFrameworkDsymBinary.existsSync(), buildMode != BuildMode.debug);

            final File vmSnapshot = fileSystem.file(
              fileSystem.path.join(outputAppFramework.path, 'flutter_assets', 'vm_snapshot_data'),
            );

            expect(vmSnapshot.existsSync(), buildMode == BuildMode.debug);
          });

          testWithoutContext('Info.plist dart VM Service Bonjour service', () {
            final String infoPlistPath = fileSystem.path.join(outputApp.path, 'Info.plist');
            final ProcessResult bonjourServices = processManager.runSync(<String>[
              'plutil',
              '-extract',
              'NSBonjourServices',
              'xml1',
              '-o',
              '-',
              infoPlistPath,
            ]);
            final bool bonjourServicesFound = (bonjourServices.stdout as String).contains(
              '_dartVmService._tcp',
            );
            expect(bonjourServicesFound, buildMode == BuildMode.debug);

            final ProcessResult localNetworkUsage = processManager.runSync(<String>[
              'plutil',
              '-extract',
              'NSLocalNetworkUsageDescription',
              'xml1',
              '-o',
              '-',
              infoPlistPath,
            ]);
            final bool localNetworkUsageFound = localNetworkUsage.exitCode == 0;
            expect(localNetworkUsageFound, buildMode == BuildMode.debug);
          });

          testWithoutContext('check symbols', () {
            final List<String> symbols = AppleTestUtils.getExportedSymbols(
              outputAppFrameworkBinary.path,
            );
            if (buildMode == BuildMode.debug) {
              expect(symbols, isEmpty);
            } else {
              expect(symbols, equals(AppleTestUtils.requiredSymbols));
            }
          });

          testWithoutContext('check symbols in dSYM', () {
            if (buildMode == BuildMode.debug) {
              // dSYM is not created for a debug build.
              expect(buildAppFrameworkDsymBinary.existsSync(), isFalse);
            } else {
              final List<String> symbols = AppleTestUtils.getExportedSymbols(
                buildAppFrameworkDsymBinary.path,
              );
              expect(symbols, containsAll(AppleTestUtils.requiredSymbols));
              // The actual number of symbols is going to vary but there should
              // be "many" in the dSYM. At the time of writing, it was 7656.
              expect(symbols.length, greaterThanOrEqualTo(5000));
            }
          });

          testWithoutContext(
            'xcode_backend embed_and_thin',
            () {
              outputFlutterFramework.deleteSync(recursive: true);
              outputAppFramework.deleteSync(recursive: true);
              expect(outputFlutterFrameworkBinary.existsSync(), isFalse);
              expect(outputAppFrameworkBinary.existsSync(), isFalse);

              final String xcodeBackendPath = fileSystem.path.join(
                flutterRoot,
                'packages',
                'flutter_tools',
                'bin',
                'xcode_backend.sh',
              );

              // Simulate a common Xcode build setting misconfiguration
              // where FLUTTER_APPLICATION_PATH is missing
              final ProcessResult xcodeBackendResult = processManager.runSync(
                <String>[xcodeBackendPath, 'embed_and_thin'],
                environment: <String, String>{
                  'SOURCE_ROOT': fileSystem.path.join(projectRoot, 'ios'),
                  'BUILT_PRODUCTS_DIR': fileSystem.path.join(
                    projectRoot,
                    'build',
                    'ios',
                    'Release-iphoneos',
                  ),
                  'TARGET_BUILD_DIR': outputPath.path,
                  'FRAMEWORKS_FOLDER_PATH': 'Runner.app/Frameworks',
                  'VERBOSE_SCRIPT_LOGGING': '1',
                  'FLUTTER_BUILD_MODE': 'release',
                  'ACTION': 'install',
                  'FLUTTER_BUILD_DIR': 'build',
                  // Skip bitcode stripping since we just checked that above.
                },
              );
              printOnFailure('Output of xcode_backend.sh:');
              printOnFailure(xcodeBackendResult.stdout.toString());
              printOnFailure(xcodeBackendResult.stderr.toString());

              expect(xcodeBackendResult.exitCode, 0);
              expect(outputFlutterFrameworkBinary.existsSync(), isTrue);
              expect(outputAppFrameworkBinary.existsSync(), isTrue);
            },
            // [intended] only makes sense on macos.
            skip: !platform.isMacOS || buildMode != BuildMode.release,
          );

          testWithoutContext('validate obfuscation', () {
            // HelloPlugin class is present in project.
            ProcessResult grepResult = processManager.runSync(<String>[
              'grep',
              '-r',
              'HelloPlugin',
              pluginRoot.path,
            ]);
            // Matches exits 0.
            expect(grepResult.exitCode, 0);

            // Not present in binary.
            grepResult = processManager.runSync(<String>[
              'grep',
              'HelloPlugin',
              outputAppFrameworkBinary.path,
            ]);
            // Does not match exits 1.
            expect(grepResult.exitCode, 1);
          });
        });
      }

      testWithoutContext('builds all plugin architectures for simulator', () {
        final ProcessResult buildSimulator = processManager.runSync(<String>[
          flutterBin,
          ...getLocalEngineArguments(),
          'build',
          'ios',
          '--simulator',
          '--verbose',
          '--no-codesign',
        ], workingDirectory: projectRoot);
        expect(buildSimulator.exitCode, 0);

        // Plugins are built either as a static library (SwiftPM's default)
        // or as a dynamic library (CocoaPods's default).
        // If built as a dynamic library, the plugin will have a .framework.
        // If built as static library, the plugin's symbols will be in the
        // Runner binary.
        final File runnerBinary = fileSystem.file(
          fileSystem.path.join(
            projectRoot,
            'build',
            'ios',
            'iphonesimulator',
            'Runner.app',
            'Runner',
          ),
        );
        final File pluginFrameworkBinary = fileSystem.file(
          fileSystem.path.join(
            projectRoot,
            'build',
            'ios',
            'iphonesimulator',
            'Runner.app',
            'Frameworks',
            'hello.framework',
            'hello',
          ),
        );
        final bool helloDynamic = pluginFrameworkBinary.existsSync();
        final bool helloStatic = AppleTestUtils.getExportedSymbols(
          runnerBinary.path,
        ).any((String symbol) => symbol.contains('HelloPlugin') && symbol.contains('handle'));

        // Plugin is a dynamic xor static framework.
        expect(helloDynamic != helloStatic, isTrue);

        if (helloDynamic) {
          final ProcessResult archs = processManager.runSync(<String>[
            'file',
            pluginFrameworkBinary.path,
          ]);
          expect(archs.stdout, contains('Mach-O 64-bit dynamically linked shared library x86_64'));
          expect(archs.stdout, contains('Mach-O 64-bit dynamically linked shared library arm64'));
        }
      });

      testWithoutContext('build for simulator with all available architectures', () {
        final ProcessResult buildSimulator = processManager.runSync(
          <String>[
            flutterBin,
            ...getLocalEngineArguments(),
            'build',
            'ios',
            '--simulator',
            '--verbose',
            '--no-codesign',
          ],
          workingDirectory: projectRoot,
          environment: <String, String>{'FLUTTER_XCODE_ONLY_ACTIVE_ARCH': 'NO'},
        );
        // This test case would fail if arm64 or x86_64 simulators could not build.
        expect(buildSimulator.exitCode, 0);

        final File simulatorAppFrameworkBinary = fileSystem.file(
          fileSystem.path.join(
            projectRoot,
            'build',
            'ios',
            'iphonesimulator',
            'Runner.app',
            'Frameworks',
            'App.framework',
            'App',
          ),
        );
        expect(simulatorAppFrameworkBinary, exists);
        final ProcessResult archs = processManager.runSync(<String>[
          'file',
          simulatorAppFrameworkBinary.path,
        ]);
        expect(archs.stdout, contains('Mach-O 64-bit dynamically linked shared library x86_64'));
        expect(archs.stdout, contains('Mach-O 64-bit dynamically linked shared library arm64'));
      });

      testWithoutContext('archive', () {
        final File appIconFile = fileSystem.file(
          fileSystem.path.join(
            projectRoot,
            'ios',
            'Runner',
            'Assets.xcassets',
            'AppIcon.appiconset',
            'Icon-App-20x20@1x.png',
          ),
        );
        // Resizes app icon to 123x456 (it is supposed to be 20x20).
        appIconFile.writeAsBytesSync(
          appIconFile.readAsBytesSync()
            ..buffer.asByteData().setInt32(16, 123)
            ..buffer.asByteData().setInt32(20, 456),
        );

        final ProcessResult output = processManager.runSync(<String>[
          flutterBin,
          ...getLocalEngineArguments(),
          'build',
          'xcarchive',
          '--verbose',
        ], workingDirectory: projectRoot);

        // Note this isBot so usage won't actually be sent,
        // this log line is printed whenever the app is archived.
        expect(output.stdout, contains('Sending archive event if usage enabled'));

        // The output contains extra time related prefix, so cannot use a single string.
        const List<String> expectedValidationMessages = <String>[
          '[!] App Settings Validation\n',
          '    • Version Number: 1.0.0\n',
          '    • Build Number: 1\n',
          '    • Display Name: Hello\n',
          '    • Deployment Target: 12.0\n',
          '    • Bundle Identifier: com.example.hello\n',
          '    ! Your application still contains the default "com.example" bundle identifier.\n',
          '[!] App Icon and Launch Image Assets Validation\n',
          '    ! App icon is set to the default placeholder icon. Replace with unique icons.\n',
          '    ! App icon is using the incorrect size (e.g. Icon-App-20x20@1x.png).\n',
          '    ! Launch image is set to the default placeholder icon. Replace with unique launch image.\n',
          'To update the settings, please refer to https://flutter.dev/to/ios-deploy\n',
        ];
        expect(expectedValidationMessages, unorderedEquals(expectedValidationMessages));

        final Directory archivePath = fileSystem.directory(
          fileSystem.path.join(projectRoot, 'build', 'ios', 'archive', 'Runner.xcarchive'),
        );

        final Directory products = archivePath.childDirectory('Products');
        expect(products, exists);

        final Directory dSYM = archivePath
            .childDirectory('dSYMs')
            .childDirectory('Runner.app.dSYM');
        expect(dSYM, exists);

        final Directory applications = products.childDirectory('Applications');

        final Directory appBundle = applications.listSync().whereType<Directory>().singleWhere(
          (Directory directory) => fileSystem.path.extension(directory.path) == '.app',
        );

        final Directory flutterFrameworkDir = fileSystem.directory(
          fileSystem.path.join(appBundle.path, 'Frameworks', 'Flutter.framework'),
        );

        final String flutterFramework = fileSystem.path.join(flutterFrameworkDir.path, 'Flutter');

        // Exits 0 only if codesigned.
        final ProcessResult flutterCodesign = processManager.runSync(<String>[
          'xcrun',
          'codesign',
          '--verify',
          flutterFramework,
        ]);
        expect(flutterCodesign, const ProcessResultMatcher());

        final String appFramework = fileSystem.path.join(
          appBundle.path,
          'Frameworks',
          'App.framework',
          'App',
        );

        final ProcessResult appCodesign = processManager.runSync(<String>[
          'xcrun',
          'codesign',
          '--verify',
          appFramework,
        ]);
        expect(appCodesign, const ProcessResultMatcher());

        // Check read/write permissions are being correctly set.
        final String statString = flutterFrameworkDir.statSync().mode.toRadixString(8);
        expect(statString, '40755');
      });
    },
    skip: !platform.isMacOS, // [intended] only makes sense for macos platform.
    timeout: const Timeout(Duration(minutes: 10)),
  );
}
