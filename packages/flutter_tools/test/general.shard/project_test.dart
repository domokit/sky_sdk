// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/android/android_sdk.dart';
import 'package:flutter_tools/src/android/android_studio.dart';
import 'package:flutter_tools/src/android/gradle_utils.dart' as gradle_utils;
import 'package:flutter_tools/src/android/java.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/os.dart';
import 'package:flutter_tools/src/base/version.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/convert.dart';
import 'package:flutter_tools/src/dart/pub.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/flutter_manifest.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/ios/plist_parser.dart';
import 'package:flutter_tools/src/ios/xcodeproj.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:meta/meta.dart';
import 'package:test/fake.dart';

import '../src/common.dart';
import '../src/context.dart';
import '../src/fake_pub_deps.dart';
import '../src/fakes.dart';

void main() {
  // TODO(zanderso): remove once FlutterProject is fully refactored.
  // this is safe since no tests have expectations on the test logger.
  final BufferLogger logger = BufferLogger.test();

  group('Project', () {
    group('construction', () {
      testWithoutContext('invalid utf8 throws a tool exit', () {
        final FileSystem fileSystem = MemoryFileSystem.test();
        final FlutterProjectFactory projectFactory = FlutterProjectFactory(
          fileSystem: fileSystem,
          logger: BufferLogger.test(),
        );
        fileSystem.file('pubspec.yaml').writeAsBytesSync(<int>[0xFFFE]);

        /// Technically this should throw a FileSystemException but this is
        /// currently a bug in package:file.
        expect(() => projectFactory.fromDirectory(fileSystem.currentDirectory), throwsToolExit());
      });

      _testInMemory('fails on invalid pubspec.yaml', () async {
        final Directory directory = globals.fs.directory('myproject');
        directory.childFile('pubspec.yaml')
          ..createSync(recursive: true)
          ..writeAsStringSync(invalidPubspec);

        expect(() => FlutterProject.fromDirectory(directory), throwsToolExit());
      });

      _testInMemory('fails on pubspec.yaml parse failure', () async {
        final Directory directory = globals.fs.directory('myproject');
        directory.childFile('pubspec.yaml')
          ..createSync(recursive: true)
          ..writeAsStringSync(parseErrorPubspec);

        expect(() => FlutterProject.fromDirectory(directory), throwsToolExit());
      });

      _testInMemory('fails on invalid example/pubspec.yaml', () async {
        final Directory directory = globals.fs.directory('myproject');
        directory.childDirectory('example').childFile('pubspec.yaml')
          ..createSync(recursive: true)
          ..writeAsStringSync(invalidPubspec);

        expect(() => FlutterProject.fromDirectory(directory), throwsToolExit());
      });

      _testInMemory('treats missing pubspec.yaml as empty', () async {
        final Directory directory = globals.fs.directory('myproject')..createSync(recursive: true);
        expect(FlutterProject.fromDirectory(directory).manifest.isEmpty, true);
      });

      _testInMemory('reads valid pubspec.yaml', () async {
        final Directory directory = globals.fs.directory('myproject');
        directory.childFile('pubspec.yaml')
          ..createSync(recursive: true)
          ..writeAsStringSync(validPubspec);
        expect(FlutterProject.fromDirectory(directory).manifest.appName, 'hello');
      });

      _testInMemory('reads dependencies from pubspec.yaml', () async {
        final Directory directory = globals.fs.directory('myproject');
        directory.childFile('pubspec.yaml')
          ..createSync(recursive: true)
          ..writeAsStringSync(validPubspecWithDependencies);
        expect(FlutterProject.fromDirectory(directory).manifest.dependencies, <String>{
          'plugin_a',
          'plugin_b',
        });
      });

      _testInMemory('sets up location', () async {
        final Directory directory = globals.fs.directory('myproject');
        expect(
          FlutterProject.fromDirectory(directory).directory.absolute.path,
          directory.absolute.path,
        );
        expect(
          FlutterProject.fromDirectoryTest(directory).directory.absolute.path,
          directory.absolute.path,
        );
        expect(
          FlutterProject.current().directory.absolute.path,
          globals.fs.currentDirectory.absolute.path,
        );
      });
    });

    group('ensure ready for platform-specific tooling', () {
      _testInMemory('does nothing, if project is not created', () async {
        final FlutterProject project = FlutterProject(
          globals.fs.directory('not_created'),
          FlutterManifest.empty(logger: logger),
          FlutterManifest.empty(logger: logger),
        );
        await project.regeneratePlatformSpecificTooling();
        expectNotExists(project.directory);
      });
      _testInMemory('does nothing in plugin or package root project', () async {
        final FlutterProject project = await aPluginProject();
        await project.regeneratePlatformSpecificTooling();
        expectNotExists(
          project.ios.hostAppRoot.childDirectory('Runner').childFile('GeneratedPluginRegistrant.h'),
        );
        expectNotExists(
          androidPluginRegistrant(project.android.hostAppGradleRoot.childDirectory('app')),
        );
        expectNotExists(
          project.ios.hostAppRoot.childDirectory('Flutter').childFile('Generated.xcconfig'),
        );
        expectNotExists(project.android.hostAppGradleRoot.childFile('local.properties'));
      });
      _testInMemory('works if there is an "example" folder', () async {
        final FlutterProject project = await someProject();
        // The presence of an "example" folder used to be used as an indicator
        // that a project was a plugin, but shouldn't be as this creates false
        // positives.
        project.directory.childDirectory('example').createSync();
        await project.regeneratePlatformSpecificTooling();
        expectExists(
          project.ios.hostAppRoot.childDirectory('Runner').childFile('GeneratedPluginRegistrant.h'),
        );
        expectExists(
          androidPluginRegistrant(project.android.hostAppGradleRoot.childDirectory('app')),
        );
        expectExists(
          project.ios.hostAppRoot.childDirectory('Flutter').childFile('Generated.xcconfig'),
        );
        expectExists(project.android.hostAppGradleRoot.childFile('local.properties'));
      });
      _testInMemory('injects plugins for iOS', () async {
        final FlutterProject project = await someProject();
        await project.regeneratePlatformSpecificTooling();
        expectExists(
          project.ios.hostAppRoot.childDirectory('Runner').childFile('GeneratedPluginRegistrant.h'),
        );
      });
      _testInMemory('generates Xcode configuration for iOS', () async {
        final FlutterProject project = await someProject();
        await project.regeneratePlatformSpecificTooling();
        expectExists(
          project.ios.hostAppRoot.childDirectory('Flutter').childFile('Generated.xcconfig'),
        );
      });
      _testInMemory('injects plugins for Android', () async {
        final FlutterProject project = await someProject();
        await project.regeneratePlatformSpecificTooling();
        expectExists(
          androidPluginRegistrant(project.android.hostAppGradleRoot.childDirectory('app')),
        );
      });
      _testInMemory('updates local properties for Android', () async {
        final FlutterProject project = await someProject();
        await project.regeneratePlatformSpecificTooling();
        expectExists(project.android.hostAppGradleRoot.childFile('local.properties'));
      });
      _testInMemory('checkForDeprecation fails on invalid android app manifest file', () async {
        // This is not a valid Xml document
        const String invalidManifest = '<manifest></application>';
        final FlutterProject project = await someProject(
          androidManifestOverride: invalidManifest,
          includePubspec: true,
        );

        expect(
          () => project.checkForDeprecation(deprecationBehavior: DeprecationBehavior.ignore),
          throwsToolExit(
            message:
                'Please ensure that the android manifest is a valid XML document and try again.',
          ),
        );
      });
      _testInMemory(
        'Project not on v2 embedding does not warn if deprecation status is irrelevant',
        () async {
          final FlutterProject project = await someProject(includePubspec: true);
          // The default someProject with an empty <manifest> already indicates
          // v1 embedding, as opposed to having <meta-data
          // android:name="flutterEmbedding" android:value="2" />.

          // Default is "DeprecationBehavior.none"
          project.checkForDeprecation();
          expect(testLogger.statusText, isEmpty);
        },
      );
      _testInMemory('Android project no pubspec continues', () async {
        final FlutterProject project = await someProject();
        // The default someProject with an empty <manifest> already indicates
        // v1 embedding, as opposed to having <meta-data
        // android:name="flutterEmbedding" android:value="2" />.

        project.checkForDeprecation(deprecationBehavior: DeprecationBehavior.ignore);
        expect(
          testLogger.statusText,
          isNot(
            contains(
              'https://github.com/flutter/flutter/blob/main/docs/platforms/android/Upgrading-pre-1.12-Android-projects.md',
            ),
          ),
        );
      });
      _testInMemory(
        'Android plugin project does not throw v1 embedding deprecation warning',
        () async {
          final FlutterProject project = await aPluginProject();

          project.checkForDeprecation(deprecationBehavior: DeprecationBehavior.exit);
          expect(
            testLogger.statusText,
            isNot(
              contains(
                'https://github.com/flutter/flutter/blob/main/docs/platforms/android/Upgrading-pre-1.12-Android-projects.md',
              ),
            ),
          );
          expect(
            testLogger.statusText,
            isNot(
              contains('No `<meta-data android:name="flutterEmbedding" android:value="2"/>` in '),
            ),
          );
        },
      );
      _testInMemory('Android plugin without example app does not show a warning', () async {
        final FlutterProject project = await aPluginProject();
        project.example.directory.deleteSync();

        await project.regeneratePlatformSpecificTooling();
        expect(
          testLogger.statusText,
          isNot(
            contains(
              'https://github.com/flutter/flutter/blob/main/docs/platforms/android/Upgrading-pre-1.12-Android-projects.md',
            ),
          ),
        );
      });
      _testInMemory('updates local properties for Android', () async {
        final FlutterProject project = await someProject();
        await project.regeneratePlatformSpecificTooling();
        expectExists(project.android.hostAppGradleRoot.childFile('local.properties'));
      });
      testUsingContext(
        'injects plugins for macOS',
        () async {
          final FlutterProject project = await someProject();
          project.macos.managedDirectory.createSync(recursive: true);
          await project.regeneratePlatformSpecificTooling();
          expectExists(project.macos.pluginRegistrantImplementation);
        },
        overrides: <Type, Generator>{
          FileSystem: () => MemoryFileSystem.test(),
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true),
          FlutterProjectFactory:
              () => FlutterProjectFactory(logger: logger, fileSystem: globals.fs),
        },
      );
      testUsingContext(
        'generates Xcode configuration for macOS',
        () async {
          final FlutterProject project = await someProject();
          project.macos.managedDirectory.createSync(recursive: true);
          await project.regeneratePlatformSpecificTooling();
          expectExists(project.macos.generatedXcodePropertiesFile);
        },
        overrides: <Type, Generator>{
          FileSystem: () => MemoryFileSystem.test(),
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true),
          FlutterProjectFactory:
              () => FlutterProjectFactory(logger: logger, fileSystem: globals.fs),
        },
      );
      testUsingContext(
        'injects plugins for Linux',
        () async {
          final FlutterProject project = await someProject();
          project.linux.cmakeFile.createSync(recursive: true);
          await project.regeneratePlatformSpecificTooling();
          expectExists(project.linux.managedDirectory.childFile('generated_plugin_registrant.h'));
          expectExists(project.linux.managedDirectory.childFile('generated_plugin_registrant.cc'));
        },
        overrides: <Type, Generator>{
          FileSystem: () => MemoryFileSystem.test(),
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: () => TestFeatureFlags(isLinuxEnabled: true),
          FlutterProjectFactory:
              () => FlutterProjectFactory(logger: logger, fileSystem: globals.fs),
        },
      );
      testUsingContext(
        'injects plugins for Windows',
        () async {
          final FlutterProject project = await someProject();
          project.windows.cmakeFile.createSync(recursive: true);
          await project.regeneratePlatformSpecificTooling();
          expectExists(project.windows.managedDirectory.childFile('generated_plugin_registrant.h'));
          expectExists(
            project.windows.managedDirectory.childFile('generated_plugin_registrant.cc'),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => MemoryFileSystem.test(),
          ProcessManager: () => FakeProcessManager.any(),
          FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
          FlutterProjectFactory:
              () => FlutterProjectFactory(logger: logger, fileSystem: globals.fs),
        },
      );
      _testInMemory('creates Android library in module', () async {
        final FlutterProject project = await aModuleProject();
        await project.regeneratePlatformSpecificTooling();
        expectExists(project.android.hostAppGradleRoot.childFile('settings.gradle'));
        expectExists(project.android.hostAppGradleRoot.childFile('local.properties'));
        expectExists(
          androidPluginRegistrant(project.android.hostAppGradleRoot.childDirectory('Flutter')),
        );
      });
      _testInMemory('creates iOS pod in module', () async {
        final FlutterProject project = await aModuleProject();
        await project.regeneratePlatformSpecificTooling();
        final Directory flutter = project.ios.hostAppRoot.childDirectory('Flutter');
        expectExists(flutter.childFile('podhelper.rb'));
        expectExists(flutter.childFile('flutter_export_environment.sh'));
        expectExists(flutter.childFile('Generated.xcconfig'));
        final Directory pluginRegistrantClasses = flutter
            .childDirectory('FlutterPluginRegistrant')
            .childDirectory('Classes');
        expectExists(pluginRegistrantClasses.childFile('GeneratedPluginRegistrant.h'));
        expectExists(pluginRegistrantClasses.childFile('GeneratedPluginRegistrant.m'));
      });

      testUsingContext('Version.json info is correct', () {
        final MemoryFileSystem fileSystem = MemoryFileSystem.test();
        final FlutterManifest manifest =
            FlutterManifest.createFromString('''
    name: test
    version: 1.0.0+3
    ''', logger: BufferLogger.test())!;
        final FlutterProject project = FlutterProject(
          fileSystem.systemTempDirectory,
          manifest,
          manifest,
        );
        final Map<String, dynamic> versionInfo =
            jsonDecode(project.getVersionInfo()) as Map<String, dynamic>;
        expect(versionInfo['app_name'], 'test');
        expect(versionInfo['version'], '1.0.0');
        expect(versionInfo['build_number'], '3');
        expect(versionInfo['package_name'], 'test');
      });
      _testInMemory('gets xcworkspace directory', () async {
        final FlutterProject project = await someProject();
        project.ios.xcodeProject.createSync();
        project.ios.hostAppRoot.childFile('._Runner.xcworkspace').createSync(recursive: true);
        project.ios.hostAppRoot.childFile('Runner.xcworkspace').createSync(recursive: true);

        expect(project.ios.xcodeWorkspace?.basename, 'Runner.xcworkspace');
      });
      _testInMemory('no xcworkspace directory found', () async {
        final FlutterProject project = await someProject();
        project.ios.xcodeProject.createSync();
        expect(project.ios.xcodeWorkspace?.basename, null);
      });
    });

    group('module status', () {
      _testInMemory('is known for module', () async {
        final FlutterProject project = await aModuleProject();
        expect(project.isModule, isTrue);
        expect(project.android.isModule, isTrue);
        expect(project.ios.isModule, isTrue);
        expect(project.android.hostAppGradleRoot.basename, '.android');
        expect(project.ios.hostAppRoot.basename, '.ios');
      });
      _testInMemory('is known for non-module', () async {
        final FlutterProject project = await someProject();
        expect(project.isModule, isFalse);
        expect(project.android.isModule, isFalse);
        expect(project.ios.isModule, isFalse);
        expect(project.android.hostAppGradleRoot.basename, 'android');
        expect(project.ios.hostAppRoot.basename, 'ios');
      });
    });

    group('example', () {
      _testInMemory('exists for plugin in legacy format', () async {
        final FlutterProject project = await aPluginProject();
        expect(project.isPlugin, isTrue);
        expect(project.hasExampleApp, isTrue);
      });
      _testInMemory('exists for plugin in multi-platform format', () async {
        final FlutterProject project = await aPluginProject(legacy: false);
        expect(project.hasExampleApp, isTrue);
      });
      _testInMemory('does not exist for non-plugin', () async {
        final FlutterProject project = await someProject();
        expect(project.isPlugin, isFalse);
        expect(project.hasExampleApp, isFalse);
      });
    });

    group('java gradle agp compatibility', () {
      Future<FlutterProject?> configureGradleAgpForTest({
        required String gradleV,
        required String agpV,
      }) async {
        final FlutterProject project = await someProject();
        addRootGradleFile(
          project.directory,
          gradleFileContent: () {
            return '''
dependencies {
    classpath 'com.android.tools.build:gradle:$agpV'
}
''';
          },
        );
        addGradleWrapperFile(project.directory, gradleV);
        return project;
      }

      // Tests in this group that use overrides and _testInMemory should
      // be placed in their own group to avoid test pollution. This is
      // especially important for filesystem.
      group('_', () {
        final FakeProcessManager processManager;
        final Java java;
        final AndroidStudio androidStudio;
        final FakeAndroidSdkWithDir androidSdk;
        final FileSystem fileSystem = getFileSystemForPlatform();
        java = FakeJava(version: Version(17, 0, 2));
        processManager = FakeProcessManager.empty();
        androidStudio = FakeAndroidStudio();
        androidSdk = FakeAndroidSdkWithDir(fileSystem.currentDirectory);
        fileSystem.currentDirectory.childDirectory(androidStudio.javaPath!).createSync();
        _testInMemory(
          'flamingo values are compatible',
          () async {
            final FlutterProject? project = await configureGradleAgpForTest(
              gradleV: '8.0',
              agpV: '7.4.2',
            );
            final CompatibilityResult value =
                await project!.android.hasValidJavaGradleAgpVersions();
            expect(value.success, isTrue);
          },
          java: java,
          androidStudio: androidStudio,
          processManager: processManager,
          androidSdk: androidSdk,
        );
      });
      group('_', () {
        final FakeProcessManager processManager;
        final Java java;
        final AndroidStudio androidStudio;
        final FakeAndroidSdkWithDir androidSdk;
        final FileSystem fileSystem = getFileSystemForPlatform();
        java = FakeJava(version: const Version.withText(1, 8, 0, '1.8.0_242'));
        processManager = FakeProcessManager.empty();
        androidStudio = FakeAndroidStudio();
        androidSdk = FakeAndroidSdkWithDir(fileSystem.currentDirectory);
        fileSystem.currentDirectory.childDirectory(androidStudio.javaPath!).createSync();
        _testInMemory(
          'java 8 era values are compatible',
          () async {
            final FlutterProject? project = await configureGradleAgpForTest(
              gradleV: '6.7.1',
              agpV: '4.2.0',
            );
            final CompatibilityResult value =
                await project!.android.hasValidJavaGradleAgpVersions();
            expect(value.success, isTrue);
          },
          java: java,
          androidStudio: androidStudio,
          processManager: processManager,
          androidSdk: androidSdk,
        );
      });

      group('_', () {
        final FakeProcessManager processManager;
        final Java java;
        final AndroidStudio androidStudio;
        final FakeAndroidSdkWithDir androidSdk;
        final FileSystem fileSystem = getFileSystemForPlatform();
        processManager = FakeProcessManager.empty();
        java = FakeJava(version: Version(11, 0, 14));
        androidStudio = FakeAndroidStudio();
        androidSdk = FakeAndroidSdkWithDir(fileSystem.currentDirectory);
        fileSystem.currentDirectory.childDirectory(androidStudio.javaPath!).createSync();
        _testInMemory(
          'electric eel era values are compatible',
          () async {
            final FlutterProject? project = await configureGradleAgpForTest(
              gradleV: '7.3.3',
              agpV: '7.2.0',
            );
            final CompatibilityResult value =
                await project!.android.hasValidJavaGradleAgpVersions();
            expect(value.success, isTrue);
          },
          java: java,
          androidStudio: androidStudio,
          processManager: processManager,
          androidSdk: androidSdk,
        );
      });
      group('_', () {
        const String javaV = '17.0.2';
        const String gradleV = '6.7.3';
        const String agpV = '7.2.0';

        final FakeProcessManager processManager;
        final Java java;
        final AndroidStudio androidStudio;
        final FakeAndroidSdkWithDir androidSdk;
        final FileSystem fileSystem = getFileSystemForPlatform();
        processManager = FakeProcessManager.empty();
        java = FakeJava(version: Version.parse(javaV));
        androidStudio = FakeAndroidStudio();
        androidSdk = FakeAndroidSdkWithDir(fileSystem.currentDirectory);
        fileSystem.currentDirectory.childDirectory(androidStudio.javaPath!).createSync();
        _testInMemory(
          'incompatible everything',
          () async {
            final FlutterProject? project = await configureGradleAgpForTest(
              gradleV: gradleV,
              agpV: agpV,
            );
            final CompatibilityResult value =
                await project!.android.hasValidJavaGradleAgpVersions();
            expect(value.success, isFalse);
            // Should not have the valid string
            expect(
              value.description,
              isNot(contains(RegExp(AndroidProject.validJavaGradleAgpString))),
            );
            // On gradle/agp error print help url and gradle and agp versions.
            expect(value.description, contains(RegExp(AndroidProject.gradleAgpCompatUrl)));
            expect(value.description, contains(RegExp(gradleV)));
            expect(value.description, contains(RegExp(agpV)));
            // On gradle/agp error print help url and java and gradle versions.
            expect(value.description, contains(RegExp(AndroidProject.javaGradleCompatUrl)));
            expect(value.description, contains(RegExp(javaV)));
            expect(value.description, contains(RegExp(gradleV)));
          },
          java: java,
          androidStudio: androidStudio,
          processManager: processManager,
          androidSdk: androidSdk,
        );
      });
      group('_', () {
        const String javaV = '17.0.2';
        const String gradleV = '6.7.3';
        const String agpV = '4.2.0';

        final FakeProcessManager processManager;
        final Java java;
        final AndroidStudio androidStudio;
        final FakeAndroidSdkWithDir androidSdk;
        final FileSystem fileSystem = getFileSystemForPlatform();
        processManager = FakeProcessManager.empty();
        java = FakeJava(version: Version(17, 0, 2));
        androidStudio = FakeAndroidStudio();
        androidSdk = FakeAndroidSdkWithDir(fileSystem.currentDirectory);
        fileSystem.currentDirectory.childDirectory(androidStudio.javaPath!).createSync();
        _testInMemory(
          'incompatible java/gradle only',
          () async {
            final FlutterProject? project = await configureGradleAgpForTest(
              gradleV: gradleV,
              agpV: agpV,
            );
            final CompatibilityResult value =
                await project!.android.hasValidJavaGradleAgpVersions();
            expect(value.success, isFalse);
            // Should not have the valid string.
            expect(
              value.description,
              isNot(contains(RegExp(AndroidProject.validJavaGradleAgpString))),
            );
            // On gradle/agp error print help url and java and gradle versions.
            expect(value.description, contains(RegExp(AndroidProject.javaGradleCompatUrl)));
            expect(value.description, contains(RegExp(javaV)));
            expect(value.description, contains(RegExp(gradleV)));
          },
          java: java,
          androidStudio: androidStudio,
          processManager: processManager,
          androidSdk: androidSdk,
        );
      });
      group('_', () {
        final FakeProcessManager processManager;
        final Java java;
        final AndroidStudio androidStudio;
        final FakeAndroidSdkWithDir androidSdk;
        final FileSystem fileSystem = getFileSystemForPlatform();
        java = FakeJava(version: Version(11, 0, 2));
        processManager = FakeProcessManager.empty();
        androidStudio = FakeAndroidStudio();
        androidSdk = FakeAndroidSdkWithDir(fileSystem.currentDirectory);
        fileSystem.currentDirectory.childDirectory(androidStudio.javaPath!).createSync();
        _testInMemory(
          'incompatible gradle/agp only',
          () async {
            const String gradleV = '7.0.3';
            const String agpV = '7.1.0';
            final FlutterProject? project = await configureGradleAgpForTest(
              gradleV: gradleV,
              agpV: agpV,
            );
            final CompatibilityResult value =
                await project!.android.hasValidJavaGradleAgpVersions();
            expect(value.success, isFalse);
            // Should not have the valid string.
            expect(
              value.description,
              isNot(contains(RegExp(AndroidProject.validJavaGradleAgpString))),
            );
            // On gradle/agp error print help url and gradle and agp versions.
            expect(value.description, contains(RegExp(AndroidProject.gradleAgpCompatUrl)));
            expect(value.description, contains(RegExp(gradleV)));
            expect(value.description, contains(RegExp(agpV)));
          },
          java: java,
          androidStudio: androidStudio,
          processManager: processManager,
          androidSdk: androidSdk,
        );
      });
      group('_', () {
        final FakeProcessManager processManager;
        final Java java;
        final AndroidStudio androidStudio;
        final FakeAndroidSdkWithDir androidSdk;
        final FileSystem fileSystem = getFileSystemForPlatform();
        java = FakeJava(version: Version(11, 0, 2));
        processManager = FakeProcessManager.empty();
        androidStudio = FakeAndroidStudio();
        androidSdk = FakeAndroidSdkWithDir(fileSystem.currentDirectory);
        fileSystem.currentDirectory.childDirectory(androidStudio.javaPath!).createSync();
        _testInMemory(
          'null agp only',
          () async {
            const String gradleV = '7.0.3';
            final FlutterProject? project = await configureGradleAgpForTest(
              gradleV: gradleV,
              agpV: '',
            );
            final CompatibilityResult value =
                await project!.android.hasValidJavaGradleAgpVersions();
            expect(value.success, isFalse);
            // Should not have the valid string.
            expect(
              value.description,
              isNot(contains(RegExp(AndroidProject.validJavaGradleAgpString))),
            );
            // On gradle/agp error print help url null value for agp.
            expect(value.description, contains(RegExp(AndroidProject.gradleAgpCompatUrl)));
            expect(value.description, contains(RegExp(gradleV)));
            expect(value.description, contains(RegExp('null')));
          },
          java: java,
          androidStudio: androidStudio,
          processManager: processManager,
          androidSdk: androidSdk,
        );
      });
    });

    group('language', () {
      late XcodeProjectInterpreter xcodeProjectInterpreter;
      late MemoryFileSystem fs;
      late FlutterProjectFactory flutterProjectFactory;
      setUp(() {
        fs = MemoryFileSystem.test();
        xcodeProjectInterpreter = XcodeProjectInterpreter.test(
          processManager: FakeProcessManager.any(),
        );
        flutterProjectFactory = FlutterProjectFactory(logger: logger, fileSystem: fs);
      });

      _testInMemory('default host app language', () async {
        final FlutterProject project = await someProject();
        expect(project.android.isKotlin, isFalse);
      });

      testUsingContext(
        'kotlin host app language',
        () async {
          final FlutterProject project = await someProject();

          addAndroidGradleFile(
            project.directory,
            gradleFileContent: () {
              return '''
apply plugin: 'com.android.application'
apply plugin: 'kotlin-android'
''';
            },
          );
          expect(project.android.isKotlin, isTrue);
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          XcodeProjectInterpreter: () => xcodeProjectInterpreter,
          FlutterProjectFactory: () => flutterProjectFactory,
        },
      );

      testUsingContext(
        'kotlin host app language with Gradle Kotlin DSL',
        () async {
          final FlutterProject project = await someProject();

          addAndroidGradleFile(
            project.directory,
            kotlinDsl: true,
            gradleFileContent: () {
              return '''
plugins {
    id "com.android.application"
    id "kotlin-android"
    id "dev.flutter.flutter-gradle-plugin"
}
''';
            },
          );
          expect(project.android.isKotlin, isTrue);
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          XcodeProjectInterpreter: () => xcodeProjectInterpreter,
          FlutterProjectFactory: () => flutterProjectFactory,
        },
      );

      testUsingContext(
        'kotlin host app language with Gradle Kotlin DSL and typesafe plugin id',
        () async {
          final FlutterProject project = await someProject();

          addAndroidGradleFile(
            project.directory,
            kotlinDsl: true,
            gradleFileContent: () {
              return '''
plugins {
    id "com.android.application"
    id "kotlin-android"
    dev.flutter.`flutter-gradle-plugin`
}
''';
            },
          );
          expect(project.android.isKotlin, isTrue);
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          XcodeProjectInterpreter: () => xcodeProjectInterpreter,
          FlutterProjectFactory: () => flutterProjectFactory,
        },
      );

      testUsingContext(
        'Gradle Groovy files are preferred to Gradle Kotlin files',
        () async {
          final FlutterProject project = await someProject();

          addAndroidGradleFile(
            project.directory,
            gradleFileContent: () {
              return '''
plugins {
    id "com.android.application"
    id "dev.flutter.flutter-gradle-plugin"
}
''';
            },
          );
          addAndroidGradleFile(
            project.directory,
            kotlinDsl: true,
            gradleFileContent: () {
              return '''
plugins {
    id("com.android.application")
    id("kotlin-android")
    id("dev.flutter.flutter-gradle-plugin")
}
''';
            },
          );

          expect(project.android.isKotlin, isFalse);
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          XcodeProjectInterpreter: () => xcodeProjectInterpreter,
          FlutterProjectFactory: () => flutterProjectFactory,
        },
      );
    });

    group('With mocked context', () {
      late MemoryFileSystem fs;
      late FakePlistParser testPlistUtils;
      late FakeXcodeProjectInterpreter xcodeProjectInterpreter;
      late FlutterProjectFactory flutterProjectFactory;
      setUp(() {
        fs = MemoryFileSystem.test();
        testPlistUtils = FakePlistParser();
        xcodeProjectInterpreter = FakeXcodeProjectInterpreter();
        flutterProjectFactory = FlutterProjectFactory(fileSystem: fs, logger: logger);
      });

      void testWithMocks(String description, Future<void> Function() testMethod) {
        testUsingContext(
          description,
          testMethod,
          overrides: <Type, Generator>{
            FileSystem: () => fs,
            ProcessManager: () => FakeProcessManager.any(),
            PlistParser: () => testPlistUtils,
            XcodeProjectInterpreter: () => xcodeProjectInterpreter,
            FlutterProjectFactory: () => flutterProjectFactory,
          },
        );
      }

      group('universal link', () {
        testWithMocks('build with flavor', () async {
          final FlutterProject project = await someProject();
          project.ios.xcodeProject.createSync();
          project.ios.defaultHostInfoPlist.createSync(recursive: true);
          const String entitlementFilePath = 'myEntitlement.Entitlement';
          project.ios.hostAppRoot.childFile(entitlementFilePath).createSync(recursive: true);

          const XcodeProjectBuildContext buildContext = XcodeProjectBuildContext(
            target: 'Runner',
            configuration: 'config',
          );
          xcodeProjectInterpreter.buildSettingsByBuildContext[buildContext] = <String, String>{
            IosProject.kProductBundleIdKey: 'io.flutter.someProject',
            IosProject.kTeamIdKey: 'ABC',
            IosProject.kEntitlementFilePathKey: entitlementFilePath,
            'SUFFIX': 'suffix',
          };
          xcodeProjectInterpreter.xcodeProjectInfo = XcodeProjectInfo(
            <String>[],
            <String>[],
            <String>['Runner'],
            logger,
          );
          testPlistUtils.setProperty(
            PlistParser.kCFBundleIdentifierKey,
            r'$(PRODUCT_BUNDLE_IDENTIFIER).$(SUFFIX)',
          );
          testPlistUtils.setProperty(PlistParser.kAssociatedDomainsKey, <String>[
            'applinks:example.com',
            'applinks:example2.com',
            'applinks:example3.com?mode=developer',
          ]);
          final String outputFilePath = await project.ios.outputsUniversalLinkSettings(
            target: 'Runner',
            configuration: 'config',
          );
          final File outputFile = fs.file(outputFilePath);
          final Map<String, Object?> json =
              jsonDecode(outputFile.readAsStringSync()) as Map<String, Object?>;

          expect(
            json['associatedDomains'],
            unorderedEquals(<String>['example.com', 'example2.com', 'example3.com']),
          );
          expect(json['teamIdentifier'], 'ABC');
          expect(json['bundleIdentifier'], 'io.flutter.someProject.suffix');
        });

        testWithMocks('can handle entitlement file in nested directory structure.', () async {
          final FlutterProject project = await someProject();
          project.ios.xcodeProject.createSync();
          project.ios.defaultHostInfoPlist.createSync(recursive: true);
          const String entitlementFilePath = 'nested/somewhere/myEntitlement.Entitlement';
          project.ios.hostAppRoot.childFile(entitlementFilePath).createSync(recursive: true);

          const XcodeProjectBuildContext buildContext = XcodeProjectBuildContext(
            target: 'Runner',
            configuration: 'config',
          );
          xcodeProjectInterpreter.buildSettingsByBuildContext[buildContext] = <String, String>{
            IosProject.kProductBundleIdKey: 'io.flutter.someProject',
            IosProject.kTeamIdKey: 'ABC',
            IosProject.kEntitlementFilePathKey: entitlementFilePath,
            'SUFFIX': 'suffix',
          };
          xcodeProjectInterpreter.xcodeProjectInfo = XcodeProjectInfo(
            <String>[],
            <String>[],
            <String>['Runner'],
            logger,
          );
          testPlistUtils.setProperty(
            PlistParser.kCFBundleIdentifierKey,
            r'$(PRODUCT_BUNDLE_IDENTIFIER).$(SUFFIX)',
          );
          testPlistUtils.setProperty(PlistParser.kAssociatedDomainsKey, <String>[
            'applinks:example.com',
            'applinks:example2.com',
          ]);

          final String outputFilePath = await project.ios.outputsUniversalLinkSettings(
            target: 'Runner',
            configuration: 'config',
          );
          final File outputFile = fs.file(outputFilePath);
          final Map<String, Object?> json =
              jsonDecode(outputFile.readAsStringSync()) as Map<String, Object?>;
          expect(
            json['associatedDomains'],
            unorderedEquals(<String>['example.com', 'example2.com']),
          );
          expect(json['teamIdentifier'], 'ABC');
          expect(json['bundleIdentifier'], 'io.flutter.someProject.suffix');
        });

        testWithMocks('return empty when no entitlement', () async {
          final FlutterProject project = await someProject();
          project.ios.xcodeProject.createSync();
          project.ios.defaultHostInfoPlist.createSync(recursive: true);

          const XcodeProjectBuildContext buildContext = XcodeProjectBuildContext(
            target: 'Runner',
            configuration: 'config',
          );
          xcodeProjectInterpreter.buildSettingsByBuildContext[buildContext] = <String, String>{
            IosProject.kProductBundleIdKey: 'io.flutter.someProject',
            IosProject.kTeamIdKey: 'ABC',
          };
          xcodeProjectInterpreter.xcodeProjectInfo = XcodeProjectInfo(
            <String>[],
            <String>[],
            <String>['Runner'],
            logger,
          );
          testPlistUtils.setProperty(
            PlistParser.kCFBundleIdentifierKey,
            r'$(PRODUCT_BUNDLE_IDENTIFIER)',
          );
          final String outputFilePath = await project.ios.outputsUniversalLinkSettings(
            target: 'Runner',
            configuration: 'config',
          );
          final File outputFile = fs.file(outputFilePath);
          final Map<String, Object?> json =
              jsonDecode(outputFile.readAsStringSync()) as Map<String, Object?>;
          expect(json['teamIdentifier'], 'ABC');
          expect(json['bundleIdentifier'], 'io.flutter.someProject');
          expect(json['associatedDomains'], unorderedEquals(<String>[]));
        });
      });

      group('product bundle identifier', () {
        testWithMocks('null, if no build settings or plist entries', () async {
          final FlutterProject project = await someProject();
          expect(await project.ios.productBundleIdentifier(null), isNull);
        });

        testWithMocks('from build settings, if no plist', () async {
          final FlutterProject project = await someProject();
          project.ios.xcodeProject.createSync();
          const XcodeProjectBuildContext buildContext = XcodeProjectBuildContext(scheme: 'Runner');
          xcodeProjectInterpreter.buildSettingsByBuildContext[buildContext] = <String, String>{
            IosProject.kProductBundleIdKey: 'io.flutter.someProject',
          };
          xcodeProjectInterpreter.xcodeProjectInfo = XcodeProjectInfo(
            <String>[],
            <String>[],
            <String>['Runner'],
            logger,
          );

          expect(await project.ios.productBundleIdentifier(null), 'io.flutter.someProject');
        });

        testWithMocks('from project file, if no plist or build settings', () async {
          final FlutterProject project = await someProject();
          xcodeProjectInterpreter.xcodeProjectInfo = XcodeProjectInfo(
            <String>[],
            <String>[],
            <String>['Runner'],
            logger,
          );

          addIosProjectFile(
            project.directory,
            projectFileContent: () {
              return projectFileWithBundleId('io.flutter.someProject');
            },
          );
          expect(await project.ios.productBundleIdentifier(null), 'io.flutter.someProject');
        });

        testWithMocks('from plist, if no variables', () async {
          final FlutterProject project = await someProject();
          project.ios.defaultHostInfoPlist.createSync(recursive: true);
          testPlistUtils.setProperty('CFBundleIdentifier', 'io.flutter.someProject');
          expect(await project.ios.productBundleIdentifier(null), 'io.flutter.someProject');
        });

        testWithMocks('from build settings and plist, if default variable', () async {
          final FlutterProject project = await someProject();
          project.ios.xcodeProject.createSync();
          const XcodeProjectBuildContext buildContext = XcodeProjectBuildContext(scheme: 'Runner');
          xcodeProjectInterpreter.buildSettingsByBuildContext[buildContext] = <String, String>{
            IosProject.kProductBundleIdKey: 'io.flutter.someProject',
          };
          xcodeProjectInterpreter.xcodeProjectInfo = XcodeProjectInfo(
            <String>[],
            <String>[],
            <String>['Runner'],
            logger,
          );
          testPlistUtils.setProperty('CFBundleIdentifier', r'$(PRODUCT_BUNDLE_IDENTIFIER)');

          expect(await project.ios.productBundleIdentifier(null), 'io.flutter.someProject');
        });

        testWithMocks('from build settings and plist, by substitution', () async {
          final FlutterProject project = await someProject();
          project.ios.xcodeProject.createSync();
          project.ios.defaultHostInfoPlist.createSync(recursive: true);
          const XcodeProjectBuildContext buildContext = XcodeProjectBuildContext(scheme: 'Runner');
          xcodeProjectInterpreter.buildSettingsByBuildContext[buildContext] = <String, String>{
            IosProject.kProductBundleIdKey: 'io.flutter.someProject',
            'SUFFIX': 'suffix',
          };
          xcodeProjectInterpreter.xcodeProjectInfo = XcodeProjectInfo(
            <String>[],
            <String>[],
            <String>['Runner'],
            logger,
          );
          testPlistUtils.setProperty(
            'CFBundleIdentifier',
            r'$(PRODUCT_BUNDLE_IDENTIFIER).$(SUFFIX)',
          );

          expect(await project.ios.productBundleIdentifier(null), 'io.flutter.someProject.suffix');
        });

        testWithMocks('Always pass parsing org on ios project with flavors', () async {
          final FlutterProject project = await someProject();
          addIosProjectFile(
            project.directory,
            projectFileContent: () {
              return projectFileWithBundleId('io.flutter.someProject', qualifier: "'");
            },
          );
          project.ios.xcodeProject.createSync();
          xcodeProjectInterpreter.xcodeProjectInfo = XcodeProjectInfo(
            <String>[],
            <String>[],
            <String>['free', 'paid'],
            logger,
          );

          expect(await project.organizationNames, <String>[]);
        });

        testWithMocks('fails with no flavor and defined schemes', () async {
          final FlutterProject project = await someProject();
          project.ios.xcodeProject.createSync();
          xcodeProjectInterpreter.xcodeProjectInfo = XcodeProjectInfo(
            <String>[],
            <String>[],
            <String>['free', 'paid'],
            logger,
          );

          await expectToolExitLater(
            project.ios.productBundleIdentifier(null),
            contains('You must specify a --flavor option to select one of the available schemes.'),
          );
        });

        testWithMocks('handles case insensitive flavor', () async {
          final FlutterProject project = await someProject();
          project.ios.xcodeProject.createSync();
          const XcodeProjectBuildContext buildContext = XcodeProjectBuildContext(scheme: 'Free');
          xcodeProjectInterpreter.buildSettingsByBuildContext[buildContext] = <String, String>{
            IosProject.kProductBundleIdKey: 'io.flutter.someProject',
          };
          xcodeProjectInterpreter.xcodeProjectInfo = XcodeProjectInfo(
            <String>[],
            <String>[],
            <String>['Free'],
            logger,
          );
          const BuildInfo buildInfo = BuildInfo(
            BuildMode.debug,
            'free',
            treeShakeIcons: false,
            packageConfigPath: '.dart_tool/package_config.json',
          );

          expect(await project.ios.productBundleIdentifier(buildInfo), 'io.flutter.someProject');
        });

        testWithMocks('fails with flavor and default schemes', () async {
          final FlutterProject project = await someProject();
          project.ios.xcodeProject.createSync();
          xcodeProjectInterpreter.xcodeProjectInfo = XcodeProjectInfo(
            <String>[],
            <String>[],
            <String>['Runner'],
            logger,
          );
          const BuildInfo buildInfo = BuildInfo(
            BuildMode.debug,
            'free',
            treeShakeIcons: false,
            packageConfigPath: '.dart_tool/package_config.json',
          );

          await expectToolExitLater(
            project.ios.productBundleIdentifier(buildInfo),
            contains(
              'The Xcode project does not define custom schemes. You cannot use the --flavor option.',
            ),
          );
        });

        testWithMocks('empty surrounded by quotes', () async {
          final FlutterProject project = await someProject();
          xcodeProjectInterpreter.xcodeProjectInfo = XcodeProjectInfo(
            <String>[],
            <String>[],
            <String>['Runner'],
            logger,
          );
          addIosProjectFile(
            project.directory,
            projectFileContent: () {
              return projectFileWithBundleId('', qualifier: '"');
            },
          );
          expect(await project.ios.productBundleIdentifier(null), '');
        });

        testWithMocks('surrounded by double quotes', () async {
          final FlutterProject project = await someProject();
          xcodeProjectInterpreter.xcodeProjectInfo = XcodeProjectInfo(
            <String>[],
            <String>[],
            <String>['Runner'],
            logger,
          );
          addIosProjectFile(
            project.directory,
            projectFileContent: () {
              return projectFileWithBundleId('io.flutter.someProject', qualifier: '"');
            },
          );
          expect(await project.ios.productBundleIdentifier(null), 'io.flutter.someProject');
        });

        testWithMocks('surrounded by single quotes', () async {
          final FlutterProject project = await someProject();
          xcodeProjectInterpreter.xcodeProjectInfo = XcodeProjectInfo(
            <String>[],
            <String>[],
            <String>['Runner'],
            logger,
          );
          addIosProjectFile(
            project.directory,
            projectFileContent: () {
              return projectFileWithBundleId('io.flutter.someProject', qualifier: "'");
            },
          );
          expect(await project.ios.productBundleIdentifier(null), 'io.flutter.someProject');
        });
      });

      group('flutterSwiftPackageInProjectSettings', () {
        testWithMocks('is false if pbxproj missing', () async {
          final FlutterProject project = await someProject();
          expect(project.ios.xcodeProjectInfoFile.existsSync(), isFalse);
          expect(project.ios.flutterPluginSwiftPackageInProjectSettings, isFalse);
        });

        testWithMocks(
          'is false if pbxproj does not contain FlutterGeneratedPluginSwiftPackage in build process',
          () async {
            final FlutterProject project = await someProject();
            project.ios.xcodeProjectInfoFile.createSync(recursive: true);
            expect(project.ios.xcodeProjectInfoFile.existsSync(), isTrue);
            expect(project.ios.flutterPluginSwiftPackageInProjectSettings, isFalse);
          },
        );

        testWithMocks(
          'is true if pbxproj does contain FlutterGeneratedPluginSwiftPackage in build process',
          () async {
            final FlutterProject project = await someProject();
            project.ios.xcodeProjectInfoFile.createSync(recursive: true);
            project.ios.xcodeProjectInfoFile.writeAsStringSync('''
'		78A318202AECB46A00862997 /* FlutterGeneratedPluginSwiftPackage in Frameworks */ = {isa = PBXBuildFile; productRef = 78A3181F2AECB46A00862997 /* FlutterGeneratedPluginSwiftPackage */; };';
''');
            expect(project.ios.xcodeProjectInfoFile.existsSync(), isTrue);
            expect(project.ios.flutterPluginSwiftPackageInProjectSettings, isTrue);
          },
        );
      });
    });

    group('application bundle name', () {
      late MemoryFileSystem fs;
      late FakeXcodeProjectInterpreter mockXcodeProjectInterpreter;
      setUp(() {
        fs = MemoryFileSystem.test();
        mockXcodeProjectInterpreter = FakeXcodeProjectInterpreter();
      });

      testUsingContext(
        'app product name defaults to Runner',
        () async {
          final FlutterProject project = await someProject();
          expect(await project.ios.productName(null), 'Runner');
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          XcodeProjectInterpreter: () => mockXcodeProjectInterpreter,
        },
      );

      testUsingContext(
        'app product name xcodebuild settings',
        () async {
          final FlutterProject project = await someProject();
          project.ios.xcodeProject.createSync();
          const XcodeProjectBuildContext buildContext = XcodeProjectBuildContext(scheme: 'Runner');
          mockXcodeProjectInterpreter.buildSettingsByBuildContext[buildContext] = <String, String>{
            'PRODUCT_NAME': 'My App',
          };
          mockXcodeProjectInterpreter.xcodeProjectInfo = XcodeProjectInfo(
            <String>[],
            <String>[],
            <String>['Runner'],
            logger,
          );

          expect(await project.ios.productName(null), 'My App');
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          XcodeProjectInterpreter: () => mockXcodeProjectInterpreter,
        },
      );
    });

    group('organization names set', () {
      _testInMemory('is empty, if project not created', () async {
        final FlutterProject project = await someProject();
        expect(await project.organizationNames, isEmpty);
      });
      _testInMemory('is empty, if no platform folders exist', () async {
        final FlutterProject project = await someProject();
        project.directory.createSync();
        expect(await project.organizationNames, isEmpty);
      });
      _testInMemory('is populated from iOS bundle identifier', () async {
        final FlutterProject project = await someProject();
        addIosProjectFile(
          project.directory,
          projectFileContent: () {
            return projectFileWithBundleId('io.flutter.someProject', qualifier: "'");
          },
        );
        expect(await project.organizationNames, <String>['io.flutter']);
      });
      _testInMemory('is populated from Android application ID', () async {
        final FlutterProject project = await someProject();
        addAndroidGradleFile(
          project.directory,
          gradleFileContent: () {
            return gradleFileWithApplicationId('io.flutter.someproject');
          },
        );
        expect(await project.organizationNames, <String>['io.flutter']);
      });
      _testInMemory('is populated from iOS bundle identifier in plugin example', () async {
        final FlutterProject project = await someProject();
        addIosProjectFile(
          project.example.directory,
          projectFileContent: () {
            return projectFileWithBundleId('io.flutter.someProject', qualifier: "'");
          },
        );
        expect(await project.organizationNames, <String>['io.flutter']);
      });
      _testInMemory('is populated from Android application ID in plugin example', () async {
        final FlutterProject project = await someProject();
        addAndroidGradleFile(
          project.example.directory,
          gradleFileContent: () {
            return gradleFileWithApplicationId('io.flutter.someproject');
          },
        );
        expect(await project.organizationNames, <String>['io.flutter']);
      });
      _testInMemory('is populated from Android group in plugin', () async {
        final FlutterProject project = await someProject();
        addAndroidWithGroup(project.directory, 'io.flutter.someproject');
        expect(await project.organizationNames, <String>['io.flutter']);
      });
      _testInMemory('is singleton, if sources agree', () async {
        final FlutterProject project = await someProject();
        addIosProjectFile(
          project.directory,
          projectFileContent: () {
            return projectFileWithBundleId('io.flutter.someProject');
          },
        );
        addAndroidGradleFile(
          project.directory,
          gradleFileContent: () {
            return gradleFileWithApplicationId('io.flutter.someproject');
          },
        );
        expect(await project.organizationNames, <String>['io.flutter']);
      });
      _testInMemory('is non-singleton, if sources disagree', () async {
        final FlutterProject project = await someProject();
        addIosProjectFile(
          project.directory,
          projectFileContent: () {
            return projectFileWithBundleId('io.flutter.someProject');
          },
        );
        addAndroidGradleFile(
          project.directory,
          gradleFileContent: () {
            return gradleFileWithApplicationId('io.clutter.someproject');
          },
        );
        expect(await project.organizationNames, <String>['io.flutter', 'io.clutter']);
      });
    });

    group('manifest', () {
      _testInMemory('can be replaced', () async {
        final FlutterProject project = await someProject(includePubspec: true);
        final String originalPubspecContents = project.pubspecFile.readAsStringSync();
        final FlutterManifest updated =
            FlutterManifest.createFromString(validPubspecWithDependencies, logger: logger)!;
        // Verifies the pubspec.yaml from [project] is overwritten with the pubspec
        // defined by [updated], both in the [FlutterProject] and on disk.
        expect(project.manifest, isNot(equals(updated)));
        project.replacePubspec(updated);
        expect(project.manifest, equals(updated));
        final String updatedPubspecContents = project.pubspecFile.readAsStringSync();
        expect(updatedPubspecContents, isNot(equals(originalPubspecContents)));
        expect(updatedPubspecContents, validPubspecWithDependenciesAndNullValues);
      });
    });
  });

  group('watch companion', () {
    late MemoryFileSystem fs;
    late FakePlistParser testPlistParser;
    late FakeXcodeProjectInterpreter mockXcodeProjectInterpreter;
    late FlutterProjectFactory flutterProjectFactory;
    setUp(() {
      fs = MemoryFileSystem.test();
      testPlistParser = FakePlistParser();
      mockXcodeProjectInterpreter = FakeXcodeProjectInterpreter();
      flutterProjectFactory = FlutterProjectFactory(fileSystem: fs, logger: logger);
    });

    testUsingContext(
      'cannot find bundle identifier',
      () async {
        final FlutterProject project = await someProject();
        final XcodeProjectInfo projectInfo = XcodeProjectInfo(
          <String>['WatchTarget'],
          <String>[],
          <String>[],
          logger,
        );
        expect(
          await project.ios.containsWatchCompanion(
            projectInfo: projectInfo,
            buildInfo: BuildInfo.debug,
            deviceId: '123',
          ),
          isFalse,
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fs,
        ProcessManager: () => FakeProcessManager.any(),
        PlistParser: () => testPlistParser,
        XcodeProjectInterpreter: () => mockXcodeProjectInterpreter,
        FlutterProjectFactory: () => flutterProjectFactory,
      },
    );

    group('with bundle identifier', () {
      setUp(() {
        const XcodeProjectBuildContext buildContext = XcodeProjectBuildContext(scheme: 'Runner');
        mockXcodeProjectInterpreter.buildSettingsByBuildContext[buildContext] = <String, String>{
          IosProject.kProductBundleIdKey: 'io.flutter.someProject',
        };
        mockXcodeProjectInterpreter.xcodeProjectInfo = XcodeProjectInfo(
          <String>['Runner', 'WatchTarget'],
          <String>[],
          <String>['Runner', 'WatchScheme'],
          logger,
        );
      });

      testUsingContext(
        'no Info.plist in target',
        () async {
          final FlutterProject project = await someProject();
          expect(
            await project.ios.containsWatchCompanion(
              projectInfo: mockXcodeProjectInterpreter.xcodeProjectInfo,
              buildInfo: BuildInfo.debug,
              deviceId: '123',
            ),
            isFalse,
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          PlistParser: () => testPlistParser,
          XcodeProjectInterpreter: () => mockXcodeProjectInterpreter,
          FlutterProjectFactory: () => flutterProjectFactory,
        },
      );

      testUsingContext(
        'Info.plist in target does not contain WKCompanionAppBundleIdentifier',
        () async {
          final FlutterProject project = await someProject();
          project.ios.hostAppRoot
              .childDirectory('WatchTarget')
              .childFile('Info.plist')
              .createSync(recursive: true);

          expect(
            await project.ios.containsWatchCompanion(
              projectInfo: mockXcodeProjectInterpreter.xcodeProjectInfo,
              buildInfo: BuildInfo.debug,
              deviceId: '123',
            ),
            isFalse,
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          PlistParser: () => testPlistParser,
          XcodeProjectInterpreter: () => mockXcodeProjectInterpreter,
          FlutterProjectFactory: () => flutterProjectFactory,
        },
      );

      testUsingContext(
        'target WKCompanionAppBundleIdentifier is not project bundle identifier',
        () async {
          final FlutterProject project = await someProject();
          project.ios.hostAppRoot
              .childDirectory('WatchTarget')
              .childFile('Info.plist')
              .createSync(recursive: true);

          testPlistParser.setProperty(
            'WKCompanionAppBundleIdentifier',
            'io.flutter.someOTHERproject',
          );
          expect(
            await project.ios.containsWatchCompanion(
              projectInfo: mockXcodeProjectInterpreter.xcodeProjectInfo,
              buildInfo: BuildInfo.debug,
              deviceId: '123',
            ),
            isFalse,
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          PlistParser: () => testPlistParser,
          XcodeProjectInterpreter: () => mockXcodeProjectInterpreter,
          FlutterProjectFactory: () => flutterProjectFactory,
        },
      );

      testUsingContext(
        'has watch companion in plist',
        () async {
          final FlutterProject project = await someProject();
          project.ios.xcodeProject.createSync();
          project.ios.hostAppRoot
              .childDirectory('WatchTarget')
              .childFile('Info.plist')
              .createSync(recursive: true);
          testPlistParser.setProperty('WKCompanionAppBundleIdentifier', 'io.flutter.someProject');

          expect(
            await project.ios.containsWatchCompanion(
              projectInfo: mockXcodeProjectInterpreter.xcodeProjectInfo,
              buildInfo: BuildInfo.debug,
              deviceId: '123',
            ),
            isTrue,
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          PlistParser: () => testPlistParser,
          XcodeProjectInterpreter: () => mockXcodeProjectInterpreter,
          FlutterProjectFactory: () => flutterProjectFactory,
        },
      );

      testUsingContext(
        'has watch companion in plist with xcode variable',
        () async {
          final FlutterProject project = await someProject();
          project.ios.xcodeProject.createSync();
          const XcodeProjectBuildContext buildContext = XcodeProjectBuildContext(
            scheme: 'Runner',
            deviceId: '123',
          );
          mockXcodeProjectInterpreter.buildSettingsByBuildContext[buildContext] = <String, String>{
            IosProject.kProductBundleIdKey: 'io.flutter.someProject',
          };
          project.ios.hostAppRoot
              .childDirectory('WatchTarget')
              .childFile('Info.plist')
              .createSync(recursive: true);
          testPlistParser.setProperty(
            'WKCompanionAppBundleIdentifier',
            r'$(PRODUCT_BUNDLE_IDENTIFIER)',
          );

          expect(
            await project.ios.containsWatchCompanion(
              projectInfo: mockXcodeProjectInterpreter.xcodeProjectInfo,
              buildInfo: BuildInfo.debug,
              deviceId: '123',
            ),
            isTrue,
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          PlistParser: () => testPlistParser,
          XcodeProjectInterpreter: () => mockXcodeProjectInterpreter,
          FlutterProjectFactory: () => flutterProjectFactory,
        },
      );

      testUsingContext(
        'has watch companion in other scheme build settings',
        () async {
          final FlutterProject project = await someProject();
          project.ios.xcodeProject.createSync();
          project.ios.xcodeProjectInfoFile.writeAsStringSync('''
        Build settings for action build and target "WatchTarget":
            INFOPLIST_KEY_WKCompanionAppBundleIdentifier = io.flutter.someProject
''');

          const XcodeProjectBuildContext buildContext = XcodeProjectBuildContext(
            scheme: 'Runner',
            deviceId: '123',
          );
          mockXcodeProjectInterpreter.buildSettingsByBuildContext[buildContext] = <String, String>{
            IosProject.kProductBundleIdKey: 'io.flutter.someProject',
          };

          const XcodeProjectBuildContext watchBuildContext = XcodeProjectBuildContext(
            scheme: 'WatchScheme',
            deviceId: '123',
            sdk: XcodeSdk.WatchOS,
          );
          mockXcodeProjectInterpreter
              .buildSettingsByBuildContext[watchBuildContext] = <String, String>{
            'INFOPLIST_KEY_WKCompanionAppBundleIdentifier': 'io.flutter.someProject',
          };

          expect(
            await project.ios.containsWatchCompanion(
              projectInfo: mockXcodeProjectInterpreter.xcodeProjectInfo,
              buildInfo: BuildInfo.debug,
              deviceId: '123',
            ),
            isTrue,
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          PlistParser: () => testPlistParser,
          XcodeProjectInterpreter: () => mockXcodeProjectInterpreter,
          FlutterProjectFactory: () => flutterProjectFactory,
        },
      );

      testUsingContext(
        'has watch companion in other scheme build settings with xcode variable',
        () async {
          final FlutterProject project = await someProject();
          project.ios.xcodeProject.createSync();
          project.ios.xcodeProjectInfoFile.writeAsStringSync(r'''
        Build settings for action build and target "WatchTarget":
            INFOPLIST_KEY_WKCompanionAppBundleIdentifier = $(PRODUCT_BUNDLE_IDENTIFIER)
''');
          const XcodeProjectBuildContext buildContext = XcodeProjectBuildContext(
            scheme: 'Runner',
            deviceId: '123',
          );
          mockXcodeProjectInterpreter.buildSettingsByBuildContext[buildContext] = <String, String>{
            IosProject.kProductBundleIdKey: 'io.flutter.someProject',
          };

          const XcodeProjectBuildContext watchBuildContext = XcodeProjectBuildContext(
            scheme: 'WatchScheme',
            deviceId: '123',
            sdk: XcodeSdk.WatchOS,
          );
          mockXcodeProjectInterpreter
              .buildSettingsByBuildContext[watchBuildContext] = <String, String>{
            IosProject.kProductBundleIdKey: 'io.flutter.someProject',
            'INFOPLIST_KEY_WKCompanionAppBundleIdentifier': r'$(PRODUCT_BUNDLE_IDENTIFIER)',
          };

          expect(
            await project.ios.containsWatchCompanion(
              projectInfo: mockXcodeProjectInterpreter.xcodeProjectInfo,
              buildInfo: BuildInfo.debug,
              deviceId: '123',
            ),
            isTrue,
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fs,
          ProcessManager: () => FakeProcessManager.any(),
          PlistParser: () => testPlistParser,
          XcodeProjectInterpreter: () => mockXcodeProjectInterpreter,
          FlutterProjectFactory: () => flutterProjectFactory,
        },
      );
    });
  });
}

Future<FlutterProject> someProject({
  String? androidManifestOverride,
  bool includePubspec = false,
}) async {
  final Directory directory = globals.fs.directory('some_project');
  directory.childDirectory('.dart_tool').childFile('package_config.json')
    ..createSync(recursive: true)
    ..writeAsStringSync('{"configVersion":2,"packages":[]}');
  if (includePubspec) {
    directory.childFile('pubspec.yaml')
      ..createSync(recursive: true)
      ..writeAsStringSync(validPubspec);
  }
  directory.childDirectory('ios').createSync(recursive: true);
  final Directory androidDirectory = directory.childDirectory('android')
    ..createSync(recursive: true);
  androidDirectory
      .childFile('AndroidManifest.xml')
      .writeAsStringSync(androidManifestOverride ?? '<manifest></manifest>');
  return FlutterProject.fromDirectory(directory);
}

Future<FlutterProject> aPluginProject({bool legacy = true}) async {
  final Directory directory = globals.fs.directory('plugin_project');
  directory.childDirectory('ios').createSync(recursive: true);
  directory.childDirectory('android').createSync(recursive: true);
  directory.childDirectory('example').createSync(recursive: true);
  String pluginPubSpec;
  if (legacy) {
    pluginPubSpec = '''
name: my_plugin
flutter:
  plugin:
    androidPackage: com.example
    pluginClass: MyPlugin
    iosPrefix: FLT
''';
  } else {
    pluginPubSpec = '''
name: my_plugin
flutter:
  plugin:
    platforms:
      android:
        package: com.example
        pluginClass: MyPlugin
      ios:
        pluginClass: MyPlugin
      linux:
        pluginClass: MyPlugin
      macos:
        pluginClass: MyPlugin
      windows:
        pluginClass: MyPlugin
''';
  }
  directory.childFile('pubspec.yaml').writeAsStringSync(pluginPubSpec);
  return FlutterProject.fromDirectory(directory);
}

Future<FlutterProject> aModuleProject() async {
  final Directory directory = globals.fs.directory('module_project');
  directory.childDirectory('.dart_tool').childFile('package_config.json')
    ..createSync(recursive: true)
    ..writeAsStringSync('{"configVersion":2,"packages":[]}');
  directory.childFile('pubspec.yaml').writeAsStringSync('''
name: my_module
flutter:
  module:
    androidPackage: com.example
''');
  return FlutterProject.fromDirectory(directory);
}

/// Executes the [testMethod] in a context where the file system
/// is in memory.
@isTest
void _testInMemory(
  String description,
  Future<void> Function() testMethod, {
  FileSystem? fileSystem,
  Java? java,
  AndroidStudio? androidStudio,
  ProcessManager? processManager,
  AndroidSdk? androidSdk,
}) {
  Cache.flutterRoot = getFlutterRoot();
  final FileSystem testFileSystem = fileSystem ?? getFileSystemForPlatform();
  testFileSystem.directory('.dart_tool').childFile('package_config.json')
    ..createSync(recursive: true)
    ..writeAsStringSync('{"configVersion":2,"packages":[]}');
  // Transfer needed parts of the Flutter installation folder
  // to the in-memory file system used during testing.
  final Logger logger = BufferLogger.test();
  transfer(
    Cache(
      fileSystem: globals.fs,
      logger: logger,
      artifacts: <ArtifactSet>[],
      osUtils: OperatingSystemUtils(
        fileSystem: globals.fs,
        logger: logger,
        platform: globals.platform,
        processManager: globals.processManager,
      ),
      platform: globals.platform,
    ).getArtifactDirectory('gradle_wrapper'),
    testFileSystem,
  );
  transfer(
    globals.fs
        .directory(Cache.flutterRoot)
        .childDirectory('packages')
        .childDirectory('flutter_tools')
        .childDirectory('templates'),
    testFileSystem,
  );
  // Set up enough of the packages to satisfy the templating code.
  final File packagesFile = testFileSystem
      .directory(Cache.flutterRoot)
      .childDirectory('packages')
      .childDirectory('flutter_tools')
      .childDirectory('.dart_tool')
      .childFile('package_config.json');
  final Directory dummyTemplateImagesDirectory = testFileSystem.directory(Cache.flutterRoot).parent;
  dummyTemplateImagesDirectory.createSync(recursive: true);
  packagesFile.createSync(recursive: true);
  packagesFile.writeAsStringSync(
    json.encode(<String, Object>{
      'configVersion': 2,
      'packages': <Object>[
        <String, Object>{
          'name': 'flutter_template_images',
          'rootUri': dummyTemplateImagesDirectory.uri.toString(),
          'packageUri': 'lib/',
          'languageVersion': '2.6',
        },
      ],
    }),
  );

  testUsingContext(
    description,
    testMethod,
    overrides: <Type, Generator>{
      FileSystem: () => testFileSystem,
      ProcessManager: () => processManager ?? FakeProcessManager.any(),
      Java: () => java,
      AndroidStudio: () => androidStudio ?? FakeAndroidStudio(),
      // Intentionally null if not set. Some ios tests fail if this is a fake.
      AndroidSdk: () => androidSdk,
      Cache:
          () => Cache(
            logger: globals.logger,
            fileSystem: testFileSystem,
            osUtils: globals.os,
            platform: globals.platform,
            artifacts: <ArtifactSet>[],
          ),
      FlutterProjectFactory:
          () => FlutterProjectFactory(fileSystem: testFileSystem, logger: globals.logger),
      // TODO(matanlurey): Remove after `explicit-package-dependencies` is enabled by default.
      // See https://github.com/flutter/flutter/issues/160257 for details.
      FeatureFlags: () => TestFeatureFlags(isExplicitPackageDependenciesEnabled: true),
      Pub: FakePubWithPrimedDeps.new,
    },
  );
}

/// Transfers files and folders from the local file system's Flutter
/// installation to an (in-memory) file system used for testing.
void transfer(FileSystemEntity entity, FileSystem target) {
  if (entity is Directory) {
    target.directory(entity.absolute.path).createSync(recursive: true);
    for (final FileSystemEntity child in entity.listSync()) {
      transfer(child, target);
    }
  } else if (entity is File) {
    target.file(entity.absolute.path).writeAsBytesSync(entity.readAsBytesSync(), flush: true);
  } else {
    throw Exception('Unsupported FileSystemEntity ${entity.runtimeType}');
  }
}

void expectExists(FileSystemEntity entity) {
  expect(entity.existsSync(), isTrue);
}

void expectNotExists(FileSystemEntity entity) {
  expect(entity.existsSync(), isFalse);
}

void addIosProjectFile(Directory directory, {required String Function() projectFileContent}) {
  directory.childDirectory('ios').childDirectory('Runner.xcodeproj').childFile('project.pbxproj')
    ..createSync(recursive: true)
    ..writeAsStringSync(projectFileContent());
}

/// Adds app-level Gradle Groovy build file (build.gradle) to [directory].
///
/// If [kotlinDsl] is true, then build.gradle.kts is created instead of
/// build.gradle. It's the caller's responsibility to make sure that
/// [gradleFileContent] is consistent with the value of the [kotlinDsl] flag.
void addAndroidGradleFile(
  Directory directory, {
  required String Function() gradleFileContent,
  bool kotlinDsl = false,
}) {
  directory
      .childDirectory('android')
      .childDirectory('app')
      .childFile(kotlinDsl ? 'build.gradle.kts' : 'build.gradle')
    ..createSync(recursive: true)
    ..writeAsStringSync(gradleFileContent());
}

void addRootGradleFile(Directory directory, {required String Function() gradleFileContent}) {
  directory.childDirectory('android').childFile('build.gradle')
    ..createSync(recursive: true)
    ..writeAsStringSync(gradleFileContent());
}

void addGradleWrapperFile(Directory directory, String gradleVersion) {
  directory
      .childDirectory('android')
      .childDirectory(gradle_utils.gradleDirectoryName)
      .childDirectory(gradle_utils.gradleWrapperDirectoryName)
      .childFile(gradle_utils.gradleWrapperPropertiesFilename)
    ..createSync(recursive: true)
    ..writeAsStringSync('''
distributionBase=GRADLE_USER_HOME
distributionPath=wrapper/dists
zipStoreBase=GRADLE_USER_HOME
zipStorePath=wrapper/dists
distributionUrl=https://services.gradle.org/distributions/gradle-$gradleVersion-all.zip
''');
}

FileSystem getFileSystemForPlatform() {
  return MemoryFileSystem(
    style: globals.platform.isWindows ? FileSystemStyle.windows : FileSystemStyle.posix,
  );
}

void addAndroidWithGroup(Directory directory, String id, {bool kotlinDsl = false}) {
  directory.childDirectory('android').childFile(kotlinDsl ? 'build.gradle.kts' : 'build.gradle')
    ..createSync(recursive: true)
    ..writeAsStringSync(gradleFileWithGroupId(id));
}

String get validPubspec => '''
name: hello
flutter:
''';

String get validPubspecWithDependencies => '''
name: hello
flutter:

dependencies:
  plugin_a:
  plugin_b:
''';

/// This is the equivalent to [validPubspecWithDependencies] after it's been
/// passed through [YamlEditor], which explicitly populates the null values
/// even if they were specified implicitly.
String get validPubspecWithDependenciesAndNullValues => '''
name: hello
flutter: null
dependencies:
  plugin_a: null
  plugin_b: null''';

String get invalidPubspec => '''
name: hello
flutter:
  invalid:
''';

String get parseErrorPubspec => '''
name: hello
# Whitespace is important.
flutter:
    something:
  something_else:
''';

String projectFileWithBundleId(String id, {String? qualifier}) {
  return '''
97C147061CF9000F007C117D /* Debug */ = {
  isa = XCBuildConfiguration;
  baseConfigurationReference = 9740EEB21CF90195004384FC /* Debug.xcconfig */;
  buildSettings = {
    PRODUCT_BUNDLE_IDENTIFIER = ${qualifier ?? ''}$id${qualifier ?? ''};
    PRODUCT_NAME = "\$(TARGET_NAME)";
  };
  name = Debug;
};
''';
}

String gradleFileWithApplicationId(String id) {
  return '''
apply plugin: 'com.android.application'
android {
    compileSdk 34

    defaultConfig {
        applicationId '$id'
    }
}
''';
}

String gradleFileWithGroupId(String id) {
  return '''
group '$id'
version '1.0-SNAPSHOT'

apply plugin: 'com.android.library'

android {
    compileSdk 34
}
''';
}

File androidPluginRegistrant(Directory parent) {
  return parent
      .childDirectory('src')
      .childDirectory('main')
      .childDirectory('java')
      .childDirectory('io')
      .childDirectory('flutter')
      .childDirectory('plugins')
      .childFile('GeneratedPluginRegistrant.java');
}

class FakeXcodeProjectInterpreter extends Fake implements XcodeProjectInterpreter {
  FakeXcodeProjectInterpreter({this.version});

  final Map<XcodeProjectBuildContext, Map<String, String>> buildSettingsByBuildContext =
      <XcodeProjectBuildContext, Map<String, String>>{};
  late XcodeProjectInfo xcodeProjectInfo;

  @override
  Future<Map<String, String>> getBuildSettings(
    String projectPath, {
    XcodeProjectBuildContext? buildContext,
    Duration timeout = const Duration(minutes: 1),
  }) async {
    if (buildSettingsByBuildContext[buildContext] == null) {
      return <String, String>{};
    }
    return buildSettingsByBuildContext[buildContext]!;
  }

  @override
  Future<XcodeProjectInfo> getInfo(String projectPath, {String? projectFilename}) async {
    return xcodeProjectInfo;
  }

  @override
  bool get isInstalled => true;

  @override
  Version? version;
}

class FakeAndroidSdkWithDir extends Fake implements AndroidSdk {
  FakeAndroidSdkWithDir(this._directory);

  final Directory _directory;

  @override
  Directory get directory => _directory;
}
