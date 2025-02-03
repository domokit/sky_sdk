// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io' as io;

import 'package:args/command_runner.dart';
import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/android/gradle_utils.dart'
    show
        templateAndroidGradlePluginVersion,
        templateAndroidGradlePluginVersionForModule,
        templateDefaultGradleVersion;
import 'package:flutter_tools/src/android/java.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/net.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/version.dart' as software;
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/create.dart';
import 'package:flutter_tools/src/commands/create_base.dart';
import 'package:flutter_tools/src/dart/pub.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/flutter_project_metadata.dart' show FlutterTemplateType;
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/version.dart';
import 'package:pub_semver/pub_semver.dart';
import 'package:pubspec_parse/pubspec_parse.dart';
import 'package:unified_analytics/unified_analytics.dart';
import 'package:uuid/uuid.dart';
import 'package:yaml/yaml.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_http_client.dart';
import '../../src/fakes.dart';
import '../../src/pubspec_schema.dart';
import '../../src/test_flutter_command_runner.dart';
import 'utils/project_testing_utils.dart';

const String _kNoPlatformsMessage =
    "You've created a plugin project that doesn't yet support any platforms.\n";
const String frameworkRevision = '12345678';
const String frameworkChannel = 'omega';
const String _kDisabledPlatformRequestedMessage =
    'currently not supported on your local environment.';
const String _kIncompatibleJavaVersionMessage =
    'The configured version of Java detected may conflict with the';
final String _kIncompatibleAgpVersionForModule =
    Version.parse(templateAndroidGradlePluginVersion) <
            Version.parse(templateAndroidGradlePluginVersionForModule)
        ? templateAndroidGradlePluginVersionForModule
        : templateAndroidGradlePluginVersion;

// This needs to be created from the local platform due to re-entrant flutter calls made in this test.
FakePlatform _kNoColorTerminalPlatform() =>
    FakePlatform.fromPlatform(const LocalPlatform())..stdoutSupportsAnsi = false;
FakePlatform _kNoColorTerminalMacOSPlatform() =>
    FakePlatform.fromPlatform(const LocalPlatform())
      ..stdoutSupportsAnsi = false
      ..operatingSystem = 'macos';

final Map<Type, Generator> noColorTerminalOverride = <Type, Generator>{
  Platform: _kNoColorTerminalPlatform,
};

const String samplesIndexJson = '''
[
  { "id": "sample1" },
  { "id": "sample2" }
]''';

/// These files are generated for all project types.
const List<String> flutterPluginsIgnores = <String>[
  '.flutter-plugins',
  '.flutter-plugins-dependencies',
];

void main() {
  late Directory tempDir;
  late Directory projectDir;
  late FakeFlutterVersion fakeFlutterVersion;
  late LoggingProcessManager loggingProcessManager;
  late FakeProcessManager fakeProcessManager;
  late BufferLogger logger;
  late FakeStdio mockStdio;
  late FakeAnalytics fakeAnalytics;

  setUpAll(() async {
    Cache.disableLocking();
    await ensureFlutterToolsSnapshot();
  });

  setUp(() {
    loggingProcessManager = LoggingProcessManager();
    logger = BufferLogger.test();
    tempDir = globals.fs.systemTempDirectory.createTempSync('flutter_tools_create_test.');
    projectDir = tempDir.childDirectory('flutter_project');
    fakeFlutterVersion = FakeFlutterVersion(
      frameworkRevision: frameworkRevision,
      branch: frameworkChannel,
    );
    fakeProcessManager = FakeProcessManager.empty();
    mockStdio = FakeStdio();
    fakeAnalytics = getInitializedFakeAnalyticsInstance(
      fs: MemoryFileSystem.test(),
      fakeFlutterVersion: fakeFlutterVersion,
    );

    // Most, but not all, tests will run some variant of "pub get" after creation,
    // which in turn will check for the presence of the Flutter SDK root. Without
    // this field set consistently, the order of the tests becomes important *or*
    // you need to remember to set it everywhere.
    Cache.flutterRoot = '../..';
  });

  tearDown(() {
    tryToDelete(tempDir);
  });

  tearDownAll(() async {
    await restoreFlutterToolsSnapshot();
  });

  test('createAndroidIdentifier emits a valid identifier', () {
    final String identifier = CreateBase.createAndroidIdentifier('42org', '8project');
    expect(identifier.contains('.'), isTrue);

    final RegExp startsWithLetter = RegExp(r'^[a-zA-Z][\w]*$');
    final List<String> segments = identifier.split('.');
    for (final String segment in segments) {
      expect(startsWithLetter.hasMatch(segment), isTrue);
    }
  });

  test('createUTIIdentifier emits a valid identifier', () {
    final String identifier = CreateBase.createUTIIdentifier('org@', 'project');
    expect(identifier.contains('.'), isTrue);
    expect(identifier.contains('@'), isFalse);
  });

  test('createWindowsIdentifier emits a GUID', () {
    final String identifier = CreateBase.createWindowsIdentifier('org', 'project');
    expect(Uuid.isValidUUID(fromString: identifier), isTrue);
  });

  testUsingContext(
    'tool exits on Windows if given a drive letter without a path',
    () async {
      // Must use LocalFileSystem as it is dependent on dart:io handling of
      // Windows paths, which the MemoryFileSystem does not implement
      final Directory workingDir = globals.fs.directory(r'X:\path\to\working\dir');
      // Must use [io.IOOverrides] as directory.absolute depends on Directory.current
      // from dart:io.
      await io.IOOverrides.runZoned<Future<void>>(() async {
        // Verify IOOverrides is working
        expect(io.Directory.current, workingDir);
        final CreateCommand command = CreateCommand();
        final CommandRunner<void> runner = createTestCommandRunner(command);
        const String driveName = 'X:';
        await expectToolExitLater(
          runner.run(<String>['create', '--project-name', 'test_app', '--offline', driveName]),
          contains('You attempted to create a flutter project at the path "$driveName"'),
        );
      }, getCurrentDirectory: () => workingDir);
    },
    overrides: <Type, Generator>{Logger: () => BufferLogger.test()},
    skip: !io.Platform.isWindows, // [intended] relies on Windows file system
  );

  // Verify that we create a default project ('app') that is
  // well-formed.
  testUsingContext(
    'can create a default project',
    () async {
      await _createAndAnalyzeProject(
        projectDir,
        <String>['-i', 'objc', '-a', 'java'],
        <String>[
          'analysis_options.yaml',
          'android/app/src/main/java/com/example/flutter_project/MainActivity.java',
          'android/app/src/main/java/io/flutter/plugins/GeneratedPluginRegistrant.java',
          'flutter_project.iml',
          'ios/Flutter/AppFrameworkInfo.plist',
          'ios/Runner/AppDelegate.m',
          'ios/Runner/GeneratedPluginRegistrant.h',
          'ios/Runner/Assets.xcassets/AppIcon.appiconset/Icon-App-1024x1024@1x.png',
          'ios/Runner/Assets.xcassets/LaunchImage.imageset/LaunchImage.png',
          'lib/main.dart',
        ],
        expectedGitignoreLines: flutterPluginsIgnores,
      );
      expect(logger.statusText, contains('In order to run your application, type:'));
      // Check that we're telling them about documentation
      expect(logger.statusText, contains('https://docs.flutter.dev/'));
      expect(logger.statusText, contains('https://api.flutter.dev/'));

      // Check for usage values sent in analytics
      expect(
        fakeAnalytics.sentEvents,
        contains(
          Event.commandUsageValues(
            workflow: 'create',
            commandHasTerminal: false,
            createAndroidLanguage: 'java',
            createIosLanguage: 'objc',
          ),
        ),
      );

      // Check that the tests run clean
      return _runFlutterTest(projectDir);
    },
    overrides: <Type, Generator>{
      Pub:
          () => Pub.test(
            fileSystem: globals.fs,
            logger: logger,
            processManager: globals.processManager,
            botDetector: globals.botDetector,
            platform: globals.platform,
            stdio: mockStdio,
          ),
      Logger: () => logger,
      Analytics: () => fakeAnalytics,
    },
  );

  testUsingContext(
    'can create a default project if empty directory exists',
    () async {
      await projectDir.create(recursive: true);
      await _createAndAnalyzeProject(
        projectDir,
        <String>['-i', 'objc', '-a', 'java'],
        <String>[
          'analysis_options.yaml',
          'android/app/src/main/java/com/example/flutter_project/MainActivity.java',
          'android/app/src/main/java/io/flutter/plugins/GeneratedPluginRegistrant.java',
          'flutter_project.iml',
          'ios/Flutter/AppFrameworkInfo.plist',
          'ios/Runner/AppDelegate.m',
          'ios/Runner/GeneratedPluginRegistrant.h',
        ],
      );
    },
    overrides: <Type, Generator>{
      Pub:
          () => Pub.test(
            fileSystem: globals.fs,
            logger: globals.logger,
            processManager: globals.processManager,
            botDetector: globals.botDetector,
            platform: globals.platform,
            stdio: mockStdio,
          ),
    },
  );

  testUsingContext(
    'creates a module project correctly',
    () async {
      await _createAndAnalyzeProject(
        projectDir,
        <String>['--template=module'],
        <String>[
          '.android/app/',
          '.gitignore',
          '.ios/Flutter',
          '.metadata',
          'analysis_options.yaml',
          'lib/main.dart',
          'pubspec.yaml',
          'README.md',
          'test/widget_test.dart',
        ],
        unexpectedPaths: <String>['android/', 'ios/'],
        expectedGitignoreLines: flutterPluginsIgnores,
      );
      return _runFlutterTest(projectDir);
    },
    overrides: <Type, Generator>{
      Pub:
          () => Pub.test(
            fileSystem: globals.fs,
            logger: globals.logger,
            processManager: globals.processManager,
            botDetector: globals.botDetector,
            platform: globals.platform,
            stdio: mockStdio,
          ),
    },
  );

  testUsingContext(
    'cannot create a project if non-empty non-project directory exists with .metadata',
    () async {
      await projectDir.absolute.childDirectory('blag').create(recursive: true);
      await projectDir.absolute.childFile('.metadata').writeAsString('project_type: blag\n');
      expect(
        () async => _createAndAnalyzeProject(
          projectDir,
          <String>[],
          <String>[],
          unexpectedPaths: <String>['android/', 'ios/', '.android/', '.ios/'],
        ),
        throwsToolExit(message: 'Sorry, unable to detect the type of project to recreate'),
      );
    },
    overrides: <Type, Generator>{
      Pub:
          () => Pub.test(
            fileSystem: globals.fs,
            logger: globals.logger,
            processManager: globals.processManager,
            botDetector: globals.botDetector,
            platform: globals.platform,
            stdio: mockStdio,
          ),
      ...noColorTerminalOverride,
    },
  );

  testUsingContext(
    'cannot create a project in flutter root',
    () async {
      final String flutterBin = globals.fs.path.join(
        getFlutterRoot(),
        'bin',
        globals.platform.isWindows ? 'flutter.bat' : 'flutter',
      );
      final ProcessResult exec = await Process.run(flutterBin, <String>[
        'create',
        'flutter_project',
      ], workingDirectory: Cache.flutterRoot);
      expect(exec.exitCode, 2);
      expect(exec.stderr, contains('Cannot create a project within the Flutter SDK'));
    },
    overrides: <Type, Generator>{
      Pub:
          () => Pub.test(
            fileSystem: globals.fs,
            logger: globals.logger,
            processManager: globals.processManager,
            botDetector: globals.botDetector,
            platform: globals.platform,
            stdio: mockStdio,
          ),
      ...noColorTerminalOverride,
    },
  );

  testUsingContext(
    'Will create an app project if non-empty non-project directory exists without .metadata',
    () async {
      await projectDir.absolute.childDirectory('blag').create(recursive: true);
      await projectDir.absolute.childDirectory('.idea').create(recursive: true);
      await _createAndAnalyzeProject(
        projectDir,
        <String>['-i', 'objc', '-a', 'java'],
        <String>[
          'android/app/src/main/java/com/example/flutter_project/MainActivity.java',
          'android/app/src/main/java/io/flutter/plugins/GeneratedPluginRegistrant.java',
          'flutter_project.iml',
          'ios/Flutter/AppFrameworkInfo.plist',
          'ios/Runner/AppDelegate.m',
          'ios/Runner/GeneratedPluginRegistrant.h',
        ],
        unexpectedPaths: <String>['.android/', '.ios/'],
      );
    },
    overrides: <Type, Generator>{
      Pub:
          () => Pub.test(
            fileSystem: globals.fs,
            logger: globals.logger,
            processManager: globals.processManager,
            botDetector: globals.botDetector,
            platform: globals.platform,
            stdio: mockStdio,
          ),
    },
  );

  testUsingContext(
    'detects and recreates an app project correctly',
    () async {
      await projectDir.absolute.childDirectory('lib').create(recursive: true);
      await projectDir.absolute.childDirectory('ios').create(recursive: true);
      await _createAndAnalyzeProject(
        projectDir,
        <String>['-i', 'objc', '-a', 'java'],
        <String>[
          'android/app/src/main/java/com/example/flutter_project/MainActivity.java',
          'android/app/src/main/java/io/flutter/plugins/GeneratedPluginRegistrant.java',
          'flutter_project.iml',
          'ios/Flutter/AppFrameworkInfo.plist',
          'ios/Runner/AppDelegate.m',
          'ios/Runner/GeneratedPluginRegistrant.h',
        ],
        unexpectedPaths: <String>['.android/', '.ios/'],
      );
    },
    overrides: <Type, Generator>{
      Pub:
          () => Pub.test(
            fileSystem: globals.fs,
            logger: globals.logger,
            processManager: globals.processManager,
            botDetector: globals.botDetector,
            platform: globals.platform,
            stdio: mockStdio,
          ),
    },
  );

  testUsingContext(
    'detects and recreates a plugin project correctly',
    () async {
      await projectDir.create(recursive: true);
      await projectDir.absolute.childFile('.metadata').writeAsString('project_type: plugin\n');
      await _createAndAnalyzeProject(
        projectDir,
        <String>[],
        <String>['example/lib/main.dart', 'flutter_project.iml', 'lib/flutter_project.dart'],
        unexpectedPaths: <String>[
          'android/app/src/main/java/com/example/flutter_project/MainActivity.java',
          'android/src/main/java/com/example/flutter_project/FlutterProjectPlugin.java',
          'example/android/app/src/main/java/com/example/flutter_project_example/MainActivity.java',
        ],
      );
    },
    overrides: <Type, Generator>{
      Pub:
          () => Pub.test(
            fileSystem: globals.fs,
            logger: globals.logger,
            processManager: globals.processManager,
            botDetector: globals.botDetector,
            platform: globals.platform,
            stdio: mockStdio,
          ),
    },
  );

  testUsingContext(
    'detects and recreates a package project correctly',
    () async {
      await projectDir.create(recursive: true);
      await projectDir.absolute.childFile('.metadata').writeAsString('project_type: package\n');
      return _createAndAnalyzeProject(
        projectDir,
        <String>[],
        <String>['lib/flutter_project.dart', 'test/flutter_project_test.dart'],
        unexpectedPaths: <String>[
          'android/app/src/main/java/com/example/flutter_project/MainActivity.java',
          'android/src/main/java/com/example/flutter_project/FlutterProjectPlugin.java',
          'example/android/app/src/main/java/com/example/flutter_project_example/MainActivity.java',
          'example/ios/Runner/AppDelegate.h',
          'example/ios/Runner/AppDelegate.m',
          'example/ios/Runner/main.m',
          'example/lib/main.dart',
          'ios/Classes/FlutterProjectPlugin.h',
          'ios/Classes/FlutterProjectPlugin.m',
          'ios/Runner/AppDelegate.h',
          'ios/Runner/AppDelegate.m',
          'ios/Runner/main.m',
          'lib/main.dart',
          'test/widget_test.dart',
        ],
      );
    },
    overrides: <Type, Generator>{
      Pub:
          () => Pub.test(
            fileSystem: globals.fs,
            logger: globals.logger,
            processManager: globals.processManager,
            botDetector: globals.botDetector,
            platform: globals.platform,
            stdio: mockStdio,
          ),
    },
  );

  testUsingContext(
    'kotlin/swift legacy app project',
    () async {
      return _createProject(
        projectDir,
        <String>['--no-pub', '--template=app', '--android-language=kotlin', '--ios-language=swift'],
        <String>[
          'android/app/src/main/kotlin/com/example/flutter_project/MainActivity.kt',
          'ios/Runner/AppDelegate.swift',
          'ios/Runner/Runner-Bridging-Header.h',
          'lib/main.dart',
          '.idea/libraries/KotlinJavaRuntime.xml',
        ],
        unexpectedPaths: <String>[
          'android/app/src/main/java/com/example/flutter_project/MainActivity.java',
          'ios/Runner/AppDelegate.h',
          'ios/Runner/AppDelegate.m',
          'ios/Runner/main.m',
        ],
      );
    },
    overrides: <Type, Generator>{
      Pub:
          () => Pub.test(
            fileSystem: globals.fs,
            logger: globals.logger,
            processManager: globals.processManager,
            botDetector: globals.botDetector,
            platform: globals.platform,
            stdio: mockStdio,
          ),
    },
  );

  testUsingContext(
    'can create a package project',
    () async {
      await _createAndAnalyzeProject(
        projectDir,
        <String>['--template=package'],
        <String>[
          'analysis_options.yaml',
          'lib/flutter_project.dart',
          'test/flutter_project_test.dart',
        ],
        unexpectedPaths: <String>[
          'android/app/src/main/java/com/example/flutter_project/MainActivity.java',
          'android/src/main/java/com/example/flutter_project/FlutterProjectPlugin.java',
          'android/app/src/main/java/io/flutter/plugins/GeneratedPluginRegistrant.java',
          'example/android/app/src/main/java/com/example/flutter_project_example/MainActivity.java',
          'example/ios/Runner/AppDelegate.h',
          'example/ios/Runner/AppDelegate.m',
          'example/ios/Runner/main.m',
          'example/lib/main.dart',
          'ios/Classes/FlutterProjectPlugin.h',
          'ios/Classes/FlutterProjectPlugin.m',
          'ios/Runner/AppDelegate.h',
          'ios/Runner/AppDelegate.m',
          'ios/Runner/GeneratedPluginRegistrant.h',
          'ios/Runner/GeneratedPluginRegistrant.m',
          'ios/Runner/main.m',
          'lib/main.dart',
          'test/widget_test.dart',
          'windows/flutter/generated_plugin_registrant.cc',
          'windows/flutter/generated_plugin_registrant.h',
          'windows/flutter/generated_plugins.cmake',
          'linux/flutter/generated_plugin_registrant.cc',
          'linux/flutter/generated_plugin_registrant.h',
          'linux/flutter/generated_plugins.cmake',
          'macos/Flutter/GeneratedPluginRegistrant.swift',
        ],
        expectedGitignoreLines: flutterPluginsIgnores,
      );
      return _runFlutterTest(projectDir);
    },
    overrides: <Type, Generator>{
      Pub:
          () => Pub.test(
            fileSystem: globals.fs,
            logger: globals.logger,
            processManager: globals.processManager,
            botDetector: globals.botDetector,
            platform: globals.platform,
            stdio: mockStdio,
          ),
    },
  );

  testUsingContext(
    'can create a plugin project',
    () async {
      await _createAndAnalyzeProject(
        projectDir,
        <String>['--template=plugin', '-i', 'objc', '-a', 'java'],
        <String>[
          'analysis_options.yaml',
          'LICENSE',
          'README.md',
          'example/lib/main.dart',
          'flutter_project.iml',
          'lib/flutter_project.dart',
        ],
        unexpectedPaths: <String>[
          'android/src/main/java/com/example/flutter_project/FlutterProjectPlugin.java',
          'example/android/app/src/main/java/com/example/flutter_project_example/MainActivity.java',
          'lib/flutter_project_web.dart',
        ],
        expectedGitignoreLines: flutterPluginsIgnores,
      );
      return _runFlutterTest(projectDir.childDirectory('example'));
    },
    overrides: <Type, Generator>{
      Pub:
          () => Pub.test(
            fileSystem: globals.fs,
            logger: globals.logger,
            processManager: globals.processManager,
            botDetector: globals.botDetector,
            platform: globals.platform,
            stdio: mockStdio,
          ),
    },
  );

  testUsingContext(
    'plugin project supports web',
    () async {
      await _createAndAnalyzeProject(
        projectDir,
        <String>['--template=plugin', '--platform=web'],
        <String>['lib/flutter_project.dart', 'lib/flutter_project_web.dart'],
      );
      final String rawPubspec = await projectDir.childFile('pubspec.yaml').readAsString();
      final Pubspec pubspec = Pubspec.parse(rawPubspec);
      // Expect the dependency on flutter_web_plugins exists
      expect(pubspec.dependencies, contains('flutter_web_plugins'));
      // The platform is correctly registered
      final YamlMap web =
          ((pubspec.flutter!['plugin'] as YamlMap)['platforms'] as YamlMap)['web'] as YamlMap;
      expect(web['pluginClass'], 'FlutterProjectWeb');
      expect(web['fileName'], 'flutter_project_web.dart');
      expect(logger.errorText, isNot(contains(_kNoPlatformsMessage)));
    },
    overrides: <Type, Generator>{
      FeatureFlags: () => TestFeatureFlags(isWebEnabled: true),
      Pub:
          () => Pub.test(
            fileSystem: globals.fs,
            logger: globals.logger,
            processManager: globals.processManager,
            botDetector: globals.botDetector,
            platform: globals.platform,
            stdio: mockStdio,
          ),
      Logger: () => logger,
    },
  );

  testUsingContext(
    'plugin example app depends on plugin',
    () async {
      await _createProject(
        projectDir,
        <String>['--template=plugin', '-i', 'objc', '-a', 'java'],
        <String>['example/pubspec.yaml'],
      );
      final String rawPubspec =
          await projectDir.childDirectory('example').childFile('pubspec.yaml').readAsString();
      final Pubspec pubspec = Pubspec.parse(rawPubspec);
      final String pluginName = projectDir.basename;
      expect(pubspec.dependencies, contains(pluginName));
      expect(pubspec.dependencies[pluginName] is PathDependency, isTrue);
      final PathDependency pathDependency = pubspec.dependencies[pluginName]! as PathDependency;
      expect(pathDependency.path, '../');
    },
    overrides: <Type, Generator>{
      Pub:
          () => Pub.test(
            fileSystem: globals.fs,
            logger: globals.logger,
            processManager: globals.processManager,
            botDetector: globals.botDetector,
            platform: globals.platform,
            stdio: mockStdio,
          ),
    },
  );

  testUsingContext(
    'plugin example app includes an integration test',
    () async {
      await _createAndAnalyzeProject(
        projectDir,
        <String>['--template=plugin'],
        <String>['example/integration_test/plugin_integration_test.dart'],
      );
      return _runFlutterTest(projectDir.childDirectory('example'));
    },
    overrides: <Type, Generator>{
      Pub:
          () => Pub.test(
            fileSystem: globals.fs,
            logger: globals.logger,
            processManager: globals.processManager,
            botDetector: globals.botDetector,
            platform: globals.platform,
            stdio: mockStdio,
          ),
    },
  );

  testUsingContext(
    'kotlin/swift plugin project without Swift Package Manager',
    () async {
      return _createProject(
        projectDir,
        <String>[
          '--no-pub',
          '--template=plugin',
          '-a',
          'kotlin',
          '--ios-language',
          'swift',
          '--platforms',
          'android,ios,macos',
        ],
        <String>[
          'analysis_options.yaml',
          'android/src/main/kotlin/com/example/flutter_project/FlutterProjectPlugin.kt',
          'example/android/app/src/main/kotlin/com/example/flutter_project_example/MainActivity.kt',
          'example/ios/Runner/AppDelegate.swift',
          'example/ios/Runner/Runner-Bridging-Header.h',
          'example/lib/main.dart',
          'ios/Classes/FlutterProjectPlugin.swift',
          'ios/Resources/PrivacyInfo.xcprivacy',
          'macos/Classes/FlutterProjectPlugin.swift',
          'macos/Resources/PrivacyInfo.xcprivacy',
          'lib/flutter_project.dart',
        ],
        unexpectedPaths: <String>[
          'android/src/main/java/com/example/flutter_project/FlutterProjectPlugin.java',
          'example/android/app/src/main/java/com/example/flutter_project_example/MainActivity.java',
          'example/ios/Runner/AppDelegate.h',
          'example/ios/Runner/AppDelegate.m',
          'example/ios/Runner/main.m',
          'ios/Classes/FlutterProjectPlugin.h',
          'ios/Classes/FlutterProjectPlugin.m',
        ],
      );
    },
    overrides: <Type, Generator>{
      // Test flags disable Swift Package Manager.
      FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true),
    },
  );

  testUsingContext(
    'swift plugin project with Swift Package Manager',
    () async {
      return _createProject(
        projectDir,
        <String>[
          '--no-pub',
          '--template=plugin',
          '--ios-language',
          'swift',
          '--platforms',
          'ios,macos',
        ],
        <String>[
          'ios/flutter_project/Package.swift',
          'ios/flutter_project/Sources/flutter_project/FlutterProjectPlugin.swift',
          'ios/flutter_project/Sources/flutter_project/PrivacyInfo.xcprivacy',
          'macos/flutter_project/Package.swift',
          'macos/flutter_project/Sources/flutter_project/FlutterProjectPlugin.swift',
          'macos/flutter_project/Sources/flutter_project/PrivacyInfo.xcprivacy',
        ],
        unexpectedPaths: <String>[
          'ios/Classes/FlutterProjectPlugin.swift',
          'macos/Classes/FlutterProjectPlugin.swift',
          'ios/Classes/FlutterProjectPlugin.h',
          'ios/Classes/FlutterProjectPlugin.m',
          'ios/Assets/.gitkeep',
          'macos/Assets/.gitkeep',
        ],
      );
    },
    overrides: <Type, Generator>{
      FeatureFlags:
          () => TestFeatureFlags(isSwiftPackageManagerEnabled: true, isMacOSEnabled: true),
    },
  );

  testUsingContext(
    'objc plugin project with Swift Package Manager',
    () async {
      return _createProject(
        projectDir,
        <String>['--no-pub', '--template=plugin', '--ios-language', 'objc', '--platforms', 'ios'],
        <String>[
          'ios/flutter_project/Package.swift',
          'ios/flutter_project/Sources/flutter_project/include/flutter_project/FlutterProjectPlugin.h',
          'ios/flutter_project/Sources/flutter_project/FlutterProjectPlugin.m',
          'ios/flutter_project/Sources/flutter_project/PrivacyInfo.xcprivacy',
        ],
        unexpectedPaths: <String>[
          'ios/Classes/FlutterProjectPlugin.swift',
          'ios/Classes/FlutterProjectPlugin.h',
          'ios/Classes/FlutterProjectPlugin.m',
          'ios/Assets/.gitkeep',
        ],
      );
    },
    overrides: <Type, Generator>{
      FeatureFlags: () => TestFeatureFlags(isSwiftPackageManagerEnabled: true),
    },
  );

  testUsingContext('plugin project with custom org', () async {
    return _createProject(
      projectDir,
      <String>[
        '--no-pub',
        '--template=plugin',
        '--org',
        'com.bar.foo',
        '-i',
        'objc',
        '-a',
        'java',
        '--platform',
        'android',
      ],
      <String>[
        'android/src/main/java/com/bar/foo/flutter_project/FlutterProjectPlugin.java',
        'example/android/app/src/main/java/com/bar/foo/flutter_project_example/MainActivity.java',
      ],
      unexpectedPaths: <String>[
        'android/src/main/java/com/example/flutter_project/FlutterProjectPlugin.java',
        'example/android/app/src/main/java/com/example/flutter_project_example/MainActivity.java',
      ],
    );
  });

  testUsingContext('plugin project with valid custom project name', () async {
    return _createProject(
      projectDir,
      <String>[
        '--no-pub',
        '--template=plugin',
        '--project-name',
        'xyz',
        '-i',
        'objc',
        '-a',
        'java',
        '--platforms',
        'android,ios',
      ],
      <String>[
        'android/src/main/java/com/example/xyz/XyzPlugin.java',
        'example/android/app/src/main/java/com/example/xyz_example/MainActivity.java',
      ],
      unexpectedPaths: <String>[
        'android/src/main/java/com/example/flutter_project/FlutterProjectPlugin.java',
        'example/android/app/src/main/java/com/example/flutter_project_example/MainActivity.java',
      ],
    );
  });

  testUsingContext('plugin project with invalid custom project name', () async {
    expect(
      () => _createProject(projectDir, <String>[
        '--no-pub',
        '--template=plugin',
        '--project-name',
        'xyz-xyz',
        '--platforms',
        'android,ios',
      ], <String>[]),
      throwsToolExit(
        message:
            '"xyz-xyz" is not a valid Dart package name. Try "xyz_xyz" instead.\n'
            '\n'
            'The name should consist of lowercase words separated by underscores, '
            '"like_this". Use only basic Latin letters and Arabic digits: [a-z0-9_], '
            'and ensure the name is a valid Dart identifier (i.e. it does not start '
            'with a digit and is not a reserved word).\n'
            '\n'
            'See https://dart.dev/tools/pub/pubspec#name for more information.',
      ),
    );
  });

  testUsingContext('recreating project uses pubspec name as project name fallback', () async {
    final Directory outputDirectory = tempDir.childDirectory('invalid-name');

    // Create the new project with a valid project name,
    // but with a directory name that would be an invalid project name.
    await _createProject(outputDirectory, <String>[
      '--no-pub',
      '--template=app',
      '--project-name',
      'valid_name',
      '--platforms',
      'android',
    ], <String>[]);

    // Now amend a new platform to the project, but omit the project name, so the fallback project name is used.
    await _createProject(outputDirectory, <String>[
      '--no-pub',
      '--template=app',
      '--platforms',
      'web',
    ], <String>[]);

    // Verify that the pubspec name was used as project name for the web project.
    final File webOutputFile = outputDirectory.childDirectory('web').childFile('index.html');

    expect(webOutputFile.readAsStringSync(), contains('<title>valid_name</title>'));
  });

  testUsingContext(
    'module project with pub',
    () async {
      return _createProject(
        projectDir,
        <String>['--template=module'],
        <String>[
          '.android/build.gradle',
          '.android/Flutter/build.gradle',
          '.android/Flutter/flutter.iml',
          '.android/Flutter/src/main/AndroidManifest.xml',
          '.android/Flutter/src/main/java/io/flutter/plugins/GeneratedPluginRegistrant.java',
          '.android/gradle.properties',
          '.android/gradle/wrapper/gradle-wrapper.jar',
          '.android/gradle/wrapper/gradle-wrapper.properties',
          '.android/gradlew',
          '.android/gradlew.bat',
          '.android/include_flutter.groovy',
          '.android/local.properties',
          '.android/settings.gradle',
          '.gitignore',
          '.metadata',
          '.dart_tool/package_config.json',
          'analysis_options.yaml',
          'lib/main.dart',
          'pubspec.lock',
          'pubspec.yaml',
          'README.md',
          'test/widget_test.dart',
        ],
        unexpectedPaths: <String>[
          'android/',
          'ios/',
          '.android/Flutter/src/main/java/io/flutter/facade/FlutterFragment.java',
          '.android/Flutter/src/main/java/io/flutter/facade/Flutter.java',
        ],
      );
    },
    overrides: <Type, Generator>{
      Pub:
          () => Pub.test(
            fileSystem: globals.fs,
            logger: globals.logger,
            processManager: globals.processManager,
            botDetector: globals.botDetector,
            platform: globals.platform,
            stdio: mockStdio,
          ),
    },
  );

  testUsingContext('androidx is used by default in an app project', () async {
    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);

    await runner.run(<String>['create', '--no-pub', projectDir.path]);

    void expectExists(String relPath) {
      expect(globals.fs.isFileSync('${projectDir.path}/$relPath'), true);
    }

    expectExists('android/gradle.properties');

    final String actualContents =
        await globals.fs.file('${projectDir.path}/android/gradle.properties').readAsString();

    expect(actualContents.contains('useAndroidX'), true);
  });

  testUsingContext('androidx is used by default in a module project', () async {
    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);

    await runner.run(<String>['create', '--template=module', '--no-pub', projectDir.path]);

    final FlutterProject project = FlutterProject.fromDirectory(projectDir);
    expect(project.usesAndroidX, true);
  });

  testUsingContext(
    'creating a new project should create v2 embedding and never show an Android v1 deprecation warning',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>['create', '--no-pub', '--platform', 'android', projectDir.path]);

      final String androidManifest =
          await globals.fs
              .file('${projectDir.path}/android/app/src/main/AndroidManifest.xml')
              .readAsString();
      expect(androidManifest.contains('android:name="flutterEmbedding"'), true);
      expect(androidManifest.contains('android:value="2"'), true);

      final String mainActivity =
          await globals.fs
              .file(
                '${projectDir.path}/android/app/src/main/kotlin/com/example/flutter_project/MainActivity.kt',
              )
              .readAsString();
      // Import for the new embedding class.
      expect(mainActivity.contains('import io.flutter.embedding.android.FlutterActivity'), true);

      expect(
        logger.statusText,
        isNot(
          contains(
            'https://github.com/flutter/flutter/blob/main/docs/platforms/android/Upgrading-pre-1.12-Android-projects.md',
          ),
        ),
      );
    },
    overrides: <Type, Generator>{Logger: () => logger},
  );

  testUsingContext('app supports android and ios by default', () async {
    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);

    await runner.run(<String>['create', '--no-pub', projectDir.path]);

    expect(projectDir.childDirectory('android'), exists);
    expect(projectDir.childDirectory('ios'), exists);
  }, overrides: <Type, Generator>{});

  testUsingContext(
    'app does not include android if disabled in config',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>['create', '--no-pub', projectDir.path]);

      expect(projectDir.childDirectory('android'), isNot(exists));
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(isAndroidEnabled: false)},
  );

  testUsingContext(
    'app does not include ios if disabled in config',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>['create', '--no-pub', projectDir.path]);

      expect(projectDir.childDirectory('ios'), isNot(exists));
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(isIOSEnabled: false)},
  );

  testUsingContext(
    'app does not include desktop or web by default',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>['create', '--no-pub', projectDir.path]);

      expect(projectDir.childDirectory('linux'), isNot(exists));
      expect(projectDir.childDirectory('macos'), isNot(exists));
      expect(projectDir.childDirectory('windows'), isNot(exists));
      expect(projectDir.childDirectory('web'), isNot(exists));
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags()},
  );

  testUsingContext(
    'plugin does not include desktop or web by default',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>['create', '--no-pub', '--template=plugin', projectDir.path]);

      expect(projectDir.childDirectory('linux'), isNot(exists));
      expect(projectDir.childDirectory('macos'), isNot(exists));
      expect(projectDir.childDirectory('windows'), isNot(exists));
      expect(projectDir.childDirectory('web'), isNot(exists));
      expect(projectDir.childDirectory('example').childDirectory('linux'), isNot(exists));
      expect(projectDir.childDirectory('example').childDirectory('macos'), isNot(exists));
      expect(projectDir.childDirectory('example').childDirectory('windows'), isNot(exists));
      expect(projectDir.childDirectory('example').childDirectory('web'), isNot(exists));
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags()},
  );

  testUsingContext(
    'app supports Linux if requested',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>['create', '--no-pub', '--platform=linux', projectDir.path]);

      expect(projectDir.childDirectory('linux').childFile('CMakeLists.txt'), exists);
      expect(projectDir.childDirectory('android'), isNot(exists));
      expect(projectDir.childDirectory('ios'), isNot(exists));
      expect(projectDir.childDirectory('windows'), isNot(exists));
      expect(projectDir.childDirectory('macos'), isNot(exists));
      expect(projectDir.childDirectory('web'), isNot(exists));
      expect(logger.errorText, isNot(contains(_kNoPlatformsMessage)));
    },
    overrides: <Type, Generator>{
      FeatureFlags: () => TestFeatureFlags(isLinuxEnabled: true),
      Logger: () => logger,
    },
  );

  testUsingContext(
    'plugin supports Linux if requested',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=linux',
        projectDir.path,
      ]);

      expect(projectDir.childDirectory('linux').childFile('CMakeLists.txt'), exists);
      expect(projectDir.childDirectory('example').childDirectory('linux'), exists);
      expect(projectDir.childDirectory('example').childDirectory('android'), isNot(exists));
      expect(projectDir.childDirectory('example').childDirectory('ios'), isNot(exists));
      expect(projectDir.childDirectory('example').childDirectory('windows'), isNot(exists));
      expect(projectDir.childDirectory('example').childDirectory('macos'), isNot(exists));
      expect(projectDir.childDirectory('example').childDirectory('web'), isNot(exists));
      validatePubspecForPlugin(
        projectDir: projectDir.absolute.path,
        expectedPlatforms: const <String>['linux'],
        pluginClass: 'FlutterProjectPlugin',
        unexpectedPlatforms: <String>['some_platform'],
      );
      expect(logger.errorText, isNot(contains(_kNoPlatformsMessage)));
    },
    overrides: <Type, Generator>{
      FeatureFlags: () => TestFeatureFlags(isLinuxEnabled: true),
      Logger: () => logger,
    },
  );

  testUsingContext(
    'app supports macOS if requested',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>['create', '--no-pub', '--platform=macos', projectDir.path]);

      expect(projectDir.childDirectory('macos').childDirectory('Runner.xcworkspace'), exists);
      expect(projectDir.childDirectory('android'), isNot(exists));
      expect(projectDir.childDirectory('ios'), isNot(exists));
      expect(projectDir.childDirectory('linux'), isNot(exists));
      expect(projectDir.childDirectory('windows'), isNot(exists));
      expect(projectDir.childDirectory('web'), isNot(exists));
      expect(logger.errorText, isNot(contains(_kNoPlatformsMessage)));
    },
    overrides: <Type, Generator>{
      FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true),
      Logger: () => logger,
    },
  );

  testUsingContext(
    'plugin supports macOS if requested',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=macos',
        projectDir.path,
      ]);

      expect(projectDir.childDirectory('macos').childFile('flutter_project.podspec'), exists);
      expect(projectDir.childDirectory('example').childDirectory('macos'), exists);
      expect(projectDir.childDirectory('example').childDirectory('linux'), isNot(exists));
      expect(projectDir.childDirectory('example').childDirectory('android'), isNot(exists));
      expect(projectDir.childDirectory('example').childDirectory('ios'), isNot(exists));
      expect(projectDir.childDirectory('example').childDirectory('windows'), isNot(exists));
      expect(projectDir.childDirectory('example').childDirectory('web'), isNot(exists));
      validatePubspecForPlugin(
        projectDir: projectDir.absolute.path,
        expectedPlatforms: const <String>['macos'],
        pluginClass: 'FlutterProjectPlugin',
        unexpectedPlatforms: <String>['some_platform'],
      );
      expect(logger.errorText, isNot(contains(_kNoPlatformsMessage)));
    },
    overrides: <Type, Generator>{
      FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true),
      Logger: () => logger,
    },
  );

  testUsingContext(
    'app supports Windows if requested',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>['create', '--no-pub', '--platform=windows', projectDir.path]);

      expect(projectDir.childDirectory('windows').childFile('CMakeLists.txt'), exists);
      expect(projectDir.childDirectory('android'), isNot(exists));
      expect(projectDir.childDirectory('ios'), isNot(exists));
      expect(projectDir.childDirectory('linux'), isNot(exists));
      expect(projectDir.childDirectory('macos'), isNot(exists));
      expect(projectDir.childDirectory('web'), isNot(exists));
      expect(logger.errorText, isNot(contains(_kNoPlatformsMessage)));
    },
    overrides: <Type, Generator>{
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
      Logger: () => logger,
    },
  );

  testUsingContext(
    'Windows has correct VERSIONINFO',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>['create', '--no-pub', '--org', 'com.foo.bar', projectDir.path]);

      final File resourceFile = projectDir
          .childDirectory('windows')
          .childDirectory('runner')
          .childFile('Runner.rc');
      expect(resourceFile, exists);
      final String contents = resourceFile.readAsStringSync();
      expect(contents, contains('"CompanyName", "com.foo.bar"'));
      expect(contents, contains('"FileDescription", "flutter_project"'));
      expect(contents, contains('"ProductName", "flutter_project"'));
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true)},
  );

  testUsingContext(
    'plugin supports Windows if requested',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=windows',
        projectDir.path,
      ]);

      expect(projectDir.childDirectory('windows').childFile('CMakeLists.txt'), exists);
      expect(projectDir.childDirectory('example').childDirectory('windows'), exists);
      expect(projectDir.childDirectory('example').childDirectory('android'), isNot(exists));
      expect(projectDir.childDirectory('example').childDirectory('ios'), isNot(exists));
      expect(projectDir.childDirectory('example').childDirectory('linux'), isNot(exists));
      expect(projectDir.childDirectory('example').childDirectory('macos'), isNot(exists));
      expect(projectDir.childDirectory('example').childDirectory('web'), isNot(exists));
      validatePubspecForPlugin(
        projectDir: projectDir.absolute.path,
        expectedPlatforms: const <String>['windows'],
        pluginClass: 'FlutterProjectPluginCApi',
        unexpectedPlatforms: <String>['some_platform'],
      );
      expect(logger.errorText, isNot(contains(_kNoPlatformsMessage)));
    },
    overrides: <Type, Generator>{
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
      Logger: () => logger,
    },
  );

  testUsingContext(
    'app supports web if requested',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>['create', '--no-pub', '--platform=web', projectDir.path]);

      expect(projectDir.childDirectory('web').childFile('index.html'), exists);
      expect(projectDir.childDirectory('android'), isNot(exists));
      expect(projectDir.childDirectory('ios'), isNot(exists));
      expect(projectDir.childDirectory('linux'), isNot(exists));
      expect(projectDir.childDirectory('macos'), isNot(exists));
      expect(projectDir.childDirectory('windows'), isNot(exists));
      expect(logger.errorText, isNot(contains(_kNoPlatformsMessage)));
    },
    overrides: <Type, Generator>{
      FeatureFlags: () => TestFeatureFlags(isWebEnabled: true),
      Logger: () => logger,
    },
  );

  testUsingContext('app creates maskable icons for web', () async {
    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);

    await runner.run(<String>['create', '--no-pub', '--platform=web', projectDir.path]);

    final Directory iconsDir = projectDir.childDirectory('web').childDirectory('icons');

    expect(iconsDir.childFile('Icon-maskable-192.png'), exists);
    expect(iconsDir.childFile('Icon-maskable-512.png'), exists);
  });

  testUsingContext('plugin uses new platform schema', () async {
    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);

    await runner.run(<String>['create', '--no-pub', '--template=plugin', projectDir.path]);

    final String pubspecContents =
        await globals.fs.directory(projectDir.path).childFile('pubspec.yaml').readAsString();

    expect(pubspecContents.contains('platforms:'), true);
  });

  testUsingContext(
    'has correct content and formatting with module template',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>[
        'create',
        '--template=module',
        '--no-pub',
        '--org',
        'com.foo.bar',
        projectDir.path,
      ]);

      void expectExists(String relPath, [bool expectation = true]) {
        expect(globals.fs.isFileSync('${projectDir.path}/$relPath'), expectation);
      }

      expectExists('lib/main.dart');
      expectExists('test/widget_test.dart');

      final String actualContents =
          await globals.fs.file('${projectDir.path}/test/widget_test.dart').readAsString();

      expect(actualContents.contains('flutter_test.dart'), true);

      for (final FileSystemEntity file in projectDir.listSync(recursive: true)) {
        if (file is File && file.path.endsWith('.dart')) {
          final String original = file.readAsStringSync();

          final Process process = await Process.start(
            globals.artifacts!.getArtifactPath(Artifact.engineDartBinary),
            <String>['format', '--output=show', file.path],
            workingDirectory: projectDir.path,
          );
          final String formatted = await process.stdout.transform(utf8.decoder).join();

          expect(formatted, contains(original), reason: file.path);
        }
      }

      await _runFlutterTest(
        projectDir,
        target: globals.fs.path.join(projectDir.path, 'test', 'widget_test.dart'),
      );

      // Generated Xcode settings
      final String xcodeConfigPath = globals.fs.path.join('.ios', 'Flutter', 'Generated.xcconfig');
      expectExists(xcodeConfigPath);
      final File xcodeConfigFile = globals.fs.file(
        globals.fs.path.join(projectDir.path, xcodeConfigPath),
      );
      final String xcodeConfig = xcodeConfigFile.readAsStringSync();
      expect(xcodeConfig, contains('FLUTTER_ROOT='));
      expect(xcodeConfig, contains('FLUTTER_APPLICATION_PATH='));
      expect(xcodeConfig, contains('FLUTTER_TARGET='));
      expect(xcodeConfig, contains('COCOAPODS_PARALLEL_CODE_SIGN=true'));
      expect(xcodeConfig, contains('EXCLUDED_ARCHS[sdk=iphoneos*]=armv7'));
      // Avoid legacy build locations to support Swift Package Manager.
      expect(xcodeConfig, isNot(contains('SYMROOT')));

      // Generated export environment variables script
      final String buildPhaseScriptPath = globals.fs.path.join(
        '.ios',
        'Flutter',
        'flutter_export_environment.sh',
      );
      expectExists(buildPhaseScriptPath);
      final File buildPhaseScriptFile = globals.fs.file(
        globals.fs.path.join(projectDir.path, buildPhaseScriptPath),
      );
      final String buildPhaseScript = buildPhaseScriptFile.readAsStringSync();
      expect(buildPhaseScript, contains('FLUTTER_ROOT='));
      expect(buildPhaseScript, contains('FLUTTER_APPLICATION_PATH='));
      expect(buildPhaseScript, contains('FLUTTER_TARGET='));
      expect(buildPhaseScript, contains('COCOAPODS_PARALLEL_CODE_SIGN=true'));
      // Do not override host app build settings.
      expect(buildPhaseScript, isNot(contains('SYMROOT')));

      // App identification
      final String xcodeProjectPath = globals.fs.path.join(
        '.ios',
        'Runner.xcodeproj',
        'project.pbxproj',
      );
      expectExists(xcodeProjectPath);
      final File xcodeProjectFile = globals.fs.file(
        globals.fs.path.join(projectDir.path, xcodeProjectPath),
      );
      final String xcodeProject = xcodeProjectFile.readAsStringSync();
      expect(xcodeProject, contains('PRODUCT_BUNDLE_IDENTIFIER = com.foo.bar.flutterProject'));
      expect(xcodeProject, contains('LastUpgradeCheck = 1510;'));
      // Xcode workspace shared data
      final Directory workspaceSharedData = globals.fs.directory(
        globals.fs.path.join('.ios', 'Runner.xcworkspace', 'xcshareddata'),
      );
      expectExists(workspaceSharedData.childFile('WorkspaceSettings.xcsettings').path);
      expectExists(workspaceSharedData.childFile('IDEWorkspaceChecks.plist').path);
      // Xcode project shared data
      final Directory projectSharedData = globals.fs.directory(
        globals.fs.path.join('.ios', 'Runner.xcodeproj', 'project.xcworkspace', 'xcshareddata'),
      );
      expectExists(projectSharedData.childFile('WorkspaceSettings.xcsettings').path);
      expectExists(projectSharedData.childFile('IDEWorkspaceChecks.plist').path);

      final String versionPath = globals.fs.path.join('.metadata');
      expectExists(versionPath);
      final String version =
          globals.fs.file(globals.fs.path.join(projectDir.path, versionPath)).readAsStringSync();
      expect(version, contains('version:'));
      expect(version, contains('revision: "12345678"'));
      expect(version, contains('channel: "omega"'));

      // IntelliJ metadata
      final String intelliJSdkMetadataPath = globals.fs.path.join(
        '.idea',
        'libraries',
        'Dart_SDK.xml',
      );
      expectExists(intelliJSdkMetadataPath);
      final String sdkMetaContents =
          globals.fs
              .file(globals.fs.path.join(projectDir.path, intelliJSdkMetadataPath))
              .readAsStringSync();
      expect(sdkMetaContents, contains('<root url="file:/'));
      expect(sdkMetaContents, contains('/bin/cache/dart-sdk/lib/core"'));
    },
    overrides: <Type, Generator>{
      FlutterVersion: () => fakeFlutterVersion,
      Platform: _kNoColorTerminalPlatform,
    },
  );

  testUsingContext(
    'has correct default content and formatting with app template',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>[
        'create',
        '--template=app',
        '--no-pub',
        '--org',
        'com.foo.bar',
        projectDir.path,
      ]);

      void expectExists(String relPath) {
        expect(globals.fs.file('${projectDir.path}/$relPath'), exists);
      }

      expectExists('lib/main.dart');
      expectExists('test/widget_test.dart');

      for (final FileSystemEntity file in projectDir.listSync(recursive: true)) {
        if (file is File && file.path.endsWith('.dart')) {
          final String original = file.readAsStringSync();

          final Process process = await Process.start(
            globals.artifacts!.getArtifactPath(Artifact.engineDartBinary),
            <String>['format', '--output=show', file.path],
            workingDirectory: projectDir.path,
          );
          final String formatted = await process.stdout.transform(utf8.decoder).join();

          expect(formatted, contains(original), reason: file.path);
        }
      }

      await _runFlutterTest(
        projectDir,
        target: globals.fs.path.join(projectDir.path, 'test', 'widget_test.dart'),
      );

      // Generated Xcode settings
      final String xcodeConfigPath = globals.fs.path.join('ios', 'Flutter', 'Generated.xcconfig');
      expectExists(xcodeConfigPath);
      final File xcodeConfigFile = globals.fs.file(
        globals.fs.path.join(projectDir.path, xcodeConfigPath),
      );
      final String xcodeConfig = xcodeConfigFile.readAsStringSync();
      expect(xcodeConfig, contains('FLUTTER_ROOT='));
      expect(xcodeConfig, contains('FLUTTER_APPLICATION_PATH='));
      expect(xcodeConfig, contains('COCOAPODS_PARALLEL_CODE_SIGN=true'));
      expect(xcodeConfig, contains('EXCLUDED_ARCHS[sdk=iphoneos*]=armv7'));
      // Xcode project
      final String xcodeProjectPath = globals.fs.path.join(
        'ios',
        'Runner.xcodeproj',
        'project.pbxproj',
      );
      expectExists(xcodeProjectPath);
      final File xcodeProjectFile = globals.fs.file(
        globals.fs.path.join(projectDir.path, xcodeProjectPath),
      );
      final String xcodeProject = xcodeProjectFile.readAsStringSync();
      expect(xcodeProject, contains('PRODUCT_BUNDLE_IDENTIFIER = com.foo.bar.flutterProject'));
      expect(xcodeProject, contains('LastUpgradeCheck = 1510;'));
      // Xcode workspace shared data
      final Directory workspaceSharedData = globals.fs.directory(
        globals.fs.path.join('ios', 'Runner.xcworkspace', 'xcshareddata'),
      );
      expectExists(workspaceSharedData.childFile('WorkspaceSettings.xcsettings').path);
      expectExists(workspaceSharedData.childFile('IDEWorkspaceChecks.plist').path);
      // Xcode project shared data
      final Directory projectSharedData = globals.fs.directory(
        globals.fs.path.join('ios', 'Runner.xcodeproj', 'project.xcworkspace', 'xcshareddata'),
      );
      expectExists(projectSharedData.childFile('WorkspaceSettings.xcsettings').path);
      expectExists(projectSharedData.childFile('IDEWorkspaceChecks.plist').path);

      final String versionPath = globals.fs.path.join('.metadata');
      expectExists(versionPath);
      final String version =
          globals.fs.file(globals.fs.path.join(projectDir.path, versionPath)).readAsStringSync();
      expect(version, contains('version:'));
      expect(version, contains('revision: "12345678"'));
      expect(version, contains('channel: "omega"'));

      // IntelliJ metadata
      final String intelliJSdkMetadataPath = globals.fs.path.join(
        '.idea',
        'libraries',
        'Dart_SDK.xml',
      );
      expectExists(intelliJSdkMetadataPath);
      final String sdkMetaContents =
          globals.fs
              .file(globals.fs.path.join(projectDir.path, intelliJSdkMetadataPath))
              .readAsStringSync();
      expect(sdkMetaContents, contains('<root url="file:/'));
      expect(sdkMetaContents, contains('/bin/cache/dart-sdk/lib/core"'));
    },
    overrides: <Type, Generator>{
      FlutterVersion: () => fakeFlutterVersion,
      Platform: _kNoColorTerminalPlatform,
    },
  );

  testUsingContext(
    'has iOS development team with app template',
    () async {
      final Completer<void> completer = Completer<void>();
      final StreamController<List<int>> controller = StreamController<List<int>>();
      const String certificates = '''
1) 86f7e437faa5a7fce15d1ddcb9eaeaea377667b8 "iPhone Developer: Profile 1 (1111AAAA11)"
    1 valid identities found''';
      fakeProcessManager.addCommands(<FakeCommand>[
        const FakeCommand(command: <String>['which', 'security']),
        const FakeCommand(command: <String>['which', 'openssl']),
        const FakeCommand(
          command: <String>['security', 'find-identity', '-p', 'codesigning', '-v'],
          stdout: certificates,
        ),
        const FakeCommand(
          command: <String>['security', 'find-certificate', '-c', '1111AAAA11', '-p'],
          stdout: 'This is a fake certificate',
        ),
        FakeCommand(
          command: const <String>['openssl', 'x509', '-subject'],
          stdin: IOSink(controller.sink),
          stdout:
              'subject= /CN=iPhone Developer: Profile 1 (1111AAAA11)/OU=3333CCCC33/O=My Team/C=US',
        ),
      ]);

      controller.stream.listen((List<int> chunk) {
        completer.complete();
      });

      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>[
        'create',
        '--template=app',
        '--no-pub',
        '--org',
        'com.foo.bar',
        projectDir.path,
      ]);

      final String xcodeProjectPath = globals.fs.path.join(
        'ios',
        'Runner.xcodeproj',
        'project.pbxproj',
      );
      final File xcodeProjectFile = globals.fs.file(
        globals.fs.path.join(projectDir.path, xcodeProjectPath),
      );
      expect(xcodeProjectFile, exists);
      final String xcodeProject = xcodeProjectFile.readAsStringSync();
      expect(xcodeProject, contains('DEVELOPMENT_TEAM = 3333CCCC33;'));
    },
    overrides: <Type, Generator>{
      FlutterVersion: () => fakeFlutterVersion,
      Java: () => null,
      Platform: _kNoColorTerminalMacOSPlatform,
      ProcessManager: () => fakeProcessManager,
    },
  );

  testUsingContext('Correct info.plist key-value pairs for objc iOS project.', () async {
    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);

    await runner.run(<String>[
      'create',
      '--template=app',
      '--no-pub',
      '--org',
      'com.foo.bar',
      '--ios-language=objc',
      '--project-name=my_project',
      projectDir.path,
    ]);

    final String plistPath = globals.fs.path.join('ios', 'Runner', 'Info.plist');
    final File plistFile = globals.fs.file(globals.fs.path.join(projectDir.path, plistPath));
    expect(plistFile, exists);
    final bool disabled = _getBooleanValueFromPlist(
      plistFile: plistFile,
      key: 'CADisableMinimumFrameDurationOnPhone',
    );
    expect(disabled, isTrue);
    final bool indirectInput = _getBooleanValueFromPlist(
      plistFile: plistFile,
      key: 'UIApplicationSupportsIndirectInputEvents',
    );
    expect(indirectInput, isTrue);
    final String displayName = _getStringValueFromPlist(
      plistFile: plistFile,
      key: 'CFBundleDisplayName',
    );
    expect(displayName, 'My Project');
  });

  testUsingContext('Correct info.plist key-value pairs for objc swift project.', () async {
    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);

    await runner.run(<String>[
      'create',
      '--template=app',
      '--no-pub',
      '--org',
      'com.foo.bar',
      '--ios-language=swift',
      '--project-name=my_project',
      projectDir.path,
    ]);

    final String plistPath = globals.fs.path.join('ios', 'Runner', 'Info.plist');
    final File plistFile = globals.fs.file(globals.fs.path.join(projectDir.path, plistPath));
    expect(plistFile, exists);
    final bool disabled = _getBooleanValueFromPlist(
      plistFile: plistFile,
      key: 'CADisableMinimumFrameDurationOnPhone',
    );
    expect(disabled, isTrue);
    final bool indirectInput = _getBooleanValueFromPlist(
      plistFile: plistFile,
      key: 'UIApplicationSupportsIndirectInputEvents',
    );
    expect(indirectInput, isTrue);
    final String displayName = _getStringValueFromPlist(
      plistFile: plistFile,
      key: 'CFBundleDisplayName',
    );
    expect(displayName, 'My Project');
  });

  testUsingContext(
    'Correct info.plist key-value pairs for objc iOS module.',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>[
        'create',
        '--template=module',
        '--org',
        'com.foo.bar',
        '--ios-language=objc',
        '--project-name=my_project',
        projectDir.path,
      ]);

      final String plistPath = globals.fs.path.join('.ios', 'Runner', 'Info.plist');
      final File plistFile = globals.fs.file(globals.fs.path.join(projectDir.path, plistPath));
      expect(plistFile, exists);
      final bool disabled = _getBooleanValueFromPlist(
        plistFile: plistFile,
        key: 'CADisableMinimumFrameDurationOnPhone',
      );
      expect(disabled, isTrue);
      final bool indirectInput = _getBooleanValueFromPlist(
        plistFile: plistFile,
        key: 'UIApplicationSupportsIndirectInputEvents',
      );
      expect(indirectInput, isTrue);
      final String displayName = _getStringValueFromPlist(
        plistFile: plistFile,
        key: 'CFBundleDisplayName',
      );
      expect(displayName, 'My Project');
    },
    overrides: <Type, Generator>{
      Pub:
          () => Pub.test(
            fileSystem: globals.fs,
            logger: globals.logger,
            processManager: globals.processManager,
            botDetector: globals.botDetector,
            platform: globals.platform,
            stdio: mockStdio,
          ),
    },
  );

  testUsingContext(
    'Correct info.plist key-value pairs for swift iOS module.',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>[
        'create',
        '--template=module',
        '--org',
        'com.foo.bar',
        '--ios-language=swift',
        '--project-name=my_project',
        projectDir.path,
      ]);

      final String plistPath = globals.fs.path.join('.ios', 'Runner', 'Info.plist');
      final File plistFile = globals.fs.file(globals.fs.path.join(projectDir.path, plistPath));
      expect(plistFile, exists);
      final bool disabled = _getBooleanValueFromPlist(
        plistFile: plistFile,
        key: 'CADisableMinimumFrameDurationOnPhone',
      );
      expect(disabled, isTrue);
      final bool indirectInput = _getBooleanValueFromPlist(
        plistFile: plistFile,
        key: 'UIApplicationSupportsIndirectInputEvents',
      );
      expect(indirectInput, isTrue);
      final String displayName = _getStringValueFromPlist(
        plistFile: plistFile,
        key: 'CFBundleDisplayName',
      );
      expect(displayName, 'My Project');
    },
    overrides: <Type, Generator>{
      Pub:
          () => Pub.test(
            fileSystem: globals.fs,
            logger: globals.logger,
            processManager: globals.processManager,
            botDetector: globals.botDetector,
            platform: globals.platform,
            stdio: mockStdio,
          ),
    },
  );

  testUsingContext('Correct info.plist key-value pairs for swift iOS plugin.', () async {
    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);

    await runner.run(<String>[
      'create',
      '--template=plugin',
      '--no-pub',
      '--org',
      'com.foo.bar',
      '--platforms=ios',
      '--ios-language=swift',
      '--project-name=my_project',
      projectDir.path,
    ]);

    final String plistPath = globals.fs.path.join('example', 'ios', 'Runner', 'Info.plist');
    final File plistFile = globals.fs.file(globals.fs.path.join(projectDir.path, plistPath));
    expect(plistFile, exists);
    final bool disabled = _getBooleanValueFromPlist(
      plistFile: plistFile,
      key: 'CADisableMinimumFrameDurationOnPhone',
    );
    expect(disabled, isTrue);
    final bool indirectInput = _getBooleanValueFromPlist(
      plistFile: plistFile,
      key: 'UIApplicationSupportsIndirectInputEvents',
    );
    expect(indirectInput, isTrue);
    final String displayName = _getStringValueFromPlist(
      plistFile: plistFile,
      key: 'CFBundleDisplayName',
    );
    expect(displayName, 'My Project');
  });

  testUsingContext('Correct info.plist key-value pairs for objc iOS plugin.', () async {
    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);

    await runner.run(<String>[
      'create',
      '--template=plugin',
      '--no-pub',
      '--org',
      'com.foo.bar',
      '--platforms=ios',
      '--ios-language=objc',
      '--project-name=my_project',
      projectDir.path,
    ]);

    final String plistPath = globals.fs.path.join('example', 'ios', 'Runner', 'Info.plist');
    final File plistFile = globals.fs.file(globals.fs.path.join(projectDir.path, plistPath));
    expect(plistFile, exists);
    final bool disabled = _getBooleanValueFromPlist(
      plistFile: plistFile,
      key: 'CADisableMinimumFrameDurationOnPhone',
    );
    expect(disabled, isTrue);
    final bool indirectInput = _getBooleanValueFromPlist(
      plistFile: plistFile,
      key: 'UIApplicationSupportsIndirectInputEvents',
    );
    expect(indirectInput, isTrue);
    final String displayName = _getStringValueFromPlist(
      plistFile: plistFile,
      key: 'CFBundleDisplayName',
    );
    expect(displayName, 'My Project');
  });

  testUsingContext(
    'should not show --ios-language deprecation warning issue for Swift',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>['create', '--no-pub', '--ios-language=swift', projectDir.path]);
      expect(
        logger.warningText,
        contains(
          'The "ios-language" option is deprecated and will be removed in a future Flutter release.',
        ),
      );
      expect(
        logger.warningText,
        isNot(contains('https://github.com/flutter/flutter/issues/148586')),
      );
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(), Logger: () => logger},
  );

  testUsingContext(
    'should show --ios-language deprecation warning issue for Objective-C',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>['create', '--no-pub', '--ios-language=objc', projectDir.path]);
      expect(
        logger.warningText,
        contains(
          'The "ios-language" option is deprecated and will be removed in a future Flutter release.',
        ),
      );
      expect(logger.warningText, contains('https://github.com/flutter/flutter/issues/148586'));
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(), Logger: () => logger},
  );

  testUsingContext(
    'has correct content and formatting with macOS app template',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>[
        'create',
        '--template=app',
        '--platforms=macos',
        '--no-pub',
        '--org',
        'com.foo.bar',
        projectDir.path,
      ]);

      void expectExists(String relPath) {
        expect(globals.fs.isFileSync('${projectDir.path}/$relPath'), true);
      }

      // Generated Xcode settings
      final String macosXcodeConfigPath = globals.fs.path.join(
        'macos',
        'Runner',
        'Configs',
        'AppInfo.xcconfig',
      );
      expectExists(macosXcodeConfigPath);
      final File macosXcodeConfigFile = globals.fs.file(
        globals.fs.path.join(projectDir.path, macosXcodeConfigPath),
      );
      final String macosXcodeConfig = macosXcodeConfigFile.readAsStringSync();
      expect(macosXcodeConfig, contains('PRODUCT_NAME = flutter_project'));
      expect(macosXcodeConfig, contains('PRODUCT_BUNDLE_IDENTIFIER = com.foo.bar.flutterProject'));
      expect(macosXcodeConfig, contains('PRODUCT_COPYRIGHT ='));

      // Xcode project
      final String xcodeProjectPath = globals.fs.path.join(
        'macos',
        'Runner.xcodeproj',
        'project.pbxproj',
      );
      expectExists(xcodeProjectPath);
      final File xcodeProjectFile = globals.fs.file(
        globals.fs.path.join(projectDir.path, xcodeProjectPath),
      );
      final String xcodeProject = xcodeProjectFile.readAsStringSync();
      expect(xcodeProject, contains('path = "flutter_project.app";'));
      expect(xcodeProject, contains('LastUpgradeCheck = 1510;'));

      // Xcode workspace shared data
      final Directory workspaceSharedData = globals.fs.directory(
        globals.fs.path.join('macos', 'Runner.xcworkspace', 'xcshareddata'),
      );
      expectExists(workspaceSharedData.childFile('IDEWorkspaceChecks.plist').path);
      // Xcode project shared data
      final Directory projectSharedData = globals.fs.directory(
        globals.fs.path.join('macos', 'Runner.xcodeproj', 'project.xcworkspace', 'xcshareddata'),
      );
      expectExists(projectSharedData.childFile('IDEWorkspaceChecks.plist').path);
    },
    overrides: <Type, Generator>{
      Platform: _kNoColorTerminalPlatform,
      FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true),
    },
  );

  testUsingContext(
    'has correct application id for android, bundle id for ios and application id for Linux',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      String tmpProjectDir = globals.fs.path.join(tempDir.path, 'hello_flutter');
      await runner.run(<String>[
        'create',
        '--template=app',
        '--no-pub',
        '--org',
        'com.example',
        tmpProjectDir,
      ]);
      FlutterProject project = FlutterProject.fromDirectory(globals.fs.directory(tmpProjectDir));
      expect(
        await project.ios.productBundleIdentifier(BuildInfo.debug),
        'com.example.helloFlutter',
      );
      expect(
        await project.ios.productBundleIdentifier(BuildInfo.profile),
        'com.example.helloFlutter',
      );
      expect(
        await project.ios.productBundleIdentifier(BuildInfo.release),
        'com.example.helloFlutter',
      );
      expect(await project.ios.productBundleIdentifier(null), 'com.example.helloFlutter');
      expect(project.android.applicationId, 'com.example.hello_flutter');
      expect(project.linux.applicationId, 'com.example.hello_flutter');

      tmpProjectDir = globals.fs.path.join(tempDir.path, 'test_abc');
      await runner.run(<String>[
        'create',
        '--template=app',
        '--no-pub',
        '--org',
        'abc^*.1#@',
        tmpProjectDir,
      ]);
      project = FlutterProject.fromDirectory(globals.fs.directory(tmpProjectDir));
      expect(await project.ios.productBundleIdentifier(BuildInfo.debug), 'abc.1.testAbc');
      expect(project.android.applicationId, 'abc.u1.test_abc');

      tmpProjectDir = globals.fs.path.join(tempDir.path, 'flutter_project');
      await runner.run(<String>[
        'create',
        '--template=app',
        '--no-pub',
        '--org',
        '#+^%',
        tmpProjectDir,
      ]);
      project = FlutterProject.fromDirectory(globals.fs.directory(tmpProjectDir));
      expect(await project.ios.productBundleIdentifier(BuildInfo.debug), 'flutterProject.untitled');
      expect(project.android.applicationId, 'flutter_project.untitled');
      expect(project.linux.applicationId, 'flutter_project.untitled');
    },
    overrides: <Type, Generator>{
      FlutterVersion: () => fakeFlutterVersion,
      Platform: _kNoColorTerminalPlatform,
      FeatureFlags: () => TestFeatureFlags(isLinuxEnabled: true),
    },
  );

  testUsingContext('can re-gen default template over existing project', () async {
    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);

    await runner.run(<String>['create', '--no-pub', projectDir.path]);

    await runner.run(<String>['create', '--no-pub', projectDir.path]);

    final String metadata =
        globals.fs.file(globals.fs.path.join(projectDir.path, '.metadata')).readAsStringSync();
    expect(LineSplitter.split(metadata), contains('project_type: app'));
  });

  testUsingContext(
    'can re-gen default template over existing app project with no metadata and detect the type',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>['create', '--no-pub', '--template=app', projectDir.path]);

      // Remove the .metadata to simulate an older instantiation that didn't generate those.
      globals.fs.file(globals.fs.path.join(projectDir.path, '.metadata')).deleteSync();

      await runner.run(<String>['create', '--no-pub', projectDir.path]);

      final String metadata =
          globals.fs.file(globals.fs.path.join(projectDir.path, '.metadata')).readAsStringSync();
      expect(LineSplitter.split(metadata), contains('project_type: app'));
    },
  );

  testUsingContext(
    'can re-gen app template over existing app project and detect the type',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>['create', '--no-pub', '--template=app', projectDir.path]);

      await runner.run(<String>['create', '--no-pub', projectDir.path]);

      final String metadata =
          globals.fs.file(globals.fs.path.join(projectDir.path, '.metadata')).readAsStringSync();
      expect(LineSplitter.split(metadata), contains('project_type: app'));
    },
  );

  testUsingContext(
    'can re-gen template over existing module project and detect the type',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>['create', '--no-pub', '--template=module', projectDir.path]);

      await runner.run(<String>['create', '--no-pub', projectDir.path]);

      final String metadata =
          globals.fs.file(globals.fs.path.join(projectDir.path, '.metadata')).readAsStringSync();
      expect(LineSplitter.split(metadata), contains('project_type: module'));
    },
  );

  testUsingContext(
    'can re-gen default template over existing plugin project and detect the type',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>['create', '--no-pub', '--template=plugin', projectDir.path]);

      await runner.run(<String>['create', '--no-pub', projectDir.path]);

      final String metadata =
          globals.fs.file(globals.fs.path.join(projectDir.path, '.metadata')).readAsStringSync();
      expect(LineSplitter.split(metadata), contains('project_type: plugin'));
    },
  );

  testUsingContext(
    'can re-gen default template over existing package project and detect the type',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>['create', '--no-pub', '--template=package', projectDir.path]);

      await runner.run(<String>['create', '--no-pub', projectDir.path]);

      final String metadata =
          globals.fs.file(globals.fs.path.join(projectDir.path, '.metadata')).readAsStringSync();
      expect(LineSplitter.split(metadata), contains('project_type: package'));
    },
  );

  testUsingContext(
    'can re-gen module .android/ folder, reusing custom org',
    () async {
      await _createProject(projectDir, <String>[
        '--template=module',
        '--org',
        'com.bar.foo',
      ], <String>[]);
      projectDir.childDirectory('.android').deleteSync(recursive: true);
      return _createProject(projectDir, <String>[], <String>[
        '.android/app/src/main/java/com/bar/foo/flutter_project/host/MainActivity.java',
      ]);
    },
    overrides: <Type, Generator>{
      Pub:
          () => Pub.test(
            fileSystem: globals.fs,
            logger: globals.logger,
            processManager: globals.processManager,
            botDetector: globals.botDetector,
            platform: globals.platform,
            stdio: mockStdio,
          ),
    },
  );

  testUsingContext(
    'can re-gen module .ios/ folder, reusing custom org',
    () async {
      await _createProject(projectDir, <String>[
        '--template=module',
        '--org',
        'com.bar.foo',
      ], <String>[]);
      projectDir.childDirectory('.ios').deleteSync(recursive: true);
      await _createProject(projectDir, <String>[], <String>[]);
      final FlutterProject project = FlutterProject.fromDirectory(projectDir);
      expect(
        await project.ios.productBundleIdentifier(BuildInfo.debug),
        'com.bar.foo.flutterProject',
      );
    },
    overrides: <Type, Generator>{
      Pub:
          () => Pub.test(
            fileSystem: globals.fs,
            logger: globals.logger,
            processManager: globals.processManager,
            botDetector: globals.botDetector,
            platform: globals.platform,
            stdio: mockStdio,
          ),
    },
  );

  testUsingContext('can re-gen app android/ folder, reusing custom org', () async {
    await _createProject(projectDir, <String>[
      '--no-pub',
      '--template=app',
      '--org',
      'com.bar.foo',
      '-i',
      'objc',
      '-a',
      'java',
    ], <String>[]);
    projectDir.childDirectory('android').deleteSync(recursive: true);
    return _createProject(
      projectDir,
      <String>['--no-pub', '-i', 'objc', '-a', 'java'],
      <String>['android/app/src/main/java/com/bar/foo/flutter_project/MainActivity.java'],
      unexpectedPaths: <String>[
        'android/app/src/main/java/com/example/flutter_project/MainActivity.java',
      ],
    );
  });

  testUsingContext('can re-gen app ios/ folder, reusing custom org', () async {
    await _createProject(projectDir, <String>[
      '--no-pub',
      '--template=app',
      '--org',
      'com.bar.foo',
    ], <String>[]);
    projectDir.childDirectory('ios').deleteSync(recursive: true);
    await _createProject(projectDir, <String>['--no-pub'], <String>[]);
    final FlutterProject project = FlutterProject.fromDirectory(projectDir);
    expect(
      await project.ios.productBundleIdentifier(BuildInfo.debug),
      'com.bar.foo.flutterProject',
    );
  });

  testUsingContext(
    'can re-gen plugin ios/ and example/ folders, reusing custom org, without Swift Package Manager',
    () async {
      await _createProject(projectDir, <String>[
        '--no-pub',
        '--template=plugin',
        '--org',
        'com.bar.foo',
        '-i',
        'objc',
        '-a',
        'java',
        '--platforms',
        'ios,android',
      ], <String>[]);
      projectDir.childDirectory('example').deleteSync(recursive: true);
      projectDir.childDirectory('ios').deleteSync(recursive: true);
      await _createProject(
        projectDir,
        <String>[
          '--no-pub',
          '--template=plugin',
          '-i',
          'objc',
          '-a',
          'java',
          '--platforms',
          'ios,android',
        ],
        <String>[
          'example/android/app/src/main/java/com/bar/foo/flutter_project_example/MainActivity.java',
          'ios/Classes/FlutterProjectPlugin.h',
        ],
        unexpectedPaths: <String>[
          'example/android/app/src/main/java/com/example/flutter_project_example/MainActivity.java',
          'android/src/main/java/com/example/flutter_project/FlutterProjectPlugin.java',
          'ios/flutter_project/Sources/flutter_project/include/flutter_project/FlutterProjectPlugin.h',
        ],
      );
      final FlutterProject project = FlutterProject.fromDirectory(projectDir);
      expect(
        await project.example.ios.productBundleIdentifier(BuildInfo.debug),
        'com.bar.foo.flutterProjectExample',
      );
    },
    overrides: <Type, Generator>{
      // Test flags disable Swift Package Manager.
      FeatureFlags: () => TestFeatureFlags(),
    },
  );

  testUsingContext(
    'can re-gen plugin ios/ and example/ folders, reusing custom org, with Swift Package Manager',
    () async {
      await _createProject(projectDir, <String>[
        '--no-pub',
        '--template=plugin',
        '--org',
        'com.bar.foo',
        '-i',
        'objc',
        '-a',
        'java',
        '--platforms',
        'ios,android',
      ], <String>[]);
      projectDir.childDirectory('example').deleteSync(recursive: true);
      projectDir.childDirectory('ios').deleteSync(recursive: true);
      await _createProject(
        projectDir,
        <String>[
          '--no-pub',
          '--template=plugin',
          '-i',
          'objc',
          '-a',
          'java',
          '--platforms',
          'ios,android',
        ],
        <String>[
          'example/android/app/src/main/java/com/bar/foo/flutter_project_example/MainActivity.java',
          'ios/flutter_project/Sources/flutter_project/include/flutter_project/FlutterProjectPlugin.h',
        ],
        unexpectedPaths: <String>[
          'example/android/app/src/main/java/com/example/flutter_project_example/MainActivity.java',
          'android/src/main/java/com/example/flutter_project/FlutterProjectPlugin.java',
          'ios/Classes/FlutterProjectPlugin.h',
        ],
      );
      final FlutterProject project = FlutterProject.fromDirectory(projectDir);
      expect(
        await project.example.ios.productBundleIdentifier(BuildInfo.debug),
        'com.bar.foo.flutterProjectExample',
      );
    },
    overrides: <Type, Generator>{
      FeatureFlags: () => TestFeatureFlags(isSwiftPackageManagerEnabled: true),
    },
  );

  testUsingContext('fails to re-gen without specified org when org is ambiguous', () async {
    await _createProject(projectDir, <String>[
      '--no-pub',
      '--template=app',
      '--org',
      'com.bar.foo',
    ], <String>[]);
    globals.fs.directory(globals.fs.path.join(projectDir.path, 'ios')).deleteSync(recursive: true);
    await _createProject(projectDir, <String>[
      '--no-pub',
      '--template=app',
      '--org',
      'com.bar.baz',
    ], <String>[]);
    expect(
      () => _createProject(projectDir, <String>[], <String>[]),
      throwsToolExit(message: 'Ambiguous organization'),
    );
  });

  testUsingContext('fails when file exists where output directory should be', () async {
    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);
    final File existingFile = globals.fs.file(globals.fs.path.join(projectDir.path, 'bad'));
    if (!existingFile.existsSync()) {
      existingFile.createSync(recursive: true);
    }
    expect(
      runner.run(<String>['create', existingFile.path]),
      throwsToolExit(message: 'existing file'),
    );
  });

  testUsingContext('fails overwrite when file exists where output directory should be', () async {
    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);
    final File existingFile = globals.fs.file(globals.fs.path.join(projectDir.path, 'bad'));
    if (!existingFile.existsSync()) {
      existingFile.createSync(recursive: true);
    }
    expect(
      runner.run(<String>['create', '--overwrite', existingFile.path]),
      throwsToolExit(message: 'existing file'),
    );
  });

  testUsingContext(
    'overwrites existing directory when requested',
    () async {
      final Directory existingDirectory = globals.fs.directory(
        globals.fs.path.join(projectDir.path, 'bad'),
      );
      if (!existingDirectory.existsSync()) {
        existingDirectory.createSync(recursive: true);
      }
      final File existingFile = globals.fs.file(
        globals.fs.path.join(existingDirectory.path, 'lib', 'main.dart'),
      );
      existingFile.createSync(recursive: true);
      await _createProject(
        globals.fs.directory(existingDirectory.path),
        <String>['--overwrite', '-i', 'objc', '-a', 'java'],
        <String>[
          'android/app/src/main/java/io/flutter/plugins/GeneratedPluginRegistrant.java',
          'lib/main.dart',
          'ios/Flutter/AppFrameworkInfo.plist',
          'ios/Runner/AppDelegate.m',
          'ios/Runner/GeneratedPluginRegistrant.h',
        ],
      );
    },
    overrides: <Type, Generator>{
      Pub:
          () => Pub.test(
            fileSystem: globals.fs,
            logger: globals.logger,
            processManager: globals.processManager,
            botDetector: globals.botDetector,
            platform: globals.platform,
            stdio: mockStdio,
          ),
    },
  );

  testUsingContext(
    'invokes pub in online and offline modes',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      // Run pub online first in order to populate the pub cache.
      await runner.run(<String>['create', '--pub', projectDir.path]);
      final RegExp dartCommand = RegExp(r'dart-sdk[\\/]bin[\\/]dart');
      expect(
        loggingProcessManager.commands,
        contains(
          predicate(
            (List<String> c) =>
                dartCommand.hasMatch(c[0]) && c[1].contains('pub') && !c.contains('--offline'),
          ),
        ),
      );

      // Run pub offline.
      loggingProcessManager.clear();
      await runner.run(<String>['create', '--pub', '--offline', projectDir.path]);
      expect(
        loggingProcessManager.commands,
        contains(
          predicate(
            (List<String> c) =>
                dartCommand.hasMatch(c[0]) && c[1].contains('pub') && c.contains('--offline'),
          ),
        ),
      );
    },
    overrides: <Type, Generator>{
      ProcessManager: () => loggingProcessManager,
      Pub:
          () => Pub.test(
            fileSystem: globals.fs,
            logger: globals.logger,
            processManager: globals.processManager,
            botDetector: globals.botDetector,
            platform: globals.platform,
            stdio: mockStdio,
          ),
    },
  );

  testUsingContext('can create an empty application project', () async {
    await _createAndAnalyzeProject(
      projectDir,
      <String>['--no-pub', '--empty'],
      <String>[
        'lib/main.dart',
        'flutter_project.iml',
        'android/app/src/main/AndroidManifest.xml',
        'ios/Flutter/AppFrameworkInfo.plist',
      ],
      unexpectedPaths: <String>['test'],
      expectedGitignoreLines: flutterPluginsIgnores,
    );
    expect(
      projectDir.childDirectory('lib').childFile('main.dart').readAsStringSync(),
      contains("Text('Hello World!')"),
    );
    expect(
      projectDir.childDirectory('lib').childFile('main.dart').readAsStringSync(),
      isNot(contains('int _counter')),
    );
    expect(projectDir.childFile('analysis_options.yaml').readAsStringSync(), isNot(contains('#')));
    expect(
      projectDir.childFile('README.md').readAsStringSync(),
      isNot(contains('Getting Started')),
    );
  });

  testUsingContext("can't create an empty non-application project", () async {
    final String outputDir = globals.fs.path.join(tempDir.path, 'test_project');
    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);
    final List<String> args = <String>[
      'create',
      '--no-pub',
      '--empty',
      '--template=plugin',
      outputDir,
    ];

    await expectLater(
      runner.run(args),
      throwsToolExit(message: 'The --empty flag is only supported for the app template.'),
    );
  });

  testUsingContext(
    'does not remove an existing test/ directory when recreating an application project with the --empty flag',
    () async {
      await _createProject(projectDir, <String>['--no-pub', '--empty'], <String>[]);

      projectDir.childDirectory('test').childFile('example_test.dart').createSync(recursive: true);

      await _createProject(
        projectDir,
        <String>['--no-pub', '--empty'],
        <String>['test/example_test.dart'],
      );

      expect(projectDir.childDirectory('test').childFile('example_test.dart'), exists);
    },
  );

  testUsingContext(
    'does not create a test/ directory when creating a new application project with the --empty flag',
    () async {
      await _createProject(
        projectDir,
        <String>['--no-pub', '--empty'],
        <String>[],
        unexpectedPaths: <String>['test'],
      );

      expect(projectDir.childDirectory('test'), isNot(exists));
    },
  );

  testUsingContext(
    "does not create a test/ directory, if it doesn't already exist, when recreating an application project with the --empty flag",
    () async {
      await _createProject(projectDir, <String>['--no-pub', '--empty'], <String>[]);

      await _createProject(
        projectDir,
        <String>['--no-pub', '--empty'],
        <String>[],
        unexpectedPaths: <String>['test'],
      );

      expect(projectDir.childDirectory('test'), isNot(exists));
    },
  );

  testUsingContext(
    'can create a sample-based project',
    () async {
      await _createAndAnalyzeProject(
        projectDir,
        <String>['--no-pub', '--sample=foo.bar.Baz'],
        <String>[
          'lib/main.dart',
          'flutter_project.iml',
          'android/app/src/main/AndroidManifest.xml',
          'ios/Flutter/AppFrameworkInfo.plist',
        ],
        unexpectedPaths: <String>['test'],
        expectedGitignoreLines: flutterPluginsIgnores,
      );
      expect(
        projectDir.childDirectory('lib').childFile('main.dart').readAsStringSync(),
        contains('void main() {}'),
      );
    },
    overrides: <Type, Generator>{
      HttpClientFactory: () {
        return () {
          return FakeHttpClient.list(<FakeRequest>[
            FakeRequest(
              Uri.parse('https://main-api.flutter.dev/snippets/foo.bar.Baz.dart'),
              response: FakeResponse(body: utf8.encode('void main() {}')),
            ),
          ]);
        };
      },
    },
  );

  testUsingContext(
    'null-safe sample-based project have no analyzer errors',
    () async {
      await _createAndAnalyzeProject(
        projectDir,
        <String>['--no-pub', '--sample=foo.bar.Baz'],
        <String>['lib/main.dart'],
      );
      expect(
        projectDir.childDirectory('lib').childFile('main.dart').readAsStringSync(),
        contains('String?'), // uses null-safe syntax
      );
    },
    overrides: <Type, Generator>{
      HttpClientFactory: () {
        return () {
          return FakeHttpClient.list(<FakeRequest>[
            FakeRequest(
              Uri.parse('https://main-api.flutter.dev/snippets/foo.bar.Baz.dart'),
              response: FakeResponse(
                body: utf8.encode(
                  'void main() { String? foo; print(foo); } // ignore: avoid_print',
                ),
              ),
            ),
          ]);
        };
      },
    },
  );

  testUsingContext(
    'can write samples index to disk',
    () async {
      final String outputFile = globals.fs.path.join(tempDir.path, 'flutter_samples.json');
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      final List<String> args = <String>['create', '--list-samples', outputFile];

      await runner.run(args);
      final File expectedFile = globals.fs.file(outputFile);
      expect(expectedFile, exists);
      expect(expectedFile.readAsStringSync(), equals(samplesIndexJson));
    },
    overrides: <Type, Generator>{
      HttpClientFactory: () {
        return () {
          return FakeHttpClient.list(<FakeRequest>[
            FakeRequest(
              Uri.parse('https://main-api.flutter.dev/snippets/index.json'),
              response: FakeResponse(body: utf8.encode(samplesIndexJson)),
            ),
          ]);
        };
      },
    },
  );

  testUsingContext(
    'Throws tool exit on empty samples index',
    () async {
      final String outputFile = globals.fs.path.join(tempDir.path, 'flutter_samples.json');
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      final List<String> args = <String>['create', '--list-samples', outputFile];

      await expectLater(
        runner.run(args),
        throwsToolExit(exitCode: 2, message: 'Unable to download samples'),
      );
    },
    overrides: <Type, Generator>{
      HttpClientFactory: () {
        return () {
          return FakeHttpClient.list(<FakeRequest>[
            FakeRequest(Uri.parse('https://main-api.flutter.dev/snippets/index.json')),
          ]);
        };
      },
    },
  );

  testUsingContext(
    'provides an error to the user if samples json download fails',
    () async {
      final String outputFile = globals.fs.path.join(tempDir.path, 'flutter_samples.json');
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      final List<String> args = <String>['create', '--list-samples', outputFile];

      await expectLater(
        runner.run(args),
        throwsToolExit(exitCode: 2, message: 'Failed to write samples'),
      );
      expect(globals.fs.file(outputFile), isNot(exists));
    },
    overrides: <Type, Generator>{
      HttpClientFactory: () {
        return () {
          return FakeHttpClient.list(<FakeRequest>[
            FakeRequest(
              Uri.parse('https://main-api.flutter.dev/snippets/index.json'),
              response: const FakeResponse(statusCode: HttpStatus.notFound),
            ),
          ]);
        };
      },
    },
  );

  testUsingContext(
    'plugin does not support any platform by default',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>['create', '--no-pub', '--template=plugin', projectDir.path]);

      expect(projectDir.childDirectory('ios'), isNot(exists));
      expect(projectDir.childDirectory('android'), isNot(exists));
      expect(projectDir.childDirectory('web'), isNot(exists));
      expect(projectDir.childDirectory('linux'), isNot(exists));
      expect(projectDir.childDirectory('windows'), isNot(exists));
      expect(projectDir.childDirectory('macos'), isNot(exists));

      expect(projectDir.childDirectory('example').childDirectory('ios'), isNot(exists));
      expect(projectDir.childDirectory('example').childDirectory('android'), isNot(exists));
      expect(projectDir.childDirectory('example').childDirectory('web'), isNot(exists));
      expect(projectDir.childDirectory('example').childDirectory('linux'), isNot(exists));
      expect(projectDir.childDirectory('example').childDirectory('windows'), isNot(exists));
      expect(projectDir.childDirectory('example').childDirectory('macos'), isNot(exists));
      validatePubspecForPlugin(
        projectDir: projectDir.absolute.path,
        expectedPlatforms: <String>['some_platform'],
        pluginClass: 'somePluginClass',
        unexpectedPlatforms: <String>['ios', 'android', 'web', 'linux', 'windows', 'macos'],
      );
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags()},
  );

  testUsingContext(
    'plugin creates platform interface by default',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>['create', '--no-pub', '--template=plugin', projectDir.path]);

      expect(
        projectDir.childDirectory('lib').childFile('flutter_project_method_channel.dart'),
        exists,
      );
      expect(
        projectDir.childDirectory('lib').childFile('flutter_project_platform_interface.dart'),
        exists,
      );
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags()},
  );

  testUsingContext(
    'plugin passes analysis and unit tests',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>['create', '--no-pub', '--template=plugin', projectDir.path]);

      await _getPackages(projectDir);
      await analyzeProject(projectDir.path);
      await _runFlutterTest(projectDir);
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags()},
  );

  testUsingContext('plugin example passes analysis and unit tests', () async {
    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);

    await runner.run(<String>['create', '--no-pub', '--template=plugin', projectDir.path]);

    final Directory exampleDir = projectDir.childDirectory('example');

    await _getPackages(exampleDir);
    await analyzeProject(exampleDir.path);
    await _runFlutterTest(exampleDir);
  });

  testUsingContext(
    'plugin supports ios if requested',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=ios',
        projectDir.path,
      ]);

      expect(projectDir.childDirectory('ios'), exists);
      expect(projectDir.childDirectory('example').childDirectory('ios'), exists);
      validatePubspecForPlugin(
        projectDir: projectDir.absolute.path,
        expectedPlatforms: <String>['ios'],
        pluginClass: 'FlutterProjectPlugin',
        unexpectedPlatforms: <String>['some_platform'],
      );
      expect(logger.errorText, isNot(contains(_kNoPlatformsMessage)));
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(), Logger: () => logger},
  );

  testUsingContext(
    'plugin supports android if requested',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=android',
        projectDir.path,
      ]);

      expect(projectDir.childDirectory('android'), exists);
      expect(projectDir.childDirectory('example').childDirectory('android'), exists);
      validatePubspecForPlugin(
        projectDir: projectDir.absolute.path,
        expectedPlatforms: const <String>['android'],
        pluginClass: 'FlutterProjectPlugin',
        unexpectedPlatforms: <String>['some_platform'],
        androidIdentifier: 'com.example.flutter_project',
      );
      expect(logger.errorText, isNot(contains(_kNoPlatformsMessage)));
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(), Logger: () => logger},
  );

  testUsingContext(
    'plugin supports web if requested',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=web',
        projectDir.path,
      ]);
      expect(projectDir.childDirectory('lib').childFile('flutter_project_web.dart'), exists);
      validatePubspecForPlugin(
        projectDir: projectDir.absolute.path,
        expectedPlatforms: const <String>['web'],
        pluginClass: 'FlutterProjectWeb',
        unexpectedPlatforms: <String>['some_platform'],
        androidIdentifier: 'com.example.flutter_project',
        webFileName: 'flutter_project_web.dart',
      );
      expect(logger.errorText, isNot(contains(_kNoPlatformsMessage)));

      await _getPackages(projectDir);
      await analyzeProject(projectDir.path);
      await _runFlutterTest(projectDir);
    },
    overrides: <Type, Generator>{
      FeatureFlags: () => TestFeatureFlags(isWebEnabled: true),
      Logger: () => logger,
    },
  );

  testUsingContext(
    'plugin does not support web if feature is not enabled',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=web',
        projectDir.path,
      ]);
      expect(projectDir.childDirectory('lib').childFile('flutter_project_web.dart'), isNot(exists));
      validatePubspecForPlugin(
        projectDir: projectDir.absolute.path,
        expectedPlatforms: const <String>['some_platform'],
        pluginClass: 'somePluginClass',
        unexpectedPlatforms: <String>['web'],
      );
      expect(logger.errorText, contains(_kNoPlatformsMessage));
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(), Logger: () => logger},
  );

  testUsingContext(
    'create an empty plugin, then add ios',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      await runner.run(<String>['create', '--no-pub', '--template=plugin', projectDir.path]);
      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=ios',
        projectDir.path,
      ]);

      expect(projectDir.childDirectory('ios'), exists);
      expect(projectDir.childDirectory('example').childDirectory('ios'), exists);
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags()},
  );

  testUsingContext(
    'create an empty plugin, then add android',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      await runner.run(<String>['create', '--no-pub', '--template=plugin', projectDir.path]);
      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=android',
        projectDir.path,
      ]);

      expect(projectDir.childDirectory('android'), exists);
      expect(projectDir.childDirectory('example').childDirectory('android'), exists);
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags()},
  );

  testUsingContext(
    'create an empty plugin, then add linux',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      await runner.run(<String>['create', '--no-pub', '--template=plugin', projectDir.path]);
      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=linux',
        projectDir.path,
      ]);

      expect(projectDir.childDirectory('linux'), exists);
      expect(projectDir.childDirectory('example').childDirectory('linux'), exists);
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(isLinuxEnabled: true)},
  );

  testUsingContext(
    'create an empty plugin, then add macos',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      await runner.run(<String>['create', '--no-pub', '--template=plugin', projectDir.path]);
      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=macos',
        projectDir.path,
      ]);

      expect(projectDir.childDirectory('macos'), exists);
      expect(projectDir.childDirectory('example').childDirectory('macos'), exists);
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true)},
  );

  testUsingContext(
    'create an empty plugin, then add windows',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      await runner.run(<String>['create', '--no-pub', '--template=plugin', projectDir.path]);
      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=windows',
        projectDir.path,
      ]);

      expect(projectDir.childDirectory('windows'), exists);
      expect(projectDir.childDirectory('example').childDirectory('windows'), exists);
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true)},
  );

  testUsingContext(
    'create an empty plugin, then add web',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      await runner.run(<String>['create', '--no-pub', '--template=plugin', projectDir.path]);
      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=web',
        projectDir.path,
      ]);

      expect(projectDir.childDirectory('lib').childFile('flutter_project_web.dart'), exists);
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(isWebEnabled: true)},
  );

  testUsingContext(
    'create a plugin with ios, then add macos',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=ios',
        projectDir.path,
      ]);
      expect(projectDir.childDirectory('ios'), exists);
      expect(projectDir.childDirectory('example').childDirectory('ios'), exists);
      validatePubspecForPlugin(
        projectDir: projectDir.absolute.path,
        expectedPlatforms: const <String>['ios'],
        pluginClass: 'FlutterProjectPlugin',
        unexpectedPlatforms: <String>['some_platform'],
      );

      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=macos',
        projectDir.path,
      ]);
      expect(projectDir.childDirectory('macos'), exists);
      expect(projectDir.childDirectory('example').childDirectory('macos'), exists);
      expect(projectDir.childDirectory('ios'), exists);
      expect(projectDir.childDirectory('example').childDirectory('ios'), exists);
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true)},
  );

  testUsingContext('create a plugin with ios and android', () async {
    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);
    await runner.run(<String>[
      'create',
      '--no-pub',
      '--template=plugin',
      '--platforms=ios,android',
      projectDir.path,
    ]);
    expect(projectDir.childDirectory('ios'), exists);
    expect(projectDir.childDirectory('example').childDirectory('ios'), exists);

    expect(projectDir.childDirectory('android'), exists);
    expect(projectDir.childDirectory('example').childDirectory('android'), exists);
    expect(projectDir.childDirectory('ios'), exists);
    expect(projectDir.childDirectory('example').childDirectory('ios'), exists);
    validatePubspecForPlugin(
      projectDir: projectDir.absolute.path,
      expectedPlatforms: const <String>['ios', 'android'],
      pluginClass: 'FlutterProjectPlugin',
      unexpectedPlatforms: <String>['some_platform'],
      androidIdentifier: 'com.example.flutter_project',
    );
  });

  testUsingContext(
    'plugin includes native Swift unit tests',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platforms=ios,macos',
        projectDir.path,
      ]);

      expect(
        projectDir
            .childDirectory('example')
            .childDirectory('ios')
            .childDirectory('RunnerTests')
            .childFile('RunnerTests.swift'),
        exists,
      );
      expect(
        projectDir
            .childDirectory('example')
            .childDirectory('macos')
            .childDirectory('RunnerTests')
            .childFile('RunnerTests.swift'),
        exists,
      );
    },
    overrides: <Type, Generator>{
      FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true),
      Logger: () => logger,
    },
  );

  testUsingContext(
    'plugin includes native Kotlin unit tests',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--org=com.example',
        '--platforms=android',
        projectDir.path,
      ]);

      expect(
        projectDir
            .childDirectory('android')
            .childDirectory('src')
            .childDirectory('test')
            .childDirectory('kotlin')
            .childDirectory('com')
            .childDirectory('example')
            .childDirectory('flutter_project')
            .childFile('FlutterProjectPluginTest.kt'),
        exists,
      );
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(), Logger: () => logger},
  );

  testUsingContext(
    'plugin includes native Java unit tests',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--org=com.example',
        '--platforms=android',
        '-a',
        'java',
        projectDir.path,
      ]);

      expect(
        projectDir
            .childDirectory('android')
            .childDirectory('src')
            .childDirectory('test')
            .childDirectory('java')
            .childDirectory('com')
            .childDirectory('example')
            .childDirectory('flutter_project')
            .childFile('FlutterProjectPluginTest.java'),
        exists,
      );
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(), Logger: () => logger},
  );

  testUsingContext(
    'plugin includes native Objective-C unit tests',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platforms=ios',
        '-i',
        'objc',
        projectDir.path,
      ]);

      expect(
        projectDir
            .childDirectory('example')
            .childDirectory('ios')
            .childDirectory('RunnerTests')
            .childFile('RunnerTests.m'),
        exists,
      );
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(), Logger: () => logger},
  );

  testUsingContext(
    'plugin includes native Windows unit tests',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platforms=windows',
        projectDir.path,
      ]);

      expect(
        projectDir
            .childDirectory('windows')
            .childDirectory('test')
            .childFile('flutter_project_plugin_test.cpp'),
        exists,
      );
    },
    overrides: <Type, Generator>{
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
      Logger: () => logger,
    },
  );

  testUsingContext(
    'plugin includes native Linux unit tests',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platforms=linux',
        projectDir.path,
      ]);

      expect(
        projectDir
            .childDirectory('linux')
            .childDirectory('test')
            .childFile('flutter_project_plugin_test.cc'),
        exists,
      );
    },
    overrides: <Type, Generator>{
      FeatureFlags: () => TestFeatureFlags(isLinuxEnabled: true),
      Logger: () => logger,
    },
  );

  testUsingContext('create a module with --platforms throws error.', () async {
    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);
    await expectLater(
      runner.run(<String>[
        'create',
        '--no-pub',
        '--template=module',
        '--platform=ios',
        projectDir.path,
      ]),
      throwsToolExit(message: 'The "--platforms" argument is not supported', exitCode: 2),
    );
  });

  testUsingContext('create a package with --platforms throws error.', () async {
    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);
    await expectLater(
      runner.run(<String>[
        'create',
        '--no-pub',
        '--template=package',
        '--platform=ios',
        projectDir.path,
      ]),
      throwsToolExit(message: 'The "--platforms" argument is not supported', exitCode: 2),
    );
  });

  testUsingContext(
    'create an ffi package with --platforms throws error.',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      await expectLater(
        runner.run(<String>[
          'create',
          '--no-pub',
          '--template=package_ffi',
          '--platform=ios',
          projectDir.path,
        ]),
        throwsToolExit(message: 'The "--platforms" argument is not supported', exitCode: 2),
      );
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(isNativeAssetsEnabled: true)},
  );

  testUsingContext('create a plugin with android, delete then re-create folders', () async {
    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);
    await runner.run(<String>[
      'create',
      '--no-pub',
      '--template=plugin',
      '--platform=android',
      projectDir.path,
    ]);
    expect(projectDir.childDirectory('android'), exists);
    expect(projectDir.childDirectory('example').childDirectory('android'), exists);

    globals.fs.file(globals.fs.path.join(projectDir.path, 'android')).deleteSync(recursive: true);
    globals.fs
        .file(globals.fs.path.join(projectDir.path, 'example/android'))
        .deleteSync(recursive: true);
    expect(projectDir.childDirectory('android'), isNot(exists));
    expect(projectDir.childDirectory('example').childDirectory('android'), isNot(exists));

    await runner.run(<String>['create', '--no-pub', projectDir.path]);

    expect(projectDir.childDirectory('android'), exists);
    expect(projectDir.childDirectory('example').childDirectory('android'), exists);
  });

  testUsingContext(
    'create a plugin with android, delete then re-create folders while also adding windows',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=android',
        projectDir.path,
      ]);
      expect(projectDir.childDirectory('android'), exists);
      expect(projectDir.childDirectory('example').childDirectory('android'), exists);

      globals.fs.file(globals.fs.path.join(projectDir.path, 'android')).deleteSync(recursive: true);
      globals.fs
          .file(globals.fs.path.join(projectDir.path, 'example/android'))
          .deleteSync(recursive: true);
      expect(projectDir.childDirectory('android'), isNot(exists));
      expect(projectDir.childDirectory('example').childDirectory('android'), isNot(exists));

      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=windows',
        projectDir.path,
      ]);

      expect(projectDir.childDirectory('android'), exists);
      expect(projectDir.childDirectory('example').childDirectory('android'), exists);
      expect(projectDir.childDirectory('windows'), exists);
      expect(projectDir.childDirectory('example').childDirectory('windows'), exists);
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true)},
  );

  testUsingContext(
    'flutter create . on and existing plugin does not add android folders if android is not supported in pubspec',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=ios',
        projectDir.path,
      ]);

      await runner.run(<String>['create', '--no-pub', projectDir.path]);
      expect(projectDir.childDirectory('android'), isNot(exists));
      expect(projectDir.childDirectory('example').childDirectory('android'), isNot(exists));
    },
  );

  testUsingContext(
    'flutter create . on and existing plugin does not add windows folder even feature is enabled',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=android',
        projectDir.path,
      ]);

      await runner.run(<String>['create', '--no-pub', projectDir.path]);
      expect(projectDir.childDirectory('windows'), isNot(exists));
      expect(projectDir.childDirectory('example').childDirectory('windows'), isNot(exists));
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true)},
  );

  testUsingContext(
    'flutter create . on and existing plugin does not add linux folder even feature is enabled',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=android',
        projectDir.path,
      ]);

      await runner.run(<String>['create', '--no-pub', projectDir.path]);
      expect(projectDir.childDirectory('linux'), isNot(exists));
      expect(projectDir.childDirectory('example').childDirectory('linux'), isNot(exists));
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(isLinuxEnabled: true)},
  );

  testUsingContext(
    'flutter create . on and existing plugin does not add web files even feature is enabled',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=android',
        projectDir.path,
      ]);

      await runner.run(<String>['create', '--no-pub', projectDir.path]);
      expect(projectDir.childDirectory('lib').childFile('flutter_project_web.dart'), isNot(exists));
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(isWebEnabled: true)},
  );

  testUsingContext(
    'flutter create . on and existing plugin does not add macos folder even feature is enabled',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=android',
        projectDir.path,
      ]);

      await runner.run(<String>['create', '--no-pub', projectDir.path]);
      expect(projectDir.childDirectory('macos'), isNot(exists));
      expect(projectDir.childDirectory('example').childDirectory('macos'), isNot(exists));
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true)},
  );

  testUsingContext(
    'flutter create . on and existing plugin should show "Your example app code in"',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      final String projectDirPath = globals.fs.path.normalize(projectDir.absolute.path);
      final String relativePluginPath = globals.fs.path.normalize(
        globals.fs.path.relative(projectDirPath),
      );
      final String relativeExamplePath = globals.fs.path.normalize(
        globals.fs.path.join(relativePluginPath, 'example/lib/main.dart'),
      );

      await runner.run(<String>[
        'create',
        '--no-pub',
        '--org=com.example',
        '--template=plugin',
        '--platform=android',
        projectDir.path,
      ]);
      expect(logger.statusText, contains('Your example app code is in $relativeExamplePath.\n'));
      await runner.run(<String>[
        'create',
        '--no-pub',
        '--org=com.example',
        '--template=plugin',
        '--platform=ios',
        projectDir.path,
      ]);
      expect(logger.statusText, contains('Your example app code is in $relativeExamplePath.\n'));
      await runner.run(<String>['create', '--no-pub', projectDir.path]);
      expect(logger.statusText, contains('Your example app code is in $relativeExamplePath.\n'));
    },
    overrides: <Type, Generator>{Logger: () => logger},
  );

  testUsingContext(
    'flutter create -t plugin in an empty folder should not show pubspec.yaml updating suggestion',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=android',
        projectDir.path,
      ]);
      final String projectDirPath = globals.fs.path.normalize(projectDir.absolute.path);
      final String relativePluginPath = globals.fs.path.normalize(
        globals.fs.path.relative(projectDirPath),
      );
      expect(
        logger.statusText,
        isNot(
          contains('You need to update $relativePluginPath/pubspec.yaml to support android.\n'),
        ),
      );
    },
    overrides: <Type, Generator>{Logger: () => logger},
  );

  testUsingContext(
    'flutter create -t plugin in an existing plugin should show pubspec.yaml updating suggestion',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      final String projectDirPath = globals.fs.path.normalize(projectDir.absolute.path);
      final String relativePluginPath = globals.fs.path.normalize(
        globals.fs.path.relative(projectDirPath),
      );
      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=ios',
        projectDir.path,
      ]);
      expect(
        logger.statusText,
        isNot(contains('You need to update $relativePluginPath/pubspec.yaml to support ios.\n')),
      );
      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=android',
        projectDir.path,
      ]);
      expect(
        logger.statusText,
        contains('You need to update $relativePluginPath/pubspec.yaml to support android.\n'),
      );
    },
    overrides: <Type, Generator>{Logger: () => logger},
  );

  testUsingContext('newly created plugin has min flutter sdk version as 3.3.0', () async {
    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);
    await runner.run(<String>['create', '--no-pub', '--template=plugin', projectDir.path]);
    final String rawPubspec = await projectDir.childFile('pubspec.yaml').readAsString();
    final Pubspec pubspec = Pubspec.parse(rawPubspec);
    final Map<String, VersionConstraint?> env = pubspec.environment;
    expect(env['flutter']!.allows(Version(3, 3, 0)), true);
    expect(env['flutter']!.allows(Version(3, 2, 9)), false);
  });

  testUsingContext('newly created iOS plugins has correct min iOS version', () async {
    final String flutterToolsAbsolutePath = globals.fs.path.join(
      Cache.flutterRoot!,
      'packages',
      'flutter_tools',
    );
    final List<String> iosPluginTemplates = <String>[
      globals.fs.path.join(
        flutterToolsAbsolutePath,
        'templates',
        'plugin',
        'ios-objc.tmpl',
        'projectName.podspec.tmpl',
      ),
      globals.fs.path.join(
        flutterToolsAbsolutePath,
        'templates',
        'plugin',
        'ios-swift.tmpl',
        'projectName.podspec.tmpl',
      ),
      globals.fs.path.join(
        flutterToolsAbsolutePath,
        'templates',
        'plugin_ffi',
        'ios.tmpl',
        'projectName.podspec.tmpl',
      ),
    ];

    for (final String templatePath in iosPluginTemplates) {
      final String rawTemplate = globals.fs.file(templatePath).readAsStringSync();
      expect(rawTemplate, contains("s.platform = :ios, '12.0'"));
    }

    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);
    await runner.run(<String>[
      'create',
      '--no-pub',
      '--template=plugin',
      '--platform=ios',
      projectDir.path,
    ]);

    expect(projectDir.childDirectory('ios').childFile('flutter_project.podspec'), exists);
    final String rawPodSpec =
        await projectDir.childDirectory('ios').childFile('flutter_project.podspec').readAsString();
    expect(rawPodSpec, contains("s.platform = :ios, '12.0'"));
  });

  testUsingContext('default app uses flutter default versions', () async {
    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);

    await runner.run(<String>['create', '--no-pub', projectDir.path]);

    expect(globals.fs.isFileSync('${projectDir.path}/android/app/build.gradle.kts'), true);

    final String buildContent =
        await globals.fs.file('${projectDir.path}/android/app/build.gradle.kts').readAsString();

    expect(buildContent.contains('compileSdk = flutter.compileSdkVersion'), true);
    expect(buildContent.contains('ndkVersion = flutter.ndkVersion'), true);
    expect(buildContent.contains('minSdk = flutter.minSdkVersion'), true);
    expect(buildContent.contains('targetSdk = flutter.targetSdkVersion'), true);
  });

  testUsingContext('Android Java plugin contains namespace', () async {
    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);

    await runner.run(<String>[
      'create',
      '--no-pub',
      '-t',
      'plugin',
      '--org',
      'com.bar.foo',
      '-a',
      'java',
      '--platforms=android',
      projectDir.path,
    ]);

    final File buildGradleFile = globals.fs.file('${projectDir.path}/android/build.gradle');

    expect(buildGradleFile.existsSync(), true);

    final String buildGradleContent = await buildGradleFile.readAsString();

    expect(buildGradleContent.contains('namespace = "com.bar.foo.flutter_project"'), true);
  });

  testUsingContext('Android FFI plugin contains namespace', () async {
    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);

    await runner.run(<String>[
      'create',
      '--no-pub',
      '-t',
      'plugin_ffi',
      '--org',
      'com.bar.foo',
      '--platforms=android',
      projectDir.path,
    ]);

    final File buildGradleFile = globals.fs.file('${projectDir.path}/android/build.gradle');

    expect(buildGradleFile.existsSync(), true);

    final String buildGradleContent = await buildGradleFile.readAsString();

    expect(buildGradleContent.contains('namespace = "com.bar.foo.flutter_project"'), true);
  });

  testUsingContext('Android FFI plugin contains 16kb page support', () async {
    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);

    await runner.run(<String>[
      'create',
      '--no-pub',
      '-t',
      'plugin_ffi',
      '--org',
      'com.bar.foo',
      '--platforms=android',
      projectDir.path,
    ]);

    final File cmakeLists = globals.fs.file('${projectDir.path}/src/CMakeLists.txt');

    expect(cmakeLists.existsSync(), true);

    final String cmakeListsContent = await cmakeLists.readAsString();
    // If we ever change the flags, this should be accounted for in the
    // migration as well:
    // lib/src/android/migrations/cmake_android_16k_pages_migration.dart
    const String expected16KbFlags = 'PRIVATE "-Wl,-z,max-page-size=16384")';
    expect(cmakeListsContent, contains(expected16KbFlags));
  });

  testUsingContext('Android Kotlin plugin contains namespace', () async {
    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);

    await runner.run(<String>[
      'create',
      '--no-pub',
      '-t',
      'plugin',
      '--org',
      'com.bar.foo',
      '-a',
      'kotlin',
      '--platforms=android',
      projectDir.path,
    ]);

    final File buildGradleFile = globals.fs.file('${projectDir.path}/android/build.gradle');

    expect(buildGradleFile.existsSync(), true);

    final String buildGradleContent = await buildGradleFile.readAsString();

    expect(buildGradleContent.contains('namespace = "com.bar.foo.flutter_project"'), true);
  });

  testUsingContext('Android Java plugin sets explicit compatibility version', () async {
    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);

    await runner.run(<String>[
      'create',
      '--no-pub',
      '-t',
      'plugin',
      '--org',
      'com.bar.foo',
      '-a',
      'java',
      '--platforms=android',
      projectDir.path,
    ]);

    final File buildGradleFile = globals.fs.file('${projectDir.path}/android/build.gradle');

    expect(buildGradleFile.existsSync(), true);

    final String buildGradleContent = await buildGradleFile.readAsString();

    expect(buildGradleContent.contains('sourceCompatibility = JavaVersion.VERSION_11'), true);
    expect(buildGradleContent.contains('targetCompatibility = JavaVersion.VERSION_11'), true);
  });

  testUsingContext('Android Kotlin plugin sets explicit compatibility version', () async {
    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);

    await runner.run(<String>[
      'create',
      '--no-pub',
      '-t',
      'plugin',
      '--org',
      'com.bar.foo',
      '-a',
      'kotlin',
      '--platforms=android',
      projectDir.path,
    ]);

    final File buildGradleFile = globals.fs.file('${projectDir.path}/android/build.gradle');

    expect(buildGradleFile.existsSync(), true);

    final String buildGradleContent = await buildGradleFile.readAsString();

    expect(buildGradleContent.contains('sourceCompatibility = JavaVersion.VERSION_11'), true);
    expect(buildGradleContent.contains('targetCompatibility = JavaVersion.VERSION_11'), true);
    // jvmTarget should be set to the same value.
    expect(buildGradleContent.contains('jvmTarget = JavaVersion.VERSION_11'), true);
  });

  testUsingContext(
    'Flutter module Android project contains namespace',
    () async {
      const String moduleBuildGradleFilePath = '.android/build.gradle';
      const String moduleAppBuildGradleFlePath = '.android/app/build.gradle';
      const String moduleFlutterBuildGradleFilePath = '.android/Flutter/build.gradle';
      await _createProject(
        projectDir,
        <String>['--template=module', '--org', 'com.bar.foo'],
        <String>[
          moduleBuildGradleFilePath,
          moduleAppBuildGradleFlePath,
          moduleFlutterBuildGradleFilePath,
        ],
      );

      final String moduleBuildGradleFileContent =
          await globals.fs
              .file(globals.fs.path.join(projectDir.path, moduleBuildGradleFilePath))
              .readAsString();
      final String moduleAppBuildGradleFileContent =
          await globals.fs
              .file(globals.fs.path.join(projectDir.path, moduleAppBuildGradleFlePath))
              .readAsString();
      final String moduleFlutterBuildGradleFileContent =
          await globals.fs
              .file(globals.fs.path.join(projectDir.path, moduleFlutterBuildGradleFilePath))
              .readAsString();

      // Each build file should contain the expected namespace.
      const String expectedNameSpace = 'namespace = "com.bar.foo.flutter_project"';
      expect(moduleBuildGradleFileContent.contains(expectedNameSpace), true);
      expect(moduleFlutterBuildGradleFileContent.contains(expectedNameSpace), true);
      const String expectedHostNameSpace = 'namespace = "com.bar.foo.flutter_project.host"';
      expect(moduleAppBuildGradleFileContent.contains(expectedHostNameSpace), true);
    },
    overrides: <Type, Generator>{
      Pub:
          () => Pub.test(
            fileSystem: globals.fs,
            logger: globals.logger,
            processManager: globals.processManager,
            botDetector: globals.botDetector,
            platform: globals.platform,
            stdio: mockStdio,
          ),
    },
  );

  testUsingContext(
    'Linux plugins handle partially camel-case project names correctly',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      const String projectName = 'foo_BarBaz';
      final Directory projectDir = tempDir.childDirectory(projectName);
      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=linux',
        '--skip-name-checks',
        projectDir.path,
      ]);
      final Directory platformDir = projectDir.childDirectory('linux');

      const String classFilenameBase = 'foo_bar_baz_plugin';
      const String headerName = '$classFilenameBase.h';
      final File headerFile = platformDir
          .childDirectory('include')
          .childDirectory(projectName)
          .childFile(headerName);
      final File implFile = platformDir.childFile('$classFilenameBase.cc');
      // Ensure that the files have the right names.
      expect(headerFile, exists);
      expect(implFile, exists);
      // Ensure that the include is correct.
      expect(implFile.readAsStringSync(), contains(headerName));
      // Ensure that the CMake file has the right target and source values.
      final String cmakeContents = platformDir.childFile('CMakeLists.txt').readAsStringSync();
      expect(cmakeContents, contains('"$classFilenameBase.cc"'));
      expect(cmakeContents, contains('set(PLUGIN_NAME "foo_BarBaz_plugin")'));
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(isLinuxEnabled: true)},
  );

  testUsingContext(
    'Windows plugins handle partially camel-case project names correctly',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      const String projectName = 'foo_BarBaz';
      final Directory projectDir = tempDir.childDirectory(projectName);
      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=windows',
        '--skip-name-checks',
        projectDir.path,
      ]);
      final Directory platformDir = projectDir.childDirectory('windows');

      const String classFilenameBase = 'foo_bar_baz_plugin';
      const String cApiHeaderName = '${classFilenameBase}_c_api.h';
      const String pluginClassHeaderName = '$classFilenameBase.h';
      final File cApiHeaderFile = platformDir
          .childDirectory('include')
          .childDirectory(projectName)
          .childFile(cApiHeaderName);
      final File cApiImplFile = platformDir.childFile('${classFilenameBase}_c_api.cpp');
      final File pluginClassHeaderFile = platformDir.childFile(pluginClassHeaderName);
      final File pluginClassImplFile = platformDir.childFile('$classFilenameBase.cpp');
      // Ensure that the files have the right names.
      expect(cApiHeaderFile, exists);
      expect(cApiImplFile, exists);
      expect(pluginClassHeaderFile, exists);
      expect(pluginClassImplFile, exists);
      // Ensure that the includes are correct.
      expect(
        cApiImplFile.readAsLinesSync(),
        containsAllInOrder(<Matcher>[
          contains('#include "include/$projectName/$cApiHeaderName"'),
          contains('#include "$pluginClassHeaderName"'),
        ]),
      );
      expect(pluginClassImplFile.readAsLinesSync(), contains('#include "$pluginClassHeaderName"'));
      // Ensure that the plugin target name matches the post-processed version.
      // Ensure that the CMake file has the right target and source values.
      final String cmakeContents = platformDir.childFile('CMakeLists.txt').readAsStringSync();
      expect(cmakeContents, contains('"$classFilenameBase.cpp"'));
      expect(cmakeContents, contains('"$classFilenameBase.h"'));
      expect(cmakeContents, contains('"${classFilenameBase}_c_api.cpp"'));
      expect(cmakeContents, contains('"include/$projectName/${classFilenameBase}_c_api.h"'));
      expect(cmakeContents, contains('set(PLUGIN_NAME "foo_BarBaz_plugin")'));
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true)},
  );

  testUsingContext(
    'Linux plugins handle project names ending in _plugin correctly',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      const String projectName = 'foo_bar_plugin';
      final Directory projectDir = tempDir.childDirectory(projectName);
      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=linux',
        projectDir.path,
      ]);
      final Directory platformDir = projectDir.childDirectory('linux');

      // If the project already ends in _plugin, it shouldn't be added again.
      const String classFilenameBase = projectName;
      const String headerName = '$classFilenameBase.h';
      final File headerFile = platformDir
          .childDirectory('include')
          .childDirectory(projectName)
          .childFile(headerName);
      final File implFile = platformDir.childFile('$classFilenameBase.cc');
      // Ensure that the files have the right names.
      expect(headerFile, exists);
      expect(implFile, exists);
      // Ensure that the include is correct.
      expect(implFile.readAsStringSync(), contains(headerName));
      // Ensure that the CMake file has the right target and source values.
      final String cmakeContents = platformDir.childFile('CMakeLists.txt').readAsStringSync();
      expect(cmakeContents, contains('"$classFilenameBase.cc"'));
      // The "_plugin_plugin" suffix is intentional; because the target names must
      // be unique across the ecosystem, no canonicalization can be done,
      // otherwise plugins called "foo_bar" and "foo_bar_plugin" would collide in
      // builds.
      expect(cmakeContents, contains('set(PLUGIN_NAME "foo_bar_plugin_plugin")'));
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(isLinuxEnabled: true)},
  );

  testUsingContext(
    'Windows plugins handle project names ending in _plugin correctly',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      const String projectName = 'foo_bar_plugin';
      final Directory projectDir = tempDir.childDirectory(projectName);
      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=windows',
        projectDir.path,
      ]);
      final Directory platformDir = projectDir.childDirectory('windows');

      // If the project already ends in _plugin, it shouldn't be added again.
      const String classFilenameBase = projectName;
      const String cApiHeaderName = '${classFilenameBase}_c_api.h';
      const String pluginClassHeaderName = '$classFilenameBase.h';
      final File cApiHeaderFile = platformDir
          .childDirectory('include')
          .childDirectory(projectName)
          .childFile(cApiHeaderName);
      final File cApiImplFile = platformDir.childFile('${classFilenameBase}_c_api.cpp');
      final File pluginClassHeaderFile = platformDir.childFile(pluginClassHeaderName);
      final File pluginClassImplFile = platformDir.childFile('$classFilenameBase.cpp');
      // Ensure that the files have the right names.
      expect(cApiHeaderFile, exists);
      expect(cApiImplFile, exists);
      expect(pluginClassHeaderFile, exists);
      expect(pluginClassImplFile, exists);
      // Ensure that the includes are correct.
      expect(
        cApiImplFile.readAsLinesSync(),
        containsAllInOrder(<Matcher>[
          contains('#include "include/$projectName/$cApiHeaderName"'),
          contains('#include "$pluginClassHeaderName"'),
        ]),
      );
      expect(pluginClassImplFile.readAsLinesSync(), contains('#include "$pluginClassHeaderName"'));
      // Ensure that the CMake file has the right target and source values.
      final String cmakeContents = platformDir.childFile('CMakeLists.txt').readAsStringSync();
      expect(cmakeContents, contains('"$classFilenameBase.cpp"'));
      // The "_plugin_plugin" suffix is intentional; because the target names must
      // be unique across the ecosystem, no canonicalization can be done,
      // otherwise plugins called "foo_bar" and "foo_bar_plugin" would collide in
      // builds.
      expect(cmakeContents, contains('set(PLUGIN_NAME "foo_bar_plugin_plugin")'));
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true)},
  );

  testUsingContext(
    'created plugin supports no platforms should print `no platforms` message',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>['create', '--no-pub', '--template=plugin', projectDir.path]);
      expect(logger.errorText, contains(_kNoPlatformsMessage));
      expect(
        logger.statusText,
        contains(
          'To add platforms, run `flutter create -t plugin --platforms <platforms> .` under ${globals.fs.path.normalize(globals.fs.path.relative(projectDir.path))}.',
        ),
      );
      expect(
        logger.statusText,
        contains('For more information, see https://flutter.dev/to/pubspec-plugin-platforms.'),
      );
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(), Logger: () => logger},
  );

  testUsingContext(
    'created FFI plugin supports no platforms should print `no platforms` message',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>['create', '--no-pub', '--template=plugin_ffi', projectDir.path]);
      expect(logger.errorText, contains(_kNoPlatformsMessage));
      expect(
        logger.statusText,
        contains(
          'To add platforms, run `flutter create -t plugin_ffi --platforms <platforms> .` under ${globals.fs.path.normalize(globals.fs.path.relative(projectDir.path))}.',
        ),
      );
      expect(
        logger.statusText,
        contains('For more information, see https://flutter.dev/to/pubspec-plugin-platforms.'),
      );
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(), Logger: () => logger},
  );

  testUsingContext(
    'created plugin with no --platforms flag should not print `no platforms` message if the existing plugin supports a platform.',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=ios',
        projectDir.path,
      ]);
      await runner.run(<String>['create', '--no-pub', '--template=plugin', projectDir.path]);
      expect(logger.errorText, isNot(contains(_kNoPlatformsMessage)));
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(), Logger: () => logger},
  );

  testUsingContext(
    'should show warning when disabled platforms are selected while creating a plugin',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platforms=android,ios,web,windows,macos,linux',
        projectDir.path,
      ]);
      await runner.run(<String>['create', '--no-pub', '--template=plugin', projectDir.path]);
      expect(logger.statusText, contains(_kDisabledPlatformRequestedMessage));
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(), Logger: () => logger},
  );

  testUsingContext(
    "shouldn't show warning when only enabled platforms are selected while creating a plugin",
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platforms=android,ios,windows',
        projectDir.path,
      ]);
      await runner.run(<String>['create', '--no-pub', '--template=plugin', projectDir.path]);
      expect(logger.statusText, isNot(contains(_kDisabledPlatformRequestedMessage)));
    },
    overrides: <Type, Generator>{
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
      Logger: () => logger,
    },
  );

  testUsingContext(
    'should show warning when disabled platforms are selected while creating a app',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>[
        'create',
        '--no-pub',
        '--platforms=android,ios,web,windows,macos,linux',
        projectDir.path,
      ]);
      await runner.run(<String>['create', '--no-pub', projectDir.path]);
      expect(logger.statusText, contains(_kDisabledPlatformRequestedMessage));
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(), Logger: () => logger},
  );

  testUsingContext(
    "shouldn't show warning when only enabled platforms are selected while creating a app",
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin',
        '--platform=windows',
        projectDir.path,
      ]);
      await runner.run(<String>['create', '--no-pub', '--template=plugin', projectDir.path]);
      expect(logger.statusText, isNot(contains(_kDisabledPlatformRequestedMessage)));
    },
    overrides: <Type, Generator>{
      FeatureFlags:
          () => TestFeatureFlags(
            isWindowsEnabled: true,
            isAndroidEnabled: false,
            isIOSEnabled: false,
          ),
      Logger: () => logger,
    },
  );

  testUsingContext(
    'default project has analysis_options.yaml set up correctly',
    () async {
      await _createProject(projectDir, <String>[], <String>['analysis_options.yaml']);
      final String dataPath = globals.fs.path.join(
        getFlutterRoot(),
        'packages',
        'flutter_tools',
        'test',
        'commands.shard',
        'permeable',
        'data',
      );
      final File toAnalyze = await globals.fs
          .file(globals.fs.path.join(dataPath, 'to_analyze.dart.test'))
          .copy(globals.fs.path.join(projectDir.path, 'lib', 'to_analyze.dart'));
      final String relativePath = globals.fs.path.join('lib', 'to_analyze.dart');
      final List<String> expectedFailures = <String>[
        '$relativePath:11:7: use_key_in_widget_constructors',
        '$relativePath:20:3: prefer_const_constructors_in_immutables',
        '$relativePath:31:26: use_full_hex_values_for_flutter_colors',
      ];
      expect(expectedFailures.length, '// LINT:'.allMatches(toAnalyze.readAsStringSync()).length);
      await analyzeProject(projectDir.path, expectedFailures: expectedFailures);
    },
    overrides: <Type, Generator>{
      Pub:
          () => Pub.test(
            fileSystem: globals.fs,
            logger: globals.logger,
            processManager: globals.processManager,
            botDetector: globals.botDetector,
            platform: globals.platform,
            stdio: mockStdio,
          ),
    },
  );

  testUsingContext('should escape ":" in project description', () async {
    await _createProject(
      projectDir,
      <String>['--no-pub', '--description', 'a: b'],
      <String>['pubspec.yaml'],
    );

    final String rawPubspec = await projectDir.childFile('pubspec.yaml').readAsString();
    final Pubspec pubspec = Pubspec.parse(rawPubspec);
    expect(pubspec.description, 'a: b');
  });

  testUsingContext('should use caret syntax in SDK version', () async {
    await _createProject(projectDir, <String>['--no-pub'], <String>['pubspec.yaml']);

    final String rawPubspec = await projectDir.childFile('pubspec.yaml').readAsString();
    final Pubspec pubspec = Pubspec.parse(rawPubspec);

    expect(
      pubspec.environment['sdk'].toString(),
      startsWith('^'),
      reason: 'The caret syntax is recommended over the traditional syntax.',
    );
  });

  testUsingContext(
    'show an error message for removed --template=skeleton',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      await expectLater(
        runner.run(<String>['create', '--no-pub', '--template=skeleton', projectDir.path]),
        throwsToolExit(message: 'The template skeleton is no longer available'),
      );
    },
    overrides: <Type, Generator>{
      Pub:
          () => Pub.test(
            fileSystem: globals.fs,
            logger: globals.logger,
            processManager: globals.processManager,
            botDetector: globals.botDetector,
            platform: globals.platform,
            stdio: mockStdio,
          ),
    },
  );

  testUsingContext(
    'create an FFI plugin with ios, then add macos',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin_ffi',
        '--platform=ios',
        projectDir.path,
      ]);
      expect(projectDir.childDirectory('src'), exists);
      expect(projectDir.childDirectory('ios'), exists);
      expect(projectDir.childDirectory('example').childDirectory('ios'), exists);
      validatePubspecForPlugin(
        projectDir: projectDir.absolute.path,
        expectedPlatforms: const <String>['ios'],
        ffiPlugin: true,
        unexpectedPlatforms: <String>['some_platform'],
      );

      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin_ffi',
        '--platform=macos',
        projectDir.path,
      ]);
      expect(projectDir.childDirectory('macos'), exists);
      expect(projectDir.childDirectory('example').childDirectory('macos'), exists);
      expect(projectDir.childDirectory('ios'), exists);
      expect(projectDir.childDirectory('example').childDirectory('ios'), exists);
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(isMacOSEnabled: true)},
  );

  for (final String template in <String>['package_ffi', 'plugin_ffi']) {
    testUsingContext(
      '$template error android language',
      () async {
        final CreateCommand command = CreateCommand();
        final CommandRunner<void> runner = createTestCommandRunner(command);
        final List<String> args = <String>[
          'create',
          '--no-pub',
          '--template=$template',
          '-a',
          'kotlin',
          if (template == 'plugin_ffi') '--platforms=android',
          projectDir.path,
        ];

        await expectLater(
          runner.run(args),
          throwsToolExit(
            message:
                'The "android-language" option is not supported with the $template template: the language will always be C or C++.',
          ),
        );
      },
      overrides: <Type, Generator>{
        FeatureFlags: () => TestFeatureFlags(isNativeAssetsEnabled: true),
      },
    );

    testUsingContext(
      '$template error ios language',
      () async {
        final CreateCommand command = CreateCommand();
        final CommandRunner<void> runner = createTestCommandRunner(command);
        final List<String> args = <String>[
          'create',
          '--no-pub',
          '--template=$template',
          '--ios-language',
          'swift',
          if (template == 'plugin_ffi') '--platforms=ios',
          projectDir.path,
        ];

        await expectLater(
          runner.run(args),
          throwsToolExit(
            message:
                'The "ios-language" option is not supported with the $template template: the language will always be C or C++.',
          ),
        );
      },
      overrides: <Type, Generator>{
        FeatureFlags: () => TestFeatureFlags(isNativeAssetsEnabled: true),
      },
    );
  }

  testUsingContext('FFI plugins error web platform', () async {
    final CreateCommand command = CreateCommand();
    final CommandRunner<void> runner = createTestCommandRunner(command);
    final List<String> args = <String>[
      'create',
      '--no-pub',
      '--template=plugin_ffi',
      '--platforms=web',
      projectDir.path,
    ];

    await expectLater(
      runner.run(args),
      throwsToolExit(message: 'The web platform is not supported in plugin_ffi template.'),
    );
  });

  testUsingContext(
    'should show warning when disabled platforms are selected while creating an FFI plugin',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>[
        'create',
        '--no-pub',
        '--template=plugin_ffi',
        '--platforms=android,ios,windows,macos,linux',
        projectDir.path,
      ]);
      await runner.run(<String>['create', '--no-pub', '--template=plugin_ffi', projectDir.path]);
      expect(logger.statusText, contains(_kDisabledPlatformRequestedMessage));
    },
    overrides: <Type, Generator>{FeatureFlags: () => TestFeatureFlags(), Logger: () => logger},
  );

  testUsingContext(
    'should not show warning for incompatible Java/template Gradle versions when Java version not found',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>['create', '--no-pub', '--platforms=android', projectDir.path]);

      expect(logger.warningText, isNot(contains(_kIncompatibleJavaVersionMessage)));
    },
    overrides: <Type, Generator>{Java: () => null, Logger: () => logger},
  );

  testUsingContext('should return correct warning for incompatible Gradle versions', () async {
    const String projectType = 'app';
    final String gradleConflict = getIncompatibleJavaGradleAgpMessageHeader(
      false,
      templateDefaultGradleVersion,
      templateAndroidGradlePluginVersion,
      projectType,
    );

    expect(
      gradleConflict,
      contains('''
The configured version of Java detected may conflict with the Gradle version in your new Flutter $projectType.

To keep the default Gradle version $templateDefaultGradleVersion, download a compatible Java version
'''),
    );
  });

  testUsingContext('should return correct warning for incompatible AGP versions', () async {
    const String projectType = 'app';
    final String agpConflict = getIncompatibleJavaGradleAgpMessageHeader(
      true,
      templateDefaultGradleVersion,
      templateAndroidGradlePluginVersion,
      projectType,
    );

    expect(
      agpConflict,
      contains('''
The configured version of Java detected may conflict with the Android Gradle Plugin (AGP) version in your new Flutter $projectType.

To keep the default AGP version $templateAndroidGradlePluginVersion, download a compatible Java version
'''),
    );
  });

  testUsingContext(
    'should not show warning for incompatible Java/template Gradle versions when created project type is irrelevant',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      // Test not creating a project for Android.
      await runner.run(<String>[
        'create',
        '--no-pub',
        '--platforms=ios,windows,macos,linux',
        projectDir.path,
      ]);
      tryToDelete(projectDir);
      // Test creating a package (Dart-only code).
      await runner.run(<String>['create', '--no-pub', '--template=package', projectDir.path]);
      tryToDelete(projectDir);
      // Test creating project types without configured Gradle versions.
      await runner.run(<String>['create', '--no-pub', '--template=plugin', projectDir.path]);
      tryToDelete(projectDir);
      await runner.run(<String>['create', '--no-pub', '--template=plugin_ffi', projectDir.path]);

      expect(
        logger.warningText,
        isNot(
          contains(
            getIncompatibleJavaGradleAgpMessageHeader(
              false,
              templateDefaultGradleVersion,
              templateAndroidGradlePluginVersion,
              'app',
            ),
          ),
        ),
      );
      expect(
        logger.warningText,
        isNot(
          contains(
            getIncompatibleJavaGradleAgpMessageHeader(
              false,
              templateDefaultGradleVersion,
              templateAndroidGradlePluginVersion,
              'package',
            ),
          ),
        ),
      );
      expect(
        logger.warningText,
        isNot(
          contains(
            getIncompatibleJavaGradleAgpMessageHeader(
              false,
              templateDefaultGradleVersion,
              templateAndroidGradlePluginVersion,
              'plugin',
            ),
          ),
        ),
      );
      expect(
        logger.warningText,
        isNot(
          contains(
            getIncompatibleJavaGradleAgpMessageHeader(
              false,
              templateDefaultGradleVersion,
              templateAndroidGradlePluginVersion,
              'pluginFfi',
            ),
          ),
        ),
      );
    },
    overrides: <Type, Generator>{
      Java:
          () => FakeJava(
            version: const software.Version.withText(1000, 0, 0, '1000.0.0'),
          ), // Too high a version for template Gradle versions.
      Logger: () => logger,
    },
  );

  testUsingContext(
    'should not show warning for incompatible Java/template AGP versions when project type unrelated',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      // Test not creating a project for Android.
      await runner.run(<String>[
        'create',
        '--no-pub',
        '--platforms=ios,windows,macos,linux',
        projectDir.path,
      ]);
      tryToDelete(projectDir);
      // Test creating a package (Dart-only code).
      await runner.run(<String>['create', '--no-pub', '--template=package', projectDir.path]);

      expect(
        logger.warningText,
        isNot(
          contains(
            getIncompatibleJavaGradleAgpMessageHeader(
              false,
              templateDefaultGradleVersion,
              templateAndroidGradlePluginVersion,
              'app',
            ),
          ),
        ),
      );
      expect(
        logger.warningText,
        isNot(
          contains(
            getIncompatibleJavaGradleAgpMessageHeader(
              false,
              templateDefaultGradleVersion,
              templateAndroidGradlePluginVersion,
              'package',
            ),
          ),
        ),
      );
    },
    overrides: <Type, Generator>{
      Java:
          () => FakeJava(
            version: const software.Version.withText(0, 0, 0, '0.0.0'),
          ), // Too low a version for template AGP versions.
      Logger: () => logger,
    },
  );

  testUsingContext(
    'should show warning for incompatible Java/template Gradle versions when detected',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      final List<FlutterTemplateType> relevantProjectTypes = <FlutterTemplateType>[
        FlutterTemplateType.app,
        FlutterTemplateType.module,
      ];

      for (final FlutterTemplateType projectType in relevantProjectTypes) {
        final String relevantAgpVersion =
            projectType == FlutterTemplateType.module
                ? _kIncompatibleAgpVersionForModule
                : templateAndroidGradlePluginVersion;
        final String expectedMessage = getIncompatibleJavaGradleAgpMessageHeader(
          false,
          templateDefaultGradleVersion,
          relevantAgpVersion,
          projectType.cliName,
        );
        final String unexpectedMessage = getIncompatibleJavaGradleAgpMessageHeader(
          true,
          templateDefaultGradleVersion,
          relevantAgpVersion,
          projectType.cliName,
        );

        await runner.run(<String>[
          'create',
          '--no-pub',
          '--template=${projectType.cliName}',
          if (projectType != FlutterTemplateType.module) '--platforms=android',
          projectDir.path,
        ]);

        // Check components of expected header warning message are printed.
        expect(logger.warningText, contains(expectedMessage));
        expect(logger.warningText, isNot(contains(unexpectedMessage)));
        expect(
          logger.warningText,
          contains('./gradlew wrapper --gradle-version=<COMPATIBLE_GRADLE_VERSION>'),
        );
        expect(
          logger.warningText,
          contains('https://docs.gradle.org/current/userguide/compatibility.html#java'),
        );

        // Check expected file for updating Gradle version is present.
        if (projectType == FlutterTemplateType.app) {
          expect(
            logger.warningText,
            contains(
              globals.fs.path.join(
                projectDir.path,
                'android/gradle/wrapper/gradle-wrapper.properties',
              ),
            ),
          );
        } else {
          // Project type is module.
          expect(
            logger.warningText,
            contains(
              globals.fs.path.join(
                projectDir.path,
                '.android/gradle/wrapper/gradle-wrapper.properties',
              ),
            ),
          );
        }

        // Cleanup to reuse projectDir and logger checks.
        tryToDelete(projectDir);
        logger.clear();
      }
    },
    overrides: <Type, Generator>{
      Java:
          () => FakeJava(
            version: const software.Version.withText(500, 0, 0, '500.0.0'),
          ), // Too high a version for template Gradle versions.
      Logger: () => logger,
    },
  );

  testUsingContext(
    'should show warning for incompatible Java/template AGP versions when detected',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      final List<FlutterTemplateType> relevantProjectTypes = <FlutterTemplateType>[
        FlutterTemplateType.app,
        FlutterTemplateType.pluginFfi,
        FlutterTemplateType.module,
        FlutterTemplateType.plugin,
      ];

      for (final FlutterTemplateType projectType in relevantProjectTypes) {
        final String relevantAgpVersion =
            projectType == FlutterTemplateType.module
                ? _kIncompatibleAgpVersionForModule
                : templateAndroidGradlePluginVersion;
        final String expectedMessage = getIncompatibleJavaGradleAgpMessageHeader(
          true,
          templateDefaultGradleVersion,
          relevantAgpVersion,
          projectType.cliName,
        );
        final String unexpectedMessage = getIncompatibleJavaGradleAgpMessageHeader(
          false,
          templateDefaultGradleVersion,
          relevantAgpVersion,
          projectType.cliName,
        );

        await runner.run(<String>[
          'create',
          '--no-pub',
          '--template=${projectType.cliName}',
          if (projectType != FlutterTemplateType.module) '--platforms=android',
          projectDir.path,
        ]);

        // Check components of expected header warning message are printed.
        expect(logger.warningText, contains(expectedMessage));
        expect(logger.warningText, isNot(contains(unexpectedMessage)));
        expect(
          logger.warningText,
          contains('https://developer.android.com/build/releases/gradle-plugin'),
        );

        // Check expected file(s) for updating AGP version is/are present.
        if (projectType == FlutterTemplateType.app ||
            projectType == FlutterTemplateType.pluginFfi) {
          expect(
            logger.warningText,
            contains(globals.fs.path.join(projectDir.path, 'android/build.gradle')),
          );
        } else if (projectType == FlutterTemplateType.plugin) {
          expect(
            logger.warningText,
            contains(globals.fs.path.join(projectDir.path, 'android/app/build.gradle')),
          );
        } else {
          // Project type is module.
          expect(
            logger.warningText,
            contains(globals.fs.path.join(projectDir.path, '.android/build.gradle')),
          );
          expect(
            logger.warningText,
            contains(globals.fs.path.join(projectDir.path, '.android/app/build.gradle')),
          );
          expect(
            logger.warningText,
            contains(globals.fs.path.join(projectDir.path, '.android/Flutter/build.gradle')),
          );
        }

        // Cleanup to reuse projectDir and logger checks.
        tryToDelete(projectDir);
        logger.clear();
      }
    },
    overrides: <Type, Generator>{
      Java:
          () => FakeJava(
            version: const software.Version.withText(1, 8, 0, '1.8.0'),
          ), // Too low a version for template AGP versions.
      Logger: () => logger,
    },
  );

  // The Java versions configured in the following tests will need updates as more Java versions are supported by AGP/Gradle:

  testUsingContext(
    'should not show warning for incompatible Java/template AGP/Gradle versions when not detected',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      final List<FlutterTemplateType> relevantProjectTypes = <FlutterTemplateType>[
        FlutterTemplateType.app,
        FlutterTemplateType.pluginFfi,
        FlutterTemplateType.module,
        FlutterTemplateType.plugin,
      ];

      for (final FlutterTemplateType projectType in relevantProjectTypes) {
        final String relevantAgpVersion =
            projectType == FlutterTemplateType.module
                ? _kIncompatibleAgpVersionForModule
                : templateAndroidGradlePluginVersion;
        final String unexpectedIncompatibleAgpMessage = getIncompatibleJavaGradleAgpMessageHeader(
          true,
          templateDefaultGradleVersion,
          relevantAgpVersion,
          projectType.cliName,
        );
        final String unexpectedIncompatibleGradleMessage =
            getIncompatibleJavaGradleAgpMessageHeader(
              false,
              templateDefaultGradleVersion,
              relevantAgpVersion,
              projectType.cliName,
            );

        await runner.run(<String>[
          'create',
          '--no-pub',
          '--template=${projectType.cliName}',
          if (projectType != FlutterTemplateType.module) '--platforms=android',
          projectDir.path,
        ]);

        // We do not expect warnings for incompatible Java/template AGP versions if they are in fact, compatible.
        expect(logger.warningText, isNot(contains(unexpectedIncompatibleAgpMessage)));
        expect(logger.warningText, isNot(contains(unexpectedIncompatibleGradleMessage)));

        // Cleanup to reuse projectDir and logger checks.
        tryToDelete(projectDir);
        logger.clear();
      }
    },
    overrides: <Type, Generator>{
      Java:
          () => FakeJava(
            version: const software.Version.withText(20, 0, 0, '20.0.0'),
          ), // Middle compatible Java version with current template AGP/Gradle versions.
      Logger: () => logger,
    },
  );

  testUsingContext(
    'should not show warning for incompatible Java/template AGP/Gradle versions when not detected -- maximum compatible Java version',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      final List<FlutterTemplateType> relevantProjectTypes = <FlutterTemplateType>[
        FlutterTemplateType.app,
        FlutterTemplateType.pluginFfi,
        FlutterTemplateType.module,
        FlutterTemplateType.plugin,
      ];

      for (final FlutterTemplateType projectType in relevantProjectTypes) {
        final String relevantAgpVersion =
            projectType == FlutterTemplateType.module
                ? _kIncompatibleAgpVersionForModule
                : templateAndroidGradlePluginVersion;
        final String unexpectedIncompatibleAgpMessage = getIncompatibleJavaGradleAgpMessageHeader(
          true,
          templateDefaultGradleVersion,
          relevantAgpVersion,
          projectType.cliName,
        );
        final String unexpectedIncompatibleGradleMessage =
            getIncompatibleJavaGradleAgpMessageHeader(
              false,
              templateDefaultGradleVersion,
              relevantAgpVersion,
              projectType.cliName,
            );

        await runner.run(<String>[
          'create',
          '--no-pub',
          '--template=${projectType.cliName}',
          if (projectType != FlutterTemplateType.module) '--platforms=android',
          projectDir.path,
        ]);

        // We do not expect warnings for incompatible Java/template AGP versions if they are in fact, compatible.
        expect(logger.warningText, isNot(contains(unexpectedIncompatibleAgpMessage)));
        expect(logger.warningText, isNot(contains(unexpectedIncompatibleGradleMessage)));

        // Cleanup to reuse projectDir and logger checks.
        tryToDelete(projectDir);
        logger.clear();
      }
    },
    overrides: <Type, Generator>{
      Java:
          () => FakeJava(
            version: const software.Version.withText(17, 0, 0, '22.0.0'),
          ), // Maximum compatible Java version with current template AGP/Gradle versions.
      Logger: () => logger,
    },
  );

  testUsingContext(
    'should not show warning for incompatible Java/template AGP/Gradle versions when not detected -- minimum compatible Java version',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);
      final List<FlutterTemplateType> relevantProjectTypes = <FlutterTemplateType>[
        FlutterTemplateType.app,
        FlutterTemplateType.pluginFfi,
        FlutterTemplateType.module,
        FlutterTemplateType.plugin,
      ];

      for (final FlutterTemplateType projectType in relevantProjectTypes) {
        final String relevantAgpVersion =
            projectType == FlutterTemplateType.module
                ? _kIncompatibleAgpVersionForModule
                : templateAndroidGradlePluginVersion;
        final String unexpectedIncompatibleAgpMessage = getIncompatibleJavaGradleAgpMessageHeader(
          true,
          templateDefaultGradleVersion,
          relevantAgpVersion,
          projectType.cliName,
        );
        final String unexpectedIncompatibleGradleMessage =
            getIncompatibleJavaGradleAgpMessageHeader(
              false,
              templateDefaultGradleVersion,
              relevantAgpVersion,
              projectType.cliName,
            );

        await runner.run(<String>[
          'create',
          '--no-pub',
          '--template=${projectType.cliName}',
          if (projectType != FlutterTemplateType.module) '--platforms=android',
          projectDir.path,
        ]);

        // We do not expect warnings for incompatible Java/template AGP versions if they are in fact, compatible.
        expect(logger.warningText, isNot(contains(unexpectedIncompatibleAgpMessage)));
        expect(logger.warningText, isNot(contains(unexpectedIncompatibleGradleMessage)));

        // Cleanup to reuse projectDir and logger checks.
        tryToDelete(projectDir);
        logger.clear();
      }
    },
    overrides: <Type, Generator>{
      Java:
          () => FakeJava(
            version: const software.Version.withText(17, 0, 0, '17.0.0'),
          ), // Minimum compatible Java version with current template AGP/Gradle versions.
      Logger: () => logger,
    },
  );

  testUsingContext('Does not double quote description in index.html on web', () async {
    await _createProject(
      projectDir,
      <String>['--no-pub', '--platforms=web'],
      <String>['pubspec.yaml', 'web/index.html'],
    );

    final String rawIndexHtml =
        await projectDir.childDirectory('web').childFile('index.html').readAsString();
    const String expectedDescription = '<meta name="description" content="A new Flutter project.">';

    expect(rawIndexHtml.contains(expectedDescription), isTrue);
  });

  testUsingContext('Does not double quote description in manifest.json on web', () async {
    await _createProject(
      projectDir,
      <String>['--no-pub', '--platforms=web'],
      <String>['pubspec.yaml', 'web/manifest.json'],
    );

    final String rawManifestJson =
        await projectDir.childDirectory('web').childFile('manifest.json').readAsString();
    const String expectedDescription = '"description": "A new Flutter project."';

    expect(rawManifestJson.contains(expectedDescription), isTrue);
  });

  testUsingContext(
    'flutter create should tool exit if the template manifest cannot be read',
    () async {
      globals.fs
          .file(
            globals.fs.path.join(
              Cache.flutterRoot!,
              'packages',
              'flutter_tools',
              'templates',
              'template_manifest.json',
            ),
          )
          .createSync(recursive: true);

      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await expectLater(
        runner.run(<String>[
          'create',
          '--no-pub',
          '--template=plugin',
          '--project-name=test',
          'dev/test',
        ]),
        throwsToolExit(message: 'Unable to read the template manifest at path'),
      );
    },
    overrides: <Type, Generator>{
      FileSystem:
          () => MemoryFileSystem.test(
            opHandle: (String context, FileSystemOp operation) {
              if (operation == FileSystemOp.read && context.contains('template_manifest.json')) {
                throw io.PathNotFoundException(context, const OSError(), 'Cannot open file');
              }
            },
          ),
      ProcessManager: () => fakeProcessManager,
    },
  );

  testUsingContext(
    'flutter create should show the incompatible java AGP message',
    () async {
      final CreateCommand command = CreateCommand();
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>['create', '--no-pub', '--platforms=android', projectDir.path]);

      final String expectedMessage = getIncompatibleJavaGradleAgpMessageHeader(
        false,
        templateDefaultGradleVersion,
        templateAndroidGradlePluginVersion,
        'app',
      );

      expect(logger.warningText, contains(expectedMessage));
    },
    overrides: <Type, Generator>{
      Java:
          () => FakeJava(
            version: const software.Version.withText(500, 0, 0, '500.0.0'),
          ), // Too high a version for template Gradle versions.
      Logger: () => logger,
    },
  );
}

Future<void> _createProject(
  Directory dir,
  List<String> createArgs,
  List<String> expectedPaths, {
  List<String> unexpectedPaths = const <String>[],
  List<String> expectedGitignoreLines = const <String>[],
}) async {
  final CreateCommand command = CreateCommand();
  final CommandRunner<void> runner = createTestCommandRunner(command);
  await runner.run(<String>['create', ...createArgs, dir.path]);

  bool pathExists(String path) {
    final String fullPath = globals.fs.path.join(dir.path, path);
    return globals.fs.typeSync(fullPath) != FileSystemEntityType.notFound;
  }

  final List<String> pathFailures = <String>[
    for (final String path in expectedPaths)
      if (!pathExists(path)) 'Path "$path" does not exist.',
    for (final String path in unexpectedPaths)
      if (pathExists(path)) 'Path "$path" exists when it shouldn\'t.',
  ];
  expect(pathFailures, isEmpty, reason: pathFailures.join('\n'));

  final String gitignorePath = globals.fs.path.join(dir.path, '.gitignore');
  final List<String> gitignore = globals.fs.file(gitignorePath).readAsLinesSync();

  final List<String> gitignoreFailures = <String>[
    for (final String line in expectedGitignoreLines)
      if (!gitignore.contains(line)) 'Expected .gitignore to contain "$line".',
  ];
  expect(gitignoreFailures, isEmpty, reason: gitignoreFailures.join('\n'));
}

Future<void> _createAndAnalyzeProject(
  Directory dir,
  List<String> createArgs,
  List<String> expectedPaths, {
  List<String> unexpectedPaths = const <String>[],
  List<String> expectedGitignoreLines = const <String>[],
}) async {
  await _createProject(
    dir,
    createArgs,
    expectedPaths,
    unexpectedPaths: unexpectedPaths,
    expectedGitignoreLines: expectedGitignoreLines,
  );
  await analyzeProject(dir.path);
}

Future<void> _getPackages(Directory workingDir) async {
  final String flutterToolsSnapshotPath = globals.fs.path.absolute(
    globals.fs.path.join('..', '..', 'bin', 'cache', 'flutter_tools.snapshot'),
  );

  // While flutter test does get packages, it doesn't write version
  // files anymore.
  await Process.run(globals.artifacts!.getArtifactPath(Artifact.engineDartBinary), <String>[
    flutterToolsSnapshotPath,
    'packages',
    'get',
  ], workingDirectory: workingDir.path);
}

Future<void> _runFlutterTest(Directory workingDir, {String? target}) async {
  final String flutterToolsSnapshotPath = globals.fs.path.absolute(
    globals.fs.path.join('..', '..', 'bin', 'cache', 'flutter_tools.snapshot'),
  );

  await _getPackages(workingDir);

  final List<String> args = <String>[
    flutterToolsSnapshotPath,
    'test',
    '--no-color',
    if (target != null) target,
  ];

  final ProcessResult exec = await Process.run(
    globals.artifacts!.getArtifactPath(Artifact.engineDartBinary),
    args,
    workingDirectory: workingDir.path,
  );
  printOnFailure('Output of running flutter test:');
  printOnFailure(exec.stdout.toString());
  printOnFailure(exec.stderr.toString());
  expect(exec.exitCode, 0);
}

String _getStringValueFromPlist({required File plistFile, String? key}) {
  final List<String> plist = plistFile.readAsLinesSync().map((String line) => line.trim()).toList();
  final int keyIndex = plist.indexOf('<key>$key</key>');
  assert(keyIndex > 0);
  return plist[keyIndex + 1].replaceAll('<string>', '').replaceAll('</string>', '');
}

bool _getBooleanValueFromPlist({required File plistFile, String? key}) {
  final List<String> plist = plistFile.readAsLinesSync().map((String line) => line.trim()).toList();
  final int keyIndex = plist.indexOf('<key>$key</key>');
  assert(keyIndex > 0);
  return plist[keyIndex + 1].replaceAll('<', '').replaceAll('/>', '') == 'true';
}
