// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/command_runner.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/common.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/process.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/build_system.dart';
import 'package:flutter_tools/src/build_system/targets/web.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/build.dart';
import 'package:flutter_tools/src/commands/build_web.dart';
import 'package:flutter_tools/src/dart/pub.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/runner/flutter_command.dart';
import 'package:flutter_tools/src/web/compile.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_pub_deps.dart';
import '../../src/fakes.dart';
import '../../src/test_build_system.dart';
import '../../src/test_flutter_command_runner.dart';

void main() {
  late FileSystem fileSystem;
  final Platform fakePlatform = FakePlatform(environment: <String, String>{'FLUTTER_ROOT': '/'});
  late ProcessUtils processUtils;
  late BufferLogger logger;
  late ProcessManager processManager;
  late Artifacts artifacts;

  // TODO(matanlurey): Remove after `explicit-package-dependencies` is enabled by default.
  // See https://github.com/flutter/flutter/issues/160257 for details.
  FeatureFlags enableExplicitPackageDependencies() {
    return TestFeatureFlags(
      isExplicitPackageDependenciesEnabled: true,
      // Assumed to be true below.
      isWebEnabled: true,
    );
  }

  setUpAll(() {
    Cache.flutterRoot = '';
    Cache.disableLocking();
  });

  setUp(() {
    fileSystem = MemoryFileSystem.test();
    fileSystem.file('pubspec.yaml')
      ..createSync()
      ..writeAsStringSync('name: foo\n');
    fileSystem.directory('.dart_tool').childFile('package_config.json').createSync(recursive: true);
    fileSystem.file(fileSystem.path.join('web', 'index.html')).createSync(recursive: true);
    fileSystem.file(fileSystem.path.join('lib', 'main.dart')).createSync(recursive: true);
    artifacts = Artifacts.test(fileSystem: fileSystem);
    logger = BufferLogger.test();
    processManager = FakeProcessManager.any();
    processUtils = ProcessUtils(logger: logger, processManager: processManager);
  });

  testUsingContext(
    'Refuses to build for web when missing index.html',
    () async {
      fileSystem.file(fileSystem.path.join('web', 'index.html')).deleteSync();
      final CommandRunner<void> runner = createTestCommandRunner(
        BuildCommand(
          artifacts: artifacts,
          androidSdk: FakeAndroidSdk(),
          buildSystem: TestBuildSystem.all(BuildResult(success: true)),
          fileSystem: fileSystem,
          logger: logger,
          processUtils: processUtils,
          osUtils: FakeOperatingSystemUtils(),
        ),
      );

      expect(
        () => runner.run(<String>['build', 'web', '--no-pub']),
        throwsToolExit(message: 'Missing index.html.'),
      );
    },
    overrides: <Type, Generator>{
      Platform: () => fakePlatform,
      FileSystem: () => fileSystem,
      FeatureFlags: () => TestFeatureFlags(isWebEnabled: true),
      ProcessManager: () => processManager,
    },
  );

  testUsingContext(
    'Refuses to build for web when feature is disabled',
    () async {
      final CommandRunner<void> runner = createTestCommandRunner(
        BuildCommand(
          artifacts: artifacts,
          androidSdk: FakeAndroidSdk(),
          buildSystem: TestBuildSystem.all(BuildResult(success: true)),
          fileSystem: MemoryFileSystem.test(),
          logger: logger,
          processUtils: processUtils,
          osUtils: FakeOperatingSystemUtils(),
        ),
      );

      expect(
        () => runner.run(<String>['build', 'web', '--no-pub']),
        throwsToolExit(
          message:
              '"build web" is not currently supported. To enable, run "flutter config --enable-web".',
        ),
      );
    },
    overrides: <Type, Generator>{
      Platform: () => fakePlatform,
      FileSystem: () => fileSystem,
      FeatureFlags: () => TestFeatureFlags(),
      ProcessManager: () => processManager,
    },
  );

  testUsingContext(
    'Setup for a web build with default output directory',
    () async {
      final BuildCommand buildCommand = BuildCommand(
        artifacts: artifacts,
        androidSdk: FakeAndroidSdk(),
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
        fileSystem: fileSystem,
        logger: logger,
        processUtils: processUtils,
        osUtils: FakeOperatingSystemUtils(),
      );
      final CommandRunner<void> runner = createTestCommandRunner(buildCommand);
      setupFileSystemForEndToEndTest(fileSystem);
      await runner.run(<String>[
        'build',
        'web',
        '--no-pub',
        '--no-web-resources-cdn',
        '--dart-define=foo=a',
        '--dart2js-optimization=O3',
      ]);

      final Directory buildDir = fileSystem.directory(fileSystem.path.join('build', 'web'));

      expect(buildDir.existsSync(), true);
      expect(testLogger.statusText, contains('✓ Built ${buildDir.path}'));
    },
    overrides: <Type, Generator>{
      Platform: () => fakePlatform,
      FileSystem: () => fileSystem,
      FeatureFlags: () => TestFeatureFlags(isWebEnabled: true),
      ProcessManager: () => processManager,
      BuildSystem:
          () => TestBuildSystem.all(BuildResult(success: true), (
            Target target,
            Environment environment,
          ) {
            expect(environment.defines, <String, String>{
              'TargetFile': 'lib/main.dart',
              'HasWebPlugins': 'true',
              'ServiceWorkerStrategy': 'offline-first',
              'BuildMode': 'release',
              'DartDefines': 'Zm9vPWE=',
              'DartObfuscation': 'false',
              'TrackWidgetCreation': 'false',
              'TreeShakeIcons': 'true',
              'UseLocalCanvasKit': 'true',
            });
          }),
    },
  );

  testUsingContext(
    'Does not allow -O0 optimization level',
    () async {
      final BuildCommand buildCommand = BuildCommand(
        artifacts: artifacts,
        androidSdk: FakeAndroidSdk(),
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
        osUtils: FakeOperatingSystemUtils(),
        processUtils: processUtils,
      );
      final CommandRunner<void> runner = createTestCommandRunner(buildCommand);
      setupFileSystemForEndToEndTest(fileSystem);
      await expectLater(
        () => runner.run(<String>[
          'build',
          'web',
          '--no-pub',
          '--no-web-resources-cdn',
          '--dart-define=foo=a',
          '--dart2js-optimization=O0',
        ]),
        throwsUsageException(
          message: '"O0" is not an allowed value for option "--dart2js-optimization"',
        ),
      );

      final Directory buildDir = fileSystem.directory(fileSystem.path.join('build', 'web'));

      expect(buildDir.existsSync(), isFalse);
    },
    overrides: <Type, Generator>{
      Platform: () => fakePlatform,
      FileSystem: () => fileSystem,
      FeatureFlags: () => TestFeatureFlags(isWebEnabled: true),
      ProcessManager: () => processManager,
      BuildSystem:
          () => TestBuildSystem.all(BuildResult(success: true), (
            Target target,
            Environment environment,
          ) {
            expect(environment.defines, <String, String>{
              'TargetFile': 'lib/main.dart',
              'HasWebPlugins': 'true',
              'cspMode': 'false',
              'SourceMaps': 'false',
              'NativeNullAssertions': 'true',
              'ServiceWorkerStrategy': 'offline-first',
              'Dart2jsDumpInfo': 'false',
              'Dart2jsNoFrequencyBasedMinification': 'false',
              'Dart2jsOptimization': 'O3',
              'BuildMode': 'release',
              'DartDefines': 'Zm9vPWE=,RkxVVFRFUl9XRUJfQVVUT19ERVRFQ1Q9dHJ1ZQ==',
              'DartObfuscation': 'false',
              'TrackWidgetCreation': 'false',
              'TreeShakeIcons': 'true',
            });
          }),
    },
  );

  testUsingContext(
    'Setup for a web build with a user specified output directory',
    () async {
      final BuildCommand buildCommand = BuildCommand(
        artifacts: artifacts,
        androidSdk: FakeAndroidSdk(),
        buildSystem: TestBuildSystem.all(BuildResult(success: true)),
        fileSystem: fileSystem,
        logger: logger,
        processUtils: processUtils,
        osUtils: FakeOperatingSystemUtils(),
      );
      final CommandRunner<void> runner = createTestCommandRunner(buildCommand);

      setupFileSystemForEndToEndTest(fileSystem);

      const String newBuildDir = 'new_dir';
      final Directory buildDir = fileSystem.directory(fileSystem.path.join(newBuildDir));

      expect(buildDir.existsSync(), false);

      await runner.run(<String>[
        'build',
        'web',
        '--no-pub',
        '--no-web-resources-cdn',
        '--output=$newBuildDir',
      ]);

      expect(buildDir.existsSync(), true);
      expect(testLogger.statusText, contains('✓ Built $newBuildDir'));
    },
    overrides: <Type, Generator>{
      Platform: () => fakePlatform,
      FileSystem: () => fileSystem,
      FeatureFlags: () => TestFeatureFlags(isWebEnabled: true),
      ProcessManager: () => processManager,
      BuildSystem:
          () => TestBuildSystem.all(BuildResult(success: true), (
            Target target,
            Environment environment,
          ) {
            expect(environment.defines, <String, String>{
              'TargetFile': 'lib/main.dart',
              'HasWebPlugins': 'true',
              'ServiceWorkerStrategy': 'offline-first',
              'BuildMode': 'release',
              'DartObfuscation': 'false',
              'TrackWidgetCreation': 'false',
              'TreeShakeIcons': 'true',
              'UseLocalCanvasKit': 'true',
            });
          }),
    },
  );

  testUsingContext(
    'hidden if feature flag is not enabled',
    () async {
      expect(
        BuildWebCommand(
          fileSystem: fileSystem,
          logger: BufferLogger.test(),
          verboseHelp: false,
        ).hidden,
        true,
      );
    },
    overrides: <Type, Generator>{
      Platform: () => fakePlatform,
      FileSystem: () => fileSystem,
      FeatureFlags: () => TestFeatureFlags(),
      ProcessManager: () => processManager,
    },
  );

  testUsingContext(
    'not hidden if feature flag is enabled',
    () async {
      expect(
        BuildWebCommand(
          fileSystem: fileSystem,
          logger: BufferLogger.test(),
          verboseHelp: false,
        ).hidden,
        false,
      );
    },
    overrides: <Type, Generator>{
      Platform: () => fakePlatform,
      FileSystem: () => fileSystem,
      FeatureFlags: () => TestFeatureFlags(isWebEnabled: true),
      ProcessManager: () => processManager,
    },
  );

  testUsingContext(
    'Defaults to web renderer canvaskit mode when no option is specified',
    () async {
      final TestWebBuildCommand buildCommand = TestWebBuildCommand(fileSystem: fileSystem);
      final CommandRunner<void> runner = createTestCommandRunner(buildCommand);
      setupFileSystemForEndToEndTest(fileSystem);
      await runner.run(<String>['build', 'web', '--no-pub']);
    },
    overrides: <Type, Generator>{
      Platform: () => fakePlatform,
      FileSystem: () => fileSystem,
      FeatureFlags: () => TestFeatureFlags(isWebEnabled: true),
      ProcessManager: () => processManager,
      BuildSystem:
          () => TestBuildSystem.all(BuildResult(success: true), (
            Target target,
            Environment environment,
          ) {
            expect(target, isA<WebServiceWorker>());
            final List<WebCompilerConfig> configs = (target as WebServiceWorker).compileConfigs;
            expect(configs.length, 1);
            expect(configs.first.renderer, WebRendererMode.canvaskit);
          }),
    },
  );

  testUsingContext(
    'Defaults to web renderer skwasm mode for wasm when no option is specified',
    () async {
      final TestWebBuildCommand buildCommand = TestWebBuildCommand(fileSystem: fileSystem);
      final CommandRunner<void> runner = createTestCommandRunner(buildCommand);
      setupFileSystemForEndToEndTest(fileSystem);
      await runner.run(<String>['build', 'web', '--no-pub', '--wasm']);
    },
    overrides: <Type, Generator>{
      Platform: () => fakePlatform,
      FileSystem: () => fileSystem,
      FeatureFlags: () => TestFeatureFlags(isWebEnabled: true),
      ProcessManager: () => processManager,
      BuildSystem:
          () => TestBuildSystem.all(BuildResult(success: true), (
            Target target,
            Environment environment,
          ) {
            expect(target, isA<WebServiceWorker>());
            final List<WebCompilerConfig> configs = (target as WebServiceWorker).compileConfigs;
            expect(configs.length, 2);
            expect(configs[0].renderer, WebRendererMode.skwasm);
            expect(configs[0].compileTarget, CompileTarget.wasm);
            expect(configs[1].renderer, WebRendererMode.canvaskit);
            expect(configs[1].compileTarget, CompileTarget.js);
          }),
    },
  );

  testUsingContext(
    'Web build supports build-name and build-number',
    () async {
      final TestWebBuildCommand buildCommand = TestWebBuildCommand(fileSystem: fileSystem);
      final CommandRunner<void> runner = createTestCommandRunner(buildCommand);
      setupFileSystemForEndToEndTest(fileSystem);

      await runner.run(<String>[
        'build',
        'web',
        '--no-pub',
        '--build-name=1.2.3',
        '--build-number=42',
      ]);

      final BuildInfo buildInfo = await buildCommand.webCommand.getBuildInfo(
        forcedBuildMode: BuildMode.debug,
      );
      expect(buildInfo.buildNumber, '42');
      expect(buildInfo.buildName, '1.2.3');
    },
    overrides: <Type, Generator>{
      Platform: () => fakePlatform,
      FileSystem: () => fileSystem,
      FeatureFlags: () => TestFeatureFlags(isWebEnabled: true),
      ProcessManager: () => processManager,
      BuildSystem: () => TestBuildSystem.all(BuildResult(success: true)),
    },
  );

  testUsingContext(
    'Does not override custom CanvasKit URL',
    () async {
      final TestWebBuildCommand buildCommand = TestWebBuildCommand(fileSystem: fileSystem);
      final CommandRunner<void> runner = createTestCommandRunner(buildCommand);
      setupFileSystemForEndToEndTest(fileSystem);
      await runner.run(<String>[
        'build',
        'web',
        '--no-pub',
        '--web-resources-cdn',
        '--dart-define=FLUTTER_WEB_CANVASKIT_URL=abcdefg',
      ]);
      final BuildInfo buildInfo = await buildCommand.webCommand.getBuildInfo(
        forcedBuildMode: BuildMode.debug,
      );
      expect(buildInfo.dartDefines, contains('FLUTTER_WEB_CANVASKIT_URL=abcdefg'));
    },
    overrides: <Type, Generator>{
      Platform: () => fakePlatform,
      FileSystem: () => fileSystem,
      FeatureFlags: () => TestFeatureFlags(isWebEnabled: true),
      ProcessManager: () => processManager,
      BuildSystem: () => TestBuildSystem.all(BuildResult(success: true)),
    },
  );

  testUsingContext(
    'Rejects --base-href value that does not start with /',
    () async {
      final TestWebBuildCommand buildCommand = TestWebBuildCommand(fileSystem: fileSystem);
      final CommandRunner<void> runner = createTestCommandRunner(buildCommand);

      await expectLater(
        runner.run(<String>[
          'build',
          'web',
          '--no-pub',
          '--base-href=i_dont_start_with_a_forward_slash',
        ]),
        throwsToolExit(
          message:
              'Received a --base-href value of "i_dont_start_with_a_forward_slash"\n'
              '--base-href should start and end with /',
        ),
      );
    },
    overrides: <Type, Generator>{
      Platform: () => fakePlatform,
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
    },
  );

  // Tests whether using a deprecated webRenderer toggles a warningText.
  Future<void> testWebRendererDeprecationMessage(WebRendererMode webRenderer) async {
    testUsingContext(
      'Using --web-renderer=${webRenderer.name} triggers a warningText.',
      () async {
        // Run the command so it parses --web-renderer, but ignore all errors.
        // We only care about the logger.
        try {
          final TestWebBuildCommand buildCommand = TestWebBuildCommand(fileSystem: fileSystem);
          await createTestCommandRunner(
            buildCommand,
          ).run(<String>['build', 'web', '--no-pub', ...webRenderer.toCliDartDefines]);
        } on ToolExit catch (error) {
          expect(error, isA<ToolExit>());
        }
        expect(
          logger.warningText,
          contains('See: https://docs.flutter.dev/to/web-html-renderer-deprecation'),
        );
      },
      overrides: <Type, Generator>{
        Platform: () => fakePlatform,
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
        Logger: () => logger,
        FeatureFlags: enableExplicitPackageDependencies,
        Pub: FakePubWithPrimedDeps.new,
      },
    );
  }

  /// Do test all the deprecated WebRendererModes
  WebRendererMode.values
      .where((WebRendererMode mode) => mode.isDeprecated)
      .forEach(testWebRendererDeprecationMessage);

  testUsingContext(
    'flutter build web option visibility',
    () async {
      final TestWebBuildCommand buildCommand = TestWebBuildCommand(fileSystem: fileSystem);
      createTestCommandRunner(buildCommand);
      final BuildWebCommand command = buildCommand.subcommands.values.single as BuildWebCommand;

      void expectVisible(String option) {
        expect(command.argParser.options.keys, contains(option));
        expect(command.argParser.options[option]!.hide, isFalse);
        expect(command.usage, contains(option));
      }

      expectVisible('pwa-strategy');
      expectVisible('web-resources-cdn');
      expectVisible('optimization-level');
      expectVisible('source-maps');
      expectVisible('csp');
      expectVisible('dart2js-optimization');
      expectVisible('dump-info');
      expectVisible('no-frequency-based-minification');
      expectVisible('wasm');
      expectVisible('strip-wasm');
      expectVisible('base-href');
    },
    overrides: <Type, Generator>{
      Platform: () => fakePlatform,
      FileSystem: () => fileSystem,
      FeatureFlags: () => TestFeatureFlags(isWebEnabled: true),
      ProcessManager: () => processManager,
    },
  );
}

void setupFileSystemForEndToEndTest(FileSystem fileSystem) {
  final List<String> dependencies = <String>[
    fileSystem.path.join(
      'packages',
      'flutter_tools',
      'lib',
      'src',
      'build_system',
      'targets',
      'web.dart',
    ),
    fileSystem.path.join('bin', 'cache', 'flutter_web_sdk'),
    fileSystem.path.join('bin', 'cache', 'dart-sdk', 'bin', 'dart'),
    fileSystem.path.join('bin', 'cache', 'dart-sdk', 'bin', 'dartaotruntime'),
    fileSystem.path.join('bin', 'cache', 'dart-sdk '),
  ];
  for (final String dependency in dependencies) {
    fileSystem.file(dependency).createSync(recursive: true);
  }

  // Project files.
  fileSystem.directory('.dart_tool').childFile('package_config.json')
    ..createSync(recursive: true)
    ..writeAsStringSync('''
{
  "packages": [
    {
      "name": "foo",
      "rootUri": "../",
      "packageUri": "lib/",
      "languageVersion": "3.2"
    },
    {
      "name": "fizz",
      "rootUri": "../bar",
      "packageUri": "lib/",
      "languageVersion": "3.2"
    }
  ],
  "configVersion": 2
}
''');
  fileSystem.file('pubspec.yaml').writeAsStringSync('''
name: foo

dependencies:
  flutter:
    sdk: flutter
  fizz:
    path:
      bar/
''');
  fileSystem.file(fileSystem.path.join('bar', 'pubspec.yaml'))
    ..createSync(recursive: true)
    ..writeAsStringSync('''
name: bar

flutter:
  plugin:
    platforms:
      web:
        pluginClass: UrlLauncherPlugin
        fileName: url_launcher_web.dart
''');
  fileSystem.file(fileSystem.path.join('bar', 'lib', 'url_launcher_web.dart'))
    ..createSync(recursive: true)
    ..writeAsStringSync('''
class UrlLauncherPlugin {}
''');
  fileSystem.file(fileSystem.path.join('lib', 'main.dart')).writeAsStringSync('void main() { }');
}

class TestWebBuildCommand extends FlutterCommand {
  TestWebBuildCommand({required FileSystem fileSystem, bool verboseHelp = false})
    : webCommand = BuildWebCommand(
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
        verboseHelp: verboseHelp,
      ) {
    addSubcommand(webCommand);
  }

  final BuildWebCommand webCommand;

  @override
  final String name = 'build';

  @override
  final String description = 'Build a test executable app.';

  @override
  Future<FlutterCommandResult> runCommand() async => FlutterCommandResult.fail();

  @override
  bool get shouldRunPub => false;
}
