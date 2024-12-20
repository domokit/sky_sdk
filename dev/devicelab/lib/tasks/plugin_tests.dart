// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ffi';
import 'dart:io';

import 'package:path/path.dart' as path;

import '../framework/framework.dart';
import '../framework/ios.dart';
import '../framework/task_result.dart';
import '../framework/utils.dart';

/// Combines several TaskFunctions with trivial success value into one.
TaskFunction combine(List<TaskFunction> tasks) {
  return () async {
    for (final TaskFunction task in tasks) {
      final TaskResult result = await task();
      if (result.failed) {
        return result;
      }
    }
    return TaskResult.success(null);
  };
}

/// Defines task that creates new Flutter project, adds a local and remote
/// plugin, and then builds the specified [buildTarget].
class PluginTest {
  PluginTest(
    this.buildTarget,
    this.options, {
    this.pluginCreateEnvironment,
    this.appCreateEnvironment,
    this.dartOnlyPlugin = false,
    this.sharedDarwinSource = false,
    this.template = 'plugin',
    this.cocoapodsTransitiveFlutterDependency = false,
  });

  final String buildTarget;
  final List<String> options;
  final Map<String, String>? pluginCreateEnvironment;
  final Map<String, String>? appCreateEnvironment;
  final bool dartOnlyPlugin;
  final bool sharedDarwinSource;
  final String template;
  final bool cocoapodsTransitiveFlutterDependency;

  Future<TaskResult> call() async {
    final Directory tempDir = Directory.systemTemp.createTempSync('flutter_devicelab_plugin_test.');
    // FFI plugins do not have support for `flutter test`.
    // `flutter test` does not do a native build.
    // Supporting `flutter test` would require invoking a native build.
    final bool runFlutterTest = template != 'plugin_ffi';
    try {
      section('Create plugin');
      final _FlutterProject plugin = await _FlutterProject.create(
        tempDir,
        options,
        buildTarget,
        name: 'plugintest',
        template: template,
        environment: pluginCreateEnvironment,
      );
      if (dartOnlyPlugin) {
        await plugin.convertDefaultPluginToDartPlugin();
      }
      if (sharedDarwinSource) {
        await plugin.convertDefaultPluginToSharedDarwinPlugin();
      }
      section('Test plugin');
      if (runFlutterTest) {
        await plugin.runFlutterTest();
        if (!dartOnlyPlugin) {
          await plugin.example.runNativeTests(buildTarget);
        }
      }
      section('Create Flutter app');
      final _FlutterProject app = await _FlutterProject.create(
        tempDir,
        options,
        buildTarget,
        name: 'plugintestapp',
        template: 'app',
        environment: appCreateEnvironment,
      );
      try {
        if (cocoapodsTransitiveFlutterDependency) {
          section('Disable Swift Package Manager');
          await app.disableSwiftPackageManager();
        }

        section('Add plugins');
        await app.addPlugin('plugintest', pluginPath: path.join('..', 'plugintest'));
        await app.addPlugin('path_provider');
        section('Build app');
        await app.build(buildTarget, validateNativeBuildProject: !dartOnlyPlugin);
        if (cocoapodsTransitiveFlutterDependency) {
          section('Test app with Flutter as a transitive CocoaPods dependency');
          await app.addCocoapodsTransitiveFlutterDependency();
          await app.build(buildTarget, validateNativeBuildProject: !dartOnlyPlugin);
        }
        if (runFlutterTest) {
          section('Test app');
          await app.runFlutterTest();
        }
        // Validate local engine handling. Currently only implemented for macOS.
        if (!dartOnlyPlugin) {
          section('Validate local engine configuration');
          final String fakeEngineSourcePath = path.join(tempDir.path, 'engine');
          await _testLocalEngineConfiguration(app, fakeEngineSourcePath);
        }
      } finally {
        await plugin.delete();
        await app.delete();
      }
      return TaskResult.success(null);
    } catch (e) {
      return TaskResult.failure(e.toString());
    } finally {
      rmTree(tempDir);
    }
  }

  Future<void> _testLocalEngineConfiguration(
    _FlutterProject app,
    String fakeEngineSourcePath,
  ) async {
    // The tool requires that a directory that looks like an engine build
    // actually exists when passing --local-engine, so create a fake skeleton.
    final Directory buildDir = Directory(path.join(fakeEngineSourcePath, 'out', 'foo'));
    buildDir.createSync(recursive: true);
    // Currently this test is only implemented for macOS; it can be extended to
    // others as needed.
    if (buildTarget == 'macos') {
      // When using a local engine, podhelper.rb will search for a "macos-"
      // directory within the FlutterMacOS.xcframework, so create a dummy one.
      Directory(
        path.join(buildDir.path, 'FlutterMacOS.xcframework/macos-arm64_x86_64'),
      ).createSync(recursive: true);

      // Clean before regenerating the config to ensure that the pod steps run.
      await inDirectory(Directory(app.rootPath), () async {
        await evalFlutter('clean');
      });
      await app.build(buildTarget, configOnly: true, localEngine: buildDir);
    }
  }
}

class _FlutterProject {
  _FlutterProject(this.parent, this.name);

  final Directory parent;
  final String name;

  String get rootPath => path.join(parent.path, name);

  File get pubspecFile => File(path.join(rootPath, 'pubspec.yaml'));

  _FlutterProject get example {
    return _FlutterProject(Directory(path.join(rootPath)), 'example');
  }

  Future<void> disableSwiftPackageManager() async {
    final File pubspec = pubspecFile;
    String content = await pubspec.readAsString();
    content = content.replaceFirst(
      '# The following section is specific to Flutter packages.\n'
          'flutter:\n',
      '# The following section is specific to Flutter packages.\n'
          'flutter:\n'
          '\n'
          '  disable-swift-package-manager: true\n',
    );
    await pubspec.writeAsString(content, flush: true);
  }

  Future<void> addPlugin(String plugin, {String? pluginPath}) async {
    final File pubspec = pubspecFile;
    String content = await pubspec.readAsString();
    final String dependency = pluginPath != null ? '$plugin:\n    path: $pluginPath' : '$plugin:';
    content = content.replaceFirst('\ndependencies:\n', '\ndependencies:\n  $dependency\n');
    await pubspec.writeAsString(content, flush: true);
  }

  /// Converts a plugin created from the standard template to a Dart-only
  /// plugin.
  Future<void> convertDefaultPluginToDartPlugin() async {
    final String dartPluginClass = 'DartClassFor$name';
    // Convert the metadata.
    final File pubspec = pubspecFile;
    String content = await pubspec.readAsString();
    content = content.replaceAll(
      RegExp(r' pluginClass: .*?\n'),
      ' dartPluginClass: $dartPluginClass\n',
    );
    await pubspec.writeAsString(content, flush: true);

    // Add the Dart registration hook that the build will generate a call to.
    final File dartCode = File(path.join(rootPath, 'lib', '$name.dart'));
    content = await dartCode.readAsString();
    content = '''
$content

class $dartPluginClass {
  static void registerWith() {}
}
''';
    await dartCode.writeAsString(content, flush: true);

    // Remove any native plugin code.
    const List<String> platforms = <String>['android', 'ios', 'linux', 'macos', 'windows'];
    for (final String platform in platforms) {
      final Directory platformDir = Directory(path.join(rootPath, platform));
      if (platformDir.existsSync()) {
        await platformDir.delete(recursive: true);
      }
    }
  }

  /// Converts an iOS/macOS plugin created from the standard template to a shared
  /// darwin directory plugin.
  Future<void> convertDefaultPluginToSharedDarwinPlugin() async {
    // Convert the metadata.
    final File pubspec = pubspecFile;
    String pubspecContent = await pubspec.readAsString();
    const String originalIOSKey = '\n      ios:\n';
    const String originalMacOSKey = '\n      macos:\n';
    if (!pubspecContent.contains(originalIOSKey) || !pubspecContent.contains(originalMacOSKey)) {
      print(pubspecContent);
      throw TaskResult.failure('Missing expected darwin platform plugin keys');
    }
    pubspecContent = pubspecContent.replaceAll(
      originalIOSKey,
      '$originalIOSKey        sharedDarwinSource: true\n',
    );
    pubspecContent = pubspecContent.replaceAll(
      originalMacOSKey,
      '$originalMacOSKey        sharedDarwinSource: true\n',
    );
    await pubspec.writeAsString(pubspecContent, flush: true);

    // Copy ios to darwin, and delete macos.
    final Directory iosDir = Directory(path.join(rootPath, 'ios'));
    final Directory darwinDir = Directory(path.join(rootPath, 'darwin'));
    recursiveCopy(iosDir, darwinDir);

    await iosDir.delete(recursive: true);
    await Directory(path.join(rootPath, 'macos')).delete(recursive: true);

    final File podspec = File(path.join(darwinDir.path, '$name.podspec'));
    String podspecContent = await podspec.readAsString();
    if (!podspecContent.contains('s.platform =')) {
      print(podspecContent);
      throw TaskResult.failure('Missing expected podspec platform');
    }

    // Remove "s.platform = :ios" to work on all platforms, including macOS.
    podspecContent = podspecContent.replaceFirst(RegExp(r'.*s\.platform.*'), '');
    podspecContent = podspecContent.replaceFirst(
      "s.dependency 'Flutter'",
      "s.ios.dependency 'Flutter'\ns.osx.dependency 'FlutterMacOS'",
    );

    await podspec.writeAsString(podspecContent, flush: true);

    // Make PlugintestPlugin.swift compile on iOS and macOS with target conditionals.
    // If SwiftPM is disabled, the file will be in `darwin/Classes/`.
    // Otherwise, the file will be in `darwin/<plugin>/Sources/<plugin>/`.
    final String pluginClass = '${name[0].toUpperCase()}${name.substring(1)}Plugin';
    print('pluginClass: $pluginClass');
    File pluginRegister = File(path.join(darwinDir.path, 'Classes', '$pluginClass.swift'));
    if (!pluginRegister.existsSync()) {
      pluginRegister = File(path.join(darwinDir.path, name, 'Sources', name, '$pluginClass.swift'));
    }
    final String pluginRegisterContent = '''
#if os(macOS)
import FlutterMacOS
#elseif os(iOS)
import Flutter
#endif

public class $pluginClass: NSObject, FlutterPlugin {
  public static func register(with registrar: FlutterPluginRegistrar) {
#if os(macOS)
    let channel = FlutterMethodChannel(name: "$name", binaryMessenger: registrar.messenger)
#elseif os(iOS)
    let channel = FlutterMethodChannel(name: "$name", binaryMessenger: registrar.messenger())
#endif
    let instance = $pluginClass()
    registrar.addMethodCallDelegate(instance, channel: channel)
  }

  public func handle(_ call: FlutterMethodCall, result: @escaping FlutterResult) {
#if os(macOS)
    result("macOS " + ProcessInfo.processInfo.operatingSystemVersionString)
#elseif os(iOS)
    result("iOS " + UIDevice.current.systemVersion)
#endif
  }
}
''';
    await pluginRegister.writeAsString(pluginRegisterContent, flush: true);
  }

  Future<void> runFlutterTest() async {
    await inDirectory(Directory(rootPath), () async {
      await flutter('test');
    });
  }

  Future<void> runNativeTests(String buildTarget) async {
    // Native unit tests rely on building the app first to generate necessary
    // build files.
    await build(buildTarget, validateNativeBuildProject: false);

    switch (buildTarget) {
      case 'apk':
        if (await exec(
              path.join('.', 'gradlew'),
              <String>['testDebugUnitTest'],
              workingDirectory: path.join(rootPath, 'android'),
              canFail: true,
            ) !=
            0) {
          throw TaskResult.failure('Platform unit tests failed');
        }
      case 'ios':
        String? simulatorDeviceId;
        try {
          await testWithNewIOSSimulator('TestNativeUnitTests', (String deviceId) async {
            simulatorDeviceId = deviceId;
            if (!await runXcodeTests(
              platformDirectory: path.join(rootPath, 'ios'),
              destination: 'id=$deviceId',
              configuration: 'Debug',
              testName: 'native_plugin_unit_tests_ios',
              skipCodesign: true,
            )) {
              throw TaskResult.failure('Platform unit tests failed');
            }
          });
        } finally {
          await removeIOSSimulator(simulatorDeviceId);
        }
      case 'linux':
        if (await exec(
              path.join(
                rootPath,
                'build',
                'linux',
                'x64',
                'release',
                'plugins',
                'plugintest',
                'plugintest_test',
              ),
              <String>[],
              canFail: true,
            ) !=
            0) {
          throw TaskResult.failure('Platform unit tests failed');
        }
      case 'macos':
        if (!await runXcodeTests(
          platformDirectory: path.join(rootPath, 'macos'),
          destination: 'platform=macOS',
          configuration: 'Debug',
          testName: 'native_plugin_unit_tests_macos',
          skipCodesign: true,
        )) {
          throw TaskResult.failure('Platform unit tests failed');
        }
      case 'windows':
        final String arch = Abi.current() == Abi.windowsX64 ? 'x64' : 'arm64';
        if (await exec(
              path.join(
                rootPath,
                'build',
                'windows',
                arch,
                'plugins',
                'plugintest',
                'Release',
                'plugintest_test.exe',
              ),
              <String>[],
              canFail: true,
            ) !=
            0) {
          throw TaskResult.failure('Platform unit tests failed');
        }
    }
  }

  static Future<_FlutterProject> create(
    Directory directory,
    List<String> options,
    String target, {
    required String name,
    required String template,
    Map<String, String>? environment,
  }) async {
    await inDirectory(directory, () async {
      await flutter(
        'create',
        options: <String>[
          '--template=$template',
          '--org',
          'io.flutter.devicelab',
          ...options,
          name,
        ],
        environment: environment,
      );
    });

    final _FlutterProject project = _FlutterProject(directory, name);
    if (template == 'plugin' && (target == 'ios' || target == 'macos')) {
      project._reduceDarwinPluginMinimumVersion(name, target);
    }
    return project;
  }

  /// Creates a Pod that uses a Flutter plugin as a dependency and therefore
  /// Flutter as a transitive dependency.
  Future<void> addCocoapodsTransitiveFlutterDependency() async {
    final String iosDirectoryPath = path.join(rootPath, 'ios');

    final File nativePod = File(path.join(iosDirectoryPath, 'NativePod', 'NativePod.podspec'));
    nativePod.createSync(recursive: true);
    nativePod.writeAsStringSync('''
Pod::Spec.new do |s|
  s.name             = 'NativePod'
  s.version          = '1.0.0'
  s.summary          = 'A pod to test Flutter as a transitive dependency.'
  s.homepage         = 'https://flutter.dev'
  s.license          = { :type => 'BSD' }
  s.author           = { 'Flutter Dev Team' => 'flutter-dev@googlegroups.com' }
  s.source           = { :path => '.' }
  s.source_files = "Classes", "Classes/**/*.{h,m}"
  s.dependency 'plugintest'
end
''');

    final File nativePodClass = File(
      path.join(iosDirectoryPath, 'NativePod', 'Classes', 'NativePodTest.m'),
    );
    nativePodClass.createSync(recursive: true);
    nativePodClass.writeAsStringSync('''
#import <Flutter/Flutter.h>

@interface NativePodTest : NSObject

@end

@implementation NativePodTest

@end
''');

    final File podfileFile = File(path.join(iosDirectoryPath, 'Podfile'));
    final List<String> podfileContents = podfileFile.readAsLinesSync();
    final int index = podfileContents.indexWhere(
      (String line) => line.contains('flutter_install_all_ios_pods'),
    );
    podfileContents.insert(index, "pod 'NativePod', :path => 'NativePod'");
    podfileFile.writeAsStringSync(podfileContents.join('\n'));
  }

  // Make the platform version artificially low to test that the "deployment
  // version too low" warning is never emitted.
  void _reduceDarwinPluginMinimumVersion(String plugin, String target) {
    final File podspec = File(path.join(rootPath, target, '$plugin.podspec'));
    if (!podspec.existsSync()) {
      throw TaskResult.failure('podspec file missing at ${podspec.path}');
    }
    final String versionString =
        target == 'ios' ? "s.platform = :ios, '12.0'" : "s.platform = :osx, '10.11'";
    String podspecContent = podspec.readAsStringSync();
    if (!podspecContent.contains(versionString)) {
      throw TaskResult.failure(
        'Update this test to match plugin minimum $target deployment version',
      );
    }
    // Add transitive dependency on AppAuth 1.6 targeting iOS 8 and macOS 10.9, which no longer builds in Xcode
    // to test the version is forced higher and builds.
    const String iosContent = '''
s.platform = :ios, '10.0'
s.dependency 'AppAuth', '1.6.0'
''';

    const String macosContent = '''
s.platform = :osx, '10.8'
s.dependency 'AppAuth', '1.6.0'
''';

    podspecContent = podspecContent.replaceFirst(
      versionString,
      target == 'ios' ? iosContent : macosContent,
    );
    podspec.writeAsStringSync(podspecContent, flush: true);
  }

  Future<void> build(
    String target, {
    bool validateNativeBuildProject = true,
    bool configOnly = false,
    Directory? localEngine,
  }) async {
    await inDirectory(Directory(rootPath), () async {
      final String buildOutput = await evalFlutter(
        'build',
        options: <String>[
          target,
          '-v',
          if (target == 'ios') '--no-codesign',
          if (configOnly) '--config-only',
          if (localEngine != null)
          // The engine directory is of the form <fake-source-path>/out/<fakename>,
          // which has to be broken up into the component flags.
          ...<String>[
            '--local-engine-src-path=${localEngine.parent.parent.path}',
            '--local-engine=${path.basename(localEngine.path)}',
            '--local-engine-host=${path.basename(localEngine.path)}',
          ],
        ],
      );

      if (target == 'ios' || target == 'macos') {
        // This warning is confusing and shouldn't be emitted. Plugins often support lower versions than the
        // Flutter app, but as long as they support the minimum it will work.
        // warning: The iOS deployment target 'IPHONEOS_DEPLOYMENT_TARGET' is set to 8.0,
        // but the range of supported deployment target versions is 9.0 to 14.0.99.
        //
        // (or "The macOS deployment target 'MACOSX_DEPLOYMENT_TARGET'"...)
        if (buildOutput.contains(
              'is set to 10.0, but the range of supported deployment target versions',
            ) ||
            buildOutput.contains(
              'is set to 10.8, but the range of supported deployment target versions',
            )) {
          throw TaskResult.failure('Minimum plugin version warning present');
        }

        if (validateNativeBuildProject) {
          final File generatedSwiftManifest = File(
            path.join(
              rootPath,
              target,
              'Flutter',
              'ephemeral',
              'Packages',
              'FlutterGeneratedPluginSwiftPackage',
              'Package.swift',
            ),
          );
          final bool swiftPackageManagerEnabled = generatedSwiftManifest.existsSync();

          if (!swiftPackageManagerEnabled) {
            final File podsProject = File(
              path.join(rootPath, target, 'Pods', 'Pods.xcodeproj', 'project.pbxproj'),
            );
            if (!podsProject.existsSync()) {
              throw TaskResult.failure('Xcode Pods project file missing at ${podsProject.path}');
            }

            final String podsProjectContent = podsProject.readAsStringSync();
            if (target == 'ios') {
              // Plugins with versions lower than the app version should not have IPHONEOS_DEPLOYMENT_TARGET set.
              // The plugintest plugin target should not have IPHONEOS_DEPLOYMENT_TARGET set since it has been lowered
              // in _reduceDarwinPluginMinimumVersion to 10, which is below the target version of 11.
              if (podsProjectContent.contains('IPHONEOS_DEPLOYMENT_TARGET = 10')) {
                throw TaskResult.failure(
                  'Plugin build setting IPHONEOS_DEPLOYMENT_TARGET not removed',
                );
              }
              // Transitive dependency AppAuth targeting too-low 8.0 was not fixed.
              if (podsProjectContent.contains('IPHONEOS_DEPLOYMENT_TARGET = 8')) {
                throw TaskResult.failure(
                  'Transitive dependency build setting IPHONEOS_DEPLOYMENT_TARGET=8 not removed',
                );
              }
              if (!podsProjectContent.contains(
                r'"EXCLUDED_ARCHS[sdk=iphonesimulator*]" = "$(inherited) i386";',
              )) {
                throw TaskResult.failure(r'EXCLUDED_ARCHS is not "$(inherited) i386"');
              }
            } else if (target == 'macos') {
              // Same for macOS deployment target, but 10.8.
              // The plugintest target should not have MACOSX_DEPLOYMENT_TARGET set.
              if (podsProjectContent.contains('MACOSX_DEPLOYMENT_TARGET = 10.8')) {
                throw TaskResult.failure(
                  'Plugin build setting MACOSX_DEPLOYMENT_TARGET not removed',
                );
              }
              // Transitive dependency AppAuth targeting too-low 10.9 was not fixed.
              if (podsProjectContent.contains('MACOSX_DEPLOYMENT_TARGET = 10.9')) {
                throw TaskResult.failure(
                  'Transitive dependency build setting MACOSX_DEPLOYMENT_TARGET=10.9 not removed',
                );
              }
            }

            if (localEngine != null) {
              final RegExp localEngineSearchPath = RegExp(
                'FRAMEWORK_SEARCH_PATHS\\s*=[^;]*${localEngine.path}',
              );
              if (!localEngineSearchPath.hasMatch(podsProjectContent)) {
                throw TaskResult.failure(
                  'FRAMEWORK_SEARCH_PATHS does not contain the --local-engine path',
                );
              }
            }
          }
        }
      }
    });
  }

  Future<void> delete() async {
    if (Platform.isWindows) {
      // A running Gradle daemon might prevent us from deleting the project
      // folder on Windows.
      final String wrapperPath = path.absolute(path.join(rootPath, 'android', 'gradlew.bat'));
      if (File(wrapperPath).existsSync()) {
        await exec(wrapperPath, <String>['--stop'], canFail: true);
      }
      // TODO(ianh): Investigating if flakiness is timing dependent.
      await Future<void>.delayed(const Duration(seconds: 10));
    }
    rmTree(parent);
  }
}
