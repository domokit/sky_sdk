// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// For `android` directory in the repo, this script generates:
//     1. The top-level build.gradle (android/build.gradle).
//     2. The top level settings.gradle (android/settings.gradle).
//     3. The gradle wrapper file (android/gradle/wrapper/gradle-wrapper.properties).
// Then it generate the lockfiles for each Gradle project.
// To regenerate these files, run `dart dev/tools/bin/generate_gradle_lockfiles.dart`.

import 'dart:collection';
import 'dart:io';

import 'package:args/args.dart';
import 'package:file/file.dart';
import 'package:file/local.dart';
import 'package:yaml/yaml.dart';

void main(List<String> arguments) {
  const String usageMessage =
      "If you don't wish to re-generate the "
      'settings.gradle, build.gradle, and gradle-wrapper.properties files,\n'
      'add the flag `--no-gradle-generation`.\n'
      'This tool automatically excludes a set of android subdirectories, '
      'defined at dev/tools/bin/config/lockfile_exclusion.yaml.\n'
      'To disable this behavior, run with `--no-exclusion`.\n';

  final ArgParser argParser =
      ArgParser()
        ..addFlag(
          'gradle-generation',
          help: 'Re-generate gradle files in each processed directory.',
          defaultsTo: true,
        )
        ..addFlag(
          'exclusion',
          help:
              'Run the script using the config file at ./configs/lockfile_exclusion.yaml to skip the specified subdirectories.',
          defaultsTo: true,
        );

  ArgResults args;
  try {
    args = argParser.parse(arguments);
  } on FormatException catch (error) {
    stderr.writeln('${error.message}\n');
    stderr.writeln(usageMessage);
    exit(1);
  }

  print(usageMessage);

  /// Re-generate gradle files in each processed directory.
  final bool gradleGeneration = (args['gradle-generation'] as bool?) ?? true;

  // Skip android subdirectories specified in the ./config/lockfile_exclusion.yaml file.
  final bool useExclusion = (args['exclusion'] as bool?) ?? true;

  const FileSystem fileSystem = LocalFileSystem();

  final Directory repoRoot =
      (() {
        final String repoRootPath =
            exec('git', const <String>['rev-parse', '--show-toplevel']).trim();
        final Directory repoRoot = fileSystem.directory(repoRootPath);
        if (!repoRoot.existsSync()) {
          throw StateError("Expected $repoRoot to exist but it didn't!");
        }
        return repoRoot;
      })();

  final Iterable<Directory> androidDirectories = discoverAndroidDirectories(repoRoot);

  final File exclusionFile = repoRoot
      .childDirectory('dev')
      .childDirectory('tools')
      .childDirectory('bin')
      .childDirectory('config')
      .childFile('lockfile_exclusion.yaml');

  // Load the exclusion set, or make an empty exclusion set.
  final Set<String> exclusionSet;
  if (useExclusion) {
    exclusionSet = HashSet<String>.from(
      (loadYaml(exclusionFile.readAsStringSync()) as YamlList).toList().cast<String>().map(
        (String s) => '${repoRoot.path}/$s',
      ),
    );
    print('Loaded exclusion file from ${exclusionFile.path}.');
  } else {
    exclusionSet = <String>{};
    print('Running without exclusion.');
  }

  for (final Directory androidDirectory in androidDirectories) {
    if (!androidDirectory.existsSync()) {
      throw '$androidDirectory does not exist';
    }

    if (exclusionSet.contains(androidDirectory.path)) {
      print(
        '${androidDirectory.path} is included in the exclusion config file at ${exclusionFile.path} - skipping',
      );
      continue;
    }

    late File rootBuildGradle;
    if (androidDirectory.childFile('build.gradle').existsSync()) {
      rootBuildGradle = androidDirectory.childFile('build.gradle');
    } else if (androidDirectory.childFile('build.gradle.kts').existsSync()) {
      rootBuildGradle = androidDirectory.childFile('build.gradle.kts');
    } else {
      print('${androidDirectory.childFile('build.gradle').path}(.kts) does not exist - skipping');
      continue;
    }

    late File settingsGradle;
    if (androidDirectory.childFile('settings.gradle').existsSync()) {
      settingsGradle = androidDirectory.childFile('settings.gradle');
    } else if (androidDirectory.childFile('settings.gradle.kts').existsSync()) {
      settingsGradle = androidDirectory.childFile('settings.gradle.kts');
    } else {
      print(
        '${androidDirectory.childFile('settings.gradle').path}(.kts) does not exist - skipping',
      );
      continue;
    }

    final File wrapperGradle = androidDirectory
        .childDirectory('gradle')
        .childDirectory('wrapper')
        .childFile('gradle-wrapper.properties');
    if (!wrapperGradle.existsSync()) {
      print('${wrapperGradle.path} does not exist - skipping');
      continue;
    }

    if (settingsGradle.readAsStringSync().contains('include_flutter.groovy')) {
      print('${settingsGradle.path} add to app - skipping');
      continue;
    }

    if (!androidDirectory.childDirectory('app').existsSync()) {
      print('${rootBuildGradle.path} is not an app - skipping');
      continue;
    }

    if (!androidDirectory.parent.childFile('pubspec.yaml').existsSync()) {
      print('${rootBuildGradle.path} no pubspec.yaml in parent directory - skipping');
      continue;
    }

    if (androidDirectory.parent
        .childFile('pubspec.yaml')
        .readAsStringSync()
        .contains('deferred-components')) {
      print('${rootBuildGradle.path} uses deferred components - skipping');
      continue;
    }

    if (!androidDirectory.parent.childDirectory('lib').childFile('main.dart').existsSync()) {
      print('${rootBuildGradle.path} no main.dart under lib - skipping');
      continue;
    }

    if (androidDirectory.path.contains('ios/.symlinks')) {
      print('${rootBuildGradle.path} is in the ios subdirectory, skipping');
      continue;
    }

    print('Processing ${androidDirectory.path}');

    try {
      androidDirectory.childFile('buildscript-gradle.lockfile').deleteSync();
    } on FileSystemException {
      // noop
    }

    if (gradleGeneration) {
      rootBuildGradle.writeAsStringSync(rootGradleFileContent);
      settingsGradle.writeAsStringSync(settingGradleFile);
      wrapperGradle.writeAsStringSync(wrapperGradleFileContent);
    }

    final String appDirectory = androidDirectory.parent.absolute.path;

    // Fetch pub dependencies.
    final String flutterPath = repoRoot.childDirectory('bin').childFile('flutter').path;
    exec(flutterPath, <String>['pub', 'get'], workingDirectory: appDirectory);

    // Verify that the Gradlew wrapper exists.
    final File gradleWrapper = androidDirectory.childFile('gradlew');
    // Generate Gradle wrapper if it doesn't exist.
    if (!gradleWrapper.existsSync()) {
      exec(flutterPath, <String>['build', 'apk', '--config-only'], workingDirectory: appDirectory);
    }

    // Generate lock files.
    exec(gradleWrapper.absolute.path, <String>[
      ':generateLockfiles',
    ], workingDirectory: androidDirectory.absolute.path);

    print('Processed');
  }
}

String exec(String cmd, List<String> args, {String? workingDirectory}) {
  final ProcessResult result = Process.runSync(cmd, args, workingDirectory: workingDirectory);
  if (result.exitCode != 0) {
    throw ProcessException(cmd, args, '${result.stdout}${result.stderr}', result.exitCode);
  }
  return result.stdout as String;
}

const String rootGradleFileContent = r'''
// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is auto generated.
// To update all the build.gradle files in the Flutter repo,
// See dev/tools/bin/generate_gradle_lockfiles.dart.

allprojects {
    repositories {
        google()
        mavenCentral()
    }
}

rootProject.layout.buildDirectory.value(rootProject.layout.buildDirectory.dir("../../build").get())

subprojects {
    project.layout.buildDirectory.value(rootProject.layout.buildDirectory.dir(project.name).get())
}
subprojects {
    project.evaluationDependsOn(':app')
    dependencyLocking {
        ignoredDependencies.add('io.flutter:*')
        lockFile = file("${rootProject.projectDir}/project-${project.name}.lockfile")
        if (!project.hasProperty('local-engine-repo')) {
          lockAllConfigurations()
        }
    }
}

tasks.register("clean", Delete) {
    delete rootProject.layout.buildDirectory
}
''';

const String settingGradleFile = r'''
// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is auto generated.
// To update all the settings.gradle files in the Flutter repo,
// See dev/tools/bin/generate_gradle_lockfiles.dart.

pluginManagement {
    def flutterSdkPath = {
        def properties = new Properties()
        file("local.properties").withInputStream { properties.load(it) }
        def flutterSdkPath = properties.getProperty("flutter.sdk")
        assert flutterSdkPath != null, "flutter.sdk not set in local.properties"
        return flutterSdkPath
    }
    settings.ext.flutterSdkPath = flutterSdkPath()

    includeBuild("${settings.ext.flutterSdkPath}/packages/flutter_tools/gradle")

    repositories {
        google()
        mavenCentral()
        gradlePluginPortal()
    }
}

buildscript {
    dependencyLocking {
        lockFile = file("${rootProject.projectDir}/buildscript-gradle.lockfile")
        lockAllConfigurations()
    }
}

plugins {
    id "dev.flutter.flutter-plugin-loader" version "1.0.0"
    id "com.android.application" version "8.7.0" apply false
    id "org.jetbrains.kotlin.android" version "1.8.10" apply false
}

include ":app"
''';

const String wrapperGradleFileContent = r'''
distributionBase=GRADLE_USER_HOME
distributionPath=wrapper/dists
zipStoreBase=GRADLE_USER_HOME
zipStorePath=wrapper/dists
distributionUrl=https\://services.gradle.org/distributions/gradle-8.10.2-all.zip
''';

Iterable<Directory> discoverAndroidDirectories(Directory repoRoot) {
  return repoRoot
      .listSync(recursive: true)
      .whereType<Directory>()
      .where((FileSystemEntity entity) => entity.basename == 'android');
}
