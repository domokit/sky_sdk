// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/android/android_studio.dart';
import 'package:flutter_tools/src/android/gradle_utils.dart';
import 'package:flutter_tools/src/android/migrations/android_studio_java_gradle_conflict_migration.dart';
import 'package:flutter_tools/src/android/migrations/min_sdk_version_migration.dart';
import 'package:flutter_tools/src/android/migrations/multidex_removal_migration.dart';
import 'package:flutter_tools/src/android/migrations/top_level_gradle_build_file_migration.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/version.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fakes.dart';

const String otherGradleVersionWrapper = r'''
distributionBase=GRADLE_USER_HOME
distributionPath=wrapper/dists
distributionUrl=https\://services.gradle.org/distributions/gradle-6.6-all.zip
zipStoreBase=GRADLE_USER_HOME
zipStorePath=wrapper/dists
''';

const String gradleWrapperToMigrate = r'''
distributionBase=GRADLE_USER_HOME
distributionPath=wrapper/dists
distributionUrl=https\://services.gradle.org/distributions/gradle-6.7-all.zip
zipStoreBase=GRADLE_USER_HOME
zipStorePath=wrapper/dists
''';

const String gradleWrapperToMigrateTo = r'''
distributionBase=GRADLE_USER_HOME
distributionPath=wrapper/dists
distributionUrl=https\://services.gradle.org/distributions/gradle-7.6.1-all.zip
zipStoreBase=GRADLE_USER_HOME
zipStorePath=wrapper/dists
''';

String sampleModuleGradleBuildFile(String minSdkVersionString) {
  return r'''
plugins {
    id "com.android.application"
    id "kotlin-android"
    id "dev.flutter.flutter-gradle-plugin"
}

def localProperties = new Properties()
def localPropertiesFile = rootProject.file('local.properties')
if (localPropertiesFile.exists()) {
    localPropertiesFile.withReader('UTF-8') { reader ->
        localProperties.load(reader)
    }
}

def flutterVersionCode = localProperties.getProperty('flutter.versionCode')
if (flutterVersionCode == null) {
    flutterVersionCode = '1'
}

def flutterVersionName = localProperties.getProperty('flutter.versionName')
if (flutterVersionName == null) {
    flutterVersionName = '1.0'
}

android {
    namespace "com.example.asset_sample"
    compileSdk flutter.compileSdkVersion
    ndkVersion flutter.ndkVersion

    compileOptions {
        sourceCompatibility JavaVersion.VERSION_1_8
        targetCompatibility JavaVersion.VERSION_1_8
    }

    kotlinOptions {
        jvmTarget = '1.8'
    }

    sourceSets {
        main.java.srcDirs += 'src/main/kotlin'
    }

    defaultConfig {
        // TODO: Specify your own unique Application ID (https://developer.android.com/studio/build/application-id.html).
        applicationId "com.example.asset_sample"
        // You can update the following values to match your application needs.
        // For more information, see: https://flutter.dev/to/review-gradle-config.
        ''' +
      minSdkVersionString +
      r'''

        targetSdkVersion flutter.targetSdkVersion
        versionCode flutterVersionCode.toInteger()
        versionName flutterVersionName
    }

    buildTypes {
        release {
            // TODO: Add your own signing config for the release build.
            // Signing with the debug keys for now, so `flutter run --release` works.
            signingConfig signingConfigs.debug
        }
    }
}

flutter {
    source '../..'
}

dependencies {}
''';
}

final Version androidStudioDolphin = Version(2021, 3, 1);

const Version _javaVersion17 = Version.withText(17, 0, 2, 'openjdk 17.0.2');
const Version _javaVersion16 = Version.withText(16, 0, 2, 'openjdk 16.0.2');

void main() {
  group('Android migration', () {
    group('migrate the Gradle "clean" task to lazy declaration', () {
      late MemoryFileSystem memoryFileSystem;
      late BufferLogger bufferLogger;
      late FakeAndroidProject project;
      late File topLevelGradleBuildFile;

      setUp(() {
        memoryFileSystem = MemoryFileSystem.test();
        bufferLogger = BufferLogger.test();
        project = FakeAndroidProject(
          root: memoryFileSystem.currentDirectory.childDirectory('android')..createSync(),
        );
        topLevelGradleBuildFile = project.hostAppGradleRoot.childFile('build.gradle');
      });

      testUsingContext('skipped if files are missing', () async {
        final TopLevelGradleBuildFileMigration androidProjectMigration =
            TopLevelGradleBuildFileMigration(project, bufferLogger);
        await androidProjectMigration.migrate();
        expect(topLevelGradleBuildFile.existsSync(), isFalse);
        expect(
          bufferLogger.traceText,
          contains('Top-level Gradle build file not found, skipping migration of task "clean".'),
        );
      });

      testUsingContext('skipped if nothing to upgrade', () async {
        topLevelGradleBuildFile.writeAsStringSync('''
tasks.register("clean", Delete) {
  delete rootProject.buildDir
}
        ''');

        final TopLevelGradleBuildFileMigration androidProjectMigration =
            TopLevelGradleBuildFileMigration(project, bufferLogger);
        final DateTime previousLastModified = topLevelGradleBuildFile.lastModifiedSync();
        await androidProjectMigration.migrate();

        expect(topLevelGradleBuildFile.lastModifiedSync(), previousLastModified);
      });

      testUsingContext('top-level build.gradle is migrated', () async {
        topLevelGradleBuildFile.writeAsStringSync('''
task clean(type: Delete) {
    delete rootProject.buildDir
}
''');

        final TopLevelGradleBuildFileMigration androidProjectMigration =
            TopLevelGradleBuildFileMigration(project, bufferLogger);
        await androidProjectMigration.migrate();

        expect(
          bufferLogger.traceText,
          contains('Migrating "clean" Gradle task to lazy declaration style.'),
        );
        expect(
          topLevelGradleBuildFile.readAsStringSync(),
          equals('''
tasks.register("clean", Delete) {
    delete rootProject.layout.buildDirectory
}
'''),
        );
      });
    });

    group('migrate the gradle version to one that does not conflict with the '
        'Android Studio-provided java version', () {
      late MemoryFileSystem memoryFileSystem;
      late BufferLogger bufferLogger;
      late FakeAndroidProject project;
      late File gradleWrapperPropertiesFile;

      setUp(() {
        memoryFileSystem = MemoryFileSystem.test();
        bufferLogger = BufferLogger.test();
        project = FakeAndroidProject(
          root: memoryFileSystem.currentDirectory.childDirectory('android')..createSync(),
        );
        project.hostAppGradleRoot
            .childDirectory(gradleDirectoryName)
            .childDirectory(gradleWrapperDirectoryName)
            .createSync(recursive: true);
        gradleWrapperPropertiesFile = project.hostAppGradleRoot
            .childDirectory(gradleDirectoryName)
            .childDirectory(gradleWrapperDirectoryName)
            .childFile(gradleWrapperPropertiesFilename);
      });

      testWithoutContext('skipped if files are missing', () async {
        final AndroidStudioJavaGradleConflictMigration migration =
            AndroidStudioJavaGradleConflictMigration(
              java: FakeJava(version: _javaVersion17),
              bufferLogger,
              project: project,
              androidStudio: FakeAndroidStudio(version: androidStudioDolphin),
            );
        await migration.migrate();
        expect(gradleWrapperPropertiesFile.existsSync(), isFalse);
        expect(bufferLogger.traceText, contains(gradleWrapperNotFound));
      });

      testWithoutContext('skipped if android studio is null', () async {
        final AndroidStudioJavaGradleConflictMigration migration =
            AndroidStudioJavaGradleConflictMigration(
              java: FakeJava(version: _javaVersion17),
              bufferLogger,
              project: project,
            );
        gradleWrapperPropertiesFile.writeAsStringSync(gradleWrapperToMigrate);
        await migration.migrate();
        expect(bufferLogger.traceText, contains(androidStudioNotFound));
        expect(gradleWrapperPropertiesFile.readAsStringSync(), gradleWrapperToMigrate);
      });

      testWithoutContext('skipped if android studio version is null', () async {
        final AndroidStudioJavaGradleConflictMigration migration =
            AndroidStudioJavaGradleConflictMigration(
              java: FakeJava(version: _javaVersion17),
              bufferLogger,
              project: project,
              androidStudio: FakeAndroidStudio(version: null),
            );
        gradleWrapperPropertiesFile.writeAsStringSync(gradleWrapperToMigrate);
        await migration.migrate();
        expect(bufferLogger.traceText, contains(androidStudioNotFound));
        expect(gradleWrapperPropertiesFile.readAsStringSync(), gradleWrapperToMigrate);
      });

      testWithoutContext('skipped if error is encountered in migrate()', () async {
        final AndroidStudioJavaGradleConflictMigration migration =
            AndroidStudioJavaGradleConflictMigration(
              java: FakeErroringJava(),
              bufferLogger,
              project: project,
              androidStudio: FakeAndroidStudio(version: androidStudioFlamingo),
            );
        gradleWrapperPropertiesFile.writeAsStringSync(gradleWrapperToMigrate);
        await migration.migrate();
        expect(bufferLogger.traceText, contains(errorWhileMigrating));
        expect(gradleWrapperPropertiesFile.readAsStringSync(), gradleWrapperToMigrate);
      });

      testWithoutContext('skipped if android studio version is less than flamingo', () async {
        final AndroidStudioJavaGradleConflictMigration migration =
            AndroidStudioJavaGradleConflictMigration(
              java: FakeJava(),
              bufferLogger,
              project: project,
              androidStudio: FakeAndroidStudio(version: androidStudioDolphin),
            );
        gradleWrapperPropertiesFile.writeAsStringSync(gradleWrapperToMigrate);
        await migration.migrate();
        expect(gradleWrapperPropertiesFile.readAsStringSync(), gradleWrapperToMigrate);
        expect(bufferLogger.traceText, contains(androidStudioVersionBelowFlamingo));
      });

      testWithoutContext('skipped if bundled java version is less than 17', () async {
        final AndroidStudioJavaGradleConflictMigration migration =
            AndroidStudioJavaGradleConflictMigration(
              java: FakeJava(version: _javaVersion16),
              bufferLogger,
              project: project,
              androidStudio: FakeAndroidStudio(version: androidStudioFlamingo),
            );
        gradleWrapperPropertiesFile.writeAsStringSync(gradleWrapperToMigrate);
        await migration.migrate();
        expect(gradleWrapperPropertiesFile.readAsStringSync(), gradleWrapperToMigrate);
        expect(bufferLogger.traceText, contains(javaVersionNot17));
      });

      testWithoutContext('nothing is changed if gradle version not one that was '
          'used by flutter create', () async {
        final AndroidStudioJavaGradleConflictMigration migration =
            AndroidStudioJavaGradleConflictMigration(
              java: FakeJava(version: _javaVersion17),
              bufferLogger,
              project: project,
              androidStudio: FakeAndroidStudio(version: androidStudioFlamingo),
            );
        gradleWrapperPropertiesFile.writeAsStringSync(otherGradleVersionWrapper);
        await migration.migrate();
        expect(gradleWrapperPropertiesFile.readAsStringSync(), otherGradleVersionWrapper);
        expect(bufferLogger.traceText, isEmpty);
      });

      testWithoutContext('change is made with one of the specific gradle versions'
          ' we migrate for', () async {
        final AndroidStudioJavaGradleConflictMigration migration =
            AndroidStudioJavaGradleConflictMigration(
              java: FakeJava(version: _javaVersion17),
              bufferLogger,
              project: project,
              androidStudio: FakeAndroidStudio(version: androidStudioFlamingo),
            );
        gradleWrapperPropertiesFile.writeAsStringSync(gradleWrapperToMigrate);
        await migration.migrate();
        expect(gradleWrapperPropertiesFile.readAsStringSync(), gradleWrapperToMigrateTo);
        expect(
          bufferLogger.statusText,
          contains(
            'Conflict detected between '
            'Android Studio Java version and Gradle version, upgrading Gradle '
            'version from 6.7 to $gradleVersion7_6_1.',
          ),
        );
      });

      testWithoutContext('change is not made when opt out flag is set', () async {
        final AndroidStudioJavaGradleConflictMigration migration =
            AndroidStudioJavaGradleConflictMigration(
              java: FakeJava(version: _javaVersion17),
              bufferLogger,
              project: project,
              androidStudio: FakeAndroidStudio(version: androidStudioFlamingo),
            );
        gradleWrapperPropertiesFile.writeAsStringSync(gradleWrapperToMigrate + optOutFlag);
        await migration.migrate();
        expect(gradleWrapperPropertiesFile.readAsStringSync(), gradleWrapperToMigrate + optOutFlag);
        expect(bufferLogger.traceText, contains(optOutFlagEnabled));
      });
    });

    group('migrate min sdk versions less than 21 to flutter.minSdkVersion '
        'when in a FlutterProject that is an app', () {
      late MemoryFileSystem memoryFileSystem;
      late BufferLogger bufferLogger;
      late FakeAndroidProject project;
      late MinSdkVersionMigration migration;

      setUp(() {
        memoryFileSystem = MemoryFileSystem.test();
        memoryFileSystem.currentDirectory.childDirectory('android').createSync();
        bufferLogger = BufferLogger.test();
        project = FakeAndroidProject(
          root: memoryFileSystem.currentDirectory.childDirectory('android'),
        );
        project.appGradleFile.parent.createSync(recursive: true);
        migration = MinSdkVersionMigration(project, bufferLogger);
      });

      testWithoutContext('do nothing when files missing', () async {
        await migration.migrate();
        expect(bufferLogger.traceText, contains(appGradleNotFoundWarning));
      });

      testWithoutContext('replace when api 19', () async {
        const String minSdkVersion19 = 'minSdkVersion 19';
        project.appGradleFile.writeAsStringSync(sampleModuleGradleBuildFile(minSdkVersion19));
        await migration.migrate();
        expect(
          project.appGradleFile.readAsStringSync(),
          sampleModuleGradleBuildFile(replacementMinSdkText),
        );
      });

      testWithoutContext('replace when api 20', () async {
        const String minSdkVersion20 = 'minSdkVersion 20';
        project.appGradleFile.writeAsStringSync(sampleModuleGradleBuildFile(minSdkVersion20));
        await migration.migrate();
        expect(
          project.appGradleFile.readAsStringSync(),
          sampleModuleGradleBuildFile(replacementMinSdkText),
        );
      });

      testWithoutContext('do nothing when >=api 21', () async {
        const String minSdkVersion21 = 'minSdkVersion 21';
        project.appGradleFile.writeAsStringSync(sampleModuleGradleBuildFile(minSdkVersion21));
        await migration.migrate();
        expect(
          project.appGradleFile.readAsStringSync(),
          sampleModuleGradleBuildFile(minSdkVersion21),
        );
      });

      testWithoutContext('do nothing when already using '
          'flutter.minSdkVersion', () async {
        project.appGradleFile.writeAsStringSync(sampleModuleGradleBuildFile(replacementMinSdkText));
        await migration.migrate();
        expect(
          project.appGradleFile.readAsStringSync(),
          sampleModuleGradleBuildFile(replacementMinSdkText),
        );
      });

      testWithoutContext('avoid rewriting comments', () async {
        const String code =
            '// minSdkVersion 19  // old default\n'
            '        minSdkVersion 23  // new version';
        project.appGradleFile.writeAsStringSync(sampleModuleGradleBuildFile(code));
        await migration.migrate();
        expect(project.appGradleFile.readAsStringSync(), sampleModuleGradleBuildFile(code));
      });

      testWithoutContext('do nothing when project is a module', () async {
        project = FakeAndroidProject(
          root: memoryFileSystem.currentDirectory.childDirectory('android'),
          module: true,
        );
        migration = MinSdkVersionMigration(project, bufferLogger);
        const String minSdkVersion19 = 'minSdkVersion 19';
        project.appGradleFile.writeAsStringSync(sampleModuleGradleBuildFile(minSdkVersion19));
        await migration.migrate();
        expect(
          project.appGradleFile.readAsStringSync(),
          sampleModuleGradleBuildFile(minSdkVersion19),
        );
      });

      testWithoutContext('do nothing when minSdkVersion is set '
          'to a constant', () async {
        const String minSdkVersionConstant = 'minSdkVersion kMinSdkversion';
        project.appGradleFile.writeAsStringSync(sampleModuleGradleBuildFile(minSdkVersionConstant));
        await migration.migrate();
        expect(
          project.appGradleFile.readAsStringSync(),
          sampleModuleGradleBuildFile(minSdkVersionConstant),
        );
      });

      testWithoutContext('do nothing when minSdkVersion is set '
          'using = syntax', () async {
        const String equalsSyntaxMinSdkVersion19 = 'minSdkVersion = 19';
        project.appGradleFile.writeAsStringSync(
          sampleModuleGradleBuildFile(equalsSyntaxMinSdkVersion19),
        );
        await migration.migrate();
        expect(
          project.appGradleFile.readAsStringSync(),
          sampleModuleGradleBuildFile(equalsSyntaxMinSdkVersion19),
        );
      });
    });

    group('delete FlutterMultiDexApplication.java, if it exists', () {
      late MemoryFileSystem memoryFileSystem;
      late BufferLogger bufferLogger;
      late FakeAndroidProject project;
      late MultidexRemovalMigration migration;

      setUp(() {
        memoryFileSystem = MemoryFileSystem.test();
        memoryFileSystem.currentDirectory.childDirectory('android').createSync();
        bufferLogger = BufferLogger.test();
        project = FakeAndroidProject(
          root: memoryFileSystem.currentDirectory.childDirectory('android'),
        );
        project.appGradleFile.parent.createSync(recursive: true);
        migration = MultidexRemovalMigration(project, bufferLogger);
      });

      testWithoutContext(
        'do nothing when FlutterMultiDexApplication.java is not present',
        () async {
          await migration.migrate();
          expect(bufferLogger.traceText, isEmpty);
        },
      );

      testWithoutContext(
        'delete and note when FlutterMultiDexApplication.java is present',
        () async {
          // Write a blank string to the FlutterMultiDexApplication.java file.
          final File flutterMultiDexApplication = project.hostAppGradleRoot
            .childDirectory('src')
            .childDirectory('main')
            .childDirectory('java')
            .childDirectory('io')
            .childDirectory('flutter')
            .childDirectory('app')
            .childFile('FlutterMultiDexApplication.java')..createSync(recursive: true);
          flutterMultiDexApplication.writeAsStringSync('');

          await migration.migrate();
          expect(bufferLogger.traceText, contains(MultidexRemovalMigration.deletionMessage));
          expect(flutterMultiDexApplication.existsSync(), false);
        },
      );
    });
  });
}

class FakeAndroidProject extends Fake implements AndroidProject {
  FakeAndroidProject({required Directory root, this.module, this.plugin})
    : hostAppGradleRoot = root;

  @override
  Directory hostAppGradleRoot;

  final bool? module;
  final bool? plugin;

  @override
  bool get isPlugin => plugin ?? false;

  @override
  bool get isModule => module ?? false;

  @override
  File get appGradleFile => hostAppGradleRoot.childDirectory('app').childFile('build.gradle');
}

class FakeAndroidStudio extends Fake implements AndroidStudio {
  FakeAndroidStudio({required Version? version}) {
    _version = version;
  }

  late Version? _version;

  @override
  Version? get version => _version;
}

class FakeErroringJava extends FakeJava {
  @override
  Version get version {
    throw Exception('How did this happen?');
  }
}
