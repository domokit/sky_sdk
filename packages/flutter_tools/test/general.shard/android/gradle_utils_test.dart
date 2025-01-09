// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/android/gradle_utils.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/version_range.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/project.dart';

import '../../src/common.dart';
import '../../src/fake_process_manager.dart';
import '../../src/fakes.dart';

void main() {
  group('injectGradleWrapperIfNeeded', () {
    late FileSystem fileSystem;
    late Directory gradleWrapperDirectory;
    late GradleUtils gradleUtils;

    setUp(() {
      fileSystem = MemoryFileSystem.test();
      gradleWrapperDirectory = fileSystem.directory('bin/cache/artifacts/gradle_wrapper');
      gradleWrapperDirectory.createSync(recursive: true);
      gradleWrapperDirectory.childFile('gradlew').writeAsStringSync('irrelevant');
      gradleWrapperDirectory
          .childDirectory('gradle')
          .childDirectory('wrapper')
          .createSync(recursive: true);
      gradleWrapperDirectory
          .childDirectory('gradle')
          .childDirectory('wrapper')
          .childFile('gradle-wrapper.jar')
          .writeAsStringSync('irrelevant');
      gradleUtils = GradleUtils(
        cache: Cache.test(processManager: FakeProcessManager.any(), fileSystem: fileSystem),
        platform: FakePlatform(environment: <String, String>{}),
        logger: BufferLogger.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      );
    });

    testWithoutContext('injects the wrapper when all files are missing', () {
      final Directory sampleAppAndroid = fileSystem.directory('/sample-app/android');
      sampleAppAndroid.createSync(recursive: true);

      gradleUtils.injectGradleWrapperIfNeeded(sampleAppAndroid);

      expect(sampleAppAndroid.childFile('gradlew').existsSync(), isTrue);

      expect(
        sampleAppAndroid
            .childDirectory('gradle')
            .childDirectory('wrapper')
            .childFile('gradle-wrapper.jar')
            .existsSync(),
        isTrue,
      );

      expect(
        sampleAppAndroid
            .childDirectory('gradle')
            .childDirectory('wrapper')
            .childFile('gradle-wrapper.properties')
            .existsSync(),
        isTrue,
      );

      expect(
        sampleAppAndroid
            .childDirectory('gradle')
            .childDirectory('wrapper')
            .childFile('gradle-wrapper.properties')
            .readAsStringSync(),
        'distributionBase=GRADLE_USER_HOME\n'
        'distributionPath=wrapper/dists\n'
        'zipStoreBase=GRADLE_USER_HOME\n'
        'zipStorePath=wrapper/dists\n'
        'distributionUrl=https\\://services.gradle.org/distributions/gradle-$templateDefaultGradleVersion-all.zip\n',
      );
    });

    testWithoutContext('injects the wrapper when some files are missing', () {
      final Directory sampleAppAndroid = fileSystem.directory('/sample-app/android');
      sampleAppAndroid.createSync(recursive: true);

      // There's an existing gradlew
      sampleAppAndroid.childFile('gradlew').writeAsStringSync('existing gradlew');

      gradleUtils.injectGradleWrapperIfNeeded(sampleAppAndroid);

      expect(sampleAppAndroid.childFile('gradlew').existsSync(), isTrue);
      expect(sampleAppAndroid.childFile('gradlew').readAsStringSync(), equals('existing gradlew'));

      expect(
        sampleAppAndroid
            .childDirectory('gradle')
            .childDirectory('wrapper')
            .childFile('gradle-wrapper.jar')
            .existsSync(),
        isTrue,
      );

      expect(
        sampleAppAndroid
            .childDirectory('gradle')
            .childDirectory('wrapper')
            .childFile('gradle-wrapper.properties')
            .existsSync(),
        isTrue,
      );

      expect(
        sampleAppAndroid
            .childDirectory('gradle')
            .childDirectory('wrapper')
            .childFile('gradle-wrapper.properties')
            .readAsStringSync(),
        'distributionBase=GRADLE_USER_HOME\n'
        'distributionPath=wrapper/dists\n'
        'zipStoreBase=GRADLE_USER_HOME\n'
        'zipStorePath=wrapper/dists\n'
        'distributionUrl=https\\://services.gradle.org/distributions/gradle-$templateDefaultGradleVersion-all.zip\n',
      );
    });

    testWithoutContext(
      'injects the wrapper and the Gradle version is derived from the AGP version',
      () {
        const Map<String, String> testCases = <String, String>{
          // AGP version : Gradle version
          '1.0.0': '2.3',
          '3.3.1': '4.10.2',
          '3.0.0': '4.1',
          '3.0.5': '4.1',
          '3.0.9': '4.1',
          '3.1.0': '4.4',
          '3.2.0': '4.6',
          '3.3.0': '4.10.2',
          '3.4.0': '5.6.2',
          '3.5.0': '5.6.2',
          '4.0.0': '6.7',
          '4.0.5': '6.7',
          '4.1.0': '6.7',
        };

        for (final MapEntry<String, String> entry in testCases.entries) {
          final Directory sampleAppAndroid = fileSystem.systemTempDirectory.createTempSync(
            'flutter_android.',
          );
          sampleAppAndroid.childFile('build.gradle').writeAsStringSync('''
  buildscript {
      dependencies {
          classpath 'com.android.tools.build:gradle:${entry.key}'
      }
  }
  ''');
          gradleUtils.injectGradleWrapperIfNeeded(sampleAppAndroid);

          expect(sampleAppAndroid.childFile('gradlew').existsSync(), isTrue);

          expect(
            sampleAppAndroid
                .childDirectory('gradle')
                .childDirectory('wrapper')
                .childFile('gradle-wrapper.jar')
                .existsSync(),
            isTrue,
          );

          expect(
            sampleAppAndroid
                .childDirectory('gradle')
                .childDirectory('wrapper')
                .childFile('gradle-wrapper.properties')
                .existsSync(),
            isTrue,
          );

          expect(
            sampleAppAndroid
                .childDirectory('gradle')
                .childDirectory('wrapper')
                .childFile('gradle-wrapper.properties')
                .readAsStringSync(),
            'distributionBase=GRADLE_USER_HOME\n'
            'distributionPath=wrapper/dists\n'
            'zipStoreBase=GRADLE_USER_HOME\n'
            'zipStorePath=wrapper/dists\n'
            'distributionUrl=https\\://services.gradle.org/distributions/gradle-${entry.value}-all.zip\n',
          );
        }
      },
    );

    testWithoutContext('returns the gradlew path', () {
      final Directory androidDirectory = fileSystem.directory('/android')..createSync();
      androidDirectory.childFile('gradlew').createSync();
      androidDirectory.childFile('gradlew.bat').createSync();
      androidDirectory.childFile('gradle.properties').createSync();

      final FlutterProject flutterProject = FlutterProjectFactory(
        logger: BufferLogger.test(),
        fileSystem: fileSystem,
      ).fromDirectory(fileSystem.currentDirectory);

      expect(gradleUtils.getExecutable(flutterProject), androidDirectory.childFile('gradlew').path);
    });
    testWithoutContext('getGradleFileName for notWindows', () {
      expect(getGradlewFileName(notWindowsPlatform), 'gradlew');
    });
    testWithoutContext('getGradleFileName for windows', () {
      expect(getGradlewFileName(windowsPlatform), 'gradlew.bat');
    });

    testWithoutContext('returns the gradle properties file', () async {
      final Directory androidDirectory = fileSystem.directory('/android')..createSync();
      final Directory wrapperDirectory = androidDirectory
        .childDirectory(gradleDirectoryName)
        .childDirectory(gradleWrapperDirectoryName)..createSync(recursive: true);
      final File expectedFile =
          await wrapperDirectory.childFile(gradleWrapperPropertiesFilename).create();
      final File gradleWrapperFile = getGradleWrapperFile(androidDirectory);
      expect(gradleWrapperFile.path, expectedFile.path);
    });

    testWithoutContext('returns the gradle wrapper version', () async {
      const String expectedVersion = '7.4.2';
      final Directory androidDirectory = fileSystem.directory('/android')..createSync();
      final Directory wrapperDirectory = androidDirectory
        .childDirectory('gradle')
        .childDirectory('wrapper')..createSync(recursive: true);
      wrapperDirectory.childFile('gradle-wrapper.properties').writeAsStringSync('''
distributionBase=GRADLE_USER_HOME
distributionPath=wrapper/dists
zipStoreBase=GRADLE_USER_HOME
zipStorePath=wrapper/dists
distributionUrl=https\\://services.gradle.org/distributions/gradle-$expectedVersion-all.zip
''');

      expect(
        await getGradleVersion(androidDirectory, BufferLogger.test(), FakeProcessManager.empty()),
        expectedVersion,
      );
    });

    testWithoutContext('ignores gradle comments', () async {
      const String expectedVersion = '7.4.2';
      final Directory androidDirectory = fileSystem.directory('/android')..createSync();
      final Directory wrapperDirectory = androidDirectory
        .childDirectory('gradle')
        .childDirectory('wrapper')..createSync(recursive: true);
      wrapperDirectory.childFile('gradle-wrapper.properties').writeAsStringSync('''
distributionBase=GRADLE_USER_HOME
distributionPath=wrapper/dists
zipStoreBase=GRADLE_USER_HOME
zipStorePath=wrapper/dists
# distributionUrl=https\\://services.gradle.org/distributions/gradle-8.0.2-all.zip
distributionUrl=https\\://services.gradle.org/distributions/gradle-$expectedVersion-all.zip
# distributionUrl=https\\://services.gradle.org/distributions/gradle-8.0.2-all.zip
''');

      expect(
        await getGradleVersion(androidDirectory, BufferLogger.test(), FakeProcessManager.empty()),
        expectedVersion,
      );
    });

    testWithoutContext('returns gradlew version, whitespace, location', () async {
      const String expectedVersion = '7.4.2';
      final Directory androidDirectory = fileSystem.directory('/android')..createSync();
      final Directory wrapperDirectory = androidDirectory
        .childDirectory('gradle')
        .childDirectory('wrapper')..createSync(recursive: true);
      // Distribution url is not the last line.
      // Whitespace around distribution url.
      wrapperDirectory.childFile('gradle-wrapper.properties').writeAsStringSync('''
distributionBase=GRADLE_USER_HOME
distributionPath=wrapper/dists
distributionUrl = https\\://services.gradle.org/distributions/gradle-$expectedVersion-all.zip
zipStoreBase=GRADLE_USER_HOME
zipStorePath=wrapper/dists
''');

      expect(
        await getGradleVersion(androidDirectory, BufferLogger.test(), FakeProcessManager.empty()),
        expectedVersion,
      );
    });

    testWithoutContext('does not crash on hypothetical new format', () async {
      final Directory androidDirectory = fileSystem.directory('/android')..createSync();
      final Directory wrapperDirectory = androidDirectory
        .childDirectory('gradle')
        .childDirectory('wrapper')..createSync(recursive: true);
      // Distribution url is not the last line.
      // Whitespace around distribution url.
      wrapperDirectory
          .childFile('gradle-wrapper.properties')
          .writeAsStringSync(
            r'distributionUrl=https\://services.gradle.org/distributions/gradle_7.4.2_all.zip',
          );

      // FakeProcessManager.any is used here and not in other getGradleVersion
      // tests because this test does not care about process fallback logic.
      expect(
        await getGradleVersion(androidDirectory, BufferLogger.test(), FakeProcessManager.any()),
        isNull,
      );
    });

    testWithoutContext('returns the installed gradle version', () async {
      const String expectedVersion = '7.4.2';
      const String gradleOutput = '''

------------------------------------------------------------
Gradle $expectedVersion
------------------------------------------------------------

Build time:   2022-03-31 15:25:29 UTC
Revision:     540473b8118064efcc264694cbcaa4b677f61041

Kotlin:       1.5.31
Groovy:       3.0.9
Ant:          Apache Ant(TM) version 1.10.11 compiled on July 10 2021
JVM:          11.0.18 (Azul Systems, Inc. 11.0.18+10-LTS)
OS:           Mac OS X 13.2.1 aarch64
''';
      final Directory androidDirectory = fileSystem.directory('/android')..createSync();
      final ProcessManager processManager =
          FakeProcessManager.empty()..addCommand(
            const FakeCommand(command: <String>['gradle', gradleVersionFlag], stdout: gradleOutput),
          );

      expect(
        await getGradleVersion(androidDirectory, BufferLogger.test(), processManager),
        expectedVersion,
      );
    });

    testWithoutContext('returns the installed gradle with whitespace formatting', () async {
      const String expectedVersion = '7.4.2';
      const String gradleOutput = 'Gradle   $expectedVersion';
      final Directory androidDirectory = fileSystem.directory('/android')..createSync();
      final ProcessManager processManager =
          FakeProcessManager.empty()..addCommand(
            const FakeCommand(command: <String>['gradle', gradleVersionFlag], stdout: gradleOutput),
          );

      expect(
        await getGradleVersion(androidDirectory, BufferLogger.test(), processManager),
        expectedVersion,
      );
    });

    testWithoutContext(
      'returns the AGP version when set in Groovy build file as classpath with single quotes and commented line',
      () async {
        const String expectedVersion = '7.3.0';
        final Directory androidDirectory = fileSystem.directory('/android')..createSync();
        androidDirectory.childFile('build.gradle').writeAsStringSync('''
buildscript {
    repositories {
        google()
        mavenCentral()
    }

    dependencies {
        // Decoy value to ensure we ignore commented out lines.
        // classpath 'com.android.application' version '6.1.0' apply false
        classpath 'com.android.tools.build:gradle:$expectedVersion'
    }
}

allprojects {
    repositories {
        google()
        mavenCentral()
    }
}
''');

        expect(getAgpVersion(androidDirectory, BufferLogger.test()), expectedVersion);
      },
    );

    testWithoutContext(
      'returns the AGP version when set in Kotlin build file as classpath',
      () async {
        const String expectedVersion = '7.3.0';
        final Directory androidDirectory = fileSystem.directory('/android')..createSync();
        androidDirectory.childFile('build.gradle.kts').writeAsStringSync('''
buildscript {
    repositories {
        google()
        mavenCentral()
    }

    dependencies {
        classpath("com.android.tools.build:gradle:$expectedVersion")
    }
}

allprojects {
    repositories {
        google()
        mavenCentral()
    }
}
''');

        expect(getAgpVersion(androidDirectory, BufferLogger.test()), expectedVersion);
      },
    );

    testWithoutContext(
      'returns the AGP version when set in Groovy build file as compileOnly with double quotes',
      () async {
        const String expectedVersion = '7.1.0';
        final Directory androidDirectory = fileSystem.directory('/android')..createSync();
        androidDirectory.childFile('build.gradle.kts').writeAsStringSync('''
dependencies {
    compileOnly "com.android.tools.build:gradle:$expectedVersion"
}
''');

        expect(getAgpVersion(androidDirectory, BufferLogger.test()), expectedVersion);
      },
    );
    testWithoutContext(
      'returns the AGP version when set in Kotlin build file as compileOnly',
      () async {
        const String expectedVersion = '7.1.0';
        final Directory androidDirectory = fileSystem.directory('/android')..createSync();
        androidDirectory.childFile('build.gradle.kts').writeAsStringSync('''
dependencies {
    compileOnly("com.android.tools.build:gradle:$expectedVersion")
}
''');

        expect(getAgpVersion(androidDirectory, BufferLogger.test()), expectedVersion);
      },
    );
    testWithoutContext('returns the AGP version when set in Groovy build file as plugin', () async {
      const String expectedVersion = '6.8';
      final Directory androidDirectory = fileSystem.directory('/android')..createSync();
      androidDirectory.childFile('build.gradle').writeAsStringSync('''
plugins {
    id 'com.android.application' version '$expectedVersion' apply false
}
      ''');
      expect(getAgpVersion(androidDirectory, BufferLogger.test()), expectedVersion);
    });

    testWithoutContext('returns the AGP version when set in Kotlin build file as plugin', () async {
      const String expectedVersion = '7.2.0';
      final Directory androidDirectory = fileSystem.directory('/android')..createSync();
      androidDirectory.childFile('build.gradle.kts').writeAsStringSync('''
plugins {
    id("com.android.application") version "$expectedVersion" apply false
}
      ''');
      expect(getAgpVersion(androidDirectory, BufferLogger.test()), expectedVersion);
    });

    testWithoutContext('prefers the AGP version when set in Groovy, ignores Kotlin', () async {
      const String versionInGroovy = '7.3.0';
      const String versionInKotlin = '7.4.2';
      final Directory androidDirectory = fileSystem.directory('/android')..createSync();

      androidDirectory.childFile('build.gradle').writeAsStringSync('''
buildscript {
    repositories {
        google()
        mavenCentral()
    }

    dependencies {
        classpath 'com.android.tools.build:gradle:$versionInGroovy'
    }
}

allprojects {
    repositories {
        google()
        mavenCentral()
    }
}
''');

      androidDirectory.childFile('build.gradle.kts').writeAsStringSync('''
buildscript {
    repositories {
        google()
        mavenCentral()
    }

    dependencies {
        classpath("com.android.tools.build:gradle:$versionInKotlin")
    }
}

allprojects {
    repositories {
        google()
        mavenCentral()
    }
}
''');

      expect(getAgpVersion(androidDirectory, BufferLogger.test()), versionInGroovy);
    });

    testWithoutContext('returns null when AGP version not set', () async {
      final Directory androidDirectory = fileSystem.directory('/android')..createSync();
      androidDirectory.childFile('build.gradle').writeAsStringSync('''
buildscript {
    repositories {
        google()
        mavenCentral()
    }
    dependencies {
    }
}
allprojects {
    repositories {
        google()
        mavenCentral()
    }
}
''');

      expect(getAgpVersion(androidDirectory, BufferLogger.test()), null);
    });
    testWithoutContext('returns the AGP version when beta', () async {
      final Directory androidDirectory = fileSystem.directory('/android')..createSync();
      androidDirectory.childFile('build.gradle').writeAsStringSync(r'''
buildscript {
    repositories {
        google()
        mavenCentral()
    }

    dependencies {
        classpath 'com.android.tools.build:gradle:7.3.0-beta03'
    }
}

allprojects {
    repositories {
        google()
        mavenCentral()
    }
}
''');

      expect(getAgpVersion(androidDirectory, BufferLogger.test()), '7.3.0');
    });

    testWithoutContext('returns the AGP version when in Groovy settings as plugin', () async {
      final Directory androidDirectory = fileSystem.directory('/android')..createSync();
      // File must exist and can not have agp defined.
      androidDirectory.childFile('build.gradle').writeAsStringSync(r'');
      androidDirectory.childFile('settings.gradle').writeAsStringSync(r'''
pluginManagement {
    plugins {
        id 'dev.flutter.flutter-gradle-plugin' version '1.0.0' apply false
        id 'dev.flutter.flutter-plugin-loader' version '1.0.0'
        // Decoy value to ensure we ignore commented out lines.
        // id 'com.android.application' version '6.1.0' apply false
        id 'com.android.application' version '8.1.0' apply false
    }
}
''');

      expect(getAgpVersion(androidDirectory, BufferLogger.test()), '8.1.0');
    });

    testWithoutContext('returns the AGP version when in Kotlin settings as plugin', () async {
      final Directory androidDirectory = fileSystem.directory('/android')..createSync();
      // File must exist and cannot have agp defined.
      androidDirectory.childFile('build.gradle.kts').writeAsStringSync(r'');
      androidDirectory.childFile('settings.gradle.kts').writeAsStringSync(r'''
pluginManagement {
  plugins {
      id("dev.flutter.flutter-plugin-loader") version "1.0.0"
      // Decoy value to ensure we ignore commented out lines.
      // id("com.android.application") version "6.1.0" apply false
      id("com.android.application") version "7.5.0" apply false
  }
}
''');

      expect(getAgpVersion(androidDirectory, BufferLogger.test()), '7.5.0');
    });

    testWithoutContext('returns null when agp version is misconfigured', () async {
      final Directory androidDirectory = fileSystem.directory('/android')..createSync();
      androidDirectory.childFile('build.gradle.kts').writeAsStringSync('''
plugins {
    `java-gradle-plugin`
    `groovy`
}

dependencies {
    // intentional typo
    compileOnl("com.android.tools.build:gradle:7.3.0")
}
''');

      expect(getAgpVersion(androidDirectory, BufferLogger.test()), null);
    });

    group('validates gradle/agp versions', () {
      final List<GradleAgpTestData> testData = <GradleAgpTestData>[
        // Values too new *these need to be updated* when
        // max known gradle and max known agp versions are updated:
        // Newer tools version supports max gradle version.
        GradleAgpTestData(true, agpVersion: '8.2', gradleVersion: '8.0'),
        // Newer tools version does not even meet current gradle version requirements.
        GradleAgpTestData(false, agpVersion: '8.2', gradleVersion: '7.3'),
        // Newer tools version requires newer gradle version.
        GradleAgpTestData(true, agpVersion: '8.3', gradleVersion: '8.1'),

        // Template versions of Gradle/AGP.
        GradleAgpTestData(
          true,
          agpVersion: templateAndroidGradlePluginVersion,
          gradleVersion: templateDefaultGradleVersion,
        ),
        GradleAgpTestData(
          true,
          agpVersion: templateAndroidGradlePluginVersionForModule,
          gradleVersion: templateDefaultGradleVersion,
        ),

        // Minimums as defined in
        // https://developer.android.com/studio/releases/gradle-plugin#updating-gradle
        GradleAgpTestData(true, agpVersion: '8.1', gradleVersion: '8.0'),
        GradleAgpTestData(true, agpVersion: '8.0', gradleVersion: '8.0'),
        GradleAgpTestData(true, agpVersion: '7.4', gradleVersion: '7.5'),
        GradleAgpTestData(true, agpVersion: '7.3', gradleVersion: '7.4'),
        GradleAgpTestData(true, agpVersion: '7.2', gradleVersion: '7.3.3'),
        GradleAgpTestData(true, agpVersion: '7.1', gradleVersion: '7.2'),
        GradleAgpTestData(true, agpVersion: '7.0', gradleVersion: '7.0'),
        GradleAgpTestData(true, agpVersion: '4.2.0', gradleVersion: '6.7.1'),
        GradleAgpTestData(true, agpVersion: '4.1.0', gradleVersion: '6.5'),
        GradleAgpTestData(true, agpVersion: '4.0.0', gradleVersion: '6.1.1'),
        GradleAgpTestData(true, agpVersion: '3.6.0', gradleVersion: '5.6.4'),
        GradleAgpTestData(true, agpVersion: '3.5.0', gradleVersion: '5.4.1'),
        GradleAgpTestData(true, agpVersion: '3.4.0', gradleVersion: '5.1.1'),
        GradleAgpTestData(true, agpVersion: '3.3.0', gradleVersion: '4.10.1'),
        // Values too old:
        GradleAgpTestData(false, agpVersion: '3.3.0', gradleVersion: '4.9'),
        GradleAgpTestData(false, agpVersion: '7.3', gradleVersion: '7.2'),
        GradleAgpTestData(false, agpVersion: '3.0.0', gradleVersion: '7.2'),
        // Null values:
        // ignore: avoid_redundant_argument_values
        GradleAgpTestData(false, agpVersion: null, gradleVersion: '7.2'),
        // ignore: avoid_redundant_argument_values
        GradleAgpTestData(false, agpVersion: '3.0.0', gradleVersion: null),
        // ignore: avoid_redundant_argument_values
        GradleAgpTestData(false, agpVersion: null, gradleVersion: null),
        // Middle AGP cases:
        GradleAgpTestData(true, agpVersion: '8.0.1', gradleVersion: '8.0'),
        GradleAgpTestData(true, agpVersion: '7.4.1', gradleVersion: '7.5'),
        GradleAgpTestData(true, agpVersion: '7.3.1', gradleVersion: '7.4'),
        GradleAgpTestData(true, agpVersion: '7.2.1', gradleVersion: '7.3.3'),
        GradleAgpTestData(true, agpVersion: '7.1.1', gradleVersion: '7.2'),
        GradleAgpTestData(true, agpVersion: '7.0.1', gradleVersion: '7.0'),
        GradleAgpTestData(true, agpVersion: '4.2.1', gradleVersion: '6.7.1'),
        GradleAgpTestData(true, agpVersion: '4.1.1', gradleVersion: '6.5'),
        GradleAgpTestData(true, agpVersion: '4.0.1', gradleVersion: '6.1.1'),
        GradleAgpTestData(true, agpVersion: '3.6.1', gradleVersion: '5.6.4'),
        GradleAgpTestData(true, agpVersion: '3.5.1', gradleVersion: '5.4.1'),
        GradleAgpTestData(true, agpVersion: '3.4.1', gradleVersion: '5.1.1'),
        GradleAgpTestData(true, agpVersion: '3.3.1', gradleVersion: '4.10.1'),

        // Higher gradle cases:
        GradleAgpTestData(true, agpVersion: '7.4', gradleVersion: '8.0'),
        GradleAgpTestData(true, agpVersion: '7.3', gradleVersion: '7.5'),
        GradleAgpTestData(true, agpVersion: '7.2', gradleVersion: '7.4'),
        GradleAgpTestData(true, agpVersion: '7.1', gradleVersion: '7.3.3'),
        GradleAgpTestData(true, agpVersion: '7.0', gradleVersion: '7.2'),
        GradleAgpTestData(true, agpVersion: '4.2.0', gradleVersion: '7.0'),
        GradleAgpTestData(true, agpVersion: '4.1.0', gradleVersion: '6.7.1'),
        GradleAgpTestData(true, agpVersion: '4.0.0', gradleVersion: '6.5'),
        GradleAgpTestData(true, agpVersion: '3.6.0', gradleVersion: '6.1.1'),
        GradleAgpTestData(true, agpVersion: '3.5.0', gradleVersion: '5.6.4'),
        GradleAgpTestData(true, agpVersion: '3.4.0', gradleVersion: '5.4.1'),
        GradleAgpTestData(true, agpVersion: '3.3.0', gradleVersion: '5.1.1'),
      ];
      for (final GradleAgpTestData data in testData) {
        test('(gradle, agp): (${data.gradleVersion}, ${data.agpVersion})', () {
          expect(
            validateGradleAndAgp(
              BufferLogger.test(),
              gradleV: data.gradleVersion,
              agpV: data.agpVersion,
            ),
            data.validPair ? isTrue : isFalse,
            reason: 'G: ${data.gradleVersion}, AGP: ${data.agpVersion}',
          );
        });
      }
    });

    group('Parse gradle version from distribution url', () {
      testWithoutContext('null distribution url returns null version', () {
        expect(parseGradleVersionFromDistributionUrl(null), null);
      });

      testWithoutContext('unparseable format returns null', () {
        const String distributionUrl = 'aString';
        expect(parseGradleVersionFromDistributionUrl(distributionUrl), null);
      });

      testWithoutContext("recognizable 'all' format returns correct version", () {
        const String distributionUrl =
            r'distributionUrl=https\://services.gradle.org/distributions/gradle-6.7-all.zip';
        expect(parseGradleVersionFromDistributionUrl(distributionUrl), '6.7');
      });

      testWithoutContext("recognizable 'bin' format returns correct version", () {
        const String distributionUrl =
            r'distributionUrl=https\://services.gradle.org/distributions/gradle-6.7-bin.zip';
        expect(parseGradleVersionFromDistributionUrl(distributionUrl), '6.7');
      });

      testWithoutContext("recognizable 'rc' format returns correct version", () {
        const String distributionUrl =
            r'distributionUrl=https\://services.gradle.org/distributions/gradle-8.1-rc-3-all.zip';
        expect(parseGradleVersionFromDistributionUrl(distributionUrl), '8.1');
      });
    });

    group('validates java/gradle versions', () {
      final List<JavaGradleTestData> testData = <JavaGradleTestData>[
        // Values too new *these need to be updated* when
        // max supported java and max known gradle versions are updated:
        // Newer tools version does not even meet current gradle version requirements.
        JavaGradleTestData(false, javaVersion: '20', gradleVersion: '7.5'),
        // Newer tools version requires newer gradle version.
        JavaGradleTestData(true, javaVersion: '20', gradleVersion: '8.1'),
        // Max known unsupported Java version.
        JavaGradleTestData(
          true,
          javaVersion: '24',
          gradleVersion: maxKnownAndSupportedGradleVersion,
        ),
        // Minimums as defined in
        // https://docs.gradle.org/current/userguide/compatibility.html#java
        JavaGradleTestData(true, javaVersion: '19', gradleVersion: '7.6'),
        JavaGradleTestData(true, javaVersion: '18', gradleVersion: '7.5'),
        JavaGradleTestData(true, javaVersion: '17', gradleVersion: '7.3'),
        JavaGradleTestData(true, javaVersion: '16', gradleVersion: '7.0'),
        JavaGradleTestData(true, javaVersion: '15', gradleVersion: '6.7'),
        JavaGradleTestData(true, javaVersion: '14', gradleVersion: '6.3'),
        JavaGradleTestData(true, javaVersion: '13', gradleVersion: '6.0'),
        JavaGradleTestData(true, javaVersion: '12', gradleVersion: '5.4'),
        JavaGradleTestData(true, javaVersion: '11', gradleVersion: '5.0'),
        JavaGradleTestData(true, javaVersion: '1.10', gradleVersion: '4.7'),
        JavaGradleTestData(true, javaVersion: '1.9', gradleVersion: '4.3'),
        JavaGradleTestData(true, javaVersion: '1.8', gradleVersion: '2.0'),
        // Gradle too old for Java version.
        JavaGradleTestData(false, javaVersion: '19', gradleVersion: '6.7'),
        JavaGradleTestData(false, javaVersion: '11', gradleVersion: '4.10.1'),
        JavaGradleTestData(false, javaVersion: '1.9', gradleVersion: '4.1'),
        // Null values:
        // ignore: avoid_redundant_argument_values
        JavaGradleTestData(false, javaVersion: null, gradleVersion: '7.2'),
        // ignore: avoid_redundant_argument_values
        JavaGradleTestData(false, javaVersion: '11', gradleVersion: null),
        // ignore: avoid_redundant_argument_values
        JavaGradleTestData(false, javaVersion: null, gradleVersion: null),
        // Middle Java cases:
        // https://www.java.com/releases/
        JavaGradleTestData(true, javaVersion: '19.0.2', gradleVersion: '8.0.2'),
        JavaGradleTestData(true, javaVersion: '19.0.2', gradleVersion: '8.0.0'),
        JavaGradleTestData(true, javaVersion: '18.0.2', gradleVersion: '8.0.2'),
        JavaGradleTestData(true, javaVersion: '17.0.3', gradleVersion: '7.5'),
        JavaGradleTestData(true, javaVersion: '16.0.1', gradleVersion: '7.3'),
        JavaGradleTestData(true, javaVersion: '15.0.2', gradleVersion: '7.3'),
        JavaGradleTestData(true, javaVersion: '14.0.1', gradleVersion: '7.0'),
        JavaGradleTestData(true, javaVersion: '13.0.2', gradleVersion: '6.7'),
        JavaGradleTestData(true, javaVersion: '12.0.2', gradleVersion: '6.3'),
        JavaGradleTestData(true, javaVersion: '11.0.18', gradleVersion: '6.0'),
        // Higher gradle cases:
        JavaGradleTestData(true, javaVersion: '19', gradleVersion: '8.0'),
        JavaGradleTestData(true, javaVersion: '18', gradleVersion: '8.0'),
        JavaGradleTestData(true, javaVersion: '17', gradleVersion: '7.5'),
        JavaGradleTestData(true, javaVersion: '16', gradleVersion: '7.3'),
        JavaGradleTestData(true, javaVersion: '15', gradleVersion: '7.3'),
        JavaGradleTestData(true, javaVersion: '14', gradleVersion: '7.0'),
        JavaGradleTestData(true, javaVersion: '13', gradleVersion: '6.7'),
        JavaGradleTestData(true, javaVersion: '12', gradleVersion: '6.3'),
        JavaGradleTestData(true, javaVersion: '11', gradleVersion: '6.0'),
        JavaGradleTestData(true, javaVersion: '1.10', gradleVersion: '5.4'),
        JavaGradleTestData(true, javaVersion: '1.9', gradleVersion: '5.0'),
        JavaGradleTestData(true, javaVersion: '1.8', gradleVersion: '4.3'),
      ];

      for (final JavaGradleTestData data in testData) {
        testWithoutContext('(Java, gradle): (${data.javaVersion}, ${data.gradleVersion})', () {
          expect(
            validateJavaAndGradle(
              BufferLogger.test(),
              javaV: data.javaVersion,
              gradleV: data.gradleVersion,
            ),
            data.validPair ? isTrue : isFalse,
            reason: 'J: ${data.javaVersion}, G: ${data.gradleVersion}',
          );
        });
      }
    });
  });

  group('getGradleVersionForAndroidPlugin', () {
    late FileSystem fileSystem;
    late Logger testLogger;

    setUp(() {
      fileSystem = MemoryFileSystem.test();
      testLogger = BufferLogger.test();
    });

    testWithoutContext('prefers build.gradle over build.gradle.kts', () async {
      const String versionInGroovy = '4.0.0';
      const String versionInKotlin = '7.4.2';
      final Directory androidDirectory = fileSystem.directory('/android')..createSync();

      androidDirectory.childFile('build.gradle').writeAsStringSync('''
buildscript {
    repositories {
        google()
        mavenCentral()
    }

    dependencies {
        classpath 'com.android.tools.build:gradle:$versionInGroovy'
    }
}

allprojects {
    repositories {
        google()
        mavenCentral()
    }
}
''');

      androidDirectory.childFile('build.gradle.kts').writeAsStringSync('''
buildscript {
    repositories {
        google()
        mavenCentral()
    }

    dependencies {
        classpath("com.android.tools.build:gradle:$versionInKotlin")
    }
}

allprojects {
    repositories {
        google()
        mavenCentral()
    }
}
''');

      expect(
        getGradleVersionForAndroidPlugin(androidDirectory, testLogger),
        '6.7', // as per compatibility matrix in gradle_utils.dart
      );
    });
  });

  group('validates java/AGP versions', () {
    final List<JavaAgpTestData> testData = <JavaAgpTestData>[
      // Strictly too old Java versions for known AGP versions.
      JavaAgpTestData(false, javaVersion: '1.6', agpVersion: maxKnownAgpVersion),
      JavaAgpTestData(false, javaVersion: '1.6', agpVersion: maxKnownAndSupportedAgpVersion),
      JavaAgpTestData(false, javaVersion: '1.6', agpVersion: '4.2'),
      // Strictly too old AGP versions.
      JavaAgpTestData(false, javaVersion: '1.8', agpVersion: '1.0'),
      JavaAgpTestData(false, javaVersion: '1.8', agpVersion: '4.1'),
      JavaAgpTestData(false, javaVersion: '1.8', agpVersion: '2.3'),
      // Strictly too new Java versions for defined AGP versions.
      JavaAgpTestData(true, javaVersion: '18', agpVersion: '8.1'),
      JavaAgpTestData(true, javaVersion: '18', agpVersion: '7.4'),
      JavaAgpTestData(true, javaVersion: '18', agpVersion: '4.2'),
      // Strictly too new AGP versions.
      // *The tests that follow need to be updated* when max supported AGP versions are updated:
      JavaAgpTestData(false, javaVersion: '24', agpVersion: '8.8'),
      JavaAgpTestData(false, javaVersion: '20', agpVersion: '8.8'),
      JavaAgpTestData(false, javaVersion: '17', agpVersion: '8.8'),
      // Java 17 & patch versions compatibility cases
      // *The tests that follow need to be updated* when maxKnownAndSupportedAgpVersion is
      // updated:
      JavaAgpTestData(false, javaVersion: '17', agpVersion: '8.8'),
      JavaAgpTestData(true, javaVersion: '17', agpVersion: maxKnownAndSupportedAgpVersion),
      JavaAgpTestData(true, javaVersion: '17', agpVersion: '8.7'),
      JavaAgpTestData(true, javaVersion: '17', agpVersion: '8.6'),
      JavaAgpTestData(true, javaVersion: '17', agpVersion: '8.5'),
      JavaAgpTestData(true, javaVersion: '17', agpVersion: '8.4'),
      JavaAgpTestData(true, javaVersion: '17', agpVersion: '8.3'),
      JavaAgpTestData(true, javaVersion: '17', agpVersion: '8.1'),
      JavaAgpTestData(true, javaVersion: '17', agpVersion: '8.0'),
      JavaAgpTestData(true, javaVersion: '17', agpVersion: '7.4'),
      JavaAgpTestData(false, javaVersion: '17.0.3', agpVersion: '8.8'),
      JavaAgpTestData(true, javaVersion: '17.0.3', agpVersion: maxKnownAndSupportedAgpVersion),
      JavaAgpTestData(true, javaVersion: '17.0.3', agpVersion: '8.7'),
      JavaAgpTestData(true, javaVersion: '17.0.3', agpVersion: '8.6'),
      JavaAgpTestData(true, javaVersion: '17.0.3', agpVersion: '8.5'),
      JavaAgpTestData(true, javaVersion: '17.0.3', agpVersion: '8.4'),
      JavaAgpTestData(true, javaVersion: '17.0.3', agpVersion: '8.3'),
      JavaAgpTestData(true, javaVersion: '17.0.3', agpVersion: '8.2'),
      JavaAgpTestData(true, javaVersion: '17.0.3', agpVersion: '8.1'),
      JavaAgpTestData(true, javaVersion: '17.0.3', agpVersion: '8.0'),
      JavaAgpTestData(true, javaVersion: '17.0.3', agpVersion: '7.4'),
      // Java 11 & patch versions compatibility cases
      JavaAgpTestData(false, javaVersion: '11', agpVersion: '8.0'),
      JavaAgpTestData(true, javaVersion: '11', agpVersion: '7.4'),
      JavaAgpTestData(true, javaVersion: '11', agpVersion: '7.2'),
      JavaAgpTestData(true, javaVersion: '11', agpVersion: '7.0'),
      JavaAgpTestData(true, javaVersion: '11', agpVersion: '4.2'),
      JavaAgpTestData(false, javaVersion: '11.0.18', agpVersion: '8.0'),
      JavaAgpTestData(true, javaVersion: '11.0.18', agpVersion: '7.4'),
      JavaAgpTestData(true, javaVersion: '11.0.18', agpVersion: '7.2'),
      JavaAgpTestData(true, javaVersion: '11.0.18', agpVersion: '7.0'),
      JavaAgpTestData(true, javaVersion: '11.0.18', agpVersion: '4.2'),
      // Java 8 compatibility cases
      JavaAgpTestData(false, javaVersion: '1.8', agpVersion: '7.0'),
      JavaAgpTestData(
        true,
        javaVersion: '1.8',
        agpVersion: oldestDocumentedJavaAgpCompatibilityVersion,
      ), // agpVersion = 4.2
      JavaAgpTestData(false, javaVersion: '1.8', agpVersion: '4.1'),
      // Null value cases
      // ignore: avoid_redundant_argument_values
      JavaAgpTestData(false, javaVersion: null, agpVersion: '4.2'),
      // ignore: avoid_redundant_argument_values
      JavaAgpTestData(false, javaVersion: '1.8', agpVersion: null),
      // ignore: avoid_redundant_argument_values
      JavaAgpTestData(false, javaVersion: null, agpVersion: null),
    ];

    for (final JavaAgpTestData data in testData) {
      testWithoutContext('(Java, agp): (${data.javaVersion}, ${data.agpVersion})', () {
        expect(
          validateJavaAndAgp(BufferLogger.test(), javaV: data.javaVersion, agpV: data.agpVersion),
          data.validPair ? isTrue : isFalse,
          reason: 'J: ${data.javaVersion}, G: ${data.agpVersion}',
        );
      });
    }
  });

  group('detecting valid Gradle/AGP versions for given Java version and vice versa', () {
    testWithoutContext(
      'getValidGradleVersionRangeForJavaVersion returns valid Gradle version range for Java version',
      () {
        final Logger testLogger = BufferLogger.test();
        // Java version too high.
        expect(
          getValidGradleVersionRangeForJavaVersion(
            testLogger,
            javaV: oneMajorVersionHigherJavaVersion,
          ),
          isNull,
        );
        // Maximum known Java version.
        // *The test case that follows needs to be updated* when higher versions of Java are supported:
        expect(
          getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '24'),
          allOf(
            equals(getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '24.0.2')),
            isNull,
          ),
        );
        expect(
          getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '23'),
          allOf(
            equals(getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '23.0.2')),
            equals(
              const JavaGradleCompat(
                javaMin: '23',
                javaMax: '24',
                gradleMin: '8.10',
                gradleMax: maxKnownAndSupportedGradleVersion,
              ),
            ),
          ),
        );
        // Known supported Java versions.
        expect(
          getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '22'),
          allOf(
            equals(getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '22.0.2')),
            equals(
              const JavaGradleCompat(
                javaMin: '22',
                javaMax: '23',
                gradleMin: '8.7',
                gradleMax: maxKnownAndSupportedGradleVersion,
              ),
            ),
          ),
        );
        expect(
          getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '21'),
          allOf(
            equals(getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '21.0.2')),
            equals(
              const JavaGradleCompat(
                javaMin: '21',
                javaMax: '22',
                gradleMin: '8.4',
                gradleMax: maxKnownAndSupportedGradleVersion,
              ),
            ),
          ),
        );
        expect(
          getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '20'),
          allOf(
            equals(getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '20.0.2')),
            equals(
              const JavaGradleCompat(
                javaMin: '20',
                javaMax: '21',
                gradleMin: '8.1',
                gradleMax: maxKnownAndSupportedGradleVersion,
              ),
            ),
          ),
        );
        expect(
          getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '19'),
          allOf(
            equals(getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '19.0.2')),
            equals(
              const JavaGradleCompat(
                javaMin: '19',
                javaMax: '20',
                gradleMin: '7.6',
                gradleMax: maxKnownAndSupportedGradleVersion,
              ),
            ),
          ),
        );
        expect(
          getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '18'),
          allOf(
            equals(getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '18.0.2')),
            equals(
              const JavaGradleCompat(
                javaMin: '18',
                javaMax: '19',
                gradleMin: '7.5',
                gradleMax: maxKnownAndSupportedGradleVersion,
              ),
            ),
          ),
        );
        expect(
          getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '17'),
          allOf(
            equals(getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '17.0.2')),
            equals(
              const JavaGradleCompat(
                javaMin: '17',
                javaMax: '18',
                gradleMin: '7.3',
                gradleMax: maxKnownAndSupportedGradleVersion,
              ),
            ),
          ),
        );
        expect(
          getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '16'),
          allOf(
            equals(getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '16.0.2')),
            equals(
              const JavaGradleCompat(
                javaMin: '16',
                javaMax: '17',
                gradleMin: '7.0',
                gradleMax: maxKnownAndSupportedGradleVersion,
              ),
            ),
          ),
        );
        expect(
          getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '15'),
          allOf(
            equals(getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '15.0.2')),
            equals(
              const JavaGradleCompat(
                javaMin: '15',
                javaMax: '16',
                gradleMin: '6.7',
                gradleMax: maxKnownAndSupportedGradleVersion,
              ),
            ),
          ),
        );
        expect(
          getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '14'),
          allOf(
            equals(getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '14.0.2')),
            equals(
              const JavaGradleCompat(
                javaMin: '14',
                javaMax: '15',
                gradleMin: '6.3',
                gradleMax: maxKnownAndSupportedGradleVersion,
              ),
            ),
          ),
        );
        expect(
          getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '13'),
          allOf(
            equals(getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '13.0.2')),
            equals(
              const JavaGradleCompat(
                javaMin: '13',
                javaMax: '14',
                gradleMin: '6.0',
                gradleMax: maxKnownAndSupportedGradleVersion,
              ),
            ),
          ),
        );
        expect(
          getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '12'),
          allOf(
            equals(getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '12.0.2')),
            equals(
              const JavaGradleCompat(
                javaMin: '12',
                javaMax: '13',
                gradleMin: '5.4',
                gradleMax: maxKnownAndSupportedGradleVersion,
              ),
            ),
          ),
        );
        expect(
          getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '11'),
          allOf(
            equals(getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '11.0.2')),
            equals(
              const JavaGradleCompat(
                javaMin: '11',
                javaMax: '12',
                gradleMin: '5.0',
                gradleMax: maxKnownAndSupportedGradleVersion,
              ),
            ),
          ),
        );
        expect(
          getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '1.10'),
          allOf(
            equals(getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '1.10.2')),
            equals(
              const JavaGradleCompat(
                javaMin: '1.10',
                javaMax: '1.11',
                gradleMin: '4.7',
                gradleMax: maxKnownAndSupportedGradleVersion,
              ),
            ),
          ),
        );
        expect(
          getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '1.9'),
          allOf(
            equals(getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '1.9.2')),
            equals(
              const JavaGradleCompat(
                javaMin: '1.9',
                javaMax: '1.10',
                gradleMin: '4.3',
                gradleMax: maxKnownAndSupportedGradleVersion,
              ),
            ),
          ),
        );
        // Java 1.8 -- return oldest documented compatibility info
        expect(
          getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '1.8'),
          allOf(
            equals(getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '1.8.2')),
            equals(
              const JavaGradleCompat(
                javaMin: '1.8',
                javaMax: '1.9',
                gradleMin: '2.0',
                gradleMax: maxKnownAndSupportedGradleVersion,
              ),
            ),
          ),
        );
        // Java version too low.
        expect(
          getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '1.7'),
          allOf(
            equals(getValidGradleVersionRangeForJavaVersion(testLogger, javaV: '1.7.2')),
            isNull,
          ),
        );
      },
    );

    testWithoutContext(
      'getMinimumAgpVersionForJavaVersion returns minimum AGP version for Java version',
      () {
        final Logger testLogger = BufferLogger.test();
        // Maximum known Java version.
        // *The test case that follows needs to be updated* as higher versions of AGP are supported:
        expect(
          getMinimumAgpVersionForJavaVersion(testLogger, javaV: oneMajorVersionHigherJavaVersion),
          equals(
            const JavaAgpCompat(
              javaMin: '17',
              javaDefault: '17',
              agpMin: '8.0',
              agpMax: maxKnownAndSupportedAgpVersion,
            ),
          ),
        );
        // Known Java versions.
        expect(
          getMinimumAgpVersionForJavaVersion(testLogger, javaV: '17'),
          allOf(
            equals(getMinimumAgpVersionForJavaVersion(testLogger, javaV: '17.0.2')),
            equals(
              const JavaAgpCompat(
                javaMin: '17',
                javaDefault: '17',
                agpMin: '8.0',
                agpMax: maxKnownAndSupportedAgpVersion,
              ),
            ),
          ),
        );
        expect(
          getMinimumAgpVersionForJavaVersion(testLogger, javaV: '15'),
          allOf(
            equals(getMinimumAgpVersionForJavaVersion(testLogger, javaV: '15.0.2')),
            equals(
              const JavaAgpCompat(javaMin: '11', javaDefault: '11', agpMin: '7.0', agpMax: '7.4'),
            ),
          ),
        );
        expect(
          getMinimumAgpVersionForJavaVersion(testLogger, javaV: '11'),
          allOf(
            equals(getMinimumAgpVersionForJavaVersion(testLogger, javaV: '11.0.2')),
            equals(
              const JavaAgpCompat(javaMin: '11', javaDefault: '11', agpMin: '7.0', agpMax: '7.4'),
            ),
          ),
        );
        expect(
          getMinimumAgpVersionForJavaVersion(testLogger, javaV: '1.9'),
          allOf(
            equals(getMinimumAgpVersionForJavaVersion(testLogger, javaV: '1.9.2')),
            equals(
              const JavaAgpCompat(javaMin: '1.8', javaDefault: '1.8', agpMin: '4.2', agpMax: '4.2'),
            ),
          ),
        );
        expect(
          getMinimumAgpVersionForJavaVersion(testLogger, javaV: '1.8'),
          allOf(
            equals(getMinimumAgpVersionForJavaVersion(testLogger, javaV: '1.8.2')),
            equals(
              const JavaAgpCompat(javaMin: '1.8', javaDefault: '1.8', agpMin: '4.2', agpMax: '4.2'),
            ),
          ),
        );
        // Java version too low.
        expect(
          getMinimumAgpVersionForJavaVersion(testLogger, javaV: '1.7'),
          allOf(equals(getMinimumAgpVersionForJavaVersion(testLogger, javaV: '1.7.2')), isNull),
        );
      },
    );

    testWithoutContext('getJavaVersionFor returns expected Java version range', () {
      // Strictly too old Gradle and AGP versions.
      expect(
        getJavaVersionFor(gradleV: '1.9', agpV: '4.1'),
        equals(const VersionRange(null, null)),
      );
      // Strictly too old Gradle or AGP version.
      expect(
        getJavaVersionFor(gradleV: '1.9', agpV: '4.2'),
        equals(const VersionRange('1.8', null)),
      );
      expect(
        getJavaVersionFor(gradleV: '2.0', agpV: '4.1'),
        equals(const VersionRange(null, '1.9')),
      );
      // Strictly too new Gradle and AGP versions.
      expect(
        getJavaVersionFor(gradleV: '8.11', agpV: '8.8'),
        equals(const VersionRange(null, null)),
      );
      // Strictly too new Gradle version and maximum version of AGP.
      //*This test case will need its expected Java range updated when a new version of AGP is supported.*
      expect(
        getJavaVersionFor(gradleV: '8.11', agpV: maxKnownAndSupportedAgpVersion),
        equals(const VersionRange('17', null)),
      );
      // Strictly too new AGP version and maximum version of Gradle.
      //*This test case will need its expected Java range updated when a new version of Gradle is supported.*
      expect(
        getJavaVersionFor(gradleV: maxKnownAndSupportedGradleVersion, agpV: '8.8'),
        equals(const VersionRange(null, '24')),
      );
      // Tests with a known compatible Gradle/AGP version pair.
      expect(
        getJavaVersionFor(gradleV: '7.0', agpV: '7.2'),
        equals(const VersionRange('11', '17')),
      );
      expect(
        getJavaVersionFor(gradleV: '7.1', agpV: '7.2'),
        equals(const VersionRange('11', '17')),
      );
      expect(
        getJavaVersionFor(gradleV: '7.2.2', agpV: '7.2'),
        equals(const VersionRange('11', '17')),
      );
      expect(
        getJavaVersionFor(gradleV: '7.1', agpV: '7.0'),
        equals(const VersionRange('11', '17')),
      );
      expect(
        getJavaVersionFor(gradleV: '7.1', agpV: '7.2'),
        equals(const VersionRange('11', '17')),
      );
      expect(
        getJavaVersionFor(gradleV: '7.1', agpV: '7.4'),
        equals(const VersionRange('11', '17')),
      );
      expect(
        getJavaVersionFor(gradleV: '8.4', agpV: '8.1'),
        equals(const VersionRange('17', '22')),
      );
      expect(
        getJavaVersionFor(gradleV: '8.7', agpV: '8.1'),
        equals(const VersionRange('17', '23')),
      );
    });
  });
}

class GradleAgpTestData {
  GradleAgpTestData(this.validPair, {this.gradleVersion, this.agpVersion});
  final String? gradleVersion;
  final String? agpVersion;
  final bool validPair;
}

class JavaGradleTestData {
  JavaGradleTestData(this.validPair, {this.javaVersion, this.gradleVersion});
  final String? gradleVersion;
  final String? javaVersion;
  final bool validPair;
}

class JavaAgpTestData {
  JavaAgpTestData(this.validPair, {this.javaVersion, this.agpVersion});
  final String? agpVersion;
  final String? javaVersion;
  final bool validPair;
}

final Platform windowsPlatform = FakePlatform(
  operatingSystem: 'windows',
  environment: <String, String>{
    'PROGRAMFILES(X86)': r'C:\Program Files (x86)\',
    'FLUTTER_ROOT': r'C:\flutter',
    'USERPROFILE': '/',
  },
);
final Platform notWindowsPlatform = FakePlatform(
  environment: <String, String>{'FLUTTER_ROOT': r'/users/someuser/flutter'},
);
