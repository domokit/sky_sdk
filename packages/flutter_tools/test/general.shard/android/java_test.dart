// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/android/android_studio.dart';
import 'package:flutter_tools/src/android/java.dart';
import 'package:flutter_tools/src/base/config.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/os.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/version.dart';
import 'package:test/fake.dart';
import 'package:webdriver/async_io.dart';

import '../../integration.shard/test_utils.dart';
import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fakes.dart';

void main() {
  late Config config;
  late Logger logger;
  late FileSystem fs;
  late Platform platform;
  late FakeProcessManager processManager;

  setUp(() {
    config = Config.test();
    logger = BufferLogger.test();
    fs = MemoryFileSystem.test();
    platform = FakePlatform(environment: <String, String>{'PATH': ''});
    processManager = FakeProcessManager.empty();
  });

  group(Java, () {
    group('find', () {
      testWithoutContext('finds the JDK bundled with Android Studio, if it exists', () {
        final AndroidStudio androidStudio = _FakeAndroidStudioWithJdk();
        final String androidStudioBundledJdkHome = androidStudio.javaPath!;
        final String expectedJavaBinaryPath = fs.path.join(
          androidStudioBundledJdkHome,
          'bin',
          'java',
        );
        const JavaSource expectedJavaHomeSource = JavaSource.androidStudio;

        processManager.addCommand(
          FakeCommand(
            command: <String>[expectedJavaBinaryPath, '--version'],
            stdout: '''
openjdk 19.0.2 2023-01-17
OpenJDK Runtime Environment Zulu19.32+15-CA (build 19.0.2+7)
OpenJDK 64-Bit Server VM Zulu19.32+15-CA (build 19.0.2+7, mixed mode, sharing)
''',
          ),
        );
        final Java java =
            Java.find(
              config: config,
              androidStudio: androidStudio,
              logger: logger,
              fileSystem: fs,
              platform: platform,
              processManager: processManager,
            )!;

        expect(java.javaHome, androidStudioBundledJdkHome);
        expect(java.binaryPath, expectedJavaBinaryPath);

        expect(
          java.version!.toString(),
          'OpenJDK Runtime Environment Zulu19.32+15-CA (build 19.0.2+7)',
        );
        expect(java.version, equals(Version(19, 0, 2)));
        expect(java.javaSource, expectedJavaHomeSource);
      });

      testWithoutContext(
        'finds JAVA_HOME if it is set and the JDK bundled with Android Studio could not be found',
        () {
          final AndroidStudio androidStudio = _FakeAndroidStudioWithoutJdk();
          const String javaHome = '/java/home';
          final String expectedJavaBinaryPath = fs.path.join(javaHome, 'bin', 'java');
          const JavaSource expectedJavaHomeSource = JavaSource.javaHome;

          final Java java =
              Java.find(
                config: config,
                androidStudio: androidStudio,
                logger: logger,
                fileSystem: fs,
                platform: FakePlatform(
                  environment: <String, String>{Java.javaHomeEnvironmentVariable: javaHome},
                ),
                processManager: processManager,
              )!;

          expect(java.javaHome, javaHome);
          expect(java.binaryPath, expectedJavaBinaryPath);
          expect(java.javaSource, expectedJavaHomeSource);
        },
      );

      testWithoutContext('returns the java binary found on PATH if no other can be found', () {
        final AndroidStudio androidStudio = _FakeAndroidStudioWithoutJdk();
        final OperatingSystemUtils os = _FakeOperatingSystemUtilsWithJava(fileSystem);
        const JavaSource expectedJavaHomeSource = JavaSource.path;

        processManager.addCommand(
          const FakeCommand(command: <String>['which', 'java'], stdout: '/fake/which/java/path'),
        );

        final Java java =
            Java.find(
              config: config,
              androidStudio: androidStudio,
              logger: logger,
              fileSystem: fs,
              platform: platform,
              processManager: processManager,
            )!;

        expect(java.javaHome, isNull);
        expect(java.binaryPath, os.which('java')!.path);
        expect(java.javaSource, expectedJavaHomeSource);
      });

      testWithoutContext('returns null if no java could be found', () {
        final AndroidStudio androidStudio = _FakeAndroidStudioWithoutJdk();
        processManager.addCommand(
          const FakeCommand(command: <String>['which', 'java'], exitCode: 1),
        );
        final Java? java = Java.find(
          config: config,
          androidStudio: androidStudio,
          logger: logger,
          fileSystem: fs,
          platform: platform,
          processManager: processManager,
        );
        expect(java, isNull);
      });

      testWithoutContext('finds and prefers JDK found at config item "jdk-dir" if it is set', () {
        const String configuredJdkPath = '/jdk';
        config.setValue('jdk-dir', configuredJdkPath);
        JavaSource expectedJavaHomeSource = JavaSource.flutterConfig;

        processManager.addCommand(
          const FakeCommand(command: <String>['which', 'java'], stdout: '/fake/which/java/path'),
        );

        final _FakeAndroidStudioWithJdk androidStudio = _FakeAndroidStudioWithJdk();
        final FakePlatform platformWithJavaHome = FakePlatform(
          environment: <String, String>{'JAVA_HOME': '/old/jdk'},
        );
        Java? java = Java.find(
          config: config,
          androidStudio: androidStudio,
          logger: logger,
          fileSystem: fs,
          platform: platformWithJavaHome,
          processManager: processManager,
        );

        expect(java, isNotNull);
        expect(java!.javaHome, configuredJdkPath);
        expect(java.binaryPath, fs.path.join(configuredJdkPath, 'bin', 'java'));
        expect(java.javaSource, expectedJavaHomeSource);

        config.removeValue('jdk-dir');

        expectedJavaHomeSource = JavaSource.androidStudio;

        java = Java.find(
          config: config,
          androidStudio: androidStudio,
          logger: logger,
          fileSystem: fs,
          platform: platformWithJavaHome,
          processManager: processManager,
        );

        expect(java, isNotNull);
        assert(androidStudio.javaPath != configuredJdkPath);
        expect(java!.javaHome, androidStudio.javaPath);
        expect(java.binaryPath, fs.path.join(androidStudio.javaPath!, 'bin', 'java'));
        expect(java.javaSource, expectedJavaHomeSource);
      });
    });

    group('version', () {
      late Java java;

      setUp(() {
        processManager = FakeProcessManager.empty();
        java = Java(
          fileSystem: fs,
          logger: logger,
          os: FakeOperatingSystemUtils(),
          platform: platform,
          processManager: processManager,
          binaryPath: 'javaHome/bin/java',
          javaHome: 'javaHome',
          javaSource: JavaSource.javaHome,
        );
      });

      void addJavaVersionCommand(String output) {
        processManager.addCommand(
          FakeCommand(command: <String>[java.binaryPath, '--version'], stdout: output),
        );
      }

      testWithoutContext('is null when java binary cannot be run', () async {
        addJavaVersionCommand('');
        processManager.excludedExecutables.add('java');

        expect(java.version, null);
      });

      testWithoutContext('is null when java --version returns a non-zero exit code', () async {
        processManager.addCommand(
          FakeCommand(command: <String>[java.binaryPath, '--version'], exitCode: 1),
        );
        expect(java.version, null);
      });

      testWithoutContext('parses jdk 8', () {
        addJavaVersionCommand('''
java version "1.8.0_202"
Java(TM) SE Runtime Environment (build 1.8.0_202-b10)
Java HotSpot(TM) 64-Bit Server VM (build 25.202-b10, mixed mode)
''');
        final Version version = java.version!;
        expect(version.toString(), 'Java(TM) SE Runtime Environment (build 1.8.0_202-b10)');
        expect(version, equals(Version(1, 8, 0)));
      });
      testWithoutContext('parses jdk 11 windows', () {
        addJavaVersionCommand('''
java version "11.0.14"
Java(TM) SE Runtime Environment (build 11.0.14+10-b13)
Java HotSpot(TM) 64-Bit Server VM (build 11.0.14+10-b13, mixed mode)
''');
        final Version version = java.version!;
        expect(version.toString(), 'Java(TM) SE Runtime Environment (build 11.0.14+10-b13)');
        expect(version, equals(Version(11, 0, 14)));
      });

      testWithoutContext('parses jdk 11 mac/linux', () {
        addJavaVersionCommand('''
openjdk version "11.0.18" 2023-01-17 LTS
OpenJDK Runtime Environment Zulu11.62+17-CA (build 11.0.18+10-LTS)
OpenJDK 64-Bit Server VM Zulu11.62+17-CA (build 11.0.18+10-LTS, mixed mode)
''');
        final Version version = java.version!;
        expect(
          version.toString(),
          'OpenJDK Runtime Environment Zulu11.62+17-CA (build 11.0.18+10-LTS)',
        );
        expect(version, equals(Version(11, 0, 18)));
      });

      testWithoutContext('parses jdk 17', () {
        addJavaVersionCommand('''
openjdk 17.0.6 2023-01-17
OpenJDK Runtime Environment (build 17.0.6+0-17.0.6b802.4-9586694)
OpenJDK 64-Bit Server VM (build 17.0.6+0-17.0.6b802.4-9586694, mixed mode)
''');
        final Version version = java.version!;
        expect(
          version.toString(),
          'OpenJDK Runtime Environment (build 17.0.6+0-17.0.6b802.4-9586694)',
        );
        expect(version, equals(Version(17, 0, 6)));
      });

      testWithoutContext('parses jdk 19', () {
        addJavaVersionCommand('''
openjdk 19.0.2 2023-01-17
OpenJDK Runtime Environment Homebrew (build 19.0.2)
OpenJDK 64-Bit Server VM Homebrew (build 19.0.2, mixed mode, sharing)
''');
        final Version version = java.version!;
        expect(version.toString(), 'OpenJDK Runtime Environment Homebrew (build 19.0.2)');
        expect(version, equals(Version(19, 0, 2)));
      });

      // https://chrome-infra-packages.appspot.com/p/flutter/java/openjdk/
      testWithoutContext('parses jdk output from ci', () {
        addJavaVersionCommand('''
openjdk 11.0.2 2019-01-15
OpenJDK Runtime Environment 18.9 (build 11.0.2+9)
OpenJDK 64-Bit Server VM 18.9 (build 11.0.2+9, mixed mode)
''');
        final Version version = java.version!;
        expect(version.toString(), 'OpenJDK Runtime Environment 18.9 (build 11.0.2+9)');
        expect(version, equals(Version(11, 0, 2)));
      });

      testWithoutContext('parses jdk two number versions', () {
        addJavaVersionCommand('openjdk 19.0 2023-01-17');
        final Version version = java.version!;
        expect(version.toString(), 'openjdk 19.0 2023-01-17');
        expect(version, equals(Version(19, 0, null)));
      });

      testWithoutContext('parses jdk 21 with patch numbers', () {
        addJavaVersionCommand('''
java 21.0.1 2023-09-19 LTS
Java(TM) SE Runtime Environment (build 21+35-LTS-2513)
Java HotSpot(TM) 64-Bit Server VM (build 21+35-LTS-2513, mixed mode, sharing)
''');
        final Version? version = java.version;
        expect(version, equals(Version(21, 0, 1)));
      });

      testWithoutContext('parses jdk 21 with no patch numbers', () {
        addJavaVersionCommand('''
java 21 2023-09-19 LTS
Java(TM) SE Runtime Environment (build 21+35-LTS-2513)
Java HotSpot(TM) 64-Bit Server VM (build 21+35-LTS-2513, mixed mode, sharing)
''');
        final Version? version = java.version;
        expect(version, equals(Version(21, 0, 0)));
      });
      testWithoutContext('parses openjdk 21 with no patch numbers', () {
        addJavaVersionCommand('''
openjdk version "21" 2023-09-19
OpenJDK Runtime Environment (build 21+35)
OpenJDK 64-Bit Server VM (build 21+35, mixed mode, sharing)
''');
        final Version? version = java.version;
        expect(version, equals(Version(21, 0, 0)));
      });
    });
  });
}

class _FakeAndroidStudioWithJdk extends Fake implements AndroidStudio {
  @override
  String? get javaPath => '/fake/android_studio/java/path/';
}

class _FakeAndroidStudioWithoutJdk extends Fake implements AndroidStudio {
  @override
  String? get javaPath => null;
}

class _FakeOperatingSystemUtilsWithJava extends FakeOperatingSystemUtils {
  _FakeOperatingSystemUtilsWithJava(this._fileSystem);

  final FileSystem _fileSystem;
  @override
  File? which(String execName) {
    if (execName == 'java') {
      return _fileSystem.file('/fake/which/java/path');
    }
    throw const InvalidArgumentException(null, null);
  }
}
