// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/android/android_sdk.dart';
import 'package:flutter_tools/src/android/android_workflow.dart';
import 'package:flutter_tools/src/android/java.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/user_messages.dart';
import 'package:flutter_tools/src/base/version.dart';
import 'package:flutter_tools/src/doctor_validator.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_process_manager.dart';
import '../../src/fakes.dart';

void main() {
  late FakeAndroidSdk sdk;
  late Logger logger;
  late MemoryFileSystem fileSystem;
  late FakeProcessManager processManager;
  late FakeStdio stdio;

  setUp(() {
    sdk = FakeAndroidSdk();
    fileSystem = MemoryFileSystem.test();
    fileSystem.directory('/home/me').createSync(recursive: true);
    logger = BufferLogger.test();
    processManager = FakeProcessManager.empty();
    stdio = FakeStdio();
  });

  testWithoutContext('AndroidWorkflow handles a null AndroidSDK', () {
    final AndroidWorkflow androidWorkflow = AndroidWorkflow(
      featureFlags: TestFeatureFlags(),
      androidSdk: null,
    );

    expect(androidWorkflow.canLaunchDevices, false);
    expect(androidWorkflow.canListDevices, false);
    expect(androidWorkflow.canListEmulators, false);
  });

  testWithoutContext('AndroidWorkflow handles a null adb', () {
    final FakeAndroidSdk androidSdk = FakeAndroidSdk();
    androidSdk.adbPath = null;
    final AndroidWorkflow androidWorkflow = AndroidWorkflow(
      featureFlags: TestFeatureFlags(),
      androidSdk: androidSdk,
    );

    expect(androidWorkflow.canLaunchDevices, false);
    expect(androidWorkflow.canListDevices, false);
    expect(androidWorkflow.canListEmulators, false);
  });

  // Android SDK is actually supported on Linux Arm64 hosts.
  testWithoutContext('Support for Android SDK on Linux Arm Hosts', () {
    final FakeAndroidSdk androidSdk = FakeAndroidSdk();
    androidSdk.adbPath = null;
    final AndroidWorkflow androidWorkflow = AndroidWorkflow(
      featureFlags: TestFeatureFlags(),
      androidSdk: androidSdk,
    );

    expect(androidWorkflow.appliesToHostPlatform, isTrue);
    expect(androidWorkflow.canLaunchDevices, isFalse);
    expect(androidWorkflow.canListDevices, isFalse);
    expect(androidWorkflow.canListEmulators, isFalse);
  });

  testWithoutContext('AndroidWorkflow is disabled if feature is disabled', () {
    final FakeAndroidSdk androidSdk = FakeAndroidSdk();
    androidSdk.adbPath = 'path/to/adb';
    final AndroidWorkflow androidWorkflow = AndroidWorkflow(
      featureFlags: TestFeatureFlags(isAndroidEnabled: false),
      androidSdk: androidSdk,
    );

    expect(androidWorkflow.appliesToHostPlatform, false);
    expect(androidWorkflow.canLaunchDevices, false);
    expect(androidWorkflow.canListDevices, false);
    expect(androidWorkflow.canListEmulators, false);
  });

  testWithoutContext('AndroidWorkflow cannot list emulators if emulatorPath is null', () {
    final FakeAndroidSdk androidSdk = FakeAndroidSdk();
    androidSdk.adbPath = 'path/to/adb';
    final AndroidWorkflow androidWorkflow = AndroidWorkflow(
      featureFlags: TestFeatureFlags(),
      androidSdk: androidSdk,
    );

    expect(androidWorkflow.appliesToHostPlatform, true);
    expect(androidWorkflow.canLaunchDevices, true);
    expect(androidWorkflow.canListDevices, true);
    expect(androidWorkflow.canListEmulators, false);
  });

  testWithoutContext('AndroidWorkflow can list emulators', () {
    final FakeAndroidSdk androidSdk = FakeAndroidSdk();
    androidSdk.adbPath = 'path/to/adb';
    androidSdk.emulatorPath = 'path/to/emulator';
    final AndroidWorkflow androidWorkflow = AndroidWorkflow(
      featureFlags: TestFeatureFlags(),
      androidSdk: androidSdk,
    );

    expect(androidWorkflow.appliesToHostPlatform, true);
    expect(androidWorkflow.canLaunchDevices, true);
    expect(androidWorkflow.canListDevices, true);
    expect(androidWorkflow.canListEmulators, true);
  });

  testWithoutContext(
    'licensesAccepted returns LicensesAccepted.unknown if cannot find sdkmanager',
    () async {
      sdk.sdkManagerPath = '/foo/bar/sdkmanager';
      processManager.excludedExecutables.add('/foo/bar/sdkmanager');
      final AndroidLicenseValidator licenseValidator = AndroidLicenseValidator(
        java: FakeJava(),
        androidSdk: sdk,
        processManager: processManager,
        platform: FakePlatform(environment: <String, String>{'HOME': '/home/me'}),
        stdio: stdio,
        logger: BufferLogger.test(),
        userMessages: UserMessages(),
      );
      final LicensesAccepted licenseStatus = await licenseValidator.licensesAccepted;

      expect(licenseStatus, LicensesAccepted.unknown);
    },
  );

  testWithoutContext(
    'licensesAccepted returns LicensesAccepted.unknown if cannot run sdkmanager',
    () async {
      sdk.sdkManagerPath = '/foo/bar/sdkmanager';
      processManager.excludedExecutables.add('/foo/bar/sdkmanager');
      final AndroidLicenseValidator licenseValidator = AndroidLicenseValidator(
        java: FakeJava(),
        androidSdk: sdk,
        processManager: processManager,
        platform: FakePlatform(environment: <String, String>{'HOME': '/home/me'}),
        stdio: stdio,
        logger: BufferLogger.test(),
        userMessages: UserMessages(),
      );
      final LicensesAccepted licenseStatus = await licenseValidator.licensesAccepted;

      expect(licenseStatus, LicensesAccepted.unknown);
    },
  );

  testWithoutContext(
    'licensesAccepted returns LicensesAccepted.unknown if cannot write to sdkmanager',
    () async {
      sdk.sdkManagerPath = '/foo/bar/sdkmanager';
      processManager.addCommand(
        FakeCommand(
          command: <String>[sdk.sdkManagerPath!, '--licenses'],
          stdin: IOSink(ClosedStdinController()),
        ),
      );
      final AndroidLicenseValidator licenseValidator = AndroidLicenseValidator(
        java: FakeJava(),
        androidSdk: sdk,
        processManager: processManager,
        platform: FakePlatform(environment: <String, String>{'HOME': '/home/me'}),
        stdio: stdio,
        logger: BufferLogger.test(),
        userMessages: UserMessages(),
      );
      final LicensesAccepted licenseStatus = await licenseValidator.licensesAccepted;

      expect(licenseStatus, LicensesAccepted.unknown);
      expect(processManager, hasNoRemainingExpectations);
    },
  );

  testWithoutContext('licensesAccepted handles garbage/no output', () async {
    sdk.sdkManagerPath = '/foo/bar/sdkmanager';
    processManager.addCommand(
      const FakeCommand(
        command: <String>['/foo/bar/sdkmanager', '--licenses'],
        stdout: 'asdasassad',
      ),
    );
    final AndroidLicenseValidator licenseValidator = AndroidLicenseValidator(
      java: FakeJava(),
      androidSdk: sdk,
      processManager: processManager,
      platform: FakePlatform(environment: <String, String>{'HOME': '/home/me'}),
      stdio: stdio,
      logger: BufferLogger.test(),
      userMessages: UserMessages(),
    );
    final LicensesAccepted result = await licenseValidator.licensesAccepted;

    expect(result, LicensesAccepted.unknown);
  });

  testWithoutContext('licensesAccepted works for all licenses accepted', () async {
    sdk.sdkManagerPath = '/foo/bar/sdkmanager';
    const String output = '''
[=======================================] 100% Computing updates...
All SDK package licenses accepted.
''';
    processManager.addCommand(
      const FakeCommand(command: <String>['/foo/bar/sdkmanager', '--licenses'], stdout: output),
    );

    final AndroidLicenseValidator licenseValidator = AndroidLicenseValidator(
      java: FakeJava(),
      androidSdk: sdk,
      processManager: processManager,
      platform: FakePlatform(environment: <String, String>{'HOME': '/home/me'}),
      stdio: stdio,
      logger: BufferLogger.test(),
      userMessages: UserMessages(),
    );
    final LicensesAccepted result = await licenseValidator.licensesAccepted;

    expect(result, LicensesAccepted.all);
  });

  testWithoutContext('licensesAccepted sets environment for finding java', () async {
    final Java java = FakeJava();
    sdk.sdkManagerPath = '/foo/bar/sdkmanager';
    processManager.addCommand(
      FakeCommand(
        command: <String>[sdk.sdkManagerPath!, '--licenses'],
        stdout: 'All SDK package licenses accepted.',
        environment: <String, String>{
          'JAVA_HOME': java.javaHome!,
          'PATH': fileSystem.path.join(java.javaHome!, 'bin'),
        },
      ),
    );
    final AndroidLicenseValidator licenseValidator = AndroidLicenseValidator(
      java: java,
      androidSdk: sdk,
      processManager: processManager,
      platform: FakePlatform(environment: <String, String>{'HOME': '/home/me'}),
      stdio: stdio,
      logger: BufferLogger.test(),
      userMessages: UserMessages(),
    );
    final LicensesAccepted licenseStatus = await licenseValidator.licensesAccepted;

    expect(licenseStatus, LicensesAccepted.all);
  });

  testWithoutContext('licensesAccepted works for some licenses accepted', () async {
    sdk.sdkManagerPath = '/foo/bar/sdkmanager';
    const String output = '''
[=======================================] 100% Computing updates...
2 of 5 SDK package licenses not accepted.
Review licenses that have not been accepted (y/N)?
''';
    processManager.addCommand(
      const FakeCommand(command: <String>['/foo/bar/sdkmanager', '--licenses'], stdout: output),
    );

    final AndroidLicenseValidator licenseValidator = AndroidLicenseValidator(
      java: FakeJava(),
      androidSdk: sdk,
      processManager: processManager,
      platform: FakePlatform(environment: <String, String>{'HOME': '/home/me'}),
      stdio: stdio,
      logger: BufferLogger.test(),
      userMessages: UserMessages(),
    );
    final LicensesAccepted result = await licenseValidator.licensesAccepted;

    expect(result, LicensesAccepted.some);
  });

  testWithoutContext('licensesAccepted works for no licenses accepted', () async {
    sdk.sdkManagerPath = '/foo/bar/sdkmanager';
    const String output = '''
[=======================================] 100% Computing updates...
5 of 5 SDK package licenses not accepted.
Review licenses that have not been accepted (y/N)?
''';
    processManager.addCommand(
      const FakeCommand(command: <String>['/foo/bar/sdkmanager', '--licenses'], stdout: output),
    );

    final AndroidLicenseValidator licenseValidator = AndroidLicenseValidator(
      java: FakeJava(),
      androidSdk: sdk,
      processManager: processManager,
      platform: FakePlatform(environment: <String, String>{'HOME': '/home/me'}),
      stdio: stdio,
      logger: BufferLogger.test(),
      userMessages: UserMessages(),
    );
    final LicensesAccepted result = await licenseValidator.licensesAccepted;

    expect(result, LicensesAccepted.none);
  });

  testWithoutContext('runLicenseManager succeeds for version >= 26', () async {
    sdk.sdkManagerPath = '/foo/bar/sdkmanager';
    sdk.sdkManagerVersion = '26.0.0';
    processManager.addCommand(
      const FakeCommand(command: <String>['/foo/bar/sdkmanager', '--licenses']),
    );

    final AndroidLicenseValidator licenseValidator = AndroidLicenseValidator(
      java: FakeJava(),
      androidSdk: sdk,
      processManager: processManager,
      platform: FakePlatform(environment: <String, String>{'HOME': '/home/me'}),
      stdio: stdio,
      logger: BufferLogger.test(),
      userMessages: UserMessages(),
    );

    expect(await licenseValidator.runLicenseManager(), isTrue);
  });

  testWithoutContext('runLicenseManager errors when sdkmanager is not found', () async {
    sdk.sdkManagerPath = '/foo/bar/sdkmanager';
    processManager.excludedExecutables.add('/foo/bar/sdkmanager');

    final AndroidLicenseValidator licenseValidator = AndroidLicenseValidator(
      java: FakeJava(),
      androidSdk: sdk,
      processManager: processManager,
      platform: FakePlatform(environment: <String, String>{'HOME': '/home/me'}),
      stdio: stdio,
      logger: BufferLogger.test(),
      userMessages: UserMessages(),
    );

    expect(licenseValidator.runLicenseManager(), throwsToolExit());
  });

  testWithoutContext('runLicenseManager handles broken pipe without ArgumentError', () async {
    sdk.sdkManagerPath = '/foo/bar/sdkmanager';
    const String exceptionMessage = 'Write failed (OS Error: Broken pipe, errno = 32), port = 0';
    const SocketException exception = SocketException(exceptionMessage);
    // By using a `Socket` generic parameter, the stdin.addStream will return a `Future<Socket>`
    // We are testing that our error handling properly handles futures of this type
    final ThrowingStdin<Socket> fakeStdin = ThrowingStdin<Socket>(exception);
    final FakeCommand licenseCommand = FakeCommand(
      command: <String>[sdk.sdkManagerPath!, '--licenses'],
      stdin: fakeStdin,
    );
    processManager.addCommand(licenseCommand);
    final BufferLogger logger = BufferLogger.test();

    final AndroidLicenseValidator licenseValidator = AndroidLicenseValidator(
      java: FakeJava(),
      androidSdk: sdk,
      processManager: processManager,
      platform: FakePlatform(environment: <String, String>{'HOME': '/home/me'}),
      stdio: stdio,
      logger: logger,
      userMessages: UserMessages(),
    );

    await licenseValidator.runLicenseManager();
    expect(logger.traceText, contains(exceptionMessage));
    expect(processManager, hasNoRemainingExpectations);
  });

  testWithoutContext('runLicenseManager errors when sdkmanager fails to run', () async {
    sdk.sdkManagerPath = '/foo/bar/sdkmanager';
    processManager.excludedExecutables.add('/foo/bar/sdkmanager');

    final AndroidLicenseValidator licenseValidator = AndroidLicenseValidator(
      java: FakeJava(),
      androidSdk: sdk,
      processManager: processManager,
      platform: FakePlatform(environment: <String, String>{'HOME': '/home/me'}),
      stdio: stdio,
      logger: BufferLogger.test(),
      userMessages: UserMessages(),
    );

    expect(licenseValidator.runLicenseManager(), throwsToolExit());
  });

  testWithoutContext('runLicenseManager errors when sdkmanager exits non-zero', () async {
    const String sdkManagerPath = '/foo/bar/sdkmanager';
    sdk.sdkManagerPath = sdkManagerPath;
    final BufferLogger logger = BufferLogger.test();
    processManager.addCommand(
      const FakeCommand(
        command: <String>[sdkManagerPath, '--licenses'],
        exitCode: 1,
        stderr: 'sdkmanager crash',
      ),
    );

    final AndroidLicenseValidator licenseValidator = AndroidLicenseValidator(
      java: FakeJava(),
      androidSdk: sdk,
      processManager: processManager,
      platform: FakePlatform(environment: <String, String>{'HOME': '/home/me'}),
      stdio: stdio,
      logger: logger,
      userMessages: UserMessages(),
    );

    await expectLater(
      licenseValidator.runLicenseManager(),
      throwsToolExit(
        message:
            'Android sdkmanager tool was found, but failed to run ($sdkManagerPath): "exited code 1"',
      ),
    );
    expect(processManager, hasNoRemainingExpectations);
    expect(logger.traceText, isEmpty);
    expect(stdio.writtenToStdout, isEmpty);
    expect(stdio.writtenToStderr, contains('sdkmanager crash'));
  });

  testWithoutContext('detects license-only SDK installation with cmdline-tools', () async {
    sdk
      ..licensesAvailable = true
      ..platformToolsAvailable = false
      ..cmdlineToolsAvailable = true
      ..directory = fileSystem.directory('/foo/bar');
    final ValidationResult validationResult =
        await AndroidValidator(
          java: FakeJava(),
          androidSdk: sdk,
          logger: logger,
          platform: FakePlatform()..environment = <String, String>{'HOME': '/home/me'},
          userMessages: UserMessages(),
        ).validate();

    expect(validationResult.type, ValidationType.partial);

    final ValidationMessage sdkMessage = validationResult.messages.first;
    expect(sdkMessage.type, ValidationMessageType.information);
    expect(sdkMessage.message, 'Android SDK at /foo/bar');

    final ValidationMessage licenseMessage = validationResult.messages.last;
    expect(licenseMessage.type, ValidationMessageType.hint);
    expect(licenseMessage.message, UserMessages().androidSdkLicenseOnly(kAndroidHome));
  });

  testUsingContext('detects minimum required SDK and buildtools', () async {
    final FakeAndroidSdkVersion sdkVersion =
        FakeAndroidSdkVersion()
          ..sdkLevel = 28
          ..buildToolsVersion = Version(26, 0, 3);

    sdk
      ..licensesAvailable = true
      ..platformToolsAvailable = true
      ..cmdlineToolsAvailable = true
      // Test with invalid SDK and build tools
      ..directory = fileSystem.directory('/foo/bar')
      ..sdkManagerPath = '/foo/bar/sdkmanager'
      ..latestVersion = sdkVersion;

    final String errorMessage = UserMessages().androidSdkBuildToolsOutdated(
      kAndroidSdkMinVersion,
      kAndroidSdkBuildToolsMinVersion.toString(),
      FakePlatform(),
    );

    final AndroidValidator androidValidator = AndroidValidator(
      java: null,
      androidSdk: sdk,
      logger: logger,
      platform: FakePlatform()..environment = <String, String>{'HOME': '/home/me'},
      userMessages: UserMessages(),
    );

    ValidationResult validationResult = await androidValidator.validate();
    expect(validationResult.type, ValidationType.missing);
    expect(validationResult.messages.last.message, errorMessage);

    // Test with valid SDK but invalid build tools
    sdkVersion.sdkLevel = 29;
    sdkVersion.buildToolsVersion = Version(28, 0, 2);

    validationResult = await androidValidator.validate();
    expect(validationResult.type, ValidationType.missing);
    expect(validationResult.messages.last.message, errorMessage);

    // Test with valid SDK and valid build tools
    // Will still be partial because AndroidSdk.findJavaBinary is static :(
    sdkVersion.sdkLevel = kAndroidSdkMinVersion;
    sdkVersion.buildToolsVersion = kAndroidSdkBuildToolsMinVersion;

    validationResult = await androidValidator.validate();
    expect(validationResult.type, ValidationType.partial); // No Java binary
    expect(
      validationResult.messages.any((ValidationMessage message) => message.message == errorMessage),
      isFalse,
    );
  });

  testWithoutContext('detects missing cmdline tools', () async {
    sdk
      ..licensesAvailable = true
      ..platformToolsAvailable = true
      ..cmdlineToolsAvailable = false
      ..directory = fileSystem.directory('/foo/bar');

    final AndroidValidator androidValidator = AndroidValidator(
      java: FakeJava(),
      androidSdk: sdk,
      logger: logger,
      platform: FakePlatform()..environment = <String, String>{'HOME': '/home/me'},
      userMessages: UserMessages(),
    );

    final ValidationResult validationResult = await androidValidator.validate();
    expect(validationResult.type, ValidationType.missing);

    final ValidationMessage sdkMessage = validationResult.messages.first;
    expect(sdkMessage.type, ValidationMessageType.information);
    expect(sdkMessage.message, 'Android SDK at /foo/bar');

    final ValidationMessage cmdlineMessage = validationResult.messages.last;
    expect(cmdlineMessage.type, ValidationMessageType.error);
    expect(
      cmdlineMessage.message,
      'cmdline-tools component is missing.\n'
      'Try installing or updating Android Studio.\n'
      'Alternatively, download the tools from https://developer.android.com/studio#command-line-tools-only and make sure to set the ANDROID_HOME environment variable.\n'
      'See https://developer.android.com/studio/command-line for more details.',
    );
  });

  testUsingContext('detects minimum required java version', () async {
    // Test with older version of JDK
    final Platform platform =
        FakePlatform()
          ..environment = <String, String>{
            'HOME': '/home/me',
            Java.javaHomeEnvironmentVariable: 'home/java',
            'PATH': '',
          };
    final FakeAndroidSdkVersion sdkVersion =
        FakeAndroidSdkVersion()
          ..sdkLevel = 29
          ..buildToolsVersion = Version(28, 0, 3);

    // Mock a pass through scenario to reach _checkJavaVersion()
    sdk
      ..licensesAvailable = true
      ..platformToolsAvailable = true
      ..cmdlineToolsAvailable = true
      ..directory = fileSystem.directory('/foo/bar')
      ..sdkManagerPath = '/foo/bar/sdkmanager';
    sdk.latestVersion = sdkVersion;

    const String javaVersionText = 'openjdk version "1.7.0_212"';
    final String errorMessage = UserMessages().androidJavaMinimumVersion(javaVersionText);

    final ValidationResult validationResult =
        await AndroidValidator(
          java: FakeJava(version: const Version.withText(1, 7, 0, javaVersionText)),
          androidSdk: sdk,
          logger: logger,
          platform: platform,
          userMessages: UserMessages(),
        ).validate();
    expect(validationResult.type, ValidationType.partial);
    expect(validationResult.messages.last.message, errorMessage);
    expect(
      validationResult.messages.any(
        (ValidationMessage message) => message.message.contains('Unable to locate Android SDK'),
      ),
      false,
    );
  });

  testWithoutContext('Mentions `flutter config --android-sdk if user has no AndroidSdk`', () async {
    final ValidationResult validationResult =
        await AndroidValidator(
          java: FakeJava(),
          androidSdk: null,
          logger: logger,
          platform:
              FakePlatform()
                ..environment = <String, String>{
                  'HOME': '/home/me',
                  Java.javaHomeEnvironmentVariable: 'home/java',
                },
          userMessages: UserMessages(),
        ).validate();

    expect(
      validationResult.messages.any(
        (ValidationMessage message) => message.message.contains('flutter config --android-sdk'),
      ),
      true,
    );
  });

  testWithoutContext(
    'Asks user to upgrade Android Studio when it is too far behind the Android SDK',
    () async {
      const String sdkManagerPath = '/foo/bar/sdkmanager';
      sdk.sdkManagerPath = sdkManagerPath;
      final BufferLogger logger = BufferLogger.test();
      processManager.addCommand(
        const FakeCommand(
          command: <String>[sdkManagerPath, '--licenses'],
          exitCode: 1,
          stderr: '''
Error: LinkageError occurred while loading main class com.android.sdklib.tool.sdkmanager.SdkManagerCli
        java.lang.UnsupportedClassVersionError: com/android/sdklib/tool/sdkmanager/SdkManagerCli has been compiled by a more recent version of the Java Runtime (class file version 61.0), this version of the Java Runtime only recognizes class file versions up to 55.0
Android sdkmanager tool was found, but failed to run
''',
        ),
      );

      final AndroidLicenseValidator licenseValidator = AndroidLicenseValidator(
        java: FakeJava(),
        androidSdk: sdk,
        processManager: processManager,
        platform: FakePlatform(environment: <String, String>{'HOME': '/home/me'}),
        stdio: stdio,
        logger: logger,
        userMessages: UserMessages(),
      );

      await expectLater(
        licenseValidator.runLicenseManager(),
        throwsToolExit(
          message: RegExp(
            '.*consider updating your installation of Android studio. Alternatively, you.*',
          ),
        ),
      );
      expect(processManager, hasNoRemainingExpectations);
      expect(stdio.stderr.getAndClear(), contains('UnsupportedClassVersionError'));
    },
  );

  testWithoutContext(
    'Mentions that JDK is provided by latest Android Studio Installation',
    () async {
      // Mock a pass through scenario to reach _checkJavaVersion()
      sdk
        ..licensesAvailable = true
        ..platformToolsAvailable = true
        ..cmdlineToolsAvailable = true
        ..directory = fileSystem.directory('/foo/bar')
        ..sdkManagerPath = '/foo/bar/sdkmanager';

      final ValidationResult validationResult =
          await AndroidValidator(
            java: FakeJava(),
            androidSdk: sdk,
            logger: logger,
            platform: FakePlatform(),
            userMessages: UserMessages(),
          ).validate();

      expect(
        validationResult.messages.any(
          (ValidationMessage message) => message.message.contains(
            'This is the JDK bundled with the latest Android Studio installation on this machine.',
          ),
        ),
        true,
      );
      expect(
        validationResult.messages.any(
          (ValidationMessage message) => message.message.contains(
            'To manually set the JDK path, use: `flutter config --jdk-dir="path/to/jdk"`.',
          ),
        ),
        true,
      );
    },
  );

  testWithoutContext(
    "Mentions that JDK is provided by user's JAVA_HOME environment variable",
    () async {
      // Mock a pass through scenario to reach _checkJavaVersion()
      sdk
        ..licensesAvailable = true
        ..platformToolsAvailable = true
        ..cmdlineToolsAvailable = true
        ..directory = fileSystem.directory('/foo/bar')
        ..sdkManagerPath = '/foo/bar/sdkmanager';

      final ValidationResult validationResult =
          await AndroidValidator(
            java: FakeJava(javaSource: JavaSource.javaHome),
            androidSdk: sdk,
            logger: logger,
            platform: FakePlatform(),
            userMessages: UserMessages(),
          ).validate();

      expect(
        validationResult.messages.any(
          (ValidationMessage message) => message.message.contains(
            'This JDK is specified by the JAVA_HOME environment variable.',
          ),
        ),
        true,
      );
      expect(
        validationResult.messages.any(
          (ValidationMessage message) => message.message.contains(
            'To manually set the JDK path, use: `flutter config --jdk-dir="path/to/jdk"`',
          ),
        ),
        true,
      );
    },
  );

  testWithoutContext('Mentions that path to Java binary is obtained from PATH', () async {
    // Mock a pass through scenario to reach _checkJavaVersion()
    sdk
      ..licensesAvailable = true
      ..platformToolsAvailable = true
      ..cmdlineToolsAvailable = true
      ..directory = fileSystem.directory('/foo/bar')
      ..sdkManagerPath = '/foo/bar/sdkmanager';

    final ValidationResult validationResult =
        await AndroidValidator(
          java: FakeJava(javaSource: JavaSource.path),
          androidSdk: sdk,
          logger: logger,
          platform: FakePlatform(),
          userMessages: UserMessages(),
        ).validate();

    expect(
      validationResult.messages.any(
        (ValidationMessage message) =>
            message.message.contains('This JDK was found in the system PATH.'),
      ),
      true,
    );
    expect(
      validationResult.messages.any(
        (ValidationMessage message) => message.message.contains(
          'To manually set the JDK path, use: `flutter config --jdk-dir="path/to/jdk"`.',
        ),
      ),
      true,
    );
  });

  testWithoutContext('Mentions that JDK is provided by Flutter config', () async {
    // Mock a pass through scenario to reach _checkJavaVersion()
    sdk
      ..licensesAvailable = true
      ..platformToolsAvailable = true
      ..cmdlineToolsAvailable = true
      ..directory = fileSystem.directory('/foo/bar')
      ..sdkManagerPath = '/foo/bar/sdkmanager';

    final ValidationResult validationResult =
        await AndroidValidator(
          java: FakeJava(javaSource: JavaSource.flutterConfig),
          androidSdk: sdk,
          logger: logger,
          platform: FakePlatform(),
          userMessages: UserMessages(),
        ).validate();

    expect(
      validationResult.messages.any(
        (ValidationMessage message) =>
            message.message.contains('This JDK is specified in your Flutter configuration.'),
      ),
      true,
    );
    expect(
      validationResult.messages.any(
        (ValidationMessage message) => message.message.contains(
          'To change the current JDK, run: `flutter config --jdk-dir="path/to/jdk"`.',
        ),
      ),
      true,
    );
  });
}

class FakeAndroidSdk extends Fake implements AndroidSdk {
  @override
  String? sdkManagerPath;

  @override
  String? sdkManagerVersion;

  @override
  String? adbPath;

  @override
  bool licensesAvailable = false;

  @override
  bool platformToolsAvailable = false;

  @override
  bool cmdlineToolsAvailable = false;

  @override
  Directory directory = MemoryFileSystem.test().directory('/foo/bar');

  @override
  AndroidSdkVersion? latestVersion;

  @override
  String? emulatorPath;

  @override
  List<String> validateSdkWellFormed() => <String>[];
}

class FakeAndroidSdkVersion extends Fake implements AndroidSdkVersion {
  @override
  int sdkLevel = 0;

  @override
  Version buildToolsVersion = Version(0, 0, 0);

  @override
  String get buildToolsVersionName => '';

  @override
  String get platformName => '';
}

class ThrowingStdin<T> extends Fake implements IOSink {
  ThrowingStdin(this.exception);

  final Exception exception;

  @override
  Future<dynamic> addStream(Stream<List<int>> stream) {
    return Future<T>.error(exception);
  }
}
