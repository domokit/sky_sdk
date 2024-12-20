// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/os.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/user_messages.dart';
import 'package:flutter_tools/src/doctor.dart';
import 'package:flutter_tools/src/doctor_validator.dart';
import 'package:flutter_tools/src/version.dart';
import 'package:test/fake.dart';

import '../src/common.dart';
import '../src/fake_process_manager.dart';
import '../src/fakes.dart';

/// Matches a doctor validation result.
Matcher _matchDoctorValidation({
  required ValidationType validationType,
  required String statusInfo,
  required Object messages,
}) {
  return const TypeMatcher<ValidationResult>()
      .having((ValidationResult result) => result.type, 'type', validationType)
      .having((ValidationResult result) => result.statusInfo, 'statusInfo', statusInfo)
      .having((ValidationResult result) => result.messages, 'messages', messages);
}

void main() {
  testWithoutContext('FlutterValidator shows an error message if gen_snapshot is '
      'downloaded and exits with code 1', () async {
    final FakeFlutterVersion flutterVersion = FakeFlutterVersion(
      frameworkVersion: '1.0.0',
      branch: 'beta',
    );
    final MemoryFileSystem fileSystem = MemoryFileSystem.test();
    final Artifacts artifacts = Artifacts.test();
    final FlutterValidator flutterValidator = FlutterValidator(
      platform: FakePlatform(localeName: 'en_US.UTF-8', environment: <String, String>{}),
      flutterVersion: () => flutterVersion,
      devToolsVersion: () => '2.8.0',
      userMessages: UserMessages(),
      artifacts: artifacts,
      fileSystem: fileSystem,
      flutterRoot: () => '/sdk/flutter',
      operatingSystemUtils: FakeOperatingSystemUtils(name: 'Linux'),
      processManager: FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(command: <String>['Artifact.genSnapshot'], exitCode: 1),
      ]),
    );
    fileSystem.file(artifacts.getArtifactPath(Artifact.genSnapshot)).createSync(recursive: true);

    expect(
      await flutterValidator.validate(),
      _matchDoctorValidation(
        validationType: ValidationType.partial,
        statusInfo: 'Channel beta, 1.0.0, on Linux, locale en_US.UTF-8',
        messages: containsAll(const <ValidationMessage>[
          ValidationMessage.error(
            'Downloaded executables cannot execute on host.\n'
            'See https://github.com/flutter/flutter/issues/6207 for more information.\n'
            'On Debian/Ubuntu/Mint: sudo apt-get install lib32stdc++6\n'
            'On Fedora: dnf install libstdc++.i686\n'
            'On Arch: pacman -S lib32-gcc-libs\n',
          ),
        ]),
      ),
    );
  });

  testWithoutContext('FlutterValidator shows an error message if Rosetta is needed', () async {
    final FakeFlutterVersion flutterVersion = FakeFlutterVersion(
      frameworkVersion: '1.0.0',
      branch: 'beta',
    );
    final MemoryFileSystem fileSystem = MemoryFileSystem.test();
    final Artifacts artifacts = Artifacts.test();
    final FlutterValidator flutterValidator = FlutterValidator(
      platform: FakePlatform(
        operatingSystem: 'macos',
        localeName: 'en_US.UTF-8',
        environment: <String, String>{},
      ),
      flutterVersion: () => flutterVersion,
      devToolsVersion: () => '2.8.0',
      userMessages: UserMessages(),
      artifacts: artifacts,
      fileSystem: fileSystem,
      flutterRoot: () => 'sdk/flutter',
      operatingSystemUtils: FakeOperatingSystemUtils(
        name: 'macOS',
        hostPlatform: HostPlatform.darwin_arm64,
      ),
      processManager: FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(command: <String>['Artifact.genSnapshot'], exitCode: 1),
      ]),
    );
    fileSystem.file(artifacts.getArtifactPath(Artifact.genSnapshot)).createSync(recursive: true);

    expect(
      await flutterValidator.validate(),
      _matchDoctorValidation(
        validationType: ValidationType.partial,
        statusInfo: 'Channel beta, 1.0.0, on macOS, locale en_US.UTF-8',
        messages: containsAll(const <ValidationMessage>[
          ValidationMessage.error(
            'Downloaded executables cannot execute on host.\n'
            'See https://github.com/flutter/flutter/issues/6207 for more information.\n'
            'Flutter requires the Rosetta translation environment on ARM Macs. Try running:\n'
            '  sudo softwareupdate --install-rosetta --agree-to-license\n',
          ),
        ]),
      ),
    );
  });

  testWithoutContext(
    'FlutterValidator does not run gen_snapshot binary check if it is not already downloaded',
    () async {
      final FakeFlutterVersion flutterVersion = FakeFlutterVersion(
        frameworkVersion: '1.0.0',
        branch: 'beta',
      );
      final FlutterValidator flutterValidator = FlutterValidator(
        platform: FakePlatform(
          operatingSystem: 'windows',
          localeName: 'en_US.UTF-8',
          environment: <String, String>{},
        ),
        flutterVersion: () => flutterVersion,
        devToolsVersion: () => '2.8.0',
        userMessages: UserMessages(),
        artifacts: Artifacts.test(),
        fileSystem: MemoryFileSystem.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(name: 'Windows'),
        processManager: FakeProcessManager.empty(),
        flutterRoot: () => '/sdk/flutter',
      );

      // gen_snapshot is downloaded on demand, and the doctor should not
      // fail if the gen_snapshot binary is not present.
      expect(
        await flutterValidator.validate(),
        _matchDoctorValidation(
          validationType: ValidationType.success,
          statusInfo: 'Channel beta, 1.0.0, on Windows, locale en_US.UTF-8',
          messages: anything,
        ),
      );
    },
  );

  testWithoutContext('FlutterValidator handles exception thrown by version checking', () async {
    final FlutterValidator flutterValidator = FlutterValidator(
      platform: FakePlatform(operatingSystem: 'windows', localeName: 'en_US.UTF-8'),
      flutterVersion: () => FakeThrowingFlutterVersion(),
      devToolsVersion: () => '2.8.0',
      userMessages: UserMessages(),
      artifacts: Artifacts.test(),
      fileSystem: MemoryFileSystem.test(),
      operatingSystemUtils: FakeOperatingSystemUtils(name: 'Windows'),
      processManager: FakeProcessManager.empty(),
      flutterRoot: () => '/sdk/flutter',
    );

    expect(
      await flutterValidator.validate(),
      _matchDoctorValidation(
        validationType: ValidationType.partial,
        statusInfo: 'Channel beta, 0.0.0, on Windows, locale en_US.UTF-8',
        messages: containsAll(const <ValidationMessage>[
          ValidationMessage('Flutter version 0.0.0 on channel beta at /sdk/flutter'),
          ValidationMessage.error('version error'),
        ]),
      ),
    );
  });

  testWithoutContext('FlutterValidator shows mirrors on pub and flutter cloud storage', () async {
    final FakeFlutterVersion flutterVersion = FakeFlutterVersion(
      frameworkVersion: '1.0.0',
      branch: 'beta',
    );
    final Platform platform = FakePlatform(
      operatingSystem: 'windows',
      localeName: 'en_US.UTF-8',
      environment: <String, String>{
        'PUB_HOSTED_URL': 'https://example.com/pub',
        'FLUTTER_STORAGE_BASE_URL': 'https://example.com/flutter',
      },
    );
    final MemoryFileSystem fileSystem = MemoryFileSystem.test();
    final Artifacts artifacts = Artifacts.test();
    final FlutterValidator flutterValidator = FlutterValidator(
      platform: platform,
      flutterVersion: () => flutterVersion,
      devToolsVersion: () => '2.8.0',
      userMessages: UserMessages(),
      artifacts: artifacts,
      fileSystem: fileSystem,
      processManager: FakeProcessManager.any(),
      operatingSystemUtils: FakeOperatingSystemUtils(name: 'Windows'),
      flutterRoot: () => '/sdk/flutter',
    );

    expect(
      await flutterValidator.validate(),
      _matchDoctorValidation(
        validationType: ValidationType.success,
        statusInfo: 'Channel beta, 1.0.0, on Windows, locale en_US.UTF-8',
        messages: containsAll(const <ValidationMessage>[
          ValidationMessage('Pub download mirror https://example.com/pub'),
          ValidationMessage('Flutter download mirror https://example.com/flutter'),
        ]),
      ),
    );
  });

  testWithoutContext(
    'FlutterValidator shows FLUTTER_GIT_URL when set and fails if upstream is not the same',
    () async {
      final FlutterValidator flutterValidator = FlutterValidator(
        platform: FakePlatform(
          localeName: 'en_US.UTF-8',
          environment: <String, String>{'FLUTTER_GIT_URL': 'https://githubmirror.com/flutter.git'},
        ),
        flutterVersion: () => FakeFlutterVersion(frameworkVersion: '1.0.0', branch: 'beta'),
        devToolsVersion: () => '2.8.0',
        userMessages: UserMessages(),
        artifacts: Artifacts.test(),
        fileSystem: MemoryFileSystem.test(),
        processManager: FakeProcessManager.any(),
        operatingSystemUtils: FakeOperatingSystemUtils(name: 'Linux'),
        flutterRoot: () => '/sdk/flutter',
      );

      expect(
        await flutterValidator.validate(),
        _matchDoctorValidation(
          validationType: ValidationType.partial,
          statusInfo: 'Channel beta, 1.0.0, on Linux, locale en_US.UTF-8',
          messages: containsAll(const <ValidationMessage>[
            ValidationMessage.hint(
              'Upstream repository https://github.com/flutter/flutter.git is not the same as FLUTTER_GIT_URL',
            ),
            ValidationMessage('FLUTTER_GIT_URL = https://githubmirror.com/flutter.git'),
            ValidationMessage(
              'If those were intentional, you can disregard the above warnings; however it is '
              'recommended to use "git" directly to perform update checks and upgrades.',
            ),
          ]),
        ),
      );
    },
  );

  testWithoutContext('FlutterValidator fails when channel is unknown', () async {
    final FlutterValidator flutterValidator = FlutterValidator(
      platform: FakePlatform(localeName: 'en_US.UTF-8'),
      flutterVersion: () => FakeFlutterVersion(branch: 'unknown', frameworkVersion: '1.0.0'),
      devToolsVersion: () => '2.8.0',
      userMessages: UserMessages(),
      artifacts: Artifacts.test(),
      fileSystem: MemoryFileSystem.test(),
      processManager: FakeProcessManager.any(),
      operatingSystemUtils: FakeOperatingSystemUtils(name: 'Linux'),
      flutterRoot: () => '/sdk/flutter',
    );

    expect(
      await flutterValidator.validate(),
      _matchDoctorValidation(
        validationType: ValidationType.partial,
        statusInfo: 'Channel [user-branch], 1.0.0, on Linux, locale en_US.UTF-8',
        messages: containsAll(<ValidationMessage>[
          const ValidationMessage.hint(
            'Flutter version 1.0.0 on channel [user-branch] at /sdk/flutter\n'
            'Currently on an unknown channel. Run `flutter channel` to switch to an official channel.\n'
            "If that doesn't fix the issue, reinstall Flutter by following instructions at https://flutter.dev/setup.",
          ),
          const ValidationMessage(
            'If those were intentional, you can disregard the above warnings; however it is '
            'recommended to use "git" directly to perform update checks and upgrades.',
          ),
        ]),
      ),
    );
  });

  testWithoutContext('FlutterValidator fails when framework version is unknown', () async {
    final FlutterValidator flutterValidator = FlutterValidator(
      platform: FakePlatform(localeName: 'en_US.UTF-8'),
      flutterVersion: () => FakeFlutterVersion(frameworkVersion: '0.0.0-unknown', branch: 'beta'),
      devToolsVersion: () => '2.8.0',
      userMessages: UserMessages(),
      artifacts: Artifacts.test(),
      fileSystem: MemoryFileSystem.test(),
      processManager: FakeProcessManager.any(),
      operatingSystemUtils: FakeOperatingSystemUtils(name: 'Linux'),
      flutterRoot: () => '/sdk/flutter',
    );

    expect(
      await flutterValidator.validate(),
      _matchDoctorValidation(
        validationType: ValidationType.partial,
        statusInfo: 'Channel beta, 0.0.0-unknown, on Linux, locale en_US.UTF-8',
        messages: containsAll(<ValidationMessage>[
          const ValidationMessage.hint(
            'Flutter version 0.0.0-unknown on channel beta at /sdk/flutter\n'
            'Cannot resolve current version, possibly due to local changes.\n'
            'Reinstall Flutter by following instructions at https://flutter.dev/setup.',
          ),
          const ValidationMessage(
            'If those were intentional, you can disregard the above warnings; however it is '
            'recommended to use "git" directly to perform update checks and upgrades.',
          ),
        ]),
      ),
    );
  });

  group('FlutterValidator shows flutter upstream remote', () {
    testWithoutContext('standard url', () async {
      final FlutterValidator flutterValidator = FlutterValidator(
        platform: FakePlatform(localeName: 'en_US.UTF-8'),
        flutterVersion: () => FakeFlutterVersion(frameworkVersion: '1.0.0', branch: 'beta'),
        devToolsVersion: () => '2.8.0',
        userMessages: UserMessages(),
        artifacts: Artifacts.test(),
        fileSystem: MemoryFileSystem.test(),
        processManager: FakeProcessManager.any(),
        operatingSystemUtils: FakeOperatingSystemUtils(name: 'Linux'),
        flutterRoot: () => '/sdk/flutter',
      );

      expect(
        await flutterValidator.validate(),
        _matchDoctorValidation(
          validationType: ValidationType.success,
          statusInfo: 'Channel beta, 1.0.0, on Linux, locale en_US.UTF-8',
          messages: contains(
            const ValidationMessage('Upstream repository https://github.com/flutter/flutter.git'),
          ),
        ),
      );
    });

    testWithoutContext('non-standard url', () async {
      final FlutterValidator flutterValidator = FlutterValidator(
        platform: FakePlatform(localeName: 'en_US.UTF-8'),
        flutterVersion:
            () => FakeFlutterVersion(
              frameworkVersion: '1.0.0',
              branch: 'beta',
              repositoryUrl: 'https://githubmirror.com/flutter.git',
            ),
        devToolsVersion: () => '2.8.0',
        userMessages: UserMessages(),
        artifacts: Artifacts.test(),
        fileSystem: MemoryFileSystem.test(),
        processManager: FakeProcessManager.any(),
        operatingSystemUtils: FakeOperatingSystemUtils(name: 'Linux'),
        flutterRoot: () => 'sdk/flutter',
      );

      expect(
        await flutterValidator.validate(),
        _matchDoctorValidation(
          validationType: ValidationType.partial,
          statusInfo: 'Channel beta, 1.0.0, on Linux, locale en_US.UTF-8',
          messages: containsAll(<ValidationMessage>[
            const ValidationMessage.hint(
              'Upstream repository https://githubmirror.com/flutter.git is not a standard remote.\n'
              'Set environment variable "FLUTTER_GIT_URL" to '
              'https://githubmirror.com/flutter.git to dismiss this error.',
            ),
            const ValidationMessage(
              'If those were intentional, you can disregard the above warnings; however it is '
              'recommended to use "git" directly to perform update checks and upgrades.',
            ),
          ]),
        ),
      );
    });

    testWithoutContext('as unknown if upstream is null', () async {
      final FlutterValidator flutterValidator = FlutterValidator(
        platform: FakePlatform(localeName: 'en_US.UTF-8'),
        flutterVersion:
            () =>
                FakeFlutterVersion(frameworkVersion: '1.0.0', branch: 'beta', repositoryUrl: null),
        devToolsVersion: () => '2.8.0',
        userMessages: UserMessages(),
        artifacts: Artifacts.test(),
        fileSystem: MemoryFileSystem.test(),
        processManager: FakeProcessManager.any(),
        operatingSystemUtils: FakeOperatingSystemUtils(name: 'Linux'),
        flutterRoot: () => 'sdk/flutter',
      );

      expect(
        await flutterValidator.validate(),
        _matchDoctorValidation(
          validationType: ValidationType.partial,
          statusInfo: 'Channel beta, 1.0.0, on Linux, locale en_US.UTF-8',
          messages: containsAll(<ValidationMessage>[
            const ValidationMessage.hint(
              'Unknown upstream repository.\n'
              'Reinstall Flutter by following instructions at https://flutter.dev/setup.',
            ),
            const ValidationMessage(
              'If those were intentional, you can disregard the above warnings; however it is '
              'recommended to use "git" directly to perform update checks and upgrades.',
            ),
          ]),
        ),
      );
    });
  });

  testWithoutContext(
    'Do not show the message for intentional errors if FlutterValidator passes',
    () async {
      final FlutterValidator flutterValidator = FlutterValidator(
        platform: FakePlatform(localeName: 'en_US.UTF-8'),
        flutterVersion: () => FakeFlutterVersion(frameworkVersion: '1.0.0', branch: 'beta'),
        devToolsVersion: () => '2.8.0',
        userMessages: UserMessages(),
        artifacts: Artifacts.test(),
        fileSystem: MemoryFileSystem.test(),
        processManager: FakeProcessManager.any(),
        operatingSystemUtils: FakeOperatingSystemUtils(name: 'Linux'),
        flutterRoot: () => '/sdk/flutter',
      );

      expect(
        await flutterValidator.validate(),
        _matchDoctorValidation(
          validationType: ValidationType.success,
          statusInfo: 'Channel beta, 1.0.0, on Linux, locale en_US.UTF-8',
          messages: isNot(
            contains(
              const ValidationMessage(
                'If those were intentional, you can disregard the above warnings; however it is '
                'recommended to use "git" directly to perform update checks and upgrades.',
              ),
            ),
          ),
        ),
      );
    },
  );

  testWithoutContext('detects no flutter and dart on path', () async {
    const String flutterRoot = 'sdk/flutter';
    final FlutterValidator flutterValidator = FlutterValidator(
      platform: FakePlatform(localeName: 'en_US.UTF-8'),
      flutterVersion: () => FakeFlutterVersion(frameworkVersion: '1.0.0', branch: 'beta'),
      devToolsVersion: () => '2.8.0',
      userMessages: UserMessages(),
      artifacts: Artifacts.test(),
      fileSystem: MemoryFileSystem.test(),
      processManager: FakeProcessManager.any(),
      operatingSystemUtils: FakeOperatingSystemUtils(
        name: 'Linux',
        whichLookup: const <String, File>{},
      ),
      flutterRoot: () => flutterRoot,
    );

    expect(
      await flutterValidator.validate(),
      _matchDoctorValidation(
        validationType: ValidationType.partial,
        statusInfo: 'Channel beta, 1.0.0, on Linux, locale en_US.UTF-8',
        messages: contains(
          const ValidationMessage.hint(
            'The flutter binary is not on your path. Consider adding $flutterRoot/bin to your path.',
          ),
        ),
      ),
    );
  });

  testWithoutContext('allows case differences in paths on Windows', () async {
    const String flutterRoot = r'c:\path\to\flutter-sdk';
    const String osName = 'Microsoft Windows';
    final MemoryFileSystem fs = MemoryFileSystem.test(style: FileSystemStyle.windows);
    // The windows' file system is not case sensitive, so changing the case
    // here should not matter.
    final File flutterBinary = fs.file('${flutterRoot.toUpperCase()}\\bin\\flutter')
      ..createSync(recursive: true);
    final FlutterValidator flutterValidator = FlutterValidator(
      platform: FakePlatform(operatingSystem: 'windows', localeName: 'en_US.UTF-8'),
      flutterVersion: () => FakeFlutterVersion(frameworkVersion: '1.0.0', branch: 'beta'),
      devToolsVersion: () => '2.8.0',
      userMessages: UserMessages(),
      artifacts: Artifacts.test(),
      fileSystem: fs,
      processManager: FakeProcessManager.empty(),
      operatingSystemUtils: FakeOperatingSystemUtils(
        name: osName,
        whichLookup: <String, File>{'flutter': flutterBinary},
      ),
      flutterRoot: () => flutterRoot,
    );

    expect(
      await flutterValidator.validate(),
      _matchDoctorValidation(
        validationType: ValidationType.partial,
        statusInfo: 'Channel beta, 1.0.0, on $osName, locale en_US.UTF-8',
        messages: everyElement(
          isA<ValidationMessage>().having(
            (ValidationMessage message) => message.message,
            'message',
            isNot(contains('Warning: `flutter` on your path resolves to')),
          ),
        ),
      ),
    );
  });

  testWithoutContext('allows different separator types in paths on Windows', () async {
    const String flutterRoot = r'c:\path\to\flutter-sdk';
    const String osName = 'Microsoft Windows';
    final MemoryFileSystem fs = MemoryFileSystem.test(style: FileSystemStyle.windows);
    const String filePath = '$flutterRoot\\bin\\flutter';
    // force posix style path separators
    final File flutterBinary = fs.file(filePath.replaceAll(r'\', '/'))..createSync(recursive: true);
    final FlutterValidator flutterValidator = FlutterValidator(
      platform: FakePlatform(operatingSystem: 'windows', localeName: 'en_US.UTF-8'),
      flutterVersion: () => FakeFlutterVersion(frameworkVersion: '1.0.0', branch: 'beta'),
      devToolsVersion: () => '2.8.0',
      userMessages: UserMessages(),
      artifacts: Artifacts.test(),
      fileSystem: fs,
      processManager: FakeProcessManager.empty(),
      operatingSystemUtils: FakeOperatingSystemUtils(
        name: osName,
        whichLookup: <String, File>{'flutter': flutterBinary},
      ),
      flutterRoot: () => flutterRoot,
    );

    expect(
      await flutterValidator.validate(),
      _matchDoctorValidation(
        validationType: ValidationType.partial,
        statusInfo: 'Channel beta, 1.0.0, on $osName, locale en_US.UTF-8',
        messages: everyElement(
          isA<ValidationMessage>().having(
            (ValidationMessage message) => message.message,
            'message',
            isNot(contains('Warning: `flutter` on your path resolves to')),
          ),
        ),
      ),
    );
  });

  testWithoutContext('detects flutter and dart from outside flutter sdk', () async {
    final FileSystem fs = MemoryFileSystem.test();
    final FlutterValidator flutterValidator = FlutterValidator(
      platform: FakePlatform(localeName: 'en_US.UTF-8'),
      flutterVersion: () => FakeFlutterVersion(frameworkVersion: '1.0.0', branch: 'beta'),
      devToolsVersion: () => '2.8.0',
      userMessages: UserMessages(),
      artifacts: Artifacts.test(),
      fileSystem: fs,
      processManager: FakeProcessManager.any(),
      operatingSystemUtils: FakeOperatingSystemUtils(
        name: 'Linux',
        whichLookup: <String, File>{
          'flutter': fs.file('/sdk/flutter-beta')..createSync(recursive: true),
          'dart': fs.file('/sdk/flutter-beta')..createSync(recursive: true),
        },
      ),
      flutterRoot: () => '/sdk/flutter',
    );

    expect(
      await flutterValidator.validate(),
      _matchDoctorValidation(
        validationType: ValidationType.partial,
        statusInfo: 'Channel beta, 1.0.0, on Linux, locale en_US.UTF-8',
        messages: contains(
          const ValidationMessage.hint(
            'Warning: `flutter` on your path resolves to /sdk/flutter-beta, which '
            'is not inside your current Flutter SDK checkout at /sdk/flutter. '
            'Consider adding /sdk/flutter/bin to the front of your path.',
          ),
        ),
      ),
    );
  });

  testWithoutContext('no warnings if flutter & dart binaries are inside the Flutter SDK', () async {
    final FileSystem fs = MemoryFileSystem.test();
    final FlutterValidator flutterValidator = FlutterValidator(
      platform: FakePlatform(localeName: 'en_US.UTF-8'),
      flutterVersion: () => FakeFlutterVersion(frameworkVersion: '1.0.0', branch: 'beta'),
      devToolsVersion: () => '2.8.0',
      userMessages: UserMessages(),
      artifacts: Artifacts.test(),
      fileSystem: fs,
      processManager: FakeProcessManager.any(),
      operatingSystemUtils: FakeOperatingSystemUtils(
        name: 'Linux',
        whichLookup: <String, File>{
          'flutter': fs.file('/sdk/flutter/bin/flutter')..createSync(recursive: true),
          'dart': fs.file('/sdk/flutter/bin/dart')..createSync(recursive: true),
        },
      ),
      flutterRoot: () => '/sdk/flutter',
    );

    expect(
      await flutterValidator.validate(),
      _matchDoctorValidation(
        validationType: ValidationType.success,
        statusInfo: 'Channel beta, 1.0.0, on Linux, locale en_US.UTF-8',
        messages: isNot(
          contains(
            isA<ValidationMessage>().having(
              (ValidationMessage message) => message.message,
              'message',
              contains('Consider adding /sdk/flutter/bin to the front of your path'),
            ),
          ),
        ),
      ),
    );
  });
}

class FakeOperatingSystemUtils extends Fake implements OperatingSystemUtils {
  FakeOperatingSystemUtils({
    required this.name,
    this.hostPlatform = HostPlatform.linux_x64,
    this.whichLookup,
    FileSystem? fs,
  }) {
    fs ??= MemoryFileSystem.test();
    whichLookup ??= <String, File>{
      'flutter': fs.file('/sdk/flutter/bin/flutter')..createSync(recursive: true),
      'dart': fs.file('/sdk/flutter/bin/dart')..createSync(recursive: true),
    };
  }

  /// A map of [File]s that calls to [which] will return.
  Map<String, File>? whichLookup;

  @override
  File? which(String execName) => whichLookup![execName];

  @override
  final String name;

  @override
  final HostPlatform hostPlatform;
}

class FakeThrowingFlutterVersion extends FakeFlutterVersion {
  @override
  String get channel => 'beta';

  @override
  String get frameworkCommitDate {
    throw VersionCheckError('version error');
  }
}
