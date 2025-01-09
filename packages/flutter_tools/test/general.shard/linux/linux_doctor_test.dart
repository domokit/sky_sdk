// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/user_messages.dart';
import 'package:flutter_tools/src/doctor_validator.dart';
import 'package:flutter_tools/src/linux/linux_doctor.dart';

import '../../src/common.dart';
import '../../src/fake_process_manager.dart';

// A command that will return typical-looking 'clang++ --version' output with
// the given version number.
FakeCommand _clangPresentCommand(String version) {
  return FakeCommand(
    command: const <String>['clang++', '--version'],
    stdout: '''
clang version $version-6+build1
Target: x86_64-pc-linux-gnu
Thread model: posix
InstalledDir: /usr/bin
''',
  );
}

// A command that will return typical-looking 'cmake --version' output with the
// given version number.
FakeCommand _cmakePresentCommand(String version) {
  return FakeCommand(
    command: const <String>['cmake', '--version'],
    stdout: '''
cmake version $version

CMake suite maintained and supported by Kitware (kitware.com/cmake).
''',
  );
}

// A command that will return typical-looking 'ninja --version' output with the
// given version number.
FakeCommand _ninjaPresentCommand(String version) {
  return FakeCommand(command: const <String>['ninja', '--version'], stdout: version);
}

// A command that will return typical-looking 'pkg-config --version' output with
// the given version number.
FakeCommand _pkgConfigPresentCommand(String version) {
  return FakeCommand(command: const <String>['pkg-config', '--version'], stdout: version);
}

/// A command that returns either success or failure for a pkg-config query
/// for [library], depending on [exists].
FakeCommand _libraryCheckCommand(String library, {bool exists = true}) {
  return FakeCommand(
    command: <String>['pkg-config', '--exists', library],
    exitCode: exists ? 0 : 1,
  );
}

// Commands that give positive replies for all the GTK library pkg-config queries.
List<FakeCommand> _gtkLibrariesPresentCommands() {
  return <FakeCommand>[
    _libraryCheckCommand('gtk+-3.0'),
    _libraryCheckCommand('glib-2.0'),
    _libraryCheckCommand('gio-2.0'),
  ];
}

// Commands that give some failures for the GTK library pkg-config queries.
List<FakeCommand> _gtkLibrariesMissingCommands() {
  return <FakeCommand>[
    _libraryCheckCommand('gtk+-3.0'),
    _libraryCheckCommand('glib-2.0', exists: false),
    // No more entries, since the first missing GTK library stops the
    // checks.
  ];
}

// A command that will failure when running '[binary] --version'.
FakeCommand _missingBinaryCommand(String binary) {
  return FakeCommand(command: <String>[binary, '--version'], exitCode: 1);
}

FakeCommand _missingBinaryException(String binary) {
  return FakeCommand(
    command: <String>[binary, '--version'],
    exitCode: 1,
    exception: ProcessException(binary, <String>[]),
  );
}

void main() {
  testWithoutContext(
    'Full validation when everything is available at the necessary version',
    () async {
      final ProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        _clangPresentCommand('4.0.1'),
        _cmakePresentCommand('3.16.3'),
        _ninjaPresentCommand('1.10.0'),
        _pkgConfigPresentCommand('0.29'),
        ..._gtkLibrariesPresentCommands(),
      ]);
      final DoctorValidator linuxDoctorValidator = LinuxDoctorValidator(
        processManager: processManager,
        userMessages: UserMessages(),
      );
      final ValidationResult result = await linuxDoctorValidator.validate();

      expect(result.type, ValidationType.success);
      expect(result.messages, const <ValidationMessage>[
        ValidationMessage('clang version 4.0.1-6+build1'),
        ValidationMessage('cmake version 3.16.3'),
        ValidationMessage('ninja version 1.10.0'),
        ValidationMessage('pkg-config version 0.29'),
      ]);
    },
  );

  testWithoutContext('Partial validation when clang++ version is too old', () async {
    final ProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      _clangPresentCommand('2.0.1'),
      _cmakePresentCommand('3.16.3'),
      _ninjaPresentCommand('1.10.0'),
      _pkgConfigPresentCommand('0.29'),
      ..._gtkLibrariesPresentCommands(),
    ]);
    final DoctorValidator linuxDoctorValidator = LinuxDoctorValidator(
      processManager: processManager,
      userMessages: UserMessages(),
    );
    final ValidationResult result = await linuxDoctorValidator.validate();

    expect(result.type, ValidationType.partial);
    expect(result.messages, const <ValidationMessage>[
      ValidationMessage('clang version 2.0.1-6+build1'),
      ValidationMessage.error('clang++ 3.4.0 or later is required.'),
      ValidationMessage('cmake version 3.16.3'),
      ValidationMessage('ninja version 1.10.0'),
      ValidationMessage('pkg-config version 0.29'),
    ]);
  });

  testWithoutContext('Partial validation when CMake version is too old', () async {
    final ProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      _clangPresentCommand('4.0.1'),
      _cmakePresentCommand('3.2.0'),
      _ninjaPresentCommand('1.10.0'),
      _pkgConfigPresentCommand('0.29'),
      ..._gtkLibrariesPresentCommands(),
    ]);
    final DoctorValidator linuxDoctorValidator = LinuxDoctorValidator(
      processManager: processManager,
      userMessages: UserMessages(),
    );
    final ValidationResult result = await linuxDoctorValidator.validate();

    expect(result.type, ValidationType.partial);
    expect(result.messages, const <ValidationMessage>[
      ValidationMessage('clang version 4.0.1-6+build1'),
      ValidationMessage('cmake version 3.2.0'),
      ValidationMessage.error('cmake 3.10.0 or later is required.'),
      ValidationMessage('ninja version 1.10.0'),
      ValidationMessage('pkg-config version 0.29'),
    ]);
  });

  testWithoutContext('Partial validation when ninja version is too old', () async {
    final ProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      _clangPresentCommand('4.0.1'),
      _cmakePresentCommand('3.16.3'),
      _ninjaPresentCommand('0.8.1'),
      _pkgConfigPresentCommand('0.29'),
      ..._gtkLibrariesPresentCommands(),
    ]);
    final DoctorValidator linuxDoctorValidator = LinuxDoctorValidator(
      processManager: processManager,
      userMessages: UserMessages(),
    );
    final ValidationResult result = await linuxDoctorValidator.validate();

    expect(result.type, ValidationType.partial);
    expect(result.messages, const <ValidationMessage>[
      ValidationMessage('clang version 4.0.1-6+build1'),
      ValidationMessage('cmake version 3.16.3'),
      ValidationMessage('ninja version 0.8.1'),
      ValidationMessage.error('ninja 1.8.0 or later is required.'),
      ValidationMessage('pkg-config version 0.29'),
    ]);
  });

  testWithoutContext('Partial validation when pkg-config version is too old', () async {
    final ProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      _clangPresentCommand('4.0.1'),
      _cmakePresentCommand('3.16.3'),
      _ninjaPresentCommand('1.10.0'),
      _pkgConfigPresentCommand('0.27.0'),
      ..._gtkLibrariesPresentCommands(),
    ]);
    final DoctorValidator linuxDoctorValidator = LinuxDoctorValidator(
      processManager: processManager,
      userMessages: UserMessages(),
    );
    final ValidationResult result = await linuxDoctorValidator.validate();

    expect(result.type, ValidationType.partial);
    expect(result.messages, const <ValidationMessage>[
      ValidationMessage('clang version 4.0.1-6+build1'),
      ValidationMessage('cmake version 3.16.3'),
      ValidationMessage('ninja version 1.10.0'),
      ValidationMessage('pkg-config version 0.27.0'),
      ValidationMessage.error('pkg-config 0.29.0 or later is required.'),
    ]);
  });

  testWithoutContext('Missing validation when pkg-config is missing', () async {
    final UserMessages userMessages = UserMessages();
    final ProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      _clangPresentCommand('4.0.1'),
      _cmakePresentCommand('3.16.3'),
      _ninjaPresentCommand('1.10.0'),
      _missingBinaryException('pkg-config'),
      // We never check libraries because pkg-config is not present
    ]);
    final DoctorValidator linuxDoctorValidator = LinuxDoctorValidator(
      processManager: processManager,
      userMessages: userMessages,
    );
    final ValidationResult result = await linuxDoctorValidator.validate();

    expect(result.type, ValidationType.missing);
    expect(result.messages, <ValidationMessage>[
      const ValidationMessage('clang version 4.0.1-6+build1'),
      const ValidationMessage('cmake version 3.16.3'),
      const ValidationMessage('ninja version 1.10.0'),
      ValidationMessage.error(userMessages.pkgConfigMissing),
    ]);
  });

  testWithoutContext('Missing validation when CMake is not available', () async {
    final ProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      _clangPresentCommand('4.0.1'),
      _missingBinaryCommand('cmake'),
      _ninjaPresentCommand('1.10.0'),
      _pkgConfigPresentCommand('0.29'),
      ..._gtkLibrariesPresentCommands(),
    ]);
    final UserMessages userMessages = UserMessages();
    final DoctorValidator linuxDoctorValidator = LinuxDoctorValidator(
      processManager: processManager,
      userMessages: userMessages,
    );
    final ValidationResult result = await linuxDoctorValidator.validate();

    expect(result.type, ValidationType.missing);
    expect(result.messages, <ValidationMessage>[
      const ValidationMessage('clang version 4.0.1-6+build1'),
      ValidationMessage.error(userMessages.cmakeMissing),
      const ValidationMessage('ninja version 1.10.0'),
      const ValidationMessage('pkg-config version 0.29'),
    ]);
  });

  testWithoutContext('Missing validation when CMake version is unparsable', () async {
    final ProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      _clangPresentCommand('4.0.1'),
      _cmakePresentCommand('bogus'),
      _ninjaPresentCommand('1.10.0'),
      _pkgConfigPresentCommand('0.29'),
      ..._gtkLibrariesPresentCommands(),
    ]);
    final UserMessages userMessages = UserMessages();
    final DoctorValidator linuxDoctorValidator = LinuxDoctorValidator(
      processManager: processManager,
      userMessages: userMessages,
    );
    final ValidationResult result = await linuxDoctorValidator.validate();

    expect(result.type, ValidationType.missing);
    expect(result.messages, <ValidationMessage>[
      const ValidationMessage('clang version 4.0.1-6+build1'),
      ValidationMessage.error(userMessages.cmakeMissing),
      const ValidationMessage('ninja version 1.10.0'),
      const ValidationMessage('pkg-config version 0.29'),
    ]);
  });

  testWithoutContext('Missing validation when clang++ is not available', () async {
    final ProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      _missingBinaryException('clang++'),
      _cmakePresentCommand('3.16.3'),
      _ninjaPresentCommand('1.10.0'),
      _pkgConfigPresentCommand('0.29'),
      ..._gtkLibrariesPresentCommands(),
    ]);
    final UserMessages userMessages = UserMessages();
    final DoctorValidator linuxDoctorValidator = LinuxDoctorValidator(
      processManager: processManager,
      userMessages: userMessages,
    );
    final ValidationResult result = await linuxDoctorValidator.validate();

    expect(result.type, ValidationType.missing);
    expect(result.messages, <ValidationMessage>[
      ValidationMessage.error(userMessages.clangMissing),
      const ValidationMessage('cmake version 3.16.3'),
      const ValidationMessage('ninja version 1.10.0'),
      const ValidationMessage('pkg-config version 0.29'),
    ]);
  });

  testWithoutContext('Missing validation when clang++ version is unparsable', () async {
    final ProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      _clangPresentCommand('bogus'),
      _cmakePresentCommand('3.16.3'),
      _ninjaPresentCommand('1.10.0'),
      _pkgConfigPresentCommand('0.29'),
      ..._gtkLibrariesPresentCommands(),
    ]);
    final UserMessages userMessages = UserMessages();
    final DoctorValidator linuxDoctorValidator = LinuxDoctorValidator(
      processManager: processManager,
      userMessages: userMessages,
    );
    final ValidationResult result = await linuxDoctorValidator.validate();

    expect(result.type, ValidationType.missing);
    expect(result.messages, <ValidationMessage>[
      ValidationMessage.error(userMessages.clangMissing),
      const ValidationMessage('cmake version 3.16.3'),
      const ValidationMessage('ninja version 1.10.0'),
      const ValidationMessage('pkg-config version 0.29'),
    ]);
  });

  testWithoutContext('Missing validation when ninja is not available', () async {
    final ProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      _clangPresentCommand('4.0.1'),
      _cmakePresentCommand('3.16.3'),
      _missingBinaryCommand('ninja'),
      _pkgConfigPresentCommand('0.29'),
      ..._gtkLibrariesPresentCommands(),
    ]);
    final UserMessages userMessages = UserMessages();
    final DoctorValidator linuxDoctorValidator = LinuxDoctorValidator(
      processManager: processManager,
      userMessages: userMessages,
    );
    final ValidationResult result = await linuxDoctorValidator.validate();

    expect(result.type, ValidationType.missing);
    expect(result.messages, <ValidationMessage>[
      const ValidationMessage('clang version 4.0.1-6+build1'),
      const ValidationMessage('cmake version 3.16.3'),
      ValidationMessage.error(userMessages.ninjaMissing),
      const ValidationMessage('pkg-config version 0.29'),
    ]);
  });

  testWithoutContext('Missing validation when ninja version is unparsable', () async {
    final ProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      _clangPresentCommand('4.0.1'),
      _cmakePresentCommand('3.16.3'),
      _ninjaPresentCommand('bogus'),
      _pkgConfigPresentCommand('0.29'),
      ..._gtkLibrariesPresentCommands(),
    ]);
    final UserMessages userMessages = UserMessages();
    final DoctorValidator linuxDoctorValidator = LinuxDoctorValidator(
      processManager: processManager,
      userMessages: userMessages,
    );
    final ValidationResult result = await linuxDoctorValidator.validate();

    expect(result.type, ValidationType.missing);
    expect(result.messages, <ValidationMessage>[
      const ValidationMessage('clang version 4.0.1-6+build1'),
      const ValidationMessage('cmake version 3.16.3'),
      ValidationMessage.error(userMessages.ninjaMissing),
      const ValidationMessage('pkg-config version 0.29'),
    ]);
  });

  testWithoutContext('Missing validation when pkg-config is not available', () async {
    final ProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      _clangPresentCommand('4.0.1'),
      _cmakePresentCommand('3.16.3'),
      _ninjaPresentCommand('1.10.0'),
      _missingBinaryCommand('pkg-config'),
      ..._gtkLibrariesPresentCommands(),
    ]);
    final UserMessages userMessages = UserMessages();
    final DoctorValidator linuxDoctorValidator = LinuxDoctorValidator(
      processManager: processManager,
      userMessages: userMessages,
    );
    final ValidationResult result = await linuxDoctorValidator.validate();

    expect(result.type, ValidationType.missing);
    expect(result.messages, <ValidationMessage>[
      const ValidationMessage('clang version 4.0.1-6+build1'),
      const ValidationMessage('cmake version 3.16.3'),
      const ValidationMessage('ninja version 1.10.0'),
      ValidationMessage.error(userMessages.pkgConfigMissing),
    ]);
  });

  testWithoutContext('Missing validation when pkg-config version is unparsable', () async {
    final ProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      _clangPresentCommand('4.0.1'),
      _cmakePresentCommand('3.16.3'),
      _ninjaPresentCommand('1.10.0'),
      _pkgConfigPresentCommand('bogus'),
      ..._gtkLibrariesPresentCommands(),
    ]);
    final UserMessages userMessages = UserMessages();
    final DoctorValidator linuxDoctorValidator = LinuxDoctorValidator(
      processManager: processManager,
      userMessages: userMessages,
    );
    final ValidationResult result = await linuxDoctorValidator.validate();

    expect(result.type, ValidationType.missing);
    expect(result.messages, <ValidationMessage>[
      const ValidationMessage('clang version 4.0.1-6+build1'),
      const ValidationMessage('cmake version 3.16.3'),
      const ValidationMessage('ninja version 1.10.0'),
      ValidationMessage.error(userMessages.pkgConfigMissing),
    ]);
  });

  testWithoutContext('Missing validation when GTK libraries are not available', () async {
    final ProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      _clangPresentCommand('4.0.1'),
      _cmakePresentCommand('3.16.3'),
      _ninjaPresentCommand('1.10.0'),
      _pkgConfigPresentCommand('0.29'),
      ..._gtkLibrariesMissingCommands(),
    ]);
    final UserMessages userMessages = UserMessages();
    final DoctorValidator linuxDoctorValidator = LinuxDoctorValidator(
      processManager: processManager,
      userMessages: userMessages,
    );
    final ValidationResult result = await linuxDoctorValidator.validate();

    expect(result.type, ValidationType.missing);
    expect(result.messages, <ValidationMessage>[
      const ValidationMessage('clang version 4.0.1-6+build1'),
      const ValidationMessage('cmake version 3.16.3'),
      const ValidationMessage('ninja version 1.10.0'),
      const ValidationMessage('pkg-config version 0.29'),
      ValidationMessage.error(userMessages.gtkLibrariesMissing),
    ]);
  });

  testWithoutContext('Missing validation when multiple dependencies are not available', () async {
    final ProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      _missingBinaryCommand('clang++'),
      _missingBinaryCommand('cmake'),
      _ninjaPresentCommand('1.10.0'),
      _pkgConfigPresentCommand('0.29'),
      ..._gtkLibrariesPresentCommands(),
    ]);
    final DoctorValidator linuxDoctorValidator = LinuxDoctorValidator(
      processManager: processManager,
      userMessages: UserMessages(),
    );

    final ValidationResult result = await linuxDoctorValidator.validate();
    expect(result.type, ValidationType.missing);
  });
}
