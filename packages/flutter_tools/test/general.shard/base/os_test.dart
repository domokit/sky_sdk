// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:archive/archive.dart';
import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/os.dart';
import 'package:flutter_tools/src/base/platform.dart';

import '../../src/common.dart';
import '../../src/fake_process_manager.dart';

const String kExecutable = 'foo';
const String kPath1 = '/bar/bin/$kExecutable';
const String kPath2 = '/another/bin/$kExecutable';

void main() {
  late FakeProcessManager fakeProcessManager;

  setUp(() {
    fakeProcessManager = FakeProcessManager.empty();
  });

  OperatingSystemUtils createOSUtils(Platform platform) {
    return OperatingSystemUtils(
      fileSystem: MemoryFileSystem.test(),
      logger: BufferLogger.test(),
      platform: platform,
      processManager: fakeProcessManager,
    );
  }

  group('which on POSIX', () {
    testWithoutContext('returns null when executable does not exist', () async {
      fakeProcessManager.addCommand(
        const FakeCommand(command: <String>['which', kExecutable], exitCode: 1),
      );
      final OperatingSystemUtils utils = createOSUtils(FakePlatform());
      expect(utils.which(kExecutable), isNull);
    });

    testWithoutContext('returns exactly one result', () async {
      fakeProcessManager.addCommand(
        const FakeCommand(command: <String>['which', 'foo'], stdout: kPath1),
      );
      final OperatingSystemUtils utils = createOSUtils(FakePlatform());
      expect(utils.which(kExecutable)!.path, kPath1);
    });

    testWithoutContext('returns all results for whichAll', () async {
      fakeProcessManager.addCommand(
        const FakeCommand(
          command: <String>['which', '-a', kExecutable],
          stdout: '$kPath1\n$kPath2',
        ),
      );
      final OperatingSystemUtils utils = createOSUtils(FakePlatform());
      final List<File> result = utils.whichAll(kExecutable);
      expect(result, hasLength(2));
      expect(result[0].path, kPath1);
      expect(result[1].path, kPath2);
    });
  });

  group('which on Windows', () {
    testWithoutContext('throws tool exit if where.exe cannot be run', () async {
      fakeProcessManager.excludedExecutables.add('where');

      final OperatingSystemUtils utils = OperatingSystemUtils(
        fileSystem: MemoryFileSystem.test(),
        logger: BufferLogger.test(),
        platform: FakePlatform(operatingSystem: 'windows'),
        processManager: fakeProcessManager,
      );

      expect(() => utils.which(kExecutable), throwsToolExit());
    });

    testWithoutContext('returns null when executable does not exist', () async {
      fakeProcessManager.addCommand(
        const FakeCommand(command: <String>['where', kExecutable], exitCode: 1),
      );

      final OperatingSystemUtils utils = createOSUtils(FakePlatform(operatingSystem: 'windows'));
      expect(utils.which(kExecutable), isNull);
    });

    testWithoutContext('returns exactly one result', () async {
      fakeProcessManager.addCommand(
        const FakeCommand(command: <String>['where', 'foo'], stdout: '$kPath1\n$kPath2'),
      );
      final OperatingSystemUtils utils = createOSUtils(FakePlatform(operatingSystem: 'windows'));
      expect(utils.which(kExecutable)!.path, kPath1);
    });

    testWithoutContext('returns all results for whichAll', () async {
      fakeProcessManager.addCommand(
        const FakeCommand(command: <String>['where', kExecutable], stdout: '$kPath1\n$kPath2'),
      );
      final OperatingSystemUtils utils = createOSUtils(FakePlatform(operatingSystem: 'windows'));
      final List<File> result = utils.whichAll(kExecutable);
      expect(result, hasLength(2));
      expect(result[0].path, kPath1);
      expect(result[1].path, kPath2);
    });
  });

  group('host platform', () {
    testWithoutContext('unknown defaults to Linux', () async {
      fakeProcessManager.addCommand(
        const FakeCommand(command: <String>['uname', '-m'], stdout: 'x86_64'),
      );

      final OperatingSystemUtils utils = createOSUtils(FakePlatform(operatingSystem: 'fuchsia'));
      expect(utils.hostPlatform, HostPlatform.linux_x64);
    });

    testWithoutContext('Windows default', () async {
      final OperatingSystemUtils utils = createOSUtils(FakePlatform(operatingSystem: 'windows'));
      expect(utils.hostPlatform, HostPlatform.windows_x64);
    });

    testWithoutContext('Linux x64', () async {
      fakeProcessManager.addCommand(
        const FakeCommand(command: <String>['uname', '-m'], stdout: 'x86_64'),
      );

      final OperatingSystemUtils utils = createOSUtils(FakePlatform());
      expect(utils.hostPlatform, HostPlatform.linux_x64);
    });

    testWithoutContext('Linux ARM', () async {
      fakeProcessManager.addCommand(
        const FakeCommand(command: <String>['uname', '-m'], stdout: 'aarch64'),
      );

      final OperatingSystemUtils utils = createOSUtils(FakePlatform());
      expect(utils.hostPlatform, HostPlatform.linux_arm64);
    });

    testWithoutContext('macOS ARM', () async {
      fakeProcessManager.addCommands(<FakeCommand>[
        const FakeCommand(command: <String>['which', 'sysctl']),
        const FakeCommand(
          command: <String>['sysctl', 'hw.optional.arm64'],
          stdout: 'hw.optional.arm64: 1',
        ),
      ]);

      final OperatingSystemUtils utils = createOSUtils(FakePlatform(operatingSystem: 'macos'));
      expect(utils.hostPlatform, HostPlatform.darwin_arm64);
    });

    testWithoutContext('macOS 11 x86', () async {
      fakeProcessManager.addCommands(<FakeCommand>[
        const FakeCommand(command: <String>['which', 'sysctl']),
        const FakeCommand(
          command: <String>['sysctl', 'hw.optional.arm64'],
          stdout: 'hw.optional.arm64: 0',
        ),
      ]);

      final OperatingSystemUtils utils = createOSUtils(FakePlatform(operatingSystem: 'macos'));
      expect(utils.hostPlatform, HostPlatform.darwin_x64);
    });

    testWithoutContext('sysctl not found', () async {
      fakeProcessManager.addCommands(<FakeCommand>[
        const FakeCommand(command: <String>['which', 'sysctl'], exitCode: 1),
      ]);

      final OperatingSystemUtils utils = createOSUtils(FakePlatform(operatingSystem: 'macos'));
      expect(() => utils.hostPlatform, throwsToolExit(message: 'sysctl'));
    });

    testWithoutContext('macOS 10 x86', () async {
      fakeProcessManager.addCommands(<FakeCommand>[
        const FakeCommand(command: <String>['which', 'sysctl']),
        const FakeCommand(command: <String>['sysctl', 'hw.optional.arm64'], exitCode: 1),
      ]);

      final OperatingSystemUtils utils = createOSUtils(FakePlatform(operatingSystem: 'macos'));
      expect(utils.hostPlatform, HostPlatform.darwin_x64);
    });

    testWithoutContext('macOS ARM name', () async {
      fakeProcessManager.addCommands(<FakeCommand>[
        const FakeCommand(command: <String>['sw_vers', '-productName'], stdout: 'product'),
        const FakeCommand(command: <String>['sw_vers', '-productVersion'], stdout: 'version'),
        const FakeCommand(command: <String>['sw_vers', '-buildVersion'], stdout: 'build'),
        const FakeCommand(command: <String>['uname', '-m'], stdout: 'arm64'),
        const FakeCommand(command: <String>['which', 'sysctl']),
        const FakeCommand(
          command: <String>['sysctl', 'hw.optional.arm64'],
          stdout: 'hw.optional.arm64: 1',
        ),
      ]);

      final OperatingSystemUtils utils = createOSUtils(FakePlatform(operatingSystem: 'macos'));
      expect(utils.name, 'product version build darwin-arm64');
    });

    testWithoutContext('macOS ARM on Rosetta name', () async {
      fakeProcessManager.addCommands(<FakeCommand>[
        const FakeCommand(command: <String>['sw_vers', '-productName'], stdout: 'product'),
        const FakeCommand(command: <String>['sw_vers', '-productVersion'], stdout: 'version'),
        const FakeCommand(command: <String>['sw_vers', '-buildVersion'], stdout: 'build'),
        const FakeCommand(
          command: <String>['uname', '-m'],
          stdout: 'x86_64', // Running on Rosetta
        ),
        const FakeCommand(command: <String>['which', 'sysctl']),
        const FakeCommand(
          command: <String>['sysctl', 'hw.optional.arm64'],
          stdout: 'hw.optional.arm64: 1',
        ),
      ]);

      final OperatingSystemUtils utils = createOSUtils(FakePlatform(operatingSystem: 'macos'));
      expect(utils.name, 'product version build darwin-arm64 (Rosetta)');
    });

    testWithoutContext('macOS x86 name', () async {
      fakeProcessManager.addCommands(<FakeCommand>[
        const FakeCommand(command: <String>['sw_vers', '-productName'], stdout: 'product'),
        const FakeCommand(command: <String>['sw_vers', '-productVersion'], stdout: 'version'),
        const FakeCommand(command: <String>['sw_vers', '-buildVersion'], stdout: 'build'),
        const FakeCommand(command: <String>['uname', '-m'], stdout: 'x86_64'),
        const FakeCommand(command: <String>['which', 'sysctl']),
        const FakeCommand(command: <String>['sysctl', 'hw.optional.arm64'], exitCode: 1),
      ]);

      final OperatingSystemUtils utils = createOSUtils(FakePlatform(operatingSystem: 'macos'));
      expect(utils.name, 'product version build darwin-x64');
    });

    testWithoutContext('Windows name', () async {
      fakeProcessManager.addCommands(<FakeCommand>[
        const FakeCommand(command: <String>['ver'], stdout: 'version'),
      ]);

      final OperatingSystemUtils utils = createOSUtils(FakePlatform(operatingSystem: 'windows'));
      expect(utils.name, 'version');
    });

    testWithoutContext('Linux name', () async {
      const String fakeOsRelease = '''
      NAME="Name"
      ID=id
      ID_LIKE=id_like
      BUILD_ID=build_id
      PRETTY_NAME="Pretty Name"
      ANSI_COLOR="ansi color"
      HOME_URL="https://home.url/"
      DOCUMENTATION_URL="https://documentation.url/"
      SUPPORT_URL="https://support.url/"
      BUG_REPORT_URL="https://bug.report.url/"
      LOGO=logo
      ''';
      final FileSystem fileSystem = MemoryFileSystem.test();
      fileSystem.directory('/etc').createSync();
      fileSystem.file('/etc/os-release').writeAsStringSync(fakeOsRelease);

      final OperatingSystemUtils utils = OperatingSystemUtils(
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
        platform: FakePlatform(
          operatingSystemVersion: 'Linux 1.2.3-abcd #1 SMP PREEMPT Sat Jan 1 00:00:00 UTC 2000',
        ),
        processManager: fakeProcessManager,
      );
      expect(utils.name, 'Pretty Name 1.2.3-abcd');
    });

    testWithoutContext(
      'Linux name reads from "/usr/lib/os-release" if "/etc/os-release" is missing',
      () async {
        const String fakeOsRelease = '''
      NAME="Name"
      ID=id
      ID_LIKE=id_like
      BUILD_ID=build_id
      PRETTY_NAME="Pretty Name"
      ANSI_COLOR="ansi color"
      HOME_URL="https://home.url/"
      DOCUMENTATION_URL="https://documentation.url/"
      SUPPORT_URL="https://support.url/"
      BUG_REPORT_URL="https://bug.report.url/"
      LOGO=logo
      ''';
        final FileSystem fileSystem = MemoryFileSystem.test();
        fileSystem.directory('/usr/lib').createSync(recursive: true);
        fileSystem.file('/usr/lib/os-release').writeAsStringSync(fakeOsRelease);

        expect(fileSystem.file('/etc/os-release').existsSync(), false);

        final OperatingSystemUtils utils = OperatingSystemUtils(
          fileSystem: fileSystem,
          logger: BufferLogger.test(),
          platform: FakePlatform(
            operatingSystemVersion: 'Linux 1.2.3-abcd #1 SMP PREEMPT Sat Jan 1 00:00:00 UTC 2000',
          ),
          processManager: fakeProcessManager,
        );
        expect(utils.name, 'Pretty Name 1.2.3-abcd');
      },
    );

    testWithoutContext('Linux name when reading "/etc/os-release" fails', () async {
      final FileExceptionHandler handler = FileExceptionHandler();
      final FileSystem fileSystem = MemoryFileSystem.test(opHandle: handler.opHandle);

      fileSystem.directory('/etc').createSync();
      final File osRelease = fileSystem.file('/etc/os-release');

      handler.addError(osRelease, FileSystemOp.read, const FileSystemException());

      final OperatingSystemUtils utils = OperatingSystemUtils(
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
        platform: FakePlatform(
          operatingSystemVersion: 'Linux 1.2.3-abcd #1 SMP PREEMPT Sat Jan 1 00:00:00 UTC 2000',
        ),
        processManager: fakeProcessManager,
      );
      expect(utils.name, 'Linux 1.2.3-abcd');
    });

    testWithoutContext('Linux name omits kernel release if undefined', () async {
      const String fakeOsRelease = '''
      NAME="Name"
      ID=id
      ID_LIKE=id_like
      BUILD_ID=build_id
      PRETTY_NAME="Pretty Name"
      ANSI_COLOR="ansi color"
      HOME_URL="https://home.url/"
      DOCUMENTATION_URL="https://documentation.url/"
      SUPPORT_URL="https://support.url/"
      BUG_REPORT_URL="https://bug.report.url/"
      LOGO=logo
      ''';
      final FileSystem fileSystem = MemoryFileSystem.test();
      fileSystem.directory('/etc').createSync();
      fileSystem.file('/etc/os-release').writeAsStringSync(fakeOsRelease);

      final OperatingSystemUtils utils = OperatingSystemUtils(
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
        platform: FakePlatform(operatingSystemVersion: 'undefinedOperatingSystemVersion'),
        processManager: fakeProcessManager,
      );
      expect(utils.name, 'Pretty Name');
    });

    // See https://snyk.io/research/zip-slip-vulnerability for more context
    testWithoutContext('Windows validates paths when unzipping', () {
      // on POSIX systems we use the `unzip` binary, which will fail to extract
      // files with paths outside the target directory
      final OperatingSystemUtils utils = createOSUtils(FakePlatform(operatingSystem: 'windows'));
      final MemoryFileSystem fs = MemoryFileSystem.test();
      final File fakeZipFile = fs.file('archive.zip');
      final Directory targetDirectory = fs.directory('output')..createSync(recursive: true);
      const String content = 'hello, world!';
      final Archive archive =
          Archive()..addFile(
            // This file would be extracted outside of the target extraction dir
            ArchiveFile(r'..\..\..\Target File.txt', content.length, content.codeUnits),
          );
      final List<int> zipData = ZipEncoder().encode(archive)!;
      fakeZipFile.writeAsBytesSync(zipData);
      expect(
        () => utils.unzip(fakeZipFile, targetDirectory),
        throwsA(
          isA<StateError>().having(
            (StateError error) => error.message,
            'correct error message',
            contains('Tried to extract the file '),
          ),
        ),
      );
    });
  });

  testWithoutContext('If unzip fails, include stderr in exception text', () {
    const String exceptionMessage = 'Something really bad happened.';
    final FileExceptionHandler handler = FileExceptionHandler();
    final FileSystem fileSystem = MemoryFileSystem.test(opHandle: handler.opHandle);

    fakeProcessManager.addCommand(
      const FakeCommand(
        command: <String>['unzip', '-o', '-q', 'bar.zip', '-d', 'foo'],
        exitCode: 1,
        stderr: exceptionMessage,
      ),
    );

    final Directory foo = fileSystem.directory('foo')..createSync();
    final File bar = fileSystem.file('bar.zip')..createSync();
    handler.addError(bar, FileSystemOp.read, const FileSystemException(exceptionMessage));

    final OperatingSystemUtils osUtils = OperatingSystemUtils(
      fileSystem: fileSystem,
      logger: BufferLogger.test(),
      platform: FakePlatform(),
      processManager: fakeProcessManager,
    );

    expect(() => osUtils.unzip(bar, foo), throwsProcessException(message: exceptionMessage));
  });

  group('unzip on macOS', () {
    testWithoutContext('falls back to unzip when rsync cannot run', () {
      final FileSystem fileSystem = MemoryFileSystem.test();
      fakeProcessManager.excludedExecutables.add('rsync');

      final BufferLogger logger = BufferLogger.test();
      final OperatingSystemUtils macOSUtils = OperatingSystemUtils(
        fileSystem: fileSystem,
        logger: logger,
        platform: FakePlatform(operatingSystem: 'macos'),
        processManager: fakeProcessManager,
      );

      final Directory targetDirectory = fileSystem.currentDirectory;
      fakeProcessManager.addCommand(
        FakeCommand(command: <String>['unzip', '-o', '-q', 'foo.zip', '-d', targetDirectory.path]),
      );

      macOSUtils.unzip(fileSystem.file('foo.zip'), targetDirectory);
      expect(fakeProcessManager, hasNoRemainingExpectations);
      expect(logger.traceText, contains('Unable to find rsync'));
    });

    testWithoutContext('unzip and rsyncs', () {
      final FileSystem fileSystem = MemoryFileSystem.test();

      final OperatingSystemUtils macOSUtils = OperatingSystemUtils(
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
        platform: FakePlatform(operatingSystem: 'macos'),
        processManager: fakeProcessManager,
      );

      final Directory targetDirectory = fileSystem.currentDirectory;
      final Directory tempDirectory = fileSystem.systemTempDirectory.childDirectory(
        'flutter_foo.zip.rand0',
      );
      fakeProcessManager.addCommands(<FakeCommand>[
        FakeCommand(
          command: <String>['unzip', '-o', '-q', 'foo.zip', '-d', tempDirectory.path],
          onRun: (_) {
            expect(tempDirectory, exists);
            tempDirectory.childDirectory('dirA').childFile('fileA').createSync(recursive: true);
            tempDirectory.childDirectory('dirB').childFile('fileB').createSync(recursive: true);
          },
        ),
        FakeCommand(
          command: <String>[
            'rsync',
            '-8',
            '-av',
            '--delete',
            tempDirectory.childDirectory('dirA').path,
            targetDirectory.path,
          ],
        ),
        FakeCommand(
          command: <String>[
            'rsync',
            '-8',
            '-av',
            '--delete',
            tempDirectory.childDirectory('dirB').path,
            targetDirectory.path,
          ],
        ),
      ]);

      macOSUtils.unzip(fileSystem.file('foo.zip'), fileSystem.currentDirectory);
      expect(fakeProcessManager, hasNoRemainingExpectations);
      expect(tempDirectory, isNot(exists));
    });
  });

  group('display an install message when unzip cannot be run', () {
    testWithoutContext('Linux', () {
      final FileSystem fileSystem = MemoryFileSystem.test();
      fakeProcessManager.excludedExecutables.add('unzip');

      final OperatingSystemUtils linuxOsUtils = OperatingSystemUtils(
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
        platform: FakePlatform(),
        processManager: fakeProcessManager,
      );

      expect(
        () => linuxOsUtils.unzip(fileSystem.file('foo.zip'), fileSystem.currentDirectory),
        throwsToolExit(
          message:
              'Missing "unzip" tool. Unable to extract foo.zip.\n'
              'Consider running "sudo apt-get install unzip".',
        ),
      );
    });

    testWithoutContext('macOS', () {
      final FileSystem fileSystem = MemoryFileSystem.test();
      fakeProcessManager.excludedExecutables.add('unzip');

      final OperatingSystemUtils macOSUtils = OperatingSystemUtils(
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
        platform: FakePlatform(operatingSystem: 'macos'),
        processManager: fakeProcessManager,
      );

      expect(
        () => macOSUtils.unzip(fileSystem.file('foo.zip'), fileSystem.currentDirectory),
        throwsToolExit(
          message:
              'Missing "unzip" tool. Unable to extract foo.zip.\n'
              'Consider running "brew install unzip".',
        ),
      );
    });

    testWithoutContext('unknown OS', () {
      final FileSystem fileSystem = MemoryFileSystem.test();
      fakeProcessManager.excludedExecutables.add('unzip');

      final OperatingSystemUtils unknownOsUtils = OperatingSystemUtils(
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
        platform: FakePlatform(operatingSystem: 'fuchsia'),
        processManager: fakeProcessManager,
      );

      expect(
        () => unknownOsUtils.unzip(fileSystem.file('foo.zip'), fileSystem.currentDirectory),
        throwsToolExit(
          message:
              'Missing "unzip" tool. Unable to extract foo.zip.\n'
              'Please install unzip.',
        ),
      );
    });
  });

  testWithoutContext('directory size', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final OperatingSystemUtils osUtils = OperatingSystemUtils(
      fileSystem: fileSystem,
      logger: BufferLogger.test(),
      platform: FakePlatform(operatingSystem: 'fuchsia'),
      processManager: fakeProcessManager,
    );

    final Directory directory = fileSystem.systemTempDirectory.childDirectory('test_directory');
    directory.createSync();
    directory.childFile('file1.txt').writeAsBytesSync(List<int>.filled(10, 0));
    directory.childFile('file2.txt').writeAsBytesSync(List<int>.filled(20, 0));
    final Directory subDirectory = directory.childDirectory('sub_directory');
    subDirectory.createSync();
    subDirectory.childFile('file3.txt').writeAsBytesSync(List<int>.filled(15, 0));

    expect(osUtils.getDirectorySize(directory), equals(10 + 20 + 15));
  });

  testWithoutContext('stream compression level', () {
    expect(OperatingSystemUtils.gzipLevel1.level, equals(1));
  });
}
