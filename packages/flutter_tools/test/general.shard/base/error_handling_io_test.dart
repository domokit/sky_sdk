// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io; // flutter_ignore: dart_io_import;

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/common.dart';
import 'package:flutter_tools/src/base/error_handling_io.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:process/process.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/fake_process_manager.dart';

final Platform windowsPlatform = FakePlatform(
  operatingSystem: 'windows',
  environment: <String, String>{},
);

final Platform linuxPlatform = FakePlatform(environment: <String, String>{});

final Platform macOSPlatform = FakePlatform(
  operatingSystem: 'macos',
  environment: <String, String>{},
);

void main() {
  testWithoutContext('deleteIfExists does not delete if file does not exist', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final File file = fileSystem.file('file');

    expect(ErrorHandlingFileSystem.deleteIfExists(file), false);
  });

  testWithoutContext('deleteIfExists deletes if file exists', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final File file = fileSystem.file('file')..createSync();

    expect(ErrorHandlingFileSystem.deleteIfExists(file), true);
  });

  testWithoutContext('create accepts exclusive argument', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    expect(fileSystem.file('file').create(exclusive: true), isNotNull);
  });

  testWithoutContext('deleteIfExists handles separate program deleting file', () {
    final File file = FakeExistsFile()..error = const FileSystemException('', '', OSError('', 2));

    expect(ErrorHandlingFileSystem.deleteIfExists(file), true);
  });

  testWithoutContext('deleteIfExists throws tool exit if file exists on read-only volume', () {
    final FileExceptionHandler exceptionHandler = FileExceptionHandler();
    final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
      delegate: MemoryFileSystem.test(opHandle: exceptionHandler.opHandle),
      platform: linuxPlatform,
    );
    final File file = fileSystem.file('file')..createSync();

    exceptionHandler.addError(
      file,
      FileSystemOp.delete,
      FileSystemException('', file.path, const OSError('', 2)),
    );

    expect(() => ErrorHandlingFileSystem.deleteIfExists(file), throwsToolExit());
  });

  testWithoutContext('deleteIfExists does not tool exit if file exists on read-only '
      'volume and it is run under noExitOnFailure', () {
    final FileExceptionHandler exceptionHandler = FileExceptionHandler();
    final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
      delegate: MemoryFileSystem.test(opHandle: exceptionHandler.opHandle),
      platform: linuxPlatform,
    );
    final File file = fileSystem.file('file')..createSync();

    exceptionHandler.addError(
      file,
      FileSystemOp.delete,
      FileSystemException('', file.path, const OSError('', 2)),
    );

    expect(() {
      ErrorHandlingFileSystem.noExitOnFailure(() {
        ErrorHandlingFileSystem.deleteIfExists(file);
      });
    }, throwsFileSystemException());
  });

  testWithoutContext('deleteIfExists throws tool exit if the path is not found on Windows', () {
    final FileExceptionHandler exceptionHandler = FileExceptionHandler();
    final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
      delegate: MemoryFileSystem.test(opHandle: exceptionHandler.opHandle),
      platform: windowsPlatform,
    );
    final File file = fileSystem.file(fileSystem.path.join('directory', 'file'))
      ..createSync(recursive: true);

    exceptionHandler.addError(
      file,
      FileSystemOp.delete,
      FileSystemException('', file.path, const OSError('', 2)),
    );

    expect(() => ErrorHandlingFileSystem.deleteIfExists(file), throwsToolExit());
  });

  group('throws ToolExit on Windows', () {
    const int kDeviceFull = 112;
    const int kUserMappedSectionOpened = 1224;
    const int kUserPermissionDenied = 5;
    const int kFatalDeviceHardwareError = 483;
    const int kDeviceDoesNotExist = 433;

    late FileExceptionHandler exceptionHandler;

    setUp(() {
      exceptionHandler = FileExceptionHandler();
    });

    testWithoutContext('bypasses error handling when noExitOnFailure is used', () {
      final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
        delegate: MemoryFileSystem.test(opHandle: exceptionHandler.opHandle),
        platform: windowsPlatform,
      );
      final File file = fileSystem.file('file');

      exceptionHandler.addError(
        file,
        FileSystemOp.write,
        FileSystemException('', file.path, const OSError('', kUserPermissionDenied)),
      );
      final Matcher throwsNonToolExit = throwsA(isNot(isA<ToolExit>()));
      expect(
        () => ErrorHandlingFileSystem.noExitOnFailure(() => file.writeAsStringSync('')),
        throwsNonToolExit,
      );

      // nesting does not unconditionally re-enable errors.
      expect(() {
        ErrorHandlingFileSystem.noExitOnFailure(() {
          ErrorHandlingFileSystem.noExitOnFailure(() {});
          file.writeAsStringSync('');
        });
      }, throwsNonToolExit);

      // Check that state does not leak.
      expect(() => file.writeAsStringSync(''), throwsToolExit());
    });

    testWithoutContext('when access is denied', () async {
      final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
        delegate: MemoryFileSystem.test(opHandle: exceptionHandler.opHandle),
        platform: windowsPlatform,
      );
      final File file = fileSystem.file('file');

      exceptionHandler.addError(
        file,
        FileSystemOp.write,
        FileSystemException('', file.path, const OSError('', kUserPermissionDenied)),
      );
      exceptionHandler.addError(
        file,
        FileSystemOp.open,
        FileSystemException('', file.path, const OSError('', kUserPermissionDenied)),
      );
      exceptionHandler.addError(
        file,
        FileSystemOp.create,
        FileSystemException('', file.path, const OSError('', kUserPermissionDenied)),
      );

      const String expectedMessage = 'The flutter tool cannot access the file';
      expect(() async => file.writeAsBytes(<int>[0]), throwsToolExit(message: expectedMessage));
      expect(() async => file.writeAsString(''), throwsToolExit(message: expectedMessage));
      expect(() => file.writeAsBytesSync(<int>[0]), throwsToolExit(message: expectedMessage));
      expect(() => file.writeAsStringSync(''), throwsToolExit(message: expectedMessage));
      expect(() => file.openSync(), throwsToolExit(message: expectedMessage));
      expect(() => file.createSync(), throwsToolExit(message: expectedMessage));
    });

    testWithoutContext('when writing to a full device', () async {
      final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
        delegate: MemoryFileSystem.test(opHandle: exceptionHandler.opHandle),
        platform: windowsPlatform,
      );
      final File file = fileSystem.file('file');

      exceptionHandler.addError(
        file,
        FileSystemOp.write,
        FileSystemException('', file.path, const OSError('', kDeviceFull)),
      );

      const String expectedMessage = 'The target device is full';
      expect(() async => file.writeAsBytes(<int>[0]), throwsToolExit(message: expectedMessage));
      expect(() async => file.writeAsString(''), throwsToolExit(message: expectedMessage));
      expect(() => file.writeAsBytesSync(<int>[0]), throwsToolExit(message: expectedMessage));
      expect(() => file.writeAsStringSync(''), throwsToolExit(message: expectedMessage));
    });

    testWithoutContext('when the file is being used by another program', () async {
      final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
        delegate: MemoryFileSystem.test(opHandle: exceptionHandler.opHandle),
        platform: windowsPlatform,
      );
      final File file = fileSystem.file('file');

      exceptionHandler.addError(
        file,
        FileSystemOp.write,
        FileSystemException('', file.path, const OSError('', kUserMappedSectionOpened)),
      );

      const String expectedMessage = 'The file is being used by another program';
      expect(() async => file.writeAsBytes(<int>[0]), throwsToolExit(message: expectedMessage));
      expect(() async => file.writeAsString(''), throwsToolExit(message: expectedMessage));
      expect(() => file.writeAsBytesSync(<int>[0]), throwsToolExit(message: expectedMessage));
      expect(() => file.writeAsStringSync(''), throwsToolExit(message: expectedMessage));
    });

    testWithoutContext('when the device driver has a fatal error', () async {
      final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
        delegate: MemoryFileSystem.test(opHandle: exceptionHandler.opHandle),
        platform: windowsPlatform,
      );
      final File file = fileSystem.file('file');

      exceptionHandler.addError(
        file,
        FileSystemOp.write,
        FileSystemException('', file.path, const OSError('', kFatalDeviceHardwareError)),
      );
      exceptionHandler.addError(
        file,
        FileSystemOp.open,
        FileSystemException('', file.path, const OSError('', kFatalDeviceHardwareError)),
      );
      exceptionHandler.addError(
        file,
        FileSystemOp.create,
        FileSystemException('', file.path, const OSError('', kFatalDeviceHardwareError)),
      );

      const String expectedMessage =
          'There is a problem with the device driver '
          'that this file or directory is stored on';
      expect(() async => file.writeAsBytes(<int>[0]), throwsToolExit(message: expectedMessage));
      expect(() async => file.writeAsString(''), throwsToolExit(message: expectedMessage));
      expect(() => file.writeAsBytesSync(<int>[0]), throwsToolExit(message: expectedMessage));
      expect(() => file.writeAsStringSync(''), throwsToolExit(message: expectedMessage));
      expect(() => file.openSync(), throwsToolExit(message: expectedMessage));
      expect(() => file.createSync(), throwsToolExit(message: expectedMessage));
    });

    testWithoutContext('when the device does not exist', () async {
      final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
        delegate: MemoryFileSystem.test(opHandle: exceptionHandler.opHandle),
        platform: windowsPlatform,
      );
      final File file = fileSystem.file('file');

      exceptionHandler.addError(
        file,
        FileSystemOp.write,
        FileSystemException('', file.path, const OSError('', kDeviceDoesNotExist)),
      );
      exceptionHandler.addError(
        file,
        FileSystemOp.open,
        FileSystemException('', file.path, const OSError('', kDeviceDoesNotExist)),
      );
      exceptionHandler.addError(
        file,
        FileSystemOp.create,
        FileSystemException('', file.path, const OSError('', kDeviceDoesNotExist)),
      );

      const String expectedMessage = 'The device was not found.';
      expect(() async => file.writeAsBytes(<int>[0]), throwsToolExit(message: expectedMessage));
      expect(() async => file.writeAsString(''), throwsToolExit(message: expectedMessage));
      expect(() => file.writeAsBytesSync(<int>[0]), throwsToolExit(message: expectedMessage));
      expect(() => file.writeAsStringSync(''), throwsToolExit(message: expectedMessage));
      expect(() => file.openSync(), throwsToolExit(message: expectedMessage));
      expect(() => file.createSync(), throwsToolExit(message: expectedMessage));
    });

    testWithoutContext('when creating a temporary dir on a full device', () async {
      final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
        delegate: MemoryFileSystem.test(opHandle: exceptionHandler.opHandle),
        platform: windowsPlatform,
      );
      final Directory directory = fileSystem.directory('directory')..createSync();

      exceptionHandler.addTempError(
        FileSystemOp.create,
        FileSystemException('', directory.path, const OSError('', kDeviceFull)),
      );

      const String expectedMessage = 'The target device is full';
      expect(() async => directory.createTemp('prefix'), throwsToolExit(message: expectedMessage));
      expect(() => directory.createTempSync('prefix'), throwsToolExit(message: expectedMessage));
    });

    testWithoutContext('when creating a directory with permission issues', () async {
      final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
        delegate: MemoryFileSystem.test(opHandle: exceptionHandler.opHandle),
        platform: windowsPlatform,
      );
      final Directory directory = fileSystem.directory('directory');

      exceptionHandler.addError(
        directory,
        FileSystemOp.create,
        FileSystemException('', directory.path, const OSError('', kUserPermissionDenied)),
      );

      const String expectedMessage = 'Flutter failed to create a directory at';
      expect(() => directory.createSync(recursive: true), throwsToolExit(message: expectedMessage));
    });

    testWithoutContext('when checking for directory existence with permission issues', () async {
      final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
        delegate: MemoryFileSystem.test(opHandle: exceptionHandler.opHandle),
        platform: windowsPlatform,
      );

      final Directory directory = fileSystem.directory('directory')..createSync();

      exceptionHandler.addError(
        directory,
        FileSystemOp.exists,
        FileSystemException('', directory.path, const OSError('', kDeviceFull)),
      );

      const String expectedMessage = 'Flutter failed to check for directory existence at';
      expect(() => directory.existsSync(), throwsToolExit(message: expectedMessage));
    });

    testWithoutContext('When reading from a file without permission', () {
      final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
        delegate: MemoryFileSystem.test(opHandle: exceptionHandler.opHandle),
        platform: windowsPlatform,
      );
      final File file = fileSystem.file('file');

      exceptionHandler.addError(
        file,
        FileSystemOp.read,
        FileSystemException('', file.path, const OSError('', kUserPermissionDenied)),
      );

      const String expectedMessage = 'Flutter failed to read a file at';
      expect(() => file.readAsStringSync(), throwsToolExit(message: expectedMessage));
    });

    testWithoutContext('When reading from a file or directory without permission', () {
      final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
        delegate: ThrowsOnCurrentDirectoryFileSystem(kUserPermissionDenied),
        platform: windowsPlatform,
      );

      expect(
        () => fileSystem.currentDirectory,
        throwsToolExit(message: 'The flutter tool cannot access the file or directory'),
      );
    });
  });

  group('throws ToolExit on Linux', () {
    const int eperm = 1;
    const int enospc = 28;
    const int eacces = 13;

    late FileExceptionHandler exceptionHandler;

    setUp(() {
      exceptionHandler = FileExceptionHandler();
    });

    testWithoutContext('when access is denied', () async {
      final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
        delegate: MemoryFileSystem.test(opHandle: exceptionHandler.opHandle),
        platform: linuxPlatform,
      );
      final Directory directory = fileSystem.directory('dir')..createSync();
      final File file = directory.childFile('file');

      exceptionHandler.addError(
        file,
        FileSystemOp.create,
        FileSystemException('', file.path, const OSError('', eacces)),
      );
      exceptionHandler.addError(
        file,
        FileSystemOp.write,
        FileSystemException('', file.path, const OSError('', eacces)),
      );
      exceptionHandler.addError(
        file,
        FileSystemOp.read,
        FileSystemException('', file.path, const OSError('', eacces)),
      );
      exceptionHandler.addError(
        file,
        FileSystemOp.delete,
        FileSystemException('', file.path, const OSError('', eacces)),
      );
      const String writeMessage =
          'Flutter failed to write to a file at "dir/file".\n'
          'Please ensure that the SDK and/or project is installed in a location that has read/write permissions for the current user.\n'
          'Try running:\n'
          r'  sudo chown -R $(whoami) /dir/file';
      expect(() async => file.writeAsBytes(<int>[0]), throwsToolExit(message: writeMessage));
      expect(() async => file.writeAsString(''), throwsToolExit(message: writeMessage));
      expect(() => file.writeAsBytesSync(<int>[0]), throwsToolExit(message: writeMessage));
      expect(() => file.writeAsStringSync(''), throwsToolExit(message: writeMessage));

      const String createMessage =
          'Flutter failed to create file at "dir/file".\n'
          'Please ensure that the SDK and/or project is installed in a location that has read/write permissions for the current user.\n'
          'Try running:\n'
          r'  sudo chown -R $(whoami) /dir';
      expect(() => file.createSync(), throwsToolExit(message: createMessage));
      // Recursive does not contain the "sudo chown" suggestion.
      expect(
        () async => file.createSync(recursive: true),
        throwsA(
          isA<ToolExit>().having(
            (ToolExit e) => e.message,
            'message',
            isNot(contains('sudo chown')),
          ),
        ),
      );

      const String readMessage =
          'Flutter failed to read a file at "dir/file".\n'
          'Please ensure that the SDK and/or project is installed in a location that has read/write permissions for the current user.\n'
          'Try running:\n'
          r'  sudo chown -R $(whoami) /dir/file';
      expect(() => file.readAsStringSync(), throwsToolExit(message: readMessage));
    });

    testWithoutContext('when access is denied for directories', () async {
      final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
        delegate: MemoryFileSystem.test(opHandle: exceptionHandler.opHandle),
        platform: linuxPlatform,
      );
      final Directory parent = fileSystem.directory('parent')..createSync();
      final Directory directory = parent.childDirectory('childDir');

      exceptionHandler.addError(
        directory,
        FileSystemOp.create,
        FileSystemException('', directory.path, const OSError('', eperm)),
      );
      exceptionHandler.addError(
        directory,
        FileSystemOp.delete,
        FileSystemException('', directory.path, const OSError('', eperm)),
      );

      const String createMessage =
          'Flutter failed to create a directory at "parent/childDir".\n'
          'Please ensure that the SDK and/or project is installed in a location that has read/write permissions for the current user.\n'
          'Try running:\n'
          r'  sudo chown -R $(whoami) /parent';
      expect(() async => directory.create(), throwsToolExit(message: createMessage));
      expect(() => directory.createSync(), throwsToolExit(message: createMessage));

      // Recursive does not contain the "sudo chown" suggestion.
      expect(
        () async => directory.createSync(recursive: true),
        throwsA(
          isA<ToolExit>().having(
            (ToolExit e) => e.message,
            'message',
            isNot(contains('sudo chown')),
          ),
        ),
      );

      const String deleteMessage =
          'Flutter failed to delete a directory at "parent/childDir".\n'
          'Please ensure that the SDK and/or project is installed in a location that has read/write permissions for the current user.\n'
          'Try running:\n'
          r'  sudo chown -R $(whoami) /parent';
      expect(() => directory.deleteSync(), throwsToolExit(message: deleteMessage));
      expect(() async => directory.delete(), throwsToolExit(message: deleteMessage));

      // Recursive does not contain the "sudo chown" suggestion.
      expect(
        () async => directory.deleteSync(recursive: true),
        throwsA(
          isA<ToolExit>().having(
            (ToolExit e) => e.message,
            'message',
            isNot(contains('sudo chown')),
          ),
        ),
      );
    });

    testWithoutContext('when writing to a full device', () async {
      final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
        delegate: MemoryFileSystem.test(opHandle: exceptionHandler.opHandle),
        platform: linuxPlatform,
      );
      final File file = fileSystem.file('file');

      exceptionHandler.addError(
        file,
        FileSystemOp.write,
        FileSystemException('', file.path, const OSError('', enospc)),
      );

      const String expectedMessage = 'The target device is full';
      expect(() async => file.writeAsBytes(<int>[0]), throwsToolExit(message: expectedMessage));
      expect(() async => file.writeAsString(''), throwsToolExit(message: expectedMessage));
      expect(() => file.writeAsBytesSync(<int>[0]), throwsToolExit(message: expectedMessage));
      expect(() => file.writeAsStringSync(''), throwsToolExit(message: expectedMessage));
    });

    testWithoutContext('when creating a temporary dir on a full device', () async {
      final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
        delegate: MemoryFileSystem.test(opHandle: exceptionHandler.opHandle),
        platform: linuxPlatform,
      );

      final Directory directory = fileSystem.directory('directory')..createSync();

      exceptionHandler.addTempError(
        FileSystemOp.create,
        FileSystemException('', directory.path, const OSError('', enospc)),
      );

      const String expectedMessage = 'The target device is full';
      expect(() async => directory.createTemp('prefix'), throwsToolExit(message: expectedMessage));
      expect(() => directory.createTempSync('prefix'), throwsToolExit(message: expectedMessage));
    });

    testWithoutContext('when checking for directory existence with permission issues', () async {
      final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
        delegate: MemoryFileSystem.test(opHandle: exceptionHandler.opHandle),
        platform: linuxPlatform,
      );

      final Directory directory = fileSystem.directory('directory')..createSync();

      exceptionHandler.addError(
        directory,
        FileSystemOp.exists,
        FileSystemException('', directory.path, const OSError('', eacces)),
      );

      const String expectedMessage = 'Flutter failed to check for directory existence at';
      expect(() => directory.existsSync(), throwsToolExit(message: expectedMessage));
    });

    testWithoutContext('When the current working directory disappears', () async {
      final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
        delegate: ThrowsOnCurrentDirectoryFileSystem(kSystemCodeCannotFindFile),
        platform: linuxPlatform,
      );

      expect(
        () => fileSystem.currentDirectory,
        throwsToolExit(message: 'Unable to read current working directory'),
      );
    });

    testWithoutContext('Rethrows os error $kSystemCodeCannotFindFile', () {
      final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
        delegate: MemoryFileSystem.test(opHandle: exceptionHandler.opHandle),
        platform: linuxPlatform,
      );
      final File file = fileSystem.file('file');

      exceptionHandler.addError(
        file,
        FileSystemOp.read,
        FileSystemException('', file.path, const OSError('', kSystemCodeCannotFindFile)),
      );

      // Error is not caught by other operations.
      expect(
        () => fileSystem.file('foo').readAsStringSync(),
        throwsFileSystemException(kSystemCodeCannotFindFile),
      );
    });
  });

  group('throws ToolExit on macOS', () {
    const int eperm = 1;
    const int enospc = 28;
    const int eacces = 13;
    late FileExceptionHandler exceptionHandler;

    setUp(() {
      exceptionHandler = FileExceptionHandler();
    });

    testWithoutContext('when access is denied', () async {
      final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
        delegate: MemoryFileSystem.test(opHandle: exceptionHandler.opHandle),
        platform: macOSPlatform,
      );
      final Directory directory = fileSystem.directory('dir')..createSync();
      final File file = directory.childFile('file');

      exceptionHandler.addError(
        file,
        FileSystemOp.create,
        FileSystemException('', file.path, const OSError('', eacces)),
      );
      exceptionHandler.addError(
        file,
        FileSystemOp.write,
        FileSystemException('', file.path, const OSError('', eacces)),
      );
      exceptionHandler.addError(
        file,
        FileSystemOp.read,
        FileSystemException('', file.path, const OSError('', eacces)),
      );
      exceptionHandler.addError(
        file,
        FileSystemOp.delete,
        FileSystemException('', file.path, const OSError('', eacces)),
      );
      const String writeMessage =
          'Flutter failed to write to a file at "dir/file".\n'
          'Please ensure that the SDK and/or project is installed in a location that has read/write permissions for the current user.\n'
          'Try running:\n'
          r'  sudo chown -R $(whoami) /dir/file';
      expect(() async => file.writeAsBytes(<int>[0]), throwsToolExit(message: writeMessage));
      expect(() async => file.writeAsString(''), throwsToolExit(message: writeMessage));
      expect(() => file.writeAsBytesSync(<int>[0]), throwsToolExit(message: writeMessage));
      expect(() => file.writeAsStringSync(''), throwsToolExit(message: writeMessage));

      const String createMessage =
          'Flutter failed to create file at "dir/file".\n'
          'Please ensure that the SDK and/or project is installed in a location that has read/write permissions for the current user.\n'
          'Try running:\n'
          r'  sudo chown -R $(whoami) /dir';
      expect(() => file.createSync(), throwsToolExit(message: createMessage));

      // Recursive does not contain the "sudo chown" suggestion.
      expect(
        () async => file.createSync(recursive: true),
        throwsA(
          isA<ToolExit>().having(
            (ToolExit e) => e.message,
            'message',
            isNot(contains('sudo chown')),
          ),
        ),
      );

      const String readMessage =
          'Flutter failed to read a file at "dir/file".\n'
          'Please ensure that the SDK and/or project is installed in a location that has read/write permissions for the current user.\n'
          'Try running:\n'
          r'  sudo chown -R $(whoami) /dir/file';
      expect(() => file.readAsStringSync(), throwsToolExit(message: readMessage));
    });

    testWithoutContext('when access is denied for directories', () async {
      final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
        delegate: MemoryFileSystem.test(opHandle: exceptionHandler.opHandle),
        platform: macOSPlatform,
      );
      final Directory parent = fileSystem.directory('parent')..createSync();
      final Directory directory = parent.childDirectory('childDir');

      exceptionHandler.addError(
        directory,
        FileSystemOp.create,
        FileSystemException('', directory.path, const OSError('', eperm)),
      );
      exceptionHandler.addError(
        directory,
        FileSystemOp.delete,
        FileSystemException('', directory.path, const OSError('', eperm)),
      );

      const String createMessage =
          'Flutter failed to create a directory at "parent/childDir".\n'
          'Please ensure that the SDK and/or project is installed in a location that has read/write permissions for the current user.\n'
          'Try running:\n'
          r'  sudo chown -R $(whoami) /parent';
      expect(() async => directory.create(), throwsToolExit(message: createMessage));
      expect(() => directory.createSync(), throwsToolExit(message: createMessage));

      // Recursive does not contain the "sudo chown" suggestion.
      expect(
        () async => directory.createSync(recursive: true),
        throwsA(
          isA<ToolExit>().having(
            (ToolExit e) => e.message,
            'message',
            isNot(contains('sudo chown')),
          ),
        ),
      );

      const String deleteMessage =
          'Flutter failed to delete a directory at "parent/childDir".\n'
          'Please ensure that the SDK and/or project is installed in a location that has read/write permissions for the current user.\n'
          'Try running:\n'
          r'  sudo chown -R $(whoami) /parent';
      expect(() => directory.deleteSync(), throwsToolExit(message: deleteMessage));
      expect(() async => directory.delete(), throwsToolExit(message: deleteMessage));

      // Recursive does not contain the "sudo chown" suggestion.
      expect(
        () async => directory.deleteSync(recursive: true),
        throwsA(
          isA<ToolExit>().having(
            (ToolExit e) => e.message,
            'message',
            isNot(contains('sudo chown')),
          ),
        ),
      );
    });

    testWithoutContext('when writing to a full device', () async {
      final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
        delegate: MemoryFileSystem.test(opHandle: exceptionHandler.opHandle),
        platform: macOSPlatform,
      );
      final File file = fileSystem.file('file');

      exceptionHandler.addError(
        file,
        FileSystemOp.write,
        FileSystemException('', file.path, const OSError('', enospc)),
      );

      const String expectedMessage = 'The target device is full';
      expect(() async => file.writeAsBytes(<int>[0]), throwsToolExit(message: expectedMessage));
      expect(() async => file.writeAsString(''), throwsToolExit(message: expectedMessage));
      expect(() => file.writeAsBytesSync(<int>[0]), throwsToolExit(message: expectedMessage));
      expect(() => file.writeAsStringSync(''), throwsToolExit(message: expectedMessage));
    });

    testWithoutContext('when creating a temporary dir on a full device', () async {
      final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
        delegate: MemoryFileSystem.test(opHandle: exceptionHandler.opHandle),
        platform: macOSPlatform,
      );

      final Directory directory = fileSystem.directory('directory')..createSync();

      exceptionHandler.addTempError(
        FileSystemOp.create,
        FileSystemException('', directory.path, const OSError('', enospc)),
      );

      const String expectedMessage = 'The target device is full';
      expect(() async => directory.createTemp('prefix'), throwsToolExit(message: expectedMessage));
      expect(() => directory.createTempSync('prefix'), throwsToolExit(message: expectedMessage));
    });

    testWithoutContext('when checking for directory existence with permission issues', () async {
      final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
        delegate: MemoryFileSystem.test(opHandle: exceptionHandler.opHandle),
        platform: macOSPlatform,
      );

      final Directory directory = fileSystem.directory('directory');

      exceptionHandler.addError(
        directory,
        FileSystemOp.exists,
        FileSystemException('', directory.path, const OSError('', eacces)),
      );

      const String expectedMessage = 'Flutter failed to check for directory existence at';
      expect(() => directory.existsSync(), throwsToolExit(message: expectedMessage));
    });

    testWithoutContext('When reading from a file without permission', () {
      final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
        delegate: MemoryFileSystem.test(opHandle: exceptionHandler.opHandle),
        platform: macOSPlatform,
      );
      final File file = fileSystem.file('file');

      exceptionHandler.addError(
        file,
        FileSystemOp.read,
        FileSystemException('', file.path, const OSError('', eacces)),
      );

      const String expectedMessage = 'Flutter failed to read a file at';
      expect(() => file.readAsStringSync(), throwsToolExit(message: expectedMessage));
    });

    testWithoutContext('When reading from current directory without permission', () {
      final ErrorHandlingFileSystem fileSystem = ErrorHandlingFileSystem(
        delegate: ThrowsOnCurrentDirectoryFileSystem(eacces),
        platform: linuxPlatform,
      );

      expect(
        () => fileSystem.currentDirectory,
        throwsToolExit(message: 'The flutter tool cannot access the file or directory'),
      );
    });
  });

  testWithoutContext('Caches path context correctly', () {
    final FakeFileSystem fileSystem = FakeFileSystem();
    final FileSystem fs = ErrorHandlingFileSystem(
      delegate: fileSystem,
      platform: const LocalPlatform(),
    );

    expect(identical(fs.path, fs.path), true);
  });

  testWithoutContext('Clears cache when CWD changes', () {
    final FakeFileSystem fileSystem = FakeFileSystem();
    final FileSystem fs = ErrorHandlingFileSystem(
      delegate: fileSystem,
      platform: const LocalPlatform(),
    );

    final Object firstPath = fs.path;
    expect(firstPath, isNotNull);

    fs.currentDirectory = null;
    expect(identical(firstPath, fs.path), false);
  });

  group('toString() gives toString() of delegate', () {
    testWithoutContext('ErrorHandlingFileSystem', () {
      final MemoryFileSystem delegate = MemoryFileSystem.test();
      final FileSystem fs = ErrorHandlingFileSystem(
        delegate: delegate,
        platform: const LocalPlatform(),
      );

      expect(delegate.toString(), isNotNull);
      expect(fs.toString(), delegate.toString());
    });

    testWithoutContext('ErrorHandlingFile', () {
      final MemoryFileSystem delegate = MemoryFileSystem.test();
      final FileSystem fs = ErrorHandlingFileSystem(
        delegate: delegate,
        platform: const LocalPlatform(),
      );
      final File file = delegate.file('file');

      expect(file.toString(), isNotNull);
      expect(fs.file('file').toString(), file.toString());
    });

    testWithoutContext('ErrorHandlingDirectory', () {
      final MemoryFileSystem delegate = MemoryFileSystem.test();
      final FileSystem fs = ErrorHandlingFileSystem(
        delegate: delegate,
        platform: const LocalPlatform(),
      );
      final Directory directory = delegate.directory('directory')..createSync();
      expect(fs.directory('directory').toString(), directory.toString());
      delegate.currentDirectory = directory;

      expect(fs.currentDirectory.toString(), delegate.currentDirectory.toString());
      expect(fs.currentDirectory, isA<ErrorHandlingDirectory>());
    });
  });

  testWithoutContext(
    "ErrorHandlingFileSystem.systemTempDirectory wraps delegate filesystem's systemTempDirectory",
    () {
      final FileExceptionHandler exceptionHandler = FileExceptionHandler();

      final MemoryFileSystem delegate = MemoryFileSystem.test(
        style: FileSystemStyle.windows,
        opHandle: exceptionHandler.opHandle,
      );

      final FileSystem fs = ErrorHandlingFileSystem(
        delegate: delegate,
        platform: FakePlatform(operatingSystem: 'windows'),
      );

      expect(fs.systemTempDirectory, isA<ErrorHandlingDirectory>());
      expect(fs.systemTempDirectory.path, delegate.systemTempDirectory.path);

      final File tempFile = delegate.systemTempDirectory.childFile('hello')
        ..createSync(recursive: true);

      exceptionHandler.addError(
        tempFile,
        FileSystemOp.write,
        FileSystemException('Oh no!', tempFile.path, const OSError('Access denied ):', 5)),
      );

      expect(
        () => fs.file(tempFile.path).writeAsStringSync('world'),
        throwsToolExit(
          message: r'''
Flutter failed to write to a file at "C:\.tmp_rand0\hello". The flutter tool cannot access the file or directory.
Please ensure that the SDK and/or project is installed in a location that has read/write permissions for the current user.''',
        ),
      );
    },
  );

  testWithoutContext(
    "ErrorHandlingFileSystem.systemTempDirectory handles any exception thrown by the delegate's systemTempDirectory implementation",
    () {
      final FileExceptionHandler exceptionHandler = FileExceptionHandler();
      exceptionHandler.addTempError(
        FileSystemOp.create,
        const FileSystemException(
          'Creation of temporary directory failed',
          'some/temp/path',
          OSError('No space left on device', 28),
        ),
      );

      final MemoryFileSystem delegate = MemoryFileSystem.test(opHandle: exceptionHandler.opHandle);

      final FileSystem fs = ErrorHandlingFileSystem(delegate: delegate, platform: FakePlatform());

      expect(() => fs.systemTempDirectory, throwsToolExit(message: 'Free up space and try again.'));
    },
  );

  group('ProcessManager on windows throws tool exit', () {
    const int kDeviceFull = 112;
    const int kUserMappedSectionOpened = 1224;
    const int kUserPermissionDenied = 5;

    testWithoutContext(
      'when PackageProcess throws an exception containing non-executable bits',
      () {
        final FakeProcessManager fakeProcessManager = FakeProcessManager.list(<FakeCommand>[
          const FakeCommand(
            command: <String>['foo'],
            exception: ProcessPackageExecutableNotFoundException(
              '',
              candidates: <String>['not-empty'],
            ),
          ),
          const FakeCommand(
            command: <String>['foo'],
            exception: ProcessPackageExecutableNotFoundException(
              '',
              candidates: <String>['not-empty'],
            ),
          ),
        ]);

        final ProcessManager processManager = ErrorHandlingProcessManager(
          delegate: fakeProcessManager,
          platform: windowsPlatform,
        );

        const String expectedMessage =
            'The Flutter tool could not locate an executable with suitable permissions';

        expect(
          () async => processManager.start(<String>['foo']),
          throwsToolExit(message: expectedMessage),
        );
        expect(
          () async => processManager.runSync(<String>['foo']),
          throwsToolExit(message: expectedMessage),
        );
      },
    );

    testWithoutContext(
      'when PackageProcess throws an exception without containing non-executable bits',
      () {
        final FakeProcessManager fakeProcessManager = FakeProcessManager.list(<FakeCommand>[
          const FakeCommand(
            command: <String>['foo'],
            exception: ProcessPackageExecutableNotFoundException(''),
          ),
          const FakeCommand(
            command: <String>['foo'],
            exception: ProcessPackageExecutableNotFoundException(''),
          ),
        ]);

        final ProcessManager processManager = ErrorHandlingProcessManager(
          delegate: fakeProcessManager,
          platform: windowsPlatform,
        );

        // If there were no located executables treat this as a programming error and rethrow the original
        // exception.
        expect(() async => processManager.start(<String>['foo']), throwsProcessException());
        expect(() async => processManager.runSync(<String>['foo']), throwsProcessException());
      },
    );

    testWithoutContext('when the device is full', () {
      final FakeProcessManager fakeProcessManager = FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['foo'],
          exception: ProcessException('', <String>[], '', kDeviceFull),
        ),
        const FakeCommand(
          command: <String>['foo'],
          exception: ProcessException('', <String>[], '', kDeviceFull),
        ),
        const FakeCommand(
          command: <String>['foo'],
          exception: ProcessException('', <String>[], '', kDeviceFull),
        ),
      ]);

      final ProcessManager processManager = ErrorHandlingProcessManager(
        delegate: fakeProcessManager,
        platform: windowsPlatform,
      );

      const String expectedMessage = 'The target device is full';

      expect(
        () async => processManager.start(<String>['foo']),
        throwsToolExit(message: expectedMessage),
      );
      expect(
        () async => processManager.run(<String>['foo']),
        throwsToolExit(message: expectedMessage),
      );
      expect(
        () => processManager.runSync(<String>['foo']),
        throwsToolExit(message: expectedMessage),
      );
    });

    testWithoutContext('when the file is being used by another program', () {
      final FakeProcessManager fakeProcessManager = FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['foo'],
          exception: ProcessException('', <String>[], '', kUserMappedSectionOpened),
        ),
        const FakeCommand(
          command: <String>['foo'],
          exception: ProcessException('', <String>[], '', kUserMappedSectionOpened),
        ),
        const FakeCommand(
          command: <String>['foo'],
          exception: ProcessException('', <String>[], '', kUserMappedSectionOpened),
        ),
      ]);

      final ProcessManager processManager = ErrorHandlingProcessManager(
        delegate: fakeProcessManager,
        platform: windowsPlatform,
      );

      const String expectedMessage = 'The file is being used by another program';
      expect(
        () async => processManager.start(<String>['foo']),
        throwsToolExit(message: expectedMessage),
      );
      expect(
        () async => processManager.run(<String>['foo']),
        throwsToolExit(message: expectedMessage),
      );
      expect(
        () => processManager.runSync(<String>['foo']),
        throwsToolExit(message: expectedMessage),
      );
    });

    testWithoutContext('when permissions are denied', () {
      final FakeProcessManager fakeProcessManager = FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['foo'],
          exception: ProcessException('', <String>[], '', kUserPermissionDenied),
        ),
        const FakeCommand(
          command: <String>['foo'],
          exception: ProcessException('', <String>[], '', kUserPermissionDenied),
        ),
        const FakeCommand(
          command: <String>['foo'],
          exception: ProcessException('', <String>[], '', kUserPermissionDenied),
        ),
      ]);

      final ProcessManager processManager = ErrorHandlingProcessManager(
        delegate: fakeProcessManager,
        platform: windowsPlatform,
      );

      const String expectedMessage =
          'Flutter failed to run "foo". The flutter tool cannot access the file or directory.\n'
          'Please ensure that the SDK and/or project is installed in a location that has read/write permissions for the current user.';
      expect(
        () async => processManager.start(<String>['foo']),
        throwsToolExit(message: expectedMessage),
      );
      expect(
        () async => processManager.run(<String>['foo']),
        throwsToolExit(message: expectedMessage),
      );
      expect(
        () => processManager.runSync(<String>['foo']),
        throwsToolExit(message: expectedMessage),
      );
    });

    testWithoutContext('when cannot run executable', () {
      final ThrowingFakeProcessManager throwingFakeProcessManager = ThrowingFakeProcessManager(
        const ProcessException('', <String>[], '', kUserPermissionDenied),
      );

      final ProcessManager processManager = ErrorHandlingProcessManager(
        delegate: throwingFakeProcessManager,
        platform: windowsPlatform,
      );

      const String expectedMessage =
          r'Flutter failed to run "C:\path\to\dart". The flutter tool cannot access the file or directory.';
      expect(
        () async => processManager.canRun(r'C:\path\to\dart'),
        throwsToolExit(message: expectedMessage),
      );
    });
  });

  group('ProcessManager on linux throws tool exit', () {
    const int enospc = 28;
    const int eacces = 13;

    testWithoutContext('when writing to a full device', () {
      final FakeProcessManager fakeProcessManager = FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['foo'],
          exception: ProcessException('', <String>[], '', enospc),
        ),
        const FakeCommand(
          command: <String>['foo'],
          exception: ProcessException('', <String>[], '', enospc),
        ),
        const FakeCommand(
          command: <String>['foo'],
          exception: ProcessException('', <String>[], '', enospc),
        ),
      ]);

      final ProcessManager processManager = ErrorHandlingProcessManager(
        delegate: fakeProcessManager,
        platform: linuxPlatform,
      );

      const String expectedMessage = 'The target device is full';
      expect(
        () async => processManager.start(<String>['foo']),
        throwsToolExit(message: expectedMessage),
      );
      expect(
        () async => processManager.run(<String>['foo']),
        throwsToolExit(message: expectedMessage),
      );
      expect(
        () => processManager.runSync(<String>['foo']),
        throwsToolExit(message: expectedMessage),
      );
    });

    testWithoutContext('when permissions are denied', () {
      final FakeProcessManager fakeProcessManager = FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['foo'],
          exception: ProcessException('', <String>[], '', eacces),
        ),
        const FakeCommand(
          command: <String>['foo'],
          exception: ProcessException('', <String>[], '', eacces),
        ),
        const FakeCommand(
          command: <String>['foo'],
          exception: ProcessException('', <String>[], '', eacces),
        ),
      ]);
      final ProcessManager processManager = ErrorHandlingProcessManager(
        delegate: fakeProcessManager,
        platform: linuxPlatform,
      );

      const String expectedMessage =
          'Flutter failed to run "foo".\n'
          'Please ensure that the SDK and/or project is installed in a location that has read/write permissions for the current user.';

      expect(
        () async => processManager.start(<String>['foo']),
        throwsToolExit(message: expectedMessage),
      );
      expect(
        () async => processManager.run(<String>['foo']),
        throwsToolExit(message: expectedMessage),
      );
      expect(
        () => processManager.runSync(<String>['foo']),
        throwsToolExit(message: expectedMessage),
      );
    });

    testWithoutContext('when cannot run executable', () {
      final ThrowingFakeProcessManager throwingFakeProcessManager = ThrowingFakeProcessManager(
        const ProcessException('', <String>[], '', eacces),
      );

      final ProcessManager processManager = ErrorHandlingProcessManager(
        delegate: throwingFakeProcessManager,
        platform: linuxPlatform,
      );

      const String expectedMessage =
          'Flutter failed to run "/path/to/dart".\n'
          'Please ensure that the SDK and/or project is installed in a location that has read/write permissions for the current user.\n'
          'Try running:\n'
          r'  sudo chown -R $(whoami) /path/to/dart && chmod u+rx /path/to/dart';

      expect(
        () async => processManager.canRun('/path/to/dart'),
        throwsToolExit(message: expectedMessage),
      );
    });
  });

  group('ProcessManager on macOS throws tool exit', () {
    const int enospc = 28;
    const int eacces = 13;
    const int ebadarch = 86;
    const int eagain = 35;

    testWithoutContext('when writing to a full device', () {
      final FakeProcessManager fakeProcessManager = FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['foo'],
          exception: ProcessException('', <String>[], '', enospc),
        ),
        const FakeCommand(
          command: <String>['foo'],
          exception: ProcessException('', <String>[], '', enospc),
        ),
        const FakeCommand(
          command: <String>['foo'],
          exception: ProcessException('', <String>[], '', enospc),
        ),
      ]);
      final ProcessManager processManager = ErrorHandlingProcessManager(
        delegate: fakeProcessManager,
        platform: macOSPlatform,
      );

      const String expectedMessage = 'The target device is full';

      expect(
        () async => processManager.start(<String>['foo']),
        throwsToolExit(message: expectedMessage),
      );
      expect(
        () async => processManager.run(<String>['foo']),
        throwsToolExit(message: expectedMessage),
      );
      expect(
        () => processManager.runSync(<String>['foo']),
        throwsToolExit(message: expectedMessage),
      );
    });

    testWithoutContext('when permissions are denied', () {
      final FakeProcessManager fakeProcessManager = FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['foo'],
          exception: ProcessException('', <String>[], '', eacces),
        ),
        const FakeCommand(
          command: <String>['foo'],
          exception: ProcessException('', <String>[], '', eacces),
        ),
        const FakeCommand(
          command: <String>['foo'],
          exception: ProcessException('', <String>[], '', eacces),
        ),
      ]);
      final ProcessManager processManager = ErrorHandlingProcessManager(
        delegate: fakeProcessManager,
        platform: macOSPlatform,
      );

      const String expectedMessage =
          'Flutter failed to run "foo".\n'
          'Please ensure that the SDK and/or project is installed in a location that has read/write permissions for the current user.';

      expect(
        () async => processManager.start(<String>['foo']),
        throwsToolExit(message: expectedMessage),
      );
      expect(
        () async => processManager.run(<String>['foo']),
        throwsToolExit(message: expectedMessage),
      );
      expect(
        () => processManager.runSync(<String>['foo']),
        throwsToolExit(message: expectedMessage),
      );
    });

    testWithoutContext('when cannot run executable', () {
      final ThrowingFakeProcessManager throwingFakeProcessManager = ThrowingFakeProcessManager(
        const ProcessException('', <String>[], '', eacces),
      );

      final ProcessManager processManager = ErrorHandlingProcessManager(
        delegate: throwingFakeProcessManager,
        platform: macOSPlatform,
      );

      const String expectedMessage =
          'Flutter failed to run "/path/to/dart".\n'
          'Please ensure that the SDK and/or project is installed in a location that has read/write permissions for the current user.\n'
          'Try running:\n'
          r'  sudo chown -R $(whoami) /path/to/dart && chmod u+rx /path/to/dart';

      expect(
        () async => processManager.canRun('/path/to/dart'),
        throwsToolExit(message: expectedMessage),
      );
    });

    testWithoutContext('when bad CPU type', () async {
      final FakeProcessManager fakeProcessManager = FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['foo', '--bar'],
          exception: ProcessException('', <String>[], '', ebadarch),
        ),
        const FakeCommand(
          command: <String>['foo', '--bar'],
          exception: ProcessException('', <String>[], '', ebadarch),
        ),
        const FakeCommand(
          command: <String>['foo', '--bar'],
          exception: ProcessException('', <String>[], '', ebadarch),
        ),
      ]);

      final ProcessManager processManager = ErrorHandlingProcessManager(
        delegate: fakeProcessManager,
        platform: macOSPlatform,
      );

      const String expectedMessage =
          'Flutter failed to run "foo --bar".\n'
          'The binary was built with the incorrect architecture to run on this machine.';

      expect(
        () async => processManager.start(<String>['foo', '--bar']),
        throwsToolExit(message: expectedMessage),
      );
      expect(
        () async => processManager.run(<String>['foo', '--bar']),
        throwsToolExit(message: expectedMessage),
      );
      expect(
        () => processManager.runSync(<String>['foo', '--bar']),
        throwsToolExit(message: expectedMessage),
      );
    });

    testWithoutContext('when up against resource limits (EAGAIN)', () async {
      final FakeProcessManager fakeProcessManager = FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['foo', '--bar'],
          exception: ProcessException('', <String>[], '', eagain),
        ),
      ]);

      final ProcessManager processManager = ErrorHandlingProcessManager(
        delegate: fakeProcessManager,
        platform: macOSPlatform,
      );

      const String expectedMessage =
          'Flutter failed to run "foo --bar".\n'
          'Your system may be running into its process limits. '
          'Consider quitting unused apps and trying again.';

      expect(
        () async => processManager.start(<String>['foo', '--bar']),
        throwsToolExit(message: expectedMessage),
      );
    });
  });

  testWithoutContext('ErrorHandlingProcessManager delegates killPid correctly', () async {
    final FakeSignalProcessManager fakeProcessManager = FakeSignalProcessManager();
    final ProcessManager processManager = ErrorHandlingProcessManager(
      delegate: fakeProcessManager,
      platform: linuxPlatform,
    );

    expect(processManager.killPid(1), true);
    expect(processManager.killPid(3, io.ProcessSignal.sigkill), true);
    expect(fakeProcessManager.killedProcesses, <int, io.ProcessSignal>{
      1: io.ProcessSignal.sigterm,
      3: io.ProcessSignal.sigkill,
    });
  });

  group('CopySync', () {
    const int eaccess = 13;
    late FileExceptionHandler exceptionHandler;
    late ErrorHandlingFileSystem fileSystem;

    setUp(() {
      exceptionHandler = FileExceptionHandler();
      fileSystem = ErrorHandlingFileSystem(
        delegate: MemoryFileSystem.test(opHandle: exceptionHandler.opHandle),
        platform: linuxPlatform,
      );
    });

    testWithoutContext('copySync handles error if openSync on source file fails', () {
      final File source = fileSystem.file('source');

      exceptionHandler.addError(
        source,
        FileSystemOp.open,
        FileSystemException('', source.path, const OSError('', eaccess)),
      );

      const String expectedMessage =
          'Flutter failed to copy source to dest due to source location error.\n'
          'Please ensure that the SDK and/or project is installed in a location that has read/write permissions for the current user.\n'
          'Try running:\n'
          r'  sudo chown -R $(whoami) /source';
      expect(
        () => fileSystem.file('source').copySync('dest'),
        throwsToolExit(message: expectedMessage),
      );
    });

    testWithoutContext('copySync handles error if createSync on destination file fails', () {
      fileSystem.file('source').createSync();
      final File dest = fileSystem.file('dest');

      exceptionHandler.addError(
        dest,
        FileSystemOp.create,
        FileSystemException('', dest.path, const OSError('', eaccess)),
      );

      const String expectedMessage =
          'Flutter failed to create file at "dest".\n'
          'Please ensure that the SDK and/or project is installed in a location that has read/write permissions for the current user.';
      expect(
        () => fileSystem.file('source').copySync('dest'),
        throwsToolExit(message: expectedMessage),
      );
    });

    // dart:io is able to clobber read-only files.
    testWithoutContext('copySync will copySync even if the destination is not writable', () {
      fileSystem.file('source').createSync();
      final File dest = fileSystem.file('dest');

      exceptionHandler.addError(
        dest,
        FileSystemOp.open,
        FileSystemException('', dest.path, const OSError('', eaccess)),
      );

      expect(dest, isNot(exists));
      fileSystem.file('source').copySync('dest');
      expect(dest, exists);
    });

    testWithoutContext('copySync will copySync if there are no exceptions', () {
      fileSystem.file('source').createSync();
      final File dest = fileSystem.file('dest');

      expect(dest, isNot(exists));
      fileSystem.file('source').copySync('dest');
      expect(dest, exists);
    });

    testWithoutContext(
      'copySync can directly copy bytes if both files can be opened but copySync fails',
      () {
        final List<int> expectedBytes = List<int>.generate(
          64 * 1024 + 3,
          (int i) => i.isEven ? 0 : 1,
        );
        fileSystem.file('source').writeAsBytesSync(expectedBytes);
        final File dest = fileSystem.file('dest');

        exceptionHandler.addError(
          dest,
          FileSystemOp.copy,
          FileSystemException('', dest.path, const OSError('', eaccess)),
        );

        fileSystem.file('source').copySync('dest');
        expect(dest.readAsBytesSync(), expectedBytes);
      },
    );
  });
}

class FakeSignalProcessManager extends Fake implements ProcessManager {
  final Map<int, io.ProcessSignal> killedProcesses = <int, io.ProcessSignal>{};

  @override
  bool killPid(int pid, [io.ProcessSignal signal = io.ProcessSignal.sigterm]) {
    killedProcesses[pid] = signal;
    return true;
  }
}

class ThrowingFakeProcessManager extends Fake implements ProcessManager {
  ThrowingFakeProcessManager(Exception exception) : _exception = exception;

  final Exception _exception;

  @override
  bool canRun(dynamic executable, {String? workingDirectory}) {
    throw _exception;
  }
}

class ThrowsOnCurrentDirectoryFileSystem extends Fake implements FileSystem {
  ThrowsOnCurrentDirectoryFileSystem(this.errorCode);

  final int errorCode;

  @override
  Directory get currentDirectory => throw FileSystemException('', '', OSError('', errorCode));
}

class FakeExistsFile extends Fake implements File {
  late Exception error;
  int existsCount = 0;

  @override
  bool existsSync() {
    if (existsCount == 0) {
      existsCount += 1;
      return true;
    }
    return false;
  }

  @override
  void deleteSync({bool recursive = false}) {
    throw error;
  }
}

class FakeFileSystem extends Fake implements FileSystem {
  @override
  Context get path => Context();

  @override
  Directory get currentDirectory {
    throw UnimplementedError();
  }

  @override
  set currentDirectory(dynamic path) {}
}
