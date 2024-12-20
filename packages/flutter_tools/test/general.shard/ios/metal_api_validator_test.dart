// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/ios/migrations/metal_api_validation_migration.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';

void main() {
  testWithoutContext('Adds Metal API setting to matching file', () {
    final FileSystem fs = MemoryFileSystem.test();

    final File file =
        fs.file('test_file')
          ..createSync()
          ..writeAsStringSync('''
<?xml version="1.0" encoding="UTF-8"?>
  <LaunchAction
    buildConfiguration = "Debug"
    selectedDebuggerIdentifier = "Xcode.DebuggerFoundation.Debugger.LLDB"
    selectedLauncherIdentifier = "Xcode.DebuggerFoundation.Launcher.LLDB"
    launchStyle = "0"
    useCustomWorkingDirectory = "NO"
    ignoresPersistentStateOnLaunch = "NO"
    debugDocumentVersioning = "YES"
    debugServiceExtension = "internal"
    allowLocationSimulation = "YES">
''');
    final FakeIosProject project = FakeIosProject(file);
    final MetalAPIValidationMigrator validator = MetalAPIValidationMigrator.ios(
      project,
      BufferLogger.test(),
    );

    expect(() async => validator.migrate(), returnsNormally);

    expect(
      file.readAsStringSync(),
      contains(
        'debugServiceExtension = "internal"'
        '\n    enableGPUValidationMode = "1"',
      ),
    );
  });

  testWithoutContext('Adds Metal API setting to matching file and crazy indentation', () {
    final FileSystem fs = MemoryFileSystem.test();

    final File file =
        fs.file('test_file')
          ..createSync()
          ..writeAsStringSync('''
<?xml version="1.0" encoding="UTF-8"?>
      <LaunchAction
        buildConfiguration = "Debug"
        selectedDebuggerIdentifier = "Xcode.DebuggerFoundation.Debugger.LLDB"
        selectedLauncherIdentifier = "Xcode.DebuggerFoundation.Launcher.LLDB"
        launchStyle = "0"
        useCustomWorkingDirectory = "NO"
        ignoresPersistentStateOnLaunch = "NO"
        debugDocumentVersioning = "YES"
        debugServiceExtension = "internal"
        allowLocationSimulation = "YES">
''');
    final FakeIosProject project = FakeIosProject(file);
    final MetalAPIValidationMigrator validator = MetalAPIValidationMigrator.ios(
      project,
      BufferLogger.test(),
    );

    expect(() async => validator.migrate(), returnsNormally);

    expect(
      file.readAsStringSync(),
      contains(
        'debugServiceExtension = "internal"'
        '\n        enableGPUValidationMode = "1"',
      ),
    );
  });

  testWithoutContext('Skips modifying file that already references Metal API setting', () {
    final FileSystem fs = MemoryFileSystem.test();

    final File file =
        fs.file('test_file')
          ..createSync()
          ..writeAsStringSync('''
<?xml version="1.0" encoding="UTF-8"?>
  <LaunchAction
    buildConfiguration = "Debug"
    selectedDebuggerIdentifier = "Xcode.DebuggerFoundation.Debugger.LLDB"
    selectedLauncherIdentifier = "Xcode.DebuggerFoundation.Launcher.LLDB"
    launchStyle = "0"
    useCustomWorkingDirectory = "NO"
    ignoresPersistentStateOnLaunch = "NO"
    debugDocumentVersioning = "YES"
    debugServiceExtension = "internal"
    enableGPUValidationMode = "1"
    allowLocationSimulation = "YES">
''');
    final FakeIosProject project = FakeIosProject(file);
    final MetalAPIValidationMigrator validator = MetalAPIValidationMigrator.ios(
      project,
      BufferLogger.test(),
    );

    final String initialContents = file.readAsStringSync();

    expect(() async => validator.migrate(), returnsNormally);
    expect(file.readAsStringSync(), initialContents);
  });

  testWithoutContext('No-op on file with no match', () {
    final FileSystem fs = MemoryFileSystem.test();

    final File file =
        fs.file('does_not_exist')
          ..createSync()
          ..writeAsStringSync('NO_OP');
    final FakeIosProject project = FakeIosProject(file);
    final MetalAPIValidationMigrator validator = MetalAPIValidationMigrator.ios(
      project,
      BufferLogger.test(),
    );

    expect(() async => validator.migrate(), returnsNormally);

    expect(file.readAsStringSync(), 'NO_OP');
  });

  testWithoutContext('No-op on missing file', () async {
    final FileSystem fs = MemoryFileSystem.test();
    final FakeIosProject project = FakeIosProject(fs.file('does_not_exist'));
    final MetalAPIValidationMigrator validator = MetalAPIValidationMigrator.ios(
      project,
      BufferLogger.test(),
    );

    expect(() async => validator.migrate(), returnsNormally);
  });
}

class FakeIosProject extends Fake implements IosProject {
  FakeIosProject(this._file);

  final File _file;

  @override
  File xcodeProjectSchemeFile({String? scheme}) {
    return _file;
  }
}
