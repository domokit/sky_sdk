// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/cmake.dart';
import 'package:flutter_tools/src/project.dart';

import '../src/common.dart';

const String _kTestFlutterRoot = '/flutter';
const String _kTestWindowsFlutterRoot = r'C:\flutter';

void main() {
  late FileSystem fileSystem;
  late BufferLogger logger;

  setUp(() {
    fileSystem = MemoryFileSystem.test();
    logger = BufferLogger.test();
  });

  testWithoutContext('parses executable name from cmake file', () async {
    final FlutterProject project = FlutterProject.fromDirectoryTest(fileSystem.currentDirectory);
    final CmakeBasedProject cmakeProject = _FakeProject.fromFlutter(project);

    cmakeProject.cmakeFile
      ..createSync(recursive: true)
      ..writeAsStringSync('set(BINARY_NAME "hello")');

    final String? name = getCmakeExecutableName(cmakeProject);

    expect(name, 'hello');
  });

  testWithoutContext('defaults executable name to null if cmake config does not exist', () async {
    final FlutterProject project = FlutterProject.fromDirectoryTest(fileSystem.currentDirectory);
    final CmakeBasedProject cmakeProject = _FakeProject.fromFlutter(project);

    final String? name = getCmakeExecutableName(cmakeProject);

    expect(name, isNull);
  });

  testWithoutContext('generates config', () async {
    final FlutterProject project = FlutterProject.fromDirectoryTest(fileSystem.currentDirectory);
    final CmakeBasedProject cmakeProject = _FakeProject.fromFlutter(project);
    const BuildInfo buildInfo = BuildInfo(
      BuildMode.release,
      null,
      treeShakeIcons: false,
      packageConfigPath: '.dart_tool/package_config.json',
    );
    final Map<String, String> environment = <String, String>{};

    writeGeneratedCmakeConfig(_kTestFlutterRoot, cmakeProject, buildInfo, environment, logger);

    final File cmakeConfig = cmakeProject.generatedCmakeConfigFile;

    expect(cmakeConfig, exists);

    final List<String> configLines = cmakeConfig.readAsLinesSync();

    expect(
      configLines,
      containsAll(<String>[
        r'# Generated code do not commit.',
        r'file(TO_CMAKE_PATH "/flutter" FLUTTER_ROOT)',
        r'file(TO_CMAKE_PATH "/" PROJECT_DIR)',

        r'set(FLUTTER_VERSION "1.0.0" PARENT_SCOPE)',
        r'set(FLUTTER_VERSION_MAJOR 1 PARENT_SCOPE)',
        r'set(FLUTTER_VERSION_MINOR 0 PARENT_SCOPE)',
        r'set(FLUTTER_VERSION_PATCH 0 PARENT_SCOPE)',
        r'set(FLUTTER_VERSION_BUILD 0 PARENT_SCOPE)',

        r'# Environment variables to pass to tool_backend.sh',
        r'list(APPEND FLUTTER_TOOL_ENVIRONMENT',
        r'  "FLUTTER_ROOT=/flutter"',
        r'  "PROJECT_DIR=/"',
        r')',
      ]),
    );
  });

  testWithoutContext('config escapes backslashes', () async {
    fileSystem = MemoryFileSystem.test(style: FileSystemStyle.windows);

    final FlutterProject project = FlutterProject.fromDirectoryTest(fileSystem.currentDirectory);
    final CmakeBasedProject cmakeProject = _FakeProject.fromFlutter(project);
    const BuildInfo buildInfo = BuildInfo(
      BuildMode.release,
      null,
      treeShakeIcons: false,
      packageConfigPath: '.dart_tool/package_config.json',
    );

    final Map<String, String> environment = <String, String>{'TEST': r'hello\world'};

    writeGeneratedCmakeConfig(
      _kTestWindowsFlutterRoot,
      cmakeProject,
      buildInfo,
      environment,
      logger,
    );

    final File cmakeConfig = cmakeProject.generatedCmakeConfigFile;

    expect(cmakeConfig, exists);

    final List<String> configLines = cmakeConfig.readAsLinesSync();

    expect(
      configLines,
      containsAll(<String>[
        r'# Generated code do not commit.',
        r'file(TO_CMAKE_PATH "C:\\flutter" FLUTTER_ROOT)',
        r'file(TO_CMAKE_PATH "C:\\" PROJECT_DIR)',

        r'set(FLUTTER_VERSION "1.0.0" PARENT_SCOPE)',
        r'set(FLUTTER_VERSION_MAJOR 1 PARENT_SCOPE)',
        r'set(FLUTTER_VERSION_MINOR 0 PARENT_SCOPE)',
        r'set(FLUTTER_VERSION_PATCH 0 PARENT_SCOPE)',
        r'set(FLUTTER_VERSION_BUILD 0 PARENT_SCOPE)',

        r'# Environment variables to pass to tool_backend.sh',
        r'list(APPEND FLUTTER_TOOL_ENVIRONMENT',
        r'  "FLUTTER_ROOT=C:\\flutter"',
        r'  "PROJECT_DIR=C:\\"',
        r'  "TEST=hello\\world"',
        r')',
      ]),
    );
  });

  testWithoutContext('generated config uses pubspec version', () async {
    fileSystem.file('pubspec.yaml')
      ..createSync()
      ..writeAsStringSync('version: 1.2.3+4');

    final FlutterProject project = FlutterProject.fromDirectoryTest(fileSystem.currentDirectory);
    final CmakeBasedProject cmakeProject = _FakeProject.fromFlutter(project);
    const BuildInfo buildInfo = BuildInfo(
      BuildMode.release,
      null,
      treeShakeIcons: false,
      packageConfigPath: '.dart_tool/package_config.json',
    );
    final Map<String, String> environment = <String, String>{};

    writeGeneratedCmakeConfig(_kTestFlutterRoot, cmakeProject, buildInfo, environment, logger);

    final File cmakeConfig = cmakeProject.generatedCmakeConfigFile;

    expect(cmakeConfig, exists);

    final List<String> configLines = cmakeConfig.readAsLinesSync();

    expect(
      configLines,
      containsAll(<String>[
        'set(FLUTTER_VERSION "1.2.3+4" PARENT_SCOPE)',
        'set(FLUTTER_VERSION_MAJOR 1 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_MINOR 2 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_PATCH 3 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_BUILD 4 PARENT_SCOPE)',
      ]),
    );
  });

  testWithoutContext('generated config uses build name', () async {
    final FlutterProject project = FlutterProject.fromDirectoryTest(fileSystem.currentDirectory);
    final CmakeBasedProject cmakeProject = _FakeProject.fromFlutter(project);
    const BuildInfo buildInfo = BuildInfo(
      BuildMode.release,
      null,
      buildName: '1.2.3',
      treeShakeIcons: false,
      packageConfigPath: '.dart_tool/package_config.json',
    );
    final Map<String, String> environment = <String, String>{};

    writeGeneratedCmakeConfig(_kTestFlutterRoot, cmakeProject, buildInfo, environment, logger);

    final File cmakeConfig = cmakeProject.generatedCmakeConfigFile;

    expect(cmakeConfig, exists);

    final List<String> configLines = cmakeConfig.readAsLinesSync();

    expect(
      configLines,
      containsAll(<String>[
        'set(FLUTTER_VERSION "1.2.3" PARENT_SCOPE)',
        'set(FLUTTER_VERSION_MAJOR 1 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_MINOR 2 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_PATCH 3 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_BUILD 0 PARENT_SCOPE)',
      ]),
    );
  });

  testWithoutContext('generated config uses build number', () async {
    final FlutterProject project = FlutterProject.fromDirectoryTest(fileSystem.currentDirectory);
    final CmakeBasedProject cmakeProject = _FakeProject.fromFlutter(project);
    const BuildInfo buildInfo = BuildInfo(
      BuildMode.release,
      null,
      buildNumber: '4',
      treeShakeIcons: false,
      packageConfigPath: '.dart_tool/package_config.json',
    );
    final Map<String, String> environment = <String, String>{};

    writeGeneratedCmakeConfig(_kTestFlutterRoot, cmakeProject, buildInfo, environment, logger);

    final File cmakeConfig = cmakeProject.generatedCmakeConfigFile;

    expect(cmakeConfig, exists);

    final List<String> configLines = cmakeConfig.readAsLinesSync();

    expect(
      configLines,
      containsAll(<String>[
        'set(FLUTTER_VERSION "1.0.0+4" PARENT_SCOPE)',
        'set(FLUTTER_VERSION_MAJOR 1 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_MINOR 0 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_PATCH 0 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_BUILD 4 PARENT_SCOPE)',
      ]),
    );
  });

  testWithoutContext('generated config uses build name and build number', () async {
    final FlutterProject project = FlutterProject.fromDirectoryTest(fileSystem.currentDirectory);
    final CmakeBasedProject cmakeProject = _FakeProject.fromFlutter(project);
    const BuildInfo buildInfo = BuildInfo(
      BuildMode.release,
      null,
      buildName: '1.2.3',
      buildNumber: '4',
      treeShakeIcons: false,
      packageConfigPath: '.dart_tool/package_config.json',
    );
    final Map<String, String> environment = <String, String>{};

    writeGeneratedCmakeConfig(_kTestFlutterRoot, cmakeProject, buildInfo, environment, logger);

    final File cmakeConfig = cmakeProject.generatedCmakeConfigFile;

    expect(cmakeConfig, exists);

    final List<String> configLines = cmakeConfig.readAsLinesSync();

    expect(
      configLines,
      containsAll(<String>[
        'set(FLUTTER_VERSION "1.2.3+4" PARENT_SCOPE)',
        'set(FLUTTER_VERSION_MAJOR 1 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_MINOR 2 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_PATCH 3 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_BUILD 4 PARENT_SCOPE)',
      ]),
    );
  });

  testWithoutContext('generated config uses build name over pubspec version', () async {
    fileSystem.file('pubspec.yaml')
      ..createSync()
      ..writeAsStringSync('version: 9.9.9+9');

    final FlutterProject project = FlutterProject.fromDirectoryTest(fileSystem.currentDirectory);
    final CmakeBasedProject cmakeProject = _FakeProject.fromFlutter(project);
    const BuildInfo buildInfo = BuildInfo(
      BuildMode.release,
      null,
      buildName: '1.2.3',
      treeShakeIcons: false,
      packageConfigPath: '.dart_tool/package_config.json',
    );
    final Map<String, String> environment = <String, String>{};

    writeGeneratedCmakeConfig(_kTestFlutterRoot, cmakeProject, buildInfo, environment, logger);

    final File cmakeConfig = cmakeProject.generatedCmakeConfigFile;

    expect(cmakeConfig, exists);

    final List<String> configLines = cmakeConfig.readAsLinesSync();

    expect(
      configLines,
      containsAll(<String>[
        'set(FLUTTER_VERSION "1.2.3" PARENT_SCOPE)',
        'set(FLUTTER_VERSION_MAJOR 1 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_MINOR 2 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_PATCH 3 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_BUILD 0 PARENT_SCOPE)',
      ]),
    );
  });

  testWithoutContext('generated config uses build number over pubspec version', () async {
    fileSystem.file('pubspec.yaml')
      ..createSync()
      ..writeAsStringSync('version: 1.2.3+4');

    final FlutterProject project = FlutterProject.fromDirectoryTest(fileSystem.currentDirectory);
    final CmakeBasedProject cmakeProject = _FakeProject.fromFlutter(project);
    const BuildInfo buildInfo = BuildInfo(
      BuildMode.release,
      null,
      buildNumber: '5',
      treeShakeIcons: false,
      packageConfigPath: '.dart_tool/package_config.json',
    );
    final Map<String, String> environment = <String, String>{};

    writeGeneratedCmakeConfig(_kTestFlutterRoot, cmakeProject, buildInfo, environment, logger);

    final File cmakeConfig = cmakeProject.generatedCmakeConfigFile;

    expect(cmakeConfig, exists);

    final List<String> configLines = cmakeConfig.readAsLinesSync();

    expect(
      configLines,
      containsAll(<String>[
        'set(FLUTTER_VERSION "1.2.3+5" PARENT_SCOPE)',
        'set(FLUTTER_VERSION_MAJOR 1 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_MINOR 2 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_PATCH 3 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_BUILD 5 PARENT_SCOPE)',
      ]),
    );
  });

  testWithoutContext(
    'generated config uses build name and build number over pubspec version',
    () async {
      fileSystem.file('pubspec.yaml')
        ..createSync()
        ..writeAsStringSync('version: 9.9.9+9');

      final FlutterProject project = FlutterProject.fromDirectoryTest(fileSystem.currentDirectory);
      final CmakeBasedProject cmakeProject = _FakeProject.fromFlutter(project);
      const BuildInfo buildInfo = BuildInfo(
        BuildMode.release,
        null,
        buildName: '1.2.3',
        buildNumber: '4',
        treeShakeIcons: false,
        packageConfigPath: '.dart_tool/package_config.json',
      );
      final Map<String, String> environment = <String, String>{};

      writeGeneratedCmakeConfig(_kTestFlutterRoot, cmakeProject, buildInfo, environment, logger);

      final File cmakeConfig = cmakeProject.generatedCmakeConfigFile;

      expect(cmakeConfig, exists);

      final List<String> configLines = cmakeConfig.readAsLinesSync();

      expect(
        configLines,
        containsAll(<String>[
          'set(FLUTTER_VERSION "1.2.3+4" PARENT_SCOPE)',
          'set(FLUTTER_VERSION_MAJOR 1 PARENT_SCOPE)',
          'set(FLUTTER_VERSION_MINOR 2 PARENT_SCOPE)',
          'set(FLUTTER_VERSION_PATCH 3 PARENT_SCOPE)',
          'set(FLUTTER_VERSION_BUILD 4 PARENT_SCOPE)',
        ]),
      );
    },
  );

  testWithoutContext('generated config ignores invalid build name', () async {
    final FlutterProject project = FlutterProject.fromDirectoryTest(fileSystem.currentDirectory);
    final CmakeBasedProject cmakeProject = _FakeProject.fromFlutter(project);
    const BuildInfo buildInfo = BuildInfo(
      BuildMode.release,
      null,
      buildName: 'hello.world',
      treeShakeIcons: false,
      packageConfigPath: '.dart_tool/package_config.json',
    );
    final Map<String, String> environment = <String, String>{};

    writeGeneratedCmakeConfig(_kTestFlutterRoot, cmakeProject, buildInfo, environment, logger);

    final File cmakeConfig = cmakeProject.generatedCmakeConfigFile;

    expect(cmakeConfig, exists);

    final List<String> configLines = cmakeConfig.readAsLinesSync();

    expect(
      configLines,
      containsAll(<String>[
        'set(FLUTTER_VERSION "1.0.0" PARENT_SCOPE)',
        'set(FLUTTER_VERSION_MAJOR 1 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_MINOR 0 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_PATCH 0 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_BUILD 0 PARENT_SCOPE)',
      ]),
    );

    expect(
      logger.warningText,
      contains('Warning: could not parse version hello.world, defaulting to 1.0.0.'),
    );
  });

  testWithoutContext('generated config ignores invalid build number', () async {
    final FlutterProject project = FlutterProject.fromDirectoryTest(fileSystem.currentDirectory);
    final CmakeBasedProject cmakeProject = _FakeProject.fromFlutter(project);
    const BuildInfo buildInfo = BuildInfo(
      BuildMode.release,
      null,
      buildName: '1.2.3',
      buildNumber: 'foo_bar',
      treeShakeIcons: false,
      packageConfigPath: '.dart_tool/package_config.json',
    );
    final Map<String, String> environment = <String, String>{};

    writeGeneratedCmakeConfig(_kTestFlutterRoot, cmakeProject, buildInfo, environment, logger);

    final File cmakeConfig = cmakeProject.generatedCmakeConfigFile;

    expect(cmakeConfig, exists);

    final List<String> configLines = cmakeConfig.readAsLinesSync();

    expect(
      configLines,
      containsAll(<String>[
        'set(FLUTTER_VERSION "1.0.0" PARENT_SCOPE)',
        'set(FLUTTER_VERSION_MAJOR 1 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_MINOR 0 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_PATCH 0 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_BUILD 0 PARENT_SCOPE)',
      ]),
    );

    expect(
      logger.warningText,
      contains('Warning: could not parse version 1.2.3+foo_bar, defaulting to 1.0.0.'),
    );
  });

  testWithoutContext('generated config handles non-numeric build number', () async {
    final FlutterProject project = FlutterProject.fromDirectoryTest(fileSystem.currentDirectory);
    final CmakeBasedProject cmakeProject = _FakeProject.fromFlutter(project);
    const BuildInfo buildInfo = BuildInfo(
      BuildMode.release,
      null,
      buildName: '1.2.3',
      buildNumber: 'hello',
      treeShakeIcons: false,
      packageConfigPath: '.dart_tool/package_config.json',
    );
    final Map<String, String> environment = <String, String>{};

    writeGeneratedCmakeConfig(_kTestFlutterRoot, cmakeProject, buildInfo, environment, logger);

    expect(logger.warningText, isEmpty);

    final File cmakeConfig = cmakeProject.generatedCmakeConfigFile;

    expect(cmakeConfig, exists);

    final List<String> configLines = cmakeConfig.readAsLinesSync();

    expect(
      configLines,
      containsAll(<String>[
        'set(FLUTTER_VERSION "1.2.3+hello" PARENT_SCOPE)',
        'set(FLUTTER_VERSION_MAJOR 1 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_MINOR 2 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_PATCH 3 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_BUILD 0 PARENT_SCOPE)',
      ]),
    );
  });

  testWithoutContext('generated config handles complex build number', () async {
    final FlutterProject project = FlutterProject.fromDirectoryTest(fileSystem.currentDirectory);
    final CmakeBasedProject cmakeProject = _FakeProject.fromFlutter(project);
    const BuildInfo buildInfo = BuildInfo(
      BuildMode.release,
      null,
      buildName: '1.2.3',
      buildNumber: '4.5',
      treeShakeIcons: false,
      packageConfigPath: '.dart_tool/package_config.json',
    );
    final Map<String, String> environment = <String, String>{};

    writeGeneratedCmakeConfig(_kTestFlutterRoot, cmakeProject, buildInfo, environment, logger);

    expect(logger.warningText, isEmpty);

    final File cmakeConfig = cmakeProject.generatedCmakeConfigFile;

    expect(cmakeConfig, exists);

    final List<String> configLines = cmakeConfig.readAsLinesSync();

    expect(
      configLines,
      containsAll(<String>[
        'set(FLUTTER_VERSION "1.2.3+4.5" PARENT_SCOPE)',
        'set(FLUTTER_VERSION_MAJOR 1 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_MINOR 2 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_PATCH 3 PARENT_SCOPE)',
        'set(FLUTTER_VERSION_BUILD 0 PARENT_SCOPE)',
      ]),
    );
  });

  testWithoutContext(
    'generated config warns on Windows project with non-numeric build number',
    () async {
      final FlutterProject project = FlutterProject.fromDirectoryTest(fileSystem.currentDirectory);
      final CmakeBasedProject cmakeProject = WindowsProject.fromFlutter(project);
      const BuildInfo buildInfo = BuildInfo(
        BuildMode.release,
        null,
        buildName: '1.2.3',
        buildNumber: 'hello',
        treeShakeIcons: false,
        packageConfigPath: '.dart_tool/package_config.json',
      );
      final Map<String, String> environment = <String, String>{};

      writeGeneratedCmakeConfig(_kTestFlutterRoot, cmakeProject, buildInfo, environment, logger);

      expect(
        logger.warningText,
        contains(
          'Warning: build identifier hello in version 1.2.3+hello is not numeric and '
          'cannot be converted into a Windows build version number. Defaulting to 0.\n'
          'This may cause issues with Windows installers.',
        ),
      );

      final File cmakeConfig = cmakeProject.generatedCmakeConfigFile;

      expect(cmakeConfig, exists);

      final List<String> configLines = cmakeConfig.readAsLinesSync();

      expect(
        configLines,
        containsAll(<String>[
          'set(FLUTTER_VERSION "1.2.3+hello" PARENT_SCOPE)',
          'set(FLUTTER_VERSION_MAJOR 1 PARENT_SCOPE)',
          'set(FLUTTER_VERSION_MINOR 2 PARENT_SCOPE)',
          'set(FLUTTER_VERSION_PATCH 3 PARENT_SCOPE)',
          'set(FLUTTER_VERSION_BUILD 0 PARENT_SCOPE)',
        ]),
      );
    },
  );

  testWithoutContext(
    'generated config warns on Windows project with complex build number',
    () async {
      final FlutterProject project = FlutterProject.fromDirectoryTest(fileSystem.currentDirectory);
      final CmakeBasedProject cmakeProject = WindowsProject.fromFlutter(project);
      const BuildInfo buildInfo = BuildInfo(
        BuildMode.release,
        null,
        buildName: '1.2.3',
        buildNumber: '4.5',
        treeShakeIcons: false,
        packageConfigPath: '.dart_tool/package_config.json',
      );
      final Map<String, String> environment = <String, String>{};

      writeGeneratedCmakeConfig(_kTestFlutterRoot, cmakeProject, buildInfo, environment, logger);

      expect(
        logger.warningText,
        contains(
          'Warning: build identifier 4.5 in version 1.2.3+4.5 is not numeric and '
          'cannot be converted into a Windows build version number. Defaulting to 0.\n'
          'This may cause issues with Windows installers.',
        ),
      );

      final File cmakeConfig = cmakeProject.generatedCmakeConfigFile;

      expect(cmakeConfig, exists);

      final List<String> configLines = cmakeConfig.readAsLinesSync();

      expect(
        configLines,
        containsAll(<String>[
          'set(FLUTTER_VERSION "1.2.3+4.5" PARENT_SCOPE)',
          'set(FLUTTER_VERSION_MAJOR 1 PARENT_SCOPE)',
          'set(FLUTTER_VERSION_MINOR 2 PARENT_SCOPE)',
          'set(FLUTTER_VERSION_PATCH 3 PARENT_SCOPE)',
          'set(FLUTTER_VERSION_BUILD 0 PARENT_SCOPE)',
        ]),
      );
    },
  );
}

class _FakeProject implements CmakeBasedProject {
  _FakeProject.fromFlutter(this._parent);

  final FlutterProject _parent;

  @override
  bool existsSync() => _editableDirectory.existsSync();

  @override
  File get cmakeFile => _editableDirectory.childFile('CMakeLists.txt');

  @override
  File get managedCmakeFile => _managedDirectory.childFile('CMakeLists.txt');

  @override
  File get generatedCmakeConfigFile => _ephemeralDirectory.childFile('generated_config.cmake');

  @override
  File get generatedPluginCmakeFile => _managedDirectory.childFile('generated_plugins.cmake');

  @override
  Directory get pluginSymlinkDirectory => _ephemeralDirectory.childDirectory('.plugin_symlinks');

  @override
  FlutterProject get parent => _parent;

  Directory get _editableDirectory => parent.directory.childDirectory('test');
  Directory get _managedDirectory => _editableDirectory.childDirectory('flutter');
  Directory get _ephemeralDirectory => _managedDirectory.childDirectory('ephemeral');
}
