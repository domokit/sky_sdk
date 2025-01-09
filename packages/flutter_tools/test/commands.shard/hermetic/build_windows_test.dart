// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/build_windows.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/windows/build_windows.dart';
import 'package:flutter_tools/src/windows/visual_studio.dart';
import 'package:test/fake.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fakes.dart';
import '../../src/test_flutter_command_runner.dart';

const String flutterRoot = r'C:\flutter';
const String buildFilePath = r'windows\CMakeLists.txt';
const String visualStudioPath = r'C:\Program Files (x86)\Microsoft Visual Studio\2017\Community';
const String _cmakePath =
    visualStudioPath + r'\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe';
const String _defaultGenerator = 'Visual Studio 16 2019';

final Platform windowsPlatform = FakePlatform(
  operatingSystem: 'windows',
  environment: <String, String>{
    'PROGRAMFILES(X86)': r'C:\Program Files (x86)\',
    'FLUTTER_ROOT': flutterRoot,
    'USERPROFILE': '/',
  },
);
final Platform notWindowsPlatform = FakePlatform(
  environment: <String, String>{'FLUTTER_ROOT': flutterRoot},
);

void main() {
  late MemoryFileSystem fileSystem;
  late ProcessManager processManager;
  late FakeAnalytics fakeAnalytics;

  setUpAll(() {
    Cache.disableLocking();
    Cache.flutterRoot = '';
  });

  setUp(() {
    fileSystem = MemoryFileSystem.test(style: FileSystemStyle.windows);
    Cache.flutterRoot = flutterRoot;
    fakeAnalytics = getInitializedFakeAnalyticsInstance(
      fs: fileSystem,
      fakeFlutterVersion: FakeFlutterVersion(),
    );
  });

  // Creates the mock files necessary to look like a Flutter project.
  void setUpMockCoreProjectFiles() {
    fileSystem.file('pubspec.yaml').createSync();
    fileSystem.directory('.dart_tool').childFile('package_config.json').createSync(recursive: true);
    fileSystem.file(fileSystem.path.join('lib', 'main.dart')).createSync(recursive: true);
  }

  // Creates the mock files necessary to run a build.
  void setUpMockProjectFilesForBuild() {
    fileSystem.file(buildFilePath).createSync(recursive: true);
    setUpMockCoreProjectFiles();
  }

  // Returns the command matching the build_windows call to generate CMake
  // files.
  FakeCommand cmakeGenerationCommand({
    void Function(List<String> command)? onRun,
    String generator = _defaultGenerator,
    TargetPlatform targetPlatform = TargetPlatform.windows_x64,
  }) {
    return FakeCommand(
      command: <String>[
        _cmakePath,
        '-S',
        fileSystem.path.absolute(fileSystem.path.dirname(buildFilePath)),
        '-B',
        r'C:\build\windows\x64',
        '-G',
        generator,
        '-A',
        getCmakeWindowsArch(targetPlatform),
        '-DFLUTTER_TARGET_PLATFORM=windows-x64',
      ],
      onRun: onRun,
    );
  }

  // Returns the command matching the build_windows call to build.
  FakeCommand buildCommand(
    String buildMode, {
    bool verbose = false,
    void Function(List<String> command)? onRun,
    String stdout = '',
  }) {
    return FakeCommand(
      command: <String>[
        _cmakePath,
        '--build',
        r'C:\build\windows\x64',
        '--config',
        buildMode,
        ...<String>['--target', 'INSTALL'],
        if (verbose) '--verbose',
      ],
      environment: <String, String>{if (verbose) 'VERBOSE_SCRIPT_LOGGING': 'true'},
      onRun: onRun,
      stdout: stdout,
    );
  }

  testUsingContext(
    'Windows build fails when there is no cmake path',
    () async {
      final BuildWindowsCommand command = BuildWindowsCommand(
        logger: BufferLogger.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      )..visualStudioOverride = FakeVisualStudio(cmakePath: null);
      setUpMockProjectFilesForBuild();

      expect(
        createTestCommandRunner(command).run(const <String>['windows', '--no-pub']),
        throwsToolExit(),
      );
    },
    overrides: <Type, Generator>{
      Platform: () => windowsPlatform,
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
    },
  );

  testUsingContext(
    'Windows build fails when there is no windows project',
    () async {
      final FakeVisualStudio fakeVisualStudio = FakeVisualStudio();
      final BuildWindowsCommand command = BuildWindowsCommand(
        logger: BufferLogger.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      )..visualStudioOverride = fakeVisualStudio;
      setUpMockCoreProjectFiles();

      expect(
        createTestCommandRunner(command).run(const <String>['windows', '--no-pub']),
        throwsToolExit(
          message:
              'No Windows desktop project configured. See '
              'https://flutter.dev/to/add-desktop-support '
              'to learn about adding Windows support to a project.',
        ),
      );
    },
    overrides: <Type, Generator>{
      Platform: () => windowsPlatform,
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
    },
  );

  testUsingContext(
    'Windows build fails on non windows platform',
    () async {
      final FakeVisualStudio fakeVisualStudio = FakeVisualStudio();
      final BuildWindowsCommand command = BuildWindowsCommand(
        logger: BufferLogger.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      )..visualStudioOverride = fakeVisualStudio;
      setUpMockProjectFilesForBuild();

      expect(
        createTestCommandRunner(command).run(const <String>['windows', '--no-pub']),
        throwsToolExit(message: '"build windows" only supported on Windows hosts.'),
      );
    },
    overrides: <Type, Generator>{
      Platform: () => notWindowsPlatform,
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
    },
  );

  testUsingContext(
    'Windows build fails when feature is disabled',
    () async {
      final FakeVisualStudio fakeVisualStudio = FakeVisualStudio();
      final BuildWindowsCommand command = BuildWindowsCommand(
        logger: BufferLogger.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      )..visualStudioOverride = fakeVisualStudio;
      setUpMockProjectFilesForBuild();

      expect(
        createTestCommandRunner(command).run(const <String>['windows', '--no-pub']),
        throwsToolExit(
          message:
              '"build windows" is not currently supported. To enable, run "flutter config --enable-windows-desktop".',
        ),
      );
    },
    overrides: <Type, Generator>{
      Platform: () => windowsPlatform,
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
      FeatureFlags: () => TestFeatureFlags(),
    },
  );

  testUsingContext(
    'Windows build does not spew stdout to status logger',
    () async {
      final FakeVisualStudio fakeVisualStudio = FakeVisualStudio();
      final BuildWindowsCommand command = BuildWindowsCommand(
        logger: BufferLogger.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      )..visualStudioOverride = fakeVisualStudio;
      setUpMockProjectFilesForBuild();

      processManager = FakeProcessManager.list(<FakeCommand>[
        cmakeGenerationCommand(),
        buildCommand('Release', stdout: 'STDOUT STUFF'),
      ]);

      await createTestCommandRunner(command).run(const <String>['windows', '--no-pub']);
      expect(testLogger.statusText, isNot(contains('STDOUT STUFF')));
      expect(testLogger.traceText, contains('STDOUT STUFF'));
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Platform: () => windowsPlatform,
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
    },
  );

  testUsingContext(
    'Windows build sends timing events',
    () async {
      final FakeVisualStudio fakeVisualStudio = FakeVisualStudio();
      final BuildWindowsCommand command = BuildWindowsCommand(
        logger: BufferLogger.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      )..visualStudioOverride = fakeVisualStudio;
      setUpMockProjectFilesForBuild();

      processManager = FakeProcessManager.list(<FakeCommand>[
        cmakeGenerationCommand(),
        buildCommand('Release'),
      ]);

      await createTestCommandRunner(command).run(const <String>['windows', '--no-pub']);

      expect(
        analyticsTimingEventExists(
          sentEvents: fakeAnalytics.sentEvents,
          workflow: 'build',
          variableName: 'windows-cmake-generation',
        ),
        true,
      );
      expect(
        analyticsTimingEventExists(
          sentEvents: fakeAnalytics.sentEvents,
          workflow: 'build',
          variableName: 'windows-cmake-build',
        ),
        true,
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Platform: () => windowsPlatform,
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
      Analytics: () => fakeAnalytics,
    },
  );

  testUsingContext(
    'Windows build extracts errors from stdout',
    () async {
      final FakeVisualStudio fakeVisualStudio = FakeVisualStudio();
      final BuildWindowsCommand command = BuildWindowsCommand(
        logger: BufferLogger.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      )..visualStudioOverride = fakeVisualStudio;
      setUpMockProjectFilesForBuild();

      // This contains a mix of routine build output and various types of errors
      // (compile error, link error, warning treated as an error) from MSBuild,
      // edited down for compactness. For instance, where similar lines are
      // repeated in actual output, one or two representative lines are chosen
      // to be included here.
      const String stdout = r'''
Microsoft (R) Build Engine version 16.6.0+5ff7b0c9e for .NET Framework
Copyright (C) Microsoft Corporation. All rights reserved.

  Checking Build System
  Generating C:/foo/windows/x64/flutter/ephemeral/flutter_windows.dll, [etc], _phony_
  Building Custom Rule C:/foo/windows/x64/flutter/CMakeLists.txt
  standard_codec.cc
  Generating Code...
  flutter_wrapper_plugin.vcxproj -> C:\foo\build\windows\x64\flutter\Debug\flutter_wrapper_plugin.lib
C:\foo\windows\x64\runner\main.cpp(18): error C2220: the following warning is treated as an error [C:\foo\build\windows\x64\runner\test.vcxproj]
C:\foo\windows\x64\runner\main.cpp(18): warning C4706: assignment within conditional expression [C:\foo\build\windows\x64\runner\test.vcxproj]
main.obj : error LNK2019: unresolved external symbol "void __cdecl Bar(void)" (?Bar@@YAXXZ) referenced in function wWinMain [C:\foo\build\windows\x64\runner\test.vcxproj]
C:\foo\build\windows\x64\runner\Debug\test.exe : fatal error LNK1120: 1 unresolved externals [C:\foo\build\windows\x64\runner\test.vcxproj]
  Building Custom Rule C:/foo/windows/x64/runner/CMakeLists.txt
  flutter_window.cpp
  main.cpp
C:\foo\windows\x64\runner\main.cpp(17,1): error C2065: 'Baz': undeclared identifier [C:\foo\build\windows\x64\runner\test.vcxproj]
  -- Install configuration: "Debug"
  -- Installing: C:/foo/build/windows/x64/runner/Debug/data/icudtl.dat
''';

      processManager = FakeProcessManager.list(<FakeCommand>[
        cmakeGenerationCommand(),
        buildCommand('Release', stdout: stdout),
      ]);

      await createTestCommandRunner(command).run(const <String>['windows', '--no-pub']);
      // Just the warnings and errors should be surfaced.
      expect(testLogger.errorText, r'''
C:\foo\windows\x64\runner\main.cpp(18): error C2220: the following warning is treated as an error [C:\foo\build\windows\x64\runner\test.vcxproj]
C:\foo\windows\x64\runner\main.cpp(18): warning C4706: assignment within conditional expression [C:\foo\build\windows\x64\runner\test.vcxproj]
main.obj : error LNK2019: unresolved external symbol "void __cdecl Bar(void)" (?Bar@@YAXXZ) referenced in function wWinMain [C:\foo\build\windows\x64\runner\test.vcxproj]
C:\foo\build\windows\x64\runner\Debug\test.exe : fatal error LNK1120: 1 unresolved externals [C:\foo\build\windows\x64\runner\test.vcxproj]
C:\foo\windows\x64\runner\main.cpp(17,1): error C2065: 'Baz': undeclared identifier [C:\foo\build\windows\x64\runner\test.vcxproj]
''');
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Platform: () => windowsPlatform,
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
    },
  );

  testUsingContext(
    'Windows verbose build sets VERBOSE_SCRIPT_LOGGING',
    () async {
      final FakeVisualStudio fakeVisualStudio = FakeVisualStudio();
      final BuildWindowsCommand command = BuildWindowsCommand(
        logger: BufferLogger.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      )..visualStudioOverride = fakeVisualStudio;
      setUpMockProjectFilesForBuild();

      processManager = FakeProcessManager.list(<FakeCommand>[
        cmakeGenerationCommand(),
        buildCommand('Release', verbose: true, stdout: 'STDOUT STUFF'),
      ]);

      await createTestCommandRunner(command).run(const <String>['windows', '--no-pub', '-v']);
      expect(testLogger.statusText, contains('STDOUT STUFF'));
      expect(testLogger.traceText, isNot(contains('STDOUT STUFF')));
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Platform: () => windowsPlatform,
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
    },
  );

  testUsingContext(
    'Windows build works around CMake generation bug',
    () async {
      final FakeVisualStudio fakeVisualStudio = FakeVisualStudio(displayVersion: '17.1.0');
      final BuildWindowsCommand command = BuildWindowsCommand(
        logger: BufferLogger.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      )..visualStudioOverride = fakeVisualStudio;
      setUpMockProjectFilesForBuild();

      processManager = FakeProcessManager.list(<FakeCommand>[
        cmakeGenerationCommand(),
        buildCommand('Release'),
      ]);
      fileSystem.file(fileSystem.path.join('lib', 'other.dart')).createSync(recursive: true);
      fileSystem.file(fileSystem.path.join('foo', 'bar.sksl.json')).createSync(recursive: true);

      // Relevant portions of an incorrectly generated project, with some
      // irrelevant details removed for length.
      const String fakeBadProjectContent = r'''
<?xml version="1.0" encoding="utf-8"?>
<Project DefaultTargets="Build" ToolsVersion="17.0" xmlns="http://schemas.microsoft.com/developer/msbuild/2003">
  <ItemGroup>
    <CustomBuild Include="somepath\build\windows\x64\CMakeFiles\8b570225f626c250e12bc1ede88babae\flutter_windows.dll.rule">
      <Message Condition="'$(Configuration)|$(Platform)'=='Debug|x64'">Generating some files</Message>
      <Command Condition="'$(Configuration)|$(Platform)'=='Debug|x64'">setlocal
"C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -E env FOO=bar C:/src/flutter/packages/flutter_tools/bin/tool_backend.bat windows-x64 Debug
endlocal &amp; call :cmErrorLevel %errorlevel% &amp; goto :cmDone
:cmErrorLevel
exit /b %1
:cmDone
if %errorlevel% neq 0 goto :VCEnd</Command>
      <Message Condition="'$(Configuration)|$(Platform)'=='Profile|x64'">Generating some files</Message>
      <Command Condition="'$(Configuration)|$(Platform)'=='Profile|x64'">setlocal
"C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -E env FOO=bar C:/src/flutter/packages/flutter_tools/bin/tool_backend.bat windows-x64 Debug
endlocal &amp; call :cmErrorLevel %errorlevel% &amp; goto :cmDone
:cmErrorLevel
exit /b %1
:cmDone
if %errorlevel% neq 0 goto :VCEnd</Command>
      <Message Condition="'$(Configuration)|$(Platform)'=='Release|x64'">Generating some files</Message>
      <Command Condition="'$(Configuration)|$(Platform)'=='Release|x64'">setlocal
"C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -E env FOO=bar C:/src/flutter/packages/flutter_tools/bin/tool_backend.bat windows-x64 Debug
endlocal &amp; call :cmErrorLevel %errorlevel% &amp; goto :cmDone
:cmErrorLevel
exit /b %1
:cmDone
if %errorlevel% neq 0 goto :VCEnd</Command>
      <Message Condition="'$(Configuration)|$(Platform)'=='Debug|x64'">Generating some files</Message>
      <Command Condition="'$(Configuration)|$(Platform)'=='Debug|x64'">setlocal
"C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -E env FOO=bar C:/src/flutter/packages/flutter_tools/bin/tool_backend.bat windows-x64 Profile
endlocal &amp; call :cmErrorLevel %errorlevel% &amp; goto :cmDone
:cmErrorLevel
exit /b %1
:cmDone
if %errorlevel% neq 0 goto :VCEnd</Command>
      <Message Condition="'$(Configuration)|$(Platform)'=='Profile|x64'">Generating some files</Message>
      <Command Condition="'$(Configuration)|$(Platform)'=='Profile|x64'">setlocal
"C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -E env FOO=bar C:/src/flutter/packages/flutter_tools/bin/tool_backend.bat windows-x64 Profile
endlocal &amp; call :cmErrorLevel %errorlevel% &amp; goto :cmDone
:cmErrorLevel
exit /b %1
:cmDone
if %errorlevel% neq 0 goto :VCEnd</Command>
      <Message Condition="'$(Configuration)|$(Platform)'=='Release|x64'">Generating some files</Message>
      <Command Condition="'$(Configuration)|$(Platform)'=='Release|x64'">setlocal
"C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -E env FOO=bar C:/src/flutter/packages/flutter_tools/bin/tool_backend.bat windows-x64 Profile
endlocal &amp; call :cmErrorLevel %errorlevel% &amp; goto :cmDone
:cmErrorLevel
exit /b %1
:cmDone
if %errorlevel% neq 0 goto :VCEnd</Command>
      <Message Condition="'$(Configuration)|$(Platform)'=='Debug|x64'">Generating some files</Message>
      <Command Condition="'$(Configuration)|$(Platform)'=='Debug|x64'">setlocal
"C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -E env FOO=bar C:/src/flutter/packages/flutter_tools/bin/tool_backend.bat windows-x64 Release
endlocal &amp; call :cmErrorLevel %errorlevel% &amp; goto :cmDone
:cmErrorLevel
exit /b %1
:cmDone
if %errorlevel% neq 0 goto :VCEnd</Command>
      <Message Condition="'$(Configuration)|$(Platform)'=='Profile|x64'">Generating some files</Message>
      <Command Condition="'$(Configuration)|$(Platform)'=='Profile|x64'">setlocal
"C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -E env FOO=bar C:/src/flutter/packages/flutter_tools/bin/tool_backend.bat windows-x64 Release
endlocal &amp; call :cmErrorLevel %errorlevel% &amp; goto :cmDone
:cmErrorLevel
exit /b %1
:cmDone
if %errorlevel% neq 0 goto :VCEnd</Command>
      <Message Condition="'$(Configuration)|$(Platform)'=='Release|x64'">Generating some files</Message>
      <Command Condition="'$(Configuration)|$(Platform)'=='Release|x64'">setlocal
"C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -E env FOO=bar C:/src/flutter/packages/flutter_tools/bin/tool_backend.bat windows-x64 Release
endlocal &amp; call :cmErrorLevel %errorlevel% &amp; goto :cmDone
:cmErrorLevel
exit /b %1
:cmDone
if %errorlevel% neq 0 goto :VCEnd</Command>
    </CustomBuild>
  </ItemGroup>
</Project>
''';
      final File assembleProject = fileSystem.currentDirectory
          .childDirectory('build')
          .childDirectory('windows')
          .childDirectory('x64')
          .childDirectory('flutter')
          .childFile('flutter_assemble.vcxproj');
      assembleProject.createSync(recursive: true);
      assembleProject.writeAsStringSync(fakeBadProjectContent);

      await createTestCommandRunner(command).run(const <String>['windows', '--no-pub']);

      final List<String> projectLines = assembleProject.readAsLinesSync();

      const String commandBase =
          r'"C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" '
          r'-E env FOO=bar C:/src/flutter/packages/flutter_tools/bin/tool_backend.bat windows-x64';
      // The duplicate commands will still be present, but with the order matching
      // the condition order (cycling through the configurations), rather than
      // three copies of Debug, then three copies of Profile, then three copies
      // of Release.
      expect(
        projectLines,
        containsAllInOrder(<String>[
          '$commandBase Debug\r',
          '$commandBase Profile\r',
          '$commandBase Release\r',
          '$commandBase Debug\r',
          '$commandBase Profile\r',
          '$commandBase Release\r',
          '$commandBase Debug\r',
          '$commandBase Profile\r',
          '$commandBase Release\r',
        ]),
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Platform: () => windowsPlatform,
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
    },
  );

  testUsingContext(
    'Windows build invokes build and writes generated files',
    () async {
      final FakeVisualStudio fakeVisualStudio = FakeVisualStudio();
      final BuildWindowsCommand command = BuildWindowsCommand(
        logger: BufferLogger.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      )..visualStudioOverride = fakeVisualStudio;
      setUpMockProjectFilesForBuild();

      processManager = FakeProcessManager.list(<FakeCommand>[
        cmakeGenerationCommand(),
        buildCommand('Release'),
      ]);
      fileSystem.file(fileSystem.path.join('lib', 'other.dart')).createSync(recursive: true);
      fileSystem.file(fileSystem.path.join('foo', 'bar.sksl.json')).createSync(recursive: true);

      await createTestCommandRunner(command).run(const <String>[
        'windows',
        '--no-pub',
        '--track-widget-creation',
        '--obfuscate',
        '--tree-shake-icons',
        '--enable-experiment=non-nullable',
        r'--split-debug-info=C:\foo\',
        '--dart-define=foo=a',
        '--dart-define=bar=b',
        r'--bundle-sksl-path=foo\bar.sksl.json',
        r'--target=lib\other.dart',
      ]);

      final File cmakeConfig = fileSystem.currentDirectory
          .childDirectory('windows')
          .childDirectory('flutter')
          .childDirectory('ephemeral')
          .childFile('generated_config.cmake');

      expect(cmakeConfig, exists);

      final List<String> configLines = cmakeConfig.readAsLinesSync();

      // Backslashes are escaped in the file, which is why this uses both raw
      // strings and double backslashes.
      expect(
        configLines,
        containsAll(<String>[
          r'file(TO_CMAKE_PATH "C:\\flutter" FLUTTER_ROOT)',
          r'file(TO_CMAKE_PATH "C:\\" PROJECT_DIR)',
          r'set(FLUTTER_VERSION "1.0.0" PARENT_SCOPE)',
          r'set(FLUTTER_VERSION_MAJOR 1 PARENT_SCOPE)',
          r'set(FLUTTER_VERSION_MINOR 0 PARENT_SCOPE)',
          r'set(FLUTTER_VERSION_PATCH 0 PARENT_SCOPE)',
          r'set(FLUTTER_VERSION_BUILD 0 PARENT_SCOPE)',
          r'  "DART_DEFINES=Zm9vPWE=,YmFyPWI="',
          r'  "DART_OBFUSCATION=true"',
          r'  "EXTRA_FRONT_END_OPTIONS=--enable-experiment=non-nullable"',
          r'  "EXTRA_GEN_SNAPSHOT_OPTIONS=--enable-experiment=non-nullable"',
          r'  "SPLIT_DEBUG_INFO=C:\\foo\\"',
          r'  "TRACK_WIDGET_CREATION=true"',
          r'  "TREE_SHAKE_ICONS=true"',
          r'  "FLUTTER_ROOT=C:\\flutter"',
          r'  "PROJECT_DIR=C:\\"',
          r'  "FLUTTER_TARGET=lib\\other.dart"',
          r'  "BUNDLE_SKSL_PATH=foo\\bar.sksl.json"',
        ]),
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Platform: () => windowsPlatform,
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
    },
  );

  testUsingContext(
    'Windows profile build passes Profile configuration',
    () async {
      final FakeVisualStudio fakeVisualStudio = FakeVisualStudio();
      final BuildWindowsCommand command = BuildWindowsCommand(
        logger: BufferLogger.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      )..visualStudioOverride = fakeVisualStudio;
      setUpMockProjectFilesForBuild();

      processManager = FakeProcessManager.list(<FakeCommand>[
        cmakeGenerationCommand(),
        buildCommand('Profile'),
      ]);

      await createTestCommandRunner(
        command,
      ).run(const <String>['windows', '--profile', '--no-pub']);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Platform: () => windowsPlatform,
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
    },
  );

  testUsingContext(
    'Windows build outputs path when successful',
    () async {
      final FakeVisualStudio fakeVisualStudio = FakeVisualStudio();
      final BuildWindowsCommand command = BuildWindowsCommand(
        logger: BufferLogger.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      )..visualStudioOverride = fakeVisualStudio;
      setUpMockProjectFilesForBuild();

      processManager = FakeProcessManager.list(<FakeCommand>[
        cmakeGenerationCommand(),
        buildCommand('Release'),
      ]);

      await createTestCommandRunner(
        command,
      ).run(const <String>['windows', '--release', '--no-pub']);
      expect(testLogger.statusText, contains(r'✓ Built build\windows\x64\runner\Release'));
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Platform: () => windowsPlatform,
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
    },
  );

  testUsingContext(
    'Windows build passes correct generator',
    () async {
      const String generator = 'A different generator';
      final FakeVisualStudio fakeVisualStudio = FakeVisualStudio(cmakeGenerator: generator);
      final BuildWindowsCommand command = BuildWindowsCommand(
        logger: BufferLogger.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      )..visualStudioOverride = fakeVisualStudio;
      setUpMockProjectFilesForBuild();

      processManager = FakeProcessManager.list(<FakeCommand>[
        cmakeGenerationCommand(generator: generator),
        buildCommand('Release'),
      ]);

      await createTestCommandRunner(
        command,
      ).run(const <String>['windows', '--release', '--no-pub']);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Platform: () => windowsPlatform,
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
    },
  );

  testUsingContext(
    "Windows build uses pubspec's version",
    () async {
      final FakeVisualStudio fakeVisualStudio = FakeVisualStudio();
      final BuildWindowsCommand command = BuildWindowsCommand(
        logger: BufferLogger.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      )..visualStudioOverride = fakeVisualStudio;
      setUpMockProjectFilesForBuild();

      fileSystem.file('pubspec.yaml')
        ..createSync()
        ..writeAsStringSync('version: 1.2.3+4');

      processManager = FakeProcessManager.list(<FakeCommand>[
        cmakeGenerationCommand(),
        buildCommand('Release'),
      ]);

      await createTestCommandRunner(command).run(const <String>['windows', '--no-pub']);

      final File cmakeConfig = fileSystem.currentDirectory
          .childDirectory('windows')
          .childDirectory('flutter')
          .childDirectory('ephemeral')
          .childFile('generated_config.cmake');

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
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Platform: () => windowsPlatform,
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
    },
  );

  testUsingContext(
    'Windows build uses build-name and build-number',
    () async {
      final FakeVisualStudio fakeVisualStudio = FakeVisualStudio();
      final BuildWindowsCommand command = BuildWindowsCommand(
        logger: BufferLogger.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      )..visualStudioOverride = fakeVisualStudio;
      setUpMockProjectFilesForBuild();

      processManager = FakeProcessManager.list(<FakeCommand>[
        cmakeGenerationCommand(),
        buildCommand('Release'),
      ]);

      await createTestCommandRunner(
        command,
      ).run(const <String>['windows', '--no-pub', '--build-name=1.2.3', '--build-number=4']);

      final File cmakeConfig = fileSystem.currentDirectory
          .childDirectory('windows')
          .childDirectory('flutter')
          .childDirectory('ephemeral')
          .childFile('generated_config.cmake');

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
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Platform: () => windowsPlatform,
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
    },
  );

  testUsingContext(
    'Windows build build-name overrides pubspec',
    () async {
      final FakeVisualStudio fakeVisualStudio = FakeVisualStudio();
      final BuildWindowsCommand command = BuildWindowsCommand(
        logger: BufferLogger.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      )..visualStudioOverride = fakeVisualStudio;
      setUpMockProjectFilesForBuild();

      fileSystem.file('pubspec.yaml')
        ..createSync()
        ..writeAsStringSync('version: 9.9.9+9');

      processManager = FakeProcessManager.list(<FakeCommand>[
        cmakeGenerationCommand(),
        buildCommand('Release'),
      ]);

      await createTestCommandRunner(
        command,
      ).run(const <String>['windows', '--no-pub', '--build-name=1.2.3']);

      final File cmakeConfig = fileSystem.currentDirectory
          .childDirectory('windows')
          .childDirectory('flutter')
          .childDirectory('ephemeral')
          .childFile('generated_config.cmake');

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
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Platform: () => windowsPlatform,
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
    },
  );

  testUsingContext(
    'Windows build build-number overrides pubspec',
    () async {
      final FakeVisualStudio fakeVisualStudio = FakeVisualStudio();
      final BuildWindowsCommand command = BuildWindowsCommand(
        logger: BufferLogger.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      )..visualStudioOverride = fakeVisualStudio;
      setUpMockProjectFilesForBuild();

      fileSystem.file('pubspec.yaml')
        ..createSync()
        ..writeAsStringSync('version: 1.2.3+9');

      processManager = FakeProcessManager.list(<FakeCommand>[
        cmakeGenerationCommand(),
        buildCommand('Release'),
      ]);

      await createTestCommandRunner(
        command,
      ).run(const <String>['windows', '--no-pub', '--build-number=4']);

      final File cmakeConfig = fileSystem.currentDirectory
          .childDirectory('windows')
          .childDirectory('flutter')
          .childDirectory('ephemeral')
          .childFile('generated_config.cmake');

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
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Platform: () => windowsPlatform,
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
    },
  );

  testUsingContext(
    'Windows build build-name and build-number override pubspec',
    () async {
      final FakeVisualStudio fakeVisualStudio = FakeVisualStudio();
      final BuildWindowsCommand command = BuildWindowsCommand(
        logger: BufferLogger.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      )..visualStudioOverride = fakeVisualStudio;
      setUpMockProjectFilesForBuild();

      fileSystem.file('pubspec.yaml')
        ..createSync()
        ..writeAsStringSync('version: 9.9.9+9');

      processManager = FakeProcessManager.list(<FakeCommand>[
        cmakeGenerationCommand(),
        buildCommand('Release'),
      ]);

      await createTestCommandRunner(
        command,
      ).run(const <String>['windows', '--no-pub', '--build-name=1.2.3', '--build-number=4']);

      final File cmakeConfig = fileSystem.currentDirectory
          .childDirectory('windows')
          .childDirectory('flutter')
          .childDirectory('ephemeral')
          .childFile('generated_config.cmake');

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
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Platform: () => windowsPlatform,
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
    },
  );

  testUsingContext(
    'Windows build warns on non-numeric build-number',
    () async {
      final FakeVisualStudio fakeVisualStudio = FakeVisualStudio();
      final BuildWindowsCommand command = BuildWindowsCommand(
        logger: BufferLogger.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      )..visualStudioOverride = fakeVisualStudio;
      setUpMockProjectFilesForBuild();

      processManager = FakeProcessManager.list(<FakeCommand>[
        cmakeGenerationCommand(),
        buildCommand('Release'),
      ]);

      await createTestCommandRunner(
        command,
      ).run(const <String>['windows', '--no-pub', '--build-name=1.2.3', '--build-number=hello']);

      final File cmakeConfig = fileSystem.currentDirectory
          .childDirectory('windows')
          .childDirectory('flutter')
          .childDirectory('ephemeral')
          .childFile('generated_config.cmake');

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

      expect(
        testLogger.warningText,
        contains(
          'Warning: build identifier hello in version 1.2.3+hello is not numeric and '
          'cannot be converted into a Windows build version number. Defaulting to 0.\n'
          'This may cause issues with Windows installers.',
        ),
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Platform: () => windowsPlatform,
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
    },
  );

  testUsingContext(
    'Windows build warns on complex build-number',
    () async {
      final FakeVisualStudio fakeVisualStudio = FakeVisualStudio();
      final BuildWindowsCommand command = BuildWindowsCommand(
        logger: BufferLogger.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      )..visualStudioOverride = fakeVisualStudio;
      setUpMockProjectFilesForBuild();

      processManager = FakeProcessManager.list(<FakeCommand>[
        cmakeGenerationCommand(),
        buildCommand('Release'),
      ]);

      await createTestCommandRunner(
        command,
      ).run(const <String>['windows', '--no-pub', '--build-name=1.2.3', '--build-number=4.5']);

      final File cmakeConfig = fileSystem.currentDirectory
          .childDirectory('windows')
          .childDirectory('flutter')
          .childDirectory('ephemeral')
          .childFile('generated_config.cmake');

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

      expect(
        testLogger.warningText,
        contains(
          'Warning: build identifier 4.5 in version 1.2.3+4.5 is not numeric and '
          'cannot be converted into a Windows build version number. Defaulting to 0.\n'
          'This may cause issues with Windows installers.',
        ),
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Platform: () => windowsPlatform,
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
    },
  );

  testUsingContext(
    'hidden when not enabled on Windows host',
    () {
      expect(
        BuildWindowsCommand(
          logger: BufferLogger.test(),
          operatingSystemUtils: FakeOperatingSystemUtils(),
        ).hidden,
        true,
      );
    },
    overrides: <Type, Generator>{
      FeatureFlags: () => TestFeatureFlags(),
      Platform: () => windowsPlatform,
    },
  );

  testUsingContext(
    'Not hidden when enabled and on Windows host',
    () {
      expect(
        BuildWindowsCommand(
          logger: BufferLogger.test(),
          operatingSystemUtils: FakeOperatingSystemUtils(),
        ).hidden,
        false,
      );
    },
    overrides: <Type, Generator>{
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
      Platform: () => windowsPlatform,
    },
  );

  testUsingContext(
    'Performs code size analysis and sends analytics',
    () async {
      final FakeVisualStudio fakeVisualStudio = FakeVisualStudio();
      final BuildWindowsCommand command = BuildWindowsCommand(
        logger: BufferLogger.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      )..visualStudioOverride = fakeVisualStudio;
      setUpMockProjectFilesForBuild();

      fileSystem.file(r'build\windows\x64\runner\Release\app.so')
        ..createSync(recursive: true)
        ..writeAsBytesSync(List<int>.generate(10000, (int index) => 0));

      processManager = FakeProcessManager.list(<FakeCommand>[
        cmakeGenerationCommand(),
        buildCommand(
          'Release',
          onRun: (_) {
            fileSystem.file(r'build\flutter_size_01\snapshot.windows-x64.json')
              ..createSync(recursive: true)
              ..writeAsStringSync('''
[
  {
    "l": "dart:_internal",
    "c": "SubListIterable",
    "n": "[Optimized] skip",
    "s": 2400
  }
]''');
            fileSystem.file(r'build\flutter_size_01\trace.windows-x64.json')
              ..createSync(recursive: true)
              ..writeAsStringSync('{}');
          },
        ),
      ]);

      await createTestCommandRunner(
        command,
      ).run(const <String>['windows', '--no-pub', '--analyze-size']);

      expect(
        testLogger.statusText,
        contains('A summary of your Windows bundle analysis can be found at'),
      );
      expect(testLogger.statusText, contains('dart devtools --appSizeBase='));
      expect(fakeAnalytics.sentEvents, contains(Event.codeSizeAnalysis(platform: 'windows')));
    },
    overrides: <Type, Generator>{
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Platform: () => windowsPlatform,
      FileSystemUtils: () => FileSystemUtils(fileSystem: fileSystem, platform: windowsPlatform),
      Analytics: () => fakeAnalytics,
    },
  );

  // Confirms that running for Windows in a directory with a
  // bad character (' in this case) throws the desired error message
  // If the issue https://github.com/flutter/flutter/issues/104802 ever
  // is resolved on the VS side, we can allow these paths again
  testUsingContext(
    'Test bad path characters',
    () async {
      final FakeVisualStudio fakeVisualStudio = FakeVisualStudio();
      final BuildWindowsCommand command = BuildWindowsCommand(
        logger: BufferLogger.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      )..visualStudioOverride = fakeVisualStudio;
      fileSystem.currentDirectory = fileSystem.directory("test_'path")..createSync();
      final String absPath = fileSystem.currentDirectory.absolute.path;
      setUpMockCoreProjectFiles();

      expect(
        createTestCommandRunner(command).run(const <String>['windows', '--no-pub']),
        throwsToolExit(
          message:
              'Path $absPath contains invalid characters in "\'#!\$^&*=|,;<>?". '
              'Please rename your directory so as to not include any of these characters '
              'and retry.',
        ),
      );
    },
    overrides: <Type, Generator>{
      Platform: () => windowsPlatform,
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
    },
  );

  // Tests the case where stdout contains the error about pubspec.yaml
  // And tests the case where stdout contains the error about missing assets
  testUsingContext(
    'Windows build extracts errors related to pubspec.yaml from stdout',
    () async {
      final FakeVisualStudio fakeVisualStudio = FakeVisualStudio();
      final BuildWindowsCommand command = BuildWindowsCommand(
        logger: BufferLogger.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      )..visualStudioOverride = fakeVisualStudio;
      setUpMockProjectFilesForBuild();

      const String stdout = r'''
Error detected in pubspec.yaml:
No file or variants found for asset: images/a_dot_burr.jpeg.
''';

      processManager = FakeProcessManager.list(<FakeCommand>[
        cmakeGenerationCommand(),
        buildCommand('Release', stdout: stdout),
      ]);

      await createTestCommandRunner(command).run(const <String>['windows', '--no-pub']);
      // Just the warnings and errors should be surfaced.
      expect(testLogger.errorText, r'''
Error detected in pubspec.yaml:
No file or variants found for asset: images/a_dot_burr.jpeg.
''');
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Platform: () => windowsPlatform,
      FeatureFlags: () => TestFeatureFlags(isWindowsEnabled: true),
    },
  );
}

class FakeVisualStudio extends Fake implements VisualStudio {
  FakeVisualStudio({
    this.cmakePath = _cmakePath,
    this.cmakeGenerator = 'Visual Studio 16 2019',
    this.displayVersion = '17.0.0',
  });

  @override
  final String? cmakePath;

  @override
  final String cmakeGenerator;

  @override
  final String displayVersion;
}
