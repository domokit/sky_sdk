// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart' show ProcessException;
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/convert.dart';
import 'package:flutter_tools/src/windows/visual_studio.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fakes.dart';

const String programFilesPath = r'C:\Program Files (x86)';
const String visualStudioPath = programFilesPath + r'\Microsoft Visual Studio\2017\Community';
const String cmakePath =
    visualStudioPath + r'\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe';
const String vswherePath = programFilesPath + r'\Microsoft Visual Studio\Installer\vswhere.exe';
const String clPath = visualStudioPath + r'\VC\Tools\MSVC\14.35.32215\bin\Hostx64\x64\cl.exe';
const String libPath = visualStudioPath + r'\VC\Tools\MSVC\14.35.32215\bin\Hostx64\x64\lib.exe';
const String linkPath = visualStudioPath + r'\VC\Tools\MSVC\14.35.32215\bin\Hostx64\x64\link.exe';
const String vcvarsPath = visualStudioPath + r'\VC\Auxiliary\Build\vcvars64.bat';

final Platform windowsPlatform = FakePlatform(
  operatingSystem: 'windows',
  environment: <String, String>{'PROGRAMFILES(X86)': r'C:\Program Files (x86)\'},
);

// A minimum version of a response where a VS installation was found.
const Map<String, dynamic> _defaultResponse = <String, dynamic>{
  'installationPath': visualStudioPath,
  'displayName': 'Visual Studio Community 2019',
  'installationVersion': '16.2.29306.81',
  'isRebootRequired': false,
  'isComplete': true,
  'isLaunchable': true,
  'isPrerelease': false,
  'catalog': <String, dynamic>{'productDisplayVersion': '16.2.5'},
};

// A minimum version of a response where a VS 2022 installation was found.
const Map<String, dynamic> _vs2022Response = <String, dynamic>{
  'installationPath': visualStudioPath,
  'displayName': 'Visual Studio Community 2022',
  'installationVersion': '17.0.31903.59',
  'isRebootRequired': false,
  'isComplete': true,
  'isLaunchable': true,
  'isPrerelease': false,
  'catalog': <String, dynamic>{'productDisplayVersion': '17.0.0'},
};

// A minimum version of a response where a Build Tools installation was found.
const Map<String, dynamic> _defaultBuildToolsResponse = <String, dynamic>{
  'installationPath': visualStudioPath,
  'displayName': 'Visual Studio Build Tools 2019',
  'installationVersion': '16.7.30413.136',
  'isRebootRequired': false,
  'isComplete': true,
  'isLaunchable': true,
  'isPrerelease': false,
  'catalog': <String, dynamic>{'productDisplayVersion': '16.7.2'},
};

// A response for a VS installation that's too old.
const Map<String, dynamic> _tooOldResponse = <String, dynamic>{
  'installationPath': visualStudioPath,
  'displayName': 'Visual Studio Community 2017',
  'installationVersion': '15.9.28307.665',
  'isRebootRequired': false,
  'isComplete': true,
  'isLaunchable': true,
  'isPrerelease': false,
  'catalog': <String, dynamic>{'productDisplayVersion': '15.9.12'},
};

// A version of a response that doesn't include certain installation status
// information that might be missing in older vswhere.
const Map<String, dynamic> _missingStatusResponse = <String, dynamic>{
  'installationPath': visualStudioPath,
  'displayName': 'Visual Studio Community 2017',
  'installationVersion': '16.4.29609.76',
  'catalog': <String, dynamic>{'productDisplayVersion': '16.4.1'},
};

const String _malformedDescriptionResponse = r'''
[
  {
    "installationPath": "C:\\Program Files (x86)\\Microsoft Visual Studio\\2017\\Community",
    "displayName": "Visual Studio Community 2019",
    "description": "This description has too many "quotes",
    "installationVersion": "16.2.29306.81",
    "isRebootRequired": false,
    "isComplete": true,
    "isPrerelease": false,
    "catalog": {
      "productDisplayVersion": "16.2.5"
    }
  }
]
''';

// Arguments for a vswhere query to search for an installation with the
// requirements.
const List<String> _requirements = <String>[
  'Microsoft.VisualStudio.Workload.NativeDesktop',
  'Microsoft.VisualStudio.Component.VC.Tools.x86.x64',
  'Microsoft.VisualStudio.Component.VC.CMake.Project',
];

// Arguments for a vswhere query to search for a Build Tools installation with the
// requirements.
const List<String> _requirementsBuildTools = <String>[
  'Microsoft.VisualStudio.Workload.VCTools',
  'Microsoft.VisualStudio.Component.VC.Tools.x86.x64',
  'Microsoft.VisualStudio.Component.VC.CMake.Project',
];

// Sets up the mock environment so that searching for Visual Studio with
// exactly the given required components will provide a result. By default it
// return a preset installation, but the response can be overridden.
void setMockVswhereResponse(
  FileSystem fileSystem,
  FakeProcessManager processManager, [
  List<String>? requiredComponents,
  List<String>? additionalArguments,
  Map<String, dynamic>? response,
  String? responseOverride,
  int? exitCode,
  Exception? exception,
]) {
  fileSystem.file(vswherePath).createSync(recursive: true);
  fileSystem.file(cmakePath).createSync(recursive: true);
  fileSystem.file(clPath).createSync(recursive: true);
  final String finalResponse =
      responseOverride ?? (response != null ? json.encode(<Map<String, dynamic>>[response]) : '[]');
  final List<String> requirementArguments =
      requiredComponents == null ? <String>[] : <String>['-requires', ...requiredComponents];

  processManager.addCommand(
    FakeCommand(
      command: <String>[
        vswherePath,
        '-format',
        'json',
        '-products',
        '*',
        '-utf8',
        '-latest',
        ...?additionalArguments,
        ...requirementArguments,
      ],
      stdout: finalResponse,
      exception: exception,
      exitCode: exitCode ?? 0,
    ),
  );
}

// Sets whether or not a vswhere query with the required components will
// return an installation.
void setMockCompatibleVisualStudioInstallation(
  Map<String, dynamic>? response,
  FileSystem fileSystem,
  FakeProcessManager processManager, [
  int? exitCode,
  Exception? exception,
]) {
  setMockVswhereResponse(
    fileSystem,
    processManager,
    _requirements,
    <String>['-version', '16'],
    response,
    null,
    exitCode,
    exception,
  );
}

// Sets whether or not a vswhere query with the required components will
// return a pre-release installation.
void setMockPrereleaseVisualStudioInstallation(
  Map<String, dynamic>? response,
  FileSystem fileSystem,
  FakeProcessManager processManager, [
  int? exitCode,
  Exception? exception,
]) {
  setMockVswhereResponse(
    fileSystem,
    processManager,
    _requirements,
    <String>['-version', '16', '-prerelease'],
    response,
    null,
    exitCode,
    exception,
  );
}

// Sets whether or not a vswhere query with the required components will
// return an Build Tools installation.
void setMockCompatibleVisualStudioBuildToolsInstallation(
  Map<String, dynamic>? response,
  FileSystem fileSystem,
  FakeProcessManager processManager, [
  int? exitCode,
  Exception? exception,
]) {
  setMockVswhereResponse(
    fileSystem,
    processManager,
    _requirementsBuildTools,
    <String>['-version', '16'],
    response,
    null,
    exitCode,
    exception,
  );
}

// Sets whether or not a vswhere query with the required components will
// return a pre-release Build Tools installation.
void setMockPrereleaseVisualStudioBuildToolsInstallation(
  Map<String, dynamic>? response,
  FileSystem fileSystem,
  FakeProcessManager processManager, [
  int? exitCode,
  Exception? exception,
]) {
  setMockVswhereResponse(
    fileSystem,
    processManager,
    _requirementsBuildTools,
    <String>['-version', '16', '-prerelease'],
    response,
    null,
    exitCode,
    exception,
  );
}

// Sets whether or not a vswhere query searching for 'all' and 'prerelease'
// versions will return an installation.
void setMockAnyVisualStudioInstallation(
  Map<String, dynamic>? response,
  FileSystem fileSystem,
  FakeProcessManager processManager, [
  int? exitCode,
  Exception? exception,
]) {
  setMockVswhereResponse(
    fileSystem,
    processManager,
    null,
    <String>['-prerelease', '-all'],
    response,
    null,
    exitCode,
    exception,
  );
}

// Set a pre-encoded query result.
void setMockEncodedAnyVisualStudioInstallation(
  String response,
  FileSystem fileSystem,
  FakeProcessManager processManager,
) {
  setMockVswhereResponse(
    fileSystem,
    processManager,
    null,
    <String>['-prerelease', '-all'],
    null,
    response,
  );
}

// Sets up the mock environment for a Windows 10 SDK query.
//
// registryPresent controls whether or not the registry key is found.
// filesPresent controls where or not there are any SDK folders at the location
// returned by the registry query.
void setMockSdkRegResponse(
  FileSystem fileSystem,
  FakeProcessManager processManager, {
  bool registryPresent = true,
  bool filesPresent = true,
}) {
  const String registryPath =
      r'HKEY_LOCAL_MACHINE\SOFTWARE\Wow6432Node\Microsoft\Microsoft SDKs\Windows\v10.0';
  const String registryKey = r'InstallationFolder';
  const String installationPath = r'C:\Program Files (x86)\Windows Kits\10\';
  final String stdout =
      registryPresent
          ? '''
$registryPath
    $registryKey    REG_SZ    $installationPath
'''
          : '''

ERROR: The system was unable to find the specified registry key or value.
''';

  if (filesPresent) {
    final Directory includeDirectory = fileSystem
        .directory(installationPath)
        .childDirectory('Include');
    includeDirectory.childDirectory('10.0.17763.0').createSync(recursive: true);
    includeDirectory.childDirectory('10.0.18362.0').createSync(recursive: true);
    // Not an actual version; added to ensure that version comparison is number, not string-based.
    includeDirectory.childDirectory('10.0.184.0').createSync(recursive: true);
  }

  processManager.addCommand(
    FakeCommand(
      command: const <String>['reg', 'query', registryPath, '/v', registryKey],
      stdout: stdout,
    ),
  );
}

// Create a visual studio instance with a FakeProcessManager.
VisualStudioFixture setUpVisualStudio() {
  final FakeProcessManager processManager = FakeProcessManager.empty();
  final FileSystem fileSystem = MemoryFileSystem.test(style: FileSystemStyle.windows);
  final BufferLogger logger = BufferLogger.test();
  final VisualStudio visualStudio = VisualStudio(
    fileSystem: fileSystem,
    platform: windowsPlatform,
    logger: logger,
    processManager: processManager,
    osUtils: FakeOperatingSystemUtils(),
  );
  return VisualStudioFixture(visualStudio, fileSystem, processManager, logger);
}

// Set all vswhere query with the required components return null.
void setNoViableToolchainInstallation(VisualStudioFixture fixture) {
  setMockCompatibleVisualStudioInstallation(null, fixture.fileSystem, fixture.processManager);
  setMockCompatibleVisualStudioBuildToolsInstallation(
    null,
    fixture.fileSystem,
    fixture.processManager,
  );
  setMockPrereleaseVisualStudioInstallation(null, fixture.fileSystem, fixture.processManager);
  setMockPrereleaseVisualStudioBuildToolsInstallation(
    null,
    fixture.fileSystem,
    fixture.processManager,
  );
}

void main() {
  group('Visual Studio', () {
    testWithoutContext('isInstalled throws when PROGRAMFILES(X86) env not set', () {
      final VisualStudio visualStudio = VisualStudio(
        logger: BufferLogger.test(),
        fileSystem: MemoryFileSystem.test(style: FileSystemStyle.windows),
        platform: FakePlatform(operatingSystem: 'windows'),
        processManager: FakeProcessManager.any(),
        osUtils: FakeOperatingSystemUtils(),
      );

      expect(
        () => visualStudio.isInstalled,
        throwsToolExit(message: '%PROGRAMFILES(X86)% environment variable not found'),
      );
    });

    testWithoutContext('isInstalled and cmakePath correct when vswhere is missing', () {
      final FileSystem fileSystem = MemoryFileSystem.test(style: FileSystemStyle.windows);
      const Exception exception = ProcessException('vswhere', <String>[]);
      final FakeProcessManager fakeProcessManager = FakeProcessManager.empty();

      setMockCompatibleVisualStudioInstallation(
        null,
        fileSystem,
        fakeProcessManager,
        null,
        exception,
      );
      setMockCompatibleVisualStudioBuildToolsInstallation(
        null,
        fileSystem,
        fakeProcessManager,
        null,
        exception,
      );
      setMockPrereleaseVisualStudioInstallation(
        null,
        fileSystem,
        fakeProcessManager,
        null,
        exception,
      );
      setMockPrereleaseVisualStudioBuildToolsInstallation(
        null,
        fileSystem,
        fakeProcessManager,
        null,
        exception,
      );
      setMockAnyVisualStudioInstallation(null, fileSystem, fakeProcessManager, null, exception);

      final VisualStudio visualStudio = VisualStudio(
        logger: BufferLogger.test(),
        fileSystem: fileSystem,
        platform: windowsPlatform,
        processManager: fakeProcessManager,
        osUtils: FakeOperatingSystemUtils(),
      );

      expect(visualStudio.isInstalled, false);
      expect(visualStudio.cmakePath, isNull);
    });

    testWithoutContext('isInstalled returns false when vswhere returns non-zero', () {
      final FileSystem fileSystem = MemoryFileSystem.test(style: FileSystemStyle.windows);
      final FakeProcessManager fakeProcessManager = FakeProcessManager.empty();

      setMockCompatibleVisualStudioInstallation(null, fileSystem, fakeProcessManager, 1);
      setMockCompatibleVisualStudioBuildToolsInstallation(null, fileSystem, fakeProcessManager, 1);
      setMockPrereleaseVisualStudioInstallation(null, fileSystem, fakeProcessManager, 1);
      setMockPrereleaseVisualStudioBuildToolsInstallation(null, fileSystem, fakeProcessManager, 1);
      setMockAnyVisualStudioInstallation(null, fileSystem, fakeProcessManager, 1);

      final VisualStudio visualStudio = VisualStudio(
        logger: BufferLogger.test(),
        fileSystem: fileSystem,
        platform: windowsPlatform,
        processManager: fakeProcessManager,
        osUtils: FakeOperatingSystemUtils(),
      );

      expect(visualStudio.isInstalled, false);
      expect(visualStudio.cmakePath, isNull);
    });

    testWithoutContext(
      'VisualStudio getters return the right values if no installation is found',
      () {
        final VisualStudioFixture fixture = setUpVisualStudio();
        final VisualStudio visualStudio = fixture.visualStudio;

        setNoViableToolchainInstallation(fixture);

        setMockAnyVisualStudioInstallation(null, fixture.fileSystem, fixture.processManager);

        expect(visualStudio.isInstalled, false);
        expect(visualStudio.isAtLeastMinimumVersion, false);
        expect(visualStudio.hasNecessaryComponents, false);
        expect(visualStudio.isComplete, false);
        expect(visualStudio.isRebootRequired, false);
        expect(visualStudio.isLaunchable, false);
        expect(visualStudio.displayName, null);
        expect(visualStudio.displayVersion, null);
        expect(visualStudio.installLocation, null);
        expect(visualStudio.fullVersion, null);
      },
    );

    testWithoutContext(
      'necessaryComponentDescriptions suggest the right VS tools on major version 16',
      () {
        final VisualStudioFixture fixture = setUpVisualStudio();
        final VisualStudio visualStudio = fixture.visualStudio;

        setMockCompatibleVisualStudioInstallation(
          _defaultResponse,
          fixture.fileSystem,
          fixture.processManager,
        );

        final String toolsString = visualStudio.necessaryComponentDescriptions()[0];

        expect(toolsString.contains('v142'), true);
      },
    );

    testWithoutContext(
      'necessaryComponentDescriptions suggest the right VS tools on an old version',
      () {
        final VisualStudioFixture fixture = setUpVisualStudio();
        final VisualStudio visualStudio = fixture.visualStudio;

        setNoViableToolchainInstallation(fixture);

        setMockAnyVisualStudioInstallation(
          _tooOldResponse,
          fixture.fileSystem,
          fixture.processManager,
        );

        final String toolsString = visualStudio.necessaryComponentDescriptions()[0];

        expect(toolsString.contains('v142'), true);
      },
    );

    testWithoutContext('isInstalled returns true even with missing status information', () {
      final VisualStudioFixture fixture = setUpVisualStudio();
      final VisualStudio visualStudio = fixture.visualStudio;

      setNoViableToolchainInstallation(fixture);

      setMockAnyVisualStudioInstallation(
        _missingStatusResponse,
        fixture.fileSystem,
        fixture.processManager,
      );

      expect(visualStudio.isInstalled, true);
    });

    testWithoutContext('isInstalled returns true when VS is present but missing components', () {
      final VisualStudioFixture fixture = setUpVisualStudio();
      final VisualStudio visualStudio = fixture.visualStudio;

      setNoViableToolchainInstallation(fixture);

      setMockAnyVisualStudioInstallation(
        _defaultResponse,
        fixture.fileSystem,
        fixture.processManager,
      );

      expect(visualStudio.isInstalled, true);
    });

    testWithoutContext('isInstalled returns true when VS is present but too old', () {
      final VisualStudioFixture fixture = setUpVisualStudio();
      final VisualStudio visualStudio = fixture.visualStudio;

      setNoViableToolchainInstallation(fixture);

      setMockAnyVisualStudioInstallation(
        _tooOldResponse,
        fixture.fileSystem,
        fixture.processManager,
      );

      expect(visualStudio.isInstalled, true);
    });

    testWithoutContext('isInstalled returns true when a prerelease version of VS is present', () {
      final VisualStudioFixture fixture = setUpVisualStudio();
      final VisualStudio visualStudio = fixture.visualStudio;

      final Map<String, dynamic> response = Map<String, dynamic>.of(_defaultResponse)
        ..['isPrerelease'] = true;
      setMockCompatibleVisualStudioInstallation(null, fixture.fileSystem, fixture.processManager);
      setMockCompatibleVisualStudioBuildToolsInstallation(
        null,
        fixture.fileSystem,
        fixture.processManager,
      );
      setMockPrereleaseVisualStudioInstallation(
        response,
        fixture.fileSystem,
        fixture.processManager,
      );

      expect(visualStudio.isInstalled, true);
      expect(visualStudio.isPrerelease, true);
    });

    testWithoutContext(
      'isInstalled returns true when a prerelease version of Build Tools is present',
      () {
        final VisualStudioFixture fixture = setUpVisualStudio();
        final VisualStudio visualStudio = fixture.visualStudio;

        final Map<String, dynamic> response = Map<String, dynamic>.of(_defaultBuildToolsResponse)
          ..['isPrerelease'] = true;
        setMockCompatibleVisualStudioInstallation(null, fixture.fileSystem, fixture.processManager);
        setMockCompatibleVisualStudioBuildToolsInstallation(
          null,
          fixture.fileSystem,
          fixture.processManager,
        );
        setMockPrereleaseVisualStudioInstallation(null, fixture.fileSystem, fixture.processManager);
        setMockPrereleaseVisualStudioBuildToolsInstallation(
          response,
          fixture.fileSystem,
          fixture.processManager,
        );

        expect(visualStudio.isInstalled, true);
        expect(visualStudio.isPrerelease, true);
      },
    );

    testWithoutContext(
      'isAtLeastMinimumVersion returns false when the version found is too old',
      () {
        final VisualStudioFixture fixture = setUpVisualStudio();
        final VisualStudio visualStudio = fixture.visualStudio;

        setNoViableToolchainInstallation(fixture);

        setMockAnyVisualStudioInstallation(
          _tooOldResponse,
          fixture.fileSystem,
          fixture.processManager,
        );

        expect(visualStudio.isInstalled, true);
        expect(visualStudio.isAtLeastMinimumVersion, false);
      },
    );

    testWithoutContext('isComplete returns false when an incomplete installation is found', () {
      final VisualStudioFixture fixture = setUpVisualStudio();
      final VisualStudio visualStudio = fixture.visualStudio;

      setNoViableToolchainInstallation(fixture);

      final Map<String, dynamic> response = Map<String, dynamic>.of(_defaultResponse)
        ..['isComplete'] = false;
      setMockAnyVisualStudioInstallation(response, fixture.fileSystem, fixture.processManager);

      expect(visualStudio.isInstalled, true);
      expect(visualStudio.isComplete, false);
    });

    testWithoutContext("isLaunchable returns false if the installation can't be launched", () {
      final VisualStudioFixture fixture = setUpVisualStudio();
      final VisualStudio visualStudio = fixture.visualStudio;

      setNoViableToolchainInstallation(fixture);

      final Map<String, dynamic> response = Map<String, dynamic>.of(_defaultResponse)
        ..['isLaunchable'] = false;
      setMockAnyVisualStudioInstallation(response, fixture.fileSystem, fixture.processManager);

      expect(visualStudio.isInstalled, true);
      expect(visualStudio.isLaunchable, false);
    });

    testWithoutContext('isRebootRequired returns true if the installation needs a reboot', () {
      final VisualStudioFixture fixture = setUpVisualStudio();
      final VisualStudio visualStudio = fixture.visualStudio;

      setNoViableToolchainInstallation(fixture);

      final Map<String, dynamic> response = Map<String, dynamic>.of(_defaultResponse)
        ..['isRebootRequired'] = true;
      setMockAnyVisualStudioInstallation(response, fixture.fileSystem, fixture.processManager);

      expect(visualStudio.isInstalled, true);
      expect(visualStudio.isRebootRequired, true);
    });

    testWithoutContext(
      'hasNecessaryComponents returns false when VS is present but missing components',
      () {
        final VisualStudioFixture fixture = setUpVisualStudio();
        final VisualStudio visualStudio = fixture.visualStudio;

        setNoViableToolchainInstallation(fixture);

        setMockAnyVisualStudioInstallation(
          _defaultResponse,
          fixture.fileSystem,
          fixture.processManager,
        );

        expect(visualStudio.hasNecessaryComponents, false);
      },
    );

    testWithoutContext('cmakePath returns null when VS is present but missing components', () {
      final VisualStudioFixture fixture = setUpVisualStudio();
      final VisualStudio visualStudio = fixture.visualStudio;

      setNoViableToolchainInstallation(fixture);

      setMockAnyVisualStudioInstallation(
        _defaultResponse,
        fixture.fileSystem,
        fixture.processManager,
      );

      expect(visualStudio.cmakePath, isNull);
    });

    testWithoutContext(
      'cmakePath returns null when VS is present but with require components but installation is faulty',
      () {
        final VisualStudioFixture fixture = setUpVisualStudio();
        final VisualStudio visualStudio = fixture.visualStudio;

        final Map<String, dynamic> response = Map<String, dynamic>.of(_defaultResponse)
          ..['isRebootRequired'] = true;
        setMockCompatibleVisualStudioInstallation(
          response,
          fixture.fileSystem,
          fixture.processManager,
        );

        expect(visualStudio.cmakePath, isNull);
      },
    );

    testWithoutContext(
      'hasNecessaryComponents returns false when VS is present with required components but installation is faulty',
      () {
        final VisualStudioFixture fixture = setUpVisualStudio();
        final VisualStudio visualStudio = fixture.visualStudio;

        final Map<String, dynamic> response = Map<String, dynamic>.of(_defaultResponse)
          ..['isRebootRequired'] = true;
        setMockCompatibleVisualStudioInstallation(
          response,
          fixture.fileSystem,
          fixture.processManager,
        );

        expect(visualStudio.hasNecessaryComponents, false);
      },
    );

    testWithoutContext(
      'VS metadata is available when VS is present, even if missing components',
      () {
        final VisualStudioFixture fixture = setUpVisualStudio();
        final VisualStudio visualStudio = fixture.visualStudio;

        setNoViableToolchainInstallation(fixture);

        setMockAnyVisualStudioInstallation(
          _defaultResponse,
          fixture.fileSystem,
          fixture.processManager,
        );

        expect(visualStudio.displayName, equals('Visual Studio Community 2019'));
        expect(visualStudio.displayVersion, equals('16.2.5'));
        expect(visualStudio.installLocation, equals(visualStudioPath));
        expect(visualStudio.fullVersion, equals('16.2.29306.81'));
      },
    );

    testWithoutContext(
      'Warns and returns no installation when VS is present but vswhere returns invalid JSON',
      () {
        final VisualStudioFixture fixture = setUpVisualStudio();
        final VisualStudio visualStudio = fixture.visualStudio;

        setNoViableToolchainInstallation(fixture);

        setMockEncodedAnyVisualStudioInstallation('{', fixture.fileSystem, fixture.processManager);

        expect(visualStudio.isInstalled, isFalse);
        expect(visualStudio.isComplete, isFalse);
        expect(visualStudio.isLaunchable, isFalse);
        expect(visualStudio.isPrerelease, isFalse);
        expect(visualStudio.isRebootRequired, isFalse);
        expect(visualStudio.hasNecessaryComponents, isFalse);
        expect(visualStudio.displayName, isNull);
        expect(visualStudio.displayVersion, isNull);
        expect(visualStudio.installLocation, isNull);
        expect(visualStudio.fullVersion, isNull);
        expect(visualStudio.cmakePath, isNull);

        expect(fixture.logger.warningText, contains('Warning: Unexpected vswhere.exe JSON output'));
      },
    );

    testWithoutContext('Everything returns good values when VS is present with all components', () {
      final VisualStudioFixture fixture = setUpVisualStudio();
      final VisualStudio visualStudio = fixture.visualStudio;

      setMockCompatibleVisualStudioInstallation(
        _defaultResponse,
        fixture.fileSystem,
        fixture.processManager,
      );

      expect(visualStudio.isInstalled, true);
      expect(visualStudio.isAtLeastMinimumVersion, true);
      expect(visualStudio.hasNecessaryComponents, true);
      expect(visualStudio.cmakePath, equals(cmakePath));
      expect(visualStudio.cmakeGenerator, equals('Visual Studio 16 2019'));
      expect(visualStudio.clPath, equals(clPath));
      expect(visualStudio.libPath, equals(libPath));
      expect(visualStudio.linkPath, equals(linkPath));
      expect(visualStudio.vcvarsPath, equals(vcvarsPath));
    });

    testWithoutContext(
      'Everything returns good values when Build Tools is present with all components',
      () {
        final VisualStudioFixture fixture = setUpVisualStudio();
        final VisualStudio visualStudio = fixture.visualStudio;

        setMockCompatibleVisualStudioInstallation(null, fixture.fileSystem, fixture.processManager);
        setMockCompatibleVisualStudioBuildToolsInstallation(
          _defaultBuildToolsResponse,
          fixture.fileSystem,
          fixture.processManager,
        );

        expect(visualStudio.isInstalled, true);
        expect(visualStudio.isAtLeastMinimumVersion, true);
        expect(visualStudio.hasNecessaryComponents, true);
        expect(visualStudio.cmakePath, equals(cmakePath));
      },
    );

    testWithoutContext('properties return the right value for Visual Studio 2022', () {
      final VisualStudioFixture fixture = setUpVisualStudio();
      final VisualStudio visualStudio = fixture.visualStudio;

      setMockCompatibleVisualStudioInstallation(
        _vs2022Response,
        fixture.fileSystem,
        fixture.processManager,
      );

      expect(visualStudio.isInstalled, true);
      expect(visualStudio.isAtLeastMinimumVersion, true);
      expect(visualStudio.hasNecessaryComponents, true);
      expect(visualStudio.cmakePath, equals(cmakePath));
      expect(visualStudio.cmakeGenerator, equals('Visual Studio 17 2022'));
      expect(visualStudio.clPath, equals(clPath));
      expect(visualStudio.libPath, equals(libPath));
      expect(visualStudio.linkPath, equals(linkPath));
      expect(visualStudio.vcvarsPath, equals(vcvarsPath));
    });

    testWithoutContext('Metadata is for compatible version when latest is missing components', () {
      final VisualStudioFixture fixture = setUpVisualStudio();
      final VisualStudio visualStudio = fixture.visualStudio;

      // Return a different version for queries without the required packages.
      final Map<String, dynamic> olderButCompleteVersionResponse = <String, dynamic>{
        'installationPath': visualStudioPath,
        'displayName': 'Visual Studio Community 2017',
        'installationVersion': '15.9.28307.665',
        'catalog': <String, dynamic>{'productDisplayVersion': '15.9.12'},
      };

      setMockCompatibleVisualStudioInstallation(
        olderButCompleteVersionResponse,
        fixture.fileSystem,
        fixture.processManager,
      );
      // Return a different version for queries without the required packages.
      final Map<String, dynamic> incompleteVersionResponse = <String, dynamic>{
        'installationPath': visualStudioPath,
        'displayName': 'Visual Studio Community 2019',
        'installationVersion': '16.1.1.1',
        'catalog': <String, String>{'productDisplayVersion': '16.1'},
      };
      setMockAnyVisualStudioInstallation(
        incompleteVersionResponse,
        fixture.fileSystem,
        fixture.processManager,
      );

      expect(visualStudio.displayName, equals('Visual Studio Community 2017'));
      expect(visualStudio.displayVersion, equals('15.9.12'));
    });

    testWithoutContext('SDK version returns the latest version when present', () {
      final VisualStudioFixture fixture = setUpVisualStudio();
      final VisualStudio visualStudio = fixture.visualStudio;

      setMockSdkRegResponse(fixture.fileSystem, fixture.processManager);

      expect(visualStudio.getWindows10SDKVersion(), '10.0.18362.0');
    });

    testWithoutContext('SDK version returns null when the registry key is not present', () {
      final VisualStudioFixture fixture = setUpVisualStudio();
      final VisualStudio visualStudio = fixture.visualStudio;

      setMockSdkRegResponse(fixture.fileSystem, fixture.processManager, registryPresent: false);

      expect(visualStudio.getWindows10SDKVersion(), null);
    });

    testWithoutContext('SDK version returns null when there are no SDK files present', () {
      final VisualStudioFixture fixture = setUpVisualStudio();
      final VisualStudio visualStudio = fixture.visualStudio;

      setMockSdkRegResponse(fixture.fileSystem, fixture.processManager, filesPresent: false);

      expect(visualStudio.getWindows10SDKVersion(), null);
    });
  });

  // The output of vswhere.exe is known to contain bad UTF8.
  // See: https://github.com/flutter/flutter/issues/102451
  group('Correctly handles bad UTF-8 from vswhere.exe output', () {
    late VisualStudioFixture fixture;
    late VisualStudio visualStudio;

    setUp(() {
      fixture = setUpVisualStudio();
      visualStudio = fixture.visualStudio;
    });

    testWithoutContext('Ignores unicode replacement char in unused properties', () {
      final Map<String, dynamic> response = Map<String, dynamic>.of(_defaultResponse)
        ..['unused'] = 'Bad UTF8 \u{FFFD}';

      setMockCompatibleVisualStudioInstallation(
        response,
        fixture.fileSystem,
        fixture.processManager,
      );

      expect(visualStudio.isInstalled, true);
      expect(visualStudio.isAtLeastMinimumVersion, true);
      expect(visualStudio.hasNecessaryComponents, true);
      expect(visualStudio.cmakePath, equals(cmakePath));
      expect(visualStudio.cmakeGenerator, equals('Visual Studio 16 2019'));
      expect(visualStudio.clPath, equals(clPath));
      expect(visualStudio.libPath, equals(libPath));
      expect(visualStudio.linkPath, equals(linkPath));
      expect(visualStudio.vcvarsPath, equals(vcvarsPath));
    });

    testWithoutContext('Throws ToolExit on bad UTF-8 in installationPath', () {
      final Map<String, dynamic> response = Map<String, dynamic>.of(_defaultResponse)
        ..['installationPath'] = '\u{FFFD}';

      setMockCompatibleVisualStudioInstallation(
        response,
        fixture.fileSystem,
        fixture.processManager,
      );

      expect(
        () => visualStudio.isInstalled,
        throwsToolExit(
          message: 'Bad UTF-8 encoding (U+FFFD; REPLACEMENT CHARACTER) found in string',
        ),
      );
    });

    testWithoutContext('Throws ToolExit on bad UTF-8 in installationVersion', () {
      final Map<String, dynamic> response = Map<String, dynamic>.of(_defaultResponse)
        ..['installationVersion'] = '\u{FFFD}';

      setMockCompatibleVisualStudioInstallation(
        response,
        fixture.fileSystem,
        fixture.processManager,
      );

      expect(
        () => visualStudio.isInstalled,
        throwsToolExit(
          message: 'Bad UTF-8 encoding (U+FFFD; REPLACEMENT CHARACTER) found in string',
        ),
      );
    });

    testWithoutContext('Ignores bad UTF-8 in displayName', () {
      final Map<String, dynamic> response = Map<String, dynamic>.of(_defaultResponse)
        ..['displayName'] = '\u{FFFD}';

      setMockCompatibleVisualStudioInstallation(
        response,
        fixture.fileSystem,
        fixture.processManager,
      );

      expect(visualStudio.isInstalled, true);
      expect(visualStudio.isAtLeastMinimumVersion, true);
      expect(visualStudio.hasNecessaryComponents, true);
      expect(visualStudio.cmakePath, equals(cmakePath));
      expect(visualStudio.cmakeGenerator, equals('Visual Studio 16 2019'));
      expect(visualStudio.displayName, equals('\u{FFFD}'));
      expect(visualStudio.clPath, equals(clPath));
      expect(visualStudio.libPath, equals(libPath));
      expect(visualStudio.linkPath, equals(linkPath));
      expect(visualStudio.vcvarsPath, equals(vcvarsPath));
    });

    testWithoutContext("Ignores bad UTF-8 in catalog's productDisplayVersion", () {
      final Map<String, dynamic> catalog = Map<String, dynamic>.of(
        _defaultResponse['catalog'] as Map<String, dynamic>,
      )..['productDisplayVersion'] = '\u{FFFD}';
      final Map<String, dynamic> response = Map<String, dynamic>.of(_defaultResponse)
        ..['catalog'] = catalog;

      setMockCompatibleVisualStudioInstallation(
        response,
        fixture.fileSystem,
        fixture.processManager,
      );

      expect(visualStudio.isInstalled, true);
      expect(visualStudio.isAtLeastMinimumVersion, true);
      expect(visualStudio.hasNecessaryComponents, true);
      expect(visualStudio.cmakePath, equals(cmakePath));
      expect(visualStudio.cmakeGenerator, equals('Visual Studio 16 2019'));
      expect(visualStudio.displayVersion, equals('\u{FFFD}'));
      expect(visualStudio.clPath, equals(clPath));
      expect(visualStudio.libPath, equals(libPath));
      expect(visualStudio.linkPath, equals(linkPath));
      expect(visualStudio.vcvarsPath, equals(vcvarsPath));
    });

    testWithoutContext('Ignores malformed JSON in description property', () {
      setMockVswhereResponse(
        fixture.fileSystem,
        fixture.processManager,
        _requirements,
        <String>['-version', '16'],
        null,
        _malformedDescriptionResponse,
      );

      expect(visualStudio.isInstalled, true);
      expect(visualStudio.isAtLeastMinimumVersion, true);
      expect(visualStudio.hasNecessaryComponents, true);
      expect(visualStudio.cmakePath, equals(cmakePath));
      expect(visualStudio.cmakeGenerator, equals('Visual Studio 16 2019'));
      expect(visualStudio.displayVersion, equals('16.2.5'));
      expect(visualStudio.clPath, equals(clPath));
      expect(visualStudio.libPath, equals(libPath));
      expect(visualStudio.linkPath, equals(linkPath));
      expect(visualStudio.vcvarsPath, equals(vcvarsPath));

      expect(fixture.logger.warningText, isEmpty);
    });
  });

  group(VswhereDetails, () {
    test('Accepts empty JSON', () {
      const bool meetsRequirements = true;
      final Map<String, dynamic> json = <String, dynamic>{};
      const String msvcVersion = '';

      final VswhereDetails result = VswhereDetails.fromJson(meetsRequirements, json, msvcVersion);

      expect(result.installationPath, null);
      expect(result.displayName, null);
      expect(result.fullVersion, null);
      expect(result.isComplete, null);
      expect(result.isLaunchable, null);
      expect(result.isRebootRequired, null);
      expect(result.isPrerelease, null);
      expect(result.catalogDisplayVersion, null);
      expect(result.isUsable, isTrue);
    });

    test('Ignores unknown JSON properties', () {
      const bool meetsRequirements = true;
      final Map<String, dynamic> json = <String, dynamic>{'hello': 'world'};
      const String msvcVersion = '';

      final VswhereDetails result = VswhereDetails.fromJson(meetsRequirements, json, msvcVersion);

      expect(result.installationPath, null);
      expect(result.displayName, null);
      expect(result.fullVersion, null);
      expect(result.isComplete, null);
      expect(result.isLaunchable, null);
      expect(result.isRebootRequired, null);
      expect(result.isPrerelease, null);
      expect(result.catalogDisplayVersion, null);
      expect(result.isUsable, isTrue);
    });

    test('Accepts JSON', () {
      const bool meetsRequirements = true;
      const String msvcVersion = '';

      final VswhereDetails result = VswhereDetails.fromJson(
        meetsRequirements,
        _defaultResponse,
        msvcVersion,
      );

      expect(result.installationPath, visualStudioPath);
      expect(result.displayName, 'Visual Studio Community 2019');
      expect(result.fullVersion, '16.2.29306.81');
      expect(result.isComplete, true);
      expect(result.isLaunchable, true);
      expect(result.isRebootRequired, false);
      expect(result.isPrerelease, false);
      expect(result.catalogDisplayVersion, '16.2.5');
      expect(result.isUsable, isTrue);
    });

    test('Installation that does not satisfy requirements is not usable', () {
      const bool meetsRequirements = false;
      const String msvcVersion = '';

      final VswhereDetails result = VswhereDetails.fromJson(
        meetsRequirements,
        _defaultResponse,
        msvcVersion,
      );

      expect(result.isUsable, isFalse);
    });

    test('Incomplete installation is not usable', () {
      const bool meetsRequirements = true;
      final Map<String, dynamic> json = Map<String, dynamic>.of(_defaultResponse)
        ..['isComplete'] = false;
      const String msvcVersion = '';

      final VswhereDetails result = VswhereDetails.fromJson(meetsRequirements, json, msvcVersion);

      expect(result.isUsable, isFalse);
    });

    test('Unlaunchable installation is not usable', () {
      const bool meetsRequirements = true;
      final Map<String, dynamic> json = Map<String, dynamic>.of(_defaultResponse)
        ..['isLaunchable'] = false;
      const String msvcVersion = '';

      final VswhereDetails result = VswhereDetails.fromJson(meetsRequirements, json, msvcVersion);

      expect(result.isUsable, isFalse);
    });

    test('Installation that requires reboot is not usable', () {
      const bool meetsRequirements = true;
      final Map<String, dynamic> json = Map<String, dynamic>.of(_defaultResponse)
        ..['isRebootRequired'] = true;
      const String msvcVersion = '';

      final VswhereDetails result = VswhereDetails.fromJson(meetsRequirements, json, msvcVersion);

      expect(result.isUsable, isFalse);
    });
  });
}

class VisualStudioFixture {
  VisualStudioFixture(this.visualStudio, this.fileSystem, this.processManager, this.logger);

  final VisualStudio visualStudio;
  final FileSystem fileSystem;
  final FakeProcessManager processManager;
  final BufferLogger logger;
}
