// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';
import 'package:process/process.dart';

import '../base/common.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/os.dart' show HostPlatform, OperatingSystemUtils;
import '../base/platform.dart';
import '../base/process.dart';
import '../base/version.dart';
import '../convert.dart';

/// Encapsulates information about the installed copy of Visual Studio, if any.
class VisualStudio {
  VisualStudio({
    required FileSystem fileSystem,
    required ProcessManager processManager,
    required Platform platform,
    required Logger logger,
    required OperatingSystemUtils osUtils,
  }) : _platform = platform,
       _fileSystem = fileSystem,
       _processUtils = ProcessUtils(processManager: processManager, logger: logger),
       _logger = logger,
       _osUtils = osUtils;

  final FileSystem _fileSystem;
  final Platform _platform;
  final ProcessUtils _processUtils;
  final Logger _logger;
  final OperatingSystemUtils _osUtils;

  /// Matches the description property from the vswhere.exe JSON output.
  final RegExp _vswhereDescriptionProperty = RegExp(r'\s*"description"\s*:\s*".*"\s*,?');

  /// True if Visual Studio installation was found.
  ///
  /// Versions older than 2017 Update 2 won't be detected, so error messages to
  /// users should take into account that [false] may mean that the user may
  /// have an old version rather than no installation at all.
  bool get isInstalled => _bestVisualStudioDetails != null;

  bool get isAtLeastMinimumVersion {
    final int? installedMajorVersion = _majorVersion;
    return installedMajorVersion != null && installedMajorVersion >= _minimumSupportedVersion;
  }

  /// True if there is a version of Visual Studio with all the components
  /// necessary to build the project.
  bool get hasNecessaryComponents => _bestVisualStudioDetails?.isUsable ?? false;

  /// The name of the Visual Studio install.
  ///
  /// For instance: "Visual Studio Community 2019". This should only be used for
  /// display purposes.
  String? get displayName => _bestVisualStudioDetails?.displayName;

  /// The user-friendly version number of the Visual Studio install.
  ///
  /// For instance: "15.4.0". This should only be used for display purposes.
  /// Logic based off the installation's version should use the `fullVersion`.
  String? get displayVersion => _bestVisualStudioDetails?.catalogDisplayVersion;

  /// The directory where Visual Studio is installed.
  String? get installLocation => _bestVisualStudioDetails?.installationPath;

  /// The full version of the Visual Studio install.
  ///
  /// For instance: "15.4.27004.2002".
  String? get fullVersion => _bestVisualStudioDetails?.fullVersion;

  // Properties that determine the status of the installation. There might be
  // Visual Studio versions that don't include them, so default to a "valid" value to
  // avoid false negatives.

  /// True if there is a complete installation of Visual Studio.
  ///
  /// False if installation is not found.
  bool get isComplete {
    if (_bestVisualStudioDetails == null) {
      return false;
    }
    return _bestVisualStudioDetails.isComplete ?? true;
  }

  /// True if Visual Studio is launchable.
  ///
  /// False if installation is not found.
  bool get isLaunchable {
    if (_bestVisualStudioDetails == null) {
      return false;
    }
    return _bestVisualStudioDetails.isLaunchable ?? true;
  }

  /// True if the Visual Studio installation is a pre-release version.
  bool get isPrerelease => _bestVisualStudioDetails?.isPrerelease ?? false;

  /// True if a reboot is required to complete the Visual Studio installation.
  bool get isRebootRequired => _bestVisualStudioDetails?.isRebootRequired ?? false;

  /// The name of the recommended Visual Studio installer workload.
  String get workloadDescription => 'Desktop development with C++';

  /// Returns the highest installed Windows 10 SDK version, or null if none is
  /// found.
  ///
  /// For instance: 10.0.18362.0.
  String? getWindows10SDKVersion() {
    final String? sdkLocation = _getWindows10SdkLocation();
    if (sdkLocation == null) {
      return null;
    }
    final Directory sdkIncludeDirectory = _fileSystem
        .directory(sdkLocation)
        .childDirectory('Include');
    if (!sdkIncludeDirectory.existsSync()) {
      return null;
    }
    // The directories in this folder are named by the SDK version.
    Version? highestVersion;
    for (final FileSystemEntity versionEntry in sdkIncludeDirectory.listSync()) {
      if (versionEntry.basename.startsWith('10.')) {
        // Version only handles 3 components; strip off the '10.' to leave three
        // components, since they all start with that.
        final Version? version = Version.parse(versionEntry.basename.substring(3));
        if (highestVersion == null || (version != null && version > highestVersion)) {
          highestVersion = version;
        }
      }
    }
    if (highestVersion == null) {
      return null;
    }
    return '10.$highestVersion';
  }

  /// The names of the components within the workload that must be installed.
  ///
  /// The descriptions of some components differ from version to version. When
  /// a supported version is present, the descriptions used will be for that
  /// version.
  List<String> necessaryComponentDescriptions() {
    return _requiredComponents().values.toList();
  }

  /// The consumer-facing version name of the minimum supported version.
  ///
  /// E.g., for Visual Studio 2019 this returns "2019" rather than "16".
  String get minimumVersionDescription {
    return '2019';
  }

  /// The path to CMake, or null if no Visual Studio installation has
  /// the components necessary to build.
  String? get cmakePath {
    final VswhereDetails? details = _bestVisualStudioDetails;
    if (details == null || !details.isUsable || details.installationPath == null) {
      return null;
    }

    return _fileSystem.path.joinAll(<String>[
      details.installationPath!,
      'Common7',
      'IDE',
      'CommonExtensions',
      'Microsoft',
      'CMake',
      'CMake',
      'bin',
      'cmake.exe',
    ]);
  }

  /// The generator string to pass to CMake to select this Visual Studio
  /// version.
  String? get cmakeGenerator {
    // From https://cmake.org/cmake/help/v3.22/manual/cmake-generators.7.html#visual-studio-generators
    return switch (_majorVersion) {
      17 => 'Visual Studio 17 2022',
      _ => 'Visual Studio 16 2019',
    };
  }

  /// The path to cl.exe, or null if no Visual Studio installation has
  /// the components necessary to build.
  String? get clPath {
    return _getMsvcBinPath('cl.exe');
  }

  /// The path to lib.exe, or null if no Visual Studio installation has
  /// the components necessary to build.
  String? get libPath {
    return _getMsvcBinPath('lib.exe');
  }

  /// The path to link.exe, or null if no Visual Studio installation has
  /// the components necessary to build.
  String? get linkPath {
    return _getMsvcBinPath('link.exe');
  }

  String? _getMsvcBinPath(String executable) {
    final VswhereDetails? details = _bestVisualStudioDetails;
    if (details == null ||
        !details.isUsable ||
        details.installationPath == null ||
        details.msvcVersion == null) {
      return null;
    }

    final String arch = _osUtils.hostPlatform == HostPlatform.windows_arm64 ? 'arm64' : 'x64';

    return _fileSystem.path.joinAll(<String>[
      details.installationPath!,
      'VC',
      'Tools',
      'MSVC',
      details.msvcVersion!,
      'bin',
      'Host$arch',
      arch,
      executable,
    ]);
  }

  /// The path to vcvars64.exe, or null if no Visual Studio installation has
  /// the components necessary to build.
  String? get vcvarsPath {
    final VswhereDetails? details = _bestVisualStudioDetails;
    if (details == null || !details.isUsable || details.installationPath == null) {
      return null;
    }

    final String arch = _osUtils.hostPlatform == HostPlatform.windows_arm64 ? 'arm64' : '64';

    return _fileSystem.path.joinAll(<String>[
      details.installationPath!,
      'VC',
      'Auxiliary',
      'Build',
      'vcvars$arch.bat',
    ]);
  }

  /// The major version of the Visual Studio install, as an integer.
  int? get _majorVersion => fullVersion != null ? int.tryParse(fullVersion!.split('.')[0]) : null;

  /// The path to vswhere.exe.
  ///
  /// vswhere should be installed for VS 2017 Update 2 and later; if it's not
  /// present then there isn't a new enough installation of VS. This path is
  /// not user-controllable, unlike the install location of Visual Studio
  /// itself.
  String get _vswherePath {
    const String programFilesEnv = 'PROGRAMFILES(X86)';
    if (!_platform.environment.containsKey(programFilesEnv)) {
      throwToolExit('%$programFilesEnv% environment variable not found.');
    }
    return _fileSystem.path.join(
      _platform.environment[programFilesEnv]!,
      'Microsoft Visual Studio',
      'Installer',
      'vswhere.exe',
    );
  }

  /// Workload ID for use with vswhere requirements.
  ///
  /// Workload ID is different between Visual Studio IDE and Build Tools.
  /// See https://docs.microsoft.com/en-us/visualstudio/install/workload-and-component-ids
  static const List<String> _requiredWorkloads = <String>[
    'Microsoft.VisualStudio.Workload.NativeDesktop',
    'Microsoft.VisualStudio.Workload.VCTools',
  ];

  /// Components for use with vswhere requirements.
  ///
  /// Maps from component IDs to description in the installer UI.
  /// See https://docs.microsoft.com/en-us/visualstudio/install/workload-and-component-ids
  Map<String, String> _requiredComponents([int? majorVersion]) {
    // The description of the C++ toolchain required by the template. The
    // component name is significantly different in different versions.
    // When a new major version of VS is supported, its toolchain description
    // should be added below. It should also be made the default, so that when
    // there is no installation, the message shows the string that will be
    // relevant for the most likely fresh install case).
    String cppToolchainDescription;
    switch (majorVersion ?? _majorVersion) {
      case 16:
      default:
        cppToolchainDescription = 'MSVC v142 - VS 2019 C++ x64/x86 build tools';
    }
    // The 'Microsoft.VisualStudio.Component.VC.Tools.x86.x64' ID is assigned to the latest
    // release of the toolchain, and there can be minor updates within a given version of
    // Visual Studio. Since it changes over time, listing a precise version would become
    // wrong after each VC++ toolchain update, so just instruct people to install the
    // latest version.
    cppToolchainDescription +=
        '\n   - If there are multiple build tool versions available, install the latest';
    // Things which are required by the workload (e.g., MSBuild) don't need to
    // be included here.
    return <String, String>{
      // The C++ toolchain required by the template.
      'Microsoft.VisualStudio.Component.VC.Tools.x86.x64': cppToolchainDescription,
      // CMake
      'Microsoft.VisualStudio.Component.VC.CMake.Project': 'C++ CMake tools for Windows',
    };
  }

  /// The minimum supported major version.
  static const int _minimumSupportedVersion = 16; // '16' is VS 2019.

  /// vswhere argument to specify the minimum version.
  static const String _vswhereMinVersionArgument = '-version';

  /// vswhere argument to allow prerelease versions.
  static const String _vswherePrereleaseArgument = '-prerelease';

  /// The registry path for Windows 10 SDK installation details.
  static const String _windows10SdkRegistryPath =
      r'HKEY_LOCAL_MACHINE\SOFTWARE\Wow6432Node\Microsoft\Microsoft SDKs\Windows\v10.0';

  /// The registry key in _windows10SdkRegistryPath for the folder where the
  /// SDKs are installed.
  static const String _windows10SdkRegistryKey = 'InstallationFolder';

  /// Returns the details of the newest version of Visual Studio.
  ///
  /// If [validateRequirements] is set, the search will be limited to versions
  /// that have all of the required workloads and components.
  VswhereDetails? _visualStudioDetails({
    bool validateRequirements = false,
    List<String>? additionalArguments,
    String? requiredWorkload,
  }) {
    final List<String> requirementArguments =
        validateRequirements
            ? <String>[
              if (requiredWorkload != null) ...<String>['-requires', requiredWorkload],
              ..._requiredComponents(_minimumSupportedVersion).keys,
            ]
            : <String>[];
    try {
      final List<String> defaultArguments = <String>[
        '-format',
        'json',
        '-products',
        '*',
        '-utf8',
        '-latest',
      ];
      // Ignore replacement characters as vswhere.exe is known to output them.
      // See: https://github.com/flutter/flutter/issues/102451
      const Encoding encoding = Utf8Codec(reportErrors: false);
      final RunResult whereResult = _processUtils.runSync(<String>[
        _vswherePath,
        ...defaultArguments,
        ...?additionalArguments,
        ...requirementArguments,
      ], encoding: encoding);
      if (whereResult.exitCode == 0) {
        final List<Map<String, dynamic>>? installations = _tryDecodeVswhereJson(whereResult.stdout);
        if (installations != null && installations.isNotEmpty) {
          final String? msvcVersion = _findMsvcVersion(installations);
          return VswhereDetails.fromJson(validateRequirements, installations[0], msvcVersion);
        }
      }
    } on ArgumentError {
      // Thrown if vswhere doesn't exist; ignore and return null below.
    } on ProcessException {
      // Ignored, return null below.
    }
    return null;
  }

  String? _findMsvcVersion(List<Map<String, dynamic>> installations) {
    final String? installationPath = installations[0]['installationPath'] as String?;
    String? msvcVersion;
    if (installationPath != null) {
      final Directory installationDir = _fileSystem.directory(installationPath);
      final Directory msvcDir = installationDir
          .childDirectory('VC')
          .childDirectory('Tools')
          .childDirectory('MSVC');
      if (msvcDir.existsSync()) {
        final Iterable<Directory> msvcVersionDirs = msvcDir.listSync().whereType<Directory>();
        if (msvcVersionDirs.isEmpty) {
          return null;
        }
        msvcVersion = msvcVersionDirs.last.uri.pathSegments.where((String e) => e.isNotEmpty).last;
      }
    }
    return msvcVersion;
  }

  List<Map<String, dynamic>>? _tryDecodeVswhereJson(String vswhereJson) {
    List<dynamic>? result;
    FormatException? originalError;
    try {
      // Some versions of vswhere.exe are known to encode their output incorrectly,
      // resulting in invalid JSON in the 'description' property when interpreted
      // as UTF-8. First, try to decode without any pre-processing.
      try {
        result = json.decode(vswhereJson) as List<dynamic>;
      } on FormatException catch (error) {
        // If that fails, remove the 'description' property and try again.
        // See: https://github.com/flutter/flutter/issues/106601
        vswhereJson = vswhereJson.replaceFirst(_vswhereDescriptionProperty, '');

        _logger.printTrace(
          'Failed to decode vswhere.exe JSON output. $error'
          'Retrying after removing the unused description property:\n$vswhereJson',
        );

        originalError = error;
        result = json.decode(vswhereJson) as List<dynamic>;
      }
    } on FormatException {
      // Removing the description property didn't help.
      // Report the original decoding error on the unprocessed JSON.
      _logger.printWarning(
        'Warning: Unexpected vswhere.exe JSON output. $originalError'
        'To see the full JSON, run flutter doctor -vv.',
      );
      return null;
    }

    return result.cast<Map<String, dynamic>>();
  }

  /// Returns the details of the best available version of Visual Studio.
  ///
  /// If there's a version that has all the required components, that
  /// will be returned, otherwise returns the latest installed version regardless
  /// of components and version, or null if no such installation is found.
  late final VswhereDetails? _bestVisualStudioDetails = () {
    // First, attempt to find the latest version of Visual Studio that satisfies
    // both the minimum supported version and the required workloads.
    // Check in the order of stable VS, stable BT, pre-release VS, pre-release BT.
    final List<String> minimumVersionArguments = <String>[
      _vswhereMinVersionArgument,
      _minimumSupportedVersion.toString(),
    ];
    for (final bool checkForPrerelease in <bool>[false, true]) {
      for (final String requiredWorkload in _requiredWorkloads) {
        final VswhereDetails? result = _visualStudioDetails(
          validateRequirements: true,
          additionalArguments:
              checkForPrerelease
                  ? <String>[...minimumVersionArguments, _vswherePrereleaseArgument]
                  : minimumVersionArguments,
          requiredWorkload: requiredWorkload,
        );

        if (result != null) {
          return result;
        }
      }
    }

    // An installation that satisfies requirements could not be found.
    // Fallback to the latest Visual Studio installation.
    return _visualStudioDetails(additionalArguments: <String>[_vswherePrereleaseArgument, '-all']);
  }();

  /// Returns the installation location of the Windows 10 SDKs, or null if the
  /// registry doesn't contain that information.
  String? _getWindows10SdkLocation() {
    try {
      final RunResult result = _processUtils.runSync(<String>[
        'reg',
        'query',
        _windows10SdkRegistryPath,
        '/v',
        _windows10SdkRegistryKey,
      ]);
      if (result.exitCode == 0) {
        final RegExp pattern = RegExp(r'InstallationFolder\s+REG_SZ\s+(.+)');
        final RegExpMatch? match = pattern.firstMatch(result.stdout);
        if (match != null) {
          return match.group(1)!.trim();
        }
      }
    } on ArgumentError {
      // Thrown if reg somehow doesn't exist; ignore and return null below.
    } on ProcessException {
      // Ignored, return null below.
    }
    return null;
  }

  /// Returns the highest-numbered SDK version in [dir], which should be the
  /// Windows 10 SDK installation directory.
  ///
  /// Returns null if no Windows 10 SDKs are found.
  String? findHighestVersionInSdkDirectory(Directory dir) {
    // This contains subfolders that are named by the SDK version.
    final Directory includeDir = dir.childDirectory('Includes');
    if (!includeDir.existsSync()) {
      return null;
    }
    Version? highestVersion;
    for (final FileSystemEntity versionEntry in includeDir.listSync()) {
      if (!versionEntry.basename.startsWith('10.')) {
        continue;
      }
      // Version only handles 3 components; strip off the '10.' to leave three
      // components, since they all start with that.
      final Version? version = Version.parse(versionEntry.basename.substring(3));
      if (highestVersion == null || (version != null && version > highestVersion)) {
        highestVersion = version;
      }
    }
    // Re-add the leading '10.' that was removed for comparison.
    return highestVersion == null ? null : '10.$highestVersion';
  }
}

/// The details of a Visual Studio installation according to vswhere.
@visibleForTesting
class VswhereDetails {
  const VswhereDetails({
    required this.meetsRequirements,
    required this.installationPath,
    required this.displayName,
    required this.fullVersion,
    required this.isComplete,
    required this.isLaunchable,
    required this.isRebootRequired,
    required this.isPrerelease,
    required this.catalogDisplayVersion,
    required this.msvcVersion,
  });

  /// Create a `VswhereDetails` from the JSON output of vswhere.exe.
  factory VswhereDetails.fromJson(
    bool meetsRequirements,
    Map<String, dynamic> details,
    String? msvcVersion,
  ) {
    final Map<String, dynamic>? catalog = details['catalog'] as Map<String, dynamic>?;

    return VswhereDetails(
      meetsRequirements: meetsRequirements,
      isComplete: details['isComplete'] as bool?,
      isLaunchable: details['isLaunchable'] as bool?,
      isRebootRequired: details['isRebootRequired'] as bool?,
      isPrerelease: details['isPrerelease'] as bool?,

      // Below are strings that must be well-formed without replacement characters.
      installationPath: _validateString(details['installationPath'] as String?),
      fullVersion: _validateString(details['installationVersion'] as String?),

      // Below are strings that are used only for display purposes and are allowed to
      // contain replacement characters.
      displayName: details['displayName'] as String?,
      catalogDisplayVersion: catalog == null ? null : catalog['productDisplayVersion'] as String?,

      msvcVersion: msvcVersion,
    );
  }

  /// Verify JSON strings from vswhere.exe output are valid.
  ///
  /// The output of vswhere.exe is known to output replacement characters.
  /// Use this to ensure values that must be well-formed are valid. Strings that
  /// are only used for display purposes should skip this check.
  /// See: https://github.com/flutter/flutter/issues/102451
  static String? _validateString(String? value) {
    if (value != null && value.contains('\u{FFFD}')) {
      throwToolExit(
        'Bad UTF-8 encoding (U+FFFD; REPLACEMENT CHARACTER) found in string: $value. '
        'The Flutter team would greatly appreciate if you could file a bug explaining '
        'exactly what you were doing when this happened:\n'
        'https://github.com/flutter/flutter/issues/new/choose\n',
      );
    }

    return value;
  }

  /// Whether the installation satisfies the required workloads and minimum version.
  final bool meetsRequirements;

  /// The root directory of the Visual Studio installation.
  final String? installationPath;

  /// The user-friendly name of the installation.
  final String? displayName;

  /// The complete version.
  final String? fullVersion;

  /// Keys for the status of the installation.
  final bool? isComplete;
  final bool? isLaunchable;
  final bool? isRebootRequired;

  /// The key for a pre-release version.
  final bool? isPrerelease;

  /// The user-friendly version.
  final String? catalogDisplayVersion;

  /// The MSVC versions.
  final String? msvcVersion;

  /// Checks if the Visual Studio installation can be used by Flutter.
  ///
  /// Returns false if the installation has issues the user must resolve.
  /// This may return true even if required information is missing as older
  /// versions of Visual Studio might not include them.
  bool get isUsable {
    if (!meetsRequirements) {
      return false;
    }

    if (!(isComplete ?? true)) {
      return false;
    }

    if (!(isLaunchable ?? true)) {
      return false;
    }

    if (isRebootRequired ?? false) {
      return false;
    }

    return true;
  }
}
