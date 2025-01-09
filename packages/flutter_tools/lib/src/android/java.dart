// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:process/process.dart';

import '../base/config.dart';
import '../base/file_system.dart';
import '../base/logger.dart';
import '../base/os.dart';
import '../base/platform.dart';
import '../base/process.dart';
import '../base/version.dart';
import 'android_studio.dart';

const String _javaExecutable = 'java';

enum JavaSource {
  /// JDK bundled with latest Android Studio installation.
  androidStudio,

  /// JDK specified by the system's JAVA_HOME environment variable.
  javaHome,

  /// JDK available through the system's PATH environment variable.
  path,

  /// JDK specified in Flutter's configuration.
  flutterConfig,
}

typedef _JavaHomePathWithSource = ({String path, JavaSource source});

/// Represents an installation of Java.
class Java {
  Java({
    required this.javaHome,
    required this.binaryPath,
    required this.javaSource,
    required Logger logger,
    required FileSystem fileSystem,
    required OperatingSystemUtils os,
    required Platform platform,
    required ProcessManager processManager,
  }) : _logger = logger,
       _fileSystem = fileSystem,
       _os = os,
       _platform = platform,
       _processManager = processManager,
       _processUtils = ProcessUtils(processManager: processManager, logger: logger);

  /// Within the Java ecosystem, this environment variable is typically set
  /// the install location of a Java Runtime Environment (JRE) or Java
  /// Development Kit (JDK).
  ///
  /// Tools that depend on Java and need to find it will often check this
  /// variable. If you are looking to set `JAVA_HOME` when stating a process,
  /// consider using the [environment] instance property instead.
  static String javaHomeEnvironmentVariable = 'JAVA_HOME';

  /// Finds the Java runtime environment that should be used for all java-dependent
  /// operations across the tool.
  ///
  /// This searches for Java in the following places, in order:
  ///
  /// 1. the runtime environment bundled with Android Studio;
  /// 2. the runtime environment found in the JAVA_HOME env variable, if set; or
  /// 3. the java binary found on PATH.
  ///
  /// Returns null if no java binary could be found.
  static Java? find({
    required Config config,
    required AndroidStudio? androidStudio,
    required Logger logger,
    required FileSystem fileSystem,
    required Platform platform,
    required ProcessManager processManager,
  }) {
    final OperatingSystemUtils os = OperatingSystemUtils(
      fileSystem: fileSystem,
      logger: logger,
      platform: platform,
      processManager: processManager,
    );
    final _JavaHomePathWithSource? home = _findJavaHome(
      config: config,
      logger: logger,
      androidStudio: androidStudio,
      platform: platform,
    );
    final String? binary = _findJavaBinary(
      logger: logger,
      javaHome: home?.path,
      fileSystem: fileSystem,
      operatingSystemUtils: os,
      platform: platform,
    );

    if (binary == null) {
      return null;
    }

    // If javaHome == null and binary is not null, it means that
    // binary obtained from PATH as fallback.
    final JavaSource javaSource = home?.source ?? JavaSource.path;

    return Java(
      javaHome: home?.path,
      binaryPath: binary,
      javaSource: javaSource,
      logger: logger,
      fileSystem: fileSystem,
      os: os,
      platform: platform,
      processManager: processManager,
    );
  }

  /// The path of the runtime environments' home directory.
  ///
  /// This should only be used for logging and validation purposes.
  /// If you need to set JAVA_HOME when starting a process, consider
  /// using [environment] instead.
  /// If you need to inspect the files of the runtime, considering adding
  /// a new method to this class instead.
  final String? javaHome;

  /// The path of the runtime environments' java binary.
  ///
  /// This should be only used for logging and validation purposes.
  /// If you need to invoke the binary directly, consider adding a new method
  /// to this class instead.
  final String binaryPath;

  /// Indicates the source from where the Java runtime was located.
  ///
  /// This information is useful for debugging and logging purposes to track
  /// which source was used to locate the Java runtime environment.
  final JavaSource javaSource;

  final Logger _logger;
  final FileSystem _fileSystem;
  final OperatingSystemUtils _os;
  final Platform _platform;
  final ProcessManager _processManager;
  final ProcessUtils _processUtils;

  /// Returns an environment variable map with
  /// 1. JAVA_HOME set if this object has a known home directory, and
  /// 2. The java binary folder appended onto PATH, if the binary location is known.
  ///
  /// This map should be used as the environment when invoking any Java-dependent
  /// processes, such as Gradle or Android SDK tools (avdmanager, sdkmanager, etc.)
  Map<String, String> get environment => <String, String>{
    if (javaHome != null) javaHomeEnvironmentVariable: javaHome!,
    'PATH':
        _fileSystem.path.dirname(binaryPath) +
        _os.pathVarSeparator +
        _platform.environment['PATH']!,
  };

  /// Returns the version of java in the format \d(.\d)+(.\d)+
  /// Returns null if version could not be determined.
  late final Version? version =
      (() {
        if (!canRun()) {
          return null;
        }

        final RunResult result = _processUtils.runSync(<String>[
          binaryPath,
          '--version',
        ], environment: environment);
        if (result.exitCode != 0) {
          _logger.printTrace(
            'java --version failed: exitCode: ${result.exitCode}'
            ' stdout: ${result.stdout} stderr: ${result.stderr}',
          );
          return null;
        }
        final String rawVersionOutput = result.stdout;
        final List<String> versionLines = rawVersionOutput.split('\n');
        // Should look something like 'openjdk 19.0.2 2023-01-17'.
        final String longVersionText = versionLines.length >= 2 ? versionLines[1] : versionLines[0];

        // The contents that matter come in the format '11.0.18', '1.8.0_202 or 21'.
        final RegExp jdkVersionRegex = RegExp(r'(?<version>\d+(\.\d+(\.\d+(?:_\d+)?)?)?)');
        final Iterable<RegExpMatch> matches = jdkVersionRegex.allMatches(rawVersionOutput);
        if (matches.isEmpty) {
          // Fallback to second string format like "java 21.0.1 2023-09-19 LTS"
          final RegExp secondJdkVersionRegex = RegExp(
            r'java\s+(?<version>\d+(\.\d+)?(\.\d+)?)\s+\d\d\d\d-\d\d-\d\d',
          );
          final RegExpMatch? match = secondJdkVersionRegex.firstMatch(versionLines[0]);
          if (match != null) {
            return Version.parse(match.namedGroup('version'));
          }
          _logger.printWarning(_formatJavaVersionWarning(rawVersionOutput));
          return null;
        }
        final String? version = matches.first.namedGroup('version');
        if (version == null || version.split('_').isEmpty) {
          _logger.printWarning(_formatJavaVersionWarning(rawVersionOutput));
          return null;
        }

        // Trim away _d+ from versions 1.8 and below.
        final String versionWithoutBuildInfo = version.split('_').first;

        final Version? parsedVersion = Version.parse(versionWithoutBuildInfo);
        if (parsedVersion == null) {
          return null;
        }
        return Version.withText(
          parsedVersion.major,
          parsedVersion.minor,
          parsedVersion.patch,
          longVersionText,
        );
      })();

  bool canRun() {
    return _processManager.canRun(binaryPath);
  }
}

_JavaHomePathWithSource? _findJavaHome({
  required Config config,
  required Logger logger,
  required AndroidStudio? androidStudio,
  required Platform platform,
}) {
  final Object? configured = config.getValue('jdk-dir');
  if (configured != null) {
    return (path: configured as String, source: JavaSource.flutterConfig);
  }

  final String? androidStudioJavaPath = androidStudio?.javaPath;
  if (androidStudioJavaPath != null) {
    return (path: androidStudioJavaPath, source: JavaSource.androidStudio);
  }

  final String? javaHomeEnv = platform.environment[Java.javaHomeEnvironmentVariable];
  if (javaHomeEnv != null) {
    return (path: javaHomeEnv, source: JavaSource.javaHome);
  }
  return null;
}

String? _findJavaBinary({
  required Logger logger,
  required String? javaHome,
  required FileSystem fileSystem,
  required OperatingSystemUtils operatingSystemUtils,
  required Platform platform,
}) {
  if (javaHome != null) {
    return fileSystem.path.join(javaHome, 'bin', 'java');
  }

  // Fallback to PATH based lookup.
  return operatingSystemUtils.which(_javaExecutable)?.path;
}

// Returns a user visible String that says the tool failed to parse
// the version of java along with the output.
String _formatJavaVersionWarning(String javaVersionRaw) {
  return 'Could not parse java version from: \n'
      '$javaVersionRaw \n'
      'If there is a version please look for an existing bug '
      'https://github.com/flutter/flutter/issues/ '
      'and if one does not exist file a new issue.';
}
