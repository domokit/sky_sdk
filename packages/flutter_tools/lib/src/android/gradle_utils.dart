// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';
import 'package:process/process.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../base/common.dart';
import '../base/file_system.dart';
import '../base/logger.dart';
import '../base/os.dart';
import '../base/platform.dart';
import '../base/utils.dart';
import '../base/version.dart';
import '../base/version_range.dart';
import '../build_info.dart';
import '../cache.dart';
import '../globals.dart' as globals;
import '../project.dart';
import 'android_sdk.dart';

// These are the versions used in the project templates.
//
// In general, Flutter aims to default to the latest version.
// However, this currently requires to migrate existing integration tests to the latest supported values.
//
// Please see the README before changing any of these values.
const String templateDefaultGradleVersion = '8.10.2';
const String templateAndroidGradlePluginVersion = '8.7.0';
const String templateAndroidGradlePluginVersionForModule = '8.7.0';
const String templateKotlinGradlePluginVersion = '1.8.22';

// The Flutter Gradle Plugin is only applied to app projects, and modules that
// are built from source using (`include_flutter.groovy`). The remaining
// projects are: plugins, and modules compiled as AARs. In modules, the
// ephemeral directory `.android` is always regenerated after `flutter pub get`,
// so new versions are picked up after a Flutter upgrade.
//
// Please see the README before changing any of these values.
const String compileSdkVersion = '35';
const String minSdkVersion = '21';
const String targetSdkVersion = '35';
const String ndkVersion = '26.1.10909125';

// Update these when new major versions of Java are supported by new Gradle
// versions that we support.
// Source of truth: https://docs.gradle.org/current/userguide/compatibility.html
const String oneMajorVersionHigherJavaVersion = '24';

// Update this when new versions of Gradle come out including minor versions
// and should correspond to the maximum Gradle version we test in CI.
//
// Supported here means supported by the tooling for
// flutter analyze --suggestions and does not imply broader flutter support.
const String maxKnownAndSupportedGradleVersion = '8.10.2';

// Update this when new versions of AGP come out.
//
// Supported here means tooling is aware of this version's Java <-> AGP
// compatibility.
@visibleForTesting
const String maxKnownAndSupportedAgpVersion = '8.7.1';

// Update this when new versions of AGP come out.
const String maxKnownAgpVersion = '8.7.1';

// Oldest documented version of AGP that has a listed minimum
// compatible Java version.
const String oldestDocumentedJavaAgpCompatibilityVersion = '4.2';

// Constant used in [_buildAndroidGradlePluginRegExp] and
// [_settingsAndroidGradlePluginRegExp] to identify the version section.
const String _versionGroupName = 'version';

// AGP can be defined in the dependencies block of [build.gradle] or [build.gradle.kts].
// Expected content (covers both classpath and compileOnly cases):
// Groovy DSL with single quotes - 'com.android.tools.build:gradle:{{agpVersion}}'
// Groovy DSL with double quotes - "com.android.tools.build:gradle:{{agpVersion}}"
// Kotlin DSL - ("com.android.tools.build.gradle:{{agpVersion}}")
// ?<version> is used to name the version group which helps with extraction.
final RegExp _androidGradlePluginRegExpFromDependencies = RegExp(
  r"""[^\/]*\s*((\bclasspath\b)|(\bcompileOnly\b))\s*\(?['"]com\.android\.tools\.build:gradle:(?<version>\d+(\.\d+){1,2})\)?""",
  multiLine: true,
);

// AGP can be defined in the plugins block of [build.gradle],
// [build.gradle.kts], [settings.gradle], or [settings.gradle.kts].
// Expected content:
// Groovy DSL with single quotes - id 'com.android.application' version '{{agpVersion}}'
// Groovy DSL with double quotes - id "com.android.application" version "{{agpVersion}}"
// Kotlin DSL - id("com.android.application") version "{{agpVersion}}"
// ?<version> is used to name the version group which helps with extraction.
final RegExp _androidGradlePluginRegExpFromId = RegExp(
  r"""[^\/]*s*id\s*\(?['"]com\.android\.application['"]\)?\s+version\s+['"](?<version>\d+(\.\d+){1,2})\)?""",
  multiLine: true,
);

// Expected content format (with lines above and below).
// Version can have 2 or 3 numbers.
// 'distributionUrl=https\://services.gradle.org/distributions/gradle-7.4.2-all.zip'
// '^\s*' protects against commented out lines.
final RegExp distributionUrlRegex = RegExp(r'^\s*distributionUrl\s*=\s*.*\.zip', multiLine: true);

// Modified version of the gradle distribution url match designed to only match
// gradle.org urls so that we can guarantee any modifications to the url
// still points to a hosted zip.
final RegExp gradleOrgVersionMatch = RegExp(
  r'^\s*distributionUrl\s*=\s*https\\://services\.gradle\.org/distributions/gradle-((?:\d|\.)+)-(.*)\.zip',
  multiLine: true,
);

// This matches uncommented minSdkVersion lines in the module-level build.gradle
// file which have minSdkVersion 16,17, 18, 19, or 20.
final RegExp tooOldMinSdkVersionMatch = RegExp(
  r'(?<=^\s*)minSdkVersion (1[6789]|20)(?=\s*(?://|$))',
  multiLine: true,
);

// From https://docs.gradle.org/current/userguide/command_line_interface.html#command_line_interface
const String gradleVersionFlag = r'--version';

// Directory under android/ that gradle uses to store gradle information.
// Regularly used with [gradleWrapperDirectory] and
// [gradleWrapperPropertiesFilename].
// Different from the directory of gradle files stored in
// `_cache.getArtifactDirectory('gradle_wrapper')`
const String gradleDirectoryName = 'gradle';
const String gradleWrapperDirectoryName = 'wrapper';
const String gradleWrapperPropertiesFilename = 'gradle-wrapper.properties';

/// Provides utilities to run a Gradle task, such as finding the Gradle executable
/// or constructing a Gradle project.
class GradleUtils {
  GradleUtils({
    required Platform platform,
    required Logger logger,
    required Cache cache,
    required OperatingSystemUtils operatingSystemUtils,
  }) : _platform = platform,
       _logger = logger,
       _cache = cache,
       _operatingSystemUtils = operatingSystemUtils;

  final Cache _cache;
  final Platform _platform;
  final Logger _logger;
  final OperatingSystemUtils _operatingSystemUtils;

  /// Gets the Gradle executable path and prepares the Gradle project.
  /// This is the `gradlew` or `gradlew.bat` script in the `android/` directory.
  String getExecutable(FlutterProject project) {
    final Directory androidDir = project.android.hostAppGradleRoot;
    injectGradleWrapperIfNeeded(androidDir);

    final File gradle = androidDir.childFile(getGradlewFileName(_platform));

    if (gradle.existsSync()) {
      _logger.printTrace('Using gradle from ${gradle.absolute.path}.');
      // If the Gradle executable doesn't have execute permission,
      // then attempt to set it.
      _operatingSystemUtils.makeExecutable(gradle);
      return gradle.absolute.path;
    }
    throwToolExit(
      'Unable to locate gradlew script. Please check that ${gradle.path} '
      'exists or that ${gradle.dirname} can be read.',
    );
  }

  /// Injects the Gradle wrapper files if any of these files don't exist in [directory].
  void injectGradleWrapperIfNeeded(Directory directory) {
    copyDirectory(
      _cache.getArtifactDirectory('gradle_wrapper'),
      directory,
      shouldCopyFile: (File sourceFile, File destinationFile) {
        // Don't override the existing files in the project.
        return !destinationFile.existsSync();
      },
      onFileCopied: (File source, File dest) {
        _operatingSystemUtils.makeExecutable(dest);
      },
    );
    // Add the `gradle-wrapper.properties` file if it doesn't exist.
    final Directory propertiesDirectory = directory
        .childDirectory(gradleDirectoryName)
        .childDirectory(gradleWrapperDirectoryName);
    final File propertiesFile = propertiesDirectory.childFile(gradleWrapperPropertiesFilename);

    if (propertiesFile.existsSync()) {
      return;
    }
    propertiesDirectory.createSync(recursive: true);
    final String gradleVersion = getGradleVersionForAndroidPlugin(directory, _logger);
    final String propertyContents = '''
distributionBase=GRADLE_USER_HOME
distributionPath=wrapper/dists
zipStoreBase=GRADLE_USER_HOME
zipStorePath=wrapper/dists
distributionUrl=https\\://services.gradle.org/distributions/gradle-$gradleVersion-all.zip
''';
    propertiesFile.writeAsStringSync(propertyContents);
  }
}

/// Returns the Gradle version that the current Android plugin depends on when found,
/// otherwise it returns a default version.
///
/// The Android plugin version is specified in the [build.gradle],
/// [build.gradle.kts], [settings.gradle], or [settings.gradle.kts] file within
/// the project's Android directory.
String getGradleVersionForAndroidPlugin(Directory directory, Logger logger) {
  final String? androidPluginVersion = getAgpVersion(directory, logger);
  if (androidPluginVersion == null) {
    logger.printTrace(
      'AGP version cannot be determined, assuming Gradle version: $templateDefaultGradleVersion',
    );
    return templateDefaultGradleVersion;
  }
  return getGradleVersionFor(androidPluginVersion);
}

/// Returns the gradle file from the top level directory.
/// The returned file is not guaranteed to be present.
File getGradleWrapperFile(Directory directory) {
  return directory
      .childDirectory(gradleDirectoryName)
      .childDirectory(gradleWrapperDirectoryName)
      .childFile(gradleWrapperPropertiesFilename);
}

/// Parses the gradle wrapper distribution url to return a string containing
/// the version number.
///
/// Expected input is of the form '...gradle-7.4.2-all.zip', and the output
/// would be of the form '7.4.2'.
String? parseGradleVersionFromDistributionUrl(String? distributionUrl) {
  if (distributionUrl == null) {
    return null;
  }
  final List<String> zipParts = distributionUrl.split('-');
  if (zipParts.length < 2) {
    return null;
  }
  return zipParts[1];
}

/// Returns either the gradle-wrapper.properties value from the passed in
/// [directory] or if not present the version available in local path.
///
/// If gradle version is not found null is returned.
/// [directory] should be an android directory with a build.gradle file.
Future<String?> getGradleVersion(
  Directory directory,
  Logger logger,
  ProcessManager processManager,
) async {
  final File propertiesFile = getGradleWrapperFile(directory);

  if (propertiesFile.existsSync()) {
    final String wrapperFileContent = propertiesFile.readAsStringSync();

    final RegExpMatch? distributionUrl = distributionUrlRegex.firstMatch(wrapperFileContent);
    if (distributionUrl != null) {
      final String? gradleVersion = parseGradleVersionFromDistributionUrl(distributionUrl.group(0));
      if (gradleVersion != null) {
        return gradleVersion;
      } else {
        // Did not find gradle zip url. Likely this is a bug in our parsing.
        logger.printWarning(_formatParseWarning(wrapperFileContent));
      }
    } else {
      // If no distributionUrl log then treat as if there was no propertiesFile.
      logger.printTrace(
        '$propertiesFile does not provide a Gradle version falling back to system gradle.',
      );
    }
  } else {
    // Could not find properties file.
    logger.printTrace('$propertiesFile does not exist falling back to system gradle');
  }
  // System installed Gradle version.
  if (processManager.canRun('gradle')) {
    final String gradleVersionVerbose =
        (await processManager.run(<String>['gradle', gradleVersionFlag])).stdout as String;
    // Expected format:
    /*

------------------------------------------------------------
Gradle 7.6
------------------------------------------------------------

Build time:   2022-11-25 13:35:10 UTC
Revision:     daece9dbc5b79370cc8e4fd6fe4b2cd400e150a8

Kotlin:       1.7.10
Groovy:       3.0.13
Ant:          Apache Ant(TM) version 1.10.11 compiled on July 10 2021
JVM:          17.0.6 (Homebrew 17.0.6+0)
OS:           Mac OS X 13.2.1 aarch64
    */
    // Observation shows that the version can have 2 or 3 numbers.
    // Inner parentheticals `(\.\d+)?` denote the optional third value.
    // Outer parentheticals `Gradle (...)` denote a grouping used to extract
    // the version number.
    final RegExp gradleVersionRegex = RegExp(r'Gradle\s+(\d+\.\d+(?:\.\d+)?)');
    final RegExpMatch? version = gradleVersionRegex.firstMatch(gradleVersionVerbose);
    if (version == null) {
      // Most likely a bug in our parse implementation/regex.
      logger.printWarning(_formatParseWarning(gradleVersionVerbose));
      return null;
    }
    return version.group(1);
  } else {
    logger.printTrace('Could not run system gradle');
    return null;
  }
}

/// Returns the Android Gradle Plugin (AGP) version that the current project
/// depends on when found, null otherwise.
///
/// The Android plugin version is specified in the [build.gradle],
/// [build.gradle.kts], [settings.gradle] or [settings.gradle.kts]
/// file within the project's Android directory ([androidDirectory]).
String? getAgpVersion(Directory androidDirectory, Logger logger) {
  File buildFile = androidDirectory.childFile('build.gradle');
  if (!buildFile.existsSync()) {
    buildFile = androidDirectory.childFile('build.gradle.kts');
  }
  if (!buildFile.existsSync()) {
    logger.printTrace('Cannot find build.gradle/build.gradle.kts in $androidDirectory');
    return null;
  }
  final String buildFileContent = buildFile.readAsStringSync();
  final RegExpMatch? buildMatchClasspath = _androidGradlePluginRegExpFromDependencies.firstMatch(
    buildFileContent,
  );
  if (buildMatchClasspath != null) {
    final String? androidPluginVersion = buildMatchClasspath.namedGroup(_versionGroupName);
    logger.printTrace('$buildFile provides AGP version from classpath: $androidPluginVersion');
    return androidPluginVersion;
  }
  final RegExpMatch? buildMatchId = _androidGradlePluginRegExpFromId.firstMatch(buildFileContent);
  if (buildMatchId != null) {
    final String? androidPluginVersion = buildMatchId.namedGroup(_versionGroupName);
    logger.printTrace('$buildFile provides AGP version from plugin id: $androidPluginVersion');
    return androidPluginVersion;
  }

  logger.printTrace("$buildFile doesn't provide an AGP version. Checking settings.");
  File settingsFile = androidDirectory.childFile('settings.gradle');
  if (!settingsFile.existsSync()) {
    settingsFile = androidDirectory.childFile('settings.gradle.kts');
  }
  if (!settingsFile.existsSync()) {
    logger.printTrace('Cannot find settings.gradle/settings.gradle.kts in $androidDirectory');
    return null;
  }
  final String settingsFileContent = settingsFile.readAsStringSync();
  final RegExpMatch? settingsMatch = _androidGradlePluginRegExpFromId.firstMatch(
    settingsFileContent,
  );

  if (settingsMatch != null) {
    final String? androidPluginVersion = settingsMatch.namedGroup(_versionGroupName);
    logger.printTrace('$settingsFile provides AGP version: $androidPluginVersion');
    return androidPluginVersion;
  }
  logger.printTrace("$settingsFile doesn't provide an AGP version.");
  return null;
}

String _formatParseWarning(String content) {
  return 'Could not parse gradle version from: \n'
      '$content \n'
      'If there is a version please look for an existing bug '
      'https://github.com/flutter/flutter/issues/'
      ' and if one does not exist file a new issue.';
}

// Validate that Gradle version and AGP are compatible with each other.
//
// Returns true if versions are compatible.
// Null Gradle version or AGP version returns false.
// If compatibility cannot be evaluated returns false.
// If versions are newer than the max known version a warning is logged and true
// returned.
//
// Source of truth found here:
// https://developer.android.com/studio/releases/gradle-plugin#updating-gradle
// AGP has a minimum version of gradle required but no max starting at
// AGP version 2.3.0+.
bool validateGradleAndAgp(Logger logger, {required String? gradleV, required String? agpV}) {
  const String oldestSupportedAgpVersion = '3.3.0';
  const String oldestSupportedGradleVersion = '4.10.1';

  if (gradleV == null || agpV == null) {
    logger.printTrace('Gradle version or AGP version unknown ($gradleV, $agpV).');
    return false;
  }

  // First check if versions are too old.
  if (isWithinVersionRange(agpV, min: '0.0', max: oldestSupportedAgpVersion, inclusiveMax: false)) {
    logger.printTrace('AGP Version: $agpV is too old.');
    return false;
  }
  if (isWithinVersionRange(
    gradleV,
    min: '0.0',
    max: oldestSupportedGradleVersion,
    inclusiveMax: false,
  )) {
    logger.printTrace('Gradle Version: $gradleV is too old.');
    return false;
  }

  // Check highest supported version before checking unknown versions.
  if (isWithinVersionRange(agpV, min: '8.0', max: maxKnownAndSupportedAgpVersion)) {
    return isWithinVersionRange(gradleV, min: '8.0', max: maxKnownAndSupportedGradleVersion);
  }
  // Check if versions are newer than the max known versions.
  if (isWithinVersionRange(agpV, min: maxKnownAndSupportedAgpVersion, max: '100.100')) {
    // Assume versions we do not know about are valid but log.
    final bool validGradle = isWithinVersionRange(gradleV, min: '8.0', max: '100.00');
    logger.printTrace(
      'Newer than known AGP version ($agpV), gradle ($gradleV).'
      '\n Treating as valid configuration.',
    );
    return validGradle;
  }

  // Begin Known Gradle <-> AGP validation.
  if (isWithinVersionRange(agpV, min: '8.4.0', max: '8.4.99')) {
    return isWithinVersionRange(gradleV, min: '8.6', max: maxKnownAndSupportedGradleVersion);
  }
  if (isWithinVersionRange(agpV, min: '8.3.0', max: '8.3.99')) {
    return isWithinVersionRange(gradleV, min: '8.4', max: maxKnownAndSupportedGradleVersion);
  }
  if (isWithinVersionRange(agpV, min: '8.2.0', max: '8.2.99')) {
    return isWithinVersionRange(gradleV, min: '8.2', max: maxKnownAndSupportedGradleVersion);
  }
  if (isWithinVersionRange(agpV, min: '8.0.0', max: '8.1.99')) {
    return isWithinVersionRange(gradleV, min: '8.0', max: maxKnownAndSupportedGradleVersion);
  }
  // Max agp here is a made up version to contain all 7.4 changes.
  if (isWithinVersionRange(agpV, min: '7.4', max: '7.5')) {
    return isWithinVersionRange(gradleV, min: '7.5', max: maxKnownAndSupportedGradleVersion);
  }
  if (isWithinVersionRange(agpV, min: '7.3', max: '7.4', inclusiveMax: false)) {
    return isWithinVersionRange(gradleV, min: '7.4', max: maxKnownAndSupportedGradleVersion);
  }
  if (isWithinVersionRange(agpV, min: '7.2', max: '7.3', inclusiveMax: false)) {
    return isWithinVersionRange(gradleV, min: '7.3.3', max: maxKnownAndSupportedGradleVersion);
  }
  if (isWithinVersionRange(agpV, min: '7.1', max: '7.2', inclusiveMax: false)) {
    return isWithinVersionRange(gradleV, min: '7.2', max: maxKnownAndSupportedGradleVersion);
  }
  if (isWithinVersionRange(agpV, min: '7.0', max: '7.1', inclusiveMax: false)) {
    return isWithinVersionRange(gradleV, min: '7.0', max: maxKnownAndSupportedGradleVersion);
  }
  if (isWithinVersionRange(agpV, min: '4.2.0', max: '7.0', inclusiveMax: false)) {
    return isWithinVersionRange(gradleV, min: '6.7.1', max: maxKnownAndSupportedGradleVersion);
  }
  if (isWithinVersionRange(agpV, min: '4.1.0', max: '4.2.0', inclusiveMax: false)) {
    return isWithinVersionRange(gradleV, min: '6.5', max: maxKnownAndSupportedGradleVersion);
  }
  if (isWithinVersionRange(agpV, min: '4.0.0', max: '4.1.0', inclusiveMax: false)) {
    return isWithinVersionRange(gradleV, min: '6.1.1', max: maxKnownAndSupportedGradleVersion);
  }
  if (isWithinVersionRange(agpV, min: '3.6.0', max: '3.6.4')) {
    return isWithinVersionRange(gradleV, min: '5.6.4', max: maxKnownAndSupportedGradleVersion);
  }
  if (isWithinVersionRange(agpV, min: '3.5.0', max: '3.5.4')) {
    return isWithinVersionRange(gradleV, min: '5.4.1', max: maxKnownAndSupportedGradleVersion);
  }
  if (isWithinVersionRange(agpV, min: '3.4.0', max: '3.4.3')) {
    return isWithinVersionRange(gradleV, min: '5.1.1', max: maxKnownAndSupportedGradleVersion);
  }
  if (isWithinVersionRange(agpV, min: '3.3.0', max: '3.3.3')) {
    return isWithinVersionRange(gradleV, min: '4.10.1', max: maxKnownAndSupportedGradleVersion);
  }

  logger.printTrace('Unknown Gradle-Agp compatibility, $gradleV, $agpV');
  return false;
}

/// Validate that the [javaVersion] and Gradle version are compatible with
/// each other.
///
/// Source of truth:
/// https://docs.gradle.org/current/userguide/compatibility.html#java
bool validateJavaAndGradle(Logger logger, {required String? javaV, required String? gradleV}) {
  // https://docs.gradle.org/current/userguide/compatibility.html#java
  const String oldestSupportedJavaVersion = '1.8';
  const String oldestDocumentedJavaGradleCompatibility = '2.0';

  // Begin Java <-> Gradle validation.

  if (javaV == null || gradleV == null) {
    logger.printTrace('Java version or Gradle version unknown ($javaV, $gradleV).');
    return false;
  }

  // First check if versions are too old.
  if (isWithinVersionRange(
    javaV,
    min: '1.1',
    max: oldestSupportedJavaVersion,
    inclusiveMax: false,
  )) {
    logger.printTrace('Java Version: $javaV is too old.');
    return false;
  }
  if (isWithinVersionRange(
    gradleV,
    min: '0.0',
    max: oldestDocumentedJavaGradleCompatibility,
    inclusiveMax: false,
  )) {
    logger.printTrace('Gradle Version: $gradleV is too old.');
    return false;
  }

  // Check if versions are newer than the max supported versions.
  if (isWithinVersionRange(javaV, min: oneMajorVersionHigherJavaVersion, max: '100.100')) {
    // Assume versions Java versions newer than [maxSupportedJavaVersion]
    // required a higher gradle version.
    final bool validGradle = isWithinVersionRange(
      gradleV,
      min: maxKnownAndSupportedGradleVersion,
      max: '100.00',
    );
    logger.printWarning(
      'Newer than known valid Java version ($javaV), gradle ($gradleV).'
      '\n Treating as valid configuration.',
    );
    return validGradle;
  }

  // Begin known Java <-> Gradle evaluation.
  for (final JavaGradleCompat data in _javaGradleCompatList) {
    if (isWithinVersionRange(javaV, min: data.javaMin, max: data.javaMax, inclusiveMax: false)) {
      return isWithinVersionRange(gradleV, min: data.gradleMin, max: data.gradleMax);
    }
  }

  logger.printTrace('Unknown Java-Gradle compatibility $javaV, $gradleV');
  return false;
}

/// Returns compatibility information for the valid range of Gradle versions for
/// the specified Java version.
///
/// Returns null when the tooling has not documented the compatible Gradle
/// versions for the Java version (either the version is too old or too new). If
/// this seems like a mistake, the caller may need to update the
/// [_javaGradleCompatList] detailing Java/Gradle compatibility.
JavaGradleCompat? getValidGradleVersionRangeForJavaVersion(Logger logger, {required String javaV}) {
  for (final JavaGradleCompat data in _javaGradleCompatList) {
    if (isWithinVersionRange(javaV, min: data.javaMin, max: data.javaMax, inclusiveMax: false)) {
      return data;
    }
  }

  logger.printTrace('Unable to determine valid Gradle version range for Java version $javaV.');
  return null;
}

/// Validate that the specified Java and Android Gradle Plugin (AGP) versions are
/// compatible with each other.
///
/// Returns true when the specified Java and AGP versions are
/// definitely compatible; otherwise, false is assumed by default. In addition,
/// this will return false when either a null Java or AGP version is provided.
///
/// Source of truth are the AGP release notes:
/// https://developer.android.com/build/releases/gradle-plugin
bool validateJavaAndAgp(Logger logger, {required String? javaV, required String? agpV}) {
  if (javaV == null || agpV == null) {
    logger.printTrace('Java version or AGP version unknown ($javaV, $agpV).');
    return false;
  }

  // Check if AGP version is too old to perform validation.
  if (isWithinVersionRange(
    agpV,
    min: '1.0',
    max: oldestDocumentedJavaAgpCompatibilityVersion,
    inclusiveMax: false,
  )) {
    logger.printTrace('AGP Version: $agpV is too old to determine Java compatibility.');
    return false;
  }

  if (isWithinVersionRange(
    agpV,
    min: maxKnownAndSupportedAgpVersion,
    max: '100.100',
    inclusiveMin: false,
  )) {
    logger.printTrace('AGP Version: $agpV is too new to determine Java compatibility.');
    return false;
  }

  // Begin known Java <-> AGP evaluation.
  for (final JavaAgpCompat data in _javaAgpCompatList) {
    if (isWithinVersionRange(agpV, min: data.agpMin, max: data.agpMax)) {
      return isWithinVersionRange(javaV, min: data.javaMin, max: '100.100');
    }
  }

  logger.printTrace('Unknown Java-AGP compatibility $javaV, $agpV');
  return false;
}

/// Returns compatibility information concerning the minimum AGP
/// version for the specified Java version.
JavaAgpCompat? getMinimumAgpVersionForJavaVersion(Logger logger, {required String javaV}) {
  for (final JavaAgpCompat data in _javaAgpCompatList) {
    if (isWithinVersionRange(javaV, min: data.javaMin, max: '100.100')) {
      return data;
    }
  }

  logger.printTrace('Unable to determine minimum AGP version for specified Java version.');
  return null;
}

/// Returns valid Java range for specified Gradle and AGP versions.
///
/// Assumes that gradleV and agpV are compatible versions.
VersionRange getJavaVersionFor({required String gradleV, required String agpV}) {
  // Find minimum Java version based on AGP compatibility.
  String? minJavaVersion;
  for (final JavaAgpCompat data in _javaAgpCompatList) {
    if (isWithinVersionRange(agpV, min: data.agpMin, max: data.agpMax)) {
      minJavaVersion = data.javaMin;
    }
  }

  // Find maximum Java version based on Gradle compatibility.
  String? maxJavaVersion;
  for (final JavaGradleCompat data in _javaGradleCompatList.reversed) {
    if (isWithinVersionRange(
      gradleV,
      min: data.gradleMin,
      max: maxKnownAndSupportedGradleVersion,
    )) {
      maxJavaVersion = data.javaMax;
    }
  }

  return VersionRange(minJavaVersion, maxJavaVersion);
}

/// Returns the Gradle version that is required by the given Android Gradle plugin version
/// by picking the largest compatible version from
/// https://developer.android.com/studio/releases/gradle-plugin#updating-gradle
String getGradleVersionFor(String androidPluginVersion) {
  final List<GradleForAgp> compatList = <GradleForAgp>[
    GradleForAgp(agpMin: '1.0.0', agpMax: '1.1.3', minRequiredGradle: '2.3'),
    GradleForAgp(agpMin: '1.2.0', agpMax: '1.3.1', minRequiredGradle: '2.9'),
    GradleForAgp(agpMin: '1.5.0', agpMax: '1.5.0', minRequiredGradle: '2.2.1'),
    GradleForAgp(agpMin: '2.0.0', agpMax: '2.1.2', minRequiredGradle: '2.13'),
    GradleForAgp(agpMin: '2.1.3', agpMax: '2.2.3', minRequiredGradle: '2.14.1'),
    GradleForAgp(agpMin: '2.3.0', agpMax: '2.9.9', minRequiredGradle: '3.3'),
    GradleForAgp(agpMin: '3.0.0', agpMax: '3.0.9', minRequiredGradle: '4.1'),
    GradleForAgp(agpMin: '3.1.0', agpMax: '3.1.9', minRequiredGradle: '4.4'),
    GradleForAgp(agpMin: '3.2.0', agpMax: '3.2.1', minRequiredGradle: '4.6'),
    GradleForAgp(agpMin: '3.3.0', agpMax: '3.3.2', minRequiredGradle: '4.10.2'),
    GradleForAgp(agpMin: '3.4.0', agpMax: '3.5.0', minRequiredGradle: '5.6.2'),
    GradleForAgp(agpMin: '4.0.0', agpMax: '4.1.0', minRequiredGradle: '6.7'),
    // 7.5 is a made up value to include everything through 7.4.*
    GradleForAgp(agpMin: '7.0.0', agpMax: '7.5', minRequiredGradle: '7.5'),
    // Use 0 and 99 as a patch values to signify every AGP patch version with
    // that major and minor version.
    GradleForAgp(agpMin: '8.0.0', agpMax: '8.1.99', minRequiredGradle: '8.0'),
    GradleForAgp(agpMin: '8.2.0', agpMax: '8.2.99', minRequiredGradle: '8.2'),
    GradleForAgp(agpMin: '8.3.0', agpMax: '8.3.99', minRequiredGradle: '8.4'),
    GradleForAgp(agpMin: '8.4.0', agpMax: '8.4.99', minRequiredGradle: '8.6'),
    GradleForAgp(agpMin: '8.5.0', agpMax: '8.6.99', minRequiredGradle: '8.7'),
    GradleForAgp(agpMin: '8.7.0', agpMax: '8.7.99', minRequiredGradle: '8.9'),
    // Assume if AGP is newer than this code know about return the highest gradle
    // version we know about.
    GradleForAgp(
      agpMin: maxKnownAgpVersion,
      agpMax: maxKnownAgpVersion,
      minRequiredGradle: maxKnownAndSupportedGradleVersion,
    ),
  ];
  for (final GradleForAgp data in compatList) {
    if (isWithinVersionRange(androidPluginVersion, min: data.agpMin, max: data.agpMax)) {
      return data.minRequiredGradle;
    }
  }
  if (isWithinVersionRange(androidPluginVersion, min: maxKnownAgpVersion, max: '100.100')) {
    return maxKnownAndSupportedGradleVersion;
  }
  throwToolExit('Unsupported Android Plugin version: $androidPluginVersion.');
}

/// Overwrite local.properties in the specified Flutter project's Android
/// sub-project, if needed.
///
/// If [requireAndroidSdk] is true (the default) and no Android SDK is found,
/// this will fail with a [ToolExit].
void updateLocalProperties({
  required FlutterProject project,
  BuildInfo? buildInfo,
  bool requireAndroidSdk = true,
}) {
  if (requireAndroidSdk && globals.androidSdk == null) {
    exitWithNoSdkMessage();
  }
  final File localProperties = project.android.localPropertiesFile;
  bool changed = false;

  SettingsFile settings;
  if (localProperties.existsSync()) {
    settings = SettingsFile.parseFromFile(localProperties);
  } else {
    settings = SettingsFile();
    changed = true;
  }

  void changeIfNecessary(String key, String? value) {
    if (settings.values[key] == value) {
      return;
    }
    if (value == null) {
      settings.values.remove(key);
    } else {
      settings.values[key] = value;
    }
    changed = true;
  }

  final AndroidSdk? androidSdk = globals.androidSdk;
  if (androidSdk != null) {
    changeIfNecessary('sdk.dir', globals.fsUtils.escapePath(androidSdk.directory.path));
  }

  changeIfNecessary('flutter.sdk', globals.fsUtils.escapePath(Cache.flutterRoot!));
  if (buildInfo != null) {
    changeIfNecessary('flutter.buildMode', buildInfo.modeName);
    final String? buildName = validatedBuildNameForPlatform(
      TargetPlatform.android_arm,
      buildInfo.buildName ?? project.manifest.buildName,
      globals.logger,
    );
    changeIfNecessary('flutter.versionName', buildName);
    final String? buildNumber = validatedBuildNumberForPlatform(
      TargetPlatform.android_arm,
      buildInfo.buildNumber ?? project.manifest.buildNumber,
      globals.logger,
    );
    changeIfNecessary('flutter.versionCode', buildNumber);
  }

  if (changed) {
    settings.writeContents(localProperties);
  }
}

/// Writes standard Android local properties to the specified [properties] file.
///
/// Writes the path to the Android SDK, if known.
void writeLocalProperties(File properties) {
  final SettingsFile settings = SettingsFile();
  final AndroidSdk? androidSdk = globals.androidSdk;
  if (androidSdk != null) {
    settings.values['sdk.dir'] = globals.fsUtils.escapePath(androidSdk.directory.path);
  }
  settings.writeContents(properties);
}

void exitWithNoSdkMessage() {
  globals.analytics.send(
    Event.flutterBuildInfo(
      label: 'unsupported-project',
      buildType: 'gradle',
      error: 'android-sdk-not-found',
    ),
  );
  throwToolExit(
    '${globals.logger.terminal.warningMark} No Android SDK found. '
    'Try setting the ANDROID_HOME environment variable.',
  );
}

// Data class to hold normal/defined Java <-> Gradle compatibility criteria.
//
// The [javaMax] is exclusive in terms of supporting the noted [gradleMin],
// whereas [javaMin] is inclusive.
@immutable
class JavaGradleCompat {
  const JavaGradleCompat({
    required this.javaMin,
    required this.javaMax,
    required this.gradleMin,
    required this.gradleMax,
  });

  final String javaMin;
  final String javaMax;
  final String gradleMin;
  final String gradleMax;

  @override
  bool operator ==(Object other) =>
      other is JavaGradleCompat &&
      other.javaMin == javaMin &&
      other.javaMax == javaMax &&
      other.gradleMin == gradleMin &&
      other.gradleMax == gradleMax;

  @override
  int get hashCode => Object.hash(javaMin, javaMax, gradleMin, gradleMax);
}

// Data class to hold defined Java <-> AGP compatibility criteria.
//
// The [agpMin] and [agpMax] are inclusive in terms of having the
// noted [javaMin] and [javaDefault] versions.
@immutable
class JavaAgpCompat {
  const JavaAgpCompat({
    required this.javaMin,
    required this.javaDefault,
    required this.agpMin,
    required this.agpMax,
  });

  final String javaMin;
  final String javaDefault;
  final String agpMin;
  final String agpMax;

  @override
  bool operator ==(Object other) =>
      other is JavaAgpCompat &&
      other.javaMin == javaMin &&
      other.javaDefault == javaDefault &&
      other.agpMin == agpMin &&
      other.agpMax == agpMax;

  @override
  int get hashCode => Object.hash(javaMin, javaDefault, agpMin, agpMax);
}

class GradleForAgp {
  GradleForAgp({required this.agpMin, required this.agpMax, required this.minRequiredGradle});

  final String agpMin;
  final String agpMax;
  final String minRequiredGradle;
}

// Returns gradlew file name based on the platform.
String getGradlewFileName(Platform platform) {
  if (platform.isWindows) {
    return 'gradlew.bat';
  } else {
    return 'gradlew';
  }
}

/// List of compatible Java/Gradle versions.
///
/// Should be updated when a new version of Java is supported by a new version
/// of Gradle, as https://docs.gradle.org/current/userguide/compatibility.html
/// details.
List<JavaGradleCompat> _javaGradleCompatList = const <JavaGradleCompat>[
  JavaGradleCompat(
    javaMin: '23',
    javaMax: '24',
    gradleMin: '8.10',
    gradleMax: maxKnownAndSupportedGradleVersion,
  ),
  JavaGradleCompat(
    javaMin: '22',
    javaMax: '23',
    gradleMin: '8.7',
    gradleMax: maxKnownAndSupportedGradleVersion,
  ),
  JavaGradleCompat(
    javaMin: '21',
    javaMax: '22',
    gradleMin: '8.4',
    gradleMax: maxKnownAndSupportedGradleVersion,
  ),
  JavaGradleCompat(
    javaMin: '20',
    javaMax: '21',
    gradleMin: '8.1',
    gradleMax: maxKnownAndSupportedGradleVersion,
  ),
  JavaGradleCompat(
    javaMin: '19',
    javaMax: '20',
    gradleMin: '7.6',
    gradleMax: maxKnownAndSupportedGradleVersion,
  ),
  JavaGradleCompat(
    javaMin: '18',
    javaMax: '19',
    gradleMin: '7.5',
    gradleMax: maxKnownAndSupportedGradleVersion,
  ),
  JavaGradleCompat(
    javaMin: '17',
    javaMax: '18',
    gradleMin: '7.3',
    gradleMax: maxKnownAndSupportedGradleVersion,
  ),
  JavaGradleCompat(
    javaMin: '16',
    javaMax: '17',
    gradleMin: '7.0',
    gradleMax: maxKnownAndSupportedGradleVersion,
  ),
  JavaGradleCompat(
    javaMin: '15',
    javaMax: '16',
    gradleMin: '6.7',
    gradleMax: maxKnownAndSupportedGradleVersion,
  ),
  JavaGradleCompat(
    javaMin: '14',
    javaMax: '15',
    gradleMin: '6.3',
    gradleMax: maxKnownAndSupportedGradleVersion,
  ),
  JavaGradleCompat(
    javaMin: '13',
    javaMax: '14',
    gradleMin: '6.0',
    gradleMax: maxKnownAndSupportedGradleVersion,
  ),
  JavaGradleCompat(
    javaMin: '12',
    javaMax: '13',
    gradleMin: '5.4',
    gradleMax: maxKnownAndSupportedGradleVersion,
  ),
  JavaGradleCompat(
    javaMin: '11',
    javaMax: '12',
    gradleMin: '5.0',
    gradleMax: maxKnownAndSupportedGradleVersion,
  ),
  // 1.11 is a made up java version to cover everything in 1.10.*
  JavaGradleCompat(
    javaMin: '1.10',
    javaMax: '1.11',
    gradleMin: '4.7',
    gradleMax: maxKnownAndSupportedGradleVersion,
  ),
  JavaGradleCompat(
    javaMin: '1.9',
    javaMax: '1.10',
    gradleMin: '4.3',
    gradleMax: maxKnownAndSupportedGradleVersion,
  ),
  JavaGradleCompat(
    javaMin: '1.8',
    javaMax: '1.9',
    gradleMin: '2.0',
    gradleMax: maxKnownAndSupportedGradleVersion,
  ),
];

// List of compatible Java/AGP versions, where agpMax versions are inclusive.
//
// Should be updated whenever a new version of AGP is released as
// https://developer.android.com/build/releases/gradle-plugin details.
List<JavaAgpCompat> _javaAgpCompatList = const <JavaAgpCompat>[
  JavaAgpCompat(
    javaMin: '17',
    javaDefault: '17',
    agpMin: '8.0',
    agpMax: maxKnownAndSupportedAgpVersion,
  ),
  JavaAgpCompat(javaMin: '11', javaDefault: '11', agpMin: '7.0', agpMax: '7.4'),
  JavaAgpCompat(
    // You may use JDK 1.7 with AGP 4.2, but we treat 1.8 as the default since
    // it is used by default for this AGP version and lower versions of Java
    // are deprecated for executing Gradle.
    javaMin: '1.8',
    javaDefault: '1.8',
    agpMin: '4.2',
    agpMax: '4.2',
  ),
];
