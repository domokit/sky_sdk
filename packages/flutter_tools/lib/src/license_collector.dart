// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:package_config/package_config.dart';

import 'base/file_system.dart';

/// Processes dependencies into a string representing the NOTICES file.
///
/// Reads the NOTICES or LICENSE file from each package in the .packages file,
/// splitting each one into each component license so that it can be de-duped
/// if possible. If the NOTICES file exists, it is preferred over the LICENSE
/// file.
///
/// Individual licenses inside each LICENSE file should be separated by 80
/// hyphens on their own on a line.
///
/// If a LICENSE or NOTICES file contains more than one component license,
/// then each component license must start with the names of the packages to
/// which the component license applies, with each package name on its own line
/// and the list of package names separated from the actual license text by a
/// blank line. The packages need not match the names of the pub package. For
/// example, a package might itself contain code from multiple third-party
/// sources, and might need to include a license for each one.
class LicenseCollector {
  LicenseCollector({required FileSystem fileSystem}) : _fileSystem = fileSystem;

  final FileSystem _fileSystem;

  /// The expected separator for multiple licenses.
  static final String licenseSeparator = '\n${'-' * 80}\n';

  /// Obtain licenses from the `packageMap` into a single result.
  ///
  /// [additionalLicenses] should contain aggregated license files from all
  /// of the current applications dependencies.
  LicenseResult obtainLicenses(
    PackageConfig packageConfig,
    Map<String, List<File>> additionalLicenses,
  ) {
    final Map<String, Set<String>> packageLicenses = <String, Set<String>>{};
    final Set<String> allPackages = <String>{};
    final List<File> dependencies = <File>[];

    for (final Package package in packageConfig.packages) {
      final Uri packageUri = package.packageUriRoot;
      if (packageUri.scheme != 'file') {
        continue;
      }
      // First check for NOTICES, then fallback to LICENSE
      File file = _fileSystem.file(packageUri.resolve('../NOTICES'));
      if (!file.existsSync()) {
        file = _fileSystem.file(packageUri.resolve('../LICENSE'));
      }
      if (!file.existsSync()) {
        continue;
      }

      dependencies.add(file);
      final List<String> rawLicenses = file.readAsStringSync().split(licenseSeparator);
      for (final String rawLicense in rawLicenses) {
        List<String> packageNames = <String>[];
        String? licenseText;
        if (rawLicenses.length > 1) {
          final int split = rawLicense.indexOf('\n\n');
          if (split >= 0) {
            packageNames = rawLicense.substring(0, split).split('\n');
            licenseText = rawLicense.substring(split + 2);
          }
        }
        if (licenseText == null) {
          packageNames = <String>[package.name];
          licenseText = rawLicense;
        }
        packageLicenses.putIfAbsent(licenseText, () => <String>{}).addAll(packageNames);
        allPackages.addAll(packageNames);
      }
    }

    final List<String> combinedLicensesList =
        packageLicenses.entries.map<String>((MapEntry<String, Set<String>> entry) {
          final List<String> packageNames = entry.value.toList()..sort();
          return '${packageNames.join('\n')}\n\n${entry.key}';
        }).toList();
    combinedLicensesList.sort();

    /// Append additional LICENSE files as specified in the pubspec.yaml.
    final List<String> additionalLicenseText = <String>[];
    final List<String> errorMessages = <String>[];
    for (final String package in additionalLicenses.keys) {
      for (final File license in additionalLicenses[package]!) {
        if (!license.existsSync()) {
          errorMessages.add(
            'package $package specified an additional license at ${license.path}, but this file '
            'does not exist.',
          );
          continue;
        }
        dependencies.add(license);
        try {
          additionalLicenseText.add(license.readAsStringSync());
        } on FormatException catch (err) {
          // File has an invalid encoding.
          errorMessages.add(
            'package $package specified an additional license at ${license.path}, but this file '
            'could not be read:\n$err',
          );
        } on FileSystemException catch (err) {
          // File cannot be parsed.
          errorMessages.add(
            'package $package specified an additional license at ${license.path}, but this file '
            'could not be read:\n$err',
          );
        }
      }
    }
    if (errorMessages.isNotEmpty) {
      return LicenseResult(
        combinedLicenses: '',
        dependencies: <File>[],
        errorMessages: errorMessages,
      );
    }

    final String combinedLicenses = combinedLicensesList
        .followedBy(additionalLicenseText)
        .join(licenseSeparator);

    return LicenseResult(
      combinedLicenses: combinedLicenses,
      dependencies: dependencies,
      errorMessages: errorMessages,
    );
  }
}

/// The result of processing licenses with a [LicenseCollector].
class LicenseResult {
  const LicenseResult({
    required this.combinedLicenses,
    required this.dependencies,
    required this.errorMessages,
  });

  /// The raw text of the consumed licenses.
  final String combinedLicenses;

  /// Each license file that was consumed as input.
  final List<File> dependencies;

  /// If non-empty, license collection failed and this messages should
  /// be displayed by the asset parser.
  final List<String> errorMessages;
}
