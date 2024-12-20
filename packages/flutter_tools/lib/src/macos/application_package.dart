// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../application_package.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../base/utils.dart';
import '../build_info.dart';
import '../globals.dart' as globals;
import '../ios/plist_parser.dart';
import '../xcode_project.dart';

/// Tests whether a [FileSystemEntity] is an macOS bundle directory.
bool _isBundleDirectory(FileSystemEntity entity) =>
    entity is Directory && entity.path.endsWith('.app');

abstract class MacOSApp extends ApplicationPackage {
  MacOSApp({required String projectBundleId}) : super(id: projectBundleId);

  /// Creates a new [MacOSApp] from a macOS project directory.
  factory MacOSApp.fromMacOSProject(MacOSProject project) {
    // projectBundleId is unused for macOS apps. Use a placeholder bundle ID.
    return BuildableMacOSApp(project, 'com.example.placeholder');
  }

  /// Creates a new [MacOSApp] from an existing app bundle.
  ///
  /// `applicationBinary` is the path to the framework directory created by an
  /// Xcode build. By default, this is located under
  /// "~/Library/Developer/Xcode/DerivedData/" and contains an executable
  /// which is expected to start the application and send the vmService
  /// port over stdout.
  static MacOSApp? fromPrebuiltApp(FileSystemEntity applicationBinary) {
    final _BundleInfo? bundleInfo = _executableFromBundle(applicationBinary);
    if (bundleInfo == null) {
      return null;
    }

    return PrebuiltMacOSApp(
      uncompressedBundle: bundleInfo.uncompressedBundle,
      bundleName: bundleInfo.uncompressedBundle.path,
      projectBundleId: bundleInfo.id,
      executable: bundleInfo.executable,
      applicationPackage: applicationBinary,
    );
  }

  /// Look up the executable name for a macOS application bundle.
  static _BundleInfo? _executableFromBundle(FileSystemEntity applicationBundle) {
    final FileSystemEntityType entityType = globals.fs.typeSync(applicationBundle.path);
    if (entityType == FileSystemEntityType.notFound) {
      globals.printError('File "${applicationBundle.path}" does not exist.');
      return null;
    }
    Directory uncompressedBundle;
    if (entityType == FileSystemEntityType.directory) {
      final Directory directory = globals.fs.directory(applicationBundle);
      if (!_isBundleDirectory(directory)) {
        globals.printError('Folder "${applicationBundle.path}" is not an app bundle.');
        return null;
      }
      uncompressedBundle = globals.fs.directory(applicationBundle);
    } else {
      // Try to unpack as a zip.
      final Directory tempDir = globals.fs.systemTempDirectory.createTempSync('flutter_app.');
      try {
        globals.os.unzip(globals.fs.file(applicationBundle), tempDir);
      } on ProcessException {
        globals.printError('Invalid prebuilt macOS app. Unable to extract bundle from archive.');
        return null;
      }
      try {
        uncompressedBundle = tempDir.listSync().whereType<Directory>().singleWhere(
          _isBundleDirectory,
        );
      } on StateError {
        globals.printError(
          'Archive "${applicationBundle.path}" does not contain a single app bundle.',
        );
        return null;
      }
    }
    final String plistPath = globals.fs.path.join(
      uncompressedBundle.path,
      'Contents',
      'Info.plist',
    );
    if (!globals.fs.file(plistPath).existsSync()) {
      globals.printError('Invalid prebuilt macOS app. Does not contain Info.plist.');
      return null;
    }
    final Map<String, dynamic> propertyValues = globals.plistParser.parseFile(plistPath);
    final String? id = propertyValues[PlistParser.kCFBundleIdentifierKey] as String?;
    final String? executableName = propertyValues[PlistParser.kCFBundleExecutableKey] as String?;
    if (id == null) {
      globals.printError(
        'Invalid prebuilt macOS app. Info.plist does not contain bundle identifier',
      );
      return null;
    }
    if (executableName == null) {
      globals.printError(
        'Invalid prebuilt macOS app. Info.plist does not contain bundle executable',
      );
      return null;
    }
    final String executable = globals.fs.path.join(
      uncompressedBundle.path,
      'Contents',
      'MacOS',
      executableName,
    );
    if (!globals.fs.file(executable).existsSync()) {
      globals.printError('Could not find macOS binary at $executable');
    }
    return _BundleInfo(executable, id, uncompressedBundle);
  }

  @override
  String get displayName => id;

  String? applicationBundle(BuildInfo buildInfo);

  String? executable(BuildInfo buildInfo);
}

class PrebuiltMacOSApp extends MacOSApp implements PrebuiltApplicationPackage {
  PrebuiltMacOSApp({
    required this.uncompressedBundle,
    required this.bundleName,
    required this.projectBundleId,
    required String executable,
    required this.applicationPackage,
  }) : _executable = executable,
       super(projectBundleId: projectBundleId);

  /// The uncompressed bundle of the application.
  ///
  /// [MacOSApp.fromPrebuiltApp] will uncompress the application into a temporary
  /// directory even when an `.zip` file was used to create the [MacOSApp] instance.
  final Directory uncompressedBundle;
  final String bundleName;
  final String projectBundleId;

  final String _executable;

  @override
  String get name => bundleName;

  @override
  String? applicationBundle(BuildInfo buildInfo) => uncompressedBundle.path;

  @override
  String? executable(BuildInfo buildInfo) => _executable;

  /// A [File] or [Directory] pointing to the application bundle.
  ///
  /// This can be either a `.zip` file or an uncompressed `.app` directory.
  @override
  final FileSystemEntity applicationPackage;
}

class BuildableMacOSApp extends MacOSApp {
  BuildableMacOSApp(this.project, String projectBundleId) : super(projectBundleId: projectBundleId);

  final MacOSProject project;

  @override
  String get name => 'macOS';

  @override
  String? applicationBundle(BuildInfo buildInfo) {
    final File appBundleNameFile = project.nameFile;
    if (!appBundleNameFile.existsSync()) {
      globals.printError('Unable to find app name. ${appBundleNameFile.path} does not exist');
      return null;
    }

    return globals.fs.path.join(
      getMacOSBuildDirectory(),
      'Build',
      'Products',
      bundleDirectory(buildInfo),
      appBundleNameFile.readAsStringSync().trim(),
    );
  }

  String bundleDirectory(BuildInfo buildInfo) {
    return sentenceCase(buildInfo.mode.cliName) +
        (buildInfo.flavor != null ? '-${buildInfo.flavor!}' : '');
  }

  @override
  String? executable(BuildInfo buildInfo) {
    final String? directory = applicationBundle(buildInfo);
    if (directory == null) {
      return null;
    }
    final _BundleInfo? bundleInfo = MacOSApp._executableFromBundle(globals.fs.directory(directory));
    return bundleInfo?.executable;
  }
}

class _BundleInfo {
  _BundleInfo(this.executable, this.id, this.uncompressedBundle);

  final Directory uncompressedBundle;
  final String executable;
  final String id;
}
