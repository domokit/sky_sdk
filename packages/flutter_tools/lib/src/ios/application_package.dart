// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../application_package.dart';
import '../base/file_system.dart';
import '../build_info.dart';
import '../cache.dart';
import '../globals.dart' as globals;
import '../template.dart';
import '../xcode_project.dart';
import 'plist_parser.dart';

/// Tests whether a [Directory] is an iOS bundle directory.
bool _isBundleDirectory(Directory dir) => dir.path.endsWith('.app');

abstract class IOSApp extends ApplicationPackage {
  IOSApp({required String projectBundleId}) : super(id: projectBundleId);

  /// Creates a new IOSApp from an existing app bundle or IPA.
  static IOSApp? fromPrebuiltApp(FileSystemEntity applicationBinary) {
    final FileSystemEntityType entityType = globals.fs.typeSync(applicationBinary.path);
    if (entityType == FileSystemEntityType.notFound) {
      globals.printError(
        'File "${applicationBinary.path}" does not exist. Use an app bundle or an ipa.',
      );
      return null;
    }
    Directory uncompressedBundle;
    if (entityType == FileSystemEntityType.directory) {
      final Directory directory = globals.fs.directory(applicationBinary);
      if (!_isBundleDirectory(directory)) {
        globals.printError('Folder "${applicationBinary.path}" is not an app bundle.');
        return null;
      }
      uncompressedBundle = globals.fs.directory(applicationBinary);
    } else {
      // Try to unpack as an ipa.
      final Directory tempDir = globals.fs.systemTempDirectory.createTempSync('flutter_app.');
      globals.os.unzip(globals.fs.file(applicationBinary), tempDir);
      final Directory payloadDir = globals.fs.directory(
        globals.fs.path.join(tempDir.path, 'Payload'),
      );
      if (!payloadDir.existsSync()) {
        globals.printError('Invalid prebuilt iOS ipa. Does not contain a "Payload" directory.');
        return null;
      }
      try {
        uncompressedBundle = payloadDir.listSync().whereType<Directory>().singleWhere(
          _isBundleDirectory,
        );
      } on StateError {
        globals.printError('Invalid prebuilt iOS ipa. Does not contain a single app bundle.');
        return null;
      }
    }
    final String plistPath = globals.fs.path.join(uncompressedBundle.path, 'Info.plist');
    if (!globals.fs.file(plistPath).existsSync()) {
      globals.printError('Invalid prebuilt iOS app. Does not contain Info.plist.');
      return null;
    }
    final String? id = globals.plistParser.getValueFromFile<String>(
      plistPath,
      PlistParser.kCFBundleIdentifierKey,
    );
    if (id == null) {
      globals.printError('Invalid prebuilt iOS app. Info.plist does not contain bundle identifier');
      return null;
    }

    return PrebuiltIOSApp(
      uncompressedBundle: uncompressedBundle,
      bundleName: globals.fs.path.basename(uncompressedBundle.path),
      projectBundleId: id,
      applicationPackage: applicationBinary,
    );
  }

  static Future<IOSApp?> fromIosProject(IosProject project, BuildInfo? buildInfo) async {
    if (!globals.platform.isMacOS) {
      return null;
    }
    if (!project.exists) {
      // If the project doesn't exist at all the current hint to run flutter
      // create is accurate.
      return null;
    }
    if (!project.xcodeProject.existsSync()) {
      globals.printError('Expected ios/Runner.xcodeproj but this file is missing.');
      return null;
    }
    if (!project.xcodeProjectInfoFile.existsSync()) {
      globals.printError('Expected ios/Runner.xcodeproj/project.pbxproj but this file is missing.');
      return null;
    }
    return BuildableIOSApp.fromProject(project, buildInfo);
  }

  @override
  String get displayName => id;

  String get simulatorBundlePath;

  String get deviceBundlePath;

  /// Directory used by ios-deploy to store incremental installation metadata for
  /// faster second installs.
  Directory? get appDeltaDirectory;
}

class BuildableIOSApp extends IOSApp {
  BuildableIOSApp(this.project, String projectBundleId, String? productName)
    : _appProductName = productName,
      super(projectBundleId: projectBundleId);

  static Future<BuildableIOSApp?> fromProject(IosProject project, BuildInfo? buildInfo) async {
    final String? productName = await project.productName(buildInfo);
    final String? projectBundleId = await project.productBundleIdentifier(buildInfo);
    if (projectBundleId != null) {
      return BuildableIOSApp(project, projectBundleId, productName);
    }
    return null;
  }

  final IosProject project;

  final String? _appProductName;

  @override
  String? get name => _appProductName;

  @override
  String get simulatorBundlePath => _buildAppPath('iphonesimulator');

  @override
  String get deviceBundlePath => _buildAppPath('iphoneos');

  @override
  Directory get appDeltaDirectory =>
      globals.fs.directory(globals.fs.path.join(getIosBuildDirectory(), 'app-delta'));

  // Xcode uses this path for the final archive bundle location,
  // not a top-level output directory.
  // Specifying `build/ios/archive/Runner` will result in `build/ios/archive/Runner.xcarchive`.
  String get archiveBundlePath =>
      globals.fs.path.join(getIosBuildDirectory(), 'archive', _appProductName ?? 'Runner');

  // The output xcarchive bundle path `build/ios/archive/Runner.xcarchive`.
  String get archiveBundleOutputPath => '$archiveBundlePath.xcarchive';

  String get builtInfoPlistPathAfterArchive => globals.fs.path.join(
    archiveBundleOutputPath,
    'Products',
    'Applications',
    _appProductName != null ? '$_appProductName.app' : 'Runner.app',
    'Info.plist',
  );

  String get projectAppIconDirName => _projectImageAssetDirName(_appIconAsset);

  String get projectLaunchImageDirName => _projectImageAssetDirName(_launchImageAsset);

  String get templateAppIconDirNameForContentsJson =>
      _templateImageAssetDirNameForContentsJson(_appIconAsset);

  String get templateLaunchImageDirNameForContentsJson =>
      _templateImageAssetDirNameForContentsJson(_launchImageAsset);

  Future<String> get templateAppIconDirNameForImages async =>
      _templateImageAssetDirNameForImages(_appIconAsset);

  Future<String> get templateLaunchImageDirNameForImages async =>
      _templateImageAssetDirNameForImages(_launchImageAsset);

  String get ipaOutputPath => globals.fs.path.join(getIosBuildDirectory(), 'ipa');

  String _buildAppPath(String type) {
    return globals.fs.path.join(getIosBuildDirectory(), type, '$_appProductName.app');
  }

  String _projectImageAssetDirName(String asset) =>
      globals.fs.path.join('ios', 'Runner', 'Assets.xcassets', asset);

  // Template asset's Contents.json file is in flutter_tools, but the actual
  String _templateImageAssetDirNameForContentsJson(String asset) => globals.fs.path.join(
    Cache.flutterRoot!,
    'packages',
    'flutter_tools',
    'templates',
    _templateImageAssetDirNameSuffix(asset),
  );

  // Template asset's images are in flutter_template_images package.
  Future<String> _templateImageAssetDirNameForImages(String asset) async {
    final Directory imageTemplate = await templatePathProvider.imageDirectory(
      null,
      globals.fs,
      globals.logger,
    );
    return globals.fs.path.join(imageTemplate.path, _templateImageAssetDirNameSuffix(asset));
  }

  String _templateImageAssetDirNameSuffix(String asset) =>
      globals.fs.path.join('app', 'ios.tmpl', 'Runner', 'Assets.xcassets', asset);

  String get _appIconAsset => 'AppIcon.appiconset';
  String get _launchImageAsset => 'LaunchImage.imageset';
}

class PrebuiltIOSApp extends IOSApp implements PrebuiltApplicationPackage {
  PrebuiltIOSApp({
    required this.uncompressedBundle,
    this.bundleName,
    required super.projectBundleId,
    required this.applicationPackage,
  });

  /// The uncompressed bundle of the application.
  ///
  /// [IOSApp.fromPrebuiltApp] will uncompress the application into a temporary
  /// directory even when an `.ipa` file was used to create the [IOSApp] instance.
  final Directory uncompressedBundle;
  final String? bundleName;

  @override
  final Directory? appDeltaDirectory = null;

  @override
  String? get name => bundleName;

  @override
  String get simulatorBundlePath => _bundlePath;

  @override
  String get deviceBundlePath => _bundlePath;

  String get _bundlePath => uncompressedBundle.path;

  /// A [File] or [Directory] pointing to the application bundle.
  ///
  /// This can be either an `.ipa` file or an uncompressed `.app` directory.
  @override
  final FileSystemEntity applicationPackage;
}
