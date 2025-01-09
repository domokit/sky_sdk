// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@Timeout(Duration(seconds: 600))
library;

import 'dart:io';
import 'package:file/file.dart';

import '../../src/common.dart';
import '../test_utils.dart';
import 'project.dart';

class MigrateProject extends Project {
  MigrateProject(this.version, {this.vanilla = true});

  @override
  Future<void> setUpIn(
    Directory dir, {
    bool useDeferredLoading = false,
    bool useSyntheticPackage = false,
  }) async {
    this.dir = dir;
    _appPath = dir.path;
    writeFile(
      fileSystem.path.join(dir.path, 'android', 'local.properties'),
      androidLocalProperties,
    );
    final Directory tempDir = createResolvedTempDirectorySync('cipd_dest.');
    final Directory depotToolsDir = createResolvedTempDirectorySync('depot_tools.');

    await processManager.run(<String>[
      'git',
      'clone',
      'https://chromium.googlesource.com/chromium/tools/depot_tools',
      depotToolsDir.path,
    ], workingDirectory: dir.path);

    final File cipdFile = depotToolsDir.childFile(Platform.isWindows ? 'cipd.bat' : 'cipd');
    await processManager.run(<String>[
      cipdFile.path,
      'init',
      tempDir.path,
      '-force',
    ], workingDirectory: dir.path);

    await processManager.run(<String>[
      cipdFile.path,
      'install',
      'flutter/test/full_app_fixtures/vanilla',
      version,
      '-root',
      tempDir.path,
    ], workingDirectory: dir.path);

    if (Platform.isWindows) {
      await processManager.run(<String>['robocopy', tempDir.path, dir.path, '*', '/E', '/mov']);
      // Add full access permissions to Users
      await processManager.run(<String>[
        'icacls',
        tempDir.path,
        '/q',
        '/c',
        '/t',
        '/grant',
        'Users:F',
      ]);
    } else {
      // This cp command changes the symlinks to real files so the tool can edit them.
      await processManager.run(<String>['cp', '-R', '-L', '-f', '${tempDir.path}/.', dir.path]);

      await processManager.run(<String>['rm', '-rf', '.cipd'], workingDirectory: dir.path);

      final List<FileSystemEntity> allFiles = dir.listSync(recursive: true);
      for (final FileSystemEntity file in allFiles) {
        if (file is! File) {
          continue;
        }
        await processManager.run(<String>['chmod', '+w', file.path], workingDirectory: dir.path);
      }
    }

    if (!vanilla) {
      writeFile(fileSystem.path.join(dir.path, 'lib', 'main.dart'), libMain);
      writeFile(fileSystem.path.join(dir.path, 'lib', 'other.dart'), libOther);
      writeFile(fileSystem.path.join(dir.path, 'pubspec.yaml'), pubspecCustom);
    }
    tryToDelete(tempDir);
    tryToDelete(depotToolsDir);
  }

  final String version;
  final bool vanilla;
  late String _appPath;

  // Maintain the same pubspec as the configured app.
  @override
  String get pubspec =>
      fileSystem.file(fileSystem.path.join(_appPath, 'pubspec.yaml')).readAsStringSync();

  String get androidLocalProperties => '''
  flutter.sdk=${getFlutterRoot()}
  ''';

  String get libMain => '''
import 'package:flutter/material.dart';
import 'other.dart';

void main() {
  runApp(MyApp());
}

class MyApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Flutter Demo',
      theme: ThemeData(
        primarySwatch: Colors.blue,
        visualDensity: VisualDensity.adaptivePlatformDensity,
      ),
      home: OtherWidget(),
    );
  }
}

''';

  String get libOther => '''
class OtherWidget extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Container(width: 100, height: 100);
  }
}

''';

  String get pubspecCustom => '''
name: vanilla_app_1_22_6_stable
description: This is a modified description from the default.

# The following line prevents the package from being accidentally published to
# pub.dev using `pub publish`. This is preferred for private packages.
publish_to: 'none' # Remove this line if you wish to publish to pub.dev

# The following defines the version and build number for your application.
# A version number is three numbers separated by dots, like 1.2.43
# followed by an optional build number separated by a +.
# Both the version and the builder number may be overridden in flutter
# build by specifying --build-name and --build-number, respectively.
# In Android, build-name is used as versionName while build-number used as versionCode.
# Read more about Android versioning at https://developer.android.com/studio/publish/versioning
# In iOS, build-name is used as CFBundleShortVersionString while build-number used as CFBundleVersion.
# Read more about iOS versioning at
# https://developer.apple.com/library/archive/documentation/General/Reference/InfoPlistKeyReference/Articles/CoreFoundationKeys.html
version: 1.0.0+1

environment:
  sdk: ^3.7.0-0

dependencies:
  flutter:
    sdk: flutter


  # The following adds the Cupertino Icons font to your application.
  # Use with the CupertinoIcons class for iOS style icons.
  cupertino_icons: ^1.0.0

dev_dependencies:
  flutter_test:
    sdk: flutter

# For information on the generic Dart part of this file, see the
# following page: https://dart.dev/tools/pub/pubspec

# The following section is specific to Flutter.
flutter:

  # The following line ensures that the Material Icons font is
  # included with your application, so that you can use the icons in
  # the material Icons class.
  uses-material-design: true

  # To add assets to your application, add an assets section, like this:
  assets:
    - images/a_dot_burr.jpeg
    - images/a_dot_ham.jpeg

  # An image asset can refer to one or more resolution-specific "variants", see
  # https://flutter.dev/to/resolution-aware-images

  # For details regarding adding assets from package dependencies, see
  # https://flutter.dev/to/asset-from-package

  # To add custom fonts to your application, add a fonts section here,
  # in this "flutter" section. Each entry in this list should have a
  # "family" key with the font family name, and a "fonts" key with a
  # list giving the asset and other descriptors for the font. For
  # example:
  # fonts:
  #   - family: Schyler
  #     fonts:
  #       - asset: fonts/Schyler-Regular.ttf
  #       - asset: fonts/Schyler-Italic.ttf
  #         style: italic
  #   - family: Trajan Pro
  #     fonts:
  #       - asset: fonts/TrajanPro.ttf
  #       - asset: fonts/TrajanPro_Bold.ttf
  #         weight: 700
  #
  # For details regarding fonts from package dependencies,
  # see https://flutter.dev/to/font-from-package

''';
}
