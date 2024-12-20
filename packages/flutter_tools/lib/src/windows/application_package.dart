// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:archive/archive.dart';

import '../application_package.dart';
import '../base/file_system.dart';
import '../base/utils.dart';
import '../build_info.dart';
import '../cmake.dart';
import '../cmake_project.dart';
import '../globals.dart' as globals;

abstract class WindowsApp extends ApplicationPackage {
  WindowsApp({required String projectBundleId}) : super(id: projectBundleId);

  /// Creates a new [WindowsApp] from a windows sub project.
  factory WindowsApp.fromWindowsProject(WindowsProject project) {
    return BuildableWindowsApp(project: project);
  }

  /// Creates a new [WindowsApp] from an existing executable or a zip archive.
  ///
  /// `applicationBinary` is the path to the executable or the zipped archive.
  static WindowsApp? fromPrebuiltApp(FileSystemEntity applicationBinary) {
    if (!applicationBinary.existsSync()) {
      globals.printError('File "${applicationBinary.path}" does not exist.');
      return null;
    }

    if (applicationBinary.path.endsWith('.exe')) {
      return PrebuiltWindowsApp(
        executable: applicationBinary.path,
        applicationPackage: applicationBinary,
      );
    }

    if (!applicationBinary.path.endsWith('.zip')) {
      // Unknown file type
      globals.printError('Unknown windows application type.');
      return null;
    }

    // Try to unpack as a zip.
    final Directory tempDir = globals.fs.systemTempDirectory.createTempSync('flutter_app.');
    try {
      globals.os.unzip(globals.fs.file(applicationBinary), tempDir);
    } on ArchiveException {
      globals.printError('Invalid prebuilt Windows app. Unable to extract from archive.');
      return null;
    }
    final List<FileSystemEntity> exeFilesFound = <FileSystemEntity>[
      for (final FileSystemEntity file in tempDir.listSync())
        if (file.basename.endsWith('.exe')) file,
    ];

    if (exeFilesFound.isEmpty) {
      globals.printError('Cannot find .exe files in the zip archive.');
      return null;
    }

    if (exeFilesFound.length > 1) {
      globals.printError('Archive "${applicationBinary.path}" contains more than one .exe files.');
      return null;
    }

    return PrebuiltWindowsApp(
      executable: exeFilesFound.single.path,
      applicationPackage: applicationBinary,
    );
  }

  @override
  String get displayName => id;

  String executable(BuildMode buildMode, TargetPlatform targetPlatform);
}

class PrebuiltWindowsApp extends WindowsApp implements PrebuiltApplicationPackage {
  PrebuiltWindowsApp({required String executable, required this.applicationPackage})
    : _executable = executable,
      super(projectBundleId: executable);

  final String _executable;

  @override
  String executable(BuildMode buildMode, TargetPlatform targetPlatform) => _executable;

  @override
  String get name => _executable;

  @override
  final FileSystemEntity applicationPackage;
}

class BuildableWindowsApp extends WindowsApp {
  BuildableWindowsApp({required this.project})
    : super(projectBundleId: project.parent.manifest.appName);

  final WindowsProject project;

  @override
  String executable(BuildMode buildMode, TargetPlatform targetPlatform) {
    final String? binaryName = getCmakeExecutableName(project);
    return globals.fs.path.join(
      getWindowsBuildDirectory(targetPlatform),
      'runner',
      sentenceCase(buildMode.cliName),
      '$binaryName.exe',
    );
  }

  @override
  String get name => project.parent.manifest.appName;
}
