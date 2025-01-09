// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'base/file_system.dart';
import 'base/utils.dart';
import 'platform_plugins.dart';
import 'project.dart';

/// Represents a CMake-based sub-project.
///
/// This defines interfaces common to Windows and Linux projects.
abstract class CmakeBasedProject {
  /// The parent of this project.
  FlutterProject get parent;

  /// Whether the subproject (either Windows or Linux) exists in the Flutter project.
  bool existsSync();

  /// The native project CMake specification.
  File get cmakeFile;

  /// Contains definitions for the Flutter library and the tool.
  File get managedCmakeFile;

  /// Contains definitions for FLUTTER_ROOT, LOCAL_ENGINE, and more flags for
  /// the build.
  File get generatedCmakeConfigFile;

  /// Included CMake with rules and variables for plugin builds.
  File get generatedPluginCmakeFile;

  /// The directory to write plugin symlinks.
  Directory get pluginSymlinkDirectory;
}

/// The Windows sub project.
class WindowsProject extends FlutterProjectPlatform implements CmakeBasedProject {
  WindowsProject.fromFlutter(this.parent);

  @override
  final FlutterProject parent;

  @override
  String get pluginConfigKey => WindowsPlugin.kConfigKey;

  String get _childDirectory => 'windows';

  @override
  bool existsSync() => _editableDirectory.existsSync() && cmakeFile.existsSync();

  @override
  File get cmakeFile => _editableDirectory.childFile('CMakeLists.txt');

  @override
  File get managedCmakeFile => managedDirectory.childFile('CMakeLists.txt');

  @override
  File get generatedCmakeConfigFile => ephemeralDirectory.childFile('generated_config.cmake');

  @override
  File get generatedPluginCmakeFile => managedDirectory.childFile('generated_plugins.cmake');

  /// The native entrypoint's CMake specification.
  File get runnerCmakeFile => runnerDirectory.childFile('CMakeLists.txt');

  /// The native entrypoint's file that adds Flutter to the window.
  File get runnerFlutterWindowFile => runnerDirectory.childFile('flutter_window.cpp');

  /// The native entrypoint's resource file. Used to configure things
  /// like the application icon, name, and version.
  File get runnerResourceFile => runnerDirectory.childFile('Runner.rc');

  @override
  Directory get pluginSymlinkDirectory => ephemeralDirectory.childDirectory('.plugin_symlinks');

  Directory get _editableDirectory => parent.directory.childDirectory(_childDirectory);

  /// The directory in the project that is managed by Flutter. As much as
  /// possible, files that are edited by Flutter tooling after initial project
  /// creation should live here.
  Directory get managedDirectory => _editableDirectory.childDirectory('flutter');

  /// The subdirectory of [managedDirectory] that contains files that are
  /// generated on the fly. All generated files that are not intended to be
  /// checked in should live here.
  Directory get ephemeralDirectory => managedDirectory.childDirectory('ephemeral');

  /// The directory in the project that is owned by the app. As much as
  /// possible, Flutter tooling should not edit files in this directory after
  /// initial project creation.
  Directory get runnerDirectory => _editableDirectory.childDirectory('runner');

  Future<void> ensureReadyForPlatformSpecificTooling() async {}
}

/// The Linux sub project.
class LinuxProject extends FlutterProjectPlatform implements CmakeBasedProject {
  LinuxProject.fromFlutter(this.parent);

  @override
  final FlutterProject parent;

  @override
  String get pluginConfigKey => LinuxPlugin.kConfigKey;

  static final RegExp _applicationIdPattern = RegExp(
    r'''^\s*set\s*\(\s*APPLICATION_ID\s*"(.*)"\s*\)\s*$''',
  );

  Directory get _editableDirectory => parent.directory.childDirectory('linux');

  /// The directory in the project that is managed by Flutter. As much as
  /// possible, files that are edited by Flutter tooling after initial project
  /// creation should live here.
  Directory get managedDirectory => _editableDirectory.childDirectory('flutter');

  /// The subdirectory of [managedDirectory] that contains files that are
  /// generated on the fly. All generated files that are not intended to be
  /// checked in should live here.
  Directory get ephemeralDirectory => managedDirectory.childDirectory('ephemeral');

  @override
  bool existsSync() => _editableDirectory.existsSync();

  @override
  File get cmakeFile => _editableDirectory.childFile('CMakeLists.txt');

  @override
  File get managedCmakeFile => managedDirectory.childFile('CMakeLists.txt');

  @override
  File get generatedCmakeConfigFile => ephemeralDirectory.childFile('generated_config.cmake');

  @override
  File get generatedPluginCmakeFile => managedDirectory.childFile('generated_plugins.cmake');

  @override
  Directory get pluginSymlinkDirectory => ephemeralDirectory.childDirectory('.plugin_symlinks');

  Future<void> ensureReadyForPlatformSpecificTooling() async {}

  String? get applicationId {
    return firstMatchInFile(cmakeFile, _applicationIdPattern)?.group(1);
  }
}
