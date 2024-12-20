// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:process/process.dart';

import 'android/android_sdk.dart';
import 'android/application_package.dart';
import 'application_package.dart';
import 'base/file_system.dart';
import 'base/logger.dart';
import 'base/process.dart';
import 'base/user_messages.dart';
import 'build_info.dart';
import 'globals.dart' as globals;
import 'ios/application_package.dart';
import 'linux/application_package.dart';
import 'macos/application_package.dart';
import 'project.dart';
import 'tester/flutter_tester.dart';
import 'web/web_device.dart';
import 'windows/application_package.dart';

/// A package factory that supports all Flutter target platforms.
class FlutterApplicationPackageFactory extends ApplicationPackageFactory {
  FlutterApplicationPackageFactory({
    required AndroidSdk? androidSdk,
    required ProcessManager processManager,
    required Logger logger,
    required UserMessages userMessages,
    required FileSystem fileSystem,
  }) : _androidSdk = androidSdk,
       _processManager = processManager,
       _logger = logger,
       _userMessages = userMessages,
       _fileSystem = fileSystem,
       _processUtils = ProcessUtils(logger: logger, processManager: processManager);

  final AndroidSdk? _androidSdk;
  final ProcessManager _processManager;
  final Logger _logger;
  final ProcessUtils _processUtils;
  final UserMessages _userMessages;
  final FileSystem _fileSystem;

  @override
  Future<ApplicationPackage?> getPackageForPlatform(
    TargetPlatform platform, {
    BuildInfo? buildInfo,
    File? applicationBinary,
  }) async {
    switch (platform) {
      case TargetPlatform.android:
      case TargetPlatform.android_arm:
      case TargetPlatform.android_arm64:
      case TargetPlatform.android_x64:
      case TargetPlatform.android_x86:
        if (applicationBinary == null) {
          return AndroidApk.fromAndroidProject(
            FlutterProject.current().android,
            processManager: _processManager,
            processUtils: _processUtils,
            logger: _logger,
            androidSdk: _androidSdk,
            userMessages: _userMessages,
            fileSystem: _fileSystem,
            buildInfo: buildInfo,
          );
        }
        return AndroidApk.fromApk(
          applicationBinary,
          processManager: _processManager,
          logger: _logger,
          androidSdk: _androidSdk!,
          userMessages: _userMessages,
          processUtils: _processUtils,
        );
      case TargetPlatform.ios:
        return applicationBinary == null
            ? await IOSApp.fromIosProject(FlutterProject.current().ios, buildInfo)
            : IOSApp.fromPrebuiltApp(applicationBinary);
      case TargetPlatform.tester:
        return FlutterTesterApp.fromCurrentDirectory(globals.fs);
      case TargetPlatform.darwin:
        return applicationBinary == null
            ? MacOSApp.fromMacOSProject(FlutterProject.current().macos)
            : MacOSApp.fromPrebuiltApp(applicationBinary);
      case TargetPlatform.web_javascript:
        if (!FlutterProject.current().web.existsSync()) {
          return null;
        }
        return WebApplicationPackage(FlutterProject.current());
      case TargetPlatform.linux_x64:
      case TargetPlatform.linux_arm64:
        return applicationBinary == null
            ? LinuxApp.fromLinuxProject(FlutterProject.current().linux)
            : LinuxApp.fromPrebuiltApp(applicationBinary);
      case TargetPlatform.windows_x64:
      case TargetPlatform.windows_arm64:
        return applicationBinary == null
            ? WindowsApp.fromWindowsProject(FlutterProject.current().windows)
            : WindowsApp.fromPrebuiltApp(applicationBinary);
      case TargetPlatform.fuchsia_arm64:
      case TargetPlatform.fuchsia_x64:
        // Unsupported yet.
        throw UnimplementedError();
    }
  }
}
