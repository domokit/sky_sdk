// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../android/android_device.dart';
import '../application_package.dart';
import '../base/common.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../device.dart';
import '../globals.dart' as globals;
import '../runner/flutter_command.dart';

class InstallCommand extends FlutterCommand with DeviceBasedDevelopmentArtifacts {
  InstallCommand({required bool verboseHelp}) {
    addBuildModeFlags(verboseHelp: verboseHelp);
    requiresPubspecYaml();
    usesApplicationBinaryOption();
    usesDeviceTimeoutOption();
    usesDeviceConnectionOption();
    usesDeviceUserOption();
    usesFlavorOption();
    argParser.addFlag(
      'uninstall-only',
      help: 'Uninstall the app if already on the device. Skip install.',
    );
  }

  @override
  final String name = 'install';

  @override
  final String description = 'Install a Flutter app on an attached device.';

  @override
  final String category = FlutterCommandCategory.tools;

  @override
  bool get refreshWirelessDevices => true;

  Device? device;

  bool get uninstallOnly => boolArg('uninstall-only');
  String? get userIdentifier => stringArg(FlutterOptions.kDeviceUser);

  String? get _applicationBinaryPath => stringArg(FlutterOptions.kUseApplicationBinary);
  File? get _applicationBinary =>
      _applicationBinaryPath == null ? null : globals.fs.file(_applicationBinaryPath);

  @override
  Future<void> validateCommand() async {
    await super.validateCommand();
    device = await findTargetDevice();
    if (device == null) {
      throwToolExit('No target device found');
    }
    if (userIdentifier != null && device is! AndroidDevice) {
      throwToolExit('--${FlutterOptions.kDeviceUser} is only supported for Android');
    }
    if (_applicationBinaryPath != null && !(_applicationBinary?.existsSync() ?? true)) {
      throwToolExit('Prebuilt binary $_applicationBinaryPath does not exist');
    }
  }

  @override
  Future<FlutterCommandResult> runCommand() async {
    final Device targetDevice = device!;
    final ApplicationPackage? package = await applicationPackages?.getPackageForPlatform(
      await targetDevice.targetPlatform,
      applicationBinary: _applicationBinary,
      buildInfo: await getBuildInfo(),
    );
    if (package == null) {
      throwToolExit('Could not find or build package');
    }

    if (uninstallOnly) {
      await _uninstallApp(package, targetDevice);
    } else {
      await _installApp(package, targetDevice);
    }
    return FlutterCommandResult.success();
  }

  Future<void> _uninstallApp(ApplicationPackage package, Device device) async {
    if (await device.isAppInstalled(package, userIdentifier: userIdentifier)) {
      globals.printStatus('Uninstalling $package from $device...');
      if (!await device.uninstallApp(package, userIdentifier: userIdentifier)) {
        globals.printError('Uninstalling old version failed');
      }
    } else {
      globals.printStatus('$package not found on $device, skipping uninstall');
    }
  }

  Future<void> _installApp(ApplicationPackage package, Device device) async {
    globals.printStatus('Installing $package to $device...');

    if (!await installApp(device, package, userIdentifier: userIdentifier)) {
      throwToolExit('Install failed');
    }
  }
}

Future<bool> installApp(
  Device device,
  ApplicationPackage package, {
  String? userIdentifier,
  bool uninstall = true,
}) async {
  try {
    if (uninstall && await device.isAppInstalled(package, userIdentifier: userIdentifier)) {
      globals.printStatus('Uninstalling old version...');
      if (!await device.uninstallApp(package, userIdentifier: userIdentifier)) {
        globals.printWarning('Warning: uninstalling old version failed');
      }
    }
  } on ProcessException catch (e) {
    globals.printError('Error accessing device ${device.id}:\n${e.message}');
  }

  return device.installApp(package, userIdentifier: userIdentifier);
}
