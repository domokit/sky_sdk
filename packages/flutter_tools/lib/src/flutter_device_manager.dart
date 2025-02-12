// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:process/process.dart';

import 'android/android_device_discovery.dart';
import 'android/android_sdk.dart';
import 'android/android_workflow.dart';
import 'artifacts.dart';
import 'base/file_system.dart';
import 'base/os.dart';
import 'base/platform.dart';
import 'base/user_messages.dart';
import 'custom_devices/custom_device.dart';
import 'custom_devices/custom_devices_config.dart';
import 'device.dart';
import 'features.dart';
import 'ios/devices.dart';
import 'ios/ios_workflow.dart';
import 'ios/simulators.dart';
import 'linux/linux_device.dart';
import 'macos/macos_device.dart';
import 'macos/macos_ipad_device.dart';
import 'macos/macos_workflow.dart';
import 'macos/xcdevice.dart';
import 'native_assets.dart';
import 'tester/flutter_tester.dart';
import 'version.dart';
import 'web/web_device.dart';
import 'windows/windows_device.dart';
import 'windows/windows_workflow.dart';

/// A provider for all of the device discovery instances.
class FlutterDeviceManager extends DeviceManager {
  FlutterDeviceManager({
    required super.logger,
    required Platform platform,
    required ProcessManager processManager,
    required FileSystem fileSystem,
    required AndroidSdk? androidSdk,
    required FeatureFlags featureFlags,
    required IOSSimulatorUtils iosSimulatorUtils,
    required XCDevice xcDevice,
    required AndroidWorkflow androidWorkflow,
    required IOSWorkflow iosWorkflow,
    required FlutterVersion flutterVersion,
    required Artifacts artifacts,
    required MacOSWorkflow macOSWorkflow,
    required UserMessages userMessages,
    required OperatingSystemUtils operatingSystemUtils,
    required WindowsWorkflow windowsWorkflow,
    required CustomDevicesConfig customDevicesConfig,
    required TestCompilerNativeAssetsBuilder? nativeAssetsBuilder,
  }) : deviceDiscoverers = <DeviceDiscovery>[
         AndroidDevices(
           logger: logger,
           androidSdk: androidSdk,
           androidWorkflow: androidWorkflow,
           processManager: processManager,
           fileSystem: fileSystem,
           platform: platform,
           userMessages: userMessages,
         ),
         IOSDevices(
           platform: platform,
           xcdevice: xcDevice,
           iosWorkflow: iosWorkflow,
           logger: logger,
         ),
         IOSSimulators(iosSimulatorUtils: iosSimulatorUtils),
         FlutterTesterDevices(
           fileSystem: fileSystem,
           flutterVersion: flutterVersion,
           processManager: processManager,
           logger: logger,
           artifacts: artifacts,
           nativeAssetsBuilder: nativeAssetsBuilder,
         ),
         MacOSDevices(
           processManager: processManager,
           macOSWorkflow: macOSWorkflow,
           logger: logger,
           platform: platform,
           fileSystem: fileSystem,
           operatingSystemUtils: operatingSystemUtils,
         ),
         MacOSDesignedForIPadDevices(
           processManager: processManager,
           iosWorkflow: iosWorkflow,
           logger: logger,
           platform: platform,
           fileSystem: fileSystem,
           operatingSystemUtils: operatingSystemUtils,
         ),
         LinuxDevices(
           platform: platform,
           featureFlags: featureFlags,
           processManager: processManager,
           logger: logger,
           fileSystem: fileSystem,
           operatingSystemUtils: operatingSystemUtils,
         ),
         WindowsDevices(
           processManager: processManager,
           operatingSystemUtils: operatingSystemUtils,
           logger: logger,
           fileSystem: fileSystem,
           windowsWorkflow: windowsWorkflow,
         ),
         WebDevices(
           featureFlags: featureFlags,
           fileSystem: fileSystem,
           platform: platform,
           processManager: processManager,
           logger: logger,
         ),
         CustomDevices(
           featureFlags: featureFlags,
           processManager: processManager,
           logger: logger,
           config: customDevicesConfig,
         ),
       ];

  @override
  final List<DeviceDiscovery> deviceDiscoverers;
}
