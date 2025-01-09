// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:process/process.dart';

import '../base/common.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/platform.dart';
import '../base/process.dart';
import '../base/user_messages.dart';
import '../device.dart';
import 'adb.dart';
import 'android_device.dart';
import 'android_sdk.dart';
import 'android_workflow.dart';

/// Device discovery for Android physical devices and emulators.
///
/// This class primarily delegates to the `adb` command line tool provided by
/// the Android SDK to discover instances of connected android devices.
///
/// See also:
///   * [AndroidDevice], the type of discovered device.
class AndroidDevices extends PollingDeviceDiscovery {
  AndroidDevices({
    required AndroidWorkflow androidWorkflow,
    required ProcessManager processManager,
    required Logger logger,
    AndroidSdk? androidSdk,
    required FileSystem fileSystem,
    required Platform platform,
    required UserMessages userMessages,
  }) : _androidWorkflow = androidWorkflow,
       _androidSdk = androidSdk,
       _processUtils = ProcessUtils(logger: logger, processManager: processManager),
       _processManager = processManager,
       _logger = logger,
       _fileSystem = fileSystem,
       _platform = platform,
       _userMessages = userMessages,
       super('Android devices');

  final AndroidWorkflow _androidWorkflow;
  final ProcessUtils _processUtils;
  final AndroidSdk? _androidSdk;
  final ProcessManager _processManager;
  final Logger _logger;
  final FileSystem _fileSystem;
  final Platform _platform;
  final UserMessages _userMessages;

  @override
  bool get supportsPlatform => _androidWorkflow.appliesToHostPlatform;

  @override
  bool get canListAnything => _androidWorkflow.canListDevices;

  @override
  Future<List<Device>> pollingGetDevices({Duration? timeout}) async {
    if (_doesNotHaveAdb()) {
      return <AndroidDevice>[];
    }
    String text;
    try {
      text =
          (await _processUtils.run(<String>[
            _androidSdk!.adbPath!,
            'devices',
            '-l',
          ], throwOnError: true)).stdout.trim();
    } on ProcessException catch (exception) {
      throwToolExit(
        'Unable to run "adb", check your Android SDK installation and '
        '$kAndroidHome environment variable: ${exception.executable}\n'
        'Error details: ${exception.message}',
      );
    }
    final List<AndroidDevice> devices = <AndroidDevice>[];
    _parseADBDeviceOutput(text, devices: devices);
    return devices;
  }

  @override
  Future<List<String>> getDiagnostics() async {
    if (_doesNotHaveAdb()) {
      return <String>[];
    }

    final RunResult result = await _processUtils.run(<String>[
      _androidSdk!.adbPath!,
      'devices',
      '-l',
    ]);
    if (result.exitCode != 0) {
      return <String>[];
    }
    final List<String> diagnostics = <String>[];
    _parseADBDeviceOutput(result.stdout, diagnostics: diagnostics);
    return diagnostics;
  }

  bool _doesNotHaveAdb() {
    return _androidSdk == null ||
        _androidSdk.adbPath == null ||
        !_processManager.canRun(_androidSdk.adbPath);
  }

  // 015d172c98400a03       device usb:340787200X product:nakasi model:Nexus_7 device:grouper
  static final RegExp _kDeviceRegex = RegExp(r'^(\S+)\s+(\S+)(.*)');

  /// Parse the given `adb devices` output in [text], and fill out the given list
  /// of devices and possible device issue diagnostics. Either argument can be null,
  /// in which case information for that parameter won't be populated.
  void _parseADBDeviceOutput(
    String text, {
    List<AndroidDevice>? devices,
    List<String>? diagnostics,
  }) {
    // Check for error messages from adb
    if (!text.contains('List of devices')) {
      diagnostics?.add(text);
      return;
    }

    for (final String line in text.trim().split('\n')) {
      // Skip lines like: * daemon started successfully *
      if (line.startsWith('* daemon ')) {
        continue;
      }

      // Skip lines about adb server and client version not matching
      if (line.startsWith(RegExp(r'adb server (version|is out of date)'))) {
        diagnostics?.add(line);
        continue;
      }

      if (line.startsWith('List of devices')) {
        continue;
      }

      if (_kDeviceRegex.hasMatch(line)) {
        final Match match = _kDeviceRegex.firstMatch(line)!;

        final String deviceID = match[1]!;
        final String deviceState = match[2]!;
        String? rest = match[3];

        final Map<String, String> info = <String, String>{};
        if (rest != null && rest.isNotEmpty) {
          rest = rest.trim();
          for (final String data in rest.split(' ')) {
            if (data.contains(':')) {
              final List<String> fields = data.split(':');
              info[fields[0]] = fields[1];
            }
          }
        }

        final String? model = info['model'];
        if (model != null) {
          info['model'] = cleanAdbDeviceName(model);
        }

        switch (deviceState) {
          case 'unauthorized':
            diagnostics?.add(
              'Device $deviceID is not authorized.\n'
              'You might need to check your device for an authorization dialog.',
            );
          case 'offline':
            diagnostics?.add('Device $deviceID is offline.');
          default:
            devices?.add(
              AndroidDevice(
                deviceID,
                productID: info['product'],
                modelID: info['model'] ?? deviceID,
                deviceCodeName: info['device'],
                androidSdk: _androidSdk!,
                fileSystem: _fileSystem,
                logger: _logger,
                platform: _platform,
                processManager: _processManager,
              ),
            );
        }
      } else {
        diagnostics?.add(
          'Unexpected failure parsing device information from adb output:\n'
          '$line\n'
          '${_userMessages.flutterToolBugInstructions}',
        );
      }
    }
  }

  @override
  List<String> get wellKnownIds => const <String>[];
}
