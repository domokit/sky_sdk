// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;
import 'dart:typed_data';

import 'package:meta/meta.dart';
import 'package:process/process.dart';

/// A minimal wrapper around the `adb` command-line tool.
@internal
class Adb {
  const Adb._(this._prefixArgs, this._process);

  /// Creates a new `adb` command runner that uses the `adb` command-line tool.
  ///
  /// If [adbPath] is not provided, the `adb` command is assumed to be in the
  /// system's PATH.
  ///
  /// If [target] is not provided, the target is assumed to be the only running
  /// emulator or connected device.
  ///
  /// For testing, [processManager] can be provided to mock the process manager.
  static Future<Adb> create({
    String? adbPath,
    AndroidDeviceTarget? target,
    @visibleForTesting ProcessManager? processManager,
  }) async {
    target ??= const AndroidDeviceTarget.onlyEmulatorOrDevice();
    final String tool = adbPath ?? 'adb';
    final Adb adb = Adb._(<String>[
      tool,
      ...target._toAdbArgs(),
    ], processManager ?? const LocalProcessManager());
    final (bool connected, String? error) = await adb.isDeviceConnected();
    if (!connected) {
      throw StateError('No device connected: $error');
    }
    return adb;
  }

  Future<AdbStringResult> _runString(List<String> args) async {
    final io.ProcessResult result = await _process.run(<String>[..._prefixArgs, ...args]);
    return AdbStringResult(
      result.stdout as String,
      exitCode: result.exitCode,
      stderr: result.stderr as String,
    );
  }

  Future<AdbBinaryResult> _runBinary(List<String> args) async {
    final io.ProcessResult result = await _process.run(<String>[
      ..._prefixArgs,
      ...args,
    ], stdoutEncoding: null);
    return AdbBinaryResult(
      result.stdout as Uint8List,
      exitCode: result.exitCode,
      stderr: result.stderr as String,
    );
  }

  final List<String> _prefixArgs;
  final ProcessManager _process;

  /// Returns whether the device is currently connected.
  ///
  /// Returns a tuple of a boolean indicating whether the device is connected
  /// and an error message if the device is not connected. If the device is
  /// connected, the error message is `null`.
  Future<(bool connected, String? error)> isDeviceConnected() async {
    final AdbStringResult result = await _runString(<String>['shell', 'echo', 'connected']);
    if (result.exitCode != 0) {
      return (false, result.stderr);
    } else {
      return (true, null);
    }
  }

  /// Takes a screenshot of the device.
  Future<Uint8List> screencap() async {
    final AdbBinaryResult result = await _runBinary(<String>['exec-out', 'screencap', '-p']);
    if (result.exitCode != 0) {
      throw StateError('Failed to take screenshot: ${result.stderr}');
    }
    return result.stdout;
  }

  /// Taps on the screen at the given [x] and [y] coordinates.
  Future<void> tap(int x, int y) async {
    final AdbStringResult result = await _runString(<String>['shell', 'input', 'tap', '$x', '$y']);
    if (result.exitCode != 0) {
      throw StateError('Failed to tap at $x, $y: ${result.stderr}');
    }
  }

  /// Sends the device to the home screen.
  Future<void> sendToHome() async {
    final AdbStringResult result = await _runString(<String>[
      'shell',
      'input',
      'keyevent',
      'KEYCODE_HOME',
    ]);
    if (result.exitCode != 0) {
      throw StateError('Failed to send to home: ${result.stderr}');
    }
  }

  /// Simulate low memory conditions on the device.
  Future<void> trimMemory({required String appName}) async {
    final AdbStringResult result = await _runString(<String>[
      'shell',
      'am',
      'send-trim-memory',
      appName,
      'MODERATE',
    ]);
    if (result.exitCode != 0) {
      throw StateError('Failed to simulate low memory: ${result.stderr}');
    }
  }

  /// Resume a backgrounded app (i.e. after [sendToHome]).
  Future<void> resumeApp({required String appName, String activityName = '.MainActivity'}) async {
    final AdbStringResult result = await _runString(<String>[
      'shell',
      'am',
      'start',
      '-n',
      '$appName/$activityName',
    ]);
    if (result.exitCode != 0) {
      throw StateError('Failed to resume app: ${result.stderr}');
    }
  }

  /// Disable confirmations for immersive mode.
  Future<void> disableImmersiveModeConfirmations() async {
    final AdbStringResult result = await _runString(<String>[
      'shell',
      'settings',
      'put',
      'secure',
      'immersive_mode_confirmations',
      'confirmed',
    ]);
    if (result.exitCode != 0) {
      throw StateError('Failed to disable immersive mode confirmations: ${result.stderr}');
    }
  }

  /// Disable animations on the device.
  Future<void> disableAnimations() async {
    const Map<String, String> settings = <String, String>{
      'show_surface_updates': '1',
      'transition_animation_scale': '0',
      'window_animation_scale': '0',
      'animator_duration_scale': '0',
    };
    for (final MapEntry<String, String> entry in settings.entries) {
      final AdbStringResult result = await _runString(<String>[
        'shell',
        'settings',
        'put',
        'global',
        entry.key,
        entry.value,
      ]);
      if (result.exitCode != 0) {
        throw StateError('Failed to disable animations: ${result.stderr}');
      }
    }
  }
}

/// Possible results of an `adb` command.
@internal
sealed class AdbResult {
  /// Creates a new `adb` result.
  const AdbResult._({this.exitCode = 0, this.stderr = ''});

  /// The exit code of the `adb` command.
  final int exitCode;

  /// The standard error output of the `adb` command.
  final String stderr;
}

@internal
final class AdbStringResult extends AdbResult {
  AdbStringResult(this.stdout, {super.stderr, super.exitCode}) : super._();

  /// The standard output of the `adb` command.
  final String stdout;
}

@internal
final class AdbBinaryResult extends AdbResult {
  AdbBinaryResult(this.stdout, {super.stderr, super.exitCode}) : super._();

  /// The standard output of the `adb` command.
  final Uint8List stdout;
}

/// Represents a target device running Android.
sealed class AndroidDeviceTarget {
  /// Represents a device with the given [serialNumber].
  ///
  /// This is the recommended way to target a specific device, and uses the
  /// device's serial number, as reported by `adb devices`, to identify the
  /// device:
  ///
  /// ```sh
  /// $ adb devices
  /// List of devices attached
  /// emulator-5554   device
  /// ```
  ///
  /// In this example, the serial number is `emulator-5554`:
  ///
  /// ```dart
  /// const AndroidDeviceTarget target = AndroidDeviceTarget.bySerial('emulator-5554');
  /// ```
  const factory AndroidDeviceTarget.bySerial(String serialNumber) = _SerialDeviceTarget;

  /// Represents the only running emulator _or_ connected device.
  ///
  /// This is equivalent to using `adb` without `-e`, `-d`, or `-s`.
  const factory AndroidDeviceTarget.onlyEmulatorOrDevice() = _SingleAnyTarget;

  /// Represents the only running emulator on the host machine.
  ///
  /// This is equivalent to using `adb -e`, a _single_ emulator must be running.
  const factory AndroidDeviceTarget.onlyEmulator() = _SingleEmulatorTarget;

  /// Represents the only connected device on the host machine.
  ///
  /// This is equivalent to using `adb -d`, a _single_ device must be connected.
  const factory AndroidDeviceTarget.onlyDevice() = _SingleDeviceTarget;

  /// Returns the arguments to pass to `adb` to target this device.
  List<String> _toAdbArgs();
}

final class _SerialDeviceTarget implements AndroidDeviceTarget {
  const _SerialDeviceTarget(this.serialNumber);
  final String serialNumber;

  @override
  List<String> _toAdbArgs() => <String>['-s', serialNumber];
}

final class _SingleEmulatorTarget implements AndroidDeviceTarget {
  const _SingleEmulatorTarget();

  @override
  List<String> _toAdbArgs() => const <String>['-e'];
}

final class _SingleDeviceTarget implements AndroidDeviceTarget {
  const _SingleDeviceTarget();

  @override
  List<String> _toAdbArgs() => const <String>['-d'];
}

final class _SingleAnyTarget implements AndroidDeviceTarget {
  const _SingleAnyTarget();

  @override
  List<String> _toAdbArgs() => const <String>[];
}

/// Represents the possible screen orientations on Android.
enum AdbUserRotation {
  /// Portrait orientation.
  portrait,

  /// Landscape orientation.
  landscape,

  /// Reverse portrait orientation, i.e., portrait upside down.
  reversePortrait,

  /// Reverse landscape orientation, i.e., landscape upside down.
  reverseLandscape,
}
