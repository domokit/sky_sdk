// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;
import 'dart:typed_data';

import 'package:flutter_driver/flutter_driver.dart';
import 'package:meta/meta.dart';
import 'package:path/path.dart' as p;

import '../../../native_driver.dart';
import '../../common.dart';
import 'adb.dart';

/// Drives an Android device or emulator that is running a Flutter application.
final class AndroidNativeDriver implements NativeDriver {
  /// Assumes a connected device or emulator via [adb].
  ///
  /// This constructor is intended for testing purposes only.
  @visibleForTesting
  AndroidNativeDriver.forTesting({
    required Adb adb,
    required FlutterDriver driver,
    required io.Directory tempDirectory,
  }) : this._(adb, driver, tempDirectory);

  AndroidNativeDriver._(this._adb, this._driver, this._tmpDir);
  final Adb _adb;
  final FlutterDriver _driver;
  final io.Directory _tmpDir;

  /// Connects to a device or emulator identified by [target], which defaults to
  /// the only running emulator or connected device.
  ///
  /// If [adbPath] is not provided, the `adb` command is assumed to be in the
  /// system's PATH.
  ///
  /// If [tempDirectory] is not provided, a temporary directory will be created
  /// in the system's temporary directory.
  static Future<AndroidNativeDriver> connect(
    FlutterDriver driver, {
    AndroidDeviceTarget? target,
    String? adbPath,
    io.Directory? tempDirectory,
  }) async {
    final Adb adb = await Adb.create(adbPath: adbPath, target: target);
    tempDirectory ??= io.Directory.systemTemp.createTempSync('native_driver.');
    final AndroidNativeDriver nativeDriver = AndroidNativeDriver.forTesting(
      adb: adb,
      driver: driver,
      tempDirectory: tempDirectory,
    );
    await nativeDriver.ping();
    return nativeDriver;
  }

  @override
  Future<void> close() async {
    await _tmpDir.delete(recursive: true);
  }

  @override
  Future<void> configureForScreenshotTesting() async {
    await _adb.disableImmersiveModeConfirmations();
    await _adb.disableAnimations();
  }

  @override
  Future<void> tap(NativeFinder finder) async {
    await _driver.sendCommand(NativeCommand.tap(finder));
  }

  @override
  Future<int> get sdkVersion async {
    final Map<String, Object?> result = await _driver.sendCommand(NativeCommand.getSdkVersion);
    return result['version']! as int;
  }

  @override
  Future<bool> get isEmulator async {
    final Map<String, Object?> result = await _driver.sendCommand(NativeCommand.getIsEmulator);
    return result['emulator']! as bool;
  }

  /// Waits for 2 seconds before completing.
  ///
  /// There is no perfect way, outside of polling, to know when the device is
  /// "stable" or has "stabilized" after a state change. The way that commands
  /// such as [FlutterDriver.screenshot] handle this is to wait for 2 seconds
  /// as a baseline, and then proceed with the command.
  static Future<void> _waitFor2s() async {
    await Future<void>.delayed(const Duration(seconds: 2));
  }

  @override
  Future<NativeScreenshot> screenshot() async {
    // Identical wait to what `FlutterDriver.screenshot` does.
    await _waitFor2s();
    return _AdbScreencap(await _adb.screencap(), _tmpDir);
  }

  @override
  Future<Duration> ping() async {
    final Stopwatch stopwatch = Stopwatch()..start();
    await _driver.sendCommand(NativeCommand.ping);
    return stopwatch.elapsed;
  }

  @override
  Future<void> rotateToLandscape() async {
    _driver.sendCommand(NativeCommand.rotateLandscape);
  }

  @override
  Future<void> rotateResetDefault() async {
    _driver.sendCommand(NativeCommand.rotateDefault);
  }

  /// Background the app by pressing the home button.
  Future<void> backgroundApp() async {
    await _adb.sendToHome();
    await _waitFor2s();
  }

  /// Resumes the app by selecting it from the recent apps list.
  Future<void> resumeApp({required String appName}) async {
    await _adb.resumeApp(appName: appName);
    await _waitFor2s();
  }

  /// Send a trim memory signal to the app to force it to release memory.
  Future<void> simulateLowMemory({required String appName}) async {
    await _adb.trimMemory(appName: appName);
    await _waitFor2s();
  }
}

final class _AdbScreencap implements NativeScreenshot {
  const _AdbScreencap(this._bytes, this._tmpDir);

  /// Raw bytes of the screenshot in PNG format.
  final Uint8List _bytes;

  /// Temporary directory to use as a default location for saving.
  final io.Directory _tmpDir;

  static int _lastScreenshotId = 0;

  @override
  Future<String> saveAs([String? path]) async {
    final int id = _lastScreenshotId++;
    path ??= p.join(_tmpDir.path, 'screenshot_$id.png');
    await io.File(path).writeAsBytes(_bytes);
    return path;
  }

  @override
  Future<Uint8List> readAsBytes() async => _bytes;
}
