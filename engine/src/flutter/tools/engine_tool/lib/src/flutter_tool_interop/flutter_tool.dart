// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import '../environment.dart';
import '../logger.dart';
import 'device.dart';

/// An interface to the `flutter` command-line tool.
interface class FlutterTool {
  /// Creates a new Flutter tool interface using the given [environment].
  ///
  /// A `flutter`[^1] binary must exist on the `PATH`.
  ///
  /// [^1]: On Windows, the binary is named `flutter.bat`.
  const FlutterTool.fromEnvironment(this._environment);
  final Environment _environment;

  String get _toolPath {
    return _environment.platform.isWindows ? 'flutter.bat' : 'flutter';
  }

  /// Returns a list of devices available via the `flutter devices` command.
  Future<List<Device>> devices() async {
    final result = await _environment.processRunner.runProcess([
      _toolPath,
      'devices',
      '--machine',
    ], failOk: true);
    if (result.exitCode != 0) {
      throw FatalError(
        'Failed to run `flutter devices --machine`.\n\n'
        'EXITED: ${result.exitCode}\n'
        'STDOUT:\n${result.stdout}\n'
        'STDERR:\n${result.stderr}',
      );
    }
    final List<Object?> jsonDevices;
    try {
      jsonDevices = jsonDecode(result.stdout) as List<Object?>;
    } on FormatException catch (e) {
      throw FatalError(
        'Failed to parse `flutter devices --machine` output:\n$e\n\n'
        'STDOUT:\n${result.stdout}',
      );
    }
    final parsedDevices = <Device>[];
    for (final device in jsonDevices) {
      if (device is! Map<String, Object?>) {
        _environment.logger.error('Skipping device: Expected a JSON Object, but got:\n$device');
        continue;
      }
      try {
        parsedDevices.add(Device.fromJson(device));
      } on FormatException catch (e) {
        _environment.logger.error('Skipping device: Failed to parse JSON Object:\n$device\n\n$e');
      }
    }
    return parsedDevices;
  }
}
