// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:meta/meta.dart';

import '../typed_json.dart';
import 'target_platform.dart';

/// A representation of the parsed device from the `flutter devices` command.
///
/// See <https://github.com/flutter/flutter/blob/9441f9d48fce1d0b425628731dd6ecab8c8b0826/packages/flutter_tools/lib/src/device.dart#L892-L911>.
@immutable
final class Device {
  /// Creates a device with the given [name], [id], and [targetPlatform].
  const Device({required this.name, required this.id, required this.targetPlatform});

  /// Parses a device from the given [json].
  factory Device.fromJson(Map<String, Object?> json) {
    return JsonObject(json).map((o) {
      return Device(
        name: o.string('name'),
        id: o.string('id'),
        targetPlatform: TargetPlatform.parse(o.string('targetPlatform')),
      );
    });
  }

  /// Name of the device.
  final String name;

  /// Identifier of the device.
  final String id;

  /// Target platform of the device.
  final TargetPlatform targetPlatform;

  @override
  bool operator ==(Object other) {
    return other is Device &&
        other.name == name &&
        other.id == id &&
        other.targetPlatform == targetPlatform;
  }

  @override
  int get hashCode => Object.hash(name, id, targetPlatform);

  /// Converts this device to a JSON object, for use within tests.
  @visibleForTesting
  JsonObject toJson() {
    return JsonObject({'name': name, 'id': id, 'targetPlatform': targetPlatform.identifier});
  }

  @override
  String toString() {
    return 'Device ${const JsonEncoder.withIndent('  ').convert(this)}';
  }
}
