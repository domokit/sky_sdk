// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';

import '../doctor_validator.dart';
import '../features.dart';

/// The custom-devices-specific implementation of a [Workflow].
///
/// Will apply to the host platform / be able to launch & list devices only if
/// the custom devices feature is enabled in the featureFlags argument.
///
/// Can't list emulators at all.
@immutable
class CustomDeviceWorkflow implements Workflow {
  const CustomDeviceWorkflow({required FeatureFlags featureFlags}) : _featureFlags = featureFlags;

  final FeatureFlags _featureFlags;

  @override
  bool get appliesToHostPlatform => _featureFlags.areCustomDevicesEnabled;

  @override
  bool get canLaunchDevices => _featureFlags.areCustomDevicesEnabled;

  @override
  bool get canListDevices => _featureFlags.areCustomDevicesEnabled;

  @override
  bool get canListEmulators => false;
}
