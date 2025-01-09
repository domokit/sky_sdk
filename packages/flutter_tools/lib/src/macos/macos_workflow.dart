// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../base/platform.dart';
import '../doctor_validator.dart';
import '../features.dart';

/// The macOS-specific implementation of a [Workflow].
class MacOSWorkflow implements Workflow {
  const MacOSWorkflow({required Platform platform, required FeatureFlags featureFlags})
    : _platform = platform,
      _featureFlags = featureFlags;

  final Platform _platform;
  final FeatureFlags _featureFlags;

  @override
  bool get appliesToHostPlatform => _platform.isMacOS && _featureFlags.isMacOSEnabled;

  @override
  bool get canLaunchDevices => _platform.isMacOS && _featureFlags.isMacOSEnabled;

  @override
  bool get canListDevices => _platform.isMacOS && _featureFlags.isMacOSEnabled;

  @override
  bool get canListEmulators => false;
}
