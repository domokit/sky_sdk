// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../base/platform.dart';
import '../doctor_validator.dart';
import '../features.dart';

/// The windows-specific implementation of a [Workflow].
///
/// This workflow requires the flutter-desktop-embedding as a sibling
/// repository to the flutter repo.
class LinuxWorkflow implements Workflow {
  const LinuxWorkflow({required Platform platform, required FeatureFlags featureFlags})
    : _platform = platform,
      _featureFlags = featureFlags;

  final Platform _platform;
  final FeatureFlags _featureFlags;

  @override
  bool get appliesToHostPlatform => _platform.isLinux && _featureFlags.isLinuxEnabled;

  @override
  bool get canLaunchDevices => _platform.isLinux && _featureFlags.isLinuxEnabled;

  @override
  bool get canListDevices => _platform.isLinux && _featureFlags.isLinuxEnabled;

  @override
  bool get canListEmulators => false;
}
