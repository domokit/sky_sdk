// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../base/platform.dart';
import '../doctor_validator.dart';
import '../features.dart';

class WebWorkflow extends Workflow {
  const WebWorkflow({required Platform platform, required FeatureFlags featureFlags})
    : _platform = platform,
      _featureFlags = featureFlags;

  final Platform _platform;
  final FeatureFlags _featureFlags;

  @override
  bool get appliesToHostPlatform =>
      _featureFlags.isWebEnabled && (_platform.isWindows || _platform.isMacOS || _platform.isLinux);

  @override
  bool get canLaunchDevices => _featureFlags.isWebEnabled;

  @override
  bool get canListDevices => _featureFlags.isWebEnabled;

  @override
  bool get canListEmulators => false;
}
