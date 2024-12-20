// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../base/context.dart';
import '../base/platform.dart';
import '../doctor_validator.dart';
import '../features.dart';

/// The [WindowsWorkflow] instance.
WindowsWorkflow? get windowsWorkflow => context.get<WindowsWorkflow>();

/// The Windows-specific implementation of a [Workflow].
///
/// This workflow requires the host machine to be Windows, and the Windows
/// desktop configuration setting to be enabled.
class WindowsWorkflow implements Workflow {
  const WindowsWorkflow({required Platform platform, required FeatureFlags featureFlags})
    : _platform = platform,
      _featureFlags = featureFlags;

  final Platform _platform;
  final FeatureFlags _featureFlags;

  @override
  bool get appliesToHostPlatform => _platform.isWindows && _featureFlags.isWindowsEnabled;

  @override
  bool get canLaunchDevices => _platform.isWindows && _featureFlags.isWindowsEnabled;

  @override
  bool get canListDevices => _platform.isWindows && _featureFlags.isWindowsEnabled;

  @override
  bool get canListEmulators => false;
}
