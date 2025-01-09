// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:unified_analytics/unified_analytics.dart';

import '../base/config.dart';
import '../base/io.dart';
import '../features.dart';
import '../globals.dart' as globals;
import '../version.dart';

/// This function is called from within the context runner to perform
/// checks that are necessary for determining if a no-op version of
/// [Analytics] gets returned.
///
/// When [enableAsserts] is set to `true`, various assert statements
/// will be enabled to ensure usage of this class is within GA4 limitations.
///
/// For testing purposes, pass in a [FakeAnalytics] instance initialized with
/// an in-memory [FileSystem] to prevent writing to disk.
Analytics getAnalytics({
  required bool runningOnBot,
  required FlutterVersion flutterVersion,
  required Map<String, String> environment,
  required String? clientIde,
  required Config config,
  bool enableAsserts = false,
  FakeAnalytics? analyticsOverride,
}) {
  final String version = flutterVersion.getVersionString(redactUnknownBranches: true);
  final bool suppressEnvFlag = environment['FLUTTER_SUPPRESS_ANALYTICS']?.toLowerCase() == 'true';

  if ( // Ignore local user branches.
  version.startsWith('[user-branch]') ||
      // Many CI systems don't do a full git checkout.
      version.endsWith('/unknown') ||
      // Ignore bots.
      runningOnBot ||
      // Ignore when suppressed by FLUTTER_SUPPRESS_ANALYTICS.
      suppressEnvFlag) {
    return const NoOpAnalytics();
  }

  // Providing an override of the [Analytics] instance is preferred when
  // running tests for this function to prevent writing to the filesystem
  if (analyticsOverride != null) {
    return analyticsOverride;
  }

  return Analytics(
    tool: DashTool.flutterTool,
    flutterChannel: flutterVersion.channel,
    flutterVersion: flutterVersion.frameworkVersion,
    dartVersion: flutterVersion.dartSdkVersion,
    enableAsserts: enableAsserts,
    clientIde: clientIde,
    enabledFeatures: getEnabledFeatures(config),
  );
}

/// Uses the [Config] object to get enabled features.
String? getEnabledFeatures(Config config) {
  // Create string with all enabled features to send as user property
  final Iterable<Feature> enabledFeatures = allFeatures.where((Feature feature) {
    final String? configSetting = feature.configSetting;
    return configSetting != null && config.getValue(configSetting) == true;
  });
  return enabledFeatures.isNotEmpty
      ? enabledFeatures.map((Feature feature) => feature.configSetting).join(',')
      : null;
}

/// Function to safely grab the max rss from [ProcessInfo].
int? getMaxRss(ProcessInfo processInfo) {
  try {
    return globals.processInfo.maxRss;
  } on Exception catch (error) {
    globals.printTrace('Querying maxRss failed with error: $error');
  }
  return null;
}
