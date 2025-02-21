// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../base/fingerprint.dart';
import '../build_info.dart';
import '../cache.dart';
import '../flutter_plugins.dart';
import '../globals.dart' as globals;
import '../plugins.dart';
import '../project.dart';
import 'swift_package_manager.dart';

/// For a given build, determines whether dependencies have changed since the
/// last call to processPods, then calls processPods with that information.
Future<void> processPodsIfNeeded(
  XcodeBasedProject xcodeProject,
  String buildDirectory,
  BuildMode buildMode, {
  bool forceCocoaPodsOnly = false,
}) async {
  final FlutterProject project = xcodeProject.parent;

  // When using Swift Package Manager, the Podfile may not exist so if there
  // isn't a Podfile, skip processing pods.
  if (xcodeProject.usesSwiftPackageManager &&
      !xcodeProject.podfile.existsSync() &&
      !forceCocoaPodsOnly) {
    return;
  }
  // Ensure that the plugin list is up to date, since hasPlugins relies on it.
  await refreshPluginsList(
    project,
    iosPlatform: project.ios.existsSync(),
    macOSPlatform: project.macos.existsSync(),
    forceCocoaPodsOnly: forceCocoaPodsOnly,

    // TODO(matanlurey): As-per discussion on https://github.com/flutter/flutter/pull/157393
    //  we'll assume that iOS/MacOS builds do not use or rely on the `.flutter-plugins` legacy
    //  file being generated. A better long-term fix would be not to have a call to refreshPluginsList
    //  at all, and instead have it implicitly run by the FlutterCommand instead. See
    //  https://github.com/flutter/flutter/issues/157391 for details.
    generateLegacyPlugins: false,
  );

  // If there are no plugins and if the project is a not module with an existing
  // podfile, skip processing pods
  if (!hasPlugins(project) && !(project.isModule && xcodeProject.podfile.existsSync())) {
    return;
  }

  // If forcing the use of only CocoaPods, but the project is using Swift
  // Package Manager, print a warning that CocoaPods will be used.
  if (forceCocoaPodsOnly && xcodeProject.usesSwiftPackageManager) {
    globals.logger.printWarning(
      'Swift Package Manager does not yet support this command. '
      'CocoaPods will be used instead.',
    );

    // If CocoaPods has been deintegrated, add it back.
    if (!xcodeProject.podfile.existsSync()) {
      await globals.cocoaPods?.setupPodfile(xcodeProject);
    }

    // Generate an empty Swift Package Manager manifest to invalidate fingerprinter
    final SwiftPackageManager swiftPackageManager = SwiftPackageManager(
      fileSystem: globals.localFileSystem,
      templateRenderer: globals.templateRenderer,
    );
    final SupportedPlatform platform =
        xcodeProject is IosProject ? SupportedPlatform.ios : SupportedPlatform.macos;

    await swiftPackageManager.generatePluginsSwiftPackage(const <Plugin>[], platform, xcodeProject);
  }

  // If the Xcode project, Podfile, generated plugin Swift Package, or podhelper
  // have changed since last run, pods should be updated.
  final Fingerprinter fingerprinter = Fingerprinter(
    fingerprintPath: globals.fs.path.join(buildDirectory, 'pod_inputs.fingerprint'),
    paths: <String>[
      xcodeProject.xcodeProjectInfoFile.path,
      xcodeProject.podfile.path,
      if (xcodeProject.flutterPluginSwiftPackageManifest.existsSync())
        xcodeProject.flutterPluginSwiftPackageManifest.path,
      globals.fs.path.join(Cache.flutterRoot!, 'packages', 'flutter_tools', 'bin', 'podhelper.rb'),
    ],
    fileSystem: globals.fs,
    logger: globals.logger,
  );

  final bool didPodInstall =
      await globals.cocoaPods?.processPods(
        xcodeProject: xcodeProject,
        buildMode: buildMode,
        dependenciesChanged: !fingerprinter.doesFingerprintMatch(),
      ) ??
      false;
  if (didPodInstall) {
    fingerprinter.writeFingerprint();
  }
}
