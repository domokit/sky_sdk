// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../base/common.dart';
import '../base/logger.dart';
import '../base/platform.dart';
import '../cache.dart';
import '../features.dart';
import '../runner/flutter_command.dart';

/// The flutter precache command allows downloading of cache artifacts without
/// the use of device/artifact autodetection.
class PrecacheCommand extends FlutterCommand {
  PrecacheCommand({
    bool verboseHelp = false,
    required Cache cache,
    required Platform platform,
    required Logger logger,
    required FeatureFlags featureFlags,
  }) : _cache = cache,
       _platform = platform,
       _logger = logger,
       _featureFlags = featureFlags {
    argParser.addFlag(
      'all-platforms',
      abbr: 'a',
      negatable: false,
      help: 'Precache artifacts for all host platforms.',
      aliases: const <String>['all'],
    );
    argParser.addFlag(
      'force',
      abbr: 'f',
      negatable: false,
      help: 'Force re-downloading of artifacts.',
    );
    argParser.addFlag(
      'android',
      help: 'Precache artifacts for Android development.',
      hide: !verboseHelp,
    );
    argParser.addFlag(
      'android_gen_snapshot',
      help: 'Precache gen_snapshot for Android development.',
      hide: !verboseHelp,
    );
    argParser.addFlag(
      'android_maven',
      help: 'Precache Gradle dependencies for Android development.',
      hide: !verboseHelp,
    );
    argParser.addFlag(
      'android_internal_build',
      help: 'Precache dependencies for internal Android development.',
      hide: !verboseHelp,
    );
    argParser.addFlag('ios', help: 'Precache artifacts for iOS development.');
    argParser.addFlag('web', help: 'Precache artifacts for web development.');
    argParser.addFlag('linux', help: 'Precache artifacts for Linux desktop development.');
    argParser.addFlag('windows', help: 'Precache artifacts for Windows desktop development.');
    argParser.addFlag('macos', help: 'Precache artifacts for macOS desktop development.');
    argParser.addFlag('fuchsia', help: 'Precache artifacts for Fuchsia development.');
    argParser.addFlag(
      'universal',
      defaultsTo: true,
      help: 'Precache artifacts required for any development platform.',
    );
    argParser.addFlag(
      'flutter_runner',
      help: 'Precache the flutter runner artifacts.',
      hide: !verboseHelp,
    );
    argParser.addFlag(
      'use-unsigned-mac-binaries',
      help: 'Precache the unsigned macOS binaries when available.',
      hide: !verboseHelp,
    );
  }

  final Cache _cache;
  final Logger _logger;
  final Platform _platform;
  final FeatureFlags _featureFlags;

  @override
  final String name = 'precache';

  @override
  final String description =
      "Populate the Flutter tool's cache of binary artifacts.\n\n"
      'If no explicit platform flags are provided, this command will download the artifacts '
      'for all currently enabled platforms';

  @override
  final String category = FlutterCommandCategory.sdk;

  @override
  bool get shouldUpdateCache => false;

  /// Some flags are umbrella names that expand to include multiple artifacts.
  static const Map<String, List<String>> _expandedArtifacts = <String, List<String>>{
    'android': <String>['android_gen_snapshot', 'android_maven', 'android_internal_build'],
  };

  /// Returns a reverse mapping of _expandedArtifacts, from child artifact name
  /// to umbrella name.
  Map<String, String> _umbrellaForArtifactMap() {
    return <String, String>{
      for (final MapEntry<String, List<String>> entry in _expandedArtifacts.entries)
        for (final String childArtifactName in entry.value) childArtifactName: entry.key,
    };
  }

  /// Returns the name of all artifacts that were explicitly chosen via flags.
  ///
  /// If an umbrella is chosen, its children will be included as well.
  Set<String> _explicitArtifactSelections() {
    final Map<String, String> umbrellaForArtifact = _umbrellaForArtifactMap();
    final Set<String> selections = <String>{};
    bool explicitlySelected(String name) => boolArg(name) && argResults!.wasParsed(name);
    for (final DevelopmentArtifact artifact in DevelopmentArtifact.values) {
      final String? umbrellaName = umbrellaForArtifact[artifact.name];
      if (explicitlySelected(artifact.name) ||
          (umbrellaName != null && explicitlySelected(umbrellaName))) {
        selections.add(artifact.name);
      }
    }
    return selections;
  }

  @override
  Future<void> validateCommand() {
    _expandedArtifacts.forEach((String umbrellaName, List<String> childArtifactNames) {
      if (!argResults!.arguments.contains('--no-$umbrellaName')) {
        return;
      }
      for (final String childArtifactName in childArtifactNames) {
        if (argResults!.arguments.contains('--$childArtifactName')) {
          throwToolExit('--$childArtifactName requires --$umbrellaName');
        }
      }
    });

    return super.validateCommand();
  }

  @override
  Future<FlutterCommandResult> runCommand() async {
    // Re-lock the cache.
    if (_platform.environment['FLUTTER_ALREADY_LOCKED'] != 'true') {
      await _cache.lock();
    }
    if (boolArg('force')) {
      _cache.clearStampFiles();
    }

    final bool includeAllPlatforms = boolArg('all-platforms');
    if (includeAllPlatforms) {
      _cache.includeAllPlatforms = true;
    }
    if (boolArg('use-unsigned-mac-binaries')) {
      _cache.useUnsignedMacBinaries = true;
    }
    final Set<String> explicitlyEnabled = _explicitArtifactSelections();
    _cache.platformOverrideArtifacts = explicitlyEnabled;

    // If the user did not provide any artifact flags, then download
    // all artifacts that correspond to an enabled platform.
    final bool downloadDefaultArtifacts = explicitlyEnabled.isEmpty;
    final Map<String, String> umbrellaForArtifact = _umbrellaForArtifactMap();
    final Set<DevelopmentArtifact> requiredArtifacts = <DevelopmentArtifact>{};
    for (final DevelopmentArtifact artifact in DevelopmentArtifact.values) {
      if (artifact.feature != null && !_featureFlags.isEnabled(artifact.feature!)) {
        continue;
      }

      final String argumentName = umbrellaForArtifact[artifact.name] ?? artifact.name;
      if (includeAllPlatforms || boolArg(argumentName) || downloadDefaultArtifacts) {
        requiredArtifacts.add(artifact);
      }
    }
    if (!await _cache.isUpToDate()) {
      await _cache.updateAll(requiredArtifacts);
    } else {
      _logger.printStatus('Already up-to-date.');
    }
    return FlutterCommandResult.success();
  }
}
