// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:meta/meta.dart';
import 'package:process/process.dart';

import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/process.dart';
import '../convert.dart';
import '../device.dart';
import '../emulator.dart';
import 'android_sdk.dart';
import 'android_workflow.dart';

class AndroidEmulators extends EmulatorDiscovery {
  AndroidEmulators({
    AndroidSdk? androidSdk,
    required AndroidWorkflow androidWorkflow,
    required FileSystem fileSystem,
    required Logger logger,
    required ProcessManager processManager,
  }) : _androidSdk = androidSdk,
       _androidWorkflow = androidWorkflow,
       _fileSystem = fileSystem,
       _logger = logger,
       _processManager = processManager,
       _processUtils = ProcessUtils(logger: logger, processManager: processManager);

  final AndroidWorkflow _androidWorkflow;
  final AndroidSdk? _androidSdk;
  final FileSystem _fileSystem;
  final Logger _logger;
  final ProcessManager _processManager;
  final ProcessUtils _processUtils;

  @override
  bool get supportsPlatform => true;

  @override
  bool get canListAnything => _androidWorkflow.canListEmulators;

  @override
  bool get canLaunchAnything =>
      _androidWorkflow.canListEmulators && _androidSdk?.getAvdManagerPath() != null;

  @override
  Future<List<Emulator>> get emulators => _getEmulatorAvds();

  /// Return the list of available emulator AVDs.
  Future<List<AndroidEmulator>> _getEmulatorAvds() async {
    final String? emulatorPath = _androidSdk?.emulatorPath;
    if (emulatorPath == null) {
      return <AndroidEmulator>[];
    }

    final String listAvdsOutput =
        (await _processUtils.run(<String>[emulatorPath, '-list-avds'])).stdout.trim();

    final List<AndroidEmulator> emulators = <AndroidEmulator>[];
    _extractEmulatorAvdInfo(listAvdsOutput, emulators);
    return emulators;
  }

  /// Parse the given `emulator -list-avds` output in [text], and fill out the given list
  /// of emulators by reading information from the relevant ini files.
  void _extractEmulatorAvdInfo(String text, List<AndroidEmulator> emulators) {
    for (final String id in text.trim().split('\n').where((String l) => l != '')) {
      emulators.add(_loadEmulatorInfo(id));
    }
  }

  AndroidEmulator _loadEmulatorInfo(String id) {
    id = id.trim();
    final String? avdPath = _androidSdk?.getAvdPath();
    final AndroidEmulator androidEmulatorWithoutProperties = AndroidEmulator(
      id,
      processManager: _processManager,
      logger: _logger,
      androidSdk: _androidSdk,
    );
    if (avdPath == null) {
      return androidEmulatorWithoutProperties;
    }
    final File iniFile = _fileSystem.file(_fileSystem.path.join(avdPath, '$id.ini'));
    if (!iniFile.existsSync()) {
      return androidEmulatorWithoutProperties;
    }
    final Map<String, String> ini = parseIniLines(iniFile.readAsLinesSync());
    final String? path = ini['path'];
    if (path == null) {
      return androidEmulatorWithoutProperties;
    }
    final File configFile = _fileSystem.file(_fileSystem.path.join(path, 'config.ini'));
    if (!configFile.existsSync()) {
      return androidEmulatorWithoutProperties;
    }
    final Map<String, String> properties = parseIniLines(configFile.readAsLinesSync());
    return AndroidEmulator(
      id,
      properties: properties,
      processManager: _processManager,
      logger: _logger,
      androidSdk: _androidSdk,
    );
  }
}

class AndroidEmulator extends Emulator {
  AndroidEmulator(
    String id, {
    Map<String, String>? properties,
    required Logger logger,
    AndroidSdk? androidSdk,
    required ProcessManager processManager,
  }) : _properties = properties,
       _logger = logger,
       _androidSdk = androidSdk,
       _processUtils = ProcessUtils(logger: logger, processManager: processManager),
       super(id, properties != null && properties.isNotEmpty);

  final Map<String, String>? _properties;
  final Logger _logger;
  final ProcessUtils _processUtils;
  final AndroidSdk? _androidSdk;

  // Android Studio uses the ID with underscores replaced with spaces
  // for the name if displayname is not set so we do the same.
  @override
  String get name => _prop('avd.ini.displayname') ?? id.replaceAll('_', ' ').trim();

  @override
  String? get manufacturer => _prop('hw.device.manufacturer');

  @override
  Category get category => Category.mobile;

  @override
  PlatformType get platformType => PlatformType.android;

  String? _prop(String name) => _properties != null ? _properties[name] : null;

  @override
  Future<void> launch({@visibleForTesting Duration? startupDuration, bool coldBoot = false}) async {
    final String? emulatorPath = _androidSdk?.emulatorPath;
    if (emulatorPath == null) {
      throw Exception('Emulator is missing from the Android SDK');
    }
    final List<String> command = <String>[
      emulatorPath,
      '-avd',
      id,
      if (coldBoot) '-no-snapshot-load',
    ];
    final Process process = await _processUtils.start(command);

    // Record output from the emulator process.
    final List<String> stdoutList = <String>[];
    final List<String> stderrList = <String>[];
    final StreamSubscription<String> stdoutSubscription = process.stdout
        .transform<String>(utf8.decoder)
        .transform<String>(const LineSplitter())
        .listen(stdoutList.add);
    final StreamSubscription<String> stderrSubscription = process.stderr
        .transform<String>(utf8.decoder)
        .transform<String>(const LineSplitter())
        .listen(stderrList.add);
    final Future<void> stdioFuture = Future.wait<void>(<Future<void>>[
      stdoutSubscription.asFuture<void>(),
      stderrSubscription.asFuture<void>(),
    ]);

    // The emulator continues running on success, so we don't wait for the
    // process to complete before continuing. However, if the process fails
    // after the startup phase (3 seconds), then we only echo its output if
    // its error code is non-zero and its stderr is non-empty.
    bool earlyFailure = true;
    unawaited(
      process.exitCode.then((int status) async {
        if (status == 0) {
          _logger.printTrace('The Android emulator exited successfully');
          return;
        }
        // Make sure the process' stdout and stderr are drained.
        await stdioFuture;
        unawaited(stdoutSubscription.cancel());
        unawaited(stderrSubscription.cancel());
        if (stdoutList.isNotEmpty) {
          _logger.printTrace('Android emulator stdout:');
          stdoutList.forEach(_logger.printTrace);
        }
        if (!earlyFailure && stderrList.isEmpty) {
          _logger.printStatus('The Android emulator exited with code $status');
          return;
        }
        final String when = earlyFailure ? 'during startup' : 'after startup';
        _logger.printError('The Android emulator exited with code $status $when');
        _logger.printError('Android emulator stderr:');
        stderrList.forEach(_logger.printError);
        _logger.printError('Address these issues and try again.');
      }),
    );

    // Wait a few seconds for the emulator to start.
    await Future<void>.delayed(startupDuration ?? const Duration(seconds: 3));
    earlyFailure = false;
    return;
  }
}

@visibleForTesting
Map<String, String> parseIniLines(List<String> contents) {
  final Map<String, String> results = <String, String>{};

  final Iterable<List<String>> properties = contents
      .map<String>((String l) => l.trim())
      // Strip blank lines/comments
      .where((String l) => l != '' && !l.startsWith('#'))
      // Discard anything that isn't simple name=value
      .where((String l) => l.contains('='))
      // Split into name/value
      .map<List<String>>((String l) => l.split('='));

  for (final List<String> property in properties) {
    results[property[0].trim()] = property[1].trim();
  }

  return results;
}
