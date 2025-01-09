// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:meta/meta.dart';
import 'package:process/process.dart';

import 'android/android_emulator.dart';
import 'android/android_sdk.dart';
import 'android/android_workflow.dart';
import 'android/java.dart';
import 'base/context.dart';
import 'base/file_system.dart';
import 'base/logger.dart';
import 'base/process.dart';
import 'device.dart';
import 'ios/ios_emulators.dart';

EmulatorManager? get emulatorManager => context.get<EmulatorManager>();

/// A class to get all available emulators.
class EmulatorManager {
  EmulatorManager({
    required Java? java,
    AndroidSdk? androidSdk,
    required Logger logger,
    required ProcessManager processManager,
    required AndroidWorkflow androidWorkflow,
    required FileSystem fileSystem,
  }) : _java = java,
       _androidSdk = androidSdk,
       _processUtils = ProcessUtils(logger: logger, processManager: processManager),
       _androidEmulators = AndroidEmulators(
         androidSdk: androidSdk,
         logger: logger,
         processManager: processManager,
         fileSystem: fileSystem,
         androidWorkflow: androidWorkflow,
       ) {
    _emulatorDiscoverers.add(_androidEmulators);
  }

  final Java? _java;
  final AndroidSdk? _androidSdk;
  final AndroidEmulators _androidEmulators;
  final ProcessUtils _processUtils;

  // Constructing EmulatorManager is cheap; they only do expensive work if some
  // of their methods are called.
  final List<EmulatorDiscovery> _emulatorDiscoverers = <EmulatorDiscovery>[IOSEmulators()];

  Future<List<Emulator>> getEmulatorsMatching(String searchText) async {
    final List<Emulator> emulators = await getAllAvailableEmulators();
    searchText = searchText.toLowerCase();
    bool exactlyMatchesEmulatorId(Emulator emulator) =>
        emulator.id.toLowerCase() == searchText || emulator.name.toLowerCase() == searchText;
    bool startsWithEmulatorId(Emulator emulator) =>
        emulator.id.toLowerCase().startsWith(searchText) ||
        emulator.name.toLowerCase().startsWith(searchText);

    Emulator? exactMatch;
    for (final Emulator emulator in emulators) {
      if (exactlyMatchesEmulatorId(emulator)) {
        exactMatch = emulator;
        break;
      }
    }
    if (exactMatch != null) {
      return <Emulator>[exactMatch];
    }

    // Match on a id or name starting with [emulatorId].
    return emulators.where(startsWithEmulatorId).toList();
  }

  Iterable<EmulatorDiscovery> get _platformDiscoverers {
    return _emulatorDiscoverers.where(
      (EmulatorDiscovery discoverer) => discoverer.supportsPlatform,
    );
  }

  /// Return the list of all available emulators.
  Future<List<Emulator>> getAllAvailableEmulators() async {
    final List<Emulator> emulators = <Emulator>[];
    await Future.forEach<EmulatorDiscovery>(_platformDiscoverers, (
      EmulatorDiscovery discoverer,
    ) async {
      emulators.addAll(await discoverer.emulators);
    });
    return emulators;
  }

  /// Return the list of all available emulators.
  Future<CreateEmulatorResult> createEmulator({String? name}) async {
    if (name == null || name.isEmpty) {
      const String autoName = 'flutter_emulator';
      // Don't use getEmulatorsMatching here, as it will only return one
      // if there's an exact match and we need all those with this prefix
      // so we can keep adding suffixes until we miss.
      final List<Emulator> all = await getAllAvailableEmulators();
      final Set<String> takenNames =
          all
              .map<String>((Emulator e) => e.id)
              .where((String id) => id.startsWith(autoName))
              .toSet();
      int suffix = 1;
      name = autoName;
      while (takenNames.contains(name)) {
        name = '${autoName}_${++suffix}';
      }
    }
    final String emulatorName = name!;
    final String? avdManagerPath = _androidSdk?.avdManagerPath;
    if (avdManagerPath == null || !_androidEmulators.canLaunchAnything) {
      return CreateEmulatorResult(
        emulatorName,
        success: false,
        error: 'avdmanager is missing from the Android SDK',
      );
    }

    final String? device = await _getPreferredAvailableDevice(avdManagerPath);
    if (device == null) {
      return CreateEmulatorResult(
        emulatorName,
        success: false,
        error: 'No device definitions are available',
      );
    }

    final String? sdkId = await _getPreferredSdkId(avdManagerPath);
    if (sdkId == null) {
      return CreateEmulatorResult(
        emulatorName,
        success: false,
        error:
            'No suitable Android AVD system images are available. You may need to install these'
            ' using sdkmanager, for example:\n'
            '  sdkmanager "system-images;android-27;google_apis_playstore;x86"',
      );
    }

    // Cleans up error output from avdmanager to make it more suitable to show
    // to flutter users. Specifically:
    // - Removes lines that say "null" (!)
    // - Removes lines that tell the user to use '--force' to overwrite emulators
    String? cleanError(String? error) {
      if (error == null || error.trim() == '') {
        return null;
      }
      return error
          .split('\n')
          .where((String l) => l.trim() != 'null')
          .where((String l) => l.trim() != 'Use --force if you want to replace it.')
          .join('\n')
          .trim();
    }

    final RunResult runResult = await _processUtils.run(<String>[
      avdManagerPath,
      'create',
      'avd',
      '-n',
      emulatorName,
      '-k',
      sdkId,
      '-d',
      device,
    ], environment: _java?.environment);
    return CreateEmulatorResult(
      emulatorName,
      success: runResult.exitCode == 0,
      output: runResult.stdout,
      error: cleanError(runResult.stderr),
    );
  }

  static const List<String> preferredDevices = <String>['pixel', 'pixel_xl'];

  Future<String?> _getPreferredAvailableDevice(String avdManagerPath) async {
    final List<String> args = <String>[avdManagerPath, 'list', 'device', '-c'];
    final RunResult runResult = await _processUtils.run(args, environment: _java?.environment);
    if (runResult.exitCode != 0) {
      return null;
    }

    final List<String> availableDevices =
        runResult.stdout
            .split('\n')
            .where((String l) => preferredDevices.contains(l.trim()))
            .toList();

    for (final String device in preferredDevices) {
      if (availableDevices.contains(device)) {
        return device;
      }
    }
    return null;
  }

  static final RegExp _androidApiVersion = RegExp(r';android-(\d+);');

  Future<String?> _getPreferredSdkId(String avdManagerPath) async {
    // It seems that to get the available list of images, we need to send a
    // request to create without the image and it'll provide us a list :-(
    final List<String> args = <String>[avdManagerPath, 'create', 'avd', '-n', 'temp'];
    final RunResult runResult = await _processUtils.run(args, environment: _java?.environment);

    // Get the list of IDs that match our criteria
    final List<String> availableIDs =
        runResult.stderr
            .split('\n')
            .where((String l) => _androidApiVersion.hasMatch(l))
            .where((String l) => l.contains('system-images'))
            .where((String l) => l.contains('google_apis_playstore'))
            .toList();

    final List<int> availableApiVersions =
        availableIDs
            .map<String>((String id) => _androidApiVersion.firstMatch(id)!.group(1)!)
            .map<int>((String apiVersion) => int.parse(apiVersion))
            .toList();

    // Get the highest Android API version or whats left
    final int apiVersion =
        availableApiVersions.isNotEmpty
            ? availableApiVersions.reduce(math.max)
            : -1; // Don't match below

    // We're out of preferences, we just have to return the first one with the high
    // API version.
    for (final String id in availableIDs) {
      if (id.contains(';android-$apiVersion;')) {
        return id;
      }
    }
    return null;
  }

  /// Whether we're capable of listing any emulators given the current environment configuration.
  bool get canListAnything {
    return _platformDiscoverers.any((EmulatorDiscovery discoverer) => discoverer.canListAnything);
  }
}

/// An abstract class to discover and enumerate a specific type of emulators.
abstract class EmulatorDiscovery {
  bool get supportsPlatform;

  /// Whether this emulator discovery is capable of listing any emulators.
  bool get canListAnything;

  /// Whether this emulator discovery is capable of launching new emulators.
  bool get canLaunchAnything;

  Future<List<Emulator>> get emulators;
}

@immutable
abstract class Emulator {
  const Emulator(this.id, this.hasConfig);

  final String id;
  final bool hasConfig;
  String get name;
  String? get manufacturer;
  Category get category;
  PlatformType get platformType;

  @override
  int get hashCode => id.hashCode;

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    return other is Emulator && other.id == id;
  }

  Future<void> launch({bool coldBoot});

  @override
  String toString() => name;

  static List<String> descriptions(List<Emulator> emulators) {
    if (emulators.isEmpty) {
      return <String>[];
    }

    const List<String> tableHeader = <String>['Id', 'Name', 'Manufacturer', 'Platform'];

    // Extract emulators information
    final List<List<String>> table = <List<String>>[
      tableHeader,
      for (final Emulator emulator in emulators)
        <String>[
          emulator.id,
          emulator.name,
          emulator.manufacturer ?? '',
          emulator.platformType.toString(),
        ],
    ];

    // Calculate column widths
    final List<int> indices = List<int>.generate(table[0].length - 1, (int i) => i);
    List<int> widths = indices.map<int>((int i) => 0).toList();
    for (final List<String> row in table) {
      widths = indices.map<int>((int i) => math.max(widths[i], row[i].length)).toList();
    }

    // Join columns into lines of text
    final RegExp whiteSpaceAndDots = RegExp(r'[•\s]+$');
    return table
        .map<String>((List<String> row) {
          return indices
              .map<String>((int i) => row[i].padRight(widths[i]))
              .followedBy(<String>[row.last])
              .join(' • ');
        })
        .map<String>((String line) => line.replaceAll(whiteSpaceAndDots, ''))
        .toList();
  }

  static void printEmulators(List<Emulator> emulators, Logger logger) {
    final List<String> emulatorDescriptions = descriptions(emulators);
    // Prints the first description as the table header, followed by a newline.
    logger.printStatus('${emulatorDescriptions.first}\n');
    emulatorDescriptions.sublist(1).forEach(logger.printStatus);
  }
}

class CreateEmulatorResult {
  CreateEmulatorResult(this.emulatorName, {required this.success, this.output, this.error});

  final bool success;
  final String emulatorName;
  final String? output;
  final String? error;
}
