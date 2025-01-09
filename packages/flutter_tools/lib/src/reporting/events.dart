// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

part of 'reporting.dart';

/// A generic usage even that does not involve custom dimensions.
///
/// If sending values for custom dimensions is required, extend this class as
/// below.
class UsageEvent {
  UsageEvent(this.category, this.parameter, {this.label, this.value, required this.flutterUsage});

  final String category;
  final String parameter;
  final String? label;
  final int? value;
  final Usage flutterUsage;

  void send() {
    flutterUsage.sendEvent(category, parameter, label: label, value: value);
  }
}

/// A usage event related to hot reload/restart.
///
/// On a successful hot reload, we collect stats that help understand scale of
/// the update. For example, [syncedLibraryCount]/[finalLibraryCount] indicates
/// how many libraries were affected by the hot reload request. Relation of
/// [invalidatedSourcesCount] to [syncedLibraryCount] should help understand
/// sync/transfer "overhead" of updating this number of source files.
class HotEvent extends UsageEvent {
  HotEvent(
    String parameter, {
    required this.targetPlatform,
    required this.sdkName,
    required this.emulator,
    required this.fullRestart,
    this.reason,
    this.finalLibraryCount,
    this.syncedLibraryCount,
    this.syncedClassesCount,
    this.syncedProceduresCount,
    this.syncedBytes,
    this.invalidatedSourcesCount,
    this.transferTimeInMs,
    this.overallTimeInMs,
    this.compileTimeInMs,
    this.findInvalidatedTimeInMs,
    this.scannedSourcesCount,
    this.reassembleTimeInMs,
    this.reloadVMTimeInMs,
    // TODO(fujino): make this required
    Usage? usage,
  }) : super('hot', parameter, flutterUsage: usage ?? globals.flutterUsage);

  final String? reason;
  final String targetPlatform;
  final String sdkName;
  final bool emulator;
  final bool fullRestart;
  final int? finalLibraryCount;
  final int? syncedLibraryCount;
  final int? syncedClassesCount;
  final int? syncedProceduresCount;
  final int? syncedBytes;
  final int? invalidatedSourcesCount;
  final int? transferTimeInMs;
  final int? overallTimeInMs;
  final int? compileTimeInMs;
  final int? findInvalidatedTimeInMs;
  final int? scannedSourcesCount;
  final int? reassembleTimeInMs;
  final int? reloadVMTimeInMs;

  @override
  void send() {
    final CustomDimensions parameters = CustomDimensions(
      hotEventTargetPlatform: targetPlatform,
      hotEventSdkName: sdkName,
      hotEventEmulator: emulator,
      hotEventFullRestart: fullRestart,
      hotEventReason: reason,
      hotEventFinalLibraryCount: finalLibraryCount,
      hotEventSyncedLibraryCount: syncedLibraryCount,
      hotEventSyncedClassesCount: syncedClassesCount,
      hotEventSyncedProceduresCount: syncedProceduresCount,
      hotEventSyncedBytes: syncedBytes,
      hotEventInvalidatedSourcesCount: invalidatedSourcesCount,
      hotEventTransferTimeInMs: transferTimeInMs,
      hotEventOverallTimeInMs: overallTimeInMs,
      hotEventCompileTimeInMs: compileTimeInMs,
      hotEventFindInvalidatedTimeInMs: findInvalidatedTimeInMs,
      hotEventScannedSourcesCount: scannedSourcesCount,
      hotEventReassembleTimeInMs: reassembleTimeInMs,
      hotEventReloadVMTimeInMs: reloadVMTimeInMs,
    );
    flutterUsage.sendEvent(category, parameter, parameters: parameters);
  }
}

/// An event that reports the result of a [DoctorValidator]
class DoctorResultEvent extends UsageEvent {
  DoctorResultEvent({required this.validator, required this.result, Usage? flutterUsage})
    : super(
        'doctor-result',
        '${validator.runtimeType}',
        label: result.typeStr,
        flutterUsage: flutterUsage ?? globals.flutterUsage,
      );

  final DoctorValidator validator;
  final ValidationResult result;

  @override
  void send() {
    if (validator is! GroupedValidator) {
      flutterUsage.sendEvent(category, parameter, label: label);
      return;
    }
    final GroupedValidator group = validator as GroupedValidator;
    // The validator crashed.
    if (group.subResults.isEmpty) {
      flutterUsage.sendEvent(category, parameter, label: label);
      return;
    }
    for (int i = 0; i < group.subValidators.length; i++) {
      final DoctorValidator v = group.subValidators[i];
      final ValidationResult r = group.subResults[i];
      DoctorResultEvent(validator: v, result: r, flutterUsage: flutterUsage).send();
    }
  }
}

/// An event that reports on the result of a pub invocation.
class PubResultEvent extends UsageEvent {
  PubResultEvent({required String context, required String result, required Usage usage})
    : super('pub-result', context, label: result, flutterUsage: usage);
}

/// An event that reports something about a build.
class BuildEvent extends UsageEvent {
  BuildEvent(
    String label, {
    String? command,
    String? settings,
    String? eventError,
    required Usage flutterUsage,
    required String type,
  }) : _command = command,
       _settings = settings,
       _eventError = eventError,
       super(
         // category
         'build',
         // parameter
         type,
         label: label,
         flutterUsage: flutterUsage,
       );

  final String? _command;
  final String? _settings;
  final String? _eventError;

  @override
  void send() {
    final CustomDimensions parameters = CustomDimensions(
      buildEventCommand: _command,
      buildEventSettings: _settings,
      buildEventError: _eventError,
    );
    flutterUsage.sendEvent(category, parameter, label: label, parameters: parameters);
  }
}

/// An event that reports the result of a top-level command.
class CommandResultEvent extends UsageEvent {
  CommandResultEvent(super.commandPath, super.result, int? maxRss)
    : _maxRss = maxRss,
      super(flutterUsage: globals.flutterUsage);

  final int? _maxRss;

  @override
  void send() {
    // An event for the command result.
    flutterUsage.sendEvent('tool-command-result', category, label: parameter);

    // A separate event for the memory highwater mark. This is a separate event
    // so that we can get the command result even if trying to grab maxRss
    // throws an exception.
    if (_maxRss != null) {
      flutterUsage.sendEvent('tool-command-max-rss', category, label: parameter, value: _maxRss);
    }
  }
}

/// An event that reports on changes in the configuration of analytics.
class AnalyticsConfigEvent extends UsageEvent {
  AnalyticsConfigEvent({
    /// Whether analytics reporting is being enabled (true) or disabled (false).
    required bool enabled,
  }) : super(
         'analytics',
         'enabled',
         label: enabled ? 'true' : 'false',
         flutterUsage: globals.flutterUsage,
       );
}

/// An event that reports when the code size measurement is run via `--analyze-size`.
class CodeSizeEvent extends UsageEvent {
  CodeSizeEvent(String platform, {required Usage flutterUsage})
    : super('code-size-analysis', platform, flutterUsage: flutterUsage);
}

/// An event for tracking the usage of specific error handling fallbacks.
class ErrorHandlingEvent extends UsageEvent {
  ErrorHandlingEvent(String parameter)
    : super('error-handling', parameter, flutterUsage: globals.flutterUsage);
}

/// Emit various null safety analytic events.
///
/// 1. The current null safety runtime mode.
/// 2. The number of packages that are migrated, along with the total number of packages
/// 3. The main packages language version.
class NullSafetyAnalysisEvent implements UsageEvent {
  NullSafetyAnalysisEvent(
    this.packageConfig,
    this.nullSafetyMode,
    this.currentPackage,
    this.flutterUsage,
  );

  /// The category for analytics events related to null safety.
  static const String kNullSafetyCategory = 'null-safety';

  final PackageConfig packageConfig;
  final NullSafetyMode nullSafetyMode;
  final String currentPackage;
  @override
  final Usage flutterUsage;

  @override
  void send() {
    if (packageConfig.packages.isEmpty) {
      return;
    }
    int migrated = 0;
    LanguageVersion? languageVersion;
    for (final Package package in packageConfig.packages) {
      final LanguageVersion? packageLanguageVersion = package.languageVersion;
      if (package.name == currentPackage) {
        languageVersion = packageLanguageVersion;
      }
      if (packageLanguageVersion != null &&
          packageLanguageVersion.major >= nullSafeVersion.major &&
          packageLanguageVersion.minor >= nullSafeVersion.minor) {
        migrated += 1;
      }
    }
    flutterUsage.sendEvent(kNullSafetyCategory, 'runtime-mode', label: nullSafetyMode.toString());
    flutterUsage.sendEvent(
      kNullSafetyCategory,
      'stats',
      parameters: CustomDimensions(
        nullSafeMigratedLibraries: migrated,
        nullSafeTotalLibraries: packageConfig.packages.length,
      ),
    );
    if (languageVersion != null) {
      final String formattedVersion = '${languageVersion.major}.${languageVersion.minor}';
      flutterUsage.sendEvent(kNullSafetyCategory, 'language-version', label: formattedVersion);
    }
  }

  @override
  String get category => kNullSafetyCategory;

  @override
  String get label => throw UnsupportedError('');

  @override
  String get parameter => throw UnsupportedError('');

  @override
  int get value => throw UnsupportedError('');
}
