// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

part of 'reporting.dart';

/// The collection of custom dimensions understood by the analytics backend.
/// When adding to this list, first ensure that the custom dimension is
/// defined in the backend, or will be defined shortly after the relevant PR
/// lands.
@immutable
class CustomDimensions {
  const CustomDimensions({
    this.sessionHostOsDetails,
    this.sessionChannelName,
    this.commandRunIsEmulator,
    this.commandRunTargetName,
    this.hotEventReason,
    this.hotEventFinalLibraryCount,
    this.hotEventSyncedLibraryCount,
    this.hotEventSyncedClassesCount,
    this.hotEventSyncedProceduresCount,
    this.hotEventSyncedBytes,
    this.hotEventInvalidatedSourcesCount,
    this.hotEventTransferTimeInMs,
    this.hotEventOverallTimeInMs,
    this.commandRunProjectType,
    this.commandRunProjectHostLanguage,
    this.commandCreateAndroidLanguage,
    this.commandCreateIosLanguage,
    this.commandRunProjectModule,
    this.commandCreateProjectType,
    this.commandPackagesNumberPlugins,
    this.commandPackagesProjectModule,
    this.commandRunTargetOsVersion,
    this.commandRunModeName,
    this.commandBuildBundleTargetPlatform,
    this.commandBuildBundleIsModule,
    this.commandResult,
    this.hotEventTargetPlatform,
    this.hotEventSdkName,
    this.hotEventEmulator,
    this.hotEventFullRestart,
    this.commandHasTerminal,
    this.enabledFlutterFeatures,
    this.localTime,
    this.commandBuildAarTargetPlatform,
    this.commandBuildAarProjectType,
    this.buildEventCommand,
    this.buildEventSettings,
    this.commandBuildApkTargetPlatform,
    this.commandBuildApkBuildMode,
    this.commandBuildApkSplitPerAbi,
    this.commandBuildAppBundleTargetPlatform,
    this.commandBuildAppBundleBuildMode,
    this.buildEventError,
    this.commandResultEventMaxRss,
    this.commandRunAndroidEmbeddingVersion,
    this.commandPackagesAndroidEmbeddingVersion,
    this.nullSafety,
    this.nullSafeMigratedLibraries,
    this.nullSafeTotalLibraries,
    this.hotEventCompileTimeInMs,
    this.hotEventFindInvalidatedTimeInMs,
    this.hotEventScannedSourcesCount,
    this.hotEventReassembleTimeInMs,
    this.hotEventReloadVMTimeInMs,
    this.commandRunEnableImpeller,
    this.commandRunIOSInterfaceType,
    this.commandRunIsTest,
  });

  final String? sessionHostOsDetails; // cd1
  final String? sessionChannelName; // cd2
  final bool? commandRunIsEmulator; // cd3
  final String? commandRunTargetName; // cd4
  final String? hotEventReason; // cd5
  final int? hotEventFinalLibraryCount; // cd6
  final int? hotEventSyncedLibraryCount; // cd7
  final int? hotEventSyncedClassesCount; // cd8
  final int? hotEventSyncedProceduresCount; // cd9
  final int? hotEventSyncedBytes; // cd10
  final int? hotEventInvalidatedSourcesCount; // cd11
  final int? hotEventTransferTimeInMs; // cd12
  final int? hotEventOverallTimeInMs; // cd13
  final String? commandRunProjectType; // cd14
  final String? commandRunProjectHostLanguage; // cd15
  final String? commandCreateAndroidLanguage; // cd16
  final String? commandCreateIosLanguage; // cd17
  final bool? commandRunProjectModule; // cd18
  final String? commandCreateProjectType; // cd19
  final int? commandPackagesNumberPlugins; // cd20
  final bool? commandPackagesProjectModule; // cd21
  final String? commandRunTargetOsVersion; // cd22
  final String? commandRunModeName; // cd23
  final String? commandBuildBundleTargetPlatform; // cd24
  final bool? commandBuildBundleIsModule; // cd25
  final String? commandResult; // cd26
  final String? hotEventTargetPlatform; // cd27
  final String? hotEventSdkName; // cd28
  final bool? hotEventEmulator; // cd29
  final bool? hotEventFullRestart; // cd30
  final bool? commandHasTerminal; // cd31
  final String? enabledFlutterFeatures; // cd32
  final String? localTime; // cd33
  final String? commandBuildAarTargetPlatform; // cd34
  final String? commandBuildAarProjectType; // cd35
  final String? buildEventCommand; // cd36
  final String? buildEventSettings; // cd37
  final String? commandBuildApkTargetPlatform; // cd38
  final String? commandBuildApkBuildMode; // cd39
  final bool? commandBuildApkSplitPerAbi; // cd40
  final String? commandBuildAppBundleTargetPlatform; // cd41
  final String? commandBuildAppBundleBuildMode; // cd42
  final String? buildEventError; // cd43
  final int? commandResultEventMaxRss; // cd44
  final String? commandRunAndroidEmbeddingVersion; // cd45
  final String? commandPackagesAndroidEmbeddingVersion; // cd46
  final bool? nullSafety; // cd47
  // cd48 was fastReassemble but that feature was removed
  final int? nullSafeMigratedLibraries; // cd49
  final int? nullSafeTotalLibraries; // cd50
  final int? hotEventCompileTimeInMs; // cd51
  final int? hotEventFindInvalidatedTimeInMs; // cd52
  final int? hotEventScannedSourcesCount; // cd53
  final int? hotEventReassembleTimeInMs; // cd54
  final int? hotEventReloadVMTimeInMs; // cd55
  final bool? commandRunEnableImpeller; // cd56
  final String? commandRunIOSInterfaceType; // cd57
  final bool? commandRunIsTest; // cd58

  /// Convert to a map that will be used to upload to the analytics backend.
  Map<String, String> toMap() => <String, String>{
    if (sessionHostOsDetails != null)
      CustomDimensionsEnum.sessionHostOsDetails.cdKey: sessionHostOsDetails.toString(),
    if (sessionChannelName != null)
      CustomDimensionsEnum.sessionChannelName.cdKey: sessionChannelName.toString(),
    if (commandRunIsEmulator != null)
      CustomDimensionsEnum.commandRunIsEmulator.cdKey: commandRunIsEmulator.toString(),
    if (commandRunTargetName != null)
      CustomDimensionsEnum.commandRunTargetName.cdKey: commandRunTargetName.toString(),
    if (hotEventReason != null)
      CustomDimensionsEnum.hotEventReason.cdKey: hotEventReason.toString(),
    if (hotEventFinalLibraryCount != null)
      CustomDimensionsEnum.hotEventFinalLibraryCount.cdKey: hotEventFinalLibraryCount.toString(),
    if (hotEventSyncedLibraryCount != null)
      CustomDimensionsEnum.hotEventSyncedLibraryCount.cdKey: hotEventSyncedLibraryCount.toString(),
    if (hotEventSyncedClassesCount != null)
      CustomDimensionsEnum.hotEventSyncedClassesCount.cdKey: hotEventSyncedClassesCount.toString(),
    if (hotEventSyncedProceduresCount != null)
      CustomDimensionsEnum.hotEventSyncedProceduresCount.cdKey:
          hotEventSyncedProceduresCount.toString(),
    if (hotEventSyncedBytes != null)
      CustomDimensionsEnum.hotEventSyncedBytes.cdKey: hotEventSyncedBytes.toString(),
    if (hotEventInvalidatedSourcesCount != null)
      CustomDimensionsEnum.hotEventInvalidatedSourcesCount.cdKey:
          hotEventInvalidatedSourcesCount.toString(),
    if (hotEventTransferTimeInMs != null)
      CustomDimensionsEnum.hotEventTransferTimeInMs.cdKey: hotEventTransferTimeInMs.toString(),
    if (hotEventOverallTimeInMs != null)
      CustomDimensionsEnum.hotEventOverallTimeInMs.cdKey: hotEventOverallTimeInMs.toString(),
    if (commandRunProjectType != null)
      CustomDimensionsEnum.commandRunProjectType.cdKey: commandRunProjectType.toString(),
    if (commandRunProjectHostLanguage != null)
      CustomDimensionsEnum.commandRunProjectHostLanguage.cdKey:
          commandRunProjectHostLanguage.toString(),
    if (commandCreateAndroidLanguage != null)
      CustomDimensionsEnum.commandCreateAndroidLanguage.cdKey:
          commandCreateAndroidLanguage.toString(),
    if (commandCreateIosLanguage != null)
      CustomDimensionsEnum.commandCreateIosLanguage.cdKey: commandCreateIosLanguage.toString(),
    if (commandRunProjectModule != null)
      CustomDimensionsEnum.commandRunProjectModule.cdKey: commandRunProjectModule.toString(),
    if (commandCreateProjectType != null)
      CustomDimensionsEnum.commandCreateProjectType.cdKey: commandCreateProjectType.toString(),
    if (commandPackagesNumberPlugins != null)
      CustomDimensionsEnum.commandPackagesNumberPlugins.cdKey:
          commandPackagesNumberPlugins.toString(),
    if (commandPackagesProjectModule != null)
      CustomDimensionsEnum.commandPackagesProjectModule.cdKey:
          commandPackagesProjectModule.toString(),
    if (commandRunTargetOsVersion != null)
      CustomDimensionsEnum.commandRunTargetOsVersion.cdKey: commandRunTargetOsVersion.toString(),
    if (commandRunModeName != null)
      CustomDimensionsEnum.commandRunModeName.cdKey: commandRunModeName.toString(),
    if (commandBuildBundleTargetPlatform != null)
      CustomDimensionsEnum.commandBuildBundleTargetPlatform.cdKey:
          commandBuildBundleTargetPlatform.toString(),
    if (commandBuildBundleIsModule != null)
      CustomDimensionsEnum.commandBuildBundleIsModule.cdKey: commandBuildBundleIsModule.toString(),
    if (commandResult != null) CustomDimensionsEnum.commandResult.cdKey: commandResult.toString(),
    if (hotEventTargetPlatform != null)
      CustomDimensionsEnum.hotEventTargetPlatform.cdKey: hotEventTargetPlatform.toString(),
    if (hotEventSdkName != null)
      CustomDimensionsEnum.hotEventSdkName.cdKey: hotEventSdkName.toString(),
    if (hotEventEmulator != null)
      CustomDimensionsEnum.hotEventEmulator.cdKey: hotEventEmulator.toString(),
    if (hotEventFullRestart != null)
      CustomDimensionsEnum.hotEventFullRestart.cdKey: hotEventFullRestart.toString(),
    if (commandHasTerminal != null)
      CustomDimensionsEnum.commandHasTerminal.cdKey: commandHasTerminal.toString(),
    if (enabledFlutterFeatures != null)
      CustomDimensionsEnum.enabledFlutterFeatures.cdKey: enabledFlutterFeatures.toString(),
    if (localTime != null) CustomDimensionsEnum.localTime.cdKey: localTime.toString(),
    if (commandBuildAarTargetPlatform != null)
      CustomDimensionsEnum.commandBuildAarTargetPlatform.cdKey:
          commandBuildAarTargetPlatform.toString(),
    if (commandBuildAarProjectType != null)
      CustomDimensionsEnum.commandBuildAarProjectType.cdKey: commandBuildAarProjectType.toString(),
    if (buildEventCommand != null)
      CustomDimensionsEnum.buildEventCommand.cdKey: buildEventCommand.toString(),
    if (buildEventSettings != null)
      CustomDimensionsEnum.buildEventSettings.cdKey: buildEventSettings.toString(),
    if (commandBuildApkTargetPlatform != null)
      CustomDimensionsEnum.commandBuildApkTargetPlatform.cdKey:
          commandBuildApkTargetPlatform.toString(),
    if (commandBuildApkBuildMode != null)
      CustomDimensionsEnum.commandBuildApkBuildMode.cdKey: commandBuildApkBuildMode.toString(),
    if (commandBuildApkSplitPerAbi != null)
      CustomDimensionsEnum.commandBuildApkSplitPerAbi.cdKey: commandBuildApkSplitPerAbi.toString(),
    if (commandBuildAppBundleTargetPlatform != null)
      CustomDimensionsEnum.commandBuildAppBundleTargetPlatform.cdKey:
          commandBuildAppBundleTargetPlatform.toString(),
    if (commandBuildAppBundleBuildMode != null)
      CustomDimensionsEnum.commandBuildAppBundleBuildMode.cdKey:
          commandBuildAppBundleBuildMode.toString(),
    if (buildEventError != null)
      CustomDimensionsEnum.buildEventError.cdKey: buildEventError.toString(),
    if (commandResultEventMaxRss != null)
      CustomDimensionsEnum.commandResultEventMaxRss.cdKey: commandResultEventMaxRss.toString(),
    if (commandRunAndroidEmbeddingVersion != null)
      CustomDimensionsEnum.commandRunAndroidEmbeddingVersion.cdKey:
          commandRunAndroidEmbeddingVersion.toString(),
    if (commandPackagesAndroidEmbeddingVersion != null)
      CustomDimensionsEnum.commandPackagesAndroidEmbeddingVersion.cdKey:
          commandPackagesAndroidEmbeddingVersion.toString(),
    if (nullSafety != null) CustomDimensionsEnum.nullSafety.cdKey: nullSafety.toString(),
    if (nullSafeMigratedLibraries != null)
      CustomDimensionsEnum.nullSafeMigratedLibraries.cdKey: nullSafeMigratedLibraries.toString(),
    if (nullSafeTotalLibraries != null)
      CustomDimensionsEnum.nullSafeTotalLibraries.cdKey: nullSafeTotalLibraries.toString(),
    if (hotEventCompileTimeInMs != null)
      CustomDimensionsEnum.hotEventCompileTimeInMs.cdKey: hotEventCompileTimeInMs.toString(),
    if (hotEventFindInvalidatedTimeInMs != null)
      CustomDimensionsEnum.hotEventFindInvalidatedTimeInMs.cdKey:
          hotEventFindInvalidatedTimeInMs.toString(),
    if (hotEventScannedSourcesCount != null)
      CustomDimensionsEnum.hotEventScannedSourcesCount.cdKey:
          hotEventScannedSourcesCount.toString(),
    if (hotEventReassembleTimeInMs != null)
      CustomDimensionsEnum.hotEventReassembleTimeInMs.cdKey: hotEventReassembleTimeInMs.toString(),
    if (hotEventReloadVMTimeInMs != null)
      CustomDimensionsEnum.hotEventReloadVMTimeInMs.cdKey: hotEventReloadVMTimeInMs.toString(),
    if (commandRunEnableImpeller != null)
      CustomDimensionsEnum.commandRunEnableImpeller.cdKey: commandRunEnableImpeller.toString(),
    if (commandRunIOSInterfaceType != null)
      CustomDimensionsEnum.commandRunIOSInterfaceType.cdKey: commandRunIOSInterfaceType.toString(),
    if (commandRunIsTest != null)
      CustomDimensionsEnum.commandRunIsTest.cdKey: commandRunIsTest.toString(),
  };

  /// Merge the values of two [CustomDimensions] into one. If a value is defined
  /// in both instances, the value in [other] will override the value in this.
  CustomDimensions merge(CustomDimensions? other) {
    if (other == null) {
      return this;
    }

    return CustomDimensions(
      sessionHostOsDetails: other.sessionHostOsDetails ?? sessionHostOsDetails,
      sessionChannelName: other.sessionChannelName ?? sessionChannelName,
      commandRunIsEmulator: other.commandRunIsEmulator ?? commandRunIsEmulator,
      commandRunTargetName: other.commandRunTargetName ?? commandRunTargetName,
      hotEventReason: other.hotEventReason ?? hotEventReason,
      hotEventFinalLibraryCount: other.hotEventFinalLibraryCount ?? hotEventFinalLibraryCount,
      hotEventSyncedLibraryCount: other.hotEventSyncedLibraryCount ?? hotEventSyncedLibraryCount,
      hotEventSyncedClassesCount: other.hotEventSyncedClassesCount ?? hotEventSyncedClassesCount,
      hotEventSyncedProceduresCount:
          other.hotEventSyncedProceduresCount ?? hotEventSyncedProceduresCount,
      hotEventSyncedBytes: other.hotEventSyncedBytes ?? hotEventSyncedBytes,
      hotEventInvalidatedSourcesCount:
          other.hotEventInvalidatedSourcesCount ?? hotEventInvalidatedSourcesCount,
      hotEventTransferTimeInMs: other.hotEventTransferTimeInMs ?? hotEventTransferTimeInMs,
      hotEventOverallTimeInMs: other.hotEventOverallTimeInMs ?? hotEventOverallTimeInMs,
      commandRunProjectType: other.commandRunProjectType ?? commandRunProjectType,
      commandRunProjectHostLanguage:
          other.commandRunProjectHostLanguage ?? commandRunProjectHostLanguage,
      commandCreateAndroidLanguage:
          other.commandCreateAndroidLanguage ?? commandCreateAndroidLanguage,
      commandCreateIosLanguage: other.commandCreateIosLanguage ?? commandCreateIosLanguage,
      commandRunProjectModule: other.commandRunProjectModule ?? commandRunProjectModule,
      commandCreateProjectType: other.commandCreateProjectType ?? commandCreateProjectType,
      commandPackagesNumberPlugins:
          other.commandPackagesNumberPlugins ?? commandPackagesNumberPlugins,
      commandPackagesProjectModule:
          other.commandPackagesProjectModule ?? commandPackagesProjectModule,
      commandRunTargetOsVersion: other.commandRunTargetOsVersion ?? commandRunTargetOsVersion,
      commandRunModeName: other.commandRunModeName ?? commandRunModeName,
      commandBuildBundleTargetPlatform:
          other.commandBuildBundleTargetPlatform ?? commandBuildBundleTargetPlatform,
      commandBuildBundleIsModule: other.commandBuildBundleIsModule ?? commandBuildBundleIsModule,
      commandResult: other.commandResult ?? commandResult,
      hotEventTargetPlatform: other.hotEventTargetPlatform ?? hotEventTargetPlatform,
      hotEventSdkName: other.hotEventSdkName ?? hotEventSdkName,
      hotEventEmulator: other.hotEventEmulator ?? hotEventEmulator,
      hotEventFullRestart: other.hotEventFullRestart ?? hotEventFullRestart,
      commandHasTerminal: other.commandHasTerminal ?? commandHasTerminal,
      enabledFlutterFeatures: other.enabledFlutterFeatures ?? enabledFlutterFeatures,
      localTime: other.localTime ?? localTime,
      commandBuildAarTargetPlatform:
          other.commandBuildAarTargetPlatform ?? commandBuildAarTargetPlatform,
      commandBuildAarProjectType: other.commandBuildAarProjectType ?? commandBuildAarProjectType,
      buildEventCommand: other.buildEventCommand ?? buildEventCommand,
      buildEventSettings: other.buildEventSettings ?? buildEventSettings,
      commandBuildApkTargetPlatform:
          other.commandBuildApkTargetPlatform ?? commandBuildApkTargetPlatform,
      commandBuildApkBuildMode: other.commandBuildApkBuildMode ?? commandBuildApkBuildMode,
      commandBuildApkSplitPerAbi: other.commandBuildApkSplitPerAbi ?? commandBuildApkSplitPerAbi,
      commandBuildAppBundleTargetPlatform:
          other.commandBuildAppBundleTargetPlatform ?? commandBuildAppBundleTargetPlatform,
      commandBuildAppBundleBuildMode:
          other.commandBuildAppBundleBuildMode ?? commandBuildAppBundleBuildMode,
      buildEventError: other.buildEventError ?? buildEventError,
      commandResultEventMaxRss: other.commandResultEventMaxRss ?? commandResultEventMaxRss,
      commandRunAndroidEmbeddingVersion:
          other.commandRunAndroidEmbeddingVersion ?? commandRunAndroidEmbeddingVersion,
      commandPackagesAndroidEmbeddingVersion:
          other.commandPackagesAndroidEmbeddingVersion ?? commandPackagesAndroidEmbeddingVersion,
      nullSafety: other.nullSafety ?? nullSafety,
      nullSafeMigratedLibraries: other.nullSafeMigratedLibraries ?? nullSafeMigratedLibraries,
      nullSafeTotalLibraries: other.nullSafeTotalLibraries ?? nullSafeTotalLibraries,
      hotEventCompileTimeInMs: other.hotEventCompileTimeInMs ?? hotEventCompileTimeInMs,
      hotEventFindInvalidatedTimeInMs:
          other.hotEventFindInvalidatedTimeInMs ?? hotEventFindInvalidatedTimeInMs,
      hotEventScannedSourcesCount: other.hotEventScannedSourcesCount ?? hotEventScannedSourcesCount,
      hotEventReassembleTimeInMs: other.hotEventReassembleTimeInMs ?? hotEventReassembleTimeInMs,
      hotEventReloadVMTimeInMs: other.hotEventReloadVMTimeInMs ?? hotEventReloadVMTimeInMs,
      commandRunEnableImpeller: other.commandRunEnableImpeller ?? commandRunEnableImpeller,
      commandRunIOSInterfaceType: other.commandRunIOSInterfaceType ?? commandRunIOSInterfaceType,
      commandRunIsTest: other.commandRunIsTest ?? commandRunIsTest,
    );
  }

  static CustomDimensions fromMap(Map<String, String> map) => CustomDimensions(
    sessionHostOsDetails: _extractString(map, CustomDimensionsEnum.sessionHostOsDetails),
    sessionChannelName: _extractString(map, CustomDimensionsEnum.sessionChannelName),
    commandRunIsEmulator: _extractBool(map, CustomDimensionsEnum.commandRunIsEmulator),
    commandRunTargetName: _extractString(map, CustomDimensionsEnum.commandRunTargetName),
    hotEventReason: _extractString(map, CustomDimensionsEnum.hotEventReason),
    hotEventFinalLibraryCount: _extractInt(map, CustomDimensionsEnum.hotEventFinalLibraryCount),
    hotEventSyncedLibraryCount: _extractInt(map, CustomDimensionsEnum.hotEventSyncedLibraryCount),
    hotEventSyncedClassesCount: _extractInt(map, CustomDimensionsEnum.hotEventSyncedClassesCount),
    hotEventSyncedProceduresCount: _extractInt(
      map,
      CustomDimensionsEnum.hotEventSyncedProceduresCount,
    ),
    hotEventSyncedBytes: _extractInt(map, CustomDimensionsEnum.hotEventSyncedBytes),
    hotEventInvalidatedSourcesCount: _extractInt(
      map,
      CustomDimensionsEnum.hotEventInvalidatedSourcesCount,
    ),
    hotEventTransferTimeInMs: _extractInt(map, CustomDimensionsEnum.hotEventTransferTimeInMs),
    hotEventOverallTimeInMs: _extractInt(map, CustomDimensionsEnum.hotEventOverallTimeInMs),
    commandRunProjectType: _extractString(map, CustomDimensionsEnum.commandRunProjectType),
    commandRunProjectHostLanguage: _extractString(
      map,
      CustomDimensionsEnum.commandRunProjectHostLanguage,
    ),
    commandCreateAndroidLanguage: _extractString(
      map,
      CustomDimensionsEnum.commandCreateAndroidLanguage,
    ),
    commandCreateIosLanguage: _extractString(map, CustomDimensionsEnum.commandCreateIosLanguage),
    commandRunProjectModule: _extractBool(map, CustomDimensionsEnum.commandRunProjectModule),
    commandCreateProjectType: _extractString(map, CustomDimensionsEnum.commandCreateProjectType),
    commandPackagesNumberPlugins: _extractInt(
      map,
      CustomDimensionsEnum.commandPackagesNumberPlugins,
    ),
    commandPackagesProjectModule: _extractBool(
      map,
      CustomDimensionsEnum.commandPackagesProjectModule,
    ),
    commandRunTargetOsVersion: _extractString(map, CustomDimensionsEnum.commandRunTargetOsVersion),
    commandRunModeName: _extractString(map, CustomDimensionsEnum.commandRunModeName),
    commandBuildBundleTargetPlatform: _extractString(
      map,
      CustomDimensionsEnum.commandBuildBundleTargetPlatform,
    ),
    commandBuildBundleIsModule: _extractBool(map, CustomDimensionsEnum.commandBuildBundleIsModule),
    commandResult: _extractString(map, CustomDimensionsEnum.commandResult),
    hotEventTargetPlatform: _extractString(map, CustomDimensionsEnum.hotEventTargetPlatform),
    hotEventSdkName: _extractString(map, CustomDimensionsEnum.hotEventSdkName),
    hotEventEmulator: _extractBool(map, CustomDimensionsEnum.hotEventEmulator),
    hotEventFullRestart: _extractBool(map, CustomDimensionsEnum.hotEventFullRestart),
    commandHasTerminal: _extractBool(map, CustomDimensionsEnum.commandHasTerminal),
    enabledFlutterFeatures: _extractString(map, CustomDimensionsEnum.enabledFlutterFeatures),
    localTime: _extractString(map, CustomDimensionsEnum.localTime),
    commandBuildAarTargetPlatform: _extractString(
      map,
      CustomDimensionsEnum.commandBuildAarTargetPlatform,
    ),
    commandBuildAarProjectType: _extractString(
      map,
      CustomDimensionsEnum.commandBuildAarProjectType,
    ),
    buildEventCommand: _extractString(map, CustomDimensionsEnum.buildEventCommand),
    buildEventSettings: _extractString(map, CustomDimensionsEnum.buildEventSettings),
    commandBuildApkTargetPlatform: _extractString(
      map,
      CustomDimensionsEnum.commandBuildApkTargetPlatform,
    ),
    commandBuildApkBuildMode: _extractString(map, CustomDimensionsEnum.commandBuildApkBuildMode),
    commandBuildApkSplitPerAbi: _extractBool(map, CustomDimensionsEnum.commandBuildApkSplitPerAbi),
    commandBuildAppBundleTargetPlatform: _extractString(
      map,
      CustomDimensionsEnum.commandBuildAppBundleTargetPlatform,
    ),
    commandBuildAppBundleBuildMode: _extractString(
      map,
      CustomDimensionsEnum.commandBuildAppBundleBuildMode,
    ),
    buildEventError: _extractString(map, CustomDimensionsEnum.buildEventError),
    commandResultEventMaxRss: _extractInt(map, CustomDimensionsEnum.commandResultEventMaxRss),
    commandRunAndroidEmbeddingVersion: _extractString(
      map,
      CustomDimensionsEnum.commandRunAndroidEmbeddingVersion,
    ),
    commandPackagesAndroidEmbeddingVersion: _extractString(
      map,
      CustomDimensionsEnum.commandPackagesAndroidEmbeddingVersion,
    ),
    nullSafety: _extractBool(map, CustomDimensionsEnum.nullSafety),
    nullSafeMigratedLibraries: _extractInt(map, CustomDimensionsEnum.nullSafeMigratedLibraries),
    nullSafeTotalLibraries: _extractInt(map, CustomDimensionsEnum.nullSafeTotalLibraries),
    hotEventCompileTimeInMs: _extractInt(map, CustomDimensionsEnum.hotEventCompileTimeInMs),
    hotEventFindInvalidatedTimeInMs: _extractInt(
      map,
      CustomDimensionsEnum.hotEventFindInvalidatedTimeInMs,
    ),
    hotEventScannedSourcesCount: _extractInt(map, CustomDimensionsEnum.hotEventScannedSourcesCount),
    hotEventReassembleTimeInMs: _extractInt(map, CustomDimensionsEnum.hotEventReassembleTimeInMs),
    hotEventReloadVMTimeInMs: _extractInt(map, CustomDimensionsEnum.hotEventReloadVMTimeInMs),
    commandRunEnableImpeller: _extractBool(map, CustomDimensionsEnum.commandRunEnableImpeller),
    commandRunIOSInterfaceType: _extractString(
      map,
      CustomDimensionsEnum.commandRunIOSInterfaceType,
    ),
    commandRunIsTest: _extractBool(map, CustomDimensionsEnum.commandRunIsTest),
  );

  static bool? _extractBool(Map<String, String> map, CustomDimensionsEnum field) =>
      map.containsKey(field.cdKey) ? map[field.cdKey] == 'true' : null;

  static String? _extractString(Map<String, String> map, CustomDimensionsEnum field) =>
      map.containsKey(field.cdKey) ? map[field.cdKey] : null;

  static int? _extractInt(Map<String, String> map, CustomDimensionsEnum field) =>
      map.containsKey(field.cdKey) ? int.parse(map[field.cdKey]!) : null;

  @override
  String toString() => toMap().toString();

  @override
  bool operator ==(Object other) {
    return other is CustomDimensions && _mapsEqual(other.toMap(), toMap());
  }

  @override
  int get hashCode => Object.hashAll(toMap().values);
}

/// List of all fields used in CustomDimensions.
///
/// The index of this enum is used to calculate the key of the fields. Always
/// append to this list when adding new fields, and do not remove or reorder
/// any elements.
enum CustomDimensionsEnum {
  sessionHostOsDetails, // cd1
  sessionChannelName, // cd2
  commandRunIsEmulator, // cd3
  commandRunTargetName, // cd4
  hotEventReason, // cd5
  hotEventFinalLibraryCount, // cd6
  hotEventSyncedLibraryCount, // cd7
  hotEventSyncedClassesCount, // cd8
  hotEventSyncedProceduresCount, // cd9
  hotEventSyncedBytes, // cd10
  hotEventInvalidatedSourcesCount, // cd11
  hotEventTransferTimeInMs, // cd12
  hotEventOverallTimeInMs, // cd13
  commandRunProjectType, // cd14
  commandRunProjectHostLanguage, // cd15
  commandCreateAndroidLanguage, // cd16
  commandCreateIosLanguage, // cd17
  commandRunProjectModule, // cd18
  commandCreateProjectType, // cd19
  commandPackagesNumberPlugins, // cd20
  commandPackagesProjectModule, // cd21
  commandRunTargetOsVersion, // cd22
  commandRunModeName, // cd23
  commandBuildBundleTargetPlatform, // cd24
  commandBuildBundleIsModule, // cd25
  commandResult, // cd26
  hotEventTargetPlatform, // cd27
  hotEventSdkName, // cd28
  hotEventEmulator, // cd29
  hotEventFullRestart, // cd30
  commandHasTerminal, // cd31
  enabledFlutterFeatures, // cd32
  localTime, // cd33
  commandBuildAarTargetPlatform, // cd34
  commandBuildAarProjectType, // cd35
  buildEventCommand, // cd36
  buildEventSettings, // cd37
  commandBuildApkTargetPlatform, // cd38
  commandBuildApkBuildMode, // cd39
  commandBuildApkSplitPerAbi, // cd40
  commandBuildAppBundleTargetPlatform, // cd41
  commandBuildAppBundleBuildMode, // cd42
  buildEventError, // cd43
  commandResultEventMaxRss, // cd44
  commandRunAndroidEmbeddingVersion, // cd45
  commandPackagesAndroidEmbeddingVersion, // cd46
  nullSafety, // cd47
  obsolete1, // cd48 (was fastReassemble)
  nullSafeMigratedLibraries, // cd49
  nullSafeTotalLibraries, // cd50
  hotEventCompileTimeInMs, // cd51
  hotEventFindInvalidatedTimeInMs, // cd52
  hotEventScannedSourcesCount, // cd53
  hotEventReassembleTimeInMs, // cd54
  hotEventReloadVMTimeInMs, // cd55
  commandRunEnableImpeller, // cd56
  commandRunIOSInterfaceType, // cd57
  commandRunIsTest; // cd58

  String get cdKey => 'cd${index + 1}';
}
