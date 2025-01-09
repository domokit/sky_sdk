// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:dds/dap.dart';

/// An implementation of [AttachRequestArguments] that includes all fields used by the Flutter debug adapter.
///
/// This class represents the data passed from the client editor to the debug
/// adapter in attachRequest, which is a request to attach to/debug a running
/// application.
class FlutterAttachRequestArguments extends DartCommonLaunchAttachRequestArguments
    implements AttachRequestArguments {
  FlutterAttachRequestArguments({
    this.toolArgs,
    this.customTool,
    this.customToolReplacesArgs,
    this.vmServiceUri,
    this.vmServiceInfoFile,
    this.program,
    super.restart,
    super.name,
    super.cwd,
    super.env,
    super.additionalProjectPaths,
    super.allowAnsiColorOutput,
    super.debugSdkLibraries,
    super.debugExternalPackageLibraries,
    super.evaluateGettersInDebugViews,
    super.evaluateToStringInDebugViews,
    super.sendLogsToClient,
    super.sendCustomProgressEvents,
  });

  FlutterAttachRequestArguments.fromMap(super.obj)
    : toolArgs = (obj['toolArgs'] as List<Object?>?)?.cast<String>(),
      customTool = obj['customTool'] as String?,
      customToolReplacesArgs = obj['customToolReplacesArgs'] as int?,
      vmServiceUri = obj['vmServiceUri'] as String?,
      vmServiceInfoFile = obj['vmServiceInfoFile'] as String?,
      program = obj['program'] as String?,
      super.fromMap();

  factory FlutterAttachRequestArguments.fromJson(Map<String, Object?> obj) =
      FlutterAttachRequestArguments.fromMap;

  /// Arguments to be passed to the tool that will run [program] (for example, the VM or Flutter tool).
  final List<String>? toolArgs;

  /// An optional tool to run instead of "flutter".
  ///
  /// In combination with [customToolReplacesArgs] allows invoking a custom
  /// tool instead of "flutter" to launch scripts/tests. The custom tool must be
  /// completely compatible with the tool/command it is replacing.
  ///
  /// This field should be a full absolute path if the tool may not be available
  /// in `PATH`.
  final String? customTool;

  /// The number of arguments to delete from the beginning of the argument list
  /// when invoking [customTool].
  ///
  /// For example, setting [customTool] to `flutter_test_wrapper` and
  /// `customToolReplacesArgs` to `1` for a test run would invoke
  /// `flutter_test_wrapper foo_test.dart` instead of `flutter test foo_test.dart`.
  final int? customToolReplacesArgs;

  /// The VM Service URI of the running Flutter app to connect to.
  ///
  /// Only one of this or [vmServiceInfoFile] (or neither) can be supplied.
  final String? vmServiceUri;

  /// The VM Service info file to extract the VM Service URI from to attach to.
  ///
  /// Only one of this or [vmServiceUri] (or neither) can be supplied.
  final String? vmServiceInfoFile;

  /// The program/Flutter app to be run.
  final String? program;

  @override
  Map<String, Object?> toJson() => <String, Object?>{
    ...super.toJson(),
    if (toolArgs != null) 'toolArgs': toolArgs,
    if (customTool != null) 'customTool': customTool,
    if (customToolReplacesArgs != null) 'customToolReplacesArgs': customToolReplacesArgs,
    if (vmServiceUri != null) 'vmServiceUri': vmServiceUri,
  };
}

/// An implementation of [LaunchRequestArguments] that includes all fields used by the Flutter debug adapter.
///
/// This class represents the data passed from the client editor to the debug
/// adapter in launchRequest, which is a request to start debugging an
/// application.
class FlutterLaunchRequestArguments extends DartCommonLaunchAttachRequestArguments
    implements LaunchRequestArguments {
  FlutterLaunchRequestArguments({
    this.noDebug,
    required this.program,
    this.args,
    this.toolArgs,
    this.customTool,
    this.customToolReplacesArgs,
    super.restart,
    super.name,
    super.cwd,
    super.env,
    super.additionalProjectPaths,
    super.allowAnsiColorOutput,
    super.debugSdkLibraries,
    super.debugExternalPackageLibraries,
    super.evaluateGettersInDebugViews,
    super.evaluateToStringInDebugViews,
    super.sendLogsToClient,
    super.sendCustomProgressEvents,
  });

  FlutterLaunchRequestArguments.fromMap(super.obj)
    : noDebug = obj['noDebug'] as bool?,
      program = obj['program'] as String?,
      args = (obj['args'] as List<Object?>?)?.cast<String>(),
      toolArgs = (obj['toolArgs'] as List<Object?>?)?.cast<String>(),
      customTool = obj['customTool'] as String?,
      customToolReplacesArgs = obj['customToolReplacesArgs'] as int?,
      super.fromMap();

  factory FlutterLaunchRequestArguments.fromJson(Map<String, Object?> obj) =
      FlutterLaunchRequestArguments.fromMap;

  /// If noDebug is true the launch request should launch the program without enabling debugging.
  @override
  final bool? noDebug;

  /// The program/Flutter app to be run.
  final String? program;

  /// Arguments to be passed to [program].
  final List<String>? args;

  /// Arguments to be passed to the tool that will run [program] (for example, the VM or Flutter tool).
  final List<String>? toolArgs;

  /// An optional tool to run instead of "flutter".
  ///
  /// In combination with [customToolReplacesArgs] allows invoking a custom
  /// tool instead of "flutter" to launch scripts/tests. The custom tool must be
  /// completely compatible with the tool/command it is replacing.
  ///
  /// This field should be a full absolute path if the tool may not be available
  /// in `PATH`.
  final String? customTool;

  /// The number of arguments to delete from the beginning of the argument list
  /// when invoking [customTool].
  ///
  /// For example, setting [customTool] to `flutter_test_wrapper` and
  /// `customToolReplacesArgs` to `1` for a test run would invoke
  /// `flutter_test_wrapper foo_test.dart` instead of `flutter test foo_test.dart`.
  final int? customToolReplacesArgs;

  @override
  Map<String, Object?> toJson() => <String, Object?>{
    ...super.toJson(),
    if (noDebug != null) 'noDebug': noDebug,
    if (program != null) 'program': program,
    if (args != null) 'args': args,
    if (toolArgs != null) 'toolArgs': toolArgs,
    if (customTool != null) 'customTool': customTool,
    if (customToolReplacesArgs != null) 'customToolReplacesArgs': customToolReplacesArgs,
  };
}
