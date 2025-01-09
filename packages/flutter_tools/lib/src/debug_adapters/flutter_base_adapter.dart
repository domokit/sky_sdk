// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:dds/dap.dart' hide PidTracker;
import 'package:vm_service/vm_service.dart' as vm;

import '../base/file_system.dart';
import '../base/io.dart';
import '../base/platform.dart';
import '../cache.dart';
import '../convert.dart';
import 'flutter_adapter_args.dart';
import 'mixins.dart';

/// A base DAP Debug Adapter for Flutter applications and tests.
abstract class FlutterBaseDebugAdapter
    extends DartDebugAdapter<FlutterLaunchRequestArguments, FlutterAttachRequestArguments>
    with PidTracker {
  FlutterBaseDebugAdapter(
    super.channel, {
    required this.fileSystem,
    required this.platform,
    super.ipv6,
    this.enableFlutterDds = true,
    super.enableAuthCodes,
    super.logger,
    super.onError,
  }) : flutterSdkRoot = Cache.flutterRoot!,
       // Always disable in the DAP layer as it's handled in the spawned
       // 'flutter' process.
       super(enableDds: false) {
    configureOrgDartlangSdkMappings();
  }

  FileSystem fileSystem;
  Platform platform;
  Process? process;

  final String flutterSdkRoot;

  /// Whether DDS should be enabled in the Flutter process.
  ///
  /// We never enable DDS in the DAP process for Flutter, so this value is not
  /// the same as what is passed to the base class, which is always provided 'false'.
  final bool enableFlutterDds;

  @override
  final FlutterLaunchRequestArguments Function(Map<String, Object?> obj) parseLaunchArgs =
      FlutterLaunchRequestArguments.fromJson;

  @override
  final FlutterAttachRequestArguments Function(Map<String, Object?> obj) parseAttachArgs =
      FlutterAttachRequestArguments.fromJson;

  /// Whether the VM Service closing should be used as a signal to terminate the debug session.
  ///
  /// Since we always have a process for Flutter (whether run or attach) we'll
  /// always use its termination instead, so this is always false.
  @override
  bool get terminateOnVmServiceClose => false;

  /// Whether or not the user requested debugging be enabled.
  ///
  /// For debugging to be enabled, the user must have chosen "Debug" (and not
  /// "Run") in the editor (which maps to the DAP `noDebug` field).
  bool get enableDebugger {
    final DartCommonLaunchAttachRequestArguments args = this.args;
    if (args is FlutterLaunchRequestArguments) {
      // Invert DAP's noDebug flag, treating it as false (so _do_ debug) if not
      // provided.
      return !(args.noDebug ?? false);
    }

    // Otherwise (attach), always debug.
    return true;
  }

  void configureOrgDartlangSdkMappings() {
    /// When a user navigates into 'dart:xxx' sources in their editor (via the
    /// analysis server) they will land in flutter_sdk/bin/cache/pkg/sky_engine.
    ///
    /// The running VM knows nothing about these paths and will resolve these
    /// libraries to 'org-dartlang-sdk://' URIs. We need to map between these
    /// to ensure that if a user puts a breakpoint inside sky_engine the VM can
    /// apply it to the correct place and once hit, we can navigate the user
    /// back to the correct file on their disk.
    ///
    /// The mapping is handled by the base adapter but we need to override the
    /// paths to match the layout used by Flutter.
    ///
    /// In future this might become unnecessary if
    /// https://github.com/dart-lang/sdk/issues/48435 is implemented. Until
    /// then, providing these mappings improves the debugging experience.

    // Clear original Dart SDK mappings because they're not valid here.
    orgDartlangSdkMappings.clear();

    // 'dart:ui' maps to /flutter/lib/ui
    final String flutterRoot = fileSystem.path.join(
      flutterSdkRoot,
      'bin',
      'cache',
      'pkg',
      'sky_engine',
      'lib',
      'ui',
    );
    orgDartlangSdkMappings[flutterRoot] = Uri.parse('org-dartlang-sdk:///flutter/lib/ui');

    // The rest of the Dart SDK maps to /flutter/third_party/dart/sdk
    final String dartRoot = fileSystem.path.join(
      flutterSdkRoot,
      'bin',
      'cache',
      'pkg',
      'sky_engine',
    );
    orgDartlangSdkMappings[dartRoot] = Uri.parse(
      'org-dartlang-sdk:///flutter/third_party/dart/sdk',
    );
  }

  @override
  Future<void> debuggerConnected(vm.VM vmInfo) async {
    // Usually we'd capture the pid from the VM here and record it for
    // terminating, however for Flutter apps it may be running on a remote
    // device so it's not valid to terminate a process with that pid locally.
    // For attach, pids should never be collected as terminateRequest() should
    // not terminate the debugger.
  }

  /// Called by [disconnectRequest] to request that we forcefully shut down the app being run (or in the case of an attach, disconnect).
  ///
  /// Client IDEs/editors should send a terminateRequest before a
  /// disconnectRequest to allow a graceful shutdown. This method must terminate
  /// quickly and therefore may leave orphaned processes.
  @override
  Future<void> disconnectImpl() async {
    if (isAttach) {
      await handleDetach();
    }
    terminatePids(ProcessSignal.sigkill);
  }

  Future<void> launchAsProcess({
    required String executable,
    required List<String> processArgs,
    required Map<String, String>? env,
  }) async {
    final Process process = await (
      String executable,
      List<String> processArgs, {
      required Map<String, String>? env,
    }) async {
      logger?.call('Spawning $executable with $processArgs in ${args.cwd}');
      final Process process = await Process.start(
        executable,
        processArgs,
        workingDirectory: args.cwd,
        environment: env,
      );
      pidsToTerminate.add(process.pid);
      return process;
    }(executable, processArgs, env: env);
    this.process = process;

    process.stdout.transform(ByteToLineTransformer()).listen(handleStdout);
    process.stderr.transform(utf8.decoder).listen(handleStderr);
    unawaited(process.exitCode.then(handleExitCode));
  }

  void handleExitCode(int code);
  void handleStderr(String data);
  void handleStdout(String data);
}
