// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/application_package.dart';
import 'package:flutter_tools/src/asset.dart';
import 'package:flutter_tools/src/base/dds.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/tools/shader_compiler.dart';
import 'package:flutter_tools/src/compile.dart';
import 'package:flutter_tools/src/devfs.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/resident_runner.dart';
import 'package:flutter_tools/src/run_hot.dart';
import 'package:flutter_tools/src/vmservice.dart';
import 'package:package_config/package_config.dart';
import 'package:test/fake.dart';
import 'package:vm_service/vm_service.dart' as vm_service;

class FakeDevFs extends Fake implements DevFS {
  @override
  Future<void> destroy() async {}

  @override
  List<Uri> sources = <Uri>[];

  @override
  DateTime? lastCompiled;

  @override
  PackageConfig? lastPackageConfig;

  @override
  Set<String> assetPathsToEvict = <String>{};

  @override
  Set<String> shaderPathsToEvict = <String>{};

  @override
  Set<String> scenePathsToEvict = <String>{};

  @override
  Uri? baseUri;
}

class FakeDevice extends Fake implements Device {
  FakeDevice({TargetPlatform targetPlatform = TargetPlatform.tester})
    : _targetPlatform = targetPlatform;

  final TargetPlatform _targetPlatform;

  bool disposed = false;

  @override
  final DartDevelopmentService dds = FakeDartDevelopmentService();

  @override
  bool isSupported() => true;

  @override
  bool supportsHotReload = true;

  @override
  bool supportsHotRestart = true;

  @override
  bool supportsFlutterExit = true;

  @override
  Future<TargetPlatform> get targetPlatform async => _targetPlatform;

  @override
  Future<String> get sdkNameAndVersion async => 'Tester';

  @override
  Future<bool> get isLocalEmulator async => false;

  @override
  String get name => 'Fake Device';

  @override
  Future<bool> stopApp(ApplicationPackage? app, {String? userIdentifier}) async {
    return true;
  }

  @override
  Future<void> dispose() async {
    disposed = true;
  }
}

class FakeDartDevelopmentService extends Fake implements DartDevelopmentService {
  bool wasShutdown = false;

  @override
  void shutdown() {
    wasShutdown = true;
  }
}

class FakeFlutterDevice extends Fake implements FlutterDevice {
  FakeFlutterDevice(this.device);

  bool stoppedEchoingDeviceLog = false;
  late Future<UpdateFSReport> Function() updateDevFSReportCallback;

  @override
  final FakeDevice device;

  @override
  Future<void> stopEchoingDeviceLog() async {
    stoppedEchoingDeviceLog = true;
  }

  @override
  DevFS? devFS = FakeDevFs();

  @override
  FlutterVmService get vmService => FakeFlutterVmService();

  @override
  ResidentCompiler? generator;

  @override
  Future<UpdateFSReport> updateDevFS({
    Uri? mainUri,
    String? target,
    AssetBundle? bundle,
    bool bundleFirstUpload = false,
    bool bundleDirty = false,
    bool fullRestart = false,
    String? projectRootPath,
    String? pathToReload,
    required String dillOutputPath,
    required List<Uri> invalidatedFiles,
    required PackageConfig packageConfig,
  }) => updateDevFSReportCallback();

  @override
  TargetPlatform? get targetPlatform => device._targetPlatform;
}

class TestFlutterDevice extends FlutterDevice {
  TestFlutterDevice({
    required Device device,
    required this.exception,
    required ResidentCompiler generator,
  }) : super(
         device,
         buildInfo: BuildInfo.debug,
         generator: generator,
         developmentShaderCompiler: const FakeShaderCompiler(),
       );

  /// The exception to throw when the connect method is called.
  final Exception exception;

  @override
  Future<void> connect({
    ReloadSources? reloadSources,
    Restart? restart,
    CompileExpression? compileExpression,
    GetSkSLMethod? getSkSLMethod,
    FlutterProject? flutterProject,
    PrintStructuredErrorLogMethod? printStructuredErrorLogMethod,
    required DebuggingOptions debuggingOptions,
    int? hostVmServicePort,
    bool? ipv6 = false,
    bool enableDevTools = false,
    bool allowExistingDdsInstance = false,
  }) async {
    throw exception;
  }
}

class TestHotRunnerConfig extends HotRunnerConfig {
  TestHotRunnerConfig({this.successfulHotRestartSetup, this.successfulHotReloadSetup});
  bool? successfulHotRestartSetup;
  bool? successfulHotReloadSetup;
  bool shutdownHookCalled = false;
  bool updateDevFSCompleteCalled = false;

  @override
  Future<bool?> setupHotRestart() async {
    assert(
      successfulHotRestartSetup != null,
      'setupHotRestart is not expected to be called in this test.',
    );
    return successfulHotRestartSetup;
  }

  @override
  Future<bool?> setupHotReload() async {
    assert(
      successfulHotReloadSetup != null,
      'setupHotReload is not expected to be called in this test.',
    );
    return successfulHotReloadSetup;
  }

  @override
  void updateDevFSComplete() {
    updateDevFSCompleteCalled = true;
  }

  @override
  Future<void> runPreShutdownOperations() async {
    shutdownHookCalled = true;
  }
}

class FakeResidentCompiler extends Fake implements ResidentCompiler {
  @override
  void accept() {}
}

class FakeFlutterVmService extends Fake implements FlutterVmService {
  @override
  vm_service.VmService get service => FakeVmService();

  @override
  Future<List<FlutterView>> getFlutterViews({
    bool returnEarly = false,
    Duration delay = const Duration(milliseconds: 50),
  }) async {
    return <FlutterView>[];
  }
}

class FakeVmService extends Fake implements vm_service.VmService {
  @override
  Future<vm_service.VM> getVM() async => FakeVm();
}

class FakeVm extends Fake implements vm_service.VM {
  @override
  List<vm_service.IsolateRef> get isolates => <vm_service.IsolateRef>[];
}

class FakeShaderCompiler implements DevelopmentShaderCompiler {
  const FakeShaderCompiler();

  @override
  void configureCompiler(TargetPlatform? platform) {}

  @override
  Future<DevFSContent> recompileShader(DevFSContent inputShader) {
    throw UnimplementedError();
  }
}
