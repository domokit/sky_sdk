// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter_tools/src/application_package.dart';
import 'package:flutter_tools/src/asset.dart';
import 'package:flutter_tools/src/base/dds.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/tools/shader_compiler.dart';
import 'package:flutter_tools/src/compile.dart';
import 'package:flutter_tools/src/devfs.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/device_port_forwarder.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/resident_runner.dart';
import 'package:flutter_tools/src/run_cold.dart';
import 'package:flutter_tools/src/run_hot.dart';
import 'package:flutter_tools/src/vmservice.dart';
import 'package:package_config/package_config.dart';
import 'package:test/fake.dart';
import 'package:vm_service/vm_service.dart' as vm_service;

import '../src/fake_vm_services.dart';

final vm_service.Event fakeUnpausedEvent = vm_service.Event(
  kind: vm_service.EventKind.kResume,
  timestamp: 0,
);

final vm_service.Event fakePausedEvent = vm_service.Event(
  kind: vm_service.EventKind.kPauseException,
  timestamp: 0,
);

final vm_service.Isolate fakeUnpausedIsolate = vm_service.Isolate(
  id: '1',
  pauseEvent: fakeUnpausedEvent,
  breakpoints: <vm_service.Breakpoint>[],
  extensionRPCs: <String>[],
  libraries: <vm_service.LibraryRef>[
    vm_service.LibraryRef(id: '1', uri: 'file:///hello_world/main.dart', name: ''),
  ],
  livePorts: 0,
  name: 'test',
  number: '1',
  pauseOnExit: false,
  runnable: true,
  startTime: 0,
  isSystemIsolate: false,
  isolateFlags: <vm_service.IsolateFlag>[],
);

final vm_service.Isolate fakePausedIsolate = vm_service.Isolate(
  id: '1',
  pauseEvent: fakePausedEvent,
  breakpoints: <vm_service.Breakpoint>[
    vm_service.Breakpoint(
      breakpointNumber: 123,
      id: 'test-breakpoint',
      location: vm_service.SourceLocation(
        tokenPos: 0,
        script: vm_service.ScriptRef(id: 'test-script', uri: 'foo.dart'),
      ),
      enabled: true,
      resolved: true,
    ),
  ],
  libraries: <vm_service.LibraryRef>[],
  livePorts: 0,
  name: 'test',
  number: '1',
  pauseOnExit: false,
  runnable: true,
  startTime: 0,
  isSystemIsolate: false,
  isolateFlags: <vm_service.IsolateFlag>[],
);

final vm_service.VM fakeVM = vm_service.VM(
  isolates: <vm_service.IsolateRef>[fakeUnpausedIsolate],
  pid: 1,
  hostCPU: '',
  isolateGroups: <vm_service.IsolateGroupRef>[],
  targetCPU: '',
  startTime: 0,
  name: 'dart',
  architectureBits: 64,
  operatingSystem: '',
  version: '',
  systemIsolateGroups: <vm_service.IsolateGroupRef>[],
  systemIsolates: <vm_service.IsolateRef>[],
);

final FlutterView fakeFlutterView = FlutterView(id: 'a', uiIsolate: fakeUnpausedIsolate);

final FakeVmServiceRequest listViews = FakeVmServiceRequest(
  method: kListViewsMethod,
  jsonResponse: <String, Object>{
    'views': <Object>[fakeFlutterView.toJson()],
  },
);

const FakeVmServiceRequest setAssetBundlePath = FakeVmServiceRequest(
  method: '_flutter.setAssetBundlePath',
  args: <String, Object>{'viewId': 'a', 'assetDirectory': 'build/flutter_assets', 'isolateId': '1'},
);

const FakeVmServiceRequest evict = FakeVmServiceRequest(
  method: 'ext.flutter.evict',
  args: <String, Object>{'value': 'asset', 'isolateId': '1'},
);

const FakeVmServiceRequest evictShader = FakeVmServiceRequest(
  method: 'ext.ui.window.reinitializeShader',
  args: <String, Object>{'assetKey': 'foo.frag', 'isolateId': '1'},
);

final Uri testUri = Uri.parse('foo://bar');

class FakeDartDevelopmentService extends Fake
    with DartDevelopmentServiceLocalOperationsMixin
    implements DartDevelopmentService {
  @override
  Future<void> get done => Future<void>.value();

  @override
  Uri? get uri => null;

  @override
  void shutdown() {}
}

class FakeDartDevelopmentServiceException implements DartDevelopmentServiceException {
  FakeDartDevelopmentServiceException({this.message = defaultMessage});

  @override
  final int errorCode = DartDevelopmentServiceException.existingDdsInstanceError;

  @override
  final String message;
  static const String defaultMessage =
      'A DDS instance is already connected at http://localhost:8181';

  @override
  Map<String, Object?> toJson() {
    throw UnimplementedError();
  }
}

class TestFlutterDevice extends FlutterDevice {
  TestFlutterDevice(super.device, {Stream<Uri>? vmServiceUris})
    : _vmServiceUris = vmServiceUris,
      super(buildInfo: BuildInfo.debug, developmentShaderCompiler: const FakeShaderCompiler());

  final Stream<Uri>? _vmServiceUris;

  @override
  Stream<Uri> get vmServiceUris => _vmServiceUris!;
}

class ThrowingForwardingFileSystem extends ForwardingFileSystem {
  ThrowingForwardingFileSystem(super.delegate);

  @override
  File file(dynamic path) {
    if (path == 'foo') {
      throw const FileSystemException();
    }
    return delegate.file(path);
  }
}

class FakeFlutterDevice extends Fake implements FlutterDevice {
  FakeVmServiceHost? Function()? vmServiceHost;
  Uri? testUri;
  UpdateFSReport report = UpdateFSReport(success: true, invalidatedSourcesCount: 1);
  Exception? reportError;
  Exception? runColdError;
  int runHotCode = 0;
  int runColdCode = 0;

  @override
  ResidentCompiler? generator;

  @override
  DevelopmentShaderCompiler get developmentShaderCompiler => const FakeShaderCompiler();

  @override
  TargetPlatform targetPlatform = TargetPlatform.android;

  @override
  Stream<Uri?> get vmServiceUris => Stream<Uri?>.value(testUri);

  @override
  FlutterVmService? get vmService => vmServiceHost?.call()?.vmService;

  DevFS? fakeDevFS;

  @override
  DevFS? get devFS => fakeDevFS;

  @override
  set devFS(DevFS? value) {}

  @override
  Device? device;

  @override
  Future<void> stopEchoingDeviceLog() async {}

  @override
  Future<Uri> setupDevFS(String fsName, Directory rootDirectory) async {
    return testUri!;
  }

  @override
  Future<int> runHot({required HotRunner hotRunner, String? route}) async {
    return runHotCode;
  }

  @override
  Future<int> runCold({required ColdRunner coldRunner, String? route}) async {
    if (runColdError != null) {
      throw runColdError!;
    }
    return runColdCode;
  }

  @override
  Future<void> connect({
    ReloadSources? reloadSources,
    Restart? restart,
    CompileExpression? compileExpression,
    FlutterProject? flutterProject,
    PrintStructuredErrorLogMethod? printStructuredErrorLogMethod,
    required DebuggingOptions debuggingOptions,
    int? hostVmServicePort,
    bool? ipv6 = false,
    bool allowExistingDdsInstance = false,
  }) async {}

  @override
  Future<UpdateFSReport> updateDevFS({
    required Uri mainUri,
    String? target,
    AssetBundle? bundle,
    bool bundleFirstUpload = false,
    bool bundleDirty = false,
    bool fullRestart = false,
    String? projectRootPath,
    required String pathToReload,
    required String dillOutputPath,
    required List<Uri> invalidatedFiles,
    required PackageConfig packageConfig,
  }) async {
    if (reportError != null) {
      throw reportError!;
    }
    return report;
  }

  @override
  Future<void> updateReloadStatus(bool wasReloadSuccessful) async {}
}

class FakeDelegateFlutterDevice extends FlutterDevice {
  FakeDelegateFlutterDevice(
    super.device,
    BuildInfo buildInfo,
    ResidentCompiler residentCompiler,
    this.fakeDevFS,
  ) : super(
        buildInfo: buildInfo,
        generator: residentCompiler,
        developmentShaderCompiler: const FakeShaderCompiler(),
      );

  @override
  Future<void> connect({
    ReloadSources? reloadSources,
    Restart? restart,
    CompileExpression? compileExpression,
    FlutterProject? flutterProject,
    PrintStructuredErrorLogMethod? printStructuredErrorLogMethod,
    required DebuggingOptions debuggingOptions,
    int? hostVmServicePort,
    bool? ipv6 = false,
    bool allowExistingDdsInstance = false,
  }) async {}

  final DevFS fakeDevFS;

  @override
  DevFS? get devFS => fakeDevFS;

  @override
  set devFS(DevFS? value) {}
}

class FakeResidentCompiler extends Fake implements ResidentCompiler {
  CompilerOutput? nextOutput;
  bool didSuppressErrors = false;
  Uri? receivedNativeAssetsYaml;
  bool recompileCalled = false;

  @override
  Future<CompilerOutput?> recompile(
    Uri mainUri,
    List<Uri>? invalidatedFiles, {
    required String outputPath,
    required PackageConfig packageConfig,
    String? projectRootPath,
    required FileSystem fs,
    bool suppressErrors = false,
    bool checkDartPluginRegistry = false,
    File? dartPluginRegistrant,
    Uri? nativeAssetsYaml,
    bool recompileRestart = false,
  }) async {
    recompileCalled = true;
    receivedNativeAssetsYaml = nativeAssetsYaml;
    didSuppressErrors = suppressErrors;
    return nextOutput ?? const CompilerOutput('foo.dill', 0, <Uri>[]);
  }

  @override
  void accept() {}

  @override
  void reset() {}
}

class FakeProjectFileInvalidator extends Fake implements ProjectFileInvalidator {
  @override
  Future<InvalidationResult> findInvalidated({
    required DateTime? lastCompiled,
    required List<Uri> urisToMonitor,
    required String packagesPath,
    required PackageConfig packageConfig,
    bool asyncScanning = false,
  }) async {
    return InvalidationResult(
      packageConfig: packageConfig,
      uris: <Uri>[Uri.parse('file:///hello_world/main.dart')],
    );
  }
}

class FakeDevice extends Fake implements Device {
  FakeDevice({
    String sdkNameAndVersion = 'Android',
    TargetPlatform targetPlatform = TargetPlatform.android_arm,
    bool isLocalEmulator = false,
    this.supportsHotRestart = true,
    this.supportsScreenshot = true,
    this.supportsFlutterExit = true,
  }) : _isLocalEmulator = isLocalEmulator,
       _targetPlatform = targetPlatform,
       _sdkNameAndVersion = sdkNameAndVersion;

  final bool _isLocalEmulator;
  final TargetPlatform _targetPlatform;
  final String _sdkNameAndVersion;

  bool disposed = false;
  bool appStopped = false;
  bool failScreenshot = false;

  @override
  bool supportsHotRestart;

  @override
  bool supportsScreenshot;

  @override
  bool supportsFlutterExit;

  @override
  PlatformType get platformType =>
      _targetPlatform == TargetPlatform.web_javascript ? PlatformType.web : PlatformType.android;

  @override
  Future<String> get sdkNameAndVersion async => _sdkNameAndVersion;

  @override
  Future<TargetPlatform> get targetPlatform async => _targetPlatform;

  @override
  Future<bool> get isLocalEmulator async => _isLocalEmulator;

  @override
  String get name => 'FakeDevice';

  @override
  String get displayName => name;

  @override
  late DartDevelopmentService dds = FakeDartDevelopmentService();

  @override
  Future<void> dispose() async {
    disposed = true;
  }

  @override
  Future<bool> stopApp(ApplicationPackage? app, {String? userIdentifier}) async {
    appStopped = true;
    return true;
  }

  @override
  Future<void> takeScreenshot(File outputFile) async {
    if (failScreenshot) {
      throw Exception();
    }
    outputFile.writeAsBytesSync(List<int>.generate(1024, (int i) => i));
  }

  @override
  FutureOr<DeviceLogReader> getLogReader({ApplicationPackage? app, bool includePastLogs = false}) =>
      NoOpDeviceLogReader(name);

  @override
  DevicePortForwarder portForwarder = const NoOpDevicePortForwarder();
}

class FakeDevFS extends Fake implements DevFS {
  @override
  DateTime? lastCompiled = DateTime(2000);

  @override
  PackageConfig? lastPackageConfig = PackageConfig.empty;

  @override
  List<Uri> sources = <Uri>[];

  @override
  Uri baseUri = Uri();

  @override
  Future<void> destroy() async {}

  @override
  Set<String> assetPathsToEvict = <String>{};

  @override
  Set<String> shaderPathsToEvict = <String>{};

  @override
  bool didUpdateFontManifest = false;

  UpdateFSReport nextUpdateReport = UpdateFSReport(success: true);

  @override
  bool hasSetAssetDirectory = false;

  @override
  Future<Uri> create() async {
    return Uri();
  }

  @override
  void resetLastCompiled() {
    lastCompiled = null;
  }

  @override
  Future<UpdateFSReport> update({
    required Uri mainUri,
    required ResidentCompiler generator,
    required bool trackWidgetCreation,
    required String pathToReload,
    required List<Uri> invalidatedFiles,
    required PackageConfig packageConfig,
    required String dillOutputPath,
    required DevelopmentShaderCompiler shaderCompiler,
    DevFSWriter? devFSWriter,
    String? target,
    AssetBundle? bundle,
    bool bundleFirstUpload = false,
    bool fullRestart = false,
    bool resetCompiler = false,
    String? projectRootPath,
    File? dartPluginRegistrant,
  }) async {
    return nextUpdateReport;
  }
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
