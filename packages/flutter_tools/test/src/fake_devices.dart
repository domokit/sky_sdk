// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter_tools/src/application_package.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/vmservice.dart';

import 'fakes.dart';

/// A list of fake devices to test JSON serialization
/// (`Device.toJson()` and `--machine` flag for `devices` command)
List<FakeDeviceJsonData> fakeDevices = <FakeDeviceJsonData>[
  FakeDeviceJsonData(
    FakeDevice('ephemeral', 'ephemeral', type: PlatformType.android),
    <String, Object>{
      'name': 'ephemeral',
      'id': 'ephemeral',
      'isSupported': true,
      'targetPlatform': 'android-arm',
      'emulator': true,
      'sdk': 'Test SDK (1.2.3)',
      'capabilities': <String, Object>{
        'hotReload': true,
        'hotRestart': true,
        'screenshot': false,
        'fastStart': false,
        'flutterExit': true,
        'hardwareRendering': true,
        'startPaused': true,
      },
    },
  ),
  FakeDeviceJsonData(
    FakeDevice('webby', 'webby')
      ..targetPlatform = Future<TargetPlatform>.value(TargetPlatform.web_javascript)
      ..sdkNameAndVersion = Future<String>.value('Web SDK (1.2.4)'),
    <String, Object>{
      'name': 'webby',
      'id': 'webby',
      'isSupported': true,
      'targetPlatform': 'web-javascript',
      'emulator': true,
      'sdk': 'Web SDK (1.2.4)',
      'capabilities': <String, Object>{
        'hotReload': true,
        'hotRestart': true,
        'screenshot': false,
        'fastStart': false,
        'flutterExit': true,
        'hardwareRendering': true,
        'startPaused': true,
      },
    },
  ),
  FakeDeviceJsonData(
    FakeDevice(
      'wireless android',
      'wireless-android',
      type: PlatformType.android,
      connectionInterface: DeviceConnectionInterface.wireless,
    ),
    <String, Object>{
      'name': 'wireless android',
      'id': 'wireless-android',
      'isSupported': true,
      'targetPlatform': 'android-arm',
      'emulator': true,
      'sdk': 'Test SDK (1.2.3)',
      'capabilities': <String, Object>{
        'hotReload': true,
        'hotRestart': true,
        'screenshot': false,
        'fastStart': false,
        'flutterExit': true,
        'hardwareRendering': true,
        'startPaused': true,
      },
    },
  ),
  FakeDeviceJsonData(
    FakeDevice(
        'wireless ios',
        'wireless-ios',
        type: PlatformType.ios,
        connectionInterface: DeviceConnectionInterface.wireless,
      )
      ..targetPlatform = Future<TargetPlatform>.value(TargetPlatform.ios)
      ..sdkNameAndVersion = Future<String>.value('iOS 16'),
    <String, Object>{
      'name': 'wireless ios',
      'id': 'wireless-ios',
      'isSupported': true,
      'targetPlatform': 'ios',
      'emulator': true,
      'sdk': 'iOS 16',
      'capabilities': <String, Object>{
        'hotReload': true,
        'hotRestart': true,
        'screenshot': false,
        'fastStart': false,
        'flutterExit': true,
        'hardwareRendering': true,
        'startPaused': true,
      },
    },
  ),
];

/// Fake device to test `devices` command.
class FakeDevice extends Device {
  FakeDevice(
    this.name,
    String id, {
    bool ephemeral = true,
    bool isSupported = true,
    bool isSupportedForProject = true,
    this.isConnected = true,
    this.connectionInterface = DeviceConnectionInterface.attached,
    PlatformType type = PlatformType.web,
    LaunchResult? launchResult,
    this.deviceLogReader,
    bool supportsFlavors = false,
  }) : _isSupported = isSupported,
       _isSupportedForProject = isSupportedForProject,
       _launchResult = launchResult ?? LaunchResult.succeeded(),
       _supportsFlavors = supportsFlavors,
       super(
         id,
         platformType: type,
         category: Category.mobile,
         ephemeral: ephemeral,
         logger: FakeLogger(),
       );

  final bool _isSupported;
  final bool _isSupportedForProject;
  final bool _supportsFlavors;
  final LaunchResult _launchResult;
  DeviceLogReader? deviceLogReader;

  @override
  final String name;

  @override
  Future<LaunchResult> startApp(
    ApplicationPackage? package, {
    String? mainPath,
    String? route,
    DebuggingOptions? debuggingOptions,
    Map<String, dynamic>? platformArgs,
    bool prebuiltApplication = false,
    bool ipv6 = false,
    String? userIdentifier,
  }) async => _launchResult;

  @override
  Future<bool> stopApp(ApplicationPackage? app, {String? userIdentifier}) async => true;

  @override
  Future<bool> uninstallApp(ApplicationPackage app, {String? userIdentifier}) async => true;

  @override
  Future<void> dispose() async {}

  @override
  Future<TargetPlatform> targetPlatform = Future<TargetPlatform>.value(TargetPlatform.android_arm);

  @override
  void noSuchMethod(Invocation invocation) => super.noSuchMethod(invocation);

  @override
  bool isSupportedForProject(FlutterProject flutterProject) => _isSupportedForProject;

  @override
  bool isSupported() => _isSupported;

  @override
  bool get supportsFlavors => _supportsFlavors;

  @override
  bool isConnected;

  @override
  DeviceConnectionInterface connectionInterface;

  @override
  Future<bool> isLocalEmulator = Future<bool>.value(true);

  @override
  Future<String> sdkNameAndVersion = Future<String>.value('Test SDK (1.2.3)');

  @override
  FutureOr<DeviceLogReader> getLogReader({ApplicationPackage? app, bool includePastLogs = false}) =>
      deviceLogReader ?? FakeDeviceLogReader();
}

/// Combines fake device with its canonical JSON representation.
class FakeDeviceJsonData {
  FakeDeviceJsonData(this.dev, this.json);

  final FakeDevice dev;
  final Map<String, Object> json;
}

class FakePollingDeviceDiscovery extends PollingDeviceDiscovery {
  FakePollingDeviceDiscovery({this.requiresExtendedWirelessDeviceDiscovery = false})
    : super('mock');

  final List<Device> _devices = <Device>[];
  final StreamController<Device> _onAddedController = StreamController<Device>.broadcast();
  final StreamController<Device> _onRemovedController = StreamController<Device>.broadcast();

  @override
  Future<List<Device>> pollingGetDevices({Duration? timeout}) async {
    lastPollingTimeout = timeout;
    return _devices;
  }

  Duration? lastPollingTimeout;

  @override
  bool get supportsPlatform => true;

  @override
  bool get canListAnything => true;

  @override
  bool requiresExtendedWirelessDeviceDiscovery;

  void addDevice(Device device) {
    _devices.add(device);
    _onAddedController.add(device);
  }

  void _removeDevice(Device device) {
    _devices.remove(device);
    _onRemovedController.add(device);
  }

  void setDevices(List<Device> devices) {
    while (_devices.isNotEmpty) {
      _removeDevice(_devices.first);
    }
    devices.forEach(addDevice);
  }

  bool discoverDevicesCalled = false;

  @override
  Future<List<Device>> discoverDevices({Duration? timeout, DeviceDiscoveryFilter? filter}) {
    discoverDevicesCalled = true;
    return super.discoverDevices(timeout: timeout);
  }

  @override
  Stream<Device> get onAdded => _onAddedController.stream;

  @override
  Stream<Device> get onRemoved => _onRemovedController.stream;

  @override
  List<String> wellKnownIds = <String>[];

  List<String> diagnostics = <String>[];

  @override
  Future<List<String>> getDiagnostics() => Future<List<String>>.value(diagnostics);
}

/// A fake implementation of the [DeviceLogReader].
class FakeDeviceLogReader extends DeviceLogReader {
  @override
  String get name => 'FakeLogReader';

  bool disposed = false;

  final List<String> _lineQueue = <String>[];
  late final StreamController<String> _linesController = StreamController<String>.broadcast(
    onListen: () {
      _lineQueue.forEach(_linesController.add);
      _lineQueue.clear();
    },
  );

  @override
  Stream<String> get logLines => _linesController.stream;

  void addLine(String line) {
    if (_linesController.hasListener) {
      _linesController.add(line);
    } else {
      _lineQueue.add(line);
    }
  }

  @override
  Future<void> dispose() async {
    _lineQueue.clear();
    await _linesController.close();
    disposed = true;
  }

  @override
  Future<void> provideVmService(FlutterVmService? connectedVmService) async {}
}
