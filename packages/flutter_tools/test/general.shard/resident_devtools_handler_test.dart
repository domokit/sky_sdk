// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/dds.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/devtools_launcher.dart';
import 'package:flutter_tools/src/resident_devtools_handler.dart';
import 'package:flutter_tools/src/resident_runner.dart';
import 'package:flutter_tools/src/vmservice.dart';
import 'package:flutter_tools/src/web/chrome.dart';
import 'package:test/fake.dart';
import 'package:vm_service/vm_service.dart' as vm_service;
import 'package:vm_service/vm_service.dart';

import '../src/common.dart';
import '../src/fake_process_manager.dart';
import '../src/fake_vm_services.dart';
import '../src/fakes.dart';

final vm_service.Isolate isolate = vm_service.Isolate(
  id: '1',
  pauseEvent: vm_service.Event(kind: vm_service.EventKind.kResume, timestamp: 0),
  breakpoints: <vm_service.Breakpoint>[],
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
  extensionRPCs: <String>['ext.flutter.connectedVmServiceUri'],
);

final FakeVmServiceRequest listViews = FakeVmServiceRequest(
  method: kListViewsMethod,
  jsonResponse: <String, Object>{
    'views': <Object>[FlutterView(id: 'a', uiIsolate: isolate).toJson()],
  },
);

void main() {
  Cache.flutterRoot = '';

  (BufferLogger, Artifacts) getTestState() => (BufferLogger.test(), Artifacts.test());

  testWithoutContext('Does not serve devtools if launcher is null', () async {
    final ResidentDevtoolsHandler handler = FlutterResidentDevtoolsHandler(
      null,
      FakeResidentRunner(),
      BufferLogger.test(),
      _FakeChromiumLauncher(),
    );

    await handler.serveAndAnnounceDevTools(flutterDevices: <FlutterDevice>[]);

    expect(handler.activeDevToolsServer, null);
  });

  testWithoutContext(
    'Does not serve devtools if ResidentRunner does not support the service protocol',
    () async {
      final ResidentDevtoolsHandler handler = FlutterResidentDevtoolsHandler(
        FakeDevtoolsLauncher(),
        FakeResidentRunner()..supportsServiceProtocol = false,
        BufferLogger.test(),
        _FakeChromiumLauncher(),
      );
      await handler.serveAndAnnounceDevTools(flutterDevices: <FlutterDevice>[]);

      expect(handler.activeDevToolsServer, null);
    },
  );

  testWithoutContext('Can use devtools with existing devtools URI', () async {
    final (BufferLogger logger, Artifacts artifacts) = getTestState();
    final DevtoolsServerLauncher launcher = DevtoolsServerLauncher(
      processManager: FakeProcessManager.empty(),
      artifacts: artifacts,
      logger: logger,
      botDetector: const FakeBotDetector(false),
    );
    final ResidentDevtoolsHandler handler = FlutterResidentDevtoolsHandler(
      // Uses real devtools instance which should be a no-op if
      // URI is already set.
      launcher,
      FakeResidentRunner(),
      BufferLogger.test(),
      _FakeChromiumLauncher(),
    );

    await handler.serveAndAnnounceDevTools(
      devToolsServerAddress: Uri.parse('http://localhost:8181'),
      flutterDevices: <FlutterDevice>[],
    );

    expect(handler.activeDevToolsServer!.host, 'localhost');
    expect(handler.activeDevToolsServer!.port, 8181);
  });

  testWithoutContext(
    'serveAndAnnounceDevTools with attached device does not fail on null vm service',
    () async {
      final ResidentDevtoolsHandler handler = FlutterResidentDevtoolsHandler(
        FakeDevtoolsLauncher()
          ..activeDevToolsServer = DevToolsServerAddress('localhost', 8080)
          ..devToolsUrl = Uri.parse('http://localhost:8080'),
        FakeResidentRunner(),
        BufferLogger.test(),
        _FakeChromiumLauncher(),
      );

      // VM Service is intentionally null
      final FakeFlutterDevice device = FakeFlutterDevice();

      await handler.serveAndAnnounceDevTools(flutterDevices: <FlutterDevice>[device]);
    },
  );

  testWithoutContext(
    'serveAndAnnounceDevTools with invokes devtools and vm_service setter',
    () async {
      final ResidentDevtoolsHandler handler = FlutterResidentDevtoolsHandler(
        FakeDevtoolsLauncher()
          ..activeDevToolsServer = DevToolsServerAddress('localhost', 8080)
          ..devToolsUrl = Uri.parse('http://localhost:8080'),
        FakeResidentRunner(),
        BufferLogger.test(),
        _FakeChromiumLauncher(),
      );
      final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[
          const FakeVmServiceRequest(
            method: 'streamListen',
            args: <String, Object>{'streamId': 'Isolate'},
          ),
          listViews,
          FakeVmServiceRequest(
            method: 'getIsolate',
            jsonResponse: isolate.toJson(),
            args: <String, Object>{'isolateId': '1'},
          ),
          listViews,
          listViews,
          const FakeVmServiceRequest(
            method: 'ext.flutter.activeDevToolsServerAddress',
            args: <String, Object>{'isolateId': '1', 'value': 'http://localhost:8080'},
          ),
          const FakeVmServiceRequest(
            method: 'ext.flutter.connectedVmServiceUri',
            args: <String, Object>{'isolateId': '1', 'value': 'http://localhost:1234'},
          ),
        ],
        httpAddress: Uri.parse('http://localhost:1234'),
      );

      final FakeFlutterDevice device = FakeFlutterDevice()..vmService = fakeVmServiceHost.vmService;

      await handler.serveAndAnnounceDevTools(flutterDevices: <FlutterDevice>[device]);
    },
  );

  testWithoutContext('serveAndAnnounceDevTools will bail if launching devtools fails', () async {
    final ResidentDevtoolsHandler handler = FlutterResidentDevtoolsHandler(
      FakeDevtoolsLauncher()..activeDevToolsServer = null,
      FakeResidentRunner(),
      BufferLogger.test(),
      _FakeChromiumLauncher(),
    );
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <VmServiceExpectation>[],
      httpAddress: Uri.parse('http://localhost:1234'),
    );

    final FakeFlutterDevice device = FakeFlutterDevice()..vmService = fakeVmServiceHost.vmService;

    await handler.serveAndAnnounceDevTools(flutterDevices: <FlutterDevice>[device]);
  });

  testWithoutContext('serveAndAnnounceDevTools with web device', () async {
    final ResidentDevtoolsHandler handler = FlutterResidentDevtoolsHandler(
      FakeDevtoolsLauncher()
        ..activeDevToolsServer = DevToolsServerAddress('localhost', 8080)
        ..devToolsUrl = Uri.parse('http://localhost:8080'),
      FakeResidentRunner(),
      BufferLogger.test(),
      _FakeChromiumLauncher(),
    );
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <VmServiceExpectation>[
        const FakeVmServiceRequest(
          method: 'streamListen',
          args: <String, Object>{'streamId': 'Isolate'},
        ),
        listViews,
        FakeVmServiceRequest(
          method: 'getIsolate',
          jsonResponse: isolate.toJson(),
          args: <String, Object>{'isolateId': '1'},
        ),
        const FakeVmServiceRequest(
          method: 'ext.flutter.activeDevToolsServerAddress',
          args: <String, Object>{'value': 'http://localhost:8080'},
        ),
        const FakeVmServiceRequest(
          method: 'ext.flutter.connectedVmServiceUri',
          args: <String, Object>{'value': 'http://localhost:1234'},
        ),
      ],
      httpAddress: Uri.parse('http://localhost:1234'),
    );

    final FakeFlutterDevice device =
        FakeFlutterDevice()
          ..vmService = fakeVmServiceHost.vmService
          ..targetPlatform = TargetPlatform.web_javascript;

    await handler.serveAndAnnounceDevTools(flutterDevices: <FlutterDevice>[device]);
  });

  testWithoutContext(
    'serveAndAnnounceDevTools with skips calling service extensions when VM service disappears',
    () async {
      final ResidentDevtoolsHandler handler = FlutterResidentDevtoolsHandler(
        FakeDevtoolsLauncher()..activeDevToolsServer = DevToolsServerAddress('localhost', 8080),
        FakeResidentRunner(),
        BufferLogger.test(),
        _FakeChromiumLauncher(),
      );
      final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[
          const FakeVmServiceRequest(
            method: 'streamListen',
            args: <String, Object>{'streamId': 'Isolate'},
          ),
          FakeVmServiceRequest(
            method: kListViewsMethod,
            error: FakeRPCError(code: RPCErrorKind.kServiceDisappeared.code),
          ),
        ],
        httpAddress: Uri.parse('http://localhost:1234'),
      );

      final FakeFlutterDevice device = FakeFlutterDevice()..vmService = fakeVmServiceHost.vmService;

      await handler.serveAndAnnounceDevTools(flutterDevices: <FlutterDevice>[device]);
    },
  );

  testWithoutContext(
    'serveAndAnnounceDevTools with multiple devices and VM service disappears on one',
    () async {
      final ResidentDevtoolsHandler handler = FlutterResidentDevtoolsHandler(
        FakeDevtoolsLauncher()
          ..activeDevToolsServer = DevToolsServerAddress('localhost', 8080)
          ..devToolsUrl = Uri.parse('http://localhost:8080'),
        FakeResidentRunner(),
        BufferLogger.test(),
        _FakeChromiumLauncher(),
      );

      final FakeVmServiceHost vmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[
          const FakeVmServiceRequest(
            method: 'streamListen',
            args: <String, Object>{'streamId': 'Isolate'},
          ),
          listViews,
          FakeVmServiceRequest(
            method: 'getIsolate',
            jsonResponse: isolate.toJson(),
            args: <String, Object>{'isolateId': '1'},
          ),
          listViews,
          listViews,
          const FakeVmServiceRequest(
            method: 'ext.flutter.activeDevToolsServerAddress',
            args: <String, Object>{'isolateId': '1', 'value': 'http://localhost:8080'},
          ),
          const FakeVmServiceRequest(
            method: 'ext.flutter.connectedVmServiceUri',
            args: <String, Object>{'isolateId': '1', 'value': 'http://localhost:1234'},
          ),
        ],
        httpAddress: Uri.parse('http://localhost:1234'),
      );

      final FakeVmServiceHost vmServiceHostThatDisappears = FakeVmServiceHost(
        requests: <VmServiceExpectation>[
          const FakeVmServiceRequest(
            method: 'streamListen',
            args: <String, Object>{'streamId': 'Isolate'},
          ),
          FakeVmServiceRequest(
            method: kListViewsMethod,
            error: FakeRPCError(code: RPCErrorKind.kServiceDisappeared.code),
          ),
        ],
        httpAddress: Uri.parse('http://localhost:5678'),
      );

      await handler.serveAndAnnounceDevTools(
        flutterDevices: <FlutterDevice>[
          FakeFlutterDevice()..vmService = vmServiceHostThatDisappears.vmService,
          FakeFlutterDevice()..vmService = vmServiceHost.vmService,
        ],
      );
    },
  );

  testWithoutContext('Does not launch devtools in browser if launcher is null', () async {
    final FlutterResidentDevtoolsHandler handler = FlutterResidentDevtoolsHandler(
      null,
      FakeResidentRunner(),
      BufferLogger.test(),
      _FakeChromiumLauncher(),
    );

    handler.launchDevToolsInBrowser(flutterDevices: <FlutterDevice>[]);
    expect(handler.launchedInBrowser, isFalse);
    expect(handler.activeDevToolsServer, null);
  });

  testWithoutContext(
    'Does not launch devtools in browser if ResidentRunner does not support the service protocol',
    () async {
      final FlutterResidentDevtoolsHandler handler = FlutterResidentDevtoolsHandler(
        FakeDevtoolsLauncher(),
        FakeResidentRunner()..supportsServiceProtocol = false,
        BufferLogger.test(),
        _FakeChromiumLauncher(),
      );

      handler.launchDevToolsInBrowser(flutterDevices: <FlutterDevice>[]);
      expect(handler.launchedInBrowser, isFalse);
      expect(handler.activeDevToolsServer, null);
    },
  );

  testWithoutContext(
    'launchDevToolsInBrowser launches after _devToolsLauncher.ready completes',
    () async {
      final Completer<void> completer = Completer<void>();
      final FlutterResidentDevtoolsHandler handler = FlutterResidentDevtoolsHandler(
        FakeDevtoolsLauncher()
          ..devToolsUrl = null
          // We need to set [activeDevToolsServer] to simulate the state we would
          // be in after serving devtools completes.
          ..activeDevToolsServer = DevToolsServerAddress('localhost', 8080)
          ..readyCompleter = completer,
        FakeResidentRunner(),
        BufferLogger.test(),
        _FakeChromiumLauncher(),
      );

      expect(handler.launchDevToolsInBrowser(flutterDevices: <FlutterDevice>[]), isTrue);
      expect(handler.launchedInBrowser, isFalse);

      completer.complete();
      // Await a short delay to give DevTools time to launch.
      await Future<void>.delayed(const Duration(microseconds: 100));

      expect(handler.launchedInBrowser, isTrue);
    },
  );

  testWithoutContext('launchDevToolsInBrowser launches successfully', () async {
    final FlutterResidentDevtoolsHandler handler = FlutterResidentDevtoolsHandler(
      FakeDevtoolsLauncher()
        ..devToolsUrl = Uri(host: 'localhost', port: 8080)
        ..activeDevToolsServer = DevToolsServerAddress('localhost', 8080),
      FakeResidentRunner(),
      BufferLogger.test(),
      _FakeChromiumLauncher(),
    );

    expect(handler.launchDevToolsInBrowser(flutterDevices: <FlutterDevice>[]), isTrue);
    expect(handler.launchedInBrowser, isTrue);
  });

  testWithoutContext('launchDevToolsInBrowser fails without Chrome installed', () async {
    final FlutterResidentDevtoolsHandler handler = FlutterResidentDevtoolsHandler(
      FakeDevtoolsLauncher()
        ..devToolsUrl = Uri(host: 'localhost', port: 8080)
        ..activeDevToolsServer = DevToolsServerAddress('localhost', 8080),
      FakeResidentRunner(),
      BufferLogger.test(),
      _ThrowingChromiumLauncher(),
    );

    expect(handler.launchedInBrowser, isFalse);
    expect(handler.launchDevToolsInBrowser(flutterDevices: <FlutterDevice>[]), isTrue);
  });

  testWithoutContext(
    'Converts a VM Service URI with a query parameter to a pretty display string',
    () {
      const String value =
          'http://127.0.0.1:9100?uri=http%3A%2F%2F127.0.0.1%3A57922%2F_MXpzytpH20%3D%2F';
      final Uri uri = Uri.parse(value);

      expect(
        urlToDisplayString(uri),
        'http://127.0.0.1:9100?uri=http://127.0.0.1:57922/_MXpzytpH20=/',
      );
    },
  );
}

class FakeResidentRunner extends Fake implements ResidentRunner {
  @override
  bool supportsServiceProtocol = true;

  @override
  bool reportedDebuggers = false;

  @override
  DebuggingOptions debuggingOptions = DebuggingOptions.disabled(BuildInfo.debug);
}

class FakeFlutterDevice extends Fake implements FlutterDevice {
  @override
  final Device device = FakeDevice();

  @override
  FlutterVmService? vmService;

  @override
  TargetPlatform targetPlatform = TargetPlatform.android_arm;
}

class FakeDevice extends Fake implements Device {
  @override
  DartDevelopmentService get dds => FakeDartDevelopmentService();
}

class FakeDartDevelopmentService extends Fake implements DartDevelopmentService {
  bool started = false;
  bool disposed = false;

  @override
  final Uri uri = Uri.parse('http://127.0.0.1:1234/');

  @override
  Future<void> startDartDevelopmentService(
    Uri vmServiceUri, {
    int? ddsPort,
    bool? disableServiceAuthCodes,
    bool? ipv6,
    bool enableDevTools = true,
    bool cacheStartupProfile = false,
    String? google3WorkspaceRoot,
    Uri? devToolsServerAddress,
  }) async {
    started = true;
  }

  @override
  Future<void> shutdown() async {
    disposed = true;
  }
}

class _ThrowingChromiumLauncher extends Fake implements ChromiumLauncher {
  @override
  Future<Chromium> launch(
    String url, {
    bool headless = false,
    int? debugPort,
    bool skipCheck = false,
    Directory? cacheDir,
    List<String> webBrowserFlags = const <String>[],
  }) async {
    throw ProcessException('ChromiumLauncher', <String>[url]);
  }
}

class _FakeChromiumLauncher extends Fake implements ChromiumLauncher {}
