// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:fake_async/fake_async.dart';
import 'package:flutter_tools/src/base/io.dart' as io;
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/convert.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/vmservice.dart';
import 'package:test/fake.dart';
import 'package:vm_service/vm_service.dart' as vm_service;

import '../src/common.dart';
import '../src/context.dart' hide testLogger;
import '../src/fake_vm_services.dart';

const String kExtensionName = 'ext.flutter.test.interestingExtension';

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
  extensionRPCs: <String>[kExtensionName],
);

final FlutterView fakeFlutterView = FlutterView(id: 'a', uiIsolate: isolate);

final FakeVmServiceRequest listViewsRequest = FakeVmServiceRequest(
  method: kListViewsMethod,
  jsonResponse: <String, Object>{
    'views': <Object>[fakeFlutterView.toJson()],
  },
);

void main() {
  testWithoutContext('VM Service registers reloadSources', () async {
    Future<void> reloadSources(String isolateId, {bool? pause, bool? force}) async {}

    final MockVMService mockVMService = MockVMService();
    await setUpVmService(reloadSources: reloadSources, vmService: mockVMService);

    expect(mockVMService.services, containsPair(kReloadSourcesServiceName, kFlutterToolAlias));
  });

  testWithoutContext('VM Service registers flutterMemoryInfo service', () async {
    final FakeDevice mockDevice = FakeDevice();

    final MockVMService mockVMService = MockVMService();
    await setUpVmService(device: mockDevice, vmService: mockVMService);

    expect(mockVMService.services, containsPair(kFlutterMemoryInfoServiceName, kFlutterToolAlias));
  });

  testWithoutContext('VM Service registers flutterPrintStructuredErrorLogMethod', () async {
    final MockVMService mockVMService = MockVMService();
    await setUpVmService(
      printStructuredErrorLogMethod: (vm_service.Event event) async => 'hello',
      vmService: mockVMService,
    );
    expect(mockVMService.listenedStreams, contains(vm_service.EventStreams.kExtension));
  });

  testWithoutContext('VM Service returns correct FlutterVersion', () async {
    final MockVMService mockVMService = MockVMService();
    await setUpVmService(vmService: mockVMService);

    expect(mockVMService.services, containsPair(kFlutterVersionServiceName, kFlutterToolAlias));
  });

  testUsingContext(
    'VM Service prints messages for connection failures',
    () {
      final BufferLogger logger = BufferLogger.test();
      FakeAsync().run((FakeAsync time) {
        final Uri uri = Uri.parse('ws://127.0.0.1:12345/QqL7EFEDNG0=/ws');
        unawaited(connectToVmService(uri, logger: logger));

        time.elapse(const Duration(seconds: 5));
        expect(logger.statusText, isEmpty);

        time.elapse(const Duration(minutes: 2));

        final String statusText = logger.statusText;
        expect(
          statusText,
          containsIgnoringWhitespace(
            'Connecting to the VM Service is taking longer than expected...',
          ),
        );
        expect(statusText, containsIgnoringWhitespace('try re-running with --host-vmservice-port'));
        expect(
          statusText,
          containsIgnoringWhitespace('Exception attempting to connect to the VM Service:'),
        );
        expect(statusText, containsIgnoringWhitespace('This was attempt #50. Will retry'));
      });
    },
    overrides: <Type, Generator>{WebSocketConnector: () => failingWebSocketConnector},
  );

  testWithoutContext('setAssetDirectory forwards arguments correctly', () async {
    final MockVMService mockVMService = MockVMService();
    final FlutterVmService flutterVmService = FlutterVmService(mockVMService);

    await flutterVmService.setAssetDirectory(
      assetsDirectory: Uri(path: 'abc', scheme: 'file'),
      viewId: 'abc',
      uiIsolateId: 'def',
      windows: false,
    );

    final ({Map<String, Object?>? args, String? isolateId}) call =
        mockVMService.calledMethods[kSetAssetBundlePathMethod]!.single;
    expect(call.isolateId, 'def');
    expect(call.args, <String, String>{'viewId': 'abc', 'assetDirectory': '/abc'});
  });

  testWithoutContext('setAssetDirectory forwards arguments correctly - windows', () async {
    final MockVMService mockVMService = MockVMService();
    final FlutterVmService flutterVmService = FlutterVmService(mockVMService);

    await flutterVmService.setAssetDirectory(
      assetsDirectory: Uri(
        path:
            'C:/Users/Tester/AppData/Local/Temp/hello_worldb42a6da5/hello_world/build/flutter_assets',
        scheme: 'file',
      ),
      viewId: 'abc',
      uiIsolateId: 'def',
      // If windows is not set to `true`, then the file path below is incorrectly prepended with a `/` which
      // causes the engine asset manager to interpret the file scheme as invalid.
      windows: true,
    );

    final ({Map<String, Object?>? args, String? isolateId}) call =
        mockVMService.calledMethods[kSetAssetBundlePathMethod]!.single;
    expect(call.isolateId, 'def');
    expect(call.args, <String, String>{
      'viewId': 'abc',
      'assetDirectory':
          r'C:\Users\Tester\AppData\Local\Temp\hello_worldb42a6da5\hello_world\build\flutter_assets',
    });
  });

  testWithoutContext('flushUIThreadTasks forwards arguments correctly', () async {
    final MockVMService mockVMService = MockVMService();
    final FlutterVmService flutterVmService = FlutterVmService(mockVMService);

    await flutterVmService.flushUIThreadTasks(uiIsolateId: 'def');

    final ({Map<String, Object?>? args, String? isolateId}) call =
        mockVMService.calledMethods[kFlushUIThreadTasksMethod]!.single;
    expect(call.isolateId, isNull);
    expect(call.args, <String, String>{'isolateId': 'def'});
  });

  testUsingContext('runInView forwards arguments correctly', () async {
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <VmServiceExpectation>[
        const FakeVmServiceRequest(
          method: 'streamListen',
          args: <String, Object>{'streamId': 'Isolate'},
        ),
        const FakeVmServiceRequest(
          method: kRunInViewMethod,
          args: <String, Object>{
            'viewId': '1234',
            'mainScript': 'main.dart',
            'assetDirectory': 'flutter_assets/',
          },
        ),
        FakeVmServiceStreamResponse(
          streamId: 'Isolate',
          event: vm_service.Event(kind: vm_service.EventKind.kIsolateRunnable, timestamp: 1),
        ),
      ],
    );

    await fakeVmServiceHost.vmService.runInView(
      viewId: '1234',
      main: Uri.file('main.dart'),
      assetsDirectory: Uri.file('flutter_assets/'),
    );
    expect(fakeVmServiceHost.hasRemainingExpectations, false);
  });

  testWithoutContext(
    'flutterDebugDumpSemanticsTreeInTraversalOrder handles missing method',
    () async {
      final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[
          FakeVmServiceRequest(
            method: 'ext.flutter.debugDumpSemanticsTreeInTraversalOrder',
            args: <String, Object>{'isolateId': '1'},
            error: FakeRPCError(code: vm_service.RPCErrorKind.kMethodNotFound.code),
          ),
        ],
      );

      expect(
        await fakeVmServiceHost.vmService.flutterDebugDumpSemanticsTreeInTraversalOrder(
          isolateId: '1',
        ),
        '',
      );
      expect(fakeVmServiceHost.hasRemainingExpectations, false);
    },
  );

  testWithoutContext(
    'flutterDebugDumpSemanticsTreeInInverseHitTestOrder handles missing method',
    () async {
      final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[
          FakeVmServiceRequest(
            method: 'ext.flutter.debugDumpSemanticsTreeInInverseHitTestOrder',
            args: <String, Object>{'isolateId': '1'},
            error: FakeRPCError(code: vm_service.RPCErrorKind.kMethodNotFound.code),
          ),
        ],
      );

      expect(
        await fakeVmServiceHost.vmService.flutterDebugDumpSemanticsTreeInInverseHitTestOrder(
          isolateId: '1',
        ),
        '',
      );
      expect(fakeVmServiceHost.hasRemainingExpectations, false);
    },
  );

  testWithoutContext('flutterDebugDumpLayerTree handles missing method', () async {
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <VmServiceExpectation>[
        FakeVmServiceRequest(
          method: 'ext.flutter.debugDumpLayerTree',
          args: <String, Object>{'isolateId': '1'},
          error: FakeRPCError(code: vm_service.RPCErrorKind.kMethodNotFound.code),
        ),
      ],
    );

    expect(await fakeVmServiceHost.vmService.flutterDebugDumpLayerTree(isolateId: '1'), '');
    expect(fakeVmServiceHost.hasRemainingExpectations, false);
  });

  testWithoutContext('flutterDebugDumpRenderTree handles missing method', () async {
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <VmServiceExpectation>[
        FakeVmServiceRequest(
          method: 'ext.flutter.debugDumpRenderTree',
          args: <String, Object>{'isolateId': '1'},
          error: FakeRPCError(code: vm_service.RPCErrorKind.kMethodNotFound.code),
        ),
      ],
    );

    expect(await fakeVmServiceHost.vmService.flutterDebugDumpRenderTree(isolateId: '1'), '');
    expect(fakeVmServiceHost.hasRemainingExpectations, false);
  });

  testWithoutContext('flutterDebugDumpApp handles missing method', () async {
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <VmServiceExpectation>[
        FakeVmServiceRequest(
          method: 'ext.flutter.debugDumpApp',
          args: <String, Object>{'isolateId': '1'},
          error: FakeRPCError(code: vm_service.RPCErrorKind.kMethodNotFound.code),
        ),
      ],
    );

    expect(await fakeVmServiceHost.vmService.flutterDebugDumpApp(isolateId: '1'), '');
    expect(fakeVmServiceHost.hasRemainingExpectations, false);
  });

  testWithoutContext('flutterDebugDumpFocusTree handles missing method', () async {
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <VmServiceExpectation>[
        FakeVmServiceRequest(
          method: 'ext.flutter.debugDumpFocusTree',
          args: <String, Object>{'isolateId': '1'},
          error: FakeRPCError(code: vm_service.RPCErrorKind.kMethodNotFound.code),
        ),
      ],
    );

    expect(await fakeVmServiceHost.vmService.flutterDebugDumpFocusTree(isolateId: '1'), '');
    expect(fakeVmServiceHost.hasRemainingExpectations, false);
  });

  testWithoutContext('flutterDebugDumpFocusTree returns data', () async {
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <VmServiceExpectation>[
        const FakeVmServiceRequest(
          method: 'ext.flutter.debugDumpFocusTree',
          args: <String, Object>{'isolateId': '1'},
          jsonResponse: <String, Object>{'data': 'Hello world'},
        ),
      ],
    );

    expect(
      await fakeVmServiceHost.vmService.flutterDebugDumpFocusTree(isolateId: '1'),
      'Hello world',
    );
    expect(fakeVmServiceHost.hasRemainingExpectations, false);
  });

  testWithoutContext(
    'Framework service extension invocations return null if service disappears ',
    () async {
      final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[
          FakeVmServiceRequest(
            method: kListViewsMethod,
            error: FakeRPCError(code: vm_service.RPCErrorKind.kServiceDisappeared.code),
          ),
          FakeVmServiceRequest(
            method: kScreenshotSkpMethod,
            error: FakeRPCError(code: vm_service.RPCErrorKind.kServiceDisappeared.code),
          ),
          FakeVmServiceRequest(
            method: 'setVMTimelineFlags',
            args: <String, dynamic>{
              'recordedStreams': <String>['test'],
            },
            error: FakeRPCError(code: vm_service.RPCErrorKind.kServiceDisappeared.code),
          ),
          FakeVmServiceRequest(
            method: 'getVMTimeline',
            error: FakeRPCError(code: vm_service.RPCErrorKind.kServiceDisappeared.code),
          ),
        ],
      );

      final List<FlutterView> views = await fakeVmServiceHost.vmService.getFlutterViews();
      expect(views, isEmpty);

      final vm_service.Response? screenshotSkp = await fakeVmServiceHost.vmService.screenshotSkp();
      expect(screenshotSkp, isNull);

      // Checking that this doesn't throw.
      await fakeVmServiceHost.vmService.setTimelineFlags(<String>['test']);

      final vm_service.Response? timeline = await fakeVmServiceHost.vmService.getTimeline();
      expect(timeline, isNull);

      expect(fakeVmServiceHost.hasRemainingExpectations, false);
    },
  );

  testWithoutContext('getIsolateOrNull returns null if service disappears ', () async {
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <VmServiceExpectation>[
        FakeVmServiceRequest(
          method: 'getIsolate',
          args: <String, Object>{'isolateId': 'isolate/123'},
          error: FakeRPCError(code: vm_service.RPCErrorKind.kServiceDisappeared.code),
        ),
      ],
    );

    final vm_service.Isolate? isolate = await fakeVmServiceHost.vmService.getIsolateOrNull(
      'isolate/123',
    );
    expect(isolate, null);

    expect(fakeVmServiceHost.hasRemainingExpectations, false);
  });

  testWithoutContext('getFlutterViews polls until a view is returned', () async {
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <VmServiceExpectation>[
        const FakeVmServiceRequest(
          method: kListViewsMethod,
          jsonResponse: <String, Object>{'views': <Object>[]},
        ),
        const FakeVmServiceRequest(
          method: kListViewsMethod,
          jsonResponse: <String, Object>{'views': <Object>[]},
        ),
        listViewsRequest,
      ],
    );

    expect(await fakeVmServiceHost.vmService.getFlutterViews(delay: Duration.zero), isNotEmpty);
    expect(fakeVmServiceHost.hasRemainingExpectations, false);
  });

  testWithoutContext('getFlutterViews does not poll if returnEarly is true', () async {
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <VmServiceExpectation>[
        const FakeVmServiceRequest(
          method: kListViewsMethod,
          jsonResponse: <String, Object>{'views': <Object>[]},
        ),
      ],
    );

    expect(await fakeVmServiceHost.vmService.getFlutterViews(returnEarly: true), isEmpty);
    expect(fakeVmServiceHost.hasRemainingExpectations, false);
  });

  group('findExtensionIsolate', () {
    testWithoutContext('returns an isolate with the registered extensionRPC', () async {
      final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[
          const FakeVmServiceRequest(
            method: 'streamListen',
            args: <String, Object>{'streamId': 'Isolate'},
          ),
          listViewsRequest,
          FakeVmServiceRequest(
            method: 'getIsolate',
            jsonResponse: isolate.toJson(),
            args: <String, Object>{'isolateId': '1'},
          ),
        ],
      );

      final vm_service.IsolateRef isolateRef = await fakeVmServiceHost.vmService
          .findExtensionIsolate(kExtensionName);
      expect(isolateRef.id, '1');
    });

    testWithoutContext(
      'returns the isolate with the registered extensionRPC when there are multiple FlutterViews',
      () async {
        const String otherExtensionName = 'ext.flutter.test.otherExtension';

        // Copy the other isolate and change a few fields.
        final vm_service.Isolate isolate2 =
            vm_service.Isolate.parse(
              isolate.toJson()
                ..['id'] = '2'
                ..['extensionRPCs'] = <String>[otherExtensionName],
            )!;

        final FlutterView fakeFlutterView2 = FlutterView(id: '2', uiIsolate: isolate2);

        final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
          requests: <VmServiceExpectation>[
            const FakeVmServiceRequest(
              method: 'streamListen',
              args: <String, Object>{'streamId': 'Isolate'},
            ),
            FakeVmServiceRequest(
              method: kListViewsMethod,
              jsonResponse: <String, Object>{
                'views': <Object>[fakeFlutterView.toJson(), fakeFlutterView2.toJson()],
              },
            ),
            FakeVmServiceRequest(
              method: 'getIsolate',
              jsonResponse: isolate.toJson(),
              args: <String, Object>{'isolateId': '1'},
            ),
            FakeVmServiceRequest(
              method: 'getIsolate',
              jsonResponse: isolate2.toJson(),
              args: <String, Object>{'isolateId': '2'},
            ),
          ],
        );

        final vm_service.IsolateRef isolateRef = await fakeVmServiceHost.vmService
            .findExtensionIsolate(otherExtensionName);
        expect(isolateRef.id, '2');
      },
    );

    testWithoutContext(
      'does not rethrow a sentinel exception if the initially queried flutter view disappears',
      () async {
        const String otherExtensionName = 'ext.flutter.test.otherExtension';
        final vm_service.Isolate? isolate2 = vm_service.Isolate.parse(
          isolate.toJson()
            ..['id'] = '2'
            ..['extensionRPCs'] = <String>[otherExtensionName],
        );

        final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
          requests: <VmServiceExpectation>[
            const FakeVmServiceRequest(
              method: 'streamListen',
              args: <String, Object>{'streamId': 'Isolate'},
            ),
            FakeVmServiceRequest(
              method: kListViewsMethod,
              jsonResponse: <String, Object>{
                'views': <Object>[fakeFlutterView.toJson()],
              },
            ),
            FakeVmServiceRequest(
              method: 'getIsolate',
              args: <String, Object>{'isolateId': '1'},
              error: FakeRPCError(code: vm_service.RPCErrorKind.kServiceDisappeared.code),
            ),
            // Assume a different isolate returns.
            FakeVmServiceStreamResponse(
              streamId: 'Isolate',
              event: vm_service.Event(
                kind: vm_service.EventKind.kServiceExtensionAdded,
                extensionRPC: otherExtensionName,
                timestamp: 1,
                isolate: isolate2,
              ),
            ),
          ],
        );

        final vm_service.IsolateRef isolateRef = await fakeVmServiceHost.vmService
            .findExtensionIsolate(otherExtensionName);
        expect(isolateRef.id, '2');
      },
    );

    testWithoutContext(
      'when the isolate stream is already subscribed, returns an isolate with the registered extensionRPC',
      () async {
        final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
          requests: <VmServiceExpectation>[
            const FakeVmServiceRequest(
              method: 'streamListen',
              args: <String, Object>{'streamId': 'Isolate'},
              // Stream already subscribed - https://github.com/dart-lang/sdk/blob/main/runtime/vm/service/service.md#streamlisten
              error: FakeRPCError(code: 103),
            ),
            listViewsRequest,
            FakeVmServiceRequest(
              method: 'getIsolate',
              jsonResponse: isolate.toJson()..['extensionRPCs'] = <String>[kExtensionName],
              args: <String, Object>{'isolateId': '1'},
            ),
          ],
        );

        final vm_service.IsolateRef isolateRef = await fakeVmServiceHost.vmService
            .findExtensionIsolate(kExtensionName);
        expect(isolateRef.id, '1');
      },
    );

    testWithoutContext('returns an isolate with a extensionRPC that is registered later', () async {
      final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[
          const FakeVmServiceRequest(
            method: 'streamListen',
            args: <String, Object>{'streamId': 'Isolate'},
          ),
          listViewsRequest,
          FakeVmServiceRequest(
            method: 'getIsolate',
            jsonResponse: isolate.toJson(),
            args: <String, Object>{'isolateId': '1'},
          ),
          FakeVmServiceStreamResponse(
            streamId: 'Isolate',
            event: vm_service.Event(
              kind: vm_service.EventKind.kServiceExtensionAdded,
              extensionRPC: kExtensionName,
              timestamp: 1,
            ),
          ),
        ],
      );

      final vm_service.IsolateRef isolateRef = await fakeVmServiceHost.vmService
          .findExtensionIsolate(kExtensionName);
      expect(isolateRef.id, '1');
    });

    testWithoutContext('throws when the service disappears', () async {
      final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[
          const FakeVmServiceRequest(
            method: 'streamListen',
            args: <String, Object>{'streamId': 'Isolate'},
          ),
          FakeVmServiceRequest(
            method: kListViewsMethod,
            error: FakeRPCError(code: vm_service.RPCErrorKind.kServiceDisappeared.code),
          ),
        ],
      );

      expect(
        () => fakeVmServiceHost.vmService.findExtensionIsolate(kExtensionName),
        throwsA(isA<VmServiceDisappearedException>()),
      );
    });

    testWithoutContext('throws when the service is disposed', () async {
      final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[],
      );

      await fakeVmServiceHost.vmService.dispose();

      expect(
        () => fakeVmServiceHost.vmService.findExtensionIsolate(kExtensionName),
        throwsA(isA<VmServiceDisappearedException>()),
      );
    });
  });

  testWithoutContext('Can process log events from the vm service', () {
    final vm_service.Event event = vm_service.Event(
      bytes: base64.encode(utf8.encode('Hello There\n')),
      timestamp: 0,
      kind: vm_service.EventKind.kLogging,
    );

    expect(processVmServiceMessage(event), 'Hello There');
  });

  testUsingContext('WebSocket URL construction uses correct URI join primitives', () async {
    final Completer<String> completer = Completer<String>();
    openChannelForTesting = (
      String url, {
      io.CompressionOptions compression = io.CompressionOptions.compressionDefault,
      required Logger logger,
    }) async {
      completer.complete(url);
      throw Exception('');
    };

    // Construct a URL that does not end in a `/`.
    await expectLater(
      () => connectToVmService(Uri.parse('http://localhost:8181/foo'), logger: BufferLogger.test()),
      throwsException,
    );
    expect(await completer.future, 'ws://localhost:8181/foo/ws');
    openChannelForTesting = null;
  });
}

class MockVMService extends Fake implements vm_service.VmService {
  final Map<String, String> services = <String, String>{};
  final Map<String, vm_service.ServiceCallback> serviceCallBacks =
      <String, vm_service.ServiceCallback>{};
  final Map<String, List<({String? isolateId, Map<String, Object?>? args})>> calledMethods =
      <String, List<({Map<String, Object?>? args, String? isolateId})>>{};
  final Set<String> listenedStreams = <String>{};
  bool errorOnRegisterService = false;

  @override
  Future<vm_service.Response> callMethod(
    String method, {
    String? isolateId,
    Map<String, Object?>? args,
  }) async {
    calledMethods
        .putIfAbsent(method, () => <({Map<String, Object?>? args, String? isolateId})>[])
        .add((isolateId: isolateId, args: args));
    return vm_service.Success();
  }

  @override
  void registerServiceCallback(String service, vm_service.ServiceCallback cb) {
    serviceCallBacks[service] = cb;
  }

  @override
  Future<vm_service.Success> registerService(String service, String alias) async {
    services[service] = alias;
    if (errorOnRegisterService) {
      throw vm_service.RPCError('registerService', 1234, 'error');
    }
    return vm_service.Success();
  }

  @override
  Stream<vm_service.Event> get onExtensionEvent => const Stream<vm_service.Event>.empty();

  @override
  Future<vm_service.Success> streamListen(String streamId) async {
    listenedStreams.add(streamId);
    return vm_service.Success();
  }
}

class FakeDevice extends Fake implements Device {}

/// A [WebSocketConnector] that always throws an [io.SocketException].
Future<io.WebSocket> failingWebSocketConnector(
  String url, {
  io.CompressionOptions? compression,
  Logger? logger,
}) {
  throw const io.SocketException('Failed WebSocket connection');
}
