// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:fuchsia_remote_debug_protocol/fuchsia_remote_debug_protocol.dart';
import 'package:test/fake.dart';
import 'package:test/test.dart';
import 'package:vm_service/vm_service.dart' as vms;

void main() {
  group('FuchsiaRemoteConnection.connect', () {
    late List<FakePortForwarder> forwardedPorts;
    List<FakeVmService> fakeVmServices;
    late List<Uri> uriConnections;

    setUp(() {
      final List<Map<String, dynamic>> flutterViewCannedResponses = <Map<String, dynamic>>[
        <String, dynamic>{
          'views': <Map<String, dynamic>>[
            <String, dynamic>{'type': 'FlutterView', 'id': 'flutterView0'},
          ],
        },
        <String, dynamic>{
          'views': <Map<String, dynamic>>[
            <String, dynamic>{
              'type': 'FlutterView',
              'id': 'flutterView1',
              'isolate': <String, dynamic>{
                'type': '@Isolate',
                'fixedId': 'true',
                'id': 'isolates/1',
                'name': 'file://flutterBinary1',
                'number': '1',
              },
            },
          ],
        },
        <String, dynamic>{
          'views': <Map<String, dynamic>>[
            <String, dynamic>{
              'type': 'FlutterView',
              'id': 'flutterView2',
              'isolate': <String, dynamic>{
                'type': '@Isolate',
                'fixedId': 'true',
                'id': 'isolates/2',
                'name': 'file://flutterBinary2',
                'number': '2',
              },
            },
          ],
        },
      ];

      forwardedPorts = <FakePortForwarder>[];
      fakeVmServices = <FakeVmService>[];
      uriConnections = <Uri>[];
      Future<vms.VmService> fakeVmConnectionFunction(Uri uri, {Duration? timeout}) {
        return Future<vms.VmService>(() async {
          final FakeVmService service = FakeVmService();
          fakeVmServices.add(service);
          uriConnections.add(uri);
          service.flutterListViews = vms.Response.parse(flutterViewCannedResponses[uri.port]);
          return service;
        });
      }

      fuchsiaVmServiceConnectionFunction = fakeVmConnectionFunction;
    });

    tearDown(() {
      /// Most tests will fake out the port forwarding and connection
      /// functions.
      restoreFuchsiaPortForwardingFunction();
      restoreVmServiceConnectionFunction();
    });

    test('end-to-end with one vm connection and flutter view query', () async {
      int port = 0;
      Future<PortForwarder> fakePortForwardingFunction(
        String address,
        int remotePort, [
        String? interface = '',
        String? configFile,
      ]) {
        return Future<PortForwarder>(() {
          final FakePortForwarder pf = FakePortForwarder();
          forwardedPorts.add(pf);
          pf.port = port++;
          pf.remotePort = remotePort;
          return pf;
        });
      }

      fuchsiaPortForwardingFunction = fakePortForwardingFunction;
      final FakeSshCommandRunner fakeRunner = FakeSshCommandRunner();
      // Adds some extra junk to make sure the strings will be cleaned up.
      fakeRunner.iqueryResponse = <String>[
        '[',
        '   {',
        '     "data_source": "Inspect",',
        '     "metadata": {',
        '       "filename": "fuchsia.inspect.Tree",',
        '       "component_url": "fuchsia-pkg://fuchsia.com/flutter_runner#meta/flutter_runner.cm",',
        '       "timestamp": 12345678901234',
        '     },',
        '     "moniker": "core/session-manager/session/flutter_runner",',
        '     "payload": {',
        '       "root": {',
        '         "vm_service_port": "12345",',
        '         "16859221": {',
        '           "empty_tree": "this semantic tree is empty"',
        '         },',
        '         "build_info": {',
        '           "dart_sdk_git_revision": "77e83fcc14fa94049f363d554579f48fbd6bb7a1",',
        '           "dart_sdk_semantic_version": "2.19.0-317.0.dev",',
        '           "flutter_engine_git_revision": "563b8e830c697a543bf0a8a9f4ae3edfad86ea86",',
        '           "fuchsia_sdk_version": "10.20221018.0.1"',
        '         },',
        '         "vm": {',
        '           "dst_status": 1,',
        '           "get_profile_status": 0,',
        '           "num_get_profile_calls": 1,',
        '           "num_intl_provider_errors": 0,',
        '           "num_on_change_calls": 0,',
        '           "timezone_content_status": 0,',
        '           "tz_data_close_status": -1,',
        '           "tz_data_status": -1',
        '         }',
        '       }',
        '     },',
        '     "version": 1',
        '   }',
        ' ]',
      ];
      fakeRunner.address = 'fe80::8eae:4cff:fef4:9247';
      fakeRunner.interface = 'eno1';

      final FuchsiaRemoteConnection connection =
          await FuchsiaRemoteConnection.connectWithSshCommandRunner(fakeRunner);

      expect(forwardedPorts.length, 1);
      expect(forwardedPorts[0].remotePort, 12345);

      // VMs should be accessed via localhost ports given by
      // [fakePortForwardingFunction].
      expect(uriConnections[0], Uri(scheme: 'ws', host: '[::1]', port: 0, path: '/ws'));

      final List<FlutterView> views = await connection.getFlutterViews();
      expect(views, isNot(null));
      expect(views.length, 1);
      // Since name can be null, check for the ID on all of them.
      expect(views[0].id, 'flutterView0');

      expect(views[0].name, equals(null));

      // Ensure the ports are all closed after stop was called.
      await connection.stop();
      expect(forwardedPorts[0].stopped, true);
    });

    test('end-to-end with one vm and remote open port', () async {
      int port = 0;
      Future<PortForwarder> fakePortForwardingFunction(
        String address,
        int remotePort, [
        String? interface = '',
        String? configFile,
      ]) {
        return Future<PortForwarder>(() {
          final FakePortForwarder pf = FakePortForwarder();
          forwardedPorts.add(pf);
          pf.port = port++;
          pf.remotePort = remotePort;
          pf.openPortAddress = 'fe80::1:2%eno2';
          return pf;
        });
      }

      fuchsiaPortForwardingFunction = fakePortForwardingFunction;
      final FakeSshCommandRunner fakeRunner = FakeSshCommandRunner();
      // Adds some extra junk to make sure the strings will be cleaned up.
      fakeRunner.iqueryResponse = <String>[
        '[',
        '   {',
        '     "data_source": "Inspect",',
        '     "metadata": {',
        '       "filename": "fuchsia.inspect.Tree",',
        '       "component_url": "fuchsia-pkg://fuchsia.com/flutter_runner#meta/flutter_runner.cm",',
        '       "timestamp": 12345678901234',
        '     },',
        '     "moniker": "core/session-manager/session/flutter_runner",',
        '     "payload": {',
        '       "root": {',
        '         "vm_service_port": "12345",',
        '         "16859221": {',
        '           "empty_tree": "this semantic tree is empty"',
        '         },',
        '         "build_info": {',
        '           "dart_sdk_git_revision": "77e83fcc14fa94049f363d554579f48fbd6bb7a1",',
        '           "dart_sdk_semantic_version": "2.19.0-317.0.dev",',
        '           "flutter_engine_git_revision": "563b8e830c697a543bf0a8a9f4ae3edfad86ea86",',
        '           "fuchsia_sdk_version": "10.20221018.0.1"',
        '         },',
        '         "vm": {',
        '           "dst_status": 1,',
        '           "get_profile_status": 0,',
        '           "num_get_profile_calls": 1,',
        '           "num_intl_provider_errors": 0,',
        '           "num_on_change_calls": 0,',
        '           "timezone_content_status": 0,',
        '           "tz_data_close_status": -1,',
        '           "tz_data_status": -1',
        '         }',
        '       }',
        '     },',
        '     "version": 1',
        '   }',
        ' ]',
      ];
      fakeRunner.address = 'fe80::8eae:4cff:fef4:9247';
      fakeRunner.interface = 'eno1';
      final FuchsiaRemoteConnection connection =
          await FuchsiaRemoteConnection.connectWithSshCommandRunner(fakeRunner);

      expect(forwardedPorts.length, 1);
      expect(forwardedPorts[0].remotePort, 12345);

      // VMs should be accessed via the alternate address given by
      // [fakePortForwardingFunction].
      expect(
        uriConnections[0],
        Uri(scheme: 'ws', host: '[fe80::1:2%25eno2]', port: 0, path: '/ws'),
      );

      final List<FlutterView> views = await connection.getFlutterViews();
      expect(views, isNot(null));
      expect(views.length, 1);
      // Since name can be null, check for the ID on all of them.
      expect(views[0].id, 'flutterView0');

      expect(views[0].name, equals(null));

      // Ensure the ports are all closed after stop was called.
      await connection.stop();
      expect(forwardedPorts[0].stopped, true);
    });

    test('end-to-end with one vm and ipv4', () async {
      int port = 0;
      Future<PortForwarder> fakePortForwardingFunction(
        String address,
        int remotePort, [
        String? interface = '',
        String? configFile,
      ]) {
        return Future<PortForwarder>(() {
          final FakePortForwarder pf = FakePortForwarder();
          forwardedPorts.add(pf);
          pf.port = port++;
          pf.remotePort = remotePort;
          return pf;
        });
      }

      fuchsiaPortForwardingFunction = fakePortForwardingFunction;
      final FakeSshCommandRunner fakeRunner = FakeSshCommandRunner();
      // Adds some extra junk to make sure the strings will be cleaned up.
      fakeRunner.iqueryResponse = <String>[
        '[',
        '   {',
        '     "data_source": "Inspect",',
        '     "metadata": {',
        '       "filename": "fuchsia.inspect.Tree",',
        '       "component_url": "fuchsia-pkg://fuchsia.com/flutter_runner#meta/flutter_runner.cm",',
        '       "timestamp": 12345678901234',
        '     },',
        '     "moniker": "core/session-manager/session/flutter_runner",',
        '     "payload": {',
        '       "root": {',
        '         "vm_service_port": "12345",',
        '         "16859221": {',
        '           "empty_tree": "this semantic tree is empty"',
        '         },',
        '         "build_info": {',
        '           "dart_sdk_git_revision": "77e83fcc14fa94049f363d554579f48fbd6bb7a1",',
        '           "dart_sdk_semantic_version": "2.19.0-317.0.dev",',
        '           "flutter_engine_git_revision": "563b8e830c697a543bf0a8a9f4ae3edfad86ea86",',
        '           "fuchsia_sdk_version": "10.20221018.0.1"',
        '         },',
        '         "vm": {',
        '           "dst_status": 1,',
        '           "get_profile_status": 0,',
        '           "num_get_profile_calls": 1,',
        '           "num_intl_provider_errors": 0,',
        '           "num_on_change_calls": 0,',
        '           "timezone_content_status": 0,',
        '           "tz_data_close_status": -1,',
        '           "tz_data_status": -1',
        '         }',
        '       }',
        '     },',
        '     "version": 1',
        '   }',
        ' ]',
      ];
      fakeRunner.address = '196.168.1.4';

      final FuchsiaRemoteConnection connection =
          await FuchsiaRemoteConnection.connectWithSshCommandRunner(fakeRunner);

      expect(forwardedPorts.length, 1);
      expect(forwardedPorts[0].remotePort, 12345);

      // VMs should be accessed via the ipv4 loopback.
      expect(uriConnections[0], Uri(scheme: 'ws', host: '127.0.0.1', port: 0, path: '/ws'));

      final List<FlutterView> views = await connection.getFlutterViews();
      expect(views, isNot(null));
      expect(views.length, 1);
      // Since name can be null, check for the ID on all of them.
      expect(views[0].id, 'flutterView0');

      expect(views[0].name, equals(null));

      // Ensure the ports are all closed after stop was called.
      await connection.stop();
      expect(forwardedPorts[0].stopped, true);
    });

    test('env variable test without remote addr', () async {
      Future<void> failingFunction() async {
        await FuchsiaRemoteConnection.connect();
      }

      // Should fail as no env variable has been passed.
      expect(failingFunction, throwsA(isA<FuchsiaRemoteConnectionError>()));
    });
  });
}

class FakeSshCommandRunner extends Fake implements SshCommandRunner {
  List<String>? iqueryResponse;
  @override
  Future<List<String>> run(String command) async {
    if (command.startsWith('iquery --format json show')) {
      return iqueryResponse!;
    }
    throw UnimplementedError(command);
  }

  @override
  String interface = '';

  @override
  String address = '';

  @override
  String get sshConfigPath => '~/.ssh';
}

class FakePortForwarder extends Fake implements PortForwarder {
  @override
  int port = 0;

  @override
  int remotePort = 0;

  @override
  String? openPortAddress;

  bool stopped = false;
  @override
  Future<void> stop() async {
    stopped = true;
  }
}

class FakeVmService extends Fake implements vms.VmService {
  bool disposed = false;
  vms.Response? flutterListViews;

  @override
  Future<void> dispose() async {
    disposed = true;
  }

  @override
  Future<vms.Response> callMethod(
    String method, {
    String? isolateId,
    Map<String, dynamic>? args,
  }) async {
    if (method == '_flutter.listViews') {
      return flutterListViews!;
    }
    throw UnimplementedError(method);
  }

  @override
  Future<void> onDone = Future<void>.value();

  @override
  Future<vms.Version> getVersion() async {
    return vms.Version(major: -1, minor: -1);
  }
}
