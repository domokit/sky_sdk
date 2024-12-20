// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io';
import 'dart:typed_data';

import 'package:file/memory.dart';
import 'package:flutter_tools/src/application_package.dart';
import 'package:flutter_tools/src/base/dds.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/utils.dart';
import 'package:flutter_tools/src/daemon.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/device_vm_service_discovery_for_attach.dart';
import 'package:flutter_tools/src/proxied_devices/devices.dart';
import 'package:flutter_tools/src/proxied_devices/file_transfer.dart';
import 'package:flutter_tools/src/resident_runner.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/fake_devices.dart';

void main() {
  late BufferLogger bufferLogger;
  late DaemonConnection serverDaemonConnection;
  late DaemonConnection clientDaemonConnection;
  setUp(() {
    bufferLogger = BufferLogger.test();
    final FakeDaemonStreams serverDaemonStreams = FakeDaemonStreams();
    serverDaemonConnection = DaemonConnection(
      daemonStreams: serverDaemonStreams,
      logger: bufferLogger,
    );
    final FakeDaemonStreams clientDaemonStreams = FakeDaemonStreams();
    clientDaemonConnection = DaemonConnection(
      daemonStreams: clientDaemonStreams,
      logger: bufferLogger,
    );

    serverDaemonStreams.inputs.addStream(clientDaemonStreams.outputs.stream);
    clientDaemonStreams.inputs.addStream(serverDaemonStreams.outputs.stream);
  });

  tearDown(() async {
    await serverDaemonConnection.dispose();
    await clientDaemonConnection.dispose();
  });

  group('ProxiedPortForwarder', () {
    testWithoutContext('works correctly without device id', () async {
      final FakeServerSocket fakeServerSocket = FakeServerSocket(200);
      final ProxiedPortForwarder portForwarder = ProxiedPortForwarder(
        clientDaemonConnection,
        logger: bufferLogger,
        createSocketServer: (Logger logger, int? hostPort, bool? ipv6) async => fakeServerSocket,
      );
      final int result = await portForwarder.forward(100);
      expect(result, 200);

      final FakeSocket fakeSocket = FakeSocket();
      fakeServerSocket.controller.add(fakeSocket);

      final Stream<DaemonMessage> broadcastOutput =
          serverDaemonConnection.incomingCommands.asBroadcastStream();

      DaemonMessage message = await broadcastOutput.first;

      expect(message.data['id'], isNotNull);
      expect(message.data['method'], 'proxy.connect');
      expect(message.data['params'], <String, Object?>{'port': 100});

      const String id = 'random_id';
      serverDaemonConnection.sendResponse(message.data['id']!, id);

      // Forwards the data received from socket to daemon.
      fakeSocket.controller.add(Uint8List.fromList(<int>[1, 2, 3]));
      message = await broadcastOutput.first;
      expect(message.data['method'], 'proxy.write');
      expect(message.data['params'], <String, Object?>{'id': id});
      expect(message.binary, isNotNull);
      final List<List<int>> binary = await message.binary!.toList();
      expect(binary, <List<int>>[
        <int>[1, 2, 3],
      ]);

      // Forwards data received as event to socket.
      expect(fakeSocket.addedData.isEmpty, true);
      serverDaemonConnection.sendEvent('proxy.data.$id', null, <int>[4, 5, 6]);
      await pumpEventQueue();
      expect(fakeSocket.addedData.isNotEmpty, true);
      expect(fakeSocket.addedData[0], <int>[4, 5, 6]);

      // Closes the socket after the remote end disconnects
      expect(fakeSocket.closeCalled, false);
      serverDaemonConnection.sendEvent('proxy.disconnected.$id');
      await pumpEventQueue();
      expect(fakeSocket.closeCalled, true);
    });

    testWithoutContext('handles errors', () async {
      final FakeServerSocket fakeServerSocket = FakeServerSocket(200);
      final ProxiedPortForwarder portForwarder = ProxiedPortForwarder(
        FakeDaemonConnection(
          handledRequests: <String, Object?>{
            'proxy.connect': '1', // id
          },
        ),
        logger: bufferLogger,
        createSocketServer: (Logger logger, int? hostPort, bool? ipv6) async => fakeServerSocket,
      );
      final int result = await portForwarder.forward(100);
      expect(result, 200);

      final FakeSocket fakeSocket = FakeSocket();
      fakeServerSocket.controller.add(fakeSocket);

      fakeSocket.controller.add(Uint8List.fromList(<int>[1, 2, 3]));
      await pumpEventQueue();
    });

    testWithoutContext('forwards the port from the remote end with device id', () async {
      final FakeServerSocket fakeServerSocket = FakeServerSocket(400);
      final ProxiedPortForwarder portForwarder = ProxiedPortForwarder(
        clientDaemonConnection,
        deviceId: 'device_id',
        logger: bufferLogger,
        createSocketServer: (Logger logger, int? hostPort, bool? ipv6) async => fakeServerSocket,
      );

      final Stream<DaemonMessage> broadcastOutput =
          serverDaemonConnection.incomingCommands.asBroadcastStream();

      final Future<int> result = portForwarder.forward(300);

      DaemonMessage message = await broadcastOutput.first;
      expect(message.data['id'], isNotNull);
      expect(message.data['method'], 'device.forward');
      expect(message.data['params'], <String, Object?>{'deviceId': 'device_id', 'devicePort': 300});

      serverDaemonConnection.sendResponse(message.data['id']!, <String, Object?>{'hostPort': 350});

      expect(await result, 400);

      final FakeSocket fakeSocket = FakeSocket();
      fakeServerSocket.controller.add(fakeSocket);
      message = await broadcastOutput.first;

      expect(message.data['id'], isNotNull);
      expect(message.data['method'], 'proxy.connect');
      expect(message.data['params'], <String, Object?>{'port': 350});

      const String id = 'random_id';
      serverDaemonConnection.sendResponse(message.data['id']!, id);

      // Unforward will try to disconnect the remote port.
      portForwarder.forwardedPorts.single.dispose();
      expect(fakeServerSocket.closeCalled, true);

      message = await broadcastOutput.first;

      expect(message.data['id'], isNotNull);
      expect(message.data['method'], 'device.unforward');
      expect(message.data['params'], <String, Object?>{
        'deviceId': 'device_id',
        'devicePort': 300,
        'hostPort': 350,
      });
    });

    group('socket done', () {
      late Stream<DaemonMessage> broadcastOutput;
      late FakeSocket fakeSocket;
      const String id = 'random_id';

      setUp(() async {
        final FakeServerSocket fakeServerSocket = FakeServerSocket(400);
        final ProxiedPortForwarder portForwarder = ProxiedPortForwarder(
          clientDaemonConnection,
          deviceId: 'device_id',
          logger: bufferLogger,
          createSocketServer: (Logger logger, int? hostPort, bool? ipv6) async => fakeServerSocket,
        );

        broadcastOutput = serverDaemonConnection.incomingCommands.asBroadcastStream();

        unawaited(portForwarder.forward(300));

        // Consumes the message.
        DaemonMessage message = await broadcastOutput.first;
        serverDaemonConnection.sendResponse(message.data['id']!, <String, Object?>{
          'hostPort': 350,
        });

        fakeSocket = FakeSocket();
        fakeServerSocket.controller.add(fakeSocket);
        // Consumes the message.
        message = await broadcastOutput.first;

        serverDaemonConnection.sendResponse(message.data['id']!, id);
        // Pump the event queue so that the socket future error handler has a
        // chance to be listened to.
        await pumpEventQueue();
      });

      testWithoutContext('without error, should calls proxy.disconnect', () async {
        // It will try to disconnect the remote port when socket is done.
        fakeSocket.doneCompleter.complete(true);
        final DaemonMessage message = await broadcastOutput.first;

        expect(message.data['id'], isNotNull);
        expect(message.data['method'], 'proxy.disconnect');
        expect(message.data['params'], <String, Object?>{'id': 'random_id'});
      });

      testWithoutContext('with error, should also calls proxy.disconnect', () async {
        fakeSocket.doneCompleter.complete(true);
        final DaemonMessage message = await broadcastOutput.first;

        expect(message.data['id'], isNotNull);
        expect(message.data['method'], 'proxy.disconnect');
        expect(message.data['params'], <String, Object?>{'id': 'random_id'});

        // Send an error response and make sure that it won't crash the client.
        serverDaemonConnection.sendErrorResponse(
          message.data['id']!,
          'some error',
          StackTrace.current,
        );

        // Wait the event queue and make sure that it doesn't crash.
        await pumpEventQueue();
      });

      testWithoutContext('should not forward new data to socket after disconnection', () async {
        // Data will be forwarded before disconnection
        serverDaemonConnection.sendEvent('proxy.data.$id', null, <int>[1, 2, 3]);
        await pumpEventQueue();
        expect(fakeSocket.addedData, <List<int>>[
          <int>[1, 2, 3],
        ]);

        // It will try to disconnect the remote port when socket is done.
        fakeSocket.doneCompleter.complete(true);
        final DaemonMessage message = await broadcastOutput.first;

        expect(message.data['id'], isNotNull);
        expect(message.data['method'], 'proxy.disconnect');
        expect(message.data['params'], <String, Object?>{'id': 'random_id'});
        await pumpEventQueue();

        serverDaemonConnection.sendEvent('proxy.data.$id', null, <int>[4, 5, 6]);
        await pumpEventQueue();
        expect(fakeSocket.addedData, <List<int>>[
          <int>[1, 2, 3],
        ]);
      });
    });

    testWithoutContext('disposes multiple sockets correctly', () async {
      final FakeServerSocket fakeServerSocket = FakeServerSocket(200);
      final ProxiedPortForwarder portForwarder = ProxiedPortForwarder(
        clientDaemonConnection,
        logger: bufferLogger,
        createSocketServer: (Logger logger, int? hostPort, bool? ipv6) async => fakeServerSocket,
      );
      final int result = await portForwarder.forward(100);
      expect(result, 200);

      final FakeSocket fakeSocket1 = FakeSocket();
      final FakeSocket fakeSocket2 = FakeSocket();
      fakeServerSocket.controller.add(fakeSocket1);
      fakeServerSocket.controller.add(fakeSocket2);

      final Stream<DaemonMessage> broadcastOutput =
          serverDaemonConnection.incomingCommands.asBroadcastStream();

      final DaemonMessage message1 = await broadcastOutput.first;

      expect(message1.data['id'], isNotNull);
      expect(message1.data['method'], 'proxy.connect');
      expect(message1.data['params'], <String, Object?>{'port': 100});

      const String id1 = 'random_id1';
      serverDaemonConnection.sendResponse(message1.data['id']!, id1);

      final DaemonMessage message2 = await broadcastOutput.first;

      expect(message2.data['id'], isNotNull);
      expect(message2.data['id'], isNot(message1.data['id']));
      expect(message2.data['method'], 'proxy.connect');
      expect(message2.data['params'], <String, Object?>{'port': 100});

      const String id2 = 'random_id2';
      serverDaemonConnection.sendResponse(message2.data['id']!, id2);

      await pumpEventQueue();

      // Closes the socket after port forwarder dispose.
      expect(fakeSocket1.closeCalled, false);
      expect(fakeSocket2.closeCalled, false);
      await portForwarder.dispose();
      expect(fakeSocket1.closeCalled, true);
      expect(fakeSocket2.closeCalled, true);
    });
  });

  final Map<String, Object> fakeDevice = <String, Object>{
    'name': 'device-name',
    'id': 'device-id',
    'category': 'mobile',
    'platformType': 'android',
    'platform': 'android-arm',
    'emulator': true,
    'ephemeral': false,
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
  };
  final Map<String, Object> fakeDevice2 = <String, Object>{
    'name': 'device-name2',
    'id': 'device-id2',
    'category': 'mobile',
    'platformType': 'android',
    'platform': 'android-arm',
    'emulator': true,
    'ephemeral': false,
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
  };
  group('ProxiedDevice', () {
    testWithoutContext('calls stopApp without application package if not passed', () async {
      bufferLogger = BufferLogger.test();
      final ProxiedDevices proxiedDevices = ProxiedDevices(
        clientDaemonConnection,
        logger: bufferLogger,
      );
      final ProxiedDevice device = proxiedDevices.deviceFromDaemonResult(fakeDevice);
      unawaited(device.stopApp(null, userIdentifier: 'user-id'));
      final DaemonMessage message = await serverDaemonConnection.incomingCommands.first;
      expect(message.data['id'], isNotNull);
      expect(message.data['method'], 'device.stopApp');
      expect(message.data['params'], <String, Object?>{
        'deviceId': 'device-id',
        'userIdentifier': 'user-id',
      });
    });

    group('when launching an app with PrebuiltApplicationPackage', () {
      late MemoryFileSystem fileSystem;
      late FakePrebuiltApplicationPackage applicationPackage;
      const List<int> fileContent = <int>[100, 120, 140];
      setUp(() {
        fileSystem =
            MemoryFileSystem.test()
              ..directory('dir').createSync()
              ..file('dir/foo').writeAsBytesSync(fileContent);
        applicationPackage = FakePrebuiltApplicationPackage(fileSystem.file('dir/foo'));
      });

      testWithoutContext('transfers file to the daemon', () async {
        bufferLogger = BufferLogger.test();
        final ProxiedDevices proxiedDevices = ProxiedDevices(
          clientDaemonConnection,
          logger: bufferLogger,
          deltaFileTransfer: false,
        );
        final ProxiedDevice device = proxiedDevices.deviceFromDaemonResult(fakeDevice);

        final Stream<DaemonMessage> broadcastOutput =
            serverDaemonConnection.incomingCommands.asBroadcastStream();

        final Future<String> resultFuture = device.applicationPackageId(applicationPackage);

        // Send proxy.writeTempFile.
        final DaemonMessage writeTempFileMessage = await broadcastOutput.first;
        expect(writeTempFileMessage.data['id'], isNotNull);
        expect(writeTempFileMessage.data['method'], 'proxy.writeTempFile');
        expect(writeTempFileMessage.data['params'], <String, Object?>{'path': 'foo'});
        expect(await writeTempFileMessage.binary?.first, fileContent);

        serverDaemonConnection.sendResponse(writeTempFileMessage.data['id']!);

        // Send device.uploadApplicationPackage.
        final DaemonMessage uploadApplicationPackageMessage = await broadcastOutput.first;
        expect(uploadApplicationPackageMessage.data['id'], isNotNull);
        expect(uploadApplicationPackageMessage.data['method'], 'device.uploadApplicationPackage');
        expect(uploadApplicationPackageMessage.data['params'], <String, Object?>{
          'targetPlatform': 'android-arm',
          'applicationBinary': 'foo',
        });

        serverDaemonConnection.sendResponse(uploadApplicationPackageMessage.data['id']!, 'test_id');
        expect(await resultFuture, 'test_id');
      });

      testWithoutContext(
        'transfers file to the daemon with delta turned on, file not exist on remote',
        () async {
          bufferLogger = BufferLogger.test();
          final FakeFileTransfer fileTransfer = FakeFileTransfer();
          final ProxiedDevices proxiedDevices = ProxiedDevices(
            clientDaemonConnection,
            logger: bufferLogger,
            fileTransfer: fileTransfer,
          );
          final ProxiedDevice device = proxiedDevices.deviceFromDaemonResult(fakeDevice);

          final Stream<DaemonMessage> broadcastOutput =
              serverDaemonConnection.incomingCommands.asBroadcastStream();

          final Future<String> resultFuture = device.applicationPackageId(applicationPackage);

          // Send proxy.calculateFileHashes.
          final DaemonMessage calculateFileHashesMessage = await broadcastOutput.first;
          expect(calculateFileHashesMessage.data['id'], isNotNull);
          expect(calculateFileHashesMessage.data['method'], 'proxy.calculateFileHashes');
          expect(calculateFileHashesMessage.data['params'], <String, Object?>{'path': 'foo'});
          serverDaemonConnection.sendResponse(calculateFileHashesMessage.data['id']!);

          // Send proxy.writeTempFile.
          final DaemonMessage writeTempFileMessage = await broadcastOutput.first;
          expect(writeTempFileMessage.data['id'], isNotNull);
          expect(writeTempFileMessage.data['method'], 'proxy.writeTempFile');
          expect(writeTempFileMessage.data['params'], <String, Object?>{'path': 'foo'});
          expect(await writeTempFileMessage.binary?.first, fileContent);

          serverDaemonConnection.sendResponse(writeTempFileMessage.data['id']!);

          // Send device.uploadApplicationPackage.
          final DaemonMessage uploadApplicationPackageMessage = await broadcastOutput.first;
          expect(uploadApplicationPackageMessage.data['id'], isNotNull);
          expect(uploadApplicationPackageMessage.data['method'], 'device.uploadApplicationPackage');
          expect(uploadApplicationPackageMessage.data['params'], <String, Object?>{
            'targetPlatform': 'android-arm',
            'applicationBinary': 'foo',
          });

          serverDaemonConnection.sendResponse(
            uploadApplicationPackageMessage.data['id']!,
            'test_id',
          );
          expect(await resultFuture, 'test_id');
        },
      );

      testWithoutContext(
        'transfers file to the daemon with delta turned on, file exists on remote',
        () async {
          bufferLogger = BufferLogger.test();
          final FakeFileTransfer fileTransfer = FakeFileTransfer();
          const BlockHashes blockHashes = BlockHashes(
            blockSize: 10,
            totalSize: 30,
            adler32: <int>[1, 2, 3],
            md5: <String>['a', 'b', 'c'],
            fileMd5: 'abc',
          );
          const List<FileDeltaBlock> deltaBlocks = <FileDeltaBlock>[
            FileDeltaBlock.fromSource(start: 10, size: 10),
            FileDeltaBlock.fromDestination(start: 30, size: 40),
          ];
          fileTransfer.binary = Uint8List.fromList(<int>[11, 12, 13]);
          fileTransfer.delta = deltaBlocks;

          final ProxiedDevices proxiedDevices = ProxiedDevices(
            clientDaemonConnection,
            logger: bufferLogger,
            fileTransfer: fileTransfer,
          );
          final ProxiedDevice device = proxiedDevices.deviceFromDaemonResult(fakeDevice);

          final Stream<DaemonMessage> broadcastOutput =
              serverDaemonConnection.incomingCommands.asBroadcastStream();

          final Future<String> resultFuture = device.applicationPackageId(applicationPackage);

          // Send proxy.calculateFileHashes.
          final DaemonMessage calculateFileHashesMessage = await broadcastOutput.first;
          expect(calculateFileHashesMessage.data['id'], isNotNull);
          expect(calculateFileHashesMessage.data['method'], 'proxy.calculateFileHashes');
          expect(calculateFileHashesMessage.data['params'], <String, Object?>{'path': 'foo'});
          serverDaemonConnection.sendResponse(
            calculateFileHashesMessage.data['id']!,
            blockHashes.toJson(),
          );

          // Send proxy.updateFile.
          final DaemonMessage updateFileMessage = await broadcastOutput.first;
          expect(updateFileMessage.data['id'], isNotNull);
          expect(updateFileMessage.data['method'], 'proxy.updateFile');
          expect(updateFileMessage.data['params'], <String, Object?>{
            'path': 'foo',
            'delta': <Map<String, Object>>[
              <String, Object>{'size': 10},
              <String, Object>{'start': 30, 'size': 40},
            ],
          });
          expect(await updateFileMessage.binary?.first, <int>[11, 12, 13]);

          serverDaemonConnection.sendResponse(updateFileMessage.data['id']!);

          // Send device.uploadApplicationPackage.
          final DaemonMessage uploadApplicationPackageMessage = await broadcastOutput.first;
          expect(uploadApplicationPackageMessage.data['id'], isNotNull);
          expect(uploadApplicationPackageMessage.data['method'], 'device.uploadApplicationPackage');
          expect(uploadApplicationPackageMessage.data['params'], <String, Object?>{
            'targetPlatform': 'android-arm',
            'applicationBinary': 'foo',
          });

          serverDaemonConnection.sendResponse(
            uploadApplicationPackageMessage.data['id']!,
            'test_id',
          );
          expect(await resultFuture, 'test_id');
        },
      );
    });
  });

  group('ProxiedDevices', () {
    testWithoutContext('devices respects the filter passed in', () async {
      bufferLogger = BufferLogger.test();
      final ProxiedDevices proxiedDevices = ProxiedDevices(
        clientDaemonConnection,
        logger: bufferLogger,
      );

      final FakeDeviceDiscoveryFilter fakeFilter = FakeDeviceDiscoveryFilter();

      final FakeDevice supportedDevice = FakeDevice('Device', 'supported');
      fakeFilter.filteredDevices = <Device>[supportedDevice];

      final Future<List<Device>> resultFuture = proxiedDevices.devices(filter: fakeFilter);

      final DaemonMessage message = await serverDaemonConnection.incomingCommands.first;
      expect(message.data['id'], isNotNull);
      expect(message.data['method'], 'device.discoverDevices');

      serverDaemonConnection.sendResponse(message.data['id']!, <Map<String, Object?>>[
        fakeDevice,
        fakeDevice2,
      ]);

      final List<Device> result = await resultFuture;
      expect(result.length, 1);
      expect(result.first.id, supportedDevice.id);

      expect(fakeFilter.devices!.length, 2);
      expect(fakeFilter.devices![0].id, fakeDevice['id']);
      expect(fakeFilter.devices![1].id, fakeDevice2['id']);
    });

    testWithoutContext('publishes the devices on deviceNotifier after startPolling', () async {
      bufferLogger = BufferLogger.test();
      final ProxiedDevices proxiedDevices = ProxiedDevices(
        clientDaemonConnection,
        logger: bufferLogger,
      );

      proxiedDevices.startPolling();

      final ItemListNotifier<Device> deviceNotifier = proxiedDevices.deviceNotifier;
      expect(deviceNotifier, isNotNull);

      final List<Device> devicesAdded = <Device>[];
      deviceNotifier.onAdded.listen((Device device) {
        devicesAdded.add(device);
      });

      final DaemonMessage message = await serverDaemonConnection.incomingCommands.first;
      expect(message.data['id'], isNotNull);
      expect(message.data['method'], 'device.discoverDevices');

      serverDaemonConnection.sendResponse(message.data['id']!, <Map<String, Object?>>[
        fakeDevice,
        fakeDevice2,
      ]);

      await pumpEventQueue();

      expect(devicesAdded.length, 2);
      expect(devicesAdded[0].id, fakeDevice['id']);
      expect(devicesAdded[1].id, fakeDevice2['id']);
    });

    testWithoutContext('handles getDiagnostics', () async {
      bufferLogger = BufferLogger.test();
      final ProxiedDevices proxiedDevices = ProxiedDevices(
        clientDaemonConnection,
        logger: bufferLogger,
      );

      final Future<List<String>> resultFuture = proxiedDevices.getDiagnostics();

      final DaemonMessage message = await serverDaemonConnection.incomingCommands.first;
      expect(message.data['id'], isNotNull);
      expect(message.data['method'], 'device.getDiagnostics');

      serverDaemonConnection.sendResponse(message.data['id']!, <String>['1', '2']);

      final List<String> result = await resultFuture;
      expect(result, <String>['1', '2']);
    });

    testWithoutContext(
      'returns empty result when daemon does not understand getDiagnostics',
      () async {
        bufferLogger = BufferLogger.test();
        final ProxiedDevices proxiedDevices = ProxiedDevices(
          clientDaemonConnection,
          logger: bufferLogger,
        );

        final Future<List<String>> resultFuture = proxiedDevices.getDiagnostics();

        final DaemonMessage message = await serverDaemonConnection.incomingCommands.first;
        expect(message.data['id'], isNotNull);
        expect(message.data['method'], 'device.getDiagnostics');

        serverDaemonConnection.sendErrorResponse(
          message.data['id']!,
          'command not understood: device.getDiagnostics',
          StackTrace.current,
        );

        final List<String> result = await resultFuture;
        expect(result, isEmpty);
      },
    );
  });

  group('ProxiedDartDevelopmentService', () {
    testWithoutContext('forwards start and shutdown to remote', () async {
      final FakeProxiedPortForwarder portForwarder = FakeProxiedPortForwarder();
      portForwarder.originalRemotePortReturnValue = 200;
      portForwarder.forwardReturnValue = 400;
      final FakeProxiedPortForwarder devicePortForwarder = FakeProxiedPortForwarder();
      final ProxiedDartDevelopmentService dds = ProxiedDartDevelopmentService(
        clientDaemonConnection,
        'test_id',
        logger: bufferLogger,
        proxiedPortForwarder: portForwarder,
        devicePortForwarder: devicePortForwarder,
      );

      final Stream<DaemonMessage> broadcastOutput =
          serverDaemonConnection.incomingCommands.asBroadcastStream();

      final Future<void> startFuture = dds.startDartDevelopmentService(
        Uri.parse('http://127.0.0.1:100/fake'),
        disableServiceAuthCodes: true,
        ddsPort: 150,
        ipv6: false,
      );

      final DaemonMessage startMessage = await broadcastOutput.first;
      expect(startMessage.data['id'], isNotNull);
      expect(startMessage.data['method'], 'device.startDartDevelopmentService');
      expect(startMessage.data['params'], <String, Object?>{
        'deviceId': 'test_id',
        'vmServiceUri': 'http://127.0.0.1:200/fake',
        'disableServiceAuthCodes': true,
      });

      serverDaemonConnection.sendResponse(startMessage.data['id']!, const <String, Object?>{
        'ddsUri': 'http://127.0.0.1:300/remote',
      });

      await startFuture;
      expect(portForwarder.receivedLocalForwardedPort, 100);
      expect(portForwarder.forwardedDevicePort, 300);
      expect(portForwarder.forwardedHostPort, 150);
      expect(portForwarder.forwardedIpv6, false);

      expect(dds.uri, Uri.parse('http://127.0.0.1:400/remote'));

      expect(
        bufferLogger.eventText.trim(),
        '{"name":"device.proxied_dds_forwarded","args":{"deviceId":"test_id","remoteUri":"http://127.0.0.1:300/remote","localUri":"http://127.0.0.1:400/remote"}}',
      );

      unawaited(dds.shutdown());

      final DaemonMessage shutdownMessage = await broadcastOutput.first;
      expect(shutdownMessage.data['id'], isNotNull);
      expect(shutdownMessage.data['method'], 'device.shutdownDartDevelopmentService');
      expect(shutdownMessage.data['params'], <String, Object?>{'deviceId': 'test_id'});
    });

    testWithoutContext(
      'forwards start and shutdown to remote if port was forwarded by the device port forwarder',
      () async {
        final FakeProxiedPortForwarder portForwarder = FakeProxiedPortForwarder();
        portForwarder.forwardReturnValue = 400;
        final FakeProxiedPortForwarder devicePortForwarder = FakeProxiedPortForwarder();
        devicePortForwarder.originalRemotePortReturnValue = 200;
        final ProxiedDartDevelopmentService dds = ProxiedDartDevelopmentService(
          clientDaemonConnection,
          'test_id',
          logger: bufferLogger,
          proxiedPortForwarder: portForwarder,
          devicePortForwarder: devicePortForwarder,
        );

        final Stream<DaemonMessage> broadcastOutput =
            serverDaemonConnection.incomingCommands.asBroadcastStream();

        final Future<void> startFuture = dds.startDartDevelopmentService(
          Uri.parse('http://127.0.0.1:100/fake'),
          disableServiceAuthCodes: true,
          ddsPort: 150,
          ipv6: false,
        );

        final DaemonMessage startMessage = await broadcastOutput.first;
        expect(startMessage.data['id'], isNotNull);
        expect(startMessage.data['method'], 'device.startDartDevelopmentService');
        expect(startMessage.data['params'], <String, Object?>{
          'deviceId': 'test_id',
          'vmServiceUri': 'http://127.0.0.1:200/fake',
          'disableServiceAuthCodes': true,
        });

        serverDaemonConnection.sendResponse(startMessage.data['id']!, <String, Object?>{
          'ddsUri': 'http://127.0.0.1:300/remote',
        });

        await startFuture;
        expect(portForwarder.receivedLocalForwardedPort, 100);
        expect(portForwarder.forwardedDevicePort, 300);
        expect(portForwarder.forwardedHostPort, 150);
        expect(portForwarder.forwardedIpv6, false);

        expect(dds.uri, Uri.parse('http://127.0.0.1:400/remote'));

        expect(
          bufferLogger.eventText.trim(),
          '{"name":"device.proxied_dds_forwarded","args":{"deviceId":"test_id","remoteUri":"http://127.0.0.1:300/remote","localUri":"http://127.0.0.1:400/remote"}}',
        );

        unawaited(dds.shutdown());

        final DaemonMessage shutdownMessage = await broadcastOutput.first;
        expect(shutdownMessage.data['id'], isNotNull);
        expect(shutdownMessage.data['method'], 'device.shutdownDartDevelopmentService');
        expect(shutdownMessage.data['params'], <String, Object?>{'deviceId': 'test_id'});
      },
    );

    testWithoutContext(
      'starts a local dds if the VM service port is not a forwarded port',
      () async {
        final FakeProxiedPortForwarder portForwarder = FakeProxiedPortForwarder();
        final FakeProxiedPortForwarder devicePortForwarder = FakeProxiedPortForwarder();
        final FakeDartDevelopmentService localDds = FakeDartDevelopmentService();
        localDds.uri = Uri.parse('http://127.0.0.1:450/local');
        final ProxiedDartDevelopmentService dds = ProxiedDartDevelopmentService(
          clientDaemonConnection,
          'test_id',
          logger: bufferLogger,
          proxiedPortForwarder: portForwarder,
          devicePortForwarder: devicePortForwarder,
          localDds: localDds,
        );

        expect(localDds.startCalled, false);
        await dds.startDartDevelopmentService(
          Uri.parse('http://127.0.0.1:100/fake'),
          disableServiceAuthCodes: true,
          ddsPort: 150,
          ipv6: false,
        );

        expect(localDds.startCalled, true);
        expect(portForwarder.receivedLocalForwardedPort, 100);
        expect(portForwarder.forwardedDevicePort, null);

        expect(dds.uri, Uri.parse('http://127.0.0.1:450/local'));

        expect(localDds.shutdownCalled, false);
        await dds.shutdown();
        expect(localDds.shutdownCalled, true);

        await serverDaemonConnection.dispose();
        expect(await serverDaemonConnection.incomingCommands.isEmpty, true);
      },
    );

    testWithoutContext(
      'starts a local dds if the remote VM does not support starting DDS',
      () async {
        final FakeProxiedPortForwarder portForwarder = FakeProxiedPortForwarder();
        portForwarder.originalRemotePortReturnValue = 200;
        final FakeProxiedPortForwarder devicePortForwarder = FakeProxiedPortForwarder();
        final FakeDartDevelopmentService localDds = FakeDartDevelopmentService();
        localDds.uri = Uri.parse('http://127.0.0.1:450/local');
        final ProxiedDartDevelopmentService dds = ProxiedDartDevelopmentService(
          clientDaemonConnection,
          'test_id',
          logger: bufferLogger,
          proxiedPortForwarder: portForwarder,
          devicePortForwarder: devicePortForwarder,
          localDds: localDds,
        );

        final Stream<DaemonMessage> broadcastOutput =
            serverDaemonConnection.incomingCommands.asBroadcastStream();

        final Future<void> startFuture = dds.startDartDevelopmentService(
          Uri.parse('http://127.0.0.1:100/fake'),
          disableServiceAuthCodes: true,
          ddsPort: 150,
          ipv6: false,
        );

        expect(localDds.startCalled, false);
        final DaemonMessage startMessage = await broadcastOutput.first;
        expect(startMessage.data['id'], isNotNull);
        expect(startMessage.data['method'], 'device.startDartDevelopmentService');
        expect(startMessage.data['params'], <String, Object?>{
          'deviceId': 'test_id',
          'vmServiceUri': 'http://127.0.0.1:200/fake',
          'disableServiceAuthCodes': true,
        });

        serverDaemonConnection.sendErrorResponse(
          startMessage.data['id']!,
          'command not understood: device.startDartDevelopmentService',
          StackTrace.current,
        );

        await startFuture;
        expect(localDds.startCalled, true);
        expect(portForwarder.receivedLocalForwardedPort, 100);
        expect(portForwarder.forwardedDevicePort, null);

        expect(dds.uri, Uri.parse('http://127.0.0.1:450/local'));

        expect(localDds.shutdownCalled, false);
        await dds.shutdown();
        expect(localDds.shutdownCalled, true);
      },
    );
  });

  group('ProxiedVMServiceDiscoveryForAttach', () {
    testWithoutContext('sends the request and forwards the port', () async {
      final FakeProxiedPortForwarder portForwarder = FakeProxiedPortForwarder();
      portForwarder.forwardReturnValue = 400;
      final ProxiedVMServiceDiscoveryForAttach discovery = ProxiedVMServiceDiscoveryForAttach(
        clientDaemonConnection,
        'test_device',
        proxiedPortForwarder: portForwarder,
        fallbackDiscovery: () => throw UnimplementedError(),
        ipv6: false,
        logger: bufferLogger,
      );

      final Completer<Uri> uriCompleter = Completer<Uri>();

      // Start listening on the stream to trigger sending the request.
      discovery.uris.listen(uriCompleter.complete);

      final Stream<DaemonMessage> broadcastOutput =
          serverDaemonConnection.incomingCommands.asBroadcastStream();
      final DaemonMessage startMessage = await broadcastOutput.first;
      expect(startMessage.data['id'], isNotNull);
      expect(startMessage.data['method'], 'device.startVMServiceDiscoveryForAttach');
      expect(startMessage.data['params'], <String, Object?>{
        'deviceId': 'test_device',
        'appId': null,
        'fuchsiaModule': null,
        'filterDevicePort': null,
        'ipv6': false,
      });

      serverDaemonConnection.sendResponse(startMessage.data['id']!, 'request_id');
      serverDaemonConnection.sendEvent(
        'device.VMServiceDiscoveryForAttach.request_id',
        'http://127.0.0.1:300/auth_code',
      );

      expect(await uriCompleter.future, Uri.parse('http://127.0.0.1:400/auth_code'));
      expect(portForwarder.forwardedDevicePort, 300);
      expect(portForwarder.forwardedHostPort, null);
    });

    testWithoutContext('sends additional information, and forwards the correct port', () async {
      final FakeProxiedPortForwarder portForwarder = FakeProxiedPortForwarder();
      portForwarder.forwardReturnValue = 400;
      final ProxiedVMServiceDiscoveryForAttach discovery = ProxiedVMServiceDiscoveryForAttach(
        clientDaemonConnection,
        'test_device',
        proxiedPortForwarder: portForwarder,
        fallbackDiscovery: () => throw UnimplementedError(),
        appId: 'test_app_id',
        fuchsiaModule: 'test_fuchsia_module',
        filterDevicePort: 100,
        expectedHostPort: 200,
        ipv6: false,
        logger: bufferLogger,
      );

      final Completer<Uri> uriCompleter = Completer<Uri>();

      // Start listening on the stream to trigger sending the request.
      discovery.uris.listen(uriCompleter.complete);

      final Stream<DaemonMessage> broadcastOutput =
          serverDaemonConnection.incomingCommands.asBroadcastStream();
      final DaemonMessage startMessage = await broadcastOutput.first;
      expect(startMessage.data['id'], isNotNull);
      expect(startMessage.data['method'], 'device.startVMServiceDiscoveryForAttach');
      expect(startMessage.data['params'], <String, Object?>{
        'deviceId': 'test_device',
        'appId': 'test_app_id',
        'fuchsiaModule': 'test_fuchsia_module',
        'filterDevicePort': 100,
        'ipv6': false,
      });

      serverDaemonConnection.sendResponse(startMessage.data['id']!, 'request_id');
      serverDaemonConnection.sendEvent(
        'device.VMServiceDiscoveryForAttach.request_id',
        'http://127.0.0.1:300/auth_code',
      );

      expect(await uriCompleter.future, Uri.parse('http://127.0.0.1:400/auth_code'));
      expect(portForwarder.forwardedDevicePort, 300);
      expect(portForwarder.forwardedHostPort, 200);
    });

    testWithoutContext(
      'use the fallback discovery if the remote daemon does not support proxied discovery',
      () async {
        final FakeProxiedPortForwarder portForwarder = FakeProxiedPortForwarder();
        final Stream<Uri> fallbackUri = Stream<Uri>.value(
          Uri.parse('http://127.0.0.1:500/fallback_auth_code'),
        );
        final ProxiedVMServiceDiscoveryForAttach discovery = ProxiedVMServiceDiscoveryForAttach(
          clientDaemonConnection,
          'test_device',
          proxiedPortForwarder: portForwarder,
          fallbackDiscovery: () => FakeVMServiceDiscoveryForAttach(fallbackUri),
          ipv6: false,
          logger: bufferLogger,
        );

        final Completer<Uri> uriCompleter = Completer<Uri>();

        // Start listening on the stream to trigger sending the request.
        discovery.uris.listen(uriCompleter.complete);

        final Stream<DaemonMessage> broadcastOutput =
            serverDaemonConnection.incomingCommands.asBroadcastStream();
        final DaemonMessage startMessage = await broadcastOutput.first;
        expect(startMessage.data['id'], isNotNull);
        expect(startMessage.data['method'], 'device.startVMServiceDiscoveryForAttach');
        expect(startMessage.data['params'], <String, Object?>{
          'deviceId': 'test_device',
          'appId': null,
          'fuchsiaModule': null,
          'filterDevicePort': null,
          'ipv6': false,
        });
        serverDaemonConnection.sendErrorResponse(
          startMessage.data['id']!,
          'command not understood: device.startDartDevelopmentService',
          StackTrace.current,
        );

        expect(await uriCompleter.future, Uri.parse('http://127.0.0.1:500/fallback_auth_code'));
        expect(portForwarder.forwardedDevicePort, null);
        expect(portForwarder.forwardedHostPort, null);
      },
    );

    testWithoutContext('forwards other error from the daemon', () async {
      final FakeProxiedPortForwarder portForwarder = FakeProxiedPortForwarder();
      final Stream<Uri> fallbackUri = Stream<Uri>.value(
        Uri.parse('http://127.0.0.1:500/fallback_auth_code'),
      );
      final ProxiedVMServiceDiscoveryForAttach discovery = ProxiedVMServiceDiscoveryForAttach(
        clientDaemonConnection,
        'test_device',
        proxiedPortForwarder: portForwarder,
        fallbackDiscovery: () => FakeVMServiceDiscoveryForAttach(fallbackUri),
        ipv6: false,
        logger: bufferLogger,
      );

      // Start listening on the stream to trigger sending the request.
      final Future<Uri> uriFuture = discovery.uris.first;

      final Stream<DaemonMessage> broadcastOutput =
          serverDaemonConnection.incomingCommands.asBroadcastStream();
      final DaemonMessage startMessage = await broadcastOutput.first;
      expect(startMessage.data['id'], isNotNull);
      expect(startMessage.data['method'], 'device.startVMServiceDiscoveryForAttach');
      expect(startMessage.data['params'], <String, Object?>{
        'deviceId': 'test_device',
        'appId': null,
        'fuchsiaModule': null,
        'filterDevicePort': null,
        'ipv6': false,
      });
      serverDaemonConnection.sendErrorResponse(
        startMessage.data['id']!,
        'other error',
        StackTrace.current,
      );

      expect(uriFuture, throwsA('other error'));
      expect(portForwarder.forwardedDevicePort, null);
      expect(portForwarder.forwardedHostPort, null);
    });

    testWithoutContext('forwards the port forwarder error', () async {
      final FakeProxiedPortForwarder portForwarder = FakeProxiedPortForwarder();
      portForwarder.forwardThrowException = TestException();
      final ProxiedVMServiceDiscoveryForAttach discovery = ProxiedVMServiceDiscoveryForAttach(
        clientDaemonConnection,
        'test_device',
        proxiedPortForwarder: portForwarder,
        fallbackDiscovery: () => throw UnimplementedError(),
        ipv6: false,
        logger: bufferLogger,
      );

      // Start listening on the stream to trigger sending the request.
      final Future<Uri> uriFuture = discovery.uris.first;

      final Stream<DaemonMessage> broadcastOutput =
          serverDaemonConnection.incomingCommands.asBroadcastStream();
      final DaemonMessage startMessage = await broadcastOutput.first;
      expect(startMessage.data['id'], isNotNull);
      expect(startMessage.data['method'], 'device.startVMServiceDiscoveryForAttach');
      expect(startMessage.data['params'], <String, Object?>{
        'deviceId': 'test_device',
        'appId': null,
        'fuchsiaModule': null,
        'filterDevicePort': null,
        'ipv6': false,
      });

      serverDaemonConnection.sendResponse(startMessage.data['id']!, 'request_id');
      serverDaemonConnection.sendEvent(
        'device.VMServiceDiscoveryForAttach.request_id',
        'http://127.0.0.1:300/auth_code',
      );

      expect(uriFuture, throwsA(isA<TestException>()));
    });
  });
}

class FakeDaemonStreams implements DaemonStreams {
  final StreamController<DaemonMessage> inputs = StreamController<DaemonMessage>();
  final StreamController<DaemonMessage> outputs = StreamController<DaemonMessage>();

  @override
  Stream<DaemonMessage> get inputStream {
    return inputs.stream;
  }

  @override
  void send(Map<String, dynamic> message, [List<int>? binary]) {
    outputs.add(DaemonMessage(message, binary != null ? Stream<List<int>>.value(binary) : null));
  }

  @override
  Future<void> dispose() async {
    await inputs.close();
    // In some tests, outputs have no listeners. We don't wait for outputs to close.
    unawaited(outputs.close());
  }
}

class FakeServerSocket extends Fake implements ServerSocket {
  FakeServerSocket(this.port);

  @override
  final int port;

  bool closeCalled = false;
  final StreamController<Socket> controller = StreamController<Socket>();

  @override
  StreamSubscription<Socket> listen(
    void Function(Socket event)? onData, {
    Function? onError,
    void Function()? onDone,
    bool? cancelOnError,
  }) {
    return controller.stream.listen(
      onData,
      onError: onError,
      onDone: onDone,
      cancelOnError: cancelOnError,
    );
  }

  @override
  Future<ServerSocket> close() async {
    closeCalled = true;
    return this;
  }
}

class FakeSocket extends Fake implements Socket {
  bool closeCalled = false;
  final StreamController<Uint8List> controller = StreamController<Uint8List>();
  final List<List<int>> addedData = <List<int>>[];
  final Completer<bool> doneCompleter = Completer<bool>();

  @override
  StreamSubscription<Uint8List> listen(
    void Function(Uint8List event)? onData, {
    Function? onError,
    void Function()? onDone,
    bool? cancelOnError,
  }) {
    return controller.stream.listen(
      onData,
      onError: onError,
      onDone: onDone,
      cancelOnError: cancelOnError,
    );
  }

  @override
  void add(List<int> data) {
    addedData.add(data);
  }

  @override
  Future<void> close() async {
    closeCalled = true;
    doneCompleter.complete(true);
  }

  @override
  Future<bool> get done => doneCompleter.future;

  @override
  void destroy() {}
}

class FakeDaemonConnection extends Fake implements DaemonConnection {
  FakeDaemonConnection({
    this.handledRequests = const <String, Object?>{},
    this.daemonEventStreams = const <String, List<DaemonEventData>>{},
  });

  /// Mapping of method name to returned object from the [sendRequest] method.
  final Map<String, Object?> handledRequests;

  final Map<String, List<DaemonEventData>> daemonEventStreams;

  @override
  Stream<DaemonEventData> listenToEvent(String eventToListen) {
    final List<DaemonEventData>? iterable = daemonEventStreams[eventToListen];
    if (iterable != null) {
      return Stream<DaemonEventData>.fromIterable(iterable);
    }
    return const Stream<DaemonEventData>.empty();
  }

  @override
  Future<Object?> sendRequest(String method, [Object? params, List<int>? binary]) async {
    final Object? response = handledRequests[method];
    if (response != null) {
      return response;
    }
    throw Exception('"$method" request failed');
  }
}

class FakeDeviceDiscoveryFilter extends Fake implements DeviceDiscoveryFilter {
  List<Device>? filteredDevices;
  List<Device>? devices;

  @override
  Future<List<Device>> filterDevices(List<Device> devices) async {
    this.devices = devices;
    return filteredDevices!;
  }
}

class FakeProxiedPortForwarder extends Fake implements ProxiedPortForwarder {
  int? originalRemotePortReturnValue;
  int? receivedLocalForwardedPort;

  Exception? forwardThrowException;
  int? forwardReturnValue;
  int? forwardedDevicePort;
  int? forwardedHostPort;
  bool? forwardedIpv6;

  @override
  int? originalRemotePort(int localForwardedPort) {
    receivedLocalForwardedPort = localForwardedPort;
    return originalRemotePortReturnValue;
  }

  @override
  Future<int> forward(int devicePort, {int? hostPort, bool? ipv6}) async {
    forwardedDevicePort = devicePort;
    forwardedHostPort = hostPort;
    forwardedIpv6 = ipv6;
    if (forwardThrowException != null) {
      throw forwardThrowException!;
    }
    return forwardReturnValue!;
  }
}

class FakeDartDevelopmentService extends Fake implements DartDevelopmentService {
  bool startCalled = false;
  Uri? startUri;

  bool shutdownCalled = false;

  @override
  Future<void> get done => _completer.future;
  final Completer<void> _completer = Completer<void>();

  @override
  Uri? uri;

  @override
  Future<void> startDartDevelopmentService(
    Uri vmServiceUri, {
    FlutterDevice? device,
    int? ddsPort,
    bool? ipv6,
    bool? disableServiceAuthCodes,
    bool enableDevTools = false,
    bool cacheStartupProfile = false,
    String? google3WorkspaceRoot,
    Uri? devToolsServerAddress,
  }) async {
    startCalled = true;
    startUri = vmServiceUri;
  }

  @override
  Future<void> shutdown() async => shutdownCalled = true;
}

class FakePrebuiltApplicationPackage extends Fake implements PrebuiltApplicationPackage {
  FakePrebuiltApplicationPackage(this.applicationPackage);
  @override
  final FileSystemEntity applicationPackage;
}

class FakeFileTransfer extends Fake implements FileTransfer {
  List<FileDeltaBlock>? delta;
  Uint8List? binary;
  @override
  Future<List<FileDeltaBlock>> computeDelta(File file, BlockHashes hashes) async => delta!;

  @override
  Future<Uint8List> binaryForRebuilding(File file, List<FileDeltaBlock> delta) async => binary!;
}

class FakeVMServiceDiscoveryForAttach extends Fake implements VMServiceDiscoveryForAttach {
  FakeVMServiceDiscoveryForAttach(this.uris);

  @override
  Stream<Uri> uris;
}

class TestException implements Exception {}
