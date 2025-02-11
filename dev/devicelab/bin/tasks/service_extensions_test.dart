// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:flutter_devicelab/framework/devices.dart';
import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/task_result.dart';
import 'package:flutter_devicelab/framework/utils.dart';
import 'package:path/path.dart' as path;
import 'package:vm_service/vm_service.dart';
import 'package:vm_service/vm_service_io.dart';

void main() {
  task(() async {
    int? vmServicePort;

    final Device device = await devices.workingDevice;
    await device.unlock();
    final Directory appDir = dir(path.join(flutterDirectory.path, 'dev/integration_tests/ui'));
    await inDirectory(appDir, () async {
      final Completer<void> ready = Completer<void>();
      late bool ok;
      print('run: starting...');
      final Process run = await startFlutter(
        'run',
        options: <String>[
          '--no-fast-start',
          '--no-publish-port',
          '--disable-service-auth-codes',
          '-d',
          device.deviceId,
          'lib/main.dart',
        ],
      );
      run.stdout.transform<String>(utf8.decoder).transform<String>(const LineSplitter()).listen((
        String line,
      ) {
        print('run:stdout: $line');
        if (vmServicePort == null) {
          vmServicePort = parseServicePort(line);
          if (vmServicePort != null) {
            print('service protocol connection available at port $vmServicePort');
            print('run: ready!');
            ready.complete();
            ok = true;
          }
        }
      });
      run.stderr.transform<String>(utf8.decoder).transform<String>(const LineSplitter()).listen((
        String line,
      ) {
        stderr.writeln('run:stderr: $line');
      });
      unawaited(
        run.exitCode.then<void>((int exitCode) {
          ok = false;
        }),
      );
      await Future.any<dynamic>(<Future<dynamic>>[ready.future, run.exitCode]);
      if (!ok) {
        throw 'Failed to run test app.';
      }

      final VmService client = await vmServiceConnectUri('ws://localhost:$vmServicePort/ws');
      final VM vm = await client.getVM();
      final IsolateRef isolate = vm.isolates!.first;

      final StreamController<Event> frameEventsController = StreamController<Event>();
      final StreamController<Event> navigationEventsController = StreamController<Event>();
      try {
        await client.streamListen(EventKind.kExtension);
      } catch (err) {
        // Do nothing on errors.
      }
      client.onExtensionEvent.listen((Event event) {
        if (event.extensionKind == 'Flutter.Frame') {
          frameEventsController.add(event);
        } else if (event.extensionKind == 'Flutter.Navigation') {
          navigationEventsController.add(event);
        }
      });

      final Stream<Event> frameEvents = frameEventsController.stream;
      final Stream<Event> navigationEvents = navigationEventsController.stream;

      print('reassembling app...');
      final Future<Event> frameFuture = frameEvents.first;
      await client.callServiceExtension('ext.flutter.reassemble', isolateId: isolate.id);

      // ensure we get an event
      final Event event = await frameFuture;
      print('${event.kind}: ${event.data}');

      // validate the fields
      // {number: 8, startTime: 0, elapsed: 1437, build: 600, raster: 800}
      print(event.extensionData!.data);
      expect(event.extensionData!.data['number'] is int);
      expect((event.extensionData!.data['number'] as int) >= 0);
      expect(event.extensionData!.data['startTime'] is int);
      expect((event.extensionData!.data['startTime'] as int) >= 0);
      expect(event.extensionData!.data['elapsed'] is int);
      expect((event.extensionData!.data['elapsed'] as int) >= 0);
      expect(event.extensionData!.data['build'] is int);
      expect((event.extensionData!.data['build'] as int) >= 0);
      expect(event.extensionData!.data['raster'] is int);
      expect((event.extensionData!.data['raster'] as int) >= 0);

      final Future<Event> navigationFuture = navigationEvents.first;
      // This tap triggers a navigation event.
      unawaited(device.tap(100, 200));

      final Event navigationEvent = await navigationFuture;
      // validate the fields
      expect(navigationEvent.extensionData!.data['route'] is Map<dynamic, dynamic>);
      final Map<dynamic, dynamic> route =
          navigationEvent.extensionData!.data['route'] as Map<dynamic, dynamic>;
      expect(route['description'] is String);
      expect(route['settings'] is Map<dynamic, dynamic>);
      final Map<dynamic, dynamic> settings = route['settings'] as Map<dynamic, dynamic>;
      expect(settings.containsKey('name'));

      run.stdin.write('q');
      final int result = await run.exitCode;
      if (result != 0) {
        throw 'Received unexpected exit code $result from run process.';
      }
    });
    return TaskResult.success(null);
  });
}

void expect(bool value) {
  if (!value) {
    throw 'failed assertion in service extensions test';
  }
}
