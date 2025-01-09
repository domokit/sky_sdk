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

void main() {
  task(() async {
    int? vmServicePort;

    final Device device = await devices.workingDevice;
    await device.unlock();
    final Directory appDir = dir(path.join(flutterDirectory.path, 'dev/integration_tests/ui'));
    section('TEST WHETHER `flutter drive --route` WORKS');
    await inDirectory(appDir, () async {
      return flutter(
        'drive',
        options: <String>[
          '--verbose',
          '-d',
          device.deviceId,
          '--route',
          '/smuggle-it',
          'lib/route.dart',
        ],
      );
    });
    section('TEST WHETHER `flutter run --route` WORKS');
    await inDirectory(appDir, () async {
      final Completer<void> ready = Completer<void>();
      late bool ok;
      print('run: starting...');
      final Process run = await startFlutter(
        'run',
        // --fast-start does not support routes.
        options: <String>[
          '--verbose',
          '--disable-service-auth-codes',
          '--no-fast-start',
          '--no-publish-port',
          '-d',
          device.deviceId,
          '--route',
          '/smuggle-it',
          'lib/route.dart',
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
      print('drive: starting...');
      final Process drive = await startFlutter(
        'drive',
        options: <String>[
          '--use-existing-app',
          'http://127.0.0.1:$vmServicePort/',
          '--no-keep-app-running',
          'lib/route.dart',
        ],
      );
      drive.stdout.transform<String>(utf8.decoder).transform<String>(const LineSplitter()).listen((
        String line,
      ) {
        print('drive:stdout: $line');
      });
      drive.stderr.transform<String>(utf8.decoder).transform<String>(const LineSplitter()).listen((
        String line,
      ) {
        stderr.writeln('drive:stderr: $line');
      });
      int result;
      result = await drive.exitCode;
      await flutter('install', options: <String>['--uninstall-only']);
      if (result != 0) {
        throw 'Failed to drive test app (exit code $result).';
      }
      result = await run.exitCode;
      if (result != 0) {
        throw 'Received unexpected exit code $result from run process.';
      }
    });
    return TaskResult.success(null);
  });
}
