// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:path/path.dart' as path;

import '../framework/devices.dart';
import '../framework/framework.dart';
import '../framework/task_result.dart';
import '../framework/utils.dart';

const String _kOrgName = 'com.example.activitydestroy';

final RegExp _lifecycleSentinelRegExp = RegExp(r'==== lifecycle\: (.+) ====');

/// Tests the following Android lifecycles: Activity#onStop(), Activity#onResume(), Activity#onPause(),
/// and Activity#onDestroy() from Dart perspective in debug, profile, and release modes.
TaskFunction androidLifecyclesTest({Map<String, String>? environment}) {
  final Directory tempDir = Directory.systemTemp.createTempSync(
    'flutter_devicelab_activity_destroy.',
  );
  return () async {
    try {
      section('Create app');
      await inDirectory(tempDir, () async {
        await flutter(
          'create',
          options: <String>['--platforms', 'android', '--org', _kOrgName, 'app'],
          environment: environment,
        );
      });

      final File mainDart = File(path.join(tempDir.absolute.path, 'app', 'lib', 'main.dart'));
      if (!mainDart.existsSync()) {
        return TaskResult.failure('${mainDart.path} does not exist');
      }

      section('Patch lib/main.dart');
      await mainDart.writeAsString(r'''
 import 'package:flutter/widgets.dart';

class LifecycleObserver extends WidgetsBindingObserver {
  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    print('==== lifecycle: $state ====');
  }
}

void main() {
  WidgetsFlutterBinding.ensureInitialized();
  WidgetsBinding.instance.addObserver(LifecycleObserver());
  runApp(Container());
}
''', flush: true);

      Future<TaskResult> runTestFor(String mode) async {
        final AndroidDevice device = await devices.workingDevice as AndroidDevice;
        await device.unlock();

        section('Flutter run on device running API level ${device.apiLevel} (mode: $mode)');

        late Process run;
        await inDirectory(path.join(tempDir.path, 'app'), () async {
          run = await startFlutter('run', options: <String>['--$mode']);
        });

        final StreamController<String> lifecycles = StreamController<String>();
        final StreamIterator<String> lifecycleItr = StreamIterator<String>(lifecycles.stream);

        final StreamSubscription<void> stdout = run.stdout
            .transform<String>(utf8.decoder)
            .transform<String>(const LineSplitter())
            .listen((String log) {
              final RegExpMatch? match = _lifecycleSentinelRegExp.firstMatch(log);
              print('stdout: $log');
              if (match == null) {
                return;
              }
              final String lifecycle = match[1]!;
              print('stdout: Found app lifecycle: $lifecycle');
              lifecycles.add(lifecycle);
            });

        final StreamSubscription<void> stderr = run.stderr
            .transform<String>(utf8.decoder)
            .transform<String>(const LineSplitter())
            .listen((String log) {
              print('stderr: $log');
            });

        Future<void> expectedLifecycle(String expected) async {
          section('Wait for lifecycle: $expected (mode: $mode)');
          await lifecycleItr.moveNext();
          final String got = lifecycleItr.current;
          if (expected != got) {
            throw TaskResult.failure('expected lifecycles: `$expected`, but got` $got`');
          }
        }

        await expectedLifecycle('AppLifecycleState.resumed');

        section('Toggling app switch (mode: $mode)');
        await device.shellExec('input', <String>['keyevent', 'KEYCODE_APP_SWITCH']);

        await expectedLifecycle('AppLifecycleState.inactive');
        if (device.apiLevel == 28) {
          // Device lab currently runs 28.
          await expectedLifecycle('AppLifecycleState.paused');
          await expectedLifecycle('AppLifecycleState.detached');
        }

        section('Bring activity to foreground (mode: $mode)');
        await device.shellExec('am', <String>['start', '-n', '$_kOrgName.app/.MainActivity']);

        await expectedLifecycle('AppLifecycleState.resumed');

        section('Launch Settings app (mode: $mode)');
        await device.shellExec('am', <String>['start', '-a', 'android.settings.SETTINGS']);

        await expectedLifecycle('AppLifecycleState.inactive');
        if (device.apiLevel == 28) {
          // Device lab currently runs 28.
          await expectedLifecycle('AppLifecycleState.paused');
          await expectedLifecycle('AppLifecycleState.detached');
        }

        section('Bring activity to foreground (mode: $mode)');
        await device.shellExec('am', <String>['start', '-n', '$_kOrgName.app/.MainActivity']);

        await expectedLifecycle('AppLifecycleState.resumed');

        run.kill();

        section('Stop subscriptions (mode: $mode)');

        await lifecycleItr.cancel();
        await lifecycles.close();
        await stdout.cancel();
        await stderr.cancel();
        return TaskResult.success(null);
      }

      final TaskResult debugResult = await runTestFor('debug');
      if (debugResult.failed) {
        return debugResult;
      }

      final TaskResult profileResult = await runTestFor('profile');
      if (profileResult.failed) {
        return profileResult;
      }

      final TaskResult releaseResult = await runTestFor('release');
      if (releaseResult.failed) {
        return releaseResult;
      }

      return TaskResult.success(null);
    } on TaskResult catch (error) {
      return error;
    } finally {
      rmTree(tempDir);
    }
  };
}
