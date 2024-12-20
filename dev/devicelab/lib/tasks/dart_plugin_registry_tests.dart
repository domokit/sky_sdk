// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:path/path.dart' as path;

import '../framework/framework.dart';
import '../framework/task_result.dart';
import '../framework/utils.dart';

TaskFunction dartPluginRegistryTest({String? deviceIdOverride, Map<String, String>? environment}) {
  final Directory tempDir = Directory.systemTemp.createTempSync(
    'flutter_devicelab_dart_plugin_test.',
  );
  return () async {
    try {
      section('Create implementation plugin');
      await inDirectory(tempDir, () async {
        await flutter(
          'create',
          options: <String>[
            '--template=plugin',
            '--org',
            'io.flutter.devicelab',
            '--platforms',
            'macos',
            'aplugin_platform_implementation',
          ],
          environment: environment,
        );
      });

      final File pluginMain = File(
        path.join(
          tempDir.absolute.path,
          'aplugin_platform_implementation',
          'lib',
          'aplugin_platform_implementation.dart',
        ),
      );
      if (!pluginMain.existsSync()) {
        return TaskResult.failure('${pluginMain.path} does not exist');
      }

      // Patch plugin main dart file.
      await pluginMain.writeAsString('''
class ApluginPlatformInterfaceMacOS {
  static void registerWith() {
    print('ApluginPlatformInterfaceMacOS.registerWith() was called');
  }
}
''', flush: true);

      // Patch plugin main pubspec file.
      final File pluginImplPubspec = File(
        path.join(tempDir.absolute.path, 'aplugin_platform_implementation', 'pubspec.yaml'),
      );
      String pluginImplPubspecContent = await pluginImplPubspec.readAsString();
      pluginImplPubspecContent = pluginImplPubspecContent.replaceFirst(
        '        pluginClass: ApluginPlatformImplementationPlugin',
        '        pluginClass: ApluginPlatformImplementationPlugin\n'
            '        dartPluginClass: ApluginPlatformInterfaceMacOS\n',
      );
      pluginImplPubspecContent = pluginImplPubspecContent.replaceFirst(
        '    platforms:\n',
        '    implements: aplugin_platform_interface\n'
            '    platforms:\n',
      );
      await pluginImplPubspec.writeAsString(pluginImplPubspecContent, flush: true);

      section('Create interface plugin');
      await inDirectory(tempDir, () async {
        await flutter(
          'create',
          options: <String>[
            '--template=plugin',
            '--org',
            'io.flutter.devicelab',
            '--platforms',
            'macos',
            'aplugin_platform_interface',
          ],
          environment: environment,
        );
      });
      final File pluginInterfacePubspec = File(
        path.join(tempDir.absolute.path, 'aplugin_platform_interface', 'pubspec.yaml'),
      );
      String pluginInterfacePubspecContent = await pluginInterfacePubspec.readAsString();
      pluginInterfacePubspecContent = pluginInterfacePubspecContent.replaceFirst(
        '        pluginClass: ApluginPlatformInterfacePlugin',
        '        default_package: aplugin_platform_implementation\n',
      );
      pluginInterfacePubspecContent = pluginInterfacePubspecContent.replaceFirst(
        'dependencies:',
        'dependencies:\n'
            '  aplugin_platform_implementation:\n'
            '    path: ../aplugin_platform_implementation\n',
      );
      await pluginInterfacePubspec.writeAsString(pluginInterfacePubspecContent, flush: true);

      section('Create app');

      await inDirectory(tempDir, () async {
        await flutter(
          'create',
          options: <String>[
            '--template=app',
            '--org',
            'io.flutter.devicelab',
            '--platforms',
            'macos',
            'app',
          ],
          environment: environment,
        );
      });

      final File appPubspec = File(path.join(tempDir.absolute.path, 'app', 'pubspec.yaml'));
      String appPubspecContent = await appPubspec.readAsString();
      appPubspecContent = appPubspecContent.replaceFirst(
        'dependencies:',
        'dependencies:\n'
            '  aplugin_platform_interface:\n'
            '    path: ../aplugin_platform_interface\n',
      );
      await appPubspec.writeAsString(appPubspecContent, flush: true);

      section('Flutter run for macos');

      late Process run;
      await inDirectory(path.join(tempDir.path, 'app'), () async {
        run = await startFlutter('run', options: <String>['-d', 'macos', '-v']);
      });

      Completer<void> registryExecutedCompleter = Completer<void>();
      final StreamSubscription<void> stdoutSub = run.stdout
          .transform<String>(utf8.decoder)
          .transform<String>(const LineSplitter())
          .listen((String line) {
            if (line.contains('ApluginPlatformInterfaceMacOS.registerWith() was called')) {
              registryExecutedCompleter.complete();
            }
            print('stdout: $line');
          });

      final StreamSubscription<void> stderrSub = run.stderr
          .transform<String>(utf8.decoder)
          .transform<String>(const LineSplitter())
          .listen((String line) {
            print('stderr: $line');
          });

      final Future<void> stdoutDone = stdoutSub.asFuture<void>();
      final Future<void> stderrDone = stderrSub.asFuture<void>();

      Future<void> waitForStreams() {
        return Future.wait<void>(<Future<void>>[stdoutDone, stderrDone]);
      }

      Future<void> waitOrExit(Future<void> future) async {
        final dynamic result = await Future.any<dynamic>(<Future<dynamic>>[future, run.exitCode]);
        if (result is int) {
          await waitForStreams();
          throw 'process exited with code $result';
        }
      }

      section('Wait for registry execution');
      await waitOrExit(registryExecutedCompleter.future);

      // Hot restart.
      run.stdin.write('R');
      await run.stdin.flush();
      await run.stdin.close();

      registryExecutedCompleter = Completer<void>();
      section('Wait for registry execution after hot restart');
      await waitOrExit(registryExecutedCompleter.future);

      run.kill();

      section('Wait for stdout/stderr streams');
      await waitForStreams();

      unawaited(stdoutSub.cancel());
      unawaited(stderrSub.cancel());

      return TaskResult.success(null);
    } finally {
      rmTree(tempDir);
    }
  };
}
