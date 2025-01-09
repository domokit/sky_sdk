// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/debug_adapters/flutter_adapter_args.dart';
import 'package:flutter_tools/src/globals.dart' as globals show platform;
import 'package:test/test.dart';

import 'mocks.dart';

void main() {
  // Use the real platform as a base so that Windows bots test paths.
  final FakePlatform platform = FakePlatform.fromPlatform(globals.platform);
  final FileSystemStyle fsStyle =
      platform.isWindows ? FileSystemStyle.windows : FileSystemStyle.posix;

  group('flutter test adapter', () {
    final String expectedFlutterExecutable =
        platform.isWindows ? r'C:\fake\flutter\bin\flutter.bat' : '/fake/flutter/bin/flutter';

    setUpAll(() {
      Cache.flutterRoot = platform.isWindows ? r'C:\fake\flutter' : '/fake/flutter';
    });

    test('includes toolArgs', () async {
      final MockFlutterTestDebugAdapter adapter = MockFlutterTestDebugAdapter(
        fileSystem: MemoryFileSystem.test(style: fsStyle),
        platform: platform,
      );
      final Completer<void> responseCompleter = Completer<void>();
      final MockRequest request = MockRequest();
      final FlutterLaunchRequestArguments args = FlutterLaunchRequestArguments(
        cwd: '.',
        program: 'foo.dart',
        toolArgs: <String>['tool_arg'],
        noDebug: true,
      );

      await adapter.configurationDoneRequest(request, null, () {});
      await adapter.launchRequest(request, args, responseCompleter.complete);
      await responseCompleter.future;

      expect(adapter.executable, equals(expectedFlutterExecutable));
      expect(adapter.processArgs, contains('tool_arg'));
    });

    test('includes env variables', () async {
      final MockFlutterTestDebugAdapter adapter = MockFlutterTestDebugAdapter(
        fileSystem: MemoryFileSystem.test(style: fsStyle),
        platform: platform,
      );
      final Completer<void> responseCompleter = Completer<void>();
      final MockRequest request = MockRequest();
      final FlutterLaunchRequestArguments args = FlutterLaunchRequestArguments(
        cwd: '.',
        program: 'foo.dart',
        env: <String, String>{'MY_TEST_ENV': 'MY_TEST_VALUE'},
      );

      await adapter.configurationDoneRequest(request, null, () {});
      await adapter.launchRequest(request, args, responseCompleter.complete);
      await responseCompleter.future;

      expect(adapter.env!['MY_TEST_ENV'], 'MY_TEST_VALUE');
    });

    group('includes customTool', () {
      test('with no args replaced', () async {
        final MockFlutterTestDebugAdapter adapter = MockFlutterTestDebugAdapter(
          fileSystem: MemoryFileSystem.test(style: fsStyle),
          platform: platform,
        );
        final Completer<void> responseCompleter = Completer<void>();
        final MockRequest request = MockRequest();
        final FlutterLaunchRequestArguments args = FlutterLaunchRequestArguments(
          cwd: '.',
          program: 'foo.dart',
          customTool: '/custom/flutter',
          noDebug: true,
        );

        await adapter.configurationDoneRequest(request, null, () {});
        await adapter.launchRequest(request, args, responseCompleter.complete);
        await responseCompleter.future;

        expect(adapter.executable, equals('/custom/flutter'));
        // args should be in-tact
        expect(adapter.processArgs, contains('--machine'));
      });

      test('with all args replaced', () async {
        final MockFlutterTestDebugAdapter adapter = MockFlutterTestDebugAdapter(
          fileSystem: MemoryFileSystem.test(style: fsStyle),
          platform: platform,
        );
        final Completer<void> responseCompleter = Completer<void>();
        final MockRequest request = MockRequest();
        final FlutterLaunchRequestArguments args = FlutterLaunchRequestArguments(
          cwd: '.',
          program: 'foo.dart',
          customTool: '/custom/flutter',
          customToolReplacesArgs: 9999, // replaces all built-in args
          noDebug: true,
          toolArgs: <String>['tool_args'], // should still be in args
        );

        await adapter.configurationDoneRequest(request, null, () {});
        await adapter.launchRequest(request, args, responseCompleter.complete);
        await responseCompleter.future;

        expect(adapter.executable, equals('/custom/flutter'));
        // normal built-in args are replaced by customToolReplacesArgs, but
        // user-provided toolArgs are not.
        expect(adapter.processArgs, isNot(contains('--machine')));
        expect(adapter.processArgs, contains('tool_args'));
      });
    });
  });
}
