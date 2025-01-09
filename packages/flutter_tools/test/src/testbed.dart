// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io';

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/context.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/os.dart';
import 'package:flutter_tools/src/base/process.dart';
import 'package:flutter_tools/src/base/signals.dart';
import 'package:flutter_tools/src/base/terminal.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/context_runner.dart';
import 'package:flutter_tools/src/dart/pub.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/reporting/reporting.dart';
import 'package:flutter_tools/src/version.dart';
import 'package:unified_analytics/unified_analytics.dart';

import 'context.dart';
import 'fake_http_client.dart';
import 'fakes.dart';
import 'throwing_pub.dart';

export 'package:flutter_tools/src/base/context.dart' show Generator;

// A default value should be provided if the vast majority of tests should use
// this provider. For example, [BufferLogger], [MemoryFileSystem].
final Map<Type, Generator> _testbedDefaults = <Type, Generator>{
  // Keeps tests fast by avoiding the actual file system.
  FileSystem:
      () => MemoryFileSystem(
        style: globals.platform.isWindows ? FileSystemStyle.windows : FileSystemStyle.posix,
      ),
  ProcessManager: () => FakeProcessManager.any(),
  Logger:
      () => BufferLogger(
        terminal: AnsiTerminal(
          stdio: globals.stdio,
          platform: globals.platform,
        ), // Danger, using real stdio.
        outputPreferences: OutputPreferences.test(),
      ), // Allows reading logs and prevents stdout.
  OperatingSystemUtils: () => FakeOperatingSystemUtils(),
  OutputPreferences:
      () => OutputPreferences.test(), // configures BufferLogger to avoid color codes.
  Usage: () => TestUsage(), // prevent addition of analytics from burdening test mocks
  Analytics: () => const NoOpAnalytics(),
  FlutterVersion: () => FakeFlutterVersion(), // prevent requirement to mock git for test runner.
  Signals: () => FakeSignals(), // prevent registering actual signal handlers.
  Pub: () => const ThrowingPub(), // prevent accidental invocations of pub.
};

/// Manages interaction with the tool injection and runner system.
///
/// The Testbed automatically injects reasonable defaults through the context
/// DI system such as a [BufferLogger] and a [MemoryFileSystem].
///
/// Example:
///
/// Testing that a filesystem operation works as expected:
///
///     void main() {
///       group('Example', () {
///         Testbed testbed;
///
///         setUp(() {
///           testbed = Testbed(setUp: () {
///             globals.fs.file('foo').createSync()
///           });
///         })
///
///         test('Can delete a file', () => testbed.run(() {
///           expect(globals.fs.file('foo').existsSync(), true);
///           globals.fs.file('foo').deleteSync();
///           expect(globals.fs.file('foo').existsSync(), false);
///         }));
///       });
///     }
///
/// For a more detailed example, see the code in test_compiler_test.dart.
class Testbed {
  /// Creates a new [TestBed]
  ///
  /// `overrides` provides more overrides in addition to the test defaults.
  /// `setup` may be provided to apply mocks within the tool managed zone,
  /// including any specified overrides.
  Testbed({FutureOr<void> Function()? setup, Map<Type, Generator>? overrides})
    : _setup = setup,
      _overrides = overrides;

  final FutureOr<void> Function()? _setup;
  final Map<Type, Generator>? _overrides;

  /// Runs `test` within a tool zone.
  ///
  /// `overrides` may be used to provide new context values for the single test
  /// case or override any context values from the setup.
  Future<T?> run<T>(FutureOr<T> Function() test, {Map<Type, Generator>? overrides}) {
    final Map<Type, Generator> testOverrides = <Type, Generator>{
      ..._testbedDefaults,
      // Add the initial setUp overrides
      ...?_overrides,
      // Add the test-specific overrides
      ...?overrides,
    };
    if (testOverrides.containsKey(ProcessUtils)) {
      throw StateError('Do not inject ProcessUtils for testing, use ProcessManager instead.');
    }
    // Cache the original flutter root to restore after the test case.
    final String? originalFlutterRoot = Cache.flutterRoot;
    // Track pending timers to verify that they were correctly cleaned up.
    final Map<Timer, StackTrace> timers = <Timer, StackTrace>{};

    return HttpOverrides.runZoned(() {
      return runInContext<T?>(() {
        return context.run<T?>(
          name: 'testbed',
          overrides: testOverrides,
          zoneSpecification: ZoneSpecification(
            createTimer: (
              Zone self,
              ZoneDelegate parent,
              Zone zone,
              Duration duration,
              void Function() timer,
            ) {
              final Timer result = parent.createTimer(zone, duration, timer);
              timers[result] = StackTrace.current;
              return result;
            },
            createPeriodicTimer: (
              Zone self,
              ZoneDelegate parent,
              Zone zone,
              Duration period,
              void Function(Timer) timer,
            ) {
              final Timer result = parent.createPeriodicTimer(zone, period, timer);
              timers[result] = StackTrace.current;
              return result;
            },
          ),
          body: () async {
            Cache.flutterRoot = '';
            if (_setup != null) {
              await _setup.call();
            }
            await test();
            Cache.flutterRoot = originalFlutterRoot;
            for (final MapEntry<Timer, StackTrace> entry in timers.entries) {
              if (entry.key.isActive) {
                throw StateError('A Timer was active at the end of a test: ${entry.value}');
              }
            }
            return null;
          },
        );
      });
    }, createHttpClient: (SecurityContext? c) => FakeHttpClient.any());
  }
}
