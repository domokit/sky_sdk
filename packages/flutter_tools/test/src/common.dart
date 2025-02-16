// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:args/command_runner.dart';
import 'package:collection/collection.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/common.dart';
import 'package:flutter_tools/src/base/context.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:meta/meta.dart';
import 'package:path/path.dart' as path; // flutter_ignore: package_path_import
import 'package:test/test.dart' as test_package show test;
import 'package:test/test.dart' hide test;
import 'package:unified_analytics/unified_analytics.dart';

import 'fakes.dart';

export 'package:path/path.dart' show Context; // flutter_ignore: package_path_import
export 'package:test/test.dart' hide isInstanceOf, test;

void tryToDelete(FileSystemEntity fileEntity) {
  // This should not be necessary, but it turns out that
  // on Windows it's common for deletions to fail due to
  // bogus (we think) "access denied" errors.
  try {
    if (fileEntity.existsSync()) {
      fileEntity.deleteSync(recursive: true);
    }
  } on FileSystemException catch (error) {
    // We print this so that it's visible in the logs, to get an idea of how
    // common this problem is, and if any patterns are ever noticed by anyone.
    // ignore: avoid_print
    print('Failed to delete ${fileEntity.path}: $error');
  }
}

/// Gets the path to the root of the Flutter repository.
///
/// This will first look for a `FLUTTER_ROOT` environment variable. If the
/// environment variable is set, it will be returned. Otherwise, this will
/// deduce the path from `platform.script`.
String getFlutterRoot() {
  const Platform platform = LocalPlatform();
  if (platform.environment.containsKey('FLUTTER_ROOT')) {
    return platform.environment['FLUTTER_ROOT']!;
  }

  Error invalidScript() => StateError(
    'Could not determine flutter_tools/ path from script URL (${globals.platform.script}); consider setting FLUTTER_ROOT explicitly.',
  );

  Uri scriptUri;
  switch (platform.script.scheme) {
    case 'file':
      scriptUri = platform.script;
    case 'data':
      final RegExp flutterTools = RegExp(
        r'(file://[^"]*[/\\]flutter_tools[/\\][^"]+\.dart)',
        multiLine: true,
      );
      final Match? match = flutterTools.firstMatch(Uri.decodeFull(platform.script.path));
      if (match == null) {
        throw invalidScript();
      }
      scriptUri = Uri.parse(match.group(1)!);
    default:
      throw invalidScript();
  }

  final List<String> parts = path.split(globals.localFileSystem.path.fromUri(scriptUri));
  final int toolsIndex = parts.indexOf('flutter_tools');
  if (toolsIndex == -1) {
    throw invalidScript();
  }
  final String toolsPath = path.joinAll(parts.sublist(0, toolsIndex + 1));
  return path.normalize(path.join(toolsPath, '..', '..'));
}

/// Capture console print events into a string buffer.
Future<StringBuffer> capturedConsolePrint(Future<void> Function() body) async {
  final StringBuffer buffer = StringBuffer();
  await runZoned<Future<void>>(
    () async {
      // Service the event loop.
      await body();
    },
    zoneSpecification: ZoneSpecification(
      print: (Zone self, ZoneDelegate parent, Zone zone, String line) {
        buffer.writeln(line);
      },
    ),
  );
  return buffer;
}

/// Matcher for functions that throw [AssertionError].
final Matcher throwsAssertionError = throwsA(isA<AssertionError>());

/// Matcher for functions that throw [ToolExit].
///
/// [message] is matched using the [contains] matcher.
Matcher throwsToolExit({int? exitCode, Pattern? message}) {
  TypeMatcher<ToolExit> result = const TypeMatcher<ToolExit>();

  if (exitCode != null) {
    result = result.having((ToolExit e) => e.exitCode, 'exitCode', equals(exitCode));
  }
  if (message != null) {
    result = result.having((ToolExit e) => e.message, 'message', contains(message));
  }

  return throwsA(result);
}

/// Matcher for functions that throw [UsageException].
Matcher throwsUsageException({Pattern? message}) {
  Matcher matcher = _isUsageException;
  if (message != null) {
    matcher = allOf(matcher, (UsageException e) => e.message.contains(message));
  }
  return throwsA(matcher);
}

/// Matcher for [UsageException]s.
final TypeMatcher<UsageException> _isUsageException = isA<UsageException>();

/// Matcher for functions that throw [ProcessException].
Matcher throwsProcessException({Pattern? message}) {
  Matcher matcher = _isProcessException;
  if (message != null) {
    matcher = allOf(matcher, (ProcessException e) => e.message.contains(message));
  }
  return throwsA(matcher);
}

/// Matcher for [ProcessException]s.
final TypeMatcher<ProcessException> _isProcessException = isA<ProcessException>();

Future<void> expectToolExitLater(Future<dynamic> future, Matcher messageMatcher) async {
  try {
    await future;
    fail('ToolExit expected, but nothing thrown');
  } on ToolExit catch (e) {
    expect(e.message, messageMatcher);
    // Catch all exceptions to give a better test failure message.
  } catch (e, trace) {
    fail('ToolExit expected, got $e\n$trace');
  }
}

Future<void> expectReturnsNormallyLater(Future<dynamic> future) async {
  try {
    await future;
    // Catch all exceptions to give a better test failure message.
  } catch (e, trace) {
    fail('Expected to run with no exceptions, got $e\n$trace');
  }
}

Matcher containsIgnoringWhitespace(String toSearch) {
  return predicate((String source) {
    return collapseWhitespace(source).contains(collapseWhitespace(toSearch));
  }, 'contains "$toSearch" ignoring whitespace.');
}

/// The tool overrides `test` to ensure that files created under the
/// system temporary directory are deleted after each test by calling
/// `LocalFileSystem.dispose()`.
@isTest
void test(
  String description,
  FutureOr<void> Function() body, {
  String? testOn,
  dynamic skip,
  List<String>? tags,
  Map<String, dynamic>? onPlatform,
  int? retry,
}) {
  test_package.test(
    description,
    () async {
      addTearDown(() async {
        await globals.localFileSystem.dispose();
      });

      return body();
    },
    skip: skip,
    tags: tags,
    onPlatform: onPlatform,
    retry: retry,
    testOn: testOn,
    // We don't support "timeout"; see ../../dart_test.yaml which
    // configures all tests to have a 15 minute timeout which should
    // definitely be enough.
  );
}

/// Executes a test body in zone that does not allow context-based injection.
///
/// For classes which have been refactored to exclude context-based injection
/// or globals like [fs] or [platform], prefer using this test method as it
/// will prevent accidentally including these context getters in future code
/// changes.
///
/// For more information, see https://github.com/flutter/flutter/issues/47161
@isTest
void testWithoutContext(
  String description,
  FutureOr<void> Function() body, {
  String? testOn,
  dynamic skip,
  List<String>? tags,
  Map<String, dynamic>? onPlatform,
  int? retry,
}) {
  return test(
    description,
    () async {
      return runZoned(body, zoneValues: <Object, Object>{contextKey: const _NoContext()});
    },
    skip: skip,
    tags: tags,
    onPlatform: onPlatform,
    retry: retry,
    testOn: testOn,
    // We don't support "timeout"; see ../../dart_test.yaml which
    // configures all tests to have a 15 minute timeout which should
    // definitely be enough.
  );
}

/// An implementation of [AppContext] that throws if context.get is called in the test.
///
/// The intention of the class is to ensure we do not accidentally regress when
/// moving towards more explicit dependency injection by accidentally using
/// a Zone value in place of a constructor parameter.
class _NoContext implements AppContext {
  const _NoContext();

  @override
  T get<T>() {
    throw UnsupportedError(
      'context.get<$T> is not supported in test methods. '
      'Use Testbed or testUsingContext if accessing Zone injected '
      'values.',
    );
  }

  @override
  String get name => 'No Context';

  @override
  Future<V> run<V>({
    required FutureOr<V> Function() body,
    String? name,
    Map<Type, Generator>? overrides,
    Map<Type, Generator>? fallbacks,
    ZoneSpecification? zoneSpecification,
  }) async {
    return body();
  }
}

/// Allows inserting file system exceptions into certain
/// [MemoryFileSystem] operations by tagging path/op combinations.
///
/// Example use:
///
/// ```dart
/// void main() {
///   var handler = FileExceptionHandler();
///   var fs = MemoryFileSystem(opHandle: handler.opHandle);
///
///   var file = fs.file('foo')..createSync();
///   handler.addError(file, FileSystemOp.read, FileSystemException('Error Reading foo'));
///
///   expect(() => file.writeAsStringSync('A'), throwsA(isA<FileSystemException>()));
/// }
/// ```
class FileExceptionHandler {
  final Map<String, Map<FileSystemOp, FileSystemException>> _contextErrors =
      <String, Map<FileSystemOp, FileSystemException>>{};
  final Map<FileSystemOp, FileSystemException> _tempErrors = <FileSystemOp, FileSystemException>{};
  static final RegExp _tempDirectoryEnd = RegExp('rand[0-9]+');

  /// Add an exception that will be thrown whenever the file system attached to this
  /// handler performs the [operation] on the [entity].
  void addError(FileSystemEntity entity, FileSystemOp operation, FileSystemException exception) {
    final String path = entity.path;
    _contextErrors[path] ??= <FileSystemOp, FileSystemException>{};
    _contextErrors[path]![operation] = exception;
  }

  void addTempError(FileSystemOp operation, FileSystemException exception) {
    _tempErrors[operation] = exception;
  }

  /// Tear-off this method and pass it to the memory filesystem `opHandle` parameter.
  void opHandle(String path, FileSystemOp operation) {
    if (path.startsWith('.tmp_') || _tempDirectoryEnd.firstMatch(path) != null) {
      final FileSystemException? exception = _tempErrors[operation];
      if (exception != null) {
        throw exception;
      }
    }
    final Map<FileSystemOp, FileSystemException>? exceptions = _contextErrors[path];
    if (exceptions == null) {
      return;
    }
    final FileSystemException? exception = exceptions[operation];
    if (exception == null) {
      return;
    }
    throw exception;
  }
}

/// This method is required to fetch an instance of [FakeAnalytics]
/// because there is initialization logic that is required. An initial
/// instance will first be created and will let package:unified_analytics
/// know that the consent message has been shown. After confirming on the first
/// instance, then a second instance will be generated and returned. This second
/// instance will be cleared to send events.
FakeAnalytics getInitializedFakeAnalyticsInstance({
  required MemoryFileSystem fs,
  required FakeFlutterVersion fakeFlutterVersion,
  String? clientIde,
  String? enabledFeatures,
}) {
  final Directory homeDirectory = fs.directory('/');
  final FakeAnalytics initialAnalytics = Analytics.fake(
    tool: DashTool.flutterTool,
    homeDirectory: homeDirectory,
    dartVersion: fakeFlutterVersion.dartSdkVersion,
    fs: fs,
    flutterChannel: fakeFlutterVersion.channel,
    flutterVersion: fakeFlutterVersion.getVersionString(),
  );
  initialAnalytics.clientShowedMessage();

  return Analytics.fake(
    tool: DashTool.flutterTool,
    homeDirectory: homeDirectory,
    dartVersion: fakeFlutterVersion.dartSdkVersion,
    fs: fs,
    flutterChannel: fakeFlutterVersion.channel,
    flutterVersion: fakeFlutterVersion.getVersionString(),
    clientIde: clientIde,
    enabledFeatures: enabledFeatures,
  );
}

/// Returns "true" if the timing event searched for exists in [sentEvents].
///
/// This utility function allows us to check for an instance of
/// [Event.timing] within a [FakeAnalytics] instance. Normally, we can
/// use the equality operator for [Event] to check if the event exists, but
/// we are unable to do so for the timing event because the elapsed time
/// is variable so we cannot predict what that value will be in tests.
///
/// This function allows us to check for the other keys that have
/// string values by removing the `elapsedMilliseconds` from the
/// [Event.eventData] map and checking for a match.
bool analyticsTimingEventExists({
  required List<Event> sentEvents,
  required String workflow,
  required String variableName,
  String? label,
}) {
  final Map<String, String> lookup = <String, String>{
    'workflow': workflow,
    'variableName': variableName,
    if (label != null) 'label': label,
  };

  for (final Event e in sentEvents) {
    final Map<String, Object?> eventData = <String, Object?>{...e.eventData};
    eventData.remove('elapsedMilliseconds');

    if (const DeepCollectionEquality().equals(lookup, eventData)) {
      return true;
    }
  }

  return false;
}
