// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:dds/dap.dart';
import 'package:file/file.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/convert.dart';
import 'package:flutter_tools/src/globals.dart' as globals;

import '../../src/common.dart';
import '../test_data/basic_project.dart';
import '../test_data/compile_error_project.dart';
import '../test_data/project.dart';
import '../test_utils.dart';
import 'test_client.dart';
import 'test_server.dart';
import 'test_support.dart';

void main() {
  late Directory tempDir;
  late DapTestSession dap;
  final String relativeMainPath = 'lib${fileSystem.path.separator}main.dart';

  setUpAll(() {
    Cache.flutterRoot = getFlutterRoot();
  });

  setUp(() async {
    tempDir = createResolvedTempDirectorySync('flutter_adapter_test.');
    dap = await DapTestSession.setUp();
  });

  tearDown(() async {
    await dap.tearDown();
    tryToDelete(tempDir);
  });

  group('launch', () {
    testWithoutContext('can run and terminate a Flutter app in debug mode', () async {
      final BasicProject project = BasicProject();
      await project.setUpIn(tempDir);

      // Once the "topLevelFunction" output arrives, we can terminate the app.
      unawaited(
        dap.client.output
            .firstWhere((String output) => output.startsWith('topLevelFunction'))
            .whenComplete(() => dap.client.terminate()),
      );

      final List<OutputEventBody> outputEvents = await dap.client.collectAllOutput(
        launch:
            () => dap.client.launch(
              cwd: project.dir.path,
              toolArgs: <String>['-d', 'flutter-tester'],
            ),
      );

      final String output = _uniqueOutputLines(outputEvents);

      expectLines(output, <Object>[
        'Launching $relativeMainPath on Flutter test device in debug mode...',
        startsWith('Connecting to VM Service at'),
        'topLevelFunction',
        'Application finished.',
        '',
        startsWith('Exited'),
      ], allowExtras: true);
    });

    testWithoutContext('logs stdout to client when sendLogsToClient=true', () async {
      final BasicProject project = BasicProject();
      await project.setUpIn(tempDir);

      // Launch the app and wait for it to print "topLevelFunction".
      await Future.wait(<Future<void>>[
        dap.client.stdoutOutput.firstWhere(
          (String output) => output.startsWith('topLevelFunction'),
        ),
        dap.client.start(
          launch:
              () => dap.client.launch(
                cwd: project.dir.path,
                noDebug: true,
                toolArgs: <String>['-d', 'flutter-tester'],
                sendLogsToClient: true,
              ),
        ),
      ], eagerError: true);

      // Capture events while terminating.
      final Future<List<Event>> logEventsFuture = dap.client.events('dart.log').toList();
      await dap.client.terminate();

      // Ensure logs contain both the app.stop request and the result.
      final List<Event> logEvents = await logEventsFuture;
      final List<String> logMessages =
          logEvents
              .map((Event l) => (l.body! as Map<String, Object?>)['message']! as String)
              .toList();
      expect(
        logMessages,
        containsAll(<Matcher>[
          startsWith('==> [Flutter] [{"id":1,"method":"app.stop"'),
          startsWith('<== [Flutter] [{"id":1,"result":true}]'),
        ]),
      );
    });

    testWithoutContext('logs stderr to client when sendLogsToClient=true', () async {
      final BasicProject project = BasicProject();
      await project.setUpIn(tempDir);

      // Capture all log events.
      final Future<List<Event>> logEventsFuture = dap.client.events('dart.log').toList();

      // Launch the app and wait for it to terminate (because of the error).
      await Future.wait(<Future<void>>[
        dap.client.event('terminated'),
        dap.client.start(
          launch:
              () => dap.client.launch(
                cwd: project.dir.path,
                noDebug: true,
                toolArgs: <String>['--not-a-valid-flag'],
                sendLogsToClient: true,
              ),
        ),
      ], eagerError: true);

      // Ensure logs contain the expected error message.
      final List<Event> logEvents = await logEventsFuture;
      final List<String> logMessages =
          logEvents
              .map((Event l) => (l.body! as Map<String, Object?>)['message']! as String)
              .toList();
      expect(
        logMessages,
        contains(
          startsWith('<== [Flutter] [stderr] Could not find an option named "--not-a-valid-flag"'),
        ),
      );
    });

    testWithoutContext('can run and terminate a Flutter app in noDebug mode', () async {
      final BasicProject project = BasicProject();
      await project.setUpIn(tempDir);

      // Once the "topLevelFunction" output arrives, we can terminate the app.
      unawaited(
        dap.client.stdoutOutput
            .firstWhere((String output) => output.startsWith('topLevelFunction'))
            .whenComplete(() => dap.client.terminate()),
      );

      final List<OutputEventBody> outputEvents = await dap.client.collectAllOutput(
        launch:
            () => dap.client.launch(
              cwd: project.dir.path,
              noDebug: true,
              toolArgs: <String>['-d', 'flutter-tester'],
            ),
      );

      final String output = _uniqueOutputLines(outputEvents);

      expectLines(output, <Object>[
        'Launching $relativeMainPath on Flutter test device in debug mode...',
        'topLevelFunction',
        'Application finished.',
        '',
        startsWith('Exited'),
      ], allowExtras: true);

      // If we're running with an out-of-process debug adapter, ensure that its
      // own process shuts down after we terminated.
      final DapTestServer server = dap.server;
      if (server is OutOfProcessDapTestServer) {
        await server.exitCode;
      }
    });

    testWithoutContext('outputs useful message on invalid DAP protocol messages', () async {
      final OutOfProcessDapTestServer server = dap.server as OutOfProcessDapTestServer;
      final CompileErrorProject project = CompileErrorProject();
      await project.setUpIn(tempDir);

      final StringBuffer stderrOutput = StringBuffer();
      dap.server.onStderrOutput = stderrOutput.write;

      // Write invalid headers and await the error.
      dap.server.sink.add(utf8.encode('foo\r\nbar\r\n\r\n'));
      await server.exitCode;

      // Verify the user-friendly message was included in the output.
      final String error = stderrOutput.toString();
      expect(error, contains('Input could not be parsed as a Debug Adapter Protocol message'));
      expect(error, contains('The "flutter debug-adapter" command is intended for use by tooling'));
      // This test only runs with out-of-process DAP as it's testing _actual_
      // stderr output and that the debug-adapter process terminates, which is
      // not possible when running the DAP Server in-process.
    }, skip: useInProcessDap); // [intended] See above.

    testWithoutContext('correctly outputs launch errors and terminates', () async {
      final CompileErrorProject project = CompileErrorProject();
      await project.setUpIn(tempDir);

      final List<OutputEventBody> outputEvents = await dap.client.collectAllOutput(
        launch:
            () => dap.client.launch(
              cwd: project.dir.path,
              toolArgs: <String>['-d', 'flutter-tester'],
            ),
      );

      final String output = _uniqueOutputLines(outputEvents);
      expect(output, contains('this code does not compile'));
      expect(output, contains('Error: Failed to build'));
      expect(output, contains('Exited (1)'));
    });

    group('structured errors', () {
      /// Helper that runs [project] and collects the output.
      ///
      /// Line and column numbers are replaced with "1" to avoid fragile tests.
      Future<String> getExceptionOutput(
        Project project, {
        required bool noDebug,
        required bool ansiColors,
      }) async {
        await project.setUpIn(tempDir);

        final List<OutputEventBody> outputEvents = await dap.client.collectAllOutput(
          launch: () {
            // Terminate the app after we see the exception because otherwise
            // it will keep running and `collectAllOutput` won't end.
            dap.client.output
                .firstWhere((String output) => output.contains(endOfErrorOutputMarker))
                .then((_) => dap.client.terminate());
            return dap.client.launch(
              noDebug: noDebug,
              cwd: project.dir.path,
              toolArgs: <String>['-d', 'flutter-tester'],
              allowAnsiColorOutput: ansiColors,
            );
          },
        );

        String output = _uniqueOutputLines(outputEvents);

        // Replace out any line/columns to make tests less fragile.
        output = output.replaceAll(RegExp(r'\.dart:\d+:\d+'), '.dart:1:1');

        return output;
      }

      testWithoutContext('correctly outputs exceptions in debug mode', () async {
        final BasicProjectThatThrows project = BasicProjectThatThrows();
        final String output = await getExceptionOutput(project, noDebug: false, ansiColors: false);

        expect(
          output,
          contains('''
════════ Exception caught by widgets library ═══════════════════════════════════
The following _Exception was thrown building App(dirty):
Exception: c

The relevant error-causing widget was:
    App App:${Uri.file(project.dir.path)}/lib/main.dart:1:1'''),
        );
      });

      testWithoutContext('correctly outputs colored exceptions when supported', () async {
        final BasicProjectThatThrows project = BasicProjectThatThrows();
        final String output = await getExceptionOutput(project, noDebug: false, ansiColors: true);

        // Frames in the stack trace that are the users own code will be unformatted, but
        // frames from the framework are faint (starting with `\x1B[2m`).

        expect(
          output,
          contains('''
════════ Exception caught by widgets library ═══════════════════════════════════
The following _Exception was thrown building App(dirty):
Exception: c

The relevant error-causing widget was:
    App App:${Uri.file(project.dir.path)}/lib/main.dart:1:1

When the exception was thrown, this was the stack:
#0      c (package:test/main.dart:1:1)
          ^ source: package:test/main.dart
#1      App.build (package:test/main.dart:1:1)
          ^ source: package:test/main.dart
\x1B[2m#2      StatelessElement.build (package:flutter/src/widgets/framework.dart:1:1)\x1B[0m
          ^ source: package:flutter/src/widgets/framework.dart
\x1B[2m#3      ComponentElement.performRebuild (package:flutter/src/widgets/framework.dart:1:1)\x1B[0m
          ^ source: package:flutter/src/widgets/framework.dart'''),
        );
      });

      testWithoutContext('correctly outputs exceptions in noDebug mode', () async {
        final BasicProjectThatThrows project = BasicProjectThatThrows();
        final String output = await getExceptionOutput(project, noDebug: true, ansiColors: false);

        // When running in noDebug mode, we don't get the Flutter.Error event so
        // we get the basic Flutter-formatted version of the error.
        expect(
          output,
          contains('''
══╡ EXCEPTION CAUGHT BY WIDGETS LIBRARY ╞═══════════════════════════════════════════════════════════
The following _Exception was thrown building App(dirty):
Exception: c

The relevant error-causing widget was:
  App'''),
        );
        expect(output, contains('App:${Uri.file(project.dir.path)}/lib/main.dart:1:1'));
      });
    });

    testWithoutContext('can hot reload', () async {
      final BasicProject project = BasicProject();
      await project.setUpIn(tempDir);

      // Launch the app and wait for it to print "topLevelFunction".
      await Future.wait(<Future<void>>[
        dap.client.stdoutOutput.firstWhere(
          (String output) => output.startsWith('topLevelFunction'),
        ),
        dap.client.start(
          launch:
              () => dap.client.launch(
                cwd: project.dir.path,
                noDebug: true,
                toolArgs: <String>['-d', 'flutter-tester'],
              ),
        ),
      ], eagerError: true);

      // Capture the next two output events that we expect to be the Reload
      // notification and then topLevelFunction being printed again.
      final Future<List<String>> outputEventsFuture =
          dap.client.stdoutOutput
              // But skip any topLevelFunctions that come before the reload.
              .skipWhile((String output) => output.startsWith('topLevelFunction'))
              .take(2)
              .toList();

      await dap.client.hotReload();

      expectLines((await outputEventsFuture).join(), <Object>[
        startsWith('Reloaded'),
        'topLevelFunction',
      ], allowExtras: true);

      // Repeat the test for hot reload with custom syntax.
      final Future<List<String>> customOutputEventsFuture =
          dap.client.stdoutOutput
              // But skip any topLevelFunctions that come before the reload.
              .skipWhile((String output) => output.startsWith('topLevelFunction'))
              .take(2)
              .toList();

      await dap.client.customSyntaxHotReload();

      expectLines((await customOutputEventsFuture).join(), <Object>[
        startsWith('Reloaded'),
        'topLevelFunction',
      ], allowExtras: true);

      await dap.client.terminate();
    });

    testWithoutContext('sends progress notifications during hot reload', () async {
      final BasicProject project = BasicProject();
      await project.setUpIn(tempDir);

      // Launch the app and wait for it to print "topLevelFunction".
      await Future.wait(<Future<void>>[
        dap.client.stdoutOutput.firstWhere(
          (String output) => output.startsWith('topLevelFunction'),
        ),
        dap.client.initialize(supportsProgressReporting: true),
        dap.client.launch(
          cwd: project.dir.path,
          noDebug: true,
          toolArgs: <String>['-d', 'flutter-tester'],
        ),
      ], eagerError: true);

      // Capture progress events during a reload.
      final Future<List<Event>> progressEventsFuture = dap.client.progressEvents().toList();
      await dap.client.hotReload();
      await dap.client.terminate();

      // Verify the progress events.
      final List<Event> progressEvents = await progressEventsFuture;
      expect(progressEvents, hasLength(2));

      final List<String> eventKinds = progressEvents.map((Event event) => event.event).toList();
      expect(eventKinds, <String>['progressStart', 'progressEnd']);

      final List<Map<String, Object?>> eventBodies =
          progressEvents.map((Event event) => event.body).cast<Map<String, Object?>>().toList();
      final ProgressStartEventBody start = ProgressStartEventBody.fromMap(eventBodies[0]);
      final ProgressEndEventBody end = ProgressEndEventBody.fromMap(eventBodies[1]);
      expect(start.progressId, isNotNull);
      expect(start.title, 'Flutter');
      expect(start.message, 'Hot reloading…');
      expect(end.progressId, start.progressId);
      expect(end.message, isNull);
    });

    testWithoutContext('can hot restart', () async {
      final BasicProject project = BasicProject();
      await project.setUpIn(tempDir);

      // Launch the app and wait for it to print "topLevelFunction".
      await Future.wait(<Future<void>>[
        dap.client.stdoutOutput.firstWhere(
          (String output) => output.startsWith('topLevelFunction'),
        ),
        dap.client.start(
          launch:
              () => dap.client.launch(
                cwd: project.dir.path,
                noDebug: true,
                toolArgs: <String>['-d', 'flutter-tester'],
              ),
        ),
      ], eagerError: true);

      // Capture the next two output events that we expect to be the Restart
      // notification and then topLevelFunction being printed again.
      final Future<List<String>> outputEventsFuture =
          dap.client.stdoutOutput
              // But skip any topLevelFunctions that come before the restart.
              .skipWhile((String output) => output.startsWith('topLevelFunction'))
              .take(2)
              .toList();

      await dap.client.hotRestart();

      expectLines((await outputEventsFuture).join(), <Object>[
        startsWith('Restarted application'),
        'topLevelFunction',
      ], allowExtras: true);

      await dap.client.terminate();
    });

    testWithoutContext('sends progress notifications during hot restart', () async {
      final BasicProject project = BasicProject();
      await project.setUpIn(tempDir);

      // Launch the app and wait for it to print "topLevelFunction".
      await Future.wait(<Future<void>>[
        dap.client.stdoutOutput.firstWhere(
          (String output) => output.startsWith('topLevelFunction'),
        ),
        dap.client.initialize(supportsProgressReporting: true),
        dap.client.launch(
          cwd: project.dir.path,
          noDebug: true,
          toolArgs: <String>['-d', 'flutter-tester'],
        ),
      ], eagerError: true);

      // Capture progress events during a restart.
      final Future<List<Event>> progressEventsFuture = dap.client.progressEvents().toList();
      await dap.client.hotRestart();
      await dap.client.terminate();

      // Verify the progress events.
      final List<Event> progressEvents = await progressEventsFuture;
      expect(progressEvents, hasLength(2));

      final List<String> eventKinds = progressEvents.map((Event event) => event.event).toList();
      expect(eventKinds, <String>['progressStart', 'progressEnd']);

      final List<Map<String, Object?>> eventBodies =
          progressEvents.map((Event event) => event.body).cast<Map<String, Object?>>().toList();
      final ProgressStartEventBody start = ProgressStartEventBody.fromMap(eventBodies[0]);
      final ProgressEndEventBody end = ProgressEndEventBody.fromMap(eventBodies[1]);
      expect(start.progressId, isNotNull);
      expect(start.title, 'Flutter');
      expect(start.message, 'Hot restarting…');
      expect(end.progressId, start.progressId);
      expect(end.message, isNull);
    });

    testWithoutContext('can hot restart when exceptions occur on outgoing isolates', () async {
      final BasicProjectThatThrows project = BasicProjectThatThrows();
      await project.setUpIn(tempDir);

      // Launch the app and wait for it to stop at an exception.
      late int originalThreadId, newThreadId;
      await Future.wait(<Future<void>>[
        // Capture the thread ID of thread when it stops on the exception
        // (ignoring the stop on entry that occurs during thread start).
        dap.client.stoppedEvents
            .where((StoppedEventBody event) => event.reason == 'exception')
            .first
            .then((StoppedEventBody event) => originalThreadId = event.threadId!),
        dap.client.start(
          exceptionPauseMode: 'All', // Ensure we stop on all exceptions
          launch:
              () => dap.client.launch(
                cwd: project.dir.path,
                toolArgs: <String>['-d', 'flutter-tester'],
              ),
        ),
      ], eagerError: true);

      // Hot restart, ensuring it completes and capturing the ID of the new thread
      // to pause.
      await Future.wait(<Future<void>>[
        // Capture the thread ID of the next stop on exception (ignoring any
        // stop on exit/entry that occurs during thread start/exit).
        dap.client.stoppedEvents
            .where((StoppedEventBody event) => event.reason == 'exception')
            .first
            .then((StoppedEventBody event) => newThreadId = event.threadId!),
        dap.client.hotRestart(),
      ], eagerError: true);

      // We should not have stopped on the original thread, but the new thread
      // from after the restart.
      expect(newThreadId, isNot(equals(originalThreadId)));

      await dap.client.terminate();
    });

    testWithoutContext('sends events for extension state updates', () async {
      final BasicProject project = BasicProject();
      await project.setUpIn(tempDir);
      const String debugPaintRpc = 'ext.flutter.debugPaint';

      // Create a future to capture the isolate ID when the debug paint service
      // extension loads, as we'll need that to call it later.
      final Future<String> isolateIdForDebugPaint = dap.client
          .serviceExtensionAdded(debugPaintRpc)
          .then((Map<String, Object?> body) => body['isolateId']! as String);

      // Launch the app and wait for it to print "topLevelFunction" so we know
      // it's up and running.
      await Future.wait(<Future<void>>[
        dap.client.stdoutOutput.firstWhere(
          (String output) => output.startsWith('topLevelFunction'),
        ),
        dap.client.start(
          launch:
              () => dap.client.launch(
                cwd: project.dir.path,
                toolArgs: <String>['-d', 'flutter-tester'],
              ),
        ),
      ], eagerError: true);

      // Capture the next relevant state-change event (which should occur as a
      // result of the call below).
      final Future<Map<String, Object?>> stateChangeEventFuture = dap.client
          .serviceExtensionStateChanged(debugPaintRpc);

      // Enable debug paint to trigger the state change.
      await dap.client.custom('callService', <String, Object?>{
        'method': debugPaintRpc,
        'params': <String, Object?>{'enabled': true, 'isolateId': await isolateIdForDebugPaint},
      });

      // Ensure the event occurred, and its value was as expected.
      final Map<String, Object?> stateChangeEvent = await stateChangeEventFuture;
      expect(stateChangeEvent['value'], 'true'); // extension state change values are always strings

      await dap.client.terminate();
    });

    testWithoutContext('provides appStarted events to the client', () async {
      final BasicProject project = BasicProject();
      await project.setUpIn(tempDir);

      // Launch the app and wait for it to send a 'flutter.appStart' event.
      final Future<Event> appStartFuture = dap.client.event('flutter.appStart');
      await Future.wait(<Future<void>>[
        appStartFuture,
        dap.client.start(
          launch:
              () => dap.client.launch(
                cwd: project.dir.path,
                toolArgs: <String>['-d', 'flutter-tester'],
              ),
        ),
      ], eagerError: true);

      await dap.client.terminate();

      final Event appStart = await appStartFuture;
      final Map<String, Object?> params = appStart.body! as Map<String, Object?>;
      expect(params['deviceId'], 'flutter-tester');
      expect(params['mode'], 'debug');
    });

    testWithoutContext('provides appStarted events to the client', () async {
      final BasicProject project = BasicProject();
      await project.setUpIn(tempDir);

      // Launch the app and wait for it to send a 'flutter.appStarted' event.
      await Future.wait(<Future<void>>[
        dap.client.event('flutter.appStarted'),
        dap.client.start(
          launch:
              () => dap.client.launch(
                cwd: project.dir.path,
                toolArgs: <String>['-d', 'flutter-tester'],
              ),
        ),
      ], eagerError: true);

      await dap.client.terminate();
    });

    group('can step', () {
      test('into SDK sources mapped to local files when debugSdkLibraries=true', () async {
        final BasicProject project = BasicProject();
        await project.setUpIn(tempDir);

        final String breakpointFilePath = globals.fs.path.join(
          project.dir.path,
          'lib',
          'main.dart',
        );
        final int breakpointLine = project.topLevelFunctionBreakpointLine;
        final String expectedPrintLibraryPath = globals.fs.path.join(
          'pkg',
          'sky_engine',
          'lib',
          'core',
          'print.dart',
        );

        // Launch the app and wait for it to print "topLevelFunction".
        await Future.wait(<Future<void>>[
          dap.client.stdoutOutput.firstWhere(
            (String output) => output.startsWith('topLevelFunction'),
          ),
          dap.client.start(
            launch:
                () => dap.client.launch(
                  cwd: project.dir.path,
                  debugSdkLibraries: true,
                  toolArgs: <String>['-d', 'flutter-tester'],
                ),
          ),
        ], eagerError: true);

        // Add a breakpoint to the `print()` line and hit it.
        unawaited(dap.client.setBreakpoint(breakpointFilePath, breakpointLine));
        int stoppedThreadId =
            (await dap.client.stoppedEvents.firstWhere(
              (StoppedEventBody e) => e.reason == 'breakpoint',
            )).threadId!;

        // Step into `print()` and wait for the next stop.
        unawaited(dap.client.stepIn(stoppedThreadId));
        stoppedThreadId = (await dap.client.stoppedEvents.first).threadId!;

        // Fetch the top stack frame and ensure it's been mapped to a local file
        // correctly.
        final StackFrame topFrame =
            (await dap.client.getValidStack(
              stoppedThreadId,
              startFrame: 0,
              numFrames: 1,
            )).stackFrames.single;
        expect(topFrame.source!.name, 'dart:core/print.dart');
        // We should have a resolved path ending with the path to the print library.
        expect(topFrame.source!.path, endsWith(expectedPrintLibraryPath));

        await dap.client.terminate();
      });
    });
  });

  group('attach', () {
    late SimpleFlutterRunner testProcess;
    late BasicProject project;
    late String breakpointFilePath;
    late int breakpointLine;
    setUp(() async {
      project = BasicProject();
      await project.setUpIn(tempDir);
      testProcess = await SimpleFlutterRunner.start(tempDir);

      breakpointFilePath = globals.fs.path.join(project.dir.path, 'lib', 'main.dart');
      breakpointLine = project.buildMethodBreakpointLine;
    });

    tearDown(() async {
      testProcess.process.kill();
      await testProcess.process.exitCode;
    });

    testWithoutContext('can attach to an already-running Flutter app and reload', () async {
      final Uri vmServiceUri = await testProcess.vmServiceUri;

      // Launch the app and wait for it to print "topLevelFunction".
      await Future.wait(<Future<void>>[
        dap.client.stdoutOutput.firstWhere(
          (String output) => output.startsWith('topLevelFunction'),
        ),
        dap.client.start(
          launch:
              () => dap.client.attach(
                cwd: project.dir.path,
                toolArgs: <String>['-d', 'flutter-tester'],
                vmServiceUri: vmServiceUri.toString(),
              ),
        ),
      ], eagerError: true);

      // Capture the "Reloaded" output and events immediately after.
      final Future<List<String>> outputEventsFuture =
          dap.client.stdoutOutput
              .skipWhile((String output) => !output.startsWith('Reloaded'))
              .take(4)
              .toList();

      // Perform the reload, and expect we get the Reloaded output followed
      // by printed output, to ensure the app is running again.
      await dap.client.hotReload();
      expectLines((await outputEventsFuture).join(), <Object>[
        startsWith('Reloaded'),
        'topLevelFunction',
      ], allowExtras: true);

      await dap.client.terminate();
    });

    testWithoutContext(
      'can attach to an already-running Flutter app and hit breakpoints',
      () async {
        final Uri vmServiceUri = await testProcess.vmServiceUri;

        // Launch the app and wait for it to print "topLevelFunction".
        await Future.wait(<Future<void>>[
          dap.client.stdoutOutput.firstWhere(
            (String output) => output.startsWith('topLevelFunction'),
          ),
          dap.client.start(
            launch:
                () => dap.client.attach(
                  cwd: project.dir.path,
                  toolArgs: <String>['-d', 'flutter-tester'],
                  vmServiceUri: vmServiceUri.toString(),
                ),
          ),
        ], eagerError: true);

        // Set a breakpoint and expect to hit it.
        final Future<StoppedEventBody> stoppedFuture = dap.client.stoppedEvents.firstWhere(
          (StoppedEventBody e) => e.reason == 'breakpoint',
        );
        await Future.wait(<Future<void>>[
          stoppedFuture,
          dap.client.setBreakpoint(breakpointFilePath, breakpointLine),
        ], eagerError: true);
      },
    );

    testWithoutContext('resumes and removes breakpoints on detach', () async {
      final Uri vmServiceUri = await testProcess.vmServiceUri;

      // Launch the app and wait for it to print "topLevelFunction".
      await Future.wait(<Future<void>>[
        dap.client.stdoutOutput.firstWhere(
          (String output) => output.startsWith('topLevelFunction'),
        ),
        dap.client.start(
          launch:
              () => dap.client.attach(
                cwd: project.dir.path,
                toolArgs: <String>['-d', 'flutter-tester'],
                vmServiceUri: vmServiceUri.toString(),
              ),
        ),
      ], eagerError: true);

      // Set a breakpoint and expect to hit it.
      final Future<StoppedEventBody> stoppedFuture = dap.client.stoppedEvents.firstWhere(
        (StoppedEventBody e) => e.reason == 'breakpoint',
      );
      await Future.wait(<Future<void>>[
        stoppedFuture,
        dap.client.setBreakpoint(breakpointFilePath, breakpointLine),
      ], eagerError: true);

      // Detach and expected resume and correct output.
      await Future.wait(<Future<void>>[
        // We should print "Detached" instead of "Exited".
        dap.client.outputEvents.firstWhere(
          (OutputEventBody event) => event.output.contains('\nDetached'),
        ),
        // We should still get terminatedEvent (this signals the DAP server terminating).
        dap.client.event('terminated'),
        // We should get output showing the app resumed.
        testProcess.output.firstWhere((String output) => output.contains('topLevelFunction')),
        // Trigger the detach.
        dap.client.terminate(),
      ]);
    });
  });
}

/// Extracts the output from a set of [OutputEventBody], removing any
/// adjacent duplicates and combining into a single string.
///
/// If the output event contains a [Source], the name will be shown on the
/// following line indented and prefixed with `^ source:`.
String _uniqueOutputLines(List<OutputEventBody> outputEvents) {
  String? lastItem;
  return outputEvents
      .map((OutputEventBody e) {
        final String output = e.output;
        final Source? source = e.source;
        return source != null ? '$output          ^ source: ${source.name}\n' : output;
      })
      .where((String output) {
        // Skip the item if it's the same as the previous one.
        final bool isDupe = output == lastItem;
        lastItem = output;
        return !isDupe;
      })
      .join();
}
