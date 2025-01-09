// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@Tags(<String>['flutter-test-driver'])
library;

import 'dart:async';

import 'package:file/file.dart';
import 'package:vm_service/vm_service.dart';

import '../src/common.dart';
import 'test_data/hot_reload_project.dart';
import 'test_driver.dart';
import 'test_utils.dart';

void main() {
  late Directory tempDir;
  final HotReloadProject project = HotReloadProject();
  late FlutterRunTestDriver flutter;

  setUp(() async {
    tempDir = createResolvedTempDirectorySync('hot_reload_test.');
    await project.setUpIn(tempDir);
    flutter = FlutterRunTestDriver(tempDir);
  });

  tearDown(() async {
    await flutter.stop();
    tryToDelete(tempDir);
  });

  testWithoutContext('hot reload works without error', () async {
    await flutter.run();
    await flutter.hotReload();
  });

  testWithoutContext('multiple overlapping hot reload are debounced and queued', () async {
    await flutter.run();
    // Capture how many *real* hot reloads occur.
    int numReloads = 0;
    final StreamSubscription<void> subscription = flutter.stdout
        .map(parseFlutterResponse)
        .where(_isHotReloadCompletionEvent)
        .listen((_) => numReloads++);

    // To reduce tests flaking, override the debounce timer to something higher than
    // the default to ensure the hot reloads that are supposed to arrive within the
    // debounce period will even on slower CI machines.
    const int hotReloadDebounceOverrideMs = 250;
    const Duration delay = Duration(milliseconds: hotReloadDebounceOverrideMs * 2);

    Future<void> doReload([void _]) =>
        flutter.hotReload(debounce: true, debounceDurationOverrideMs: hotReloadDebounceOverrideMs);

    try {
      await Future.wait<void>(<Future<void>>[
        doReload(),
        doReload(),
        Future<void>.delayed(delay).then(doReload),
        Future<void>.delayed(delay).then(doReload),
      ]);

      // We should only get two reloads, as the first two will have been
      // merged together by the debounce, and the second two also.
      expect(numReloads, equals(2));
    } finally {
      await subscription.cancel();
    }
  });

  testWithoutContext('newly added code executes during hot reload', () async {
    final StringBuffer stdout = StringBuffer();
    final Completer<void> completer = Completer<void>();
    final StreamSubscription<String> subscription = flutter.stdout.listen((String e) {
      stdout.writeln(e);
      // If hot reload properly executes newly added code, the 'RELOAD WORKED' message should
      // be printed before 'TICK 2'. If we don't wait for some signal that the build method
      // has executed after the reload, this test can encounter a race.
      if (e.contains('((((TICK 2))))')) {
        completer.complete();
      }
    });
    await flutter.run();
    project.uncommentHotReloadPrint();
    try {
      await flutter.hotReload();
      await completer.future;
      expect(stdout.toString(), contains('(((((RELOAD WORKED)))))'));
    } finally {
      await subscription.cancel();
    }
  });

  testWithoutContext('hot restart works without error', () async {
    await flutter.run(verbose: true);
    await flutter.hotRestart();
  });

  testWithoutContext('breakpoints are hit after hot reload', () async {
    Isolate isolate;
    final Completer<void> sawTick1 = Completer<void>();
    final Completer<void> sawDebuggerPausedMessage = Completer<void>();
    final StreamSubscription<String> subscription = flutter.stdout.listen((String line) {
      if (line.contains('((((TICK 1))))')) {
        expect(sawTick1.isCompleted, isFalse);
        sawTick1.complete();
      }
      if (line.contains('The application is paused in the debugger on a breakpoint.')) {
        expect(sawDebuggerPausedMessage.isCompleted, isFalse);
        sawDebuggerPausedMessage.complete();
      }
    });
    await flutter.run(withDebugger: true, startPaused: true);
    await flutter
        .resume(); // we start paused so we can set up our TICK 1 listener before the app starts
    unawaited(
      sawTick1.future.timeout(
        const Duration(seconds: 5),
        onTimeout: () {
          // This print is useful for people debugging this test. Normally we would avoid printing in
          // a test but this is an exception because it's useful ambient information.
          // ignore: avoid_print
          print('The test app is taking longer than expected to print its synchronization line...');
        },
      ),
    );
    printOnFailure('waiting for synchronization line...');
    await sawTick1.future; // after this, app is in steady state
    await flutter.addBreakpoint(project.scheduledBreakpointUri, project.scheduledBreakpointLine);
    await Future<void>.delayed(const Duration(seconds: 2));
    await flutter.hotReload(); // reload triggers code which eventually hits the breakpoint
    isolate = await flutter.waitForPause();
    expect(isolate.pauseEvent?.kind, equals(EventKind.kPauseBreakpoint));
    await flutter.resume();
    await flutter.addBreakpoint(project.buildBreakpointUri, project.buildBreakpointLine);
    bool reloaded = false;
    final Future<void> reloadFuture = flutter.hotReload().then((void value) {
      reloaded = true;
    });
    printOnFailure('waiting for pause...');
    isolate = await flutter.waitForPause();
    expect(isolate.pauseEvent?.kind, equals(EventKind.kPauseBreakpoint));
    printOnFailure('waiting for debugger message...');
    await sawDebuggerPausedMessage.future;
    expect(reloaded, isFalse);
    printOnFailure('waiting for resume...');
    await flutter.resume();
    printOnFailure('waiting for reload future...');
    await reloadFuture;
    expect(reloaded, isTrue);
    reloaded = false;
    printOnFailure('subscription cancel...');
    await subscription.cancel();
  });

  testWithoutContext("hot reload doesn't reassemble if paused", () async {
    final Completer<void> sawTick1 = Completer<void>();
    final Completer<void> sawDebuggerPausedMessage1 = Completer<void>();
    final Completer<void> sawDebuggerPausedMessage2 = Completer<void>();
    final StreamSubscription<String> subscription = flutter.stdout.listen((String line) {
      printOnFailure('[LOG]:"$line"');
      if (line.contains('(((TICK 1)))')) {
        expect(sawTick1.isCompleted, isFalse);
        sawTick1.complete();
      }
      if (line.contains('The application is paused in the debugger on a breakpoint.')) {
        expect(sawDebuggerPausedMessage1.isCompleted, isFalse);
        sawDebuggerPausedMessage1.complete();
      }
      if (line.contains(
        'The application is paused in the debugger on a breakpoint; interface might not update.',
      )) {
        expect(sawDebuggerPausedMessage2.isCompleted, isFalse);
        sawDebuggerPausedMessage2.complete();
      }
    });
    await flutter.run(withDebugger: true);
    await Future<void>.delayed(const Duration(seconds: 1));
    await sawTick1.future;
    await flutter.addBreakpoint(project.buildBreakpointUri, project.buildBreakpointLine);
    bool reloaded = false;
    await Future<void>.delayed(const Duration(seconds: 1));
    final Future<void> reloadFuture = flutter.hotReload().then((void value) {
      reloaded = true;
    });
    final Isolate isolate = await flutter.waitForPause();
    expect(isolate.pauseEvent?.kind, equals(EventKind.kPauseBreakpoint));
    expect(reloaded, isFalse);
    await sawDebuggerPausedMessage1
        .future; // this is the one where it say "uh, you broke into the debugger while reloading"
    await reloadFuture; // this is the one where it times out because you're in the debugger
    expect(reloaded, isTrue);
    await flutter.hotReload(); // now we're already paused
    await sawDebuggerPausedMessage2.future; // so we just get told that nothing is going to happen
    await flutter.resume();
    await subscription.cancel();
  });
}

bool _isHotReloadCompletionEvent(Map<String, Object?>? event) {
  return event != null &&
      event['event'] == 'app.progress' &&
      event['params'] != null &&
      (event['params']! as Map<String, Object?>)['progressId'] == 'hot.reload' &&
      (event['params']! as Map<String, Object?>)['finished'] == true;
}
