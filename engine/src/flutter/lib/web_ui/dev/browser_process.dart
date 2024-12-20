// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:stack_trace/stack_trace.dart';
import 'package:typed_data/typed_buffers.dart';

class BrowserProcess {
  /// Creates a new browser.
  ///
  /// Clients pass in [startBrowser], which asynchronously returns the browser
  /// process. Any errors in [startBrowser] (even those raised asynchronously
  /// after it returns) are piped to [onExit] and will cause the browser to be
  /// killed.
  BrowserProcess(Future<Process> Function() startBrowser) {
    // Don't return a Future here because there's no need for the caller to wait
    // for the process to actually start. They should just wait for the HTTP
    // request instead.
    runZonedGuarded(
      () async {
        final Process process = await startBrowser();
        _processCompleter.complete(process);

        final Uint8Buffer output = Uint8Buffer();
        void drainOutput(Stream<List<int>> stream) {
          try {
            _ioSubscriptions.add(stream.listen(output.addAll, cancelOnError: true));
          } on StateError catch (_) {}
        }

        // If we don't drain the stdout and stderr the process can hang.
        drainOutput(process.stdout);
        drainOutput(process.stderr);

        final int exitCode = await process.exitCode;

        // This hack dodges an otherwise intractable race condition. When the user
        // presses Control-C, the signal is sent to the browser and the test
        // runner at the same time. It's possible for the browser to exit before
        // the [Browser.close] is called, which would trigger the error below.
        //
        // A negative exit code signals that the process exited due to a signal.
        // However, it's possible that this signal didn't come from the user's
        // Control-C, in which case we do want to throw the error. The only way to
        // resolve the ambiguity is to wait a brief amount of time and see if this
        // browser is actually closed.
        if (!_closed && exitCode < 0) {
          await Future<void>.delayed(const Duration(milliseconds: 200));
        }

        if (!_closed && exitCode != 0) {
          final String outputString = utf8.decode(output);
          String message = 'Browser process failed with exit code $exitCode.';
          if (outputString.isNotEmpty) {
            message += '\nStandard output:\n$outputString';
          }

          throw Exception(message);
        }

        _onExitCompleter.complete();
      },
      (dynamic error, StackTrace? stackTrace) {
        // Ignore any errors after the browser has been closed.
        if (_closed) {
          return;
        }

        // Make sure the process dies even if the error wasn't fatal.
        _process.then((Process process) => process.kill());

        stackTrace ??= Trace.current();

        if (_onExitCompleter.isCompleted) {
          return;
        }
        _onExitCompleter.completeError(
          Exception('Failed to run browser process: $error.'),
          stackTrace,
        );
      },
    );
  }

  /// The underlying process.
  ///
  /// This will fire once the process has started successfully.
  Future<Process> get _process => _processCompleter.future;
  final Completer<Process> _processCompleter = Completer<Process>();

  /// Whether [close] has been called.
  bool _closed = false;

  /// A future that completes when the browser exits.
  ///
  /// If there's a problem starting or running the browser, this will complete
  /// with an error.
  Future<void> get onExit => _onExitCompleter.future;
  final Completer<void> _onExitCompleter = Completer<void>();

  /// Standard IO streams for the underlying browser process.
  final List<StreamSubscription<void>> _ioSubscriptions = <StreamSubscription<void>>[];

  /// Kills the browser process.
  ///
  /// Returns the same [Future] as [onExit], except that it won't emit
  /// exceptions.
  Future<void> close() async {
    _closed = true;

    // If we don't manually close the stream the test runner can hang.
    // For example this happens with Chrome Headless.
    // See SDK issue: https://github.com/dart-lang/sdk/issues/31264
    for (final StreamSubscription<void> stream in _ioSubscriptions) {
      unawaited(stream.cancel());
    }

    (await _process).kill();

    // Swallow exceptions. The user should explicitly use [onExit] for these.
    return onExit.catchError((dynamic _) {});
  }
}
