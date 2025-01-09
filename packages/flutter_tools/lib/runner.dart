// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:args/command_runner.dart';
import 'package:intl/intl.dart' as intl;
import 'package:intl/intl_standalone.dart' as intl_standalone;
import 'package:process/process.dart';
import 'package:unified_analytics/unified_analytics.dart';

import 'src/base/async_guard.dart';
import 'src/base/common.dart';
import 'src/base/context.dart';
import 'src/base/error_handling_io.dart';
import 'src/base/file_system.dart';
import 'src/base/io.dart';
import 'src/base/logger.dart';
import 'src/base/process.dart';
import 'src/context_runner.dart';
import 'src/doctor.dart';
import 'src/features.dart';
import 'src/globals.dart' as globals;
import 'src/reporting/crash_reporting.dart';
import 'src/runner/flutter_command.dart';
import 'src/runner/flutter_command_runner.dart';

/// Runs the Flutter tool with support for the specified list of [commands].
Future<int> run(
  List<String> args,
  List<FlutterCommand> Function() commands, {
  bool muteCommandLogging = false,
  bool verbose = false,
  bool verboseHelp = false,
  bool? reportCrashes,
  String? flutterVersion,
  Map<Type, Generator>? overrides,
  required ShutdownHooks shutdownHooks,
}) async {
  if (muteCommandLogging) {
    // Remove the verbose option; for help and doctor, users don't need to see
    // verbose logs.
    args = List<String>.of(args);
    args.removeWhere((String option) => option == '-vv' || option == '-v' || option == '--verbose');
  }

  return runInContext<int>(() async {
    globals.terminal.applyFeatureFlags(featureFlags);

    reportCrashes ??= !await globals.isRunningOnBot;
    final FlutterCommandRunner runner = FlutterCommandRunner(verboseHelp: verboseHelp);
    commands().forEach(runner.addCommand);

    // Initialize the system locale.
    final String systemLocale = await intl_standalone.findSystemLocale();
    intl.Intl.defaultLocale = intl.Intl.verifiedLocale(
      systemLocale,
      intl.NumberFormat.localeExists,
      onFailure: (String _) => 'en_US',
    );

    String getVersion() =>
        flutterVersion ?? globals.flutterVersion.getVersionString(redactUnknownBranches: true);
    Object? firstError;
    StackTrace? firstStackTrace;
    return runZoned<Future<int>>(
      () async {
        try {
          if (args.contains('--disable-analytics') && args.contains('--enable-analytics')) {
            throwToolExit(
              'Both enable and disable analytics commands were detected '
              'when only one can be supplied per invocation.',
              exitCode: 1,
            );
          }

          if (args.contains('--disable-analytics')) {
            if (globals.analytics.telemetryEnabled) {
              globals.analytics.send(Event.analyticsCollectionEnabled(status: false));
              // Before disablig analytics, we need to close the client to make
              // sure the above collection event is sent.
              await globals.analytics.close();
            }
            await globals.analytics.setTelemetry(false);
            globals.printStatus('Analytics reporting disabled.');
          }

          if (args.contains('--enable-analytics')) {
            final bool alreadyEnabled = globals.analytics.telemetryEnabled;
            await globals.analytics.setTelemetry(true);
            if (!alreadyEnabled) {
              globals.analytics.send(Event.analyticsCollectionEnabled(status: true));
            }
            globals.printStatus('Analytics reporting enabled.');
          }

          await runner.run(args);

          // Triggering [runZoned]'s error callback does not necessarily mean that
          // we stopped executing the body. See https://github.com/dart-lang/sdk/issues/42150.
          if (firstError == null) {
            return await exitWithHooks(0, shutdownHooks: shutdownHooks);
          }

          // We already hit some error, so don't return success. The error path
          // (which should be in progress) is responsible for calling _exit().
          return 1;
          // This catches all exceptions to send to crash logging, etc.
          // ignore: avoid_catches_without_on_clauses
        } catch (error, stackTrace) {
          firstError = error;
          firstStackTrace = stackTrace;
          return _handleToolError(
            error,
            stackTrace,
            verbose,
            args,
            reportCrashes!,
            getVersion,
            shutdownHooks,
          );
        }
      },
      onError: (Object error, StackTrace stackTrace) async {
        // If sending a crash report throws an error into the zone, we don't want
        // to re-try sending the crash report with *that* error. Rather, we want
        // to send the original error that triggered the crash report.
        firstError ??= error;
        firstStackTrace ??= stackTrace;
        await _handleToolError(
          firstError!,
          firstStackTrace,
          verbose,
          args,
          reportCrashes!,
          getVersion,
          shutdownHooks,
        );
      },
    );
  }, overrides: overrides);
}

Future<int> _handleToolError(
  Object error,
  StackTrace? stackTrace,
  bool verbose,
  List<String> args,
  bool reportCrashes,
  String Function() getFlutterVersion,
  ShutdownHooks shutdownHooks,
) async {
  if (error is UsageException) {
    globals.printError('${error.message}\n');
    globals.printError(
      "Run 'flutter -h' (or 'flutter <command> -h') for available flutter commands and options.",
    );
    // Argument error exit code.
    return exitWithHooks(64, shutdownHooks: shutdownHooks);
  } else if (error is ToolExit) {
    if (error.message != null) {
      globals.printError(error.message!);
    }
    if (verbose) {
      globals.printError('\n$stackTrace\n');
    }
    return exitWithHooks(error.exitCode ?? 1, shutdownHooks: shutdownHooks);
  } else if (error is ProcessExit) {
    // We've caught an exit code.
    if (error.immediate) {
      exit(error.exitCode);
      return error.exitCode;
    } else {
      return exitWithHooks(error.exitCode, shutdownHooks: shutdownHooks);
    }
  } else if (error is ProcessException &&
      _isErrorDueToGitMissing(error, globals.processManager, globals.logger)) {
    globals.printError('${error.message}\n');
    globals.printError(
      'An error was encountered when trying to run git.\n'
      "Please ensure git is installed and available in your system's search path. "
      'See https://docs.flutter.dev/get-started/install for instructions on '
      'installing git for your platform.',
    );
    return exitWithHooks(1, shutdownHooks: shutdownHooks);
  } else {
    // We've crashed; emit a log report.
    globals.stdio.stderrWrite('\n');

    if (!reportCrashes) {
      // Print the stack trace on the bots - don't write a crash report.
      globals.stdio.stderrWrite('$error\n');
      globals.stdio.stderrWrite('$stackTrace\n');
      return exitWithHooks(1, shutdownHooks: shutdownHooks);
    }

    globals.analytics.send(Event.exception(exception: error.runtimeType.toString()));
    await asyncGuard(
      () async {
        final CrashReportSender crashReportSender = CrashReportSender(
          platform: globals.platform,
          logger: globals.logger,
          operatingSystemUtils: globals.os,
          analytics: globals.analytics,
        );
        await crashReportSender.sendReport(
          error: error,
          stackTrace: stackTrace!,
          getFlutterVersion: getFlutterVersion,
          command: args.join(' '),
        );
      },
      onError: (dynamic error) {
        globals.printError('Error sending crash report: $error');
      },
    );

    globals.printError('Oops; flutter has exited unexpectedly: "$error".');

    try {
      final BufferLogger logger = BufferLogger(
        terminal: globals.terminal,
        outputPreferences: globals.outputPreferences,
      );

      final DoctorText doctorText = DoctorText(logger);

      final CrashDetails details = CrashDetails(
        command: _crashCommand(args),
        error: error,
        stackTrace: stackTrace!,
        doctorText: doctorText,
      );
      final File file = await _createLocalCrashReport(details);
      await globals.crashReporter!.informUser(details, file);

      return exitWithHooks(1, shutdownHooks: shutdownHooks);
      // This catch catches all exceptions to ensure the message below is printed.
      // ignore: avoid_catches_without_on_clauses
    } catch (error, st) {
      globals.stdio.stderrWrite(
        'Unable to generate crash report due to secondary error: $error\n$st\n'
        '${globals.userMessages.flutterToolBugInstructions}\n',
      );
      // Any exception thrown here (including one thrown by `_exit()`) will
      // get caught by our zone's `onError` handler. In order to avoid an
      // infinite error loop, we throw an error that is recognized above
      // and will trigger an immediate exit.
      throw ProcessExit(1, immediate: true);
    }
  }
}

String _crashCommand(List<String> args) => 'flutter ${args.join(' ')}';

String _crashException(dynamic error) => '${error.runtimeType}: $error';

/// Saves the crash report to a local file.
Future<File> _createLocalCrashReport(CrashDetails details) async {
  final StringBuffer buffer = StringBuffer();

  buffer.writeln('Flutter crash report.');
  buffer.writeln('${globals.userMessages.flutterToolBugInstructions}\n');

  buffer.writeln('## command\n');
  buffer.writeln('${details.command}\n');

  buffer.writeln('## exception\n');
  buffer.writeln('${_crashException(details.error)}\n');
  buffer.writeln('```\n${details.stackTrace}```\n');

  buffer.writeln('## flutter doctor\n');
  buffer.writeln('```\n${await details.doctorText.text}```');

  late File crashFile;
  ErrorHandlingFileSystem.noExitOnFailure(() {
    try {
      crashFile = globals.fsUtils.getUniqueFile(globals.fs.currentDirectory, 'flutter', 'log');
      crashFile.writeAsStringSync(buffer.toString());
    } on FileSystemException catch (_) {
      // Fallback to the system temporary directory.
      try {
        crashFile = globals.fsUtils.getUniqueFile(globals.fs.systemTempDirectory, 'flutter', 'log');
        crashFile.writeAsStringSync(buffer.toString());
      } on FileSystemException catch (e) {
        globals.printError('Could not write crash report to disk: $e');
        globals.printError(buffer.toString());

        rethrow;
      }
    }
  });

  return crashFile;
}

bool _isErrorDueToGitMissing(
  ProcessException exception,
  ProcessManager processManager,
  Logger logger,
) {
  if (!exception.message.contains('git')) {
    return false;
  }

  try {
    return !processManager.canRun('git');
  } on Object catch (error) {
    logger.printTrace('Unable to check whether git is runnable: $error\n');
    return true;
  }
}
