// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:meta/meta.dart';
import 'package:process/process.dart';

import '../artifacts.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/platform.dart';
import '../base/process.dart';
import '../cache.dart';
import '../convert.dart';
import '../device.dart';
import 'code_signing.dart';

// Error message patterns from ios-deploy output
const String noProvisioningProfileErrorOne = 'Error 0xe8008015';
const String noProvisioningProfileErrorTwo = 'Error 0xe8000067';
const String deviceLockedError = 'e80000e2';
const String deviceLockedErrorMessage = 'the device was not, or could not be, unlocked';
const String unknownAppLaunchError = 'Error 0xe8000022';

class IOSDeploy {
  IOSDeploy({
    required Artifacts artifacts,
    required Cache cache,
    required Logger logger,
    required Platform platform,
    required ProcessManager processManager,
  }) : _platform = platform,
       _cache = cache,
       _processUtils = ProcessUtils(processManager: processManager, logger: logger),
       _logger = logger,
       _binaryPath = artifacts.getHostArtifact(HostArtifact.iosDeploy).path;

  final Cache _cache;
  final String _binaryPath;
  final Logger _logger;
  final Platform _platform;
  final ProcessUtils _processUtils;

  Map<String, String> get iosDeployEnv {
    // Push /usr/bin to the front of PATH to pick up default system python, package 'six'.
    //
    // ios-deploy transitively depends on LLDB.framework, which invokes a
    // Python script that uses package 'six'. LLDB.framework relies on the
    // python at the front of the path, which may not include package 'six'.
    // Ensure that we pick up the system install of python, which includes it.
    final Map<String, String> environment = Map<String, String>.of(_platform.environment);
    environment['PATH'] = '/usr/bin:${environment['PATH']}';
    environment.addEntries(<MapEntry<String, String>>[_cache.dyLdLibEntry]);
    return environment;
  }

  /// Uninstalls the specified app bundle.
  ///
  /// Uses ios-deploy and returns the exit code.
  Future<int> uninstallApp({required String deviceId, required String bundleId}) async {
    final List<String> launchCommand = <String>[
      _binaryPath,
      '--id',
      deviceId,
      '--uninstall_only',
      '--bundle_id',
      bundleId,
    ];

    return _processUtils.stream(
      launchCommand,
      mapFunction: _monitorFailure,
      trace: true,
      environment: iosDeployEnv,
    );
  }

  /// Installs the specified app bundle.
  ///
  /// Uses ios-deploy and returns the exit code.
  Future<int> installApp({
    required String deviceId,
    required String bundlePath,
    required List<String> launchArguments,
    required DeviceConnectionInterface interfaceType,
    Directory? appDeltaDirectory,
  }) async {
    appDeltaDirectory?.createSync(recursive: true);
    final List<String> launchCommand = <String>[
      _binaryPath,
      '--id',
      deviceId,
      '--bundle',
      bundlePath,
      if (appDeltaDirectory != null) ...<String>['--app_deltas', appDeltaDirectory.path],
      if (interfaceType != DeviceConnectionInterface.wireless) '--no-wifi',
      if (launchArguments.isNotEmpty) ...<String>['--args', launchArguments.join(' ')],
    ];

    return _processUtils.stream(
      launchCommand,
      mapFunction: _monitorFailure,
      trace: true,
      environment: iosDeployEnv,
    );
  }

  /// Returns [IOSDeployDebugger] wrapping attached debugger logic.
  ///
  /// This method does not install the app. Call [IOSDeployDebugger.launchAndAttach()]
  /// to install and attach the debugger to the specified app bundle.
  IOSDeployDebugger prepareDebuggerForLaunch({
    required String deviceId,
    required String bundlePath,
    required List<String> launchArguments,
    required DeviceConnectionInterface interfaceType,
    Directory? appDeltaDirectory,
    required bool uninstallFirst,
    bool skipInstall = false,
  }) {
    appDeltaDirectory?.createSync(recursive: true);
    // Interactive debug session to support sending the lldb detach command.
    final List<String> launchCommand = <String>[
      'script',
      '-t',
      '0',
      '/dev/null',
      _binaryPath,
      '--id',
      deviceId,
      '--bundle',
      bundlePath,
      if (appDeltaDirectory != null) ...<String>['--app_deltas', appDeltaDirectory.path],
      if (uninstallFirst) '--uninstall',
      if (skipInstall) '--noinstall',
      '--debug',
      if (interfaceType != DeviceConnectionInterface.wireless) '--no-wifi',
      if (launchArguments.isNotEmpty) ...<String>['--args', launchArguments.join(' ')],
    ];
    return IOSDeployDebugger(
      launchCommand: launchCommand,
      logger: _logger,
      processUtils: _processUtils,
      iosDeployEnv: iosDeployEnv,
    );
  }

  /// Installs and then runs the specified app bundle.
  ///
  /// Uses ios-deploy and returns the exit code.
  Future<int> launchApp({
    required String deviceId,
    required String bundlePath,
    required List<String> launchArguments,
    required DeviceConnectionInterface interfaceType,
    required bool uninstallFirst,
    Directory? appDeltaDirectory,
  }) async {
    appDeltaDirectory?.createSync(recursive: true);
    final List<String> launchCommand = <String>[
      _binaryPath,
      '--id',
      deviceId,
      '--bundle',
      bundlePath,
      if (appDeltaDirectory != null) ...<String>['--app_deltas', appDeltaDirectory.path],
      if (interfaceType != DeviceConnectionInterface.wireless) '--no-wifi',
      if (uninstallFirst) '--uninstall',
      '--justlaunch',
      if (launchArguments.isNotEmpty) ...<String>['--args', launchArguments.join(' ')],
    ];

    return _processUtils.stream(
      launchCommand,
      mapFunction: _monitorFailure,
      trace: true,
      environment: iosDeployEnv,
    );
  }

  Future<bool> isAppInstalled({required String bundleId, required String deviceId}) async {
    final List<String> launchCommand = <String>[
      _binaryPath,
      '--id',
      deviceId,
      '--exists',
      '--timeout', // If the device is not connected, ios-deploy will wait forever.
      '10',
      '--bundle_id',
      bundleId,
    ];
    final RunResult result = await _processUtils.run(launchCommand, environment: iosDeployEnv);
    // Device successfully connected, but app not installed.
    if (result.exitCode == 255) {
      _logger.printTrace('$bundleId not installed on $deviceId');
      return false;
    }
    if (result.exitCode != 0) {
      _logger.printTrace('App install check failed: ${result.stderr}');
      return false;
    }
    return true;
  }

  String _monitorFailure(String stdout) => _monitorIOSDeployFailure(stdout, _logger);
}

/// lldb attach state flow.
enum _IOSDeployDebuggerState { detached, launching, attached }

/// Wrapper to launch app and attach the debugger with ios-deploy.
class IOSDeployDebugger {
  IOSDeployDebugger({
    required Logger logger,
    required ProcessUtils processUtils,
    required List<String> launchCommand,
    required Map<String, String> iosDeployEnv,
  }) : _processUtils = processUtils,
       _logger = logger,
       _launchCommand = launchCommand,
       _iosDeployEnv = iosDeployEnv,
       _debuggerState = _IOSDeployDebuggerState.detached;

  /// Create a [IOSDeployDebugger] for testing.
  ///
  /// Sets the command to "ios-deploy" and environment to an empty map.
  @visibleForTesting
  factory IOSDeployDebugger.test({required ProcessManager processManager, Logger? logger}) {
    final Logger debugLogger = logger ?? BufferLogger.test();
    return IOSDeployDebugger(
      logger: debugLogger,
      processUtils: ProcessUtils(logger: debugLogger, processManager: processManager),
      launchCommand: <String>['ios-deploy'],
      iosDeployEnv: <String, String>{},
    );
  }

  final Logger _logger;
  final ProcessUtils _processUtils;
  final List<String> _launchCommand;
  final Map<String, String> _iosDeployEnv;

  Process? _iosDeployProcess;

  Stream<String> get logLines => _debuggerOutput.stream;
  final StreamController<String> _debuggerOutput = StreamController<String>.broadcast();

  bool get debuggerAttached => _debuggerState == _IOSDeployDebuggerState.attached;
  _IOSDeployDebuggerState _debuggerState;

  @visibleForTesting
  String? symbolsDirectoryPath;

  // (lldb)    platform select remote-'ios' --sysroot
  // https://github.com/ios-control/ios-deploy/blob/1.11.2-beta.1/src/ios-deploy/ios-deploy.m#L33
  // This regex is to get the configurable lldb prompt. By default this prompt will be "lldb".
  static final RegExp _lldbPlatformSelect = RegExp(r"\s*platform select remote-'ios' --sysroot");

  // (lldb)     run
  // https://github.com/ios-control/ios-deploy/blob/1.11.2-beta.1/src/ios-deploy/ios-deploy.m#L51
  static final RegExp _lldbProcessExit = RegExp(r'Process \d* exited with status =');

  // (lldb) Process 6152 stopped
  static final RegExp _lldbProcessStopped = RegExp(r'Process \d* stopped');

  // (lldb) Process 6152 detached
  static final RegExp _lldbProcessDetached = RegExp(r'Process \d* detached');

  // (lldb) Process 6152 resuming
  static final RegExp _lldbProcessResuming = RegExp(r'Process \d+ resuming');

  // Symbol Path: /Users/swarming/Library/Developer/Xcode/iOS DeviceSupport/16.2 (20C65) arm64e/Symbols
  static final RegExp _symbolsPathPattern = RegExp(r'.*Symbol Path: ');

  // Send signal to stop (pause) the app. Used before a backtrace dump.
  static const String _signalStop = 'process signal SIGSTOP';
  static const String _signalStopError = 'Failed to send signal 17';

  static const String _processResume = 'process continue';
  static const String _processInterrupt = 'process interrupt';

  // Print backtrace for all threads while app is stopped.
  static const String _backTraceAll = 'thread backtrace all';

  /// If this is non-null, then the app process is paused and awaiting backtrace logging.
  ///
  /// The future should be completed once the backtraces are logged.
  Completer<void>? _processResumeCompleter;

  // Process 525 exited with status = -1 (0xffffffff) lost connection
  static final RegExp _lostConnectionPattern = RegExp(
    r'exited with status = -1 \(0xffffffff\) lost connection',
  );

  /// Whether ios-deploy received a message matching [_lostConnectionPattern],
  /// indicating that it lost connection to the device.
  bool get lostConnection => _lostConnection;
  bool _lostConnection = false;

  /// Launch the app on the device, and attach the debugger.
  ///
  /// Returns whether or not the debugger successfully attached.
  Future<bool> launchAndAttach() async {
    // Return when the debugger attaches, or the ios-deploy process exits.

    // (lldb)     run
    // https://github.com/ios-control/ios-deploy/blob/1.11.2-beta.1/src/ios-deploy/ios-deploy.m#L51
    RegExp lldbRun = RegExp(r'\(lldb\)\s*run');

    final Completer<bool> debuggerCompleter = Completer<bool>();

    bool receivedLogs = false;
    try {
      _iosDeployProcess = await _processUtils.start(_launchCommand, environment: _iosDeployEnv);
      String? lastLineFromDebugger;
      final StreamSubscription<String> stdoutSubscription = _iosDeployProcess!.stdout
          .transform<String>(utf8.decoder)
          .transform<String>(const LineSplitter())
          .listen((String line) {
            _monitorIOSDeployFailure(line, _logger);

            // (lldb)    platform select remote-'ios' --sysroot
            // Use the configurable custom lldb prompt in the regex. The developer can set this prompt to anything.
            // For example `settings set prompt "(mylldb)"` in ~/.lldbinit results in:
            // "(mylldb)    platform select remote-'ios' --sysroot"
            if (_lldbPlatformSelect.hasMatch(line)) {
              final String platformSelect = _lldbPlatformSelect.stringMatch(line) ?? '';
              if (platformSelect.isEmpty) {
                return;
              }
              final int promptEndIndex = line.indexOf(platformSelect);
              if (promptEndIndex == -1) {
                return;
              }
              final String prompt = line.substring(0, promptEndIndex);
              lldbRun = RegExp(RegExp.escape(prompt) + r'\s*run');
              _logger.printTrace(line);
              return;
            }

            // Symbol Path: /Users/swarming/Library/Developer/Xcode/iOS DeviceSupport/16.2 (20C65) arm64e/Symbols
            if (_symbolsPathPattern.hasMatch(line)) {
              _logger.printTrace('Detected path to iOS debug symbols: "$line"');
              final String prefix = _symbolsPathPattern.stringMatch(line) ?? '';
              if (prefix.isEmpty) {
                return;
              }
              symbolsDirectoryPath = line.substring(prefix.length);
              return;
            }

            // (lldb)     run
            // success
            // 2020-09-15 13:42:25.185474-0700 Runner[477:181141] flutter: The Dart VM service is listening on http://127.0.0.1:57782/
            if (lldbRun.hasMatch(line)) {
              _logger.printTrace(line);
              _debuggerState = _IOSDeployDebuggerState.launching;
              return;
            }
            // Next line after "run" must be "success", or the attach failed.
            // Example: "error: process launch failed"
            if (_debuggerState == _IOSDeployDebuggerState.launching) {
              _logger.printTrace(line);
              final bool attachSuccess = line == 'success';
              _debuggerState =
                  attachSuccess
                      ? _IOSDeployDebuggerState.attached
                      : _IOSDeployDebuggerState.detached;
              if (!debuggerCompleter.isCompleted) {
                debuggerCompleter.complete(attachSuccess);
              }
              return;
            }

            // (lldb) process signal SIGSTOP
            // or
            // process signal SIGSTOP
            if (line.contains(_signalStop)) {
              // The app is about to be stopped. Only show in verbose mode.
              _logger.printTrace(line);
              return;
            }

            // error: Failed to send signal 17: failed to send signal 17
            if (line.contains(_signalStopError)) {
              // The stop signal failed, force exit.
              exit();
              return;
            }

            if (line == _backTraceAll) {
              // The app is stopped and the backtrace for all threads will be printed.
              _logger.printTrace(line);
              // Even though we're not "detached", just stopped, mark as detached so the backtrace
              // is only show in verbose.
              _debuggerState = _IOSDeployDebuggerState.detached;

              // If we paused the app and are waiting to resume it, complete the completer
              final Completer<void>? processResumeCompleter = _processResumeCompleter;
              if (processResumeCompleter != null) {
                _processResumeCompleter = null;
                processResumeCompleter.complete();
              }
              return;
            }

            if (line.contains('PROCESS_STOPPED') || _lldbProcessStopped.hasMatch(line)) {
              // The app has been stopped. Dump the backtrace, and detach.
              _logger.printTrace(line);
              _iosDeployProcess?.stdin.writeln(_backTraceAll);
              if (_processResumeCompleter == null) {
                detach();
              }
              return;
            }

            if (line.contains('PROCESS_EXITED') || _lldbProcessExit.hasMatch(line)) {
              // The app exited or crashed, so exit. Continue passing debugging
              // messages to the log reader until it exits to capture crash dumps.
              _logger.printTrace(line);
              if (line.contains(_lostConnectionPattern)) {
                _lostConnection = true;
              }
              exit();
              return;
            }
            if (_lldbProcessDetached.hasMatch(line)) {
              // The debugger has detached from the app, and there will be no more debugging messages.
              // Kill the ios-deploy process.
              _logger.printTrace(line);
              exit();
              return;
            }

            if (_lldbProcessResuming.hasMatch(line)) {
              _logger.printTrace(line);
              // we marked this detached when we received [_backTraceAll]
              _debuggerState = _IOSDeployDebuggerState.attached;
              return;
            }

            if (_debuggerState != _IOSDeployDebuggerState.attached) {
              _logger.printTrace(line);
              return;
            }
            if (lastLineFromDebugger != null && lastLineFromDebugger!.isNotEmpty && line.isEmpty) {
              // The lldb console stream from ios-deploy is separated lines by an extra \r\n.
              // To avoid all lines being double spaced, if the last line from the
              // debugger was not an empty line, skip this empty line.
              // This will still cause "legit" logged newlines to be doubled...
            } else if (!_debuggerOutput.isClosed) {
              _debuggerOutput.add(line);

              // Sometimes the `ios-deploy` process does not return logs from the
              // application after attaching, such as the Dart VM url. In CI,
              // `idevicesyslog` is used as a fallback to get logs. Print a
              // message to indicate whether logs were received from `ios-deploy`
              // to help with debugging.
              if (!receivedLogs) {
                _logger.printTrace('Received logs from ios-deploy.');
                receivedLogs = true;
              }
            }
            lastLineFromDebugger = line;
          });
      final StreamSubscription<String> stderrSubscription = _iosDeployProcess!.stderr
          .transform<String>(utf8.decoder)
          .transform<String>(const LineSplitter())
          .listen((String line) {
            _monitorIOSDeployFailure(line, _logger);
            _logger.printTrace(line);
          });
      unawaited(
        _iosDeployProcess!.exitCode
            .then((int status) async {
              _logger.printTrace('ios-deploy exited with code $exitCode');
              _debuggerState = _IOSDeployDebuggerState.detached;
              await stdoutSubscription.cancel();
              await stderrSubscription.cancel();
            })
            .whenComplete(() async {
              if (_debuggerOutput.hasListener) {
                // Tell listeners the process died.
                await _debuggerOutput.close();
              }
              if (!debuggerCompleter.isCompleted) {
                debuggerCompleter.complete(false);
              }
              _iosDeployProcess = null;
            }),
      );
    } on ProcessException catch (exception, stackTrace) {
      _logger.printTrace('ios-deploy failed: $exception');
      _debuggerState = _IOSDeployDebuggerState.detached;
      if (!_debuggerOutput.isClosed) {
        _debuggerOutput.addError(exception, stackTrace);
      }
    } on ArgumentError catch (exception, stackTrace) {
      _logger.printTrace('ios-deploy failed: $exception');
      _debuggerState = _IOSDeployDebuggerState.detached;
      if (!_debuggerOutput.isClosed) {
        _debuggerOutput.addError(exception, stackTrace);
      }
    }
    // Wait until the debugger attaches, or the attempt fails.
    return debuggerCompleter.future;
  }

  bool exit() {
    final bool success = (_iosDeployProcess == null) || _iosDeployProcess!.kill();
    _iosDeployProcess = null;
    return success;
  }

  /// Pause app, dump backtrace for debugging, and resume.
  Future<void> pauseDumpBacktraceResume() async {
    if (!debuggerAttached) {
      return;
    }
    final Completer<void> completer = Completer<void>();
    _processResumeCompleter = completer;
    try {
      // Stop the app, which will prompt the backtrace to be printed for all threads in the stdoutSubscription handler.
      _iosDeployProcess?.stdin.writeln(_processInterrupt);
    } on SocketException catch (error) {
      _logger.printTrace('Could not stop app from debugger: $error');
    }
    // wait for backtrace to be dumped
    await completer.future;
    _iosDeployProcess?.stdin.writeln(_processResume);
  }

  /// Check what files are found in the device's iOS DeviceSupport directory.
  ///
  /// Expected files include Symbols (directory), Info.plist, and .finalized.
  ///
  /// If any of the expected files are missing or there are additional files
  /// (such as .copying_lock or .processing_lock), this may indicate the
  /// symbols may still be fetching or something went wrong when fetching them.
  ///
  /// Used for debugging test flakes: https://github.com/flutter/flutter/issues/121231
  Future<void> checkForSymbolsFiles(FileSystem fileSystem) async {
    if (symbolsDirectoryPath == null) {
      _logger.printTrace('No path provided for Symbols directory.');
      return;
    }
    final Directory symbolsDirectory = fileSystem.directory(symbolsDirectoryPath);
    if (!symbolsDirectory.existsSync()) {
      _logger.printTrace('Unable to find Symbols directory at $symbolsDirectoryPath');
      return;
    }
    final Directory currentDeviceSupportDir = symbolsDirectory.parent;
    final List<FileSystemEntity> symbolStatusFiles = currentDeviceSupportDir.listSync();
    _logger.printTrace('Symbol files:');
    for (final FileSystemEntity file in symbolStatusFiles) {
      _logger.printTrace('  ${file.basename}');
    }
  }

  Future<void> stopAndDumpBacktrace() async {
    if (!debuggerAttached) {
      return;
    }
    // Stop the app, which will prompt the backtrace to be printed for all
    // threads in the stdoutSubscription handler.
    await stdinWriteln(
      _signalStop,
      onError: (Object error, _) {
        _logger.printTrace('Could not stop the app: $error');
      },
    );

    // Wait for logging to finish on process exit.
    return logLines.drain();
  }

  Future<void>? _stdinWriteFuture;

  /// Queue write of [line] to STDIN of [_iosDeployProcess].
  ///
  /// No-op if [_iosDeployProcess] is null.
  ///
  /// This write will not happen until the flush of any previous writes have
  /// completed, because calling [IOSink.flush()] before a previous flush has
  /// completed will throw a [StateError].
  ///
  /// This method needs to keep track of the [_stdinWriteFuture] from previous
  /// calls because the future returned by [detach] is not always await-ed.
  Future<void> stdinWriteln(
    String line, {
    required void Function(Object, StackTrace) onError,
  }) async {
    final Process? process = _iosDeployProcess;
    if (process == null) {
      return;
    }

    Future<void> writeln() {
      return ProcessUtils.writelnToStdinGuarded(stdin: process.stdin, line: line, onError: onError);
    }

    _stdinWriteFuture = _stdinWriteFuture?.then<void>((_) => writeln()) ?? writeln();
    return _stdinWriteFuture;
  }

  Future<void> detach() async {
    if (!debuggerAttached) {
      return;
    }

    return stdinWriteln(
      'process detach',
      onError: (Object error, _) {
        // Best effort, try to detach, but maybe the app already exited or already detached.
        _logger.printTrace('Could not detach from debugger: $error');
      },
    );
  }
}

// Maps stdout line stream. Must return original line.
String _monitorIOSDeployFailure(String stdout, Logger logger) {
  // Installation issues.
  if (stdout.contains(noProvisioningProfileErrorOne) ||
      stdout.contains(noProvisioningProfileErrorTwo)) {
    logger.printError(noProvisioningProfileInstruction, emphasis: true);

    // Launch issues.
  } else if (stdout.contains(deviceLockedError) || stdout.contains(deviceLockedErrorMessage)) {
    logger.printError(
      '''
═══════════════════════════════════════════════════════════════════════════════════
Your device is locked. Unlock your device first before running.
═══════════════════════════════════════════════════════════════════════════════════''',
      emphasis: true,
    );
  } else if (stdout.contains(unknownAppLaunchError)) {
    logger.printError(
      '''
═══════════════════════════════════════════════════════════════════════════════════
Error launching app. Try launching from within Xcode via:
    open ios/Runner.xcworkspace

Your Xcode version may be too old for your iOS version.
═══════════════════════════════════════════════════════════════════════════════════''',
      emphasis: true,
    );
  }

  return stdout;
}
