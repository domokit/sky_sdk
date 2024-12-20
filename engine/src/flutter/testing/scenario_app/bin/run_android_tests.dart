// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:dir_contents_diff/dir_contents_diff.dart' show dirContentsDiff;
import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:path/path.dart';
import 'package:process/process.dart';
import 'package:skia_gold_client/skia_gold_client.dart';

import 'utils/adb_logcat_filtering.dart';
import 'utils/environment.dart';
import 'utils/logs.dart';
import 'utils/options.dart';
import 'utils/process_manager_extension.dart';
import 'utils/screenshot_transformer.dart';

// If you update the arguments, update the documentation in the README.md file.
void main(List<String> args) async {
  // Get some basic environment information to guide the rest of the program.
  final Environment environment = Environment(
    isCi: Platform.environment['LUCI_CONTEXT'] != null,
    showVerbose: Options.showVerbose(args),
    logsDir: Platform.environment['FLUTTER_LOGS_DIR'],
  );

  // Determine if the CWD is within an engine checkout.
  final Engine? localEngineDir = Engine.tryFindWithin();

  // Show usage if requested.
  if (Options.showUsage(args)) {
    stdout.writeln(Options.usage(environment: environment, localEngineDir: localEngineDir));
    return;
  }

  // Parse the command line arguments.
  final Options options;
  try {
    options = Options.parse(args, environment: environment, localEngine: localEngineDir);
  } on FormatException catch (error) {
    stderr.writeln(error);
    stderr.writeln(Options.usage(environment: environment, localEngineDir: localEngineDir));
    exitCode = 1;
    return;
  }

  // Capture CTRL-C.
  late final StreamSubscription<void> onSigint;

  // Capture requested termination. The goal is to catch timeouts.
  late final StreamSubscription<void> onSigterm;
  void cancelSignalHandlers() {
    onSigint.cancel();
    onSigterm.cancel();
  }

  runZonedGuarded(
    () async {
      onSigint = ProcessSignal.sigint.watch().listen((_) {
        cancelSignalHandlers();
        panic(<String>['Received SIGINT']);
      });
      onSigterm = ProcessSignal.sigterm.watch().listen((_) {
        cancelSignalHandlers();
        panic(<String>['Received SIGTERM']);
      });
      await _run(
        verbose: options.verbose,
        outDir: Directory(options.outDir),
        adb: File(options.adb),
        smokeTestFullPath: options.smokeTest,
        useSkiaGold: options.useSkiaGold,
        enableImpeller: options.enableImpeller,
        impellerBackend: _ImpellerBackend.tryParse(options.impellerBackend),
        logsDir: Directory(options.logsDir),
        contentsGolden: options.outputContentsGolden,
        ndkStack: options.ndkStack,
        forceSurfaceProducerSurfaceTexture: options.forceSurfaceProducerSurfaceTexture,
        prefixLogsPerRun: options.prefixLogsPerRun,
        recordScreen: options.recordScreen,
      );
      onSigint.cancel();
      exit(0);
    },
    (Object error, StackTrace stackTrace) {
      onSigint.cancel();
      if (error is! Panic) {
        stderr.writeln('Unhandled error: $error');
        stderr.writeln(stackTrace);
      }
      exitCode = 1;
    },
  );
}

const int _tcpPort = 3001;

enum _ImpellerBackend {
  vulkan,
  opengles;

  static _ImpellerBackend? tryParse(String? value) {
    for (final _ImpellerBackend backend in _ImpellerBackend.values) {
      if (backend.name == value) {
        return backend;
      }
    }
    return null;
  }
}

Future<void> _run({
  required bool verbose,
  required Directory outDir,
  required File adb,
  required String? smokeTestFullPath,
  required bool useSkiaGold,
  required bool enableImpeller,
  required _ImpellerBackend? impellerBackend,
  required Directory logsDir,
  required String? contentsGolden,
  required String ndkStack,
  required bool forceSurfaceProducerSurfaceTexture,
  required bool prefixLogsPerRun,
  required bool recordScreen,
}) async {
  const ProcessManager pm = LocalProcessManager();
  final String scenarioAppPath = join(outDir.path, 'scenario_app');

  // Due to the CI environment, the logs directory persists between runs and
  // even different builds. Because we're checking the output directory after
  // each run, we need a clean logs directory to avoid false positives.
  //
  // Only after the runner is done, we can move the logs to the final location.
  //
  // See [_copyFiles] below and https://github.com/flutter/flutter/issues/144402.
  final Directory finalLogsDir = logsDir..createSync(recursive: true);
  logsDir = Directory.systemTemp.createTempSync('scenario_app_test_logs.');
  final String logcatPath = join(logsDir.path, 'logcat.txt');

  final String screenshotPath = logsDir.path;
  final String apkOutPath = join(scenarioAppPath, 'app', 'outputs', 'apk');
  final File testApk = File(join(apkOutPath, 'androidTest', 'debug', 'app-debug-androidTest.apk'));
  final File appApk = File(join(apkOutPath, 'debug', 'app-debug.apk'));
  log('writing logs and screenshots to ${logsDir.path}');

  if (!testApk.existsSync()) {
    panic(<String>[
      'test apk does not exist: ${testApk.path}',
      'make sure to build the selected engine variant',
    ]);
  }

  if (!appApk.existsSync()) {
    panic(<String>[
      'app apk does not exist: ${appApk.path}',
      'make sure to build the selected engine variant',
    ]);
  }

  // Start a TCP socket in the host, and forward it to the device that runs the tests.
  // This allows the test process to start a connection with the host, and write the bytes
  // for the screenshots.
  // On LUCI, the host uploads the screenshots to Skia Gold.
  SkiaGoldClient? skiaGoldClient;
  late final ServerSocket server;
  final List<Future<void>> pendingComparisons = <Future<void>>[];
  final List<Socket> pendingConnections = <Socket>[];
  int comparisonsFailed = 0;
  await step('Starting server...', () async {
    server = await ServerSocket.bind(InternetAddress.anyIPv4, _tcpPort);
    if (verbose) {
      stdout.writeln('listening on host ${server.address.address}:${server.port}');
    }
    server.listen((Socket client) {
      if (verbose) {
        stdout.writeln('client connected ${client.remoteAddress.address}:${client.remotePort}');
      }
      pendingConnections.add(client);
      client
          .transform(const ScreenshotBlobTransformer())
          .listen(
            (Screenshot screenshot) async {
              final String fileName = screenshot.filename;
              final String filePath = join(screenshotPath, fileName);
              {
                const String remotePath = '/data/local/tmp/flutter_screenshot.png';
                ProcessResult result = await pm.run(<String>[
                  'adb',
                  'shell',
                  'screencap',
                  '-p',
                  remotePath,
                ]);
                if (result.exitCode != 0) {
                  panic(<String>['Failed to capture screenshot']);
                }
                result = await pm.run(<String>['adb', 'pull', remotePath, filePath]);
                if (result.exitCode != 0) {
                  panic(<String>['Failed to pull screenshot']);
                }
                result = await pm.run(<String>['adb', 'shell', 'rm', remotePath]);
                if (result.exitCode != 0) {
                  stderr.writeln('Warning: failed to delete old screenshot on device.');
                }
              }
              // Write a single byte into the socket as a signal to ScreenshotUtil.java
              // that the screenshot was taken.
              client.write(0x8);

              assert(skiaGoldClient != null, 'expected Skia Gold client');
              final File goldenFile = File(filePath);
              if (verbose) {
                log('wrote ${goldenFile.absolute.path}');
              }
              if (SkiaGoldClient.isAvailable()) {
                final Future<void> comparison = skiaGoldClient!
                    .addImg(
                      fileName,
                      goldenFile,
                      screenshotSize: screenshot.pixelCount,
                      // Each color channel can be off by 2.
                      pixelColorDelta: 8,
                    )
                    .then((_) => logImportant('skia gold comparison succeeded: $fileName'))
                    .catchError((Object error) {
                      logWarning('skia gold comparison failed: $error');
                      comparisonsFailed++;
                    });
                pendingComparisons.add(comparison);
              }
            },
            onDone: () {
              pendingConnections.remove(client);
            },
          );
    });
  });

  late Process logcatProcess;
  late Future<int> logcatProcessExitCode;
  _ImpellerBackend? actualImpellerBackend;
  Process? screenRecordProcess;

  final IOSink logcat = File(logcatPath).openWrite();
  try {
    await step('Creating screenshot directory `$screenshotPath`...', () async {
      Directory(screenshotPath).createSync(recursive: true);
    });

    await step('Starting logcat...', () async {
      final int exitCode = await pm.runAndForward(<String>[adb.path, 'logcat', '-c']);
      if (exitCode != 0) {
        panic(<String>['could not clear logs']);
      }

      logcatProcess = await pm.start(<String>[adb.path, 'logcat', '-T', '1']);
      final (Future<int> logcatExitCode, Stream<String> logcatOutput) = getProcessStreams(
        logcatProcess,
      );

      logcatProcessExitCode = logcatExitCode;
      String? filterProcessId;

      logcatOutput.listen(
        (String line) {
          // Always write to the full log.
          logcat.writeln(line);
          if (enableImpeller &&
              actualImpellerBackend == null &&
              line.contains('Using the Impeller rendering backend')) {
            if (line.contains('OpenGLES')) {
              actualImpellerBackend = _ImpellerBackend.opengles;
            } else if (line.contains('Vulkan')) {
              actualImpellerBackend = _ImpellerBackend.vulkan;
            } else {
              panic(<String>[
                'Impeller was enabled, but $line did not contain "OpenGLES" or "Vulkan".',
              ]);
            }
          }

          // Conditionally parse and write to stderr.
          final AdbLogLine? adbLogLine = AdbLogLine.tryParse(line);
          if (verbose || adbLogLine == null) {
            log(line);
            return;
          }

          // If we haven't already found a process ID, try to find one.
          // The process ID will help us filter out logs from other processes.
          filterProcessId ??= adbLogLine.tryParseProcess();

          // If this is a "verbose" log, possibly skip it.
          final bool isVerbose = adbLogLine.isVerbose(filterProcessId: filterProcessId);
          if (isVerbose || filterProcessId == null) {
            // We've requested verbose output, so print everything.
            if (verbose) {
              adbLogLine.printFormatted();
            }
            return;
          }

          // It's a non-verbose log, so print it.
          adbLogLine.printFormatted();
        },
        onError: (Object? err) {
          if (verbose) {
            logWarning('logcat stream error: $err');
          }
        },
      );
    });

    await step('Configuring emulator...', () async {
      final int exitCode = await pm.runAndForward(<String>[
        adb.path,
        'shell',
        'settings',
        'put',
        'secure',
        'immersive_mode_confirmations',
        'confirmed',
      ]);
      if (exitCode != 0) {
        panic(<String>['could not configure emulator']);
      }
    });

    await step('Get API level of connected device...', () async {
      final ProcessResult apiLevelProcessResult = await pm.run(<String>[
        adb.path,
        'shell',
        'getprop',
        'ro.build.version.sdk',
      ]);
      if (apiLevelProcessResult.exitCode != 0) {
        panic(<String>['could not get API level of the connected device']);
      }
      final String connectedDeviceAPILevel = (apiLevelProcessResult.stdout as String).trim();
      final Map<String, String> dimensions = <String, String>{
        'AndroidAPILevel': connectedDeviceAPILevel,
        'GraphicsBackend': enableImpeller ? 'impeller-${impellerBackend!.name}' : 'skia',
        'ForceSurfaceProducerSurfaceTexture': '$forceSurfaceProducerSurfaceTexture',
      };
      log('using dimensions: ${json.encode(dimensions)}');
      skiaGoldClient = SkiaGoldClient(outDir, dimensions: dimensions);
    });

    await step('Skia Gold auth...', () async {
      if (SkiaGoldClient.isAvailable()) {
        await skiaGoldClient!.auth();
        log('skia gold client is available');
      } else {
        if (useSkiaGold) {
          panic(<String>['skia gold client is unavailable']);
        } else {
          log('skia gold client is unavaialble');
        }
      }
    });

    await step('Reverse port...', () async {
      final int exitCode = await pm.runAndForward(<String>[
        adb.path,
        'reverse',
        'tcp:3000',
        'tcp:$_tcpPort',
      ]);
      if (exitCode != 0) {
        panic(<String>['could not forward port']);
      }
    });

    await step('Installing app APK...', () async {
      final int exitCode = await pm.runAndForward(<String>[adb.path, 'install', appApk.path]);
      if (exitCode != 0) {
        panic(<String>['could not install app apk']);
      }
    });

    await step('Installing test APK...', () async {
      final int exitCode = await pm.runAndForward(<String>[adb.path, 'install', testApk.path]);
      if (exitCode != 0) {
        panic(<String>['could not install test apk']);
      }
    });

    if (recordScreen) {
      await step('Recording screen...', () async {
        // Create a /tmp directory on the device to store the screen recording.
        final int exitCode = await pm.runAndForward(<String>[
          adb.path,
          'shell',
          'mkdir',
          '-p',
          join(_emulatorStoragePath, 'tmp'),
        ]);
        if (exitCode != 0) {
          panic(<String>['could not create /tmp directory on device']);
        }
        final String screenRecordingPath = join(_emulatorStoragePath, 'tmp', 'screen.mp4');
        screenRecordProcess = await pm.start(<String>[
          adb.path,
          'shell',
          'screenrecord',
          '--time-limit=0',
          '--bugreport',
          screenRecordingPath,
        ]);
        log('writing screen recording to $screenRecordingPath');
      });
    }

    await step('Running instrumented tests...', () async {
      final (int exitCode, StringBuffer out) = await pm.runAndCapture(<String>[
        adb.path,
        'shell',
        'am',
        'instrument',
        '-w',
        '--no-window-animation',
        if (smokeTestFullPath != null) '-e class $smokeTestFullPath',
        if (enableImpeller) '-e enable-impeller true' else '-e enable-impeller false',
        if (impellerBackend != null) '-e impeller-backend ${impellerBackend.name}',
        if (forceSurfaceProducerSurfaceTexture) '-e force-surface-producer-surface-texture true',
        'dev.flutter.scenarios.test/dev.flutter.TestRunner',
      ]);
      if (exitCode != 0) {
        panic(<String>['instrumented tests failed to run']);
      }
      // Unfortunately adb shell am instrument does not return a non-zero exit
      // code when tests fail, but it does seem to print "FAILURES!!!" to
      // stdout, so we can use that as a signal that something went wrong.
      if (out.toString().contains('FAILURES!!!')) {
        stdout.write(out);
        panic(<String>['1 or more tests failed']);
      } else if (comparisonsFailed > 0) {
        panic(<String>['$comparisonsFailed Skia Gold comparisons failed']);
      }
    });

    if (enableImpeller) {
      await step('Validating Impeller...', () async {
        final _ImpellerBackend expectedImpellerBackend = impellerBackend ?? _ImpellerBackend.vulkan;
        if (actualImpellerBackend != expectedImpellerBackend) {
          panic(<String>[
            '--enable-impeller was specified and expected to find "${expectedImpellerBackend.name}", which did not match "${actualImpellerBackend?.name ?? '<impeller disabled>'}".',
          ]);
        }
      });
    }

    await step('Wait for pending Skia gold comparisons...', () async {
      await Future.wait(pendingComparisons);
    });

    final bool allTestsRun = smokeTestFullPath == null;
    final bool checkGoldens = contentsGolden != null;
    if (allTestsRun && checkGoldens) {
      // Check the output here.
      await step('Check output files...', () async {
        // TODO(matanlurey): Resolve this in a better way. On CI this file always exists.
        File(join(screenshotPath, 'noop.txt')).writeAsStringSync('');
        // TODO(gaaclarke): We should move this into dir_contents_diff.
        final String diffScreenhotPath = absolute(screenshotPath);
        _withTemporaryCwd(absolute(dirname(contentsGolden)), () {
          final int exitCode = dirContentsDiff(basename(contentsGolden), diffScreenhotPath);
          if (exitCode != 0) {
            panic(<String>['Output contents incorrect.']);
          }
        });
      });
    }
  } finally {
    // The finally clause is entered if:
    // - The tests have completed successfully.
    // - Any step has failed.
    //
    // Do *NOT* throw exceptions or errors in this block, as these are cleanup
    // steps and the program is about to exit. Instead, just log the error and
    // continue with the cleanup.

    await server.close();
    for (final Socket client in pendingConnections.toList()) {
      client.close();
    }

    await step('Killing test app and test runner...', () async {
      final int exitCode = await pm.runAndForward(<String>[
        adb.path,
        'shell',
        'am',
        'force-stop',
        'dev.flutter.scenarios',
      ]);
      if (exitCode != 0) {
        logError('could not kill test app');
      }
    });

    if (screenRecordProcess != null) {
      await step('Killing screen recording process...', () async {
        // Kill the screen recording process.
        screenRecordProcess!.kill(ProcessSignal.sigkill);
        await screenRecordProcess!.exitCode;

        // Pull the screen recording from the device.
        final String screenRecordingPath = join(_emulatorStoragePath, 'tmp', 'screen.mp4');
        final String screenRecordingLocalPath = join(logsDir.path, 'screen.mp4');
        final int exitCode = await pm.runAndForward(<String>[
          adb.path,
          'pull',
          screenRecordingPath,
          screenRecordingLocalPath,
        ]);
        if (exitCode != 0) {
          logError('could not pull screen recording from device');
        }

        log('wrote screen recording to $screenRecordingLocalPath');

        // Remove the screen recording from the device.
        final int removeExitCode = await pm.runAndForward(<String>[
          adb.path,
          'shell',
          'rm',
          screenRecordingPath,
        ]);
        if (removeExitCode != 0) {
          logError('could not remove screen recording from device');
        }
      });
    }

    await step('Killing logcat process...', () async {
      final bool delivered = logcatProcess.kill(ProcessSignal.sigkill);
      assert(delivered);
      await logcatProcessExitCode;
    });

    await step('Flush logcat...', () async {
      await logcat.flush();
      await logcat.close();
      log('wrote logcat to $logcatPath');

      // Copy the logs to the final location.
      // Optionally prefix the logs with a run number and backend name.
      // See https://github.com/flutter/flutter/issues/144402.
      final StringBuffer prefix = StringBuffer();
      if (prefixLogsPerRun) {
        final int rerunNumber = _getAndIncrementRerunNumber(finalLogsDir.path);
        prefix.write('run_$rerunNumber.');
        if (enableImpeller) {
          prefix.write('impeller');
        } else {
          prefix.write('skia');
        }
        if (enableImpeller) {
          prefix.write('_${impellerBackend!.name}');
        }
        if (forceSurfaceProducerSurfaceTexture) {
          prefix.write('_force-st');
        }
        prefix.write('.');
      }
      _copyFiles(source: logsDir, destination: finalLogsDir, prefix: prefix.toString());
    });

    await step('Symbolize stack traces', () async {
      final ProcessResult result = await pm.run(<String>[
        ndkStack,
        '-sym',
        outDir.path,
        '-dump',
        logcatPath,
      ]);
      if (result.exitCode != 0) {
        panic(<String>['Failed to symbolize stack traces']);
      }
    });

    await step('Remove reverse port...', () async {
      final int exitCode = await pm.runAndForward(<String>[
        adb.path,
        'reverse',
        '--remove',
        'tcp:3000',
      ]);
      if (exitCode != 0) {
        logError('could not unforward port');
      }
    });

    await step('Uninstalling app APK...', () async {
      final int exitCode = await pm.runAndForward(<String>[
        adb.path,
        'uninstall',
        'dev.flutter.scenarios',
      ]);
      if (exitCode != 0) {
        logError('could not uninstall app apk');
      }
    });

    await step('Uninstalling test APK...', () async {
      final int exitCode = await pm.runAndForward(<String>[
        adb.path,
        'uninstall',
        'dev.flutter.scenarios.test',
      ]);
      if (exitCode != 0) {
        logError('could not uninstall app apk');
      }
    });
  }
}

const String _emulatorStoragePath = '/storage/emulated/0/Download';

void _withTemporaryCwd(String path, void Function() callback) {
  final String originalCwd = Directory.current.path;
  Directory.current = Directory(path).path;

  try {
    callback();
  } finally {
    Directory.current = originalCwd;
  }
}

/// Reads the file named `reruns.txt` in the logs directory and returns the number of reruns.
///
/// If the file does not exist, it is created with the number 1 and that number is returned.
int _getAndIncrementRerunNumber(String logsDir) {
  final File rerunFile = File(join(logsDir, 'reruns.txt'));
  if (!rerunFile.existsSync()) {
    rerunFile.writeAsStringSync('1');
    return 1;
  }
  final int rerunNumber = int.parse(rerunFile.readAsStringSync()) + 1;
  rerunFile.writeAsStringSync(rerunNumber.toString());
  return rerunNumber;
}

/// Copies the contents of [source] to [destination], optionally adding a [prefix] to the destination path.
///
/// This function is used to copy the screenshots from the device to the logs directory.
void _copyFiles({required Directory source, required Directory destination, String prefix = ''}) {
  for (final FileSystemEntity entity in source.listSync()) {
    if (entity is File) {
      entity.copySync(join(destination.path, prefix + basename(entity.path)));
    }
  }
}
