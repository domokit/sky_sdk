// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:ffi';
import 'dart:io';

import '../framework/devices.dart';
import '../framework/framework.dart';
import '../framework/task_result.dart';
import '../framework/utils.dart';

TaskFunction createAndroidRunDebugTest() {
  return AndroidRunOutputTest(release: false).call;
}

TaskFunction createAndroidRunReleaseTest() {
  return AndroidRunOutputTest(release: true).call;
}

TaskFunction createLinuxRunDebugTest() {
  return DesktopRunOutputTest(
    '${flutterDirectory.path}/dev/integration_tests/ui',
    'lib/empty.dart',
    release: false,
  ).call;
}

TaskFunction createLinuxRunReleaseTest() {
  return DesktopRunOutputTest(
    '${flutterDirectory.path}/dev/integration_tests/ui',
    'lib/empty.dart',
    release: true,
  ).call;
}

TaskFunction createMacOSRunDebugTest() {
  return DesktopRunOutputTest(
    '${flutterDirectory.path}/dev/integration_tests/ui',
    'lib/main.dart',
    release: false,
    allowStderr: true,
  ).call;
}

TaskFunction createMacOSRunReleaseTest() {
  return DesktopRunOutputTest(
    '${flutterDirectory.path}/dev/integration_tests/ui',
    'lib/main.dart',
    release: true,
    allowStderr: true,
  ).call;
}

TaskFunction createWindowsRunDebugTest() {
  return WindowsRunOutputTest(
    '${flutterDirectory.path}/dev/integration_tests/ui',
    'lib/empty.dart',
    release: false,
  ).call;
}

TaskFunction createWindowsRunReleaseTest() {
  return WindowsRunOutputTest(
    '${flutterDirectory.path}/dev/integration_tests/ui',
    'lib/empty.dart',
    release: true,
  ).call;
}

class AndroidRunOutputTest extends RunOutputTask {
  AndroidRunOutputTest({required super.release})
    : super('${flutterDirectory.path}/dev/integration_tests/ui', 'lib/main.dart');

  @override
  Future<void> prepare(String deviceId) async {
    // Uninstall if the app is already installed on the device to get to a clean state.
    final List<String> stderr = <String>[];
    print('uninstalling...');
    final Process uninstall = await startFlutter(
      'install',
      // TODO(andrewkolos): consider removing -v after
      //  https://github.com/flutter/flutter/issues/153367 is troubleshot.
      options: <String>['--suppress-analytics', '--uninstall-only', '-d', deviceId, '-v'],
      isBot: false,
    );
    uninstall.stdout.transform<String>(utf8.decoder).transform<String>(const LineSplitter()).listen(
      (String line) {
        print('uninstall:stdout: $line');
      },
    );
    uninstall.stderr.transform<String>(utf8.decoder).transform<String>(const LineSplitter()).listen(
      (String line) {
        print('uninstall:stderr: $line');
        stderr.add(line);
      },
    );
    if (await uninstall.exitCode != 0) {
      throw 'flutter install --uninstall-only failed.';
    }
    if (stderr.isNotEmpty) {
      throw 'flutter install --uninstall-only had output on standard error.';
    }
  }

  @override
  bool isExpectedStderr(String line) {
    // TODO(egarciad): Remove once https://github.com/flutter/flutter/issues/95131 is fixed.
    return line.contains('Mapping new ns');
  }

  @override
  TaskResult verify(List<String> stdout, List<String> stderr) {
    final String gradleTask = release ? 'assembleRelease' : 'assembleDebug';
    final String apk = release ? 'app-release.apk' : 'app-debug.apk';

    _findNextMatcherInList(
      stdout,
      (String line) =>
          line.startsWith('Launching lib/main.dart on ') &&
          line.endsWith(' in ${release ? 'release' : 'debug'} mode...'),
      'Launching lib/main.dart on',
    );

    _findNextMatcherInList(
      stdout,
      (String line) => line.startsWith("Running Gradle task '$gradleTask'..."),
      "Running Gradle task '$gradleTask'...",
    );

    // Size information is only included in release builds.
    _findNextMatcherInList(
      stdout,
      (String line) =>
          line.contains('Built build/app/outputs/flutter-apk/$apk') &&
          (!release || line.contains('MB)')),
      'Built build/app/outputs/flutter-apk/$apk',
    );

    _findNextMatcherInList(
      stdout,
      (String line) => line.startsWith('Installing build/app/outputs/flutter-apk/$apk...'),
      'Installing build/app/outputs/flutter-apk/$apk...',
    );

    _findNextMatcherInList(
      stdout,
      (String line) => line.contains('Quit (terminate the application on the device).'),
      'q Quit (terminate the application on the device)',
    );

    _findNextMatcherInList(
      stdout,
      (String line) => line == 'Application finished.',
      'Application finished.',
    );

    return TaskResult.success(null);
  }
}

class WindowsRunOutputTest extends DesktopRunOutputTest {
  WindowsRunOutputTest(
    super.testDirectory,
    super.testTarget, {
    required super.release,
    super.allowStderr = false,
  });

  final String arch = Abi.current() == Abi.windowsX64 ? 'x64' : 'arm64';

  static final RegExp _buildOutput = RegExp(
    r'Building Windows application\.\.\.\s*\d+(\.\d+)?(ms|s)',
    multiLine: true,
  );
  static final RegExp _builtOutput = RegExp(
    r'Built build\\windows\\(x64|arm64)\\runner\\(Debug|Release)\\\w+\.exe( \(\d+(\.\d+)?MB\))?',
  );

  @override
  void verifyBuildOutput(List<String> stdout) {
    _findNextMatcherInList(stdout, _buildOutput.hasMatch, 'Building Windows application...');

    final String buildMode = release ? 'Release' : 'Debug';
    _findNextMatcherInList(stdout, (String line) {
      if (!_builtOutput.hasMatch(line) || !line.contains(buildMode)) {
        return false;
      }

      return true;
    }, '√ Built build\\windows\\$arch\\runner\\$buildMode\\ui.exe');
  }
}

class DesktopRunOutputTest extends RunOutputTask {
  DesktopRunOutputTest(
    super.testDirectory,
    super.testTarget, {
    required super.release,
    this.allowStderr = false,
  });

  /// Whether `flutter run` is expected to produce output on stderr.
  final bool allowStderr;

  @override
  bool isExpectedStderr(String line) => allowStderr;

  @override
  TaskResult verify(List<String> stdout, List<String> stderr) {
    _findNextMatcherInList(
      stdout,
      (String line) =>
          line.startsWith('Launching $testTarget on ') &&
          line.endsWith(' in ${release ? 'release' : 'debug'} mode...'),
      'Launching $testTarget on',
    );

    verifyBuildOutput(stdout);

    _findNextMatcherInList(
      stdout,
      (String line) => line.contains('Quit (terminate the application on the device).'),
      'q Quit (terminate the application on the device)',
    );

    _findNextMatcherInList(
      stdout,
      (String line) => line == 'Application finished.',
      'Application finished.',
    );

    return TaskResult.success(null);
  }

  /// Verify the output from `flutter run`'s build step.
  void verifyBuildOutput(List<String> stdout) {}
}

/// Test that the output of `flutter run` is expected.
abstract class RunOutputTask {
  RunOutputTask(this.testDirectory, this.testTarget, {required this.release});

  static final RegExp _engineLogRegex = RegExp(r'\[(VERBOSE|INFO|WARNING|ERROR|FATAL):.+\(\d+\)\]');

  /// The directory where the app under test is defined.
  final String testDirectory;

  /// The main entry-point file of the application, as run on the device.
  final String testTarget;

  /// Whether to run the app in release mode.
  final bool release;

  Future<TaskResult> call() {
    return inDirectory<TaskResult>(testDirectory, () async {
      final Device device = await devices.workingDevice;
      await device.unlock();
      final String deviceId = device.deviceId;

      final Completer<void> ready = Completer<void>();
      final List<String> stdout = <String>[];
      final List<String> stderr = <String>[];

      await prepare(deviceId);

      final List<String> options = <String>[testTarget, '-d', deviceId, if (release) '--release'];

      final Process run = await startFlutter('run', options: options, isBot: false);

      int? runExitCode;
      run.stdout.transform<String>(utf8.decoder).transform<String>(const LineSplitter()).listen((
        String line,
      ) {
        print('run:stdout: $line');
        stdout.add(line);
        if (line.contains('Quit (terminate the application on the device).')) {
          ready.complete();
        }
      });
      final Stream<String> runStderr =
          run.stderr
              .transform<String>(utf8.decoder)
              .transform<String>(const LineSplitter())
              .asBroadcastStream();
      runStderr.listen((String line) => print('run:stderr: $line'));
      runStderr.skipWhile(isExpectedStderr).listen((String line) => stderr.add(line));
      unawaited(
        run.exitCode.then<void>((int exitCode) {
          runExitCode = exitCode;
        }),
      );
      await Future.any<dynamic>(<Future<dynamic>>[ready.future, run.exitCode]);
      if (runExitCode != null) {
        throw 'Failed to run test app; runner unexpected exited, with exit code $runExitCode.';
      }
      run.stdin.write('q');

      await run.exitCode;

      if (stderr.isNotEmpty) {
        throw 'flutter run ${release ? '--release' : ''} had unexpected output on standard error.';
      }

      final List<String> engineLogs = List<String>.from(stdout.where(_engineLogRegex.hasMatch));
      if (engineLogs.isNotEmpty) {
        throw 'flutter run had unexpected Flutter engine logs $engineLogs';
      }

      return verify(stdout, stderr);
    });
  }

  /// Prepare the device for running the test app.
  Future<void> prepare(String deviceId) => Future<void>.value();

  /// Returns true if this stderr output line is expected.
  bool isExpectedStderr(String line) => false;

  /// Verify the output of `flutter run`.
  TaskResult verify(List<String> stdout, List<String> stderr) =>
      throw UnimplementedError('verify is not implemented');

  /// Helper that verifies a line in [list] matches [matcher].
  /// The [list] is updated to contain the lines remaining after the match.
  void _findNextMatcherInList(
    List<String> list,
    bool Function(String testLine) matcher,
    String errorMessageExpectedLine,
  ) {
    final List<String> copyOfListForErrorMessage = List<String>.from(list);

    while (list.isNotEmpty) {
      final String nextLine = list.first;
      list.removeAt(0);

      if (matcher(nextLine)) {
        return;
      }
    }

    throw '''
Did not find expected line

$errorMessageExpectedLine

in flutter run ${release ? '--release' : ''} stdout

$copyOfListForErrorMessage
''';
  }
}
