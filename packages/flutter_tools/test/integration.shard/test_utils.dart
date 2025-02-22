// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/local.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:process/process.dart';
import 'package:vm_service/vm_service.dart';

import '../src/common.dart';
import 'test_driver.dart';

/// The [FileSystem] for the integration test environment.
const FileSystem fileSystem = LocalFileSystem();

/// The (real) `flutter` binary (i.e. `{ROOT}/bin/flutter`) to execute in tests.
final String flutterBin = fileSystem.path.join(
  getFlutterRoot(),
  'bin',
  platform.isWindows ? 'flutter.bat' : 'flutter',
);

/// The [Platform] for the integration test environment.
const Platform platform = LocalPlatform();

/// The [ProcessManager] for the integration test environment.
const ProcessManager processManager = LocalProcessManager();

/// Creates a temporary directory but resolves any symlinks to return the real
/// underlying path to avoid issues with breakpoints/hot reload.
/// https://github.com/flutter/flutter/pull/21741
Directory createResolvedTempDirectorySync(String prefix) {
  assert(prefix.endsWith('.'));
  final Directory tempDirectory = fileSystem.systemTempDirectory.createTempSync('flutter_$prefix');
  return fileSystem.directory(tempDirectory.resolveSymbolicLinksSync());
}

void writeFile(String path, String content, {bool writeFutureModifiedDate = false}) {
  final File file =
      fileSystem.file(path)
        ..createSync(recursive: true)
        ..writeAsStringSync(content, flush: true);
  // Some integration tests on Windows to not see this file as being modified
  // recently enough for the hot reload to pick this change up unless the
  // modified time is written in the future.
  if (writeFutureModifiedDate) {
    file.setLastModifiedSync(DateTime.now().add(const Duration(seconds: 5)));
  }
}

void writeBytesFile(String path, List<int> content) {
  fileSystem.file(path)
    ..createSync(recursive: true)
    ..writeAsBytesSync(content, flush: true);
}

Future<void> getPackages(String folder) async {
  final List<String> command = <String>[
    fileSystem.path.join(getFlutterRoot(), 'bin', 'flutter'),
    'pub',
    'get',
  ];
  final ProcessResult result = await processManager.run(command, workingDirectory: folder);
  if (result.exitCode != 0) {
    throw Exception('flutter pub get failed: ${result.stderr}\n${result.stdout}');
  }
}

const String kLocalEngineEnvironment = 'FLUTTER_LOCAL_ENGINE';
const String kLocalEngineHostEnvironment = 'FLUTTER_LOCAL_ENGINE_HOST';
const String kLocalEngineLocation = 'FLUTTER_LOCAL_ENGINE_SRC_PATH';

List<String> getLocalEngineArguments() {
  return <String>[
    if (platform.environment.containsKey(kLocalEngineEnvironment))
      '--local-engine=${platform.environment[kLocalEngineEnvironment]}',
    if (platform.environment.containsKey(kLocalEngineLocation))
      '--local-engine-src-path=${platform.environment[kLocalEngineLocation]}',
    if (platform.environment.containsKey(kLocalEngineHostEnvironment))
      '--local-engine-host=${platform.environment[kLocalEngineHostEnvironment]}',
  ];
}

Future<void> pollForServiceExtensionValue<T>({
  required FlutterTestDriver testDriver,
  required String extension,
  required T continuePollingValue,
  required Matcher matches,
  String valueKey = 'value',
}) async {
  for (int i = 0; i < 10; i++) {
    final Response response = await testDriver.callServiceExtension(extension);
    if (response.json?[valueKey] as T == continuePollingValue) {
      await Future<void>.delayed(const Duration(seconds: 1));
    } else {
      expect(response.json?[valueKey] as T, matches);
      return;
    }
  }
  fail(
    "Did not find expected value for service extension '$extension'. All call"
    " attempts responded with '$continuePollingValue'.",
  );
}

abstract final class AppleTestUtils {
  static const List<String> requiredSymbols = <String>[
    '_kDartIsolateSnapshotData',
    '_kDartIsolateSnapshotInstructions',
    '_kDartVmSnapshotData',
    '_kDartVmSnapshotInstructions',
  ];

  static List<String> getExportedSymbols(String dwarfPath) {
    final ProcessResult nm = processManager.runSync(<String>[
      'nm',
      '--debug-syms', // nm docs: 'Show all symbols, even debugger only'
      '--defined-only',
      '--just-symbol-name',
      dwarfPath,
      '-arch',
      'arm64',
    ]);
    final String nmOutput = (nm.stdout as String).trim();
    return nmOutput.isEmpty ? const <String>[] : nmOutput.split('\n');
  }
}

/// Matcher to be used for [ProcessResult] returned
/// from a process run
///
/// The default for [exitCode] will be 0 while
/// [stdoutPattern] and [stderrPattern] are both optional
class ProcessResultMatcher extends Matcher {
  const ProcessResultMatcher({this.exitCode = 0, this.stdoutPattern, this.stderrPattern});

  /// The expected exit code to get returned from a process run
  final int exitCode;

  /// Substring to find in the process's stdout
  final Pattern? stdoutPattern;

  /// Substring to find in the process's stderr
  final Pattern? stderrPattern;

  @override
  Description describe(Description description) {
    description.add('a process with exit code $exitCode');
    if (stdoutPattern != null) {
      description.add(' and stdout: "$stdoutPattern"');
    }
    if (stderrPattern != null) {
      description.add(' and stderr: "$stderrPattern"');
    }

    return description;
  }

  @override
  bool matches(dynamic item, Map<dynamic, dynamic> matchState) {
    final ProcessResult result = item as ProcessResult;
    bool foundStdout = true;
    bool foundStderr = true;

    final String stdout = result.stdout as String;
    final String stderr = result.stderr as String;
    if (stdoutPattern != null) {
      foundStdout = stdout.contains(stdoutPattern!);
      matchState['stdout'] = stdout;
    } else if (stdout.isNotEmpty) {
      // even if we were not asserting on stdout, show stdout for debug purposes
      matchState['stdout'] = stdout;
    }

    if (stderrPattern != null) {
      foundStderr = stderr.contains(stderrPattern!);
      matchState['stderr'] = stderr;
    } else if (stderr.isNotEmpty) {
      matchState['stderr'] = stderr;
    }

    return result.exitCode == exitCode && foundStdout && foundStderr;
  }

  @override
  Description describeMismatch(
    Object? item,
    Description mismatchDescription,
    Map<dynamic, dynamic> matchState,
    bool verbose,
  ) {
    final ProcessResult result = item! as ProcessResult;

    if (result.exitCode != exitCode) {
      mismatchDescription.add('Actual exitCode was ${result.exitCode}\n');
    }

    if (matchState.containsKey('stdout')) {
      mismatchDescription.add('Actual stdout:\n${matchState["stdout"]}\n');
    }

    if (matchState.containsKey('stderr')) {
      mismatchDescription.add('Actual stderr:\n${matchState["stderr"]}\n');
    }

    return mismatchDescription;
  }
}
