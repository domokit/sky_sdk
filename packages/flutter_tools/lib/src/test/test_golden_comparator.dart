// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';

import 'package:meta/meta.dart';
import 'package:process/process.dart';

import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../convert.dart';
import 'test_compiler.dart';
import 'test_config.dart';

/// Runs a [GoldenFileComparator] (that may depend on `dart:ui`) in a `flutter_tester`.
///
/// The [`goldenFileComparator`](https://api.flutter.dev/flutter/flutter_test/goldenFileComparator.html)
/// is configured using [`flutter_test_config.dart`](https://api.flutter.dev/flutter/flutter_test/flutter_test-library.html)
/// and that file often contains arbitrary Dart code that depends on [`dart:ui`](https://api.flutter.dev/flutter/dart-ui/dart-ui-library.html).
///
/// This proxying comparator creates a minimal application that runs on a
/// `flutter_tester` instance, runs a golden comparison, and then returns the
/// results through [compareGoldens].
///
/// ## Example
///
/// ```dart
/// final comparator = TestGoldenComparator(
///   flutterTesterBinPath: '/path/to/flutter_tester',
///   logger: ...,
///   fileSystem: ...,
///   processManager: ...,
/// )
///
/// final result = await comparator.compare(testUri, bytes, goldenKey);
/// ```
final class TestGoldenComparator {
  /// Creates a [TestGoldenComparator] instance.
  TestGoldenComparator({
    required String flutterTesterBinPath,
    required TestCompiler Function() compilerFactory,
    required Logger logger,
    required FileSystem fileSystem,
    required ProcessManager processManager,
    Map<String, String> environment = const <String, String>{},
  }) : _tempDir = fileSystem.systemTempDirectory.createTempSync('flutter_web_platform.'),
       _flutterTesterBinPath = flutterTesterBinPath,
       _compilerFactory = compilerFactory,
       _logger = logger,
       _fileSystem = fileSystem,
       _processManager = processManager,
       _environment = environment;

  final String _flutterTesterBinPath;
  final Directory _tempDir;
  final Logger _logger;
  final FileSystem _fileSystem;
  final ProcessManager _processManager;
  final Map<String, String> _environment;

  final TestCompiler Function() _compilerFactory;
  late final TestCompiler _compiler = _compilerFactory();

  TestGoldenComparatorProcess? _previousComparator;
  Uri? _previousTestUri;

  /// Closes the comparator.
  ///
  /// Any operation in process is terminated and the comparator can no longer be used.
  Future<void> close() async {
    _tempDir.deleteSync(recursive: true);
    await _compiler.dispose();
    await _previousComparator?.close();
  }

  /// Start golden comparator in a separate process. Start one file per test file
  /// to reduce the overhead of starting `flutter_tester`.
  Future<TestGoldenComparatorProcess?> _processForTestFile(Uri testUri) async {
    if (testUri == _previousTestUri) {
      return _previousComparator!;
    }

    final String bootstrap = TestGoldenComparatorProcess.generateBootstrap(
      _fileSystem.file(testUri),
      testUri,
      logger: _logger,
    );
    final Process? process = await _startProcess(bootstrap);
    if (process == null) {
      return null;
    }
    unawaited(_previousComparator?.close());
    _previousComparator = TestGoldenComparatorProcess(process, logger: _logger);
    _previousTestUri = testUri;

    return _previousComparator!;
  }

  Future<Process?> _startProcess(String testBootstrap) async {
    // Prepare the Dart file that will talk to us and start the test.
    final File listenerFile = (await _tempDir.createTemp('listener')).childFile('listener.dart');
    await listenerFile.writeAsString(testBootstrap);

    final String? output = await _compiler.compile(listenerFile.uri);
    if (output == null) {
      return null;
    }
    final List<String> command = <String>[
      _flutterTesterBinPath,
      '--disable-vm-service',
      '--non-interactive',
      '--packages=${_fileSystem.path.join('.dart_tool', 'package_config.json')}',
      output,
    ];

    return _processManager.start(command, environment: _environment);
  }

  /// Compares the golden file designated by [goldenKey], relative to [testUri], to the provide [bytes].
  Future<TestGoldenComparison> compare(Uri testUri, Uint8List bytes, Uri goldenKey) async {
    final String? result = await _compareGoldens(testUri, bytes, goldenKey, false);
    return switch (result) {
      null => const TestGoldenComparisonDone(matched: true),
      'does not match' => const TestGoldenComparisonDone(matched: false),
      final String error => TestGoldenComparisonError(error: error),
    };
  }

  /// Updates the golden file designated by [goldenKey], relative to [testUri], to the provide [bytes].
  Future<TestGoldenUpdate> update(Uri testUri, Uint8List bytes, Uri goldenKey) async {
    final String? result = await _compareGoldens(testUri, bytes, goldenKey, true);
    return switch (result) {
      null => const TestGoldenUpdateDone(),
      final String error => TestGoldenUpdateError(error: error),
    };
  }

  @useResult
  Future<String?> _compareGoldens(
    Uri testUri,
    Uint8List bytes,
    Uri goldenKey,
    bool? updateGoldens,
  ) async {
    final File imageFile = await (await _tempDir.createTemp(
      'image',
    )).childFile('image').writeAsBytes(bytes);
    final TestGoldenComparatorProcess? process = await _processForTestFile(testUri);
    if (process == null) {
      return 'process was null';
    }

    process.sendCommand(imageFile, goldenKey, updateGoldens);

    final Map<String, dynamic> result = await process.getResponse();
    return (result['success'] as bool)
        ? null
        : ((result['message'] as String?) ?? 'does not match');
  }
}

/// The result of [TestGoldenComparator.compare].
///
/// See also:
///
///   * [TestGoldenComparisonDone]
///   * [TestGoldenComparisonError]
@immutable
sealed class TestGoldenComparison {}

/// A successful comparison that resulted in [matched].
final class TestGoldenComparisonDone implements TestGoldenComparison {
  const TestGoldenComparisonDone({required this.matched});

  /// Whether the bytes matched the file specified.
  ///
  /// A value of `true` is a match, and `false` is a "did not match".
  final bool matched;

  @override
  bool operator ==(Object other) {
    return other is TestGoldenComparisonDone && matched == other.matched;
  }

  @override
  int get hashCode => matched.hashCode;

  @override
  String toString() {
    return 'TestGoldenComparisonDone(matched: $matched)';
  }
}

/// A failed comparison that could not be completed for a reason in [error].
final class TestGoldenComparisonError implements TestGoldenComparison {
  const TestGoldenComparisonError({required this.error});

  /// Why the comparison failed, which should be surfaced to the user as an error.
  final String error;

  @override
  bool operator ==(Object other) {
    return other is TestGoldenComparisonError && error == other.error;
  }

  @override
  int get hashCode => error.hashCode;

  @override
  String toString() {
    return 'TestGoldenComparisonError(error: $error)';
  }
}

/// The result of [TestGoldenComparator.update].
///
/// See also:
///
///   * [TestGoldenUpdateDone]
///   * [TestGoldenUpdateError]
@immutable
sealed class TestGoldenUpdate {}

/// A successful update.
final class TestGoldenUpdateDone implements TestGoldenUpdate {
  const TestGoldenUpdateDone();

  @override
  bool operator ==(Object other) => other is TestGoldenUpdateDone;

  @override
  int get hashCode => (TestGoldenUpdateDone).hashCode;

  @override
  String toString() {
    return 'TestGoldenUpdateDone()';
  }
}

/// A failed update that could not be completed for a reason in [error].
final class TestGoldenUpdateError implements TestGoldenUpdate {
  const TestGoldenUpdateError({required this.error});

  /// Why the comparison failed, which should be surfaced to the user as an error.
  final String error;

  @override
  bool operator ==(Object other) {
    return other is TestGoldenUpdateError && error == other.error;
  }

  @override
  int get hashCode => error.hashCode;

  @override
  String toString() {
    return 'TestGoldenUpdateError(error: $error)';
  }
}

/// Represents a `flutter_tester` process started for golden comparison. Also
/// handles communication with the child process.
class TestGoldenComparatorProcess {
  /// Creates a [TestGoldenComparatorProcess] backed by [process].
  TestGoldenComparatorProcess(this.process, {required Logger logger}) : _logger = logger {
    // Pipe stdout and stderr to printTrace and printError.
    // Also parse stdout as a stream of JSON objects.
    streamIterator = StreamIterator<Map<String, dynamic>>(
      process.stdout
          .transform<String>(utf8.decoder)
          .transform<String>(const LineSplitter())
          .where((String line) {
            logger.printTrace('<<< $line');
            return line.isNotEmpty && line[0] == '{';
          })
          .map<dynamic>(jsonDecode)
          .cast<Map<String, dynamic>>(),
    );

    process.stderr.transform<String>(utf8.decoder).transform<String>(const LineSplitter()).forEach((
      String line,
    ) {
      logger.printError('<<< $line');
    });
  }

  final Logger _logger;
  final Process process;
  late StreamIterator<Map<String, dynamic>> streamIterator;

  Future<void> close() async {
    process.kill();
    await process.exitCode;
  }

  void sendCommand(File imageFile, Uri? goldenKey, bool? updateGoldens) {
    final Object command = jsonEncode(<String, dynamic>{
      'imageFile': imageFile.path,
      'key': goldenKey.toString(),
      'update': updateGoldens,
    });
    _logger.printTrace('Preparing to send command: $command');
    process.stdin.writeln(command);
  }

  Future<Map<String, dynamic>> getResponse() async {
    final bool available = await streamIterator.moveNext();
    assert(available);
    return streamIterator.current;
  }

  static String generateBootstrap(File testFile, Uri testUri, {required Logger logger}) {
    final File? testConfigFile = findTestConfigFile(testFile, logger);
    // Generate comparator process for the file.
    return '''
import 'dart:convert'; // flutter_ignore: dart_convert_import
import 'dart:io'; // flutter_ignore: dart_io_import

import 'package:flutter_test/flutter_test.dart';

${testConfigFile != null ? "import '${Uri.file(testConfigFile.path)}' as test_config;" : ""}

void main() async {
  LocalFileComparator comparator = LocalFileComparator(Uri.parse('$testUri'));
  goldenFileComparator = comparator;

  ${testConfigFile != null ? 'test_config.testExecutable(() async {' : ''}
  final commands = stdin
    .transform<String>(utf8.decoder)
    .transform<String>(const LineSplitter())
    .map<dynamic>(jsonDecode);
  await for (final dynamic command in commands) {
    if (command is Map<String, dynamic>) {
      File imageFile = File(command['imageFile'] as String);
      Uri goldenKey = Uri.parse(command['key'] as String);
      bool update = command['update'] as bool;

      final bytes = await File(imageFile.path).readAsBytes();
      if (update) {
        await goldenFileComparator.update(goldenKey, bytes);
        print(jsonEncode({'success': true}));
      } else {
        try {
          bool success = await goldenFileComparator.compare(bytes, goldenKey);
          print(jsonEncode({'success': success}));
        } on Exception catch (ex) {
          print(jsonEncode({'success': false, 'message': '\$ex'}));
        }
      }
    } else {
      print('object type is not right');
    }
  }
  ${testConfigFile != null ? '});' : ''}
}
    ''';
  }
}
