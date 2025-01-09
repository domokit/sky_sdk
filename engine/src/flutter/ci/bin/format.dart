// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Checks and fixes format on files with changes.
//
// Run with --help for usage.

import 'dart:ffi';
import 'dart:io';

import 'package:args/args.dart';
import 'package:meta/meta.dart';
import 'package:path/path.dart' as path;
import 'package:process/process.dart';
import 'package:process_runner/process_runner.dart';

const engineSubPath = 'engine/src/flutter';

class FormattingException implements Exception {
  FormattingException(this.message, [this.result]);

  final String message;
  final ProcessResult? result;

  @override
  String toString() {
    final StringBuffer output = StringBuffer(runtimeType.toString());
    output.write(': $message');
    final String? stderr = result?.stderr as String?;
    if (stderr?.isNotEmpty ?? false) {
      output.write(':\n$stderr');
    }
    return output.toString();
  }
}

enum MessageType { message, error, warning }

enum FormatCheck {
  dart,
  gn,
  java,
  python,
  whitespace,
  header,
  // Run clang after the header check.
  clang,
}

FormatCheck nameToFormatCheck(String name) {
  switch (name) {
    case 'clang':
      return FormatCheck.clang;
    case 'dart':
      return FormatCheck.dart;
    case 'gn':
      return FormatCheck.gn;
    case 'java':
      return FormatCheck.java;
    case 'python':
      return FormatCheck.python;
    case 'whitespace':
      return FormatCheck.whitespace;
    case 'header':
      return FormatCheck.header;
    default:
      throw FormattingException('Unknown FormatCheck type $name');
  }
}

String formatCheckToName(FormatCheck check) {
  switch (check) {
    case FormatCheck.clang:
      return 'C++/ObjC/Shader';
    case FormatCheck.dart:
      return 'Dart';
    case FormatCheck.gn:
      return 'GN';
    case FormatCheck.java:
      return 'Java';
    case FormatCheck.python:
      return 'Python';
    case FormatCheck.whitespace:
      return 'Trailing whitespace';
    case FormatCheck.header:
      return 'Header guards';
  }
}

List<String> formatCheckNames() {
  return FormatCheck.values.map<String>((FormatCheck check) => check.name).toList();
}

Future<String> _runGit(
  List<String> args,
  ProcessRunner processRunner, {
  bool failOk = false,
}) async {
  final ProcessRunnerResult result = await processRunner.runProcess(<String>[
    'git',
    ...args,
  ], failOk: failOk);
  return result.stdout;
}

typedef MessageCallback = void Function(String? message, {MessageType type});

/// Base class for format checkers.
///
/// Provides services that all format checkers need.
abstract class FormatChecker {
  FormatChecker({
    ProcessManager processManager = const LocalProcessManager(),
    required this.baseGitRef,
    required this.repoDir,
    this.allFiles = false,
    this.messageCallback,
  }) : _processRunner = ProcessRunner(
         defaultWorkingDirectory: repoDir,
         processManager: processManager,
       );

  /// Factory method that creates subclass format checkers based on the type of check.
  factory FormatChecker.ofType(
    FormatCheck check, {
    ProcessManager processManager = const LocalProcessManager(),
    required String baseGitRef,
    required Directory repoDir,
    required Directory srcDir,
    bool allFiles = false,
    MessageCallback? messageCallback,
  }) {
    switch (check) {
      case FormatCheck.clang:
        return ClangFormatChecker(
          processManager: processManager,
          baseGitRef: baseGitRef,
          repoDir: repoDir,
          srcDir: srcDir,
          allFiles: allFiles,
          messageCallback: messageCallback,
        );
      case FormatCheck.dart:
        return DartFormatChecker(
          processManager: processManager,
          baseGitRef: baseGitRef,
          repoDir: repoDir,
          allFiles: allFiles,
          messageCallback: messageCallback,
        );
      case FormatCheck.gn:
        return GnFormatChecker(
          processManager: processManager,
          baseGitRef: baseGitRef,
          repoDir: repoDir,
          allFiles: allFiles,
          messageCallback: messageCallback,
        );
      case FormatCheck.java:
        return JavaFormatChecker(
          processManager: processManager,
          baseGitRef: baseGitRef,
          repoDir: repoDir,
          srcDir: srcDir,
          allFiles: allFiles,
          messageCallback: messageCallback,
        );
      case FormatCheck.python:
        return PythonFormatChecker(
          processManager: processManager,
          baseGitRef: baseGitRef,
          repoDir: repoDir,
          allFiles: allFiles,
          messageCallback: messageCallback,
        );
      case FormatCheck.whitespace:
        return WhitespaceFormatChecker(
          processManager: processManager,
          baseGitRef: baseGitRef,
          repoDir: repoDir,
          allFiles: allFiles,
          messageCallback: messageCallback,
        );
      case FormatCheck.header:
        return HeaderFormatChecker(
          processManager: processManager,
          baseGitRef: baseGitRef,
          repoDir: repoDir,
          allFiles: allFiles,
          messageCallback: messageCallback,
        );
    }
  }

  final ProcessRunner _processRunner;
  final Directory repoDir;
  final bool allFiles;
  MessageCallback? messageCallback;
  final String baseGitRef;

  /// Override to provide format checking for a specific type.
  Future<bool> checkFormatting();

  /// Override to provide format fixing for a specific type.
  Future<bool> fixFormatting();

  @protected
  void message(String? string) => messageCallback?.call(string, type: MessageType.message);

  @protected
  void error(String string) => messageCallback?.call(string, type: MessageType.error);

  @protected
  Future<String> runGit(List<String> args) async => _runGit(args, _processRunner);

  /// Converts a given raw string of code units to a stream that yields those
  /// code units.
  ///
  /// Uses to convert the stdout of a previous command into an input stream for
  /// the next command.
  @protected
  Stream<List<int>> codeUnitsAsStream(List<int>? input) async* {
    if (input != null) {
      yield input;
    }
  }

  @protected
  Future<bool> applyPatch(List<String> patches) async {
    final ProcessPool patchPool = ProcessPool(
      processRunner: _processRunner,
      printReport: namedReport('patch'),
    );
    final List<WorkerJob> jobs =
        patches.map<WorkerJob>((String patch) {
          return WorkerJob(<String>[
            'git',
            'apply',
            '--ignore-space-change',
          ], stdinRaw: codeUnitsAsStream(patch.codeUnits));
        }).toList();
    final List<WorkerJob> completedJobs = await patchPool.runToCompletion(jobs);
    if (patchPool.failedJobs != 0) {
      error(
        '${patchPool.failedJobs} patch${patchPool.failedJobs > 1 ? 'es' : ''} '
        'failed to apply.',
      );
      completedJobs
          .where((WorkerJob job) => job.result.exitCode != 0)
          .map<String>((WorkerJob job) => job.result.output)
          .forEach(message);
    }
    return patchPool.failedJobs == 0;
  }

  /// Gets the list of files to operate on.
  ///
  /// If [allFiles] is true, then returns all git controlled files in the repo
  /// of the given types.
  ///
  /// If [allFiles] is false, then only return those files of the given types
  /// that have changed between the current working tree and the [baseGitRef].
  @protected
  Future<List<String>> getFileList(List<String> types) async {
    String output;
    if (allFiles) {
      output = await runGit(<String>['ls-files', '--', ...types]);
    } else {
      output = await runGit(<String>[
        'diff',
        '-U0',
        '--no-color',
        '--diff-filter=d',
        '--name-only',
        baseGitRef,
        '--',
        ...types,
      ]);
    }
    return [
      ...output
          .split('\n')
          .where(
            (String line) =>
                line.isNotEmpty && !line.contains('third_party') && line.contains(engineSubPath),
          ),
    ];
  }

  /// Generates a reporting function to supply to ProcessRunner to use instead
  /// of the default reporting function.
  @protected
  ProcessPoolProgressReporter namedReport(String name) {
    return (int total, int completed, int inProgress, int pending, int failed) {
      final String percent =
          total == 0 ? '100' : ((100 * completed) ~/ total).toString().padLeft(3);
      final String completedStr = completed.toString().padLeft(3);
      final String totalStr = total.toString().padRight(3);
      final String inProgressStr = inProgress.toString().padLeft(2);
      final String pendingStr = pending.toString().padLeft(3);
      final String failedStr = failed.toString().padLeft(3);

      stdout.write(
        '$name Jobs: $percent% done, '
        '$completedStr/$totalStr completed, '
        '$inProgressStr in progress, '
        '$pendingStr pending, '
        '$failedStr failed.${' ' * 20}\r',
      );
    };
  }

  /// Clears the last printed report line so garbage isn't left on the terminal.
  @protected
  void reportDone() {
    stdout.write('\r${' ' * 100}\r');
  }
}

/// Checks and formats C++/ObjC/Shader files using clang-format.
class ClangFormatChecker extends FormatChecker {
  ClangFormatChecker({
    super.processManager,
    required super.baseGitRef,
    required super.repoDir,
    required Directory srcDir,
    super.allFiles,
    super.messageCallback,
  }) {
    final clangOs = switch (Abi.current()) {
      Abi.linuxArm64 => 'linux-arm64',
      Abi.linuxX64 => 'linux-x64',
      Abi.macosArm64 => 'mac-arm64',
      Abi.macosX64 => 'mac-x64',
      Abi.windowsX64 => 'windows-x64',
      (_) =>
        throw FormattingException(
          "Unknown operating system: don't know how to run clang-format here.",
        ),
    };
    clangFormat = File(
      path.join(
        srcDir.absolute.path,
        'flutter',
        'buildtools',
        clangOs,
        'clang',
        'bin',
        'clang-format',
      ),
    );
  }

  late final File clangFormat;

  @override
  Future<bool> checkFormatting() async {
    final List<String> failures = await _getCFormatFailures();
    failures.map(stdout.writeln);
    return failures.isEmpty;
  }

  @override
  Future<bool> fixFormatting() async {
    message('Fixing C++/ObjC/Shader formatting...');
    final List<String> failures = await _getCFormatFailures(fixing: true);
    if (failures.isEmpty) {
      return true;
    }
    return applyPatch(failures);
  }

  Future<String> _getClangFormatVersion() async {
    final ProcessRunnerResult result = await _processRunner.runProcess(<String>[
      clangFormat.path,
      '--version',
    ]);
    return result.stdout.trim();
  }

  Future<List<String>> _getCFormatFailures({bool fixing = false}) async {
    message('Checking C++/ObjC/Shader formatting...');
    const List<String> clangFiletypes = <String>[
      '*.c',
      '*.cc',
      '*.cxx',
      '*.cpp',
      '*.h',
      '*.m',
      '*.mm',
      '*.glsl',
      '*.hlsl',
      '*.comp',
      '*.tese',
      '*.tesc',
      '*.vert',
      '*.frag',
    ];
    final List<String> files = await getFileList(clangFiletypes);
    if (files.isEmpty) {
      message('No C++/ObjC/Shader files with changes, skipping C++/ObjC/Shader format check.');
      return <String>[];
    }
    if (verbose) {
      message('Using ${await _getClangFormatVersion()}');
    }
    final List<WorkerJob> clangJobs = <WorkerJob>[];
    for (final String file in files) {
      if (file.trim().isEmpty) {
        continue;
      }
      clangJobs.add(WorkerJob(<String>[clangFormat.path, '--style=file', file.trim()]));
    }
    final ProcessPool clangPool = ProcessPool(
      processRunner: _processRunner,
      printReport: namedReport('clang-format'),
    );
    final Stream<WorkerJob> completedClangFormats = clangPool.startWorkers(clangJobs);
    final List<WorkerJob> diffJobs = <WorkerJob>[];
    await for (final WorkerJob completedJob in completedClangFormats) {
      if (completedJob.result.exitCode == 0) {
        diffJobs.add(
          WorkerJob(<String>[
            'git',
            'diff',
            '--no-index',
            '--no-color',
            '--ignore-cr-at-eol',
            '--',
            completedJob.command.last,
            '-',
          ], stdinRaw: codeUnitsAsStream(completedJob.result.stdoutRaw)),
        );
      } else {
        final String formatterCommand = completedJob.command.join(' ');
        error(
          "Formatter command '$formatterCommand' failed with exit code "
          '${completedJob.result.exitCode}. Command output follows:\n\n'
          '${completedJob.result.output}',
        );
      }
    }
    final ProcessPool diffPool = ProcessPool(
      processRunner: _processRunner,
      printReport: namedReport('diff'),
    );
    final List<WorkerJob> completedDiffs = await diffPool.runToCompletion(diffJobs);
    final Iterable<WorkerJob> failed = completedDiffs.where((WorkerJob job) {
      return job.result.exitCode != 0;
    });
    reportDone();
    if (failed.isNotEmpty) {
      final bool plural = failed.length > 1;
      if (fixing) {
        message(
          'Fixing ${failed.length} C++/ObjC/Shader file${plural ? 's' : ''}'
          ' which ${plural ? 'were' : 'was'} formatted incorrectly.',
        );
      } else {
        error(
          'Found ${failed.length} C++/ObjC/Shader file${plural ? 's' : ''}'
          ' which ${plural ? 'were' : 'was'} formatted incorrectly.',
        );
        stdout.writeln('To fix, run `et format` or:');
        stdout.writeln();
        stdout.writeln('git apply <<DONE');
        for (final WorkerJob job in failed) {
          stdout.write(
            job.result.stdout
                .replaceFirst('b/-', 'b/${job.command[job.command.length - 2]}')
                .replaceFirst('b/-', 'b/${job.command[job.command.length - 2]}'),
          );
        }
        stdout.writeln('DONE');
        stdout.writeln();
      }
    } else {
      message(
        'Completed checking ${diffJobs.length} C++/ObjC/Shader files with no formatting problems.',
      );
    }
    return failed.map<String>((WorkerJob job) {
      return job.result.stdout
          .replaceFirst('b/-', 'b/${job.command[job.command.length - 2]}')
          .replaceFirst('b/-', 'b/${job.command[job.command.length - 2]}');
    }).toList();
  }
}

/// Checks the format of Java files using the Google Java format checker.
class JavaFormatChecker extends FormatChecker {
  JavaFormatChecker({
    super.processManager,
    required super.baseGitRef,
    required super.repoDir,
    required Directory srcDir,
    super.allFiles,
    super.messageCallback,
  }) {
    googleJavaFormatJar = File(
      path.absolute(
        path.join(
          srcDir.absolute.path,
          'flutter',
          'third_party',
          'android_tools',
          'google-java-format',
          'google-java-format-1.7-all-deps.jar',
        ),
      ),
    );
    // Use java from the checkout to avoid contributors needing to install java.
    final File hermetic = hermeticJava(srcDir);
    // If for some reason the hermetic java doesn't exist, fall back to the system java.
    javaExe = hermetic.existsSync() ? hermetic.path : 'java';
  }

  /// Returns the path to the java executable in the flutter repository.
  static File hermeticJava(Directory srcDir) {
    final List<String> javaPath = <String>[
      srcDir.absolute.path,
      'flutter',
      'third_party',
      'java',
      'openjdk',
    ];
    if (Platform.isMacOS) {
      javaPath.add('Contents');
      javaPath.add('Home');
    }
    javaPath.add('bin');
    javaPath.add(Platform.isWindows ? 'java.exe' : 'java');
    return File(path.joinAll(javaPath));
  }

  late final String javaExe;
  late final File googleJavaFormatJar;

  // String to return if java formatting cant check java code for any reson.
  static const String _javaFormatErrorString = 'Java Formatting Error';

  Future<String> _getGoogleJavaFormatVersion() async {
    final ProcessRunnerResult result = await _processRunner.runProcess(<String>[
      javaExe,
      '-jar',
      googleJavaFormatJar.path,
      '--version',
    ]);
    return result.stderr.trim();
  }

  @override
  Future<bool> checkFormatting() async {
    final List<String> failures = await _getJavaFormatFailures();
    failures.map(stdout.writeln);
    return failures.isEmpty;
  }

  @override
  Future<bool> fixFormatting() async {
    message('Fixing Java formatting...');
    final List<String> failures = await _getJavaFormatFailures(fixing: true);
    if (failures.isEmpty) {
      return true;
    }
    if (failures.length == 1 && failures.first == _javaFormatErrorString) {
      // _javaFormatErrorString is a string that indicates java formatting failed
      // without creating a patch that can be applied.
      return false;
    } else {
      return applyPatch(failures);
    }
  }

  Future<String> _getJavaVersion() async {
    final ProcessRunnerResult result = await _processRunner.runProcess(<String>[
      javaExe,
      '-version',
    ]);
    return result.stderr.trim().split('\n')[0];
  }

  Future<List<String>> _getJavaFormatFailures({bool fixing = false}) async {
    message('Checking Java formatting...');
    final List<WorkerJob> formatJobs = <WorkerJob>[];
    final List<String> files = await getFileList(<String>['*.java']);
    if (files.isEmpty) {
      message('No Java files with changes, skipping Java format check.');
      return <String>[];
    }
    String javaVersion = '<unknown>';
    String javaFormatVersion = '<unknown>';
    try {
      javaVersion = await _getJavaVersion();
    } on ProcessRunnerException {
      if (!_processRunner.processManager.canRun(javaExe)) {
        error(
          'Cannot find Java ($javaExe). '
          'Skipping Java format check.',
        );
        return const <String>[_javaFormatErrorString];
      }
      error('Cannot run Java ($javaExe), skipping Java file formatting!');
      return const <String>[_javaFormatErrorString];
    }
    try {
      javaFormatVersion = await _getGoogleJavaFormatVersion();
    } on ProcessRunnerException {
      error('Cannot find google-java-format, skipping Java format check.');
      return const <String>[_javaFormatErrorString];
    }
    if (verbose) {
      message('Using $javaFormatVersion with Java $javaVersion');
    }
    for (final String file in files) {
      if (file.trim().isEmpty) {
        continue;
      }
      formatJobs.add(WorkerJob(<String>[javaExe, '-jar', googleJavaFormatJar.path, file.trim()]));
    }
    final ProcessPool formatPool = ProcessPool(
      processRunner: _processRunner,
      printReport: namedReport('Java format'),
    );
    final Stream<WorkerJob> completedJavaFormats = formatPool.startWorkers(formatJobs);
    final List<WorkerJob> diffJobs = <WorkerJob>[];
    await for (final WorkerJob completedJob in completedJavaFormats) {
      if (completedJob.result.exitCode == 0) {
        diffJobs.add(
          WorkerJob(<String>[
            'git',
            'diff',
            '--no-index',
            '--no-color',
            '--ignore-cr-at-eol',
            '--',
            completedJob.command.last,
            '-',
          ], stdinRaw: codeUnitsAsStream(completedJob.result.stdoutRaw)),
        );
      } else {
        final String formatterCommand = completedJob.command.join(' ');
        error(
          "Formatter command '$formatterCommand' failed with exit code "
          '${completedJob.result.exitCode}. Command output follows:\n\n'
          '${completedJob.result.output}',
        );
      }
    }
    final ProcessPool diffPool = ProcessPool(
      processRunner: _processRunner,
      printReport: namedReport('diff'),
    );
    final List<WorkerJob> completedDiffs = await diffPool.runToCompletion(diffJobs);
    final Iterable<WorkerJob> failed = completedDiffs.where((WorkerJob job) {
      return job.result.exitCode != 0;
    });
    reportDone();
    if (failed.isNotEmpty) {
      final bool plural = failed.length > 1;
      if (fixing) {
        message(
          'Fixing ${failed.length} Java file${plural ? 's' : ''}'
          ' which ${plural ? 'were' : 'was'} formatted incorrectly.',
        );
      } else {
        error(
          'Found ${failed.length} Java file${plural ? 's' : ''}'
          ' which ${plural ? 'were' : 'was'} formatted incorrectly.',
        );
        stdout.writeln('To fix, run `et format` or:');
        stdout.writeln();
        stdout.writeln('git apply <<DONE');
        for (final WorkerJob job in failed) {
          stdout.write(
            job.result.stdout
                .replaceFirst('b/-', 'b/${job.command[job.command.length - 2]}')
                .replaceFirst('b/-', 'b/${job.command[job.command.length - 2]}'),
          );
        }
        stdout.writeln('DONE');
        stdout.writeln();
      }
    } else {
      message('Completed checking ${diffJobs.length} Java files with no formatting problems.');
    }
    return failed.map<String>((WorkerJob job) {
      return job.result.stdout
          .replaceFirst('b/-', 'b/${job.command[job.command.length - 2]}')
          .replaceFirst('b/-', 'b/${job.command[job.command.length - 2]}');
    }).toList();
  }
}

/// Checks the format of any BUILD.gn files using the "gn format" command.
class GnFormatChecker extends FormatChecker {
  GnFormatChecker({
    super.processManager,
    required super.baseGitRef,
    required Directory repoDir,
    super.allFiles,
    super.messageCallback,
  }) : super(repoDir: repoDir) {
    gnBinary = File(
      path.join(engineDir(repoDir).path, 'third_party', 'gn', Platform.isWindows ? 'gn.exe' : 'gn'),
    );
  }

  late final File gnBinary;

  @override
  Future<bool> checkFormatting() async {
    message('Checking GN formatting...');
    return (await _runGnCheck(fixing: false)) == 0;
  }

  @override
  Future<bool> fixFormatting() async {
    message('Fixing GN formatting...');
    await _runGnCheck(fixing: true);
    // The GN script shouldn't fail when fixing errors.
    return true;
  }

  Future<int> _runGnCheck({required bool fixing}) async {
    final List<String> filesToCheck = await getFileList(<String>['*.gn', '*.gni']);

    final List<String> cmd = <String>[gnBinary.path, 'format', if (!fixing) '--stdin'];
    final List<WorkerJob> jobs = <WorkerJob>[];
    for (final String file in filesToCheck) {
      if (fixing) {
        jobs.add(WorkerJob(<String>[...cmd, file], name: <String>[...cmd, file].join(' ')));
      } else {
        final WorkerJob job = WorkerJob(
          cmd,
          stdinRaw: codeUnitsAsStream(
            File(path.join(repoDir.absolute.path, file)).readAsBytesSync(),
          ),
          name: <String>[...cmd, file].join(' '),
        );
        jobs.add(job);
      }
    }
    final ProcessPool gnPool = ProcessPool(
      processRunner: _processRunner,
      printReport: namedReport('gn format'),
    );
    final Stream<WorkerJob> completedJobs = gnPool.startWorkers(jobs);
    final List<WorkerJob> diffJobs = <WorkerJob>[];
    await for (final WorkerJob completedJob in completedJobs) {
      if (completedJob.result.exitCode == 0) {
        diffJobs.add(
          WorkerJob(<String>[
            'git',
            'diff',
            '--no-index',
            '--no-color',
            '--ignore-cr-at-eol',
            '--',
            completedJob.name.split(' ').last,
            '-',
          ], stdinRaw: codeUnitsAsStream(completedJob.result.stdoutRaw)),
        );
      } else {
        final String formatterCommand = completedJob.command.join(' ');
        error(
          "Formatter command '$formatterCommand' failed with exit code "
          '${completedJob.result.exitCode}. Command output follows:\n\n'
          '${completedJob.result.output}',
        );
      }
    }
    final ProcessPool diffPool = ProcessPool(
      processRunner: _processRunner,
      printReport: namedReport('diff'),
    );
    final List<WorkerJob> completedDiffs = await diffPool.runToCompletion(diffJobs);
    final Iterable<WorkerJob> failed = completedDiffs.where((WorkerJob job) {
      return job.result.exitCode != 0;
    });
    reportDone();
    if (failed.isNotEmpty) {
      final bool plural = failed.length > 1;
      if (fixing) {
        message(
          'Fixed ${failed.length} GN file${plural ? 's' : ''}'
          ' which ${plural ? 'were' : 'was'} formatted incorrectly.',
        );
      } else {
        error(
          'Found ${failed.length} GN file${plural ? 's' : ''}'
          ' which ${plural ? 'were' : 'was'} formatted incorrectly.',
        );
        stdout.writeln('To fix, run `et format` or:');
        stdout.writeln();
        stdout.writeln('git apply <<DONE');
        for (final WorkerJob job in failed) {
          stdout.write(
            job.result.stdout
                .replaceFirst('b/-', 'b/${job.command[job.command.length - 2]}')
                .replaceFirst('b/-', 'b/${job.command[job.command.length - 2]}'),
          );
        }
        stdout.writeln('DONE');
        stdout.writeln();
      }
    } else {
      message(
        'Completed checking ${completedDiffs.length} GN files with no '
        'formatting problems.',
      );
    }
    return failed.length;
  }
}

/// Checks the format of any .dart files using the "dart format" command.
class DartFormatChecker extends FormatChecker {
  DartFormatChecker({
    super.processManager,
    required super.baseGitRef,
    required Directory repoDir,
    super.allFiles,
    super.messageCallback,
  }) : super(repoDir: repoDir) {
    // $ENGINE/flutter/third_party/dart/tools/sdks/dart-sdk/bin/dart
    _dartBin = path.join(
      engineDir(repoDir).parent.path,
      'flutter',
      'third_party',
      'dart',
      'tools',
      'sdks',
      'dart-sdk',
      'bin',
      Platform.isWindows ? 'dart.exe' : 'dart',
    );
  }

  late final String _dartBin;

  @override
  Future<bool> checkFormatting() async {
    message('Checking Dart formatting...');
    return (await _runDartFormat(fixing: false)) == 0;
  }

  @override
  Future<bool> fixFormatting() async {
    message('Fixing Dart formatting...');
    return (await _runDartFormat(fixing: true)) == 0;
  }

  Future<int> _runDartFormat({required bool fixing}) async {
    final List<String> filesToCheck = await getFileList(<String>['*.dart']);

    final List<String> cmd = <String>[
      _dartBin,
      'format',
      '--set-exit-if-changed',
      '--show=none',
      if (!fixing) '--output=show',
      if (fixing) '--output=write',
    ];
    final List<WorkerJob> jobs = <WorkerJob>[];
    for (final String file in filesToCheck) {
      jobs.add(WorkerJob(<String>[...cmd, file]));
    }
    final ProcessPool dartFmt = ProcessPool(
      processRunner: _processRunner,
      printReport: namedReport('dart format'),
    );

    Iterable<WorkerJob> incorrect;
    final List<WorkerJob> errorJobs = [];
    if (!fixing) {
      final Stream<WorkerJob> completedJobs = dartFmt.startWorkers(jobs);
      final List<WorkerJob> diffJobs = <WorkerJob>[];
      await for (final WorkerJob completedJob in completedJobs) {
        if (completedJob.result.exitCode != 0 && completedJob.result.exitCode != 1) {
          // The formatter had a problem formatting the file.
          errorJobs.add(completedJob);
        } else if (completedJob.result.exitCode == 1) {
          diffJobs.add(
            WorkerJob(<String>[
              'git',
              'diff',
              '--no-index',
              '--no-color',
              '--ignore-cr-at-eol',
              '--',
              completedJob.command.last,
              '-',
            ], stdinRaw: codeUnitsAsStream(completedJob.result.stdoutRaw)),
          );
        }
      }
      final ProcessPool diffPool = ProcessPool(
        processRunner: _processRunner,
        printReport: namedReport('diff'),
      );
      final List<WorkerJob> completedDiffs = await diffPool.runToCompletion(diffJobs);
      incorrect = completedDiffs.where((WorkerJob job) {
        return job.result.exitCode != 0;
      });
    } else {
      final List<WorkerJob> completedJobs = await dartFmt.runToCompletion(jobs);
      final List<WorkerJob> incorrectJobs = incorrect = [];
      for (final WorkerJob job in completedJobs) {
        if (job.result.exitCode != 0 && job.result.exitCode != 1) {
          // The formatter had a problem formatting the file.
          errorJobs.add(job);
        } else if (job.result.exitCode == 1) {
          incorrectJobs.add(job);
        }
      }
    }

    reportDone();

    if (incorrect.isNotEmpty) {
      final bool plural = incorrect.length > 1;
      if (fixing) {
        message(
          'Fixing ${incorrect.length} dart file${plural ? 's' : ''}'
          ' which ${plural ? 'were' : 'was'} formatted incorrectly.',
        );
      } else {
        error(
          'Found ${incorrect.length} Dart file${plural ? 's' : ''}'
          ' which ${plural ? 'were' : 'was'} formatted incorrectly.',
        );
        stdout.writeln();
        stdout.writeln('To fix, run `et format` or:');
        stdout.writeln();
        stdout.writeln('git apply <<DONE');
        for (final WorkerJob job in incorrect) {
          stdout.write(
            job.result.stdout
                .replaceFirst('b/-', 'b/${job.command[job.command.length - 2]}')
                .replaceFirst('b/-', 'b/${job.command[job.command.length - 2]}')
                .replaceFirst(
                  RegExp('\\+Formatted \\d+ files? \\(\\d+ changed\\) in \\d+.\\d+ seconds.\n'),
                  '',
                ),
          );
        }
        stdout.writeln('DONE');
        stdout.writeln();
      }
      _printErrorJobs(errorJobs);
    } else if (errorJobs.isNotEmpty) {
      _printErrorJobs(errorJobs);
    } else {
      message('All dart files formatted correctly.');
    }
    return fixing ? errorJobs.length : (incorrect.length + errorJobs.length);
  }

  void _printErrorJobs(List<WorkerJob> errorJobs) {
    if (errorJobs.isNotEmpty) {
      final bool plural = errorJobs.length > 1;
      error('The formatter failed to run on ${errorJobs.length} Dart file${plural ? 's' : ''}.');
      stdout.writeln();
      for (final WorkerJob job in errorJobs) {
        stdout.writeln('--> ${job.command.last} produced the following error:');
        stdout.write(job.result.stderr);
        stdout.writeln();
      }
    }
  }
}

/// Checks the format of any .py files using the "yapf" command.
class PythonFormatChecker extends FormatChecker {
  PythonFormatChecker({
    super.processManager,
    required super.baseGitRef,
    required Directory repoDir,
    super.allFiles,
    super.messageCallback,
  }) : super(repoDir: repoDir) {
    yapfBin = File(
      path.join(engineDir(repoDir).path, 'tools', Platform.isWindows ? 'yapf.bat' : 'yapf.sh'),
    );
    _yapfStyle = File(path.join(engineDir(repoDir).path, '.style.yapf'));
  }

  late final File yapfBin;
  late final File _yapfStyle;

  @override
  Future<bool> checkFormatting() async {
    message('Checking Python formatting...');
    return (await _runYapfCheck(fixing: false)) == 0;
  }

  @override
  Future<bool> fixFormatting() async {
    message('Fixing Python formatting...');
    await _runYapfCheck(fixing: true);
    // The yapf script shouldn't fail when fixing errors.
    return true;
  }

  Future<int> _runYapfCheck({required bool fixing}) async {
    final List<String> filesToCheck = <String>[
      ...await getFileList(<String>['*.py']),
      // Always include flutter/tools/gn.
      '${engineDir(repoDir).path}/tools/gn',
    ];

    final List<String> cmd = <String>[
      yapfBin.path,
      '--style',
      _yapfStyle.path,
      if (!fixing) '--diff',
      if (fixing) '--in-place',
    ];
    final List<WorkerJob> jobs = <WorkerJob>[];
    for (final String file in filesToCheck) {
      jobs.add(WorkerJob(<String>[...cmd, file]));
    }
    final ProcessPool yapfPool = ProcessPool(
      processRunner: _processRunner,
      printReport: namedReport('python format'),
    );
    final List<WorkerJob> completedJobs = await yapfPool.runToCompletion(jobs);
    reportDone();
    final List<String> incorrect = <String>[];
    for (final WorkerJob job in completedJobs) {
      if (job.result.exitCode == 1) {
        incorrect.add('  ${job.command.last}\n${job.result.output}');
      }
    }
    if (incorrect.isNotEmpty) {
      final bool plural = incorrect.length > 1;
      if (fixing) {
        message(
          'Fixed ${incorrect.length} python file${plural ? 's' : ''}'
          ' which ${plural ? 'were' : 'was'} formatted incorrectly.',
        );
      } else {
        error(
          'Found ${incorrect.length} python file${plural ? 's' : ''}'
          ' which ${plural ? 'were' : 'was'} formatted incorrectly:',
        );
        stdout.writeln('To fix, run `et format` or:');
        stdout.writeln();
        stdout.writeln('git apply <<DONE');
        incorrect.forEach(stdout.writeln);
        stdout.writeln('DONE');
        stdout.writeln();
      }
    } else {
      message('All python files formatted correctly.');
    }
    return incorrect.length;
  }
}

@immutable
class _GrepResult {
  const _GrepResult(this.file, [this.hits = const <String>[], this.lineNumbers = const <int>[]]);
  bool get isEmpty => hits.isEmpty && lineNumbers.isEmpty;
  final File file;
  final List<String> hits;
  final List<int> lineNumbers;
}

/// Checks for trailing whitspace in Dart files.
class WhitespaceFormatChecker extends FormatChecker {
  WhitespaceFormatChecker({
    super.processManager,
    required super.baseGitRef,
    required super.repoDir,
    super.allFiles,
    super.messageCallback,
  });

  @override
  Future<bool> checkFormatting() async {
    final List<File> failures = await _getWhitespaceFailures();
    return failures.isEmpty;
  }

  static final RegExp trailingWsRegEx = RegExp(r'[ \t]+$', multiLine: true);

  @override
  Future<bool> fixFormatting() async {
    final List<File> failures = await _getWhitespaceFailures();
    if (failures.isNotEmpty) {
      for (final File file in failures) {
        stderr.writeln('Fixing $file');
        String contents = file.readAsStringSync();
        contents = contents.replaceAll(trailingWsRegEx, '');
        file.writeAsStringSync(contents);
      }
    }
    return true;
  }

  static _GrepResult _hasTrailingWhitespace(File file) {
    final List<String> hits = <String>[];
    final List<int> lineNumbers = <int>[];
    int lineNumber = 0;
    for (final String line in file.readAsLinesSync()) {
      if (trailingWsRegEx.hasMatch(line)) {
        hits.add(line);
        lineNumbers.add(lineNumber);
      }
      lineNumber++;
    }
    if (hits.isEmpty) {
      return _GrepResult(file);
    }
    return _GrepResult(file, hits, lineNumbers);
  }

  Iterable<_GrepResult> _whereHasTrailingWhitespace(Iterable<File> files) {
    return files.map(_hasTrailingWhitespace);
  }

  Future<List<File>> _getWhitespaceFailures() async {
    final List<String> files = await getFileList(<String>[
      '*.c',
      '*.cc',
      '*.cpp',
      '*.cxx',
      '*.dart',
      '*.gn',
      '*.gni',
      '*.gradle',
      '*.h',
      '*.java',
      '*.json',
      '*.m',
      '*.mm',
      '*.py',
      '*.sh',
      '*.yaml',
    ]);
    if (files.isEmpty) {
      message('No files that differ, skipping whitespace check.');
      return <File>[];
    }
    message(
      'Checking for trailing whitespace on ${files.length} source '
      'file${files.length > 1 ? 's' : ''}...',
    );

    final ProcessPoolProgressReporter reporter = namedReport('whitespace');
    final List<_GrepResult> found = <_GrepResult>[];
    final int total = files.length;
    int completed = 0;
    int inProgress = Platform.numberOfProcessors;
    int pending = total;
    int failed = 0;
    for (final _GrepResult result in _whereHasTrailingWhitespace(
      files.map<File>((String file) => File(path.join(repoDir.absolute.path, file))),
    )) {
      if (result.isEmpty) {
        completed++;
      } else {
        failed++;
        found.add(result);
      }
      pending--;
      inProgress = pending < Platform.numberOfProcessors ? pending : Platform.numberOfProcessors;
      reporter(total, completed, inProgress, pending, failed);
    }
    reportDone();
    if (found.isNotEmpty) {
      error('Whitespace check failed. The following files have trailing spaces:');
      for (final _GrepResult result in found) {
        for (int i = 0; i < result.hits.length; ++i) {
          message('  ${result.file.path}:${result.lineNumbers[i]}:${result.hits[i]}');
        }
      }
    } else {
      message('No trailing whitespace found.');
    }
    return found.map<File>((_GrepResult result) => result.file).toList();
  }
}

final class HeaderFormatChecker extends FormatChecker {
  HeaderFormatChecker({
    required super.baseGitRef,
    required super.repoDir,
    super.processManager,
    super.allFiles,
    super.messageCallback,
  });

  // $ENGINE/flutter/third_party/dart/tools/sdks/dart-sdk/bin/dart
  late final String _dartBin = path.join(
    engineDir(repoDir).path,
    'third_party',
    'dart',
    'tools',
    'sdks',
    'dart-sdk',
    'bin',
    'dart',
  );

  // $ENGINE/src/flutter/tools/bin/main.dart
  late final String _headerGuardCheckBin = path.join(
    engineDir(repoDir).path,
    'tools',
    'header_guard_check',
    'bin',
    'main.dart',
  );

  @override
  Future<bool> checkFormatting() async {
    final List<String> include = <String>[];
    if (!allFiles) {
      include.addAll(await getFileList(<String>['*.h']));
      if (include.isEmpty) {
        message('No header files with changes, skipping header guard check.');
        return true;
      }
    }
    final List<String> args = <String>[
      _dartBin,
      _headerGuardCheckBin,
      ...include.map((String f) => '--include=$f'),
    ];
    // TIP: --exclude is encoded into the tool itself.
    // see tools/header_guard_check/lib/header_guard_check.dart
    final ProcessRunnerResult result = await _processRunner.runProcess(args);
    if (result.exitCode != 0) {
      error('Header check failed. The following files have incorrect header guards:');
      message(result.stdout);
      return false;
    }
    return true;
  }

  @override
  Future<bool> fixFormatting() async {
    final List<String> include = <String>[];
    if (!allFiles) {
      include.addAll(await getFileList(<String>['*.h']));
      if (include.isEmpty) {
        message('No header files with changes, skipping header guard fix.');
        return true;
      }
    }
    final List<String> args = <String>[
      _dartBin,
      _headerGuardCheckBin,
      '--fix',
      ...include.map((String f) => '--include=$f'),
    ];
    // TIP: --exclude is encoded into the tool itself.
    // see tools/header_guard_check/lib/header_guard_check.dart
    final ProcessRunnerResult result = await _processRunner.runProcess(args);
    if (result.exitCode != 0) {
      error('Header check fix failed:');
      message(result.stdout);
      return false;
    }
    return true;
  }
}

Future<String> _getDiffBaseRevision(ProcessManager processManager, Directory repoDir) async {
  final ProcessRunner processRunner = ProcessRunner(
    defaultWorkingDirectory: repoDir,
    processManager: processManager,
  );
  String upstream = 'upstream';
  final String upstreamUrl = await _runGit(
    <String>['remote', 'get-url', upstream],
    processRunner,
    failOk: true,
  );
  if (upstreamUrl.isEmpty) {
    upstream = 'origin';
  }
  await _runGit(<String>['fetch', upstream, 'main'], processRunner);
  String result = '';
  try {
    // This is the preferred command to use, but developer checkouts often do
    // not have a clear fork point, so we fall back to just the regular
    // merge-base in that case.
    result = await _runGit(<String>[
      'merge-base',
      '--fork-point',
      'FETCH_HEAD',
      'HEAD',
    ], processRunner);
  } on ProcessRunnerException {
    result = await _runGit(<String>['merge-base', 'FETCH_HEAD', 'HEAD'], processRunner);
  }
  return result.trim();
}

void _usage(ArgParser parser, {int exitCode = 1}) {
  stderr.writeln(
    'format.dart [--help] [--fix] [--all-files] '
    '[--check <${formatCheckNames().join('|')}>]',
  );
  stderr.writeln(parser.usage);
  exit(exitCode);
}

bool verbose = false;

/// Retrieve the root of the repository, i.e. the directory containing engine/src/flutter.
Directory repositoryRoot() {
  final enginePath = path.split(engineSubPath);
  final File script = File.fromUri(Platform.script).absolute;
  final searchPath = path.split(script.parent.path);

  while (searchPath.isNotEmpty) {
    final search = path.joinAll([...searchPath, ...enginePath]);
    if (path.isWithin(search, script.path)) {
      break;
    }
    searchPath.length--;
  }
  if (searchPath.isEmpty) {
    stderr.writeln('Unable to find root form ${script.path}');
    exit(-1);
  }
  return Directory(path.joinAll(searchPath));
}

Directory engineDir(Directory repository) {
  return Directory(path.join(repository.path, engineSubPath));
}

Future<int> main(List<String> arguments) async {
  final ArgParser parser = ArgParser();
  parser.addFlag('help', help: 'Print help.', abbr: 'h');
  parser.addFlag(
    'fix',
    abbr: 'f',
    help: 'Instead of just checking for formatting errors, fix them in place.',
  );
  parser.addFlag(
    'all-files',
    abbr: 'a',
    help:
        'Instead of just checking for formatting errors in changed files, '
        'check for them in all files.',
  );
  parser.addMultiOption(
    'check',
    abbr: 'c',
    allowed: formatCheckNames(),
    defaultsTo: formatCheckNames(),
    help:
        'Specifies which checks will be performed. Defaults to all checks. '
        'May be specified more than once to perform multiple types of checks. ',
  );
  parser.addFlag('verbose', help: 'Print verbose output.', defaultsTo: verbose);

  late final ArgResults options;
  try {
    options = parser.parse(arguments);
  } on FormatException catch (e) {
    stderr.writeln('ERROR: $e');
    _usage(parser, exitCode: 0);
  }

  verbose = options['verbose'] as bool;

  if (options['help'] as bool) {
    _usage(parser, exitCode: 0);
  }

  final Directory repoDir = repositoryRoot();
  final Directory srcDir = Directory(path.join(repoDir.path, 'engine/src'));
  if (verbose) {
    stderr.writeln('Repo: $repoDir');
    stderr.writeln('Src: $srcDir');
  }

  void message(String? message, {MessageType type = MessageType.message}) {
    message ??= '';
    switch (type) {
      case MessageType.message:
        stdout.writeln(message);
      case MessageType.error:
        stderr.writeln('ERROR: $message');
      case MessageType.warning:
        stderr.writeln('WARNING: $message');
    }
  }

  const ProcessManager processManager = LocalProcessManager();
  final String baseGitRef = await _getDiffBaseRevision(processManager, repoDir);

  bool result = true;
  final List<String> checks = options['check'] as List<String>;
  try {
    for (final String checkName in checks) {
      final FormatCheck check = nameToFormatCheck(checkName);
      final String humanCheckName = formatCheckToName(check);
      final FormatChecker checker = FormatChecker.ofType(
        check,
        baseGitRef: baseGitRef,
        repoDir: repoDir,
        srcDir: srcDir,
        allFiles: options['all-files'] as bool,
        messageCallback: message,
      );
      bool stepResult;
      if (options['fix'] as bool) {
        message('Fixing any $humanCheckName format problems');
        stepResult = await checker.fixFormatting();
        if (!stepResult) {
          message('Unable to apply $humanCheckName format fixes.');
        }
      } else {
        stepResult = await checker.checkFormatting();
      }
      result = result && stepResult;
    }
  } on FormattingException catch (e) {
    message('ERROR: $e', type: MessageType.error);
  }

  exit(result ? 0 : 1);
}
