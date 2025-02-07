// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert' show jsonEncode;
import 'dart:io' as io show Directory, File, Platform, ProcessSignal, stderr;

import 'package:clang_tidy/clang_tidy.dart';
import 'package:clang_tidy/src/command.dart';
import 'package:clang_tidy/src/lint_target.dart';
import 'package:clang_tidy/src/options.dart';
import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:path/path.dart' as path;
import 'package:process/process.dart';
import 'package:process_fakes/process_fakes.dart';
import 'package:process_runner/process_runner.dart';
import 'package:test/test.dart';

/// A test fixture for the `clang-tidy` tool.
final class Fixture {
  /// Simulates running the tool with the given [args].
  factory Fixture.fromCommandLine(
    List<String> args, {
    ProcessManager? processManager,
    Engine? engine,
  }) {
    processManager ??= FakeProcessManager();
    final StringBuffer outBuffer = StringBuffer();
    final StringBuffer errBuffer = StringBuffer();
    return Fixture._(
      ClangTidy.fromCommandLine(
        args,
        outSink: outBuffer,
        errSink: errBuffer,
        processManager: processManager,
        engine: engine,
      ),
      errBuffer,
    );
  }

  /// Simulates running the tool with the given [options].
  factory Fixture.fromOptions(Options options, {ProcessManager? processManager}) {
    processManager ??= FakeProcessManager();
    final StringBuffer outBuffer = StringBuffer();
    final StringBuffer errBuffer = StringBuffer();
    return Fixture._(
      ClangTidy(
        buildCommandsPath: options.buildCommandsPath,
        lintTarget: options.lintTarget,
        fix: options.fix,
        outSink: outBuffer,
        errSink: errBuffer,
        processManager: processManager,
      ),
      errBuffer,
    );
  }

  Fixture._(this.tool, this.errBuffer);

  /// The `clang-tidy` tool.
  final ClangTidy tool;

  /// Captured `stderr` from the tool.
  final StringBuffer errBuffer;
}

// Recorded locally from clang-tidy.
const String _tidyOutput = '''
/runtime.dart_isolate.o" in /Users/aaclarke/dev/engine/src/out/host_debug exited with code 1
3467 warnings generated.
/Users/aaclarke/dev/engine/src/flutter/runtime/dart_isolate.cc:167:32: error: std::move of the const variable 'dart_entrypoint_args' has no effect; remove std::move() or make the variable non-const [performance-move-const-arg,-warnings-as-errors]
                               std::move(dart_entrypoint_args))) {
                               ^~~~~~~~~~                    ~
Suppressed 3474 warnings (3466 in non-user code, 8 NOLINT).
Use -header-filter=.* to display errors from all non-system headers. Use -system-headers to display errors from system headers as well.
1 warning treated as error
:
3467 warnings generated.
Suppressed 3474 warnings (3466 in non-user code, 8 NOLINT).
Use -header-filter=.* to display errors from all non-system headers. Use -system-headers to display errors from system headers as well.
1 warning treated as error



''';

const String _tidyTrimmedOutput = '''
/Users/aaclarke/dev/engine/src/flutter/runtime/dart_isolate.cc:167:32: error: std::move of the const variable 'dart_entrypoint_args' has no effect; remove std::move() or make the variable non-const [performance-move-const-arg,-warnings-as-errors]
                               std::move(dart_entrypoint_args))) {
                               ^~~~~~~~~~                    ~
Suppressed 3474 warnings (3466 in non-user code, 8 NOLINT).
Use -header-filter=.* to display errors from all non-system headers. Use -system-headers to display errors from system headers as well.
1 warning treated as error''';

void _withTempFile(String prefix, void Function(String path) func) {
  final String filePath = path.join(io.Directory.systemTemp.path, '$prefix-temp-file');
  final io.File file = io.File(filePath);
  file.createSync();
  try {
    func(file.path);
  } finally {
    file.deleteSync();
  }
}

final _engineRoot = Engine.findWithin();

void main() {
  // This test requires a compile_commands.json file to exist.
  //
  // We can provide exactly which build to use, i.e.:
  //    COMPILE_COMMANDS_PATH=/path/to/compile_commands.json dart test
  //
  // Or, we can fall back to the latest build output if one isn't provided.
  final String buildCommands;
  if (io.Platform.environment['COMPILE_COMMANDS_PATH'] case final String compileCommandsPath) {
    buildCommands = compileCommandsPath;
  } else {
    final String? inferredPath = _engineRoot.latestOutput()?.compileCommandsJson.path;
    io.stderr.writeln('No COMPILE_COMMANDS_PATH found in environment.');
    if (inferredPath != null) {
      io.stderr.writeln('Inferring the last build output: $inferredPath');
      buildCommands = inferredPath;
    } else {
      fail('No outputs or build commands found.');
    }
  }

  test('--help gives help, and uses host_debug by default outside of an engine root', () async {
    final io.Directory rootDir = io.Directory.systemTemp.createTempSync('clang_tidy_test');
    try {
      final Fixture fixture = Fixture.fromCommandLine(<String>[
        '--help',
      ], engine: TestEngine.createTemp(rootDir: rootDir));
      final int result = await fixture.tool.run();

      expect(fixture.tool.options.help, isTrue);
      expect(result, equals(0));

      final String errors = fixture.errBuffer.toString();
      expect(errors, contains('Usage: '));
      expect(errors, contains('defaults to "host_debug"'));
    } finally {
      rootDir.deleteSync(recursive: true);
    }
  });

  test(
    '--help gives help, and uses the latest build by default outside in an engine root',
    () async {
      final io.Directory rootDir = io.Directory.systemTemp.createTempSync('clang_tidy_test');
      final io.Directory buildDir = io.Directory(
        path.join(rootDir.path, 'out', 'host_debug_unopt_arm64'),
      )..createSync(recursive: true);
      try {
        final Fixture fixture = Fixture.fromCommandLine(
          <String>['--help'],
          engine: TestEngine.createTemp(
            rootDir: rootDir,
            outputs: <TestOutput>[TestOutput(buildDir)],
          ),
        );
        final int result = await fixture.tool.run();

        expect(fixture.tool.options.help, isTrue);
        expect(result, equals(0));

        final String errors = fixture.errBuffer.toString();
        expect(errors, contains('Usage: '));
        expect(errors, contains('defaults to "host_debug_unopt_arm64"'));
      } finally {
        rootDir.deleteSync(recursive: true);
      }
    },
  );

  test('trimmed clang-tidy output', () {
    expect(_tidyTrimmedOutput, equals(ClangTidy.trimOutput(_tidyOutput)));
  });

  test('Error when --compile-commands and --target-variant are used together', () async {
    final Fixture fixture = Fixture.fromCommandLine(<String>[
      '--compile-commands',
      '/unused',
      '--target-variant',
      'unused',
    ]);

    final int result = await fixture.tool.run();

    expect(result, equals(1));
    expect(
      fixture.errBuffer.toString(),
      contains('ERROR: --compile-commands option cannot be used with --target-variant.'),
    );
  });

  test('Error when --compile-commands and --src-dir are used together', () async {
    final Fixture fixture = Fixture.fromCommandLine(<String>[
      '--compile-commands',
      '/unused',
      '--src-dir',
      '/unused',
    ]);
    final int result = await fixture.tool.run();

    expect(result, equals(1));
    expect(
      fixture.errBuffer.toString(),
      contains('ERROR: --compile-commands option cannot be used with --src-dir.'),
    );
  });

  test('shard-id valid', () async {
    _withTempFile('shard-id-valid', (String path) {
      final Options options = Options.fromCommandLine(<String>[
        '--compile-commands=$path',
        '--shard-variants=variant',
        '--shard-id=1',
      ]);
      expect(options.errorMessage, isNull);
      expect(options.shardId, equals(1));
    });
  });

  test('clang-tidy specified', () async {
    _withTempFile('shard-id-valid', (String path) {
      final Options options = Options.fromCommandLine(<String>[
        '--compile-commands=$path',
        '--clang-tidy=foo/bar',
      ]);
      expect(options.clangTidyPath, isNotNull);
      expect(options.clangTidyPath!.path, equals('foo/bar'));
    });
  });

  test('clang-tidy unspecified', () async {
    _withTempFile('shard-id-valid', (String path) {
      final Options options = Options.fromCommandLine(<String>[]);
      expect(options.clangTidyPath, isNull);
    });
  });

  test('shard-id invalid', () async {
    _withTempFile('shard-id-valid', (String path) {
      final StringBuffer errBuffer = StringBuffer();
      final Options options = Options.fromCommandLine(<String>[
        '--compile-commands=$path',
        '--shard-variants=variant',
        '--shard-id=2',
      ], errSink: errBuffer);
      expect(options.errorMessage, isNotNull);
      expect(options.shardId, isNull);
      expect(options.errorMessage, contains('Invalid shard-id value'));
    });
  });

  test('Error when --compile-commands path does not exist', () async {
    final Fixture fixture = Fixture.fromCommandLine(<String>[
      '--compile-commands',
      '/does/not/exist',
    ]);
    final int result = await fixture.tool.run();

    expect(result, equals(1));
    expect(
      fixture.errBuffer.toString().split('\n')[0],
      matches(r"ERROR: Build commands path .*/does/not/exist doesn't exist."),
    );
  });

  test('Error when --src-dir path does not exist, uses target variant in path', () async {
    final Fixture fixture = Fixture.fromCommandLine(<String>[
      '--src-dir',
      '/does/not/exist',
      '--target-variant',
      'ios_debug_unopt',
    ]);
    final int result = await fixture.tool.run();

    expect(result, equals(1));
    expect(
      fixture.errBuffer.toString().split('\n')[0],
      matches(
        r'ERROR: Build commands path .*/does/not/exist'
        r'[/\\]out[/\\]ios_debug_unopt[/\\]compile_commands.json'
        r" doesn't exist.",
      ),
    );
  });

  test('Error when --lint-all and --lint-head are used together', () async {
    final Fixture fixture = Fixture.fromCommandLine(<String>[
      '--compile-commands',
      '/unused',
      '--lint-all',
      '--lint-head',
    ]);
    final int result = await fixture.tool.run();

    expect(result, equals(1));
    expect(
      fixture.errBuffer.toString(),
      contains('ERROR: At most one of --lint-all, --lint-head, --lint-regex can be passed.'),
    );
  });

  test('Error when --lint-all and --lint-regex are used together', () async {
    final Fixture fixture = Fixture.fromCommandLine(<String>[
      '--compile-commands',
      '/unused',
      '--lint-all',
      '--lint-regex=".*"',
    ]);
    final int result = await fixture.tool.run();

    expect(result, equals(1));
    expect(
      fixture.errBuffer.toString(),
      contains('ERROR: At most one of --lint-all, --lint-head, --lint-regex can be passed.'),
    );
  });

  test('lintAll=true checks all files', () async {
    final Fixture fixture = Fixture.fromOptions(
      Options(buildCommandsPath: io.File(buildCommands), lintTarget: const LintAll()),
    );
    final List<io.File> fileList = await fixture.tool.computeFilesOfInterest();
    expect(fileList.length, greaterThan(1000));
  });

  test('lintAll=false does not check all files', () async {
    final Fixture fixture = Fixture.fromOptions(
      Options(
        buildCommandsPath: io.File(buildCommands),
        // Intentional:
        // ignore: avoid_redundant_argument_values
        lintTarget: const LintChanged(),
      ),
      processManager: FakeProcessManager(
        onStart: (FakeCommandLogEntry entry) {
          if (entry.command.first == 'git') {
            // This just allows git to not actually be called.
            return FakeProcess();
          }
          return FakeProcessManager.unhandledStart(entry);
        },
      ),
    );
    final List<io.File> fileList = await fixture.tool.computeFilesOfInterest();
    expect(fileList.length, lessThan(300));
  });

  test('lintAll=pattern checks based on a RegEx', () async {
    final Fixture fixture = Fixture.fromOptions(
      Options(
        buildCommandsPath: io.File(buildCommands),
        lintTarget: const LintRegex(r'.*test.*\.cc$'),
      ),
      processManager: FakeProcessManager(
        onStart: (FakeCommandLogEntry entry) {
          if (entry.command.first == 'git') {
            // This just allows git to not actually be called.
            return FakeProcess();
          }
          return FakeProcessManager.unhandledStart(entry);
        },
      ),
    );
    final List<io.File> fileList = await fixture.tool.computeFilesOfInterest();
    expect(fileList.length, lessThan(2000));
  });

  test('Sharding', () async {
    final Fixture fixture = Fixture.fromOptions(
      Options(buildCommandsPath: io.File(buildCommands), lintTarget: const LintAll()),
      processManager: FakeProcessManager(
        onStart: (FakeCommandLogEntry entry) {
          if (entry.command.first == 'git') {
            // This just allows git to not actually be called.
            return FakeProcess();
          }
          return FakeProcessManager.unhandledStart(entry);
        },
      ),
    );

    Map<String, String> makeBuildCommandEntry(String filePath) {
      return <String, String>{
        'directory': '/unused',
        'command': '../../buildtools/mac-x64/clang/bin/clang $filePath',
        'file': filePath,
      };
    }

    final List<String> filePaths = <String>[
      for (int i = 0; i < 10; ++i) '/path/to/a/source_file_$i.cc',
    ];
    final List<Map<String, String>> buildCommandsData =
        filePaths.map((String e) => makeBuildCommandEntry(e)).toList();
    final List<Map<String, String>> shardBuildCommandsData =
        filePaths.sublist(6).map((String e) => makeBuildCommandEntry(e)).toList();

    {
      final List<Command> commands = await fixture.tool.getLintCommandsForFiles(
        buildCommandsData,
        filePaths.map((String e) => io.File(e)).toList(),
        <List<dynamic>>[shardBuildCommandsData],
        0,
      );
      final Iterable<String> commandFilePaths = commands.map((Command e) => e.filePath);
      expect(commands.length, equals(8));
      expect(commandFilePaths.contains('/path/to/a/source_file_0.cc'), true);
      expect(commandFilePaths.contains('/path/to/a/source_file_1.cc'), true);
      expect(commandFilePaths.contains('/path/to/a/source_file_2.cc'), true);
      expect(commandFilePaths.contains('/path/to/a/source_file_3.cc'), true);
      expect(commandFilePaths.contains('/path/to/a/source_file_4.cc'), true);
      expect(commandFilePaths.contains('/path/to/a/source_file_5.cc'), true);
      expect(commandFilePaths.contains('/path/to/a/source_file_6.cc'), true);
      expect(commandFilePaths.contains('/path/to/a/source_file_7.cc'), false);
      expect(commandFilePaths.contains('/path/to/a/source_file_8.cc'), true);
      expect(commandFilePaths.contains('/path/to/a/source_file_9.cc'), false);
    }
    {
      final List<Command> commands = await fixture.tool.getLintCommandsForFiles(
        buildCommandsData,
        filePaths.map((String e) => io.File(e)).toList(),
        <List<Map<String, String>>>[shardBuildCommandsData],
        1,
      );

      final Iterable<String> commandFilePaths = commands.map((Command e) => e.filePath);
      expect(commands.length, equals(8));
      expect(commandFilePaths.contains('/path/to/a/source_file_0.cc'), true);
      expect(commandFilePaths.contains('/path/to/a/source_file_1.cc'), true);
      expect(commandFilePaths.contains('/path/to/a/source_file_2.cc'), true);
      expect(commandFilePaths.contains('/path/to/a/source_file_3.cc'), true);
      expect(commandFilePaths.contains('/path/to/a/source_file_4.cc'), true);
      expect(commandFilePaths.contains('/path/to/a/source_file_5.cc'), true);
      expect(commandFilePaths.contains('/path/to/a/source_file_6.cc'), false);
      expect(commandFilePaths.contains('/path/to/a/source_file_7.cc'), true);
      expect(commandFilePaths.contains('/path/to/a/source_file_8.cc'), false);
      expect(commandFilePaths.contains('/path/to/a/source_file_9.cc'), true);
    }
  });

  test('No Commands are produced when no files changed', () async {
    final Fixture fixture = Fixture.fromOptions(
      Options(buildCommandsPath: io.File(buildCommands), lintTarget: const LintAll()),
    );

    const String filePath = '/path/to/a/source_file.cc';
    final List<dynamic> buildCommandsData = <Map<String, dynamic>>[
      <String, dynamic>{
        'directory': '/unused',
        'command': '../../buildtools/mac-x64/clang/bin/clang $filePath',
        'file': filePath,
      },
    ];
    final List<Command> commands = await fixture.tool.getLintCommandsForFiles(
      buildCommandsData,
      <io.File>[],
      <List<dynamic>>[],
      null,
    );

    expect(commands, isEmpty);
  });

  test('A Command is produced when a file is changed', () async {
    final Fixture fixture = Fixture.fromOptions(
      Options(buildCommandsPath: io.File(buildCommands), lintTarget: const LintAll()),
    );

    // This file needs to exist, and be UTF8 line-parsable.
    final String filePath = path.join(
      _engineRoot.flutterDir.path,
      'tools',
      'clang_tidy',
      'test',
      'clang_tidy_test.dart',
    );
    final List<dynamic> buildCommandsData = <Map<String, dynamic>>[
      <String, dynamic>{
        'directory': '/unused',
        'command': '../../buildtools/mac-x64/clang/bin/clang $filePath',
        'file': filePath,
      },
    ];
    final List<Command> commands = await fixture.tool.getLintCommandsForFiles(
      buildCommandsData,
      <io.File>[io.File(filePath)],
      <List<dynamic>>[],
      null,
    );

    expect(commands, isNotEmpty);
    final Command command = commands.first;
    expect(command.tidyPath, contains('clang/bin/clang-tidy'));
    final Options noFixOptions = Options(buildCommandsPath: io.File('.'));
    expect(noFixOptions.fix, isFalse);
    final WorkerJob jobNoFix = command.createLintJob(noFixOptions);
    expect(jobNoFix.command[0], endsWith('../../buildtools/mac-x64/clang/bin/clang-tidy'));
    expect(jobNoFix.command[1], endsWith(filePath.replaceAll('/', io.Platform.pathSeparator)));
    expect(jobNoFix.command[2], '--warnings-as-errors=*');
    expect(jobNoFix.command[3], '--');
    expect(jobNoFix.command[4], '');
    expect(jobNoFix.command[5], endsWith(filePath));

    final Options fixOptions = Options(buildCommandsPath: io.File('.'), fix: true);
    final WorkerJob jobWithFix = command.createLintJob(fixOptions);
    expect(jobWithFix.command[0], endsWith('../../buildtools/mac-x64/clang/bin/clang-tidy'));
    expect(jobWithFix.command[1], endsWith(filePath.replaceAll('/', io.Platform.pathSeparator)));
    expect(jobWithFix.command[2], '--warnings-as-errors=*');
    expect(jobWithFix.command[3], '--fix');
    expect(jobWithFix.command[4], '--format-style=file');
    expect(jobWithFix.command[5], '--');
    expect(jobWithFix.command[6], '');
    expect(jobWithFix.command[7], endsWith(filePath));
  });

  test('Command getLintAction flags third_party files', () async {
    final LintAction lintAction = await Command.getLintAction(
      '/some/file/in/a/third_party/dependency',
    );

    expect(lintAction, equals(LintAction.skipThirdParty));
  });

  test('Command getLintAction flags missing files', () async {
    final LintAction lintAction = await Command.getLintAction('/does/not/exist');

    expect(lintAction, equals(LintAction.skipMissing));
  });

  test('Command getLintActionFromContents flags FLUTTER_NOLINT', () async {
    final LintAction lintAction = await Command.lintActionFromContents(
      Stream<String>.fromIterable(<String>[
        '// Copyright 2013 The Flutter Authors. All rights reserved.\n',
        '// Use of this source code is governed by a BSD-style license that can be\n',
        '// found in the LICENSE file.\n',
        '\n',
        '// FLUTTER_NOLINT: https://github.com/flutter/flutter/issues/68332\n',
        '\n',
        '#include "flutter/shell/version/version.h"\n',
      ]),
    );

    expect(lintAction, equals(LintAction.skipNoLint));
  });

  test('Command getLintActionFromContents flags malformed FLUTTER_NOLINT', () async {
    final LintAction lintAction = await Command.lintActionFromContents(
      Stream<String>.fromIterable(<String>[
        '// Copyright 2013 The Flutter Authors. All rights reserved.\n',
        '// Use of this source code is governed by a BSD-style license that can be\n',
        '// found in the LICENSE file.\n',
        '\n',
        '// FLUTTER_NOLINT: https://gir/flutter/issues/68332\n',
        '\n',
        '#include "flutter/shell/version/version.h"\n',
      ]),
    );

    expect(lintAction, equals(LintAction.failMalformedNoLint));
  });

  test('Command getLintActionFromContents flags that we should lint', () async {
    final LintAction lintAction = await Command.lintActionFromContents(
      Stream<String>.fromIterable(<String>[
        '// Copyright 2013 The Flutter Authors. All rights reserved.\n',
        '// Use of this source code is governed by a BSD-style license that can be\n',
        '// found in the LICENSE file.\n',
        '\n',
        '#include "flutter/shell/version/version.h"\n',
      ]),
    );

    expect(lintAction, equals(LintAction.lint));
  });

  test('Command filters out sed command after a compile command', () {
    final Command command = Command.fromMap(<String, String>{
      'directory': '/unused',
      'command':
          '../../buildtools/mac-x64/clang/bin/clang filename '
          "&& sed -i 's@/b/f/w@../..@g' filename",
      'file': 'unused',
    });
    expect(command.tidyArgs.trim(), 'filename');
  });

  test('Command filters out the -MF flag', () {
    final Command command = Command.fromMap(<String, String>{
      'directory': '/unused',
      'command': '../../buildtools/mac-x64/clang/bin/clang -MF stuff filename ',
      'file': 'unused',
    });
    expect(command.tidyArgs.trim(), 'filename');
  });

  test('Command filters out rewrapper command before a compile command', () {
    final Command command = Command.fromMap(<String, String>{
      'directory': '/unused',
      'command':
          'flutter/engine/src/buildtools/mac-arm64/reclient/rewrapper '
          '--cfg=flutter/engine/src/flutter/build/rbe/rewrapper-mac-arm64.cfg '
          '--exec_root=flutter/engine/src/ '
          '--labels=type=compile,compiler=clang,lang=cpp '
          '../../buildtools/mac-x64/clang/bin/clang++ filename ',
      'file': 'unused',
    });
    expect(command.tidyArgs.trim(), 'filename');
  });

  test('Files that cause clang-tidy to segfault are skipped', () async {
    final Fixture fileListFixture = Fixture.fromOptions(
      Options(buildCommandsPath: io.File(buildCommands), lintTarget: const LintAll()),
    );
    final String firstFilePath = (await fileListFixture.tool.computeFilesOfInterest()).first.path;

    final FakeProcessManager fakeProcessManager = FakeProcessManager(
      onStart: (FakeCommandLogEntry entry) {
        if (entry.command.first.endsWith('clang-tidy')) {
          return FakeProcess(exitCode: -io.ProcessSignal.sigsegv.signalNumber);
        }
        return FakeProcessManager.unhandledStart(entry);
      },
    );

    final List<dynamic> commandsData = <Map<String, dynamic>>[
      <String, dynamic>{
        'file': firstFilePath,
        'directory': '/unused',
        'command': 'clang/bin/clang $firstFilePath',
      },
    ];

    final io.File commands = io.File(
      path.join(io.Directory.systemTemp.path, 'test_compile_commands.json'),
    );
    int result;
    try {
      commands.writeAsStringSync(jsonEncode(commandsData));
      final Fixture fixture = Fixture.fromOptions(
        Options(buildCommandsPath: commands, lintTarget: const LintAll()),
        processManager: fakeProcessManager,
      );
      result = await fixture.tool.run();
    } finally {
      commands.deleteSync();
    }

    expect(result, 0);
  });
}
