// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This test builds an integration test from the list of samples in the
// examples/api/lib directory, and then runs it. The tests are just smoke tests,
// designed to start up each example and run it for a couple of frames to make
// sure it doesn't throw an exception or fail to compile.

import 'dart:async';
import 'dart:convert';
import 'dart:io' show Process, ProcessException, exitCode, stderr, stdout;

import 'package:file/file.dart';
import 'package:file/local.dart';
import 'package:path/path.dart' as path;
import 'package:platform/platform.dart';
import 'package:process/process.dart';

const FileSystem _kFilesystem = LocalFileSystem();
const ProcessManager _kProcessManager = LocalProcessManager();
const Platform _kPlatform = LocalPlatform();

FutureOr<dynamic> main() async {
  if (!_kPlatform.isLinux && !_kPlatform.isWindows && !_kPlatform.isMacOS) {
    stderr.writeln('Example smoke tests are only designed to run on desktop platforms');
    exitCode = 4;
    return;
  }
  final Directory flutterDir = _kFilesystem.directory(
    path.absolute(path.dirname(path.dirname(path.dirname(_kPlatform.script.toFilePath())))),
  );
  final Directory apiDir = flutterDir.childDirectory('examples').childDirectory('api');
  final File integrationTest = await generateTest(apiDir);
  try {
    await runSmokeTests(flutterDir: flutterDir, integrationTest: integrationTest, apiDir: apiDir);
  } finally {
    await cleanUp(integrationTest);
  }
}

Future<void> cleanUp(File integrationTest) async {
  try {
    await integrationTest.delete();
    // Delete the integration_test directory if it is empty.
    await integrationTest.parent.delete();
  } on FileSystemException {
    // Ignore, there might be other files in there preventing it from
    // being removed, or it might not exist.
  }
}

// Executes the generated smoke test.
Future<void> runSmokeTests({
  required Directory flutterDir,
  required File integrationTest,
  required Directory apiDir,
}) async {
  final File flutterExe = flutterDir
      .childDirectory('bin')
      .childFile(_kPlatform.isWindows ? 'flutter.bat' : 'flutter');
  final List<String> cmd = <String>[
    // If we're in a container with no X display, then use the virtual framebuffer.
    if (_kPlatform.isLinux &&
        (_kPlatform.environment['DISPLAY'] == null || _kPlatform.environment['DISPLAY']!.isEmpty))
      '/usr/bin/xvfb-run',
    flutterExe.absolute.path,
    'test',
    '--reporter=expanded',
    '--device-id=${_kPlatform.operatingSystem}',
    integrationTest.absolute.path,
  ];
  await runCommand(cmd, workingDirectory: apiDir);
}

// A class to hold information related to an example, used to generate names
// from for the tests.
class ExampleInfo {
  ExampleInfo(File file, Directory examplesLibDir)
    : importPath = _getImportPath(file, examplesLibDir),
      importName = '' {
    importName = importPath.replaceAll(RegExp(r'\.dart$'), '').replaceAll(RegExp(r'\W'), '_');
  }

  final String importPath;
  String importName;

  static String _getImportPath(File example, Directory examplesLibDir) {
    final String relativePath = path.relative(
      example.absolute.path,
      from: examplesLibDir.absolute.path,
    );
    // So that Windows paths are proper URIs in the import statements.
    return path.toUri(relativePath).toFilePath(windows: false);
  }
}

// Generates the combined smoke test.
Future<File> generateTest(Directory apiDir) async {
  final Directory examplesLibDir = apiDir.childDirectory('lib');

  // Get files from git, to avoid any non-repo files that might be in someone's
  // workspace.
  final List<String> gitFiles = (await runCommand(
    <String>['git', 'ls-files', '**/*.dart'],
    workingDirectory: examplesLibDir,
    quiet: true,
  )).replaceAll(r'\', '/').trim().split('\n');
  final Iterable<File> examples = gitFiles.map<File>((String examplePath) {
    return _kFilesystem.file(path.join(examplesLibDir.absolute.path, examplePath));
  });

  // Collect the examples, and import them all as separate symbols.
  final List<ExampleInfo> infoList = <ExampleInfo>[
    for (final File example in examples) ExampleInfo(example, examplesLibDir),
  ];
  infoList.sort((ExampleInfo a, ExampleInfo b) => a.importPath.compareTo(b.importPath));
  final List<String> imports = <String>[
    "import 'package:flutter/widgets.dart';",
    "import 'package:flutter/scheduler.dart';",
    "import 'package:flutter_test/flutter_test.dart';",
    "import 'package:integration_test/integration_test.dart';",
    for (final ExampleInfo info in infoList)
      "import 'package:flutter_api_samples/${info.importPath}' as ${info.importName};",
  ]..sort();

  final StringBuffer buffer = StringBuffer();
  buffer.writeln('// Temporary generated file. Do not commit.');
  buffer.writeln("import 'dart:io';");
  buffer.writeAll(imports, '\n');
  buffer.writeln(r'''


import '../../../dev/manual_tests/test/mock_image_http.dart';

void main() {
  IntegrationTestWidgetsFlutterBinding? binding;
  try {
    binding = IntegrationTestWidgetsFlutterBinding.ensureInitialized() as IntegrationTestWidgetsFlutterBinding;
  } catch (e) {
    stderr.writeln('Unable to initialize binding${binding == null ? '' : ' $binding'}: $e');
    exitCode = 128;
    return;
  }

''');
  for (final ExampleInfo info in infoList) {
    buffer.writeln('''
  testWidgets(
    'Smoke test ${info.importPath}',
    (WidgetTester tester) async {
      final ErrorWidgetBuilder originalBuilder = ErrorWidget.builder;
      try {
        HttpOverrides.runZoned(() {
          ${info.importName}.main();
        }, createHttpClient: (SecurityContext? context) => FakeHttpClient(context));
        await tester.pump();
        await tester.pump();
        expect(find.byType(WidgetsApp), findsOneWidget);
      } finally {
        ErrorWidget.builder = originalBuilder;
        timeDilation = 1.0;
      }
    },
  );
''');
  }
  buffer.writeln('}');

  final File integrationTest = apiDir
      .childDirectory('integration_test')
      .childFile('smoke_integration_test.dart');
  integrationTest.createSync(recursive: true);
  integrationTest.writeAsStringSync(buffer.toString());
  return integrationTest;
}

// Run a command, and optionally stream the output as it runs, returning the
// stdout.
Future<String> runCommand(
  List<String> cmd, {
  required Directory workingDirectory,
  bool quiet = false,
  List<String>? output,
  Map<String, String>? environment,
}) async {
  final List<int> stdoutOutput = <int>[];
  final List<int> combinedOutput = <int>[];
  final Completer<void> stdoutComplete = Completer<void>();
  final Completer<void> stderrComplete = Completer<void>();

  late Process process;
  Future<int> allComplete() async {
    await stderrComplete.future;
    await stdoutComplete.future;
    return process.exitCode;
  }

  try {
    process = await _kProcessManager.start(
      cmd,
      workingDirectory: workingDirectory.absolute.path,
      environment: environment,
    );
    process.stdout.listen((List<int> event) {
      stdoutOutput.addAll(event);
      combinedOutput.addAll(event);
      if (!quiet) {
        stdout.add(event);
      }
    }, onDone: () async => stdoutComplete.complete());
    process.stderr.listen((List<int> event) {
      combinedOutput.addAll(event);
      if (!quiet) {
        stderr.add(event);
      }
    }, onDone: () async => stderrComplete.complete());
  } on ProcessException catch (e) {
    stderr.writeln(
      'Running "${cmd.join(' ')}" in ${workingDirectory.path} '
      'failed with:\n$e',
    );
    exitCode = 2;
    return utf8.decode(stdoutOutput);
  } on ArgumentError catch (e) {
    stderr.writeln(
      'Running "${cmd.join(' ')}" in ${workingDirectory.path} '
      'failed with:\n$e',
    );
    exitCode = 3;
    return utf8.decode(stdoutOutput);
  }

  final int processExitCode = await allComplete();
  if (processExitCode != 0) {
    stderr.writeln(
      'Running "${cmd.join(' ')}" in ${workingDirectory.path} exited with code $processExitCode',
    );
    exitCode = processExitCode;
  }

  if (output != null) {
    output.addAll(utf8.decode(combinedOutput).split('\n'));
  }

  return utf8.decode(stdoutOutput);
}
