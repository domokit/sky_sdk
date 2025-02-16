// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/features.dart';

import '../src/common.dart';
import 'test_utils.dart';

// This test file does not use [getLocalEngineArguments] because it is testing
// command output and not using cached artifacts.

void main() {
  testWithoutContext(
    'All development tools and deprecated commands are hidden and help text is not verbose',
    () async {
      final ProcessResult result = await processManager.run(<String>[flutterBin, '-h', '-v']);

      // Development tools.
      expect(result.stdout, isNot(contains('update-packages')));

      // Deprecated.
      expect(result.stdout, isNot(contains('make-host-app-editable')));

      // Only printed by verbose tool.
      expect(result.stdout, isNot(contains('exiting with code 0')));
    },
  );

  testWithoutContext('Flutter help is shown with -? command line argument', () async {
    final ProcessResult result = await processManager.run(<String>[flutterBin, '-?']);

    // Development tools.
    expect(
      result.stdout,
      contains(
        'Run "flutter help <command>" for more information about a command.\n'
        'Run "flutter help -v" for verbose help output, including less commonly used options.',
      ),
    );
  });

  testWithoutContext('flutter doctor is not verbose', () async {
    final ProcessResult result = await processManager.run(<String>[flutterBin, 'doctor', '-v']);

    // Only printed by verbose tool.
    expect(result.stdout, isNot(contains('exiting with code 0')));
  });

  testWithoutContext('flutter doctor -vv super verbose', () async {
    final ProcessResult result = await processManager.run(<String>[flutterBin, 'doctor', '-vv']);

    // Check for message only printed in verbose mode.
    expect(result.stdout, contains('Shutdown hooks complete'));
  });

  testWithoutContext('flutter config --list contains all features', () async {
    final ProcessResult result = await processManager.run(<String>[flutterBin, 'config', '--list']);

    // contains all of the experiments in features.dart
    expect(
      (result.stdout as String).split('\n'),
      containsAll(<Matcher>[
        for (final Feature feature in allConfigurableFeatures) contains(feature.configSetting),
      ]),
    );
  });

  testWithoutContext('flutter run --machine uses AppRunLogger', () async {
    final Directory directory = createResolvedTempDirectorySync(
      'flutter_run_test.',
    ).createTempSync('_flutter_run_test.')..createSync(recursive: true);

    try {
      directory.childFile('pubspec.yaml').writeAsStringSync('name: foo');
      directory.childDirectory('lib').childFile('main.dart').createSync(recursive: true);
      final ProcessResult result = await processManager.run(<String>[
        flutterBin,
        'run',
        '--show-test-device', // ensure command can fail to run and hit injection of correct logger.
        '--machine',
        '-v',
        '--no-resident',
      ], workingDirectory: directory.path);
      expect(result.stderr, isNot(contains('Oops; flutter has exited unexpectedly:')));
    } finally {
      tryToDelete(directory);
    }
  });

  testWithoutContext('flutter attach --machine uses AppRunLogger', () async {
    final ProcessResult result = await processManager.run(<String>[
      flutterBin,
      'attach',
      '--machine',
      '-v',
    ]);

    expect(
      result.stderr,
      contains('Target file'),
    ); // Target file not found, but different paths on Windows and Linux/macOS.
  });

  testWithoutContext('flutter --version --machine outputs JSON with flutterRoot', () async {
    final ProcessResult result = await processManager.run(<String>[
      flutterBin,
      '--version',
      '--machine',
    ]);

    final Map<String, Object?> versionInfo =
        json.decode(
              result.stdout
                  .toString()
                  .replaceAll('Building flutter tool...', '')
                  .replaceAll(
                    'Waiting for another flutter command to release the startup lock...',
                    '',
                  )
                  .trim(),
            )
            as Map<String, Object?>;

    expect(versionInfo, containsPair('flutterRoot', isNotNull));
  });

  testWithoutContext('A tool exit is thrown for an invalid debug-url in flutter attach', () async {
    // This test is almost exactly like the next one; update them together please.
    final String helloWorld = fileSystem.path.join(getFlutterRoot(), 'examples', 'hello_world');
    final ProcessResult result = await processManager.run(<String>[
      flutterBin,
      '--show-test-device',
      'attach',
      '-d',
      'flutter-tester',
      '--debug-url=http://127.0.0.1:3333*/',
    ], workingDirectory: helloWorld);

    expect(
      result,
      const ProcessResultMatcher(
        exitCode: 1,
        stderrPattern: 'Invalid `--debug-url`: http://127.0.0.1:3333*/',
      ),
    );
  });

  testWithoutContext('--debug-uri is an alias for --debug-url', () async {
    // This text is exactly the same as the previous one but with a "l" turned to an "i".
    final String helloWorld = fileSystem.path.join(getFlutterRoot(), 'examples', 'hello_world');
    final ProcessResult result = await processManager.run(<String>[
      flutterBin,
      '--show-test-device',
      'attach',
      '-d',
      'flutter-tester',
      '--debug-uri=http://127.0.0.1:3333*/', // "uri" not "url"
    ], workingDirectory: helloWorld);

    expect(
      result,
      const ProcessResultMatcher(
        exitCode: 1,
        // _"url"_ not "uri"!
        stderrPattern: 'Invalid `--debug-url`: http://127.0.0.1:3333*/',
      ),
    );
  });

  testWithoutContext('will load bootstrap script before starting', () async {
    final File bootstrap = fileSystem.file(
      fileSystem.path.join(
        getFlutterRoot(),
        'bin',
        'internal',
        platform.isWindows ? 'bootstrap.bat' : 'bootstrap.sh',
      ),
    );

    try {
      bootstrap.writeAsStringSync('echo TESTING 1 2 3');
      final ProcessResult result = await processManager.run(<String>[flutterBin]);

      expect(result.stdout, contains('TESTING 1 2 3'));
    } finally {
      bootstrap.deleteSync();
    }
  });

  testWithoutContext('flutter attach does not support --release', () async {
    final String helloWorld = fileSystem.path.join(getFlutterRoot(), 'examples', 'hello_world');
    final ProcessResult result = await processManager.run(<String>[
      flutterBin,
      '--show-test-device',
      'attach',
      '--release',
    ], workingDirectory: helloWorld);

    expect(result.exitCode, isNot(0));
    expect(result.stderr, contains('Could not find an option named "--release"'));
  });

  testWithoutContext('flutter can report crashes', () async {
    final ProcessResult result = await processManager.run(
      <String>[flutterBin, 'update-packages', '--crash'],
      environment: <String, String>{'BOT': 'false'},
    );

    expect(result.exitCode, isNot(0));
    expect(
      result.stderr,
      contains(
        'Oops; flutter has exited unexpectedly: "Bad state: test crash please ignore.".\n'
        'A crash report has been written to',
      ),
    );
  });

  testWithoutContext('flutter supports trailing args', () async {
    final String helloWorld = fileSystem.path.join(getFlutterRoot(), 'examples', 'hello_world');
    final ProcessResult result = await processManager.run(<String>[
      flutterBin,
      'test',
      'test/hello_test.dart',
      '-r',
      'json',
    ], workingDirectory: helloWorld);

    expect(result, const ProcessResultMatcher());
    expect(result.stderr, isEmpty);
  });
}
