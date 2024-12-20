// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// The purpose of this test is to verify the end-to-end behavior of
// "flutter run" and other such commands, as closely as possible to
// the default behavior. To that end, it avoids the use of any test
// features that are not critical (-dflutter-test being the primary
// example of a test feature that it does use). For example, no use
// is made of "--machine" in these tests.

// There are a number of risks when it comes to writing a test such
// as this one. Typically these tests are hard to debug if they are
// in a failing condition, because they just hang as they await the
// next expected line that never comes. To avoid this, here we have
// the policy of looking for multiple lines, printing what expected
// lines were not seen when a short timeout expires (but timing out
// does not cause the test to fail, to reduce flakes), and wherever
// possible recording all output and comparing the actual output to
// the expected output only once the test is completed.

// To aid in debugging, consider passing the `debug: true` argument
// to the runFlutter function.

// This file intentionally assumes the tests run in order.
@Tags(<String>['no-shuffle'])
library;

import 'dart:io';

import '../src/common.dart';
import 'test_utils.dart' show fileSystem;
import 'transition_test_utils.dart';

void main() {
  testWithoutContext(
    'flutter run writes and clears pidfile appropriately',
    () async {
      final String tempDirectory =
          fileSystem.systemTempDirectory
              .createTempSync('flutter_overall_experience_test.')
              .resolveSymbolicLinksSync();
      final String pidFile = fileSystem.path.join(tempDirectory, 'flutter.pid');
      final String testDirectory = fileSystem.path.join(flutterRoot, 'examples', 'hello_world');
      bool? existsDuringTest;
      try {
        expect(fileSystem.file(pidFile).existsSync(), isFalse);
        final ProcessTestResult result = await runFlutter(
          <String>['run', '-dflutter-tester', '--pid-file', pidFile],
          testDirectory,
          <Transition>[
            Barrier(
              'q Quit (terminate the application on the device).',
              handler: (String line) {
                existsDuringTest = fileSystem.file(pidFile).existsSync();
                return 'q';
              },
            ),
            const Barrier('Application finished.'),
          ],
        );
        expect(existsDuringTest, isNot(isNull));
        expect(existsDuringTest, isTrue);
        expect(result.exitCode, 0, reason: 'subprocess failed; $result');
        expect(fileSystem.file(pidFile).existsSync(), isFalse);
        // This first test ignores the stdout and stderr, so that if the
        // first run outputs "building flutter", or the "there's a new
        // flutter" banner, or other such first-run messages, they won't
        // fail the tests. This does mean that running this test first is
        // actually important in the case where you're running the tests
        // manually. (On CI, all those messages are expected to be seen
        // long before we get here, e.g. because we run "flutter doctor".)
      } finally {
        tryToDelete(fileSystem.directory(tempDirectory));
      }
    },
    // [intended] Windows doesn't support sending signals so we don't care if it can store the PID.
    skip: Platform.isWindows,
  );

  testWithoutContext('flutter run handle SIGUSR1/2 run', () async {
    final String tempDirectory =
        fileSystem.systemTempDirectory
            .createTempSync('flutter_overall_experience_test.')
            .resolveSymbolicLinksSync();
    final String pidFile = fileSystem.path.join(tempDirectory, 'flutter.pid');
    final String testDirectory = fileSystem.path.join(
      flutterRoot,
      'dev',
      'integration_tests',
      'ui',
    );
    final String testScript = fileSystem.path.join('lib', 'commands.dart');
    late int pid;
    final List<String> command = <String>[
      'run',
      '-dflutter-tester',
      '--report-ready',
      '--pid-file',
      pidFile,
      '--no-devtools',
      testScript,
    ];
    try {
      final ProcessTestResult result = await runFlutter(
        command,
        testDirectory,
        <Transition>[
          Multiple(
            <Pattern>['Flutter run key commands.', 'called paint'],
            handler: (String line) {
              pid = int.parse(fileSystem.file(pidFile).readAsStringSync());
              processManager.killPid(pid, ProcessSignal.sigusr1);
              return null;
            },
          ),
          Barrier('Performing hot reload...'.padRight(progressMessageWidth), logging: true),
          Multiple(
            <Pattern>[
              RegExp(
                r'^Reloaded 0 libraries in [0-9]+ms \(compile: \d+ ms, reload: \d+ ms, reassemble: \d+ ms\)\.$',
              ),
              'called reassemble',
              'called paint',
            ],
            handler: (String line) {
              processManager.killPid(pid, ProcessSignal.sigusr2);
              return null;
            },
          ),
          Barrier('Performing hot restart...'.padRight(progressMessageWidth)),
          // This could look like 'Restarted application in 1,237ms.'
          Multiple(
            <Pattern>[RegExp(r'^Restarted application in .+m?s.$'), 'called main', 'called paint'],
            handler: (String line) {
              return 'q';
            },
          ),
          const Barrier('Application finished.'),
        ],
        logging:
            false, // we ignore leading log lines to avoid making this test sensitive to e.g. the help message text
      );
      // We check the output from the app (all starts with "called ...") and the output from the tool
      // (everything else) separately, because their relative timing isn't guaranteed. Their rough timing
      // is verified by the expected transitions above.
      expect(result.stdout.where((String line) => line.startsWith('called ')), <Object>[
        // logs start after we receive the response to sending SIGUSR1
        // SIGUSR1:
        'called reassemble',
        'called paint',
        // SIGUSR2:
        'called main',
        'called paint',
      ]);
      expect(
        result.stdout.where((String line) => !line.startsWith('called ')),
        <Object>[
          // logs start after we receive the response to sending SIGUSR1
          'Performing hot reload...'.padRight(progressMessageWidth),
          startsWith('Reloaded 0 libraries in '),
          'Performing hot restart...'.padRight(progressMessageWidth),
          startsWith('Restarted application in '),
          '', // this newline is the one for after we hit "q"
          'Application finished.',
          'ready',
        ],
        reason:
            'stdout from command ${command.join(' ')} was unexpected, '
            'full Stdout:\n\n${result.stdout.join('\n')}\n\n'
            'Stderr:\n\n${result.stderr.join('\n')}',
      );
      expect(result.exitCode, 0);
    } finally {
      tryToDelete(fileSystem.directory(tempDirectory));
    }
  }, skip: Platform.isWindows); // [intended] Windows doesn't support sending signals.

  testWithoutContext('flutter run can hot reload and hot restart, handle "p" key', () async {
    final String tempDirectory =
        fileSystem.systemTempDirectory
            .createTempSync('flutter_overall_experience_test.')
            .resolveSymbolicLinksSync();
    final String testDirectory = fileSystem.path.join(
      flutterRoot,
      'dev',
      'integration_tests',
      'ui',
    );
    final String testScript = fileSystem.path.join('lib', 'commands.dart');
    final List<String> command = <String>[
      'run',
      '-dflutter-tester',
      '--report-ready',
      '--no-devtools',
      testScript,
    ];
    try {
      final ProcessTestResult result = await runFlutter(
        command,
        testDirectory,
        <Transition>[
          Multiple(
            <Pattern>['Flutter run key commands.', 'called main', 'called paint'],
            handler: (String line) {
              return 'r';
            },
          ),
          Barrier('Performing hot reload...'.padRight(progressMessageWidth), logging: true),
          Multiple(
            <Pattern>['ready', 'called reassemble', 'called paint'],
            handler: (String line) {
              return 'R';
            },
          ),
          Barrier('Performing hot restart...'.padRight(progressMessageWidth)),
          Multiple(
            <Pattern>['ready', 'called main', 'called paint'],
            handler: (String line) {
              return 'p';
            },
          ),
          Multiple(
            <Pattern>['ready', 'called paint', 'called debugPaintSize'],
            handler: (String line) {
              return 'p';
            },
          ),
          Multiple(
            <Pattern>['ready', 'called paint'],
            handler: (String line) {
              return 'q';
            },
          ),
          const Barrier('Application finished.'),
        ],
        logging:
            false, // we ignore leading log lines to avoid making this test sensitive to e.g. the help message text
      );
      // We check the output from the app (all starts with "called ...") and the output from the tool
      // (everything else) separately, because their relative timing isn't guaranteed. Their rough timing
      // is verified by the expected transitions above.
      expect(result.stdout.where((String line) => line.startsWith('called ')), <Object>[
        // logs start after we initiate the hot reload
        // hot reload:
        'called reassemble',
        'called paint',
        // hot restart:
        'called main',
        'called paint',
        // debugPaintSizeEnabled = true:
        'called paint',
        'called debugPaintSize',
        // debugPaintSizeEnabled = false:
        'called paint',
      ]);
      expect(
        result.stdout.where((String line) => !line.startsWith('called ')),
        <Object>[
          // logs start after we receive the response to hitting "r"
          'Performing hot reload...'.padRight(progressMessageWidth),
          startsWith('Reloaded 0 libraries in '),
          'ready',
          '', // this newline is the one for after we hit "R"
          'Performing hot restart...'.padRight(progressMessageWidth),
          startsWith('Restarted application in '),
          'ready',
          '', // newline for after we hit "p" the first time
          'ready',
          '', // newline for after we hit "p" the second time
          'ready',
          '', // this newline is the one for after we hit "q"
          'Application finished.',
          'ready',
        ],
        reason:
            'stdout from command ${command.join(' ')} was unexpected, '
            'full Stdout:\n\n${result.stdout.join('\n')}\n\n'
            'Stderr:\n\n${result.stderr.join('\n')}',
      );
      expect(result.exitCode, 0);
    } finally {
      tryToDelete(fileSystem.directory(tempDirectory));
    }
  });

  testWithoutContext('flutter error messages include a DevTools link', () async {
    final String testDirectory = fileSystem.path.join(
      flutterRoot,
      'dev',
      'integration_tests',
      'ui',
    );
    final String tempDirectory =
        fileSystem.systemTempDirectory
            .createTempSync('flutter_overall_experience_test.')
            .resolveSymbolicLinksSync();
    final String testScript = fileSystem.path.join('lib', 'overflow.dart');
    try {
      final ProcessTestResult result = await runFlutter(
        <String>['run', '-dflutter-tester', testScript],
        testDirectory,
        <Transition>[
          Barrier(RegExp(r'^A Dart VM Service on Flutter test device is available at: ')),
          Barrier(
            RegExp(
              r'^The Flutter DevTools debugger and profiler on Flutter test device is available at: ',
            ),
            handler: (String line) {
              return 'r';
            },
          ),
          Barrier('Performing hot reload...'.padRight(progressMessageWidth), logging: true),
          Barrier(
            RegExp(r'^Reloaded 0 libraries in [0-9]+ms.'),
            handler: (String line) {
              return 'q';
            },
          ),
        ],
        logging: false,
      );
      expect(result.exitCode, 0);
      expect(result.stdout, <Object>[
        startsWith('Performing hot reload...'),
        '',
        '══╡ EXCEPTION CAUGHT BY RENDERING LIBRARY ╞═════════════════════════════════════════════════════════',
        'The following assertion was thrown during layout:',
        'A RenderFlex overflowed by 69200 pixels on the right.',
        '',
        'The relevant error-causing widget was:',
        matches(RegExp(r'^  Row.+/dev/integration_tests/ui/lib/overflow\.dart:32:18$')),
        '',
        'To inspect this widget in Flutter DevTools, visit:',
        startsWith('http'),
        '',
        'The overflowing RenderFlex has an orientation of Axis.horizontal.',
        'The edge of the RenderFlex that is overflowing has been marked in the rendering with a yellow and',
        'black striped pattern. This is usually caused by the contents being too big for the RenderFlex.',
        'Consider applying a flex factor (e.g. using an Expanded widget) to force the children of the',
        'RenderFlex to fit within the available space instead of being sized to their natural size.',
        'This is considered an error condition because it indicates that there is content that cannot be',
        'seen. If the content is legitimately bigger than the available space, consider clipping it with a',
        'ClipRect widget before putting it in the flex, or using a scrollable container rather than a Flex,',
        'like a ListView.',
        matches(RegExp(r'^The specific RenderFlex in question is: RenderFlex#..... OVERFLOWING:$')),
        startsWith('  creator: Row ← Test ← '),
        contains(' ← '),
        endsWith(' ⋯'),
        '  parentData: <none> (can use size)',
        '  constraints: BoxConstraints(w=800.0, h=600.0)',
        '  size: Size(800.0, 600.0)',
        '  direction: horizontal',
        '  mainAxisAlignment: start',
        '  mainAxisSize: max',
        '  crossAxisAlignment: center',
        '  textDirection: ltr',
        '  verticalDirection: down',
        '  spacing: 0.0',
        '◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤',
        '════════════════════════════════════════════════════════════════════════════════════════════════════',
        '',
        startsWith('Reloaded 0 libraries in '),
        '',
        'Application finished.',
      ]);
    } finally {
      tryToDelete(fileSystem.directory(tempDirectory));
    }
  });

  testWithoutContext('flutter run help output', () async {
    // This test enables all logging so that it checks the exact text of starting up an application.
    // The idea is to verify that we're not outputting spurious messages.
    // WHEN EDITING THIS TEST PLEASE CAREFULLY CONSIDER WHETHER THE NEW OUTPUT IS AN IMPROVEMENT.
    final String testDirectory = fileSystem.path.join(flutterRoot, 'examples', 'hello_world');
    final RegExp finalLine = RegExp(r'^The Flutter DevTools');
    final ProcessTestResult result = await runFlutter(
      <String>['run', '-dflutter-tester'],
      testDirectory,
      <Transition>[
        Barrier(
          finalLine,
          handler: (String line) {
            return 'h';
          },
        ),
        Barrier(
          finalLine,
          handler: (String line) {
            return 'q';
          },
        ),
        const Barrier('Application finished.'),
      ],
    );
    expect(result.exitCode, 0);
    expect(result.stderr, isEmpty);
    expect(
      result.stdout,
      containsAllInOrder(<Object>[
        startsWith('Launching '),
        startsWith('Syncing files to device Flutter test device...'),
        '',
        'Flutter run key commands.',
        startsWith('r Hot reload.'),
        'R Hot restart.',
        'h List all available interactive commands.',
        'd Detach (terminate "flutter run" but leave application running).',
        'c Clear the screen',
        'q Quit (terminate the application on the device).',
        '',
        startsWith('A Dart VM Service on Flutter test device is available at: http://'),
        startsWith(
          'The Flutter DevTools debugger and profiler on Flutter test device is available at: http://',
        ),
        '',
        'Flutter run key commands.',
        startsWith('r Hot reload.'),
        'R Hot restart.',
        'v Open Flutter DevTools.',
        'w Dump widget hierarchy to the console.                                               (debugDumpApp)',
        't Dump rendering tree to the console.                                          (debugDumpRenderTree)',
        'L Dump layer tree to the console.                                               (debugDumpLayerTree)',
        'f Dump focus tree to the console.                                               (debugDumpFocusTree)',
        'S Dump accessibility tree in traversal order.                                   (debugDumpSemantics)',
        'U Dump accessibility tree in inverse hit test order.                            (debugDumpSemantics)',
        'i Toggle widget inspector.                                  (WidgetsApp.showWidgetInspectorOverride)',
        'p Toggle the display of construction lines.                                  (debugPaintSizeEnabled)',
        'I Toggle oversized image inversion.                                     (debugInvertOversizedImages)',
        'o Simulate different operating systems.                                      (defaultTargetPlatform)',
        'b Toggle platform brightness (dark and light mode).                        (debugBrightnessOverride)',
        'P Toggle performance overlay.                                    (WidgetsApp.showPerformanceOverlay)',
        'a Toggle timeline events for all widget build methods.                    (debugProfileWidgetBuilds)',
        'M Write SkSL shaders to a unique file in the project directory.',
        'g Run source code generators.',
        'h Repeat this help message.',
        'd Detach (terminate "flutter run" but leave application running).',
        'c Clear the screen',
        'q Quit (terminate the application on the device).',
        '',
        startsWith('A Dart VM Service on Flutter test device is available at: http://'),
        startsWith(
          'The Flutter DevTools debugger and profiler on Flutter test device is available at: http://',
        ),
        '',
        'Application finished.',
      ]),
    );
  });
}
