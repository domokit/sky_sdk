// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/process.dart';
import 'package:flutter_tools/src/base/terminal.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/fakes.dart';

void main() {
  group('output preferences', () {
    testWithoutContext('can wrap output', () async {
      final BufferLogger bufferLogger = BufferLogger(
        outputPreferences: OutputPreferences.test(wrapText: true, wrapColumn: 40),
        terminal: TestTerminal(platform: FakePlatform()..stdoutSupportsAnsi = true),
      );
      bufferLogger.printStatus('0123456789' * 8);

      expect(bufferLogger.statusText, equals('${'0123456789' * 4}\n' * 2));
    });

    testWithoutContext('can turn off wrapping', () async {
      final BufferLogger bufferLogger = BufferLogger(
        outputPreferences: OutputPreferences.test(),
        terminal: TestTerminal(platform: FakePlatform()..stdoutSupportsAnsi = true),
      );
      final String testString = '0123456789' * 20;
      bufferLogger.printStatus(testString);

      expect(bufferLogger.statusText, equals('$testString\n'));
    });
  });

  group('ANSI coloring, bold, and clearing', () {
    late AnsiTerminal terminal;

    setUp(() {
      terminal = AnsiTerminal(
        stdio: Stdio(), // Danger, using real stdio.
        platform: FakePlatform()..stdoutSupportsAnsi = true,
      );
    });

    testWithoutContext('adding colors works', () {
      for (final TerminalColor color in TerminalColor.values) {
        expect(
          terminal.color('output', color),
          equals('${AnsiTerminal.colorCode(color)}output${AnsiTerminal.resetColor}'),
        );
      }
    });

    testWithoutContext('adding bold works', () {
      expect(
        terminal.bolden('output'),
        equals('${AnsiTerminal.bold}output${AnsiTerminal.resetBold}'),
      );
    });

    testWithoutContext('nesting bold within color works', () {
      expect(
        terminal.color(terminal.bolden('output'), TerminalColor.blue),
        equals(
          '${AnsiTerminal.blue}${AnsiTerminal.bold}output${AnsiTerminal.resetBold}${AnsiTerminal.resetColor}',
        ),
      );
      expect(
        terminal.color('non-bold ${terminal.bolden('output')} also non-bold', TerminalColor.blue),
        equals(
          '${AnsiTerminal.blue}non-bold ${AnsiTerminal.bold}output${AnsiTerminal.resetBold} also non-bold${AnsiTerminal.resetColor}',
        ),
      );
    });

    testWithoutContext('nesting color within bold works', () {
      expect(
        terminal.bolden(terminal.color('output', TerminalColor.blue)),
        equals(
          '${AnsiTerminal.bold}${AnsiTerminal.blue}output${AnsiTerminal.resetColor}${AnsiTerminal.resetBold}',
        ),
      );
      expect(
        terminal.bolden('non-color ${terminal.color('output', TerminalColor.blue)} also non-color'),
        equals(
          '${AnsiTerminal.bold}non-color ${AnsiTerminal.blue}output${AnsiTerminal.resetColor} also non-color${AnsiTerminal.resetBold}',
        ),
      );
    });

    testWithoutContext('nesting color within color works', () {
      expect(
        terminal.color(terminal.color('output', TerminalColor.blue), TerminalColor.magenta),
        equals(
          '${AnsiTerminal.magenta}${AnsiTerminal.blue}output${AnsiTerminal.resetColor}${AnsiTerminal.magenta}${AnsiTerminal.resetColor}',
        ),
      );
      expect(
        terminal.color(
          'magenta ${terminal.color('output', TerminalColor.blue)} also magenta',
          TerminalColor.magenta,
        ),
        equals(
          '${AnsiTerminal.magenta}magenta ${AnsiTerminal.blue}output${AnsiTerminal.resetColor}${AnsiTerminal.magenta} also magenta${AnsiTerminal.resetColor}',
        ),
      );
    });

    testWithoutContext('nesting bold within bold works', () {
      expect(
        terminal.bolden(terminal.bolden('output')),
        equals('${AnsiTerminal.bold}output${AnsiTerminal.resetBold}'),
      );
      expect(
        terminal.bolden('bold ${terminal.bolden('output')} still bold'),
        equals('${AnsiTerminal.bold}bold output still bold${AnsiTerminal.resetBold}'),
      );
    });

    testWithoutContext('clearing lines works', () {
      expect(
        terminal.clearLines(3),
        equals(
          '${AnsiTerminal.cursorBeginningOfLineCode}'
          '${AnsiTerminal.clearEntireLineCode}'
          '${AnsiTerminal.cursorUpLineCode}'
          '${AnsiTerminal.clearEntireLineCode}'
          '${AnsiTerminal.cursorUpLineCode}'
          '${AnsiTerminal.clearEntireLineCode}',
        ),
      );

      expect(
        terminal.clearLines(1),
        equals(
          '${AnsiTerminal.cursorBeginningOfLineCode}'
          '${AnsiTerminal.clearEntireLineCode}',
        ),
      );
    });

    testWithoutContext('clearing lines when color is not supported does not work', () {
      terminal = AnsiTerminal(
        stdio: Stdio(), // Danger, using real stdio.
        platform: FakePlatform()..stdoutSupportsAnsi = false,
      );
      expect(terminal.clearLines(3), equals(''));
    });
  });

  group('character input prompt', () {
    late AnsiTerminal terminalUnderTest;

    setUp(() {
      terminalUnderTest = TestTerminal(stdio: FakeStdio());
    });

    testWithoutContext('character prompt throws if usesTerminalUi is false', () async {
      expect(
        terminalUnderTest.promptForCharInput(
          <String>['a', 'b', 'c'],
          prompt: 'Please choose something',
          logger: BufferLogger.test(),
        ),
        throwsStateError,
      );
    });

    testWithoutContext('character prompt', () async {
      final BufferLogger bufferLogger = BufferLogger(
        terminal: terminalUnderTest,
        outputPreferences: OutputPreferences.test(),
      );
      terminalUnderTest.usesTerminalUi = true;
      mockStdInStream =
          Stream<String>.fromFutures(<Future<String>>[
            Future<String>.value('d'), // Not in accepted list.
            Future<String>.value('\n'), // Not in accepted list
            Future<String>.value('b'),
          ]).asBroadcastStream();
      final String choice = await terminalUnderTest.promptForCharInput(
        <String>['a', 'b', 'c'],
        prompt: 'Please choose something',
        logger: bufferLogger,
      );
      expect(choice, 'b');
      expect(
        bufferLogger.statusText,
        'Please choose something [a|b|c]: d\n'
        'Please choose something [a|b|c]: \n'
        'Please choose something [a|b|c]: b\n',
      );
    });

    testWithoutContext('default character choice without displayAcceptedCharacters', () async {
      final BufferLogger bufferLogger = BufferLogger(
        terminal: terminalUnderTest,
        outputPreferences: OutputPreferences.test(),
      );
      terminalUnderTest.usesTerminalUi = true;
      mockStdInStream =
          Stream<String>.fromFutures(<Future<String>>[
            Future<String>.value('\n'), // Not in accepted list
          ]).asBroadcastStream();
      final String choice = await terminalUnderTest.promptForCharInput(
        <String>['a', 'b', 'c'],
        prompt: 'Please choose something',
        displayAcceptedCharacters: false,
        defaultChoiceIndex: 1, // which is b.
        logger: bufferLogger,
      );

      expect(choice, 'b');
      expect(bufferLogger.statusText, 'Please choose something: \n');
    });

    testWithoutContext('Does not set single char mode when a terminal is not attached', () {
      final Stdio stdio = FakeStdio()..stdinHasTerminal = false;
      final AnsiTerminal ansiTerminal = AnsiTerminal(stdio: stdio, platform: const LocalPlatform());

      expect(() => ansiTerminal.singleCharMode = true, returnsNormally);
    });
  });

  testWithoutContext('AnsiTerminal.preferredStyle', () {
    final Stdio stdio = FakeStdio();
    expect(
      AnsiTerminal(stdio: stdio, platform: const LocalPlatform()).preferredStyle,
      0,
    ); // Defaults to 0 for backwards compatibility.

    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018),
      ).preferredStyle,
      0,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 2),
      ).preferredStyle,
      1,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 3),
      ).preferredStyle,
      2,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 4),
      ).preferredStyle,
      3,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 5),
      ).preferredStyle,
      4,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 6),
      ).preferredStyle,
      5,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 7),
      ).preferredStyle,
      5,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 8),
      ).preferredStyle,
      0,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 9),
      ).preferredStyle,
      1,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 10),
      ).preferredStyle,
      2,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 11),
      ).preferredStyle,
      3,
    );

    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 1, 1),
      ).preferredStyle,
      0,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 2, 1),
      ).preferredStyle,
      1,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 3, 1),
      ).preferredStyle,
      2,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 4, 1),
      ).preferredStyle,
      3,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 5, 1),
      ).preferredStyle,
      4,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 6, 1),
      ).preferredStyle,
      6,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 7, 1),
      ).preferredStyle,
      6,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 8, 1),
      ).preferredStyle,
      0,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 9, 1),
      ).preferredStyle,
      1,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 10, 1),
      ).preferredStyle,
      2,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 11, 1),
      ).preferredStyle,
      3,
    );

    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 1, 23),
      ).preferredStyle,
      0,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 2, 23),
      ).preferredStyle,
      1,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 3, 23),
      ).preferredStyle,
      2,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 4, 23),
      ).preferredStyle,
      3,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 5, 23),
      ).preferredStyle,
      4,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 6, 23),
      ).preferredStyle,
      28,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 7, 23),
      ).preferredStyle,
      28,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 8, 23),
      ).preferredStyle,
      0,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 9, 23),
      ).preferredStyle,
      1,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 10, 23),
      ).preferredStyle,
      2,
    );
    expect(
      AnsiTerminal(
        stdio: stdio,
        platform: const LocalPlatform(),
        now: DateTime(2018, 1, 11, 23),
      ).preferredStyle,
      3,
    );
  });

  testWithoutContext('set singleCharMode resilient to StdinException', () async {
    final FakeStdio stdio = FakeStdio();
    final AnsiTerminal terminal = AnsiTerminal(stdio: stdio, platform: const LocalPlatform());
    stdio.stdinHasTerminal = true;
    stdio._stdin =
        FakeStdin()
          ..echoModeCallback =
              (bool _) =>
                  throw const StdinException(
                    'Error setting terminal echo mode, OS Error: The handle is invalid.',
                  );
    terminal.singleCharMode = true;
  });

  testWithoutContext('singleCharMode is reset by shutdown hook', () {
    final ShutdownHooks shutdownHooks = ShutdownHooks();
    final FakeStdio stdio = FakeStdio();
    final AnsiTerminal terminal = AnsiTerminal(
      stdio: stdio,
      platform: const LocalPlatform(),
      shutdownHooks: shutdownHooks,
    );
    stdio.stdinHasTerminal = true;
    stdio._stdin = FakeStdin();

    terminal.singleCharMode = true;
    shutdownHooks.runShutdownHooks(BufferLogger.test());
    expect(terminal.singleCharMode, false);
  });
}

late Stream<String> mockStdInStream;

class TestTerminal extends AnsiTerminal {
  TestTerminal({Stdio? stdio, super.platform = const LocalPlatform(), DateTime? now})
    : super(stdio: stdio ?? Stdio(), now: now ?? DateTime(2018));

  @override
  Stream<String> get keystrokes {
    return mockStdInStream;
  }

  bool _singleCharMode = false;

  @override
  bool get singleCharMode => _singleCharMode;

  void Function(bool newMode)? _singleCharModeCallback;

  @override
  set singleCharMode(bool newMode) {
    _singleCharMode = newMode;
    if (_singleCharModeCallback != null) {
      _singleCharModeCallback!(newMode);
    }
  }

  @override
  int get preferredStyle => 0;
}

class FakeStdio extends Fake implements Stdio {
  Stream<List<int>>? _stdin;

  @override
  Stream<List<int>> get stdin {
    if (_stdin != null) {
      return _stdin!;
    }
    throw UnimplementedError('stdin');
  }

  @override
  bool stdinHasTerminal = false;
}
