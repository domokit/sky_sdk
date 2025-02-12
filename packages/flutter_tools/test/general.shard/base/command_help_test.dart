// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/command_help.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/terminal.dart' show AnsiTerminal, OutputPreferences;

import '../../src/common.dart';
import '../../src/fakes.dart';

CommandHelp _createCommandHelp({required bool ansi, required int wrapColumn}) {
  final Platform platform = FakePlatform(stdoutSupportsAnsi: ansi);
  return CommandHelp(
    logger: BufferLogger.test(),
    terminal: AnsiTerminal(stdio: FakeStdio(), platform: platform),
    platform: platform,
    outputPreferences: OutputPreferences.test(showColor: ansi, wrapColumn: wrapColumn),
  );
}

// Used to use the message length in different scenarios in a DRY way
void _testMessageLength({
  required bool stdoutSupportsAnsi,
  required int maxTestLineLength,
  required int wrapColumn,
}) {
  final CommandHelp commandHelp = _createCommandHelp(
    ansi: stdoutSupportsAnsi,
    wrapColumn: wrapColumn,
  );

  int expectedWidth = maxTestLineLength;

  if (stdoutSupportsAnsi) {
    const int ansiMetaCharactersLength = 33;
    expectedWidth += ansiMetaCharactersLength;
  }

  expect(commandHelp.I.toString().length, lessThanOrEqualTo(expectedWidth));
  expect(commandHelp.L.toString().length, lessThanOrEqualTo(expectedWidth));
  expect(commandHelp.P.toString().length, lessThanOrEqualTo(expectedWidth));
  expect(commandHelp.R.toString().length, lessThanOrEqualTo(expectedWidth));
  expect(commandHelp.S.toString().length, lessThanOrEqualTo(expectedWidth));
  expect(commandHelp.U.toString().length, lessThanOrEqualTo(expectedWidth));
  expect(commandHelp.a.toString().length, lessThanOrEqualTo(expectedWidth));
  expect(commandHelp.b.toString().length, lessThanOrEqualTo(expectedWidth));
  expect(commandHelp.c.toString().length, lessThanOrEqualTo(expectedWidth));
  expect(commandHelp.d.toString().length, lessThanOrEqualTo(expectedWidth));
  expect(commandHelp.f.toString().length, lessThanOrEqualTo(expectedWidth));
  expect(commandHelp.g.toString().length, lessThanOrEqualTo(expectedWidth));
  expect(commandHelp.hWithDetails.toString().length, lessThanOrEqualTo(expectedWidth));
  expect(commandHelp.hWithoutDetails.toString().length, lessThanOrEqualTo(expectedWidth));
  expect(commandHelp.i.toString().length, lessThanOrEqualTo(expectedWidth));
  expect(commandHelp.k.toString().length, lessThanOrEqualTo(expectedWidth));
  expect(commandHelp.o.toString().length, lessThanOrEqualTo(expectedWidth));
  expect(commandHelp.p.toString().length, lessThanOrEqualTo(expectedWidth));
  expect(commandHelp.q.toString().length, lessThanOrEqualTo(expectedWidth));
  expect(commandHelp.r.toString().length, lessThanOrEqualTo(expectedWidth));
  expect(commandHelp.s.toString().length, lessThanOrEqualTo(expectedWidth));
  expect(commandHelp.t.toString().length, lessThanOrEqualTo(expectedWidth));
  expect(commandHelp.w.toString().length, lessThanOrEqualTo(expectedWidth));
}

void main() {
  group('CommandHelp', () {
    group('toString', () {
      testWithoutContext('ends with a resetBold when it has parenthetical text', () {
        // This is apparently required to work around bugs in some terminal clients.
        final Platform platform = FakePlatform(stdoutSupportsAnsi: true);
        final AnsiTerminal terminal = AnsiTerminal(stdio: FakeStdio(), platform: platform);

        final CommandHelpOption commandHelpOption = CommandHelpOption(
          'tester',
          'for testing',
          platform: platform,
          outputPreferences: OutputPreferences.test(showColor: true),
          terminal: terminal,
          logger: BufferLogger.test(),
          inParenthesis: 'Parenthetical',
        );
        expect(commandHelpOption.toString(), endsWith(AnsiTerminal.resetBold));
      });

      testWithoutContext('should have a bold command key', () {
        final CommandHelp commandHelp = _createCommandHelp(ansi: true, wrapColumn: maxLineWidth);

        expect(commandHelp.I.toString(), startsWith('\x1B[1mI\x1B[22m'));
        expect(commandHelp.L.toString(), startsWith('\x1B[1mL\x1B[22m'));
        expect(commandHelp.P.toString(), startsWith('\x1B[1mP\x1B[22m'));
        expect(commandHelp.R.toString(), startsWith('\x1B[1mR\x1B[22m'));
        expect(commandHelp.S.toString(), startsWith('\x1B[1mS\x1B[22m'));
        expect(commandHelp.U.toString(), startsWith('\x1B[1mU\x1B[22m'));
        expect(commandHelp.a.toString(), startsWith('\x1B[1ma\x1B[22m'));
        expect(commandHelp.b.toString(), startsWith('\x1B[1mb\x1B[22m'));
        expect(commandHelp.c.toString(), startsWith('\x1B[1mc\x1B[22m'));
        expect(commandHelp.d.toString(), startsWith('\x1B[1md\x1B[22m'));
        expect(commandHelp.g.toString(), startsWith('\x1B[1mg\x1B[22m'));
        expect(commandHelp.hWithDetails.toString(), startsWith('\x1B[1mh\x1B[22m'));
        expect(commandHelp.hWithoutDetails.toString(), startsWith('\x1B[1mh\x1B[22m'));
        expect(commandHelp.i.toString(), startsWith('\x1B[1mi\x1B[22m'));
        expect(commandHelp.k.toString(), startsWith('\x1B[1mk\x1B[22m'));
        expect(commandHelp.o.toString(), startsWith('\x1B[1mo\x1B[22m'));
        expect(commandHelp.p.toString(), startsWith('\x1B[1mp\x1B[22m'));
        expect(commandHelp.q.toString(), startsWith('\x1B[1mq\x1B[22m'));
        expect(commandHelp.r.toString(), startsWith('\x1B[1mr\x1B[22m'));
        expect(commandHelp.s.toString(), startsWith('\x1B[1ms\x1B[22m'));
        expect(commandHelp.t.toString(), startsWith('\x1B[1mt\x1B[22m'));
        expect(commandHelp.w.toString(), startsWith('\x1B[1mw\x1B[22m'));
      });

      testWithoutContext('commands that should have a grey bolden parenthetical text', () {
        final CommandHelp commandHelp = _createCommandHelp(ansi: true, wrapColumn: maxLineWidth);

        expect(commandHelp.I.toString(), contains('\x1B[90m(debugInvertOversizedImages)\x1B[39m'));
        expect(commandHelp.L.toString(), contains('\x1B[90m(debugDumpLayerTree)\x1B[39m'));
        expect(
          commandHelp.P.toString(),
          contains('\x1B[90m(WidgetsApp.showPerformanceOverlay)\x1B[39m'),
        );
        expect(commandHelp.S.toString(), contains('\x1B[90m(debugDumpSemantics)\x1B[39m'));
        expect(commandHelp.U.toString(), contains('\x1B[90m(debugDumpSemantics)\x1B[39m'));
        expect(commandHelp.a.toString(), contains('\x1B[90m(debugProfileWidgetBuilds)\x1B[39m'));
        expect(commandHelp.b.toString(), contains('\x1B[90m(debugBrightnessOverride)\x1B[39m'));
        expect(commandHelp.f.toString(), contains('\x1B[90m(debugDumpFocusTree)\x1B[39m'));
        expect(
          commandHelp.i.toString(),
          contains('\x1B[90m(WidgetsApp.showWidgetInspectorOverride)\x1B[39m'),
        );
        expect(commandHelp.o.toString(), contains('\x1B[90m(defaultTargetPlatform)\x1B[39m'));
        expect(commandHelp.p.toString(), contains('\x1B[90m(debugPaintSizeEnabled)\x1B[39m'));
        expect(commandHelp.t.toString(), contains('\x1B[90m(debugDumpRenderTree)\x1B[39m'));
        expect(commandHelp.w.toString(), contains('\x1B[90m(debugDumpApp)\x1B[39m'));
      });

      testWithoutContext(
        'should not create a help text longer than maxLineWidth without ansi support',
        () {
          _testMessageLength(
            stdoutSupportsAnsi: false,
            wrapColumn: 0,
            maxTestLineLength: maxLineWidth,
          );
        },
      );

      testWithoutContext(
        'should not create a help text longer than maxLineWidth with ansi support',
        () {
          _testMessageLength(
            stdoutSupportsAnsi: true,
            wrapColumn: 0,
            maxTestLineLength: maxLineWidth,
          );
        },
      );

      testWithoutContext(
        'should not create a help text longer than outputPreferences.wrapColumn without ansi support',
        () {
          _testMessageLength(
            stdoutSupportsAnsi: false,
            wrapColumn: OutputPreferences.kDefaultTerminalColumns,
            maxTestLineLength: OutputPreferences.kDefaultTerminalColumns,
          );
        },
      );

      testWithoutContext(
        'should not create a help text longer than outputPreferences.wrapColumn with ansi support',
        () {
          _testMessageLength(
            stdoutSupportsAnsi: true,
            wrapColumn: OutputPreferences.kDefaultTerminalColumns,
            maxTestLineLength: OutputPreferences.kDefaultTerminalColumns,
          );
        },
      );

      testWithoutContext('should create the correct help text with ansi support', () {
        final CommandHelp commandHelp = _createCommandHelp(ansi: true, wrapColumn: maxLineWidth);

        // The trailing \x1B[22m is to work around reported bugs in some terminal clients.
        expect(
          commandHelp.I.toString(),
          equals(
            '\x1B[1mI\x1B[22m Toggle oversized image inversion.                     \x1B[90m(debugInvertOversizedImages)\x1B[39m\x1B[22m',
          ),
        );
        expect(
          commandHelp.L.toString(),
          equals(
            '\x1B[1mL\x1B[22m Dump layer tree to the console.                               \x1B[90m(debugDumpLayerTree)\x1B[39m\x1B[22m',
          ),
        );
        expect(
          commandHelp.P.toString(),
          equals(
            '\x1B[1mP\x1B[22m Toggle performance overlay.                    \x1B[90m(WidgetsApp.showPerformanceOverlay)\x1B[39m\x1B[22m',
          ),
        );
        expect(commandHelp.R.toString(), equals('\x1B[1mR\x1B[22m Hot restart.'));
        expect(
          commandHelp.S.toString(),
          equals(
            '\x1B[1mS\x1B[22m Dump accessibility tree in traversal order.                   \x1B[90m(debugDumpSemantics)\x1B[39m\x1B[22m',
          ),
        );
        expect(
          commandHelp.U.toString(),
          equals(
            '\x1B[1mU\x1B[22m Dump accessibility tree in inverse hit test order.            \x1B[90m(debugDumpSemantics)\x1B[39m\x1B[22m',
          ),
        );
        expect(
          commandHelp.a.toString(),
          equals(
            '\x1B[1ma\x1B[22m Toggle timeline events for all widget build methods.    \x1B[90m(debugProfileWidgetBuilds)\x1B[39m\x1B[22m',
          ),
        );
        expect(
          commandHelp.b.toString(),
          equals(
            '\x1B[1mb\x1B[22m Toggle platform brightness (dark and light mode).        \x1B[90m(debugBrightnessOverride)\x1B[39m\x1B[22m',
          ),
        );
        expect(commandHelp.c.toString(), equals('\x1B[1mc\x1B[22m Clear the screen'));
        expect(
          commandHelp.d.toString(),
          equals(
            '\x1B[1md\x1B[22m Detach (terminate "flutter run" but leave application running).',
          ),
        );
        expect(
          commandHelp.f.toString(),
          equals(
            '\x1B[1mf\x1B[22m Dump focus tree to the console.                               \x1B[90m(debugDumpFocusTree)\x1B[39m\x1B[22m',
          ),
        );
        expect(commandHelp.g.toString(), equals('\x1B[1mg\x1B[22m Run source code generators.'));
        expect(
          commandHelp.hWithDetails.toString(),
          equals('\x1B[1mh\x1B[22m Repeat this help message.'),
        );
        expect(
          commandHelp.hWithoutDetails.toString(),
          equals('\x1B[1mh\x1B[22m List all available interactive commands.'),
        );
        expect(
          commandHelp.i.toString(),
          equals(
            '\x1B[1mi\x1B[22m Toggle widget inspector.                  \x1B[90m(WidgetsApp.showWidgetInspectorOverride)\x1B[39m\x1B[22m',
          ),
        );
        expect(
          commandHelp.o.toString(),
          equals(
            '\x1B[1mo\x1B[22m Simulate different operating systems.                      \x1B[90m(defaultTargetPlatform)\x1B[39m\x1B[22m',
          ),
        );
        expect(
          commandHelp.p.toString(),
          equals(
            '\x1B[1mp\x1B[22m Toggle the display of construction lines.                  \x1B[90m(debugPaintSizeEnabled)\x1B[39m\x1B[22m',
          ),
        );
        expect(
          commandHelp.q.toString(),
          equals('\x1B[1mq\x1B[22m Quit (terminate the application on the device).'),
        );
        expect(commandHelp.r.toString(), equals('\x1B[1mr\x1B[22m Hot reload. $fire$fire$fire'));
        expect(
          commandHelp.s.toString(),
          equals('\x1B[1ms\x1B[22m Save a screenshot to flutter.png.'),
        );
        expect(
          commandHelp.t.toString(),
          equals(
            '\x1B[1mt\x1B[22m Dump rendering tree to the console.                          \x1B[90m(debugDumpRenderTree)\x1B[39m\x1B[22m',
          ),
        );
        expect(commandHelp.v.toString(), equals('\x1B[1mv\x1B[22m Open Flutter DevTools.'));
        expect(
          commandHelp.w.toString(),
          equals(
            '\x1B[1mw\x1B[22m Dump widget hierarchy to the console.                               \x1B[90m(debugDumpApp)\x1B[39m\x1B[22m',
          ),
        );
      });

      testWithoutContext('should create the correct help text without ansi support', () {
        final CommandHelp commandHelp = _createCommandHelp(ansi: false, wrapColumn: maxLineWidth);

        expect(
          commandHelp.I.toString(),
          equals(
            'I Toggle oversized image inversion.                     (debugInvertOversizedImages)',
          ),
        );
        expect(
          commandHelp.L.toString(),
          equals(
            'L Dump layer tree to the console.                               (debugDumpLayerTree)',
          ),
        );
        expect(
          commandHelp.P.toString(),
          equals(
            'P Toggle performance overlay.                    (WidgetsApp.showPerformanceOverlay)',
          ),
        );
        expect(commandHelp.R.toString(), equals('R Hot restart.'));
        expect(
          commandHelp.S.toString(),
          equals(
            'S Dump accessibility tree in traversal order.                   (debugDumpSemantics)',
          ),
        );
        expect(
          commandHelp.U.toString(),
          equals(
            'U Dump accessibility tree in inverse hit test order.            (debugDumpSemantics)',
          ),
        );
        expect(
          commandHelp.a.toString(),
          equals(
            'a Toggle timeline events for all widget build methods.    (debugProfileWidgetBuilds)',
          ),
        );
        expect(
          commandHelp.b.toString(),
          equals(
            'b Toggle platform brightness (dark and light mode).        (debugBrightnessOverride)',
          ),
        );
        expect(commandHelp.c.toString(), equals('c Clear the screen'));
        expect(
          commandHelp.d.toString(),
          equals('d Detach (terminate "flutter run" but leave application running).'),
        );
        expect(commandHelp.g.toString(), equals('g Run source code generators.'));
        expect(commandHelp.hWithDetails.toString(), equals('h Repeat this help message.'));
        expect(
          commandHelp.hWithoutDetails.toString(),
          equals('h List all available interactive commands.'),
        );
        expect(
          commandHelp.i.toString(),
          equals(
            'i Toggle widget inspector.                  (WidgetsApp.showWidgetInspectorOverride)',
          ),
        );
        expect(
          commandHelp.o.toString(),
          equals(
            'o Simulate different operating systems.                      (defaultTargetPlatform)',
          ),
        );
        expect(
          commandHelp.p.toString(),
          equals(
            'p Toggle the display of construction lines.                  (debugPaintSizeEnabled)',
          ),
        );
        expect(
          commandHelp.q.toString(),
          equals('q Quit (terminate the application on the device).'),
        );
        expect(commandHelp.r.toString(), equals('r Hot reload. $fire$fire$fire'));
        expect(commandHelp.s.toString(), equals('s Save a screenshot to flutter.png.'));
        expect(
          commandHelp.t.toString(),
          equals(
            't Dump rendering tree to the console.                          (debugDumpRenderTree)',
          ),
        );
        expect(commandHelp.v.toString(), equals('v Open Flutter DevTools.'));
        expect(
          commandHelp.w.toString(),
          equals(
            'w Dump widget hierarchy to the console.                               (debugDumpApp)',
          ),
        );
      });
    });
  });
}
