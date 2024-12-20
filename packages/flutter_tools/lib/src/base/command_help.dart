// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'logger.dart';
import 'platform.dart';
import 'terminal.dart';

const String fire = '🔥';
const int maxLineWidth = 84;

/// Encapsulates the help text construction and printing.
class CommandHelp {
  CommandHelp({
    required Logger logger,
    required AnsiTerminal terminal,
    required Platform platform,
    required OutputPreferences outputPreferences,
  }) : _logger = logger,
       _terminal = terminal,
       _platform = platform,
       _outputPreferences = outputPreferences;

  final Logger _logger;

  final AnsiTerminal _terminal;

  final Platform _platform;

  final OutputPreferences _outputPreferences;

  // COMMANDS IN ALPHABETICAL ORDER.
  // Uppercase first, then lowercase.
  // When updating this, update all the tests in command_help_test.dart accordingly.

  late final CommandHelpOption I = _makeOption(
    'I',
    'Toggle oversized image inversion.',
    'debugInvertOversizedImages',
  );

  late final CommandHelpOption L = _makeOption(
    'L',
    'Dump layer tree to the console.',
    'debugDumpLayerTree',
  );

  late final CommandHelpOption M = _makeOption(
    'M',
    'Write SkSL shaders to a unique file in the project directory.',
  );

  late final CommandHelpOption P = _makeOption(
    'P',
    'Toggle performance overlay.',
    'WidgetsApp.showPerformanceOverlay',
  );

  late final CommandHelpOption R = _makeOption('R', 'Hot restart.');

  late final CommandHelpOption S = _makeOption(
    'S',
    'Dump accessibility tree in traversal order.',
    'debugDumpSemantics',
  );

  late final CommandHelpOption U = _makeOption(
    'U',
    'Dump accessibility tree in inverse hit test order.',
    'debugDumpSemantics',
  );

  late final CommandHelpOption a = _makeOption(
    'a',
    'Toggle timeline events for all widget build methods.',
    'debugProfileWidgetBuilds',
  );

  late final CommandHelpOption b = _makeOption(
    'b',
    'Toggle platform brightness (dark and light mode).',
    'debugBrightnessOverride',
  );

  late final CommandHelpOption c = _makeOption('c', 'Clear the screen');

  late final CommandHelpOption d = _makeOption(
    'd',
    'Detach (terminate "flutter run" but leave application running).',
  );

  late final CommandHelpOption f = _makeOption(
    'f',
    'Dump focus tree to the console.',
    'debugDumpFocusTree',
  );

  late final CommandHelpOption g = _makeOption('g', 'Run source code generators.');

  late final CommandHelpOption hWithDetails = _makeOption('h', 'Repeat this help message.');

  late final CommandHelpOption hWithoutDetails = _makeOption(
    'h',
    'List all available interactive commands.',
  );

  late final CommandHelpOption i = _makeOption(
    'i',
    'Toggle widget inspector.',
    'WidgetsApp.showWidgetInspectorOverride',
  );

  late final CommandHelpOption k = _makeOption('k', 'Toggle CanvasKit rendering.');

  late final CommandHelpOption o = _makeOption(
    'o',
    'Simulate different operating systems.',
    'defaultTargetPlatform',
  );

  late final CommandHelpOption p = _makeOption(
    'p',
    'Toggle the display of construction lines.',
    'debugPaintSizeEnabled',
  );

  late final CommandHelpOption q = _makeOption(
    'q',
    'Quit (terminate the application on the device).',
  );

  late final CommandHelpOption r = _makeOption('r', 'Hot reload. $fire$fire$fire');

  late final CommandHelpOption s = _makeOption('s', 'Save a screenshot to flutter.png.');

  late final CommandHelpOption t = _makeOption(
    't',
    'Dump rendering tree to the console.',
    'debugDumpRenderTree',
  );

  late final CommandHelpOption v = _makeOption('v', 'Open Flutter DevTools.');

  late final CommandHelpOption w = _makeOption(
    'w',
    'Dump widget hierarchy to the console.',
    'debugDumpApp',
  );

  // When updating the list above, see the notes above the list regarding order
  // and tests.

  CommandHelpOption _makeOption(String key, String description, [String inParenthesis = '']) {
    return CommandHelpOption(
      key,
      description,
      inParenthesis: inParenthesis,
      logger: _logger,
      terminal: _terminal,
      platform: _platform,
      outputPreferences: _outputPreferences,
    );
  }
}

/// Encapsulates printing help text for a single option.
class CommandHelpOption {
  CommandHelpOption(
    this.key,
    this.description, {
    this.inParenthesis = '',
    required Logger logger,
    required Terminal terminal,
    required Platform platform,
    required OutputPreferences outputPreferences,
  }) : _logger = logger,
       _terminal = terminal,
       _platform = platform,
       _outputPreferences = outputPreferences;

  final Logger _logger;

  final Terminal _terminal;

  final Platform _platform;

  final OutputPreferences _outputPreferences;

  /// The key associated with this command.
  final String key;

  /// A description of what this command does.
  final String description;

  /// Text shown in parenthesis to give the context.
  final String inParenthesis;

  bool get _hasTextInParenthesis => inParenthesis.isNotEmpty;

  int get _rawMessageLength => key.length + description.length;

  @override
  String toString() {
    final StringBuffer message = StringBuffer();
    message.writeAll(<String>[_terminal.bolden(key), description], ' ');
    if (!_hasTextInParenthesis) {
      return message.toString();
    }

    bool wrap = false;
    final int maxWidth = math.max(_outputPreferences.wrapColumn, maxLineWidth);
    final int adjustedMessageLength =
        _platform.stdoutSupportsAnsi ? _rawMessageLength + 1 : message.length;
    int width = maxWidth - adjustedMessageLength;
    final String parentheticalText = '($inParenthesis)';
    if (width < parentheticalText.length) {
      width = maxWidth;
      wrap = true;
    }
    if (wrap) {
      message.write('\n');
    }
    // pad according to the raw text
    message.write(''.padLeft(width - parentheticalText.length));
    message.write(_terminal.color(parentheticalText, TerminalColor.grey));

    // Some terminals seem to have a buggy implementation of the SGR ANSI escape
    // codes and seem to require that we explicitly request "normal intensity"
    // at the end of the line to prevent the next line comes out bold, despite
    // the fact that the line already contains a "normal intensity" code.
    // This doesn't make much sense but has been reproduced by multiple users.
    // See: https://github.com/flutter/flutter/issues/52204
    if (_terminal.supportsColor) {
      message.write(AnsiTerminal.resetBold);
    }

    return message.toString();
  }

  void print() {
    _logger.printStatus(toString());
  }
}
