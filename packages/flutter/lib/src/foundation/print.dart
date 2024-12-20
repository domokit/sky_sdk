// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file implements debugPrint in terms of print, so avoiding
// calling "print" is sort of a non-starter here...
// ignore_for_file: avoid_print

/// @docImport 'package:flutter/widgets.dart';
library;

import 'dart:async';
import 'dart:collection';

/// Signature for [debugPrint] implementations.
///
/// If a [wrapWidth] is provided, each line of the [message] is word-wrapped to
/// that width. (Lines may be separated by newline characters, as in '\n'.)
///
/// By default, this function very crudely attempts to throttle the rate at
/// which messages are sent to avoid data loss on Android. This means that
/// interleaving calls to this function (directly or indirectly via, e.g.,
/// [debugDumpRenderTree] or [debugDumpApp]) and to the Dart [print] method can
/// result in out-of-order messages in the logs.
///
/// The implementation of this function can be replaced by setting the
/// [debugPrint] variable to a new implementation that matches the
/// [DebugPrintCallback] signature. For example, flutter_test does this.
///
/// The default value is [debugPrintThrottled]. For a version that acts
/// identically but does not throttle, use [debugPrintSynchronously].
typedef DebugPrintCallback = void Function(String? message, {int? wrapWidth});

/// Prints a message to the console, which you can access using the "flutter"
/// tool's "logs" command ("flutter logs").
///
/// The [debugPrint] function logs to console even in [release mode](https://docs.flutter.dev/testing/build-modes#release).
/// As per convention, calls to [debugPrint] should be within a debug mode check or an assert:
/// ```dart
/// if (kDebugMode) {
///   debugPrint('A useful message');
/// }
/// ```
///
/// See also:
///
///   * [DebugPrintCallback], for function parameters and usage details.
///   * [debugPrintThrottled], the default implementation.
DebugPrintCallback debugPrint = debugPrintThrottled;

/// Alternative implementation of [debugPrint] that does not throttle.
/// Used by tests.
void debugPrintSynchronously(String? message, {int? wrapWidth}) {
  if (message != null && wrapWidth != null) {
    print(
      message
          .split('\n')
          .expand<String>((String line) => debugWordWrap(line, wrapWidth))
          .join('\n'),
    );
  } else {
    print(message);
  }
}

/// Implementation of [debugPrint] that throttles messages. This avoids dropping
/// messages on platforms that rate-limit their logging (for example, Android).
///
/// If `wrapWidth` is not null, the message is wrapped using [debugWordWrap].
void debugPrintThrottled(String? message, {int? wrapWidth}) {
  final List<String> messageLines = message?.split('\n') ?? <String>['null'];
  if (wrapWidth != null) {
    _debugPrintBuffer.addAll(
      messageLines.expand<String>((String line) => debugWordWrap(line, wrapWidth)),
    );
  } else {
    _debugPrintBuffer.addAll(messageLines);
  }
  if (!_debugPrintScheduled) {
    _debugPrintTask();
  }
}

int _debugPrintedCharacters = 0;
const int _kDebugPrintCapacity = 12 * 1024;
const Duration _kDebugPrintPauseTime = Duration(seconds: 1);
final Queue<String> _debugPrintBuffer = Queue<String>();
final Stopwatch _debugPrintStopwatch = Stopwatch(); // flutter_ignore: stopwatch (see analyze.dart)
// Ignore context: This is not used in tests, only for throttled logging.
Completer<void>? _debugPrintCompleter;
bool _debugPrintScheduled = false;
void _debugPrintTask() {
  _debugPrintScheduled = false;
  if (_debugPrintStopwatch.elapsed > _kDebugPrintPauseTime) {
    _debugPrintStopwatch.stop();
    _debugPrintStopwatch.reset();
    _debugPrintedCharacters = 0;
  }
  while (_debugPrintedCharacters < _kDebugPrintCapacity && _debugPrintBuffer.isNotEmpty) {
    final String line = _debugPrintBuffer.removeFirst();
    _debugPrintedCharacters += line.length; // TODO(ianh): Use the UTF-8 byte length instead
    print(line);
  }
  if (_debugPrintBuffer.isNotEmpty) {
    _debugPrintScheduled = true;
    _debugPrintedCharacters = 0;
    Timer(_kDebugPrintPauseTime, _debugPrintTask);
    _debugPrintCompleter ??= Completer<void>();
  } else {
    _debugPrintStopwatch.start();
    _debugPrintCompleter?.complete();
    _debugPrintCompleter = null;
  }
}

/// A Future that resolves when there is no longer any buffered content being
/// printed by [debugPrintThrottled] (which is the default implementation for
/// [debugPrint], which is used to report errors to the console).
Future<void> get debugPrintDone => _debugPrintCompleter?.future ?? Future<void>.value();

final RegExp _indentPattern = RegExp('^ *(?:[-+*] |[0-9]+[.):] )?');

enum _WordWrapParseMode { inSpace, inWord, atBreak }

/// Wraps the given string at the given width.
///
/// The `message` should not contain newlines (`\n`, U+000A). Strings that may
/// contain newlines should be [String.split] before being wrapped.
///
/// Wrapping occurs at space characters (U+0020). Lines that start with an
/// octothorpe ("#", U+0023) are not wrapped (so for example, Dart stack traces
/// won't be wrapped).
///
/// Subsequent lines attempt to duplicate the indentation of the first line, for
/// example if the first line starts with multiple spaces. In addition, if a
/// `wrapIndent` argument is provided, each line after the first is prefixed by
/// that string.
///
/// This is not suitable for use with arbitrary Unicode text. For example, it
/// doesn't implement UAX #14, can't handle ideographic text, doesn't hyphenate,
/// and so forth. It is only intended for formatting error messages.
///
/// The default [debugPrint] implementation uses this for its line wrapping.
Iterable<String> debugWordWrap(String message, int width, {String wrapIndent = ''}) {
  if (message.length < width || message.trimLeft()[0] == '#') {
    return <String>[message];
  }
  final List<String> wrapped = <String>[];
  final Match prefixMatch = _indentPattern.matchAsPrefix(message)!;
  final String prefix = wrapIndent + ' ' * prefixMatch.group(0)!.length;
  int start = 0;
  int startForLengthCalculations = 0;
  bool addPrefix = false;
  int index = prefix.length;
  _WordWrapParseMode mode = _WordWrapParseMode.inSpace;
  late int lastWordStart;
  int? lastWordEnd;
  while (true) {
    switch (mode) {
      case _WordWrapParseMode
          .inSpace: // at start of break point (or start of line); can't break until next break
        while ((index < message.length) && (message[index] == ' ')) {
          index += 1;
        }
        lastWordStart = index;
        mode = _WordWrapParseMode.inWord;
      case _WordWrapParseMode.inWord: // looking for a good break point
        while ((index < message.length) && (message[index] != ' ')) {
          index += 1;
        }
        mode = _WordWrapParseMode.atBreak;
      case _WordWrapParseMode.atBreak: // at start of break point
        if ((index - startForLengthCalculations > width) || (index == message.length)) {
          // we are over the width line, so break
          if ((index - startForLengthCalculations <= width) || (lastWordEnd == null)) {
            // we should use this point, because either it doesn't actually go over the
            // end (last line), or it does, but there was no earlier break point
            lastWordEnd = index;
          }
          if (addPrefix) {
            wrapped.add(prefix + message.substring(start, lastWordEnd));
          } else {
            wrapped.add(message.substring(start, lastWordEnd));
            addPrefix = true;
          }
          if (lastWordEnd >= message.length) {
            return wrapped;
          }
          // just yielded a line
          if (lastWordEnd == index) {
            // we broke at current position
            // eat all the spaces, then set our start point
            while ((index < message.length) && (message[index] == ' ')) {
              index += 1;
            }
            start = index;
            mode = _WordWrapParseMode.inWord;
          } else {
            // we broke at the previous break point, and we're at the start of a new one
            assert(lastWordStart > lastWordEnd);
            start = lastWordStart;
            mode = _WordWrapParseMode.atBreak;
          }
          startForLengthCalculations = start - prefix.length;
          assert(addPrefix);
          lastWordEnd = null;
        } else {
          // save this break point, we're not yet over the line width
          lastWordEnd = index;
          // skip to the end of this break point
          mode = _WordWrapParseMode.inSpace;
        }
    }
  }
}
