// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dom.dart';

DomElement? _logElement;
late DomElement _logContainer;
List<_LogMessage> _logBuffer = <_LogMessage>[];

class _LogMessage {
  _LogMessage(this.message);

  int duplicateCount = 1;
  final String message;

  @override
  String toString() {
    if (duplicateCount == 1) {
      return message;
    }
    return '${duplicateCount}x $message';
  }
}

/// A drop-in replacement for [print] that prints on the screen into a
/// fixed-positioned element.
///
/// This is useful, for example, for print-debugging on iOS when debugging over
/// USB is not available.
void printOnScreen(Object object) {
  if (_logElement == null) {
    _initialize();
  }

  final String message = '$object';
  if (_logBuffer.isNotEmpty && _logBuffer.last.message == message) {
    _logBuffer.last.duplicateCount += 1;
  } else {
    _logBuffer.add(_LogMessage(message));
  }

  if (_logBuffer.length > 80) {
    _logBuffer = _logBuffer.sublist(_logBuffer.length - 50);
  }

  _logContainer.text = _logBuffer.join('\n');

  // Also log to console for browsers that give you access to it.
  print(message);
}

void _initialize() {
  _logElement = createDomElement('flt-onscreen-log');
  _logElement!.setAttribute('aria-hidden', 'true');
  _logElement!.style
    ..position = 'fixed'
    ..left = '0'
    ..right = '0'
    ..bottom = '0'
    ..height = '25%'
    ..backgroundColor = 'rgba(0, 0, 0, 0.85)'
    ..color = 'white'
    ..fontSize = '8px'
    ..whiteSpace = 'pre-wrap'
    ..overflow = 'hidden'
    ..zIndex = '1000';

  _logContainer = createDomElement('flt-log-container');
  _logContainer.setAttribute('aria-hidden', 'true');
  _logContainer.style
    ..position = 'absolute'
    ..bottom = '0';
  _logElement!.append(_logContainer);

  domDocument.body!.append(_logElement!);
}

/// Dump the current stack to the console using [print] and
/// [defaultStackFilter].
///
/// The current stack is obtained using [StackTrace.current].
///
/// The `maxFrames` argument can be given to limit the stack to the given number
/// of lines. By default, all non-filtered stack lines are shown.
///
/// The `label` argument, if present, will be printed before the stack.
void debugPrintStack({String? label, int? maxFrames}) {
  if (label != null) {
    print(label);
  }
  Iterable<String> lines = StackTrace.current.toString().trimRight().split('\n');
  if (maxFrames != null) {
    lines = lines.take(maxFrames);
  }
  print(defaultStackFilter(lines).join('\n'));
}

/// Converts a stack to a string that is more readable by omitting stack
/// frames that correspond to Dart internals.
///
/// This function expects its input to be in the format used by
/// [StackTrace.toString()]. The output of this function is similar to that
/// format but the frame numbers will not be consecutive (frames are elided)
/// and the final line may be prose rather than a stack frame.
Iterable<String> defaultStackFilter(Iterable<String> frames) {
  const List<String> filteredPackages = <String>['dart:async-patch', 'dart:async', 'dart:_runtime'];
  final RegExp stackParser = RegExp(
    r'^#[0-9]+ +([^.]+).* \(([^/\\]*)[/\\].+:[0-9]+(?::[0-9]+)?\)$',
  );
  final RegExp packageParser = RegExp(r'^([^:]+):(.+)$');
  final List<String> result = <String>[];
  final List<String> skipped = <String>[];
  for (final String line in frames) {
    final Match? match = stackParser.firstMatch(line);
    if (match != null) {
      assert(match.groupCount == 2);
      if (filteredPackages.contains(match.group(2))) {
        final Match? packageMatch = packageParser.firstMatch(match.group(2)!);
        if (packageMatch != null && packageMatch.group(1) == 'package') {
          skipped.add('package ${packageMatch.group(2)}'); // avoid "package package:foo"
        } else {
          skipped.add('package ${match.group(2)}');
        }
        continue;
      }
    }
    result.add(line);
  }
  if (skipped.length == 1) {
    result.add('(elided one frame from ${skipped.single})');
  } else if (skipped.length > 1) {
    final List<String> where = Set<String>.from(skipped).toList()..sort();
    if (where.length > 1) {
      where[where.length - 1] = 'and ${where.last}';
    }
    if (where.length > 2) {
      result.add('(elided ${skipped.length} frames from ${where.join(", ")})');
    } else {
      result.add('(elided ${skipped.length} frames from ${where.join(" ")})');
    }
  }
  return result;
}

String debugIdentify(Object? object) {
  return '${object!.runtimeType}(@${object.hashCode})';
}
