// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

bool _supportsAnsi = stdout.supportsAnsiEscapes;
String _green = _supportsAnsi ? '\u001b[1;32m' : '';
String _red = _supportsAnsi ? '\u001b[31m' : '';
String _yellow = _supportsAnsi ? '\u001b[33m' : '';
String _gray = _supportsAnsi ? '\u001b[90m' : '';
String _reset = _supportsAnsi ? '\u001B[0m' : '';

Future<void> step(String msg, Future<void> Function() fn) async {
  stdout.writeln('-> $_green$msg$_reset');
  try {
    await fn();
  } catch (_) {
    stderr.writeln('~~ ${_red}Failed$_reset');
    rethrow;
  } finally {
    stdout.writeln('<- ${_gray}Done$_reset');
  }
}

void _logWithColor(String color, String msg) {
  stdout.writeln('$color$msg$_reset');
}

void log(String msg) {
  _logWithColor(_gray, msg);
}

void logImportant(String msg) {
  stdout.writeln(msg);
}

void logWarning(String msg) {
  _logWithColor(_yellow, msg);
}

void logError(String msg) {
  _logWithColor(_red, msg);
}

final class Panic extends Error {}

Never panic(List<String> messages) {
  messages.forEach(logError);
  throw Panic();
}
