// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

const String _redBoldUnderline = '\x1B[31;1;4m';
const String _reset = '\x1B[0m';

/// Prints a reminder to stdout to run `gclient sync -D`. Uses colors when
/// stdout supports ANSI escape codes.
void printGclientSyncReminder(String command) {
  final String prefix = io.stdout.supportsAnsiEscapes ? _redBoldUnderline : '';
  final String postfix = io.stdout.supportsAnsiEscapes ? _reset : '';
  io.stderr.writeln('$command: The engine source tree has been updated.');
  io.stderr.writeln('\n${prefix}You may need to run "gclient sync -D"$postfix\n');
}
