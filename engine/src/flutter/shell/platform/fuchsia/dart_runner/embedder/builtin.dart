// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

library fuchsia_builtin;

import 'dart:async';
import 'dart:io';
import 'dart:_internal' show VMLibraryHooks;

// Corelib 'print' implementation.
void _print(arg) {
  _Logger._printString(arg.toString());
  try {
    // If stdout is connected, print to it as well.
    stdout.writeln(arg);
  } on FileSystemException catch (_) {
    // Some Fuchsia applications will not have stdout connected.
  }
}

class _Logger {
  @pragma('vm:external-name', 'Logger_PrintString')
  external static void _printString(String s);
}

@pragma('vm:entry-point')
late String _rawScript;

Uri _scriptUri() {
  if (_rawScript.startsWith('http:') ||
      _rawScript.startsWith('https:') ||
      _rawScript.startsWith('file:')) {
    return Uri.parse(_rawScript);
  } else {
    return Uri.base.resolveUri(Uri.file(_rawScript));
  }
}

@pragma('vm:external-name', 'ScheduleMicrotask')
external void _scheduleMicrotask(void callback());

@pragma('vm:entry-point')
_getScheduleMicrotaskClosure() => _scheduleMicrotask;

@pragma('vm:entry-point')
_setupHooks() {
  VMLibraryHooks.platformScript = _scriptUri;
}

@pragma('vm:entry-point')
_getPrintClosure() => _print;

typedef EchoStringCallback = String? Function(String? str);
late EchoStringCallback? receiveEchoStringCallback;

@pragma('vm:entry-point')
String? _receiveEchoString(String? str) {
  return receiveEchoStringCallback?.call(str);
}
