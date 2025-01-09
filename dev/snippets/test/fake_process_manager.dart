// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io';

import 'package:process/process.dart';

class FakeProcessManager extends LocalProcessManager {
  FakeProcessManager({this.stdout = '', this.stderr = '', this.exitCode = 0, this.pid = 1});

  int runs = 0;
  String stdout;
  String stderr;
  int exitCode;
  int pid;

  @override
  ProcessResult runSync(
    List<Object> command, {
    String? workingDirectory,
    Map<String, String>? environment,
    bool includeParentEnvironment = true,
    bool runInShell = false,
    Encoding? stdoutEncoding = systemEncoding,
    Encoding? stderrEncoding = systemEncoding,
  }) {
    runs++;
    return ProcessResult(pid, exitCode, stdout, stderr);
  }
}
