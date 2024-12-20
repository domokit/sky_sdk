// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Runs clang-tidy on files with changes.
//
// Basic Usage:
// dart bin/main.dart --compile-commands <path to compile_commands.json>
// dart bin/main.dart --target-variant <engine-variant>
//
// User environment variable FLUTTER_LINT_ALL to run on all files.

import 'dart:io' as io;

import 'package:clang_tidy/clang_tidy.dart';

Future<int> main(List<String> arguments) async {
  final int result = await ClangTidy.fromCommandLine(arguments).run();
  if (result != 0) {
    io.exit(result);
  }
  return result;
}
