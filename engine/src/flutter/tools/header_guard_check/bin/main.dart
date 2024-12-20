// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:header_guard_check/header_guard_check.dart';

Future<int> main(List<String> arguments) async {
  final int result = await HeaderGuardCheck.fromCommandLine(arguments).run();
  if (result != 0) {
    io.exit(result);
  }
  return result;
}
