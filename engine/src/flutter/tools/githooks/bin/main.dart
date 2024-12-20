// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:githooks/githooks.dart';

Future<void> main(List<String> args) async {
  io.exitCode = await run(args);
}
