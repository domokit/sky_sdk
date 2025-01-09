// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:path/path.dart' as path;

import '../run_command.dart';
import '../utils.dart';

/// Runs the skp_generator from the flutter/tests repo.
///
/// See also the customer_tests shard.
///
/// Generated SKPs are ditched, this just verifies that it can run without failure.
Future<void> skpGeneratorTestsRunner() async {
  printProgress('${green}Running skp_generator from flutter/tests$reset');
  final Directory checkout = Directory.systemTemp.createTempSync('flutter_skp_generator.');
  await runCommand('git', <String>[
    '-c',
    'core.longPaths=true',
    'clone',
    'https://github.com/flutter/tests.git',
    '.',
  ], workingDirectory: checkout.path);
  await runCommand(
    './build.sh',
    const <String>[],
    workingDirectory: path.join(checkout.path, 'skp_generator'),
  );
}
