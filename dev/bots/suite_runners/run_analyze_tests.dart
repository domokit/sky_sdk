// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:path/path.dart' as path;

import '../run_command.dart';
import '../utils.dart';

Future<void> analyzeRunner() async {
  printProgress('${green}Running analysis testing$reset');
  await runCommand('dart', <String>[
    '--enable-asserts',
    path.join(flutterRoot, 'dev', 'bots', 'analyze.dart'),
  ], workingDirectory: flutterRoot);
}
