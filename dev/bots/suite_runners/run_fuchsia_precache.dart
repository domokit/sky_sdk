// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../run_command.dart';
import '../utils.dart';

// Runs flutter_precache.
Future<void> fuchsiaPrecacheRunner() async {
  printProgress('${green}Running flutter precache tests$reset');
  await runCommand('flutter', const <String>[
    'config',
    '--enable-fuchsia',
  ], workingDirectory: flutterRoot);
  await runCommand('flutter', const <String>[
    'precache',
    '--flutter_runner',
    '--fuchsia',
    '--no-android',
    '--no-ios',
    '--force',
  ], workingDirectory: flutterRoot);
}
