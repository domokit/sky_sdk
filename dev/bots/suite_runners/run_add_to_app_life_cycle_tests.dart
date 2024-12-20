// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' show Platform;

import 'package:path/path.dart' as path;

import '../run_command.dart';
import '../utils.dart';

Future<void> addToAppLifeCycleRunner() async {
  if (Platform.isMacOS) {
    printProgress('${green}Running add-to-app life cycle iOS integration tests$reset...');
    final String addToAppDir = path.join(
      flutterRoot,
      'dev',
      'integration_tests',
      'ios_add2app_life_cycle',
    );
    await runCommand('./build_and_test.sh', <String>[], workingDirectory: addToAppDir);
  } else {
    throw Exception('Only iOS has add-to-add lifecycle tests at this time.');
  }
}
