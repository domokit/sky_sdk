// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io';

import 'package:args/command_runner.dart';

import 'package:flutter_devicelab/command/test.dart';
import 'package:flutter_devicelab/command/upload_results.dart';

final CommandRunner<void> runner =
    CommandRunner<void>('devicelab_runner', 'DeviceLab test runner for recording test results')
      ..addCommand(TestCommand())
      ..addCommand(UploadResultsCommand());

Future<void> main(List<String> rawArgs) async {
  unawaited(
    runner.run(rawArgs).catchError((dynamic error) {
      stderr.writeln('$error\n');
      stderr.writeln('Usage:\n');
      stderr.writeln(runner.usage);
      exit(64); // Exit code 64 indicates a usage error.
    }),
  );
}
