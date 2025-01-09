// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';

import 'package:file/file.dart';
import 'package:flutter_tools/src/base/io.dart';

import '../src/common.dart';
import 'test_utils.dart';

final String flutterRootPath = getFlutterRoot();
final Directory flutterRoot = fileSystem.directory(flutterRootPath);

Future<void> main() async {
  // Regression test for https://github.com/flutter/flutter/issues/132592
  test('flutter/bin/dart updates the Dart SDK without hanging', () async {
    // Run the Dart entrypoint once to ensure the Dart SDK is downloaded.
    await runDartBatch();

    expect(dartSdkStamp.existsSync(), true);

    // Remove the Dart SDK stamp and run the Dart entrypoint again to trigger
    // the Dart SDK update.
    dartSdkStamp.deleteSync();
    final Future<String> runFuture = runDartBatch();
    final Timer timer = Timer(const Duration(minutes: 5), () {
      // This print is useful for people debugging this test. Normally we would
      // avoid printing in a test but this is an exception because it's useful
      // ambient information.
      // ignore: avoid_print
      print(
        'The Dart batch entrypoint did not complete after 5 minutes. '
        'Historically this is a sign that 7-Zip zip extraction is waiting for '
        'the user to confirm they would like to overwrite files. '
        "This likely means the test isn't a flake and will fail. "
        'See: https://github.com/flutter/flutter/issues/132592',
      );
    });

    final String output = await runFuture;
    timer.cancel();

    // Check the Dart SDK was re-downloaded and extracted.
    // If 7-Zip is installed, unexpected overwrites causes this to hang.
    // If 7-Zip is not installed, unexpected overwrites results in error messages.
    // See: https://github.com/flutter/flutter/issues/132592
    expect(dartSdkStamp.existsSync(), true);
    expect(output, contains('Downloading Dart SDK from Flutter engine ...'));
    // Do not assert on the exact unzipping method, as this could change on CI
    expect(output, contains(RegExp(r'Expanding downloaded archive with (.*)...')));
    expect(output, isNot(contains('Use the -Force parameter' /* Luke */)));
  }, skip: !platform.isWindows); // [intended] Only Windows uses the batch entrypoint
}

Future<String> runDartBatch() async {
  String output = '';
  final Process process = await processManager.start(<String>[dartBatch.path]);
  final Future<Object?> stdoutFuture = process.stdout.transform<String>(utf8.decoder).forEach((
    String str,
  ) {
    output += str;
  });
  final Future<Object?> stderrFuture = process.stderr.transform<String>(utf8.decoder).forEach((
    String str,
  ) {
    output += str;
  });

  // Wait for the output to complete
  await Future.wait(<Future<Object?>>[stdoutFuture, stderrFuture]);
  // Ensure child exited successfully
  expect(
    await process.exitCode,
    0,
    reason:
        'child process exited with code ${await process.exitCode}, and '
        'output:\n$output',
  );

  // Check the Dart tool prints the expected output.
  expect(output, contains('A command-line utility for Dart development.'));
  expect(output, contains('Usage: dart <command|dart-file> [arguments]'));

  return output;
}

// The executable batch entrypoint for the Dart binary.
File get dartBatch {
  return flutterRoot.childDirectory('bin').childFile('dart.bat').absolute;
}

// The Dart SDK's stamp file.
File get dartSdkStamp {
  return flutterRoot
      .childDirectory('bin')
      .childDirectory('cache')
      .childFile('engine-dart-sdk.stamp')
      .absolute;
}
