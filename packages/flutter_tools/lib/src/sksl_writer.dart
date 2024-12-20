// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:file/file.dart';

import 'base/file_system.dart';
import 'base/logger.dart';
import 'build_info.dart';
import 'convert.dart';
import 'device.dart';
import 'globals.dart' as globals;

Future<String?> sharedSkSlWriter(
  Device device,
  Map<String, Object?>? data, {
  File? outputFile,
  Logger? logger,
}) async {
  logger ??= globals.logger;
  if (data == null || data.isEmpty) {
    logger.printStatus(
      'No data was received. To ensure SkSL data can be generated use a '
      'physical device then:\n'
      '  1. Pass "--cache-sksl" as an argument to flutter run.\n'
      '  2. Interact with the application to force shaders to be compiled.\n',
    );
    return null;
  }
  if (outputFile == null) {
    outputFile = globals.fsUtils.getUniqueFile(globals.fs.currentDirectory, 'flutter', 'sksl.json');
  } else if (!outputFile.parent.existsSync()) {
    outputFile.parent.createSync(recursive: true);
  }
  // Convert android sub-platforms to single target platform.
  TargetPlatform targetPlatform = await device.targetPlatform;
  switch (targetPlatform) {
    case TargetPlatform.android_arm:
    case TargetPlatform.android_arm64:
    case TargetPlatform.android_x64:
    case TargetPlatform.android_x86:
      targetPlatform = TargetPlatform.android;
    case TargetPlatform.android:
    case TargetPlatform.darwin:
    case TargetPlatform.ios:
    case TargetPlatform.fuchsia_arm64:
    case TargetPlatform.fuchsia_x64:
    case TargetPlatform.linux_arm64:
    case TargetPlatform.linux_x64:
    case TargetPlatform.tester:
    case TargetPlatform.web_javascript:
    case TargetPlatform.windows_x64:
    case TargetPlatform.windows_arm64:
      break;
  }
  final Map<String, Object> manifest = <String, Object>{
    'platform': getNameForTargetPlatform(targetPlatform),
    'name': device.name,
    'engineRevision': globals.flutterVersion.engineRevision,
    'data': data,
  };
  outputFile.writeAsStringSync(json.encode(manifest));
  logger.printStatus('Wrote SkSL data to ${outputFile.path}.');
  return outputFile.path;
}
