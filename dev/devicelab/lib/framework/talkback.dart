// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:path/path.dart' as path;
import 'package:pub_semver/pub_semver.dart';

String adbPath() {
  final String? androidHome =
      io.Platform.environment['ANDROID_HOME'] ?? io.Platform.environment['ANDROID_SDK_ROOT'];
  if (androidHome == null) {
    return 'adb';
  } else {
    return path.join(androidHome, 'platform-tools', 'adb');
  }
}

Future<Version> getTalkbackVersion() async {
  final io.ProcessResult result = await io.Process.run(adbPath(), const <String>[
    'shell',
    'dumpsys',
    'package',
    'com.google.android.marvin.talkback',
  ]);
  if (result.exitCode != 0) {
    throw Exception(
      'Failed to get TalkBack version: ${result.stdout as String}\n${result.stderr as String}',
    );
  }
  final List<String> lines = (result.stdout as String).split('\n');
  String? version;
  for (final String line in lines) {
    if (line.contains('versionName')) {
      version = line.replaceAll(RegExp(r'\s*versionName='), '');
      break;
    }
  }
  if (version == null) {
    throw Exception('Unable to determine TalkBack version.');
  }

  // Android doesn't quite use semver, so convert the version string to semver form.
  final RegExp startVersion = RegExp(
    r'(?<major>\d+)\.(?<minor>\d+)\.(?<patch>\d+)(\.(?<build>\d+))?',
  );
  final RegExpMatch? match = startVersion.firstMatch(version);
  if (match == null) {
    return Version(0, 0, 0);
  }
  return Version(
    int.parse(match.namedGroup('major')!),
    int.parse(match.namedGroup('minor')!),
    int.parse(match.namedGroup('patch')!),
    build: match.namedGroup('build'),
  );
}

Future<void> enableTalkBack() async {
  final io.Process run = await io.Process.start(adbPath(), const <String>[
    'shell',
    'settings',
    'put',
    'secure',
    'enabled_accessibility_services',
    'com.google.android.marvin.talkback/com.google.android.marvin.talkback.TalkBackService',
  ]);
  await run.exitCode;

  print('TalkBack version is ${await getTalkbackVersion()}');
}

Future<void> disableTalkBack() async {
  final io.Process run = await io.Process.start(adbPath(), const <String>[
    'shell',
    'settings',
    'put',
    'secure',
    'enabled_accessibility_services',
    'null',
  ]);
  await run.exitCode;
}
