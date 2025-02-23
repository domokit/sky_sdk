// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('FlutterVersion.version contains the current version', () {
    expect(FlutterVersion.version, const String.fromEnvironment('FLUTTER_VERSION'));
  });

  test('FlutterVersion.channel contains the current channel', () {
    expect(FlutterVersion.channel, const String.fromEnvironment('FLUTTER_CHANNEL'));
  });

  test('FlutterVersion.gitUrl contains the current git URL', () {
    expect(FlutterVersion.gitUrl, const String.fromEnvironment('FLUTTER_GIT_URL'));
  });

  test('FlutterVersion.frameworkRevision contains the current framework revision', () {
    expect(
      FlutterVersion.frameworkRevision,
      const String.fromEnvironment('FLUTTER_FRAMEWORK_REVISION'),
    );
  });

  test('FlutterVersion.engineRevision contains the current engine revision', () {
    expect(FlutterVersion.engineRevision, const String.fromEnvironment('FLUTTER_ENGINE_REVISION'));
  });

  test('FlutterVersion.dartVersion contains the current Dart version', () {
    expect(FlutterVersion.dartVersion, const String.fromEnvironment('FLUTTER_DART_VERSION'));
  });
}
