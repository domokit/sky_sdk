// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/platform.dart';

import '../../src/common.dart';

void _expectPlatformsEqual(Platform actual, Platform expected) {
  expect(actual.numberOfProcessors, expected.numberOfProcessors);
  expect(actual.pathSeparator, expected.pathSeparator);
  expect(actual.operatingSystem, expected.operatingSystem);
  expect(actual.operatingSystemVersion, expected.operatingSystemVersion);
  expect(actual.localHostname, expected.localHostname);
  expect(actual.environment, expected.environment);
  expect(actual.executable, expected.executable);
  expect(actual.resolvedExecutable, expected.resolvedExecutable);
  expect(actual.script, expected.script);
  expect(actual.executableArguments, expected.executableArguments);
  expect(actual.packageConfig, expected.packageConfig);
  expect(actual.version, expected.version);
  expect(actual.localeName, expected.localeName);
}

void main() {
  group('FakePlatform.fromPlatform', () {
    late FakePlatform fake;
    late LocalPlatform local;

    setUp(() {
      local = const LocalPlatform();
      fake = FakePlatform.fromPlatform(local);
    });

    testWithoutContext('copiesAllProperties', () {
      _expectPlatformsEqual(fake, local);
    });

    testWithoutContext('convertsPropertiesToMutable', () {
      final String key = fake.environment.keys.first;

      expect(fake.environment[key], local.environment[key]);
      fake.environment[key] = 'FAKE';
      expect(fake.environment[key], 'FAKE');

      expect(fake.executableArguments.length, local.executableArguments.length);
      fake.executableArguments.add('ARG');
      expect(fake.executableArguments.last, 'ARG');
    });
  });
}
