// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:file/memory.dart';
import 'package:platform/platform.dart';
import 'package:pub_semver/pub_semver.dart';
import 'package:snippets/snippets.dart';
import 'package:test/test.dart' hide TypeMatcher, isInstanceOf;

import 'fake_process_manager.dart';

const String testVersionInfo = r'''
{
  "frameworkVersion": "2.5.0-2.0.pre.63",
  "channel": "master",
  "repositoryUrl": "git@github.com:flutter/flutter.git",
  "frameworkRevision": "9b2f6f7f9ab96bb3302f81b814a094f33023e79a",
  "frameworkCommitDate": "2021-07-28 13:03:40 -0700",
  "engineRevision": "0ed62a16f36348e97b2baadd8ccfec3825f80c5d",
  "dartSdkVersion": "2.14.0 (build 2.14.0-360.0.dev)",
  "flutterRoot": "/home/user/flutter"
}
''';

void main() {
  group('FlutterInformation', () {
    late FakeProcessManager fakeProcessManager;
    late FakePlatform fakePlatform;
    late MemoryFileSystem memoryFileSystem;
    late FlutterInformation flutterInformation;

    setUp(() {
      fakeProcessManager = FakeProcessManager();
      memoryFileSystem = MemoryFileSystem();
      fakePlatform = FakePlatform(environment: <String, String>{});
      flutterInformation = FlutterInformation(
        filesystem: memoryFileSystem,
        processManager: fakeProcessManager,
        platform: fakePlatform,
      );
    });

    test('calls out to flutter if FLUTTER_VERSION is not set', () async {
      fakeProcessManager.stdout = testVersionInfo;
      final Map<String, dynamic> info = flutterInformation.getFlutterInformation();
      expect(fakeProcessManager.runs, equals(1));
      expect(info['frameworkVersion'], equals(Version.parse('2.5.0-2.0.pre.63')));
    });
    test("doesn't call out to flutter if FLUTTER_VERSION is set", () async {
      fakePlatform.environment['FLUTTER_VERSION'] = testVersionInfo;
      final Map<String, dynamic> info = flutterInformation.getFlutterInformation();
      expect(fakeProcessManager.runs, equals(0));
      expect(info['frameworkVersion'], equals(Version.parse('2.5.0-2.0.pre.63')));
    });
    test('getFlutterRoot calls out to flutter if FLUTTER_ROOT is not set', () async {
      fakeProcessManager.stdout = testVersionInfo;
      final Directory root = flutterInformation.getFlutterRoot();
      expect(fakeProcessManager.runs, equals(1));
      expect(root.path, equals('/home/user/flutter'));
    });
    test("getFlutterRoot doesn't call out to flutter if FLUTTER_ROOT is set", () async {
      fakePlatform.environment['FLUTTER_ROOT'] = '/home/user/flutter';
      final Directory root = flutterInformation.getFlutterRoot();
      expect(fakeProcessManager.runs, equals(0));
      expect(root.path, equals('/home/user/flutter'));
    });
    test('parses version properly', () async {
      fakePlatform.environment['FLUTTER_VERSION'] = testVersionInfo;
      final Map<String, dynamic> info = flutterInformation.getFlutterInformation();
      expect(info['frameworkVersion'], isNotNull);
      expect(info['frameworkVersion'], equals(Version.parse('2.5.0-2.0.pre.63')));
      expect(info['dartSdkVersion'], isNotNull);
      expect(info['dartSdkVersion'], equals(Version.parse('2.14.0-360.0.dev')));
    });
  });
}
