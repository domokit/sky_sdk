// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_devicelab/framework/host_agent.dart';
import 'package:platform/platform.dart';

import 'common.dart';

void main() {
  late FileSystem fs;
  setUp(() {
    fs = MemoryFileSystem();
    hostAgent.resetDumpDirectory();
  });

  tearDown(() {
    hostAgent.resetDumpDirectory();
  });

  group('dump directory', () {
    test('set by environment', () async {
      final Directory environmentDir = fs.directory(fs.path.join('home', 'logs'));
      final FakePlatform fakePlatform = FakePlatform(
        environment: <String, String>{'FLUTTER_LOGS_DIR': environmentDir.path},
        operatingSystem: 'windows',
      );
      final HostAgent agent = HostAgent(platform: fakePlatform, fileSystem: fs);

      expect(agent.dumpDirectory!.existsSync(), isTrue);
      expect(agent.dumpDirectory!.path, environmentDir.path);
    });

    test('not set by environment', () async {
      final FakePlatform fakePlatform = FakePlatform(
        environment: <String, String>{},
        operatingSystem: 'windows',
      );
      final HostAgent agent = HostAgent(platform: fakePlatform, fileSystem: fs);

      expect(agent.dumpDirectory, null);
    });
  });
}
