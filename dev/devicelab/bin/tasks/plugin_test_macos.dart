// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/tasks/plugin_tests.dart';

Future<void> main() async {
  await task(
    combine(<TaskFunction>[
      PluginTest('macos', <String>['--platforms=macos']).call,
      // Test that Dart-only plugins are supported.
      PluginTest('macos', <String>['--platforms=macos'], dartOnlyPlugin: true).call,
      // Test that shared darwin directories are supported.
      PluginTest('macos', <String>['--platforms=ios,macos'], sharedDarwinSource: true).call,
      // Test that FFI plugins are supported.
      PluginTest('macos', <String>['--platforms=macos'], template: 'plugin_ffi').call,
    ]),
  );
}
