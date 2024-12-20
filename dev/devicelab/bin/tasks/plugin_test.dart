// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/tasks/plugin_tests.dart';

Future<void> main() async {
  await task(
    combine(<TaskFunction>[
      PluginTest('apk', <String>['-a', 'java', '--platforms=android']).call,
      PluginTest('apk', <String>['-a', 'kotlin', '--platforms=android']).call,
      // Test that Dart-only plugins are supported.
      PluginTest('apk', <String>['--platforms=android'], dartOnlyPlugin: true).call,
      // Test that FFI plugins are supported.
      PluginTest('apk', <String>['--platforms=android'], template: 'plugin_ffi').call,
    ]),
  );
}
