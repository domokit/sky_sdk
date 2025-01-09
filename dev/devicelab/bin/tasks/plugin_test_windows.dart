// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/tasks/plugin_tests.dart';

Future<void> main() async {
  await task(
    combine(<TaskFunction>[
      PluginTest('windows', <String>['--platforms=windows']).call,
      // Test that Dart-only plugins are supported.
      PluginTest('windows', <String>['--platforms=windows'], dartOnlyPlugin: true).call,
      // Test that FFI plugins are supported.
      PluginTest('windows', <String>['--platforms=windows'], template: 'plugin_ffi').call,
    ]),
  );
}
