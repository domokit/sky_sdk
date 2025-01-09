// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:process/process.dart';

import '../base/io.dart';
import '../base/logger.dart';
import '../base/process.dart';

/// Wraps iproxy command line tool port forwarding.
///
/// See https://github.com/libimobiledevice/libusbmuxd.
class IProxy {
  IProxy({
    required String iproxyPath,
    required Logger logger,
    required ProcessManager processManager,
    required MapEntry<String, String> dyLdLibEntry,
  }) : _dyLdLibEntry = dyLdLibEntry,
       _processUtils = ProcessUtils(processManager: processManager, logger: logger),
       _logger = logger,
       _iproxyPath = iproxyPath;

  /// Create a [IProxy] for testing.
  ///
  /// This specifies the path to iproxy as 'iproxy` and the dyLdLibEntry as
  /// 'DYLD_LIBRARY_PATH: /path/to/libs'.
  factory IProxy.test({required Logger logger, required ProcessManager processManager}) {
    return IProxy(
      iproxyPath: 'iproxy',
      logger: logger,
      processManager: processManager,
      dyLdLibEntry: const MapEntry<String, String>('DYLD_LIBRARY_PATH', '/path/to/libs'),
    );
  }

  final String _iproxyPath;
  final ProcessUtils _processUtils;
  final Logger _logger;
  final MapEntry<String, String> _dyLdLibEntry;

  Future<Process> forward(int devicePort, int hostPort, String deviceId) {
    // Usage: iproxy LOCAL_PORT:DEVICE_PORT --udid UDID
    return _processUtils.start(<String>[
      _iproxyPath,
      '$hostPort:$devicePort',
      '--udid',
      deviceId,
      if (_logger.isVerbose) '--debug',
    ], environment: Map<String, String>.fromEntries(<MapEntry<String, String>>[_dyLdLibEntry]));
  }
}
