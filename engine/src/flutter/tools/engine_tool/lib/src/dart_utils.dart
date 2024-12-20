// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:path/path.dart' as p;
import 'environment.dart';

/// Returns a dart-sdk/bin directory path that is compatible with the host.
String findDartBinDirectory(Environment env) {
  return p.dirname(env.platform.resolvedExecutable);
}

/// Returns a dart-sdk/bin/dart file pthat that is executable on the host.
String findDartBinary(Environment env) {
  return p.join(findDartBinDirectory(env), 'dart');
}
