// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:collection/collection.dart';
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

/// Returns the path to `.gclient` file, or null if it cannot be found.
String? findDotGclient(Environment env) {
  io.Directory directory = env.engine.srcDir;
  io.File? dotGclientFile;
  while (dotGclientFile == null) {
    dotGclientFile = directory.listSync().whereType<io.File>().firstWhereOrNull((file) {
      return p.basename(file.path) == '.gclient';
    });

    final parent = directory.parent;
    if (parent.path == directory.path) {
      break;
    }
    directory = parent;
  }
  return dotGclientFile?.path;
}
