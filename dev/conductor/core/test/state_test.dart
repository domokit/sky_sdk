// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:conductor_core/src/proto/conductor_state.pb.dart' as pb;
import 'package:conductor_core/src/state.dart';
import 'package:file/file.dart';
import 'package:file/memory.dart';

import './common.dart';

void main() {
  test('writeStateToFile() pretty-prints JSON with 2 spaces', () {
    final MemoryFileSystem fileSystem = MemoryFileSystem.test();
    final File stateFile = fileSystem.file('/path/to/statefile.json')..createSync(recursive: true);
    const String candidateBranch = 'flutter-2.3-candidate.0';
    final pb.ConductorState state =
        pb.ConductorState.create()
          ..releaseChannel = 'stable'
          ..releaseVersion = '2.3.4'
          ..engine =
              (pb.Repository.create()
                ..candidateBranch = candidateBranch
                ..upstream =
                    (pb.Remote.create()
                      ..name = 'upstream'
                      ..url = 'git@github.com:flutter/engine.git'))
          ..framework =
              (pb.Repository.create()
                ..candidateBranch = candidateBranch
                ..upstream =
                    (pb.Remote.create()
                      ..name = 'upstream'
                      ..url = 'git@github.com:flutter/flutter.git'));
    writeStateToFile(stateFile, state, <String>['[status] hello world']);
    final String serializedState = stateFile.readAsStringSync();
    const String expectedString = '''
{
  "releaseChannel": "stable",
  "releaseVersion": "2.3.4",
  "engine": {
    "candidateBranch": "flutter-2.3-candidate.0",
    "upstream": {
      "name": "upstream",
      "url": "git@github.com:flutter/engine.git"
    }
  },
  "framework": {
    "candidateBranch": "flutter-2.3-candidate.0",
    "upstream": {
      "name": "upstream",
      "url": "git@github.com:flutter/flutter.git"
    }
  },
  "logs": [
    "[status] hello world"
  ]
}''';
    expect(serializedState, expectedString);
  });
}
