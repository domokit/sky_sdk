// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../test_utils.dart';
import 'project.dart';

/// Spawns a background isolate that prints a debug message.
class BackgroundProject extends Project {
  @override
  final String pubspec = '''
  name: test
  environment:
    sdk: ^3.7.0-0

  dependencies:
    flutter:
      sdk: flutter
  ''';

  @override
  final String main = r'''
  import 'dart:async';
  import 'dart:isolate';

  import 'package:flutter/widgets.dart';
  import 'package:flutter/material.dart';

  void main() {
    Isolate.spawn<void>(background, null, debugName: 'background');
    TestMain();
  }

  void background(void message) {
    TestIsolate();
  }

  class TestMain {
    TestMain() {
      debugPrint('Main thread');
    }
  }

  class TestIsolate {
    TestIsolate() {
      debugPrint('Isolate thread');
    }
  }
  ''';

  void updateTestIsolatePhrase(String message) {
    final String newMainContents = main.replaceFirst('Isolate thread', message);
    writeFile(
      fileSystem.path.join(dir.path, 'lib', 'main.dart'),
      newMainContents,
      writeFutureModifiedDate: true,
    );
  }
}

// Spawns a background isolate that repeats a message.
class RepeatingBackgroundProject extends Project {
  @override
  final String pubspec = '''
  name: test
  environment:
    sdk: ^3.7.0-0

  dependencies:
    flutter:
      sdk: flutter
  ''';

  @override
  final String main = r'''
  import 'dart:async';
  import 'dart:isolate';

  import 'package:flutter/widgets.dart';
  import 'package:flutter/material.dart';

  void main() {
    Isolate.spawn<void>(background, null, debugName: 'background');
    TestMain();
  }

  void background(void message) {
    Timer.periodic(const Duration(milliseconds: 500), (Timer timer) => TestIsolate());
  }

  class TestMain {
    TestMain() {
      debugPrint('Main thread');
    }
  }

  class TestIsolate {
    TestIsolate() {
      debugPrint('Isolate thread');
    }
  }
  ''';

  void updateTestIsolatePhrase(String message) {
    final String newMainContents = main.replaceFirst('Isolate thread', message);
    writeFile(
      fileSystem.path.join(dir.path, 'lib', 'main.dart'),
      newMainContents,
      writeFutureModifiedDate: true,
    );
  }
}
