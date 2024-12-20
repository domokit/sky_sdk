// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:path/path.dart' as p;
import 'package:test/test.dart';

void main() {
  late io.Directory emptyDir;

  void setUp() {
    emptyDir = io.Directory.systemTemp.createTempSync('engine_repo_tools.test');
  }

  void tearDown() {
    emptyDir.deleteSync(recursive: true);
  }

  group('Engine.fromSrcPath', () {
    group('should fail when', () {
      test('the path does not end in `${p.separator}src`', () {
        setUp();
        try {
          expect(
            () => Engine.fromSrcPath(emptyDir.path),
            throwsA(const TypeMatcher<InvalidEngineException>()),
          );
        } finally {
          tearDown();
        }
      });

      test('the path does not exist', () {
        setUp();
        try {
          expect(
            () => Engine.fromSrcPath(p.join(emptyDir.path, 'src')),
            throwsA(const TypeMatcher<InvalidEngineException>()),
          );
        } finally {
          tearDown();
        }
      });

      test('the path does not contain a "flutter" directory', () {
        setUp();
        try {
          final io.Directory srcDir = io.Directory(p.join(emptyDir.path, 'src'))..createSync();
          expect(
            () => Engine.fromSrcPath(srcDir.path),
            throwsA(const TypeMatcher<InvalidEngineException>()),
          );
        } finally {
          tearDown();
        }
      });

      test('returns an Engine', () {
        setUp();
        try {
          final io.Directory srcDir = io.Directory(p.join(emptyDir.path, 'src'))..createSync();
          io.Directory(p.join(srcDir.path, 'flutter')).createSync();
          io.Directory(p.join(srcDir.path, 'out')).createSync();

          final Engine engine = Engine.fromSrcPath(srcDir.path);

          expect(engine.srcDir.path, srcDir.path);
          expect(engine.flutterDir.path, p.join(srcDir.path, 'flutter'));
          expect(engine.outDir.path, p.join(srcDir.path, 'out'));
        } finally {
          tearDown();
        }
      });
    });
  });

  group('Engine.findWithin', () {
    late io.Directory emptyDir;

    void setUp() {
      emptyDir = io.Directory.systemTemp.createTempSync('engine_repo_tools.test');
    }

    void tearDown() {
      emptyDir.deleteSync(recursive: true);
    }

    group('should fail when', () {
      test('the path does not contain a "src" directory', () {
        setUp();
        try {
          expect(() => Engine.findWithin(emptyDir.path), throwsStateError);
        } finally {
          tearDown();
        }
      });

      test('the path contains a "src" directory but it is not an engine root', () {
        setUp();
        try {
          final io.Directory srcDir = io.Directory(p.join(emptyDir.path, 'src'))..createSync();
          expect(() => Engine.findWithin(srcDir.path), throwsStateError);
        } finally {
          tearDown();
        }
      });

      test('returns an Engine', () {
        setUp();
        try {
          final io.Directory srcDir = io.Directory(p.join(emptyDir.path, 'src'))..createSync();
          io.Directory(p.join(srcDir.path, 'flutter')).createSync();
          io.Directory(p.join(srcDir.path, 'out')).createSync();

          final Engine engine = Engine.findWithin(srcDir.path);

          expect(engine.srcDir.path, srcDir.path);
          expect(engine.flutterDir.path, p.join(srcDir.path, 'flutter'));
          expect(engine.outDir.path, p.join(srcDir.path, 'out'));
        } finally {
          tearDown();
        }
      });

      test('returns an Engine even if a "src" directory exists deeper in the tree', () {
        // It's common to have "src" directories, so if we have something like:
        //  /Users/.../engine/src/foo/bar/src/baz
        //
        // And we use `Engine.findWithin('/Users/.../engine/src/flutter/bar/src/baz')`,
        // we should still find the engine (in this case, the engine root is
        // `/Users/.../engine/src`).
        setUp();
        try {
          final io.Directory srcDir = io.Directory(p.join(emptyDir.path, 'src'))..createSync();
          io.Directory(p.join(srcDir.path, 'flutter')).createSync();
          io.Directory(p.join(srcDir.path, 'out')).createSync();

          final io.Directory nestedSrcDir = io.Directory(
            p.join(srcDir.path, 'flutter', 'bar', 'src', 'baz'),
          )..createSync(recursive: true);

          final Engine engine = Engine.findWithin(nestedSrcDir.path);

          expect(engine.srcDir.path, srcDir.path);
          expect(engine.flutterDir.path, p.join(srcDir.path, 'flutter'));
          expect(engine.outDir.path, p.join(srcDir.path, 'out'));
        } finally {
          tearDown();
        }
      });
    });
  });

  test('outputs an empty list of targets', () {
    setUp();

    try {
      // Create a valid engine.
      io.Directory(p.join(emptyDir.path, 'src', 'flutter')).createSync(recursive: true);
      io.Directory(p.join(emptyDir.path, 'src', 'out')).createSync(recursive: true);

      final Engine engine = Engine.fromSrcPath(p.join(emptyDir.path, 'src'));
      expect(engine.outputs(), <Output>[]);
      expect(engine.latestOutput(), isNull);
    } finally {
      tearDown();
    }
  });

  test('outputs a list of targets', () {
    setUp();

    try {
      // Create a valid engine.
      io.Directory(p.join(emptyDir.path, 'src', 'flutter')).createSync(recursive: true);
      io.Directory(p.join(emptyDir.path, 'src', 'out')).createSync(recursive: true);

      // Create two targets in out: host_debug and host_debug_unopt_arm64.
      io.Directory(p.join(emptyDir.path, 'src', 'out', 'host_debug')).createSync(recursive: true);
      io.Directory(
        p.join(emptyDir.path, 'src', 'out', 'host_debug_unopt_arm64'),
      ).createSync(recursive: true);

      final Engine engine = Engine.fromSrcPath(p.join(emptyDir.path, 'src'));
      final List<String> outputs =
          engine.outputs().map((Output o) => p.basename(o.path.path)).toList()..sort();
      expect(outputs, <String>['host_debug', 'host_debug_unopt_arm64']);
    } finally {
      tearDown();
    }
  });

  test('outputs the latest target and compile_commands.json', () {
    setUp();

    try {
      // Create a valid engine.
      final io.Directory srcDir = io.Directory(p.join(emptyDir.path, 'src'))
        ..createSync(recursive: true);
      final io.Directory flutterDir = io.Directory(p.join(srcDir.path, 'flutter'))
        ..createSync(recursive: true);
      final io.Directory outDir = io.Directory(p.join(srcDir.path, 'out'))
        ..createSync(recursive: true);

      // Create two targets in out: host_debug and host_debug_unopt_arm64.
      final io.Directory hostDebug = io.Directory(p.join(outDir.path, 'host_debug'))
        ..createSync(recursive: true);
      final io.Directory hostDebugUnoptArm64 = io.Directory(
        p.join(outDir.path, 'host_debug_unopt_arm64'),
      )..createSync(recursive: true);

      final Engine engine = TestEngine.withPaths(
        srcDir: srcDir,
        flutterDir: flutterDir,
        outDir: outDir,
        outputs: <TestOutput>[
          TestOutput(hostDebug, lastModified: DateTime.utc(2023, 9, 23, 21, 16)),
          TestOutput(hostDebugUnoptArm64, lastModified: DateTime.utc(2023, 9, 23, 22, 16)),
        ],
      );

      final Output? latestOutput = engine.latestOutput();
      expect(latestOutput, isNotNull);
      expect(p.basename(latestOutput!.path.path), 'host_debug_unopt_arm64');
      expect(latestOutput.compileCommandsJson, isNotNull);
    } finally {
      tearDown();
    }
  });
}
