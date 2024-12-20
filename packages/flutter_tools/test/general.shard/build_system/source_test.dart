// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/build_system.dart';
import 'package:flutter_tools/src/build_system/exceptions.dart';
import 'package:flutter_tools/src/globals.dart' as globals;

import '../../src/common.dart';
import '../../src/fake_process_manager.dart';
import '../../src/testbed.dart';

final Platform windowsPlatform = FakePlatform(operatingSystem: 'windows');

void main() {
  late Testbed testbed;
  late SourceVisitor visitor;
  late Environment environment;

  setUp(() {
    testbed = Testbed(
      setup: () {
        globals.fs.directory('cache').createSync();
        final Directory outputs = globals.fs.directory('outputs')..createSync();
        environment = Environment.test(
          globals.fs.currentDirectory,
          outputDir: outputs,
          artifacts: globals.artifacts!, // using real artifacts
          processManager: FakeProcessManager.any(),
          fileSystem: globals.fs,
          // engineVersion being null simulates a local engine.
          logger: globals.logger,
        );
        visitor = SourceVisitor(environment);
        environment.buildDir.createSync(recursive: true);
      },
    );
  });

  test(
    'configures implicit vs explicit correctly',
    () => testbed.run(() {
      expect(const Source.pattern('{PROJECT_DIR}/foo').implicit, false);
      expect(const Source.pattern('{PROJECT_DIR}/*foo').implicit, true);
    }),
  );

  test(
    'can substitute {PROJECT_DIR}/foo',
    () => testbed.run(() {
      globals.fs.file('foo').createSync();
      const Source fooSource = Source.pattern('{PROJECT_DIR}/foo');
      fooSource.accept(visitor);

      expect(visitor.sources.single.path, globals.fs.path.absolute('foo'));
    }),
  );

  test(
    'can substitute {OUTPUT_DIR}/foo',
    () => testbed.run(() {
      globals.fs.file('foo').createSync();
      const Source fooSource = Source.pattern('{OUTPUT_DIR}/foo');
      fooSource.accept(visitor);

      expect(
        visitor.sources.single.path,
        globals.fs.path.absolute(globals.fs.path.join('outputs', 'foo')),
      );
    }),
  );

  test(
    'can substitute {BUILD_DIR}/bar',
    () => testbed.run(() {
      final String path = globals.fs.path.join(environment.buildDir.path, 'bar');
      globals.fs.file(path).createSync();
      const Source barSource = Source.pattern('{BUILD_DIR}/bar');
      barSource.accept(visitor);

      expect(visitor.sources.single.path, globals.fs.path.absolute(path));
    }),
  );

  test(
    'can substitute {FLUTTER_ROOT}/foo',
    () => testbed.run(() {
      final String path = globals.fs.path.join(environment.flutterRootDir.path, 'foo');
      globals.fs.file(path).createSync();
      const Source barSource = Source.pattern('{FLUTTER_ROOT}/foo');
      barSource.accept(visitor);

      expect(visitor.sources.single.path, globals.fs.path.absolute(path));
    }),
  );

  test(
    'can substitute Artifact',
    () => testbed.run(() {
      final String path = globals.fs.path.join(
        globals.cache.getArtifactDirectory('engine').path,
        'windows-x64',
        'foo',
      );
      globals.fs.file(path).createSync(recursive: true);
      const Source fizzSource = Source.artifact(
        Artifact.windowsDesktopPath,
        platform: TargetPlatform.windows_x64,
      );
      fizzSource.accept(visitor);

      expect(visitor.sources.single.resolveSymbolicLinksSync(), globals.fs.path.absolute(path));
    }),
  );

  test(
    'can substitute {PROJECT_DIR}/*.fizz',
    () => testbed.run(() {
      const Source fizzSource = Source.pattern('{PROJECT_DIR}/*.fizz');
      fizzSource.accept(visitor);

      expect(visitor.sources, isEmpty);

      globals.fs.file('foo.fizz').createSync();
      globals.fs.file('foofizz').createSync();

      fizzSource.accept(visitor);

      expect(visitor.sources.single.path, globals.fs.path.absolute('foo.fizz'));
    }),
  );

  test(
    'can substitute {PROJECT_DIR}/fizz.*',
    () => testbed.run(() {
      const Source fizzSource = Source.pattern('{PROJECT_DIR}/fizz.*');
      fizzSource.accept(visitor);

      expect(visitor.sources, isEmpty);

      globals.fs.file('fizz.foo').createSync();
      globals.fs.file('fizz').createSync();

      fizzSource.accept(visitor);

      expect(visitor.sources.single.path, globals.fs.path.absolute('fizz.foo'));
    }),
  );

  test(
    'can substitute {PROJECT_DIR}/a*bc',
    () => testbed.run(() {
      const Source fizzSource = Source.pattern('{PROJECT_DIR}/bc*bc');
      fizzSource.accept(visitor);

      expect(visitor.sources, isEmpty);

      globals.fs.file('bcbc').createSync();
      globals.fs.file('bc').createSync();

      fizzSource.accept(visitor);

      expect(visitor.sources.single.path, globals.fs.path.absolute('bcbc'));
    }),
  );

  test(
    'crashes on bad substitute of two **',
    () => testbed.run(() {
      const Source fizzSource = Source.pattern('{PROJECT_DIR}/*.*bar');

      globals.fs.file('abcd.bar').createSync();

      expect(() => fizzSource.accept(visitor), throwsA(isA<InvalidPatternException>()));
    }),
  );

  test(
    "can't substitute foo",
    () => testbed.run(() {
      const Source invalidBase = Source.pattern('foo');

      expect(() => invalidBase.accept(visitor), throwsA(isA<InvalidPatternException>()));
    }),
  );

  test(
    'can substitute optional files',
    () => testbed.run(() {
      const Source missingSource = Source.pattern('{PROJECT_DIR}/foo', optional: true);

      expect(globals.fs.file('foo').existsSync(), false);
      missingSource.accept(visitor);
      expect(visitor.sources, isEmpty);
    }),
  );

  test(
    'can resolve a missing depfile',
    () => testbed.run(() {
      visitor.visitDepfile('foo.d');

      expect(visitor.sources, isEmpty);
      expect(visitor.containsNewDepfile, true);
    }),
  );

  test(
    'can resolve a populated depfile',
    () => testbed.run(() {
      environment.buildDir.childFile('foo.d').writeAsStringSync('a.dart : c.dart');

      visitor.visitDepfile('foo.d');
      expect(visitor.sources.single.path, 'c.dart');
      expect(visitor.containsNewDepfile, false);

      final SourceVisitor outputVisitor = SourceVisitor(environment, false);
      outputVisitor.visitDepfile('foo.d');

      expect(outputVisitor.sources.single.path, 'a.dart');
      expect(outputVisitor.containsNewDepfile, false);
    }),
  );

  test(
    'does not crash on completely invalid depfile',
    () => testbed.run(() {
      environment.buildDir.childFile('foo.d').writeAsStringSync('hello, world');

      visitor.visitDepfile('foo.d');
      expect(visitor.sources, isEmpty);
      expect(visitor.containsNewDepfile, false);
    }),
  );

  test(
    'can parse depfile with windows paths',
    () => testbed.run(() {
      environment.buildDir.childFile('foo.d').writeAsStringSync(r'a.dart: C:\\foo\\bar.txt');

      visitor.visitDepfile('foo.d');
      expect(visitor.sources.single.path, r'C:\foo\bar.txt');
      expect(visitor.containsNewDepfile, false);
    }, overrides: <Type, Generator>{Platform: () => windowsPlatform}),
  );

  test(
    'can parse depfile with spaces in paths',
    () => testbed.run(() {
      environment.buildDir.childFile('foo.d').writeAsStringSync(r'a.dart: foo\ bar.txt');

      visitor.visitDepfile('foo.d');
      expect(visitor.sources.single.path, r'foo bar.txt');
      expect(visitor.containsNewDepfile, false);
    }),
  );

  test(
    'Non-local engine builds use the engine.version file as an Artifact dependency',
    () => testbed.run(() {
      final Artifacts artifacts = Artifacts.test();
      final Environment environment = Environment.test(
        globals.fs.currentDirectory,
        artifacts: artifacts,
        processManager: FakeProcessManager.any(),
        fileSystem: globals.fs,
        logger: globals.logger,
        engineVersion: 'abcdefghijklmon', // Use a versioned engine.
      );
      visitor = SourceVisitor(environment);

      const Source fizzSource = Source.artifact(
        Artifact.windowsDesktopPath,
        platform: TargetPlatform.windows_x64,
      );
      fizzSource.accept(visitor);

      expect(visitor.sources.single.path, contains('engine.version'));
    }),
  );
}
