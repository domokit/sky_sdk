// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:engine_tool/src/gn.dart';
import 'package:engine_tool/src/label.dart';
import 'package:engine_tool/src/logger.dart';
import 'package:test/test.dart';

import '../src/utils.dart';

void main() {
  test('gn.desc handles a non-zero exit code', () async {
    final testEnv = TestEnvironment.withTestEngine(
      cannedProcesses: [
        CannedProcess(
          (List<String> command) => command.contains('desc'),
          exitCode: 1,
          stdout: 'stdout',
          stderr: 'stderr',
        ),
      ],
    );
    addTearDown(testEnv.cleanup);

    final gn = Gn.fromEnvironment(testEnv.environment);
    expect(
      () => gn.desc('out/Release', TargetPattern('//foo', 'bar')),
      throwsA(
        isA<FatalError>().having(
          (a) => a.toString(),
          'toString()',
          allOf([
            contains('Failed to run'),
            contains('exit code 1'),
            contains('STDOUT:\nstdout'),
            contains('STDERR:\nstderr'),
          ]),
        ),
      ),
    );
  });

  test('gn.desc handles unparseable stdout', () async {
    final testEnv = TestEnvironment.withTestEngine(
      cannedProcesses: [
        CannedProcess((List<String> command) => command.contains('desc'), stdout: 'not json'),
      ],
    );
    addTearDown(testEnv.cleanup);

    final gn = Gn.fromEnvironment(testEnv.environment);
    expect(
      () => gn.desc('out/Release', TargetPattern('//foo', 'bar')),
      throwsA(
        isA<FatalError>().having(
          (a) => a.toString(),
          'toString()',
          allOf([contains('Failed to parse JSON'), contains('not json')]),
        ),
      ),
    );
  });

  test('gn.desc parses build targets', () async {
    final testEnv = TestEnvironment.withTestEngine(
      cannedProcesses: [
        CannedProcess(
          (List<String> command) => command.contains('desc'),
          stdout: '''
            {
              "//foo/bar:baz_test": {
                "outputs": ["//out/host_debug/foo/bar/baz_test"],
                "testonly": true,
                "type": "executable"
              },
              "//foo/bar:baz_shared_library": {
                "testonly": false,
                "type": "shared_library"
              },
              "//foo/bar:baz_static_library": {
                "testonly": false,
                "type": "static_library"
              }
            }
          ''',
        ),
      ],
    );
    addTearDown(testEnv.cleanup);

    final gn = Gn.fromEnvironment(testEnv.environment);
    final targets = await gn.desc('out/Release', TargetPattern('//foo', 'bar'));
    expect(targets, hasLength(3));

    // There should be exactly one binary test target and two library targets.
    final testTarget = targets.whereType<ExecutableBuildTarget>().single;
    expect(
      testTarget,
      ExecutableBuildTarget(
        label: Label('//foo/bar', 'baz_test'),
        testOnly: true,
        executable: 'out/host_debug/foo/bar/baz_test',
      ),
    );

    final libraryTargets = targets.whereType<LibraryBuildTarget>().toList();
    expect(libraryTargets, hasLength(2));
    expect(
      libraryTargets.contains(
        LibraryBuildTarget(label: Label('//foo/bar', 'baz_shared_library'), testOnly: false),
      ),
      isTrue,
    );
    expect(
      libraryTargets.contains(
        LibraryBuildTarget(label: Label('//foo/bar', 'baz_static_library'), testOnly: false),
      ),
      isTrue,
    );
  });

  test('parses a group', () async {
    final testEnv = TestEnvironment.withTestEngine(
      cannedProcesses: [
        CannedProcess(
          (List<String> command) => command.contains('desc'),
          stdout: '''
            {
              "//foo/bar:baz_group": {
                "deps": ["//foo/bar:baz_shared_library"],
                "testonly": true,
                "type": "group"
              }
            }
          ''',
        ),
      ],
    );
    addTearDown(testEnv.cleanup);

    final gn = Gn.fromEnvironment(testEnv.environment);
    final targets = await gn.desc('out/Release', TargetPattern('//foo', 'bar'));
    expect(targets, hasLength(1));

    final groupTarget = targets.single;
    expect(
      groupTarget,
      GroupBuildTarget(
        label: Label('//foo/bar', 'baz_group'),
        testOnly: true,
        deps: [Label('//foo/bar', 'baz_shared_library')],
      ),
    );
  });

  test('parses a dart_test action as an executable', () async {
    final testEnv = TestEnvironment.withTestEngine(
      cannedProcesses: [
        CannedProcess(
          (List<String> command) => command.contains('desc'),
          stdout: '''
            {
              "//foo/bar:baz_test": {
                "outputs": ["//out/host_debug/foo/bar/baz_test"],
                "testonly": true,
                "type": "action",
                "metadata": {
                  "action_type": ["dart_test"]
                }
              }
            }
          ''',
        ),
      ],
    );
    addTearDown(testEnv.cleanup);

    final gn = Gn.fromEnvironment(testEnv.environment);
    final targets = await gn.desc('out/Release', TargetPattern('//foo', 'bar'));
    expect(targets, hasLength(1));

    final testTarget = targets.single;
    expect(
      testTarget,
      ExecutableBuildTarget(
        label: Label('//foo/bar', 'baz_test'),
        testOnly: true,
        executable: 'out/host_debug/foo/bar/baz_test',
      ),
    );
  });
}
