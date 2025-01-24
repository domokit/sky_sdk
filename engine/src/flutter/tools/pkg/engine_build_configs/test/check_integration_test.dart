// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@TestOn('vm')
library;

import 'dart:convert';
import 'dart:io' as io;

import 'package:path/path.dart' as p;
import 'package:platform/platform.dart';
import 'package:test/test.dart';

import '../bin/check.dart' as check show run;

void main() {
  late io.Directory tmpFlutterEngineRoot;
  late StringBuffer stdout;
  late StringBuffer stderr;
  late int exitCode;

  late io.Directory tmpFlutterEngineSrc;
  late io.Directory tmpCiBuilders;
  late io.File tmpCiYaml;
  late List<Map<String, Object?>> ciYamlTargets;

  setUp(() {
    tmpFlutterEngineRoot = io.Directory.systemTemp.createTempSync('check_integration_test.');
    stdout = StringBuffer();
    stderr = StringBuffer();
    exitCode = 0;

    // Create a synthetic engine directory.
    tmpFlutterEngineSrc = io.Directory(p.join(tmpFlutterEngineRoot.path, 'flutter', 'src'))
      ..createSync(recursive: true);
    tmpCiBuilders = io.Directory(p.join(tmpFlutterEngineSrc.path, 'flutter', 'ci', 'builders'))
      ..createSync(recursive: true);
    tmpCiYaml = io.File(p.join(p.join(tmpFlutterEngineSrc.path, 'flutter', '.ci.yaml')));
    ciYamlTargets = [];
  });

  tearDown(() {
    tmpFlutterEngineRoot.deleteSync(recursive: true);
  });

  void run(Iterable<String> args, {Platform? platform, bool allowFailure = false}) {
    check.run(
      args,
      stderr: stderr,
      stdout: stdout,
      setExitCode: (newExitCode) {
        exitCode = newExitCode;
      },
      platform: platform ?? FakePlatform(operatingSystem: Platform.linux),
    );
    if (exitCode != 0 && !allowFailure) {
      fail('$args failed: $stderr');
    }
  }

  test('should produce usage on --help', () {
    run(['--help']);

    expect(
      stdout.toString(),
      allOf([contains('--verbose'), contains('--help'), contains('--engine-src-path')]),
    );
  });

  test('fails if no build configurations were found', () {
    run(['--engine-src-path=${tmpFlutterEngineSrc.path}'], allowFailure: true);

    expect(stderr.toString(), contains('❌ Loaded build configs under'));
  });

  void addConfig(
    String name,
    List<Map<String, Object?>> builds, {
    bool releaseBuild = false,
    bool specifyConfig = true,
    bool writeGenerators = false,
  }) {
    if (specifyConfig) {
      io.File(p.join(tmpCiBuilders.path, '$name.json')).writeAsStringSync(
        jsonEncode({
          'builds': builds,
          if (writeGenerators)
            'generators': {
              'tasks': <Object?>[{}],
            },
        }),
      );
    }

    ciYamlTargets.add({
      'name': name,
      'recipe': 'flutter/some_recipe',
      'properties': {
        if (specifyConfig) 'config_name': name,
        if (releaseBuild) 'release_build': 'true',
      },
    });
    tmpCiYaml.writeAsStringSync(jsonEncode({'targets': ciYamlTargets}));
  }

  test('fails if a configuration file had a deserialization error', () {
    addConfig('linux_unopt', []);

    // Malform the .ci.yaml file.
    tmpCiYaml.writeAsStringSync('bad{}');
    run(['--engine-src-path=${tmpFlutterEngineSrc.path}'], allowFailure: true);

    expect(stderr.toString(), contains('❌ .ci.yaml at'));
  });

  test('fails if an individual builder has a schema error', () {
    addConfig('linux_unopt', [
      {'ninja': 1234},
    ]);

    run(['--engine-src-path=${tmpFlutterEngineSrc.path}'], allowFailure: true);

    expect(stderr.toString(), contains('❌ All configuration files are valid'));
  });

  test('fails if all builds within a builder are not uniquely named', () {
    addConfig('linux_unopt', [
      {'name': 'foo'},
      {'name': 'foo'},
    ]);

    run(['--engine-src-path=${tmpFlutterEngineSrc.path}'], allowFailure: true);

    expect(stderr.toString(), contains('❌ All builds within a builder are uniquely named'));
  });

  test('fails if a build does not use a conforming OS prefix or "ci"', () {
    addConfig('linux_unopt', [
      {'name': 'not_an_os_or_ci/foo'},
    ]);

    run(['--engine-src-path=${tmpFlutterEngineSrc.path}'], allowFailure: true);

    expect(stderr.toString(), contains('❌ All build names must have a conforming prefix'));
  });

  test('skips targets without a config_name', () {
    addConfig('linux_unopt', []);
    addConfig('linux_cache', [], specifyConfig: false);

    run(['--engine-src-path=${tmpFlutterEngineSrc.path}', '--verbose']);

    expect(stderr.toString(), contains('Skipping linux_cache'));
  });

  test('skips checking if a global "generators" field is present', () {
    addConfig(
      'linux_befuzzled',
      [
        {'name': 'ci/test'},
      ],
      releaseBuild: true,
      writeGenerators: true,
    );

    run(['--engine-src-path=${tmpFlutterEngineSrc.path}', '--verbose']);

    expect(stderr.toString(), contains('Skipping linux_befuzzled: Has "generators"'));
  });

  test('fails if a release builder omits archives', () {
    addConfig('linux_engine', [
      {
        'name': 'ci/host_debug',
        'archives': <Object?>[{}],
      },
    ], releaseBuild: true);

    run(['--engine-src-path=${tmpFlutterEngineSrc.path}'], allowFailure: true);

    expect(stderr.toString(), contains('❌ All builder files conform to release_build standards'));
  });

  test('fails if a release builder includes tests', () {
    addConfig('linux_engine', [
      {
        'name': 'ci/host_debug',
        'archives': <Object?>[
          {
            'include_paths': ['out/foo'],
          },
        ],
        'tests': <Object?>[{}],
      },
    ], releaseBuild: true);

    run(['--engine-src-path=${tmpFlutterEngineSrc.path}'], allowFailure: true);

    expect(stderr.toString(), contains('❌ All builder files conform to release_build standards'));
  });

  test('fails if archives.include_paths is empty', () {
    addConfig('linux_engine', [
      {
        'name': 'ci/host_debug',
        'archives': <Object?>[
          {'include_paths': <Object?>[]},
        ],
      },
    ], releaseBuild: true);

    run(['--engine-src-path=${tmpFlutterEngineSrc.path}'], allowFailure: true);

    expect(stderr.toString(), contains('❌ All builder files conform to release_build standards'));
  });
}
