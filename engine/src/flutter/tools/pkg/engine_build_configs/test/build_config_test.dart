// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert' as convert;

import 'package:engine_build_configs/src/build_config.dart';
import 'package:platform/platform.dart';
import 'package:test/test.dart';

import 'fixtures.dart' as fixtures;

int main() {
  test('BuildConfig parser works', () {
    final BuilderConfig buildConfig = BuilderConfig.fromJson(
      path: 'linux_test_config',
      map: convert.jsonDecode(fixtures.buildConfigJson) as Map<String, Object?>,
    );
    expect(buildConfig.valid, isTrue);
    expect(buildConfig.errors, isNull);
    expect(buildConfig.builds.length, equals(1));

    final Build globalBuild = buildConfig.builds[0];
    expect(globalBuild.name, equals('build_name'));
    expect(globalBuild.description, equals('build_description'));
    expect(globalBuild.gn.length, equals(3));
    expect(globalBuild.gn[0], equals('--gn-arg'));
    expect(globalBuild.droneDimensions.length, equals(1));
    expect(globalBuild.droneDimensions[0], equals('os=Linux'));
    expect(globalBuild.canRunOn(FakePlatform(operatingSystem: Platform.linux)), isTrue);
    expect(globalBuild.canRunOn(FakePlatform(operatingSystem: Platform.macOS)), isFalse);

    final BuildNinja ninja = globalBuild.ninja;
    expect(ninja.config, equals('build_name'));
    expect(ninja.targets.length, equals(1));
    expect(ninja.targets[0], equals('ninja_target'));

    expect(globalBuild.archives.length, equals(1));
    final BuildArchive buildArchive = globalBuild.archives[0];
    expect(buildArchive.name, equals('build_name'));
    expect(buildArchive.basePath, equals('base/path'));
    expect(buildArchive.type, equals('gcs'));
    expect(buildArchive.includePaths.length, equals(1));
    expect(buildArchive.includePaths[0], equals('include/path'));

    expect(globalBuild.tests.length, equals(1));
    final BuildTest tst = globalBuild.tests[0];
    expect(tst.name, equals('build_name tests'));
    expect(tst.language, equals('python3'));
    expect(tst.script, equals('test/script.py'));
    expect(tst.parameters.length, equals(1));
    expect(tst.parameters[0], equals('--test-params'));
    expect(tst.contexts.length, equals(1));
    expect(tst.contexts[0], equals('context'));

    expect(globalBuild.generators.length, equals(1));
    final BuildTask buildTask = globalBuild.generators[0];
    expect(buildTask.name, equals('generator_task'));
    expect(buildTask.scripts.length, equals(1));
    expect(buildTask.scripts[0], equals('gen/script.py'));
    expect(buildTask.parameters.length, equals(1));
    expect(buildTask.parameters[0], equals('--gen-param'));

    expect(buildConfig.generators.length, equals(1));
    final TestTask testTask = buildConfig.generators[0];
    expect(testTask.name, equals('global generator task'));
    expect(testTask.language, equals('dart'));
    expect(testTask.script, equals('global/gen_script.dart'));
    expect(testTask.parameters.length, equals(1));
    expect(testTask.parameters[0], equals('--global-gen-param'));

    expect(buildConfig.tests.length, equals(1));
    final GlobalTest globalTest = buildConfig.tests[0];
    expect(globalTest.name, equals('global test'));
    expect(globalTest.recipe, equals('engine_v2/tester_engine'));
    expect(globalTest.droneDimensions.length, equals(1));
    expect(globalTest.droneDimensions[0], equals('os=Linux'));
    expect(globalTest.canRunOn(FakePlatform(operatingSystem: Platform.linux)), isTrue);
    expect(globalTest.canRunOn(FakePlatform(operatingSystem: Platform.macOS)), isFalse);
    expect(globalTest.dependencies.length, equals(1));
    expect(globalTest.dependencies[0], equals('dependency'));

    expect(globalTest.tasks.length, equals(1));
    final TestTask globalTestTask = globalTest.tasks[0];
    expect(globalTestTask.name, equals('global test task'));
    expect(globalTestTask.script, equals('global/test/script.py'));
    expect(globalTestTask.language, equals('<undef>'));
  });

  test('BuildConfig flags invalid input', () {
    const String invalidInput = '''
{
  "builds": 5,
  "generators": {},
  "tests": []
}
''';
    final BuilderConfig buildConfig = BuilderConfig.fromJson(
      path: 'linux_test_config',
      map: convert.jsonDecode(invalidInput) as Map<String, Object?>,
    );
    expect(buildConfig.valid, isFalse);
    expect(
      buildConfig.errors![0],
      equals('For field "builds", expected type: list, actual type: int.'),
    );
  });

  test('GlobalBuild flags invalid input', () {
    const String invalidInput = '''
{
  "builds": [
    {
      "name": 5
    }
  ],
  "generators": {},
  "tests": []
}
''';
    final BuilderConfig buildConfig = BuilderConfig.fromJson(
      path: 'linux_test_config',
      map: convert.jsonDecode(invalidInput) as Map<String, Object?>,
    );
    expect(buildConfig.valid, isTrue);
    expect(buildConfig.builds.length, equals(1));
    expect(buildConfig.builds[0].valid, isFalse);
    expect(
      buildConfig.builds[0].errors![0],
      equals('For field "name", expected type: string, actual type: int.'),
    );
  });

  test('BuildNinja flags invalid input', () {
    const String invalidInput = '''
{
  "builds": [
    {
      "ninja": {
        "config": 5
      }
    }
  ],
  "generators": {},
  "tests": []
}
''';
    final BuilderConfig buildConfig = BuilderConfig.fromJson(
      path: 'linux_test_config',
      map: convert.jsonDecode(invalidInput) as Map<String, Object?>,
    );
    expect(buildConfig.valid, isTrue);
    expect(buildConfig.builds.length, equals(1));
    expect(buildConfig.builds[0].valid, isTrue);
    expect(buildConfig.builds[0].ninja.valid, isFalse);
    expect(
      buildConfig.builds[0].ninja.errors![0],
      equals('For field "config", expected type: string, actual type: int.'),
    );
  });

  test('BuildTest flags invalid input', () {
    const String invalidInput = '''
{
  "builds": [
    {
      "tests": [
        {
          "language": 5
        }
      ]
    }
  ],
  "generators": {},
  "tests": []
}
''';
    final BuilderConfig buildConfig = BuilderConfig.fromJson(
      path: 'linux_test_config',
      map: convert.jsonDecode(invalidInput) as Map<String, Object?>,
    );
    expect(buildConfig.valid, isTrue);
    expect(buildConfig.builds.length, equals(1));
    expect(buildConfig.builds[0].valid, isTrue);
    expect(buildConfig.builds[0].tests[0].valid, isFalse);
    expect(
      buildConfig.builds[0].tests[0].errors![0],
      equals('For field "language", expected type: string, actual type: int.'),
    );
  });

  test('BuildTask flags invalid input', () {
    const String invalidInput = '''
{
  "builds": [
    {
      "generators": {
        "tasks": [
          {
            "name": 5
          }
        ]
      }
    }
  ],
  "generators": {},
  "tests": []
}
''';
    final BuilderConfig buildConfig = BuilderConfig.fromJson(
      path: 'linux_test_config',
      map: convert.jsonDecode(invalidInput) as Map<String, Object?>,
    );
    expect(buildConfig.valid, isTrue);
    expect(buildConfig.builds.length, equals(1));
    expect(buildConfig.builds[0].valid, isTrue);
    expect(buildConfig.builds[0].generators[0].valid, isFalse);
    expect(
      buildConfig.builds[0].generators[0].errors![0],
      equals('For field "name", expected type: string, actual type: int.'),
    );
  });

  test('BuildArchive flags invalid input', () {
    const String invalidInput = '''
{
  "builds": [
    {
      "archives": [
        {
          "name": 5
        }
      ]
    }
  ],
  "generators": {},
  "tests": []
}
''';
    final BuilderConfig buildConfig = BuilderConfig.fromJson(
      path: 'linux_test_config',
      map: convert.jsonDecode(invalidInput) as Map<String, Object?>,
    );
    expect(buildConfig.valid, isTrue);
    expect(buildConfig.builds.length, equals(1));
    expect(buildConfig.builds[0].valid, isTrue);
    expect(buildConfig.builds[0].archives[0].valid, isFalse);
    expect(
      buildConfig.builds[0].archives[0].errors![0],
      equals('For field "name", expected type: string, actual type: int.'),
    );
  });

  test('GlobalTest flags invalid input', () {
    const String invalidInput = '''
{
  "tests": [
    {
      "name": 5
    }
  ]
}
''';
    final BuilderConfig buildConfig = BuilderConfig.fromJson(
      path: 'linux_test_config',
      map: convert.jsonDecode(invalidInput) as Map<String, Object?>,
    );
    expect(buildConfig.valid, isTrue);
    expect(buildConfig.tests.length, equals(1));
    expect(buildConfig.tests[0].valid, isFalse);
    expect(
      buildConfig.tests[0].errors![0],
      equals('For field "name", expected type: string, actual type: int.'),
    );
  });

  test('TestTask flags invalid input', () {
    const String invalidInput = '''
{
  "tests": [
    {
      "tasks": [
        {
          "name": 5
        }
      ]
    }
  ]
}
''';
    final BuilderConfig buildConfig = BuilderConfig.fromJson(
      path: 'linux_test_config',
      map: convert.jsonDecode(invalidInput) as Map<String, Object?>,
    );
    expect(buildConfig.valid, isTrue);
    expect(buildConfig.tests.length, equals(1));
    expect(buildConfig.tests[0].tasks[0].valid, isFalse);
    expect(
      buildConfig.tests[0].tasks[0].errors![0],
      contains('For field "name", expected type: string, actual type: int.'),
    );
  });
  return 0;
}
