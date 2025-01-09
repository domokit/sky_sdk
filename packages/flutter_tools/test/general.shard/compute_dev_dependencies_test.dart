// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/compute_dev_dependencies.dart';
import 'package:flutter_tools/src/dart/pub.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/reporting/reporting.dart';

import '../src/common.dart';
import '../src/fake_process_manager.dart';
import '../src/fakes.dart';

const String _dartBin = 'bin/cache/dart-sdk/bin/dart';

// For all of these examples, imagine the following package structure:
//
// /
//   /my_app
//     pubspec.yaml
//   /package_a
//     pubspec.yaml
//   /package_b
//     pubspec.yaml
//   /package_c
//     pubspec.yaml
void main() {
  late FileSystem fileSystem;
  late FlutterProject project;
  late BufferLogger logger;

  setUp(() {
    Cache.flutterRoot = '';
    fileSystem = MemoryFileSystem.test();
    project = FlutterProject.fromDirectoryTest(fileSystem.currentDirectory);
    logger = BufferLogger.test();
  });

  Pub pub(ProcessManager processManager) {
    return Pub.test(
      fileSystem: fileSystem,
      logger: logger,
      processManager: processManager,
      usage: TestUsage(),
      platform: FakePlatform(),
      botDetector: const FakeBotDetector(false),
      stdio: FakeStdio(),
    );
  }

  test('no dev dependencies at all', () async {
    // Simulates the following:
    //
    // # /my_app/pubspec.yaml
    // name: my_app
    // dependencies:
    //   package_a:
    //
    // # /package_a/pubspec.yaml
    // name: package_a
    // dependencies:
    //   package_b:
    final ProcessManager processes = _dartPubDepsReturns('''
    {
      "root": "my_app",
      "packages": [
        {
          "name": "my_app",
          "kind": "root",
          "dependencies": [
            "package_a",
            "package_b"
          ],
          "directDependencies": [
            "package_a"
          ],
          "devDependencies": []
        },
        {
          "name": "package_a",
          "kind": "direct",
          "dependencies": [
            "package_b"
          ],
          "directDependencies": [
            "package_b"
          ]
        },
        {
          "name": "package_b",
          "kind": "transitive",
          "dependencies": [],
          "directDependencies": []
        }
      ]
    }''', project: project);

    final Set<String> dependencies = await computeExclusiveDevDependencies(
      pub(processes),
      project: project,
      logger: logger,
    );

    expect(dependencies, isEmpty, reason: 'There are no dev_dependencies of "my_app".');
  });

  test('dev dependency', () async {
    // Simulates the following:
    //
    // # /my_app/pubspec.yaml
    // name: my_app
    // dependencies:
    //   package_a:
    //
    // dev_dependencies:
    //   package_b:
    //
    // # /package_a/pubspec.yaml
    // name: package_a
    final ProcessManager processes = _dartPubDepsReturns('''
    {
      "root": "my_app",
      "packages": [
        {
          "name": "my_app",
          "kind": "root",
          "dependencies": [
            "package_a",
            "package_b"
          ],
          "directDependencies": [
            "package_a"
          ],
          "devDependencies": [
            "package_b"
          ]
        },
        {
          "name": "package_a",
          "kind": "direct",
          "dependencies": [],
          "directDependencies": []
        },
        {
          "name": "package_b",
          "kind": "dev",
          "dependencies": [],
          "directDependencies": []
        }
      ]
    }''', project: project);

    final Set<String> dependencies = await computeExclusiveDevDependencies(
      pub(processes),
      project: project,
      logger: logger,
    );

    expect(dependencies, <String>{
      'package_b',
    }, reason: 'There is a single dev_dependency of my_app: package_b.');
  });

  test('dev used as a non-dev dependency transitively', () async {
    // Simulates the following:
    //
    // # /my_app/pubspec.yaml
    // name: my_app
    // dependencies:
    //   package_a:
    //
    // dev_dependencies:
    //   package_b:
    //
    // # /package_a/pubspec.yaml
    // name: package_a
    // dependencies:
    //   package_b:
    final ProcessManager processes = _dartPubDepsReturns('''
    {
      "root": "my_app",
      "packages": [
        {
          "name": "my_app",
          "kind": "root",
          "dependencies": [
            "package_a",
            "package_b"
          ],
          "directDependencies": [
            "package_a"
          ],
          "devDependencies": [
            "package_b"
          ]
        },
        {
          "name": "package_a",
          "kind": "direct",
          "dependencies": [
            "package_b"
          ],
          "directDependencies": [
            "package_b"
          ]
        },
        {
          "name": "package_b",
          "kind": "dev",
          "dependencies": [],
          "directDependencies": []
        }
      ]
    }''', project: project);

    final Set<String> dependencies = await computeExclusiveDevDependencies(
      pub(processes),
      project: project,
      logger: logger,
    );

    expect(
      dependencies,
      isEmpty,
      reason: 'There is a dev_dependency also used as a standard dependency',
    );
  });

  test('combination of an included and excluded dev_dependency', () async {
    // Simulates the following:
    //
    // # /my_app/pubspec.yaml
    // name: my_app
    // dependencies:
    //   package_a:
    //
    // dev_dependencies:
    //   package_b:
    //   package_c:
    //
    // # /package_a/pubspec.yaml
    // name: package_a
    // dependencies:
    //   package_b:
    //
    // # /package_b/pubspec.yaml
    // name: package_b
    //
    // # /package_c/pubspec.yaml
    // name: package_c
    final ProcessManager processes = _dartPubDepsReturns('''
    {
      "root": "my_app",
      "packages": [
        {
          "name": "my_app",
          "kind": "root",
          "dependencies": [
            "package_a",
            "package_b"
          ],
          "directDependencies": [
            "package_a"
          ],
          "devDependencies": [
            "package_b",
            "package_c"
          ]
        },
        {
          "name": "package_a",
          "kind": "direct",
          "dependencies": [
            "package_b"
          ],
          "directDependencies": [
            "package_b"
          ]
        },
        {
          "name": "package_b",
          "kind": "dev",
          "dependencies": [
            "package_c"
          ],
          "directDependencies": [
            "package_c"
          ]
        },
        {
          "name": "package_c",
          "kind": "dev",
          "dependencies": [],
          "directDependencies": []
        }
      ]
    }''', project: project);

    final Set<String> dependencies = await computeExclusiveDevDependencies(
      pub(processes),
      project: project,
      logger: logger,
    );

    expect(dependencies, <String>{
      'package_c',
    }, reason: 'package_b is excluded but package_c should not');
  });

  test('omitted devDependencies in app package', () async {
    // Simulates the following:
    //
    // # /my_app/pubspec.yaml
    // name: my_app
    // dependencies:
    //   package_a:
    //
    // # /package_a/pubspec.yaml
    // name: package_a
    final ProcessManager processes = _dartPubDepsReturns('''
    {
      "root": "my_app",
      "packages": [
        {
          "name": "my_app",
          "kind": "root",
          "dependencies": [
            "package_a"
          ],
          "directDependencies": [
            "package_a"
          ]
        },
        {
          "name": "package_a",
          "kind": "direct",
          "dependencies": [],
          "directDependencies": []
        }
      ]
    }''', project: project);

    final Set<String> dependencies = await computeExclusiveDevDependencies(
      pub(processes),
      project: project,
      logger: logger,
    );

    expect(
      dependencies,
      isEmpty,
      reason: 'No devDependencies: [] specified but still parsed successfully',
    );
  });

  test('throws and logs on invalid JSON', () async {
    final ProcessManager processes = _dartPubDepsReturns('''
    {
      "root": "my_app",
      "packages": [
        {
          "name": "my_app",
          "kind": "root",
          "dependencies": [
            "package_a",
            "package_b"
          ],
          "directDependencies": [
            1
          ],
          "devDependencies": []
        },
        {
          "name": "package_a",
          "kind": "direct",
          "dependencies": [
            "package_b"
          ],
          "directDependencies": [
            "package_b"
          ]
        },
        {
          "name": "package_b",
          "kind": "transitive",
          "dependencies": [],
          "directDependencies": []
        }
      ]
    }''', project: project);

    await expectLater(
      computeExclusiveDevDependencies(pub(processes), project: project, logger: logger),
      throwsA(
        isA<StateError>().having(
          (StateError e) => e.message,
          'message',
          contains('dart pub deps --json had unexpected output'),
        ),
      ),
    );

    expect(
      logger.traceText,
      contains('"root": "my_app"'),
      reason: 'Stdout should include the JSON blob',
    );
  });
}

ProcessManager _dartPubDepsReturns(String dartPubDepsOutput, {required FlutterProject project}) {
  return FakeProcessManager.list(<FakeCommand>[
    FakeCommand(
      command: const <String>[_dartBin, 'pub', '--suppress-analytics', 'deps', '--json'],
      stdout: dartPubDepsOutput,
      workingDirectory: project.directory.path,
    ),
  ]);
}
