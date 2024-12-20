// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/args.dart';
import 'package:collection/collection.dart' show IterableExtension;
import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/user_messages.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/build_system.dart';
import 'package:flutter_tools/src/build_system/depfile.dart';
import 'package:flutter_tools/src/build_system/targets/assets.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/convert.dart';
import 'package:flutter_tools/src/devfs.dart';
import 'package:flutter_tools/src/globals.dart' as globals;

import '../../../src/common.dart';
import '../../../src/context.dart';
import '../../../src/fake_process_manager.dart';

void main() {
  late Environment environment;
  late FileSystem fileSystem;
  late BufferLogger logger;

  setUp(() {
    fileSystem = MemoryFileSystem.test();
    environment = Environment.test(
      fileSystem.currentDirectory,
      processManager: FakeProcessManager.any(),
      artifacts: Artifacts.test(),
      fileSystem: fileSystem,
      logger: BufferLogger.test(),
      platform: FakePlatform(),
      defines: <String, String>{kBuildMode: BuildMode.debug.cliName},
    );
    fileSystem.file(environment.buildDir.childFile('app.dill')).createSync(recursive: true);
    fileSystem
        .file(environment.buildDir.childFile('native_assets.json'))
        .createSync(recursive: true);
    fileSystem
        .file('packages/flutter_tools/lib/src/build_system/targets/assets.dart')
        .createSync(recursive: true);
    fileSystem.file('assets/foo/bar.png').createSync(recursive: true);
    fileSystem.file('assets/wildcard/#bar.png').createSync(recursive: true);
    fileSystem.directory('.dart_tool').childFile('package_config.json').createSync(recursive: true);
    fileSystem.file('pubspec.yaml')
      ..createSync()
      ..writeAsStringSync('''
name: example

flutter:
  assets:
    - assets/foo/bar.png
    - assets/wildcard/
''');
    logger = BufferLogger.test();
  });

  testUsingContext(
    'includes LICENSE file inputs in dependencies',
    () async {
      fileSystem.directory('.dart_tool').childFile('package_config.json')
        ..createSync(recursive: true)
        ..writeAsStringSync('''
{
  "configVersion": 2,
  "packages": [
    {
      "name": "foo",
      "rootUri": "file:///bar",
      "packageUri": "lib/"
    }
  ]
}
''');
      fileSystem.file('bar/LICENSE')
        ..createSync(recursive: true)
        ..writeAsStringSync('THIS IS A LICENSE');

      await const CopyAssets().build(environment);

      final File depfile = environment.buildDir.childFile('flutter_assets.d');

      expect(depfile, exists);

      final Depfile dependencies = environment.depFileService.parse(depfile);

      expect(
        dependencies.inputs.firstWhereOrNull((File file) => file.path == '/bar/LICENSE'),
        isNotNull,
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'Copies files to correct asset directory',
    () async {
      await const CopyAssets().build(environment);

      expect(
        fileSystem.file('${environment.buildDir.path}/flutter_assets/AssetManifest.json'),
        exists,
      );
      expect(
        fileSystem.file('${environment.buildDir.path}/flutter_assets/FontManifest.json'),
        exists,
      );
      expect(fileSystem.file('${environment.buildDir.path}/flutter_assets/NOTICES.Z'), exists);
      // See https://github.com/flutter/flutter/issues/35293
      expect(
        fileSystem.file('${environment.buildDir.path}/flutter_assets/assets/foo/bar.png'),
        exists,
      );
      // See https://github.com/flutter/flutter/issues/46163
      expect(
        fileSystem.file('${environment.buildDir.path}/flutter_assets/assets/wildcard/%23bar.png'),
        exists,
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  group(
    "Only copies assets with a flavor if the assets' flavor matches the flavor in the environment",
    () {
      testUsingContext(
        'When the environment does not have a flavor defined',
        () async {
          fileSystem.file('pubspec.yaml')
            ..createSync()
            ..writeAsStringSync('''
  name: example
  flutter:
    assets:
      - assets/common/
      - path: assets/vanilla/
        flavors:
          - vanilla
      - path: assets/strawberry/
        flavors:
          - strawberry
  ''');

          fileSystem.file('assets/common/image.png').createSync(recursive: true);
          fileSystem.file('assets/vanilla/ice-cream.png').createSync(recursive: true);
          fileSystem.file('assets/strawberry/ice-cream.png').createSync(recursive: true);

          await const CopyAssets().build(environment);

          expect(
            fileSystem.file('${environment.buildDir.path}/flutter_assets/assets/common/image.png'),
            exists,
          );
          expect(
            fileSystem.file(
              '${environment.buildDir.path}/flutter_assets/assets/vanilla/ice-cream.png',
            ),
            isNot(exists),
          );
          expect(
            fileSystem.file(
              '${environment.buildDir.path}/flutter_assets/assets/strawberry/ice-cream.png',
            ),
            isNot(exists),
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fileSystem,
          ProcessManager: () => FakeProcessManager.any(),
        },
      );

      testUsingContext(
        'When the environment has a flavor defined',
        () async {
          environment.defines[kFlavor] = 'strawberry';
          fileSystem.file('pubspec.yaml')
            ..createSync()
            ..writeAsStringSync('''
  name: example
  flutter:
    assets:
      - assets/common/
      - path: assets/vanilla/
        flavors:
          - vanilla
      - path: assets/strawberry/
        flavors:
          - strawberry
  ''');

          fileSystem.file('assets/common/image.png').createSync(recursive: true);
          fileSystem.file('assets/vanilla/ice-cream.png').createSync(recursive: true);
          fileSystem.file('assets/strawberry/ice-cream.png').createSync(recursive: true);

          await const CopyAssets().build(environment);

          expect(
            fileSystem.file('${environment.buildDir.path}/flutter_assets/assets/common/image.png'),
            exists,
          );
          expect(
            fileSystem.file(
              '${environment.buildDir.path}/flutter_assets/assets/vanilla/ice-cream.png',
            ),
            isNot(exists),
          );
          expect(
            fileSystem.file(
              '${environment.buildDir.path}/flutter_assets/assets/strawberry/ice-cream.png',
            ),
            exists,
          );
        },
        overrides: <Type, Generator>{
          FileSystem: () => fileSystem,
          ProcessManager: () => FakeProcessManager.any(),
        },
      );
    },
  );

  testUsingContext(
    'transforms assets declared with transformers',
    () async {
      Cache.flutterRoot = Cache.defaultFlutterRoot(
        platform: globals.platform,
        fileSystem: fileSystem,
        userMessages: UserMessages(),
      );

      final Environment environment = Environment.test(
        fileSystem.currentDirectory,
        processManager: globals.processManager,
        artifacts: Artifacts.test(),
        fileSystem: fileSystem,
        logger: logger,
        platform: globals.platform,
        defines: <String, String>{kBuildMode: BuildMode.debug.cliName},
      );

      fileSystem
          .directory('.dart_tool')
          .childFile('package_config.json')
          .createSync(recursive: true);

      fileSystem.file('pubspec.yaml')
        ..createSync()
        ..writeAsStringSync('''
name: example
flutter:
  assets:
    - path: input.txt
      transformers:
        - package: my_capitalizer_transformer
          args: ["-a", "-b", "--color", "green"]
''');

      fileSystem.file('input.txt')
        ..createSync(recursive: true)
        ..writeAsStringSync('abc');

      await const CopyAssets().build(environment);

      expect(logger.errorText, isEmpty);
      expect(globals.processManager, hasNoRemainingExpectations);
      expect(fileSystem.file('${environment.buildDir.path}/flutter_assets/input.txt'), exists);
    },
    overrides: <Type, Generator>{
      Logger: () => logger,
      FileSystem: () => fileSystem,
      Platform: () => FakePlatform(),
      ProcessManager:
          () => FakeProcessManager.list(<FakeCommand>[
            FakeCommand(
              command: <Pattern>[
                Artifacts.test().getArtifactPath(Artifact.engineDartBinary),
                'run',
                'my_capitalizer_transformer',
                RegExp('--input=.*'),
                RegExp('--output=.*'),
                '-a',
                '-b',
                '--color',
                'green',
              ],
              onRun: (List<String> args) {
                final ArgResults parsedArgs = (ArgParser()
                      ..addOption('input')
                      ..addOption('output')
                      ..addOption('color')
                      ..addFlag('aaa', abbr: 'a')
                      ..addFlag('bbb', abbr: 'b'))
                    .parse(args);

                expect(parsedArgs['aaa'], true);
                expect(parsedArgs['bbb'], true);
                expect(parsedArgs['color'], 'green');

                final File input = fileSystem.file(parsedArgs['input'] as String);
                expect(input, exists);
                final String inputContents = input.readAsStringSync();
                expect(inputContents, 'abc');
                fileSystem.file(parsedArgs['output'])
                  ..createSync()
                  ..writeAsStringSync(inputContents.toUpperCase());
              },
            ),
          ]),
    },
  );

  testUsingContext(
    'exits tool if an asset transformation fails',
    () async {
      Cache.flutterRoot = Cache.defaultFlutterRoot(
        platform: globals.platform,
        fileSystem: fileSystem,
        userMessages: UserMessages(),
      );

      final Environment environment = Environment.test(
        fileSystem.currentDirectory,
        processManager: globals.processManager,
        artifacts: Artifacts.test(),
        fileSystem: fileSystem,
        logger: logger,
        platform: globals.platform,
        defines: <String, String>{kBuildMode: BuildMode.debug.cliName},
      );

      fileSystem
          .directory('.dart_tool')
          .childFile('package_config.json')
          .createSync(recursive: true);

      fileSystem.file('pubspec.yaml')
        ..createSync()
        ..writeAsStringSync('''
name: example
flutter:
  assets:
    - path: input.txt
      transformers:
        - package: my_transformer
          args: ["-a", "-b", "--color", "green"]
''');

      await fileSystem.file('input.txt').create(recursive: true);

      await expectToolExitLater(
        const CopyAssets().build(environment),
        startsWith('User-defined transformation of asset "input.txt" failed.\n'),
      );
      expect(globals.processManager, hasNoRemainingExpectations);
    },
    overrides: <Type, Generator>{
      Logger: () => logger,
      FileSystem: () => fileSystem,
      Platform: () => FakePlatform(),
      ProcessManager:
          () => FakeProcessManager.list(<FakeCommand>[
            FakeCommand(
              command: <Pattern>[
                Artifacts.test().getArtifactPath(Artifact.engineDartBinary),
                'run',
                'my_transformer',
                RegExp('--input=.*'),
                RegExp('--output=.*'),
                '-a',
                '-b',
                '--color',
                'green',
              ],
              exitCode: 1,
            ),
          ]),
    },
  );

  testUsingContext(
    'asset transformation, per each asset, uses unique paths for temporary files',
    () async {
      final List<String> inputFilePaths = <String>[];
      final List<String> outputFilePaths = <String>[];

      final FakeCommand transformerCommand = FakeCommand(
        command: <Pattern>[
          Artifacts.test().getArtifactPath(Artifact.engineDartBinary),
          'run',
          'my_capitalizer_transformer',
          RegExp('--input=.*'),
          RegExp('--output=.*'),
        ],
        onRun: (List<String> args) {
          final ArgResults parsedArgs = (ArgParser()
                ..addOption('input')
                ..addOption('output'))
              .parse(args);

          final String input = parsedArgs['input'] as String;
          final String output = parsedArgs['output'] as String;

          inputFilePaths.add(input);
          outputFilePaths.add(output);

          fileSystem.file(output)
            ..createSync()
            ..writeAsStringSync('foo');
        },
      );

      Cache.flutterRoot = Cache.defaultFlutterRoot(
        platform: globals.platform,
        fileSystem: fileSystem,
        userMessages: UserMessages(),
      );

      final Environment environment = Environment.test(
        fileSystem.currentDirectory,
        processManager: FakeProcessManager.list(<FakeCommand>[
          transformerCommand,
          transformerCommand,
        ]),
        artifacts: Artifacts.test(),
        fileSystem: fileSystem,
        logger: logger,
        platform: globals.platform,
        defines: <String, String>{kBuildMode: BuildMode.debug.cliName},
      );

      fileSystem
          .directory('.dart_tool')
          .childFile('package_config.json')
          .createSync(recursive: true);

      fileSystem.file('pubspec.yaml')
        ..createSync()
        ..writeAsStringSync('''
  name: example
  flutter:
    assets:
      - path: input.txt
        transformers:
          - package: my_capitalizer_transformer
  ''');

      fileSystem.file('input.txt')
        ..createSync(recursive: true)
        ..writeAsStringSync('abc');

      fileSystem.directory('2x').childFile('input.txt')
        ..createSync(recursive: true)
        ..writeAsStringSync('def');

      await const CopyAssets().build(environment);

      expect(inputFilePaths.toSet(), hasLength(inputFilePaths.length));
      expect(outputFilePaths.toSet(), hasLength(outputFilePaths.length));
    },
    overrides: <Type, Generator>{
      Logger: () => logger,
      FileSystem: () => fileSystem,
      Platform: () => FakePlatform(),
      ProcessManager: () => FakeProcessManager.empty(),
    },
  );

  testUsingContext(
    'Throws exception if pubspec contains missing files',
    () async {
      fileSystem.file('pubspec.yaml')
        ..createSync()
        ..writeAsStringSync('''
name: example

flutter:
  assets:
    - assets/foo/bar2.png

''');

      expect(() async => const CopyAssets().build(environment), throwsException);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testWithoutContext('processSkSLBundle returns null if there is no path '
      'to the bundle', () {
    expect(
      processSkSLBundle(
        null,
        targetPlatform: TargetPlatform.android,
        fileSystem: MemoryFileSystem.test(),
        logger: logger,
      ),
      isNull,
    );
  });

  testWithoutContext('processSkSLBundle throws exception if bundle file is '
      'missing', () {
    expect(
      () => processSkSLBundle(
        'does_not_exist.sksl',
        targetPlatform: TargetPlatform.android,
        fileSystem: MemoryFileSystem.test(),
        logger: logger,
      ),
      throwsException,
    );
  });

  testWithoutContext('processSkSLBundle throws exception if the bundle is not '
      'valid JSON', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final BufferLogger logger = BufferLogger.test();
    fileSystem.file('bundle.sksl').writeAsStringSync('{');

    expect(
      () => processSkSLBundle(
        'bundle.sksl',
        targetPlatform: TargetPlatform.android,
        fileSystem: fileSystem,
        logger: logger,
      ),
      throwsException,
    );
    expect(logger.errorText, contains('was not a JSON object'));
  });

  testWithoutContext('processSkSLBundle throws exception if the bundle is not '
      'a JSON object', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final BufferLogger logger = BufferLogger.test();
    fileSystem.file('bundle.sksl').writeAsStringSync('[]');

    expect(
      () => processSkSLBundle(
        'bundle.sksl',
        targetPlatform: TargetPlatform.android,
        fileSystem: fileSystem,
        logger: logger,
      ),
      throwsException,
    );
    expect(logger.errorText, contains('was not a JSON object'));
  });

  testWithoutContext('processSkSLBundle throws an exception if the engine '
      'revision is different', () {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final BufferLogger logger = BufferLogger.test();
    fileSystem
        .file('bundle.sksl')
        .writeAsStringSync(json.encode(<String, String>{'engineRevision': '1'}));

    expect(
      () => processSkSLBundle(
        'bundle.sksl',
        targetPlatform: TargetPlatform.android,
        fileSystem: fileSystem,
        logger: logger,
        engineVersion: '2',
      ),
      throwsException,
    );
    expect(logger.errorText, contains('Expected Flutter 1, but found 2'));
  });

  testWithoutContext('processSkSLBundle warns if the bundle target platform is '
      'different from the current target', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final BufferLogger logger = BufferLogger.test();
    fileSystem
        .file('bundle.sksl')
        .writeAsStringSync(
          json.encode(<String, Object>{
            'engineRevision': '2',
            'platform': 'fuchsia-arm64',
            'data': <String, Object>{},
          }),
        );

    final DevFSContent content =
        processSkSLBundle(
          'bundle.sksl',
          targetPlatform: TargetPlatform.android,
          fileSystem: fileSystem,
          logger: logger,
          engineVersion: '2',
        )!;

    expect(await content.contentsAsBytes(), utf8.encode('{"data":{}}'));
    expect(logger.errorText, contains('This may lead to less efficient shader caching'));
  });

  testWithoutContext('processSkSLBundle does not warn and produces bundle', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final BufferLogger logger = BufferLogger.test();
    fileSystem
        .file('bundle.sksl')
        .writeAsStringSync(
          json.encode(<String, Object>{
            'engineRevision': '2',
            'platform': 'android',
            'data': <String, Object>{},
          }),
        );

    final DevFSContent content =
        processSkSLBundle(
          'bundle.sksl',
          targetPlatform: TargetPlatform.android,
          fileSystem: fileSystem,
          logger: logger,
          engineVersion: '2',
        )!;

    expect(await content.contentsAsBytes(), utf8.encode('{"data":{}}'));
    expect(logger.errorText, isEmpty);
  });
}
