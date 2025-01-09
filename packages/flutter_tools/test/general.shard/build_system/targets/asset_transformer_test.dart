// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/args.dart';
import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/tools/asset_transformer.dart';
import 'package:flutter_tools/src/flutter_manifest.dart';

import '../../../src/common.dart';
import '../../../src/fake_process_manager.dart';

void main() {
  testWithoutContext('Invokes dart properly', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final BufferLogger logger = BufferLogger.test();
    final Artifacts artifacts = Artifacts.test();

    final File asset =
        fileSystem.file('asset.txt')
          ..createSync()
          ..writeAsStringSync('hello world');
    const String outputPath = 'output.txt';

    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      FakeCommand(
        command: <String>[
          artifacts.getArtifactPath(Artifact.engineDartBinary),
          'run',
          'my_copy_transformer',
          '--input=/.tmp_rand0/rand0/asset.txt-transformOutput0.txt',
          '--output=/.tmp_rand0/rand0/asset.txt-transformOutput1.txt',
          '-f',
          '--my_option',
          'my_option_value',
        ],
        onRun: (List<String> args) {
          final ArgResults parsedArgs = (ArgParser()
                ..addOption('input')
                ..addOption('output')
                ..addFlag('foo', abbr: 'f')
                ..addOption('my_option'))
              .parse(args);

          fileSystem.file(parsedArgs['input']).copySync(parsedArgs['output'] as String);
        },
      ),
    ]);

    final AssetTransformer transformer = AssetTransformer(
      processManager: processManager,
      fileSystem: fileSystem,
      dartBinaryPath: artifacts.getArtifactPath(Artifact.engineDartBinary),
      buildMode: BuildMode.debug,
    );

    final AssetTransformationFailure? transformationFailure = await transformer.transformAsset(
      asset: asset,
      outputPath: outputPath,
      workingDirectory: fileSystem.currentDirectory.path,
      transformerEntries: <AssetTransformerEntry>[
        const AssetTransformerEntry(
          package: 'my_copy_transformer',
          args: <String>['-f', '--my_option', 'my_option_value'],
        ),
      ],
      logger: logger,
    );

    expect(transformationFailure, isNull, reason: logger.errorText);
    expect(processManager, hasNoRemainingExpectations);
    expect(fileSystem.file(outputPath).readAsStringSync(), 'hello world');
    expect(
      fileSystem.directory('.tmp_rand0').listSync(),
      isEmpty,
      reason: 'Transformer did not clean up after itself.',
    );
  });

  testWithoutContext(
    'logs useful error information when transformation process returns a nonzero exit code',
    () async {
      final FileSystem fileSystem = MemoryFileSystem.test();
      final Artifacts artifacts = Artifacts.test();

      final File asset = fileSystem.file('asset.txt')..createSync();
      const String outputPath = 'output.txt';

      final String dartBinaryPath = artifacts.getArtifactPath(Artifact.engineDartBinary);
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        FakeCommand(
          command: <String>[
            dartBinaryPath,
            'run',
            'my_copy_transformer',
            '--input=/.tmp_rand0/rand0/asset.txt-transformOutput0.txt',
            '--output=/.tmp_rand0/rand0/asset.txt-transformOutput1.txt',
          ],
          onRun: (List<String> args) {
            final ArgResults parsedArgs = (ArgParser()
                  ..addOption('input')
                  ..addOption('output'))
                .parse(args);
            fileSystem.file(parsedArgs['input']).copySync(parsedArgs['output'] as String);
          },
          exitCode: 1,
          stdout: 'Beginning transformation',
          stderr: 'Something went wrong',
        ),
      ]);

      final AssetTransformer transformer = AssetTransformer(
        processManager: processManager,
        fileSystem: fileSystem,
        dartBinaryPath: dartBinaryPath,
        buildMode: BuildMode.debug,
      );

      final AssetTransformationFailure? failure = await transformer.transformAsset(
        asset: asset,
        outputPath: outputPath,
        workingDirectory: fileSystem.currentDirectory.path,
        transformerEntries: <AssetTransformerEntry>[
          const AssetTransformerEntry(package: 'my_copy_transformer', args: <String>[]),
        ],
        logger: BufferLogger.test(),
      );

      expect(asset, exists);
      expect(processManager, hasNoRemainingExpectations);
      expect(failure, isNotNull);
      expect(failure!.message, '''
Transformer process terminated with non-zero exit code: 1
Transformer package: my_copy_transformer
Full command: $dartBinaryPath run my_copy_transformer --input=/.tmp_rand0/rand0/asset.txt-transformOutput0.txt --output=/.tmp_rand0/rand0/asset.txt-transformOutput1.txt
stdout:
Beginning transformation
stderr:
Something went wrong''');
      expect(
        fileSystem.directory('.tmp_rand0').listSync(),
        isEmpty,
        reason: 'Transformer did not clean up after itself.',
      );
    },
  );

  testWithoutContext(
    'prints error message when the transformer does not produce an output file',
    () async {
      final FileSystem fileSystem = MemoryFileSystem.test();
      final Artifacts artifacts = Artifacts.test();

      final File asset = fileSystem.file('asset.txt')..createSync();
      const String outputPath = 'output.txt';

      final String dartBinaryPath = artifacts.getArtifactPath(Artifact.engineDartBinary);
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        FakeCommand(
          command: <String>[
            dartBinaryPath,
            'run',
            'my_transformer',
            '--input=/.tmp_rand0/rand0/asset.txt-transformOutput0.txt',
            '--output=/.tmp_rand0/rand0/asset.txt-transformOutput1.txt',
          ],
          onRun: (_) {
            // Do nothing.
          },
          stderr: 'Transformation failed, but I forgot to exit with a non-zero code.',
        ),
      ]);

      final AssetTransformer transformer = AssetTransformer(
        processManager: processManager,
        fileSystem: fileSystem,
        dartBinaryPath: dartBinaryPath,
        buildMode: BuildMode.debug,
      );

      final AssetTransformationFailure? failure = await transformer.transformAsset(
        asset: asset,
        outputPath: outputPath,
        workingDirectory: fileSystem.currentDirectory.path,
        transformerEntries: <AssetTransformerEntry>[
          const AssetTransformerEntry(package: 'my_transformer', args: <String>[]),
        ],
        logger: BufferLogger.test(),
      );

      expect(processManager, hasNoRemainingExpectations);
      expect(failure, isNotNull);
      expect(failure!.message, '''
Asset transformer my_transformer did not produce an output file.
Input file provided to transformer: "/.tmp_rand0/rand0/asset.txt-transformOutput0.txt"
Expected output file at: "/.tmp_rand0/rand0/asset.txt-transformOutput1.txt"
Full command: $dartBinaryPath run my_transformer --input=/.tmp_rand0/rand0/asset.txt-transformOutput0.txt --output=/.tmp_rand0/rand0/asset.txt-transformOutput1.txt
stdout:

stderr:
Transformation failed, but I forgot to exit with a non-zero code.''');
      expect(
        fileSystem.directory('.tmp_rand0').listSync(),
        isEmpty,
        reason: 'Transformer did not clean up after itself.',
      );
    },
  );

  testWithoutContext('correctly chains transformations when there are multiple of them', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final Artifacts artifacts = Artifacts.test();

    final File asset =
        fileSystem.file('asset.txt')
          ..createSync()
          ..writeAsStringSync('ABC');
    const String outputPath = 'output.txt';

    final String dartBinaryPath = artifacts.getArtifactPath(Artifact.engineDartBinary);
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      FakeCommand(
        command: <String>[
          dartBinaryPath,
          'run',
          'my_lowercase_transformer',
          '--input=/.tmp_rand0/rand0/asset.txt-transformOutput0.txt',
          '--output=/.tmp_rand0/rand0/asset.txt-transformOutput1.txt',
        ],
        onRun: (List<String> args) {
          final ArgResults parsedArgs = (ArgParser()
                ..addOption('input')
                ..addOption('output'))
              .parse(args);

          final String inputFileContents = fileSystem.file(parsedArgs['input']).readAsStringSync();
          fileSystem.file(parsedArgs['output'])
            ..createSync()
            ..writeAsStringSync(inputFileContents.toLowerCase());
        },
      ),
      FakeCommand(
        command: <String>[
          dartBinaryPath,
          'run',
          'my_distance_from_ascii_a_transformer',
          '--input=/.tmp_rand0/rand0/asset.txt-transformOutput1.txt',
          '--output=/.tmp_rand0/rand0/asset.txt-transformOutput2.txt',
        ],
        onRun: (List<String> args) {
          final ArgResults parsedArgs = (ArgParser()
                ..addOption('input')
                ..addOption('output'))
              .parse(args);

          final String inputFileContents = fileSystem.file(parsedArgs['input']).readAsStringSync();
          final StringBuffer outputContents = StringBuffer();

          for (int i = 0; i < inputFileContents.length; i++) {
            outputContents.write(inputFileContents.codeUnitAt(i) - 'a'.codeUnits.first);
          }

          fileSystem.file(parsedArgs['output'])
            ..createSync()
            ..writeAsStringSync(outputContents.toString());
        },
      ),
    ]);

    final AssetTransformer transformer = AssetTransformer(
      processManager: processManager,
      fileSystem: fileSystem,
      dartBinaryPath: dartBinaryPath,
      buildMode: BuildMode.debug,
    );

    final AssetTransformationFailure? failure = await transformer.transformAsset(
      asset: asset,
      outputPath: outputPath,
      workingDirectory: fileSystem.currentDirectory.path,
      transformerEntries: <AssetTransformerEntry>[
        const AssetTransformerEntry(package: 'my_lowercase_transformer', args: <String>[]),
        const AssetTransformerEntry(
          package: 'my_distance_from_ascii_a_transformer',
          args: <String>[],
        ),
      ],
      logger: BufferLogger.test(),
    );

    expect(processManager, hasNoRemainingExpectations);
    expect(failure, isNull);
    expect(fileSystem.file(outputPath).readAsStringSync(), '012');
    expect(
      fileSystem.directory('.tmp_rand0').listSync(),
      isEmpty,
      reason: 'Transformer did not clean up after itself.',
    );
  });

  testWithoutContext(
    "prints an error when a transformer in a chain (that's not the first) does not produce an output",
    () async {
      final FileSystem fileSystem = MemoryFileSystem();
      final Artifacts artifacts = Artifacts.test();

      final File asset =
          fileSystem.file('asset.txt')
            ..createSync()
            ..writeAsStringSync('ABC');
      const String outputPath = 'output.txt';

      final String dartBinaryPath = artifacts.getArtifactPath(Artifact.engineDartBinary);
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        FakeCommand(
          command: <String>[
            dartBinaryPath,
            'run',
            'my_lowercase_transformer',
            '--input=/.tmp_rand0/rand0/asset.txt-transformOutput0.txt',
            '--output=/.tmp_rand0/rand0/asset.txt-transformOutput1.txt',
          ],
          onRun: (List<String> args) {
            final ArgResults parsedArgs = (ArgParser()
                  ..addOption('input')
                  ..addOption('output'))
                .parse(args);

            final String inputFileContents =
                fileSystem.file(parsedArgs['input']).readAsStringSync();
            fileSystem.file(parsedArgs['output'])
              ..createSync()
              ..writeAsStringSync(inputFileContents.toLowerCase());
          },
        ),
        FakeCommand(
          command: <String>[
            dartBinaryPath,
            'run',
            'my_distance_from_ascii_a_transformer',
            '--input=/.tmp_rand0/rand0/asset.txt-transformOutput1.txt',
            '--output=/.tmp_rand0/rand0/asset.txt-transformOutput2.txt',
          ],
          onRun: (List<String> args) {
            // Do nothing.
          },
          stderr: 'Transformation failed, but I forgot to exit with a non-zero code.',
          environment: const <String, String>{'FLUTTER_BUILD_MODE': 'debug'},
        ),
      ]);

      final AssetTransformer transformer = AssetTransformer(
        processManager: processManager,
        fileSystem: fileSystem,
        dartBinaryPath: dartBinaryPath,
        buildMode: BuildMode.debug,
      );

      final AssetTransformationFailure? failure = await transformer.transformAsset(
        asset: asset,
        outputPath: outputPath,
        workingDirectory: fileSystem.currentDirectory.path,
        transformerEntries: <AssetTransformerEntry>[
          const AssetTransformerEntry(package: 'my_lowercase_transformer', args: <String>[]),
          const AssetTransformerEntry(
            package: 'my_distance_from_ascii_a_transformer',
            args: <String>[],
          ),
        ],
        logger: BufferLogger.test(),
      );

      expect(failure, isNotNull);
      expect(failure!.message, '''
Asset transformer my_distance_from_ascii_a_transformer did not produce an output file.
Input file provided to transformer: "/.tmp_rand0/rand0/asset.txt-transformOutput1.txt"
Expected output file at: "/.tmp_rand0/rand0/asset.txt-transformOutput2.txt"
Full command: Artifact.engineDartBinary run my_distance_from_ascii_a_transformer --input=/.tmp_rand0/rand0/asset.txt-transformOutput1.txt --output=/.tmp_rand0/rand0/asset.txt-transformOutput2.txt
stdout:

stderr:
Transformation failed, but I forgot to exit with a non-zero code.''');
      expect(processManager, hasNoRemainingExpectations);
      expect(fileSystem.file(outputPath), isNot(exists));
      expect(
        fileSystem.directory('.tmp_rand0').listSync(),
        isEmpty,
        reason: 'Transformer did not clean up after itself.',
      );
    },
  );
}
