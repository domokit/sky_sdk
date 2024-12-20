// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:pool/pool.dart';
import 'package:process/process.dart';

import '../../base/error_handling_io.dart';
import '../../base/file_system.dart';
import '../../base/io.dart';
import '../../base/logger.dart';
import '../../build_info.dart';
import '../../devfs.dart';
import '../../flutter_manifest.dart';
import '../build_system.dart';

/// Applies a series of user-specified asset-transforming packages to an asset file.
final class AssetTransformer {
  AssetTransformer({
    required ProcessManager processManager,
    required FileSystem fileSystem,
    required String dartBinaryPath,
    required BuildMode buildMode,
  }) : _processManager = processManager,
       _fileSystem = fileSystem,
       _dartBinaryPath = dartBinaryPath,
       _buildMode = buildMode;

  static const String buildModeEnvVar = 'FLUTTER_BUILD_MODE';

  final ProcessManager _processManager;
  final FileSystem _fileSystem;
  final String _dartBinaryPath;
  final BuildMode _buildMode;

  /// The [Source] inputs that targets using this should depend on.
  ///
  /// See [Target.inputs].
  static const List<Source> inputs = <Source>[
    Source.pattern(
      '{FLUTTER_ROOT}/packages/flutter_tools/lib/src/build_system/targets/asset_transformer.dart',
    ),
  ];

  /// Applies, in sequence, a list of transformers to an [asset] and then copies
  /// the output to [outputPath].
  Future<AssetTransformationFailure?> transformAsset({
    required File asset,
    required String outputPath,
    required String workingDirectory,
    required List<AssetTransformerEntry> transformerEntries,
    required Logger logger,
  }) async {
    final Directory tempDirectory = _fileSystem.systemTempDirectory.createTempSync();

    int transformStep = 0;
    File nextTempFile() {
      final String basename = _fileSystem.path.basename(asset.path);
      final String ext = _fileSystem.path.extension(asset.path);

      final File result = tempDirectory.childFile('$basename-transformOutput$transformStep$ext');
      transformStep++;
      return result;
    }

    File tempInputFile = nextTempFile();
    await asset.copy(tempInputFile.path);
    File tempOutputFile = nextTempFile();

    final Stopwatch stopwatch = Stopwatch()..start();
    try {
      for (final (int i, AssetTransformerEntry transformer) in transformerEntries.indexed) {
        final AssetTransformationFailure? transformerFailure = await _applyTransformer(
          asset: tempInputFile,
          output: tempOutputFile,
          transformer: transformer,
          workingDirectory: workingDirectory,
          logger: logger,
        );

        if (transformerFailure != null) {
          return AssetTransformationFailure(transformerFailure.message);
        }

        ErrorHandlingFileSystem.deleteIfExists(tempInputFile);
        if (i == transformerEntries.length - 1) {
          await _fileSystem.file(outputPath).create(recursive: true);
          await tempOutputFile.copy(outputPath);
        } else {
          tempInputFile = tempOutputFile;
          tempOutputFile = nextTempFile();
        }
      }

      logger.printTrace(
        "Finished transforming asset at path '${asset.path}' (${stopwatch.elapsedMilliseconds}ms)",
      );
    } finally {
      ErrorHandlingFileSystem.deleteIfExists(tempDirectory, recursive: true);
    }

    return null;
  }

  Future<AssetTransformationFailure?> _applyTransformer({
    required File asset,
    required File output,
    required AssetTransformerEntry transformer,
    required String workingDirectory,
    required Logger logger,
  }) async {
    final List<String> transformerArguments = <String>[
      '--input=${asset.absolute.path}',
      '--output=${output.absolute.path}',
      ...?transformer.args,
    ];

    final List<String> command = <String>[
      _dartBinaryPath,
      'run',
      transformer.package,
      ...transformerArguments,
    ];

    // Delete the output file if it already exists for whatever reason.
    // With this, we can check for the existence of the file after transformation
    // to make sure the transformer produced an output file.
    ErrorHandlingFileSystem.deleteIfExists(output);

    logger.printTrace("Transforming asset using command '${command.join(' ')}'");
    final ProcessResult result = await _processManager.run(
      command,
      workingDirectory: workingDirectory,
      environment: <String, String>{AssetTransformer.buildModeEnvVar: _buildMode.cliName},
    );
    final String stdout = result.stdout as String;
    final String stderr = result.stderr as String;

    if (result.exitCode != 0) {
      return AssetTransformationFailure(
        'Transformer process terminated with non-zero exit code: ${result.exitCode}\n'
        'Transformer package: ${transformer.package}\n'
        'Full command: ${command.join(' ')}\n'
        'stdout:\n$stdout\n'
        'stderr:\n$stderr',
      );
    }

    if (!_fileSystem.file(output).existsSync()) {
      return AssetTransformationFailure(
        'Asset transformer ${transformer.package} did not produce an output file.\n'
        'Input file provided to transformer: "${asset.path}"\n'
        'Expected output file at: "${output.absolute.path}"\n'
        'Full command: ${command.join(' ')}\n'
        'stdout:\n$stdout\n'
        'stderr:\n$stderr',
      );
    }

    return null;
  }
}

// A wrapper around [AssetTransformer] to support hot reload of transformed assets.
final class DevelopmentAssetTransformer {
  DevelopmentAssetTransformer({
    required FileSystem fileSystem,
    required AssetTransformer transformer,
    required Logger logger,
  }) : _fileSystem = fileSystem,
       _transformer = transformer,
       _logger = logger;

  final AssetTransformer _transformer;
  final FileSystem _fileSystem;
  final Pool _transformationPool = Pool(4);
  final Logger _logger;

  /// Re-transforms an asset and returns a [DevFSContent] that should be synced
  /// to the attached device in its place.
  ///
  /// Returns `null` if any of the transformation subprocesses failed.
  Future<DevFSContent?> retransformAsset({
    required String inputAssetKey,
    required DevFSContent inputAssetContent,
    required List<AssetTransformerEntry> transformerEntries,
    required String workingDirectory,
  }) async {
    final File output = _fileSystem.systemTempDirectory.childFile(
      'retransformerInput-$inputAssetKey',
    );
    ErrorHandlingFileSystem.deleteIfExists(output);
    File? inputFile;
    bool cleanupInput = false;
    Uint8List result;
    PoolResource? resource;
    try {
      resource = await _transformationPool.request();
      if (inputAssetContent is DevFSFileContent) {
        inputFile = inputAssetContent.file as File;
      } else {
        inputFile = _fileSystem.systemTempDirectory.childFile('retransformerInput-$inputAssetKey');
        inputFile.writeAsBytesSync(await inputAssetContent.contentsAsBytes());
        cleanupInput = true;
      }
      final AssetTransformationFailure? failure = await _transformer.transformAsset(
        asset: inputFile,
        outputPath: output.path,
        transformerEntries: transformerEntries,
        workingDirectory: workingDirectory,
        logger: _logger,
      );
      if (failure != null) {
        _logger.printError(failure.message);
        return null;
      }
      result = output.readAsBytesSync();
    } finally {
      resource?.release();
      ErrorHandlingFileSystem.deleteIfExists(output);
      if (cleanupInput && inputFile != null) {
        ErrorHandlingFileSystem.deleteIfExists(inputFile);
      }
    }
    return DevFSByteContent(result);
  }
}

final class AssetTransformationFailure {
  const AssetTransformationFailure(this.message);

  final String message;
}
