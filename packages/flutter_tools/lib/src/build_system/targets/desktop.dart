// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../../base/file_system.dart';
import '../depfile.dart';

/// Unpack the engine artifact list [artifacts] from [engineSourcePath], ICU
/// data (if provided), and [clientSourcePaths] (if provided) into a directory
/// [outputDirectory].
///
/// Returns a [Depfile] including all copied files.
///
/// Throws an [Exception] if [artifacts] includes missing files, directories,
/// or links.
Depfile unpackDesktopArtifacts({
  required FileSystem fileSystem,
  required List<String> artifacts,
  required Directory outputDirectory,
  required String engineSourcePath,
  List<String>? clientSourcePaths,
  String? icuDataPath,
}) {
  final List<File> inputs = <File>[];
  final List<File> outputs = <File>[];
  for (final String artifact in artifacts) {
    final String entityPath = fileSystem.path.join(engineSourcePath, artifact);
    final FileSystemEntityType entityType = fileSystem.typeSync(entityPath);

    if (entityType == FileSystemEntityType.notFound ||
        entityType == FileSystemEntityType.directory ||
        entityType == FileSystemEntityType.link) {
      throw Exception('Unsupported file type "$entityType" for $entityPath');
    }
    assert(entityType == FileSystemEntityType.file);
    final String outputPath = fileSystem.path.join(
      outputDirectory.path,
      fileSystem.path.relative(entityPath, from: engineSourcePath),
    );
    final File destinationFile = fileSystem.file(outputPath);
    if (!destinationFile.parent.existsSync()) {
      destinationFile.parent.createSync(recursive: true);
    }
    final File inputFile = fileSystem.file(entityPath);
    inputFile.copySync(destinationFile.path);
    inputs.add(inputFile);
    outputs.add(destinationFile);
  }
  if (icuDataPath != null) {
    final File inputFile = fileSystem.file(icuDataPath);
    final File outputFile = fileSystem.file(
      fileSystem.path.join(outputDirectory.path, inputFile.basename),
    );
    inputFile.copySync(outputFile.path);
    inputs.add(inputFile);
    outputs.add(outputFile);
  }
  if (clientSourcePaths == null) {
    return Depfile(inputs, outputs);
  }
  for (final String clientSourcePath in clientSourcePaths) {
    final Directory clientSourceDirectory = fileSystem.directory(clientSourcePath);
    if (!clientSourceDirectory.existsSync()) {
      throw Exception('Missing clientSourceDirectory: $clientSourcePath');
    }
    for (final File input in clientSourceDirectory.listSync(recursive: true).whereType<File>()) {
      final String outputPath = fileSystem.path.join(
        outputDirectory.path,
        fileSystem.path.relative(input.path, from: clientSourceDirectory.parent.path),
      );
      final File destinationFile = fileSystem.file(outputPath);
      if (!destinationFile.parent.existsSync()) {
        destinationFile.parent.createSync(recursive: true);
      }
      final File inputFile = fileSystem.file(input);
      inputFile.copySync(destinationFile.path);
      inputs.add(inputFile);
      outputs.add(destinationFile);
    }
  }
  return Depfile(inputs, outputs);
}
