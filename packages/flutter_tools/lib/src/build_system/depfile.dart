// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../base/error_handling_io.dart';
import '../base/file_system.dart';
import '../base/logger.dart';

/// A service for creating and parsing [Depfile]s.
class DepfileService {
  DepfileService({required Logger logger, required FileSystem fileSystem})
    : _logger = logger,
      _fileSystem = fileSystem;

  final Logger _logger;
  final FileSystem _fileSystem;
  static final RegExp _separatorExpr = RegExp(r'([^\\]) ');
  static final RegExp _escapeExpr = RegExp(r'\\(.)');

  /// Given an [depfile] File, write the depfile contents.
  ///
  /// If both [inputs] and [outputs] are empty, ensures the file does not
  /// exist. This can be overridden with the [writeEmpty] parameter when
  /// both static and runtime dependencies exist and it is not desired
  /// to force a rerun due to no depfile.
  void writeToFile(Depfile depfile, File output, {bool writeEmpty = false}) {
    if (depfile.inputs.isEmpty && depfile.outputs.isEmpty && !writeEmpty) {
      ErrorHandlingFileSystem.deleteIfExists(output);
      return;
    }
    final StringBuffer buffer = StringBuffer();
    _writeFilesToBuffer(depfile.outputs, buffer);
    buffer.write(': ');
    _writeFilesToBuffer(depfile.inputs, buffer);
    output.writeAsStringSync(buffer.toString());
  }

  /// Parse the depfile contents from [file].
  ///
  /// If the syntax is invalid, returns an empty [Depfile].
  Depfile parse(File file) {
    final String contents = file.readAsStringSync();
    final List<String> colonSeparated = contents.split(': ');
    if (colonSeparated.length != 2) {
      _logger.printError('Invalid depfile: ${file.path}');
      return const Depfile(<File>[], <File>[]);
    }
    final List<File> inputs = _processList(colonSeparated[1].trim());
    final List<File> outputs = _processList(colonSeparated[0].trim());
    return Depfile(inputs, outputs);
  }

  /// Parse the output of dart2js's used dependencies.
  ///
  /// The [file] contains a list of newline separated file URIs. The output
  /// file must be manually specified.
  Depfile parseDart2js(File file, File output) {
    final List<File> inputs = <File>[
      for (final String rawUri in file.readAsLinesSync())
        if (rawUri.trim().isNotEmpty)
          if (Uri.tryParse(rawUri) case final Uri fileUri when fileUri.scheme == 'file')
            _fileSystem.file(fileUri),
    ];
    return Depfile(inputs, <File>[output]);
  }

  void _writeFilesToBuffer(List<File> files, StringBuffer buffer) {
    for (final File outputFile in files) {
      if (_fileSystem.path.style.separator == r'\') {
        // backslashes and spaces in a depfile have to be escaped if the
        // platform separator is a backslash.
        final String path = outputFile.path.replaceAll(r'\', r'\\').replaceAll(r' ', r'\ ');
        buffer.write(' $path');
      } else {
        final String path = outputFile.path.replaceAll(r' ', r'\ ');
        buffer.write(' $path');
      }
    }
  }

  List<File> _processList(String rawText) {
    return rawText
        // Put every file on right-hand side on the separate line
        .replaceAllMapped(_separatorExpr, (Match match) => '${match.group(1)}\n')
        .split('\n')
        // Expand escape sequences, so that '\ ', for example,ß becomes ' '
        .map<String>(
          (String path) =>
              path.replaceAllMapped(_escapeExpr, (Match match) => match.group(1)!).trim(),
        )
        .where((String path) => path.isNotEmpty)
        // The tool doesn't write duplicates to these lists. This call is an attempt to
        // be resilient to the outputs of other tools which write or user edits to depfiles.
        .toSet()
        .map(_fileSystem.file)
        .toList();
  }
}

/// A class for representing depfile formats.
class Depfile {
  /// Create a [Depfile] from a list of [input] files and [output] files.
  const Depfile(this.inputs, this.outputs);

  /// The input files for this depfile.
  final List<File> inputs;

  /// The output files for this depfile.
  final List<File> outputs;
}
