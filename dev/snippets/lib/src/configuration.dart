// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/local.dart';
import 'package:path/path.dart' as path;

// Represents the locations of all of the data for snippets.
class SnippetConfiguration {
  const SnippetConfiguration({
    required this.configDirectory,
    required this.skeletonsDirectory,
    this.filesystem = const LocalFileSystem(),
  });

  final FileSystem filesystem;

  /// This is the configuration directory for the snippets system, containing
  /// the skeletons and templates.
  final Directory configDirectory;

  /// The directory containing the HTML skeletons to be filled out with metadata
  /// and returned to dartdoc for insertion in the output.
  final Directory skeletonsDirectory;

  /// Gets the skeleton file to use for the given [SampleType] and DartPad
  /// preference.
  File getHtmlSkeletonFile(String type) {
    final String filename = type == 'dartpad' ? 'dartpad-sample.html' : '$type.html';
    return filesystem.file(path.join(skeletonsDirectory.path, filename));
  }
}

/// A class to compute the configuration of the snippets input and output
/// locations based in the current location of the snippets main.dart.
class FlutterRepoSnippetConfiguration extends SnippetConfiguration {
  FlutterRepoSnippetConfiguration({required this.flutterRoot, super.filesystem})
    : super(
        configDirectory: _underRoot(filesystem, flutterRoot, const <String>[
          'dev',
          'snippets',
          'config',
        ]),
        skeletonsDirectory: _underRoot(filesystem, flutterRoot, const <String>[
          'dev',
          'snippets',
          'config',
          'skeletons',
        ]),
      );

  final Directory flutterRoot;

  static Directory _underRoot(FileSystem fs, Directory flutterRoot, List<String> dirs) =>
      fs.directory(path.canonicalize(path.joinAll(<String>[flutterRoot.absolute.path, ...dirs])));
}
